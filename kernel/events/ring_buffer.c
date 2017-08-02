/*
 * Performance events ring-buffer code:
 *
 *  Copyright (C) 2008 Thomas Gleixner <tglx@linutronix.de>
 *  Copyright (C) 2008-2011 Red Hat, Inc., Ingo Molnar
 *  Copyright (C) 2008-2011 Red Hat, Inc., Peter Zijlstra
 *  Copyright  ©  2009 Paul Mackerras, IBM Corp. <paulus@au1.ibm.com>
 *
 * For licensing details see kernel-base/COPYING
 */

#include <linux/perf_event.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/circ_buf.h>
#include <linux/poll.h>
#include <linux/shmem_fs.h>
#include <linux/mman.h>
#include <linux/sched/mm.h>

#include "internal.h"

static void perf_output_wakeup(struct perf_output_handle *handle)
{
	atomic_set(&handle->rb->poll, POLLIN);

	handle->event->pending_wakeup = 1;
	irq_work_queue(&handle->event->pending);
}

/*
 * We need to ensure a later event_id doesn't publish a head when a former
 * event isn't done writing. However since we need to deal with NMIs we
 * cannot fully serialize things.
 *
 * We only publish the head (and generate a wakeup) when the outer-most
 * event completes.
 */
static void perf_output_get_handle(struct perf_output_handle *handle)
{
	struct ring_buffer *rb = handle->rb;

	preempt_disable();
	local_inc(&rb->nest);
	handle->wakeup = local_read(&rb->wakeup);
}

static void perf_output_put_handle(struct perf_output_handle *handle)
{
	struct ring_buffer *rb = handle->rb;
	unsigned long head;

again:
	head = local_read(&rb->head);

	/*
	 * IRQ/NMI can happen here, which means we can miss a head update.
	 */

	if (!local_dec_and_test(&rb->nest))
		goto out;

	/*
	 * Since the mmap() consumer (userspace) can run on a different CPU:
	 *
	 *   kernel				user
	 *
	 *   if (LOAD ->data_tail) {		LOAD ->data_head
	 *			(A)		smp_rmb()	(C)
	 *	STORE $data			LOAD $data
	 *	smp_wmb()	(B)		smp_mb()	(D)
	 *	STORE ->data_head		STORE ->data_tail
	 *   }
	 *
	 * Where A pairs with D, and B pairs with C.
	 *
	 * In our case (A) is a control dependency that separates the load of
	 * the ->data_tail and the stores of $data. In case ->data_tail
	 * indicates there is no room in the buffer to store $data we do not.
	 *
	 * D needs to be a full barrier since it separates the data READ
	 * from the tail WRITE.
	 *
	 * For B a WMB is sufficient since it separates two WRITEs, and for C
	 * an RMB is sufficient since it separates two READs.
	 *
	 * See perf_output_begin().
	 */
	smp_wmb(); /* B, matches C */
	rb->user_page->data_head = head;

	/*
	 * Now check if we missed an update -- rely on previous implied
	 * compiler barriers to force a re-read.
	 */
	if (unlikely(head != local_read(&rb->head))) {
		local_inc(&rb->nest);
		goto again;
	}

	if (handle->wakeup != local_read(&rb->wakeup))
		perf_output_wakeup(handle);

out:
	preempt_enable();
}

static bool __always_inline
ring_buffer_has_space(unsigned long head, unsigned long tail,
		      unsigned long data_size, unsigned int size,
		      bool backward)
{
	if (!backward)
		return CIRC_SPACE(head, tail, data_size) >= size;
	else
		return CIRC_SPACE(tail, head, data_size) >= size;
}

static int __always_inline
__perf_output_begin(struct perf_output_handle *handle,
		    struct perf_event *event, unsigned int size,
		    bool backward)
{
	struct ring_buffer *rb;
	unsigned long tail, offset, head;
	int have_lost, page_shift;
	struct {
		struct perf_event_header header;
		u64			 id;
		u64			 lost;
	} lost_event;

	rcu_read_lock();
	/*
	 * For inherited events we send all the output towards the parent.
	 */
	if (event->parent)
		event = event->parent;

	rb = rcu_dereference(event->rb);
	if (unlikely(!rb))
		goto out;

	if (unlikely(rb->paused)) {
		if (rb->nr_pages)
			local_inc(&rb->lost);
		goto out;
	}

	handle->rb    = rb;
	handle->event = event;

	have_lost = local_read(&rb->lost);
	if (unlikely(have_lost)) {
		size += sizeof(lost_event);
		if (event->attr.sample_id_all)
			size += event->id_header_size;
	}

	perf_output_get_handle(handle);

	do {
		tail = READ_ONCE(rb->user_page->data_tail);
		offset = head = local_read(&rb->head);
		if (!rb->overwrite) {
			if (unlikely(!ring_buffer_has_space(head, tail,
							    perf_data_size(rb),
							    size, backward)))
				goto fail;
		}

		/*
		 * The above forms a control dependency barrier separating the
		 * @tail load above from the data stores below. Since the @tail
		 * load is required to compute the branch to fail below.
		 *
		 * A, matches D; the full memory barrier userspace SHOULD issue
		 * after reading the data and before storing the new tail
		 * position.
		 *
		 * See perf_output_put_handle().
		 */

		if (!backward)
			head += size;
		else
			head -= size;
	} while (local_cmpxchg(&rb->head, offset, head) != offset);

	if (backward) {
		offset = head;
		head = (u64)(-head);
	}

	/*
	 * We rely on the implied barrier() by local_cmpxchg() to ensure
	 * none of the data stores below can be lifted up by the compiler.
	 */

	if (unlikely(head - local_read(&rb->wakeup) > rb->watermark))
		local_add(rb->watermark, &rb->wakeup);

	page_shift = PAGE_SHIFT + page_order(rb);

	handle->page = (offset >> page_shift) & (rb->nr_pages - 1);
	offset &= (1UL << page_shift) - 1;
	handle->addr = rb->data_pages[handle->page] + offset;
	handle->size = (1UL << page_shift) - offset;

	if (unlikely(have_lost)) {
		struct perf_sample_data sample_data;

		lost_event.header.size = sizeof(lost_event);
		lost_event.header.type = PERF_RECORD_LOST;
		lost_event.header.misc = 0;
		lost_event.id          = event->id;
		lost_event.lost        = local_xchg(&rb->lost, 0);

		perf_event_header__init_id(&lost_event.header,
					   &sample_data, event);
		perf_output_put(handle, lost_event);
		perf_event__output_id_sample(event, handle, &sample_data);
	}

	return 0;

fail:
	local_inc(&rb->lost);
	perf_output_put_handle(handle);
out:
	rcu_read_unlock();

	return -ENOSPC;
}

int perf_output_begin_forward(struct perf_output_handle *handle,
			     struct perf_event *event, unsigned int size)
{
	return __perf_output_begin(handle, event, size, false);
}

int perf_output_begin_backward(struct perf_output_handle *handle,
			       struct perf_event *event, unsigned int size)
{
	return __perf_output_begin(handle, event, size, true);
}

int perf_output_begin(struct perf_output_handle *handle,
		      struct perf_event *event, unsigned int size)
{

	return __perf_output_begin(handle, event, size,
				   unlikely(is_write_backward(event)));
}

unsigned int perf_output_copy(struct perf_output_handle *handle,
		      const void *buf, unsigned int len)
{
	return __output_copy(handle, buf, len);
}

unsigned int perf_output_skip(struct perf_output_handle *handle,
			      unsigned int len)
{
	return __output_skip(handle, NULL, len);
}

void perf_output_end(struct perf_output_handle *handle)
{
	perf_output_put_handle(handle);
	rcu_read_unlock();
}

static void perf_event_init_pmu_info(struct perf_event *event,
				     struct perf_event_mmap_page *userpg)
{
	const struct pmu_info_header *pih = NULL;
	void *ptr = (void *)userpg + sizeof(*userpg);
	size_t size = sizeof(event->attr);

	if (event->pmu && event->pmu->pmu_info) {
		pih = event->pmu->pmu_info;
		size += pih->header_size;
		size += pih->pmu_info_size;
	}

	if (size + sizeof(*userpg) > PAGE_SIZE)
		return;

	userpg->pmu_offset = offset_in_page(ptr);
	userpg->pmu_size = size;

	memcpy(ptr, &event->attr, sizeof(event->attr));
	if (pih) {
		ptr += sizeof(event->attr);
		memcpy(ptr, (void *)pih, pih->header_size +
		       pih->pmu_info_size);
	}
}

static void perf_event_init_userpage(struct perf_event *event,
				     struct ring_buffer *rb)
{
	struct perf_event_mmap_page *userpg;

	userpg = rb->user_page;

	/* Allow new userspace to detect that bit 0 is deprecated */
	userpg->cap_bit0_is_deprecated = 1;
	userpg->size = offsetof(struct perf_event_mmap_page, __reserved);
	userpg->data_offset = PAGE_SIZE;
	userpg->data_size = perf_data_size(rb);
	if (event->attach_state & PERF_ATTACH_DETACHED) {
		userpg->aux_offset =
			(event->attr.detached_nr_pages + 1) << PAGE_SHIFT;
		userpg->aux_size =
			event->attr.detached_aux_nr_pages << PAGE_SHIFT;
	}

	perf_event_init_pmu_info(event, userpg);
}

static void
ring_buffer_init(struct ring_buffer *rb, struct perf_event *event, int flags)
{
	long max_size = perf_data_size(rb);
	long watermark =
		event->attr.watermark ? event->attr.wakeup_watermark : 0;

	if (watermark)
		rb->watermark = min(max_size, watermark);

	if (!rb->watermark)
		rb->watermark = max_size / 2;

	if (flags & RING_BUFFER_WRITABLE)
		rb->overwrite = 0;
	else
		rb->overwrite = 1;

	atomic_set(&rb->refcount, 1);

	INIT_LIST_HEAD(&rb->event_list);
	spin_lock_init(&rb->event_lock);

	/*
	 * perf_output_begin() only checks rb->paused, therefore
	 * rb->paused must be true if we have no pages for output.
	 */
	if (!rb->nr_pages || (flags & RING_BUFFER_SHMEM))
		rb->paused = 1;

	if (!(flags & RING_BUFFER_SHMEM))
		perf_event_init_userpage(event, rb);
}

void perf_aux_output_flag(struct perf_output_handle *handle, u64 flags)
{
	/*
	 * OVERWRITE is determined by perf_aux_output_end() and can't
	 * be passed in directly.
	 */
	if (WARN_ON_ONCE(flags & PERF_AUX_FLAG_OVERWRITE))
		return;

	handle->aux_flags |= flags;
}
EXPORT_SYMBOL_GPL(perf_aux_output_flag);

/*
 * This is called before hardware starts writing to the AUX area to
 * obtain an output handle and make sure there's room in the buffer.
 * When the capture completes, call perf_aux_output_end() to commit
 * the recorded data to the buffer.
 *
 * The ordering is similar to that of perf_output_{begin,end}, with
 * the exception of (B), which should be taken care of by the pmu
 * driver, since ordering rules will differ depending on hardware.
 *
 * Call this from pmu::start(); see the comment in perf_aux_output_end()
 * about its use in pmu callbacks. Both can also be called from the PMI
 * handler if needed.
 */
void *perf_aux_output_begin(struct perf_output_handle *handle,
			    struct perf_event *event)
{
	struct perf_event *output_event = event;
	unsigned long aux_head, aux_tail;
	struct ring_buffer *rb;

	if (output_event->parent) {
		WARN_ON_ONCE(is_detached_event(event));
		WARN_ON_ONCE(event->attach_state & PERF_ATTACH_SHMEM);
		output_event = output_event->parent;
	}

	/*
	 * Since this will typically be open across pmu::add/pmu::del, we
	 * grab ring_buffer's refcount instead of holding rcu read lock
	 * to make sure it doesn't disappear under us.
	 */
	rb = ring_buffer_get(output_event);
	if (!rb)
		return NULL;

	if (!rb_has_aux(rb))
		goto err;

	/*
	 * If aux_mmap_count is zero, the aux buffer is in perf_mmap_close(),
	 * about to get freed, so we leave immediately.
	 *
	 * Checking rb::aux_mmap_count and rb::refcount has to be done in
	 * the same order, see perf_mmap_close. Otherwise we end up freeing
	 * aux pages in this path, which is a bug, because in_atomic().
	 */
	if (!atomic_read(&rb->aux_mmap_count))
		goto err;

	if (!atomic_inc_not_zero(&rb->aux_refcount))
		goto err;

	/*
	 * Nesting is not supported for AUX area, make sure nested
	 * writers are caught early
	 */
	if (WARN_ON_ONCE(local_xchg(&rb->aux_nest, 1)))
		goto err_put;

	aux_head = rb->aux_head;

	handle->rb = rb;
	handle->event = event;
	handle->head = aux_head;
	handle->size = 0;
	handle->aux_flags = 0;

	/*
	 * In overwrite mode, AUX data stores do not depend on aux_tail,
	 * therefore (A) control dependency barrier does not exist. The
	 * (B) <-> (C) ordering is still observed by the pmu driver.
	 */
	if (!rb->aux_overwrite) {
		aux_tail = READ_ONCE(rb->user_page->aux_tail);
		handle->wakeup = rb->aux_wakeup + rb->aux_watermark;
		if (aux_head - aux_tail < perf_aux_size(rb))
			handle->size = CIRC_SPACE(aux_head, aux_tail, perf_aux_size(rb));

		/*
		 * handle->size computation depends on aux_tail load; this forms a
		 * control dependency barrier separating aux_tail load from aux data
		 * store that will be enabled on successful return
		 */
		if (!handle->size) { /* A, matches D */
			event->pending_disable = 1;
			perf_output_wakeup(handle);
			local_set(&rb->aux_nest, 0);
			goto err_put;
		}
	}

	return handle->rb->aux_priv;

err_put:
	/* can't be last */
	rb_free_aux(rb);

err:
	ring_buffer_put(rb);
	handle->event = NULL;

	return NULL;
}
EXPORT_SYMBOL_GPL(perf_aux_output_begin);

static bool __always_inline rb_need_aux_wakeup(struct ring_buffer *rb)
{
	if (rb->aux_overwrite)
		return false;

	if (rb->aux_head - rb->aux_wakeup >= rb->aux_watermark) {
		rb->aux_wakeup = rounddown(rb->aux_head, rb->aux_watermark);
		return true;
	}

	return false;
}

/*
 * Commit the data written by hardware into the ring buffer by adjusting
 * aux_head and posting a PERF_RECORD_AUX into the perf buffer. It is the
 * pmu driver's responsibility to observe ordering rules of the hardware,
 * so that all the data is externally visible before this is called.
 *
 * Note: this has to be called from pmu::stop() callback, as the assumption
 * of the AUX buffer management code is that after pmu::stop(), the AUX
 * transaction must be stopped and therefore drop the AUX reference count.
 */
void perf_aux_output_end(struct perf_output_handle *handle, unsigned long size)
{
	bool wakeup = !!(handle->aux_flags & PERF_AUX_FLAG_TRUNCATED);
	struct ring_buffer *rb = handle->rb;
	unsigned long aux_head;

	/* in overwrite mode, driver provides aux_head via handle */
	if (rb->aux_overwrite) {
		handle->aux_flags |= PERF_AUX_FLAG_OVERWRITE;

		aux_head = handle->head;
		rb->aux_head = aux_head;
	} else {
		handle->aux_flags &= ~PERF_AUX_FLAG_OVERWRITE;

		aux_head = rb->aux_head;
		rb->aux_head += size;
	}

	if (size || handle->aux_flags) {
		/*
		 * Only send RECORD_AUX if we have something useful to communicate
		 */

		perf_event_aux_event(handle->event, aux_head, size,
		                     handle->aux_flags);
	}

	rb->user_page->aux_head = rb->aux_head;
	if (rb_need_aux_wakeup(rb))
		wakeup = true;

	if (wakeup) {
		if (handle->aux_flags & PERF_AUX_FLAG_TRUNCATED)
			handle->event->pending_disable = 1;
		perf_output_wakeup(handle);
	}

	handle->event = NULL;

	local_set(&rb->aux_nest, 0);
	/* can't be last */
	rb_free_aux(rb);
	ring_buffer_put(rb);
}
EXPORT_SYMBOL_GPL(perf_aux_output_end);

/*
 * Skip over a given number of bytes in the AUX buffer, due to, for example,
 * hardware's alignment constraints.
 */
int perf_aux_output_skip(struct perf_output_handle *handle, unsigned long size)
{
	struct ring_buffer *rb = handle->rb;

	if (size > handle->size)
		return -ENOSPC;

	rb->aux_head += size;

	rb->user_page->aux_head = rb->aux_head;
	if (rb_need_aux_wakeup(rb)) {
		perf_output_wakeup(handle);
		handle->wakeup = rb->aux_wakeup + rb->aux_watermark;
	}

	handle->head = rb->aux_head;
	handle->size -= size;

	return 0;
}
EXPORT_SYMBOL_GPL(perf_aux_output_skip);

void *perf_get_aux(struct perf_output_handle *handle)
{
	/* this is only valid between perf_aux_output_begin and *_end */
	if (!handle->event)
		return NULL;

	return handle->rb->aux_priv;
}
EXPORT_SYMBOL_GPL(perf_get_aux);

static struct user_struct *get_users_pinned_events(void)
{
	struct user_struct *user = current_user(), *ret = NULL;

	if (atomic_long_inc_not_zero(&user->nr_pinnable_events))
		return user;

	mutex_lock(&user->pinned_mutex);
	if (!atomic_long_read(&user->nr_pinnable_events)) {
		if (WARN_ON_ONCE(!!user->pinned_events))
			goto unlock;

		user->pinned_events = alloc_percpu(struct perf_event *);
		if (!user->pinned_events) {
			goto unlock;
		} else {
			atomic_long_inc(&user->nr_pinnable_events);
			ret = get_current_user();
		}
	}

unlock:
	mutex_unlock(&user->pinned_mutex);

	return ret;
}

static void put_users_pinned_events(struct user_struct *user)
{
	if (!atomic_long_dec_and_test(&user->nr_pinnable_events))
		return;

	mutex_lock(&user->pinned_mutex);
	free_percpu(user->pinned_events);
	user->pinned_events = NULL;
	mutex_unlock(&user->pinned_mutex);
}

/*
 * Check if the current user can afford @nr_pages, considering the
 * perf_event_mlock sysctl and their mlock limit. If the former is exceeded,
 * pin the remainder on their mm, if the latter is not sufficient either,
 * error out. Otherwise, keep track of the pages used in the ring_buffer so
 * that the accounting can be undone when the pages are freed.
 */
static int __ring_buffer_account(struct ring_buffer *rb, struct mm_struct *mm,
                                 unsigned long nr_pages, unsigned long *locked)
{
	unsigned long total, limit, pinned;
	struct user_struct *user;

	if (!mm)
		mm = rb->mmap_mapping;

	user = get_users_pinned_events();
	if (!user)
		return -ENOMEM;

	limit = sysctl_perf_event_mlock >> (PAGE_SHIFT - 10);

	/*
	 * Increase the limit linearly with more CPUs:
	 */
	limit *= num_online_cpus();

	total = atomic_long_read(&user->locked_vm) + nr_pages;

	pinned = 0;
	if (total > limit) {
		/*
		 * Everything that's over the sysctl_perf_event_mlock
		 * limit needs to be accounted to the consumer's mm.
		 */
		if (!mm)
			goto err_put_user;

		pinned = total - limit;

		limit = rlimit(RLIMIT_MEMLOCK);
		limit >>= PAGE_SHIFT;
		total = mm->pinned_vm + pinned;

		if ((total > limit) && perf_paranoid_tracepoint_raw() &&
		    !capable(CAP_IPC_LOCK))
			goto err_put_user;

		*locked = pinned;
		mm->pinned_vm += pinned;
	}

	if (!rb->mmap_mapping)
		rb->mmap_mapping = mm;

	rb->mmap_user = user;
	atomic_long_add(nr_pages, &user->locked_vm);

	return 0;

err_put_user:
	put_users_pinned_events(user);

	return -EPERM;
}

static int ring_buffer_account(struct ring_buffer *rb, struct mm_struct *mm,
			       unsigned long nr_pages, bool aux)
{
	int ret;

	/* account for user page */
	if (!aux)
		nr_pages++;
	ret = __ring_buffer_account(rb, mm, nr_pages,
	                            aux ? &rb->aux_mmap_locked : &rb->mmap_locked);

	return ret;
}

/*
 * Undo the mlock pages accounting done in ring_buffer_account().
 */
void ring_buffer_unaccount(struct ring_buffer *rb, bool aux)
{
	unsigned long nr_pages = aux ? rb->aux_nr_pages : rb->nr_pages + 1;
	unsigned long pinned = aux ? rb->aux_mmap_locked : rb->mmap_locked;

	if (!rb->nr_pages && !rb->aux_nr_pages)
		return;

	if (WARN_ON_ONCE(!rb->mmap_user))
		return;

	atomic_long_sub(nr_pages, &rb->mmap_user->locked_vm);
	if (rb->mmap_mapping)
		rb->mmap_mapping->pinned_vm -= pinned;

	put_users_pinned_events(rb->mmap_user);
}

#define PERF_AUX_GFP	(GFP_KERNEL | __GFP_ZERO | __GFP_NOWARN | __GFP_NORETRY)

static struct page *
rb_alloc_aux_page(struct ring_buffer *rb, int node, int order, int pgoff)
{
	struct file *file = rb->shmem_file;
	struct page *page;

	if (order && file)
		return NULL;

	if (order > MAX_ORDER)
		order = MAX_ORDER;

	do {
		page = alloc_pages_node(node, PERF_AUX_GFP, order);
	} while (!page && order--);

	if (page && order) {
		/*
		 * Communicate the allocation size to the driver:
		 * if we managed to secure a high-order allocation,
		 * set its first page's private to this order;
		 * !PagePrivate(page) means it's just a normal page.
		 */
		split_page(page, order);
		SetPagePrivate(page);
		set_page_private(page, order);
	}

	return page;
}

static void rb_free_aux_page(struct ring_buffer *rb, int idx)
{
	struct page *page = virt_to_page(rb->aux_pages[idx]);

	/* SHMEM pages are freed elsewhere */
	if (rb->shmem_file || rb->shmem_pages_in == -1)
		return;

	page->mapping = NULL;

	ClearPagePrivate(page);
	__free_page(page);
}

static void __rb_free_aux(struct ring_buffer *rb)
{
	int pg;

	/*
	 * Should never happen, the last reference should be dropped from
	 * perf_mmap_close() path, which first stops aux transactions (which
	 * in turn are the atomic holders of aux_refcount) and then does the
	 * last rb_free_aux().
	 */
	WARN_ON_ONCE(in_atomic());

	if (rb->aux_priv) {
		rb->free_aux(rb->aux_priv);
		rb->free_aux = NULL;
		rb->aux_priv = NULL;
	}

	if (rb->aux_nr_pages) {
		for (pg = 0; pg < rb->aux_nr_pages; pg++)
			rb_free_aux_page(rb, pg);

		kfree(rb->aux_pages);
		rb->aux_nr_pages = 0;
	}
}

int rb_alloc_aux(struct ring_buffer *rb, struct perf_event *event,
		 pgoff_t pgoff, int nr_pages, long watermark, int flags)
{
	bool overwrite = !(flags & RING_BUFFER_WRITABLE);
	bool shmem = !!(flags & RING_BUFFER_SHMEM);
	int node = (event->cpu == -1) ? -1 : cpu_to_node(event->cpu);
	int ret, max_order = 0;

	if (!has_aux(event))
		return -EOPNOTSUPP;

	if (!shmem) {
		ret = ring_buffer_account(rb, NULL, nr_pages, true);
		if (ret)
			return ret;
	}

	ret = -EINVAL;
	if (event->pmu->capabilities & PERF_PMU_CAP_AUX_NO_SG) {
		/*
		 * We need to start with the max_order that fits in nr_pages,
		 * not the other way around, hence ilog2() and not get_order.
		 */
		max_order = ilog2(nr_pages);

		/*
		 * PMU requests more than one contiguous chunks of memory
		 * for SW double buffering
		 */
		if ((event->pmu->capabilities & PERF_PMU_CAP_AUX_SW_DOUBLEBUF) &&
		    !overwrite) {
			if (!max_order)
				goto out;

			max_order--;
		}
	}

	ret = -ENOMEM;
	rb->aux_pages = kzalloc_node(nr_pages * sizeof(void *), GFP_KERNEL, node);
	if (!rb->aux_pages)
		goto out;

	rb->free_aux = event->pmu->free_aux;

	if (shmem) {
		/*
		 * Can't guarantee contuguous high order allocations.
		 */
		if (max_order)
			goto out;

		/*
		 * Skip page allocation; it's done in rb_get_kernel_pages().
		 */
		rb->aux_nr_pages = nr_pages;

		goto post_setup;
	}

	for (rb->aux_nr_pages = 0; rb->aux_nr_pages < nr_pages;) {
		struct page *page;
		int last, order;

		order = min(max_order, ilog2(nr_pages - rb->aux_nr_pages));
		page = rb_alloc_aux_page(rb, node, order, pgoff + rb->aux_nr_pages);
		if (!page)
			goto out;

		if (order)
			order = page_private(page);

		for (last = rb->aux_nr_pages + (1 << order);
		     last > rb->aux_nr_pages; rb->aux_nr_pages++)
			rb->aux_pages[rb->aux_nr_pages] = page_address(page++);
	}

	/*
	 * In overwrite mode, PMUs that don't support SG may not handle more
	 * than one contiguous allocation, since they rely on PMI to do double
	 * buffering. In this case, the entire buffer has to be one contiguous
	 * chunk.
	 */
	if ((event->pmu->capabilities & PERF_PMU_CAP_AUX_NO_SG) &&
	    overwrite) {
		struct page *page = virt_to_page(rb->aux_pages[0]);

		if (page_private(page) != max_order)
			goto out;
	}

	rb->aux_priv = event->pmu->setup_aux(event->cpu, rb->aux_pages, nr_pages,
					     overwrite);
	if (!rb->aux_priv)
		goto out;

post_setup:
	ret = 0;

	/*
	 * aux_pages (and pmu driver's private data, aux_priv) will be
	 * referenced in both producer's and consumer's contexts, thus
	 * we keep a refcount here to make sure either of the two can
	 * reference them safely.
	 */
	atomic_set(&rb->aux_refcount, 1);

	rb->aux_overwrite = overwrite;
	rb->aux_watermark = watermark;

	if (!rb->aux_watermark && !rb->aux_overwrite)
		rb->aux_watermark = nr_pages << (PAGE_SHIFT - 1);

out:
	if (!ret) {
		rb->aux_pgoff = pgoff;
	} else {
		if (!shmem)
			ring_buffer_unaccount(rb, true);
		__rb_free_aux(rb);
	}

	return ret;
}

void rb_free_aux(struct ring_buffer *rb)
{
	if (atomic_dec_and_test(&rb->aux_refcount)) {
		if (!rb->shmem_file)
			ring_buffer_unaccount(rb, true);

		__rb_free_aux(rb);
	}
}

static unsigned long perf_rb_size(struct ring_buffer *rb)
{
	return perf_data_size(rb) + perf_aux_size(rb) + PAGE_SIZE;
}

int rb_inject(struct perf_event *event)
{
	struct ring_buffer *rb = event->rb;
	struct mm_struct *mm;
	unsigned long addr;
	int err = -ENOMEM;

	mm = get_task_mm(current);
	if (!mm)
		return -ESRCH;

	err = rb_get_kernel_pages(event);
	if (err)
		goto err_mmput;

	addr = vm_mmap(rb->shmem_file, 0, perf_rb_size(rb), PROT_READ,
		       MAP_SHARED | MAP_POPULATE, 0);

	mmput(mm);
	rb->mmap_mapping = mm;
	rb->shmem_file_addr = addr;

	return 0;

err_mmput:
	mmput(mm);

	return err;
}

static void rb_shmem_unmap(struct perf_event *event)
{
	struct ring_buffer *rb = event->rb;
	struct mm_struct *mm = rb->mmap_mapping;

	rb_toggle_paused(rb, true);

	if (!rb->shmem_file_addr)
		return;

	/*
	 * EXIT state means the task is past exit_mm(),
	 * no need to unmap anything
	 */
	if (event->state == PERF_EVENT_STATE_EXIT)
		return;

	down_write(&mm->mmap_sem);
	(void)do_munmap(mm, rb->shmem_file_addr, perf_rb_size(rb), NULL);
	up_write(&mm->mmap_sem);
	rb->shmem_file_addr = 0;
}

static int rb_shmem_setup(struct perf_event *event,
			  struct task_struct *task,
			  struct ring_buffer *rb)
{
	int nr_pages, err;
	char *name;

	if (WARN_ON_ONCE(!task))
		return -EINVAL;

	name = event->dent && event->dent->d_name.name ?
		kasprintf(GFP_KERNEL, "perf/%s",
			  event->dent->d_name.name) :
		kasprintf(GFP_KERNEL, "perf/%s/%d", event->pmu->name,
			  task_pid_nr_ns(task, event->ns));
	if (!name)
		return -ENOMEM;

	WARN_ON_ONCE(rb->user_page);

	nr_pages = rb->nr_pages + rb->aux_nr_pages + 1;
	rb->shmem_file = shmem_file_setup(name, nr_pages << PAGE_SHIFT,
					  VM_NORESERVE);
	kfree(name);

	if (IS_ERR(rb->shmem_file)) {
		err = PTR_ERR(rb->shmem_file);
		rb->shmem_file = NULL;
		return err;
	}

	mapping_set_gfp_mask(rb->shmem_file->f_mapping,
			     GFP_HIGHUSER | __GFP_RECLAIMABLE);

	event->dent->d_inode->i_mapping = rb->shmem_file->f_mapping;
	event->attach_state |= PERF_ATTACH_SHMEM;

	return 0;
}

/*
 * Pin ring_buffer's pages to memory while the task is scheduled in;
 * populate its page arrays (data_pages, aux_pages, user_page).
 */
int rb_get_kernel_pages(struct perf_event *event)
{
	struct ring_buffer *rb = event->rb;
	struct address_space *mapping;
	int nr_pages, i = 0, err = -EINVAL, changed = 0, mc = 0;
	struct page *page;

	/*
	 * The mmap_count rules for SHMEM buffers:
	 *  - they are always taken together
	 *  - except for perf_mmap(), which doesn't work for shmem buffers:
	 *    mmaping will force-pin more user's pages than is allowed
	 *  - if either of them was taken before us, the pages are there
	 */
	if (atomic_inc_return(&rb->mmap_count) == 1)
		mc++;

	if (atomic_inc_return(&rb->aux_mmap_count) == 1)
		mc++;

	if (mc < 2)
		goto done;

	if (WARN_ON_ONCE(!rb->shmem_file))
		goto err_put;

	nr_pages = perf_rb_size(rb) >> PAGE_SHIFT;

	mapping = rb->shmem_file->f_mapping;

restart:
	for (i = 0; i < nr_pages; i++) {
		WRITE_ONCE(rb->shmem_pages_in, i);
		err = shmem_getpage(mapping->host, i, &page, SGP_NOHUGE);
		if (err)
			goto err_put;

		unlock_page(page);

		if (READ_ONCE(rb->shmem_pages_in) != i) {
			put_page(page);
			goto restart;
		}

		mark_page_accessed(page);
		set_page_dirty(page);
		page->mapping = mapping;

		if (page == perf_mmap_to_page(rb, i))
			continue;

		changed++;
		if (!i) {
			bool init = !rb->user_page;

			rb->user_page = page_address(page);
			if (init)
				perf_event_init_userpage(event, rb);
		} else if (i <= rb->nr_pages) {
			rb->data_pages[i - 1] = page_address(page);
		} else {
			rb->aux_pages[i - rb->nr_pages - 1] = page_address(page);
		}
	}

	/* rebuild SG tables: pages may have changed */
	if (changed) {
		if (rb->aux_priv)
			rb->free_aux(rb->aux_priv);

		rb->aux_priv = event->pmu->setup_aux(smp_processor_id(),
						     rb->aux_pages,
						     rb->aux_nr_pages, true);
	}

	if (!rb->aux_priv) {
		err = -ENOMEM;
		goto err_put;
	}

done:
	rb_toggle_paused(rb, false);
	if (changed)
		perf_event_update_userpage(event);

	return 0;

err_put:
	for (i--; i >= 0; i--) {
		page = perf_mmap_to_page(rb, i);
		put_page(page);
	}

	atomic_dec(&rb->aux_mmap_count);
	atomic_dec(&rb->mmap_count);

	return err;
}

void rb_put_kernel_pages(struct ring_buffer *rb, bool final)
{
	struct page *page;
	int i;

	if (!rb || !rb->shmem_file)
		return;

	rb_toggle_paused(rb, true);

	/*
	 * If both mmap_counts go to zero, put the pages, otherwise
	 * do nothing.
	 */
	if (!atomic_dec_and_test(&rb->aux_mmap_count) ||
	    !atomic_dec_and_test(&rb->mmap_count))
		return;

	for (i = 0; i < READ_ONCE(rb->shmem_pages_in); i++) {
		page = perf_mmap_to_page(rb, i);
		set_page_dirty(page);
		if (final)
			page->mapping = NULL;
		put_page(page);
	}

	WRITE_ONCE(rb->shmem_pages_in, 0);
}

/*
 * SHMEM memory is accounted once per user allocated event (via
 * the syscall), since we can have at most NR_CPUS * nr_pages
 * pinned pages at any given point in time, regardless of how
 * many events there actually are.
 *
 * The first one (parent_rb==NULL) is where we do the accounting;
 * it will also be the one coming from the syscall, so if it fails,
 * we'll hand them back the error.
 * Others just inherit and bump the counter; can't fail.
 */
static int
rb_shmem_account(struct ring_buffer *rb, struct ring_buffer *parent_rb)
{
	unsigned long nr_pages = perf_rb_size(rb) >> PAGE_SHIFT;
	int ret = 0;

	if (parent_rb) {
		/* "parent" rb *must* have accounting refcounter */
		if (WARN_ON_ONCE(!parent_rb->acct_refcount))
			return -EINVAL;

		rb->acct_refcount = parent_rb->acct_refcount;
		atomic_inc(rb->acct_refcount);
		rb->mmap_user = get_uid(parent_rb->mmap_user);

		return 0;
	}

	/* All (data + aux + user page) in one go */
	ret = __ring_buffer_account(rb, NULL, nr_pages,
	                            &rb->mmap_locked);
	if (ret)
		return ret;

	rb->acct_refcount = kmalloc(sizeof(*rb->acct_refcount),
	                            GFP_KERNEL);
	if (!rb->acct_refcount)
		return -ENOMEM;

	atomic_set(rb->acct_refcount, 1);

	return 0;
}

static void rb_shmem_unaccount(struct ring_buffer *rb)
{
	free_uid(rb->mmap_user);

	if (!atomic_dec_and_test(rb->acct_refcount)) {
		rb->acct_refcount = NULL;
		return;
	}

	ring_buffer_unaccount(rb, false);
	kfree(rb->acct_refcount);
}

/*
 * Allocate a ring_buffer for a detached event and attach it to this event.
 * There's one ring_buffer per detached event and vice versa, so
 * ring_buffer_attach() does not apply.
 */
int rb_alloc_detached(struct perf_event *event, struct task_struct *task,
		      struct mm_struct *mm, struct ring_buffer *parent_rb)
{
	int aux_nr_pages = event->attr.detached_aux_nr_pages;
	int nr_pages = event->attr.detached_nr_pages;
	int ret, pgoff = nr_pages + 1;
	struct ring_buffer *rb;
	int flags = 0;

	/*
	 * These are basically coredump conditions. If these are
	 * not met, we proceed as we would, but with pinned pages
	 * and therefore *no inheritance*.
	 */
	if (event->attr.inherit && event->attr.exclude_kernel &&
	    event->cpu == -1)
		flags = RING_BUFFER_SHMEM;
	else if (event->attr.inherit)
		return -EINVAL;

	rb = rb_alloc(event, mm, nr_pages, flags);
	if (IS_ERR(rb))
		return PTR_ERR(rb);

	if (flags & RING_BUFFER_SHMEM) {
		ret = rb_shmem_account(rb, parent_rb);
		if (ret)
			goto err_free;
	}

	if (aux_nr_pages) {
		ret = rb_alloc_aux(rb, event, pgoff, aux_nr_pages, 0, flags);
		if (ret)
			goto err_unaccount;
	}

	if (flags & RING_BUFFER_SHMEM) {
		ret = rb_shmem_setup(event, task, rb);
		if (ret)
			goto err_free_aux;

		rb_toggle_paused(rb, true);
	} else {
		atomic_inc(&rb->mmap_count);
		if (aux_nr_pages)
			atomic_inc(&rb->aux_mmap_count);
	}

	/*
	 * Detached events don't need ring buffer wakeups, therefore we don't
	 * use ring_buffer_attach() here and event->rb_entry stays empty.
	 */
	rcu_assign_pointer(event->rb, rb);

	event->attach_state |= PERF_ATTACH_DETACHED;

	return 0;

err_free_aux:
	if (!(flags & RING_BUFFER_SHMEM))
		rb_free_aux(rb);

err_unaccount:
	if (flags & RING_BUFFER_SHMEM)
		rb_shmem_unaccount(rb);

err_free:
	if (flags & RING_BUFFER_SHMEM)
		kfree(rb);
	else
		rb_free(rb);

	return ret;
}

void rb_free_detached(struct ring_buffer *rb, struct perf_event *event)
{
	/* Must be the last one */
	WARN_ON_ONCE(atomic_read(&rb->refcount) != 1);

	if (rb->shmem_file) {
		rb_shmem_unmap(event);
		shmem_truncate_range(rb->shmem_file->f_inode, 0, (loff_t)-1);
		rb_put_kernel_pages(rb, true);
	} else {
		ring_buffer_unaccount(rb, false);
	}

	if (event->attach_state & PERF_ATTACH_SHMEM)
		rb_shmem_unaccount(rb);

	atomic_set(&rb->aux_mmap_count, 0);
	rcu_assign_pointer(event->rb, NULL);
	rb_free_aux(rb);
	rb_free(rb);
}

#ifndef CONFIG_PERF_USE_VMALLOC

/*
 * Back perf_mmap() with regular GFP_KERNEL-0 pages.
 */

static struct page *
__perf_mmap_to_page(struct ring_buffer *rb, unsigned long pgoff)
{
	if (pgoff > rb->nr_pages)
		return NULL;

	if (pgoff == 0)
		return virt_to_page(rb->user_page);

	return virt_to_page(rb->data_pages[pgoff - 1]);
}

static void *perf_mmap_alloc_page(int cpu)
{
	struct page *page;
	int node;

	node = (cpu == -1) ? cpu : cpu_to_node(cpu);
	page = alloc_pages_node(node, GFP_KERNEL | __GFP_ZERO, 0);
	if (!page)
		return NULL;

	return page_address(page);
}

struct ring_buffer *rb_alloc(struct perf_event *event, struct mm_struct *mm,
			     int nr_pages, int flags)
{
	unsigned long size = offsetof(struct ring_buffer, data_pages[nr_pages]);
	bool shmem = !!(flags & RING_BUFFER_SHMEM);
	struct ring_buffer *rb;
	int i, ret = -ENOMEM;

	rb = kzalloc(size, GFP_KERNEL);
	if (!rb)
		return ERR_PTR(-ENOMEM);

	if (shmem) {
		rb->shmem_pages_in = -1;
		goto post_alloc;
	}

	ret = ring_buffer_account(rb, mm, nr_pages, false);
	if (ret)
		goto fail_free_rb;

	ret = -ENOMEM;
	rb->user_page = perf_mmap_alloc_page(event->cpu);
	if (!rb->user_page)
		goto fail_unaccount;

	for (i = 0; i < nr_pages; i++) {
		rb->data_pages[i] = perf_mmap_alloc_page(event->cpu);

		if (!rb->data_pages[i])
			goto fail_data_pages;
	}

post_alloc:
	rb->nr_pages = nr_pages;

	ring_buffer_init(rb, event, flags);

	return rb;

fail_data_pages:
	for (i--; i >= 0; i--)
		put_page(virt_to_page(rb->data_pages[i]));

	put_page(virt_to_page(rb->user_page));

fail_unaccount:
	ring_buffer_unaccount(rb, false);

fail_free_rb:
	kfree(rb);

	return ERR_PTR(ret);
}

static void perf_mmap_free_page(unsigned long addr)
{
	struct page *page = virt_to_page((void *)addr);

	page->mapping = NULL;
	__free_page(page);
}

void rb_free(struct ring_buffer *rb)
{
	int i;

	if (rb->shmem_file) {
		/* the pages should have been freed before */
		fput(rb->shmem_file);
		goto out_free;
	}

	if (rb->shmem_pages_in == -1)
		goto out_free;

	perf_mmap_free_page((unsigned long)rb->user_page);
	for (i = 0; i < rb->nr_pages; i++)
		perf_mmap_free_page((unsigned long)rb->data_pages[i]);
out_free:
	kfree(rb);
}

#else
static int data_page_nr(struct ring_buffer *rb)
{
	return rb->nr_pages << page_order(rb);
}

static struct page *
__perf_mmap_to_page(struct ring_buffer *rb, unsigned long pgoff)
{
	/* The '>' counts in the user page. */
	if (pgoff > data_page_nr(rb))
		return NULL;

	return vmalloc_to_page((void *)rb->user_page + pgoff * PAGE_SIZE);
}

static void perf_mmap_unmark_page(void *addr)
{
	struct page *page = vmalloc_to_page(addr);

	page->mapping = NULL;
}

static void rb_free_work(struct work_struct *work)
{
	struct ring_buffer *rb;
	void *base;
	int i, nr;

	rb = container_of(work, struct ring_buffer, work);
	nr = data_page_nr(rb);

	base = rb->user_page;
	/* The '<=' counts in the user page. */
	for (i = 0; i <= nr; i++)
		perf_mmap_unmark_page(base + (i * PAGE_SIZE));

	vfree(base);
	kfree(rb);
}

void rb_free(struct ring_buffer *rb)
{
	schedule_work(&rb->work);
}

struct ring_buffer *rb_alloc(struct perf_event *event, struct mm_struct *mm,
			     int nr_pages, int flags)
{
	unsigned long size = offsetof(struct ring_buffer, data_pages[1]);
	struct ring_buffer *rb;
	void *all_buf;
	int ret = -ENOMEM;

	if (flags & RING_BUFFER_SHMEM)
		return -EOPNOTSUPP;

	rb = kzalloc(size, GFP_KERNEL);
	if (!rb)
		goto fail;

	ret = ring_buffer_account(rb, mm, nr_pages, false);
	if (ret)
		goto fail_free;

	ret = -ENOMEM;
	INIT_WORK(&rb->work, rb_free_work);

	all_buf = vmalloc_user((nr_pages + 1) * PAGE_SIZE);
	if (!all_buf)
		goto fail_all_buf;

	rb->user_page = all_buf;
	rb->data_pages[0] = all_buf + PAGE_SIZE;
	if (nr_pages) {
		rb->nr_pages = 1;
		rb->page_order = ilog2(nr_pages);
	}

	ring_buffer_init(rb, event, flags);

	return rb;

fail_all_buf:
	ring_buffer_unaccount(rb, false);

fail_free:
	kfree(rb);

fail:
	return ERR_PTR(ret);
}

#endif

struct page *
perf_mmap_to_page(struct ring_buffer *rb, unsigned long pgoff)
{
	if (rb->aux_nr_pages) {
		/* above AUX space */
		if (pgoff > rb->aux_pgoff + rb->aux_nr_pages)
			return NULL;

		/* AUX space */
		if (pgoff >= rb->aux_pgoff)
			return virt_to_page(rb->aux_pages[pgoff - rb->aux_pgoff]);
	}

	return __perf_mmap_to_page(rb, pgoff);
}
