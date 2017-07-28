/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _KERNEL_EVENTS_INTERNAL_H
#define _KERNEL_EVENTS_INTERNAL_H

#include <linux/hardirq.h>
#include <linux/uaccess.h>

/* Buffer handling */

#define RING_BUFFER_WRITABLE		0x01
#define RING_BUFFER_SHMEM		0x02

struct ring_buffer {
	atomic_t			refcount;
	struct rcu_head			rcu_head;
#ifdef CONFIG_PERF_USE_VMALLOC
	struct work_struct		work;
	int				page_order;	/* allocation order  */
#endif
	int				nr_pages;	/* nr of data pages  */
	int				overwrite;	/* can overwrite itself */
	int				paused;		/* can write into ring buffer */

	atomic_t			poll;		/* POLL_ for wakeups */

	local_t				head;		/* write position    */
	local_t				nest;		/* nested writers    */
	local_t				events;		/* event limit       */
	local_t				wakeup;		/* wakeup stamp      */
	local_t				lost;		/* nr records lost   */

	long				watermark;	/* wakeup watermark  */
	long				aux_watermark;
	/* poll crap */
	spinlock_t			event_lock;
	struct list_head		event_list;

	atomic_t			mmap_count;
	unsigned long			mmap_locked;
	struct user_struct		*mmap_user;
	struct mm_struct		*mmap_mapping;

	/* AUX area */
	long				aux_head;
	local_t				aux_nest;
	long				aux_wakeup;	/* last aux_watermark boundary crossed by aux_head */
	unsigned long			aux_pgoff;
	int				aux_nr_pages;
	int				aux_overwrite;
	atomic_t			aux_mmap_count;
	unsigned long			aux_mmap_locked;
	void				(*free_aux)(void *);
	atomic_t			aux_refcount;
	void				**aux_pages;
	void				*aux_priv;

	atomic_t			*acct_refcount;

	/* tmpfs file for kernel-owned ring buffers */
	struct file			*shmem_file;
	unsigned long			shmem_file_addr;
	int				shmem_pages_in;

	struct perf_event_mmap_page	*user_page;
	void				*data_pages[0];
};

extern int rb_inject(struct perf_event *event);
extern int rb_get_kernel_pages(struct perf_event *event);
extern void rb_put_kernel_pages(struct ring_buffer *rb, bool final);
extern void rb_free(struct ring_buffer *rb);
extern void ring_buffer_unaccount(struct ring_buffer *rb, bool aux);

extern int perffs_create_event_file(struct perf_event *event,
				    struct task_struct *task,
				    const struct file_operations *fops);
extern void perffs_remove(struct dentry *dentry);

static inline void rb_free_rcu(struct rcu_head *rcu_head)
{
	struct ring_buffer *rb;

	rb = container_of(rcu_head, struct ring_buffer, rcu_head);
	rb_free(rb);
}

static inline void rb_toggle_paused(struct ring_buffer *rb, bool pause)
{
	if (!pause && rb->nr_pages)
		rb->paused = 0;
	else
		rb->paused = 1;
}

extern struct ring_buffer *
rb_alloc(struct perf_event *event, struct mm_struct *mm, int nr_pages,
	 int flags);
extern void perf_event_wakeup(struct perf_event *event);
extern int rb_alloc_aux(struct ring_buffer *rb, struct perf_event *event,
			pgoff_t pgoff, int nr_pages, long watermark, int flags);
extern void rb_free_aux(struct ring_buffer *rb);
extern int rb_alloc_detached(struct perf_event *event,
			     struct task_struct *task,
			     struct mm_struct *mm,
			     struct ring_buffer *parent_rb);
extern void rb_free_detached(struct ring_buffer *rb, struct perf_event *event);
extern struct ring_buffer *ring_buffer_get(struct perf_event *event);
extern void ring_buffer_put(struct ring_buffer *rb);

static inline bool rb_has_aux(struct ring_buffer *rb)
{
	return !!rb->aux_nr_pages;
}

void perf_event_aux_event(struct perf_event *event, unsigned long head,
			  unsigned long size, u64 flags);

extern struct page *
perf_mmap_to_page(struct ring_buffer *rb, unsigned long pgoff);

#ifdef CONFIG_PERF_USE_VMALLOC
/*
 * Back perf_mmap() with vmalloc memory.
 *
 * Required for architectures that have d-cache aliasing issues.
 */

static inline int page_order(struct ring_buffer *rb)
{
	return rb->page_order;
}

#else

static inline int page_order(struct ring_buffer *rb)
{
	return 0;
}
#endif

static inline unsigned long perf_data_size(struct ring_buffer *rb)
{
	return rb->nr_pages << (PAGE_SHIFT + page_order(rb));
}

static inline unsigned long perf_aux_size(struct ring_buffer *rb)
{
	return rb->aux_nr_pages << PAGE_SHIFT;
}

static inline bool is_detached_event(struct perf_event *event)
{
	lockdep_assert_held(&event->ctx->mutex);
	return !!(event->attach_state & PERF_ATTACH_DETACHED);
}

#define __DEFINE_OUTPUT_COPY_BODY(advance_buf, memcpy_func, ...)	\
{									\
	unsigned long size, written;					\
									\
	do {								\
		size    = min(handle->size, len);			\
		written = memcpy_func(__VA_ARGS__);			\
		written = size - written;				\
									\
		len -= written;						\
		handle->addr += written;				\
		if (advance_buf)					\
			buf += written;					\
		handle->size -= written;				\
		if (!handle->size) {					\
			struct ring_buffer *rb = handle->rb;		\
									\
			handle->page++;					\
			handle->page &= rb->nr_pages - 1;		\
			handle->addr = rb->data_pages[handle->page];	\
			handle->size = PAGE_SIZE << page_order(rb);	\
		}							\
	} while (len && written == size);				\
									\
	return len;							\
}

#define DEFINE_OUTPUT_COPY(func_name, memcpy_func)			\
static inline unsigned long						\
func_name(struct perf_output_handle *handle,				\
	  const void *buf, unsigned long len)				\
__DEFINE_OUTPUT_COPY_BODY(true, memcpy_func, handle->addr, buf, size)

static inline unsigned long
__output_custom(struct perf_output_handle *handle, perf_copy_f copy_func,
		const void *buf, unsigned long len)
{
	unsigned long orig_len = len;
	__DEFINE_OUTPUT_COPY_BODY(false, copy_func, handle->addr, buf,
				  orig_len - len, size)
}

static inline unsigned long
memcpy_common(void *dst, const void *src, unsigned long n)
{
	memcpy(dst, src, n);
	return 0;
}

DEFINE_OUTPUT_COPY(__output_copy, memcpy_common)

static inline unsigned long
memcpy_skip(void *dst, const void *src, unsigned long n)
{
	return 0;
}

DEFINE_OUTPUT_COPY(__output_skip, memcpy_skip)

#ifndef arch_perf_out_copy_user
#define arch_perf_out_copy_user arch_perf_out_copy_user

static inline unsigned long
arch_perf_out_copy_user(void *dst, const void *src, unsigned long n)
{
	unsigned long ret;

	pagefault_disable();
	ret = __copy_from_user_inatomic(dst, src, n);
	pagefault_enable();

	return ret;
}
#endif

DEFINE_OUTPUT_COPY(__output_copy_user, arch_perf_out_copy_user)

static inline int get_recursion_context(int *recursion)
{
	int rctx;

	if (unlikely(in_nmi()))
		rctx = 3;
	else if (in_irq())
		rctx = 2;
	else if (in_softirq())
		rctx = 1;
	else
		rctx = 0;

	if (recursion[rctx])
		return -1;

	recursion[rctx]++;
	barrier();

	return rctx;
}

static inline void put_recursion_context(int *recursion, int rctx)
{
	barrier();
	recursion[rctx]--;
}

#ifdef CONFIG_HAVE_PERF_USER_STACK_DUMP
static inline bool arch_perf_have_user_stack_dump(void)
{
	return true;
}

#define perf_user_stack_pointer(regs) user_stack_pointer(regs)
#else
static inline bool arch_perf_have_user_stack_dump(void)
{
	return false;
}

#define perf_user_stack_pointer(regs) 0
#endif /* CONFIG_HAVE_PERF_USER_STACK_DUMP */

#endif /* _KERNEL_EVENTS_INTERNAL_H */
