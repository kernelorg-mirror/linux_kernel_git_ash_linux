/*
 * Intel(R) Real-Time Instruction Trace PMU driver for perf
 * Copyright (c) 2015, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * Intel PT is specified in the Intel Architecture Instruction Set Extensions
 * Programming Reference:
 * http://software.intel.com/en-us/intel-isa-extensions
 */

#undef DEBUG

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/types.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/sizes.h>
#include <linux/module.h>

#include "../perf_event.h"
#include <asm/intel_pt.h>
#include "pt.h"

/*
 * Intel RTIT is not architectural
 */
#define MSR_RTIT_CTL			0x00000768
#define RTIT_CTL_DRAM			BIT(8)
#define RTIT_CTL_STS_ON_CR3		BIT(4)
#define RTIT_CTL_STS_EN			BIT(10)
#define RTIT_CTL_RETCOMP		BIT(11)
#define RTIT_CTL_LESS_PKTS		BIT(12)
#define RTIT_CTL_TRACEACTIVE		BIT(13)
#define MSR_RTIT_STATUS			0x00000769
#define MSR_RTIT_EVENTS			0x0000076c
#define RTIT_EVENTS_FILTER_OFFSET	0
#define RTIT_EVENTS_TRACESTOP_OFFSET	3
#define RTIT_EVENTS_ALWAYS_OFF		6
#define RTIT_EVENTS_ALWAYS_ON		7
#define MSR_RTIT_BASE_ADDR		0x00000770
#define MSR_RTIT_LIMIT_MASK		0x00000771
#define MSR_RTIT_OFFSET			0x00000772

struct rtit_buffer {
	void		*buf;
	local_t		head;
	local_t		data_size;
	unsigned long	nr_pages;
	void		**data_pages;
};

/**
 * struct rtit - per-cpu rtit context
 * @handle:	perf output handle
 */
struct rtit {
	struct perf_output_handle handle;
};

static DEFINE_PER_CPU(struct rtit, rtit_ctx);

struct pmu rtit_pmu;

PMU_FORMAT_ATTR(cyc,		"config:1"	);
PMU_FORMAT_ATTR(sts_on_cr3,	"config:4"	);
PMU_FORMAT_ATTR(mtc,		"config:9"	);
PMU_FORMAT_ATTR(sts,		"config:10"	);
PMU_FORMAT_ATTR(retcomp,	"config:11"	);
PMU_FORMAT_ATTR(lesspackets,	"config:12"	);
PMU_FORMAT_ATTR(mtc_period,	"config:14-15"	);

static struct attribute *rtit_formats_attr[] = {
	&format_attr_cyc.attr,
	&format_attr_sts_on_cr3.attr,
	&format_attr_mtc.attr,
	&format_attr_sts.attr,
	&format_attr_retcomp.attr,
	&format_attr_lesspackets.attr,
	&format_attr_mtc_period.attr,
	NULL,
};

static struct attribute_group rtit_format_group = {
	.name	= "format",
	.attrs	= rtit_formats_attr,
};

static const struct attribute_group *rtit_attr_groups[] = {
	&rtit_format_group,
	NULL,
};

static void *
rtit_buffer_setup_aux(struct perf_event *event, void **pages, int nr_pages, bool snapshot)
{
	struct rtit_buffer *buf;
	int node, cpu = event->cpu;

	if (!nr_pages || !snapshot)
		return NULL;

	if (nr_pages << PAGE_SHIFT > SZ_4M)
		return NULL;

	if (cpu == -1)
		cpu = raw_smp_processor_id();
	node = cpu_to_node(cpu);

	buf = kzalloc_node(sizeof(*buf), GFP_KERNEL, node);
	if (!buf)
		return NULL;

	buf->nr_pages = nr_pages;
	buf->data_pages = pages;

	return buf;
}

/**
 * rtit_buffer_free_aux() - perf AUX deallocation path callback
 * @data:	RTIT buffer.
 */
static void rtit_buffer_free_aux(void *data)
{
	struct rtit_buffer *buf = data;

	kfree(buf);
}

static void rtit_config_stop(struct perf_event *event)
{
	u64 ctl = READ_ONCE(event->hw.config);

	ctl &= ~RTIT_CTL_TRACEEN;
	wrmsrl(MSR_RTIT_CTL, ctl);
	WRITE_ONCE(event->hw.config, ctl);

	/*
	 * A wrmsr that disables trace generation serializes other RTIT
	 * registers and causes all data packets to be written to memory,
	 * but a fence is required for the data to become globally visible.
	 *
	 * The below WMB, separating data store and aux_head store matches
	 * the consumer's RMB that separates aux_head load and data load.
	 */
	wmb();
}

#define RTIT_CTL_MTC (RTIT_CTL_MTC_EN | RTIT_CTL_MTC_RANGE)

#define RTIT_CONFIG_MASK (RTIT_CTL_TRACEEN	| \
			  RTIT_CTL_STS_ON_CR3	| \
			  RTIT_CTL_MTC		| \
			  RTIT_CTL_STS_EN	| \
			  RTIT_CTL_RETCOMP	| \
			  RTIT_CTL_LESS_PKTS)

static void rtit_config(struct perf_event *event, struct rtit_buffer *buf)
{
	u64 reg;

	/* set up filtering and TraceStop */
	reg =
		RTIT_EVENTS_ALWAYS_ON << RTIT_EVENTS_FILTER_OFFSET |
		RTIT_EVENTS_ALWAYS_OFF << RTIT_EVENTS_TRACESTOP_OFFSET;
	wrmsrl(MSR_RTIT_EVENTS, reg);

	/* configure buffer address and size and write cursor*/
	wrmsrl(MSR_RTIT_BASE_ADDR, virt_to_phys(buf->data_pages[0]));
	wrmsrl(MSR_RTIT_LIMIT_MASK, (buf->nr_pages << PAGE_SHIFT) - 1);
	wrmsrl(MSR_RTIT_OFFSET, local_read(&buf->head));

	reg = RTIT_CTL_DRAM | RTIT_CTL_TRACEEN | RTIT_CTL_TRACEACTIVE;

	if (!event->attr.exclude_kernel)
		reg |= RTIT_CTL_OS;
	if (!event->attr.exclude_user)
		reg |= RTIT_CTL_USR;

	reg |= (event->attr.config & RTIT_CONFIG_MASK);
	WRITE_ONCE(event->hw.config, reg);

	wrmsrl(MSR_RTIT_CTL, reg);
}

/*
 * PMU callbacks
 */

static void rtit_event_start(struct perf_event *event, int mode)
{
	struct rtit *rtit = this_cpu_ptr(&rtit_ctx);
	struct rtit_buffer *buf = perf_get_aux(&rtit->handle);

	if (WARN_ON_ONCE(!buf)) {
		event->hw.state = PERF_HES_STOPPED;
		return;
	}

	event->hw.state = 0;

	wrmsrl(MSR_RTIT_STATUS, 0);
	rtit_config(event, buf);
}

static void rtit_event_stop(struct perf_event *event, int mode)
{
	struct rtit *rtit = this_cpu_ptr(&rtit_ctx);

	rtit_config_stop(event);

	if (event->hw.state == PERF_HES_STOPPED)
		return;

	event->hw.state = PERF_HES_STOPPED;

	if (mode & PERF_EF_UPDATE) {
		struct rtit_buffer *buf = perf_get_aux(&rtit->handle);
		u64 off;

		if (!buf)
			return;

		if (WARN_ON_ONCE(rtit->handle.event != event))
			return;

		rdmsrl(MSR_RTIT_OFFSET, off);
		rtit->handle.head = off;

		local_set(&buf->data_size, buf->nr_pages << PAGE_SHIFT);
	}
}

static void rtit_event_del(struct perf_event *event, int mode)
{
	struct rtit *rtit = this_cpu_ptr(&rtit_ctx);
	struct rtit_buffer *buf;

	rtit_event_stop(event, PERF_EF_UPDATE);

	buf = perf_get_aux(&rtit->handle);

	if (buf)
		perf_aux_output_end(&rtit->handle,
				    local_xchg(&buf->data_size, 0));
}

static int rtit_event_add(struct perf_event *event, int mode)
{
	struct rtit_buffer *buf;
	struct rtit *rtit = this_cpu_ptr(&rtit_ctx);
	struct hw_perf_event *hwc = &event->hw;
	int ret = -EBUSY;

	if (rtit->handle.event)
		goto fail;

	buf = perf_aux_output_begin(&rtit->handle, event);
	ret = -EINVAL;
	if (!buf)
		goto fail_stop;

	local_set(&buf->head, rtit->handle.head);

	if (mode & PERF_EF_START) {
		rtit_event_start(event, 0);
		ret = -EBUSY;
		if (hwc->state == PERF_HES_STOPPED)
			goto fail_end_stop;
	} else {
		hwc->state = PERF_HES_STOPPED;
	}

	return 0;

fail_end_stop:
	perf_aux_output_end(&rtit->handle, 0);
fail_stop:
	hwc->state = PERF_HES_STOPPED;
fail:
	return ret;
}

static void rtit_event_read(struct perf_event *event)
{
}

static void rtit_event_destroy(struct perf_event *event)
{
	x86_del_exclusive(x86_lbr_exclusive_pt);
}

static int rtit_event_init(struct perf_event *event)
{
	if (event->attr.type != rtit_pmu.type)
		return -ENOENT;

	if (x86_add_exclusive(x86_lbr_exclusive_pt))
		return -EBUSY;

	event->destroy = rtit_event_destroy;

	return 0;
}

static __init int rtit_init(void)
{
	int ret, cpu, prior_warn = 0;
	u64 ctl;

	/* check the presence of RTIT */
	ret = rdmsrl_safe(MSR_RTIT_CTL, &ctl);
	if (ret)
		return -ENODEV;

	get_online_cpus();
	for_each_online_cpu(cpu) {
		ret = rdmsrl_safe_on_cpu(cpu, MSR_RTIT_CTL, &ctl);
		if (!ret && (ctl & RTIT_CTL_TRACEEN))
			prior_warn++;
	}
	put_online_cpus();

	if (prior_warn) {
		x86_add_exclusive(x86_lbr_exclusive_pt);
		pr_warn("RTIT is enabled at boot time, doing nothing\n");

		return -EBUSY;
	}

	rtit_pmu.capabilities	|=
		PERF_PMU_CAP_EXCLUSIVE | PERF_PMU_CAP_ITRACE |
		PERF_PMU_CAP_AUX_NO_SG;
	rtit_pmu.attr_groups	= rtit_attr_groups;
	rtit_pmu.task_ctx_nr	= perf_sw_context;
	rtit_pmu.event_init	= rtit_event_init;
	rtit_pmu.add		= rtit_event_add;
	rtit_pmu.del		= rtit_event_del;
	rtit_pmu.start		= rtit_event_start;
	rtit_pmu.stop		= rtit_event_stop;
	rtit_pmu.read		= rtit_event_read;
	rtit_pmu.setup_aux	= rtit_buffer_setup_aux;
	rtit_pmu.free_aux	= rtit_buffer_free_aux;
	ret = perf_pmu_register(&rtit_pmu, "intel_pt", -1);

	return ret;
}

module_init(rtit_init);
