/*  linux/include/linux/clocksource.h
 *
 *  This file contains the structure definitions for clocksources.
 *
 *  If you are not a clocksource, or timekeeping code, you should
 *  not be including this file!
 */
#ifndef _LINUX_CLOCKSOURCE_H
#define _LINUX_CLOCKSOURCE_H

#include <linux/types.h>
#include <linux/timex.h>
#include <linux/time.h>
#include <linux/list.h>
#include <linux/cache.h>
#include <linux/timer.h>
#include <asm/div64.h>
#include <asm/io.h>

/* clocksource cycle base type */
typedef u64 cycle_t;
struct clocksource;

/**
 * struct clocksource - hardware abstraction for a free running counter
 *	Provides mostly state-free accessors to the underlying hardware.
 *      Also provides utility functions which convert the underlying
 *      hardware cycle values into a non-decreasing count of nanoseconds
 *      ("time").
 *
 * @name:		ptr to clocksource name
 * @list:		list head for registration
 * @rating:		rating value for selection (higher is better)
 *			To avoid rating inflation the following
 *			list should give you a guide as to how
 *			to assign your clocksource a rating
 *			1-99: Unfit for real use
 *				Only available for bootup and testing purposes.
 *			100-199: Base level usability.
 *				Functional for real use, but not desired.
 *			200-299: Good.
 *				A correct and usable clocksource.
 *			300-399: Desired.
 *				A reasonably fast and accurate clocksource.
 *			400-499: Perfect
 *				The ideal clocksource. A must-use where
 *				available.
 * @read:		returns a cycle value
 * @read_clock:         alternative to read which gets a pointer to the clock
 *                      source so that the same code can read different clocks;
 *                      either read or read_clock must be set
 * @mask:		bitmask for two's complement
 *			subtraction of non 64 bit counters
 * @mult:		cycle to nanosecond multiplier (adjusted by NTP)
 * @mult_orig:		cycle to nanosecond multiplier (unadjusted by NTP)
 * @shift:		cycle to nanosecond divisor (power of two)
 * @flags:		flags describing special properties
 * @vread:		vsyscall based read
 * @resume:		resume function for the clocksource, if necessary
 * @cycle_interval:	Used internally by timekeeping core, please ignore.
 * @xtime_interval:	Used internally by timekeeping core, please ignore.
 */
struct clocksource {
	/*
	 * First part of structure is read mostly
	 */
	char *name;
	struct list_head list;
	int rating;
	cycle_t (*read)(void);
        cycle_t (*read_clock)(struct clocksource *cs);
	cycle_t mask;
	u32 mult;
	u32 mult_orig;
	u32 shift;
	unsigned long flags;
	cycle_t (*vread)(void);
	void (*resume)(void);
#ifdef CONFIG_IA64
	void *fsys_mmio;        /* used by fsyscall asm code */
#define CLKSRC_FSYS_MMIO_SET(mmio, addr)      ((mmio) = (addr))
#else
#define CLKSRC_FSYS_MMIO_SET(mmio, addr)      do { } while (0)
#endif

	/* timekeeping specific data, ignore */
	cycle_t cycle_interval;
	u64	xtime_interval;
	u32	raw_interval;
	/*
	 * Second part is written at each timer interrupt
	 * Keep it in a different cache line to dirty no
	 * more than one cache line.
	 */
	cycle_t cycle_last ____cacheline_aligned_in_smp;
	u64 xtime_nsec;
	s64 error;
	struct timespec raw_time;

#ifdef CONFIG_CLOCKSOURCE_WATCHDOG
	/* Watchdog related data, used by the framework */
	struct list_head wd_list;
	cycle_t wd_last;
#endif
};

extern struct clocksource *clock;	/* current clocksource */

/*
 * Clock source flags bits::
 */
#define CLOCK_SOURCE_IS_CONTINUOUS		0x01
#define CLOCK_SOURCE_MUST_VERIFY		0x02

#define CLOCK_SOURCE_WATCHDOG			0x10
#define CLOCK_SOURCE_VALID_FOR_HRES		0x20

/* simplify initialization of mask field */
#define CLOCKSOURCE_MASK(bits) (cycle_t)((bits) < 64 ? ((1ULL<<(bits))-1) : -1)

/**
 * clocksource_khz2mult - calculates mult from khz and shift
 * @khz:		Clocksource frequency in KHz
 * @shift_constant:	Clocksource shift factor
 *
 * Helper functions that converts a khz counter frequency to a timsource
 * multiplier, given the clocksource shift value
 */
static inline u32 clocksource_khz2mult(u32 khz, u32 shift_constant)
{
	/*  khz = cyc/(Million ns)
	 *  mult/2^shift  = ns/cyc
	 *  mult = ns/cyc * 2^shift
	 *  mult = 1Million/khz * 2^shift
	 *  mult = 1000000 * 2^shift / khz
	 *  mult = (1000000<<shift) / khz
	 */
	u64 tmp = ((u64)1000000) << shift_constant;

	tmp += khz/2; /* round for do_div */
	do_div(tmp, khz);

	return (u32)tmp;
}

/**
 * clocksource_hz2mult - calculates mult from hz and shift
 * @hz:			Clocksource frequency in Hz
 * @shift_constant:	Clocksource shift factor
 *
 * Helper functions that converts a hz counter
 * frequency to a timsource multiplier, given the
 * clocksource shift value
 */
static inline u32 clocksource_hz2mult(u32 hz, u32 shift_constant)
{
	/*  hz = cyc/(Billion ns)
	 *  mult/2^shift  = ns/cyc
	 *  mult = ns/cyc * 2^shift
	 *  mult = 1Billion/hz * 2^shift
	 *  mult = 1000000000 * 2^shift / hz
	 *  mult = (1000000000<<shift) / hz
	 */
	u64 tmp = ((u64)1000000000) << shift_constant;

	tmp += hz/2; /* round for do_div */
	do_div(tmp, hz);

	return (u32)tmp;
}

/**
 * clocksource_read: - Access the clocksource's current cycle value
 * @cs:		pointer to clocksource being read
 *
 * Uses the clocksource to return the current cycle_t value
 */
static inline cycle_t clocksource_read(struct clocksource *cs)
{
	return (cs->read ? cs->read() : cs->read_clock(cs));
}

/**
 * cyc2ns - converts clocksource cycles to nanoseconds
 * @cs:		Pointer to clocksource
 * @cycles:	Cycles
 *
 * Uses the clocksource and ntp ajdustment to convert cycle_ts to nanoseconds.
 *
 * XXX - This could use some mult_lxl_ll() asm optimization
 */
static inline s64 cyc2ns(struct clocksource *cs, cycle_t cycles)
{
	u64 ret = (u64)cycles;
	ret = (ret * cs->mult) >> cs->shift;
	return ret;
}

/**
 * clocksource_read_ns - get nanoseconds since last call of this function
 *                       (never negative)
 * @cs:         Pointer to clocksource
 *
 * When the underlying cycle counter runs over, this will be handled
 * correctly as long as it does not run over more than once between
 * calls.
 *
 * The first call to this function for a new clock source initializes
 * the time tracking and returns bogus results.
 */
static inline s64 clocksource_read_ns(struct clocksource *cs)
{
	cycle_t cycle_now, cycle_delta;
	s64 ns_offset;

	/* read clocksource: */
	cycle_now = clocksource_read(cs);

	/* calculate the delta since the last clocksource_read_ns: */
	cycle_delta = (cycle_now - cs->cycle_last) & cs->mask;

	/* convert to nanoseconds: */
	ns_offset = cyc2ns(cs, cycle_delta);

	/* update time stamp of clocksource_read_ns call: */
	cs->cycle_last = cycle_now;

	return ns_offset;
}

/**
 * clocksource_init_time - initialize a clock source for use with
 *                         %clocksource_read_time() and
 *                         %clocksource_cyc2time()
 * @cs:            Pointer to clocksource.
 * @start_tstamp:  Arbitrary initial time stamp.
 *
 * After this call the current cycle register (roughly) corresponds to
 * the initial time stamp. Every call to %clocksource_read_time()
 * increments the time stamp counter by the number of elapsed
 * nanoseconds.
 */
static inline void clocksource_init_time(struct clocksource *cs,
					u64 start_tstamp)
{
	cs->cycle_last = clocksource_read(cs);
	cs->xtime_nsec = start_tstamp;
}

/**
 * clocksource_read_time - return nanoseconds since %clocksource_init_time()
 *                         plus the initial time stamp
 * @cs:          Pointer to clocksource.
 *
 * In other words, keeps track of time since the same epoch as
 * the function which generated the initial time stamp. Don't mix
 * with calls to %clocksource_read_ns()!
 */
static inline u64 clocksource_read_time(struct clocksource *cs)
{
	u64 nsec;

	/* increment time by nanoseconds since last call */
	nsec = clocksource_read_ns(cs);
	nsec += cs->xtime_nsec;
	cs->xtime_nsec = nsec;

	return nsec;
}

/**
 * clocksource_cyc2time - convert an absolute cycle time stamp to same
 *                        time base as values returned by
 *                        %clocksource_read_time()
 * @cs:            Pointer to clocksource.
 * @cycle_tstamp:  a value returned by cs->read()
 *
 * Cycle time stamps that are converted correctly as long as they
 * fall into the time interval [-1/2 max cycle count, 1/2 cycle count],
 * with "max cycle count" == cs->mask+1.
 *
 * This avoids situations where a cycle time stamp is generated, the
 * current cycle counter is updated, and then when transforming the
 * time stamp the value is treated as if it was in the future. Always
 * updating the cycle counter would also work, but incurr additional
 * overhead.
 */
static inline u64 clocksource_cyc2time(struct clocksource *cs,
				cycle_t cycle_tstamp)
{
	u64 cycle_delta = (cycle_tstamp - cs->cycle_last) & cs->mask;
	u64 nsec;

	/*
	 * Instead of always treating cycle_tstamp as more recent
	 * than cs->cycle_last, detect when it is too far in the
	 * future and treat it as old time stamp instead.
	 */
	if (cycle_delta > cs->mask / 2) {
		cycle_delta = (cs->cycle_last - cycle_tstamp) & cs->mask;
		nsec = cs->xtime_nsec - cyc2ns(cs, cycle_delta);
	} else {
		nsec = cyc2ns(cs, cycle_delta) + cs->xtime_nsec;
	}

	return nsec;
}

/**
 * clocksource_calculate_interval - Calculates a clocksource interval struct
 *
 * @c:		Pointer to clocksource.
 * @length_nsec: Desired interval length in nanoseconds.
 *
 * Calculates a fixed cycle/nsec interval for a given clocksource/adjustment
 * pair and interval request.
 *
 * Unless you're the timekeeping code, you should not be using this!
 */
static inline void clocksource_calculate_interval(struct clocksource *c,
					  	  unsigned long length_nsec)
{
	u64 tmp;

	/* Do the ns -> cycle conversion first, using original mult */
	tmp = length_nsec;
	tmp <<= c->shift;
	tmp += c->mult_orig/2;
	do_div(tmp, c->mult_orig);

	c->cycle_interval = (cycle_t)tmp;
	if (c->cycle_interval == 0)
		c->cycle_interval = 1;

	/* Go back from cycles -> shifted ns, this time use ntp adjused mult */
	c->xtime_interval = (u64)c->cycle_interval * c->mult;
	c->raw_interval = ((u64)c->cycle_interval * c->mult_orig) >> c->shift;
}


/* used to install a new clocksource */
extern int clocksource_register(struct clocksource*);
extern void clocksource_unregister(struct clocksource*);
extern void clocksource_touch_watchdog(void);
extern struct clocksource* clocksource_get_next(void);
extern void clocksource_change_rating(struct clocksource *cs, int rating);
extern void clocksource_resume(void);

#ifdef CONFIG_GENERIC_TIME_VSYSCALL
extern void update_vsyscall(struct timespec *ts, struct clocksource *c);
extern void update_vsyscall_tz(void);
#else
static inline void update_vsyscall(struct timespec *ts, struct clocksource *c)
{
}

static inline void update_vsyscall_tz(void)
{
}
#endif

#endif /* _LINUX_CLOCKSOURCE_H */
