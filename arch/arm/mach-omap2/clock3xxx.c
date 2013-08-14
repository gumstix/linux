/*
 * OMAP3-specific clock framework functions
 *
 * Copyright (C) 2007-2008 Texas Instruments, Inc.
 * Copyright (C) 2007-2010 Nokia Corporation
 *
 * Paul Walmsley
 * Jouni Högander
 *
 * Parts of this code are based on code written by
 * Richard Woodruff, Tony Lindgren, Tuukka Tikkanen, Karthik Dasu
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#undef DEBUG

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <plat/hardware.h>
#include <plat/clock.h>

#include "clock.h"
#include "clock3xxx.h"
#include "prm2xxx_3xxx.h"
#include "prm-regbits-34xx.h"
#include "cm2xxx_3xxx.h"
#include "cm-regbits-34xx.h"

/*
 * DPLL5_FREQ_FOR_USBHOST: USBHOST and USBTLL are the only clocks
 * that are sourced by DPLL5, and both of these require this clock
 * to be at 120 MHz for proper operation.
 */
#define DPLL5_FREQ_FOR_USBHOST		120000000

/* needed by omap3_core_dpll_m2_set_rate() */
struct clk *sdrc_ick_p, *arm_fck_p;

struct dpll_settings {
	int rate, m, n, f;
};


static int omap3_dpll5_apply_erratum21(struct clk *clk, struct clk *dpll5_m2)
{
	struct clk *sys_clk;
	int i, rv;
	static const struct dpll_settings precomputed[] = {
		/* From DM3730 errata (sprz319e), table 36
		* N+1 is because the values in the table are register values;
		* dpll_program() will subtract one from the N we give it,
		* so ...
		*/
		{ 13000000, 443, 5+1, 8 },
		{ 26000000, 443, 11+1, 8 }
	};

	sys_clk = clk_get(NULL, "sys_ck");

	for (i = 0 ; i < (sizeof(precomputed)/sizeof(struct dpll_settings)) ;
		++i) {
		const struct dpll_settings *d = &precomputed[i];
		if (sys_clk->rate == d->rate) {
			rv =  omap3_noncore_dpll_program(clk, d->m , d->n, 0);
			if (rv)
				return 1;
			rv =  omap2_clksel_force_divisor(dpll5_m2 , d->f);
			return 1;
		}
	}
	return 0;
}

int omap3_dpll5_set_rate(struct clk *clk, unsigned long rate)
{
	struct clk *dpll5_m2;
	int rv;
	dpll5_m2 = clk_get(NULL, "dpll5_m2_ck");

	if (cpu_is_omap3630() && rate == DPLL5_FREQ_FOR_USBHOST &&
		omap3_dpll5_apply_erratum21(clk, dpll5_m2)) {
		return 1;
	}
	rv = omap3_noncore_dpll_set_rate(clk, rate);
	if (rv)
		goto out;
	rv = clk_set_rate(dpll5_m2, rate);

out:
	return rv;
}

int omap3_dpll4_set_rate(struct clk *clk, unsigned long rate)
{
	/*
	 * According to the 12-5 CDP code from TI, "Limitation 2.5"
	 * on 3430ES1 prevents us from changing DPLL multipliers or dividers
	 * on DPLL4.
	 */
	if (omap_rev() == OMAP3430_REV_ES1_0) {
		pr_err("clock: DPLL4 cannot change rate due to "
		       "silicon 'Limitation 2.5' on 3430ES1.\n");
		return -EINVAL;
	}

	return omap3_noncore_dpll_set_rate(clk, rate);
}

void __init omap3_clk_lock_dpll5(void)
{
	struct clk *dpll5_clk;

	dpll5_clk = clk_get(NULL, "dpll5_ck");
	clk_set_rate(dpll5_clk, DPLL5_FREQ_FOR_USBHOST);

	/* dpll5_m2_ck is now (grottily!) handled by dpll5_clk's set routine,
	 * to cope with an erratum on DM3730
	 */

	return;
}

/* Common clock code */

/*
 * Switch the MPU rate if specified on cmdline.  We cannot do this
 * early until cmdline is parsed.  XXX This should be removed from the
 * clock code and handled by the OPP layer code in the near future.
 */
static int __init omap3xxx_clk_arch_init(void)
{
	int ret;

	if (!cpu_is_omap34xx())
		return 0;

	ret = omap2_clk_switch_mpurate_at_boot("dpll1_ck");
	if (!ret)
		omap2_clk_print_new_rates("osc_sys_ck", "core_ck", "arm_fck");

	return ret;
}

arch_initcall(omap3xxx_clk_arch_init);


