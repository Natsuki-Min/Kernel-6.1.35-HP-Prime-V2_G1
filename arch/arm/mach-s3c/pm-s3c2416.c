// SPDX-License-Identifier: GPL-2.0
//
// Copyright (c) 2010 Samsung Electronics Co., Ltd.
//		http://www.samsung.com
//
// S3C2416 - PM support (Based on Ben Dooks' S3C2412 PM support)

#include <linux/device.h>
#include <linux/syscore_ops.h>
#include <linux/io.h>
#include <asm/cacheflush.h> // 用于 clean_dcache_area

#include "regs-s3c2443-clock.h"

#include "cpu.h"
#include "pm.h"

#include "s3c2412-power.h"

#ifdef CONFIG_PM_SLEEP
extern void s3c2412_sleep_enter(void);

struct s3c2416_sleep_save {
	unsigned long pc;    // Offset 0x00: R7 -> PC
	unsigned long sctlr; // Offset 0x04: R8 -> CP15 c1 (Control)
	unsigned long ttb;   // Offset 0x08: R9 -> CP15 c2 (Translation Table Base)
	unsigned long dacr;  // Offset 0x0C: R10-> CP15 c3 (Domain Access Control)
};

static struct s3c2416_sleep_save core_save;

static int s3c2416_cpu_suspend(unsigned long arg)
{
	/* enable wakeup sources regardless of battery state */
	__raw_writel(S3C2443_PWRCFG_SLEEP, S3C2443_PWRCFG);

	/* set the mode as sleep, 2BED represents "Go to BED" */
	__raw_writel(0x2BED, S3C2443_PWRMODE);

	s3c2412_sleep_enter();

	pr_info("Failed to suspend the system\n");
	return 1; /* Aborting suspend */
}

static void s3c2416_pm_prepare(void)
{
    uint32_t tmp;
    __raw_writel(virt_to_phys(s3c_cpu_resume), S3C2412_INFORM0);
    __raw_writel(virt_to_phys(s3c_cpu_resume), S3C2412_INFORM1);

    core_save.pc = __pa_symbol(s3c_cpu_resume);
    
    core_save.ttb = __pa(swapper_pg_dir);

    asm("mrc p15, 0, %0, c3, c0, 0" : "=r" (core_save.dacr));

    asm("mrc p15, 0, %0, c1, c0, 0" : "=r" (tmp));
    tmp &= ~(1 << 0);  // Disable MMU
    tmp &= ~(1 << 2);  // Disable D-Cache
    tmp &= ~(1 << 12); // Disable I-Cache
    core_save.sctlr = tmp;

    pr_info("PM: Saving to core_save.ttb = 0x%08lx (Should be 0x%08x)\n", 
            core_save.ttb, __pa(swapper_pg_dir));

    __cpuc_flush_dcache_area(&core_save, sizeof(struct s3c2416_sleep_save));
    outer_flush_range(__pa(&core_save), __pa(&core_save) + sizeof(struct s3c2416_sleep_save));

    __raw_writel(__pa(&core_save), S3C2412_INFORM3);
    __raw_writel(0x55AA, S3C2412_INFORM2);


}

static int s3c2416_pm_add(struct device *dev, struct subsys_interface *sif)
{
	pm_cpu_prep = s3c2416_pm_prepare;
	pm_cpu_sleep = s3c2416_cpu_suspend;

	return 0;
}

static struct subsys_interface s3c2416_pm_interface = {
	.name		= "s3c2416_pm",
	.subsys		= &s3c2416_subsys,
	.add_dev	= s3c2416_pm_add,
};

static __init int s3c2416_pm_init(void)
{
	return subsys_interface_register(&s3c2416_pm_interface);
}

arch_initcall(s3c2416_pm_init);
#endif

static void s3c2416_pm_resume(void)
{	pr_info("WE GET HERE!!WE RESUME!!\n");
	/* unset the return-from-sleep and inform flags */
	__raw_writel(0x0, S3C2443_PWRMODE);
	__raw_writel(0x0, S3C2412_INFORM0);
	__raw_writel(0x0, S3C2412_INFORM1);
	/* Clear the custom bootloader flags too */
	__raw_writel(0x0, S3C2412_INFORM2);
	__raw_writel(0x0, S3C2412_INFORM3);

}

struct syscore_ops s3c2416_pm_syscore_ops = {
	.resume		= s3c2416_pm_resume,
};

