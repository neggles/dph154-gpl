/*
 * picohdp setup and early boot code plus other random bits.
 *
 * Maintained by Kumar Gala (see MAINTAINERS for contact information)
 *
 * Copyright 2005 Freescale Semiconductor Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/stddef.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/kdev_t.h>
#include <linux/delay.h>
#include <linux/seq_file.h>
#include <linux/of_platform.h>

#include <asm/system.h>
#include <asm/time.h>
#include <asm/machdep.h>
#include <asm/pci-bridge.h>
#include <asm/mpic.h>
#include <mm/mmu_decl.h>
#include <asm/udbg.h>

#include <sysdev/fsl_soc.h>
#include <sysdev/fsl_pci.h>

#ifdef CONFIG_CPM2
#include <asm/cpm2.h>
#include <sysdev/cpm2_pic.h>
#endif

#ifdef CONFIG_PCI
static int picohdp_exclude_device(struct pci_controller *hose,
				   u_char bus, u_char devfn)
{
	if (bus == 0 && PCI_SLOT(devfn) == 0)
		return PCIBIOS_DEVICE_NOT_FOUND;
	else
		return PCIBIOS_SUCCESSFUL;
}
#endif /* CONFIG_PCI */

#ifdef CONFIG_CPM2

static void cpm2_cascade(unsigned int irq, struct irq_desc *desc)
{
	int cascade_irq;

	while ((cascade_irq = cpm2_get_irq()) >= 0)
		generic_handle_irq(cascade_irq);

	desc->chip->eoi(irq);
}

#endif /* CONFIG_CPM2 */

static int __init picohdp_guts_init(void)
{
        struct device_node *np =
            of_find_compatible_node(NULL, NULL, "fsl,mpc8560-guts");
        struct resource r;

        if (NULL == np) {
                printk(KERN_ERR "GUTS init: failed to find guts node\n");
                return -EIO;
        }

        if (of_address_to_resource(np, 0, &r)) {
                printk(KERN_ERR "Failed to map GUTS register space\n");
                of_node_put(np);
                return -EIO;
        }

        if (NULL == platform_device_register_simple("mpc8560-guts",
                                                    0, &r, 1)) {
                printk(KERN_ERR "Failed to register guts device\n");
                of_node_put(np);
                return -EIO;
        }

        of_node_put(np);

        return 0;
}
arch_initcall(picohdp_guts_init);

static int __init picohdp_pa_init(void)
{
        struct device_node *np;
        /* 4 resources - procif, dma base, interrupt and ccr. */
        struct resource res[4];
        struct platform_device *pdev;
        int cindex;
        const unsigned int *prop;
        int prop_len;
        int ret;
        unsigned num_props;

        memset(res, 0, sizeof(res));

        for_each_compatible_node(np, NULL, "picoArray") {
                num_props = 0;

                prop = of_get_property(np, "cell-index", NULL);
                if (!prop) {
                        printk(KERN_ERR "pA: failed to get cell-index\n");
                        return -EINVAL;
                }
                cindex = (int)*prop;

                pdev = platform_device_alloc("picoArray", (int)*prop);
                if (!pdev) {
                        printk(KERN_ERR
                               "pA: failed to create platform device\n");
                        return -EIO;
                }

                prop = of_get_property(np, "procif", &prop_len);
                if (!prop || prop_len != 8) {
                        printk(KERN_ERR "invalid procif attribute\n");
                        platform_device_put(pdev);
                        return -EIO;
                } else {
                        ++num_props;
                        res[0].start = (int)(prop[0]);
                        res[0].end = (int)prop[1] + res[0].start - 1;
                        res[0].flags = IORESOURCE_MEM;
                        res[0].name = "procif";
                }

                prop = of_get_property(np, "dma_base", &prop_len);
                if (!prop || prop_len != 8) {
                        printk(KERN_ERR "invalid dma base attribute\n");
                        platform_device_put(pdev);
                        return -EIO;
                } else {
                        ++num_props;
                        res[1].start = (int)prop[0];
                        res[1].end = (int)prop[1] + res[1].start - 1;
                        res[1].flags = IORESOURCE_MEM;
                        res[1].name = "dma_base";
                }

                if (NO_IRQ == of_irq_to_resource(np, 0, &res[2])) {
                        printk(KERN_ERR "failed to get IRQ\n");
                        return -EIO;
                }
                res[2].name = "procif_irq";
                ++num_props;

                prop = of_get_property(np, "ccr_base", &prop_len);
                if (prop) {
                        if (prop_len != 8) {
                                printk(KERN_ERR "invalid ccr base attribute\n");
                                platform_device_put(pdev);
                                return -EIO;
                        } else {
                                ++num_props;
                                res[3].start = (int)prop[0];
                                res[3].end = (int)prop[1] + res[3].start - 1;
                                res[3].flags = IORESOURCE_MEM;
                                res[3].name = "ccr_base";
                        }
                }

                ret = platform_device_add_resources(pdev, res, num_props);
                ret = platform_device_add(pdev); }

        return 0;
}
arch_initcall(picohdp_pa_init);

static int __init picohdp_dma_init(void)
{
        struct device_node *np;
        struct resource r[2];
        const unsigned *prop;

        for_each_compatible_node(np, NULL, "fsl,mpc8560-dma-channel") {
            memset(r, 0, sizeof(r));
            if (NULL == np) {
                printk(KERN_ERR "DMA init: failed to find dma channel node\n");
                return -EIO;
            }

            if (of_address_to_resource(np, 0, &r[0])) {
                printk(KERN_ERR "Failed to map dma register space\n");
                of_node_put(np);
                return -EIO;
            }

            if (NO_IRQ == of_irq_to_resource(np, 0, &r[1])) {
                printk(KERN_ERR "Failed to map dma IRQ\n");
                of_node_put(np);
                return -EIO;
            }

            prop = of_get_property(np, "cell-index", NULL);
            if (!prop) {
                printk(KERN_ERR "failed to get dma cell-index\n");
                of_node_put(np);
                return -EIO;
            }

            if (NULL == platform_device_register_simple("dma-chan",
                        *(int *)prop, r, 2)) {
                printk(KERN_ERR "Failed to register dma device\n");
                of_node_put(np);
                return -EIO;
            }
        }

        return 0;
}
arch_initcall(picohdp_dma_init);

static void __init picohdp_pic_init(void)
{
	struct mpic *mpic;
	struct resource r;
	struct device_node *np = NULL;
#ifdef CONFIG_CPM2
	int irq;
#endif

	np = of_find_node_by_type(np, "open-pic");
	if (!np) {
		printk(KERN_ERR "Could not find open-pic node\n");
		return;
	}

	if (of_address_to_resource(np, 0, &r)) {
		printk(KERN_ERR "Could not map mpic register space\n");
		of_node_put(np);
		return;
	}

	mpic = mpic_alloc(np, r.start,
			MPIC_PRIMARY | MPIC_WANTS_RESET | MPIC_BIG_ENDIAN,
			0, 256, " OpenPIC  ");
	BUG_ON(mpic == NULL);
	of_node_put(np);

	mpic_init(mpic);

#ifdef CONFIG_CPM2
	/* Setup CPM2 PIC */
	np = of_find_compatible_node(NULL, NULL, "fsl,cpm2-pic");
	if (np == NULL) {
		printk(KERN_ERR "PIC init: can not find fsl,cpm2-pic node\n");
		return;
	}
	irq = irq_of_parse_and_map(np, 0);

	cpm2_pic_init(np);
	of_node_put(np);
	set_irq_chained_handler(irq, cpm2_cascade);

#endif
}

/*
 * Setup the architecture
 */
static void __init picohdp_setup_arch(void)
{
#ifdef CONFIG_PCI
	struct device_node *np;
#endif

	if (ppc_md.progress)
		ppc_md.progress("picohdp_setup_arch()", 0);

#ifdef CONFIG_CPM2
	cpm2_reset();
#endif

#ifdef CONFIG_PCI
	for_each_compatible_node(np, "pci", "fsl,mpc8540-pci")
		fsl_add_bridge(np, 1);

	ppc_md.pci_exclude_device = picohdp_exclude_device;
#endif
}

static void picohdp_show_cpuinfo(struct seq_file *m)
{
	uint pvid, svid, phid1;
	uint memsize = total_memory;

	pvid = mfspr(SPRN_PVR);
	svid = mfspr(SPRN_SVR);

	seq_printf(m, "Vendor\t\t: Freescale Semiconductor\n");
	seq_printf(m, "Machine\t\t: picohdp\n");
	seq_printf(m, "PVR\t\t: 0x%x\n", pvid);
	seq_printf(m, "SVR\t\t: 0x%x\n", svid);

	/* Display cpu Pll setting */
	phid1 = mfspr(SPRN_HID1);
	seq_printf(m, "PLL setting\t: 0x%x\n", ((phid1 >> 24) & 0x3f));

	/* Display the amount of memory */
	seq_printf(m, "Memory\t\t: %d MB\n", memsize / (1024 * 1024));
}

static struct of_device_id __initdata of_bus_ids[] = {
	{ .name = "soc", },
	{ .type = "soc", },
	{ .name = "cpm", },
	{ .name = "localbus", },
	{ .compatible = "simple-bus", },
	{},
};

static int __init declare_of_platform_devices(void)
{
	of_platform_bus_probe(NULL, of_bus_ids, NULL);

	return 0;
}
machine_device_initcall(mpc85xx_ads, declare_of_platform_devices);

extern void abort(void);

void picohdp_restart(char *cmd)
{
        abort();

	while (1) ;
}

/*
 * Called very early, device-tree isn't unflattened
 */
static int __init picohdp_probe(void)
{
        unsigned long root = of_get_flat_dt_root();

        return of_flat_dt_is_compatible(root, "MPC85xxADS");
}

define_machine(mpc85xx_ads) {
	.name			= "MPC85xx ADS",
	.probe			= picohdp_probe,
	.setup_arch		= picohdp_setup_arch,
	.init_IRQ		= picohdp_pic_init,
	.show_cpuinfo		= picohdp_show_cpuinfo,
	.get_irq		= mpic_get_irq,
	.restart		= picohdp_restart,
	.calibrate_decr		= generic_calibrate_decr,
	.progress		= udbg_progress,
};
