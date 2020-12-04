/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*
 * Copyright (c) 2006-2009 picoChip Designs Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All enquiries to support@picochip.com
 */

/*!
 * \file soft_reset.h
 * \brief Perform a software reset of the picoArray on PC202/PC205
 */

#include <asm/io.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/types.h>

#include "soft_reset.h"
#include "soft_reset_memif.h"

static void __iomem *procif_base;
static void __iomem *ahb2pico_base;

#define CONFIG_READ_OFFSET                0x78
#define CONFIG_WRITE_OFFSET               0x7c

#define CAEID_AHB                         0x0008
#define CAEID_MEMIF                       0x0028
#define CAEID_PROCIF                      0x0048
#define PA_CONFIG_WRITE                   0x00010000
#define PA_CONFIG_READ                    0x00020000
#define PA_CONFIG_ADDR                    0x00040000
#define PA_CONFIG_AEID                    0x00080000
#define PA_CONFIG_VALID                   0x00010000
#define PA_RUN_STEP_REQUEST_REG           0x4018

#define AHBIF_CONFIG_REG                  0x0
#define AHBIF_STATUS_REG                  0x4
#define AHBIF_DATA_REG                    0x8

#define MEMIF_BUFFERS                     6
#define MEMIF_EMPTY_CAEID                 0x2c28
#define OTHER_CTRL_CAEID                  0x2088

#define AHB2PICO_VP_DIR_MASK              0x00000002
#define AHB2PICO_VP_DIR_IS_TX             0x00000000
#define AHB2PICO_VP_DIR_IS_RX             0x00000002
#define AHB2PICO_VP_SNGL_MASK             0x00000800
#define AHB2PICO_VP_STATE_MASK            0x00000008
#define AHB2PICO_VP_LEVEL_MASK            0x000007f0

#define SDGPIO_BASE 0x9800
#define SDGPIO_PINS 32
#define SDGPIO_REG_CONFIG(pin)            (SDGPIO_BASE + (pin * 4))
#define SDGPIO_REG_AVALUE(pin)            (SDGPIO_BASE + (pin * 4) + 1)
#define SDGPIO_REG_ARATE(pin)             (SDGPIO_BASE + (pin * 4) + 2)
#define SDGPIO_REG_GROUP                  (SDGPIO_BASE + 0x86)

#define DMA_REG_THRESHOLD(dma)            (dma * 4 + 1)
#define DMA_REG_DREQ(dma)                 (dma * 4 + 2)
#define DMA_REG_POINTER(dma)              (dma * 4 + 3)

static inline void reset_out32(void __iomem *base, u32 offset, u32 value)
{
    iowrite32(value, base + offset);
}

static inline u32 reset_in32(void __iomem *base, u32 offset)
{
    return ioread32(base + offset);
}

/*!
 * Read data from the config bus
 *
 * \param caeid The AEID to select
 * \param addr The address to start reading from
 * \param buf[out] The buffer to write the data to
 * \param len The number of u16 words to read
 * \return 0 on success, -1 on failure
 */
static int config_read_data (u16 caeid, u16 addr, u16 *buf, u16 len)
{
    u32 data;
    int i;

    reset_out32(procif_base, CONFIG_WRITE_OFFSET, PA_CONFIG_AEID | caeid);
    reset_out32(procif_base, CONFIG_WRITE_OFFSET, PA_CONFIG_ADDR | addr);
    for (i = 0; i < len; ++i)
    {
        reset_out32(procif_base, CONFIG_WRITE_OFFSET, PA_CONFIG_READ);
        do
        {
            data = reset_in32(procif_base, CONFIG_READ_OFFSET);
        }
        while (!data);

        if (data & PA_CONFIG_VALID)
            buf[i] = (u16)(data & 0xffff);
        else
            return -1;
    }

    return 0;
}

/*!
 * Read a single word of data from the config bus
 *
 * \param caeid The AEID to select
 * \param addr The address to read
 * \return The 16-bit data read from the picoArray
 */
static u16 config_read_word (u16 caeid, u16 addr)
{
    u32 data;

    reset_out32(procif_base, CONFIG_WRITE_OFFSET, PA_CONFIG_AEID | caeid);
    reset_out32(procif_base, CONFIG_WRITE_OFFSET, PA_CONFIG_ADDR | addr);
    reset_out32(procif_base, CONFIG_WRITE_OFFSET, PA_CONFIG_READ);
    do
    {
        data = reset_in32(procif_base, CONFIG_READ_OFFSET);
    }
    while (!data);

    if (data & PA_CONFIG_VALID)
        return (u16)(data & 0xffff);

    pr_err("softreset: read failure, caeid: 0x%04x addr 0x%04x\n", caeid, addr);
    return 0;
}

/*!
 * Write a single 16-bit word of data to the config bus
 *
 * \param caeid The AEID to select
 * \param addr The address to write to
 * \param data The data to write
 */
static void config_write_word (u16 caeid, u16 addr, u16 data)
{
    reset_out32(procif_base, CONFIG_WRITE_OFFSET, PA_CONFIG_AEID | caeid);
    reset_out32(procif_base, CONFIG_WRITE_OFFSET, PA_CONFIG_ADDR | addr);
    reset_out32(procif_base, CONFIG_WRITE_OFFSET, PA_CONFIG_WRITE | data);
}

/*!
 * Write a stream of configuration port data into the picoArray
 *
 * \param buf A buffer of u32 data items to write into the config port
 * \param len The number of u32 data items in buf
 */
static void config_stream_write (const u32 *buf, u16 len)
{
    int i;

    for (i = 0; i < len; ++i)
        reset_out32(procif_base, CONFIG_WRITE_OFFSET, buf[i]);
}

/*!
 * Start the picoArray
 */
static void reset_run_system (void)
{
    config_write_word (CAEID_PROCIF, PA_RUN_STEP_REQUEST_REG, 2);
}

/*!
 * Stop the picoArray
 */
static void reset_stop_system (void)
{
    config_write_word(CAEID_PROCIF, PA_RUN_STEP_REQUEST_REG, 0);
}

/*!
 * Reset just the hardware ports of an AE
 *
 * \param caeid The CAEID of the AE to reset
 */
static void reset_ae_hwp (u16 caeid)
{
    config_write_word(caeid, 0xa065, 2);
    config_write_word(caeid, 0xa063, 1);
    udelay(10);
    config_write_word(caeid, 0xa063, 0);
    config_write_word(caeid, 0xa065, 3);
}

/*!
 * Reset all AEs except for the ProcIf, MemIf and AHB2pico. We can't just reset
 * these ones because there is no there is no synchronisation between the
 * domains and it would kill the ARM, so just reset their HWPs. Send them to
 * sleep after they are reset.
 */
static void reset_programmable_aes (void)
{
    config_write_word(0xef9f, 0xa063, 1);
    udelay(10);
    config_write_word(0xef9f, 0xa063, 0);
    udelay(10);
    /* Reset all other HWPs */
    reset_ae_hwp(CAEID_MEMIF);
    reset_ae_hwp(CAEID_AHB);
    reset_ae_hwp(CAEID_PROCIF);
    /* Put all AEs except the ProcIf to sleep */
    config_write_word(0xffbf, 0xa060, 1);
}

/*!
 * Write data into the ProcIf, decode region 0
 *
 * \param regNum The register number to write to (pad_ebi_addr[6:0]/4)
 * \param value The data to write
 */
static void write_procif_data(u8 regNum, u32 value)
{
    reset_out32(procif_base, regNum * 4, value);
}

/*!
 * Read data from the ProcIf, decode region 0
 *
 * \param regNum The register number to read (pad_ebi_addr[6:0]/4)
 * \return The data in that register
 */
static u32 read_procif_data (u8 regNum)
{
    return reset_in32(procif_base, regNum * 4);
}

/*!
 * Write a value into an AHB2pico GPR or DMA register
 *
 * \param vp The VP number of the DMA or GPR
 * \param offset The offset from the start of the VP registers
 * \param value The value to write to the register
 */
static void set_ahb_vp_register (u8 vp, u8 offset, u32 value)
{
    reset_out32(ahb2pico_base, (vp<<4) + offset, value);
}

/*!
 * Read a DMA or GPR register from the AHB2pico.
 *
 * \param vp The VP of the DMA or GPR to read
 * \param offset The offset from the start of the VP registers
 * \return The data from the register
 */
static u32 get_ahb_vp_register (u8 vp, u8 offset)
{
    return reset_in32(ahb2pico_base, (vp<<4) + offset);
}

/*!
 * From the VP (DMAC[0:3], GPR[0:24]) return the HWP instance and signal port
 * number. This mapping is in section 3 of the ahb2pico specs.
 *
 * \param vp The virtual port
 * \param hwp[out] The HWP instance the VP is on
 * \param signal[out] The signal port number
 */
static void map_vp2hwp (u8 vp, u8 *hwp, u8 *signal)
{
    if(vp < 2 ) {
        *hwp = 0;
        *signal = vp;
    } else if(vp < 4) {
        *hwp = 1;
        *signal = vp + 11;
    } else if(vp < 17) {
        *hwp = 0;
        *signal = vp - 2;
    } else {
        *hwp = 1;
        *signal = vp - 17;
    }
}

/*!
 * Get the ConfigLow register of a HWP
 *
 * \param caeid The CAEID
 * \param port The HWP number to read the config value of
 * \param inst The HWP block instance the port is on
 * \return The 16-bit value of the register
 */
static u16 get_hwp_config_low (u16 caeid, u8 port, u8 inst)
{
    unsigned int port_address = 0x9000 + (port<< 3) + (inst<< 8);

    return config_read_word(caeid, port_address);
}

/*!
 * Set a HWP ConfigLow register
 *
 * \param caeid The CAEID
 * \param port The HWP number to set
 * \param inst The HWP block instance the port is on
 * \param value The new value of the ConfigLow register
 */
static void set_hwp_config_low (u16 caeid, u8 port, u8 inst, u16 value)
{
    unsigned int hwp_address = 0x9000 + (port << 3) + (inst << 8);
    config_write_word(caeid, hwp_address, value);
}

/*!
 * Set a HWP to non-blocking
 *
 * \param caeid The CAEID
 * \param port The HWP number to set
 * \param inst The HWP block instance the port is on
 */
static void set_hwp_nonblocking(u16 caeid, u8 port, u8 inst)
{
    u16 config_low = get_hwp_config_low(caeid, port, inst);
    config_low |= 5; /* Non-blocking, Right bus enabled */
    set_hwp_config_low(caeid, port, inst, config_low);
}

/*!
 * Disable a HWP
 *
 * \param caeid The CAEID
 * \param port The HWP number to disable
 * \param inst The HWP block instance the port is on
 */
static void disable_hwp (u16 caeid, u8 port, u8 inst)
{
    u16 config_low = get_hwp_config_low(caeid, port, inst);
    config_low &= ~(3); /* Clear PortEnable */
    set_hwp_config_low(caeid, port, inst, config_low);
}

/*!
 * Reset the AHB2pico
 */
static void reset_ahb2pico (int print_errors)
{
    u32 data, fifo_level;
    u8 vp, pin, hwp, signal;

    /* For each DMA VP check the 'single' bit of the status register and read out
     * data if it is set. This bit is used rather than the 'state' bit since there
     * might not be a full block in the DMA.  */
    for (vp = 0; vp < 4 ; ++vp )
    {
        data = get_ahb_vp_register(vp, AHBIF_STATUS_REG);
        if ((data & AHB2PICO_VP_DIR_MASK) == AHB2PICO_VP_DIR_IS_RX)
            while (data & AHB2PICO_VP_SNGL_MASK)
            {
                (void)get_ahb_vp_register(vp, AHBIF_DATA_REG);
                data = get_ahb_vp_register(vp, AHBIF_STATUS_REG);
            }
    }

    /* For each GPR VP check the 'state' bit of the status register and read out
     * data if it is set. Don't do number 29 since this is the fuse.  */
    for (vp = 4; vp < 29 ; ++vp )
    {
        data = get_ahb_vp_register(vp, AHBIF_STATUS_REG);
        if ((data & AHB2PICO_VP_DIR_MASK) == AHB2PICO_VP_DIR_IS_RX)
            while (data & AHB2PICO_VP_STATE_MASK)
            {
                (void)get_ahb_vp_register(vp, AHBIF_DATA_REG);
                data = get_ahb_vp_register(vp, AHBIF_STATUS_REG);
            }
    }

    /* All VPs, whether DMA or GPR can be drained the same way on the pA side.
     * Read the status register to find out the port direction:
     *   AHB->pA: set to non-blocking to drain data
     *   pA->AHB: disable so new new data can get added */
    for (vp = 0; vp < 29 ; vp++ )
    {
        map_vp2hwp(vp, &hwp, &signal);
        data = get_ahb_vp_register(vp, AHBIF_STATUS_REG);
        if ((data & AHB2PICO_VP_DIR_MASK) == AHB2PICO_VP_DIR_IS_TX)
            set_hwp_nonblocking(CAEID_AHB, signal, hwp);
        else
            disable_hwp(CAEID_AHB, signal, hwp);
    }

    /* This runs the currently loaded design for a short while because we can't
     * just run the AHB2pico. Make sure the currently loaded design can't harm
     * anything */
    config_write_word(0xffbf, 0xa060, 1);
    reset_run_system();
    reset_stop_system();

    /* Check the DMA FIFO fill levels are correct - all the data has been drained
     * out. A TX DMA should have space for 64 writes and an RX should contain no
     * data */
    for (vp = 0; vp < 4 ; vp++ ) {
      data = get_ahb_vp_register(vp, AHBIF_STATUS_REG);
      fifo_level = (data & AHB2PICO_VP_LEVEL_MASK) >> 4;
      if ((data & AHB2PICO_VP_DIR_MASK) == AHB2PICO_VP_DIR_IS_TX)
      {
          if(fifo_level != 64)
              if (print_errors)
                  pr_err("softreset: error resetting DMA%d TX FIFO (%d)\n", vp,
                         fifo_level);
      }
      else if (fifo_level != 0)
      {
          if (print_errors)
              pr_err("softreset: error resetting DMA%d RX FIFO (%d)\n", vp,
                     fifo_level);
      }
    }

    reset_ae_hwp(CAEID_AHB);

    /* Set all AHB-side registers to post-reset values */
    for (vp = 0; vp < 29 ; vp++ )
        set_ahb_vp_register(vp, AHBIF_CONFIG_REG, 0x00000000);

    /* Reset all of the picoArray controlled SD-GPIO pins to their reset values.
     * Some pins may be controlled by the ARM and we should leave those alone */
    for (pin = 0; pin < SDGPIO_PINS; ++pin)
    {
        data = config_read_word(CAEID_AHB, SDGPIO_REG_CONFIG(pin));
        if (!(data & 0x8000))
            continue; /* Skip if Control Source bit is 0 (config controlled) */
        config_write_word(CAEID_AHB, SDGPIO_REG_CONFIG(pin), 0x000f);
        config_write_word(CAEID_AHB, SDGPIO_REG_AVALUE(pin), 0x0000);
        config_write_word(CAEID_AHB, SDGPIO_REG_ARATE(pin), 0x0000);
    }

    /* Reset picoBus grouping of pins */
    config_write_word(CAEID_AHB, SDGPIO_REG_GROUP, 0x0000);
}

/*!
 * Reset the MemIf and GprSrc registers. This must be done from inside the
 * picoArray.
 */
static int reset_memif_gpr (void)
{
    int buffer, i, ertc_moved = 0;
    s32 data;
    u16 activeBuffers = 0, ertc_start[4], ertc_end[4];

    /* Wakeup the MemIf */
    config_write_word(CAEID_MEMIF, 0xa060, 0);

    /* Determine which MEMIF buffers are in use by the current pA design */
    for (buffer = 0; buffer < MEMIF_BUFFERS; ++buffer)
    {
        data = config_read_word(CAEID_MEMIF, buffer * 0x0004);
        if ((data & 0x000f) != 0x0000)
            activeBuffers |= (1 << buffer);
    }

    /* Reset the HWPs and clear error flags in the ProcIf*/
    reset_ae_hwp(CAEID_MEMIF);
    config_write_word(CAEID_PROCIF, 0xa030, 0);
    config_write_word(CAEID_PROCIF, 0xa031, 0);
    config_write_word(CAEID_PROCIF, 0xa032, 0);

    /* Load the config file and set r9 to the active buffers */
    config_stream_write(soft_reset_memif, (sizeof(soft_reset_memif)/4));
    config_write_word(MEMIF_EMPTY_CAEID, 0xa009, activeBuffers);
    data = config_read_word(MEMIF_EMPTY_CAEID, 0xa060);
    if (data != 0)
        pr_err("softreset: incorrect MEMIF CAEID\n");
    data = config_read_word(OTHER_CTRL_CAEID, 0xa060);
    if (data != 1)
        pr_err("softreset: incorrect MEMIF CAEID\n");

    /* Get the ERTC so we can check it ran, then start it */
    if (config_read_data(CAEID_PROCIF, 0x4008, ertc_start, 4) == -1)
        pr_err("softreset: failed to read ERTC\n");
    reset_run_system();

    /* Give the picoArray some time to get started. Otherwise the operation
     * status register might not have been updated and we disable the MemIf
     * buffers while the design is still running. */
    udelay(10);

    /* wait for the picoArray to stop */
    do
    {
        data = config_read_word(CAEID_PROCIF, 0x401c);
    }
    while (data & 2);

    if (config_read_data(CAEID_PROCIF, 0x4008, ertc_end, 4) == -1)
        pr_err("softreset: failed to read ERTC\n");

    for (i = 0; i < 4; ++i)
        ertc_moved |= (ertc_start[i] != ertc_end[i]);
    if (!ertc_moved)
        pr_err("softreset: picoArray didn't run correctly\n");

    /* Return buffer config registers to their reset value*/
    for (buffer = 0; buffer < 6; ++buffer)
    {
        config_write_word(CAEID_MEMIF, 4 * buffer, 0x0000);
        config_write_word(CAEID_MEMIF, 1 + 4 * buffer, 0x0000);
        config_write_word(CAEID_MEMIF, 2 + 4 * buffer, 0x0000);
    }
    config_write_word(CAEID_MEMIF, 0x80, 0);

    /* Reset the HWPs */
    reset_ae_hwp(CAEID_MEMIF);

    return 0;
}

/*!
 * Reset the ProcIf. Drain GprSinks, drain DMAs
 */
static void reset_procif (void)
{
    int gpr, addr, dma, reads;

    /* Configuration bus stuff */
    config_write_word(CAEID_PROCIF, 0x5020, 0x0000); /*AutoBurstPacing*/

    /* SyncIf stuff */
    config_write_word(CAEID_PROCIF, 0x4010, 0x0004); /*SyncRetimeOffset*/
    config_write_word(CAEID_PROCIF, 0x4014, 0x0000); /*DebugMode*/
    config_write_word(CAEID_PROCIF, 0x4020, 0x0000); /*StartUpOffset*/

    /* Generic AE stuff */
    config_write_word(CAEID_PROCIF, 0xa062, 0x0000); /*ErrorRegistor*/
    config_write_word(CAEID_PROCIF, 0xa038, 0x0000); /*ErrorMaskLow*/
    config_write_word(CAEID_PROCIF, 0xa039, 0x0000); /*ErrorMaskMid*/
    config_write_word(CAEID_PROCIF, 0xa03A, 0x0000); /*ErrorMaskHigh*/
    config_write_word(CAEID_PROCIF, 0xa030, 0x0000); /*ErrorFlagsLow*/
    config_write_word(CAEID_PROCIF, 0xa031, 0x0000); /*ErrorFlagsMid*/
    config_write_word(CAEID_PROCIF, 0xa032, 0x0000); /*ErrorFlagsHigh*/

    /* IRQ stuff */
    config_write_word(CAEID_PROCIF, 0x0024, 0x00ff); /*IRQMode&Pulse*/

    /*  Turn each GPR into a blocking input port */
    for (addr = 0x9000; addr < 0x90d0; addr += 0x8)
    {
        if (addr == 0x9068 || 0x9070)
            continue; /* Skip DMA HWPs */
        config_write_word(CAEID_PROCIF, addr, 0x0003);
        config_write_word(CAEID_PROCIF, addr+1, 0x0011);
    }

    /* Run just the ProcIf */
    config_write_word(CAEID_PROCIF, 0xa061, 0x0002);

    /* Read out the contents of each GPR (four times) to empty it. Also empty the
     * ITS register */
    for (gpr = 0; gpr < 30; gpr++)
    {
        if (gpr == 20 || gpr == 22 || gpr == 24 || gpr == 26)
            continue; /* Skip the DMA registers in decode region 0 */
        if (gpr == 28)
            continue; /* Skip the ITM register in decode region 0 */
        for(reads = 0; reads < 4; reads++)
            read_procif_data(gpr);
    }

    /* Set the value of ITM to zero*/
    write_procif_data(28, 0x00000000);

    /* Stop the ProcIf */
    config_write_word(CAEID_PROCIF, 0xa061, 0x0000);

    /* DMA stuff */
    for (dma = 0; dma < 5; ++dma)
    {
        config_write_word(CAEID_PROCIF, DMA_REG_DREQ(dma), 0x0000);
        config_write_word(CAEID_PROCIF, DMA_REG_THRESHOLD(dma), 0x1000);
        config_write_word(CAEID_PROCIF, DMA_REG_POINTER(dma), 0x0000);
    }
    config_write_word(CAEID_PROCIF, 0x0309, 0x0004); /*GlobalDREQSettings*/
    config_write_word(CAEID_PROCIF, 0xa032, 0x0000); /*ErrorFlagsMid*/

    reset_ae_hwp(CAEID_PROCIF);

    /* External interface registers */
    config_write_word(CAEID_PROCIF, 0x0028, 0x0000); /*Acc2First*/
    config_write_word(CAEID_PROCIF, 0x002a, 0x0000); /*Acc2Next*/
    config_write_word(CAEID_PROCIF, 0x002b, 0x0000); /*ConfigPortMode*/

    /* Fuses */
    config_write_word(CAEID_PROCIF, 0x0034, 0x0000); /*RowFuse*/
    config_write_word(CAEID_PROCIF, 0x0035, 0x0000); /*Site3DisableFuse*/

    /* Sleep */
    config_write_word(CAEID_PROCIF, 0xa060, 0x0001);
}

/*!
 * Reset just the picoArray part of firecracker
 */
void picoArraySoftReset (void __iomem *procif,
                         void __iomem *ahb)
{
    procif_base = (void __iomem *)procif;
    ahb2pico_base = (void __iomem *)ahb;

    /* soft reset the AHB2Pico - don't print errors */
    reset_ahb2pico(0);

    /* soft reset the picoArray without the AHB2Pico - must be done after
     * AHB2Pico to prevent rogue data getting transfered */
    reset_programmable_aes();

    /* soft reset the MemIf and GPRs */
    reset_memif_gpr();

    /* soft reset the AHB2Pico - print errors */
    reset_ahb2pico(1);

    /* soft reset the PROCIF - do this last so the pA is not run, otherwise
     * rogue data will end up in the HWPs */
    reset_procif();

    /* soft reset the picoArray without the AHB2Pico - must be done after
     * AHB2Pico to prevent rogue data getting transfered */
    reset_programmable_aes();

    /* Wake the AHB2Pico block - must be done so that the GPIO driver
     * can keep running without losing access to this block */
   config_write_word(CAEID_AHB, 0xa060, 0x0000); 
}
