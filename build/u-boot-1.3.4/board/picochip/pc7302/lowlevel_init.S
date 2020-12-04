/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*!
* \file lowlevel_init.S
* \brief Various low level setup functions.
*
* Copyright (c) 2006-2009 picoChip Designs Ltd
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* All enquiries to support@picochip.com
*/

/* Includes ---------------------------------------------------------------- */
#include <config.h>
#include <asm/arch/pc302.h>
#include <asm/arch/mem_arm.h>
#include <asm/arch/mem_shd.h>
#include <asm/arch/gpio.h>
#include <asm/arch/pa.h>
#include <asm/arch/axi2cfg.h>

/* Macros ------------------------------------------------------------------ */
/* This gives us a peripheral bus base address of 0x80000000
   and a memory region size of 4Mbytes */
#define PERIPHERAL_BUS_SETUP        0x8000000D

#define ASSERT_BOOT_ERROR_PIN       0
#define NEGATE_BOOT_ERROR_PIN       1
#define BOOT_ERROR_PIN_IS_OUTPUT    1

/* Prototypes--------------------------------------------------------------- */

/* Macros ------------------------------------------------------------------ */

/*****************************************************************************
 *
 * configWriteLoadFile
 *
 *****************************************************************************/
.macro  configWriteLoadFile Reg0, Reg1, Reg2, Reg3, Reg4, lfbase, lfwords

        LDR     \Reg0,=\lfbase

        /* We are running in Flash at this point therefore we
           must 'noodle' with the linker created address
           for the 'lfbase' address */
        LDR     \Reg4,=TEXT_BASE
        EOR     \Reg0, \Reg0, \Reg4

        LDR     \Reg1,=PC302_AXI2CFG_BASE
        LDR     \Reg2,=\lfwords
0:
        LDR     \Reg3, [\Reg0],#0x4
        STR     \Reg3, [\Reg1,#AXI2CFG_CONFIG_WRITE_REG_OFFSET]
        SUBS    \Reg2, \Reg2, #1
        BNE     0b
.endm

/*****************************************************************************
 *
 * configRead
 *
 *****************************************************************************/
.macro  configRead Reg0, Reg1, Reg2, aeid, addr

        LDR     \Reg0,=PC302_AXI2CFG_BASE

        /* aeid */
        LDR     \Reg1,=\aeid
        ORR     \Reg1,\Reg1,#PA_CONFIG_AEID
        STR     \Reg1, [\Reg0,#AXI2CFG_CONFIG_WRITE_REG_OFFSET]

        /* addr */
        LDR     \Reg1,=\addr
        ORR     \Reg1,\Reg1,#PA_CONFIG_ADDR
        STR     \Reg1, [\Reg0,#AXI2CFG_CONFIG_WRITE_REG_OFFSET]

        /* data */
        MOV     \Reg1,#PA_CONFIG_READ
        STR     \Reg1, [\Reg0,#AXI2CFG_CONFIG_WRITE_REG_OFFSET]
        MOV     \Reg2,#0x20

        /* read */
0:
        LDR     \Reg1, [\Reg0,#AXI2CFG_CONFIG_READ_REG_OFFSET]
        TST     \Reg1, #(PA_CONFIG_VALID | PA_CONFIG_FAIL)
        BNE     1f
        SUBS    \Reg2,\Reg2,#1
        BNE     0b
        MOV     \Reg1, #PA_CONFIG_TIMEOUT
1:
        MOV     \Reg0, \Reg1

.endm

/* Functions --------------------------------------------------------------- */

/*****************************************************************************
 *
 * lowlevel_init
 *
 * Description: Perform some low level PC302 initialisation
 *
 *****************************************************************************/
 .global lowlevel_init

 lowlevel_init:

        /* Preserve link register across call */
        mov     r11,lr

        /* Setup the Periheral Port Memory Remap Register */
        LDR     r0, =PERIPHERAL_BUS_SETUP
        MCR     p15, 0, r0, c15, c2, 4

#ifdef CONFIG_PC302_SIMULATION
        bl      register_initialisation
#endif

        /* Initialise the boot error pin */
        bl      initialiseBootErrorPin

        /* Setup the ARM memory interface */
        bl      memif_arm_init

        /* Set up the Shared memory interface */
        bl      memif_shared_init

        /* Restore link register */
        mov     lr, r11

        /* Return to my caller */
        mov	pc, lr

/*****************************************************************************
 *
 * memif_arm_init
 *
 * Description: Initialise the ARM Memory Controller
 *
 *****************************************************************************/
memif_arm_init:

        LDR     r3, =PC302_MEMIF_BASE

        /* ;; =MEMIF_ARM_DRAM_INIT_PARAM_OFFSET */
        LDR     r4, [r3, #MEMIF_ARM_DRAM_INIT_PARAM_OFFSET]
        BIC     r4, r4, #(MEMIF_ARM_DRAM_INIT_PARAM_FWAIT_MSK << MEMIF_ARM_DRAM_INIT_PARAM_FWAIT_IDX)
        ORR     r4, r4, #(MEMIF_ARM_DRAM_INIT_PARAM_FWAIT_VAL << MEMIF_ARM_DRAM_INIT_PARAM_FWAIT_IDX)
        BIC     r4, r4, #(MEMIF_ARM_DRAM_INIT_PARAM_PRE_CKE_MSK << MEMIF_ARM_DRAM_INIT_PARAM_PRE_CKE_IDX)
        ORR     r4, r4, #(MEMIF_ARM_DRAM_INIT_PARAM_PRE_CKE_VAL << MEMIF_ARM_DRAM_INIT_PARAM_PRE_CKE_IDX)
        BIC     r4, r4, #(MEMIF_ARM_DRAM_INIT_PARAM_PST_CKE_MSK << MEMIF_ARM_DRAM_INIT_PARAM_PST_CKE_IDX)
        ORR     r4, r4, #(MEMIF_ARM_DRAM_INIT_PARAM_PST_CKE_VAL << MEMIF_ARM_DRAM_INIT_PARAM_PST_CKE_IDX)
        STR     r4, [r3, #MEMIF_ARM_DRAM_INIT_PARAM_OFFSET]

        /* ;; =MEMIF_ARM_ADDR_MAP_0_OFFSET */
        LDR     r4, =MEMIF_ARM_ADDR_MAP_0_VAL_1Gbx16
        STR     r4, [r3, #MEMIF_ARM_ADDR_MAP_0_OFFSET]

        /* ;; =MEMIF_ARM_ADDR_MAP_1_OFFSET */
        LDR     r4, =MEMIF_ARM_ADDR_MAP_1_VAL_1Gbx16
        STR     r4, [r3, #MEMIF_ARM_ADDR_MAP_1_OFFSET]

        /* ;; =MEMIF_ARM_ADDR_MAP_2_OFFSET */
        LDR     r4, =MEMIF_ARM_ADDR_MAP_2_VAL_1Gbx16
        STR     r4, [r3, #MEMIF_ARM_ADDR_MAP_2_OFFSET]

        /* ;; =MEMIF_ARM_DRAM_PARAM_2_OFFSET */
        LDR     r4, [r3, #MEMIF_ARM_DRAM_PARAM_2_OFFSET]
        AND     r4, r4, #0xFFFFFFF0
        ORR     r4, r4, #0x00000004 ; wl = cl - 2
        STR     r4, [r3, #MEMIF_ARM_DRAM_PARAM_2_OFFSET]

        /* ;; =MEMIF_ARM_DRAM_BL_OFFSET */
        LDR     r4, [r3, #MEMIF_ARM_DRAM_BL_OFFSET]
        BIC     r4, r4, #0x00000001 //; bl = 4
        STR     r4, [r3, #MEMIF_ARM_DRAM_BL_OFFSET]

        /* ;;  =MEMIF_ARM_DRAM_EMR_MR_OFFSET */
        LDR     r4, [r3, #MEMIF_ARM_DRAM_EMR_MR_OFFSET]
        AND     r4, r4, #0xFFFFFF88
        ORR     r4, r4, #0x00000002 //; bl = 4
        ORR     r4, r4, #0x00000060 //; cl = 6
        ORR     r4, r4, #0x04000000 //; dqs# = 0
        STR     r4, [r3, #MEMIF_ARM_DRAM_EMR_MR_OFFSET]

        /* ;;  =MEMIF_ARM_PHY_CMD_RDC_OFFSET */
        LDR     r4, [r3, #MEMIF_ARM_PHY_CMD_RDC_OFFSET]
        AND     r4, r4, #0xFFFFFF00
        ORR     r4, r4, #0x00000050 //;  phy wl
        ORR     r4, r4, #0x00000006 //;  phy rl
        AND     r4, r4, #0xFFFFFCFF
        ORR     r4, r4, #0x00000300 //;  rdc_we_to_re_delay = 3 clks
        AND     r4, r4, #0xFFFEFFFF
        ORR     r4, r4, #0x00010000 //;  use_fixed_re = 1
        STR     r4, [r3, #MEMIF_ARM_PHY_CMD_RDC_OFFSET]

        /* ;; =MEMIF_ARM_PHY_SLV_DLL_OFFSET */
        LDR     r4, [r3, #MEMIF_ARM_PHY_SLV_DLL_OFFSET]
        AND     r4, r4, #0x0
        ORR     r4, r4, #0x30000000 //;  dll rd_slave1 ratio
        ORR     r4, r4, #0x00300000 //;  dll rd_slave0 ratio
        ORR     r4, r4, #0x00000040 //;  dll wr_slave ratio
        STR     r4, [r3, #MEMIF_ARM_PHY_SLV_DLL_OFFSET]

        /* ;; =MEMIF_ARM_DRAM_ODT_OFFSET */
        LDR     r4, [r3, #MEMIF_ARM_DRAM_ODT_OFFSET]
        BIC     r4, r4, #0x00000001 //; disable dram odt during reads
        STR     r4, [r3,#MEMIF_ARM_DRAM_ODT_OFFSET]

        /* ;; =MEMIF_ARM_ODT_CTRL_OFFSET */
        LDR     r4, [r3, #MEMIF_ARM_ODT_CTRL_OFFSET]
        AND     r4, r4, #0xFFFFFF00
        AND     r4, r4, #0xFFFF00FF
        ORR     r4, r4, #0x0010 //; wr_odt_delay, cl-5
        ORR     r4, r4, #0x3000 //; wr_odt_hold, 4 cycles required
        STR     r4, [r3, #MEMIF_ARM_ODT_CTRL_OFFSET]

        /* ;; =MEMIF_ARM_PHY_LOCAL_ODT_OFFSET */
        LDR     r4, [r3, #MEMIF_ARM_PHY_LOCAL_ODT_OFFSET]
        AND     r4, r4, #0xFFFFFFC0
        ORR     r4, r4, #0x0001 //; rd_local_odt = 75 ohm
                                //; wr_local_odt = off
                                //; idle_local_odt = off
        STR     r4, [r3, #MEMIF_ARM_PHY_LOCAL_ODT_OFFSET]

        /* ;; =MEMIF_ARM_PHY_LOCAL_DRV_STRENGTH_OFFSET */
        LDR     r4, [r3, #MEMIF_ARM_PHY_LOCAL_DRV_STRENGTH_OFFSET]
        ORR     r4, r4, #0x0010 //; fifo_we_in odt = 75 ohm
        ORR     r4, r4, #0x000F //; All control signals half drive strength
        STR     r4, [r3, #MEMIF_ARM_GP2_OFFSET]

        /* ;; =MEMIF_ARM_INDIRECT_RW_CMD_OFFSET */
        LDR     r4, [r3, #MEMIF_ARM_INDIRECT_RW_CMD_OFFSET]
        ORR     r4, r4, #0x1  //; de-assert soft reset
        STR     r4, [r3, #MEMIF_ARM_INDIRECT_RW_CMD_OFFSET]

/* Check to see if the mem-if ARM is ready yet? */
memif_arm_init_poll:
        LDR     r4, [r3, #MEMIF_ARM_CTRL_MODE_OFFSET]
        AND     r4, r4, #0x3
        CMP     r4, #0x1
        BNE     memif_arm_init_poll

        mov	pc, lr		/* Return to my caller */

/*****************************************************************************
 *
 * memif_shared_init
 *
 * Description: Initialise the Shared Memory Controller
 *
 *****************************************************************************/

/* Data to set-up the memif-Shared Shared, this is effectively a load file */

memif_shared_init_data:
        .word   LF_PA_AEID_MEMIF, LF_CBFM_SLEEPREG_ADDR, PA_WRITE_ZERO_DATA
        .word   LF_PA_AEID_MEMIF, LF_ADDR_SDRAM_ARB_G0_S0_S1, ADDR_SDRAM_ARB_G0_S0_S1_DATA
        .word   LF_PA_AEID_MEMIF, LF_ADDR_SDRAM_ARB_G0_S2_S3, ADDR_SDRAM_ARB_G0_S2_S3_DATA
        .word   LF_PA_AEID_MEMIF, LF_ADDR_SDRAM_ARB_G1_S0_S1, ADDR_SDRAM_ARB_G1_S0_S1_DATA
        .word   LF_PA_AEID_MEMIF, LF_ADDR_SDRAM_ARB_G1_S2_S3, ADDR_SDRAM_ARB_G1_S2_S3_DATA
        .word   LF_PA_AEID_MEMIF, LF_ADDR_SDRAM_VALID_GROUPS, ADDR_SDRAM_VALID_GROUPS_DATA
        .word   LF_PA_AEID_MEMIF, LF_ADDR_SDRAM_SETUP, ADDR_SDRAM_SETUP_DATA
        .word   LF_PA_AEID_MEMIF, LF_ADDR_SDRAM_REFRESH, ADDR_SDRAM_REFRESH_DATA
        .word   LF_PA_AEID_MEMIF, LF_ADDR_SDRAM_ODT_SETUP, ADDR_SDRAM_ODT_DATA
        .word   LF_PA_AEID_MEMIF, LF_ADDR_SDRAM_AXI_CONFIG, ADDR_SDRAM_AXI_CONFIG_DATA
        .word   LF_PA_AEID_MEMIF, LF_ADDR_SDRAM_MRS, ADDR_SDRAM_MRS_DATA
        .word   LF_PA_AEID_MEMIF, LF_ADDR_SDRAM_EMRS, ADDR_SDRAM_EMRS_DATA
        .word   LF_PA_AEID_MEMIF, LF_ADDR_PHY_CONFIG, ADDR_PHY_CONFIG_DATA
        .word   LF_PA_AEID_MEMIF, LF_ADDR_PHY_LOCAL_ODT_CONFIG, ADDR_PHY_LOCAL_ODT_CONFIG_DATA
        .word   LF_PA_AEID_MEMIF, LF_ADDR_PHY_RD_SLAVE, ADDR_PHY_RD_SLAVE_DATA
        .word   LF_PA_AEID_MEMIF, LF_ADDR_PHY_WR_SLAVE, ADDR_PHY_WR_SLAVE_DATA
        .word   LF_PA_AEID_MEMIF, LF_ADDR_PHY_IO_CELL_CONFIG, ADDR_PHY_IO_CELL_CONFIG_DATA
        .word   LF_PA_AEID_MEMIF, LF_ADDR_SDRAM_CFG_DONE, ADDR_SDRAM_CFG_DONE_DATA

memif_shared_init:

        /* Write the intialisation data into the picoArray */
        configWriteLoadFile r0, r1, r2, r3, r4, memif_shared_init_data, 54

/* Check to see if the mem-if Shared is ready yet? */
memif_shared_init_poll:

        configRead r0, r1, r2, PA_AEID_MEMIF, ADDR_SDRAM_STATUS

        /* Has it timed out? */
        TST     r0, #PA_CONFIG_TIMEOUT
        BNE     memif_shared_init_poll

        /* Has it configured? */
        TST     r0, #0x01
        BEQ     memif_shared_init_poll

        /* Return to my caller */
        mov	pc, lr

/*****************************************************************************
 *
 * register_initialisation
 *
 * Description: Initialise some CPU registers
 *
 * Note:
 * Used to prevent X propagation if running this code in a simulation
 *
 *****************************************************************************/
register_initialisation:

        mov     r0, #0x00000000
        mov     r1, #0x00000000
        mov     r2, #0x00000000
        mov     r3, #0x00000000
        mov     r4, #0x00000000
        mov     r5, #0x00000000
        mov     r6, #0x00000000
        mov     r7, #0x00000000
        mov     r8, #0x00000000
        mov     r9, #0x00000000
        mov     r10,#0x00000000

        /* Don't noodle with r11 (fp)
                             r12 (ip)
                             r13 (sp)
                             r14 (lr)
                             r15 (pc)
           as this would cause big probs ! */

        /* Return to my caller */
        mov	pc, lr

/*****************************************************************************
 *
 * initialiseBootErrorPin
 *
 * Initialise the GPIO that is used as the Boot Error pin
 *
 *****************************************************************************/
 initialiseBootErrorPin:

        /* Get base address of the arm gpio block */
        ldr     r0,=PC302_GPIO_BASE

        /* Negate the Boot error pin (in the data register) */
        ldr     r1,[r0, #GPIO_SW_PORT_C_DR_REG_OFFSET]
        orr     r1,r1,#NEGATE_BOOT_ERROR_PIN
        str     r1,[r0, #GPIO_SW_PORT_C_DR_REG_OFFSET]

        /* Set up Boot Error pin to be an output */
        ldr     r1,[r0, #GPIO_SW_PORT_C_DDR_REG_OFFSET]
        orr     r1,r1,#BOOT_ERROR_PIN_IS_OUTPUT
        str     r1,[r0, #GPIO_SW_PORT_C_DDR_REG_OFFSET]

        /* Return to my caller */
        mov	pc, lr

/*****************************************************************************
 *
 * assertBootError
 *
 * Assert the Boot Error pin. On the SVB302 this is used to illuminate an LED
 *
 *****************************************************************************/
 .global assertBootError

 assertBootError:

        /* Get base address of the arm gpio block */
        ldr     r0,=PC302_GPIO_BASE

        /* Assert the Boot Error pin */
        mov     r1, #ASSERT_BOOT_ERROR_PIN
        str     r1,[r0, #GPIO_SW_PORT_C_DR_REG_OFFSET]

        /* Return to my caller */
        mov	pc, lr

       .end
