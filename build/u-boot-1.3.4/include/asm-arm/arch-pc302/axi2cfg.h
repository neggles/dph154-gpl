/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*!
* \file axi2cfg.h
* \brief Definitions for the PC302 AXI2CFG Block.
*
* Copyright (c) 2006-2009 picoChip Designs Ltd
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* All enquiries to support@picochip.com
*/

#ifndef PC302_AXI2CFG_H
#define PC302_AXI2CFG_H

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* Constants --------------------------------------------------------------- */

/*****************************************************************************/
/* Register Offset Addresses                                                 */
/*****************************************************************************/

/* Functional Registers */

#define AXI2CFG_SYS_CONFIG_REG_OFFSET               (0x0000)
#define AXI2CFG_JTAG_ISC_REG_OFFSET                 (0x0004)
#define AXI2CFG_IRQ_REG_OFFSET                      (0x0008)
#define AXI2CFG_PURGE_CFG_PORT_REG_OFFSET           (0x000C)
#define AXI2CFG_DMA_CFG_REG_OFFSET                  (0x0010)
#define AXI2CFG_DEVICE_ID_REG_OFFSET                (0x0014)
#define AXI2CFG_REVISION_ID_REG_OFFSET              (0x0018)
#define AXI2CFG_CONFIG_WRITE_REG_OFFSET             (0x0100)
#define AXI2CFG_CONFIG_READ_REG_OFFSET              (0x0200)
#define AXI2CFG_DMAC1_CONFIG_REG_OFFSET             (0x0300)


/*****************************************************************************/
/* Register Bit Fields		                                             */
/*****************************************************************************/

#define AXI2CFG_SYS_CONFIG_PA_RST_IDX               (30)
#define AXI2CFG_SYS_CONFIG_SD_ARM_GPIO_SEL_SZ       (8)
#define AXI2CFG_SYS_CONFIG_SD_ARM_GPIO_SEL_HI       (23)
#define AXI2CFG_SYS_CONFIG_SD_ARM_GPIO_SEL_LO       (16)
#define AXI2CFG_SYS_CONFIG_RW_EBI_CLK_DISABLE_IDX   (15)
#define AXI2CFG_SYS_CONFIG_RW_EXCVEC_EN_IDX         (14)
#define AXI2CFG_SYS_CONFIG_RW_RMII_EN_IDX           (13)
#define AXI2CFG_SYS_CONFIG_RW_REVMII_EN_IDX         (12)
#define AXI2CFG_SYS_CONFIG_SSI_EBI_SEL_SZ           (4)
#define AXI2CFG_SYS_CONFIG_SSI_EBI_SEL_HI           (11)
#define AXI2CFG_SYS_CONFIG_SSI_EBI_SEL_LO           (8)
#define AXI2CFG_SYS_CONFIG_FREQ_SYNTH_MUX_IDX       (7)
#define AXI2CFG_SYS_CONFIG_MASK_AXI_ERR_IDX         (6)
#define AXI2CFG_SYS_CONFIG_RW_REMAP_IDX             (5)
#define AXI2CFG_SYS_CONFIG_WDG_PAUSE_IDX            (4)
#define AXI2CFG_SYS_CONFIG_DMAC1_CH6_IDX            (3)
#define AXI2CFG_SYS_CONFIG_DMAC1_CH7_IDX            (2)
#define AXI2CFG_SYS_CONFIG_BOOT_MODE_SZ             (2)
#define AXI2CFG_SYS_CONFIG_BOOT_MODE_HI             (1)
#define AXI2CFG_SYS_CONFIG_BOOT_MODE_LO             (0)

#define AXI2CFG_SYS_CONFIG_PA_RST_MASK              (1 << AXI2CFG_SYS_CONFIG_PA_RST_IDX)
#define AXI2CFG_SYS_CONFIG_SD_ARM_GPIO_MASK         (((1 << AXI2CFG_SYS_CONFIG_SD_ARM_GPIO_SEL_SZ)-1) << AXI2CFG_SYS_CONFIG_SD_ARM_GPIO_SEL_LO)
#define AXI2CFG_SYS_CONFIG_RW_EXCVEC_EN_MASK        (1 << AXI2CFG_SYS_CONFIG_RW_EXCVEC_EN_IDX)
#define AXI2CFG_SYS_CONFIG_RW_RMII_EN_MASK          (1 << AXI2CFG_SYS_CONFIG_RW_RMII_EN_IDX)
#define AXI2CFG_SYS_CONFIG_RW_REVMII_EN_MASK        (1 << AXI2CFG_SYS_CONFIG_RW_REVMII_EN_IDX)
#define AXI2CFG_SYS_CONFIG_SSI_EBI_SEL_MASK         (((1 << AXI2CFG_SYS_CONFIG_SSI_EBI_SEL_SZ)-1) << AXI2CFG_SYS_CONFIG_SSI_EBI_SEL_LO)
#define AXI2CFG_SYS_CONFIG_FREQ_SYNTH_MUX_MASK      (1<<AXI2CFG_SYS_CONFIG_FREQ_SYNTH_MUX_IDX)
#define AXI2CFG_SYS_CONFIG_MASK_AXI_ERR_MASK        (1 << AXI2CFG_SYS_CONFIG_MASK_AXI_ERR_IDX)
#define AXI2CFG_SYS_CONFIG_RW_REMAP_MASK            (1 << AXI2CFG_SYS_CONFIG_RW_REMAP_IDX)
#define AXI2CFG_SYS_CONFIG_WDG_PAUSE_MASK           (1 << AXI2CFG_SYS_CONFIG_WDG_PAUSE_IDX)
#define AXI2CFG_SYS_CONFIG_DMAC1_CH6_MASK           (1 << AXI2CFG_SYS_CONFIG_DMAC1_CH6_IDX)
#define AXI2CFG_SYS_CONFIG_DMAC1_CH7_MASK           (1 << AXI2CFG_SYS_CONFIG_DMAC1_CH7_IDX)
#define AXI2CFG_SYS_CONFIG_BOOT_MODE_MASK           (((1 << AXI2CFG_SYS_CONFIG_BOOT_MODE_SZ)-1) << AXI2CFG_SYS_CONFIG_BOOT_MODE_LO)

#define AXI2CFG_DMAC1_CONFIG_WR_SZ                  (7)
#define AXI2CFG_DMAC1_CONFIG_WR_HI                  (16)
#define AXI2CFG_DMAC1_CONFIG_WR_LO                  (10)
#define AXI2CFG_DMAC1_CONFIG_WATERMARK_SZ           (7)
#define AXI2CFG_DMAC1_CONFIG_WATERMARK_HI           (9)
#define AXI2CFG_DMAC1_CONFIG_WATERMARK_LO           (3)
#define AXI2CFG_DMAC1_CONFIG_SNGL_IDX               (2)
#define AXI2CFG_DMAC1_CONFIG_STATE_IDX              (1)
#define AXI2CFG_DMAC1_CONFIG_ENABLE_IDX             (0)

#define AXI2CFG_JTAG_ISC_REGISTER_IDX               (0)
#define AXI2CFG_JTAG_ISC_IN_CTRL_IDX                (1)
#define AXI2CFG_JTAG_ISC_DISABLED_IDX               (2)
/* [31:2] - Reserved */

#define AXI2CFG_PURGE_CFG_RD_PORT_IDX               (0)
#define AXI2CFG_PURGE_CFG_WR_PORT_IDX               (1)
#define AXI2CFG_PURGE_CFG_WR_PRGSS_PORT_IDX         (2)
/* [31:3]   Reserved */

#define AXI2CFG_DEVICE_ID_NML_302_REG_VALUE         (0x03020004)

/*****************************************************************************/
/* Register Bit Field Manipulation                                           */
/*****************************************************************************/

#define AXI2CFG_PA_SOFT_RESET_IDX                   (30)
#define AXI2CFG_SHD_GPIO_7_IDX                      (23)
#define AXI2CFG_SHD_GPIO_6_IDX                      (22)
#define AXI2CFG_SHD_GPIO_5_IDX                      (21)
#define AXI2CFG_SHD_GPIO_4_IDX                      (20)
#define AXI2CFG_SHD_GPIO_3_IDX                      (19)
#define AXI2CFG_SHD_GPIO_2_IDX                      (18)
#define AXI2CFG_SHD_GPIO_1_IDX                      (17)
#define AXI2CFG_SHD_GPIO_0_IDX                      (16)
#define AXI2CFG_EBI_CLK_DISABLE_IDX                 (15)
#define AXI2CFG_EXCEPTION_VECT_EN_IDX               (14)
#define AXI2CFG_RMII_EN_IDX                         (13)
#define AXI2CFG_REV_MII_EN_IDX                      (12)
#define AXI2CFG_DECODE_MUX_3_IDX                    (11)
#define AXI2CFG_DECODE_MUX_2_IDX                    (10)
#define AXI2CFG_DECODE_MUX_1_IDX                    (9)
#define AXI2CFG_DECODE_MUX_0_IDX                    (8)
#define AXI2CFG_MASK_AXI_ERROR_IDX                  (6)
#define AXI2CFG_REMAP_IDX                           (5)
#define AXI2CFG_WDG_PAUSE_IDX                       (4)
#define AXI2CFG_DMA_CHAN_6_MUX_IDX                  (3)
#define AXI2CFG_DMA_CHAN_7_MUX_IDX                  (2)
#define AXI2CFG_BOOT_MODE_IDX                       (0)

#define AXI2CFG_PA_SOFT_RESET               (1 << AXI2CFG_PA_SOFT_RESET_IDX)
#define AXI2CFG_SHD_GPIO_7                  (1 << AXI2CFG_SHD_GPIO_7_IDX)
#define AXI2CFG_SHD_GPIO_6                  (1 << AXI2CFG_SHD_GPIO_6_IDX)
#define AXI2CFG_SHD_GPIO_5                  (1 << AXI2CFG_SHD_GPIO_5_IDX)
#define AXI2CFG_SHD_GPIO_4                  (1 << AXI2CFG_SHD_GPIO_4_IDX)
#define AXI2CFG_SHD_GPIO_3                  (1 << AXI2CFG_SHD_GPIO_3_IDX)
#define AXI2CFG_SHD_GPIO_2                  (1 << AXI2CFG_SHD_GPIO_2_IDX)
#define AXI2CFG_SHD_GPIO_1                  (1 << AXI2CFG_SHD_GPIO_1_IDX)
#define AXI2CFG_SHD_GPIO_0                  (1 << AXI2CFG_SHD_GPIO_0_IDX)
#define AXI2CFG_EBI_CLK_DISABLE             (1 << AXI2CFG_EBI_CLK_DISABLE_IDX)
#define AXI2CFG_EXCEPTION_VECT_EN           (1 << AXI2CFG_EXCEPTION_VECT_EN_IDX)
#define AXI2CFG_RMII_EN                     (1 << AXI2CFG_RMII_EN_IDX)
#define AXI2CFG_REV_MII_EN                  (1 << AXI2CFG_REV_MII_EN_IDX)
#define AXI2CFG_DECODE_MUX_3                (1 << AXI2CFG_DECODE_MUX_3_IDX)
#define AXI2CFG_DECODE_MUX_2                (1 << AXI2CFG_DECODE_MUX_2_IDX)
#define AXI2CFG_DECODE_MUX_1                (1 << AXI2CFG_DECODE_MUX_1_IDX)
#define AXI2CFG_DECODE_MUX_0                (1 << AXI2CFG_DECODE_MUX_0_IDX)
#define AXI2CFG_MASK_AXI_ERROR              (1 << AXI2CFG_MASK_AXI_ERROR_IDX)
#define AXI2CFG_REMAP                       (1 << AXI2CFG_REMAP_IDX)
#define AXI2CFG_WDG_PAUSE                   (1 << AXI2CFG_WDG_PAUSE_IDX)
#define AXI2CFG_DMA_CHAN_6_MUX              (1 << AXI2CFG_DMA_CHAN_6_MUX_IDX)
#define AXI2CFG_DMA_CHAN_7_MUX              (1 << AXI2CFG_DMA_CHAN_7_MUX_IDX)
#define AXI2CFG_BOOT_MODE                   (1 << AXI2CFG_BOOT_MODE_IDX)

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif /* PC302_AXI2CFG_H */
