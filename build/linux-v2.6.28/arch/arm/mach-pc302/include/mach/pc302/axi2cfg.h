/* Copyright (c) 2008 picoChip Designs Ltd.
 *
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef PC302_AXI2CFG_H
#define PC302_AXI2CFG_H

/* Constants -------------------------------------------------------------- */

/*****************************************************************************/
/* Register Offset Addresses                                                 */
/*****************************************************************************/

/* Functional Registers */

#define AXI2CFG_SYS_CONFIG_REG_OFFSET       0x0000
#define AXI2CFG_JTAG_ISC_REG_OFFSET         0x0004
#define AXI2CFG_IRQ_REG_OFFSET              0x0008
#define AXI2CFG_PURGE_CFG_PORT_REG_OFFSET   0x000C
#define AXI2CFG_DMA_CFG_REG_OFFSET          0x0010
#define AXI2CFG_DEVICE_ID_REG_OFFSET        0x0014
#define AXI2CFG_REVISION_ID_REG_OFFSET      0x0018
#define AXI2CFG_AXI_ERR_ENABLE_REG_OFFSET   0x001C
#define AXI2CFG_AXI_ERR_CLEAR_REG_OFFSET    0x0020
#define AXI2CFG_AXI_ERR_MASK_REG_OFFSET     0x0024
#define AXI2CFG_AXI_ERR_TEST_REG_OFFSET     0x0028
#define AXI2CFG_AXI_ERR_RAW_REG_OFFSET      0x002C
#define AXI2CFG_AXI_ERR_STATE_REG_OFFSET    0x0030
#define AXI2CFG_CONFIG_WRITE_REG_OFFSET     0x0100
#define AXI2CFG_CONFIG_READ_REG_OFFSET      0x0200
#define AXI2CFG_DMAC1_CONFIG_REG_OFFSET     0x0300


/*****************************************************************************/
/* Register Bit Fields		                                             */
/*****************************************************************************/

#define AXI2CFG_SYS_CONFIG_PA_RST_IDX               30
#define AXI2CFG_SYS_CONFIG_SD_ARM_GPIO_SEL_SZ       8
#define AXI2CFG_SYS_CONFIG_SD_ARM_GPIO_SEL_HI       23
#define AXI2CFG_SYS_CONFIG_SD_ARM_GPIO_SEL_LO       16
#define AXI2CFG_SYS_CONFIG_RW_EBI_CLK_DISABLE_IDX   15
#define AXI2CFG_SYS_CONFIG_RW_EXCVEC_EN_IDX         14
#define AXI2CFG_SYS_CONFIG_RW_RMII_EN_IDX           13
#define AXI2CFG_SYS_CONFIG_RW_REVMII_EN_IDX         12
#define AXI2CFG_SYS_CONFIG_SSI_EBI_SEL_SZ           4
#define AXI2CFG_SYS_CONFIG_SSI_EBI_SEL_HI           11
#define AXI2CFG_SYS_CONFIG_SSI_EBI_SEL_LO           8
#define AXI2CFG_SYS_CONFIG_FREQ_SYNTH_MUX_IDX       7
#define AXI2CFG_SYS_CONFIG_MASK_AXI_ERR_IDX         6
#define AXI2CFG_SYS_CONFIG_RW_REMAP_IDX             5
#define AXI2CFG_SYS_CONFIG_WDG_PAUSE_IDX            4
#define AXI2CFG_SYS_CONFIG_CP15DISABLE_IDX          3
#define AXI2CFG_SYS_CONFIG_DMAC1_CH7_IDX            2
#define AXI2CFG_SYS_CONFIG_BOOT_MODE_SZ             2
#define AXI2CFG_SYS_CONFIG_BOOT_MODE_HI             1
#define AXI2CFG_SYS_CONFIG_BOOT_MODE_LO             0

#define AXI2CFG_SYS_CONFIG_PA_RST_MASK              1<<AXI2CFG_SYS_CONFIG_PA_RST_IDX
#define AXI2CFG_SYS_CONFIG_SD_ARM_GPIO_MASK         ((1<<AXI2CFG_SYS_CONFIG_SD_ARM_GPIO_SEL_SZ)-1)<<AXI2CFG_SYS_CONFIG_SD_ARM_GPIO_SEL_LO
#define AXI2CFG_SYS_CONFIG_RW_EXCVEC_EN_MASK        1<<AXI2CFG_SYS_CONFIG_RW_EXCVEC_EN_IDX
#define AXI2CFG_SYS_CONFIG_RW_RMII_EN_MASK          1<<AXI2CFG_SYS_CONFIG_RW_RMII_EN_IDX
#define AXI2CFG_SYS_CONFIG_RW_REVMII_EN_MASK        1<<AXI2CFG_SYS_CONFIG_RW_REVMII_EN_IDX
#define AXI2CFG_SYS_CONFIG_SSI_EBI_SEL_MASK         ((1<<AXI2CFG_SYS_CONFIG_SSI_EBI_SEL_SZ)-1)<<AXI2CFG_SYS_CONFIG_SSI_EBI_SEL_LO
#define AXI2CFG_SYS_CONFIG_FREQ_SYNTH_MUX_MASK      1<<AXI2CFG_SYS_CONFIG_FREQ_SYNTH_MUX_IDX
#define AXI2CFG_SYS_CONFIG_MASK_AXI_ERR_MASK        1<<AXI2CFG_SYS_CONFIG_MASK_AXI_ERR_IDX
#define AXI2CFG_SYS_CONFIG_RW_REMAP_MASK            1<<AXI2CFG_SYS_CONFIG_RW_REMAP_IDX
#define AXI2CFG_SYS_CONFIG_WDG_PAUSE_MASK           1<<AXI2CFG_SYS_CONFIG_WDG_PAUSE_IDX
#define AXI2CFG_SYS_CONFIG_CP15DISABLE_MASK         1<<AXI2CFG_SYS_CONFIG_CP15DISABLE_IDX
#define AXI2CFG_SYS_CONFIG_DMAC1_CH7_MASK           1<<AXI2CFG_SYS_CONFIG_DMAC1_CH7_IDX
#define AXI2CFG_SYS_CONFIG_BOOT_MODE_MASK           ((1<<AXI2CFG_SYS_CONFIG_BOOT_MODE_SZ)-1)<<AXI2CFG_SYS_CONFIG_BOOT_MODE_LO

#define AXI2CFG_DMAC1_CONFIG_DMA_WRITE_IDX          17
#define AXI2CFG_DMAC1_CONFIG_FILL_LEVEL_SZ          7
#define AXI2CFG_DMAC1_CONFIG_FILL_LEVEL_HI          16
#define AXI2CFG_DMAC1_CONFIG_FILL_LEVEL_LO          10
#define AXI2CFG_DMAC1_CONFIG_WATERMARK_SZ           7
#define AXI2CFG_DMAC1_CONFIG_WATERMARK_HI           9
#define AXI2CFG_DMAC1_CONFIG_WATERMARK_LO           3
#define AXI2CFG_DMAC1_CONFIG_SNGL_IDX               2
#define AXI2CFG_DMAC1_CONFIG_STATE_IDX              1
#define AXI2CFG_DMAC1_CONFIG_ENABLE_IDX             0

#define AXI2CFG_DMAC1_CONFIG_DMA_WRITE_MASK         1<<AXI2CFG_DMAC1_CONFIG_DMA_WRITE_IDX)
#define AXI2CFG_DMAC1_CONFIG_FILL_LEVEL_MASK        ((1<<AXI2CFG_DMAC1_CONFIG_FILL_LEVEL_SZ)-1)<<AXI2CFG_DMAC1_CONFIG_FILL_LEVEL_LO
#define AXI2CFG_DMAC1_CONFIG_WATERMARK_MASK         ((1<<AXI2CFG_DMAC1_CONFIG_WATERMARK_SZ)-1)<<AXI2CFG_DMAC1_CONFIG_WATERMARK_LO
#define AXI2CFG_DMAC1_CONFIG_SNGL_MASK              1<<AXI2CFG_DMAC1_CONFIG_SNGL_IDX
#define AXI2CFG_DMAC1_CONFIG_STATE_MASK             1<<AXI2CFG_DMAC1_CONFIG_STATE_IDX
#define AXI2CFG_DMAC1_CONFIG_ENABLE_MASK            1<<AXI2CFG_DMAC1_CONFIG_ENABLE_IDX

#define AXI2CFG_JTAG_ISC_REGISTER_IDX               0
#define AXI2CFG_JTAG_ISC_IN_CTRL_IDX                1
#define AXI2CFG_JTAG_ISC_DISABLED_IDX               2
/* [31:2] - Reserved */

#define AXI2CFG_PURGE_CFG_RD_PORT_IDX               0
#define AXI2CFG_PURGE_CFG_WR_PORT_IDX               1
#define AXI2CFG_PURGE_CFG_WR_PRGSS_PORT_IDX         2
/* [31:3]   Reserved */

#define AXI2CFG_DEVICE_ID_FAT_302_REG_VALUE         0x03020003
#define AXI2CFG_DEVICE_ID_NML_302_REG_VALUE         0x03020004

/*****************************************************************************/
/* AXI2CFG IRQ Register Bit Allocation                                       */
/*****************************************************************************/
#define AXI2CFG_CFG_IRQ_EN_IDX                      0
#define AXI2CFG_CFG_IRQ_CLEAR_IDX                   1
#define AXI2CFG_CFG_IRQ_MASK_IDX                    2
#define AXI2CFG_CFG_IRQ_TEST_IDX                    3
#define AXI2CFG_CFG_IRQ_PRE_IDX                     4
#define AXI2CFG_CFG_IRQ_POST_IDX                    5
#define AXI2CFG_PICO_IRQ_EN_IDX                     8
#define AXI2CFG_PICO_IRQ_CLEAR_IDX                  9
#define AXI2CFG_PICO_IRQ_MASK_IDX                   10
#define AXI2CFG_PICO_IRQ_TEST_IDX                   11
#define AXI2CFG_PICO_IRQ_PRE_IDX                    12
#define AXI2CFG_PICO_IRQ_POST_IDX                   13

#define AXI2CFG_CFG_IRQ_EN_MASK                     1 << AXI2CFG_CFG_IRQ_EN_IDX
#define AXI2CFG_CFG_IRQ_CLEAR_MASK                  1 << AXI2CFG_CFG_IRQ_CLEAR_IDX
#define AXI2CFG_CFG_IRQ_MASK_MASK                   1 << AXI2CFG_CFG_IRQ_MASK_IDX
#define AXI2CFG_CFG_IRQ_TEST_MASK                   1 << AXI2CFG_CFG_IRQ_TEST_IDX
#define AXI2CFG_CFG_IRQ_PRE_MASK                    1 << AXI2CFG_CFG_IRQ_PRE_IDX
#define AXI2CFG_CFG_IRQ_POST_MASK                   1 << AXI2CFG_CFG_IRQ_POST_IDX
#define AXI2CFG_PICO_IRQ_EN_MASK                    1 << AXI2CFG_PICO_IRQ_EN_IDX
#define AXI2CFG_PICO_IRQ_CLEAR_MASK                 1 << AXI2CFG_PICO_IRQ_CLEAR_IDX
#define AXI2CFG_PICO_IRQ_MASK_MASK                  1 << AXI2CFG_PICO_IRQ_MASK_IDX
#define AXI2CFG_PICO_IRQ_TEST_MASK                  1 << AXI2CFG_PICO_IRQ_TEST_IDX
#define AXI2CFG_PICO_IRQ_PRE_MASK                   1 << AXI2CFG_PICO_IRQ_PRE_IDX
#define AXI2CFG_PICO_IRQ_POST_MASK                  1 << AXI2CFG_PICO_IRQ_POST_IDX

/*****************************************************************************/
/* AXI Error Register Bit Allocation                                         */
/*****************************************************************************/
#define AXI2CFG_AXI_RD_ERR_DMA1_MST1_IDX           0
#define AXI2CFG_AXI_RD_ERR_DMA1_MST2_IDX           1
#define AXI2CFG_AXI_RD_ERR_DMA1_MST3_IDX           2
#define AXI2CFG_AXI_RD_ERR_DMA1_MST4_IDX           3
#define AXI2CFG_AXI_RD_ERR_DMA2_MST1_IDX           4
#define AXI2CFG_AXI_RD_ERR_DMA2_MST2_IDX           5
#define AXI2CFG_AXI_RD_ERR_DMA2_MST3_IDX           6
#define AXI2CFG_AXI_RD_ERR_DMA2_MST4_IDX           7
#define AXI2CFG_AXI_RD_ERR_EMAC_IDX                8
#define AXI2CFG_AXI_RD_ERR_SPACC3_IDX              9
#define AXI2CFG_AXI_RD_ERR_SPACC2_IDX              10
#define AXI2CFG_AXI_RD_ERR_SPACC1_IDX              11
#define AXI2CFG_AXI_WR_ERR_DMA1_MST1_IDX           12
#define AXI2CFG_AXI_WR_ERR_DMA1_MST2_IDX           13
#define AXI2CFG_AXI_WR_ERR_DMA1_MST3_IDX           14
#define AXI2CFG_AXI_WR_ERR_DMA1_MST4_IDX           15
#define AXI2CFG_AXI_WR_ERR_DMA2_MST1_IDX           16
#define AXI2CFG_AXI_WR_ERR_DMA2_MST2_IDX           17
#define AXI2CFG_AXI_WR_ERR_DMA2_MST3_IDX           18
#define AXI2CFG_AXI_WR_ERR_DMA2_MST4_IDX           19
#define AXI2CFG_AXI_WR_ERR_EMAC_IDX                20
#define AXI2CFG_AXI_WR_ERR_SPACC3_IDX              21
#define AXI2CFG_AXI_WR_ERR_SPACC2_IDX              22
#define AXI2CFG_AXI_WR_ERR_SPACC1_IDX              23

#define AXI2CFG_AXI_RD_ERR_DMA1_MST1_MASK          1<<AXI2CFG_AXI_RD_ERR_DMA1_MST1_IDX
#define AXI2CFG_AXI_RD_ERR_DMA1_MST2_MASK          1<<AXI2CFG_AXI_RD_ERR_DMA1_MST2_IDX
#define AXI2CFG_AXI_RD_ERR_DMA1_MST3_MASK          1<<AXI2CFG_AXI_RD_ERR_DMA1_MST3_IDX
#define AXI2CFG_AXI_RD_ERR_DMA1_MST4_MASK          1<<AXI2CFG_AXI_RD_ERR_DMA1_MST4_IDX
#define AXI2CFG_AXI_RD_ERR_DMA2_MST1_MASK          1<<AXI2CFG_AXI_RD_ERR_DMA2_MST1_IDX
#define AXI2CFG_AXI_RD_ERR_DMA2_MST2_MASK          1<<AXI2CFG_AXI_RD_ERR_DMA2_MST2_IDX
#define AXI2CFG_AXI_RD_ERR_DMA2_MST3_MASK          1<<AXI2CFG_AXI_RD_ERR_DMA2_MST3_IDX
#define AXI2CFG_AXI_RD_ERR_DMA2_MST4_MASK          1<<AXI2CFG_AXI_RD_ERR_DMA2_MST4_IDX
#define AXI2CFG_AXI_RD_ERR_EMAC_MASK               1<<AXI2CFG_AXI_RD_ERR_EMAC_IDX
#define AXI2CFG_AXI_RD_ERR_SPACC3_MASK             1<<AXI2CFG_AXI_RD_ERR_SPACC3_IDX
#define AXI2CFG_AXI_RD_ERR_SPACC2_MASK             1<<AXI2CFG_AXI_RD_ERR_SPACC2_IDX
#define AXI2CFG_AXI_RD_ERR_SPACC1_MASK             1<<AXI2CFG_AXI_RD_ERR_SPACC1_IDX
#define AXI2CFG_AXI_WR_ERR_DMA1_MST1_MASK          1<<AXI2CFG_AXI_WR_ERR_DMA1_MST1_IDX
#define AXI2CFG_AXI_WR_ERR_DMA1_MST2_MASK          1<<AXI2CFG_AXI_WR_ERR_DMA1_MST2_IDX
#define AXI2CFG_AXI_WR_ERR_DMA1_MST3_MASK          1<<AXI2CFG_AXI_WR_ERR_DMA1_MST3_IDX
#define AXI2CFG_AXI_WR_ERR_DMA1_MST4_MASK          1<<AXI2CFG_AXI_WR_ERR_DMA1_MST4_IDX
#define AXI2CFG_AXI_WR_ERR_DMA2_MST1_MASK          1<<AXI2CFG_AXI_WR_ERR_DMA2_MST1_IDX
#define AXI2CFG_AXI_WR_ERR_DMA2_MST2_MASK          1<<AXI2CFG_AXI_WR_ERR_DMA2_MST2_IDX
#define AXI2CFG_AXI_WR_ERR_DMA2_MST3_MASK          1<<AXI2CFG_AXI_WR_ERR_DMA2_MST3_IDX
#define AXI2CFG_AXI_WR_ERR_DMA2_MST4_MASK          1<<AXI2CFG_AXI_WR_ERR_DMA2_MST4_IDX
#define AXI2CFG_AXI_WR_ERR_EMAC_MASK               1<<AXI2CFG_AXI_WR_ERR_EMAC_IDX
#define AXI2CFG_AXI_WR_ERR_SPACC3_MASK             1<<AXI2CFG_AXI_WR_ERR_SPACC3_IDX
#define AXI2CFG_AXI_WR_ERR_SPACC2_MASK             1<<AXI2CFG_AXI_WR_ERR_SPACC2_IDX
#define AXI2CFG_AXI_WR_ERR_SPACC1_MASK             1<<AXI2CFG_AXI_WR_ERR_SPACC1_IDX

#define AXI2CFG_AXI_RD_ERR_MASK                    (0x00000FFF)
#define AXI2CFG_AXI_WR_ERR_MASK                    (0x00FFF000)
#define AXI2CFG_AXI_ERR_MASK_NONE                  (0)
#define AXI2CFG_AXI_ERR_ENABLE_ALL                 (0x00FFFFFF)

/*****************************************************************************/
/* Register Bit Field Manipulation                                           */
/*****************************************************************************/

#define AXI2CFG_PA_SOFT_RESET_IDX          30
#define AXI2CFG_SHD_GPIO_7_IDX             23
#define AXI2CFG_SHD_GPIO_6_IDX             22
#define AXI2CFG_SHD_GPIO_5_IDX             21
#define AXI2CFG_SHD_GPIO_4_IDX             20
#define AXI2CFG_SHD_GPIO_3_IDX             19
#define AXI2CFG_SHD_GPIO_2_IDX             18
#define AXI2CFG_SHD_GPIO_1_IDX             17
#define AXI2CFG_SHD_GPIO_0_IDX             16
#define AXI2CFG_EBI_CLK_DISABLE_IDX        15
#define AXI2CFG_EXCEPTION_VECT_EN_IDX      14
#define AXI2CFG_RMII_EN_IDX                13
#define AXI2CFG_REV_MII_EN_IDX             12
#define AXI2CFG_DECODE_MUX_3_IDX           11
#define AXI2CFG_DECODE_MUX_2_IDX           10
#define AXI2CFG_DECODE_MUX_1_IDX            9
#define AXI2CFG_DECODE_MUX_0_IDX            8
#define AXI2CFG_FREQ_SYNTH_MUX_IDX          7
#define AXI2CFG_MASK_AXI_ERROR_IDX          6
#define AXI2CFG_REMAP_IDX                   5
#define AXI2CFG_WDG_PAUSE_IDX               4
#define AXI2CFG_CP15DISABLE_IDX             3
#define AXI2CFG_DMA_CHAN_7_MUX_IDX          2
#define AXI2CFG_BOOT_MODE_IDX               0

#define AXI2CFG_PA_SOFT_RESET               1 << AXI2CFG_PA_SOFT_RESET_IDX
#define AXI2CFG_SHD_GPIO_7                  1 << AXI2CFG_SHD_GPIO_7_IDX
#define AXI2CFG_SHD_GPIO_6                  1 << AXI2CFG_SHD_GPIO_6_IDX
#define AXI2CFG_SHD_GPIO_5                  1 << AXI2CFG_SHD_GPIO_5_IDX
#define AXI2CFG_SHD_GPIO_4                  1 << AXI2CFG_SHD_GPIO_4_IDX
#define AXI2CFG_SHD_GPIO_3                  1 << AXI2CFG_SHD_GPIO_3_IDX
#define AXI2CFG_SHD_GPIO_2                  1 << AXI2CFG_SHD_GPIO_2_IDX
#define AXI2CFG_SHD_GPIO_1                  1 << AXI2CFG_SHD_GPIO_1_IDX
#define AXI2CFG_SHD_GPIO_0                  1 << AXI2CFG_SHD_GPIO_0_IDX
#define AXI2CFG_EBI_CLK_DISABLE             1 << AXI2CFG_EBI_CLK_DISABLE_IDX
#define AXI2CFG_EXCEPTION_VECT_EN           1 << AXI2CFG_EXCEPTION_VECT_EN_IDX
#define AXI2CFG_RMII_EN                     1 << AXI2CFG_RMII_EN_IDX
#define AXI2CFG_REV_MII_EN                  1 << AXI2CFG_REV_MII_EN_IDX
#define AXI2CFG_DECODE_MUX_3                1 << AXI2CFG_DECODE_MUX_3_IDX
#define AXI2CFG_DECODE_MUX_2                1 << AXI2CFG_DECODE_MUX_2_IDX
#define AXI2CFG_DECODE_MUX_1                1 << AXI2CFG_DECODE_MUX_1_IDX
#define AXI2CFG_DECODE_MUX_0                1 << AXI2CFG_DECODE_MUX_0_IDX
#define AXI2CFG_FREQ_SYNTH_MUX              1 << AXI2CFG_FREQ_SYNTH_MUX_IDX
#define AXI2CFG_MASK_AXI_ERROR              1 << AXI2CFG_MASK_AXI_ERROR_IDX
#define AXI2CFG_REMAP                       1 << AXI2CFG_REMAP_IDX
#define AXI2CFG_WDG_PAUSE                   1 << AXI2CFG_WDG_PAUSE_IDX
#define AXI2CFG_CP15DISABLE                 1 << AXI2CFG_CP15DISABLE_IDX
#define AXI2CFG_DMA_CHAN_7_MUX              1 << AXI2CFG_DMA_CHAN_7_MUX_IDX
#define AXI2CFG_BOOT_MODE                   1 << AXI2CFG_BOOT_MODE_IDX

#endif /* PC302_AXI2CFG_H */
