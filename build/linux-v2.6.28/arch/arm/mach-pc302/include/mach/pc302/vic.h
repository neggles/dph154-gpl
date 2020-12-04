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

#ifndef PC302_VIC_H
#define PC302_VIC_H

/* Constants -------------------------------------------------------------- */

/*****************************************************************************/
/* Register Offset Addresses                                                 */
/*****************************************************************************/

/* Functional Registers */

#define VIC_IRQ_STATUS_REG_OFFSET               0x0000
#define VIC_FIQ_STATUS_REG_OFFSET               0x0004
#define VIC_RAW_INT_STATUS_REG_OFFSET           0x0008
#define VIC_INT_SELECT_REG_OFFSET               0x000C
#define VIC_INT_ENABLE_REG_OFFSET               0x0010
#define VIC_INT_ENABLE_CLEAR_REG_OFFSET         0x0014
#define VIC_SOFT_INT_REG_OFFSET                 0x0018
#define VIC_SOFT_INT_CLEAR_REG_OFFSET           0x001C
#define VIC_PROTECTION_REG_OFFSET               0x0020
#define VIC_SW_PRIORITY_MASK_REG_OFFSET         0x0024
#define VIC_VECT_PRIORITY_DAISYCHAIN_REG_OFFSET 0x0028
#define VIC_ADDRESS_REG_OFFSET                  0x0F00
#define VIC_VECT_ADDRESS_0_REG_OFFSET           0x0100
#define VIC_VECT_ADDRESS_1_REG_OFFSET           0x0104
#define VIC_VECT_ADDRESS_2_REG_OFFSET           0x0108
#define VIC_VECT_ADDRESS_3_REG_OFFSET           0x010C
#define VIC_VECT_ADDRESS_4_REG_OFFSET           0x0110
#define VIC_VECT_ADDRESS_5_REG_OFFSET           0x0114
#define VIC_VECT_ADDRESS_6_REG_OFFSET           0x0118
#define VIC_VECT_ADDRESS_7_REG_OFFSET           0x011C
#define VIC_VECT_ADDRESS_8_REG_OFFSET           0x0120
#define VIC_VECT_ADDRESS_9_REG_OFFSET           0x0124
#define VIC_VECT_ADDRESS_10_REG_OFFSET          0x0128
#define VIC_VECT_ADDRESS_11_REG_OFFSET          0x012C
#define VIC_VECT_ADDRESS_12_REG_OFFSET          0x0130
#define VIC_VECT_ADDRESS_13_REG_OFFSET          0x0134
#define VIC_VECT_ADDRESS_14_REG_OFFSET          0x0138
#define VIC_VECT_ADDRESS_15_REG_OFFSET          0x013C
#define VIC_VECT_ADDRESS_16_REG_OFFSET          0x0140
#define VIC_VECT_ADDRESS_17_REG_OFFSET          0x0144
#define VIC_VECT_ADDRESS_18_REG_OFFSET          0x0148
#define VIC_VECT_ADDRESS_19_REG_OFFSET          0x014C
#define VIC_VECT_ADDRESS_20_REG_OFFSET          0x0150
#define VIC_VECT_ADDRESS_21_REG_OFFSET          0x0154
#define VIC_VECT_ADDRESS_22_REG_OFFSET          0x0158
#define VIC_VECT_ADDRESS_23_REG_OFFSET          0x015C
#define VIC_VECT_ADDRESS_24_REG_OFFSET          0x0160
#define VIC_VECT_ADDRESS_25_REG_OFFSET          0x0164
#define VIC_VECT_ADDRESS_26_REG_OFFSET          0x0168
#define VIC_VECT_ADDRESS_27_REG_OFFSET          0x016C
#define VIC_VECT_ADDRESS_28_REG_OFFSET          0x0170
#define VIC_VECT_ADDRESS_29_REG_OFFSET          0x0174
#define VIC_VECT_ADDRESS_30_REG_OFFSET          0x0178
#define VIC_VECT_ADDRESS_31_REG_OFFSET          0x017C
#define VIC_VECT_PRIORITY_0_REG_OFFSET          0x0200
#define VIC_VECT_PRIORITY_1_REG_OFFSET          0x0204
#define VIC_VECT_PRIORITY_2_REG_OFFSET          0x0208
#define VIC_VECT_PRIORITY_3_REG_OFFSET          0x020C
#define VIC_VECT_PRIORITY_4_REG_OFFSET          0x0210
#define VIC_VECT_PRIORITY_5_REG_OFFSET          0x0214
#define VIC_VECT_PRIORITY_6_REG_OFFSET          0x0218
#define VIC_VECT_PRIORITY_7_REG_OFFSET          0x021C
#define VIC_VECT_PRIORITY_8_REG_OFFSET          0x0220
#define VIC_VECT_PRIORITY_9_REG_OFFSET          0x0224
#define VIC_VECT_PRIORITY_10_REG_OFFSET         0x0228
#define VIC_VECT_PRIORITY_11_REG_OFFSET         0x022C
#define VIC_VECT_PRIORITY_12_REG_OFFSET         0x0230
#define VIC_VECT_PRIORITY_13_REG_OFFSET         0x0234
#define VIC_VECT_PRIORITY_14_REG_OFFSET         0x0238
#define VIC_VECT_PRIORITY_15_REG_OFFSET         0x023C
#define VIC_VECT_PRIORITY_16_REG_OFFSET         0x0240
#define VIC_VECT_PRIORITY_17_REG_OFFSET         0x0244
#define VIC_VECT_PRIORITY_18_REG_OFFSET         0x0248
#define VIC_VECT_PRIORITY_19_REG_OFFSET         0x024C
#define VIC_VECT_PRIORITY_20_REG_OFFSET         0x0250
#define VIC_VECT_PRIORITY_21_REG_OFFSET         0x0254
#define VIC_VECT_PRIORITY_22_REG_OFFSET         0x0258
#define VIC_VECT_PRIORITY_23_REG_OFFSET         0x025C
#define VIC_VECT_PRIORITY_24_REG_OFFSET         0x0260
#define VIC_VECT_PRIORITY_25_REG_OFFSET         0x0264
#define VIC_VECT_PRIORITY_26_REG_OFFSET         0x0268
#define VIC_VECT_PRIORITY_27_REG_OFFSET         0x026C
#define VIC_VECT_PRIORITY_28_REG_OFFSET         0x0270
#define VIC_VECT_PRIORITY_29_REG_OFFSET         0x0274
#define VIC_VECT_PRIORITY_30_REG_OFFSET         0x0278
#define VIC_VECT_PRIORITY_31_REG_OFFSET         0x027C

/* Integration Test Registers */

#define VIC_INTEGRATION_CONTROL_REG_OFFSET                  0x0300
#define VIC_INTEGRATION_INPUT_1_REG_OFFSET                  0x0304
#define VIC_INTEGRATION_INPUT_2_REG_OFFSET                  0x0308
#define VIC_INTEGRATION_OUTPUT_1_REG_OFFSET                 0x030C
#define VIC_INTEGRATION_OUTPUT_2_REG_OFFSET                 0x0310
#define VIC_INTEGRATION_SAMPLED_SOURCE_STATUS_REG_OFFSET    0x0314
#define VIC_INTEGRATION_SAMPLED_SOURCE_CLEAR_REG_OFFSET     0x0318

/* Identification Registers */

#define VIC_PERIPHERAL_ID_0_REG_OFFSET          0x0FE0
#define VIC_PERIPHERAL_ID_1_REG_OFFSET          0x0FE4
#define VIC_PERIPHERAL_ID_2_REG_OFFSET          0x0FE8
#define VIC_PERIPHERAL_ID_3_REG_OFFSET          0x0FEC
#define VIC_PRIMECELL_ID_0_REG_OFFSET           0x0FF0
#define VIC_PRIMECELL_ID_1_REG_OFFSET           0x0FF4
#define VIC_PRIMECELL_ID_2_REG_OFFSET           0x0FF8
#define VIC_PRIMECELL_ID_3_REG_OFFSET           0x0FFC


/*****************************************************************************/
/* Register Reset Values                                                     */
/*****************************************************************************/

/* Functional Registers */
#define VIC_IRQ_STATUS_REG_RESET                0x00000000
#define VIC_FIQ_STATUS_REG_RESET                0x00000000
/* VIC_RAW_INT_STATUS_REG_RESET Unknown reset value */
#define VIC_INT_SELECT_REG_RESET                0x00000000
#define VIC_INT_ENABLE_REG_RESET                0x00000000
/* VIC_INT_ENABLE_CLEAR_REG_RESET Unknown reset value */
#define VIC_SOFT_INT_REG_RESET                  0x00000000
/* VIC_SOFT_INT_CLEAR_REG_RESET Unknown reset value */
#define VIC_PROTECTION_REG_RESET                0x00000000
#define VIC_SW_PRIORITY_MASK_REG_RESET          0x0000FFFF
#define VIC_VECT_PRIORITY_DAISYCHAIN_REG_RESET  0x0000000F
#define VIC_ADDRESS_REG_RESET                   0x00000000
#define VIC_VECT_ADDRESS_0_REG_RESET            0x00000000
#define VIC_VECT_ADDRESS_1_REG_RESET            0x00000000
#define VIC_VECT_ADDRESS_2_REG_RESET            0x00000000
#define VIC_VECT_ADDRESS_3_REG_RESET            0x00000000
#define VIC_VECT_ADDRESS_4_REG_RESET            0x00000000
#define VIC_VECT_ADDRESS_5_REG_RESET            0x00000000
#define VIC_VECT_ADDRESS_6_REG_RESET            0x00000000
#define VIC_VECT_ADDRESS_7_REG_RESET            0x00000000
#define VIC_VECT_ADDRESS_8_REG_RESET            0x00000000
#define VIC_VECT_ADDRESS_9_REG_RESET            0x00000000
#define VIC_VECT_ADDRESS_10_REG_RESET           0x00000000
#define VIC_VECT_ADDRESS_11_REG_RESET           0x00000000
#define VIC_VECT_ADDRESS_12_REG_RESET           0x00000000
#define VIC_VECT_ADDRESS_13_REG_RESET           0x00000000
#define VIC_VECT_ADDRESS_14_REG_RESET           0x00000000
#define VIC_VECT_ADDRESS_15_REG_RESET           0x00000000
#define VIC_VECT_ADDRESS_16_REG_RESET           0x00000000
#define VIC_VECT_ADDRESS_17_REG_RESET           0x00000000
#define VIC_VECT_ADDRESS_18_REG_RESET           0x00000000
#define VIC_VECT_ADDRESS_19_REG_RESET           0x00000000
#define VIC_VECT_ADDRESS_20_REG_RESET           0x00000000
#define VIC_VECT_ADDRESS_21_REG_RESET           0x00000000
#define VIC_VECT_ADDRESS_22_REG_RESET           0x00000000
#define VIC_VECT_ADDRESS_23_REG_RESET           0x00000000
#define VIC_VECT_ADDRESS_24_REG_RESET           0x00000000
#define VIC_VECT_ADDRESS_25_REG_RESET           0x00000000
#define VIC_VECT_ADDRESS_26_REG_RESET           0x00000000
#define VIC_VECT_ADDRESS_27_REG_RESET           0x00000000
#define VIC_VECT_ADDRESS_28_REG_RESET           0x00000000
#define VIC_VECT_ADDRESS_29_REG_RESET           0x00000000
#define VIC_VECT_ADDRESS_30_REG_RESET           0x00000000
#define VIC_VECT_ADDRESS_31_REG_RESET           0x00000000
#define VIC_VECT_PRIORITY_0_REG_RESET           0x0000000F
#define VIC_VECT_PRIORITY_1_REG_RESET           0x0000000F
#define VIC_VECT_PRIORITY_2_REG_RESET           0x0000000F
#define VIC_VECT_PRIORITY_3_REG_RESET           0x0000000F
#define VIC_VECT_PRIORITY_4_REG_RESET           0x0000000F
#define VIC_VECT_PRIORITY_5_REG_RESET           0x0000000F
#define VIC_VECT_PRIORITY_6_REG_RESET           0x0000000F
#define VIC_VECT_PRIORITY_7_REG_RESET           0x0000000F
#define VIC_VECT_PRIORITY_8_REG_RESET           0x0000000F
#define VIC_VECT_PRIORITY_9_REG_RESET           0x0000000F
#define VIC_VECT_PRIORITY_10_REG_RESET          0x0000000F
#define VIC_VECT_PRIORITY_11_REG_RESET          0x0000000F
#define VIC_VECT_PRIORITY_12_REG_RESET          0x0000000F
#define VIC_VECT_PRIORITY_13_REG_RESET          0x0000000F
#define VIC_VECT_PRIORITY_14_REG_RESET          0x0000000F
#define VIC_VECT_PRIORITY_15_REG_RESET          0x0000000F
#define VIC_VECT_PRIORITY_16_REG_RESET          0x0000000F
#define VIC_VECT_PRIORITY_17_REG_RESET          0x0000000F
#define VIC_VECT_PRIORITY_18_REG_RESET          0x0000000F
#define VIC_VECT_PRIORITY_19_REG_RESET          0x0000000F
#define VIC_VECT_PRIORITY_20_REG_RESET          0x0000000F
#define VIC_VECT_PRIORITY_21_REG_RESET          0x0000000F
#define VIC_VECT_PRIORITY_22_REG_RESET          0x0000000F
#define VIC_VECT_PRIORITY_23_REG_RESET          0x0000000F
#define VIC_VECT_PRIORITY_24_REG_RESET          0x0000000F
#define VIC_VECT_PRIORITY_25_REG_RESET          0x0000000F
#define VIC_VECT_PRIORITY_26_REG_RESET          0x0000000F
#define VIC_VECT_PRIORITY_27_REG_RESET          0x0000000F
#define VIC_VECT_PRIORITY_28_REG_RESET          0x0000000F
#define VIC_VECT_PRIORITY_29_REG_RESET          0x0000000F
#define VIC_VECT_PRIORITY_30_REG_RESET          0x0000000F
#define VIC_VECT_PRIORITY_31_REG_RESET          0x0000000F

/* Integration Test Registers */

#define VIC_INTEGRATION_CONTROL_REG_RESET                   0x00000000
/* VIC_INTEGRATION_INPUT_1_REG_RESET Unknown reset value */
/* VIC_INTEGRATION_INPUT_2_REG_RESET Unknown reset value */
#define VIC_INTEGRATION_OUTPUT_1_REG_RESET                  0x00000000
#define VIC_INTEGRATION_OUTPUT_2_REG_RESET                  0x00000000
#define VIC_INTEGRATION_SAMPLED_SOURCE_STATUS_REG_RESET     0x00000000
/* VIC_INTEGRATION_SAMPLED_SOURCE_CLEAR_REG_RESET Unknown reset value */

/* Identification Registers */

#define VIC_PERIPHERAL_ID_0_REG_RESET           0x00000092
#define VIC_PERIPHERAL_ID_1_REG_RESET           0x00000011
#define VIC_PERIPHERAL_ID_2_REG_RESET           0x00000004
#define VIC_PERIPHERAL_ID_3_REG_RESET           0x00000000
#define VIC_PRIMECELL_ID_0_REG_RESET            0x0000000D
#define VIC_PRIMECELL_ID_1_REG_RESET            0x000000F0
#define VIC_PRIMECELL_ID_2_REG_RESET            0x00000005
#define VIC_PRIMECELL_ID_3_REG_RESET            0x000000B1


/*****************************************************************************/
/* Register Bit Field Manipulation                                           */
/*****************************************************************************/

#define VIC_PROTECTION_REG_MASK                 0x00000001
#define VIC_SW_PRIORITY_MASK_REG_MASK           0x0000FFFF
#define VIC_VECT_PRIORITY_DAISYCHAIN_REG_MASK   0x0000000F
#define VIC_VECT_PRIORITY_REG_MASK              0x0000000F

#define VIC_PERIPHERAL_ID_MASK                  0x000000FF
#define VIC_PRIMECELL_ID_MASK                   0x000000FF

#endif /* PC302_VIC_H */
