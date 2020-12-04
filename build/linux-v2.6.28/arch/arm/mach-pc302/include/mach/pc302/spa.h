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

#ifndef PC302_SPA_H
#define PC302_SPA_H

/* Constants -------------------------------------------------------------- */

/*****************************************************************************/
/* Register Offset Addresses                                                 */
/*****************************************************************************/

/* base addresses */
#define SPA_1_BASE                      IPSEC_BASE
#define SPA_2_BASE                      SRTP_BASE
#define SPA_3_BASE                      CIPHER_BASE

/* Functional Registers */

#define SPA_IRQ_EN_REG_OFFSET           0x00000000
#define SPA_IRQ_STAT_REG_OFFSET         0x00000004
#define SPA_IRQ_CTRL_REG_OFFSET         0x00000008
#define SPA_FIFO_STAT_REG_OFFSET        0x0000000C
#define SPA_SDMA_BRST_SZ_REG_OFFSET     0x00000010
#define SPA_SRC_PTR_REG_OFFSET          0x00000020
#define SPA_DST_PTR_REG_OFFSET          0x00000024
#define SPA_OFFSET_REG_OFFSET           0x00000028
#define SPA_AAD_LEN_REG_OFFSET          0x0000002C
#define SPA_PROC_LEN_REG_OFFSET         0x00000030
#define SPA_ICV_LEN_REG_OFFSET          0x00000034
#define SPA_ICV_OFFSET_REG_OFFSET       0x00000038
#define SPA_SW_CTRL_REG_OFFSET          0x0000003C
#define SPA_CTRL_REG_OFFSET             0x00000040
#define SPA_AUX_INFO_REG_OFFSET         0x0000004C
#define SPA_STAT_POP_REG_OFFSET         0x00000050
#define SPA_STATUS_REG_OFFSET           0x00000054
#define SPA_KEY_SZ_REG_OFFSET           0x00000100
#define SPA_CIPH_KEY_BASE_REG_OFFSET    0x00004000
#define SPA_HASH_KEY_BASE_REG_OFFSET    0x00008000
#define SPA_RC4_CTX_BASE_REG_OFFSET     0x00020000
    
    
/*****************************************************************************/
/* Register Reset Values                                                     */
/*****************************************************************************/

#define SPA_IRQ_EN_REG_RESET            0x00000000
#define SPA_IRQ_CTRL_REG_RESET          0x00000000
#define SPA_FIFO_STAT_REG_RESET         0x00000000
#define SPA_SDMA_BRST_SZ_REG_RESET      0x00000000
#define SPA_SRC_PTR_REG_RESET           0x00000000
#define SPA_DST_PTR_REG_RESET           0x00000000
#define SPA_OFFSET_REG_RESET            0x00000000
#define SPA_AAD_LEN_REG_RESET           0x00000000
#define SPA_PROC_LEN_REG_RESET          0x00000000
#define SPA_ICV_LEN_REG_RESET           0x00000000
#define SPA_ICV_OFFSET_REG_RESET        0x00000000
#define SPA_SW_CTRL_REG_RESET           0x00000000
#define SPA_CTRL_REG_RESET              0x00000000
#define SPA_AUX_INFO_REG_RESET          0x00000000
#define SPA_STAT_POP_REG_RESET          0x00000000
#define SPA_STATUS_REG_RESET            0x00000000
#define SPA_KEY_SZ_REG_RESET            0x00000000
/* SPA_CIPH_KEY_BASE_REG_RESET    EQU Unknown reset value */
/* SPA_HASH_KEY_BASE_REG_RESET    EQU Unknown reset value */
/* SPA_RC4_CTX_BASE_REG_RESET     EQU Unknown reset value */


/*****************************************************************************/
/* Control Register Bit Definitions                                          */
/*****************************************************************************/
#define SPA_CTRL_HASH_ALG_IDX   4
#define SPA_CTRL_CIPH_MODE_IDX  8
#define SPA_CTRL_HASH_MODE_IDX 12
#define SPA_CTRL_CTX_IDX       16
#define SPA_CTRL_ENCRYPT_IDX   24

/*****************************************************************************/
/* Key Size Register Bit Definitions                                         */
/*****************************************************************************/
#define SPA_KEY_SZ_CXT_IDX     8
#define SPA_KEY_SZ_CIPHER_IDX 31

/*****************************************************************************/
/* Control Cipher Algorithm Register Bit Definitions                         */
/*****************************************************************************/
#define SPA_CTRL_CIPH_ALG_NULL    0x00
#define SPA_CTRL_CIPH_ALG_DES     0x01
#define SPA_CTRL_CIPH_ALG_AES     0x02
#define SPA_CTRL_CIPH_ALG_RC4     0x03
#define SPA_CTRL_CIPH_ALG_MULTI2  0x04
#define SPA_CTRL_CIPH_ALG_KASUMI  0x05

#define SPA_CTRL_HASH_ALG_NULL    0x00 << SPA_CTRL_HASH_ALG_IDX
#define SPA_CTRL_HASH_ALG_MD5     0x01 << SPA_CTRL_HASH_ALG_IDX
#define SPA_CTRL_HASH_ALG_SHA     0x02 << SPA_CTRL_HASH_ALG_IDX
#define SPA_CTRL_HASH_ALG_SHA224  0x03 << SPA_CTRL_HASH_ALG_IDX
#define SPA_CTRL_HASH_ALG_SHA256  0x04 << SPA_CTRL_HASH_ALG_IDX
#define SPA_CTRL_HASH_ALG_SHA384  0x05 << SPA_CTRL_HASH_ALG_IDX
#define SPA_CTRL_HASH_ALG_SHA512  0x06 << SPA_CTRL_HASH_ALG_IDX
#define SPA_CTRL_HASH_ALG_AESMAC  0x07 << SPA_CTRL_HASH_ALG_IDX
#define SPA_CTRL_HASH_ALG_AESCMAC 0x08 << SPA_CTRL_HASH_ALG_IDX
#define SPA_CTRL_HASH_ALG_KASF9   0x09 << SPA_CTRL_HASH_ALG_IDX

#define SPA_CTRL_CIPH_MODE_NULL   0x00 << SPA_CTRL_CIPH_MODE_IDX
#define SPA_CTRL_CIPH_MODE_ECB    0x00 << SPA_CTRL_CIPH_MODE_IDX
#define SPA_CTRL_CIPH_MODE_CBC    0x01 << SPA_CTRL_CIPH_MODE_IDX
#define SPA_CTRL_CIPH_MODE_CTR    0x02 << SPA_CTRL_CIPH_MODE_IDX
#define SPA_CTRL_CIPH_MODE_CCM    0x03 << SPA_CTRL_CIPH_MODE_IDX
#define SPA_CTRL_CIPH_MODE_GCM    0x05 << SPA_CTRL_CIPH_MODE_IDX
#define SPA_CTRL_CIPH_MODE_OFB    0x07 << SPA_CTRL_CIPH_MODE_IDX
#define SPA_CTRL_CIPH_MODE_CFB    0x08 << SPA_CTRL_CIPH_MODE_IDX
#define SPA_CTRL_CIPH_MODE_F8     0x09 << SPA_CTRL_CIPH_MODE_IDX

#define SPA_CTRL_HASH_MODE_RAW    0x00 << SPA_CTRL_HASH_MODE_IDX
#define SPA_CTRL_HASH_MODE_SSLMAC 0x01 << SPA_CTRL_HASH_MODE_IDX
#define SPA_CTRL_HASH_MODE_HMAC   0x02 << SPA_CTRL_HASH_MODE_IDX

#endif /* PC302_SPA_H */
