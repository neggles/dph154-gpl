/* linux/include/asm-arm/arch-firecracker/fpga_debug.h
 *
 * Copyright (c) 2006 ip.access Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_FPGA_DEBUG_H
#define __ASM_ARCH_FPGA_DEBUG_H __FILE__

typedef void          (* IPA_FPGA_DEBUG_0) (const char* text);
typedef void          (* IPA_FPGA_DEBUG_1) (const char* text, int num1);
typedef void          (* IPA_FPGA_DEBUG_2) (const char* text, int num1, int num2);
typedef void          (* IPA_FPGA_DEBUG_3) (const char* text, int num1, int num2, int num3);
typedef void          (* IPA_FPGA_DEBUG_4) (const char* text, int num1, int num2, int num3, int num4);
typedef unsigned long (* IPA_HIRES_TIMER)  (void);

extern IPA_FPGA_DEBUG_0 ipaFpgaDebug0Ptr;
extern IPA_FPGA_DEBUG_1 ipaFpgaDebug1Ptr;
extern IPA_FPGA_DEBUG_2 ipaFpgaDebug2Ptr;
extern IPA_FPGA_DEBUG_3 ipaFpgaDebug3Ptr;
extern IPA_FPGA_DEBUG_4 ipaFpgaDebug4Ptr;
extern IPA_HIRES_TIMER  ipaHiresTimerPtr;

void           ipaFpgaDebug0(const char* text);
void           ipaFpgaDebug1(const char* text, int num1);
void           ipaFpgaDebug2(const char* text, int num1, int num2);
void           ipaFpgaDebug3(const char* text, int num1, int num2, int num3);
void           ipaFpgaDebug4(const char* text, int num1, int num2, int num3, int num4);
unsigned long  ipaHiresTimer(void);

#endif /* __ASM_ARCH_FPGA_DEBUG_H */
