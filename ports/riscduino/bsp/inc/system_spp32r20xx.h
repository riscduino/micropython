/********************************** (C) COPYRIGHT *******************************
* Copyright (c) 2024  SiPlusPlus Semiconductor Private Limited
* SPDX-License-Identifier: Apache-2.0
*
* File Name          : system_spp32r20xx.h
* Author             : Dinesh Annayya
* Version            : V1.0.0
* Date               : 20-June-2024
* Description        : SPP32R20XX Device Peripheral Access Layer System Header File.
*******************************************************************************/
#ifndef __SYSTEM_SPP32R20XX_H
#define __SYSTEM_SPP32R20XX_H

#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t SystemCoreClock;          /* System Clock Frequency (Core Clock) */

/* System_Exported_Functions */
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);

#ifdef __cplusplus
}
#endif

#endif /*__SPP32R20XX_SYSTEM_H */
