/********************************** (C) COPYRIGHT *******************************
* Copyright (c) 2024  SiPlusPlus Semiconductor Private Limited
* SPDX-License-Identifier: Apache-2.0
*
* File Name          : system_spp32r20xx.c
* Author             : Dinesh Annayya
* Version            : V1.0.0
* Date               : 20-June-2024
* Description        : System clock set 10Mhz
*********************************************************************************/
#include "spp32r20xx.h"

uint32_t SystemCoreClock = 50000000;     /* System Clock Frequency: 50Mhz  */
//uint32_t SystemCoreClock = 29491200;     /* System Clock Frequency: 29.4912Mhz  */


/* system_private_function_proto_types */
static void SetSysClock(void);




/*********************************************************************
 * @fn      SystemInit
 *
 * @brief   Setup the microcontroller system Initialize the Embedded Flash Interface,
 *        the PLL and update the SystemCoreClock variable.
 *
 * @return  none
 */
void SystemInit(void) {
    SetSysClock();
}

/*********************************************************************
 * @fn      SystemCoreClockUpdate
 *
 * @brief   Update SystemCoreClock variable according to Clock Register Values.
 *
 * @return  none
 */
void SystemCoreClockUpdate(void) {
}

/*********************************************************************
 * @fn      SetSysClock
 *
 * @brief   Configures the System clock frequency.
 *
 * @return  none
 */
static void SetSysClock(void) {
}

