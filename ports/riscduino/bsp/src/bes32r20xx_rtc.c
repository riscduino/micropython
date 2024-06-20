/********************************** (C) COPYRIGHT *******************************
* Copyright (c) 2024  BigEndian Semiconductor Private Limited
* SPDX-License-Identifier: Apache-2.0
*
* File Name          : bes32r20xx_rtc.c
* Author             : Dinesh Annayya
* Version            : V1.0.0
* Date               : 20-June-2024
* Description        : This file provides all the RTC firmware functions.
********************************************************************************/
#include "bes32r20xx_rtc.h"

/* RTC_Private_Defines */
#define RTC_LSB_MASK     ((uint32_t)0x0000FFFF) /* RTC LSB Mask */
#define PRLH_MSB_MASK    ((uint32_t)0x000F0000) /* RTC Prescaler MSB Mask */

/*********************************************************************
 * @fn      RTC_ITConfig
 *
 * @brief   Enables or disables the specified RTC interrupts.
 *
 * @param   RTC_IT - specifies the RTC interrupts sources to be enabled or disabled.
 *            RTC_IT_OW - Overflow interrupt
 *            RTC_IT_ALR - Alarm interrupt
 *            RTC_IT_SEC - Second interrupt
 *
 * @return  NewState - new state of the specified RTC interrupts(ENABLE or DISABLE).
 */
void RTC_ITConfig(uint16_t RTC_IT, FunctionalState NewState) {
}

/*********************************************************************
 * @fn      RTC_EnterConfigMode
 *
 * @brief   Enters the RTC configuration mode.
 *
 * @return  none
 */
void RTC_EnterConfigMode(void) {
}

/*********************************************************************
 * @fn      RTC_ExitConfigMode
 *
 * @brief   Exits from the RTC configuration mode.
 *
 * @return  none
 */
void RTC_ExitConfigMode(void) {
}

/*********************************************************************
 * @fn      RTC_GetCounter
 *
 * @brief   Gets the RTC counter value
 *
 * @return  RTC counter value
 */
uint32_t RTC_GetCounter(void) {
}

/*********************************************************************
 * @fn      RTC_SetCounter
 *
 * @brief   Sets the RTC counter value.
 *
 * @param   CounterValue - RTC counter new value.
 *
 * @return  RTC counter value
 */
void RTC_SetCounter(uint32_t CounterValue) {
}

/*********************************************************************
 * @fn      RTC_SetPrescaler
 *
 * @brief   Sets the RTC prescaler value
 *
 * @param   PrescalerValue - RTC prescaler new value
 *
 * @return  none
 */
void RTC_SetPrescaler(uint32_t PrescalerValue) {
}

/*********************************************************************
 * @fn      RTC_SetAlarm
 *
 * @brief   Sets the RTC alarm value
 *
 * @param   AlarmValue - RTC alarm new value
 *
 * @return  none
 */
void RTC_SetAlarm(uint32_t AlarmValue) {
}

/*********************************************************************
 * @fn      RTC_GetDivider
 *
 * @brief   Gets the RTC divider value
 *
 * @return  RTC Divider value
 */
uint32_t RTC_GetDivider(void) {
    return 0;
}

/*********************************************************************
 * @fn      RTC_WaitForLastTask
 *
 * @brief   Waits until last write operation on RTC registers has finished
 *
 * @return  none
 */
void RTC_WaitForLastTask(void) {
}

/*********************************************************************
 * @fn      RTC_WaitForSynchro
 *
 * @brief   Waits until the RTC registers are synchronized with RTC APB clock
 *
 * @return  none
 */
void RTC_WaitForSynchro(void) {
}

/*********************************************************************
 * @fn      RTC_GetFlagStatus
 *
 * @brief   Checks whether the specified RTC flag is set or not
 *
 * @param   RTC_FLAG- specifies the flag to check
 *            RTC_FLAG_RTOFF - RTC Operation OFF flag
 *            RTC_FLAG_RSF - Registers Synchronized flag
 *            RTC_FLAG_OW - Overflow flag
 *            RTC_FLAG_ALR - Alarm flag
 *            RTC_FLAG_SEC - Second flag
 *
 * @return  none
 */
FlagStatus RTC_GetFlagStatus(uint16_t RTC_FLAG) {
    return 0;
}

/*********************************************************************
 * @fn      RTC_ClearFlag
 *
 * @brief   Clears the RTC's pending flags
 *
 * @param   RTC_FLAG - specifies the flag to clear
 *            RTC_FLAG_RSF - Registers Synchronized flag
 *            RTC_FLAG_OW - Overflow flag
 *            RTC_FLAG_ALR - Alarm flag
 *            RTC_FLAG_SEC - Second flag
 *
 * @return  none
 */
void RTC_ClearFlag(uint16_t RTC_FLAG) {
}

/*********************************************************************
 * @fn      RTC_GetITStatus
 *
 * @brief   Checks whether the specified RTC interrupt has occurred or not
 *
 * @param   RTC_IT - specifies the RTC interrupts sources to check
 *            RTC_FLAG_OW - Overflow interrupt
 *            RTC_FLAG_ALR - Alarm interrupt
 *            RTC_FLAG_SEC - Second interrupt
 *
 * @return  The new state of the RTC_IT (SET or RESET)
 */
ITStatus RTC_GetITStatus(uint16_t RTC_IT) {
    return 0;
}

/*********************************************************************
 * @fn      RTC_ClearITPendingBit
 *
 * @brief   Clears the RTC's interrupt pending bits
 *
 * @param   RTC_IT - specifies the interrupt pending bit to clear
 *            RTC_FLAG_OW - Overflow interrupt
 *            RTC_FLAG_ALR - Alarm interrupt
 *            RTC_FLAG_SEC - Second interrupt
 *
 * @return  none
 */
void RTC_ClearITPendingBit(uint16_t RTC_IT) {
}
