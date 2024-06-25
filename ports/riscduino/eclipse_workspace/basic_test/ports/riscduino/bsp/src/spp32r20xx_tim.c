/********************************** (C) COPYRIGHT *******************************
* Copyright (c) 2024  SiPlusPlus Semiconductor Private Limited
* SPDX-License-Identifier: Apache-2.0
*
* File Name          : spp32r20xx_tim.c
* Author             : Dinesh Annayya
* Version            : V1.0.0
* Date               : 20-June-2024
* Description        : This file provides all the TIM firmware functions.
*******************************************************************************/
#include "spp32r20xx_tim.h"
#include "spp32r20xx_rcc.h"

/* TIM registers bit mask */
#define SMCFGR_ETR_Mask    ((uint16_t)0x00FF)
#define CHCTLR_Offset      ((uint16_t)0x0018)
#define CCER_CCE_Set       ((uint16_t)0x0001)
#define CCER_CCNE_Set      ((uint16_t)0x0004)


/*********************************************************************
 * @fn      TIM_DeInit
 *
 * @brief   Deinitializes the TIMx peripheral registers to their default
 *        reset values.
 *
 * @param   TIMx - where x can be 1 to 4 to select the TIM peripheral.
 *
 * @return  none
 */
void TIM_DeInit(TIM_TypeDef *TIMx) {
}

/*********************************************************************
 * @fn      TIM_TimeBaseInit
 *
 * @brief   Initializes the TIMx Time Base Unit peripheral according to
 *        the specified parameters in the TIM_TimeBaseInitStruct.
 *
 * @param   TIMx - where x can be 1 to 4 to select the TIM peripheral.
 *          TIM_TimeBaseInitStruct - pointer to a TIM_TimeBaseInitTypeDef
 *        structure.
 *
 * @return  none
 */
void TIM_TimeBaseInit(TIM_TypeDef *TIMx, TIM_TimeBaseInitTypeDef *TIM_TimeBaseInitStruct) {
}

/*********************************************************************
 * @fn      TIM_OC1Init
 *
 * @brief   Initializes the TIMx Channel1 according to the specified
 *        parameters in the TIM_OCInitStruct.
 *
 * @param   TIMx - where x can be 1 to 4 to select the TIM peripheral.
 *          TIM_OCInitStruct - pointer to a TIM_OCInitTypeDef structure.
 *
 * @return  none
 */
void TIM_OC1Init(TIM_TypeDef *TIMx, TIM_OCInitTypeDef *TIM_OCInitStruct) {
}

/*********************************************************************
 * @fn      TIM_OC2Init
 *
 * @brief   Initializes the TIMx Channel2 according to the specified
 *        parameters in the TIM_OCInitStruct.
 *
 * @param   TIMx - where x can be 1 to 4 to select the TIM peripheral.
 *          TIM_OCInitStruct - pointer to a TIM_OCInitTypeDef structure.
 *
 * @return  none
 */
void TIM_OC2Init(TIM_TypeDef *TIMx, TIM_OCInitTypeDef *TIM_OCInitStruct) {
}

/*********************************************************************
 * @fn      TIM_OC3Init
 *
 * @brief   Initializes the TIMx Channel3 according to the specified
 *        parameters in the TIM_OCInitStruct.
 *
 * @param   TIMx - where x can be 1 to 4 to select the TIM peripheral.
 *          TIM_OCInitStruct - pointer to a TIM_OCInitTypeDef structure.
 *
 * @return  none
 */
void TIM_OC3Init(TIM_TypeDef *TIMx, TIM_OCInitTypeDef *TIM_OCInitStruct) {
}

/*********************************************************************
 * @fn      TIM_OC4Init
 *
 * @brief   Initializes the TIMx Channel4 according to the specified
 *        parameters in the TIM_OCInitStruct.
 *
 * @param   TIMx - where x can be 1 to 4 to select the TIM peripheral.
 *          TIM_OCInitStruct - pointer to a TIM_OCInitTypeDef structure.
 *
 * @return  none
 */
void TIM_OC4Init(TIM_TypeDef *TIMx, TIM_OCInitTypeDef *TIM_OCInitStruct) {
}

/*********************************************************************
 * @fn      TIM_ICInit
 *
 * @brief   IInitializes the TIM peripheral according to the specified
 *        parameters in the TIM_ICInitStruct.
 * @param   TIMx - where x can be 1 to 4 to select the TIM peripheral.
 *          TIM_ICInitStruct - pointer to a TIM_ICInitTypeDef structure.
 *
 * @return  none
 */
void TIM_ICInit(TIM_TypeDef *TIMx, TIM_ICInitTypeDef *TIM_ICInitStruct) {
}

/*********************************************************************
 * @fn      TIM_PWMIConfig
 *
 * @brief   Configures the TIM peripheral according to the specified
 *        parameters in the TIM_ICInitStruct to measure an external
 *        PWM signal.
 *
 * @param   TIMx - where x can be 1 to 4 to select the TIM peripheral.
 *          TIM_ICInitStruct - pointer to a TIM_ICInitTypeDef structure.
 *
 * @return  none
 */
void TIM_PWMIConfig(TIM_TypeDef *TIMx, TIM_ICInitTypeDef *TIM_ICInitStruct) {
}

/*********************************************************************
 * @fn      TIM_BDTRConfig
 *
 * @brief   Configures the: Break feature, dead time, Lock level, the OSSI,
 *      the OSSR State and the AOE(automatic output enable).
 *
 * @param   TIMx - where x can be 1 to 4 to select the TIM peripheral.
 *          TIM_BDTRInitStruct - pointer to a TIM_BDTRInitTypeDef structure.
 *
 * @return  none
 */
void TIM_BDTRConfig(TIM_TypeDef *TIMx, TIM_BDTRInitTypeDef *TIM_BDTRInitStruct) {
}

/*********************************************************************
 * @fn      TIM_TimeBaseStructInit
 *
 * @brief   Fills each TIM_TimeBaseInitStruct member with its default value.
 *
 * @param   TIM_TimeBaseInitStruct - pointer to a TIM_TimeBaseInitTypeDef structure.
 *
 * @return  none
 */
void TIM_TimeBaseStructInit(TIM_TimeBaseInitTypeDef *TIM_TimeBaseInitStruct) {
}

/*********************************************************************
 * @fn      TIM_OCStructInit
 *
 * @brief   Fills each TIM_OCInitStruct member with its default value.
 *
 * @param   TIM_OCInitStruct - pointer to a TIM_OCInitTypeDef structure.
 *
 * @return  none
 */
void TIM_OCStructInit(TIM_OCInitTypeDef *TIM_OCInitStruct) {
}

/*********************************************************************
 * @fn      TIM_ICStructInit
 *
 * @brief   Fills each TIM_ICInitStruct member with its default value.
 *
 * @param   TIM_ICInitStruct - pointer to a TIM_ICInitTypeDef structure.
 *
 * @return  none
 */
void TIM_ICStructInit(TIM_ICInitTypeDef *TIM_ICInitStruct) {
}

/*********************************************************************
 * @fn      TIM_BDTRStructInit
 *
 * @brief   Fills each TIM_BDTRInitStruct member with its default value.
 *
 * @param   TIM_BDTRInitStruct - pointer to a TIM_BDTRInitTypeDef structure.
 *
 * @return  none
 */
void TIM_BDTRStructInit(TIM_BDTRInitTypeDef *TIM_BDTRInitStruct) {
}

/*********************************************************************
 * @fn      TIM_Cmd
 *
 * @brief   Enables or disables the specified TIM peripheral.
 *
 * @param   TIMx - where x can be 1 to 4 to select the TIM peripheral.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void TIM_Cmd(TIM_TypeDef *TIMx, FunctionalState NewState) {
}

/*********************************************************************
 * @fn      TIM_CtrlPWMOutputs
 *
 * @brief   Enables or disables the TIM peripheral Main Outputs.
 *
 * @param   TIMx - where x can be 1 to 4 to select the TIM peripheral.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void TIM_CtrlPWMOutputs(TIM_TypeDef *TIMx, FunctionalState NewState) {
}

/*********************************************************************
 * @fn      TIM_ITConfig
 *
 * @brief   Enables or disables the specified TIM interrupts.
 *
 * @param   TIMx - where x can be 1 to 4 to select the TIM peripheral.
 *          TIM_IT - specifies the TIM interrupts sources to be enabled or disabled.
 *            TIM_IT_Update - TIM update Interrupt source.
 *            TIM_IT_CC1 - TIM Capture Compare 1 Interrupt source.
 *            TIM_IT_CC2 - TIM Capture Compare 2 Interrupt source
 *            TIM_IT_CC3 - TIM Capture Compare 3 Interrupt source.
 *            TIM_IT_CC4 - TIM Capture Compare 4 Interrupt source.
 *            TIM_IT_COM - TIM Commutation Interrupt source.
 *            TIM_IT_Trigger - TIM Trigger Interrupt source.
 *            TIM_IT_Break - TIM Break Interrupt source.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void TIM_ITConfig(TIM_TypeDef *TIMx, uint16_t TIM_IT, FunctionalState NewState) {
}

void TIM_GenerateEvent(TIM_TypeDef *TIMx, uint16_t TIM_EventSource) {
}



/*********************************************************************
 * @fn      TIM_InternalClockConfig
 *
 * @brief   Configures the TIMx internal Clock.
 *
 * @param   TIMx - where x can be 1 to 4 to select the TIM peripheral.
 *
 * @return  none
 */
void TIM_InternalClockConfig(TIM_TypeDef *TIMx) {
}




/*********************************************************************
 * @fn      TIM_ETRConfig
 *
 * @brief   Configures the TIMx External Trigger (ETR).
 *
 * @param   TIMx - where x can be 1 to 4 to select the TIM peripheral.
 *          TIM_ExtTRGPrescaler - The external Trigger Prescaler.
 *            TIM_ExtTRGPSC_OFF - ETRP Prescaler OFF.
 *            TIM_ExtTRGPSC_DIV2 - ETRP frequency divided by 2.
 *            TIM_ExtTRGPSC_DIV4 - ETRP frequency divided by 4.
 *            TIM_ExtTRGPSC_DIV8 - ETRP frequency divided by 8.
 *          TIM_ExtTRGPolarity - The external Trigger Polarity.
 *            TIM_ExtTRGPolarity_Inverted - active low or falling edge active.
 *            TIM_ExtTRGPolarity_NonInverted - active high or rising edge active.
 *          ExtTRGFilter - External Trigger Filter.
 *            This parameter must be a value between 0x0 and 0xF.
 *
 * @return  none
 */
void TIM_ETRConfig(TIM_TypeDef *TIMx, uint16_t TIM_ExtTRGPrescaler, uint16_t TIM_ExtTRGPolarity,
    uint16_t ExtTRGFilter) {
}

/*********************************************************************
 * @fn      TIM_PrescalerConfig
 *
 * @brief   Configures the TIMx Prescaler.
 *
 * @param   TIMx - where x can be 1 to 4 to select the TIM peripheral.
 *          Prescaler - specifies the Prescaler Register value.
 *          TIM_PSCReloadMode - specifies the TIM Prescaler Reload mode.
 *            TIM_PSCReloadMode - specifies the TIM Prescaler Reload mode.
 *            TIM_PSCReloadMode_Update - The Prescaler is loaded at the update event.
 *            TIM_PSCReloadMode_Immediate - The Prescaler is loaded immediately.
 *
 * @return  none
 */
void TIM_PrescalerConfig(TIM_TypeDef *TIMx, uint16_t Prescaler, uint16_t TIM_PSCReloadMode) {
}

/*********************************************************************
 * @fn      TIM_CounterModeConfig
 *
 * @brief   Specifies the TIMx Counter Mode to be used.
 *
 * @param   TIMx - where x can be 1 to 4 to select the TIM peripheral.
 *          TIM_CounterMode - specifies the Counter Mode to be used.
 *            TIM_CounterMode_Up - TIM Up Counting Mode.
 *            TIM_CounterMode_Down - TIM Down Counting Mode.
 *            TIM_CounterMode_CenterAligned1 - TIM Center Aligned Mode1.
 *            TIM_CounterMode_CenterAligned2 - TIM Center Aligned Mode2.
 *            TIM_CounterMode_CenterAligned3 - TIM Center Aligned Mode3.
 *
 * @return  none
 */
void TIM_CounterModeConfig(TIM_TypeDef *TIMx, uint16_t TIM_CounterMode) {
}

/*********************************************************************
 * @fn      TIM_SelectInputTrigger
 *
 * @brief   Selects the Input Trigger source.
 *
 * @param   TIMx - where x can be 1 to 4 to select the TIM peripheral.
 *          TIM_InputTriggerSource - The Input Trigger source.
 *            TIM_TS_ITR0 - Internal Trigger 0.
 *            TIM_TS_ITR1 - Internal Trigger 1.
 *            TIM_TS_ITR2 - Internal Trigger 2.
 *            TIM_TS_ITR3 - Internal Trigger 3.
 *            TIM_TS_TI1F_ED - TI1 Edge Detector.
 *            TIM_TS_TI1FP1 - Filtered Timer Input 1.
 *            TIM_TS_TI2FP2 - Filtered Timer Input 2.
 *            TIM_TS_ETRF - External Trigger input.
 *
 * @return  none
 */
void TIM_SelectInputTrigger(TIM_TypeDef *TIMx, uint16_t TIM_InputTriggerSource) {
}




