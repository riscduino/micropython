/********************************** (C) COPYRIGHT *******************************
* Copyright (c) 2024  SiPlusPlus Semiconductor Private Limited
* SPDX-License-Identifier: Apache-2.0
*
* File Name              : spp32r20xx_rcc.c
* Author                 : Dinesh Annayya
* Version                : V1.0.0
* Date                   : 20-June-2024
* Description            : This file provides all the RCC firmware functions.
*******************************************************************************/
#include "spp32r20xx_rcc.h"

/* RCC registers bit address in the alias region */
#define RCC_OFFSET                 (RCC_BASE - PERIPH_BASE)

/* BDCTLR Register */
#define BDCTLR_OFFSET              (RCC_OFFSET + 0x20)

/* RCC registers bit mask */

/* CTLR register bit mask */
#define CTLR_HSEBYP_Reset          ((uint32_t)0xFFFBFFFF)
#define CTLR_HSEBYP_Set            ((uint32_t)0x00040000)
#define CTLR_HSEON_Reset           ((uint32_t)0xFFFEFFFF)
#define CTLR_HSEON_Set             ((uint32_t)0x00010000)
#define CTLR_HSITRIM_Mask          ((uint32_t)0xFFFFFF07)

#define CFGR0_PLL_Mask             ((uint32_t)0xFFC0FFFF) /* 103 */
#define CFGR0_PLL_Mask_1           ((uint32_t)0xFFC2FFFF) /* 107 */

#define CFGR0_PLLMull_Mask         ((uint32_t)0x003C0000)
#define CFGR0_PLLSRC_Mask          ((uint32_t)0x00010000)
#define CFGR0_PLLXTPRE_Mask        ((uint32_t)0x00020000)
#define CFGR0_SWS_Mask             ((uint32_t)0x0000000C)
#define CFGR0_SW_Mask              ((uint32_t)0xFFFFFFFC)
#define CFGR0_HPRE_Reset_Mask      ((uint32_t)0xFFFFFF0F)
#define CFGR0_HPRE_Set_Mask        ((uint32_t)0x000000F0)
#define CFGR0_PPRE1_Reset_Mask     ((uint32_t)0xFFFFF8FF)
#define CFGR0_PPRE1_Set_Mask       ((uint32_t)0x00000700)
#define CFGR0_PPRE2_Reset_Mask     ((uint32_t)0xFFFFC7FF)
#define CFGR0_PPRE2_Set_Mask       ((uint32_t)0x00003800)
#define CFGR0_ADCPRE_Reset_Mask    ((uint32_t)0xFFFF3FFF)
#define CFGR0_ADCPRE_Set_Mask      ((uint32_t)0x0000C000)

/* RSTSCKR register bit mask */
#define RSTSCKR_RMVF_Set           ((uint32_t)0x01000000)

/* CFGR2 register bit mask */
#define CFGR2_PREDIV1SRC           ((uint32_t)0x00010000)
#define CFGR2_PREDIV1              ((uint32_t)0x0000000F)
#define CFGR2_PREDIV2              ((uint32_t)0x000000F0)
#define CFGR2_PLL2MUL              ((uint32_t)0x00000F00)
#define CFGR2_PLL3MUL              ((uint32_t)0x0000F000)

/* RCC Flag Mask */
#define FLAG_Mask                  ((uint8_t)0x1F)

/* INTR register byte 2 (Bits[15:8]) base address */
#define INTR_BYTE2_ADDRESS         ((uint32_t)0x40021009)

/* INTR register byte 3 (Bits[23:16]) base address */
#define INTR_BYTE3_ADDRESS         ((uint32_t)0x4002100A)

/* CFGR0 register byte 4 (Bits[31:24]) base address */
#define CFGR0_BYTE4_ADDRESS        ((uint32_t)0x40021007)

/* BDCTLR register base address */
#define BDCTLR_ADDRESS             (PERIPH_BASE + BDCTLR_OFFSET)

static __I uint8_t APBAHBPrescTable[16] = {0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9};
static __I uint8_t ADCPrescTable[4] = {2, 4, 6, 8};

/*********************************************************************
 * @fn      RCC_DeInit
 *
 * @brief   Resets the RCC clock configuration to the default reset state.
 *
 * @return  none
 */
void RCC_DeInit(void) {
}

/*********************************************************************
 * @fn      RCC_HSEConfig
 *
 * @brief   Configures the External High Speed oscillator (HSE).
 *
 * @param   RCC_HSE -
 *            RCC_HSE_OFF - HSE oscillator OFF.
 *            RCC_HSE_ON - HSE oscillator ON.
 *            RCC_HSE_Bypass - HSE oscillator bypassed with external clock.
 *
 * @return  none
 */
void RCC_HSEConfig(uint32_t RCC_HSE) {
}

/*********************************************************************
 * @fn      RCC_WaitForHSEStartUp
 *
 * @brief   Waits for HSE start-up.
 *
 * @return  SUCCESS - HSE oscillator is stable and ready to use.
 *                  ERROR - HSE oscillator not yet ready.
 */
ErrorStatus RCC_WaitForHSEStartUp(void) {
    return 0;
}

/*********************************************************************
 * @fn      RCC_AdjustHSICalibrationValue
 *
 * @brief   Adjusts the Internal High Speed oscillator (HSI) calibration value.
 *
 * @param   HSICalibrationValue - specifies the calibration trimming value.
 *                    This parameter must be a number between 0 and 0x1F.
 *
 * @return  none
 */
void RCC_AdjustHSICalibrationValue(uint8_t HSICalibrationValue) {
}

/*********************************************************************
 * @fn      RCC_HSICmd
 *
 * @brief   Enables or disables the Internal High Speed oscillator (HSI).
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_HSICmd(FunctionalState NewState) {
}

/*********************************************************************
 * @fn      RCC_PLLConfig
 *
 * @brief   Configures the PLL clock source and multiplication factor.
 *
 * @param   RCC_PLLSource - specifies the PLL entry clock source.
 *            RCC_PLLSource_HSI_Div2 - HSI oscillator clock divided by 2
 *        selected as PLL clock entry.
 *            RCC_PLLSource_PREDIV1 - PREDIV1 clock selected as PLL clock
 *        entry.
 *          RCC_PLLMul - specifies the PLL multiplication factor.
 *            This parameter can be RCC_PLLMul_x where x:[2,16].
 *            For CH32V307 -
 *              RCC_PLLMul_18_EXTEN
 *              RCC_PLLMul_3_EXTEN
 *              RCC_PLLMul_4_EXTEN
 *              RCC_PLLMul_5_EXTEN
 *              RCC_PLLMul_6_EXTEN
 *              RCC_PLLMul_7_EXTEN
 *              RCC_PLLMul_8_EXTEN
 *              RCC_PLLMul_9_EXTEN
 *              RCC_PLLMul_10_EXTEN
 *              RCC_PLLMul_11_EXTEN
 *              RCC_PLLMul_12_EXTEN
 *              RCC_PLLMul_13_EXTEN
 *              RCC_PLLMul_14_EXTEN
 *              RCC_PLLMul_6_5_EXTEN
 *              RCC_PLLMul_15_EXTEN
 *              RCC_PLLMul_16_EXTEN
 *            For other CH32V30x -
 *              RCC_PLLMul_2
 *              RCC_PLLMul_3
 *              RCC_PLLMul_4
 *              RCC_PLLMul_5
 *              RCC_PLLMul_6
 *              RCC_PLLMul_7
 *              RCC_PLLMul_8
 *              RCC_PLLMul_9
 *              RCC_PLLMul_10
 *              RCC_PLLMul_11
 *              RCC_PLLMul_12
 *              RCC_PLLMul_13
 *              RCC_PLLMul_14
 *              RCC_PLLMul_15
 *              RCC_PLLMul_16
 *              RCC_PLLMul_18
 *
 * @return  none
 */
void RCC_PLLConfig(uint32_t RCC_PLLSource, uint32_t RCC_PLLMul) {
}

/*********************************************************************
 * @fn      RCC_PLLCmd
 *
 * @brief   Enables or disables the PLL.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_PLLCmd(FunctionalState NewState) {
}

/*********************************************************************
 * @fn      RCC_SYSCLKConfig
 *
 * @brief   Configures the system clock (SYSCLK).
 *
 * @param   RCC_SYSCLKSource - specifies the clock source used as system clock.
 *            RCC_SYSCLKSource_HSI - HSI selected as system clock.
 *            RCC_SYSCLKSource_HSE - HSE selected as system clock.
 *            RCC_SYSCLKSource_PLLCLK - PLL selected as system clock.
 *
 * @return  none
 */
void RCC_SYSCLKConfig(uint32_t RCC_SYSCLKSource) {
}

/*********************************************************************
 * @fn      RCC_GetSYSCLKSource
 *
 * @brief   Returns the clock source used as system clock.
 *
 * @return  0x00 - HSI used as system clock.
 *          0x04 - HSE used as system clock.
 *          0x08 - PLL used as system clock.
 */
uint8_t RCC_GetSYSCLKSource(void) {
}

/*********************************************************************
 * @fn      RCC_HCLKConfig
 *
 * @brief   Configures the AHB clock (HCLK).
 *
 * @param   RCC_SYSCLK - defines the AHB clock divider. This clock is derived from
 *        the system clock (SYSCLK).
 *            RCC_SYSCLK_Div1 - AHB clock = SYSCLK.
 *            RCC_SYSCLK_Div2 - AHB clock = SYSCLK/2.
 *            RCC_SYSCLK_Div4 - AHB clock = SYSCLK/4.
 *            RCC_SYSCLK_Div8 - AHB clock = SYSCLK/8.
 *            RCC_SYSCLK_Div16 - AHB clock = SYSCLK/16.
 *            RCC_SYSCLK_Div64 - AHB clock = SYSCLK/64.
 *            RCC_SYSCLK_Div128 - AHB clock = SYSCLK/128.
 *            RCC_SYSCLK_Div256 - AHB clock = SYSCLK/256.
 *            RCC_SYSCLK_Div512 - AHB clock = SYSCLK/512.
 *
 * @return  none
 */
void RCC_HCLKConfig(uint32_t RCC_SYSCLK) {
}

/*********************************************************************
 * @fn      RCC_PCLK1Config
 *
 * @brief   Configures the Low Speed APB clock (PCLK1).
 *
 * @param   RCC_HCLK - defines the APB1 clock divider. This clock is derived from
 *        the AHB clock (HCLK).
 *            RCC_HCLK_Div1 - APB1 clock = HCLK.
 *            RCC_HCLK_Div2 - APB1 clock = HCLK/2.
 *            RCC_HCLK_Div4 - APB1 clock = HCLK/4.
 *            RCC_HCLK_Div8 - APB1 clock = HCLK/8.
 *            RCC_HCLK_Div16 - APB1 clock = HCLK/16.
 *
 * @return  none
 */
void RCC_PCLK1Config(uint32_t RCC_HCLK) {
}

/*********************************************************************
 * @fn      RCC_PCLK2Config
 *
 * @brief   Configures the High Speed APB clock (PCLK2).
 *
 * @param   RCC_HCLK - defines the APB2 clock divider. This clock is derived from
 *        the AHB clock (HCLK).
 *            RCC_HCLK_Div1 - APB1 clock = HCLK.
 *            RCC_HCLK_Div2 - APB1 clock = HCLK/2.
 *            RCC_HCLK_Div4 - APB1 clock = HCLK/4.
 *            RCC_HCLK_Div8 - APB1 clock = HCLK/8.
 *            RCC_HCLK_Div16 - APB1 clock = HCLK/16.
 *
 * @return  none
 */
void RCC_PCLK2Config(uint32_t RCC_HCLK) {
}

/*********************************************************************
 * @fn      RCC_ITConfig
 *
 * @brief   Enables or disables the specified RCC interrupts.
 *
 * @param   RCC_IT - specifies the RCC interrupt sources to be enabled or disabled.
 *            RCC_IT_LSIRDY - LSI ready interrupt.
 *            RCC_IT_LSERDY - LSE ready interrupt.
 *            RCC_IT_HSIRDY - HSI ready interrupt.
 *            RCC_IT_HSERDY - HSE ready interrupt.
 *            RCC_IT_PLLRDY - PLL ready interrupt.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_ITConfig(uint8_t RCC_IT, FunctionalState NewState) {
}

/*********************************************************************
 * @fn      RCC_ADCCLKConfig
 *
 * @brief   Configures the ADC clock (ADCCLK).
 *
 * @param   RCC_PCLK2 - defines the ADC clock divider. This clock is derived from
 *        the APB2 clock (PCLK2).
 *          RCC_PCLK2_Div2 - ADC clock = PCLK2/2.
 *          RCC_PCLK2_Div4 - ADC clock = PCLK2/4.
 *          RCC_PCLK2_Div6 - ADC clock = PCLK2/6.
 *          RCC_PCLK2_Div8 - ADC clock = PCLK2/8.
 *
 * @return  none
 */
void RCC_ADCCLKConfig(uint32_t RCC_PCLK2) {
}

/*********************************************************************
 * @fn      RCC_LSEConfig
 *
 * @brief   Configures the External Low Speed oscillator (LSE).
 *
 * @param   RCC_LSE - specifies the new state of the LSE.
 *            RCC_LSE_OFF - LSE oscillator OFF.
 *            RCC_LSE_ON - LSE oscillator ON.
 *            RCC_LSE_Bypass - LSE oscillator bypassed with external clock.
 *
 * @return  none
 */
void RCC_LSEConfig(uint8_t RCC_LSE) {
}

/*********************************************************************
 * @fn      RCC_LSICmd
 *
 * @brief   Enables or disables the Internal Low Speed oscillator (LSI).
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_LSICmd(FunctionalState NewState) {
}

/*********************************************************************
 * @fn      RCC_RTCCLKConfig
 *
 * @brief   Once the RTC clock is selected it can't be changed unless the Backup domain is reset.
 *
 * @param   RCC_RTCCLKSource - specifies the RTC clock source.
 *            RCC_RTCCLKSource_LSE - LSE selected as RTC clock.
 *            RCC_RTCCLKSource_LSI - LSI selected as RTC clock.
 *            RCC_RTCCLKSource_HSE_Div128 - HSE clock divided by 128 selected as RTC clock.
 *
 * @return  none
 */
void RCC_RTCCLKConfig(uint32_t RCC_RTCCLKSource) {
}

/*********************************************************************
 * @fn      RCC_RTCCLKCmd
 *
 * @brief   This function must be used only after the RTC clock was selected
 *        using the RCC_RTCCLKConfig function.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_RTCCLKCmd(FunctionalState NewState) {
}

/*********************************************************************
 * @fn      RCC_GetClocksFreq
 *
 * @brief   The result of this function could be not correct when using
 *        fractional value for HSE crystal.
 *
 * @param   RCC_Clocks - pointer to a RCC_ClocksTypeDef structure which will hold
 *        the clocks frequencies.
 *
 * @return  none
 */
void RCC_GetClocksFreq(RCC_ClocksTypeDef *RCC_Clocks) {
}

/*********************************************************************
 * @fn      RCC_AHBPeriphClockCmd
 *
 * @brief   Enables or disables the AHB peripheral clock.
 *
 * @param   RCC_AHBPeriph - specifies the AHB peripheral to gates its clock.
 *            RCC_AHBPeriph_DMA1.
 *            RCC_AHBPeriph_DMA2.
 *            RCC_AHBPeriph_SRAM.
 *            RCC_AHBPeriph_CRC.
 *            RCC_AHBPeriph_FSMC
 *            RCC_AHBPeriph_RNG
 *            RCC_AHBPeriph_SDIO
 *            RCC_AHBPeriph_USBHS
 *            RCC_AHBPeriph_OTG_FS
 *            RCC_AHBPeriph_DVP
 *            RCC_AHBPeriph_ETH_MAC
 *            RCC_AHBPeriph_ETH_MAC_Tx
 *            RCC_AHBPeriph_ETH_MAC_Rx
 *          NewState: ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_AHBPeriphClockCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState) {
}

/*********************************************************************
 * @fn      RCC_APB2PeriphClockCmd
 *
 * @brief   Enables or disables the High Speed APB (APB2) peripheral clock.
 *
 * @param   RCC_APB2Periph - specifies the APB2 peripheral to gates its clock.
 *            RCC_APB2Periph_AFIO.
 *            RCC_APB2Periph_GPIOA.
 *            RCC_APB2Periph_GPIOB.
 *            RCC_APB2Periph_GPIOC.
 *            RCC_APB2Periph_GPIOD.
 *            RCC_APB2Periph_GPIOE
 *            RCC_APB2Periph_ADC1.
 *            RCC_APB2Periph_ADC2
 *            RCC_APB2Periph_TIM1.
 *            RCC_APB2Periph_SPI1.
 *            RCC_APB2Periph_TIM8
 *            RCC_APB2Periph_USART1.
 *            RCC_APB2Periph_TIM9
 *            RCC_APB2Periph_TIM10
 *          NewState - ENABLE or DISABLE
 *
 * @return  none
 */
void RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState) {
}

/*********************************************************************
 * @fn      RCC_APB1PeriphClockCmd
 *
 * @brief   Enables or disables the Low Speed APB (APB1) peripheral clock.
 *
 * @param   RCC_APB1Periph - specifies the APB1 peripheral to gates its clock.
 *            RCC_APB1Periph_TIM2.
 *            RCC_APB1Periph_TIM3.
 *            RCC_APB1Periph_TIM4.
 *            RCC_APB1Periph_TIM5
 *            RCC_APB1Periph_TIM6
 *            RCC_APB1Periph_TIM7
 *            RCC_APB1Periph_UART6
 *            RCC_APB1Periph_UART7
 *            RCC_APB1Periph_UART8
 *            RCC_APB1Periph_WWDG.
 *            RCC_APB1Periph_SPI2.
 *            RCC_APB1Periph_SPI3.
 *            RCC_APB1Periph_USART2.
 *            RCC_APB1Periph_USART3.
 *            RCC_APB1Periph_UART4
 *            RCC_APB1Periph_UART5
 *            RCC_APB1Periph_I2C1.
 *            RCC_APB1Periph_I2C2.
 *            RCC_APB1Periph_USB.
 *            RCC_APB1Periph_CAN1.
 *            RCC_APB1Periph_BKP.
 *            RCC_APB1Periph_PWR.
 *            RCC_APB1Periph_DAC.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState NewState) {
}

/*********************************************************************
 * @fn      RCC_APB2PeriphResetCmd
 *
 * @brief   Forces or releases High Speed APB (APB2) peripheral reset.
 *
 * @param   RCC_APB2Periph - specifies the APB2 peripheral to reset.
 *            RCC_APB2Periph_AFIO.
 *            RCC_APB2Periph_GPIOA.
 *            RCC_APB2Periph_GPIOB.
 *            RCC_APB2Periph_GPIOC.
 *            RCC_APB2Periph_GPIOD.
 *            RCC_APB2Periph_GPIOE
 *            RCC_APB2Periph_ADC1.
 *            RCC_APB2Periph_ADC2
 *            RCC_APB2Periph_TIM1.
 *            RCC_APB2Periph_SPI1.
 *            RCC_APB2Periph_TIM8
 *            RCC_APB2Periph_USART1.
 *            RCC_APB2Periph_TIM9
 *            RCC_APB2Periph_TIM10
 *          NewState - ENABLE or DISABLE
 *
 * @return  none
 */
void RCC_APB2PeriphResetCmd(uint32_t RCC_APB2Periph, FunctionalState NewState) {
}

/*********************************************************************
 * @fn      RCC_APB1PeriphResetCmd
 *
 * @brief   Forces or releases Low Speed APB (APB1) peripheral reset.
 *
 * @param   RCC_APB1Periph - specifies the APB1 peripheral to reset.
 *            RCC_APB1Periph_TIM2.
 *            RCC_APB1Periph_TIM3.
 *            RCC_APB1Periph_TIM4.
 *            RCC_APB1Periph_TIM5
 *            RCC_APB1Periph_TIM6
 *            RCC_APB1Periph_TIM7
 *            RCC_APB1Periph_UART6
 *            RCC_APB1Periph_UART7
 *            RCC_APB1Periph_UART8
 *            RCC_APB1Periph_WWDG.
 *            RCC_APB1Periph_SPI2.
 *            RCC_APB1Periph_SPI3.
 *            RCC_APB1Periph_USART2.
 *            RCC_APB1Periph_USART3.
 *            RCC_APB1Periph_UART4
 *            RCC_APB1Periph_UART5
 *            RCC_APB1Periph_I2C1.
 *            RCC_APB1Periph_I2C2.
 *            RCC_APB1Periph_USB.
 *            RCC_APB1Periph_CAN1.
 *            RCC_APB1Periph_BKP.
 *            RCC_APB1Periph_PWR.
 *            RCC_APB1Periph_DAC.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_APB1PeriphResetCmd(uint32_t RCC_APB1Periph, FunctionalState NewState) {
}

/*********************************************************************
 * @fn      RCC_BackupResetCmd
 *
 * @brief   Forces or releases the Backup domain reset.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_BackupResetCmd(FunctionalState NewState) {
}

/*********************************************************************
 * @fn      RCC_ClockSecuritySystemCmd
 *
 * @brief   Enables or disables the Clock Security System.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_ClockSecuritySystemCmd(FunctionalState NewState) {
}

/*********************************************************************
 * @fn      RCC_MCOConfig
 *
 * @brief   Selects the clock source to output on MCO pin.
 *
 * @param   RCC_MCO - specifies the clock source to output.
 *            RCC_MCO_NoClock - No clock selected.
 *            RCC_MCO_SYSCLK - System clock selected.
 *            RCC_MCO_HSI - HSI oscillator clock selected.
 *            RCC_MCO_HSE - HSE oscillator clock selected.
 *            RCC_MCO_PLLCLK_Div2 - PLL clock divided by 2 selected.
 *            RCC_MCO_PLL2CLK - PLL2 clock selected
 *            RCC_MCO_PLL3CLK_Div2 - PLL3 clock divided by 2 selected
 *            RCC_MCO_XT1 - External 3-25 MHz oscillator clock selected
 *            RCC_MCO_PLL3CLK - PLL3 clock selected
 *
 * @return  none
 */
void RCC_MCOConfig(uint8_t RCC_MCO) {
}

/*********************************************************************
 * @fn      RCC_GetFlagStatus
 *
 * @brief   Checks whether the specified RCC flag is set or not.
 *
 * @param   RCC_FLAG - specifies the flag to check.
 *            RCC_FLAG_HSIRDY - HSI oscillator clock ready.
 *            RCC_FLAG_HSERDY - HSE oscillator clock ready.
 *            RCC_FLAG_PLLRDY - PLL clock ready.
 *            RCC_FLAG_PLL2RDY - PLL2 clock ready.
 *            RCC_FLAG_PLL3RDY - PLL3 clock ready.
 *            RCC_FLAG_LSERDY - LSE oscillator clock ready.
 *            RCC_FLAG_LSIRDY - LSI oscillator clock ready.
 *            RCC_FLAG_PINRST - Pin reset.
 *            RCC_FLAG_PORRST - POR/PDR reset.
 *            RCC_FLAG_SFTRST - Software reset.
 *            RCC_FLAG_IWDGRST - Independent Watchdog reset.
 *            RCC_FLAG_WWDGRST - Window Watchdog reset.
 *            RCC_FLAG_LPWRRST - Low Power reset.
 *
 * @return  FlagStatus - SET or RESET.
 */
FlagStatus RCC_GetFlagStatus(uint8_t RCC_FLAG) {
    return 0;
}

/*********************************************************************
 * @fn      RCC_ClearFlag
 *
 * @brief   Clears the RCC reset flags.
 *
 * @return  none
 */
void RCC_ClearFlag(void) {
}

/*********************************************************************
 * @fn      RCC_GetITStatus
 *
 * @brief   Checks whether the specified RCC interrupt has occurred or not.
 *
 * @param   RCC_IT - specifies the RCC interrupt source to check.
 *            RCC_IT_LSIRDY - LSI ready interrupt.
 *            RCC_IT_LSERDY - LSE ready interrupt.
 *            RCC_IT_HSIRDY - HSI ready interrupt.
 *            RCC_IT_HSERDY - HSE ready interrupt.
 *            RCC_IT_PLLRDY - PLL ready interrupt.
 *            RCC_IT_PLL2RDY - PLL2 ready interrupt.
 *            RCC_IT_PLL3RDY - PLL3 ready interrupt.
 *            RCC_IT_CSS - Clock Security System interrupt.
 *
 * @return  ITStatus - SET or RESET.
 */

ITStatus RCC_GetITStatus(uint8_t RCC_IT) {
    return 0;
}

/*********************************************************************
 * @fn      RCC_ClearITPendingBit
 *
 * @brief   Clears the RCC's interrupt pending bits.
 *
 * @param   RCC_IT - specifies the interrupt pending bit to clear.
 *            RCC_IT_LSIRDY - LSI ready interrupt.
 *            RCC_IT_LSERDY - LSE ready interrupt.
 *            RCC_IT_HSIRDY - HSI ready interrupt.
 *            RCC_IT_HSERDY - HSE ready interrupt.
 *            RCC_IT_PLLRDY - PLL ready interrupt.
 *            RCC_IT_PLL2RDY - PLL2 ready interrupt.
 *            RCC_IT_PLL3RDY - PLL3 ready interrupt.
 *            RCC_IT_CSS - Clock Security System interrupt.
 *
 * @return  none
 */
void RCC_ClearITPendingBit(uint8_t RCC_IT) {
}

/*********************************************************************
 * @fn      RCC_PREDIV1Config
 *
 * @brief   Configures the PREDIV1 division factor.
 *
 * @param   RCC_PREDIV1_Source - specifies the PREDIV1 clock source.
 *            RCC_PREDIV1_Source_HSE - HSE selected as PREDIV1 clock
 *            RCC_PREDIV1_Source_PLL2 - PLL2 selected as PREDIV1 clock
 *          RCC_PREDIV1_Div - specifies the PREDIV1 clock division factor.
 *            This parameter can be RCC_PREDIV1_Divx where x[1,16]
 *
 * @return  none
 */
void RCC_PREDIV1Config(uint32_t RCC_PREDIV1_Source, uint32_t RCC_PREDIV1_Div) {
}

/*********************************************************************
 * @fn      RCC_PREDIV2Config
 *
 * @brief   Configures the PREDIV2 division factor.
 *
 * @param   RCC_PREDIV2_Div - specifies the PREDIV2 clock division factor.
 *            This parameter can be RCC_PREDIV2_Divx where x:[1,16]
 *
 * @return  none
 */
void RCC_PREDIV2Config(uint32_t RCC_PREDIV2_Div) {
}

/*********************************************************************
 * @fn      RCC_PLL2Config
 *
 * @brief   Configures the PLL2 multiplication factor.
 *
 * @param   RCC_PLL2Mul - specifies the PLL2 multiplication factor.
 *            This parameter can be RCC_PLL2Mul_x where x:{[8,14], 16, 20}
 *
 * @return  none
 */
void RCC_PLL2Config(uint32_t RCC_PLL2Mul) {
}

/*********************************************************************
 * @fn      RCC_PLL2Cmd
 *
 * @brief   Enables or disables the PLL2.
 *
 * @param   NewState - new state of the PLL2. This parameter can be
 *        ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_PLL2Cmd(FunctionalState NewState) {
}

/*********************************************************************
 * @fn      RCC_PLL3Config
 *
 * @brief   Configures the PLL3 multiplication factor.
 *
 * @param   RCC_PLL3Mul - specifies the PLL2 multiplication factor.
 *            This parameter can be RCC_PLL2Mul_x where x:{[8,14], 16, 20}
 *
 * @return  none
 */
void RCC_PLL3Config(uint32_t RCC_PLL3Mul) {
}

/*********************************************************************
 * @fn      RCC_PLL3Cmd
 *
 * @brief   Enables or disables the PLL3.
 *
 * @param   NewState - new state of the PLL2. This parameter can be
 *        ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_PLL3Cmd(FunctionalState NewState) {
}

/*********************************************************************
 * @fn      RCC_OTGFSCLKConfig
 *
 * @brief   Configures the USB OTG FS clock (OTGFSCLK).
 *
 * @param   RCC_OTGFSCLKSource - specifies the USB OTG FS clock source.
 *            RCC_OTGFSCLKSource_PLLCLK_Div1 - PLL clock divided by 1
 *        selected as USB OTG FS clock source
 *            RCC_OTGFSCLKSource_PLLCLK_Div2 - PLL clock divided by 2
 *        selected as USB OTG FS clock source
 *            RCC_OTGFSCLKSource_PLLCLK_Div3 - PLL clock divided by 3
 *        selected as USB OTG FS clock source
 *
 * @return  none
 */
void RCC_OTGFSCLKConfig(uint32_t RCC_OTGFSCLKSource) {
}

/*********************************************************************
 * @fn      RCC_I2S2CLKConfig
 *
 * @brief   Configures the I2S2 clock source(I2S2CLK).
 *
 * @param   RCC_I2S2CLKSource - specifies the I2S2 clock source.
 *          RCC_I2S2CLKSource_SYSCLK - system clock selected as I2S2 clock entry
 *          RCC_I2S2CLKSource_PLL3_VCO - PLL3 VCO clock selected as I2S2 clock entry
 *
 * @return  none
 */
void RCC_I2S2CLKConfig(uint32_t RCC_I2S2CLKSource) {
}

/*********************************************************************
 * @fn      RCC_I2S3CLKConfig
 *
 * @brief   Configures the I2S3 clock source(I2S2CLK).
 *
 * @param   RCC_I2S3CLKSource - specifies the I2S3 clock source.
 *            RCC_I2S3CLKSource_SYSCLK - system clock selected as I2S3 clock entry
 *            RCC_I2S3CLKSource_PLL3_VCO - PLL3 VCO clock selected as I2S3 clock entry
 *
 * @return  none
 */
void RCC_I2S3CLKConfig(uint32_t RCC_I2S3CLKSource) {
}

/*********************************************************************
 * @fn      RCC_AHBPeriphResetCmd
 *
 * @brief   Forces or releases AHB peripheral reset.
 *
 * @param   RCC_AHBPeriph - specifies the AHB peripheral to reset.
 *            RCC_AHBPeriph_OTG_FS
 *            RCC_AHBPeriph_ETH_MAC
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_AHBPeriphResetCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState) {
}

/*********************************************************************
 * @fn      RCC_ADCCLKADJcmd
 *
 * @brief   Enable ADC clock duty cycle adjustment.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_ADCCLKADJcmd(FunctionalState NewState) {
}

/*********************************************************************
 * @fn      RCC_RNGCLKConfig
 *
 * @brief   Configures the RNG clock source.
 *
 * @param   RCC_RNGCLKSource - specifies the RNG clock source.
 *            RCC_RNGCLKSource_SYSCLK - system clock selected as RNG clock entry
 *            RCC_RNGCLKSource_PLL3_VCO - PLL3 VCO clock selected as RNG clock entry
 *
 * @return  none
 */
void RCC_RNGCLKConfig(uint32_t RCC_RNGCLKSource) {
}

/*********************************************************************
 * @fn      RCC_ETH1GCLKConfig
 *
 * @brief   Configures the ETH1G clock source.
 *
 * @param   RCC_RNGCLKSource - specifies the ETH1G clock source.
 *            RCC_ETH1GCLKSource_PLL2_VCO - system clock selected as ETH1G clock entry
 *            RCC_ETH1GCLKSource_PLL3_VCO - PLL3 VCO clock selected as ETH1G clock entry
 *            RCC_ETH1GCLKSource_PB1_IN -  GPIO PB1 input clock selected as ETH1G clock entry
 *
 * @return  none
 */
void RCC_ETH1GCLKConfig(uint32_t RCC_ETH1GCLKSource) {
}

/*********************************************************************
 * @fn      RCC_ETH1G_125Mcmd
 *
 * @brief   Enable ETH1G 125M.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_ETH1G_125Mcmd(FunctionalState NewState) {
}

/*********************************************************************
 * @fn      RCC_USBHSConfig
 *
 * @brief   Configures the USBHS clock.
 *
 * @param   RCC_USBHS - defines the USBHS clock divider.
 *            RCC_USBPLL_Div1 - USBHS clock = USBPLL.
 *            RCC_USBPLL_Div2 - USBHS clock = USBPLL/2.
 *            RCC_USBPLL_Div3 - USBHS clock = USBPLL/3.
 *            RCC_USBPLL_Div4 - USBHS clock = USBPLL/4.
 *            RCC_USBPLL_Div5 - USBHS clock = USBPLL/5.
 *            RCC_USBPLL_Div6 - USBHS clock = USBPLL/6.
 *            RCC_USBPLL_Div7 - USBHS clock = USBPLL/7.
 *            RCC_USBPLL_Div8 - USBHS clock = USBPLL/8.
 *
 * @return  none
 */
void RCC_USBHSConfig(uint32_t RCC_USBHS) {
}

/*********************************************************************
 * @fn      RCC_USBHSPLLCLKConfig
 *
 * @brief   Configures the USBHSPLL clock source.
 *
 * @param   RCC_HSBHSPLLCLKSource - specifies the USBHSPLL clock source.
 *            RCC_HSBHSPLLCLKSource_HSE - HSE clock selected as USBHSPLL clock entry
 *            RCC_HSBHSPLLCLKSource_HSI - HSI clock selected as USBHSPLL clock entry
 *
 * @return  none
 */
void RCC_USBHSPLLCLKConfig(uint32_t RCC_USBHSPLLCLKSource) {
}

/*********************************************************************
 * @fn      RCC_USBHSPLLCKREFCLKConfig
 *
 * @brief   Configures the USBHSPLL reference clock.
 *
 * @param   RCC_USBHSPLLCKREFCLKSource - Select reference clock.
 *            RCC_USBHSPLLCKREFCLK_3M - reference clock 3Mhz.
 *            RCC_USBHSPLLCKREFCLK_4M - reference clock 4Mhz.
 *            RCC_USBHSPLLCKREFCLK_8M - reference clock 8Mhz.
 *            RCC_USBHSPLLCKREFCLK_5M - reference clock 5Mhz.
 *
 * @return  none
 */
void RCC_USBHSPLLCKREFCLKConfig(uint32_t RCC_USBHSPLLCKREFCLKSource) {
}

/*********************************************************************
 * @fn      RCC_USBHSPHYPLLALIVEcmd
 *
 * @brief   Enable USBHS PHY control.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_USBHSPHYPLLALIVEcmd(FunctionalState NewState) {
}

/*********************************************************************
 * @fn      RCC_USBCLK48MConfig
 *
 * @brief   Configures the USB clock 48MHz source.
 *
 * @param   RCC_USBCLK48MSource - specifies the USB clock 48MHz source.
 *            RCC_USBCLK48MCLKSource_PLLCLK - PLLCLK clock selected as USB clock 48MHz clock entry
 *            RCC_USBCLK48MCLKSource_USBPHY - USBPHY clock selected as USB clock 48MHz clock entry
 *
 * @return  none
 */
void RCC_USBCLK48MConfig(uint32_t RCC_USBCLK48MSource) {
}
