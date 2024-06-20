/********************************** (C) COPYRIGHT *******************************
* Copyright (c) 2024  BigEndian Semiconductor Private Limited
* SPDX-License-Identifier: Apache-2.0
*
* File Name              : bes32r20xx_usart.c
* Author                 : Dinesh Annayya
* Version                : V1.0.0
* Date                   : 20-June-2024
* Description            : This file provides all the USART firmware functions.
*******************************************************************************/
#include "bes32r20xx_usart.h"
#include "bes32r20xx_rcc.h"

/* USART_Private_Defines */
#define CTLR1_UE_Set              ((uint16_t)0x2000) /* USART Enable Mask */
#define CTLR1_UE_Reset            ((uint16_t)0xDFFF) /* USART Disable Mask */

#define CTLR1_WAKE_Mask           ((uint16_t)0xF7FF) /* USART WakeUp Method Mask */

#define CTLR1_RWU_Set             ((uint16_t)0x0002) /* USART mute mode Enable Mask */
#define CTLR1_RWU_Reset           ((uint16_t)0xFFFD) /* USART mute mode Enable Mask */
#define CTLR1_SBK_Set             ((uint16_t)0x0001) /* USART Break Character send Mask */
#define CTLR1_CLEAR_Mask          ((uint16_t)0xE9F3) /* USART CR1 Mask */
#define CTLR2_Address_Mask        ((uint16_t)0xFFF0) /* USART address Mask */

#define CTLR2_LINEN_Set           ((uint16_t)0x4000) /* USART LIN Enable Mask */
#define CTLR2_LINEN_Reset         ((uint16_t)0xBFFF) /* USART LIN Disable Mask */

#define CTLR2_LBDL_Mask           ((uint16_t)0xFFDF) /* USART LIN Break detection Mask */
#define CTLR2_STOP_CLEAR_Mask     ((uint16_t)0xCFFF) /* USART CR2 STOP Bits Mask */
#define CTLR2_CLOCK_CLEAR_Mask    ((uint16_t)0xF0FF) /* USART CR2 Clock Mask */

#define GPR_LSB_Mask              ((uint16_t)0x00FF) /* Guard Time Register LSB Mask */
#define GPR_MSB_Mask              ((uint16_t)0xFF00) /* Guard Time Register MSB Mask */
#define IT_Mask                   ((uint16_t)0x001F) /* USART Interrupt Mask */


/*********************************************************************
 * @fn      USART_DeInit
 *
 * @brief   Deinitializes the USARTx peripheral registers to their default
 *        reset values.
 *
 * @param   USARTx - where x can be 1, 2 or 3 to select the UART peripheral.
 *
 * @return  none
 */
void USART_DeInit(USART_TypeDef *USARTx) {
    if (USARTx == USART1) {
        RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, ENABLE);
        RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, DISABLE);
    } else if (USARTx == USART2) {
        RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2, ENABLE);
        RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2, DISABLE);
    }
}

/*********************************************************************
 * @fn      USART_Init
 *
 * @brief   Initializes the USARTx peripheral according to the specified
 *        parameters in the USART_InitStruct.
 *
 * @param   USARTx - where x can be 1, 2 or 3 to select the UART peripheral.
 *          USART_InitStruct - pointer to a USART_InitTypeDef structure
 *        that contains the configuration information for the specified
 *        USART peripheral.
 *
 * @return  none
 */
void USART_Init(USART_TypeDef *USARTx, USART_InitTypeDef *USART_InitStruct) {
    uint32_t tmpreg = 0x00, apbclock = 0x00;
    uint32_t integerdivider = 0x00;
    uint32_t fractionaldivider = 0x00;
    uint32_t usartxbase = 0;
    RCC_ClocksTypeDef RCC_ClocksStatus;

    if (USART_InitStruct->USART_HardwareFlowControl != USART_HardwareFlowControl_None) {
    }

    usartxbase = (uint32_t)USARTx;
    tmpreg = USARTx->CTLR2;
    tmpreg &= CTLR2_STOP_CLEAR_Mask;
    tmpreg |= (uint32_t)USART_InitStruct->USART_StopBits;

    USARTx->CTLR2 = (uint16_t)tmpreg;
    tmpreg = USARTx->CTLR1;
    tmpreg &= CTLR1_CLEAR_Mask;
    tmpreg |= (uint32_t)USART_InitStruct->USART_WordLength | USART_InitStruct->USART_Parity |
        USART_InitStruct->USART_Mode;
    USARTx->CTLR1 = (uint16_t)tmpreg;

    RCC_GetClocksFreq(&RCC_ClocksStatus);

    if (usartxbase == USART1_BASE) {
        apbclock = RCC_ClocksStatus.PCLK2_Frequency;
    } else {
        apbclock = RCC_ClocksStatus.PCLK1_Frequency;
    }

    tmpreg = (integerdivider / 100) << 4;

    fractionaldivider = integerdivider - (100 * (tmpreg >> 4));


    USARTx->BRR = (uint16_t)tmpreg;
}

/*********************************************************************
 * @fn      USART_StructInit
 *
 * @brief   Fills each USART_InitStruct member with its default value.
 *
 * @param   SPIx - where x can be 1, 2 or 3 to select the SPI peripheral.
 *
 * @return  none
 */
void USART_StructInit(USART_InitTypeDef *USART_InitStruct) {
    USART_InitStruct->USART_BaudRate = 9600;
    USART_InitStruct->USART_WordLength = USART_WordLength_8b;
    USART_InitStruct->USART_StopBits = USART_StopBits_1;
    USART_InitStruct->USART_Parity = USART_Parity_No;
    USART_InitStruct->USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct->USART_HardwareFlowControl = USART_HardwareFlowControl_None;
}

/*********************************************************************
 * @fn      USART_ClockInit
 *
 * @brief   Initializes the USARTx peripheral Clock according to the
 *        specified parameters in the USART_ClockInitStruct .
 *
 * @param   USARTx - where x can be 1, 2, 3 to select the USART peripheral.
 *          USART_ClockInitStruct - pointer to a USART_ClockInitTypeDef
 *        structure that contains the configuration information for the specified
 *        USART peripheral.
 *
 * @return  none
 */
void USART_ClockInit(USART_TypeDef *USARTx, USART_ClockInitTypeDef *USART_ClockInitStruct) {
    uint32_t tmpreg = 0x00;

    tmpreg = USARTx->CTLR2;
    tmpreg &= CTLR2_CLOCK_CLEAR_Mask;
    tmpreg |= (uint32_t)USART_ClockInitStruct->USART_Clock | USART_ClockInitStruct->USART_CPOL |
        USART_ClockInitStruct->USART_CPHA | USART_ClockInitStruct->USART_LastBit;
    USARTx->CTLR2 = (uint16_t)tmpreg;
}

/*********************************************************************
 * @fn      USART_ClockStructInit
 *
 * @brief   Fills each USART_ClockStructInit member with its default value.
 *
 * @param   USART_ClockInitStruct - pointer to a USART_ClockInitTypeDef
 *        structure which will be initialized.
 *
 * @return  none
 */
void USART_ClockStructInit(USART_ClockInitTypeDef *USART_ClockInitStruct) {
    USART_ClockInitStruct->USART_Clock = USART_Clock_Disable;
    USART_ClockInitStruct->USART_CPOL = USART_CPOL_Low;
    USART_ClockInitStruct->USART_CPHA = USART_CPHA_1Edge;
    USART_ClockInitStruct->USART_LastBit = USART_LastBit_Disable;
}

/*********************************************************************
 * @fn      USART_Cmd
 *
 * @brief   Enables or disables the specified USART peripheral.
 *        reset values (Affects also the I2Ss).
 *
 * @param   USARTx - where x can be 1, 2, 3 to select the USART peripheral.
 *          NewState: ENABLE or DISABLE.
 *
 * @return  none
 */
void USART_Cmd(USART_TypeDef *USARTx, FunctionalState NewState) {
    if (NewState != DISABLE) {
        USARTx->CTLR1 |= CTLR1_UE_Set;
    } else {
        USARTx->CTLR1 &= CTLR1_UE_Reset;
    }
}

/*********************************************************************
 * @fn      USART_ITConfig
 *
 * @brief   Enables or disables the specified USART interrupts.
 *        reset values (Affects also the I2Ss).
 *
 * @param   USARTx - where x can be 1, 2, 3 to select the USART peripheral.
 *          USART_IT - specifies the USART interrupt sources to be enabled or disabled.
 *            USART_IT_CTS - CTS change interrupt.
 *            USART_IT_LBD - LIN Break detection interrupt.
 *            USART_IT_TXE - Transmit Data Register empty interrupt.
 *            USART_IT_TC - Transmission complete interrupt.
 *            USART_IT_RXNE - Receive Data register not empty interrupt.
 *            USART_IT_IDLE - Idle line detection interrupt.
 *            USART_IT_PE - Parity Error interrupt.
 *            USART_IT_ERR - Error interrupt.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void USART_ITConfig(USART_TypeDef *USARTx, uint16_t USART_IT, FunctionalState NewState) {
    uint32_t usartreg = 0x00, itpos = 0x00, itmask = 0x00;
    uint32_t usartxbase = 0x00;

    if (USART_IT == USART_IT_CTS) {
    }

    usartxbase = (uint32_t)USARTx;
    usartreg = (((uint8_t)USART_IT) >> 0x05);
    itpos = USART_IT & IT_Mask;
    itmask = (((uint32_t)0x01) << itpos);

    if (usartreg == 0x01) {
        usartxbase += 0x0C;
    } else if (usartreg == 0x02) {
        usartxbase += 0x10;
    } else {
        usartxbase += 0x14;
    }

    if (NewState != DISABLE) {
        *(__IO uint32_t *)usartxbase |= itmask;
    } else {
        *(__IO uint32_t *)usartxbase &= ~itmask;
    }
}


/*********************************************************************
 * @fn      USART_SendData
 *
 * @brief   Transmits single data through the USARTx peripheral.
 *
 * @param   USARTx - where x can be 1, 2, 3 to select the USART peripheral.
 *          Data - the data to transmit.
 *
 * @return  none
 */
void USART_SendData(USART_TypeDef *USARTx, uint16_t Data) {
    USARTx->DATAR = (Data & (uint16_t)0x01FF);
}

/*********************************************************************
 * @fn      USART_ReceiveData
 *
 * @brief   Returns the most recent received data by the USARTx peripheral.
 *
 * @param   USARTx - where x can be 1, 2, 3 to select the USART peripheral.
 *
 * @return  The received data.
 */
uint16_t USART_ReceiveData(USART_TypeDef *USARTx) {
    return (uint16_t)(USARTx->DATAR & (uint16_t)0x01FF);
}

/*********************************************************************
 * @fn      USART_SendBreak
 *
 * @brief   Transmits break characters.
 *
 * @param   USARTx - where x can be 1, 2, 3 to select the USART peripheral.
 *
 * @return  none
 */
void USART_SendBreak(USART_TypeDef *USARTx) {
    USARTx->CTLR1 |= CTLR1_SBK_Set;
}

/*********************************************************************
 * @fn      USART_SetGuardTime
 *
 * @brief   Sets the specified USART guard time.
 *
 * @param   USARTx - where x can be 1, 2, 3 to select the USART peripheral.
 *          USART_GuardTime - specifies the guard time.
 *
 * @return  none
 */
void USART_SetGuardTime(USART_TypeDef *USARTx, uint8_t USART_GuardTime) {
    USARTx->GPR &= GPR_LSB_Mask;
    USARTx->GPR |= (uint16_t)((uint16_t)USART_GuardTime << 0x08);
}

/*********************************************************************
 * @fn      USART_SetPrescaler
 *
 * @brief   Sets the system clock prescaler.
 *
 * @param   USARTx - where x can be 1, 2, 3 to select the USART peripheral.
 *          USART_Prescaler - specifies the prescaler clock.
 *
 * @return  none
 */
void USART_SetPrescaler(USART_TypeDef *USARTx, uint8_t USART_Prescaler) {
    USARTx->GPR &= GPR_MSB_Mask;
    USARTx->GPR |= USART_Prescaler;
}

/*********************************************************************
 * @fn      USART_GetFlagStatus
 *
 * @brief   Checks whether the specified USART flag is set or not.
 *
 * @param   USARTx - where x can be 1, 2, 3 to select the USART peripheral.
 *          USART_FLAG - specifies the flag to check.
 *            USART_FLAG_CTS - CTS Change flag.
 *            USART_FLAG_LBD - LIN Break detection flag.
 *            USART_FLAG_TXE - Transmit data register empty flag.
 *            USART_FLAG_TC - Transmission Complete flag.
 *            USART_FLAG_RXNE - Receive data register not empty flag.
 *            USART_FLAG_IDLE - Idle Line detection flag.
 *            USART_FLAG_ORE - OverRun Error flag.
 *            USART_FLAG_NE - Noise Error flag.
 *            USART_FLAG_FE - Framing Error flag.
 *            USART_FLAG_PE - Parity Error flag.
 *
 * @return  none
 */
FlagStatus USART_GetFlagStatus(USART_TypeDef *USARTx, uint16_t USART_FLAG) {
    FlagStatus bitstatus = RESET;

    if (USART_FLAG == USART_FLAG_CTS) {
    }

    if ((USARTx->STATR & USART_FLAG) != (uint16_t)RESET) {
        bitstatus = SET;
    } else {
        bitstatus = RESET;
    }
    return bitstatus;
}

/*********************************************************************
 * @fn      USART_ClearFlag
 *
 * @brief   Clears the USARTx's pending flags.
 *
 * @param   USARTx - where x can be 1, 2, 3 to select the USART peripheral.
 *          USART_FLAG - specifies the flag to clear.
 *            USART_FLAG_CTS - CTS Change flag.
 *            USART_FLAG_LBD - LIN Break detection flag.
 *            USART_FLAG_TC - Transmission Complete flag.
 *            USART_FLAG_RXNE - Receive data register not empty flag.
 *
 * @return  none
 */
void USART_ClearFlag(USART_TypeDef *USARTx, uint16_t USART_FLAG) {
    if ((USART_FLAG & USART_FLAG_CTS) == USART_FLAG_CTS) {
    }

    USARTx->STATR = (uint16_t) ~USART_FLAG;
}

/*********************************************************************
 * @fn      USART_GetITStatus
 *
 * @brief   Checks whether the specified USART interrupt has occurred or not.
 *
 * @param   USARTx - where x can be 1, 2, 3 to select the USART peripheral.
 *          USART_IT - specifies the USART interrupt source to check.
 *            USART_IT_CTS - CTS change interrupt.
 *            USART_IT_LBD - LIN Break detection interrupt.
 *            USART_IT_TXE - Tansmit Data Register empty interrupt.
 *            USART_IT_TC - Transmission complete interrupt.
 *            USART_IT_RXNE - Receive Data register not empty interrupt.
 *            USART_IT_IDLE - Idle line detection interrupt.
 *            USART_IT_ORE_RX - OverRun Error interrupt if the RXNEIE bit is set.
 *            USART_IT_ORE_ER - OverRun Error interrupt if the EIE bit is set.
 *            USART_IT_NE - Noise Error interrupt.
 *            USART_IT_FE - Framing Error interrupt.
 *            USART_IT_PE - Parity Error interrupt.
 *
 * @return  none
 */
ITStatus USART_GetITStatus(USART_TypeDef *USARTx, uint16_t USART_IT) {
    uint32_t bitpos = 0x00, itmask = 0x00, usartreg = 0x00;
    ITStatus bitstatus = RESET;

    if (USART_IT == USART_IT_CTS) {
    }

    usartreg = (((uint8_t)USART_IT) >> 0x05);
    itmask = USART_IT & IT_Mask;
    itmask = (uint32_t)0x01 << itmask;

    if (usartreg == 0x01) {
        itmask &= USARTx->CTLR1;
    } else if (usartreg == 0x02) {
        itmask &= USARTx->CTLR2;
    } 

    bitpos = USART_IT >> 0x08;
    bitpos = (uint32_t)0x01 << bitpos;
    bitpos &= USARTx->STATR;

    if ((itmask != (uint16_t)RESET) && (bitpos != (uint16_t)RESET)) {
        bitstatus = SET;
    } else {
        bitstatus = RESET;
    }

    return bitstatus;
}

/*********************************************************************
 * @fn      USART_ClearITPendingBit
 *
 * @brief   Clears the USARTx's interrupt pending bits.
 *
 * @param   USARTx - where x can be 1, 2, 3 to select the USART peripheral.
 *          USART_IT - specifies the interrupt pending bit to clear.
 *            USART_IT_CTS - CTS change interrupt.
 *            USART_IT_LBD - LIN Break detection interrupt.
 *            USART_IT_TC - Transmission complete interrupt.
 *            USART_IT_RXNE - Receive Data register not empty interrupt.
 *
 * @return  none
 */
void USART_ClearITPendingBit(USART_TypeDef *USARTx, uint16_t USART_IT) {
    uint16_t bitpos = 0x00, itmask = 0x00;

    if (USART_IT == USART_IT_CTS) {
    }

    bitpos = USART_IT >> 0x08;
    itmask = ((uint16_t)0x01 << (uint16_t)bitpos);
    USARTx->STATR = (uint16_t) ~itmask;
}
