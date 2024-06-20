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

#ifndef __BES32R20XX_USART_H
#define __BES32R20XX_USART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bes32r20xx.h"


/* USART Init Structure definition */
typedef struct
{
    uint32_t USART_BaudRate;          /* This member configures the USART communication baud rate.
                                         The baud rate is computed using the following formula:
                                          - IntegerDivider = ((PCLKx) / (16 * (USART_InitStruct->USART_BaudRate)))
                                          - FractionalDivider = ((IntegerDivider - ((u32) IntegerDivider)) * 16) + 0.5 */

    uint16_t USART_WordLength;        /* Specifies the number of data bits transmitted or received in a frame.
                                         This parameter can be a value of @ref USART_Word_Length */

    uint16_t USART_StopBits;          /* Specifies the number of stop bits transmitted.
                                         This parameter can be a value of @ref USART_Stop_Bits */

    uint16_t USART_Parity;            /* Specifies the parity mode.
                                         This parameter can be a value of @ref USART_Parity
                                         @note When parity is enabled, the computed parity is inserted
                                               at the MSB position of the transmitted data (9th bit when
                                               the word length is set to 9 data bits; 8th bit when the
                                               word length is set to 8 data bits). */

    uint16_t USART_Mode;              /* Specifies wether the Receive or Transmit mode is enabled or disabled.
                                         This parameter can be a value of @ref USART_Mode */

    uint16_t USART_HardwareFlowControl; /* Specifies wether the hardware flow control mode is enabled
                                         or disabled.
                                         This parameter can be a value of @ref USART_Hardware_Flow_Control */
} USART_InitTypeDef;

/* USART Clock Init Structure definition */
typedef struct
{

    uint16_t USART_Clock; /* Specifies whether the USART clock is enabled or disabled.
                             This parameter can be a value of @ref USART_Clock */

    uint16_t USART_CPOL;  /* Specifies the steady state value of the serial clock.
                             This parameter can be a value of @ref USART_Clock_Polarity */

    uint16_t USART_CPHA;  /* Specifies the clock transition on which the bit capture is made.
                             This parameter can be a value of @ref USART_Clock_Phase */

    uint16_t USART_LastBit; /* Specifies whether the clock pulse corresponding to the last transmitted
                             data bit (MSB) has to be output on the SCLK pin in synchronous mode.
                             This parameter can be a value of @ref USART_Last_Bit */
} USART_ClockInitTypeDef;

/* USART_Word_Length */
#define USART_WordLength_8b                  ((uint16_t)0x0000)
#define USART_WordLength_9b                  ((uint16_t)0x1000)

/* USART_Stop_Bits */
#define USART_StopBits_1                     ((uint16_t)0x0000)
#define USART_StopBits_0_5                   ((uint16_t)0x1000)
#define USART_StopBits_2                     ((uint16_t)0x2000)
#define USART_StopBits_1_5                   ((uint16_t)0x3000)

/* USART_Parity */
#define USART_Parity_No                      ((uint16_t)0x0000)
#define USART_Parity_Even                    ((uint16_t)0x0400)
#define USART_Parity_Odd                     ((uint16_t)0x0600)

/* USART_Mode */
#define USART_Mode_Rx                        ((uint16_t)0x0004)
#define USART_Mode_Tx                        ((uint16_t)0x0008)

/* USART_Hardware_Flow_Control */
#define USART_HardwareFlowControl_None       ((uint16_t)0x0000)
#define USART_HardwareFlowControl_RTS        ((uint16_t)0x0100)
#define USART_HardwareFlowControl_CTS        ((uint16_t)0x0200)
#define USART_HardwareFlowControl_RTS_CTS    ((uint16_t)0x0300)

/* USART_Clock */
#define USART_Clock_Disable                  ((uint16_t)0x0000)
#define USART_Clock_Enable                   ((uint16_t)0x0800)

/* USART_Clock_Polarity */
#define USART_CPOL_Low                       ((uint16_t)0x0000)
#define USART_CPOL_High                      ((uint16_t)0x0400)

/* USART_Clock_Phase */
#define USART_CPHA_1Edge                     ((uint16_t)0x0000)
#define USART_CPHA_2Edge                     ((uint16_t)0x0200)

/* USART_Last_Bit */
#define USART_LastBit_Disable                ((uint16_t)0x0000)
#define USART_LastBit_Enable                 ((uint16_t)0x0100)

/* USART_Interrupt_definition */
#define USART_IT_PE                          ((uint16_t)0x0028)
#define USART_IT_TXE                         ((uint16_t)0x0727)
#define USART_IT_TC                          ((uint16_t)0x0626)
#define USART_IT_RXNE                        ((uint16_t)0x0525)
#define USART_IT_ORE_RX                      ((uint16_t)0x0325)
#define USART_IT_IDLE                        ((uint16_t)0x0424)
#define USART_IT_LBD                         ((uint16_t)0x0846)
#define USART_IT_CTS                         ((uint16_t)0x096A)
#define USART_IT_ERR                         ((uint16_t)0x0060)
#define USART_IT_ORE_ER                      ((uint16_t)0x0360)
#define USART_IT_NE                          ((uint16_t)0x0260)
#define USART_IT_FE                          ((uint16_t)0x0160)

#define USART_IT_ORE                          USART_IT_ORE_ER

/* USART_DMA_Requests */
#define USART_DMAReq_Tx                      ((uint16_t)0x0080)
#define USART_DMAReq_Rx                      ((uint16_t)0x0040)

/* USART_WakeUp_methods */
#define USART_WakeUp_IdleLine                ((uint16_t)0x0000)
#define USART_WakeUp_AddressMark             ((uint16_t)0x0800)

/* USART_LIN_Break_Detection_Length */
#define USART_LINBreakDetectLength_10b       ((uint16_t)0x0000)
#define USART_LINBreakDetectLength_11b       ((uint16_t)0x0020)

/* USART_IrDA_Low_Power */
#define USART_IrDAMode_LowPower              ((uint16_t)0x0004)
#define USART_IrDAMode_Normal                ((uint16_t)0x0000)

/* USART_Flags */
#define USART_FLAG_CTS                       ((uint16_t)0x0200)
#define USART_FLAG_LBD                       ((uint16_t)0x0100)
#define USART_FLAG_TXE                       ((uint16_t)0x0080)
#define USART_FLAG_TC                        ((uint16_t)0x0040)
#define USART_FLAG_RXNE                      ((uint16_t)0x0020)
#define USART_FLAG_IDLE                      ((uint16_t)0x0010)
#define USART_FLAG_ORE                       ((uint16_t)0x0008)
#define USART_FLAG_NE                        ((uint16_t)0x0004)
#define USART_FLAG_FE                        ((uint16_t)0x0002)
#define USART_FLAG_PE                        ((uint16_t)0x0001)

//--------- Added by Dinesh

/* USART_Private_Defines */
#define UART_CTRL_TXEN(x)   ((x & 0x1) << 0)    // [0]
#define UART_CTRL_RXEN(x)   ((x & 0x1) << 1)    // [1]
#define UART_CTRL_TSTOP(x)  ((x & 0x1) << 2)    // [2]
#define UART_CTRL_RSTOP(x)  ((x & 0x1) << 3)    // [3]
#define UART_CTRL_PMOD(x)   ((x & 0x3) << 4)    // [5:4]
#define UART_CTRL_IFG(x)    ((x & 0x3) << 6)    // [7:6]
						

#define UART_CTRL_TXENM     ~(0x1 << 0)    // [0]
#define UART_CTRL_RXENM     ~(0x1 << 1)    // [1]
#define UART_CTRL_TSTOPM    ~(0x1 << 2)    // [2]
#define UART_CTRL_RSTOPM    ~(0x1 << 3)    // [3]
#define UART_CTRL_PMODM     ~(0x3 << 4)    // [5:4]
#define UART_CTRL_IFGM      ~(0x3 << 6)    // [7:6]


#define UART_STOP_BIT_1     0x00
#define UART_STOP_BIT_2     0x02
						 
#define UART_PRI_MODE_NOP   0x00
#define UART_PRI_MODE_EVEN  0x01
#define UART_PRI_MODE_ODD   0x10

#define UART_RX_FIFO_EMPTY  0x02
#define UART_TX_FIFO_FULL   0x01

void USART_DeInit(USART_TypeDef *USARTx);
void USART_Init(USART_TypeDef *USARTx, USART_InitTypeDef *USART_InitStruct);
void USART_StructInit(USART_InitTypeDef *USART_InitStruct);
void USART_ClockInit(USART_TypeDef *USARTx, USART_ClockInitTypeDef *USART_ClockInitStruct);
void USART_ClockStructInit(USART_ClockInitTypeDef *USART_ClockInitStruct);
void USART_Cmd(USART_TypeDef *USARTx, FunctionalState NewState);
void USART_ITConfig(USART_TypeDef *USARTx, uint16_t USART_IT, FunctionalState NewState);
void USART_SendData(USART_TypeDef *USARTx, uint8_t Data);
uint8_t USART_ReceiveData(USART_TypeDef *USARTx);
void USART_SendBreak(USART_TypeDef *USARTx);
void USART_SetGuardTime(USART_TypeDef *USARTx, uint8_t USART_GuardTime);
void USART_SetPrescaler(USART_TypeDef *USARTx, uint8_t USART_Prescaler);
FlagStatus USART_GetFlagStatus(USART_TypeDef *USARTx, uint16_t USART_FLAG);
void USART_ClearFlag(USART_TypeDef *USARTx, uint16_t USART_FLAG);
ITStatus USART_GetITStatus(USART_TypeDef *USARTx, uint16_t USART_IT);
void USART_ClearITPendingBit(USART_TypeDef *USARTx, uint16_t USART_IT);

void USART_Setbaud(USART_TypeDef *USARTx,int bauds);

#ifdef __cplusplus
}
#endif

#endif
