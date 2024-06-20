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



//Uart Defination

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
