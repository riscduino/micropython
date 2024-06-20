/********************************** (C) COPYRIGHT  *******************************
* Copyright (c) 2024  BigEndian Semiconductor Private Limited
* SPDX-License-Identifier: Apache-2.0
*
* File Name          : debug.h
* Author             : Dinesh Annayya
* Version            : V1.0.0
* Date               : 20-June-2024
* Description        : This file contains all the functions prototypes for UART
*                      Printf , Delay functions.
*******************************************************************************/
#ifndef __DEBUG_H
#define __DEBUG_H

#include "stdio.h"
#include "bes32r20xx.h"

/* UART Printf Definition */
#define DEBUG_UART1    1
#define DEBUG_UART2    2

/* DEBUG UATR Definition */
#define DEBUG   DEBUG_UART1
// #define DEBUG   DEBUG_UART2
// #define DEBUG   DEBUG_UART3

void Delay_Init(void);
void Delay_Us(uint32_t n);
void Delay_Ms(uint32_t n);
void USART_Printf_Init(uint32_t baudrate);

#endif
