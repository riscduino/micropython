/********************************** (C) COPYRIGHT  *******************************
* Copyright (c) 2024  BigEndian Semiconductor Private Limited
* SPDX-License-Identifier: Apache-2.0
*
* File Name          : debug.c
* Author             : Dinesh Annayya
* Version            : V1.0.0
* Date               : 20-June-2024
* Description        : This file contains all the functions prototypes for UART
*                      Printf , Delay functions.
*******************************************************************************/
#include "debug.h"

static uint8_t p_us = 0;
static uint16_t p_ms = 0;

/*********************************************************************
 * @fn      Delay_Init
 *
 * @brief   Initializes Delay Funcation.
 *
 * @return  none
 */
void Delay_Init(void) {
    p_us = SystemCoreClock / 1000000;
    p_ms = (uint16_t)p_us * 1000;
}

#define rdmcycle(x)  {				       \
    uint32_t lo, hi, hi2;			       \
    __asm__ __volatile__ ("1:\n\t"		       \
			  "csrr %0, mcycleh\n\t"       \
			  "csrr %1, mcycle\n\t"	       \
			  "csrr %2, mcycleh\n\t"       \
			  "bne  %0, %2, 1b\n\t"			\
			  : "=r" (hi), "=r" (lo), "=r" (hi2)) ;	\
    *(x) = lo | ((uint64_t) hi << 32); 				\
  }


/**
 * \brief Pauses the program for the amount of time (in microseconds) specified as parameter.
 *
 * \param dwUs the number of microseconds to pause (uint32_t)
 */
/*********************************************************************
 * @fn      Delay_Us
 *
 * @brief   Microsecond Delay Time.
 *
 * @param   n - Microsecond number.
 *
 * @return  None
 */

void Delay_Us(uint32_t usec) {
  if (usec == 0) {
    return;
  }
  // TODO: Short delays at low frequencies.
  uint64_t current, later;
  rdmcycle(&current);
  later = current + usec * (SystemCoreClock/1000000);
  if (later > current) // usual case
    {
      while (later > current) {
	rdmcycle(&current);
      }
    }
  else // wrap. Though this is unlikely to be hit w/ 64-bit mcycle
    {
      while (later < current) {
	rdmcycle(&current);
      }
      while (current < later) {
	rdmcycle(&current);
      }
    }
}



/*********************************************************************
 * @fn      Delay_Ms
 *
 * @brief   Millisecond Delay Time.
 *
 * @param   n - Millisecond number.
 *
 * @return  None
 */
void Delay_Ms(uint32_t msec) {

  if (msec == 0) {
    return;
  }
  // TODO: Short delays at low frequencies.
  uint64_t current, later;
  rdmcycle(&current);
  later = current + msec * (SystemCoreClock/1000);
  if (later > current) // usual case
    {
      while (later > current) {
	rdmcycle(&current);
      }
    }
  else // wrap. Though this is unlikely to be hit w/ 64-bit mcycle
    {
      while (later < current) {
	rdmcycle(&current);
      }
      while (current < later) {
	rdmcycle(&current);
      }
    }
}

/*********************************************************************
 * @fn      USART_Printf_Init
 *
 * @brief   Initializes the USARTx peripheral.
 *
 * @param   baudrate - USART communication baud rate.
 *
 * @return  None
 */
void USART_Printf_Init(uint32_t baudrate) {
    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = 0;
    USART_InitStructure.USART_StopBits = UART_STOP_BIT_1;
    USART_InitStructure.USART_Parity = UART_PRI_MODE_NOP;
    USART_InitStructure.USART_HardwareFlowControl = 0;
    USART_InitStructure.USART_Mode = UART_CTRL_TXEN(1) | UART_CTRL_RXEN(1);


    #if (DEBUG == DEBUG_UART1)
    USART_Init(UART0, &USART_InitStructure);
    USART_Cmd(UART0, ENABLE);

    #elif (DEBUG == DEBUG_UART2)
    USART_Init(UART1, &USART_InitStructure);
    USART_Cmd(UART1, ENABLE);

    #endif
}

/*********************************************************************
 * @fn      _write
 *
 * @brief   Support Printf Function
 *
 * @param   *buf - UART send Data.
 *          size - Data length
 *
 * @return  size: Data length
 */
__attribute__((used)) int _write(int fd, char *buf, int size) {
    int i;

    for (i = 0; i < size; i++)
    {
        #if (DEBUG == DEBUG_UART1)
        USART_SendData(UART0, *buf++);
        #elif (DEBUG == DEBUG_UART2)
        USART_SendData(UART1, *buf++);
        #endif
    }

    return size;
}

/*********************************************************************
 * @fn      _sbrk
 *
 * @brief   Change the spatial position of data segment.
 *
 * @return  size: Data length
 */
void *_sbrk(ptrdiff_t incr) {
    extern char _end[];
    extern char _heap_end[];
    static char *curbrk = _end;

    if ((curbrk + incr < _end) || (curbrk + incr > _heap_end)) {
        return NULL - 1;
    }

    curbrk += incr;
    return curbrk - incr;
}
