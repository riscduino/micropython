/********************************** (C) COPYRIGHT *******************************
* Copyright (c) 2024  SiPlusPlus Semiconductor Private Limited
* SPDX-License-Identifier: Apache-2.0
*
* File Name              : spp32r20xx_usart.c
* Author                 : Dinesh Annayya
* Version                : V1.0.0
* Date                   : 20-June-2024
* Description            : This file provides all the USART firmware functions.
*******************************************************************************/
#include "spp32r20xx_usart.h"
#include "spp32r20xx_rcc.h"


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
     if(USARTx == UART0) { // UART-0
     GLBL_REG(GLBL_CFG0)        |= DRST_UART0;
     GLBL_REG(GLBL_MULTI_FUNC) &= ~MFUNC_MUART_ENB; // Disable Master UART
     GLBL_REG(GLBL_MULTI_FUNC) |= MFUNC_UART0_ENB;  // Enable Slave UART
  } else { // UART-1
     GLBL_REG(GLBL_CFG0)        |= DRST_UART1;
     GLBL_REG(GLBL_MULTI_FUNC)  |= MFUNC_UART1_ENB;
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

  if(USARTx == UART0) { // UART-0
     GLBL_REG(GLBL_CFG0)        |= DRST_UART0;
     GLBL_REG(GLBL_MULTI_FUNC) &= ~MFUNC_MUART_ENB; // Disable Master UART
     GLBL_REG(GLBL_MULTI_FUNC) |= MFUNC_UART0_ENB;  // Enable Slave UART
  } else { // UART-1
     GLBL_REG(GLBL_CFG0)        |= DRST_UART1;
     GLBL_REG(GLBL_MULTI_FUNC) |= MFUNC_UART1_ENB;
  }

  USART_Setbaud(USARTx,USART_InitStruct->USART_BaudRate);

  USARTx->CTRL |= UART_CTRL_TXEN(1);
  USARTx->CTRL |= UART_CTRL_RXEN(1);

  USARTx->CTRL &= UART_CTRL_IFGM;
  USARTx->CTRL |= UART_CTRL_IFG(2); // 2 bit IFG

  USARTx->CTRL |= UART_CTRL_TSTOP(USART_InitStruct->USART_StopBits); // 2 bit IFG
  USARTx->CTRL |= UART_CTRL_RSTOP(USART_InitStruct->USART_StopBits); // 2 bit IFG
  USARTx->CTRL |= UART_CTRL_PMOD(USART_InitStruct->USART_Parity); // 2 bit IFG

}

/****************************************************************
// 16x Baud clock generation
//  Baud Rate config = (F_CPU / (BAUD * 16)) - 2 
// Example: to generate 19200 Baud clock from 50Mhz Link clock
//    cfg_baud_16x = ((50 * 1000 * 1000) / (19200 * 16)) - 2
//    cfg_baud_16x = 0xA0 (160)
****************************************************************/

void USART_Setbaud(USART_TypeDef *USARTx,int bauds)
{

  uint32_t F_Baud; 
  F_Baud = (SystemCoreClock/(bauds * 16)) - 2;

  USARTx->BAUD_LSB = F_Baud & 0xFF;
  USARTx->BAUD_MSB = (F_Baud >> 8) & 0x0F;
 
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
    USART_InitStruct->USART_WordLength = 8;
    USART_InitStruct->USART_StopBits = UART_STOP_BIT_1;
    USART_InitStruct->USART_Parity = UART_PRI_MODE_NOP;
    USART_InitStruct->USART_Mode = UART_CTRL_TXEN(1) | UART_CTRL_RXEN(1);
    USART_InitStruct->USART_HardwareFlowControl = 0;
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
	if(NewState == ENABLE) {
             USARTx->CTRL |= UART_CTRL_TXEN(1);
             USARTx->CTRL |= UART_CTRL_RXEN(1);
	} else {
             USARTx->CTRL &= UART_CTRL_TXENM;
             USARTx->CTRL &= UART_CTRL_RXENM;
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
void USART_SendData(USART_TypeDef *USARTx, uint8_t Data) {

  while (USARTx->STAT & UART_TX_FIFO_FULL); // Wait if FIFO if full
  USARTx->TDATA = Data;

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
uint8_t USART_ReceiveData(USART_TypeDef *USARTx) {

   while (USARTx->STAT & UART_RX_FIFO_EMPTY); // Wait if FIFO if full
   return (uint8_t)(USARTx->RDATA);

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
	USART_SendData(USARTx,0x00);
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
    USARTx->CTRL &= UART_CTRL_IFGM;
    USARTx->CTRL |= UART_CTRL_IFG(USART_GuardTime);
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

    return bitstatus;
}

/*********************************************************************
 * @fn      USART_ClearFlag
 *
 * @brief   Clears the USARTx's interrupt pending flags.
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
    USARTx->ISTAT = (uint16_t) USART_FLAG;
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
    ITStatus bitstatus = RESET;


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

}
