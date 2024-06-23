/********************************** (C) COPYRIGHT  *******************************
* Copyright (c) 2024  SiPlusPlus Semiconductor Private Limited
* SPDX-License-Identifier: Apache-2.0
*
* File Name          : spp32r20xx_i2c.c
* Author             : Dinesh Annayya
* Version            : V1.0.0
* Date               : 20-June-2024
* Description        : This file provides all the I2C firmware functions.
*******************************************************************************/
#include "spp32r20xx_i2c.h"
#include "spp32r20xx_rcc.h"


/*********************************************************************
 * @fn      I2C_DeInit
 *
 * @brief   Deinitializes the I2Cx peripheral registers to their default
 *        reset values.
 *
 * @param   I2Cx - where x can be 1 or 2 to select the I2C peripheral.
 *
 * @return  none
 */
void I2C_DeInit(I2C_TypeDef *I2Cx) {
}

/*********************************************************************
 * @fn      I2C_Init
 *
 * @brief   Initializes the I2Cx peripheral according to the specified
 *        parameters in the I2C_InitStruct.
 * @param   I2Cx - where x can be 1 or 2 to select the I2C peripheral.
 *          I2C_InitStruct - pointer to a I2C_InitTypeDef structure that
 *        contains the configuration information for the specified I2C peripheral.
 *
 * @return  none
 */
void I2C_Init(I2C_TypeDef *I2Cx, I2C_InitTypeDef *I2C_InitStruct) {
}

/*********************************************************************
 * @fn      I2C_StructInit
 *
 * @brief   Fills each I2C_InitStruct member with its default value.
 *
 * @param   I2C_InitStruct - pointer to an I2C_InitTypeDef structure which
 *        will be initialized.
 *
 * @return  none
 */
void I2C_StructInit(I2C_InitTypeDef *I2C_InitStruct) {
    I2C_InitStruct->I2C_ClockSpeed = 5000;
    I2C_InitStruct->I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct->I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStruct->I2C_OwnAddress1 = 0;
    I2C_InitStruct->I2C_Ack = I2C_Ack_Disable;
    I2C_InitStruct->I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
}

/*********************************************************************
 * @fn      I2C_Cmd
 *
 * @brief   Enables or disables the specified I2C peripheral.
 *
 * @param   I2Cx - where x can be 1 or 2 to select the I2C peripheral.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void I2C_Cmd(I2C_TypeDef *I2Cx, FunctionalState NewState) {
}

/*********************************************************************
 * @fn      I2C_DMACmd
 *
 * @brief   Enables or disables the specified I2C DMA requests.
 *
 * @param   I2Cx - where x can be 1 or 2 to select the I2C peripheral.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void I2C_DMACmd(I2C_TypeDef *I2Cx, FunctionalState NewState) {
}

/*********************************************************************
 * @fn      I2C_DMALastTransferCmd
 *
 * @brief   Specifies if the next DMA transfer will be the last one.
 *
 * @param   I2Cx - where x can be 1 or 2 to select the I2C peripheral.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void I2C_DMALastTransferCmd(I2C_TypeDef *I2Cx, FunctionalState NewState) {
}

/*********************************************************************
 * @fn      I2C_GenerateSTART
 *
 * @brief   Generates I2Cx communication START condition.
 *
 * @param   I2Cx - where x can be 1 or 2 to select the I2C peripheral.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void I2C_GenerateSTART(I2C_TypeDef *I2Cx, FunctionalState NewState) {
}

/*********************************************************************
 * @fn      I2C_GenerateSTOP
 *
 * @brief   Generates I2Cx communication STOP condition.
 *
 * @param   I2Cx - where x can be 1 or 2 to select the I2C peripheral.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void I2C_GenerateSTOP(I2C_TypeDef *I2Cx, FunctionalState NewState) {
}

/*********************************************************************
 * @fn      I2C_AcknowledgeConfig
 *
 * @brief   Enables or disables the specified I2C acknowledge feature.
 *
 * @param   I2Cx - where x can be 1 or 2 to select the I2C peripheral.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void I2C_AcknowledgeConfig(I2C_TypeDef *I2Cx, FunctionalState NewState) {
}

/*********************************************************************
 * @fn      I2C_OwnAddress2Config
 *
 * @brief   Configures the specified I2C own address2.
 *
 * @param   I2Cx - where x can be 1 or 2 to select the I2C peripheral.
 *          Address - specifies the 7bit I2C own address2.
 *
 * @return  none
 */
void I2C_OwnAddress2Config(I2C_TypeDef *I2Cx, uint8_t Address) {
}

/*********************************************************************
 * @fn      I2C_DualAddressCmd
 *
 * @brief   Enables or disables the specified I2C dual addressing mode.
 *
 * @param   I2Cx - where x can be 1 or 2 to select the I2C peripheral.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void I2C_DualAddressCmd(I2C_TypeDef *I2Cx, FunctionalState NewState) {
}

/*********************************************************************
 * @fn      I2C_GeneralCallCmd
 *
 * @brief   Enables or disables the specified I2C general call feature.
 *
 * @param   I2Cx - where x can be 1 or 2 to select the I2C peripheral.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void I2C_GeneralCallCmd(I2C_TypeDef *I2Cx, FunctionalState NewState) {
}

/*********************************************************************
 * @fn      I2C_ITConfig
 *
 * @brief   Enables or disables the specified I2C interrupts.
 *
 * @param   I2Cx - where x can be 1 or 2 to select the I2C peripheral.
 *          I2C_IT - specifies the I2C interrupts sources to be enabled or disabled.
 *            I2C_IT_BUF - Buffer interrupt mask.
 *            I2C_IT_EVT - Event interrupt mask.
 *            I2C_IT_ERR - Error interrupt mask.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void I2C_ITConfig(I2C_TypeDef *I2Cx, uint16_t I2C_IT, FunctionalState NewState) {
}

/*********************************************************************
 * @fn      I2C_SendData
 *
 * @brief   Sends a data byte through the I2Cx peripheral.
 *
 * @param   I2Cx - where x can be 1 or 2 to select the I2C peripheral.
 *          Data - Byte to be transmitted.
 *
 * @return  none
 */
void I2C_SendData(I2C_TypeDef *I2Cx, uint8_t Data) {
}

/*********************************************************************
 * @fn      I2C_ReceiveData
 *
 * @brief   Returns the most recent received data by the I2Cx peripheral.
 *
 * @param   I2Cx - where x can be 1 or 2 to select the I2C peripheral.
 *
 * @return  The value of the received data.
 */
uint8_t I2C_ReceiveData(I2C_TypeDef *I2Cx) {
    return 0;
}

/*********************************************************************
 * @fn      I2C_Send7bitAddress
 *
 * @brief   Transmits the address byte to select the slave device.
 *
 * @param   I2Cx - where x can be 1 or 2 to select the I2C peripheral.
 *          Address - specifies the slave address which will be transmitted.
 *          I2C_Direction - specifies whether the I2C device will be a
 *        Transmitter or a Receiver.
 *            I2C_Direction_Transmitter - Transmitter mode.
 *            I2C_Direction_Receiver - Receiver mode.
 *
 * @return  none
 */
void I2C_Send7bitAddress(I2C_TypeDef *I2Cx, uint8_t Address, uint8_t I2C_Direction) {
}

/*********************************************************************
 * @fn      I2C_ReadRegister
 *
 * @brief   Reads the specified I2C register and returns its value.
 *
 * @param   I2Cx - where x can be 1 or 2 to select the I2C peripheral.
 *          I2C_Register - specifies the register to read.
 *            I2C_Register_CTLR1.
 *            I2C_Register_CTLR2.
 *            I2C_Register_OADDR1.
 *            I2C_Register_OADDR2.
 *            I2C_Register_DATAR.
 *            I2C_Register_STAR1.
 *            I2C_Register_STAR2.
 *            I2C_Register_CKCFGR.
 *            I2C_Register_RTR.
 *
 * @return  none
 */
uint16_t I2C_ReadRegister(I2C_TypeDef *I2Cx, uint8_t I2C_Register) {

    return 0;
}

/*********************************************************************
 * @fn      I2C_SoftwareResetCmd
 *
 * @brief   Enables or disables the specified I2C software reset.
 *
 * @param   I2Cx - where x can be 1 or 2 to select the I2C peripheral.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void I2C_SoftwareResetCmd(I2C_TypeDef *I2Cx, FunctionalState NewState) {
}

/*********************************************************************
 * @fn      I2C_NACKPositionConfig
 *
 * @brief   Selects the specified I2C NACK position in master receiver mode.
 *
 * @param   I2Cx - where x can be 1 or 2 to select the I2C peripheral.
 *          I2C_NACKPosition - specifies the NACK position.
 *            I2C_NACKPosition_Next - indicates that the next byte will be
 *        the last received byte.
 *            I2C_NACKPosition_Current - indicates that current byte is the
 *        last received byte.
 *
 * @return  none
 */
void I2C_NACKPositionConfig(I2C_TypeDef *I2Cx, uint16_t I2C_NACKPosition) {
}

/*********************************************************************
 * @fn      I2C_SMBusAlertConfig
 *
 * @brief   Drives the SMBusAlert pin high or low for the specified I2C.
 *
 * @param   I2Cx - where x can be 1 or 2 to select the I2C peripheral.
 *          I2C_SMBusAlert - specifies SMBAlert pin level.
 *            I2C_SMBusAlert_Low - SMBAlert pin driven low.
 *            I2C_SMBusAlert_High - SMBAlert pin driven high.
 *
 * @return  none
 */
void I2C_SMBusAlertConfig(I2C_TypeDef *I2Cx, uint16_t I2C_SMBusAlert) {
}

/*********************************************************************
 * @fn      I2C_StretchClockCmd
 *
 * @brief   Enables or disables the specified I2C Clock stretching.
 *
 * @param   I2Cx - where x can be 1 or 2 to select the I2C peripheral.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void I2C_StretchClockCmd(I2C_TypeDef *I2Cx, FunctionalState NewState) {
}

/*********************************************************************
 * @fn      I2C_FastModeDutyCycleConfig
 *
 * @brief   Selects the specified I2C fast mode duty cycle.
 *
 * @param   I2Cx - where x can be 1 or 2 to select the I2C peripheral.
 *          I2C_DutyCycle - specifies the fast mode duty cycle.
 *            I2C_DutyCycle_2 - I2C fast mode Tlow/Thigh = 2.
 *            I2C_DutyCycle_16_9 - I2C fast mode Tlow/Thigh = 16/9.
 *
 * @return  none
 */
void I2C_FastModeDutyCycleConfig(I2C_TypeDef *I2Cx, uint16_t I2C_DutyCycle) {
}

/*********************************************************************
 * @fn      I2C_CheckEvent
 *
 * @brief   Checks whether the last I2Cx Event is equal to the one passed
 *        as parameter.
 *
 * @param   I2Cx- where x can be 1 or 2 to select the I2C peripheral.
 *          I2C_EVENT: specifies the event to be checked.
 *             I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED - EV1.
 *             I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED - EV1.
 *             I2C_EVENT_SLAVE_TRANSMITTER_SECONDADDRESS_MATCHED - EV1.
 *             I2C_EVENT_SLAVE_RECEIVER_SECONDADDRESS_MATCHED - EV1.
 *             I2C_EVENT_SLAVE_GENERALCALLADDRESS_MATCHED - EV1.
 *             I2C_EVENT_SLAVE_BYTE_RECEIVED - EV2.
 *             (I2C_EVENT_SLAVE_BYTE_RECEIVED | I2C_FLAG_DUALF) - EV2.
 *             (I2C_EVENT_SLAVE_BYTE_RECEIVED | I2C_FLAG_GENCALL) - EV2.
 *             I2C_EVENT_SLAVE_BYTE_TRANSMITTED - EV3.
 *             (I2C_EVENT_SLAVE_BYTE_TRANSMITTED | I2C_FLAG_DUALF) - EV3.
 *             (I2C_EVENT_SLAVE_BYTE_TRANSMITTED | I2C_FLAG_GENCALL) - EV3.
 *             I2C_EVENT_SLAVE_ACK_FAILURE - EV3_2.
 *             I2C_EVENT_SLAVE_STOP_DETECTED - EV4.
 *             I2C_EVENT_MASTER_MODE_SELECT - EV5.
 *             I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED - EV6.
 *             I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED - EV6.
 *             I2C_EVENT_MASTER_BYTE_RECEIVED - EV7.
 *             I2C_EVENT_MASTER_BYTE_TRANSMITTING - EV8.
 *             I2C_EVENT_MASTER_BYTE_TRANSMITTED - EV8_2.
 *             I2C_EVENT_MASTER_MODE_ADDRESS10 - EV9.
 *
 * @return  none
 */
ErrorStatus I2C_CheckEvent(I2C_TypeDef *I2Cx, uint32_t I2C_EVENT) {
    return 0;
}

/*********************************************************************
 * @fn      I2C_GetLastEvent
 *
 * @brief   Returns the last I2Cx Event.
 *
 * @param   I2Cx - where x can be 1 or 2 to select the I2C peripheral.
 *
 * @return  none
 */
uint32_t I2C_GetLastEvent(I2C_TypeDef *I2Cx) {
    return 0;
}

/*********************************************************************
 * @fn      I2C_GetFlagStatus
 *
 * @brief   Checks whether the last I2Cx Event is equal to the one passed
 *        as parameter.
 *
 * @param   I2Cx - where x can be 1 or 2 to select the I2C peripheral.
 *          I2C_FLAG - specifies the flag to check.
 *            I2C_FLAG_DUALF - Dual flag (Slave mode).
 *            I2C_FLAG_SMBHOST - SMBus host header (Slave mode).
 *            I2C_FLAG_SMBDEFAULT - SMBus default header (Slave mode).
 *            I2C_FLAG_GENCALL - General call header flag (Slave mode).
 *            I2C_FLAG_TRA - Transmitter/Receiver flag.
 *            I2C_FLAG_BUSY - Bus busy flag.
 *            I2C_FLAG_MSL - Master/Slave flag.
 *            I2C_FLAG_SMBALERT - SMBus Alert flag.
 *            I2C_FLAG_TIMEOUT - Timeout or Tlow error flag.
 *            I2C_FLAG_PECERR - PEC error in reception flag.
 *            I2C_FLAG_OVR - Overrun/Underrun flag (Slave mode).
 *            I2C_FLAG_AF - Acknowledge failure flag.
 *            I2C_FLAG_ARLO - Arbitration lost flag (Master mode).
 *            I2C_FLAG_BERR - Bus error flag.
 *            I2C_FLAG_TXE - Data register empty flag (Transmitter).
 *            I2C_FLAG_RXNE- Data register not empty (Receiver) flag.
 *            I2C_FLAG_STOPF - Stop detection flag (Slave mode).
 *            I2C_FLAG_ADD10 - 10-bit header sent flag (Master mode).
 *            I2C_FLAG_BTF - Byte transfer finished flag.
 *            I2C_FLAG_ADDR - Address sent flag (Master mode) "ADSL"
 *        Address matched flag (Slave mode)"ENDA".
 *            I2C_FLAG_SB - Start bit flag (Master mode).
 *
 * @return  none
 */
FlagStatus I2C_GetFlagStatus(I2C_TypeDef *I2Cx, uint32_t I2C_FLAG) {
    return 0;
}

/*********************************************************************
 * @fn      I2C_ClearFlag
 *
 * @brief   Clears the I2Cx's pending flags.
 *
 * @param   I2Cx - where x can be 1 or 2 to select the I2C peripheral.
 *          I2C_FLAG - specifies the flag to clear.
 *            I2C_FLAG_SMBALERT - SMBus Alert flag.
 *            I2C_FLAG_TIMEOUT - Timeout or Tlow error flag.
 *            I2C_FLAG_PECERR - PEC error in reception flag.
 *            I2C_FLAG_OVR - Overrun/Underrun flag (Slave mode).
 *            I2C_FLAG_AF - Acknowledge failure flag.
 *            I2C_FLAG_ARLO - Arbitration lost flag (Master mode).
 *            I2C_FLAG_BERR - Bus error flag.
 *
 * @return  none
 */
void I2C_ClearFlag(I2C_TypeDef *I2Cx, uint32_t I2C_FLAG) {
}

/*********************************************************************
 * @fn      I2C_GetITStatus
 *
 * @brief   Checks whether the specified I2C interrupt has occurred or not.
 *
 * @param   I2Cx - where x can be 1 or 2 to select the I2C peripheral.
 *          II2C_IT - specifies the interrupt source to check.
 *            I2C_IT_SMBALERT - SMBus Alert flag.
 *            I2C_IT_TIMEOUT - Timeout or Tlow error flag.
 *            I2C_IT_PECERR - PEC error in reception flag.
 *            I2C_IT_OVR - Overrun/Underrun flag (Slave mode).
 *            I2C_IT_AF - Acknowledge failure flag.
 *            I2C_IT_ARLO - Arbitration lost flag (Master mode).
 *            I2C_IT_BERR - Bus error flag.
 *            I2C_IT_TXE - Data register empty flag (Transmitter).
 *            I2C_IT_RXNE - Data register not empty (Receiver) flag.
 *            I2C_IT_STOPF - Stop detection flag (Slave mode).
 *            I2C_IT_ADD10 - 10-bit header sent flag (Master mode).
 *            I2C_IT_BTF - Byte transfer finished flag.
 *            I2C_IT_ADDR - Address sent flag (Master mode) "ADSL"  Address matched
 *        flag (Slave mode)"ENDAD".
 *            I2C_IT_SB - Start bit flag (Master mode).
 *
 * @return  none
 */
ITStatus I2C_GetITStatus(I2C_TypeDef *I2Cx, uint32_t I2C_IT) {
    return 0;
}

/*********************************************************************
 * @fn      I2C_ClearITPendingBit
 *
 * @brief   Clears the I2Cx interrupt pending bits.
 *
 * @param   I2Cx - where x can be 1 or 2 to select the I2C peripheral.
 *          I2C_IT - specifies the interrupt pending bit to clear.
 *            I2C_IT_SMBALERT - SMBus Alert interrupt.
 *            I2C_IT_TIMEOUT - Timeout or Tlow error interrupt.
 *            I2C_IT_PECERR - PEC error in reception  interrupt.
 *            I2C_IT_OVR - Overrun/Underrun interrupt (Slave mode).
 *            I2C_IT_AF - Acknowledge failure interrupt.
 *            I2C_IT_ARLO - Arbitration lost interrupt (Master mode).
 *            I2C_IT_BERR - Bus error interrupt.
 *
 * @return  none
 */
void I2C_ClearITPendingBit(I2C_TypeDef *I2Cx, uint32_t I2C_IT) {
}
