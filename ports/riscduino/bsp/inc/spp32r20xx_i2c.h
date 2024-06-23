/********************************** (C) COPYRIGHT  *******************************
* Copyright (c) 2024  SiPlusPlus Semiconductor Private Limited
* SPDX-License-Identifier: Apache-2.0
*
* File Name          : spp32r20xx_i2c.h
* Author             : Dinesh annayya
* Version            : V1.0.0
* Date               : 20-June-2024
* Description        : This file contains all the functions prototypes for the
*                      I2C firmware library.
*******************************************************************************/
#ifndef __SPP32R20XX_I2C_H
#define __SPP32R20XX_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "spp32r20xx.h"

/* I2C Init structure definition  */
typedef struct
{
    uint32_t I2C_ClockSpeed;        /* Specifies the clock frequency.
                                       This parameter must be set to a value lower than 400kHz */

    uint16_t I2C_Mode;              /* Specifies the I2C mode.
                                       This parameter can be a value of @ref I2C_mode */

    uint16_t I2C_DutyCycle;         /* Specifies the I2C fast mode duty cycle.
                                       This parameter can be a value of @ref I2C_duty_cycle_in_fast_mode */

    uint16_t I2C_OwnAddress1;       /* Specifies the first device own address.
                                       This parameter can be a 7-bit or 10-bit address. */

    uint16_t I2C_Ack;               /* Enables or disables the acknowledgement.
                                       This parameter can be a value of @ref I2C_acknowledgement */

    uint16_t I2C_AcknowledgedAddress; /* Specifies if 7-bit or 10-bit address is acknowledged.
                                       This parameter can be a value of @ref I2C_acknowledged_address */
}I2C_InitTypeDef;

#define I2C_Mode_I2C                    ((uint16_t)0x0000)
#define I2C_Mode_SMBusDevice            ((uint16_t)0x0002)
#define I2C_Mode_SMBusHost              ((uint16_t)0x000A)

/* I2C_duty_cycle_in_fast_mode */
#define I2C_DutyCycle_16_9              ((uint16_t)0x4000) /* I2C fast mode Tlow/Thigh = 16/9 */
#define I2C_DutyCycle_2                 ((uint16_t)0xBFFF) /* I2C fast mode Tlow/Thigh = 2 */

/* I2C_acknowledgement */
#define I2C_Ack_Enable                  ((uint16_t)0x0400)
#define I2C_Ack_Disable                 ((uint16_t)0x0000)

/* I2C_transfer_direction */
#define I2C_Direction_Transmitter       ((uint8_t)0x00)
#define I2C_Direction_Receiver          ((uint8_t)0x01)

/* I2C_acknowledged_address */
#define I2C_AcknowledgedAddress_7bit    ((uint16_t)0x4000)
#define I2C_AcknowledgedAddress_10bit   ((uint16_t)0xC000)



void I2C_DeInit(I2C_TypeDef *I2Cx);
void I2C_Init(I2C_TypeDef *I2Cx, I2C_InitTypeDef *I2C_InitStruct);
void I2C_StructInit(I2C_InitTypeDef *I2C_InitStruct);
void I2C_Cmd(I2C_TypeDef *I2Cx, FunctionalState NewState);
void I2C_DMACmd(I2C_TypeDef *I2Cx, FunctionalState NewState);
void I2C_DMALastTransferCmd(I2C_TypeDef *I2Cx, FunctionalState NewState);
void I2C_GenerateSTART(I2C_TypeDef *I2Cx, FunctionalState NewState);
void I2C_GenerateSTOP(I2C_TypeDef *I2Cx, FunctionalState NewState);
void I2C_AcknowledgeConfig(I2C_TypeDef *I2Cx, FunctionalState NewState);
void I2C_OwnAddress2Config(I2C_TypeDef *I2Cx, uint8_t Address);
void I2C_DualAddressCmd(I2C_TypeDef *I2Cx, FunctionalState NewState);
void I2C_GeneralCallCmd(I2C_TypeDef *I2Cx, FunctionalState NewState);
void I2C_ITConfig(I2C_TypeDef *I2Cx, uint16_t I2C_IT, FunctionalState NewState);
void I2C_SendData(I2C_TypeDef *I2Cx, uint8_t Data);
uint8_t I2C_ReceiveData(I2C_TypeDef *I2Cx);
void I2C_Send7bitAddress(I2C_TypeDef *I2Cx, uint8_t Address, uint8_t I2C_Direction);
uint16_t I2C_ReadRegister(I2C_TypeDef *I2Cx, uint8_t I2C_Register);
void I2C_SoftwareResetCmd(I2C_TypeDef *I2Cx, FunctionalState NewState);
void I2C_NACKPositionConfig(I2C_TypeDef *I2Cx, uint16_t I2C_NACKPosition);
void I2C_ARPCmd(I2C_TypeDef *I2Cx, FunctionalState NewState);
void I2C_StretchClockCmd(I2C_TypeDef *I2Cx, FunctionalState NewState);
void I2C_FastModeDutyCycleConfig(I2C_TypeDef *I2Cx, uint16_t I2C_DutyCycle);


/****************************************************************************************
*                         I2C State Monitoring Functions
****************************************************************************************/

ErrorStatus I2C_CheckEvent(I2C_TypeDef *I2Cx, uint32_t I2C_EVENT);
uint32_t I2C_GetLastEvent(I2C_TypeDef *I2Cx);
FlagStatus I2C_GetFlagStatus(I2C_TypeDef *I2Cx, uint32_t I2C_FLAG);

void I2C_ClearFlag(I2C_TypeDef *I2Cx, uint32_t I2C_FLAG);
ITStatus I2C_GetITStatus(I2C_TypeDef *I2Cx, uint32_t I2C_IT);
void I2C_ClearITPendingBit(I2C_TypeDef *I2Cx, uint32_t I2C_IT);

#ifdef __cplusplus
}
#endif

#endif
