/********************************** (C) COPYRIGHT *******************************
* Copyright (c) 2024  BigEndian Semiconductor Private Limited
* SPDX-License-Identifier: Apache-2.0
*
* File Name          : bes32r20xx_spi.c
* Author             : Dinesh Annayya
* Version            : V1.0.0
* Date               : 20-June-2024
* Description        : This file provides all the SPI firmware functions.
*********************************************************************************/
#include "bes32r20xx_spi.h"
#include "bes32r20xx_rcc.h"

/* SPI SPE mask */
#define CTLR1_SPE_Set         ((uint16_t)0x0040)
#define CTLR1_SPE_Reset       ((uint16_t)0xFFBF)

/* I2S I2SE mask */
#define I2SCFGR_I2SE_Set      ((uint16_t)0x0400)
#define I2SCFGR_I2SE_Reset    ((uint16_t)0xFBFF)

/* SPI CRCNext mask */
#define CTLR1_CRCNext_Set     ((uint16_t)0x1000)

/* SPI CRCEN mask */
#define CTLR1_CRCEN_Set       ((uint16_t)0x2000)
#define CTLR1_CRCEN_Reset     ((uint16_t)0xDFFF)

/* SPI SSOE mask */
#define CTLR2_SSOE_Set        ((uint16_t)0x0004)
#define CTLR2_SSOE_Reset      ((uint16_t)0xFFFB)

/* SPI registers Masks */
#define CTLR1_CLEAR_Mask      ((uint16_t)0x3040)
#define I2SCFGR_CLEAR_Mask    ((uint16_t)0xF040)

/* SPI or I2S mode selection masks */
#define SPI_Mode_Select       ((uint16_t)0xF7FF)
#define I2S_Mode_Select       ((uint16_t)0x0800)

/* I2S clock source selection masks */
#define I2S2_CLOCK_SRC        ((uint32_t)(0x00020000))
#define I2S3_CLOCK_SRC        ((uint32_t)(0x00040000))
#define I2S_MUL_MASK          ((uint32_t)(0x0000F000))
#define I2S_DIV_MASK          ((uint32_t)(0x000000F0))

/*********************************************************************
 * @fn      SPI_I2S_DeInit
 *
 * @brief   Deinitializes the SPIx peripheral registers to their default
 *        reset values (Affects also the I2Ss).
 * @param   SPIx - where x can be 1, 2 or 3 to select the SPI peripheral.
 *
 * @return  none
 */
void SPI_I2S_DeInit(SPI_TypeDef *SPIx) {
}

/*********************************************************************
 * @fn      SPI_Init
 *
 * @brief   Initializes the SPIx peripheral according to the specified
 *        parameters in the SPI_InitStruct.
 *
 * @param   SPIx - where x can be 1, 2 or 3 to select the SPI peripheral.
 *          SPI_InitStruct - pointer to a SPI_InitTypeDef structure that
 *        contains the configuration information for the specified SPI peripheral.
 *
 * @return  none
 */
void SPI_Init(SPI_TypeDef *SPIx, SPI_InitTypeDef *SPI_InitStruct) {
}

/*********************************************************************
 * @fn      I2S_Init
 *
 * @brief   Initializes the SPIx peripheral according to the specified
 *        parameters in the I2S_InitStruct.
 * @param   SPIx - where x can be 1, 2 or 3 to select the SPI peripheral.
 *        (configured in I2S mode).
 *          I2S_InitStruct - pointer to an I2S_InitTypeDef structure that
 *        contains the configuration information for the specified SPI peripheral
 *        configured in I2S mode.
 * @return  none
 */
void I2S_Init(SPI_TypeDef *SPIx, I2S_InitTypeDef *I2S_InitStruct) {
}

/*********************************************************************
 * @fn      SPI_StructInit
 *
 * @brief   Fills each SPI_InitStruct member with its default value.
 *
 * @param   SPI_InitStruct - pointer to a SPI_InitTypeDef structure which
 *        will be initialized.
 *
 * @return  none
 */
void SPI_StructInit(SPI_InitTypeDef *SPI_InitStruct) {
    SPI_InitStruct->SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStruct->SPI_Mode = SPI_Mode_Slave;
    SPI_InitStruct->SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStruct->SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStruct->SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStruct->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    SPI_InitStruct->SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStruct->SPI_CRCPolynomial = 7;
}

/*********************************************************************
 * @fn      I2S_StructInit
 *
 * @brief   Fills each I2S_InitStruct member with its default value.
 *
 * @param   I2S_InitStruct - pointer to a I2S_InitTypeDef structure which
 *        will be initialized.
 *
 * @return  none
 */
void I2S_StructInit(I2S_InitTypeDef *I2S_InitStruct) {
    I2S_InitStruct->I2S_Mode = I2S_Mode_SlaveTx;
    I2S_InitStruct->I2S_Standard = I2S_Standard_Phillips;
    I2S_InitStruct->I2S_DataFormat = I2S_DataFormat_16b;
    I2S_InitStruct->I2S_MCLKOutput = I2S_MCLKOutput_Disable;
    I2S_InitStruct->I2S_AudioFreq = I2S_AudioFreq_Default;
    I2S_InitStruct->I2S_CPOL = I2S_CPOL_Low;
}

/*********************************************************************
 * @fn      SPI_Cmd
 *
 * @brief   Enables or disables the specified SPI peripheral.
 *
 * @param   SPIx - where x can be 1, 2 or 3 to select the SPI peripheral.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void SPI_Cmd(SPI_TypeDef *SPIx, FunctionalState NewState) {
}

/*********************************************************************
 * @fn      I2S_Cmd
 *
 * @brief   Enables or disables the specified SPI peripheral (in I2S mode).
 *
 * @param   SPIx - where x can be 1, 2 or 3 to select the SPI peripheral.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void I2S_Cmd(SPI_TypeDef *SPIx, FunctionalState NewState) {
}

/*********************************************************************
 * @fn      SPI_I2S_ITConfig
 *
 * @brief   Enables or disables the specified SPI/I2S interrupts.
 *
 * @param   SPIx - where x can be
 *            - 1, 2 or 3 in SPI mode.
 *            - 2 or 3 in I2S mode.
 *          SPI_I2S_IT - specifies the SPI/I2S interrupt source to be
 *        enabled or disabled.
 *            SPI_I2S_IT_TXE - Tx buffer empty interrupt mask.
 *            SPI_I2S_IT_RXNE - Rx buffer not empty interrupt mask.
 *            SPI_I2S_IT_ERR - Error interrupt mask.
 *          NewState: ENABLE or DISABLE.
 * @return  none
 */
void SPI_I2S_ITConfig(SPI_TypeDef *SPIx, uint8_t SPI_I2S_IT, FunctionalState NewState) {
}

/*********************************************************************
 * @fn      SPI_I2S_DMACmd
 *
 * @brief   Enables or disables the SPIx/I2Sx DMA interface.
 *
 * @param   SPIx - where x can be
 *            - 1, 2 or 3 in SPI mode.
 *            - 2 or 3 in I2S mode.
 *          SPI_I2S_DMAReq - specifies the SPI/I2S DMA transfer request to
 *        be enabled or disabled.
 *            SPI_I2S_DMAReq_Tx - Tx buffer DMA transfer request.
 *            SPI_I2S_DMAReq_Rx - Rx buffer DMA transfer request.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void SPI_I2S_DMACmd(SPI_TypeDef *SPIx, uint16_t SPI_I2S_DMAReq, FunctionalState NewState) {
}

/*********************************************************************
 * @fn      SPI_I2S_SendData
 *
 * @brief   Transmits a Data through the SPIx/I2Sx peripheral.
 *
 * @param   SPIx - where x can be
 *            - 1, 2 or 3 in SPI mode.
 *            - 2 or 3 in I2S mode.
 *          Data - Data to be transmitted.
 *
 * @return  none
 */
void SPI_I2S_SendData(SPI_TypeDef *SPIx, uint16_t Data) {
}

/*********************************************************************
 * @fn      SPI_I2S_ReceiveData
 *
 * @brief   Returns the most recent received data by the SPIx/I2Sx peripheral.
 *
 * @param   SPIx - where x can be
 *            - 1, 2 or 3 in SPI mode.
 *            - 2 or 3 in I2S mode.
 *          Data - Data to be transmitted.
 *
 * @return  SPIx->DATAR - The value of the received data.
 */
uint16_t SPI_I2S_ReceiveData(SPI_TypeDef *SPIx) {
    return 0;
}

/*********************************************************************
 * @fn      SPI_NSSInternalSoftwareConfig
 *
 * @brief   Configures internally by software the NSS pin for the selected SPI.
 *
 * @param   SPIx - where x can be 1, 2 or 3 to select the SPI peripheral.
 *          SPI_NSSInternalSoft -
 *            SPI_NSSInternalSoft_Set - Set NSS pin internally.
 *            SPI_NSSInternalSoft_Reset - Reset NSS pin internally.
 *
 * @return  none
 */
void SPI_NSSInternalSoftwareConfig(SPI_TypeDef *SPIx, uint16_t SPI_NSSInternalSoft) {
}

/*********************************************************************
 * @fn      SPI_SSOutputCmd
 *
 * @brief   Enables or disables the SS output for the selected SPI.
 *
 * @param   SPIx - where x can be 1, 2 or 3 to select the SPI peripheral.
 *          NewState - new state of the SPIx SS output.
 *
 * @return  none
 */
void SPI_SSOutputCmd(SPI_TypeDef *SPIx, FunctionalState NewState) {
}

/*********************************************************************
 * @fn      SPI_DataSizeConfig
 *
 * @brief   Configures the data size for the selected SPI.
 *
 * @param   SPIx - where x can be 1, 2 or 3 to select the SPI peripheral.
 *          SPI_DataSize - specifies the SPI data size.
 *            SPI_DataSize_16b - Set data frame format to 16bit.
 *            SPI_DataSize_8b - Set data frame format to 8bit.
 *
 * @return  none
 */
void SPI_DataSizeConfig(SPI_TypeDef *SPIx, uint16_t SPI_DataSize) {
}

/*********************************************************************
 * @fn      SPI_TransmitCRC
 *
 * @brief   Transmit the SPIx CRC value.
 *
 * @param   SPIx - where x can be 1, 2 or 3 to select the SPI peripheral.
 *
 * @return  none
 */
void SPI_TransmitCRC(SPI_TypeDef *SPIx) {
}

/*********************************************************************
 * @fn      SPI_CalculateCRC
 *
 * @brief   Enables or disables the CRC value calculation of the transferred bytes.
 *
 * @param   SPIx - where x can be 1, 2 or 3 to select the SPI peripheral.
 *          NewState - new state of the SPIx CRC value calculation.
 *
 * @return  none
 */
void SPI_CalculateCRC(SPI_TypeDef *SPIx, FunctionalState NewState) {
}

/*********************************************************************
 * @fn      SPI_GetCRC
 *
 * @brief   Returns the transmit or the receive CRC register value for the specified SPI.
 *
 * @param   SPIx - where x can be 1, 2 or 3 to select the SPI peripheral.
 *          SPI_CRC - specifies the CRC register to be read.
 *            SPI_CRC_Tx - Selects Tx CRC register.
 *            SPI_CRC_Rx - Selects Rx CRC register.
 *
 * @return  crcreg: The selected CRC register value.
 */
uint16_t SPI_GetCRC(SPI_TypeDef *SPIx, uint8_t SPI_CRC) {
    return 0;
}

/*********************************************************************
 * @fn      SPI_GetCRCPolynomial
 *
 * @brief   Returns the CRC Polynomial register value for the specified SPI.
 *
 * @param   SPIx - where x can be 1, 2 or 3 to select the SPI peripheral.
 *
 * @return  SPIx->CRCR - The CRC Polynomial register value.
 */
uint16_t SPI_GetCRCPolynomial(SPI_TypeDef *SPIx) {
    return 0;
}

/*********************************************************************
 * @fn      SPI_BiDirectionalLineConfig
 *
 * @brief   Selects the data transfer direction in bi-directional mode
 *      for the specified SPI.
 *
 * @param   SPIx - where x can be 1, 2 or 3 to select the SPI peripheral.
 *          SPI_Direction - specifies the data transfer direction in
 *        bi-directional mode.
 *            SPI_Direction_Tx - Selects Tx transmission direction.
 *            SPI_Direction_Rx - Selects Rx receive direction.
 *
 * @return  none
 */
void SPI_BiDirectionalLineConfig(SPI_TypeDef *SPIx, uint16_t SPI_Direction) {
}

/*********************************************************************
 * @fn      SPI_I2S_GetFlagStatus
 *
 * @brief   Checks whether the specified SPI/I2S flag is set or not.
 *
 * @param   SPIx - where x can be
 *            - 1, 2 or 3 in SPI mode.
 *            - 2 or 3 in I2S mode.
 *          SPI_I2S_FLAG - specifies the SPI/I2S flag to check.
 *            SPI_I2S_FLAG_TXE - Transmit buffer empty flag.
 *            SPI_I2S_FLAG_RXNE - Receive buffer not empty flag.
 *            SPI_I2S_FLAG_BSY - Busy flag.
 *            SPI_I2S_FLAG_OVR - Overrun flag.
 *            SPI_FLAG_MODF - Mode Fault flag.
 *            SPI_FLAG_CRCERR - CRC Error flag.
 *            I2S_FLAG_UDR - Underrun Error flag.
 *            I2S_FLAG_CHSIDE - Channel Side flag.
 *
 * @return  none
 */
FlagStatus SPI_I2S_GetFlagStatus(SPI_TypeDef *SPIx, uint16_t SPI_I2S_FLAG) {
    return 0;
}

/*********************************************************************
 * @fn      SPI_I2S_ClearFlag
 *
 * @brief   Clears the SPIx CRC Error (CRCERR) flag.
 *
 * @param   SPIx - where x can be
 *            - 1, 2 or 3 in SPI mode.
 *            - 2 or 3 in I2S mode.
 *          SPI_I2S_FLAG - specifies the SPI flag to clear.
 *            SPI_FLAG_CRCERR - CRC Error flag.
 *
 * @return  none
 */
void SPI_I2S_ClearFlag(SPI_TypeDef *SPIx, uint16_t SPI_I2S_FLAG) {
}

/*********************************************************************
 * @fn      SPI_I2S_GetITStatus
 *
 * @brief   Checks whether the specified SPI/I2S interrupt has occurred or not.
 *
 * @param   SPIx - where x can be
 *            - 1, 2 or 3 in SPI mode.
 *            - 2 or 3 in I2S mode.
 *          SPI_I2S_IT - specifies the SPI/I2S interrupt source to check..
 *            SPI_I2S_IT_TXE - Transmit buffer empty interrupt.
 *            SPI_I2S_IT_RXNE - Receive buffer not empty interrupt.
 *            SPI_I2S_IT_OVR - Overrun interrupt.
 *            SPI_IT_MODF - Mode Fault interrupt.
 *            SPI_IT_CRCERR - CRC Error interrupt.
 *            I2S_IT_UDR - Underrun Error interrupt.
 *
 * @return  FlagStatus: SET or RESET.
 */
ITStatus SPI_I2S_GetITStatus(SPI_TypeDef *SPIx, uint8_t SPI_I2S_IT) {
    return 0;
}

/*********************************************************************
 * @fn      SPI_I2S_ClearITPendingBit
 *
 * @brief   Clears the SPIx CRC Error (CRCERR) interrupt pending bit.
 *
 * @param   SPIx - where x can be
 *            - 1, 2 or 3 in SPI mode.
 *          SPI_I2S_IT - specifies the SPI interrupt pending bit to clear.
 *            SPI_IT_CRCERR - CRC Error interrupt.
 *
 * @return  none
 */
void SPI_I2S_ClearITPendingBit(SPI_TypeDef *SPIx, uint8_t SPI_I2S_IT) {
}
