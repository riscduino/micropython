/********************************** (C) COPYRIGHT  *******************************
* Copyright (c) 2024  BigEndian Semiconductor Private Limited
* SPDX-License-Identifier: Apache-2.0
*
* File Name          : bes32r20xx_misc.h
* Author             : Dinesh Annayya
* Version            : V1.0.0
* Date               : 20-June-2024
* Description        : This file contains all the functions prototypes for the
*                      miscellaneous firmware library functions.
*******************************************************************************/
#ifndef __BES32R20XX_MISC_H
#define __BES32R20XX_MISC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bes32r20xx.h"

/* NVIC Init Structure definition */
typedef struct
{
    uint8_t NVIC_IRQChannel;
    uint8_t NVIC_IRQChannelPreemptionPriority;
    uint8_t NVIC_IRQChannelSubPriority;
    FunctionalState NVIC_IRQChannelCmd;
} NVIC_InitTypeDef;


/* Preemption_Priority_Group */
#define NVIC_PriorityGroup_0           ((uint32_t)0x00)
#define NVIC_PriorityGroup_1           ((uint32_t)0x01)
#define NVIC_PriorityGroup_2           ((uint32_t)0x02)
#define NVIC_PriorityGroup_3           ((uint32_t)0x03)
#define NVIC_PriorityGroup_4           ((uint32_t)0x04)


void NVIC_PriorityGroupConfig(uint32_t NVIC_PriorityGroup);
void NVIC_Init(NVIC_InitTypeDef *NVIC_InitStruct);

#ifdef __cplusplus
}
#endif

#endif
