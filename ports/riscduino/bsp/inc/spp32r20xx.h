/********************************** (C) COPYRIGHT *******************************
* Copyright (c) 2024  SiPlusPlus Semiconductor Private Limited
* SPDX-License-Identifier: Apache-2.0
*
* File Name              : spp32r20xx.h
* Author                 : Dinesh Annayya
* Version                : V1.0.0
* Date                   : 20-June-2024
* Description            : SOC Level Defines
*******************************************************************************/

#ifndef __SPP32R20XX_H
#define __SPP32R20XX_H

#ifdef __cplusplus
extern "C" {
#endif

#define __MPU_PRESENT             0 /* Other CH32 devices does not provide an MPU */
#define __Vendor_SysTickConfig    0 /* Set to 1 if different SysTick Config is used */

#define HSE_VALUE    ((uint32_t)8000000) /* Value of the External oscillator in Hz */

/* In the following line adjust the External High Speed oscillator (HSE) Startup Timeout value */
#define HSE_STARTUP_TIMEOUT   ((uint16_t)0x1000) /* Time out for HSE start up */

#define HSI_VALUE    ((uint32_t)8000000) /* Value of the Internal oscillator in Hz */

/* Interrupt Number Definition, according to the selected device */
typedef enum IRQn
{
    /******  RISC-V Processor Exceptions Numbers *******************************************************/
    NonMaskableInt_IRQn         = 2,     /* 2 Non Maskable Interrupt                             */
    EXC_IRQn                    = 3,     /* 3 Exception Interrupt                                */
    Ecall_M_Mode_IRQn           = 5,     /* 5 Ecall M Mode Interrupt                             */
    Ecall_U_Mode_IRQn           = 8,     /* 8 Ecall U Mode Interrupt                             */
    Break_Point_IRQn            = 9,     /* 9 Break Point Interrupt                              */
    SysTicK_IRQn                = 12,    /* 12 System timer Interrupt                            */
    Software_IRQn               = 14,    /* 14 software Interrupt                                */

    /******  RISC-V specific Interrupt Numbers *********************************************************/
    RTC_IRQn                    = 19,    /* RTC global Interrupt                                 */
    RCC_IRQn                    = 21,    /* RCC global Interrupt                                 */
    EXTI0_IRQn                  = 22,    /* EXTI Line0 Interrupt                                 */
    EXTI1_IRQn                  = 23,    /* EXTI Line1 Interrupt                                 */
    EXTI2_IRQn                  = 24,    /* EXTI Line2 Interrupt                                 */
    EXTI3_IRQn                  = 25,    /* EXTI Line3 Interrupt                                 */
    EXTI4_IRQn                  = 26,    /* EXTI Line4 Interrupt                                 */
    EXTI9_5_IRQn                = 39,    /* External Line[9:5] Interrupts                        */
    TIM1_BRK_IRQn               = 40,    /* TIM1 Break Interrupt                                 */
    TIM1_UP_IRQn                = 41,    /* TIM1 Update Interrupt                                */
    TIM1_TRG_COM_IRQn           = 42,    /* TIM1 Trigger and Commutation Interrupt               */
    TIM1_CC_IRQn                = 43,    /* TIM1 Capture Compare Interrupt                       */
    TIM2_IRQn                   = 44,    /* TIM2 global Interrupt                                */
    TIM3_IRQn                   = 45,    /* TIM3 global Interrupt                                */
    I2C1_EV_IRQn                = 47,    /* I2C1 Event Interrupt                                 */
    I2C1_ER_IRQn                = 48,    /* I2C1 Error Interrupt                                 */
    I2C2_EV_IRQn                = 49,    /* I2C2 Event Interrupt                                 */
    I2C2_ER_IRQn                = 50,    /* I2C2 Error Interrupt                                 */
    SPI1_IRQn                   = 51,    /* SPI1 global Interrupt                                */
    SPI2_IRQn                   = 52,    /* SPI2 global Interrupt                                */
    USART1_IRQn                 = 53,    /* USART1 global Interrupt                              */
    USART2_IRQn                 = 54,    /* USART2 global Interrupt                              */
    EXTI15_10_IRQn              = 56,    /* External Line[15:10] Interrupts                      */
    RTCAlarm_IRQn               = 57,    /* RTC Alarm through EXTI Line Interrupt                */

} IRQn_Type;

#define HardFault_IRQn   EXC_IRQn
#define ADC1_2_IRQn      ADC_IRQn


#include <stdint.h>
#include "core_riscv.h"
#include "system_spp32r20xx.h"


/* Standard Peripheral Library old definitions (maintained for legacy purpose) */
#define HSI_Value            HSI_VALUE
#define HSE_Value            HSE_VALUE
#define HSEStartUp_TimeOut   HSE_STARTUP_TIMEOUT

/* CRC Calculation Unit */
typedef struct
{
    __IO uint32_t DATAR;
    __IO uint8_t IDATAR;
    uint8_t RESERVED0;
    uint16_t RESERVED1;
    __IO uint32_t CTLR;
} CRC_TypeDef;

/* DMA Channel Controller */
typedef struct
{
    __IO uint32_t CFGR;
    __IO uint32_t CNTR;
    __IO uint32_t PADDR;
    __IO uint32_t MADDR;
} DMA_Channel_TypeDef;

/* DMA Controller */
typedef struct
{
    __IO uint32_t INTFR;
    __IO uint32_t INTFCR;
} DMA_TypeDef;

/* External Interrupt/Event Controller */
typedef struct
{
    __IO uint32_t INTENR;
    __IO uint32_t EVENR;
    __IO uint32_t RTENR;
    __IO uint32_t FTENR;
    __IO uint32_t SWIEVR;
    __IO uint32_t INTFR;
} EXTI_TypeDef;

/* General Purpose I/O */
typedef struct
{
    __IO uint32_t CFGLR;
    __IO uint32_t CFGHR;
    __IO uint32_t INDR;
    __IO uint32_t OUTDR;
    __IO uint32_t BSHR;
    __IO uint32_t BCR;
    __IO uint32_t LCKR;
} GPIO_TypeDef;

/* Alternate Function I/O */
typedef struct
{
    __IO uint32_t ECR;
    __IO uint32_t PCFR1;
    __IO uint32_t EXTICR[4];
    uint32_t RESERVED0;
    __IO uint32_t PCFR2;
} AFIO_TypeDef;

/* Inter Integrated Circuit Interface */
typedef struct
{
    __IO uint16_t CTLR1;
    uint16_t RESERVED0;
    __IO uint16_t CTLR2;
    uint16_t RESERVED1;
    __IO uint16_t OADDR1;
    uint16_t RESERVED2;
    __IO uint16_t OADDR2;
    uint16_t RESERVED3;
    __IO uint16_t DATAR;
    uint16_t RESERVED4;
    __IO uint16_t STAR1;
    uint16_t RESERVED5;
    __IO uint16_t STAR2;
    uint16_t RESERVED6;
    __IO uint16_t CKCFGR;
    uint16_t RESERVED7;
    __IO uint16_t RTR;
    uint16_t RESERVED8;
} I2C_TypeDef;


/* Power Control */
typedef struct
{
    __IO uint32_t CTLR;
    __IO uint32_t CSR;
} PWR_TypeDef;

/* Reset and Clock Control */
typedef struct
{
    __IO uint32_t CTLR;
    __IO uint32_t CFGR0;
    __IO uint32_t INTR;
    __IO uint32_t APB2PRSTR;
    __IO uint32_t APB1PRSTR;
    __IO uint32_t AHBPCENR;
    __IO uint32_t APB2PCENR;
    __IO uint32_t APB1PCENR;
    __IO uint32_t BDCTLR;
    __IO uint32_t RSTSCKR;

    __IO uint32_t AHBRSTR;
    __IO uint32_t CFGR2;
} RCC_TypeDef;

/* Real-Time Clock */
typedef struct
{
    __IO uint16_t CTLRH;
    uint16_t RESERVED0;
    __IO uint16_t CTLRL;
    uint16_t RESERVED1;
    __IO uint16_t PSCRH;
    uint16_t RESERVED2;
    __IO uint16_t PSCRL;
    uint16_t RESERVED3;
    __IO uint16_t DIVH;
    uint16_t RESERVED4;
    __IO uint16_t DIVL;
    uint16_t RESERVED5;
    __IO uint16_t CNTH;
    uint16_t RESERVED6;
    __IO uint16_t CNTL;
    uint16_t RESERVED7;
    __IO uint16_t ALRMH;
    uint16_t RESERVED8;
    __IO uint16_t ALRML;
    uint16_t RESERVED9;
} RTC_TypeDef;


/* Serial Peripheral Interface */
typedef struct
{
    __IO uint16_t CTLR1;
    uint16_t RESERVED0;
    __IO uint16_t CTLR2;
    uint16_t RESERVED1;
    __IO uint16_t STATR;
    uint16_t RESERVED2;
    __IO uint16_t DATAR;
    uint16_t RESERVED3;
    __IO uint16_t CRCR;
    uint16_t RESERVED4;
    __IO uint16_t RCRCR;
    uint16_t RESERVED5;
    __IO uint16_t TCRCR;
    uint16_t RESERVED6;
    __IO uint16_t I2SCFGR;
    uint16_t RESERVED7;
    __IO uint16_t I2SPR;
    uint16_t RESERVED8;
} SPI_TypeDef;

/* TIM */
typedef struct
{
    __IO uint16_t CTLR1;
    uint16_t RESERVED0;
    __IO uint16_t CTLR2;
    uint16_t RESERVED1;
    __IO uint16_t SMCFGR;
    uint16_t RESERVED2;
    __IO uint16_t DMAINTENR;
    uint16_t RESERVED3;
    __IO uint16_t INTFR;
    uint16_t RESERVED4;
    __IO uint16_t SWEVGR;
    uint16_t RESERVED5;
    __IO uint16_t CHCTLR1;
    uint16_t RESERVED6;
    __IO uint16_t CHCTLR2;
    uint16_t RESERVED7;
    __IO uint16_t CCER;
    uint16_t RESERVED8;
    __IO uint16_t CNT;
    uint16_t RESERVED9;
    __IO uint16_t PSC;
    uint16_t RESERVED10;
    __IO uint16_t ATRLR;
    uint16_t RESERVED11;
    __IO uint16_t RPTCR;
    uint16_t RESERVED12;
    __IO uint16_t CH1CVR;
    uint16_t RESERVED13;
    __IO uint16_t CH2CVR;
    uint16_t RESERVED14;
    __IO uint16_t CH3CVR;
    uint16_t RESERVED15;
    __IO uint16_t CH4CVR;
    uint16_t RESERVED16;
    __IO uint16_t BDTR;
    uint16_t RESERVED17;
    __IO uint16_t DMACFGR;
    uint16_t RESERVED18;
    __IO uint16_t DMAADR;
    uint16_t RESERVED19;
} TIM_TypeDef;

/* Universal Synchronous Asynchronous Receiver Transmitter */
typedef struct {
  __IO uint32_t CTRL;                              /**< LPUART Control Register, offset: 0x00 */
  __IO uint32_t ISTAT;                             /**< LPUART Interrupt Status Register, offset: 0x04 */
  __IO uint32_t BAUD_LSB;                          /**< LPUART LSB Baud Rate Register, offset: 0x08 */
  __IO uint32_t BAUD_MSB;                          /**< LPUART MSB Baud Rate Register, offset: 0x0C */
  __IO uint32_t STAT;                              /**< LPUART Status Register, offset: 0x10 */
  __IO uint32_t TDATA;                             /**< LPUART Tx Data Register, offset: 0x14 */
  __IO uint32_t RDATA;                             /**< LPUART Rx Data Register, offset: 0x18 */
  __IO uint32_t TFIFO_FSPACE;                      /**< LPUART TXFIFO Free Space Register, offset: 0x1C */
  __IO uint32_t RFIFO_DAVAL;                       /**< LPUART RXFIFO Free Space Register, offset: 0x1C */
  __IO uint32_t WATER;                             /**< LPUART Watermark Register, offset: 0x2C */
} USART_TypeDef;


/* Enhanced Registers */
typedef struct
{
    __IO uint32_t EXTEN_CTR;
} EXTEN_TypeDef;

/* OPA Registers */
typedef struct
{
    __IO uint32_t CR;
} OPA_TypeDef;

/* RNG Registers */
typedef struct
{
    __IO uint32_t CR;
    __IO uint32_t SR;
    __IO uint32_t DR;
} RNG_TypeDef;


/* Peripheral memory map - riscduino */
#define GLBL_BASE_ADDR          0x10020000
#define GPIO_BASE_ADDR          0x10020080
#define PWM_BASE_ADDR           0x10020100
#define TIMER_BASE_ADDR         0x10020180
#define SEMA_BASE_ADDR          0x10020200
#define WS281X_BASE_ADDR        0x10020280
#define UART0_BASE_ADDR         0x10010000
#define UART1_BASE_ADDR         0x10010100
#define WIRE_BASE_ADDR          0x10010040
#define SPI0_BASE_ADDR          0x100100C0

// Helper functions
#define _REG32(p, i)         (*(volatile uint32_t *) ((p) + (i)))
#define _REG32P(p, i)        ((volatile uint32_t *) ((p) + (i)))
#define _REG8(p, i)          (*(volatile uint8_t *) ((p) + (i)))
#define _REG8P(p, i)         ((volatile uint8_t *) ((p) + (i)))
#define GLBL_REG(offset)      _REG32(GLBL_BASE_ADDR, offset)
#define GPIO_REG(offset)      _REG32(GPIO_BASE_ADDR, offset)
#define PWM_REG(offset)       _REG32(PWM_BASE_ADDR, offset)
#define TIMER_REG(offset)     _REG32(TIMER_BASE_ADDR, offset)
#define SEMA_REG(offset)      _REG32(SEMA_BASE_ADDR, offset)
#define WS281X_REG(offset)    _REG32(WS281X_BASE_ADDR, offset)
#define PWM0_REG(offset)      _REG32(PWM0_BASE_ADDR, offset)
#define PWM1_REG(offset)      _REG32(PWM1_BASE_ADDR, offset)
#define PWM2_REG(offset)      _REG32(PWM2_BASE_ADDR, offset)
#define SPI0_REG(offset)      _REG32(SPI0_BASE_ADDR, offset)
#define UART0_REG(offset)     _REG32(UART0_BASE_ADDR, offset)
#define UART1_REG(offset)     _REG32(UART1_BASE_ADDR, offset)
#define UART_REG(base,offset) _REG32(base, offset)
#define WIRE_REG(offset)      _REG32(WIRE_BASE_ADDR, offset)


// GPIO Port Defination
// GPIO DIRECT SEL
#define DDRA  _REG8(GPIO_BASE_ADDR, GPIO_DSEL+0)
#define DDRB  _REG8(GPIO_BASE_ADDR, GPIO_DSEL+1)
#define DDRC  _REG8(GPIO_BASE_ADDR, GPIO_DSEL+2)
#define DDRD  _REG8(GPIO_BASE_ADDR, GPIO_DSEL+3)

// GPIO OUTPUT PORT
#define PORTA  _REG8(GPIO_BASE_ADDR, GPIO_ODATA+0)
#define PORTA7  7
#define PORTA6  6
#define PORTA5  5
#define PORTA4  4
#define PORTA3  3
#define PORTA2  2
#define PORTA1  1
#define PORTA0  0

#define PORTB  _REG8(GPIO_BASE_ADDR, GPIO_ODATA+1)
#define PORTB7  7
#define PORTB6  6
#define PORTB5  5
#define PORTB4  4
#define PORTB3  3
#define PORTB2  2
#define PORTB1  1
#define PORTB0  0

#define PORTC  _REG8(GPIO_BASE_ADDR, GPIO_ODATA+2)
#define PORTC7  7
#define PORTC6  6
#define PORTC5  5
#define PORTC4  4
#define PORTC3  3
#define PORTC2  2
#define PORTC1  1
#define PORTC0  0

#define PORTD  _REG8(GPIO_BASE_ADDR, GPIO_ODATA+3)
#define PORTD7  7
#define PORTD6  6
#define PORTD5  5
#define PORTD4  4
#define PORTD3  3
#define PORTD2  2
#define PORTD1  1
#define PORTD0  0

// GPIO IN PORT
#define PINA  _REG8(GPIO_BASE_ADDR, GPIO_IDATA+0)
#define PINA7  7
#define PINA6  6
#define PINA5  5
#define PINA4  4
#define PINA3  3
#define PINA2  2
#define PINA1  1
#define PINA0  0

#define PINB  _REG8(GPIO_BASE_ADDR, GPIO_IDATA+1)
#define PINB7  7
#define PINB6  6
#define PINB5  5
#define PINB4  4
#define PINB3  3
#define PINB2  2
#define PINB1  1
#define PINB0  0

#define PINC  _REG8(GPIO_BASE_ADDR, GPIO_IDATA+2)
#define PINC7  7
#define PINC6  6
#define PINC5  5
#define PINC4  4
#define PINC3  3
#define PINC2  2
#define PINC1  1
#define PINC0  0

#define PIND  _REG8(GPIO_BASE_ADDR, GPIO_IDATA+3)
#define PIND7  7
#define PIND6  6
#define PIND5  5
#define PIND4  4
#define PIND3  3
#define PIND2  2
#define PIND1  1
#define PIND0  0


// MULTI FUNCTION ENABLE
// MASTER UART ENABLE
#define MFUNC_PWM_ENB            0x00000001
#define MFUNC_PWM0_ENB           0x00000001
#define MFUNC_PWM1_ENB           0x00000002
#define MFUNC_PWM2_ENB           0x00000004
#define MFUNC_PWM3_ENB           0x00000008
#define MFUNC_PWM4_ENB           0x00000010
#define MFUNC_PWM5_ENB           0x00000020
#define MFUNC_INT0_ENB           0x00000040
#define MFUNC_INT1_ENB           0x00000080
#define MFUNC_UART0_ENB          0x00000100
#define MFUNC_UART1_ENB          0x00000200
#define MFUNC_SPI_ENB            0x00000400
#define MFUNC_SPI_CS0_ENB        0x00000800
#define MFUNC_SPI_CS1_ENB        0x00001000
#define MFUNC_SPI_CS2_ENB        0x00002000
#define MFUNC_SPI_CS3_ENB        0x00004000
#define MFUNC_WIRE_ENB           0x00008000
#define MFUNC_USB_ENB            0x00010000
#define MFUNC_TAP_ENB            0x40000000
#define MFUNC_MUART_ENB          0x80000000

// DE-RESET CONTROL
#define DRST_SPI                0x00000004
#define DRST_UART0              0x00000008
#define DRST_WIRE               0x00000010
#define DRST_USB                0x00000020
#define DRST_UART1              0x00000040
#define DRST_CORE0              0x00000100
#define DRST_CORE1              0x00000200
#define DRST_CORE2              0x00000400
#define DRST_CORE3              0x00000800



/* LPUART - Peripheral instance base addresses */
#define UART0                ((USART_TypeDef *)UART0_BASE_ADDR)
#define UART1                ((USART_TypeDef *)UART1_BASE_ADDR)


/**** End of Riscduino Defination ************/



////
#define FLASH_BASE            ((uint32_t)0x08000000) /* FLASH base address in the alias region */
#define SRAM_BASE             ((uint32_t)0x20000000) /* SRAM base address in the alias region */
#define PERIPH_BASE           ((uint32_t)0x40000000) /* Peripheral base address in the alias region */

#define FSMC_R_BASE           ((uint32_t)0xA0000000) /* FSMC registers base address */


#define APB1PERIPH_BASE       (PERIPH_BASE)
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x10000)
#define AHBPERIPH_BASE        (PERIPH_BASE + 0x20000)

#define TIM2_BASE             (APB1PERIPH_BASE + 0x0000)
#define TIM3_BASE             (APB1PERIPH_BASE + 0x0400)
#define RTC_BASE              (APB1PERIPH_BASE + 0x2800)
#define I2C1_BASE             (0x10010040u)

#define AFIO_BASE             (APB2PERIPH_BASE + 0x0000)
#define EXTI_BASE             (APB2PERIPH_BASE + 0x0400)
#define GPIOA_BASE            (APB2PERIPH_BASE + 0x0800) 
#define GPIOB_BASE            (APB2PERIPH_BASE + 0x0C00)
#define GPIOC_BASE            (APB2PERIPH_BASE + 0x1000)
#define GPIOD_BASE            (APB2PERIPH_BASE + 0x1400)
#define GPIOE_BASE            (APB2PERIPH_BASE + 0x1800)
#define GPIOG_BASE            (APB2PERIPH_BASE + 0x2000)
#define TIM1_BASE             (0x10020180u)
#define SPI1_BASE             (0x100100C0u)
#define USART1_BASE           (0x10010000u)
#define USART2_BASE           (0x10010100u)

#define RCC_BASE              (AHBPERIPH_BASE + 0x1000)
#define EXTEN_BASE            (AHBPERIPH_BASE + 0x3800)

/* Peripheral declaration */
#define TIM2                ((TIM_TypeDef *)TIM2_BASE)
#define TIM3                ((TIM_TypeDef *)TIM3_BASE)
#define RTC                 ((RTC_TypeDef *)RTC_BASE)
#define I2C1                ((I2C_TypeDef *)I2C1_BASE)

#define GPIOA               ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *)GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *)GPIOD_BASE)
#define TIM1                ((TIM_TypeDef *)TIMER_BASE_ADDR)
#define SPI1                ((SPI_TypeDef *)SPI0_BASE_ADDR)

#define RCC                 ((RCC_TypeDef *)RCC_BASE)
#define EXTEN               ((EXTEN_TypeDef *)EXTEN_BASE)

/******************************************************************************/
/*                         Peripheral Registers Bits Definition               */
/******************************************************************************/

#define  GPIO_CFGLR_MODE                       ((uint32_t)0x33333333)        /* Port x mode bits */

#define  GPIO_CFGLR_MODE0                      ((uint32_t)0x00000003)        /* MODE0[1:0] bits (Port x mode bits, pin 0) */
#define  GPIO_CFGLR_MODE0_0                    ((uint32_t)0x00000001)        /* Bit 0 */
#define  GPIO_CFGLR_MODE0_1                    ((uint32_t)0x00000002)        /* Bit 1 */

#define  GPIO_CFGLR_MODE1                      ((uint32_t)0x00000030)        /* MODE1[1:0] bits (Port x mode bits, pin 1) */
#define  GPIO_CFGLR_MODE1_0                    ((uint32_t)0x00000010)        /* Bit 0 */
#define  GPIO_CFGLR_MODE1_1                    ((uint32_t)0x00000020)        /* Bit 1 */

#define  GPIO_CFGLR_MODE2                      ((uint32_t)0x00000300)        /* MODE2[1:0] bits (Port x mode bits, pin 2) */
#define  GPIO_CFGLR_MODE2_0                    ((uint32_t)0x00000100)        /* Bit 0 */
#define  GPIO_CFGLR_MODE2_1                    ((uint32_t)0x00000200)        /* Bit 1 */

#define  GPIO_CFGLR_MODE3                      ((uint32_t)0x00003000)        /* MODE3[1:0] bits (Port x mode bits, pin 3) */
#define  GPIO_CFGLR_MODE3_0                    ((uint32_t)0x00001000)        /* Bit 0 */
#define  GPIO_CFGLR_MODE3_1                    ((uint32_t)0x00002000)        /* Bit 1 */

#define  GPIO_CFGLR_MODE4                      ((uint32_t)0x00030000)        /* MODE4[1:0] bits (Port x mode bits, pin 4) */
#define  GPIO_CFGLR_MODE4_0                    ((uint32_t)0x00010000)        /* Bit 0 */
#define  GPIO_CFGLR_MODE4_1                    ((uint32_t)0x00020000)        /* Bit 1 */

#define  GPIO_CFGLR_MODE5                      ((uint32_t)0x00300000)        /* MODE5[1:0] bits (Port x mode bits, pin 5) */
#define  GPIO_CFGLR_MODE5_0                    ((uint32_t)0x00100000)        /* Bit 0 */
#define  GPIO_CFGLR_MODE5_1                    ((uint32_t)0x00200000)        /* Bit 1 */

#define  GPIO_CFGLR_MODE6                      ((uint32_t)0x03000000)        /* MODE6[1:0] bits (Port x mode bits, pin 6) */
#define  GPIO_CFGLR_MODE6_0                    ((uint32_t)0x01000000)        /* Bit 0 */
#define  GPIO_CFGLR_MODE6_1                    ((uint32_t)0x02000000)        /* Bit 1 */

#define  GPIO_CFGLR_MODE7                      ((uint32_t)0x30000000)        /* MODE7[1:0] bits (Port x mode bits, pin 7) */
#define  GPIO_CFGLR_MODE7_0                    ((uint32_t)0x10000000)        /* Bit 0 */
#define  GPIO_CFGLR_MODE7_1                    ((uint32_t)0x20000000)        /* Bit 1 */

#define  GPIO_CFGLR_CNF                        ((uint32_t)0xCCCCCCCC)        /* Port x configuration bits */

#define  GPIO_CFGLR_CNF0                       ((uint32_t)0x0000000C)        /* CNF0[1:0] bits (Port x configuration bits, pin 0) */
#define  GPIO_CFGLR_CNF0_0                     ((uint32_t)0x00000004)        /* Bit 0 */
#define  GPIO_CFGLR_CNF0_1                     ((uint32_t)0x00000008)        /* Bit 1 */

#define  GPIO_CFGLR_CNF1                       ((uint32_t)0x000000C0)        /* CNF1[1:0] bits (Port x configuration bits, pin 1) */
#define  GPIO_CFGLR_CNF1_0                     ((uint32_t)0x00000040)        /* Bit 0 */
#define  GPIO_CFGLR_CNF1_1                     ((uint32_t)0x00000080)        /* Bit 1 */

#define  GPIO_CFGLR_CNF2                       ((uint32_t)0x00000C00)        /* CNF2[1:0] bits (Port x configuration bits, pin 2) */
#define  GPIO_CFGLR_CNF2_0                     ((uint32_t)0x00000400)        /* Bit 0 */
#define  GPIO_CFGLR_CNF2_1                     ((uint32_t)0x00000800)        /* Bit 1 */

#define  GPIO_CFGLR_CNF3                       ((uint32_t)0x0000C000)        /* CNF3[1:0] bits (Port x configuration bits, pin 3) */
#define  GPIO_CFGLR_CNF3_0                     ((uint32_t)0x00004000)        /* Bit 0 */
#define  GPIO_CFGLR_CNF3_1                     ((uint32_t)0x00008000)        /* Bit 1 */

#define  GPIO_CFGLR_CNF4                       ((uint32_t)0x000C0000)        /* CNF4[1:0] bits (Port x configuration bits, pin 4) */
#define  GPIO_CFGLR_CNF4_0                     ((uint32_t)0x00040000)        /* Bit 0 */
#define  GPIO_CFGLR_CNF4_1                     ((uint32_t)0x00080000)        /* Bit 1 */

#define  GPIO_CFGLR_CNF5                       ((uint32_t)0x00C00000)        /* CNF5[1:0] bits (Port x configuration bits, pin 5) */
#define  GPIO_CFGLR_CNF5_0                     ((uint32_t)0x00400000)        /* Bit 0 */
#define  GPIO_CFGLR_CNF5_1                     ((uint32_t)0x00800000)        /* Bit 1 */

#define  GPIO_CFGLR_CNF6                       ((uint32_t)0x0C000000)        /* CNF6[1:0] bits (Port x configuration bits, pin 6) */
#define  GPIO_CFGLR_CNF6_0                     ((uint32_t)0x04000000)        /* Bit 0 */
#define  GPIO_CFGLR_CNF6_1                     ((uint32_t)0x08000000)        /* Bit 1 */

#define  GPIO_CFGLR_CNF7                       ((uint32_t)0xC0000000)        /* CNF7[1:0] bits (Port x configuration bits, pin 7) */
#define  GPIO_CFGLR_CNF7_0                     ((uint32_t)0x40000000)        /* Bit 0 */
#define  GPIO_CFGLR_CNF7_1                     ((uint32_t)0x80000000)        /* Bit 1 */

/*******************  Bit definition for GPIO_CFGHR register  *******************/
#define  GPIO_CFGHR_MODE                       ((uint32_t)0x33333333)        /* Port x mode bits */

#define  GPIO_CFGHR_MODE8                      ((uint32_t)0x00000003)        /* MODE8[1:0] bits (Port x mode bits, pin 8) */
#define  GPIO_CFGHR_MODE8_0                    ((uint32_t)0x00000001)        /* Bit 0 */
#define  GPIO_CFGHR_MODE8_1                    ((uint32_t)0x00000002)        /* Bit 1 */

#define  GPIO_CFGHR_MODE9                      ((uint32_t)0x00000030)        /* MODE9[1:0] bits (Port x mode bits, pin 9) */
#define  GPIO_CFGHR_MODE9_0                    ((uint32_t)0x00000010)        /* Bit 0 */
#define  GPIO_CFGHR_MODE9_1                    ((uint32_t)0x00000020)        /* Bit 1 */

#define  GPIO_CFGHR_MODE10                     ((uint32_t)0x00000300)        /* MODE10[1:0] bits (Port x mode bits, pin 10) */
#define  GPIO_CFGHR_MODE10_0                   ((uint32_t)0x00000100)        /* Bit 0 */
#define  GPIO_CFGHR_MODE10_1                   ((uint32_t)0x00000200)        /* Bit 1 */

#define  GPIO_CFGHR_MODE11                     ((uint32_t)0x00003000)        /* MODE11[1:0] bits (Port x mode bits, pin 11) */
#define  GPIO_CFGHR_MODE11_0                   ((uint32_t)0x00001000)        /* Bit 0 */
#define  GPIO_CFGHR_MODE11_1                   ((uint32_t)0x00002000)        /* Bit 1 */

#define  GPIO_CFGHR_MODE12                     ((uint32_t)0x00030000)        /* MODE12[1:0] bits (Port x mode bits, pin 12) */
#define  GPIO_CFGHR_MODE12_0                   ((uint32_t)0x00010000)        /* Bit 0 */
#define  GPIO_CFGHR_MODE12_1                   ((uint32_t)0x00020000)        /* Bit 1 */

#define  GPIO_CFGHR_MODE13                     ((uint32_t)0x00300000)        /* MODE13[1:0] bits (Port x mode bits, pin 13) */
#define  GPIO_CFGHR_MODE13_0                   ((uint32_t)0x00100000)        /* Bit 0 */
#define  GPIO_CFGHR_MODE13_1                   ((uint32_t)0x00200000)        /* Bit 1 */

#define  GPIO_CFGHR_MODE14                     ((uint32_t)0x03000000)        /* MODE14[1:0] bits (Port x mode bits, pin 14) */
#define  GPIO_CFGHR_MODE14_0                   ((uint32_t)0x01000000)        /* Bit 0 */
#define  GPIO_CFGHR_MODE14_1                   ((uint32_t)0x02000000)        /* Bit 1 */

#define  GPIO_CFGHR_MODE15                     ((uint32_t)0x30000000)        /* MODE15[1:0] bits (Port x mode bits, pin 15) */
#define  GPIO_CFGHR_MODE15_0                   ((uint32_t)0x10000000)        /* Bit 0 */
#define  GPIO_CFGHR_MODE15_1                   ((uint32_t)0x20000000)        /* Bit 1 */

#define  GPIO_CFGHR_CNF                        ((uint32_t)0xCCCCCCCC)        /* Port x configuration bits */

#define  GPIO_CFGHR_CNF8                       ((uint32_t)0x0000000C)        /* CNF8[1:0] bits (Port x configuration bits, pin 8) */
#define  GPIO_CFGHR_CNF8_0                     ((uint32_t)0x00000004)        /* Bit 0 */
#define  GPIO_CFGHR_CNF8_1                     ((uint32_t)0x00000008)        /* Bit 1 */

#define  GPIO_CFGHR_CNF9                       ((uint32_t)0x000000C0)        /* CNF9[1:0] bits (Port x configuration bits, pin 9) */
#define  GPIO_CFGHR_CNF9_0                     ((uint32_t)0x00000040)        /* Bit 0 */
#define  GPIO_CFGHR_CNF9_1                     ((uint32_t)0x00000080)        /* Bit 1 */

#define  GPIO_CFGHR_CNF10                      ((uint32_t)0x00000C00)        /* CNF10[1:0] bits (Port x configuration bits, pin 10) */
#define  GPIO_CFGHR_CNF10_0                    ((uint32_t)0x00000400)        /* Bit 0 */
#define  GPIO_CFGHR_CNF10_1                    ((uint32_t)0x00000800)        /* Bit 1 */

#define  GPIO_CFGHR_CNF11                      ((uint32_t)0x0000C000)        /* CNF11[1:0] bits (Port x configuration bits, pin 11) */
#define  GPIO_CFGHR_CNF11_0                    ((uint32_t)0x00004000)        /* Bit 0 */
#define  GPIO_CFGHR_CNF11_1                    ((uint32_t)0x00008000)        /* Bit 1 */

#define  GPIO_CFGHR_CNF12                      ((uint32_t)0x000C0000)        /* CNF12[1:0] bits (Port x configuration bits, pin 12) */
#define  GPIO_CFGHR_CNF12_0                    ((uint32_t)0x00040000)        /* Bit 0 */
#define  GPIO_CFGHR_CNF12_1                    ((uint32_t)0x00080000)        /* Bit 1 */

#define  GPIO_CFGHR_CNF13                      ((uint32_t)0x00C00000)        /* CNF13[1:0] bits (Port x configuration bits, pin 13) */
#define  GPIO_CFGHR_CNF13_0                    ((uint32_t)0x00400000)        /* Bit 0 */
#define  GPIO_CFGHR_CNF13_1                    ((uint32_t)0x00800000)        /* Bit 1 */

#define  GPIO_CFGHR_CNF14                      ((uint32_t)0x0C000000)        /* CNF14[1:0] bits (Port x configuration bits, pin 14) */
#define  GPIO_CFGHR_CNF14_0                    ((uint32_t)0x04000000)        /* Bit 0 */
#define  GPIO_CFGHR_CNF14_1                    ((uint32_t)0x08000000)        /* Bit 1 */

#define  GPIO_CFGHR_CNF15                      ((uint32_t)0xC0000000)        /* CNF15[1:0] bits (Port x configuration bits, pin 15) */
#define  GPIO_CFGHR_CNF15_0                    ((uint32_t)0x40000000)        /* Bit 0 */
#define  GPIO_CFGHR_CNF15_1                    ((uint32_t)0x80000000)        /* Bit 1 */

/*******************  Bit definition for GPIO_INDR register  *******************/
#define GPIO_INDR_IDR0                         ((uint16_t)0x0001)            /* Port input data, bit 0 */
#define GPIO_INDR_IDR1                         ((uint16_t)0x0002)            /* Port input data, bit 1 */
#define GPIO_INDR_IDR2                         ((uint16_t)0x0004)            /* Port input data, bit 2 */
#define GPIO_INDR_IDR3                         ((uint16_t)0x0008)            /* Port input data, bit 3 */
#define GPIO_INDR_IDR4                         ((uint16_t)0x0010)            /* Port input data, bit 4 */
#define GPIO_INDR_IDR5                         ((uint16_t)0x0020)            /* Port input data, bit 5 */
#define GPIO_INDR_IDR6                         ((uint16_t)0x0040)            /* Port input data, bit 6 */
#define GPIO_INDR_IDR7                         ((uint16_t)0x0080)            /* Port input data, bit 7 */
#define GPIO_INDR_IDR8                         ((uint16_t)0x0100)            /* Port input data, bit 8 */
#define GPIO_INDR_IDR9                         ((uint16_t)0x0200)            /* Port input data, bit 9 */
#define GPIO_INDR_IDR10                        ((uint16_t)0x0400)            /* Port input data, bit 10 */
#define GPIO_INDR_IDR11                        ((uint16_t)0x0800)            /* Port input data, bit 11 */
#define GPIO_INDR_IDR12                        ((uint16_t)0x1000)            /* Port input data, bit 12 */
#define GPIO_INDR_IDR13                        ((uint16_t)0x2000)            /* Port input data, bit 13 */
#define GPIO_INDR_IDR14                        ((uint16_t)0x4000)            /* Port input data, bit 14 */
#define GPIO_INDR_IDR15                        ((uint16_t)0x8000)            /* Port input data, bit 15 */

/*******************  Bit definition for GPIO_OUTDR register  *******************/
#define GPIO_OUTDR_ODR0                        ((uint16_t)0x0001)            /* Port output data, bit 0 */
#define GPIO_OUTDR_ODR1                        ((uint16_t)0x0002)            /* Port output data, bit 1 */
#define GPIO_OUTDR_ODR2                        ((uint16_t)0x0004)            /* Port output data, bit 2 */
#define GPIO_OUTDR_ODR3                        ((uint16_t)0x0008)            /* Port output data, bit 3 */
#define GPIO_OUTDR_ODR4                        ((uint16_t)0x0010)            /* Port output data, bit 4 */
#define GPIO_OUTDR_ODR5                        ((uint16_t)0x0020)            /* Port output data, bit 5 */
#define GPIO_OUTDR_ODR6                        ((uint16_t)0x0040)            /* Port output data, bit 6 */
#define GPIO_OUTDR_ODR7                        ((uint16_t)0x0080)            /* Port output data, bit 7 */
#define GPIO_OUTDR_ODR8                        ((uint16_t)0x0100)            /* Port output data, bit 8 */
#define GPIO_OUTDR_ODR9                        ((uint16_t)0x0200)            /* Port output data, bit 9 */
#define GPIO_OUTDR_ODR10                       ((uint16_t)0x0400)            /* Port output data, bit 10 */
#define GPIO_OUTDR_ODR11                       ((uint16_t)0x0800)            /* Port output data, bit 11 */
#define GPIO_OUTDR_ODR12                       ((uint16_t)0x1000)            /* Port output data, bit 12 */
#define GPIO_OUTDR_ODR13                       ((uint16_t)0x2000)            /* Port output data, bit 13 */
#define GPIO_OUTDR_ODR14                       ((uint16_t)0x4000)            /* Port output data, bit 14 */
#define GPIO_OUTDR_ODR15                       ((uint16_t)0x8000)            /* Port output data, bit 15 */

/******************  Bit definition for GPIO_BSHR register  *******************/
#define GPIO_BSHR_BS0                          ((uint32_t)0x00000001)        /* Port x Set bit 0 */
#define GPIO_BSHR_BS1                          ((uint32_t)0x00000002)        /* Port x Set bit 1 */
#define GPIO_BSHR_BS2                          ((uint32_t)0x00000004)        /* Port x Set bit 2 */
#define GPIO_BSHR_BS3                          ((uint32_t)0x00000008)        /* Port x Set bit 3 */
#define GPIO_BSHR_BS4                          ((uint32_t)0x00000010)        /* Port x Set bit 4 */
#define GPIO_BSHR_BS5                          ((uint32_t)0x00000020)        /* Port x Set bit 5 */
#define GPIO_BSHR_BS6                          ((uint32_t)0x00000040)        /* Port x Set bit 6 */
#define GPIO_BSHR_BS7                          ((uint32_t)0x00000080)        /* Port x Set bit 7 */
#define GPIO_BSHR_BS8                          ((uint32_t)0x00000100)        /* Port x Set bit 8 */
#define GPIO_BSHR_BS9                          ((uint32_t)0x00000200)        /* Port x Set bit 9 */
#define GPIO_BSHR_BS10                         ((uint32_t)0x00000400)        /* Port x Set bit 10 */
#define GPIO_BSHR_BS11                         ((uint32_t)0x00000800)        /* Port x Set bit 11 */
#define GPIO_BSHR_BS12                         ((uint32_t)0x00001000)        /* Port x Set bit 12 */
#define GPIO_BSHR_BS13                         ((uint32_t)0x00002000)        /* Port x Set bit 13 */
#define GPIO_BSHR_BS14                         ((uint32_t)0x00004000)        /* Port x Set bit 14 */
#define GPIO_BSHR_BS15                         ((uint32_t)0x00008000)        /* Port x Set bit 15 */

#define GPIO_BSHR_BR0                          ((uint32_t)0x00010000)        /* Port x Reset bit 0 */
#define GPIO_BSHR_BR1                          ((uint32_t)0x00020000)        /* Port x Reset bit 1 */
#define GPIO_BSHR_BR2                          ((uint32_t)0x00040000)        /* Port x Reset bit 2 */
#define GPIO_BSHR_BR3                          ((uint32_t)0x00080000)        /* Port x Reset bit 3 */
#define GPIO_BSHR_BR4                          ((uint32_t)0x00100000)        /* Port x Reset bit 4 */
#define GPIO_BSHR_BR5                          ((uint32_t)0x00200000)        /* Port x Reset bit 5 */
#define GPIO_BSHR_BR6                          ((uint32_t)0x00400000)        /* Port x Reset bit 6 */
#define GPIO_BSHR_BR7                          ((uint32_t)0x00800000)        /* Port x Reset bit 7 */
#define GPIO_BSHR_BR8                          ((uint32_t)0x01000000)        /* Port x Reset bit 8 */
#define GPIO_BSHR_BR9                          ((uint32_t)0x02000000)        /* Port x Reset bit 9 */
#define GPIO_BSHR_BR10                         ((uint32_t)0x04000000)        /* Port x Reset bit 10 */
#define GPIO_BSHR_BR11                         ((uint32_t)0x08000000)        /* Port x Reset bit 11 */
#define GPIO_BSHR_BR12                         ((uint32_t)0x10000000)        /* Port x Reset bit 12 */
#define GPIO_BSHR_BR13                         ((uint32_t)0x20000000)        /* Port x Reset bit 13 */
#define GPIO_BSHR_BR14                         ((uint32_t)0x40000000)        /* Port x Reset bit 14 */
#define GPIO_BSHR_BR15                         ((uint32_t)0x80000000)        /* Port x Reset bit 15 */

/*******************  Bit definition for GPIO_BCR register  *******************/
#define GPIO_BCR_BR0                           ((uint16_t)0x0001)            /* Port x Reset bit 0 */
#define GPIO_BCR_BR1                           ((uint16_t)0x0002)            /* Port x Reset bit 1 */
#define GPIO_BCR_BR2                           ((uint16_t)0x0004)            /* Port x Reset bit 2 */
#define GPIO_BCR_BR3                           ((uint16_t)0x0008)            /* Port x Reset bit 3 */
#define GPIO_BCR_BR4                           ((uint16_t)0x0010)            /* Port x Reset bit 4 */
#define GPIO_BCR_BR5                           ((uint16_t)0x0020)            /* Port x Reset bit 5 */
#define GPIO_BCR_BR6                           ((uint16_t)0x0040)            /* Port x Reset bit 6 */
#define GPIO_BCR_BR7                           ((uint16_t)0x0080)            /* Port x Reset bit 7 */
#define GPIO_BCR_BR8                           ((uint16_t)0x0100)            /* Port x Reset bit 8 */
#define GPIO_BCR_BR9                           ((uint16_t)0x0200)            /* Port x Reset bit 9 */
#define GPIO_BCR_BR10                          ((uint16_t)0x0400)            /* Port x Reset bit 10 */
#define GPIO_BCR_BR11                          ((uint16_t)0x0800)            /* Port x Reset bit 11 */
#define GPIO_BCR_BR12                          ((uint16_t)0x1000)            /* Port x Reset bit 12 */
#define GPIO_BCR_BR13                          ((uint16_t)0x2000)            /* Port x Reset bit 13 */
#define GPIO_BCR_BR14                          ((uint16_t)0x4000)            /* Port x Reset bit 14 */
#define GPIO_BCR_BR15                          ((uint16_t)0x8000)            /* Port x Reset bit 15 */

/******************  Bit definition for GPIO_LCKR register  *******************/
#define GPIO_LCK0                              ((uint32_t)0x00000001)        /* Port x Lock bit 0 */
#define GPIO_LCK1                              ((uint32_t)0x00000002)        /* Port x Lock bit 1 */
#define GPIO_LCK2                              ((uint32_t)0x00000004)        /* Port x Lock bit 2 */
#define GPIO_LCK3                              ((uint32_t)0x00000008)        /* Port x Lock bit 3 */
#define GPIO_LCK4                              ((uint32_t)0x00000010)        /* Port x Lock bit 4 */
#define GPIO_LCK5                              ((uint32_t)0x00000020)        /* Port x Lock bit 5 */
#define GPIO_LCK6                              ((uint32_t)0x00000040)        /* Port x Lock bit 6 */
#define GPIO_LCK7                              ((uint32_t)0x00000080)        /* Port x Lock bit 7 */
#define GPIO_LCK8                              ((uint32_t)0x00000100)        /* Port x Lock bit 8 */
#define GPIO_LCK9                              ((uint32_t)0x00000200)        /* Port x Lock bit 9 */
#define GPIO_LCK10                             ((uint32_t)0x00000400)        /* Port x Lock bit 10 */
#define GPIO_LCK11                             ((uint32_t)0x00000800)        /* Port x Lock bit 11 */
#define GPIO_LCK12                             ((uint32_t)0x00001000)        /* Port x Lock bit 12 */
#define GPIO_LCK13                             ((uint32_t)0x00002000)        /* Port x Lock bit 13 */
#define GPIO_LCK14                             ((uint32_t)0x00004000)        /* Port x Lock bit 14 */
#define GPIO_LCK15                             ((uint32_t)0x00008000)        /* Port x Lock bit 15 */
#define GPIO_LCKK                              ((uint32_t)0x00010000)        /* Lock key */


/******************************************************************************/
/*                      Inter-integrated Circuit Interface                    */
/******************************************************************************/

/*******************  Bit definition for I2C_CTLR1 register  ********************/
#define  I2C_CTLR1_PE                          ((uint16_t)0x0001)            /* Peripheral Enable */
#define  I2C_CTLR1_SMBUS                       ((uint16_t)0x0002)            /* SMBus Mode */
#define  I2C_CTLR1_SMBTYPE                     ((uint16_t)0x0008)            /* SMBus Type */
#define  I2C_CTLR1_ENARP                       ((uint16_t)0x0010)            /* ARP Enable */
#define  I2C_CTLR1_ENPEC                       ((uint16_t)0x0020)            /* PEC Enable */
#define  I2C_CTLR1_ENGC                        ((uint16_t)0x0040)            /* General Call Enable */
#define  I2C_CTLR1_NOSTRETCH                   ((uint16_t)0x0080)            /* Clock Stretching Disable (Slave mode) */
#define  I2C_CTLR1_START                       ((uint16_t)0x0100)            /* Start Generation */
#define  I2C_CTLR1_STOP                        ((uint16_t)0x0200)            /* Stop Generation */
#define  I2C_CTLR1_ACK                         ((uint16_t)0x0400)            /* Acknowledge Enable */
#define  I2C_CTLR1_POS                         ((uint16_t)0x0800)            /* Acknowledge/PEC Position (for data reception) */
#define  I2C_CTLR1_PEC                         ((uint16_t)0x1000)            /* Packet Error Checking */
#define  I2C_CTLR1_ALERT                       ((uint16_t)0x2000)            /* SMBus Alert */
#define  I2C_CTLR1_SWRST                       ((uint16_t)0x8000)            /* Software Reset */

/*******************  Bit definition for I2C_CTLR2 register  ********************/
#define  I2C_CTLR2_FREQ                        ((uint16_t)0x003F)            /* FREQ[5:0] bits (Peripheral Clock Frequency) */
#define  I2C_CTLR2_FREQ_0                      ((uint16_t)0x0001)            /* Bit 0 */
#define  I2C_CTLR2_FREQ_1                      ((uint16_t)0x0002)            /* Bit 1 */
#define  I2C_CTLR2_FREQ_2                      ((uint16_t)0x0004)            /* Bit 2 */
#define  I2C_CTLR2_FREQ_3                      ((uint16_t)0x0008)            /* Bit 3 */
#define  I2C_CTLR2_FREQ_4                      ((uint16_t)0x0010)            /* Bit 4 */
#define  I2C_CTLR2_FREQ_5                      ((uint16_t)0x0020)            /* Bit 5 */

#define  I2C_CTLR2_ITERREN                     ((uint16_t)0x0100)            /* Error Interrupt Enable */
#define  I2C_CTLR2_ITEVTEN                     ((uint16_t)0x0200)            /* Event Interrupt Enable */
#define  I2C_CTLR2_ITBUFEN                     ((uint16_t)0x0400)            /* Buffer Interrupt Enable */
#define  I2C_CTLR2_DMAEN                       ((uint16_t)0x0800)            /* DMA Requests Enable */
#define  I2C_CTLR2_LAST                        ((uint16_t)0x1000)            /* DMA Last Transfer */

/*******************  Bit definition for I2C_OADDR1 register  *******************/
#define  I2C_OADDR1_ADD1_7                     ((uint16_t)0x00FE)            /* Interface Address */
#define  I2C_OADDR1_ADD8_9                     ((uint16_t)0x0300)            /* Interface Address */

#define  I2C_OADDR1_ADD0                       ((uint16_t)0x0001)            /* Bit 0 */
#define  I2C_OADDR1_ADD1                       ((uint16_t)0x0002)            /* Bit 1 */
#define  I2C_OADDR1_ADD2                       ((uint16_t)0x0004)            /* Bit 2 */
#define  I2C_OADDR1_ADD3                       ((uint16_t)0x0008)            /* Bit 3 */
#define  I2C_OADDR1_ADD4                       ((uint16_t)0x0010)            /* Bit 4 */
#define  I2C_OADDR1_ADD5                       ((uint16_t)0x0020)            /* Bit 5 */
#define  I2C_OADDR1_ADD6                       ((uint16_t)0x0040)            /* Bit 6 */
#define  I2C_OADDR1_ADD7                       ((uint16_t)0x0080)            /* Bit 7 */
#define  I2C_OADDR1_ADD8                       ((uint16_t)0x0100)            /* Bit 8 */
#define  I2C_OADDR1_ADD9                       ((uint16_t)0x0200)            /* Bit 9 */

#define  I2C_OADDR1_ADDMODE                    ((uint16_t)0x8000)            /* Addressing Mode (Slave mode) */

/*******************  Bit definition for I2C_OADDR2 register  *******************/
#define  I2C_OADDR2_ENDUAL                     ((uint8_t)0x01)               /* Dual addressing mode enable */
#define  I2C_OADDR2_ADD2                       ((uint8_t)0xFE)               /* Interface address */

/********************  Bit definition for I2C_DATAR register  ********************/
#define  I2C_DR_DATAR                          ((uint8_t)0xFF)               /* 8-bit Data Register */

/*******************  Bit definition for I2C_STAR1 register  ********************/
#define  I2C_STAR1_SB                          ((uint16_t)0x0001)            /* Start Bit (Master mode) */
#define  I2C_STAR1_ADDR                        ((uint16_t)0x0002)            /* Address sent (master mode)/matched (slave mode) */
#define  I2C_STAR1_BTF                         ((uint16_t)0x0004)            /* Byte Transfer Finished */
#define  I2C_STAR1_ADD10                       ((uint16_t)0x0008)            /* 10-bit header sent (Master mode) */
#define  I2C_STAR1_STOPF                       ((uint16_t)0x0010)            /* Stop detection (Slave mode) */
#define  I2C_STAR1_RXNE                        ((uint16_t)0x0040)            /* Data Register not Empty (receivers) */
#define  I2C_STAR1_TXE                         ((uint16_t)0x0080)            /* Data Register Empty (transmitters) */
#define  I2C_STAR1_BERR                        ((uint16_t)0x0100)            /* Bus Error */
#define  I2C_STAR1_ARLO                        ((uint16_t)0x0200)            /* Arbitration Lost (master mode) */
#define  I2C_STAR1_AF                          ((uint16_t)0x0400)            /* Acknowledge Failure */
#define  I2C_STAR1_OVR                         ((uint16_t)0x0800)            /* Overrun/Underrun */
#define  I2C_STAR1_PECERR                      ((uint16_t)0x1000)            /* PEC Error in reception */
#define  I2C_STAR1_TIMEOUT                     ((uint16_t)0x4000)            /* Timeout or Tlow Error */
#define  I2C_STAR1_SMBALERT                    ((uint16_t)0x8000)            /* SMBus Alert */

/*******************  Bit definition for I2C_STAR2 register  ********************/
#define  I2C_STAR2_MSL                         ((uint16_t)0x0001)            /* Master/Slave */
#define  I2C_STAR2_BUSY                        ((uint16_t)0x0002)            /* Bus Busy */
#define  I2C_STAR2_TRA                         ((uint16_t)0x0004)            /* Transmitter/Receiver */
#define  I2C_STAR2_GENCALL                     ((uint16_t)0x0010)            /* General Call Address (Slave mode) */
#define  I2C_STAR2_SMBDEFAULT                  ((uint16_t)0x0020)            /* SMBus Device Default Address (Slave mode) */
#define  I2C_STAR2_SMBHOST                     ((uint16_t)0x0040)            /* SMBus Host Header (Slave mode) */
#define  I2C_STAR2_DUALF                       ((uint16_t)0x0080)            /* Dual Flag (Slave mode) */
#define  I2C_STAR2_PEC                         ((uint16_t)0xFF00)            /* Packet Error Checking Register */

/*******************  Bit definition for I2C_CKCFGR register  ********************/
#define  I2C_CKCFGR_CCR                        ((uint16_t)0x0FFF)            /* Clock Control Register in Fast/Standard mode (Master mode) */
#define  I2C_CKCFGR_DUTY                       ((uint16_t)0x4000)            /* Fast Mode Duty Cycle */
#define  I2C_CKCFGR_FS                         ((uint16_t)0x8000)            /* I2C Master Mode Selection */

/******************  Bit definition for I2C_RTR register  *******************/
#define  I2C_RTR_TRISE                         ((uint8_t)0x3F)               /* Maximum Rise Time in Fast/Standard mode (Master mode) */


/******************************************************************************/
/*                         Reset and Clock Control                            */
/******************************************************************************/

/********************  Bit definition for RCC_CTLR register  ********************/
#define  RCC_HSION                       ((uint32_t)0x00000001)        /* Internal High Speed clock enable */
#define  RCC_HSIRDY                      ((uint32_t)0x00000002)        /* Internal High Speed clock ready flag */
#define  RCC_HSITRIM                     ((uint32_t)0x000000F8)        /* Internal High Speed clock trimming */
#define  RCC_HSICAL                      ((uint32_t)0x0000FF00)        /* Internal High Speed clock Calibration */
#define  RCC_HSEON                       ((uint32_t)0x00010000)        /* External High Speed clock enable */
#define  RCC_HSERDY                      ((uint32_t)0x00020000)        /* External High Speed clock ready flag */
#define  RCC_HSEBYP                      ((uint32_t)0x00040000)        /* External High Speed clock Bypass */
#define  RCC_CSSON                       ((uint32_t)0x00080000)        /* Clock Security System enable */
#define  RCC_PLLON                       ((uint32_t)0x01000000)        /* PLL enable */
#define  RCC_PLLRDY                      ((uint32_t)0x02000000)        /* PLL clock ready flag */


/*******************  Bit definition for RCC_CFGR0 register  *******************/
#define  RCC_SW                          ((uint32_t)0x00000003)        /* SW[1:0] bits (System clock Switch) */
#define  RCC_SW_0                        ((uint32_t)0x00000001)        /* Bit 0 */
#define  RCC_SW_1                        ((uint32_t)0x00000002)        /* Bit 1 */

#define  RCC_SW_HSI                      ((uint32_t)0x00000000)        /* HSI selected as system clock */
#define  RCC_SW_HSE                      ((uint32_t)0x00000001)        /* HSE selected as system clock */
#define  RCC_SW_PLL                      ((uint32_t)0x00000002)        /* PLL selected as system clock */

#define  RCC_SWS                         ((uint32_t)0x0000000C)        /* SWS[1:0] bits (System Clock Switch Status) */
#define  RCC_SWS_0                       ((uint32_t)0x00000004)        /* Bit 0 */
#define  RCC_SWS_1                       ((uint32_t)0x00000008)        /* Bit 1 */

#define  RCC_SWS_HSI                     ((uint32_t)0x00000000)        /* HSI oscillator used as system clock */
#define  RCC_SWS_HSE                     ((uint32_t)0x00000004)        /* HSE oscillator used as system clock */
#define  RCC_SWS_PLL                     ((uint32_t)0x00000008)        /* PLL used as system clock */

#define  RCC_HPRE                        ((uint32_t)0x000000F0)        /* HPRE[3:0] bits (AHB prescaler) */
#define  RCC_HPRE_0                      ((uint32_t)0x00000010)        /* Bit 0 */
#define  RCC_HPRE_1                      ((uint32_t)0x00000020)        /* Bit 1 */
#define  RCC_HPRE_2                      ((uint32_t)0x00000040)        /* Bit 2 */
#define  RCC_HPRE_3                      ((uint32_t)0x00000080)        /* Bit 3 */

#define  RCC_HPRE_DIV1                   ((uint32_t)0x00000000)        /* SYSCLK not divided */
#define  RCC_HPRE_DIV2                   ((uint32_t)0x00000080)        /* SYSCLK divided by 2 */
#define  RCC_HPRE_DIV4                   ((uint32_t)0x00000090)        /* SYSCLK divided by 4 */
#define  RCC_HPRE_DIV8                   ((uint32_t)0x000000A0)        /* SYSCLK divided by 8 */
#define  RCC_HPRE_DIV16                  ((uint32_t)0x000000B0)        /* SYSCLK divided by 16 */
#define  RCC_HPRE_DIV64                  ((uint32_t)0x000000C0)        /* SYSCLK divided by 64 */
#define  RCC_HPRE_DIV128                 ((uint32_t)0x000000D0)        /* SYSCLK divided by 128 */
#define  RCC_HPRE_DIV256                 ((uint32_t)0x000000E0)        /* SYSCLK divided by 256 */
#define  RCC_HPRE_DIV512                 ((uint32_t)0x000000F0)        /* SYSCLK divided by 512 */

#define  RCC_PPRE1                       ((uint32_t)0x00000700)        /* PRE1[2:0] bits (APB1 prescaler) */
#define  RCC_PPRE1_0                     ((uint32_t)0x00000100)        /* Bit 0 */
#define  RCC_PPRE1_1                     ((uint32_t)0x00000200)        /* Bit 1 */
#define  RCC_PPRE1_2                     ((uint32_t)0x00000400)        /* Bit 2 */

#define  RCC_PPRE1_DIV1                  ((uint32_t)0x00000000)        /* HCLK not divided */
#define  RCC_PPRE1_DIV2                  ((uint32_t)0x00000400)        /* HCLK divided by 2 */
#define  RCC_PPRE1_DIV4                  ((uint32_t)0x00000500)        /* HCLK divided by 4 */
#define  RCC_PPRE1_DIV8                  ((uint32_t)0x00000600)        /* HCLK divided by 8 */
#define  RCC_PPRE1_DIV16                 ((uint32_t)0x00000700)        /* HCLK divided by 16 */

#define  RCC_PPRE2                       ((uint32_t)0x00003800)        /* PRE2[2:0] bits (APB2 prescaler) */
#define  RCC_PPRE2_0                     ((uint32_t)0x00000800)        /* Bit 0 */
#define  RCC_PPRE2_1                     ((uint32_t)0x00001000)        /* Bit 1 */
#define  RCC_PPRE2_2                     ((uint32_t)0x00002000)        /* Bit 2 */

#define  RCC_PPRE2_DIV1                  ((uint32_t)0x00000000)        /* HCLK not divided */
#define  RCC_PPRE2_DIV2                  ((uint32_t)0x00002000)        /* HCLK divided by 2 */
#define  RCC_PPRE2_DIV4                  ((uint32_t)0x00002800)        /* HCLK divided by 4 */
#define  RCC_PPRE2_DIV8                  ((uint32_t)0x00003000)        /* HCLK divided by 8 */
#define  RCC_PPRE2_DIV16                 ((uint32_t)0x00003800)        /* HCLK divided by 16 */

#define  RCC_ADCPRE                      ((uint32_t)0x0000C000)        /* ADCPRE[1:0] bits (ADC prescaler) */
#define  RCC_ADCPRE_0                    ((uint32_t)0x00004000)        /* Bit 0 */
#define  RCC_ADCPRE_1                    ((uint32_t)0x00008000)        /* Bit 1 */

#define  RCC_ADCPRE_DIV2                 ((uint32_t)0x00000000)        /* PCLK2 divided by 2 */
#define  RCC_ADCPRE_DIV4                 ((uint32_t)0x00004000)        /* PCLK2 divided by 4 */
#define  RCC_ADCPRE_DIV6                 ((uint32_t)0x00008000)        /* PCLK2 divided by 6 */
#define  RCC_ADCPRE_DIV8                 ((uint32_t)0x0000C000)        /* PCLK2 divided by 8 */

#define  RCC_PLLSRC                      ((uint32_t)0x00010000)        /* PLL entry clock source */

#define  RCC_PLLXTPRE                    ((uint32_t)0x00020000)        /* HSE divider for PLL entry */

#define  RCC_PLLMULL                     ((uint32_t)0x003C0000)        /* PLLMUL[3:0] bits (PLL multiplication factor) */
#define  RCC_PLLMULL_0                   ((uint32_t)0x00040000)        /* Bit 0 */
#define  RCC_PLLMULL_1                   ((uint32_t)0x00080000)        /* Bit 1 */
#define  RCC_PLLMULL_2                   ((uint32_t)0x00100000)        /* Bit 2 */
#define  RCC_PLLMULL_3                   ((uint32_t)0x00200000)        /* Bit 3 */

#define  RCC_PLLSRC_HSI_Div2             ((uint32_t)0x00000000)        /* HSI clock divided by 2 selected as PLL entry clock source */
#define  RCC_PLLSRC_HSE                  ((uint32_t)0x00010000)        /* HSE clock selected as PLL entry clock source */

#define  RCC_PLLXTPRE_HSE                ((uint32_t)0x00000000)        /* HSE clock not divided for PLL entry */
#define  RCC_PLLXTPRE_HSE_Div2           ((uint32_t)0x00020000)        /* HSE clock divided by 2 for PLL entry */

/* for other SPP32R20XX */
#define  RCC_PLLMULL2                    ((uint32_t)0x00000000)        /* PLL input clock*2 */
#define  RCC_PLLMULL3                    ((uint32_t)0x00040000)        /* PLL input clock*3 */
#define  RCC_PLLMULL4                    ((uint32_t)0x00080000)        /* PLL input clock*4 */
#define  RCC_PLLMULL5                    ((uint32_t)0x000C0000)        /* PLL input clock*5 */
#define  RCC_PLLMULL6                    ((uint32_t)0x00100000)        /* PLL input clock*6 */
#define  RCC_PLLMULL7                    ((uint32_t)0x00140000)        /* PLL input clock*7 */
#define  RCC_PLLMULL8                    ((uint32_t)0x00180000)        /* PLL input clock*8 */
#define  RCC_PLLMULL9                    ((uint32_t)0x001C0000)        /* PLL input clock*9 */
#define  RCC_PLLMULL10                   ((uint32_t)0x00200000)        /* PLL input clock10 */
#define  RCC_PLLMULL11                   ((uint32_t)0x00240000)        /* PLL input clock*11 */
#define  RCC_PLLMULL12                   ((uint32_t)0x00280000)        /* PLL input clock*12 */
#define  RCC_PLLMULL13                   ((uint32_t)0x002C0000)        /* PLL input clock*13 */
#define  RCC_PLLMULL14                   ((uint32_t)0x00300000)        /* PLL input clock*14 */
#define  RCC_PLLMULL15                   ((uint32_t)0x00340000)        /* PLL input clock*15 */
#define  RCC_PLLMULL16                   ((uint32_t)0x00380000)        /* PLL input clock*16 */
#define  RCC_PLLMULL18                   ((uint32_t)0x003C0000)        /* PLL input clock*18 */
/* for CH32V307 */
#define  RCC_PLLMULL18_EXTEN             ((uint32_t)0x00000000)        /* PLL input clock*18 */
#define  RCC_PLLMULL3_EXTEN              ((uint32_t)0x00040000)        /* PLL input clock*3 */
#define  RCC_PLLMULL4_EXTEN              ((uint32_t)0x00080000)        /* PLL input clock*4 */
#define  RCC_PLLMULL5_EXTEN              ((uint32_t)0x000C0000)        /* PLL input clock*5 */
#define  RCC_PLLMULL6_EXTEN              ((uint32_t)0x00100000)        /* PLL input clock*6 */
#define  RCC_PLLMULL7_EXTEN              ((uint32_t)0x00140000)        /* PLL input clock*7 */
#define  RCC_PLLMULL8_EXTEN              ((uint32_t)0x00180000)        /* PLL input clock*8 */
#define  RCC_PLLMULL9_EXTEN              ((uint32_t)0x001C0000)        /* PLL input clock*9 */
#define  RCC_PLLMULL10_EXTEN             ((uint32_t)0x00200000)        /* PLL input clock10 */
#define  RCC_PLLMULL11_EXTEN             ((uint32_t)0x00240000)        /* PLL input clock*11 */
#define  RCC_PLLMULL12_EXTEN             ((uint32_t)0x00280000)        /* PLL input clock*12 */
#define  RCC_PLLMULL13_EXTEN             ((uint32_t)0x002C0000)        /* PLL input clock*13 */
#define  RCC_PLLMULL14_EXTEN             ((uint32_t)0x00300000)        /* PLL input clock*14 */
#define  RCC_PLLMULL6_5_EXTEN            ((uint32_t)0x00340000)        /* PLL input clock*6.5 */
#define  RCC_PLLMULL15_EXTEN             ((uint32_t)0x00380000)        /* PLL input clock*15 */
#define  RCC_PLLMULL16_EXTEN             ((uint32_t)0x003C0000)        /* PLL input clock*16 */

#define  RCC_USBPRE                      ((uint32_t)0x00400000)        /* USB Device prescaler */

#define  RCC_CFGR0_MCO                   ((uint32_t)0x07000000)        /* MCO[2:0] bits (Microcontroller Clock Output) */
#define  RCC_MCO_0                       ((uint32_t)0x01000000)        /* Bit 0 */
#define  RCC_MCO_1                       ((uint32_t)0x02000000)        /* Bit 1 */
#define  RCC_MCO_2                       ((uint32_t)0x04000000)        /* Bit 2 */

#define  RCC_MCO_NOCLOCK                 ((uint32_t)0x00000000)        /* No clock */
#define  RCC_CFGR0_MCO_SYSCLK            ((uint32_t)0x04000000)        /* System clock selected as MCO source */
#define  RCC_CFGR0_MCO_HSI               ((uint32_t)0x05000000)        /* HSI clock selected as MCO source */
#define  RCC_CFGR0_MCO_HSE               ((uint32_t)0x06000000)        /* HSE clock selected as MCO source  */
#define  RCC_CFGR0_MCO_PLL               ((uint32_t)0x07000000)        /* PLL clock divided by 2 selected as MCO source */

/*******************  Bit definition for RCC_INTR register  ********************/
#define  RCC_LSIRDYF                     ((uint32_t)0x00000001)        /* LSI Ready Interrupt flag */
#define  RCC_LSERDYF                     ((uint32_t)0x00000002)        /* LSE Ready Interrupt flag */
#define  RCC_HSIRDYF                     ((uint32_t)0x00000004)        /* HSI Ready Interrupt flag */
#define  RCC_HSERDYF                     ((uint32_t)0x00000008)        /* HSE Ready Interrupt flag */
#define  RCC_PLLRDYF                     ((uint32_t)0x00000010)        /* PLL Ready Interrupt flag */
#define  RCC_CSSF                        ((uint32_t)0x00000080)        /* Clock Security System Interrupt flag */
#define  RCC_LSIRDYIE                    ((uint32_t)0x00000100)        /* LSI Ready Interrupt Enable */
#define  RCC_LSERDYIE                    ((uint32_t)0x00000200)        /* LSE Ready Interrupt Enable */
#define  RCC_HSIRDYIE                    ((uint32_t)0x00000400)        /* HSI Ready Interrupt Enable */
#define  RCC_HSERDYIE                    ((uint32_t)0x00000800)        /* HSE Ready Interrupt Enable */
#define  RCC_PLLRDYIE                    ((uint32_t)0x00001000)        /* PLL Ready Interrupt Enable */
#define  RCC_LSIRDYC                     ((uint32_t)0x00010000)        /* LSI Ready Interrupt Clear */
#define  RCC_LSERDYC                     ((uint32_t)0x00020000)        /* LSE Ready Interrupt Clear */
#define  RCC_HSIRDYC                     ((uint32_t)0x00040000)        /* HSI Ready Interrupt Clear */
#define  RCC_HSERDYC                     ((uint32_t)0x00080000)        /* HSE Ready Interrupt Clear */
#define  RCC_PLLRDYC                     ((uint32_t)0x00100000)        /* PLL Ready Interrupt Clear */
#define  RCC_CSSC                        ((uint32_t)0x00800000)        /* Clock Security System Interrupt Clear */


/*****************  Bit definition for RCC_APB2PRSTR register  *****************/
#define  RCC_AFIORST                     ((uint32_t)0x00000001)        /* Alternate Function I/O reset */
#define  RCC_IOPARST                     ((uint32_t)0x00000004)        /* I/O port A reset */
#define  RCC_IOPBRST                     ((uint32_t)0x00000008)        /* I/O port B reset */
#define  RCC_IOPCRST                     ((uint32_t)0x00000010)        /* I/O port C reset */
#define  RCC_IOPDRST                     ((uint32_t)0x00000020)        /* I/O port D reset */
#define  RCC_ADC1RST                     ((uint32_t)0x00000200)        /* ADC 1 interface reset */


#define  RCC_ADC2RST                     ((uint32_t)0x00000400)        /* ADC 2 interface reset */


#define  RCC_TIM1RST                     ((uint32_t)0x00000800)        /* TIM1 Timer reset */
#define  RCC_SPI1RST                     ((uint32_t)0x00001000)        /* SPI 1 reset */
#define  RCC_USART1RST                   ((uint32_t)0x00004000)        /* USART1 reset */

#define  RCC_IOPERST                     ((uint32_t)0x00000040)        /* I/O port E reset */

/*****************  Bit definition for RCC_APB1PRSTR register  *****************/
#define  RCC_TIM2RST                     ((uint32_t)0x00000001)        /* Timer 2 reset */
#define  RCC_TIM3RST                     ((uint32_t)0x00000002)        /* Timer 3 reset */
#define  RCC_WWDGRST                     ((uint32_t)0x00000800)        /* Window Watchdog reset */
#define  RCC_USART2RST                   ((uint32_t)0x00020000)        /* USART 2 reset */
#define  RCC_I2C1RST                     ((uint32_t)0x00200000)        /* I2C 1 reset */

#define  RCC_CAN1RST                     ((uint32_t)0x02000000)        /* CAN1 reset */


#define  RCC_BKPRST                      ((uint32_t)0x08000000)        /* Backup interface reset */
#define  RCC_PWRRST                      ((uint32_t)0x10000000)        /* Power interface reset */


#define  RCC_TIM4RST                     ((uint32_t)0x00000004)        /* Timer 4 reset */
#define  RCC_SPI2RST                     ((uint32_t)0x00004000)        /* SPI 2 reset */
#define  RCC_USART3RST                   ((uint32_t)0x00040000)        /* USART 3 reset */
#define  RCC_I2C2RST                     ((uint32_t)0x00400000)        /* I2C 2 reset */

#define  RCC_USBRST                      ((uint32_t)0x00800000)        /* USB Device reset */

/******************  Bit definition for RCC_AHBPCENR register  ******************/
#define  RCC_DMA1EN                      ((uint16_t)0x0001)            /* DMA1 clock enable */
#define  RCC_SRAMEN                      ((uint16_t)0x0004)            /* SRAM interface clock enable */
#define  RCC_FLITFEN                     ((uint16_t)0x0010)            /* FLITF clock enable */
#define  RCC_CRCEN                       ((uint16_t)0x0040)            /* CRC clock enable */
#define  RCC_USBHD                       ((uint16_t)0x1000)

/******************  Bit definition for RCC_APB2PCENR register  *****************/
#define  RCC_AFIOEN                      ((uint32_t)0x00000001)         /* Alternate Function I/O clock enable */
#define  RCC_IOPAEN                      ((uint32_t)0x00000004)         /* I/O port A clock enable */
#define  RCC_IOPBEN                      ((uint32_t)0x00000008)         /* I/O port B clock enable */
#define  RCC_IOPCEN                      ((uint32_t)0x00000010)         /* I/O port C clock enable */
#define  RCC_IOPDEN                      ((uint32_t)0x00000020)         /* I/O port D clock enable */
#define  RCC_ADC1EN                      ((uint32_t)0x00000200)         /* ADC 1 interface clock enable */

#define  RCC_ADC2EN                      ((uint32_t)0x00000400)         /* ADC 2 interface clock enable */


#define  RCC_TIM1EN                      ((uint32_t)0x00000800)         /* TIM1 Timer clock enable */
#define  RCC_SPI1EN                      ((uint32_t)0x00001000)         /* SPI 1 clock enable */
#define  RCC_USART1EN                    ((uint32_t)0x00004000)         /* USART1 clock enable */

/*****************  Bit definition for RCC_APB1PCENR register  ******************/
#define  RCC_TIM2EN                      ((uint32_t)0x00000001)        /* Timer 2 clock enabled*/
#define  RCC_TIM3EN                      ((uint32_t)0x00000002)        /* Timer 3 clock enable */
#define  RCC_WWDGEN                      ((uint32_t)0x00000800)        /* Window Watchdog clock enable */
#define  RCC_USART2EN                    ((uint32_t)0x00020000)        /* USART 2 clock enable */
#define  RCC_I2C1EN                      ((uint32_t)0x00200000)        /* I2C 1 clock enable */

#define  RCC_BKPEN                       ((uint32_t)0x08000000)        /* Backup interface clock enable */
#define  RCC_PWREN                       ((uint32_t)0x10000000)        /* Power interface clock enable */


#define  RCC_USBEN                       ((uint32_t)0x00800000)        /* USB Device clock enable */

/*******************  Bit definition for RCC_BDCTLR register  *******************/
#define  RCC_LSEON                       ((uint32_t)0x00000001)        /* External Low Speed oscillator enable */
#define  RCC_LSERDY                      ((uint32_t)0x00000002)        /* External Low Speed oscillator Ready */
#define  RCC_LSEBYP                      ((uint32_t)0x00000004)        /* External Low Speed oscillator Bypass */

#define  RCC_RTCSEL                      ((uint32_t)0x00000300)        /* RTCSEL[1:0] bits (RTC clock source selection) */
#define  RCC_RTCSEL_0                    ((uint32_t)0x00000100)        /* Bit 0 */
#define  RCC_RTCSEL_1                    ((uint32_t)0x00000200)        /* Bit 1 */

#define  RCC_RTCSEL_NOCLOCK              ((uint32_t)0x00000000)        /* No clock */
#define  RCC_RTCSEL_LSE                  ((uint32_t)0x00000100)        /* LSE oscillator clock used as RTC clock */
#define  RCC_RTCSEL_LSI                  ((uint32_t)0x00000200)        /* LSI oscillator clock used as RTC clock */
#define  RCC_RTCSEL_HSE                  ((uint32_t)0x00000300)        /* HSE oscillator clock divided by 128 used as RTC clock */

#define  RCC_RTCEN                       ((uint32_t)0x00008000)        /* RTC clock enable */
#define  RCC_BDRST                       ((uint32_t)0x00010000)        /* Backup domain software reset  */

/*******************  Bit definition for RCC_RSTSCKR register  ********************/
#define  RCC_LSION                       ((uint32_t)0x00000001)        /* Internal Low Speed oscillator enable */
#define  RCC_LSIRDY                      ((uint32_t)0x00000002)        /* Internal Low Speed oscillator Ready */
#define  RCC_RMVF                        ((uint32_t)0x01000000)        /* Remove reset flag */
#define  RCC_PINRSTF                     ((uint32_t)0x04000000)        /* PIN reset flag */
#define  RCC_PORRSTF                     ((uint32_t)0x08000000)        /* POR/PDR reset flag */
#define  RCC_SFTRSTF                     ((uint32_t)0x10000000)        /* Software Reset flag */
#define  RCC_IWDGRSTF                    ((uint32_t)0x20000000)        /* Independent Watchdog reset flag */
#define  RCC_WWDGRSTF                    ((uint32_t)0x40000000)        /* Window watchdog reset flag */
#define  RCC_LPWRRSTF                    ((uint32_t)0x80000000)        /* Low-Power reset flag */

/******************************************************************************/
/*                                    RNG                                     */
/******************************************************************************/
/********************  Bit definition for RNG_CR register  *******************/
#define  RNG_CR_RNGEN                         ((uint32_t)0x00000004)
#define  RNG_CR_IE                            ((uint32_t)0x00000008)

/********************  Bit definition for RNG_SR register  *******************/
#define  RNG_SR_DRDY                          ((uint32_t)0x00000001)
#define  RNG_SR_CECS                          ((uint32_t)0x00000002)
#define  RNG_SR_SECS                          ((uint32_t)0x00000004)
#define  RNG_SR_CEIS                          ((uint32_t)0x00000020)
#define  RNG_SR_SEIS                          ((uint32_t)0x00000040)

/******************************************************************************/
/*                             Real-Time Clock                                */
/******************************************************************************/

/*******************  Bit definition for RTC_CTLRH register  ********************/
#define  RTC_CTLRH_SECIE                     ((uint8_t)0x01)               /* Second Interrupt Enable */
#define  RTC_CTLRH_ALRIE                     ((uint8_t)0x02)               /* Alarm Interrupt Enable */
#define  RTC_CTLRH_OWIE                      ((uint8_t)0x04)               /* OverfloW Interrupt Enable */

/*******************  Bit definition for RTC_CTLRL register  ********************/
#define  RTC_CTLRL_SECF                      ((uint8_t)0x01)               /* Second Flag */
#define  RTC_CTLRL_ALRF                      ((uint8_t)0x02)               /* Alarm Flag */
#define  RTC_CTLRL_OWF                       ((uint8_t)0x04)               /* OverfloW Flag */
#define  RTC_CTLRL_RSF                       ((uint8_t)0x08)               /* Registers Synchronized Flag */
#define  RTC_CTLRL_CNF                       ((uint8_t)0x10)               /* Configuration Flag */
#define  RTC_CTLRL_RTOFF                     ((uint8_t)0x20)               /* RTC operation OFF */

/*******************  Bit definition for RTC_PSCH register  *******************/
#define  RTC_PSCH_PRL                        ((uint16_t)0x000F)            /* RTC Prescaler Reload Value High */

/*******************  Bit definition for RTC_PRLL register  *******************/
#define  RTC_PSCL_PRL                        ((uint16_t)0xFFFF)            /* RTC Prescaler Reload Value Low */

/*******************  Bit definition for RTC_DIVH register  *******************/
#define  RTC_DIVH_RTC_DIV                    ((uint16_t)0x000F)            /* RTC Clock Divider High */

/*******************  Bit definition for RTC_DIVL register  *******************/
#define  RTC_DIVL_RTC_DIV                    ((uint16_t)0xFFFF)            /* RTC Clock Divider Low */

/*******************  Bit definition for RTC_CNTH register  *******************/
#define  RTC_CNTH_RTC_CNT                    ((uint16_t)0xFFFF)            /* RTC Counter High */

/*******************  Bit definition for RTC_CNTL register  *******************/
#define  RTC_CNTL_RTC_CNT                    ((uint16_t)0xFFFF)            /* RTC Counter Low */

/*******************  Bit definition for RTC_ALRMH register  *******************/
#define  RTC_ALRMH_RTC_ALRM                  ((uint16_t)0xFFFF)            /* RTC Alarm High */

/*******************  Bit definition for RTC_ALRML register  *******************/
#define  RTC_ALRML_RTC_ALRM                  ((uint16_t)0xFFFF)            /* RTC Alarm Low */

/******************************************************************************/
/*                        Serial Peripheral Interface                         */
/******************************************************************************/

/*******************  Bit definition for SPI_CTLR1 register  ********************/
#define  SPI_CTLR1_CPHA                      ((uint16_t)0x0001)            /* Clock Phase */
#define  SPI_CTLR1_CPOL                      ((uint16_t)0x0002)            /* Clock Polarity */
#define  SPI_CTLR1_MSTR                      ((uint16_t)0x0004)            /* Master Selection */

#define  SPI_CTLR1_BR                        ((uint16_t)0x0038)            /* BR[2:0] bits (Baud Rate Control) */
#define  SPI_CTLR1_BR_0                      ((uint16_t)0x0008)            /* Bit 0 */
#define  SPI_CTLR1_BR_1                      ((uint16_t)0x0010)            /* Bit 1 */
#define  SPI_CTLR1_BR_2                      ((uint16_t)0x0020)            /* Bit 2 */

#define  SPI_CTLR1_SPE                       ((uint16_t)0x0040)            /* SPI Enable */
#define  SPI_CTLR1_LSBFIRST                  ((uint16_t)0x0080)            /* Frame Format */
#define  SPI_CTLR1_SSI                       ((uint16_t)0x0100)            /* Internal slave select */
#define  SPI_CTLR1_SSM                       ((uint16_t)0x0200)            /* Software slave management */
#define  SPI_CTLR1_RXONLY                    ((uint16_t)0x0400)            /* Receive only */
#define  SPI_CTLR1_DFF                       ((uint16_t)0x0800)            /* Data Frame Format */
#define  SPI_CTLR1_CRCNEXT                   ((uint16_t)0x1000)            /* Transmit CRC next */
#define  SPI_CTLR1_CRCEN                     ((uint16_t)0x2000)            /* Hardware CRC calculation enable */
#define  SPI_CTLR1_BIDIOE                    ((uint16_t)0x4000)            /* Output enable in bidirectional mode */
#define  SPI_CTLR1_BIDIMODE                  ((uint16_t)0x8000)            /* Bidirectional data mode enable */

/*******************  Bit definition for SPI_CTLR2 register  ********************/
#define  SPI_CTLR2_RXDMAEN                   ((uint8_t)0x01)               /* Rx Buffer DMA Enable */
#define  SPI_CTLR2_TXDMAEN                   ((uint8_t)0x02)               /* Tx Buffer DMA Enable */
#define  SPI_CTLR2_SSOE                      ((uint8_t)0x04)               /* SS Output Enable */
#define  SPI_CTLR2_ERRIE                     ((uint8_t)0x20)               /* Error Interrupt Enable */
#define  SPI_CTLR2_RXNEIE                    ((uint8_t)0x40)               /* RX buffer Not Empty Interrupt Enable */
#define  SPI_CTLR2_TXEIE                     ((uint8_t)0x80)               /* Tx buffer Empty Interrupt Enable */

/********************  Bit definition for SPI_STATR register  ********************/
#define  SPI_STATR_RXNE                      ((uint8_t)0x01)               /* Receive buffer Not Empty */
#define  SPI_STATR_TXE                       ((uint8_t)0x02)               /* Transmit buffer Empty */
#define  SPI_STATR_CHSIDE                    ((uint8_t)0x04)               /* Channel side */
#define  SPI_STATR_UDR                       ((uint8_t)0x08)               /* Underrun flag */
#define  SPI_STATR_CRCERR                    ((uint8_t)0x10)               /* CRC Error flag */
#define  SPI_STATR_MODF                      ((uint8_t)0x20)               /* Mode fault */
#define  SPI_STATR_OVR                       ((uint8_t)0x40)               /* Overrun flag */
#define  SPI_STATR_BSY                       ((uint8_t)0x80)               /* Busy flag */

/********************  Bit definition for SPI_DATAR register  ********************/
#define  SPI_DATAR_DR                        ((uint16_t)0xFFFF)            /* Data Register */

/*******************  Bit definition for SPI_CRCR register  ******************/
#define  SPI_CRCR_CRCPOLY                    ((uint16_t)0xFFFF)            /* CRC polynomial register */

/******************  Bit definition for SPI_RCRCR register  ******************/
#define  SPI_RCRCR_RXCRC                     ((uint16_t)0xFFFF)            /* Rx CRC Register */

/******************  Bit definition for SPI_TCRCR register  ******************/
#define  SPI_TCRCR_TXCRC                     ((uint16_t)0xFFFF)            /* Tx CRC Register */

/******************  Bit definition for SPI_I2SCFGR register  *****************/
#define  SPI_I2SCFGR_CHLEN                   ((uint16_t)0x0001)            /* Channel length (number of bits per audio channel) */

#define  SPI_I2SCFGR_DATLEN                  ((uint16_t)0x0006)            /* DATLEN[1:0] bits (Data length to be transferred) */
#define  SPI_I2SCFGR_DATLEN_0                ((uint16_t)0x0002)            /* Bit 0 */
#define  SPI_I2SCFGR_DATLEN_1                ((uint16_t)0x0004)            /* Bit 1 */

#define  SPI_I2SCFGR_CKPOL                   ((uint16_t)0x0008)            /* steady state clock polarity */

#define  SPI_I2SCFGR_I2SSTD                  ((uint16_t)0x0030)            /* I2SSTD[1:0] bits (I2S standard selection) */
#define  SPI_I2SCFGR_I2SSTD_0                ((uint16_t)0x0010)            /* Bit 0 */
#define  SPI_I2SCFGR_I2SSTD_1                ((uint16_t)0x0020)            /* Bit 1 */

#define  SPI_I2SCFGR_PCMSYNC                 ((uint16_t)0x0080)            /* PCM frame synchronization */

#define  SPI_I2SCFGR_I2SCFG                  ((uint16_t)0x0300)            /* I2SCFG[1:0] bits (I2S configuration mode) */
#define  SPI_I2SCFGR_I2SCFG_0                ((uint16_t)0x0100)            /* Bit 0 */
#define  SPI_I2SCFGR_I2SCFG_1                ((uint16_t)0x0200)            /* Bit 1 */

#define  SPI_I2SCFGR_I2SE                    ((uint16_t)0x0400)            /* I2S Enable */
#define  SPI_I2SCFGR_I2SMOD                  ((uint16_t)0x0800)            /* I2S mode selection */

/******************  Bit definition for SPI_I2SPR register  *******************/
#define  SPI_I2SPR_I2SDIV                    ((uint16_t)0x00FF)            /* I2S Linear prescaler */
#define  SPI_I2SPR_ODD                       ((uint16_t)0x0100)            /* Odd factor for the prescaler */
#define  SPI_I2SPR_MCKOE                     ((uint16_t)0x0200)            /* Master Clock Output Enable */

/******************************************************************************/
/*                                    TIM                                     */
/******************************************************************************/

/*******************  Bit definition for TIM_CTLR1 register  ********************/
#define  TIM_CEN                         ((uint16_t)0x0001)            /* Counter enable */
#define  TIM_UDIS                        ((uint16_t)0x0002)            /* Update disable */
#define  TIM_URS                         ((uint16_t)0x0004)            /* Update request source */
#define  TIM_OPM                         ((uint16_t)0x0008)            /* One pulse mode */
#define  TIM_DIR                         ((uint16_t)0x0010)            /* Direction */

#define  TIM_CMS                         ((uint16_t)0x0060)            /* CMS[1:0] bits (Center-aligned mode selection) */
#define  TIM_CMS_0                       ((uint16_t)0x0020)            /* Bit 0 */
#define  TIM_CMS_1                       ((uint16_t)0x0040)            /* Bit 1 */

#define  TIM_ARPE                        ((uint16_t)0x0080)            /* Auto-reload preload enable */

#define  TIM_CTLR1_CKD                   ((uint16_t)0x0300)            /* CKD[1:0] bits (clock division) */
#define  TIM_CKD_0                       ((uint16_t)0x0100)            /* Bit 0 */
#define  TIM_CKD_1                       ((uint16_t)0x0200)            /* Bit 1 */

/*******************  Bit definition for TIM_CTLR2 register  ********************/
#define  TIM_CCPC                        ((uint16_t)0x0001)            /* Capture/Compare Preloaded Control */
#define  TIM_CCUS                        ((uint16_t)0x0004)            /* Capture/Compare Control Update Selection */
#define  TIM_CCDS                        ((uint16_t)0x0008)            /* Capture/Compare DMA Selection */

#define  TIM_MMS                         ((uint16_t)0x0070)            /* MMS[2:0] bits (Master Mode Selection) */
#define  TIM_MMS_0                       ((uint16_t)0x0010)            /* Bit 0 */
#define  TIM_MMS_1                       ((uint16_t)0x0020)            /* Bit 1 */
#define  TIM_MMS_2                       ((uint16_t)0x0040)            /* Bit 2 */

#define  TIM_TI1S                        ((uint16_t)0x0080)            /* TI1 Selection */
#define  TIM_OIS1                        ((uint16_t)0x0100)            /* Output Idle state 1 (OC1 output) */
#define  TIM_OIS1N                       ((uint16_t)0x0200)            /* Output Idle state 1 (OC1N output) */
#define  TIM_OIS2                        ((uint16_t)0x0400)            /* Output Idle state 2 (OC2 output) */
#define  TIM_OIS2N                       ((uint16_t)0x0800)            /* Output Idle state 2 (OC2N output) */
#define  TIM_OIS3                        ((uint16_t)0x1000)            /* Output Idle state 3 (OC3 output) */
#define  TIM_OIS3N                       ((uint16_t)0x2000)            /* Output Idle state 3 (OC3N output) */
#define  TIM_OIS4                        ((uint16_t)0x4000)            /* Output Idle state 4 (OC4 output) */

/*******************  Bit definition for TIM_SMCFGR register  *******************/
#define  TIM_SMS                         ((uint16_t)0x0007)            /* SMS[2:0] bits (Slave mode selection) */
#define  TIM_SMS_0                       ((uint16_t)0x0001)            /* Bit 0 */
#define  TIM_SMS_1                       ((uint16_t)0x0002)            /* Bit 1 */
#define  TIM_SMS_2                       ((uint16_t)0x0004)            /* Bit 2 */

#define  TIM_TS                          ((uint16_t)0x0070)            /* TS[2:0] bits (Trigger selection) */
#define  TIM_TS_0                        ((uint16_t)0x0010)            /* Bit 0 */
#define  TIM_TS_1                        ((uint16_t)0x0020)            /* Bit 1 */
#define  TIM_TS_2                        ((uint16_t)0x0040)            /* Bit 2 */

#define  TIM_MSM                         ((uint16_t)0x0080)            /* Master/slave mode */

#define  TIM_ETF                         ((uint16_t)0x0F00)            /* ETF[3:0] bits (External trigger filter) */
#define  TIM_ETF_0                       ((uint16_t)0x0100)            /* Bit 0 */
#define  TIM_ETF_1                       ((uint16_t)0x0200)            /* Bit 1 */
#define  TIM_ETF_2                       ((uint16_t)0x0400)            /* Bit 2 */
#define  TIM_ETF_3                       ((uint16_t)0x0800)            /* Bit 3 */

#define  TIM_ETPS                        ((uint16_t)0x3000)            /* ETPS[1:0] bits (External trigger prescaler) */
#define  TIM_ETPS_0                      ((uint16_t)0x1000)            /* Bit 0 */
#define  TIM_ETPS_1                      ((uint16_t)0x2000)            /* Bit 1 */

#define  TIM_ECE                         ((uint16_t)0x4000)            /* External clock enable */
#define  TIM_ETP                         ((uint16_t)0x8000)            /* External trigger polarity */

/*******************  Bit definition for TIM_DMAINTENR register  *******************/
#define  TIM_UIE                         ((uint16_t)0x0001)            /* Update interrupt enable */
#define  TIM_CC1IE                       ((uint16_t)0x0002)            /* Capture/Compare 1 interrupt enable */
#define  TIM_CC2IE                       ((uint16_t)0x0004)            /* Capture/Compare 2 interrupt enable */
#define  TIM_CC3IE                       ((uint16_t)0x0008)            /* Capture/Compare 3 interrupt enable */
#define  TIM_CC4IE                       ((uint16_t)0x0010)            /* Capture/Compare 4 interrupt enable */
#define  TIM_COMIE                       ((uint16_t)0x0020)            /* COM interrupt enable */
#define  TIM_TIE                         ((uint16_t)0x0040)            /* Trigger interrupt enable */
#define  TIM_BIE                         ((uint16_t)0x0080)            /* Break interrupt enable */
#define  TIM_UDE                         ((uint16_t)0x0100)            /* Update DMA request enable */
#define  TIM_CC1DE                       ((uint16_t)0x0200)            /* Capture/Compare 1 DMA request enable */
#define  TIM_CC2DE                       ((uint16_t)0x0400)            /* Capture/Compare 2 DMA request enable */
#define  TIM_CC3DE                       ((uint16_t)0x0800)            /* Capture/Compare 3 DMA request enable */
#define  TIM_CC4DE                       ((uint16_t)0x1000)            /* Capture/Compare 4 DMA request enable */
#define  TIM_COMDE                       ((uint16_t)0x2000)            /* COM DMA request enable */
#define  TIM_TDE                         ((uint16_t)0x4000)            /* Trigger DMA request enable */

/********************  Bit definition for TIM_INTFR register  ********************/
#define  TIM_UIF                         ((uint16_t)0x0001)            /* Update interrupt Flag */
#define  TIM_CC1IF                       ((uint16_t)0x0002)            /* Capture/Compare 1 interrupt Flag */
#define  TIM_CC2IF                       ((uint16_t)0x0004)            /* Capture/Compare 2 interrupt Flag */
#define  TIM_CC3IF                       ((uint16_t)0x0008)            /* Capture/Compare 3 interrupt Flag */
#define  TIM_CC4IF                       ((uint16_t)0x0010)            /* Capture/Compare 4 interrupt Flag */
#define  TIM_COMIF                       ((uint16_t)0x0020)            /* COM interrupt Flag */
#define  TIM_TIF                         ((uint16_t)0x0040)            /* Trigger interrupt Flag */
#define  TIM_BIF                         ((uint16_t)0x0080)            /* Break interrupt Flag */
#define  TIM_CC1OF                       ((uint16_t)0x0200)            /* Capture/Compare 1 Overcapture Flag */
#define  TIM_CC2OF                       ((uint16_t)0x0400)            /* Capture/Compare 2 Overcapture Flag */
#define  TIM_CC3OF                       ((uint16_t)0x0800)            /* Capture/Compare 3 Overcapture Flag */
#define  TIM_CC4OF                       ((uint16_t)0x1000)            /* Capture/Compare 4 Overcapture Flag */

/*******************  Bit definition for TIM_SWEVGR register  ********************/
#define  TIM_UG                          ((uint8_t)0x01)               /* Update Generation */
#define  TIM_CC1G                        ((uint8_t)0x02)               /* Capture/Compare 1 Generation */
#define  TIM_CC2G                        ((uint8_t)0x04)               /* Capture/Compare 2 Generation */
#define  TIM_CC3G                        ((uint8_t)0x08)               /* Capture/Compare 3 Generation */
#define  TIM_CC4G                        ((uint8_t)0x10)               /* Capture/Compare 4 Generation */
#define  TIM_COMG                        ((uint8_t)0x20)               /* Capture/Compare Control Update Generation */
#define  TIM_TG                          ((uint8_t)0x40)               /* Trigger Generation */
#define  TIM_BG                          ((uint8_t)0x80)               /* Break Generation */

/******************  Bit definition for TIM_CHCTLR1 register  *******************/
#define  TIM_CC1S                        ((uint16_t)0x0003)            /* CC1S[1:0] bits (Capture/Compare 1 Selection) */
#define  TIM_CC1S_0                      ((uint16_t)0x0001)            /* Bit 0 */
#define  TIM_CC1S_1                      ((uint16_t)0x0002)            /* Bit 1 */

#define  TIM_OC1FE                       ((uint16_t)0x0004)            /* Output Compare 1 Fast enable */
#define  TIM_OC1PE                       ((uint16_t)0x0008)            /* Output Compare 1 Preload enable */

#define  TIM_OC1M                        ((uint16_t)0x0070)            /* OC1M[2:0] bits (Output Compare 1 Mode) */
#define  TIM_OC1M_0                      ((uint16_t)0x0010)            /* Bit 0 */
#define  TIM_OC1M_1                      ((uint16_t)0x0020)            /* Bit 1 */
#define  TIM_OC1M_2                      ((uint16_t)0x0040)            /* Bit 2 */

#define  TIM_OC1CE                       ((uint16_t)0x0080)            /* Output Compare 1Clear Enable */

#define  TIM_CC2S                        ((uint16_t)0x0300)            /* CC2S[1:0] bits (Capture/Compare 2 Selection) */
#define  TIM_CC2S_0                      ((uint16_t)0x0100)            /* Bit 0 */
#define  TIM_CC2S_1                      ((uint16_t)0x0200)            /* Bit 1 */

#define  TIM_OC2FE                       ((uint16_t)0x0400)            /* Output Compare 2 Fast enable */
#define  TIM_OC2PE                       ((uint16_t)0x0800)            /* Output Compare 2 Preload enable */

#define  TIM_OC2M                        ((uint16_t)0x7000)            /* OC2M[2:0] bits (Output Compare 2 Mode) */
#define  TIM_OC2M_0                      ((uint16_t)0x1000)            /* Bit 0 */
#define  TIM_OC2M_1                      ((uint16_t)0x2000)            /* Bit 1 */
#define  TIM_OC2M_2                      ((uint16_t)0x4000)            /* Bit 2 */

#define  TIM_OC2CE                       ((uint16_t)0x8000)            /* Output Compare 2 Clear Enable */


#define  TIM_IC1PSC                      ((uint16_t)0x000C)            /* IC1PSC[1:0] bits (Input Capture 1 Prescaler) */
#define  TIM_IC1PSC_0                    ((uint16_t)0x0004)            /* Bit 0 */
#define  TIM_IC1PSC_1                    ((uint16_t)0x0008)            /* Bit 1 */

#define  TIM_IC1F                        ((uint16_t)0x00F0)            /* IC1F[3:0] bits (Input Capture 1 Filter) */
#define  TIM_IC1F_0                      ((uint16_t)0x0010)            /* Bit 0 */
#define  TIM_IC1F_1                      ((uint16_t)0x0020)            /* Bit 1 */
#define  TIM_IC1F_2                      ((uint16_t)0x0040)            /* Bit 2 */
#define  TIM_IC1F_3                      ((uint16_t)0x0080)            /* Bit 3 */

#define  TIM_IC2PSC                      ((uint16_t)0x0C00)            /* IC2PSC[1:0] bits (Input Capture 2 Prescaler) */
#define  TIM_IC2PSC_0                    ((uint16_t)0x0400)            /* Bit 0 */
#define  TIM_IC2PSC_1                    ((uint16_t)0x0800)            /* Bit 1 */

#define  TIM_IC2F                        ((uint16_t)0xF000)            /* IC2F[3:0] bits (Input Capture 2 Filter) */
#define  TIM_IC2F_0                      ((uint16_t)0x1000)            /* Bit 0 */
#define  TIM_IC2F_1                      ((uint16_t)0x2000)            /* Bit 1 */
#define  TIM_IC2F_2                      ((uint16_t)0x4000)            /* Bit 2 */
#define  TIM_IC2F_3                      ((uint16_t)0x8000)            /* Bit 3 */

/******************  Bit definition for TIM_CHCTLR2 register  *******************/
#define  TIM_CC3S                        ((uint16_t)0x0003)            /* CC3S[1:0] bits (Capture/Compare 3 Selection) */
#define  TIM_CC3S_0                      ((uint16_t)0x0001)            /* Bit 0 */
#define  TIM_CC3S_1                      ((uint16_t)0x0002)            /* Bit 1 */

#define  TIM_OC3FE                       ((uint16_t)0x0004)            /* Output Compare 3 Fast enable */
#define  TIM_OC3PE                       ((uint16_t)0x0008)            /* Output Compare 3 Preload enable */

#define  TIM_OC3M                        ((uint16_t)0x0070)            /* OC3M[2:0] bits (Output Compare 3 Mode) */
#define  TIM_OC3M_0                      ((uint16_t)0x0010)            /* Bit 0 */
#define  TIM_OC3M_1                      ((uint16_t)0x0020)            /* Bit 1 */
#define  TIM_OC3M_2                      ((uint16_t)0x0040)            /* Bit 2 */

#define  TIM_OC3CE                       ((uint16_t)0x0080)            /* Output Compare 3 Clear Enable */

#define  TIM_CC4S                        ((uint16_t)0x0300)            /* CC4S[1:0] bits (Capture/Compare 4 Selection) */
#define  TIM_CC4S_0                      ((uint16_t)0x0100)            /* Bit 0 */
#define  TIM_CC4S_1                      ((uint16_t)0x0200)            /* Bit 1 */

#define  TIM_OC4FE                       ((uint16_t)0x0400)            /* Output Compare 4 Fast enable */
#define  TIM_OC4PE                       ((uint16_t)0x0800)            /* Output Compare 4 Preload enable */

#define  TIM_OC4M                        ((uint16_t)0x7000)            /* OC4M[2:0] bits (Output Compare 4 Mode) */
#define  TIM_OC4M_0                      ((uint16_t)0x1000)            /* Bit 0 */
#define  TIM_OC4M_1                      ((uint16_t)0x2000)            /* Bit 1 */
#define  TIM_OC4M_2                      ((uint16_t)0x4000)            /* Bit 2 */

#define  TIM_OC4CE                       ((uint16_t)0x8000)            /* Output Compare 4 Clear Enable */


#define  TIM_IC3PSC                      ((uint16_t)0x000C)            /* IC3PSC[1:0] bits (Input Capture 3 Prescaler) */
#define  TIM_IC3PSC_0                    ((uint16_t)0x0004)            /* Bit 0 */
#define  TIM_IC3PSC_1                    ((uint16_t)0x0008)            /* Bit 1 */

#define  TIM_IC3F                        ((uint16_t)0x00F0)            /* IC3F[3:0] bits (Input Capture 3 Filter) */
#define  TIM_IC3F_0                      ((uint16_t)0x0010)            /* Bit 0 */
#define  TIM_IC3F_1                      ((uint16_t)0x0020)            /* Bit 1 */
#define  TIM_IC3F_2                      ((uint16_t)0x0040)            /* Bit 2 */
#define  TIM_IC3F_3                      ((uint16_t)0x0080)            /* Bit 3 */

#define  TIM_IC4PSC                      ((uint16_t)0x0C00)            /* IC4PSC[1:0] bits (Input Capture 4 Prescaler) */
#define  TIM_IC4PSC_0                    ((uint16_t)0x0400)            /* Bit 0 */
#define  TIM_IC4PSC_1                    ((uint16_t)0x0800)            /* Bit 1 */

#define  TIM_IC4F                        ((uint16_t)0xF000)            /* IC4F[3:0] bits (Input Capture 4 Filter) */
#define  TIM_IC4F_0                      ((uint16_t)0x1000)            /* Bit 0 */
#define  TIM_IC4F_1                      ((uint16_t)0x2000)            /* Bit 1 */
#define  TIM_IC4F_2                      ((uint16_t)0x4000)            /* Bit 2 */
#define  TIM_IC4F_3                      ((uint16_t)0x8000)            /* Bit 3 */

/*******************  Bit definition for TIM_CCER register  *******************/
#define  TIM_CC1E                        ((uint16_t)0x0001)            /* Capture/Compare 1 output enable */
#define  TIM_CC1P                        ((uint16_t)0x0002)            /* Capture/Compare 1 output Polarity */
#define  TIM_CC1NE                       ((uint16_t)0x0004)            /* Capture/Compare 1 Complementary output enable */
#define  TIM_CC1NP                       ((uint16_t)0x0008)            /* Capture/Compare 1 Complementary output Polarity */
#define  TIM_CC2E                        ((uint16_t)0x0010)            /* Capture/Compare 2 output enable */
#define  TIM_CC2P                        ((uint16_t)0x0020)            /* Capture/Compare 2 output Polarity */
#define  TIM_CC2NE                       ((uint16_t)0x0040)            /* Capture/Compare 2 Complementary output enable */
#define  TIM_CC2NP                       ((uint16_t)0x0080)            /* Capture/Compare 2 Complementary output Polarity */
#define  TIM_CC3E                        ((uint16_t)0x0100)            /* Capture/Compare 3 output enable */
#define  TIM_CC3P                        ((uint16_t)0x0200)            /* Capture/Compare 3 output Polarity */
#define  TIM_CC3NE                       ((uint16_t)0x0400)            /* Capture/Compare 3 Complementary output enable */
#define  TIM_CC3NP                       ((uint16_t)0x0800)            /* Capture/Compare 3 Complementary output Polarity */
#define  TIM_CC4E                        ((uint16_t)0x1000)            /* Capture/Compare 4 output enable */
#define  TIM_CC4P                        ((uint16_t)0x2000)            /* Capture/Compare 4 output Polarity */
#define  TIM_CC4NP                       ((uint16_t)0x8000)            /* Capture/Compare 4 Complementary output Polarity */

/*******************  Bit definition for TIM_CNT register  ********************/
#define  TIM_CNT                         ((uint16_t)0xFFFF)            /* Counter Value */

/*******************  Bit definition for TIM_PSC register  ********************/
#define  TIM_PSC                         ((uint16_t)0xFFFF)            /* Prescaler Value */

/*******************  Bit definition for TIM_ATRLR register  ********************/
#define  TIM_ARR                         ((uint16_t)0xFFFF)            /* actual auto-reload Value */

/*******************  Bit definition for TIM_RPTCR register  ********************/
#define  TIM_REP                         ((uint8_t)0xFF)               /* Repetition Counter Value */

/*******************  Bit definition for TIM_CH1CVR register  *******************/
#define  TIM_CCR1                        ((uint16_t)0xFFFF)            /* Capture/Compare 1 Value */

/*******************  Bit definition for TIM_CH2CVR register  *******************/
#define  TIM_CCR2                        ((uint16_t)0xFFFF)            /* Capture/Compare 2 Value */

/*******************  Bit definition for TIM_CH3CVR register  *******************/
#define  TIM_CCR3                        ((uint16_t)0xFFFF)            /* Capture/Compare 3 Value */

/*******************  Bit definition for TIM_CH4CVR register  *******************/
#define  TIM_CCR4                        ((uint16_t)0xFFFF)            /* Capture/Compare 4 Value */

/*******************  Bit definition for TIM_BDTR register  *******************/
#define  TIM_DTG                         ((uint16_t)0x00FF)            /* DTG[0:7] bits (Dead-Time Generator set-up) */
#define  TIM_DTG_0                       ((uint16_t)0x0001)            /* Bit 0 */
#define  TIM_DTG_1                       ((uint16_t)0x0002)            /* Bit 1 */
#define  TIM_DTG_2                       ((uint16_t)0x0004)            /* Bit 2 */
#define  TIM_DTG_3                       ((uint16_t)0x0008)            /* Bit 3 */
#define  TIM_DTG_4                       ((uint16_t)0x0010)            /* Bit 4 */
#define  TIM_DTG_5                       ((uint16_t)0x0020)            /* Bit 5 */
#define  TIM_DTG_6                       ((uint16_t)0x0040)            /* Bit 6 */
#define  TIM_DTG_7                       ((uint16_t)0x0080)            /* Bit 7 */

#define  TIM_LOCK                        ((uint16_t)0x0300)            /* LOCK[1:0] bits (Lock Configuration) */
#define  TIM_LOCK_0                      ((uint16_t)0x0100)            /* Bit 0 */
#define  TIM_LOCK_1                      ((uint16_t)0x0200)            /* Bit 1 */

#define  TIM_OSSI                        ((uint16_t)0x0400)            /* Off-State Selection for Idle mode */
#define  TIM_OSSR                        ((uint16_t)0x0800)            /* Off-State Selection for Run mode */
#define  TIM_BKE                         ((uint16_t)0x1000)            /* Break enable */
#define  TIM_BKP                         ((uint16_t)0x2000)            /* Break Polarity */
#define  TIM_AOE                         ((uint16_t)0x4000)            /* Automatic Output enable */
#define  TIM_MOE                         ((uint16_t)0x8000)            /* Main Output enable */

/*******************  Bit definition for TIM_DMACFGR register  ********************/
#define  TIM_DBA                         ((uint16_t)0x001F)            /* DBA[4:0] bits (DMA Base Address) */
#define  TIM_DBA_0                       ((uint16_t)0x0001)            /* Bit 0 */
#define  TIM_DBA_1                       ((uint16_t)0x0002)            /* Bit 1 */
#define  TIM_DBA_2                       ((uint16_t)0x0004)            /* Bit 2 */
#define  TIM_DBA_3                       ((uint16_t)0x0008)            /* Bit 3 */
#define  TIM_DBA_4                       ((uint16_t)0x0010)            /* Bit 4 */

#define  TIM_DBL                         ((uint16_t)0x1F00)            /* DBL[4:0] bits (DMA Burst Length) */
#define  TIM_DBL_0                       ((uint16_t)0x0100)            /* Bit 0 */
#define  TIM_DBL_1                       ((uint16_t)0x0200)            /* Bit 1 */
#define  TIM_DBL_2                       ((uint16_t)0x0400)            /* Bit 2 */
#define  TIM_DBL_3                       ((uint16_t)0x0800)            /* Bit 3 */
#define  TIM_DBL_4                       ((uint16_t)0x1000)            /* Bit 4 */

/*******************  Bit definition for TIM_DMAADR register  *******************/
#define  TIM_DMAR_DMAB                   ((uint16_t)0xFFFF)            /* DMA register for burst accesses */





#include "spp32r20xx_conf.h"


#ifdef __cplusplus
}
#endif

#endif
