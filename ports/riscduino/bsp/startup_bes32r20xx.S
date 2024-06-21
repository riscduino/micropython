/********************************** (C) COPYRIGHT *******************************
* Copyright (c) 2024  BigEndian Semiconductor Private Limited
* SPDX-License-Identifier: Apache-2.0
*
* File Name          : startup_bes32r20xx.s
* Author             : Dinesh Annayya
* Version            : V1.0.0
* Date               : 20-June-2024
* Description        : Startup Sequence
*******************************************************************************/

	.section	.init,"ax",@progbits
	.global	_start
	.global	isr_wrapper
	.align	4
_vector_base:
	/* Interrupts */
	j isr_wrapper	/* 0x1c0 IRQ 0 */
	j isr_wrapper	/* 0x1c4 IRQ 1 */
	j isr_wrapper	/* 0x1c8 IRQ 2 */
	j isr_wrapper	/* 0x1cc IRQ 3 */
	j isr_wrapper	/* 0x1d0 IRQ 4 */
	j isr_wrapper	/* 0x1d4 IRQ 5 */
	j isr_wrapper	/* 0x1d8 IRQ 6 */
	j isr_wrapper	/* 0x1dc IRQ 7 */
	j isr_wrapper	/* 0x1e0 IRQ 8 */
	j isr_wrapper	/* 0x1e4 IRQ 9 */
	j isr_wrapper	/* 0x1e8 IRQ 10 */
	j isr_wrapper	/* 0x1ec IRQ 11 */
	j isr_wrapper	/* 0x1f0 IRQ 12 */
	j isr_wrapper	/* 0x1f4 IRQ 13 */
	j isr_wrapper	/* 0x1f8 IRQ 13 */
	j _start	/* 0x1fC reset */
	.align	4

_start:
   /* Init All local Register */
    li	ra,0
    li	sp,0
    li	gp,0
    li	tp,0
    li	t0,0
    li	t1,0
    li	t2,0
    li	s0,0
    li	s1,0
    li	a0,0
    li	a1,0
    li	a2,0
    li	a3,0
    li	a4,0
    li	a5,0
    li	a6,0
    li	a7,0
    li	s2,0
    li	s3,0
    li	s4,0
    li	s5,0
    li	s6,0
    li	s7,0
    li	s8,0
    li	s9,0
    li	s10,0
    li	s11,0
    li	t3,0
    li	t4,0
    li	t5,0
    li	t6,0

	la gp, __global_pointer
	la sp, _eusrstack 

	/* Load data section from flash to RAM */
	la a0, _data_lma
	la a1, _data_vma
	la a2, _edata
	bgeu a1, a2, 2f
1:
	lw t0, (a0)
	sw t0, (a1)
	addi a0, a0, 4
	addi a1, a1, 4
	bltu a1, a2, 1b
2:
	/* Clear bss section */
	la a0, _sbss
	la a1, _ebss
	bgeu a0, a1, 2f
1:
	sw zero, (a0)
	addi a0, a0, 4
	bltu a0, a1, 1b
2:

        /* Set mtvec to point at _vector_base. */
 	la t0, _vector_base
        csrw mtvec, t0
        csrsi  mtvec, 1  // set vectored mode

        jal  SystemInit
        jal  DeviceInit
	la t0, main
	csrw mepc, t0
	mret
