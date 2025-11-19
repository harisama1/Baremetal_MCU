/*
 * systick.h
 *
 *  Created on: Nov 19, 2025
 *      Author: Harisama1
 */

#ifndef SYSTICK_H
#define SYSTICK_H

#include <stdint.h>

/* ============================================================
 *  SysTick BASE ADDRESS (Fixed in Cortex-M)
 * ============================================================*/
#define SYSTICK_BASE        (0xE000E010UL)

/* ============================================================
 *  SysTick STRUCTURE (CMSIS Standard)
 * ============================================================*/
typedef struct
{
    volatile uint32_t CTRL;     // 0x00 Control and Status Register
    volatile uint32_t LOAD;     // 0x04 Reload Value Register
    volatile uint32_t VAL;      // 0x08 Current Value Register
    volatile uint32_t CALIB;    // 0x0C Calibration Register
} SysTick_Type;

/* SysTick pointer */
#define SysTick     ((SysTick_Type *) SYSTICK_BASE)

/* ============================================================
 *  CTRL REGISTER BIT DEFINITIONS
 * ============================================================*/
/* Bit positions */
#define SYSTICK_CTRL_ENABLE_Pos        0U
#define SYSTICK_CTRL_TICKINT_Pos       1U
#define SYSTICK_CTRL_CLKSOURCE_Pos     2U
#define SYSTICK_CTRL_COUNTFLAG_Pos     16U

/* Bit masks */
#define SYSTICK_CTRL_ENABLE_Msk        (1U << SYSTICK_CTRL_ENABLE_Pos)
#define SYSTICK_CTRL_TICKINT_Msk       (1U << SYSTICK_CTRL_TICKINT_Pos)
#define SYSTICK_CTRL_CLKSOURCE_Msk     (1U << SYSTICK_CTRL_CLKSOURCE_Pos)
#define SYSTICK_CTRL_COUNTFLAG_Msk     (1U << SYSTICK_CTRL_COUNTFLAG_Pos)

/* ============================================================
 *  LOAD REGISTER BIT DEFINITIONS
 * ============================================================*/
#define SYSTICK_LOAD_RELOAD_Pos        0U
#define SYSTICK_LOAD_RELOAD_Msk        (0xFFFFFFU << SYSTICK_LOAD_RELOAD_Pos)

/* ============================================================
 *  VAL REGISTER BIT DEFINITIONS
 * ============================================================*/
#define SYSTICK_VAL_CURRENT_Pos        0U
#define SYSTICK_VAL_CURRENT_Msk        (0xFFFFFFU << SYSTICK_VAL_CURRENT_Pos)

/* ============================================================
 *  CALIB REGISTER BIT DEFINITIONS
 * ============================================================*/
#define SYSTICK_CALIB_TENMS_Pos        0U
#define SYSTICK_CALIB_TENMS_Msk        (0xFFFFFFU << SYSTICK_CALIB_TENMS_Pos)

#define SYSTICK_CALIB_SKEW_Pos         30U
#define SYSTICK_CALIB_SKEW_Msk         (1U << SYSTICK_CALIB_SKEW_Pos)

#define SYSTICK_CALIB_NOREF_Pos        31U
#define SYSTICK_CALIB_NOREF_Msk        (1U << SYSTICK_CALIB_NOREF_Pos)

/* ============================================================
 *  SYSTICK HELPER MACROS (optional but useful)
 * ============================================================*/
#define SYSTICK_ENABLE()        (SysTick->CTRL |= SYSTICK_CTRL_ENABLE_Msk)
#define SYSTICK_DISABLE()       (SysTick->CTRL &= ~SYSTICK_CTRL_ENABLE_Msk)

#define SYSTICK_INT_ENABLE()    (SysTick->CTRL |= SYSTICK_CTRL_TICKINT_Msk)
#define SYSTICK_INT_DISABLE()   (SysTick->CTRL &= ~SYSTICK_CTRL_TICKINT_Msk)

#define SYSTICK_SET_CLK_CPU()   (SysTick->CTRL |= SYSTICK_CTRL_CLKSOURCE_Msk)
#define SYSTICK_SET_CLK_EXT()   (SysTick->CTRL &= ~SYSTICK_CTRL_CLKSOURCE_Msk)

#define SYSTICK_SET_RELOAD(x)   (SysTick->LOAD = ((x) & 0xFFFFFFU))
#define SYSTICK_CLEAR()         (SysTick->VAL  = 0U)

void SystickDelayMS(uint16_t delay);

#endif /* SYSTICK_H */
