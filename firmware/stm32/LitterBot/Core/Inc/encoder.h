/**
 * @file    encoder.h
 * @brief   Quadrature encoder driver for 4-wheel mecanum drive
 *
 * Reads four goBILDA Yellow Jacket motor encoders via STM32 hardware
 * encoder mode (timers counting A/B quadrature signals automatically).
 *
 * Encoder Specs (goBILDA 5203-2402-0019):
 *   - Hall-effect quadrature encoder, 28 CPR at motor shaft
 *   - Gear ratio: 19.2:1
 *   - Output shaft resolution: 28 * 19.2 = 537.6 ticks/rev (4x counting)
 *   - Wheel: 104mm GripForce Mecanum (52mm radius)
 *
 * Timer Assignments:
 *   Encoder | CHA Pin | CHB Pin | Timer
 *   --------|---------|---------|------
 *   FL      | PB6     | PB7    | TIM4   (PB7 conflicts with LD2 LED)
 *   FR      | PA6     | PA7    | TIM3
 *   RL      | PC6     | PC7    | TIM8
 *   RR      | PE9     | PE11   | TIM1
 *
 * Encoder wires: White = CHA, Blue = CHB
 *
 * NOTE: CubeMX must configure TIM1, TIM3, TIM4, TIM8 in Encoder Mode
 *       (TI1 and TI2, both edges) before this driver will function.
 *       Enabling TIM4 on PB7 will disable LD2 LED.
 *
 * Usage in main.c:
 *   #include "encoder.h"
 *   // after peripheral init:
 *   Encoder_Init();
 *   // in while(1) loop (call at fixed rate, e.g. every 20ms):
 *   Encoder_Update();
 *   const Encoder_Data_t *fl = Encoder_Get(ENC_FL);
 *
 * @author  LitterBot Team
 * @date    2026
 */

#ifndef ENCODER_H
#define ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include <stdint.h>

/* Encoder constants ---------------------------------------------------------*/

/** @brief Encoder ticks per output shaft revolution (28 CPR * 19.2 gear ratio) */
#define ENCODER_TICKS_PER_REV   537.7f

/** @brief Wheel radius in meters (104mm diameter / 2) */
#define ENCODER_WHEEL_RADIUS_M  0.052f

/* Encoder identification ----------------------------------------------------*/

/** @brief Encoder index matching motor positions */
typedef enum {
    ENC_FL = 0,  /**< Front-Left encoder (TIM4, PB6/PB7)  */
    ENC_FR,      /**< Front-Right encoder (TIM3, PA6/PA7)  */
    ENC_RL,      /**< Rear-Left encoder (TIM8, PC6/PC7)    */
    ENC_RR,      /**< Rear-Right encoder (TIM1, PE9/PE11)  */
    ENC_COUNT    /**< Total number of encoders (always last) */
} Encoder_ID_t;

/* Encoder data structure ----------------------------------------------------*/

/**
 * @brief Per-encoder runtime data
 *
 * Updated by Encoder_Update(). Ticks accumulate into a 32-bit counter
 * to handle 16-bit timer overflow. Velocity is computed from the delta
 * since the last update call.
 */
typedef struct {
    int32_t  ticks;         /**< Accumulated tick count (signed, handles overflow) */
    int32_t  delta;         /**< Ticks since last Encoder_Update() call */
    float    velocity_rps;  /**< Output shaft revolutions per second */
} Encoder_Data_t;

/* Public function prototypes ------------------------------------------------*/

/**
 * @brief  Initialize the encoder driver subsystem
 *
 * Starts all four timers in encoder mode by calling
 * HAL_TIM_Encoder_Start() on both channels of each timer.
 *
 * Call this after HAL_Init(), SystemClock_Config(), and MX_TIMx_Init().
 *
 * @retval HAL_OK on success, HAL_ERROR if any timer fails to start
 */
HAL_StatusTypeDef Encoder_Init(void);

/**
 * @brief  Sample all encoder counters and compute deltas/velocity
 *
 * Reads the 16-bit hardware counter from each timer, computes the
 * signed delta (handling 16-bit wrap-around), accumulates into 32-bit
 * ticks, and calculates velocity in revolutions per second.
 *
 * Must be called at a fixed rate for accurate velocity. Recommended
 * interval: 20ms (50 Hz) from the main loop.
 */
void Encoder_Update(void);

/**
 * @brief  Get encoder data for a specific wheel
 *
 * @param  id  Which encoder (ENC_FL..ENC_RR)
 * @return Pointer to encoder data struct, or NULL if id is invalid
 */
const Encoder_Data_t* Encoder_Get(Encoder_ID_t id);

/**
 * @brief  Get accumulated tick count for a specific encoder
 *
 * Convenience function -- equivalent to Encoder_Get(id)->ticks.
 *
 * @param  id  Which encoder (ENC_FL..ENC_RR)
 * @return Accumulated ticks, or 0 if id is invalid
 */
int32_t Encoder_GetTicks(Encoder_ID_t id);

/**
 * @brief  Reset all encoder counters and accumulators to zero
 *
 * Zeroes out the hardware timer counters and all accumulated
 * tick/delta/velocity data. Call before a measurement sequence.
 */
void Encoder_ResetAll(void);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */
