/**
 * @file    cl57t_stepper.h
 * @brief   CL57T closed-loop stepper motor driver interface
 *
 * Controls a NEMA 23 closed-loop stepper motor via a CL57T driver
 * using step/direction GPIO signals (bit-banged, not timer PWM).
 *
 * The CL57T driver handles the closed-loop position control internally
 * using the motor's built-in encoder. The STM32 only needs to generate
 * step pulses and set direction -- the driver does the rest.
 *
 * Pin Assignments (updated 2.24.2026):
 *   Signal  | Pin  | Description
 *   --------|------|----------------------------------
 *   PUL+    | PD14 | Step pulse output (active high)
 *   DIR+    | PF15 | Direction output (HIGH/LOW)
 *
 * NOTE: Originally PA0/PB0, then PC6/PC7. Reassigned to PD14/PF15
 *       per user routing preference (both free on Nucleo-144 morpho).
 *
 * Timing Requirements (CL57T datasheet):
 *   - Minimum PUL high time:  2.5 us
 *   - Minimum PUL low time:   2.5 us
 *   - DIR setup time before PUL rising edge: 5 us
 *   - Maximum step frequency: ~200 kHz (depends on motor/load)
 *
 * NOTE: This driver will not function until CubeMX enables:
 *   - GPIOC clock (already enabled in default config for PC13 button)
 *   - The pins must not be configured as alternate function by CubeMX
 *     if using GPIO bit-bang mode
 *
 * @author  LitterBot Team
 * @date    2026
 */

#ifndef CL57T_STEPPER_H
#define CL57T_STEPPER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include <stdbool.h>

/* Pin definitions -----------------------------------------------------------*/

#define CL57T_PUL_PIN           GPIO_PIN_14
#define CL57T_PUL_PORT          GPIOD
#define CL57T_DIR_PIN           GPIO_PIN_15
#define CL57T_DIR_PORT          GPIOF

/* Timing constants ----------------------------------------------------------*/

/**
 * @brief Minimum pulse high time in microseconds
 *
 * The CL57T requires the PUL signal to remain HIGH for at least 2.5 us
 * for the step to be registered. We use 3 us for margin.
 */
#define CL57T_PULSE_WIDTH_US    3

/**
 * @brief Direction setup time in microseconds
 *
 * The CL57T datasheet minimum is 5 us, but many CL57T-class drivers
 * (especially the Chinese clones) latch direction more slowly and
 * silently ignore direction changes if the first step pulse arrives
 * too soon. We tried 50 us and reverse jog was ignored on our unit;
 * bumped to 500 us, which still only costs 500 µs per MoveSteps call.
 */
#define CL57T_DIR_SETUP_US      500

/* Direction enumeration -----------------------------------------------------*/

/** @brief Stepper rotation direction */
typedef enum {
    CL57T_DIR_CW  = 0,  /**< Clockwise rotation (DIR pin LOW)  */
    CL57T_DIR_CCW = 1   /**< Counter-clockwise rotation (DIR pin HIGH) */
} CL57T_Direction_t;

typedef bool (*CL57T_StopPredicate_t)(void);

/* Stepper data structure ----------------------------------------------------*/

/**
 * @brief Stepper motor configuration
 *
 * Holds the GPIO port/pin assignments for the pulse and direction
 * signals. This struct is populated by CL57T_Init().
 */
typedef struct {
    GPIO_TypeDef *pul_port;    /**< Pulse signal GPIO port */
    uint16_t      pul_pin;     /**< Pulse signal GPIO pin  */
    GPIO_TypeDef *dir_port;    /**< Direction signal GPIO port */
    uint16_t      dir_pin;     /**< Direction signal GPIO pin  */
} CL57T_Stepper_t;

/* Public function prototypes ------------------------------------------------*/

/**
 * @brief  Initialize the stepper motor driver
 *
 * Configures PUL and DIR pins as push-pull GPIO outputs.
 * Both pins are initialized LOW (no pulse, default direction).
 *
 * Call this after HAL_Init() and SystemClock_Config().
 */
void CL57T_Init(void);

/**
 * @brief  Set the stepper rotation direction
 *
 * Changes the DIR pin state. Must be called at least CL57T_DIR_SETUP_US
 * microseconds before the next step pulse for the direction change to
 * take effect.
 *
 * @param  dir  CL57T_DIR_CW or CL57T_DIR_CCW
 */
void CL57T_SetDirection(CL57T_Direction_t dir);

/**
 * @brief  Generate a single step pulse
 *
 * Produces one rising-then-falling edge on the PUL pin.
 * The pulse width is at least CL57T_PULSE_WIDTH_US microseconds.
 *
 * This is a blocking call -- it busy-waits for the pulse duration.
 * For time-critical applications, consider a timer-based approach.
 */
void CL57T_Step(void);

/**
 * @brief  Move a specified number of steps (blocking)
 *
 * Generates the requested number of step pulses with a configurable
 * delay between each pulse to control speed. This function blocks
 * until all steps are completed.
 *
 * Total time per step = CL57T_PULSE_WIDTH_US + delay_us (approximately).
 *
 * @param  steps    Number of step pulses to generate (must be > 0)
 * @param  dir      Rotation direction (CL57T_DIR_CW or CL57T_DIR_CCW)
 * @param  delay_us Delay in microseconds between step pulses.
 *                  Controls speed -- smaller delay = faster rotation.
 *                  Minimum recommended: 5 us (with CL57T_PULSE_WIDTH_US
 *                  this gives ~125 kHz step rate).
 */
void CL57T_MoveSteps(uint32_t steps, CL57T_Direction_t dir, uint32_t delay_us);

/**
 * @brief  Run the stepper continuously until a cancel flag is set
 *
 * Blocks in a tight step loop with no inter-burst gaps. Designed for
 * UI "hold to jog" behavior: the caller passes a pointer to a volatile
 * flag (typically the UART command-ready flag) that the loop polls
 * between steps. When the flag becomes non-zero, the loop exits
 * cleanly so the next command can be processed.
 *
 * This replaces the burst-based jog (repeated CL57T_MoveSteps calls
 * from the ROS bridge at ~22 Hz). Bursts had a ~1 ms silent gap at
 * each burst boundary which, at microstep 16 + 3+ rev/s, was enough
 * for the closed-loop driver to see the rotor drift and slam back
 * when pulses resumed — producing an audible 22 Hz hammering. A
 * continuous loop has no such gaps.
 *
 * @param  dir       Rotation direction (CL57T_DIR_CW or CL57T_DIR_CCW)
 * @param  delay_us  Per-step delay in microseconds (minimum 5)
 * @param  cancel    Pointer to a volatile flag. Loop exits when
 *                   *cancel becomes non-zero. If NULL, loops forever
 *                   (caller is responsible for not passing NULL).
 */
void CL57T_Jog(CL57T_Direction_t dir, uint32_t delay_us, volatile int *cancel,
               CL57T_StopPredicate_t stop_predicate);

/**
 * @brief  Get pointer to the stepper configuration struct
 *
 * @return Pointer to the internal CL57T_Stepper_t struct
 */
const CL57T_Stepper_t* CL57T_GetStepper(void);

#ifdef __cplusplus
}
#endif

#endif /* CL57T_STEPPER_H */
