/**
 * @file    limit_switch.h
 * @brief   Micro limit switch driver for arm travel limits
 *
 * Two normally-closed (NC) micro limit switches wired to GND provide
 * homing references and safety stops for the scoop arm mechanism.
 *
 * Wiring (NC configuration — fail-safe):
 *   - Switch COM terminal  → STM32 GPIO pin
 *   - Switch NC terminal   → GND
 *   - Internal pull-up enabled on STM32
 *
 *   Normal state:  switch closed → pin pulled to GND → reads LOW  → not triggered
 *   Triggered:     switch opens  → pull-up to VDD    → reads HIGH → triggered
 *   Wire break:    same as triggered → reads HIGH → fail-safe (stops motion)
 *
 * Pin Assignments:
 *   Switch   | Pin  | Location
 *   ---------|------|--------------------------------------------------
 *   HOME     | PD15 | Fully retracted position (adjacent to stepper PUL+ PD14)
 *   EXTEND   | PE10 | Fully extended position  (near encoder pins PE9/PE11)
 *
 * @author  LitterBot Team
 * @date    2026
 */

#ifndef LIMIT_SWITCH_H
#define LIMIT_SWITCH_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include <stdbool.h>

/* Pin definitions -----------------------------------------------------------*/

#define LIMIT_HOME_PIN          GPIO_PIN_15
#define LIMIT_HOME_PORT         GPIOD

#define LIMIT_EXTEND_PIN        GPIO_PIN_10
#define LIMIT_EXTEND_PORT       GPIOE

/* Public function prototypes ------------------------------------------------*/

/**
 * @brief  Initialize both limit switch GPIO pins
 *
 * Configures PD15 and PE10 as inputs with internal pull-up.
 * Enables the GPIO port clocks if not already enabled.
 *
 * Call after HAL_Init() and SystemClock_Config().
 */
void LimitSwitch_Init(void);

/**
 * @brief  Check if the HOME limit switch is triggered
 *
 * @return true if triggered (arm at fully retracted position or wire break)
 * @return false if not triggered (switch closed, arm is away from home)
 */
bool LimitSwitch_HomeTriggered(void);

/**
 * @brief  Check if the EXTEND limit switch is triggered
 *
 * @return true if triggered (arm at fully extended position or wire break)
 * @return false if not triggered (switch closed, arm is away from extend limit)
 */
bool LimitSwitch_ExtendTriggered(void);

#ifdef __cplusplus
}
#endif

#endif /* LIMIT_SWITCH_H */
