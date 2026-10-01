/**
 * @file    limit_switch.c
 * @brief   Micro limit switch driver implementation
 *
 * See limit_switch.h for wiring, pin assignments, and fail-safe logic.
 *
 * @author  LitterBot Team
 * @date    2026
 */

/* Includes ------------------------------------------------------------------*/
#include "limit_switch.h"

/* Public functions ----------------------------------------------------------*/

void LimitSwitch_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* Enable GPIO port clocks */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* HOME limit switch — PD15, input with pull-up */
    gpio.Pin   = LIMIT_HOME_PIN;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LIMIT_HOME_PORT, &gpio);

    /* EXTEND limit switch — PE10, input with pull-up */
    gpio.Pin = LIMIT_EXTEND_PIN;
    HAL_GPIO_Init(LIMIT_EXTEND_PORT, &gpio);
}

bool LimitSwitch_HomeTriggered(void)
{
    /* NC to GND: HIGH = triggered (switch open or wire break) */
    return HAL_GPIO_ReadPin(LIMIT_HOME_PORT, LIMIT_HOME_PIN) == GPIO_PIN_SET;
}

bool LimitSwitch_ExtendTriggered(void)
{
    /* NC to GND: HIGH = triggered (switch open or wire break) */
    return HAL_GPIO_ReadPin(LIMIT_EXTEND_PORT, LIMIT_EXTEND_PIN) == GPIO_PIN_SET;
}
