/**
 * @file    cytron_md10c.h
 * @brief   Cytron MD10C DC motor driver interface for 4-wheel drive
 *
 * Controls four Cytron MD10C motor drivers in sign-magnitude mode:
 *   - PWM signal controls motor speed (duty cycle)
 *   - DIR signal controls direction (GPIO output)
 *
 * All four PWM channels share TIM2 (PA0-PA3, channels 1-4).
 * Each motor has an independent direction pin (FL/FR/RL on Port B,
 * RR on Port F).
 *
 * PWM Configuration:
 *   - Timer clock: ~47.333 MHz (APB1 timer clock)
 *   - Prescaler: 0  (no division)
 *   - ARR: 2366     (period = 2367 counts)
 *   - Frequency: 47,333,333 / 2367 = ~19,996 Hz (~20 kHz)
 *   - 20 kHz is above audible range, reducing motor whine
 *
 * Pin Assignments (from logbook Week 4 Tues):
 *   Motor  | PWM Pin | Timer Channel | DIR Pin
 *   -------|---------|---------------|--------
 *   FL     | PA0     | TIM2_CH1      | PB0
 *   FR     | PA1     | TIM2_CH2      | PB1
 *   RL     | PA2     | TIM2_CH3      | PB2
 *   RR     | PA3     | TIM2_CH4      | PF14
 *
 * NOTE: PA0/PB0 conflict with CL57T stepper was resolved by
 *       reassigning stepper to PD14/PF15.
 * NOTE: RR DIR was moved from PB10 to PF14 to free PB10 for servo UART.
 *       PB10 later damaged; servo USART3 now on PC10.
 *
 * NOTE: This driver will not function until CubeMX enables:
 *   - HAL_TIM_MODULE_ENABLED in stm32l4xx_hal_conf.h
 *   - TIM2 configured with PWM outputs on PA0-PA3
 *   - GPIOB clock enabled for direction pins
 *
 * @author  LitterBot Team
 * @date    2026
 */

#ifndef CYTRON_MD10C_H
#define CYTRON_MD10C_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Motor identification ------------------------------------------------------*/

/** @brief Motor index enumeration for 4-wheel drive */
typedef enum {
    MD10C_MOTOR_FL = 0,  /**< Front-Left motor  */
    MD10C_MOTOR_FR,      /**< Front-Right motor */
    MD10C_MOTOR_RL,      /**< Rear-Left motor   */
    MD10C_MOTOR_RR,      /**< Rear-Right motor  */
    MD10C_MOTOR_COUNT    /**< Total number of motors (always last) */
} MD10C_MotorID_t;

/* Pin definitions -----------------------------------------------------------*/

/* PWM pins (active via TIM2 alternate function, defined here for reference) */
#define MD10C_FL_PWM_PIN        GPIO_PIN_0
#define MD10C_FL_PWM_PORT       GPIOA
#define MD10C_FR_PWM_PIN        GPIO_PIN_1
#define MD10C_FR_PWM_PORT       GPIOA
#define MD10C_RL_PWM_PIN        GPIO_PIN_2
#define MD10C_RL_PWM_PORT       GPIOA
#define MD10C_RR_PWM_PIN        GPIO_PIN_3
#define MD10C_RR_PWM_PORT       GPIOA

/* Direction pins (GPIO outputs) */
#define MD10C_FL_DIR_PIN        GPIO_PIN_0
#define MD10C_FL_DIR_PORT       GPIOB
#define MD10C_FR_DIR_PIN        GPIO_PIN_1
#define MD10C_FR_DIR_PORT       GPIOB
#define MD10C_RL_DIR_PIN        GPIO_PIN_2
#define MD10C_RL_DIR_PORT       GPIOB
#define MD10C_RR_DIR_PIN        GPIO_PIN_14
#define MD10C_RR_DIR_PORT       GPIOF

/* Timer configuration -------------------------------------------------------*/

/** @brief TIM2 prescaler value (0 = no prescaling, timer runs at bus clock) */
#define MD10C_TIM_PRESCALER     0

/**
 * @brief TIM2 auto-reload value for ~20 kHz PWM
 *
 * With 47.333 MHz timer clock and PSC=0:
 *   f_PWM = 47,333,333 / (ARR + 1) = 47,333,333 / 2367 = ~19,996 Hz
 */
#define MD10C_TIM_PERIOD        2366

/* Motor data structure ------------------------------------------------------*/

/**
 * @brief Per-motor configuration and runtime state
 *
 * Each motor has a timer channel for PWM output and a GPIO pin for direction.
 * The timer handle is shared across all four motors (all use TIM2).
 */
typedef struct {
    TIM_HandleTypeDef *htim;       /**< Pointer to timer handle (TIM2) */
    uint32_t           channel;    /**< Timer channel (TIM_CHANNEL_x) */
    GPIO_TypeDef      *dir_port;   /**< Direction GPIO port */
    uint16_t           dir_pin;    /**< Direction GPIO pin */
} MD10C_Motor_t;

/* Public function prototypes ------------------------------------------------*/

/**
 * @brief  Initialize the DC motor driver subsystem
 *
 * Configures TIM2 for PWM output on all four channels and sets up
 * direction GPIO pins as push-pull outputs. Starts PWM generation
 * with 0% duty cycle on all channels (motors stopped).
 *
 * Call this after HAL_Init() and SystemClock_Config().
 *
 * @retval HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef MD10C_Init(void);

/**
 * @brief  Set motor speed and direction
 *
 * Accepts a signed speed percentage:
 *   - Positive values: forward rotation
 *   - Negative values: reverse rotation
 *   - Zero: motor stopped (0% duty cycle)
 *
 * The sign determines the DIR pin state. The magnitude sets the
 * PWM duty cycle proportionally (0-100% maps to 0-ARR compare value).
 *
 * @param  motor_id  Which motor to control (MD10C_MOTOR_FL..RR)
 * @param  speed_pct Speed as percentage, range [-100.0, +100.0]
 *                   Values outside this range are clamped.
 * @retval HAL_OK on success, HAL_ERROR if motor_id is invalid
 */
HAL_StatusTypeDef MD10C_SetSpeed(MD10C_MotorID_t motor_id, float speed_pct);

/**
 * @brief  Stop a single motor (set PWM to 0%)
 *
 * @param  motor_id  Which motor to stop (MD10C_MOTOR_FL..RR)
 * @retval HAL_OK on success, HAL_ERROR if motor_id is invalid
 */
HAL_StatusTypeDef MD10C_Stop(MD10C_MotorID_t motor_id);

/**
 * @brief  Emergency stop all motors
 *
 * Immediately sets all four PWM outputs to 0% duty cycle.
 * Call this from E-stop handlers or fault conditions.
 */
void MD10C_StopAll(void);

/**
 * @brief  Get the motor configuration struct for direct access
 *
 * Useful for debugging or advanced control scenarios where you
 * need direct access to the timer handle or channel.
 *
 * @param  motor_id  Which motor (MD10C_MOTOR_FL..RR)
 * @return Pointer to motor struct, or NULL if motor_id is invalid
 */
const MD10C_Motor_t* MD10C_GetMotor(MD10C_MotorID_t motor_id);

#ifdef __cplusplus
}
#endif

#endif /* CYTRON_MD10C_H */
