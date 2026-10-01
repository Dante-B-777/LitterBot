/**
 * @file    cytron_md10c.c
 * @brief   Cytron MD10C DC motor driver implementation
 *
 * Implements PWM-based speed control for four Cytron MD10C motor drivers
 * using TIM2 channels 1-4 and GPIO direction pins.
 *
 * TIM2 and its PWM GPIO pins (PA0-PA3) are configured by CubeMX in main.c.
 * This driver references the CubeMX-generated htim2 handle, overrides the
 * period (ARR) to get ~20 kHz PWM, sets up direction GPIO pins, and starts
 * the PWM channels.
 *
 * @author  LitterBot Team
 * @date    2026
 */

/* Includes ------------------------------------------------------------------*/
#include "cytron_md10c.h"

/* External variables --------------------------------------------------------*/

/**
 * @brief TIM2 handle declared by CubeMX in main.c
 *
 * CubeMX generates this and calls MX_TIM2_Init() during startup.
 * We reference it here to start PWM and set compare values.
 */
extern TIM_HandleTypeDef htim2;

/* Private variables ---------------------------------------------------------*/

/**
 * @brief Motor configuration array
 *
 * Stores the timer channel and direction pin for each motor.
 * Indexed by MD10C_MotorID_t enum values.
 */
static MD10C_Motor_t motors[MD10C_MOTOR_COUNT];

/* Private function prototypes -----------------------------------------------*/
static void MD10C_InitDirectionGPIO(void);

/* Public functions ----------------------------------------------------------*/

HAL_StatusTypeDef MD10C_Init(void)
{
    HAL_StatusTypeDef status;

    /* --- Populate motor configuration table --- */

    /* Front-Left: TIM2_CH1 on PA0, DIR on PB0 */
    motors[MD10C_MOTOR_FL].htim     = &htim2;
    motors[MD10C_MOTOR_FL].channel  = TIM_CHANNEL_1;
    motors[MD10C_MOTOR_FL].dir_port = MD10C_FL_DIR_PORT;
    motors[MD10C_MOTOR_FL].dir_pin  = MD10C_FL_DIR_PIN;

    /* Front-Right: TIM2_CH2 on PA1, DIR on PB1 */
    motors[MD10C_MOTOR_FR].htim     = &htim2;
    motors[MD10C_MOTOR_FR].channel  = TIM_CHANNEL_2;
    motors[MD10C_MOTOR_FR].dir_port = MD10C_FR_DIR_PORT;
    motors[MD10C_MOTOR_FR].dir_pin  = MD10C_FR_DIR_PIN;

    /* Rear-Left: TIM2_CH3 on PA2, DIR on PB2 */
    motors[MD10C_MOTOR_RL].htim     = &htim2;
    motors[MD10C_MOTOR_RL].channel  = TIM_CHANNEL_3;
    motors[MD10C_MOTOR_RL].dir_port = MD10C_RL_DIR_PORT;
    motors[MD10C_MOTOR_RL].dir_pin  = MD10C_RL_DIR_PIN;

    /* Rear-Right: TIM2_CH4 on PA3, DIR on PF14 */
    motors[MD10C_MOTOR_RR].htim     = &htim2;
    motors[MD10C_MOTOR_RR].channel  = TIM_CHANNEL_4;
    motors[MD10C_MOTOR_RR].dir_port = MD10C_RR_DIR_PORT;
    motors[MD10C_MOTOR_RR].dir_pin  = MD10C_RR_DIR_PIN;

    /* --- Initialize hardware --- */

    /* Configure direction GPIO pins (must be done before starting PWM) */
    MD10C_InitDirectionGPIO();

    /*
     * Override the timer period to get ~20 kHz PWM.
     *
     * CubeMX may have set a different ARR value (e.g., max 32-bit).
     * We force it to 2366 here so that:
     *   f_PWM = 47,333,333 / (0+1) / (2366+1) = ~19,996 Hz
     *
     * __HAL_TIM_SET_AUTORELOAD writes the ARR register directly.
     */
    __HAL_TIM_SET_AUTORELOAD(&htim2, MD10C_TIM_PERIOD);

    /* Start PWM on all four channels with 0% duty cycle */
    for (int i = 0; i < MD10C_MOTOR_COUNT; i++) {
        __HAL_TIM_SET_COMPARE(&htim2, motors[i].channel, 0);
        status = HAL_TIM_PWM_Start(&htim2, motors[i].channel);
        if (status != HAL_OK) {
            return status;
        }
    }

    return HAL_OK;
}

HAL_StatusTypeDef MD10C_SetSpeed(MD10C_MotorID_t motor_id, float speed_pct)
{
    if (motor_id >= MD10C_MOTOR_COUNT) {
        return HAL_ERROR;
    }

    MD10C_Motor_t *motor = &motors[motor_id];

    /* Clamp speed to valid range */
    if (speed_pct > 100.0f) {
        speed_pct = 100.0f;
    } else if (speed_pct < -100.0f) {
        speed_pct = -100.0f;
    }

    /* Determine direction from sign */
    if (speed_pct >= 0.0f) {
        HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, GPIO_PIN_SET);
        speed_pct = -speed_pct;  /* Work with magnitude from here */
    }

    /*
     * Convert percentage to timer compare value.
     *
     * duty_cycle = (speed_pct / 100.0) * (ARR + 1)
     *
     * At 100%: compare = 2367 (ARR+1), which gives full-on output.
     * At   0%: compare = 0, output stays low.
     */
    uint32_t compare = (uint32_t)((speed_pct / 100.0f) * (float)(MD10C_TIM_PERIOD + 1));

    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, compare);

    return HAL_OK;
}

HAL_StatusTypeDef MD10C_Stop(MD10C_MotorID_t motor_id)
{
    if (motor_id >= MD10C_MOTOR_COUNT) {
        return HAL_ERROR;
    }

    __HAL_TIM_SET_COMPARE(motors[motor_id].htim, motors[motor_id].channel, 0);

    return HAL_OK;
}

void MD10C_StopAll(void)
{
    for (int i = 0; i < MD10C_MOTOR_COUNT; i++) {
        __HAL_TIM_SET_COMPARE(motors[i].htim, motors[i].channel, 0);
    }
}

const MD10C_Motor_t* MD10C_GetMotor(MD10C_MotorID_t motor_id)
{
    if (motor_id >= MD10C_MOTOR_COUNT) {
        return NULL;
    }
    return &motors[motor_id];
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Configure direction GPIO pins as push-pull outputs
 *
 * FL/FR/RL direction pins are on Port B (PB0, PB1, PB2).
 * RR direction pin is on Port F (PF14).
 * All are initialized LOW (forward direction).
 */
static void MD10C_InitDirectionGPIO(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Ensure GPIO clocks are enabled */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    /* Set initial direction to forward (LOW) for all pins */
    HAL_GPIO_WritePin(GPIOB, MD10C_FL_DIR_PIN | MD10C_FR_DIR_PIN |
                              MD10C_RL_DIR_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MD10C_RR_DIR_PORT, MD10C_RR_DIR_PIN, GPIO_PIN_RESET);

    /* Configure PB0, PB1, PB2 as outputs (FL, FR, RL) */
    GPIO_InitStruct.Pin   = MD10C_FL_DIR_PIN | MD10C_FR_DIR_PIN |
                            MD10C_RL_DIR_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Configure PF14 as output (RR) */
    GPIO_InitStruct.Pin   = MD10C_RR_DIR_PIN;
    HAL_GPIO_Init(MD10C_RR_DIR_PORT, &GPIO_InitStruct);
}
