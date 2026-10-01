/**
 * @file    dc_motor_test.c
 * @brief   Cytron MD10C DC motor test
 *
 * Waits for the user button press, then tests each motor individually
 * at 25% speed (forward and reverse), followed by all motors together.
 *
 * The LD2 LED (PB7) indicates test status:
 *   - OFF:      idle, waiting for button press
 *   - ON:       test is running
 *   - BLINK 3x: test complete (success)
 *   - FAST BLINK: error occurred
 *
 * Safety: MD10C_StopAll() is called before every return path.
 *
 * @author  LitterBot Team
 * @date    2026
 */

/* Includes ------------------------------------------------------------------*/
#include "dc_motor_test.h"
#include "cytron_md10c.h"
#include "main.h"

/* Private defines -----------------------------------------------------------*/

/** @brief Time in ms to run each motor at speed */
#define RUN_MS              1500

/** @brief Pause time in ms between motor actions */
#define PAUSE_MS            500

/** @brief Debounce delay in ms after detecting button press */
#define DEBOUNCE_MS         200

/* Private variables ---------------------------------------------------------*/

/** @brief Tracks whether the test module has been initialized */
static uint8_t initialized = 0;

/* Private function prototypes -----------------------------------------------*/
static void DCMotorTest_RunSequence(void);
static void DCMotorTest_BlinkOK(void);
static void DCMotorTest_BlinkError(void);
static uint8_t DCMotorTest_ButtonPressed(void);

/* Public functions ----------------------------------------------------------*/

void DCMotorTest_Init(void)
{
    HAL_StatusTypeDef status = MD10C_Init();
    if (status != HAL_OK) {
        initialized = 0;
        return;
    }

    MD10C_StopAll();
    initialized = 1;

    /* Turn off LD2 to indicate idle state */
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
}

void DCMotorTest_Poll(void)
{
    if (!initialized) {
        return;
    }

    /* Check if button is pressed */
    if (DCMotorTest_ButtonPressed()) {
        /* Debounce: wait and verify button is still pressed */
        HAL_Delay(DEBOUNCE_MS);

        if (DCMotorTest_ButtonPressed()) {
            /* LD2 ON = test running */
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

            DCMotorTest_RunSequence();

            /* Wait for button release before returning to idle */
            while (DCMotorTest_ButtonPressed()) {
                HAL_Delay(10);
            }
            HAL_Delay(DEBOUNCE_MS);
        }
    }
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Run the DC motor test sequence
 *
 * Phase 1: Each motor individually
 *   FL forward 25% -> pause -> FL reverse 25% -> pause
 *   FR forward 25% -> pause -> FR reverse 25% -> pause
 *   RL forward 25% -> pause -> RL reverse 25% -> pause
 *   RR forward 25% -> pause -> RR reverse 25% -> pause
 *
 * Phase 2: All motors forward 25% -> pause
 *
 * Phase 3: All motors reverse 25%
 *
 * Total duration: ~20 seconds
 */
static void DCMotorTest_RunSequence(void)
{
    HAL_StatusTypeDef status;
    const MD10C_MotorID_t motors[] = {
        MD10C_MOTOR_FL, MD10C_MOTOR_FR,
        MD10C_MOTOR_RL, MD10C_MOTOR_RR
    };

    /* --- Phase 1: Individual motor test --- */
    for (int i = 0; i < 4; i++) {
        /* Forward */
        status = MD10C_SetSpeed(motors[i], TEST_SPEED_PCT);
        if (status != HAL_OK) {
            MD10C_StopAll();
            DCMotorTest_BlinkError();
            return;
        }
        HAL_Delay(RUN_MS);

        MD10C_Stop(motors[i]);
        HAL_Delay(PAUSE_MS);

        /* Reverse */
        status = MD10C_SetSpeed(motors[i], -TEST_SPEED_PCT);
        if (status != HAL_OK) {
            MD10C_StopAll();
            DCMotorTest_BlinkError();
            return;
        }
        HAL_Delay(RUN_MS);

        MD10C_Stop(motors[i]);
        HAL_Delay(PAUSE_MS);
    }

    /* --- Phase 2: All motors forward --- */
    for (int i = 0; i < 4; i++) {
        status = MD10C_SetSpeed(motors[i], TEST_SPEED_PCT);
        if (status != HAL_OK) {
            MD10C_StopAll();
            DCMotorTest_BlinkError();
            return;
        }
    }
    HAL_Delay(RUN_MS);

    MD10C_StopAll();
    HAL_Delay(PAUSE_MS);

    /* --- Phase 3: All motors reverse --- */
    for (int i = 0; i < 4; i++) {
        status = MD10C_SetSpeed(motors[i], -TEST_SPEED_PCT);
        if (status != HAL_OK) {
            MD10C_StopAll();
            DCMotorTest_BlinkError();
            return;
        }
    }
    HAL_Delay(RUN_MS);

    /* Always stop before finishing */
    MD10C_StopAll();

    /* Test passed */
    DCMotorTest_BlinkOK();
}

/**
 * @brief  Blink LD2 three times to indicate success
 */
static void DCMotorTest_BlinkOK(void)
{
    for (int i = 0; i < 3; i++) {
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        HAL_Delay(200);
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        HAL_Delay(200);
    }
}

/**
 * @brief  Fast-blink LD2 to indicate an error
 *
 * Blinks 10 times rapidly so it's visually distinct from the
 * success pattern.
 */
static void DCMotorTest_BlinkError(void)
{
    /* Safety: ensure motors are stopped on error */
    MD10C_StopAll();

    for (int i = 0; i < 10; i++) {
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        HAL_Delay(50);
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        HAL_Delay(50);
    }
}

/**
 * @brief  Check if the B1 user button is currently pressed
 *
 * On the Nucleo-144, B1 (PC13) is active LOW -- pressing the
 * button pulls the pin to ground.
 *
 * @return 1 if pressed, 0 if released
 */
static uint8_t DCMotorTest_ButtonPressed(void)
{
    return (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET);
}
