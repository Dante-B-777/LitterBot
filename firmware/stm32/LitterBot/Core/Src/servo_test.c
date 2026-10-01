/**
 * @file    servo_test.c
 * @brief   HX-65HM servo range-of-motion test
 *
 * Waits for the user button press, then sweeps the servo through
 * its full 360-degree range in steps, pausing at each position.
 *
 * The LD2 LED (PB7) indicates test status:
 *   - OFF:      idle, waiting for button press
 *   - ON:       test is running
 *   - BLINK 3x: test complete (success)
 *   - FAST BLINK: error occurred
 *
 * @author  LitterBot Team
 * @date    2026
 */

/* Includes ------------------------------------------------------------------*/
#include "servo_test.h"
#include "hx65_servo.h"
#include "main.h"

/* Private defines -----------------------------------------------------------*/

/** @brief Time in ms to hold each position during the sweep */
#define SWEEP_HOLD_MS       1500

/** @brief Time in ms for servo to travel between positions */
#define SWEEP_MOVE_MS       1000

/** @brief Debounce delay in ms after detecting button press */
#define DEBOUNCE_MS         200

/* Private variables ---------------------------------------------------------*/

/** @brief Tracks whether the test module has been initialized */
static uint8_t initialized = 0;

/* Private function prototypes -----------------------------------------------*/
static void ServoTest_RunSweep(void);
static void ServoTest_BlinkOK(void);
static void ServoTest_BlinkError(void);
static uint8_t ServoTest_ButtonPressed(void);

/* Public functions ----------------------------------------------------------*/

void ServoTest_Init(UART_HandleTypeDef *huart)
{
    HX65_Init(huart);
    initialized = 1;

    /* Turn off LD2 to indicate idle state */
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
}

void ServoTest_Poll(void)
{
    if (!initialized) {
        return;
    }

    /* Check if button is pressed */
    if (ServoTest_ButtonPressed()) {
        /* Debounce: wait and verify button is still pressed */
        HAL_Delay(DEBOUNCE_MS);

        if (ServoTest_ButtonPressed()) {
            /* LD2 ON = test running */
            HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);

            ServoTest_RunSweep();

            /* Wait for button release before returning to idle */
            while (ServoTest_ButtonPressed()) {
                HAL_Delay(10);
            }
            HAL_Delay(DEBOUNCE_MS);
        }
    }
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Run the servo sweep sequence
 *
 * Moves the servo through these positions:
 *   1. Position 0    (0 degrees)    - one end of travel
 *   2. Position 1024 (90 degrees)   - quarter turn
 *   3. Position 2048 (180 degrees)  - midpoint
 *   4. Position 3072 (270 degrees)  - three-quarter turn
 *   5. Position 4095 (360 degrees)  - full rotation
 *   6. Position 2048 (180 degrees)  - return to center
 *
 * Each move takes SWEEP_MOVE_MS to complete (servo interpolates),
 * then holds for SWEEP_HOLD_MS before the next move.
 */
static void ServoTest_RunSweep(void)
{
    HX65_Status_t status;

    /* Positions to visit during the sweep */
    const uint16_t positions[] = { 0, 1024, 2048, 3072, 4095, 2048 };
    const int num_positions = sizeof(positions) / sizeof(positions[0]);

    /* First, try to ping the servo to verify communication */
    status = HX65_Ping(SERVO_TEST_ID);
    if (status != HX65_OK) {
        /* Servo not responding -- blink error and abort */
        ServoTest_BlinkError();
        return;
    }

    /* Small delay after ping before sending move commands */
    HAL_Delay(100);

    /* Sweep through each position */
    for (int i = 0; i < num_positions; i++) {
        status = HX65_MovePosition(SERVO_TEST_ID, positions[i], SWEEP_MOVE_MS);
        if (status != HX65_OK) {
            ServoTest_BlinkError();
            return;
        }

        /* Wait for move to complete + hold time */
        HAL_Delay(SWEEP_MOVE_MS + SWEEP_HOLD_MS);
    }

    /* Test passed */
    ServoTest_BlinkOK();
}

/**
 * @brief  Blink LD2 three times to indicate success
 */
static void ServoTest_BlinkOK(void)
{
    for (int i = 0; i < 3; i++) {
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
        HAL_Delay(200);
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
        HAL_Delay(200);
    }
}

/**
 * @brief  Fast-blink LD2 to indicate an error
 *
 * Blinks 10 times rapidly so it's visually distinct from the
 * success pattern.
 */
static void ServoTest_BlinkError(void)
{
    for (int i = 0; i < 10; i++) {
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
        HAL_Delay(50);
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
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
static uint8_t ServoTest_ButtonPressed(void)
{
    return (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET);
}
