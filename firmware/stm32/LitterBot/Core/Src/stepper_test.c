/**
 * @file    stepper_test.c
 * @brief   CL57T stepper motor test
 *
 * Waits for the user button press, then runs the stepper through
 * three speed phases (slow, medium, fast). Each phase moves 200 steps
 * CW then 200 steps CCW, returning to the starting position.
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
#include "stepper_test.h"
#include "cl57t_stepper.h"
#include "main.h"

/* Private defines -----------------------------------------------------------*/

/** @brief Number of steps per direction per phase */
#define STEPS_PER_MOVE      200

/** @brief Step delay for slow phase (microseconds) */
#define DELAY_SLOW_US       2000

/** @brief Step delay for medium phase (microseconds) */
#define DELAY_MEDIUM_US     1000

/** @brief Step delay for fast phase (microseconds) */
#define DELAY_FAST_US       500

/** @brief Pause time in ms between phases */
#define PHASE_PAUSE_MS      1000

/** @brief Debounce delay in ms after detecting button press */
#define DEBOUNCE_MS         200

/* Private variables ---------------------------------------------------------*/

/** @brief Tracks whether the test module has been initialized */
static uint8_t initialized = 0;

/* Private function prototypes -----------------------------------------------*/
static void StepperTest_RunSequence(void);
static void StepperTest_BlinkOK(void);
static void StepperTest_BlinkError(void);
static uint8_t StepperTest_ButtonPressed(void);

/* Public functions ----------------------------------------------------------*/

void StepperTest_Init(void)
{
    CL57T_Init();
    initialized = 1;

    /* Turn off LD2 to indicate idle state */
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
}

void StepperTest_Poll(void)
{
    if (!initialized) {
        return;
    }

    /* Check if button is pressed */
    if (StepperTest_ButtonPressed()) {
        /* Debounce: wait and verify button is still pressed */
        HAL_Delay(DEBOUNCE_MS);

        if (StepperTest_ButtonPressed()) {
            /* LD2 ON = test running */
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

            StepperTest_RunSequence();

            /* Wait for button release before returning to idle */
            while (StepperTest_ButtonPressed()) {
                HAL_Delay(10);
            }
            HAL_Delay(DEBOUNCE_MS);
        }
    }
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Run the stepper test sequence
 *
 * Three phases at increasing speed, each returning to start:
 *   Phase 1 (Slow):   200 steps CW at 2000us, 200 steps CCW at 2000us
 *   Phase 2 (Medium): 200 steps CW at 1000us, 200 steps CCW at 1000us
 *   Phase 3 (Fast):   200 steps CW at  500us, 200 steps CCW at  500us
 *
 * 1 second pause between phases.
 * Total duration: ~7 seconds
 */
static void StepperTest_RunSequence(void)
{
    const uint32_t delays[] = { DELAY_SLOW_US, DELAY_MEDIUM_US, DELAY_FAST_US };
    const int num_phases = sizeof(delays) / sizeof(delays[0]);

    for (int i = 0; i < num_phases; i++) {
        /* CW */
        CL57T_MoveSteps(STEPS_PER_MOVE, CL57T_DIR_CW, delays[i]);

        /* Brief pause before reversing */
        HAL_Delay(200);

        /* CCW (return to start) */
        CL57T_MoveSteps(STEPS_PER_MOVE, CL57T_DIR_CCW, delays[i]);

        /* Pause between phases (skip after last phase) */
        if (i < num_phases - 1) {
            HAL_Delay(PHASE_PAUSE_MS);
        }
    }

    /* Test passed */
    StepperTest_BlinkOK();
}

/**
 * @brief  Blink LD2 three times to indicate success
 */
static void StepperTest_BlinkOK(void)
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
static void StepperTest_BlinkError(void)
{
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
static uint8_t StepperTest_ButtonPressed(void)
{
    return (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET);
}
