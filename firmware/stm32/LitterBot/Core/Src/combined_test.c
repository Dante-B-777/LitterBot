/**
 * @file    combined_test.c
 * @brief   Combined peripheral test implementation
 *
 * Runs all subsystems in sequence on a single button press:
 * DC motors (fwd, rev, left, right), stepper (fwd), servo (up, down).
 *
 * @author  LitterBot Team
 * @date    2026
 */

/* Includes ------------------------------------------------------------------*/
#include "combined_test.h"
#include "cytron_md10c.h"
#include "cl57t_stepper.h"
#include "hx65_servo.h"
#include "main.h"

/* Private defines -----------------------------------------------------------*/

/** @brief Duration each DC motor phase runs (milliseconds) */
#define DC_PHASE_MS         1500

/** @brief Pause between phases (milliseconds) */
#define PHASE_PAUSE_MS      500

/** @brief Number of stepper steps for the forward move */
#define STEPPER_STEPS       200

/** @brief Step delay for stepper (microseconds) -- medium speed */
#define STEPPER_DELAY_US    1000

/** @brief Servo "up" position (270 degrees, 3/4 turn) */
#define SERVO_POS_UP        3072

/** @brief Servo "down" position (90 degrees, 1/4 turn) */
#define SERVO_POS_DOWN      1024

/** @brief Time for servo to travel between positions (milliseconds) */
#define SERVO_MOVE_MS       1000

/** @brief Hold time at each servo position (milliseconds) */
#define SERVO_HOLD_MS       1000

/** @brief Debounce delay in ms after detecting button press */
#define DEBOUNCE_MS         200

/* Private variables ---------------------------------------------------------*/

/** @brief Tracks whether the test module has been initialized */
static uint8_t initialized = 0;

/* Private function prototypes -----------------------------------------------*/
static void CombinedTest_RunSequence(void);
static void CombinedTest_BlinkOK(void);
static void CombinedTest_BlinkError(void);
static uint8_t CombinedTest_ButtonPressed(void);

/* Public functions ----------------------------------------------------------*/

void CombinedTest_Init(UART_HandleTypeDef *huart)
{
    /* Initialize all three subsystems */
    MD10C_Init();
    CL57T_Init();
    HX65_Init(huart);

    /* Make sure all motors are stopped */
    MD10C_StopAll();

    initialized = 1;

    /* Turn off LD2 to indicate idle state */
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
}

void CombinedTest_Poll(void)
{
    if (!initialized) {
        return;
    }

    /* Check if button is pressed */
    if (CombinedTest_ButtonPressed()) {
        /* Debounce: wait and verify button is still pressed */
        HAL_Delay(DEBOUNCE_MS);

        if (CombinedTest_ButtonPressed()) {
            /* LD2 ON = test running */
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

            CombinedTest_RunSequence();

            /* Wait for button release before returning to idle */
            while (CombinedTest_ButtonPressed()) {
                HAL_Delay(10);
            }
            HAL_Delay(DEBOUNCE_MS);
        }
    }
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Run the full combined test sequence
 *
 * Order of operations:
 *   1. Forward  -- all 4 DC motors forward at 25%
 *   2. Reverse  -- all 4 DC motors reverse at 25%
 *   3. Left     -- skid-steer left (left side reverse, right side forward)
 *   4. Right    -- skid-steer right (left side forward, right side reverse)
 *   5. Stepper  -- 200 steps CW then 200 steps CCW (returns to start)
 *   6. Servo up -- move to 270 degrees
 *   7. Servo down -- move to 90 degrees
 *
 * Motors are always stopped between phases and on any error.
 */
static void CombinedTest_RunSequence(void)
{
    HX65_Status_t servo_status;

    /* ---- Phase 1: DC motors forward ---- */
    MD10C_SetSpeed(MD10C_MOTOR_FL, COMBINED_TEST_SPEED_PCT);
    MD10C_SetSpeed(MD10C_MOTOR_FR, COMBINED_TEST_SPEED_PCT);
    MD10C_SetSpeed(MD10C_MOTOR_RL, COMBINED_TEST_SPEED_PCT);
    MD10C_SetSpeed(MD10C_MOTOR_RR, COMBINED_TEST_SPEED_PCT);
    HAL_Delay(DC_PHASE_MS);
    MD10C_StopAll();
    HAL_Delay(PHASE_PAUSE_MS);

    /* ---- Phase 2: DC motors reverse ---- */
    MD10C_SetSpeed(MD10C_MOTOR_FL, -COMBINED_TEST_SPEED_PCT);
    MD10C_SetSpeed(MD10C_MOTOR_FR, -COMBINED_TEST_SPEED_PCT);
    MD10C_SetSpeed(MD10C_MOTOR_RL, -COMBINED_TEST_SPEED_PCT);
    MD10C_SetSpeed(MD10C_MOTOR_RR, -COMBINED_TEST_SPEED_PCT);
    HAL_Delay(DC_PHASE_MS);
    MD10C_StopAll();
    HAL_Delay(PHASE_PAUSE_MS);

    /* ---- Phase 3: Skid-steer left ---- */
    /* Left motors reverse, right motors forward */
    MD10C_SetSpeed(MD10C_MOTOR_FL, -COMBINED_TEST_SPEED_PCT);
    MD10C_SetSpeed(MD10C_MOTOR_RL, -COMBINED_TEST_SPEED_PCT);
    MD10C_SetSpeed(MD10C_MOTOR_FR, COMBINED_TEST_SPEED_PCT);
    MD10C_SetSpeed(MD10C_MOTOR_RR, COMBINED_TEST_SPEED_PCT);
    HAL_Delay(DC_PHASE_MS);
    MD10C_StopAll();
    HAL_Delay(PHASE_PAUSE_MS);

    /* ---- Phase 4: Skid-steer right ---- */
    /* Left motors forward, right motors reverse */
    MD10C_SetSpeed(MD10C_MOTOR_FL, COMBINED_TEST_SPEED_PCT);
    MD10C_SetSpeed(MD10C_MOTOR_RL, COMBINED_TEST_SPEED_PCT);
    MD10C_SetSpeed(MD10C_MOTOR_FR, -COMBINED_TEST_SPEED_PCT);
    MD10C_SetSpeed(MD10C_MOTOR_RR, -COMBINED_TEST_SPEED_PCT);
    HAL_Delay(DC_PHASE_MS);
    MD10C_StopAll();
    HAL_Delay(PHASE_PAUSE_MS);

    /* ---- Phase 5: Stepper forward and back ---- */
    CL57T_MoveSteps(STEPPER_STEPS, CL57T_DIR_CW, STEPPER_DELAY_US);
    HAL_Delay(200);
    CL57T_MoveSteps(STEPPER_STEPS, CL57T_DIR_CCW, STEPPER_DELAY_US);
    HAL_Delay(PHASE_PAUSE_MS);

    /* ---- Phase 6: Servo up ---- */
    servo_status = HX65_MovePosition(COMBINED_SERVO_ID, SERVO_POS_UP, SERVO_MOVE_MS);
    if (servo_status != HX65_OK) {
        CombinedTest_BlinkError();
        return;
    }
    HAL_Delay(SERVO_MOVE_MS + SERVO_HOLD_MS);

    /* ---- Phase 7: Servo down ---- */
    servo_status = HX65_MovePosition(COMBINED_SERVO_ID, SERVO_POS_DOWN, SERVO_MOVE_MS);
    if (servo_status != HX65_OK) {
        CombinedTest_BlinkError();
        return;
    }
    HAL_Delay(SERVO_MOVE_MS + SERVO_HOLD_MS);

    /* All phases complete */
    CombinedTest_BlinkOK();
}

/**
 * @brief  Blink LD2 three times to indicate success
 */
static void CombinedTest_BlinkOK(void)
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
 * success pattern. Ensures motors are stopped on error.
 */
static void CombinedTest_BlinkError(void)
{
    /* Safety: stop all DC motors on error */
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
static uint8_t CombinedTest_ButtonPressed(void)
{
    return (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET);
}
