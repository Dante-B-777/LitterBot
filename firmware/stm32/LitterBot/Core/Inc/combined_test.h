/**
 * @file    combined_test.h
 * @brief   Combined peripheral test -- exercises all subsystems in sequence
 *
 * Press the blue B1 button (PC13) on the Nucleo-144 to start the test.
 * The sequence exercises every peripheral in order:
 *   1. DC motors forward  (all 4 wheels, 25%, 1.5s)
 *   2. DC motors reverse  (all 4 wheels, 25%, 1.5s)
 *   3. DC motors left     (skid-steer: left reverse, right forward, 1.5s)
 *   4. DC motors right    (skid-steer: left forward, right reverse, 1.5s)
 *   5. Stepper forward    (200 steps CW at 1000us delay)
 *   6. Servo up           (move to position 3072 / 270 degrees)
 *   7. Servo down         (move to position 1024 / 90 degrees)
 *
 * Press the button again to repeat the entire sequence.
 *
 * Speed is capped at 25% for safety during testing.
 *
 * LD2 (blue LED, PB7) indicates test status:
 *   - OFF:      idle
 *   - ON:       test running
 *   - BLINK 3x: success
 *   - FAST BLINK: error
 *
 * Usage in main.c:
 *   #include "combined_test.h"
 *   // after peripheral init:
 *   CombinedTest_Init(&huart3);
 *   // in while(1) loop:
 *   CombinedTest_Poll();
 *
 * @author  LitterBot Team
 * @date    2026
 */

#ifndef COMBINED_TEST_H
#define COMBINED_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

/** @brief Maximum DC motor speed percentage for safety during testing */
#define COMBINED_TEST_SPEED_PCT     25.0f

/** @brief Servo ID to use for the test */
#define COMBINED_SERVO_ID           1

/**
 * @brief  Initialize the combined test module
 *
 * Calls MD10C_Init(), CL57T_Init(), and HX65_Init() to set up
 * all peripherals. Ensures motors are stopped on entry.
 *
 * @param  huart  Pointer to the UART handle for servo communication
 *                (must be initialized in half-duplex mode at 115200 baud)
 */
void CombinedTest_Init(UART_HandleTypeDef *huart);

/**
 * @brief  Poll for button press and run test when triggered
 *
 * Call this repeatedly from the main while(1) loop.
 * When B1 is pressed, runs the full combined test sequence (blocking),
 * then returns. Includes debouncing so one press = one test run.
 */
void CombinedTest_Poll(void);

#ifdef __cplusplus
}
#endif

#endif /* COMBINED_TEST_H */
