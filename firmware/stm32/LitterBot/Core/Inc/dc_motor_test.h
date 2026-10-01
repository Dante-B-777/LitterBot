/**
 * @file    dc_motor_test.h
 * @brief   Cytron MD10C DC motor test triggered by user button
 *
 * Press the blue B1 button (PC13) on the Nucleo-144 to start the test.
 * The test runs each motor individually, then all together:
 *   Phase 1: FL fwd/rev, FR fwd/rev, RL fwd/rev, RR fwd/rev (individual)
 *   Phase 2: All forward
 *   Phase 3: All reverse
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
 *   #include "dc_motor_test.h"
 *   // after peripheral init:
 *   DCMotorTest_Init();
 *   // in while(1) loop:
 *   DCMotorTest_Poll();
 *
 * @author  LitterBot Team
 * @date    2026
 */

#ifndef DC_MOTOR_TEST_H
#define DC_MOTOR_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

/** @brief Maximum speed percentage for safety during testing */
#define TEST_SPEED_PCT      25.0f

/**
 * @brief  Initialize the DC motor test module
 *
 * Calls MD10C_Init() to set up PWM and direction GPIO,
 * then ensures all motors are stopped.
 */
void DCMotorTest_Init(void);

/**
 * @brief  Poll for button press and run test when triggered
 *
 * Call this repeatedly from the main while(1) loop.
 * When B1 is pressed, runs the DC motor test sequence (blocking),
 * then returns. Includes debouncing so one press = one test run.
 */
void DCMotorTest_Poll(void);

#ifdef __cplusplus
}
#endif

#endif /* DC_MOTOR_TEST_H */
