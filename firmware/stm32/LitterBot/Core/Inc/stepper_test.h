/**
 * @file    stepper_test.h
 * @brief   CL57T stepper motor test triggered by user button
 *
 * Press the blue B1 button (PC13) on the Nucleo-144 to start the test.
 * The stepper runs through three speed phases, returning to the
 * starting position after each:
 *   Phase 1 (Slow):   200 steps CW, 200 steps CCW at 2000us delay
 *   Phase 2 (Medium): 200 steps CW, 200 steps CCW at 1000us delay
 *   Phase 3 (Fast):   200 steps CW, 200 steps CCW at 500us delay
 *
 * LD2 (blue LED, PB7) indicates test status:
 *   - OFF:      idle
 *   - ON:       test running
 *   - BLINK 3x: success
 *   - FAST BLINK: error
 *
 * Usage in main.c:
 *   #include "stepper_test.h"
 *   // after peripheral init:
 *   StepperTest_Init();
 *   // in while(1) loop:
 *   StepperTest_Poll();
 *
 * @author  LitterBot Team
 * @date    2026
 */

#ifndef STEPPER_TEST_H
#define STEPPER_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

/**
 * @brief  Initialize the stepper test module
 *
 * Calls CL57T_Init() to set up step/direction GPIO pins.
 */
void StepperTest_Init(void);

/**
 * @brief  Poll for button press and run test when triggered
 *
 * Call this repeatedly from the main while(1) loop.
 * When B1 is pressed, runs the stepper test sequence (blocking),
 * then returns. Includes debouncing so one press = one test run.
 */
void StepperTest_Poll(void);

#ifdef __cplusplus
}
#endif

#endif /* STEPPER_TEST_H */
