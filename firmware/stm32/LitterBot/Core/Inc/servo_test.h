/**
 * @file    servo_test.h
 * @brief   HX-65HM servo range-of-motion test triggered by user button
 *
 * Press the blue B1 button (PC13) on the Nucleo-144 to start the test.
 * The servo sweeps through its full range of motion:
 *   0 -> 1024 -> 2048 -> 3072 -> 4095 -> 2048 (center)
 *
 * LD2 (blue LED, PB7) lights up while the test is running.
 *
 * Usage in main.c:
 *   #include "servo_test.h"
 *   // after peripheral init:
 *   ServoTest_Init(&huart3);
 *   // in while(1) loop:
 *   ServoTest_Poll();
 *
 * @author  LitterBot Team
 * @date    2026
 */

#ifndef SERVO_TEST_H
#define SERVO_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

/** @brief Default servo ID to test (factory default is typically 1) */
#define SERVO_TEST_ID       1

/**
 * @brief  Initialize the servo test module
 *
 * Passes the UART handle to the HX65 driver and prepares for
 * button-triggered testing.
 *
 * @param  huart  Pointer to the half-duplex UART handle (USART3)
 */
void ServoTest_Init(UART_HandleTypeDef *huart);

/**
 * @brief  Poll for button press and run test when triggered
 *
 * Call this repeatedly from the main while(1) loop.
 * When B1 is pressed, runs the servo sweep sequence (blocking),
 * then returns. Includes debouncing so one press = one test run.
 */
void ServoTest_Poll(void);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_TEST_H */
