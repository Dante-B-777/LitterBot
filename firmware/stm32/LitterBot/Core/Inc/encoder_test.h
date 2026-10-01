/**
 * @file    encoder_test.h
 * @brief   Quadrature encoder test module
 *
 * Two ways to trigger the encoder test:
 *
 *   1. UART command: Send "T\n" over serial (handled by uart_cmd.c).
 *      This is the intended method when using ROS2 teleop -- your
 *      bridge node sends "T\n" when an unused teleop key is pressed.
 *
 *   2. B1 button: Press the blue button (PC13) on the Nucleo board.
 *      Works in standalone mode (EncoderTest_Init + EncoderTest_Poll).
 *
 * The test runs all motors forward then reverse at 25%, reads encoder
 * ticks after each phase, and prints results via the provided UART.
 *
 * Uses LD3 (PB14, red LED) instead of LD2 (PB7) because PB7 is
 * shared with TIM4_CH2 (FL encoder channel B).
 *
 * LD3 (red LED, PB14) indicates test status:
 *   - OFF:      idle
 *   - ON:       test running
 *   - BLINK 3x: success (all encoders reading non-zero)
 *   - FAST BLINK: error (one or more encoders read 0 = no signal)
 *
 * Test sequence (~5 seconds):
 *   1. Reset encoders and odometry
 *   2. All motors forward 25% for 2 seconds
 *   3. Stop, print ticks (should be positive and similar)
 *   4. All motors reverse 25% for 2 seconds
 *   5. Stop, print ticks (should be near zero)
 *   6. Print odometry pose
 *
 * Standalone usage in main.c:
 *   #include "encoder_test.h"
 *   EncoderTest_Init(&hlpuart1);
 *   while (1) { EncoderTest_Poll(); }
 *
 * Combined with UART teleop (preferred):
 *   // encoder_test.h is included by uart_cmd.c
 *   Encoder_Init();
 *   Odometry_Init();
 *   UartCmd_Init(&hlpuart1);
 *   // uart_cmd handles "T\n" -> EncoderTest_Run()
 *
 * @author  LitterBot Team
 * @date    2026
 */

#ifndef ENCODER_TEST_H
#define ENCODER_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Public function prototypes ------------------------------------------------*/

/**
 * @brief  Run the encoder test sequence (blocking, ~5 seconds)
 *
 * Call this to execute the test immediately. Assumes Encoder_Init()
 * and MD10C_Init() have already been called. Prints results via
 * the provided UART handle, signals result via LD3.
 *
 * This is what uart_cmd.c calls when it receives "T\n".
 *
 * @param  huart  UART handle for printing results (typically &hlpuart1)
 */
void EncoderTest_Run(UART_HandleTypeDef *huart);

/**
 * @brief  Initialize the encoder test module (standalone mode)
 *
 * Initializes DC motor driver, encoder driver, and odometry.
 * Only needed when running encoder_test standalone (without uart_cmd).
 *
 * @param  hlpuart  UART handle for printing results (typically &hlpuart1)
 */
void EncoderTest_Init(UART_HandleTypeDef *hlpuart);

/**
 * @brief  Poll for B1 button press and run test when triggered
 *
 * Call this repeatedly from the main while(1) loop.
 * Only needed in standalone mode (without uart_cmd).
 */
void EncoderTest_Poll(void);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_TEST_H */
