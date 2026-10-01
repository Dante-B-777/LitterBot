/**
 * @file    uart_cmd.h
 * @brief   UART command parser for teleop motor control
 *
 * Adapted from LB_FIRM's wheels4.c to work with the main firmware's
 * cytron_md10c motor driver. Receives ASCII commands over LPUART1
 * (ST-LINK VCP) and drives motors via MD10C_SetSpeed().
 *
 * Protocol:
 *   M <fl> <fr> <rl> <rr>\n  — Set motor speeds (-1000 to +1000)     [GATED]
 *   X\n                       — Emergency stop                         [ALWAYS]
 *   S SCOOP\n                 — Execute scoop sequence (blocking)      [GATED]
 *   C CALIBRATE\n             — Run arm calibration                    [GATED]
 *   D DEPLOY\n                — Deploy (arm homing → ACTIVE)           [STANDBY/ACTIVE]
 *   DEV ON\n / DEV OFF\n      — Toggle dev mode                        [ALWAYS]
 *   Q\n                       — Query state (responds STATE <state>)   [ALWAYS]
 *   T\n                       — Run encoder test (~5 sec, blocking)    [ALWAYS]
 *
 * [GATED] commands are blocked in STANDBY unless dev mode is on.
 * STM32 responds with ERR:STANDBY\r\n when a gated command is rejected.
 *
 * Safety:
 *   - 250ms timeout: motors auto-stop if no command received
 *   - Values clamped to [-1000, +1000] range
 *   - Maps to MD10C_SetSpeed() range of [-100.0, +100.0]%
 *
 * Usage in main.c (requires UART RX callback):
 *   #include "uart_cmd.h"
 *   static uint8_t rx_byte;
 *
 *   // in USER CODE BEGIN 2:
 *   MD10C_Init();
 *   UartCmd_Init(&hlpuart1);
 *   HAL_UART_Receive_IT(&hlpuart1, &rx_byte, 1);
 *
 *   // in while(1) loop:
 *   UartCmd_Task(HAL_GetTick());
 *
 *   // in USER CODE BEGIN 4:
 *   void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
 *     if (huart->Instance == LPUART1) {
 *       UartCmd_OnRxByte(rx_byte);
 *       HAL_UART_Receive_IT(&hlpuart1, &rx_byte, 1);
 *     }
 *   }
 *
 * @author  LitterBot Team
 * @date    2026
 */

#ifndef UART_CMD_H
#define UART_CMD_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include <stdint.h>

/* Public function prototypes ------------------------------------------------*/

/**
 * @brief  Initialize the UART command parser
 *
 * Initializes the DC motor driver (MD10C_Init) and resets the
 * command parser state. Does NOT start UART RX interrupt -- caller
 * must call HAL_UART_Receive_IT() separately.
 *
 * @param  huart  Pointer to UART handle (typically &hlpuart1)
 */
void UartCmd_Init(UART_HandleTypeDef *huart);

/**
 * @brief  Feed a received byte into the command parser
 *
 * Call this from HAL_UART_RxCpltCallback() for each byte received.
 * Buffers characters until '\n' is received, then marks the line
 * as ready for processing by UartCmd_Task().
 *
 * @param  byte  The received byte
 */
void UartCmd_OnRxByte(uint8_t byte);

/**
 * @brief  Process pending commands and check timeout
 *
 * Call this from the main loop. Processes any complete command line
 * and stops motors if the 250ms command timeout has elapsed.
 *
 * @param  now_ms  Current system tick (HAL_GetTick())
 */
void UartCmd_Task(uint32_t now_ms);

/**
 * @brief  Periodically publish encoder telemetry over LPUART1
 *
 * Emits absolute encoder tick counts in the form:
 *   ENC <fl> <fr> <rl> <rr>\r\n
 *
 * Call this from the main loop after Encoder_Update().
 *
 * @param  now_ms  Current system tick (HAL_GetTick())
 */
void UartCmd_TelemetryTask(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* UART_CMD_H */
