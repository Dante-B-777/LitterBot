/**
 * @file    encoder_test.c
 * @brief   Quadrature encoder test implementation
 *
 * Runs motors forward then reverse at 25% speed while reading encoder
 * ticks. Prints results via UART and signals success/error via LD3.
 *
 * Two entry points:
 *   - EncoderTest_Run(huart): called by uart_cmd.c when "T\n" received
 *   - EncoderTest_Poll():     polls B1 button in standalone mode
 *
 * Uses LD3 (PB14, red LED) because PB7 (LD2) conflicts with
 * TIM4_CH2 (FL encoder channel B).
 *
 * @author  LitterBot Team
 * @date    2026
 */

/* Includes ------------------------------------------------------------------*/
#include "encoder_test.h"
#include "encoder.h"
#include "odometry.h"
#include "cytron_md10c.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

/* Private defines -----------------------------------------------------------*/

/** @brief Motor test speed in percent (safety cap) */
#define TEST_SPEED_PCT      25.0f

/** @brief Duration to run motors in each direction (ms) */
#define RUN_MS              2000

/** @brief Pause after stopping motors before reading (ms) */
#define SETTLE_MS           200

/** @brief Button debounce delay (ms) */
#define DEBOUNCE_MS         200

/** @brief Encoder update interval during motor run (ms) */
#define UPDATE_INTERVAL_MS  20

/* Private variables ---------------------------------------------------------*/

/** @brief UART handle for standalone mode */
static UART_HandleTypeDef *g_huart = NULL;

/** @brief Flag: standalone module initialized */
static uint8_t initialized = 0;

/* Private function prototypes -----------------------------------------------*/
static void EncoderTest_RunSequence(UART_HandleTypeDef *huart);
static void EncoderTest_BlinkOK(void);
static void EncoderTest_BlinkError(void);
static uint8_t EncoderTest_ButtonPressed(void);
static void EncoderTest_Print(UART_HandleTypeDef *huart, const char *msg);
static void EncoderTest_PrintTicks(UART_HandleTypeDef *huart, const char *label);
static void EncoderTest_PrintPose(UART_HandleTypeDef *huart);
static void EncoderTest_RunMotorsWithUpdates(float speed_pct, uint32_t duration_ms);

/* Public functions ----------------------------------------------------------*/

void EncoderTest_Run(UART_HandleTypeDef *huart)
{
    /* LD3 ON = test running */
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);

    EncoderTest_RunSequence(huart);

    EncoderTest_Print(huart, "\r\nSend T to run again.\r\n");
}

void EncoderTest_Init(UART_HandleTypeDef *hlpuart)
{
    g_huart = hlpuart;

    /* Initialize subsystems */
    if (MD10C_Init() != HAL_OK) {
        initialized = 0;
        return;
    }

    if (Encoder_Init() != HAL_OK) {
        initialized = 0;
        return;
    }

    Odometry_Init();
    MD10C_StopAll();

    /* LD3 off = idle */
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

    initialized = 1;

    EncoderTest_Print(g_huart,
        "\r\nEncoder test ready. Press B1 button to start.\r\n");
}

void EncoderTest_Poll(void)
{
    if (!initialized) {
        return;
    }

    /* Trigger on B1 button press */
    if (EncoderTest_ButtonPressed()) {
        HAL_Delay(DEBOUNCE_MS);

        if (EncoderTest_ButtonPressed()) {
            /* LD3 ON = test running */
            HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);

            EncoderTest_RunSequence(g_huart);

            /* Wait for button release */
            while (EncoderTest_ButtonPressed()) {
                HAL_Delay(10);
            }
            HAL_Delay(DEBOUNCE_MS);

            EncoderTest_Print(g_huart,
                "\r\nPress B1 button to run again.\r\n");
        }
    }
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Run the full encoder test sequence (~5 seconds, blocking)
 */
static void EncoderTest_RunSequence(UART_HandleTypeDef *huart)
{
    EncoderTest_Print(huart, "\r\n=== ENCODER TEST START ===\r\n");

    /* Reset encoder counts and odometry */
    Encoder_ResetAll();
    Odometry_Reset();

    EncoderTest_PrintTicks(huart, "After reset");

    /* Phase 1: All motors forward at 25% for 2 seconds */
    EncoderTest_Print(huart, "\r\n--- Forward 25% for 2s ---\r\n");
    EncoderTest_RunMotorsWithUpdates(TEST_SPEED_PCT, RUN_MS);

    /* Stop and settle */
    MD10C_StopAll();
    HAL_Delay(SETTLE_MS);
    Encoder_Update();
    Odometry_Update();

    EncoderTest_PrintTicks(huart, "After forward");

    /* Check: all encoders should have non-zero ticks */
    uint8_t any_zero = 0;
    for (int i = 0; i < ENC_COUNT; i++) {
        if (Encoder_GetTicks((Encoder_ID_t)i) == 0) {
            any_zero = 1;
        }
    }

    /* Phase 2: All motors reverse at 25% for 2 seconds */
    EncoderTest_Print(huart, "\r\n--- Reverse 25% for 2s ---\r\n");
    EncoderTest_RunMotorsWithUpdates(-TEST_SPEED_PCT, RUN_MS);

    /* Stop and settle */
    MD10C_StopAll();
    HAL_Delay(SETTLE_MS);
    Encoder_Update();
    Odometry_Update();

    EncoderTest_PrintTicks(huart, "After reverse");
    EncoderTest_PrintPose(huart);

    EncoderTest_Print(huart, "\r\n=== ENCODER TEST END ===\r\n");

    /* Signal result */
    if (any_zero) {
        EncoderTest_Print(huart,
            "RESULT: ERROR -- one or more encoders read 0\r\n");
        EncoderTest_BlinkError();
    } else {
        EncoderTest_Print(huart,
            "RESULT: OK -- all encoders responding\r\n");
        EncoderTest_BlinkOK();
    }
}

/**
 * @brief  Run motors at given speed while updating encoders periodically
 */
static void EncoderTest_RunMotorsWithUpdates(float speed_pct,
                                              uint32_t duration_ms)
{
    MD10C_SetSpeed(MD10C_MOTOR_FL, speed_pct);
    MD10C_SetSpeed(MD10C_MOTOR_FR, speed_pct);
    MD10C_SetSpeed(MD10C_MOTOR_RL, speed_pct);
    MD10C_SetSpeed(MD10C_MOTOR_RR, speed_pct);

    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < duration_ms) {
        Encoder_Update();
        Odometry_Update();
        HAL_Delay(UPDATE_INTERVAL_MS);
    }
}

/**
 * @brief  Print a string via UART (blocking transmit)
 */
static void EncoderTest_Print(UART_HandleTypeDef *huart, const char *msg)
{
    if (huart) {
        HAL_UART_Transmit(huart, (uint8_t *)msg,
                          (uint16_t)strlen(msg), 100);
    }
}

/**
 * @brief  Print all encoder tick counts with a label
 */
static void EncoderTest_PrintTicks(UART_HandleTypeDef *huart,
                                    const char *label)
{
    char buf[128];
    snprintf(buf, sizeof(buf),
             "%s: FL=%ld  FR=%ld  RL=%ld  RR=%ld\r\n",
             label,
             (long)Encoder_GetTicks(ENC_FL),
             (long)Encoder_GetTicks(ENC_FR),
             (long)Encoder_GetTicks(ENC_RL),
             (long)Encoder_GetTicks(ENC_RR));
    EncoderTest_Print(huart, buf);
}

/**
 * @brief  Print current odometry pose
 */
static void EncoderTest_PrintPose(UART_HandleTypeDef *huart)
{
    const Odometry_Pose_t *p = Odometry_GetPose();
    char buf[128];

    snprintf(buf, sizeof(buf),
             "Odom: x=%.3f y=%.3f theta=%.2f deg\r\n",
             (double)p->x, (double)p->y,
             (double)(p->theta * 180.0f / 3.14159265f));
    EncoderTest_Print(huart, buf);
}

/**
 * @brief  Blink LD3 three times (success)
 */
static void EncoderTest_BlinkOK(void)
{
    for (int i = 0; i < 3; i++) {
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
        HAL_Delay(200);
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
        HAL_Delay(200);
    }
}

/**
 * @brief  Fast-blink LD3 ten times (error)
 */
static void EncoderTest_BlinkError(void)
{
    MD10C_StopAll();

    for (int i = 0; i < 10; i++) {
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
        HAL_Delay(50);
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
        HAL_Delay(50);
    }
}

/**
 * @brief  Check if B1 button is pressed (active LOW on Nucleo-144)
 */
static uint8_t EncoderTest_ButtonPressed(void)
{
    return (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET);
}
