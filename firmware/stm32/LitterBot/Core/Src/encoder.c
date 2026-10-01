/**
 * @file    encoder.c
 * @brief   Quadrature encoder driver implementation
 *
 * Uses STM32 timer encoder mode for hardware quadrature counting.
 * Each timer's 16-bit counter automatically increments/decrements
 * on encoder edges. This driver samples the counters, computes
 * signed deltas (handling 16-bit wrap-around), and accumulates
 * into 32-bit tick counts.
 *
 * 16-bit overflow handling:
 *   The timer counter wraps at 0 and 65535. By casting the counter
 *   to int16_t and subtracting the previous int16_t value, the
 *   signed result correctly handles wrap-around in either direction
 *   (assuming less than 32767 ticks elapse between updates -- at
 *   537.7 ticks/rev and 312 RPM max, that's ~58 updates/sec minimum).
 *
 * @author  LitterBot Team
 * @date    2026
 */

/* Includes ------------------------------------------------------------------*/
#include "encoder.h"

/* External timer handles from CubeMX-generated code -------------------------*/
extern TIM_HandleTypeDef htim1;   /* RR encoder */
extern TIM_HandleTypeDef htim3;   /* FR encoder */
extern TIM_HandleTypeDef htim4;   /* FL encoder */
extern TIM_HandleTypeDef htim8;   /* RL encoder */

/* Private types -------------------------------------------------------------*/

/** @brief Internal per-encoder state */
typedef struct {
    TIM_HandleTypeDef *htim;     /**< Timer handle for this encoder */
    int16_t            prev_cnt; /**< Previous counter reading (for delta) */
    Encoder_Data_t     data;     /**< Public data (ticks, delta, velocity) */
} Encoder_State_t;

/* Private variables ---------------------------------------------------------*/

/** @brief Encoder state array indexed by Encoder_ID_t */
static Encoder_State_t encoders[ENC_COUNT];

/** @brief Timestamp of last Encoder_Update() call for dt calculation */
static uint32_t last_update_ms = 0;

/** @brief Flag indicating whether Encoder_Init() has been called */
static uint8_t initialized = 0;

/* Public functions ----------------------------------------------------------*/

HAL_StatusTypeDef Encoder_Init(void)
{
    /* Map timers to encoder positions */
    encoders[ENC_FL].htim = &htim4;
    encoders[ENC_FR].htim = &htim3;
    encoders[ENC_RL].htim = &htim8;
    encoders[ENC_RR].htim = &htim1;

    /* Zero out all state */
    for (int i = 0; i < ENC_COUNT; i++) {
        encoders[i].prev_cnt       = 0;
        encoders[i].data.ticks     = 0;
        encoders[i].data.delta     = 0;
        encoders[i].data.velocity_rps = 0.0f;
    }

    /* Start all timers in encoder mode (both channels) */
    for (int i = 0; i < ENC_COUNT; i++) {
        if (HAL_TIM_Encoder_Start(encoders[i].htim,
                                  TIM_CHANNEL_ALL) != HAL_OK) {
            return HAL_ERROR;
        }
        /* Read initial counter value */
        encoders[i].prev_cnt = (int16_t)__HAL_TIM_GET_COUNTER(encoders[i].htim);
    }

    last_update_ms = HAL_GetTick();
    initialized = 1;

    return HAL_OK;
}

void Encoder_Update(void)
{
    if (!initialized) {
        return;
    }

    uint32_t now_ms = HAL_GetTick();
    uint32_t elapsed_ms = now_ms - last_update_ms;

    /* Avoid division by zero on very fast calls */
    if (elapsed_ms == 0) {
        elapsed_ms = 1;
    }

    float dt = (float)elapsed_ms / 1000.0f;

    for (int i = 0; i < ENC_COUNT; i++) {
        /* Read 16-bit counter */
        int16_t raw = (int16_t)__HAL_TIM_GET_COUNTER(encoders[i].htim);

        /* Signed subtraction handles 16-bit wrap-around correctly */
        int16_t delta = raw - encoders[i].prev_cnt;
        encoders[i].prev_cnt = raw;

        /* Accumulate into 32-bit ticks */
        encoders[i].data.delta  = (int32_t)delta;
        encoders[i].data.ticks += (int32_t)delta;

        /* Velocity in output shaft revolutions per second */
        encoders[i].data.velocity_rps =
            (float)delta / ENCODER_TICKS_PER_REV / dt;
    }

    last_update_ms = now_ms;
}

const Encoder_Data_t* Encoder_Get(Encoder_ID_t id)
{
    if (id >= ENC_COUNT) {
        return NULL;
    }
    return &encoders[id].data;
}

int32_t Encoder_GetTicks(Encoder_ID_t id)
{
    if (id >= ENC_COUNT) {
        return 0;
    }
    return encoders[id].data.ticks;
}

void Encoder_ResetAll(void)
{
    for (int i = 0; i < ENC_COUNT; i++) {
        /* Zero hardware counter */
        __HAL_TIM_SET_COUNTER(encoders[i].htim, 0);
        encoders[i].prev_cnt       = 0;
        encoders[i].data.ticks     = 0;
        encoders[i].data.delta     = 0;
        encoders[i].data.velocity_rps = 0.0f;
    }

    last_update_ms = HAL_GetTick();
}
