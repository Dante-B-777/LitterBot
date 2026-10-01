/**
 * @file    arm_control.c
 * @brief   Scoop arm control implementation
 *
 * Single-switch front-reference model:
 *   - PD15 is the front/extend reference switch
 *   - homing drives forward until PD15 is active and defines that as step 0
 *   - positive arm position is the retract offset back from that front zero
 */

/* Includes ------------------------------------------------------------------*/
#include "arm_control.h"
#include "cl57t_stepper.h"
#include "hx65_servo.h"
#include "limit_switch.h"
#include "main.h"
#include "calibration_store.h"
#include "scoop_sequence.h"

#include <stdio.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/

static volatile bool b1_abort = false;
static uint32_t g_arm_pos_steps = 0U;
static bool g_arm_pos_valid = false;

/* Private function prototypes -----------------------------------------------*/
static bool B1_Pressed(void);
static void print_uart(UART_HandleTypeDef *huart, const char *msg);
static uint16_t servo_speed_for_transition(uint32_t steps, uint32_t speed_us,
                                           uint16_t servo_start, uint16_t servo_end);
static ArmResult_t Arm_MoveWithServo(uint32_t steps, uint8_t dir, uint32_t speed_us,
                                     bool stop_on_extend_limit, uint8_t servo_id,
                                     bool use_servo, uint16_t servo_start,
                                     uint16_t servo_end, uint32_t hold_steps);
static bool arm_dir_is_forward(uint8_t dir);

/* Public functions ----------------------------------------------------------*/

void Arm_Init(void)
{
    extern UART_HandleTypeDef huart3;
    LimitSwitch_Init();
    CL57T_Init();
    HX65_Init(&huart3);
    g_arm_pos_steps = 0U;
    g_arm_pos_valid = false;
}

ArmResult_t Arm_Calibrate(UART_HandleTypeDef *huart)
{
    char buf[96];
    ArmResult_t rc;

    b1_abort = false;

    print_uart(huart, "\r\n=== ARM FRONT-ZERO CAL START ===\r\n");
    print_uart(huart, "Single-switch mode: PD15 is the front reference.\r\n");

    rc = Arm_Home(huart);
    if (rc != ARM_OK) {
        return rc;
    }

    snprintf(buf, sizeof(buf),
             "CAL ZEROED: front_ref=0 saved_retract=%lu\r\n",
             (unsigned long)CalStore_GetExtendSteps());
    print_uart(huart, buf);
    print_uart(huart,
               "Jog backward to the desired endpoint, then send CAL SAVE STEPS.\r\n");
    return ARM_OK;
}

ArmResult_t Arm_Extend(uint32_t steps, uint8_t dir, uint32_t speed_us)
{
    return Arm_MoveWithServo(steps, dir, speed_us, true, 0, false, 0, 0, 0);
}

ArmResult_t Arm_ExtendWithServo(uint32_t steps, uint8_t dir, uint32_t speed_us,
                                uint8_t servo_id, uint16_t servo_start,
                                uint16_t servo_end)
{
    return Arm_MoveWithServo(steps, dir, speed_us, true, servo_id, true,
                             servo_start, servo_end, 0);
}

ArmResult_t Arm_RetractWithServo(uint32_t steps, uint8_t dir, uint32_t speed_us,
                                 uint8_t servo_id, uint16_t servo_start,
                                 uint16_t servo_end)
{
    return Arm_MoveWithServo(steps, dir, speed_us, false, servo_id, true,
                             servo_start, servo_end, 0);
}

ArmResult_t Arm_RetractWithServoProfile(uint32_t steps, uint8_t dir,
                                        uint32_t speed_us, uint8_t servo_id,
                                        uint16_t servo_hold, uint16_t servo_end,
                                        uint32_t hold_steps)
{
    return Arm_MoveWithServo(steps, dir, speed_us, false, servo_id, true,
                             servo_hold, servo_end, hold_steps);
}

ArmResult_t Arm_Home(UART_HandleTypeDef *huart)
{
    char buf[96];
    CL57T_Direction_t forward_dir = (CL57T_Direction_t)SCOOP_EXTEND_DIR;

    if (LimitSwitch_HomeTriggered()) {
        g_arm_pos_steps = 0U;
        g_arm_pos_valid = true;
        print_uart(huart, "ARM HOME: Front reference already active. Zeroed.\r\n");
        return ARM_OK;
    }

    print_uart(huart, "ARM HOME: Driving forward to front reference switch...\r\n");

    CL57T_SetDirection(forward_dir);
    HAL_Delay(1);

    for (uint32_t i = 0; i < ARM_CAL_MAX_STEPS; i++) {
        if (B1_Pressed()) {
            b1_abort = true;
            print_uart(huart, "ARM HOME ABORT: B1 pressed.\r\n");
            return ARM_ERR_B1_ABORT;
        }

        if (LimitSwitch_HomeTriggered()) {
            snprintf(buf, sizeof(buf),
                     "ARM HOME: Front reference reached at %lu steps. Zeroed.\r\n",
                     (unsigned long)i);
            g_arm_pos_steps = 0U;
            g_arm_pos_valid = true;
            print_uart(huart, buf);
            return ARM_OK;
        }

        CL57T_Step();
        HAL_Delay(0);
        for (volatile uint32_t d = 0; d < ARM_CAL_SPEED_US * 10U; d++) { }
    }

    print_uart(huart, "ARM HOME ERROR: Max steps reached without hitting front reference!\r\n");
    return ARM_ERR_MAX_STEPS;
}

bool Arm_PositionKnown(void)
{
    return g_arm_pos_valid;
}

uint32_t Arm_GetCurrentPositionSteps(void)
{
    return g_arm_pos_steps;
}

bool Arm_CanStep(uint8_t dir)
{
    if (arm_dir_is_forward(dir)) {
        return !LimitSwitch_HomeTriggered();
    }

    if (!g_arm_pos_valid) {
        return false;
    }

    return g_arm_pos_steps < CalStore_GetExtendSteps();
}

bool Arm_CanJogStep(uint8_t dir)
{
    if (arm_dir_is_forward(dir)) {
        return !LimitSwitch_HomeTriggered();
    }

    return true;
}

void Arm_NotifyStep(uint8_t dir)
{
    if (arm_dir_is_forward(dir)) {
        if (g_arm_pos_steps > 0U) {
            g_arm_pos_steps--;
        }
        if (LimitSwitch_HomeTriggered()) {
            g_arm_pos_steps = 0U;
            g_arm_pos_valid = true;
        }
        return;
    }

    if (g_arm_pos_valid) {
        g_arm_pos_steps++;
    }
}

/* Private functions ---------------------------------------------------------*/

static bool B1_Pressed(void)
{
    if (b1_abort) {
        return true;
    }
    return (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET);
}

static void print_uart(UART_HandleTypeDef *huart, const char *msg)
{
    HAL_UART_Transmit(huart, (uint8_t *)msg, strlen(msg), 100);
}

static uint16_t servo_speed_for_transition(uint32_t steps, uint32_t speed_us,
                                           uint16_t servo_start, uint16_t servo_end)
{
    uint32_t servo_delta;
    uint32_t motion_time_ms;
    uint32_t speed_units_per_s;

    if (servo_end >= servo_start) {
        servo_delta = (uint32_t)(servo_end - servo_start);
    } else {
        servo_delta = (uint32_t)(servo_start - servo_end);
    }

    if ((steps == 0U) || (servo_delta == 0U)) {
        return 600U;
    }

    motion_time_ms = (steps * speed_us) / 1000U;
    if (motion_time_ms < 50U) {
        motion_time_ms = 50U;
    }

    speed_units_per_s = (servo_delta * 1000U) / motion_time_ms;
    if (speed_units_per_s < 120U) {
        speed_units_per_s = 120U;
    }
    if (speed_units_per_s > 2200U) {
        speed_units_per_s = 2200U;
    }

    return (uint16_t)speed_units_per_s;
}

static ArmResult_t Arm_MoveWithServo(uint32_t steps, uint8_t dir, uint32_t speed_us,
                                     bool stop_on_extend_limit, uint8_t servo_id,
                                     bool use_servo, uint16_t servo_start,
                                     uint16_t servo_end, uint32_t hold_steps)
{
    (void)stop_on_extend_limit;
    uint16_t servo_speed = 600U;
    uint32_t step_index = 0U;

    CL57T_SetDirection((CL57T_Direction_t)dir);
    HAL_Delay(1);

    if (use_servo) {
        servo_speed = servo_speed_for_transition(steps, speed_us, servo_start, servo_end);
        HX65_MovePositionSpeed(servo_id, servo_start, servo_speed);
    }

    if (hold_steps > steps) {
        hold_steps = steps;
    }

    while (step_index < steps) {
        uint32_t chunk_steps = use_servo ? ARM_SERVO_UPDATE_INTERVAL : 1U;
        uint32_t chunk_end = step_index + chunk_steps;

        if (chunk_end > steps) {
            chunk_end = steps;
        }

        if (use_servo && (steps > 0U)) {
            uint16_t pos = servo_start;
            uint32_t progress_index = chunk_end + ARM_SERVO_LEAD_STEPS;

            if (progress_index > steps) {
                progress_index = steps;
            }

            if ((hold_steps == 0U) || (progress_index > hold_steps)) {
                uint32_t interp_done = (hold_steps == 0U) ? progress_index : (progress_index - hold_steps);
                uint32_t interp_span = (hold_steps >= steps) ? 0U : (steps - hold_steps);

                if (interp_span > 0U) {
                    int32_t range = (int32_t)servo_end - (int32_t)servo_start;
                    pos = (uint16_t)((int32_t)servo_start +
                                     (range * (int32_t)interp_done) / (int32_t)interp_span);
                } else {
                    pos = servo_end;
                }
            }

            /* Re-target the servo only every coarse chunk so the stepper can
             * keep pulsing near full speed instead of being throttled by
             * UART-side wait time. */
            HX65_MovePositionSpeed(servo_id, pos, servo_speed);
        }

        while (step_index < chunk_end) {
            if (!Arm_CanStep(dir)) {
                return ARM_ERR_LIMIT_SAFETY;
            }

            CL57T_Step();
            Arm_NotifyStep(dir);
            step_index++;

            for (volatile uint32_t d = 0; d < speed_us * 10U; d++) { }
        }
    }

    if (use_servo) {
        HX65_MovePositionSpeed(servo_id, servo_end, servo_speed);
    }

    return ARM_OK;
}

static bool arm_dir_is_forward(uint8_t dir)
{
    return ((CL57T_Direction_t)dir == (CL57T_Direction_t)SCOOP_EXTEND_DIR);
}
