/**
 * @file    scoop_sequence.c
 * @brief   Coordinated litter scoop sequence using saved full-pose waypoints
 */

#include "scoop_sequence.h"
#include "calibration_store.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

static ScoopStatus_t g_last_status = {
    .result = SCOOP_OK,
    .phase = SCOOP_PHASE_IDLE,
    .arm_result = ARM_OK,
    .servo_result = HX65_OK,
};

static void reply(UART_HandleTypeDef *huart, const char *msg)
{
    if (huart != NULL) {
        HAL_UART_Transmit(huart, (uint8_t *)msg, strlen(msg), 100);
    }
}

static void replyf(UART_HandleTypeDef *huart, const char *fmt, ...)
{
    if (huart != NULL) {
        char buf[128];
        va_list args;
        va_start(args, fmt);
        vsnprintf(buf, sizeof(buf), fmt, args);
        va_end(args);
        reply(huart, buf);
    }
}

static bool servo_pos_valid(uint16_t pos)
{
    return (pos <= 4095U);
}

static uint8_t saved_pose_count(uint32_t mask)
{
    uint8_t count = 0U;
    uint32_t i;
    for (i = 0; i < CAL_POSE_COUNT; i++) {
        if (mask & (1UL << i)) {
            count++;
        }
    }
    return count;
}

static bool config_valid(const ScoopStatus_t *status)
{
    uint32_t i;

    if (status->valid_mask == 0U) {
        return false;
    }

    for (i = 0; i < CAL_POSE_COUNT; i++) {
        if ((status->valid_mask & (1UL << i)) == 0U) {
            continue;
        }
        if (!servo_pos_valid(CalStore_GetServoPos((CalServoPos_t)i))) {
            return false;
        }
        if (CalStore_GetPoseSteps((CalServoPos_t)i) > status->extend_steps) {
            return false;
        }
    }

    return true;
}

static bool servo_move_wait(UART_HandleTypeDef *huart, uint16_t position,
                            uint16_t move_ms, ScoopResult_t fail_result)
{
    HX65_Status_t rc = HX65_MovePosition(SCOOP_SERVO_ID, position, move_ms);
    g_last_status.servo_result = rc;
    if (rc != HX65_OK) {
        g_last_status.result = fail_result;
        replyf(huart, "SCOOP ERR: servo move failed (%d)\r\n", (int)rc);
        return false;
    }

    HAL_Delay((uint32_t)move_ms + 50U);
    return true;
}

static bool move_pose_transition(UART_HandleTypeDef *huart,
                                 uint32_t from_steps, uint32_t to_steps,
                                 uint16_t from_servo, uint16_t to_servo,
                                 uint32_t speed_us, ScoopResult_t fail_result)
{
    uint8_t retract_dir;

    if ((from_steps == to_steps) && (from_servo == to_servo)) {
        return true;
    }

    if (from_steps == to_steps) {
        return servo_move_wait(huart, to_servo, SCOOP_SERVO_MOVE_MS, fail_result);
    }

    retract_dir = (SCOOP_EXTEND_DIR == CL57T_DIR_CW) ? CL57T_DIR_CCW : CL57T_DIR_CW;
    if (to_steps > from_steps) {
        g_last_status.arm_result = Arm_RetractWithServo(to_steps - from_steps,
                                                        retract_dir, speed_us,
                                                        SCOOP_SERVO_ID,
                                                        from_servo, to_servo);
    } else {
        g_last_status.arm_result = Arm_ExtendWithServo(from_steps - to_steps,
                                                       SCOOP_EXTEND_DIR, speed_us,
                                                       SCOOP_SERVO_ID,
                                                       from_servo, to_servo);
    }

    if (g_last_status.arm_result != ARM_OK) {
        g_last_status.result = fail_result;
        replyf(huart, "SCOOP ERR: coordinated move failed (%d)\r\n",
               (int)g_last_status.arm_result);
        return false;
    }

    return true;
}

static void set_phase(UART_HandleTypeDef *huart, ScoopPhase_t phase, const char *detail)
{
    g_last_status.phase = phase;
    if (detail != NULL) {
        replyf(huart, "SCOOP %s: %s\r\n", Scoop_PhaseString(phase), detail);
    } else {
        replyf(huart, "SCOOP %s\r\n", Scoop_PhaseString(phase));
    }
}

static ArmResult_t move_arm_to(uint32_t target_steps, uint32_t speed_us)
{
    uint32_t current_steps;
    uint8_t retract_dir;

    if (!Arm_PositionKnown()) {
        return ARM_ERR_LIMIT_SAFETY;
    }

    current_steps = Arm_GetCurrentPositionSteps();
    if (current_steps == target_steps) {
        return ARM_OK;
    }

    retract_dir = (SCOOP_EXTEND_DIR == CL57T_DIR_CW) ? CL57T_DIR_CCW : CL57T_DIR_CW;
    if (target_steps > current_steps) {
        return Arm_Extend(target_steps - current_steps, retract_dir, speed_us);
    }

    return Arm_Extend(current_steps - target_steps, SCOOP_EXTEND_DIR, speed_us);
}

const ScoopStatus_t* Scoop_GetLastStatus(void)
{
    return &g_last_status;
}

const char* Scoop_PhaseString(ScoopPhase_t phase)
{
    switch (phase) {
        case SCOOP_PHASE_IDLE:          return "IDLE";
        case SCOOP_PHASE_PREHOME:       return "PREHOME";
        case SCOOP_PHASE_MOVE_TO_START: return "MOVE_TO_START";
        case SCOOP_PHASE_RUN_PATH:      return "RUN_PATH";
        case SCOOP_PHASE_COMPLETE:      return "COMPLETE";
        default:                        return "UNKNOWN";
    }
}

const char* Scoop_ResultString(ScoopResult_t result)
{
    switch (result) {
        case SCOOP_OK:                    return "OK";
        case SCOOP_ERR_CONFIG:            return "CONFIG";
        case SCOOP_ERR_ARM_HOME:          return "ARM_HOME";
        case SCOOP_ERR_SERVO_PRECHECK:    return "SERVO_PRECHECK";
        case SCOOP_ERR_SERVO_HOME:        return "SERVO_HOME";
        case SCOOP_ERR_ARM_APPROACH:      return "ARM_APPROACH";
        case SCOOP_ERR_SERVO_CAPTURE:     return "SERVO_CAPTURE";
        case SCOOP_ERR_ARM_RETRACT:       return "ARM_RETRACT";
        case SCOOP_ERR_SERVO_DUMP:        return "SERVO_DUMP";
        case SCOOP_ERR_SERVO_RETURN_HOME: return "SERVO_RETURN_HOME";
        default:                          return "UNKNOWN";
    }
}

ScoopResult_t Scoop_Execute(UART_HandleTypeDef *huart)
{
    uint32_t valid_mask;
    uint32_t current_steps = 0U;
    uint16_t current_servo = CalStore_GetServoPos(CAL_POS_HOME);
    int32_t first_idx = -1;
    uint32_t i;

    memset(&g_last_status, 0, sizeof(g_last_status));
    g_last_status.result = SCOOP_OK;
    g_last_status.phase = SCOOP_PHASE_IDLE;
    g_last_status.arm_result = ARM_OK;
    g_last_status.servo_result = HX65_OK;
    g_last_status.extend_steps = CalStore_GetExtendSteps();
    g_last_status.valid_mask = CalStore_GetValidMask();

    valid_mask = g_last_status.valid_mask;
    g_last_status.executed_pose_count = saved_pose_count(valid_mask);
    g_last_status.last_pose_index = 0U;

    if (!config_valid(&g_last_status)) {
        g_last_status.result = SCOOP_ERR_CONFIG;
        reply(huart, "SCOOP ERR: invalid waypoint calibration\r\n");
        return g_last_status.result;
    }

    for (i = 0; i < CAL_POSE_COUNT; i++) {
        if (valid_mask & (1UL << i)) {
            first_idx = (int32_t)i;
            break;
        }
    }

    if (first_idx < 0) {
        g_last_status.result = SCOOP_ERR_CONFIG;
        reply(huart, "SCOOP ERR: no saved poses\r\n");
        return g_last_status.result;
    }

    replyf(huart, "SCOOP START: saved_mask=0x%03lX count=%u max=%lu\r\n",
           (unsigned long)valid_mask,
           (unsigned)g_last_status.executed_pose_count,
           (unsigned long)g_last_status.extend_steps);

    set_phase(huart, SCOOP_PHASE_PREHOME, "front-zero home");
    g_last_status.arm_result = Arm_Home(huart);
    if (g_last_status.arm_result != ARM_OK) {
        g_last_status.result = SCOOP_ERR_ARM_HOME;
        replyf(huart, "SCOOP ERR: arm home failed (%d)\r\n",
               (int)g_last_status.arm_result);
        return g_last_status.result;
    }

    set_phase(huart, SCOOP_PHASE_MOVE_TO_START, "move to first saved pose");
    current_steps = 0U;
    current_servo = CalStore_GetServoPos(CAL_POS_HOME);

    g_last_status.arm_result = move_arm_to(CalStore_GetPoseSteps((CalServoPos_t)first_idx),
                                           SCOOP_RETRACT_SPEED_US);
    if (g_last_status.arm_result != ARM_OK) {
        g_last_status.result = SCOOP_ERR_ARM_APPROACH;
        replyf(huart, "SCOOP ERR: move to P%ld failed (%d)\r\n",
               (long)(first_idx + 1), (int)g_last_status.arm_result);
        return g_last_status.result;
    }

    current_steps = CalStore_GetPoseSteps((CalServoPos_t)first_idx);
    if (!servo_move_wait(huart,
                         CalStore_GetServoPos((CalServoPos_t)first_idx),
                         SCOOP_SERVO_MOVE_MS,
                         SCOOP_ERR_SERVO_HOME)) {
        return g_last_status.result;
    }
    current_servo = CalStore_GetServoPos((CalServoPos_t)first_idx);
    g_last_status.last_pose_index = (uint8_t)first_idx;

    set_phase(huart, SCOOP_PHASE_RUN_PATH, "play saved waypoints");
    for (i = (uint32_t)first_idx + 1U; i < CAL_POSE_COUNT; i++) {
        uint32_t target_steps;
        uint16_t target_servo;

        if ((valid_mask & (1UL << i)) == 0U) {
            continue;
        }

        target_steps = CalStore_GetPoseSteps((CalServoPos_t)i);
        target_servo = CalStore_GetServoPos((CalServoPos_t)i);
        replyf(huart, "SCOOP PATH: P%lu=(%lu,%u)\r\n",
               (unsigned long)(i + 1U),
               (unsigned long)target_steps,
               (unsigned)target_servo);

        if (!move_pose_transition(huart,
                                  current_steps, target_steps,
                                  current_servo, target_servo,
                                  SCOOP_COORD_SPEED_US, SCOOP_ERR_ARM_RETRACT)) {
            return g_last_status.result;
        }

        current_steps = target_steps;
        current_servo = target_servo;
        g_last_status.last_pose_index = (uint8_t)i;
        HAL_Delay(SCOOP_PICKUP_SETTLE_MS);
    }

    g_last_status.phase = SCOOP_PHASE_COMPLETE;
    g_last_status.result = SCOOP_OK;
    reply(huart, "SCOOP COMPLETE\r\n");
    return g_last_status.result;
}
