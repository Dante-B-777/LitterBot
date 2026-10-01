/**
 * @file    uart_cmd.c
 * @brief   UART command parser implementation
 *
 * Adapted from LB_FIRM/Core/Src/wheels4.c. Receives ASCII commands
 * over LPUART1 and dispatches to the appropriate subsystem.
 *
 * Supported commands:
 *   M <fl> <fr> <rl> <rr>\n  — Motor speeds (-1000 to +1000)    [GATED]
 *   X\n                       — Emergency stop                    [ALWAYS]
 *   S SCOOP\n                 — Execute scoop sequence            [GATED]
 *   C CALIBRATE\n             — Arm calibration                   [GATED]
 *   D DEPLOY\n                — Trigger deploy (arm homing)       [STANDBY/ACTIVE]
 *   DEV ON\n / DEV OFF\n      — Toggle dev mode                   [ALWAYS]
 *   Q\n                       — Query current state               [ALWAYS]
 *   T\n                       — Run encoder test                  [ALWAYS]
 *
 * Commands marked [GATED] are blocked in STANDBY unless dev mode is on.
 * Commands marked [ALWAYS] work in any state.
 *
 * @author  LitterBot Team
 * @date    2026
 */

/* Includes ------------------------------------------------------------------*/
#include "uart_cmd.h"
#include "cytron_md10c.h"
#include "encoder_test.h"
#include "scoop_sequence.h"
#include "arm_control.h"
#include "deploy_state.h"
#include "calibration_store.h"
#include "hx65_servo.h"
#include "encoder.h"
#include "cl57t_stepper.h"
#include "limit_switch.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

/* Private defines -----------------------------------------------------------*/

/** @brief Line buffer size for incoming commands */
#define LINE_BUF_SZ         80

/** @brief Motor command timeout in milliseconds.
 *
 * Raised from 250 → 500 ms to absorb browser/WebSocket jitter without
 * zeroing motors between commands. UI sends /cmd_vel at 10 Hz (100 ms)
 * so 500 ms gives 5x margin before the watchdog trips. Previously the
 * tight 250 ms window caused visible "kakakak" stuttering whenever the
 * browser tab lost focus (Chrome throttles setInterval to ~1 Hz for
 * background tabs, which immediately exceeds 250 ms). */
#define DRIVE_TIMEOUT_MS    500
#define TELEMETRY_PERIOD_MS 100

/** @brief Max command value from UART protocol */
#define CMD_MAX             1000

/** @brief Servo jog glide speed, in raw HX-65HM units (≈ position steps/s).
 *
 * Used by SERVO JOG START/STOP. One MovePositionSpeed is sent at press,
 * targeting the endpoint (0 or 4095); servo glides smoothly on its own
 * until a STOP freezes it. Lower = smoother/slower. 600 gives ~6.8 s
 * full 0→4095 sweep which feels like a controlled jog. */
#define SCOOP_SERVO_JOG_SPEED  600U

/* Private variables ---------------------------------------------------------*/

/** @brief UART handle for debug/command interface */
static UART_HandleTypeDef *g_huart = NULL;

/** @brief Line buffer for incoming characters */
static volatile char g_line[LINE_BUF_SZ];

/** @brief Current position in line buffer */
static volatile int g_len = 0;

/** @brief Flag: 1 when a complete line is ready for processing */
static volatile int g_ready = 0;

/** @brief Timestamp of last valid motor command (for timeout) */
static uint32_t g_last_cmd_ms = 0;
static uint32_t g_last_telemetry_ms = 0;

/** @brief True when servo cal mode is active (prints position every 500ms) */
/* ---- Servo jog state (used by SERVO JOG START / STOP) -------------------- */
static uint16_t g_servo_jog_pos        = 0;      /* last-known software position */
static uint16_t g_servo_jog_start_pos  = 0;      /* position when jog started */
static uint32_t g_servo_jog_start_tick = 0;      /* HAL_GetTick() at jog start */
static int8_t   g_servo_jog_sign       = 0;      /* +1 or -1 */
static bool     g_servo_jog_active     = false;  /* true while gliding */

static bool servo_cal_active = false;

/** @brief Timestamp of last position print during servo cal mode */
static uint32_t servo_cal_last_ms = 0;

/* Private function prototypes -----------------------------------------------*/
static int clamp_i(int v, int lo, int hi);
static void handle_line(const char *s, uint32_t now_ms);
static void reply(const char *msg);
static const char* servo_preset_name(CalServoPos_t which);
static bool parse_servo_preset(const char *name, CalServoPos_t *which_out);
static void handle_test_command(const char *s);
static bool single_limit_blocks_forward(CL57T_Direction_t dir);
static uint16_t current_servo_position(void);
static void reply_pose_status(const char *prefix);

static bool single_limit_blocks_forward(CL57T_Direction_t dir)
{
    return (dir == SCOOP_EXTEND_DIR) && LimitSwitch_HomeTriggered();
}

static uint16_t current_servo_position(void)
{
    return g_servo_jog_pos;
}

static void reply_pose_status(const char *prefix)
{
    char buf[160];
    uint32_t i;
    uint32_t valid_mask = CalStore_GetValidMask();

    snprintf(buf, sizeof(buf),
             "%s STATUS ref=%d pos=%lu max=%lu front_sw=%d servo=%u mask=0x%03lX\r\n",
             prefix,
             Arm_PositionKnown() ? 1 : 0,
             (unsigned long)Arm_GetCurrentPositionSteps(),
             (unsigned long)CalStore_GetExtendSteps(),
             LimitSwitch_HomeTriggered() ? 1 : 0,
             current_servo_position(),
             (unsigned long)valid_mask);
    reply(buf);

    for (i = 0; i < CAL_POSE_COUNT; i++) {
        snprintf(buf, sizeof(buf),
                 "%s P%lu saved=%d steps=%lu servo=%u\r\n",
                 prefix,
                 (unsigned long)(i + 1U),
                 CalStore_PoseIsSaved((CalServoPos_t)i) ? 1 : 0,
                 (unsigned long)CalStore_GetPoseSteps((CalServoPos_t)i),
                 CalStore_GetServoPos((CalServoPos_t)i));
        reply(buf);
    }
}

/* Public functions ----------------------------------------------------------*/

void UartCmd_Init(UART_HandleTypeDef *huart)
{
    g_huart = huart;

    memset((void *)g_line, 0, sizeof(g_line));
    g_len   = 0;
    g_ready = 0;

    /* Initialize subsystems */
    MD10C_Init();
    Arm_Init();
    DeployState_Init();
    g_servo_jog_pos = CalStore_GetServoPos(CAL_POS_HOME);

    g_last_cmd_ms = HAL_GetTick();
    g_last_telemetry_ms = HAL_GetTick();
}

void UartCmd_OnRxByte(uint8_t byte)
{
    char c = (char)byte;

    /* Ignore bytes if previous line hasn't been processed */
    if (g_ready) {
        return;
    }

    /* Ignore carriage return */
    if (c == '\r') {
        return;
    }

    /* Newline or buffer full = line complete */
    if (c == '\n' || g_len >= (LINE_BUF_SZ - 1)) {
        g_line[g_len] = '\0';
        g_ready = 1;
        return;
    }

    g_line[g_len++] = c;
}

void UartCmd_Task(uint32_t now_ms)
{
    /* Check command timeout */
    if ((now_ms - g_last_cmd_ms) > DRIVE_TIMEOUT_MS) {
        MD10C_StopAll();
    }

    /* Process pending line if available */
    if (g_ready) {
        char local[LINE_BUF_SZ];

        /* Copy line buffer with interrupts disabled to avoid race */
        __disable_irq();
        int n = g_len;
        if (n >= LINE_BUF_SZ) n = LINE_BUF_SZ - 1;
        memcpy(local, (const void *)g_line, (size_t)n);
        local[n] = '\0';
        g_len   = 0;
        g_ready = 0;
        __enable_irq();

        handle_line(local, now_ms);
    }

    /* Servo calibration mode — print position every 500ms */
    if (servo_cal_active && (now_ms - servo_cal_last_ms >= 500)) {
        uint16_t pos;
        if (HX65_ReadPosition(SCOOP_SERVO_ID, &pos) == HX65_OK) {
            char buf[24];
            snprintf(buf, sizeof(buf), "POS: %u\r\n", pos);
            reply(buf);
        }
        servo_cal_last_ms = now_ms;
    }
}

/* Private functions ---------------------------------------------------------*/

static int clamp_i(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/**
 * @brief  Send a response string over UART
 */
static void reply(const char *msg)
{
    if (g_huart) {
        HAL_UART_Transmit(g_huart, (uint8_t *)msg, strlen(msg), 100);
    }
}

void UartCmd_TelemetryTask(uint32_t now_ms)
{
    if ((now_ms - g_last_telemetry_ms) < TELEMETRY_PERIOD_MS) {
        return;
    }

    g_last_telemetry_ms = now_ms;

    char buf[96];
    snprintf(buf, sizeof(buf), "ENC %ld %ld %ld %ld\r\n",
             (long)Encoder_GetTicks(ENC_FL),
             (long)Encoder_GetTicks(ENC_FR),
             (long)Encoder_GetTicks(ENC_RL),
             (long)Encoder_GetTicks(ENC_RR));
    reply(buf);
}

static const char* servo_preset_name(CalServoPos_t which)
{
    switch (which) {
        case CAL_POS_1:  return "P1";
        case CAL_POS_2:  return "P2";
        case CAL_POS_3:  return "P3";
        case CAL_POS_4:  return "P4";
        case CAL_POS_5:  return "P5";
        case CAL_POS_6:  return "P6";
        case CAL_POS_7:  return "P7";
        case CAL_POS_8:  return "P8";
        case CAL_POS_9:  return "P9";
        case CAL_POS_10: return "P10";
        default:             return "UNKNOWN";
    }
}

static bool parse_servo_preset(const char *name, CalServoPos_t *which_out)
{
    if (name == NULL || which_out == NULL) {
        return false;
    }

    if ((name[0] == 'P') || (name[0] == 'p')) {
        unsigned long idx = 0UL;
        if ((sscanf(name + 1, "%lu", &idx) == 1) &&
            (idx >= 1UL) &&
            (idx <= (unsigned long)CAL_POSE_COUNT)) {
            *which_out = (CalServoPos_t)(idx - 1UL);
            return true;
        }
    }

    if ((strncmp(name, "HOME", 4) == 0) || (strncmp(name, "STOW", 4) == 0)) {
        *which_out = CAL_POS_1;
        return true;
    }
    if ((strncmp(name, "OPEN", 4) == 0) || (strncmp(name, "APPROACH", 8) == 0)) {
        *which_out = CAL_POS_2;
        return true;
    }
    if ((strncmp(name, "CLOSED", 6) == 0) || (strncmp(name, "CAPTURE", 7) == 0)) {
        *which_out = CAL_POS_3;
        return true;
    }
    if (strncmp(name, "DUMP", 4) == 0) {
        *which_out = CAL_POS_4;
        return true;
    }

    return false;
}

static void handle_test_command(const char *s)
{
    if (strncmp(s, "TEST STEP ", 10) == 0) {
        char dir_name[8] = {0};
        unsigned long steps = 0;
        unsigned long delay_us = 1000;
        CL57T_Direction_t dir;
        char buf[96];

        if (sscanf(s + 10, "%7s %lu %lu", dir_name, &steps, &delay_us) < 2 || steps == 0UL) {
            reply("TEST ERR: expected TEST STEP <CW|CCW> <steps> [delay_us]\r\n");
            return;
        }

        if (strncmp(dir_name, "CW", 2) == 0) {
            dir = CL57T_DIR_CW;
        } else if (strncmp(dir_name, "CCW", 3) == 0) {
            dir = CL57T_DIR_CCW;
        } else {
            reply("TEST ERR: expected direction CW or CCW\r\n");
            return;
        }

        if (single_limit_blocks_forward(dir)) {
            reply("TEST STEP BLOCKED: PD15 limit active\r\n");
            return;
        }

        if (delay_us < 5UL) {
            delay_us = 5UL;
        }

        /* For small "jog-sized" bursts (<= 500 steps) we suppress the
         * verbose diagnostic replies. A continuous jog sends a burst every
         * ~45 ms, and blocking UART prints at 115200 baud can eat 8-10 ms
         * of that budget — enough to make the stepper stutter audibly. The
         * diagnostic start/end lines are only useful for one-shot manual
         * testing. Large bursts still print so the scoop/arm test output
         * is unchanged. */
        const bool verbose = (steps > 500UL);

        if (verbose) {
            CL57T_SetDirection(dir);
            HAL_Delay(1);  /* Let pin settle before readback */
            const CL57T_Stepper_t *st = CL57T_GetStepper();
            int dir_level = HAL_GPIO_ReadPin(st->dir_port, st->dir_pin) == GPIO_PIN_SET ? 1 : 0;
            int expected  = (dir == CL57T_DIR_CCW) ? 1 : 0;

            snprintf(buf, sizeof(buf),
                     "TEST STEP START dir=%s steps=%lu delay=%lu DIR_pin=%d exp=%d%s\r\n",
                     dir == CL57T_DIR_CW ? "CW" : "CCW",
                     steps,
                     delay_us,
                     dir_level,
                     expected,
                     dir_level == expected ? "" : " MISMATCH!");
            reply(buf);
        }

        CL57T_SetDirection(dir);
        HAL_Delay(1);

        for (uint32_t i = 0; i < (uint32_t)steps; i++) {
            if (!Arm_CanJogStep((uint8_t)dir)) {
                if (verbose) {
                    snprintf(buf, sizeof(buf),
                             "TEST STEP HIT limit dir=%s step=%lu\r\n",
                             dir == CL57T_DIR_CW ? "CW" : "CCW",
                             (unsigned long)i);
                    reply(buf);
                }
                break;
            }
            CL57T_Step();
            Arm_NotifyStep((uint8_t)dir);
            for (volatile uint32_t d = 0; d < ((uint32_t)delay_us * 10U); d++) { }
        }

        if (verbose) {
            snprintf(buf, sizeof(buf), "TEST STEP OK dir=%s steps=%lu delay=%lu\r\n",
                     dir == CL57T_DIR_CW ? "CW" : "CCW",
                     steps,
                     delay_us);
            reply(buf);
        }
        return;
    }

    /* TEST DIR <CW|CCW> — set the DIR pin without pulsing and report its
     * readback state. Use this to probe PF15 with a multimeter while
     * debugging reverse-direction issues. */
    if (strncmp(s, "TEST DIR ", 9) == 0) {
        const char *arg = s + 9;
        CL57T_Direction_t dir;
        char buf[80];

        if (strncmp(arg, "CCW", 3) == 0) {
            dir = CL57T_DIR_CCW;
        } else if (strncmp(arg, "CW", 2) == 0) {
            dir = CL57T_DIR_CW;
        } else {
            reply("TEST ERR: expected TEST DIR <CW|CCW>\r\n");
            return;
        }

        CL57T_SetDirection(dir);
        HAL_Delay(1);
        const CL57T_Stepper_t *st = CL57T_GetStepper();
        int dir_level = HAL_GPIO_ReadPin(st->dir_port, st->dir_pin) == GPIO_PIN_SET ? 1 : 0;
        int expected  = (dir == CL57T_DIR_CCW) ? 1 : 0;

        snprintf(buf, sizeof(buf),
                 "TEST DIR OK dir=%s DIR_pin=%d exp=%d%s\r\n",
                 dir == CL57T_DIR_CW ? "CW" : "CCW",
                 dir_level,
                 expected,
                 dir_level == expected ? "" : " MISMATCH!");
        reply(buf);
        return;
    }

    if (strncmp(s, "TEST SERVO ", 11) == 0) {
        CalServoPos_t which;
        uint16_t pos;
        HX65_Status_t rc;
        char buf[64];

        if (!parse_servo_preset(s + 11, &which)) {
            reply("TEST ERR: unknown servo preset\r\n");
            return;
        }

        pos = CalStore_GetServoPos(which);
        rc = HX65_MovePosition(SCOOP_SERVO_ID, pos, SCOOP_SERVO_MOVE_MS);
        if (rc != HX65_OK) {
            snprintf(buf, sizeof(buf), "TEST ERR: servo move failed (%d)\r\n", (int)rc);
            reply(buf);
            return;
        }

        snprintf(buf, sizeof(buf), "TEST OK: SERVO %s=%u\r\n", servo_preset_name(which), pos);
        reply(buf);
        return;
    }

    if (strncmp(s, "TEST ARM EXTEND ", 16) == 0) {
        unsigned long steps = 0;
        char buf[64];
        ArmResult_t rc;

        if (sscanf(s + 16, "%lu", &steps) != 1 || steps == 0UL) {
            reply("TEST ERR: expected step count\r\n");
            return;
        }

        rc = Arm_Extend((uint32_t)steps, SCOOP_EXTEND_DIR, SCOOP_EXTEND_SPEED_US);
        snprintf(buf, sizeof(buf), "TEST ARM EXTEND rc=%d steps=%lu\r\n", (int)rc, steps);
        reply(buf);
        return;
    }

    if (strncmp(s, "TEST ARM RETRACT ", 17) == 0) {
        unsigned long steps = 0;
        char buf[64];
        ArmResult_t rc;
        uint8_t retract_dir = (SCOOP_EXTEND_DIR == CL57T_DIR_CW) ? CL57T_DIR_CCW : CL57T_DIR_CW;

        if (sscanf(s + 17, "%lu", &steps) != 1 || steps == 0UL) {
            reply("TEST ERR: expected step count\r\n");
            return;
        }

        rc = Arm_Extend((uint32_t)steps, retract_dir, SCOOP_RETRACT_SPEED_US);
        snprintf(buf, sizeof(buf), "TEST ARM RETRACT rc=%d steps=%lu\r\n", (int)rc, steps);
        reply(buf);
        return;
    }

    if (strncmp(s, "TEST ARM HOME", 13) == 0) {
        ArmResult_t rc = Arm_Home(g_huart);
        char buf[48];
        snprintf(buf, sizeof(buf), "TEST ARM HOME rc=%d\r\n", (int)rc);
        reply(buf);
        return;
    }

    if (strncmp(s, "TEST ARM STATUS", 15) == 0) {
        reply_pose_status("TEST ARM");
        return;
    }

    if (strncmp(s, "TEST ARM SAVE", 13) == 0) {
        char buf[64];
        if (!Arm_PositionKnown()) {
            reply("TEST ARM SAVE ERR: home first\r\n");
            return;
        }
        CalStore_SaveExtendSteps(Arm_GetCurrentPositionSteps());
        snprintf(buf, sizeof(buf), "TEST ARM SAVE OK retract_steps=%lu\r\n",
                 (unsigned long)Arm_GetCurrentPositionSteps());
        reply(buf);
        return;
    }

    if (strncmp(s, "TEST SAVE ", 10) == 0) {
        uint16_t pos;
        uint32_t steps;
        char buf[80];
        CalServoPos_t which;

        if (!parse_servo_preset(s + 10, &which)) {
            reply("TEST SAVE ERR: unknown position\r\n");
            return;
        }

        pos = current_servo_position();
        steps = Arm_GetCurrentPositionSteps();
        CalStore_SavePose(which, steps, pos);
        snprintf(buf, sizeof(buf), "TEST SAVE OK P%u=(%lu,%u)\r\n",
                 (unsigned)(which + 1U),
                 (unsigned long)steps,
                 pos);
        reply(buf);
        return;
    }

    if (strncmp(s, "TEST SCOOP", 10) == 0) {
        if (strncmp(s, "TEST SCOOP STATUS", 17) == 0) {
            const ScoopStatus_t *status = Scoop_GetLastStatus();
            char buf[160];

            snprintf(buf, sizeof(buf),
                     "TEST SCOOP STATUS rc=%s phase=%s arm=%d servo=%d max=%lu mask=0x%03lX count=%u last=P%u\r\n",
                     Scoop_ResultString(status->result),
                     Scoop_PhaseString(status->phase),
                     (int)status->arm_result,
                     (int)status->servo_result,
                     (unsigned long)status->extend_steps,
                     (unsigned long)status->valid_mask,
                     (unsigned)status->executed_pose_count,
                     (unsigned)(status->last_pose_index + 1U));
            reply(buf);
            return;
        }

        ScoopResult_t rc = Scoop_Execute(g_huart);
        char buf[64];
        snprintf(buf, sizeof(buf), "TEST SCOOP rc=%s\r\n", Scoop_ResultString(rc));
        reply(buf);
        return;
    }

    reply("TEST ERR: unknown test command\r\n");
}

/**
 * @brief  Parse and execute a command line
 */
static void handle_line(const char *s, uint32_t now_ms)
{
    if (!s || !s[0]) {
        return;
    }

    /* =================================================================
     * ALWAYS-ALLOWED COMMANDS (never gated by deploy state)
     * ================================================================= */

    /* X — Emergency stop (never gated, single char for easy trigger) */
    if (s[0] == 'X') {
        MD10C_StopAll();
        g_last_cmd_ms = now_ms;  /* Reset timeout on explicit stop */
        return;
    }

    /* Q — Query current state */
    if (s[0] == 'Q') {
        char buf[40];
        snprintf(buf, sizeof(buf), "STATE %s%s\r\n",
                 DeployState_GetStateString(),
                 DeployState_IsDevMode() ? " DEV" : "");
        reply(buf);
        return;
    }

    /* T — Encoder test (harmless debug output) */
    if ((s[0] == 'T') && (s[1] == '\0')) {
        EncoderTest_Run(g_huart);
        g_last_cmd_ms = HAL_GetTick();
        return;
    }

    /* DEV ON / DEV OFF — Toggle dev mode (always allowed) */
    if (strncmp(s, "DEV ", 4) == 0) {
        if (strncmp(s + 4, "ON", 2) == 0) {
            DeployState_SetDevMode(true);
            reply("DEV ON\r\n");
        } else if (strncmp(s + 4, "OFF", 3) == 0) {
            DeployState_SetDevMode(false);
            reply("DEV OFF\r\n");
        }
        return;
    }

    /* D DEPLOY — Trigger deploy sequence (allowed from STANDBY or ACTIVE) */
    if (strncmp(s, "D DEPLOY", 8) == 0) {
        /* If ACTIVE, return to STANDBY first for re-deploy */
        if (DeployState_Get() == DEPLOY_ACTIVE) {
            MD10C_StopAll();
            DeployState_ReturnToStandby();
        }
        DeployResult_t r = DeployState_StartDeploy(g_huart);
        if (r == DEPLOY_OK) {
            reply("DEPLOY OK\r\n");
        } else {
            char buf[40];
            snprintf(buf, sizeof(buf), "DEPLOY FAIL %d\r\n", (int)r);
            reply(buf);
        }
        g_last_cmd_ms = HAL_GetTick();
        return;
    }

    /* =================================================================
     * GATED COMMANDS (blocked in STANDBY unless dev mode)
     * ================================================================= */

    if (!DeployState_CommandAllowed(s[0])) {
        reply("ERR:STANDBY\r\n");
        return;
    }

    if (strncmp(s, "TEST ", 5) == 0) {
        MD10C_StopAll();
        handle_test_command(s);
        g_last_cmd_ms = HAL_GetTick();
        return;
    }

    /* STEPPER JOG START <CW|CCW> [delay_us] — continuous stepper jog.
     *
     * Blocks in CL57T_Jog polling g_ready. Any subsequent UART line
     * breaks the loop cleanly: STEPPER JOG STOP (the expected release),
     * X (e-stop), DEV OFF, even an M command. The next iteration of
     * UartCmd_Task then processes whichever line actually arrived.
     *
     * This replaces the old burst-based jog (TEST STEP called every
     * 45 ms from the ROS bridge), which had an audible ~22 Hz hammer
     * from the tiny silent gaps between bursts. Continuous stepping
     * has zero gaps. */
    if (strncmp(s, "STEPPER JOG START ", 18) == 0) {
        MD10C_StopAll();
        char dir_name[8] = {0};
        unsigned long delay_us = 85UL;
        sscanf(s + 18, "%7s %lu", dir_name, &delay_us);

        CL57T_Direction_t dir;
        /* Check CCW before CW — otherwise strncmp with n=2 matches "CCW" too */
        if (strncmp(dir_name, "CCW", 3) == 0) {
            dir = CL57T_DIR_CCW;
        } else if (strncmp(dir_name, "CW", 2) == 0) {
            dir = CL57T_DIR_CW;
        } else {
            reply("JOG ERR: direction must be CW or CCW\r\n");
            return;
        }

        if (single_limit_blocks_forward(dir)) {
            reply("JOG BLOCKED: PD15 limit active\r\n");
            g_last_cmd_ms = HAL_GetTick();
            return;
        }

        CL57T_SetDirection(dir);
        HAL_Delay(1);
        while (g_ready == 0) {
            if (!Arm_CanJogStep((uint8_t)dir)) {
                break;
            }
            CL57T_Step();
            Arm_NotifyStep((uint8_t)dir);
            for (volatile uint32_t d = 0; d < ((uint32_t)delay_us * 10U); d++) { }
        }
        /* Loop exited because a new command arrived. UartCmd_Task will
         * process it next iteration — don't handle it here. */
        g_last_cmd_ms = HAL_GetTick();
        return;
    }

    /* STEPPER JOG STOP — explicit stop. By the time we get here the
     * blocking CL57T_Jog loop has already exited (this command arriving
     * is what set g_ready). Just acknowledge and return. */
    if (strncmp(s, "STEPPER JOG STOP", 16) == 0) {
        g_last_cmd_ms = HAL_GetTick();
        return;
    }

    /* SERVO JOG START <+|-> / SERVO JOG STOP — continuous bucket servo jog.
     *
     * On START, we send ONE MovePositionSpeed targeting the corresponding
     * endpoint (0 for -, 4095 for +) at a tunable slow speed. The HX-65HM
     * begins a smooth internal glide toward that target and keeps going
     * on its own — no repeated UART traffic needed.
     *
     * On STOP, we estimate the current position from elapsed time and
     * the jog speed (feedforward, since PC10 is TX-only half-duplex so
     * HX65_ReadPosition isn't reliable on this board) and send one final
     * MovePositionSpeed at that position to freeze the servo in place.
     *
     * SCOOP_SERVO_JOG_SPEED picks how fast the glide is. 600 steps/s
     * gives ~6.8 s for a full 0→4095 sweep — comfortable controlled jog,
     * not a slam. Lower = slower/smoother. */
    if (strncmp(s, "SERVO JOG START ", 16) == 0) {
        static bool s_servo_pos_init = false;
        const char *arg = s + 16;
        HX65_Status_t rc;
        char buf[64];
        int sign;
        uint16_t target;

        if (arg[0] == '+') {
            sign = +1;
            target = 4095U;
        } else if (arg[0] == '-') {
            sign = -1;
            target = 0U;
        } else {
            reply("SERVO JOG ERR: expected + or -\r\n");
            return;
        }

        if (!s_servo_pos_init) {
            g_servo_jog_pos = CalStore_GetServoPos(CAL_POS_HOME);
            s_servo_pos_init = true;
        }

        g_servo_jog_start_pos  = g_servo_jog_pos;
        g_servo_jog_start_tick = HAL_GetTick();
        g_servo_jog_sign       = sign;
        g_servo_jog_active     = true;

        rc = HX65_MovePositionSpeed(SCOOP_SERVO_ID, target, SCOOP_SERVO_JOG_SPEED);
        if (rc != HX65_OK) {
            g_servo_jog_active = false;
            snprintf(buf, sizeof(buf), "SERVO JOG ERR: start rc=%d\r\n", (int)rc);
            reply(buf);
            return;
        }

        g_last_cmd_ms = HAL_GetTick();
        return;
    }

    if (strncmp(s, "SERVO JOG STOP", 14) == 0) {
        HX65_Status_t rc;
        char buf[64];
        uint32_t elapsed_ms;
        int32_t travelled;
        int32_t new_pos;

        if (!g_servo_jog_active) {
            g_last_cmd_ms = HAL_GetTick();
            return;
        }

        elapsed_ms = HAL_GetTick() - g_servo_jog_start_tick;
        /* travelled_units = speed[units/s] * elapsed[ms] / 1000 */
        travelled = (int32_t)((SCOOP_SERVO_JOG_SPEED * elapsed_ms) / 1000U);
        new_pos = (int32_t)g_servo_jog_start_pos + g_servo_jog_sign * travelled;
        if (new_pos < 0)    new_pos = 0;
        if (new_pos > 4095) new_pos = 4095;

        rc = HX65_MovePositionSpeed(SCOOP_SERVO_ID, (uint16_t)new_pos, SCOOP_SERVO_JOG_SPEED);
        if (rc != HX65_OK) {
            snprintf(buf, sizeof(buf), "SERVO JOG ERR: stop rc=%d\r\n", (int)rc);
            reply(buf);
            g_servo_jog_active = false;
            return;
        }

        g_servo_jog_pos    = (uint16_t)new_pos;
        g_servo_jog_active = false;
        g_last_cmd_ms      = HAL_GetTick();
        return;
    }

    /* M — Motor command (format unchanged: "M <fl> <fr> <rl> <rr>\n") */
    if (s[0] == 'M') {
        int a, b, c, d;
        if (sscanf(s, "M %d %d %d %d", &a, &b, &c, &d) == 4) {
            a = clamp_i(a, -CMD_MAX, CMD_MAX);
            b = clamp_i(b, -CMD_MAX, CMD_MAX);
            c = clamp_i(c, -CMD_MAX, CMD_MAX);
            d = clamp_i(d, -CMD_MAX, CMD_MAX);

            /* Map -1000..+1000 to -100.0..+100.0%
             * Right side motors (FR, RR) are mounted facing opposite direction,
             * so their speed is negated to produce the correct wheel rotation. */
            MD10C_SetSpeed(MD10C_MOTOR_FL, (float)a / 10.0f);
            MD10C_SetSpeed(MD10C_MOTOR_FR, -(float)b / 10.0f);
            MD10C_SetSpeed(MD10C_MOTOR_RL, (float)c / 10.0f);
            MD10C_SetSpeed(MD10C_MOTOR_RR, -(float)d / 10.0f);

            g_last_cmd_ms = now_ms;
        }
        return;
    }

    /* S SCOOP — Scoop sequence (requires full keyword to prevent noise trigger) */
    if (s[0] == 'S') {
        if (strncmp(s, "S SCOOP", 7) != 0) {
            return;  /* Reject bare "S\n" — require "S SCOOP\n" */
        }
        MD10C_StopAll();
        ScoopResult_t scoop_result = Scoop_Execute(g_huart);
        if (scoop_result != SCOOP_OK) {
            const ScoopStatus_t *status = Scoop_GetLastStatus();
            char buf[96];
            snprintf(buf, sizeof(buf),
                     "SCOOP FAIL %s phase=%s arm=%d servo=%d\r\n",
                     Scoop_ResultString(scoop_result),
                     Scoop_PhaseString(status->phase),
                     (int)status->arm_result,
                     (int)status->servo_result);
            reply(buf);
        }
        g_last_cmd_ms = HAL_GetTick();
        return;
    }

    /* C CALIBRATE — Arm calibration (requires exact prefix, must not swallow CAL ...) */
    if ((s[0] == 'C') && (s[1] == ' ')) {
        if (strncmp(s, "C CALIBRATE", 11) != 0) {
            return;  /* Reject bare "C\n" — require "C CALIBRATE\n" */
        }
        MD10C_StopAll();
        Arm_Calibrate(g_huart);
        g_last_cmd_ms = HAL_GetTick();
        return;
    }

    if (strncmp(s, "CAL STATUS", 10) == 0) {
        reply_pose_status("CAL");
        return;
    }

    if (strncmp(s, "CAL HOME", 8) == 0) {
        ArmResult_t rc = Arm_Home(g_huart);
        char buf[64];
        snprintf(buf, sizeof(buf), "CAL HOME rc=%d pos=%lu\r\n",
                 (int)rc,
                 (unsigned long)Arm_GetCurrentPositionSteps());
        reply(buf);
        g_last_cmd_ms = HAL_GetTick();
        return;
    }

    if (strncmp(s, "CAL SAVE STEPS", 14) == 0) {
        char buf[64];
        if (!Arm_PositionKnown()) {
            reply("CAL SAVE ERR: home first\r\n");
            return;
        }
        CalStore_SaveExtendSteps(Arm_GetCurrentPositionSteps());
        snprintf(buf, sizeof(buf), "CAL SAVED: retract_steps=%lu\r\n",
                 (unsigned long)Arm_GetCurrentPositionSteps());
        reply(buf);
        return;
    }

    if (strncmp(s, "CAL SERVO READ", 14) == 0) {
        char buf[48];
        snprintf(buf, sizeof(buf), "CAL SERVO POS: %u\r\n", current_servo_position());
        reply(buf);
        return;
    }

    if (strncmp(s, "CAL SERVO SET ", 14) == 0) {
        unsigned long pos = 0UL;
        unsigned long move_ms = 500UL;
        HX65_Status_t rc;
        char buf[72];

        if (sscanf(s + 14, "%lu %lu", &pos, &move_ms) < 1 || pos > 4095UL) {
            reply("CAL SERVO ERR: expected CAL SERVO SET <0-4095> [move_ms]\r\n");
            return;
        }

        rc = HX65_MovePosition(SCOOP_SERVO_ID, (uint16_t)pos, (uint16_t)move_ms);
        if (rc != HX65_OK) {
            snprintf(buf, sizeof(buf), "CAL SERVO ERR: move rc=%d\r\n", (int)rc);
            reply(buf);
            return;
        }

        g_servo_jog_pos = (uint16_t)pos;
        snprintf(buf, sizeof(buf), "CAL SERVO SET pos=%u move_ms=%u\r\n",
                 (unsigned)g_servo_jog_pos, (unsigned)move_ms);
        reply(buf);
        return;
    }

    /* CAL SERVO START — disable torque, begin printing position every 500ms */
    if (strncmp(s, "CAL SERVO START", 15) == 0) {
        HX65_DisableTorque(SCOOP_SERVO_ID);
        servo_cal_active = true;
        servo_cal_last_ms = now_ms;
        reply("CAL SERVO: torque off, printing position...\r\n");
        return;
    }

    /* CAL SERVO STOP — re-enable torque, stop printing */
    if (strncmp(s, "CAL SERVO STOP", 14) == 0) {
        servo_cal_active = false;
        HX65_EnableTorque(SCOOP_SERVO_ID);
        reply("CAL SERVO: torque on, stopped.\r\n");
        return;
    }

    /* CAL SAVE HOME/OPEN/CLOSED/DUMP — read current position and save */
    if (strncmp(s, "CAL SAVE ", 9) == 0) {
        uint16_t pos;
        uint32_t steps;
        char buf[80];
        CalServoPos_t which;

        if (!parse_servo_preset(s + 9, &which)) {
            reply("CAL SAVE ERR: unknown position\r\n");
            return;
        }

        pos = current_servo_position();
        steps = Arm_GetCurrentPositionSteps();
        CalStore_SavePose(which, steps, pos);
        snprintf(buf, sizeof(buf), "CAL SAVED: P%u=(%lu,%u)\r\n",
                 (unsigned)(which + 1U),
                 (unsigned long)steps,
                 pos);
        reply(buf);
        return;
    }
}
