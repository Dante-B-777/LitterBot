/**
 * @file    scoop_sequence.h
 * @brief   Coordinated litter scoop sequence (stepper + servo)
 *
 * Executes the full scoop motion: pre-position bucket → extend arm →
 * close bucket → retract arm → dump → return home.
 *
 * =========================================================================
 * PLACEHOLDER VALUES — ALL NUMERIC CONSTANTS NEED PHYSICAL MEASUREMENT
 * =========================================================================
 * Before this sequence will work correctly you must measure and fill in:
 *
 *   SCOOP_EXTEND_STEPS       — count steps needed to fully extend the arm
 *                              Measure: run the arm until it reaches litter,
 *                              count encoder ticks / step count.
 *
 *   SCOOP_EXTEND_SPEED_US    — step delay for extension speed
 *                              Tune: slow enough not to slam, fast enough to
 *                              scoop before litter moves.
 *
 *   SCOOP_RETRACT_SPEED_US   — step delay for retraction speed
 *                              Can be faster than extension.
 *
 *   SCOOP_EXTEND_DIR         — CL57T_DIR_CW or CL57T_DIR_CCW for extension
 *                              Measure: which direction physically extends arm.
 *
 *   SCOOP_SERVO_ID           — HX65 bus ID of the bucket servo
 *                              Measure: use servo_test.c ping to find ID.
 *
 *   SCOOP_POS_HOME           — Servo position (0-4095) at stowed/travel angle
 *                              Measure: manually move servo to stowed position,
 *                              read position with HX65_ReadPosition().
 *
 *   SCOOP_POS_OPEN           — Servo position at fully open (pre-scoop) angle
 *                              Measure: position where bucket is flat/open to
 *                              slide under litter.
 *
 *   SCOOP_POS_CLOSED         — Servo position at closed/gripping angle
 *                              Measure: position that traps litter in the bucket.
 *
 *   SCOOP_POS_DUMP           — Servo position over the collection bin
 *                              Measure: angle where litter falls into the bin.
 *
 *   SCOOP_SERVO_MOVE_MS      — Time in ms for each servo move
 *                              Tune: fast enough to not jam, slow enough to
 *                              not throw litter.
 *
 * See also: Litterbot_Obsidian/Technical Documentation/Project Progress.md
 *           — "STM32 / Firmware" section for measurement procedure.
 * =========================================================================
 *
 * Sequence overview:
 *   Phase 1 — Pre-position: rotate bucket to OPEN angle
 *   Phase 2 — Extend:       drive arm forward SCOOP_EXTEND_STEPS
 *                            (limit switch provides passive safety backup)
 *   Phase 3 — Close:        rotate bucket to CLOSED angle (trap litter)
 *   Phase 4 — Retract:      pull the arm back while keeping the bucket secured
 *                            (limit switch provides passive safety backup)
 *   Phase 5 — Dump:         rotate bucket to DUMP angle over collection bin
 *   Phase 6 — Home:         rotate bucket back to HOME angle
 *
 * @note This is a BLOCKING call. It occupies the CPU for the full sequence
 *       duration (~several seconds). The STM32 UART watchdog timer is NOT
 *       refreshed during the sequence — the 250 ms motor watchdog will fire
 *       and stop the drive motors while scooping, which is intentional.
 *
 * @author  LitterBot Team
 * @date    2026
 */

#ifndef SCOOP_SEQUENCE_H
#define SCOOP_SEQUENCE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "cl57t_stepper.h"
#include "hx65_servo.h"
#include "arm_control.h"

/* =========================================================================
 * DEFAULT FALLBACK CONSTANTS
 * These values are used only if no calibration data has been saved to flash.
 * To set real values: use C CALIBRATE command (extend_steps) and
 * CAL SERVO / CAL SAVE commands (servo positions) via the UI.
 * See calibration_store.h for the runtime API.
 * ========================================================================= */

/**
 * @brief Number of steps to fully extend the arm to the scoop position.
 *
 * TODO: Measure physically.
 * Procedure: power stepper, run CL57T_MoveSteps in small increments,
 * count total steps from retracted to extended position.
 */
#define SCOOP_EXTEND_STEPS          500U    /* PLACEHOLDER — measure arm travel */

/**
 * @brief Step pulse delay (us) for arm extension — controls extension speed.
 *
 * TODO: Tune empirically. Smaller = faster. Min recommended 100 us.
 * Start slow (500 us) and reduce until motion is smooth and reliable.
 */
#define SCOOP_EXTEND_SPEED_US       300U    /* PLACEHOLDER — tune for smooth extension */

/**
 * @brief Step pulse delay (us) for arm retraction.
 *
 * TODO: Tune. Can typically be faster than extension.
 */
#define SCOOP_RETRACT_SPEED_US      200U    /* PLACEHOLDER — tune for smooth retraction */

/**
 * @brief Step delay used specifically for coordinated pose-to-pose moves.
 *
 * Slower than the free retract speed so the bucket can maintain contact with
 * the ramp while the servo tracks the saved pose curve.
 */
#define SCOOP_COORD_SPEED_US        20U

/**
 * @brief Direction of stepper rotation that extends the arm (toward litter).
 *
 * TODO: Verify physically — run stepper and observe which direction extends.
 * Set to CL57T_DIR_CW or CL57T_DIR_CCW.
 */
#define SCOOP_EXTEND_DIR            CL57T_DIR_CW    /* PLACEHOLDER — verify extend direction */

/**
 * @brief HX65 servo bus ID for the bucket servo.
 *
 * TODO: Verify with servo_test.c. Factory default ID is 1.
 * If you have multiple servos, confirm with a ping.
 */
#define SCOOP_SERVO_ID              1U      /* PLACEHOLDER — verify servo ID */

/**
 * @brief Servo position (0-4095) at home / stowed / travel angle.
 *
 * TODO: Measure. Move servo to the safe travel position by hand
 * (torque off), read position with HX65_ReadPosition(), enter here.
 * 0-4095 maps to 0-360 degrees.
 */
#define SCOOP_POS_HOME              512U    /* PLACEHOLDER — measure stowed angle */

/**
 * @brief Servo position at fully open (pre-scoop) angle.
 *
 * TODO: Measure. This is the position where the bucket is flat/open
 * so it can slide under litter when the arm extends.
 */
#define SCOOP_POS_OPEN              256U    /* PLACEHOLDER — measure ramp approach angle */

/**
 * @brief Servo position at closed/gripping angle (traps litter).
 *
 * TODO: Measure. This is the position after the arm has extended —
 * the bucket rotates to trap the litter inside before retracting.
 */
#define SCOOP_POS_CLOSED            768U    /* PLACEHOLDER — measure capture/retain angle */

/**
 * @brief Servo position at dump angle (over the collection bin).
 *
 * TODO: Measure. After retraction, the bucket rotates to this angle
 * to drop the litter into the internal collection bin.
 */
#define SCOOP_POS_DUMP              1024U   /* PLACEHOLDER — measure dump angle */

/* Aliases that match the ramp scoop terminology used during calibration. */
#define SCOOP_POS_APPROACH          SCOOP_POS_OPEN
#define SCOOP_POS_CAPTURE           SCOOP_POS_CLOSED

/**
 * @brief Time in milliseconds for each servo position move.
 *
 * TODO: Tune. This is passed as time_ms to HX65_MovePosition().
 * Fast enough to be snappy, slow enough to not throw litter out.
 */
#define SCOOP_SERVO_MOVE_MS         500U    /* PLACEHOLDER — tune servo speed */

/**
 * @brief Pause after extension before closing the bucket.
 */
#define SCOOP_PICKUP_SETTLE_MS      150U

/**
 * @brief Pause after closing to let the bucket fully trap the litter.
 */
#define SCOOP_CLOSE_SETTLE_MS       200U

/**
 * @brief Pause after retraction before tipping to dump.
 */
#define SCOOP_PRE_DUMP_SETTLE_MS    200U

/**
 * @brief Time to hold the bucket in dump position.
 */
#define SCOOP_DUMP_HOLD_MS          600U

/**
 * @brief Time to hold the bucket in home position after the sequence.
 */
#define SCOOP_HOME_SETTLE_MS        150U

typedef enum {
    SCOOP_PHASE_IDLE = 0,
    SCOOP_PHASE_PREHOME,
    SCOOP_PHASE_MOVE_TO_START,
    SCOOP_PHASE_RUN_PATH,
    SCOOP_PHASE_COMPLETE
} ScoopPhase_t;

typedef enum {
    SCOOP_OK = 0,
    SCOOP_ERR_CONFIG = -1,
    SCOOP_ERR_ARM_HOME = -2,
    SCOOP_ERR_SERVO_PRECHECK = -3,
    SCOOP_ERR_SERVO_HOME = -4,
    SCOOP_ERR_ARM_APPROACH = -5,
    SCOOP_ERR_SERVO_CAPTURE = -6,
    SCOOP_ERR_ARM_RETRACT = -7,
    SCOOP_ERR_SERVO_DUMP = -8,
    SCOOP_ERR_SERVO_RETURN_HOME = -9
} ScoopResult_t;

typedef struct {
    ScoopResult_t   result;
    ScoopPhase_t    phase;
    ArmResult_t     arm_result;
    HX65_Status_t   servo_result;
    uint32_t        extend_steps;
    uint32_t        valid_mask;
    uint8_t         executed_pose_count;
    uint8_t         last_pose_index;
} ScoopStatus_t;

/* Public function prototypes ------------------------------------------------*/

/**
 * @brief  Execute the full litter scoop sequence (blocking).
 *
 * Coordinates the stepper (arm extension) and servo (bucket rotation)
 * through all six phases. Returns when the sequence is complete and
 * both actuators are back at their home positions.
 *
 * Call this from uart_cmd.c when 'S\n' is received.
 * Do NOT call from an ISR — this function uses HAL_Delay() and blocks.
 *
 * @note Drive motors will stop during this sequence (UART watchdog fires).
 *       This is intentional — the robot should be stationary while scooping.
 */
ScoopResult_t Scoop_Execute(UART_HandleTypeDef *huart);

/**
 * @brief Get the most recent scoop execution status snapshot.
 */
const ScoopStatus_t* Scoop_GetLastStatus(void);

/**
 * @brief Convert a scoop phase enum to a human-readable string.
 */
const char* Scoop_PhaseString(ScoopPhase_t phase);

/**
 * @brief Convert a scoop result enum to a human-readable string.
 */
const char* Scoop_ResultString(ScoopResult_t result);

#ifdef __cplusplus
}
#endif

#endif /* SCOOP_SEQUENCE_H */
