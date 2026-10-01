/**
 * @file    arm_control.h
 * @brief   Scoop arm control with calibration and coordinated servo motion
 *
 * Provides two operating modes for the scoop arm:
 *
 * 1. CALIBRATION MODE (triggered by 'C\n' UART command):
 *    - Slowly extends arm until EXTEND limit switch triggers, counts steps
 *    - Slowly retracts arm until HOME limit switch triggers, counts steps
 *    - Prints step counts over LPUART1 for the user to hardcode
 *    - Safety: B1 button = latched emergency stop (requires MCU reset)
 *    - Safety: Wrong-direction limit switch = immediate stop + error
 *      (e.g., HOME switch triggers while extending = mechanical fault)
 *
 * 2. NORMAL MODE (used during scoop sequence):
 *    - Uses hardcoded step count (from calibration) for arm travel
 *    - Limit switches act as passive emergency backup only
 *    - Servo angle interpolates as a function of stepper position
 *      during retraction (keeps litter from spilling off the bucket)
 *
 * Dependencies:
 *    - cl57t_stepper.h  — stepper step/direction control
 *    - hx65_servo.h     — servo position control
 *    - limit_switch.h   — limit switch reading
 *    - main.h           — B1_Pin, B1_GPIO_Port definitions
 *
 * @author  LitterBot Team
 * @date    2026
 */

#ifndef ARM_CONTROL_H
#define ARM_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* Calibration speed ---------------------------------------------------------*/

/**
 * @brief Step delay (us) used during calibration — intentionally slow.
 *
 * 500 us per step = ~1000 steps/sec. Slow enough to stop quickly
 * and observe the arm moving. Adjust if too slow or too fast.
 */
#define ARM_CAL_SPEED_US        500U

/**
 * @brief Maximum steps allowed during calibration before auto-stopping.
 *
 * Safety limit in case both limit switches fail during calibration.
 * If the arm hasn't hit a switch after this many steps, something
 * is wrong. Set conservatively high — the arm physically can't
 * travel this far.
 */
#define ARM_CAL_MAX_STEPS       10000U

/* Servo interpolation -------------------------------------------------------*/

/**
 * @brief How often to update the servo position during retraction.
 *
 * Updating every single step would flood the UART bus. Instead,
 * update every N steps. A smaller interval keeps the servo visibly synced
 * with the lead-screw motion instead of lagging behind it.
 */
#define ARM_SERVO_UPDATE_INTERVAL  80U

/**
 * @brief Servo lookahead in step counts during coordinated moves.
 *
 * The servo target is computed slightly ahead of the current stepper slice so
 * the bucket lifts into the ramp before the lead screw pulls farther back.
 */
#define ARM_SERVO_LEAD_STEPS       120U

/* Return codes --------------------------------------------------------------*/

typedef enum {
    ARM_OK              =  0,   /**< Operation completed normally */
    ARM_ERR_B1_ABORT    = -1,   /**< B1 button pressed — latched, requires reset */
    ARM_ERR_WRONG_LIMIT = -2,   /**< Wrong-direction limit switch triggered */
    ARM_ERR_MAX_STEPS   = -3,   /**< Hit max step safety limit */
    ARM_ERR_LIMIT_SAFETY = -4   /**< Limit switch emergency stop in normal mode */
} ArmResult_t;

/* Public function prototypes ------------------------------------------------*/

/**
 * @brief  Initialize the arm control module
 *
 * Calls LimitSwitch_Init() and CL57T_Init(). The servo (HX65) must
 * be initialized separately (it needs the UART handle from main).
 */
void Arm_Init(void);

/**
 * @brief  Run calibration: extend until limit, then retract until limit
 *
 * Intended to be run once with the operator watching and B1 ready.
 *
 * Procedure:
 *   1. Arm extends slowly until EXTEND limit switch triggers
 *   2. Step count printed over LPUART1
 *   3. Arm retracts slowly until HOME limit switch triggers
 *   4. Step count printed over LPUART1
 *
 * Safety stops (any of these halts motion immediately):
 *   - B1 button pressed (latched — requires MCU reset)
 *   - Wrong-direction limit switch (HOME while extending, EXTEND while retracting)
 *   - ARM_CAL_MAX_STEPS reached without hitting expected switch
 *
 * @param  huart   LPUART1 handle for printing step counts
 * @return ARM_OK on success, or error code indicating what stopped it
 */
ArmResult_t Arm_Calibrate(UART_HandleTypeDef *huart);

/**
 * @brief  Extend arm using hardcoded step count (normal operation)
 *
 * Moves the arm forward by the specified number of steps.
 * Limit switches serve as passive emergency backup — if the EXTEND
 * switch triggers unexpectedly, motion stops and returns an error.
 *
 * @param  steps     Number of steps to extend
 * @param  dir       Direction for extension (CL57T_DIR_CW or CL57T_DIR_CCW)
 * @param  speed_us  Step delay in microseconds
 * @return ARM_OK on success, ARM_ERR_LIMIT_SAFETY if limit switch triggered
 */
ArmResult_t Arm_Extend(uint32_t steps, uint8_t dir, uint32_t speed_us);

/**
 * @brief  Extend arm while interpolating the scoop servo position
 *
 * Moves the arm forward by the specified number of steps while updating the
 * servo position every ARM_SERVO_UPDATE_INTERVAL steps. This is intended for
 * the scoop approach stroke so the bucket can gradually settle into its
 * approach angle instead of snapping there before the arm starts moving.
 *
 * Passive limit-switch safety matches Arm_Extend().
 */
ArmResult_t Arm_ExtendWithServo(uint32_t steps, uint8_t dir, uint32_t speed_us,
                                uint8_t servo_id, uint16_t servo_start,
                                uint16_t servo_end);

/**
 * @brief  Retract arm with coordinated servo interpolation (normal operation)
 *
 * Moves the arm backward by the specified number of steps. During
 * retraction, the servo angle interpolates linearly from servo_start
 * to servo_end based on progress through the step count.
 *
 * This keeps the bucket angle changing smoothly as the arm pulls back,
 * preventing litter from spilling.
 *
 * Limit switches serve as passive emergency backup.
 *
 * @param  steps        Number of steps to retract
 * @param  dir          Direction for retraction (opposite of extend)
 * @param  speed_us     Step delay in microseconds
 * @param  servo_id     HX65 servo bus ID
 * @param  servo_start  Servo position at start of retraction (0-4095)
 * @param  servo_end    Servo position at end of retraction (0-4095)
 * @return ARM_OK on success, ARM_ERR_LIMIT_SAFETY if limit switch triggered
 */
ArmResult_t Arm_RetractWithServo(uint32_t steps, uint8_t dir, uint32_t speed_us,
                                  uint8_t servo_id, uint16_t servo_start,
                                  uint16_t servo_end);

/**
 * @brief  Retract arm while holding a servo angle, then interpolating to end
 *
 * Keeps the servo fixed at @p servo_hold for @p hold_steps of the retract
 * stroke, then linearly interpolates to @p servo_end across the remaining
 * steps. This allows the bucket to stay in a capture posture early in the
 * retract and tip toward the bin later in the stroke.
 */
ArmResult_t Arm_RetractWithServoProfile(uint32_t steps, uint8_t dir,
                                        uint32_t speed_us, uint8_t servo_id,
                                        uint16_t servo_hold, uint16_t servo_end,
                                        uint32_t hold_steps);

/**
 * @brief  Home the arm by retracting to the HOME limit switch
 *
 * Used during deploy sequence. If the arm is already at the HOME
 * switch, returns immediately. Otherwise, slowly retracts until
 * the HOME switch triggers or a safety stop occurs.
 *
 * Safety stops:
 *   - Wrong-direction limit switch (EXTEND triggers while retracting)
 *   - ARM_CAL_MAX_STEPS reached without hitting HOME
 *
 * @param  huart   LPUART1 handle for status messages
 * @return ARM_OK on success, or error code
 */
ArmResult_t Arm_Home(UART_HandleTypeDef *huart);

/**
 * @brief Return true once the arm has been homed to the front reference switch.
 */
bool Arm_PositionKnown(void);

/**
 * @brief Current arm offset in steps from the front reference switch.
 *
 * 0 means the carriage is at the front switch. Positive values are retracted
 * back from that front-zero reference.
 */
uint32_t Arm_GetCurrentPositionSteps(void);

/**
 * @brief Check whether a single step in @p dir is currently safe.
 *
 * Forward/extend is blocked when the front switch is active. Retract/backward
 * is blocked until the arm has been homed, and then by the stored travel limit.
 */
bool Arm_CanStep(uint8_t dir);

/**
 * @brief Calibration/manual jog safety gate.
 *
 * Uses the front switch as the only forward stop, but allows retraction up to
 * the broad calibration safety limit instead of the saved scoop travel.
 */
bool Arm_CanJogStep(uint8_t dir);

/**
 * @brief Notify the arm tracker that one physical step has been executed.
 */
void Arm_NotifyStep(uint8_t dir);

#ifdef __cplusplus
}
#endif

#endif /* ARM_CONTROL_H */
