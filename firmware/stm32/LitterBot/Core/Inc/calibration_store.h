/**
 * @file    calibration_store.h
 * @brief   Persistent calibration storage in STM32 flash
 *
 * Stores scoop mechanism calibration values in the last flash page
 * (Bank 2, page 255, address 0x080FF800). Values are loaded into RAM
 * on boot and used by scoop_sequence.c at runtime.
 *
 * Flash layout stores up to 10 full arm poses:
 *   - max retract steps from front-zero
 *   - valid bitmask for saved poses
 *   - per-pose stepper offset from front-zero
 *   - per-pose servo position
 *
 * If magic is absent (first boot / erased), all getters return the
 * #define defaults from scoop_sequence.h.
 */

#ifndef CALIBRATION_STORE_H
#define CALIBRATION_STORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include <stdint.h>

/** @brief Flash page reserved for calibration data */
#define CAL_FLASH_BANK    FLASH_BANK_2
#define CAL_FLASH_PAGE    255
#define CAL_FLASH_ADDR    0x080FF800UL
#define CAL_MAGIC         0xCAFE0003UL
#define CAL_POSE_COUNT    10U

/** @brief Servo position identifiers */
typedef enum {
    CAL_POS_1  = 0,
    CAL_POS_2  = 1,
    CAL_POS_3  = 2,
    CAL_POS_4  = 3,
    CAL_POS_5  = 4,
    CAL_POS_6  = 5,
    CAL_POS_7  = 6,
    CAL_POS_8  = 7,
    CAL_POS_9  = 8,
    CAL_POS_10 = 9,
    CAL_POS_HOME   = CAL_POS_1,
    CAL_POS_OPEN   = CAL_POS_2,
    CAL_POS_CLOSED = CAL_POS_3,
    CAL_POS_DUMP   = CAL_POS_4,
    CAL_POS_COUNT  = CAL_POSE_COUNT
} CalServoPos_t;

/**
 * @brief Load calibration values from flash into RAM.
 * Call once at boot, before UartCmd_Init.
 */
void CalStore_Init(void);

/**
 * @brief Get the calibrated arm extend step count.
 * Returns stored value if valid, otherwise SCOOP_EXTEND_STEPS default.
 */
uint32_t CalStore_GetExtendSteps(void);

/**
 * @brief Get a calibrated servo position.
 * Returns stored value if valid, otherwise the corresponding SCOOP_POS_* default.
 * @param which  Which position to retrieve (CAL_POS_HOME/OPEN/CLOSED/DUMP)
 */
uint16_t CalStore_GetServoPos(CalServoPos_t which);
uint32_t CalStore_GetPoseSteps(CalServoPos_t which);
uint32_t CalStore_GetValidMask(void);
uint8_t CalStore_PoseIsSaved(CalServoPos_t which);

/**
 * @brief Save extend step count to flash and update RAM immediately.
 * @param steps  Measured step count from arm calibration
 */
void CalStore_SaveExtendSteps(uint32_t steps);

/**
 * @brief Save a servo position to flash and update RAM immediately.
 * @param which  Which position to save
 * @param pos    Servo position value (0-4095)
 */
void CalStore_SaveServoPos(CalServoPos_t which, uint16_t pos);
void CalStore_SavePose(CalServoPos_t which, uint32_t steps, uint16_t pos);

#ifdef __cplusplus
}
#endif

#endif /* CALIBRATION_STORE_H */
