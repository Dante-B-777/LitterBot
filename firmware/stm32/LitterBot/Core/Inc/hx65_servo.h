/**
 * @file    hx65_servo.h
 * @brief   Hiwonder HX-65HM bus servo driver interface
 *
 * Controls Hiwonder HX-65HM magnetic encoder bus servos over a
 * half-duplex UART bus. Multiple servos (up to 253) can share
 * a single UART line, addressed by unique IDs.
 *
 * Protocol Summary:
 *   - Baud rate: 115200 (default)
 *   - Half-duplex: single wire for TX and RX
 *   - Packet format: [0xFF] [0xFF] [ID] [Length] [Cmd] [Params...] [Checksum]
 *   - Checksum = ~(ID + Length + Cmd + Params) & 0xFF
 *   - Length = number of params + 2 (includes Cmd and Checksum bytes)
 *
 * UART Assignment:
 *   - USART3 TX on PC10 (half-duplex, single-wire)
 *   - Originally PB10 (soldered), but PB10 was damaged; rewired to PC10
 *
 * Servo Specifications (HX-65HM):
 *   - Position range: 0-4095 (maps to 0-360 degrees)
 *   - Resolution: 0.088 degrees (12-bit magnetic encoder)
 *   - Torque: 50 kg*cm @ 9.6V, 65 kg*cm @ 12.6V
 *   - Operating voltage: 9-12.6V
 *   - Feedback: position, speed, load, temperature, voltage
 *
 * NOTE: This driver will not function until CubeMX enables:
 *   - A USART peripheral in half-duplex mode
 *   - The corresponding GPIO pin in alternate function mode
 *
 * @author  LitterBot Team
 * @date    2026
 */

#ifndef HX65_SERVO_H
#define HX65_SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include <stdint.h>

/* Protocol constants --------------------------------------------------------*/

/** @brief Packet header bytes (two consecutive 0xFF) */
#define HX65_HEADER             0xFF

/** @brief Broadcast ID -- commands sent to all servos (no response) */
#define HX65_BROADCAST_ID       0xFE

/** @brief Maximum number of parameter bytes in a single packet */
#define HX65_MAX_PARAMS         10

/** @brief Maximum total packet size: header(2) + ID(1) + len(1) + cmd(1) + params(max) + checksum(1) */
#define HX65_MAX_PACKET_SIZE    (2 + 1 + 1 + 1 + HX65_MAX_PARAMS + 1)

/** @brief Default UART timeout in milliseconds for TX/RX operations */
#define HX65_UART_TIMEOUT_MS    10

/** @brief Timeout waiting for a servo response packet in milliseconds */
#define HX65_RESPONSE_TIMEOUT_MS  20

/* Servo command instruction set ---------------------------------------------*/

/**
 * @brief HX-65HM instruction codes
 *
 * These are the command bytes sent in position [4] of each packet.
 * The servo echoes a response packet with the same ID for read commands.
 */
#define HX65_CMD_MOVE_TIME_WRITE    1   /**< Move to position over time */
#define HX65_CMD_MOVE_TIME_READ     2   /**< Read pending move command */
#define HX65_CMD_MOVE_TIME_WAIT     7   /**< Load move (execute on trigger) */
#define HX65_CMD_MOVE_START         11  /**< Trigger loaded move */
#define HX65_CMD_MOVE_STOP          12  /**< Stop current movement */
#define HX65_CMD_ID_WRITE           13  /**< Set servo ID */
#define HX65_CMD_ID_READ            14  /**< Read servo ID */
#define HX65_CMD_ANGLE_OFFSET       17  /**< Set angle offset */
#define HX65_CMD_ANGLE_LIMIT_WRITE  20  /**< Set angle limits */
#define HX65_CMD_ANGLE_LIMIT_READ   21  /**< Read angle limits */
#define HX65_CMD_VIN_LIMIT_WRITE    22  /**< Set voltage limits */
#define HX65_CMD_VIN_LIMIT_READ     23  /**< Read voltage limits */
#define HX65_CMD_TEMP_LIMIT_WRITE   24  /**< Set temperature limit */
#define HX65_CMD_TEMP_LIMIT_READ    25  /**< Read temperature limit */
#define HX65_CMD_POS_READ           28  /**< Read current position */
#define HX65_CMD_MODE_WRITE         29  /**< Set servo/motor mode */
#define HX65_CMD_MODE_READ          30  /**< Read current mode */
#define HX65_CMD_LOAD_UNLOAD_WRITE  31  /**< Enable/disable torque */
#define HX65_CMD_LOAD_UNLOAD_READ   32  /**< Read torque enable state */
#define HX65_CMD_LED_CTRL_WRITE     33  /**< LED on/off control */
#define HX65_CMD_LED_CTRL_READ      34  /**< Read LED state */
#define HX65_CMD_LED_ERROR_WRITE    35  /**< Set LED error display */
#define HX65_CMD_LED_ERROR_READ     36  /**< Read LED error display */
#define HX65_CMD_VIN_READ           27  /**< Read input voltage */
#define HX65_CMD_TEMP_READ          26  /**< Read temperature */

/* Return status codes -------------------------------------------------------*/

/** @brief HX65 function return codes */
typedef enum {
    HX65_OK          = 0,   /**< Operation successful */
    HX65_ERR_PARAM   = -1,  /**< Invalid parameter */
    HX65_ERR_UART    = -2,  /**< UART communication error */
    HX65_ERR_TIMEOUT = -3,  /**< No response from servo */
    HX65_ERR_CHECKSUM = -4  /**< Response checksum mismatch */
} HX65_Status_t;

/* Public function prototypes ------------------------------------------------*/

/**
 * @brief  Initialize the servo driver with a UART handle
 *
 * Stores the UART handle reference for all subsequent communication.
 * The UART must already be initialized in half-duplex mode at 115200 baud
 * before calling this function.
 *
 * @param  huart  Pointer to an initialized UART handle (half-duplex mode)
 */
void HX65_Init(UART_HandleTypeDef *huart);

/**
 * @brief  Ping a servo to check if it's alive on the bus
 *
 * Sends a ping packet to the specified ID and waits for a response.
 * If the servo is present and working, it will echo back a status packet.
 *
 * @param  id  Servo ID to ping (0-253)
 * @return HX65_OK if servo responds, HX65_ERR_TIMEOUT if no response,
 *         or other error code on communication failure
 */
HX65_Status_t HX65_Ping(uint8_t id);

/**
 * @brief  Move a servo to a position over a specified time
 *
 * Commands the servo to move to the target position, arriving in
 * the specified time duration. The servo interpolates smoothly.
 *
 * @param  id        Servo ID (0-253, or HX65_BROADCAST_ID for all)
 * @param  position  Target position (0-4095, maps to 0-360 degrees)
 * @param  time_ms   Movement duration in milliseconds (0-30000)
 *                   0 = move as fast as possible
 * @return HX65_OK on success, or error code
 */
HX65_Status_t HX65_MovePosition(uint8_t id, uint16_t position, uint16_t time_ms);

/**
 * @brief  Move a servo to a position at a specified speed
 *
 * Lower-level variant of HX65_MovePosition that exposes the raw speed
 * field from the v2 packet format (cmd 0x03 register write at 0x2A).
 * Used by the jog path so the caller can pick a speed that matches the
 * desired glide rate — lower speeds give smoother, slower motion.
 *
 * @param  id        Servo ID (0-253, or HX65_BROADCAST_ID for all)
 * @param  position  Target position (0-4095)
 * @param  speed     Speed in raw HX-65HM units (position steps per second,
 *                   approximately). 800 matches v2 default. Useful range
 *                   ~200 (very slow) to ~2000 (fast).
 * @return HX65_OK on success, or error code
 */
HX65_Status_t HX65_MovePositionSpeed(uint8_t id, uint16_t position, uint16_t speed);

/**
 * @brief  Read the current position of a servo
 *
 * Queries the servo for its current angular position.
 * The returned value is in the range 0-4095 (0-360 degrees).
 *
 * @param  id           Servo ID (0-253, not broadcast)
 * @param  position_out Pointer to store the read position value
 * @return HX65_OK on success, or error code
 */
HX65_Status_t HX65_ReadPosition(uint8_t id, uint16_t *position_out);

/**
 * @brief  Change a servo's ID
 *
 * Assigns a new bus address to the servo. The servo will only respond
 * to the new ID after this command. Use with caution -- if two servos
 * share the same ID, the bus will have conflicts.
 *
 * @param  old_id  Current servo ID (0-253)
 * @param  new_id  Desired new ID (0-253)
 * @return HX65_OK on success, or error code
 */
HX65_Status_t HX65_SetID(uint8_t old_id, uint8_t new_id);

/**
 * @brief  Turn on the servo's LED indicator
 *
 * @param  id  Servo ID (0-253, or HX65_BROADCAST_ID for all)
 * @return HX65_OK on success, or error code
 */
HX65_Status_t HX65_SetLEDOn(uint8_t id);

/**
 * @brief  Turn off the servo's LED indicator
 *
 * @param  id  Servo ID (0-253, or HX65_BROADCAST_ID for all)
 * @return HX65_OK on success, or error code
 */
HX65_Status_t HX65_SetLEDOff(uint8_t id);

/**
 * @brief  Enable servo torque output (engage motor)
 *
 * When torque is enabled, the servo actively holds its position.
 * Torque is enabled by default on power-up.
 *
 * @param  id  Servo ID (0-253, or HX65_BROADCAST_ID for all)
 * @return HX65_OK on success, or error code
 */
HX65_Status_t HX65_EnableTorque(uint8_t id);

/**
 * @brief  Disable servo torque output (release motor)
 *
 * When torque is disabled, the servo can be moved freely by hand.
 * Useful for manual positioning or to reduce power consumption.
 *
 * @param  id  Servo ID (0-253, or HX65_BROADCAST_ID for all)
 * @return HX65_OK on success, or error code
 */
HX65_Status_t HX65_DisableTorque(uint8_t id);

/**
 * @brief  Read the servo's input voltage
 *
 * @param  id       Servo ID (0-253)
 * @param  mv_out   Pointer to store voltage in millivolts
 * @return HX65_OK on success, or error code
 */
HX65_Status_t HX65_ReadVoltage(uint8_t id, uint16_t *mv_out);

/**
 * @brief  Read the servo's internal temperature
 *
 * @param  id        Servo ID (0-253)
 * @param  temp_out  Pointer to store temperature in degrees Celsius
 * @return HX65_OK on success, or error code
 */
HX65_Status_t HX65_ReadTemperature(uint8_t id, uint8_t *temp_out);

#ifdef __cplusplus
}
#endif

#endif /* HX65_SERVO_H */
