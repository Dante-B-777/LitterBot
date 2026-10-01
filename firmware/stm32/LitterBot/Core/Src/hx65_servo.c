/**
 * @file    hx65_servo.c
 * @brief   Hiwonder HX-65HM bus servo driver implementation
 *
 * Implements the serial packet protocol for controlling HX-65HM servos
 * over a half-duplex UART bus. Each command is sent as a framed packet
 * and responses are read back using the same UART line.
 *
 * Half-Duplex Operation:
 *   The STM32 HAL provides HAL_HalfDuplex_EnableTransmitter() and
 *   HAL_HalfDuplex_EnableReceiver() to switch direction on a single-wire
 *   UART. The flow for each transaction is:
 *     1. Enable transmitter
 *     2. Send command packet
 *     3. Enable receiver
 *     4. Read response packet (if expected)
 *
 * Packet Format (both TX and RX):
 *   Byte 0:    0xFF (header)
 *   Byte 1:    0xFF (header)
 *   Byte 2:    ID   (servo address, 0-253)
 *   Byte 3:    Length (number of bytes after this: cmd + params + checksum)
 *   Byte 4:    Command/Instruction byte
 *   Byte 5..N: Parameter bytes (0 or more)
 *   Byte N+1:  Checksum = ~(ID + Length + Cmd + Params) & 0xFF
 *
 * @author  LitterBot Team
 * @date    2026
 */

/* Includes ------------------------------------------------------------------*/
#include "hx65_servo.h"
#include <string.h>

/* Private variables ---------------------------------------------------------*/

/** @brief UART handle used for servo communication */
static UART_HandleTypeDef *hx65_huart = NULL;

/* Private function prototypes -----------------------------------------------*/
static uint8_t HX65_CalcChecksum(uint8_t id, uint8_t length, uint8_t cmd,
                                  const uint8_t *params, uint8_t param_count);
static uint8_t HX65_CalcRawChecksum(const uint8_t *buf, uint8_t len);
static HX65_Status_t HX65_SendPacket(uint8_t id, uint8_t cmd,
                                      const uint8_t *params, uint8_t param_count);
static HX65_Status_t HX65_SendRawMovePacket(uint16_t position, uint16_t speed_steps_per_s);
static HX65_Status_t HX65_ReceivePacket(uint8_t expected_id, uint8_t *resp_buf,
                                         uint8_t *resp_length);

/* Public functions ----------------------------------------------------------*/

void HX65_Init(UART_HandleTypeDef *huart)
{
    hx65_huart = huart;
}

HX65_Status_t HX65_Ping(uint8_t id)
{
    HX65_Status_t status;
    uint8_t resp_buf[HX65_MAX_PACKET_SIZE];
    uint8_t resp_len;

    if (id > 253) {
        return HX65_ERR_PARAM;
    }

    /* Send ID_READ command as a ping -- servo will respond with its ID */
    status = HX65_SendPacket(id, HX65_CMD_ID_READ, NULL, 0);
    if (status != HX65_OK) {
        return status;
    }

    /* Wait for response */
    status = HX65_ReceivePacket(id, resp_buf, &resp_len);
    return status;
}

HX65_Status_t HX65_MovePosition(uint8_t id, uint16_t position, uint16_t time_ms)
{
    (void)id;
    (void)time_ms;

    if (position > 4095) {
        position = 4095;
    }

    return HX65_SendRawMovePacket(position, 800);
}

HX65_Status_t HX65_MovePositionSpeed(uint8_t id, uint16_t position, uint16_t speed)
{
    (void)id;

    if (position > 4095) {
        position = 4095;
    }

    return HX65_SendRawMovePacket(position, speed);
}

HX65_Status_t HX65_ReadPosition(uint8_t id, uint16_t *position_out)
{
    HX65_Status_t status;
    uint8_t resp_buf[HX65_MAX_PACKET_SIZE];
    uint8_t resp_len;

    if (id > 253 || position_out == NULL) {
        return HX65_ERR_PARAM;
    }

    /* Send position read command */
    status = HX65_SendPacket(id, HX65_CMD_POS_READ, NULL, 0);
    if (status != HX65_OK) {
        return status;
    }

    /* Read response */
    status = HX65_ReceivePacket(id, resp_buf, &resp_len);
    if (status != HX65_OK) {
        return status;
    }

    /*
     * Response params: 2 bytes, little-endian position value.
     * resp_buf contains the full packet starting from ID.
     * Params start at offset 3 (after ID, Length, Cmd).
     */
    if (resp_len < 5) {  /* ID + Len + Cmd + 2 param bytes minimum */
        return HX65_ERR_TIMEOUT;
    }

    *position_out = (uint16_t)(resp_buf[3] | (resp_buf[4] << 8));

    return HX65_OK;
}

HX65_Status_t HX65_SetID(uint8_t old_id, uint8_t new_id)
{
    uint8_t params[1];

    if (old_id > 253 || new_id > 253) {
        return HX65_ERR_PARAM;
    }

    params[0] = new_id;

    return HX65_SendPacket(old_id, HX65_CMD_ID_WRITE, params, 1);
}

HX65_Status_t HX65_SetLEDOn(uint8_t id)
{
    uint8_t params[1];

    /* LED control: 0 = LED ON (active low in protocol) */
    params[0] = 0;

    return HX65_SendPacket(id, HX65_CMD_LED_CTRL_WRITE, params, 1);
}

HX65_Status_t HX65_SetLEDOff(uint8_t id)
{
    uint8_t params[1];

    /* LED control: 1 = LED OFF (active low in protocol) */
    params[0] = 1;

    return HX65_SendPacket(id, HX65_CMD_LED_CTRL_WRITE, params, 1);
}

HX65_Status_t HX65_EnableTorque(uint8_t id)
{
    uint8_t params[1];

    /* Load/unload: 1 = torque enabled (motor engaged) */
    params[0] = 1;

    return HX65_SendPacket(id, HX65_CMD_LOAD_UNLOAD_WRITE, params, 1);
}

HX65_Status_t HX65_DisableTorque(uint8_t id)
{
    uint8_t params[1];

    /* Load/unload: 0 = torque disabled (motor free) */
    params[0] = 0;

    return HX65_SendPacket(id, HX65_CMD_LOAD_UNLOAD_WRITE, params, 1);
}

HX65_Status_t HX65_ReadVoltage(uint8_t id, uint16_t *mv_out)
{
    HX65_Status_t status;
    uint8_t resp_buf[HX65_MAX_PACKET_SIZE];
    uint8_t resp_len;

    if (id > 253 || mv_out == NULL) {
        return HX65_ERR_PARAM;
    }

    status = HX65_SendPacket(id, HX65_CMD_VIN_READ, NULL, 0);
    if (status != HX65_OK) {
        return status;
    }

    status = HX65_ReceivePacket(id, resp_buf, &resp_len);
    if (status != HX65_OK) {
        return status;
    }

    if (resp_len < 5) {
        return HX65_ERR_TIMEOUT;
    }

    /* Voltage returned in millivolts, little-endian */
    *mv_out = (uint16_t)(resp_buf[3] | (resp_buf[4] << 8));

    return HX65_OK;
}

HX65_Status_t HX65_ReadTemperature(uint8_t id, uint8_t *temp_out)
{
    HX65_Status_t status;
    uint8_t resp_buf[HX65_MAX_PACKET_SIZE];
    uint8_t resp_len;

    if (id > 253 || temp_out == NULL) {
        return HX65_ERR_PARAM;
    }

    status = HX65_SendPacket(id, HX65_CMD_TEMP_READ, NULL, 0);
    if (status != HX65_OK) {
        return status;
    }

    status = HX65_ReceivePacket(id, resp_buf, &resp_len);
    if (status != HX65_OK) {
        return status;
    }

    if (resp_len < 4) {
        return HX65_ERR_TIMEOUT;
    }

    /* Temperature in degrees Celsius, single byte */
    *temp_out = resp_buf[3];

    return HX65_OK;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Calculate packet checksum
 *
 * Checksum = ~(ID + Length + Cmd + param_bytes) & 0xFF
 *
 * @param  id          Servo ID
 * @param  length      Packet length field value
 * @param  cmd         Command/instruction byte
 * @param  params      Parameter bytes (may be NULL if param_count is 0)
 * @param  param_count Number of parameter bytes
 * @return Calculated checksum byte
 */
static uint8_t HX65_CalcChecksum(uint8_t id, uint8_t length, uint8_t cmd,
                                  const uint8_t *params, uint8_t param_count)
{
    uint8_t sum = id + length + cmd;

    for (uint8_t i = 0; i < param_count; i++) {
        sum += params[i];
    }

    return (~sum) & 0xFF;
}

static uint8_t HX65_CalcRawChecksum(const uint8_t *buf, uint8_t len)
{
    uint16_t sum = 0U;

    for (uint8_t i = 2U; i < (uint8_t)(len - 1U); i++) {
        sum += buf[i];
    }

    return (uint8_t)(~sum);
}

/**
 * @brief  Build and transmit a command packet to a servo
 *
 * Constructs the packet in a local buffer and sends it via UART.
 * For half-duplex operation, enables the transmitter before sending.
 *
 * Packet layout:
 *   [0xFF] [0xFF] [ID] [Length] [Cmd] [Params...] [Checksum]
 *
 * Length = param_count + 2 (cmd byte + checksum byte)
 *
 * @param  id          Target servo ID (0-253 or 0xFE for broadcast)
 * @param  cmd         Command instruction byte
 * @param  params      Parameter data (may be NULL if param_count is 0)
 * @param  param_count Number of parameter bytes (0-HX65_MAX_PARAMS)
 * @return HX65_OK on success, HX65_ERR_UART on UART failure
 */
static HX65_Status_t HX65_SendPacket(uint8_t id, uint8_t cmd,
                                      const uint8_t *params, uint8_t param_count)
{
    uint8_t packet[HX65_MAX_PACKET_SIZE];
    uint8_t length;
    uint8_t checksum;
    uint8_t packet_size;
    HAL_StatusTypeDef hal_status;

    if (hx65_huart == NULL) {
        return HX65_ERR_UART;
    }

    if (param_count > HX65_MAX_PARAMS) {
        return HX65_ERR_PARAM;
    }

    /* Length field = cmd(1) + params(N) + checksum(1) = param_count + 2 */
    length = param_count + 2;

    /* Calculate checksum */
    checksum = HX65_CalcChecksum(id, length, cmd, params, param_count);

    /* Build packet */
    packet[0] = HX65_HEADER;
    packet[1] = HX65_HEADER;
    packet[2] = id;
    packet[3] = length;
    packet[4] = cmd;

    /* Copy parameters */
    if (params != NULL && param_count > 0) {
        memcpy(&packet[5], params, param_count);
    }

    /* Append checksum */
    packet[5 + param_count] = checksum;

    packet_size = 5 + param_count + 1;  /* header(2) + id(1) + len(1) + cmd(1) + params + checksum(1) */

    /* Switch to transmit mode for half-duplex */
    HAL_HalfDuplex_EnableTransmitter(hx65_huart);

    /* Transmit the packet */
    hal_status = HAL_UART_Transmit(hx65_huart, packet, packet_size, HX65_UART_TIMEOUT_MS);
    if (hal_status != HAL_OK) {
        return HX65_ERR_UART;
    }

    return HX65_OK;
}

static HX65_Status_t HX65_SendRawMovePacket(uint16_t position, uint16_t speed_steps_per_s)
{
    uint8_t packet[13];
    HAL_StatusTypeDef hal_status;

    if (hx65_huart == NULL) {
        return HX65_ERR_UART;
    }

    packet[0]  = HX65_HEADER;
    packet[1]  = HX65_HEADER;
    packet[2]  = HX65_BROADCAST_ID;
    packet[3]  = 0x09;
    packet[4]  = 0x03;
    packet[5]  = 0x2A;
    packet[6]  = (uint8_t)(position & 0xFF);
    packet[7]  = (uint8_t)((position >> 8) & 0xFF);
    packet[8]  = 0x00;
    packet[9]  = 0x00;
    packet[10] = (uint8_t)(speed_steps_per_s & 0xFF);
    packet[11] = (uint8_t)((speed_steps_per_s >> 8) & 0xFF);
    packet[12] = HX65_CalcRawChecksum(packet, (uint8_t)sizeof(packet));

    HAL_HalfDuplex_EnableTransmitter(hx65_huart);
    hal_status = HAL_UART_Transmit(hx65_huart, packet, (uint16_t)sizeof(packet), HX65_UART_TIMEOUT_MS);
    if (hal_status != HAL_OK) {
        return HX65_ERR_UART;
    }

    return HX65_OK;
}

/**
 * @brief  Receive a response packet from a servo
 *
 * Switches to receive mode and reads a complete response packet.
 * Validates the header bytes, ID match, and checksum.
 *
 * The response data (ID, Length, Cmd, Params -- without headers or checksum)
 * is copied to resp_buf. The total number of bytes in resp_buf (from ID
 * through the last param, excluding checksum) is stored in resp_length.
 *
 * @param  expected_id  Expected servo ID in the response
 * @param  resp_buf     Buffer to store response data (must be >= HX65_MAX_PACKET_SIZE)
 * @param  resp_length  Pointer to store the number of valid bytes in resp_buf
 * @return HX65_OK on success, error code on failure
 */
static HX65_Status_t HX65_ReceivePacket(uint8_t expected_id, uint8_t *resp_buf,
                                         uint8_t *resp_length)
{
    uint8_t header[3];      /* 2 header bytes + ID */
    uint8_t length_byte;
    uint8_t body[HX65_MAX_PARAMS + 2];  /* cmd + params + checksum */
    uint8_t checksum;
    uint8_t calc_checksum;
    HAL_StatusTypeDef hal_status;

    if (hx65_huart == NULL) {
        return HX65_ERR_UART;
    }

    /* Switch to receive mode for half-duplex */
    HAL_HalfDuplex_EnableReceiver(hx65_huart);

    /* Read header: 0xFF 0xFF ID */
    hal_status = HAL_UART_Receive(hx65_huart, header, 3, HX65_RESPONSE_TIMEOUT_MS);
    if (hal_status != HAL_OK) {
        return HX65_ERR_TIMEOUT;
    }

    /* Validate header bytes */
    if (header[0] != HX65_HEADER || header[1] != HX65_HEADER) {
        return HX65_ERR_TIMEOUT;  /* Garbled data, treat as timeout */
    }

    /* Validate ID */
    if (header[2] != expected_id) {
        return HX65_ERR_TIMEOUT;  /* Wrong servo responded */
    }

    /* Read length byte */
    hal_status = HAL_UART_Receive(hx65_huart, &length_byte, 1, HX65_RESPONSE_TIMEOUT_MS);
    if (hal_status != HAL_OK) {
        return HX65_ERR_TIMEOUT;
    }

    /* Sanity check length (must be at least 2: cmd + checksum) */
    if (length_byte < 2 || length_byte > (HX65_MAX_PARAMS + 2)) {
        return HX65_ERR_TIMEOUT;
    }

    /* Read remaining bytes: cmd + params + checksum = length_byte bytes */
    hal_status = HAL_UART_Receive(hx65_huart, body, length_byte, HX65_RESPONSE_TIMEOUT_MS);
    if (hal_status != HAL_OK) {
        return HX65_ERR_TIMEOUT;
    }

    /* Verify checksum (last byte of body) */
    checksum = body[length_byte - 1];
    calc_checksum = HX65_CalcChecksum(expected_id, length_byte, body[0],
                                       &body[1], length_byte - 2);

    if (checksum != calc_checksum) {
        return HX65_ERR_CHECKSUM;
    }

    /*
     * Copy response data to caller's buffer.
     * Layout: [ID] [Length] [Cmd] [Params...]
     * (headers and checksum are stripped)
     */
    resp_buf[0] = expected_id;
    resp_buf[1] = length_byte;
    /* Copy cmd + params (everything except checksum) */
    memcpy(&resp_buf[2], body, length_byte - 1);

    /* Total bytes: ID(1) + Length(1) + Cmd(1) + Params(length_byte - 2) */
    *resp_length = 1 + 1 + (length_byte - 1);  /* ID + Length + (cmd + params) */

    return HX65_OK;
}
