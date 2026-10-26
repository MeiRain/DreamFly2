/**
 * @file rcprotocol.h
 * @brief Transport-independent remote-control protocol.
 * @author 梅雨
 * @date 2026-10-22
 */
#ifndef DREAMFLY2_COMMON_RCPROTOCOL_H
#define DREAMFLY2_COMMON_RCPROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#define RCPROTOCOL_CONTROL_FRAME_SIZE 13U
#define RCPROTOCOL_STATUS_FRAME_SIZE 18U

/* Reserved unavailable value for signed attitude/height fields; zero is valid data. */
#define RCPROTOCOL_INVALID_I16 INT16_MIN

#define RCPROTOCOL_SENSOR_ERROR_IMU   (1U << 0)
#define RCPROTOCOL_SENSOR_ERROR_MAG   (1U << 1)
#define RCPROTOCOL_SENSOR_ERROR_BARO  (1U << 2)
#define RCPROTOCOL_SENSOR_ERROR_FLOW  (1U << 3)
#define RCPROTOCOL_SENSOR_ERROR_RANGE (1U << 4)

#define RCPROTOCOL_STATUS_ARMED       0x01U /* Bit 0: aircraft is armed. */
#define RCPROTOCOL_STATUS_LOW_BATTERY 0x02U /* Bit 1: flight battery is low. */
#define RCPROTOCOL_STATUS_ALT_HOLD    0x04U /* Bit 2: altitude hold is active. */
#define RCPROTOCOL_STATUS_POS_HOLD    0x08U /* Bit 3: position hold is active. */

typedef struct
{
    uint8_t sequence; /* Increment for each new frame; wraps after 255. */
    uint16_t roll; /* All four channels: 1000..2000; centered axes: 1500. */
    uint16_t pitch;
    uint16_t yaw;
    uint16_t throttle;
    uint8_t keys; /* Bits 0..5: current debounced states of keys 1..6. */
} rcprotocol_control_t;

typedef struct
{
    uint8_t sequence;
    uint8_t flags; /* Bit 0: armed; 1: low battery; 2: altitude hold; 3: position hold.
                   * Multiple bits may be set; bits 4..7 are reserved and must be zero.
                   */
    int16_t roll_cdeg; /* Attitude angles in degrees * 100; INVALID_I16 = unavailable. */
    int16_t pitch_cdeg;
    int16_t yaw_cdeg; /* Normalize yaw to -180..180 degrees before encoding. */
    uint16_t throttle; /* Accepted RC throttle channel, 1000..2000; not motor PWM. */
    int16_t height_cm; /* Relative height in cm; INVALID_I16 = unavailable/not connected. */
    uint16_t battery_cv; /* Battery voltage in volts * 100; 0 = unavailable. */
    uint8_t sensor_errors; /* Bits 0..4: IMU, MAG, BARO, FLOW, RANGE; 1 = fault. */
} rcprotocol_status_t;

/* Caller owns buffers; input and output must not overlap.
 * Packers return frame length, or zero on invalid input/insufficient capacity.
 * Parsers require the exact frame length and leave output unchanged on failure.
 * This layer does not send data or track connection/sequence state.
 */
/**
 * @brief Pack a remote-control frame.
 * @param control Channel values, key states and caller-managed sequence number.
 * @param[out] buffer Output frame buffer.
 * @param capacity Buffer capacity in bytes.
 * @return Frame length, or 0 for invalid arguments, channels or reserved key bits.
 *         Insufficient capacity also returns 0.
 */
uint8_t rcprotocol_pack_control(const rcprotocol_control_t *control,
                            uint8_t *buffer, uint8_t capacity);

/**
 * @brief Validate and parse a remote-control frame.
 * @param buffer Received frame.
 * @param size Actual frame length; must equal RCPROTOCOL_CONTROL_FRAME_SIZE.
 * @param[out] control Decoded data; unchanged on failure.
 * @return true if arguments, frame format, checksum, channels and key bits are valid.
 * @note Sequence freshness, key events and link timeout belong to the application.
 */
bool rcprotocol_parse_control(const uint8_t *buffer, uint8_t size,
                          rcprotocol_control_t *control);

/**
 * @brief Pack a flight-status frame.
 * @param status Flight state in the encoded units documented in rcprotocol_status_t.
 * @param[out] buffer Output frame buffer.
 * @param capacity Buffer capacity in bytes.
 * @return Frame length, or 0 for a NULL pointer or insufficient capacity.
 * @note Caller converts units, normalizes yaw and sets status bits.
 */
uint8_t rcprotocol_pack_status(const rcprotocol_status_t *status,
                           uint8_t *buffer, uint8_t capacity);

/**
 * @brief Validate and parse a flight-status frame.
 * @param buffer Received frame.
 * @param size Actual frame length; must equal RCPROTOCOL_STATUS_FRAME_SIZE.
 * @param[out] status Decoded data in protocol units; unchanged on failure.
 * @return true if arguments, frame format and checksum are valid.
 * @note Does not validate data freshness or the meaning of status fields.
 */
bool rcprotocol_parse_status(const uint8_t *buffer, uint8_t size,
                         rcprotocol_status_t *status);

#endif /* DREAMFLY2_COMMON_RCPROTOCOL_H */
