/**
 * @file rcprotocol.c
 * @brief Remote-control frame packing and parsing.
 * @author 梅雨
 * @date 2026-10-22
 */
#include "rcprotocol.h"

#include <stddef.h>

#define RCPROTOCOL_HEADER 0xABU
#define RCPROTOCOL_TYPE_CONTROL 0x01U
#define RCPROTOCOL_TYPE_STATUS 0x02U

/** @brief Sum bytes modulo 256, excluding the checksum byte itself. */
static uint8_t rcprotocol_checksum(const uint8_t *buffer, uint8_t size)
{
    uint8_t sum = 0U;
    for (uint8_t i = 0U; i < size; i++)
    {
        sum = (uint8_t)(sum + buffer[i]);
    }
    return sum;
}

/** @brief Encode an unsigned integer, high byte first; requires two writable bytes. */
static void rcprotocol_write_u16(uint8_t *buffer, uint16_t value)
{
    buffer[0] = (uint8_t)(value >> 8);
    buffer[1] = (uint8_t)value;
}

/** @brief Decode an unsigned integer, high byte first; requires two readable bytes. */
static uint16_t rcprotocol_read_u16(const uint8_t *buffer)
{
    return (uint16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
}

/** @brief Decode a high-byte-first, 16-bit two's-complement integer. */
static int16_t rcprotocol_read_i16(const uint8_t *buffer)
{
    uint16_t value = rcprotocol_read_u16(buffer);
    return (int16_t)(value <= 32767U ? (int32_t)value : (int32_t)value - 65536);
}

/** @brief Validate channel ranges and reserved key bits; control must not be NULL. */
static bool rcprotocol_channels_valid(const rcprotocol_control_t *control)
{
    return control->roll >= 1000U && control->roll <= 2000U &&
           control->pitch >= 1000U && control->pitch <= 2000U &&
           control->yaw >= 1000U && control->yaw <= 2000U &&
           control->throttle >= 1000U && control->throttle <= 2000U &&
           (control->keys & 0xC0U) == 0U;
}

/**
 * @brief Validate the pointer, frame length, header, type and checksum.
 * @note Internal callers supply a fixed expected_size of at least three bytes.
 */
static bool rcprotocol_frame_valid(const uint8_t *buffer, uint8_t size,
                               uint8_t expected_size, uint8_t type)
{
    return buffer != NULL && size == expected_size &&
           buffer[0] == RCPROTOCOL_HEADER && buffer[1] == type &&
           buffer[size - 1U] == rcprotocol_checksum(buffer, (uint8_t)(size - 1U));
}

uint8_t rcprotocol_pack_control(const rcprotocol_control_t *control,
                            uint8_t *buffer, uint8_t capacity)
{
    if (control == NULL || buffer == NULL || capacity < RCPROTOCOL_CONTROL_FRAME_SIZE ||
        !rcprotocol_channels_valid(control))
    {
        return 0U;
    }

    buffer[0] = RCPROTOCOL_HEADER;
    buffer[1] = RCPROTOCOL_TYPE_CONTROL;
    buffer[2] = control->sequence;
    rcprotocol_write_u16(&buffer[3], control->roll);
    rcprotocol_write_u16(&buffer[5], control->pitch);
    rcprotocol_write_u16(&buffer[7], control->yaw);
    rcprotocol_write_u16(&buffer[9], control->throttle);
    buffer[11] = control->keys;
    buffer[12] = rcprotocol_checksum(buffer, 12U);
    return RCPROTOCOL_CONTROL_FRAME_SIZE;
}

bool rcprotocol_parse_control(const uint8_t *buffer, uint8_t size,
                          rcprotocol_control_t *control)
{
    if (control == NULL || !rcprotocol_frame_valid(buffer, size,
        RCPROTOCOL_CONTROL_FRAME_SIZE, RCPROTOCOL_TYPE_CONTROL))
    {
        return false;
    }

    rcprotocol_control_t decoded = {
        .sequence = buffer[2],
        .roll = rcprotocol_read_u16(&buffer[3]),
        .pitch = rcprotocol_read_u16(&buffer[5]),
        .yaw = rcprotocol_read_u16(&buffer[7]),
        .throttle = rcprotocol_read_u16(&buffer[9]),
        .keys = buffer[11]
    };
    if (!rcprotocol_channels_valid(&decoded))
    {
        return false;
    }
    *control = decoded;
    return true;
}

uint8_t rcprotocol_pack_status(const rcprotocol_status_t *status,
                           uint8_t *buffer, uint8_t capacity)
{
    if (status == NULL || buffer == NULL || capacity < RCPROTOCOL_STATUS_FRAME_SIZE)
    {
        return 0U;
    }

    buffer[0] = RCPROTOCOL_HEADER;
    buffer[1] = RCPROTOCOL_TYPE_STATUS;
    buffer[2] = status->sequence;
    buffer[3] = status->flags;
    rcprotocol_write_u16(&buffer[4], (uint16_t)status->roll_cdeg);
    rcprotocol_write_u16(&buffer[6], (uint16_t)status->pitch_cdeg);
    rcprotocol_write_u16(&buffer[8], (uint16_t)status->yaw_cdeg);
    rcprotocol_write_u16(&buffer[10], status->battery_cv);
    buffer[12] = status->sensor_errors;
    rcprotocol_write_u16(&buffer[13], status->throttle);
    rcprotocol_write_u16(&buffer[15], (uint16_t)status->height_cm);
    buffer[17] = rcprotocol_checksum(buffer, 17U);
    return RCPROTOCOL_STATUS_FRAME_SIZE;
}

bool rcprotocol_parse_status(const uint8_t *buffer, uint8_t size,
                         rcprotocol_status_t *status)
{
    if (status == NULL || !rcprotocol_frame_valid(buffer, size,
        RCPROTOCOL_STATUS_FRAME_SIZE, RCPROTOCOL_TYPE_STATUS))
    {
        return false;
    }

    rcprotocol_status_t decoded = {
        .sequence = buffer[2],
        .flags = buffer[3],
        .roll_cdeg = rcprotocol_read_i16(&buffer[4]),
        .pitch_cdeg = rcprotocol_read_i16(&buffer[6]),
        .yaw_cdeg = rcprotocol_read_i16(&buffer[8]),
        .battery_cv = rcprotocol_read_u16(&buffer[10]),
        .sensor_errors = buffer[12],
        .throttle = rcprotocol_read_u16(&buffer[13]),
        .height_cm = rcprotocol_read_i16(&buffer[15])
    };
    *status = decoded;
    return true;
}
