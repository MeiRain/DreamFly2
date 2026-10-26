/**
 * @file anoprotocol.c
 * @brief Transport-independent ANO v8 frame packing.
 * @author 梅雨
 * @date 2026-10-12
 */
#include "anoprotocol.h"

#include <stddef.h>
#include <string.h>

#define ANOPROTOCOL_HEADER 0xABU
#define ANOPROTOCOL_SOURCE_ADDRESS 0x66U
#define ANOPROTOCOL_DESTINATION_ADDRESS 0xFEU
#define ANOPROTOCOL_FRAME_OVERHEAD 8U

/** @brief Encode a 16-bit value low byte first; data must have two writable bytes. */
static void anoprotocol_write_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

/** @brief Encode a 32-bit value low byte first; data must have four writable bytes. */
static void anoprotocol_write_u32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}

uint16_t anoprotocol_pack_frame(uint8_t source, uint8_t destination, uint8_t id,
                        const uint8_t *data, uint16_t length,
                        uint8_t *buffer, uint16_t capacity)
{
    uint8_t sc = 0U; /* Sum check, modulo 256. */
    uint8_t ac = 0U; /* Accumulated sum check, modulo 256. */

    if ((buffer == NULL) || ((data == NULL) && (length != 0U))
        || ((uint32_t)length + ANOPROTOCOL_FRAME_OVERHEAD > capacity))
    {
        return 0U;
    }

    buffer[0] = ANOPROTOCOL_HEADER;
    buffer[1] = source;
    buffer[2] = destination;
    buffer[3] = id;
    anoprotocol_write_u16(&buffer[4], length);
    if (length != 0U)
    {
        memcpy(&buffer[6], data, length);
    }

    /* V8.10: both checksums cover the AB header through the last payload byte. */
    for (uint16_t i = 0U; i < length + 6U; i++)
    {
        sc += buffer[i];
        ac += sc;
    }
    buffer[length + 6U] = sc;
    buffer[length + 7U] = ac;

    return length + ANOPROTOCOL_FRAME_OVERHEAD;
}

uint16_t anoprotocol_pack_imu(const int16_t acc[3], const int16_t gyro[3], uint8_t status,
                      uint8_t *buffer, uint16_t capacity)
{
    uint8_t data[13];

    if ((acc == NULL) || (gyro == NULL))
    {
        return 0U;
    }
    for (uint8_t i = 0U; i < 3U; i++)
    {
        anoprotocol_write_u16(&data[i * 2U], (uint16_t)acc[i]);
        anoprotocol_write_u16(&data[6U + i * 2U], (uint16_t)gyro[i]);
    }
    data[12] = status;
    return anoprotocol_pack_frame(ANOPROTOCOL_SOURCE_ADDRESS, ANOPROTOCOL_DESTINATION_ADDRESS,
                          0x01U, data, sizeof(data), buffer, capacity);
}

uint16_t anoprotocol_pack_mag(const int16_t mag[3], int16_t temperature_decic,
                      uint8_t status,
                      uint8_t *buffer, uint16_t capacity)
{
    uint8_t data[9];

    if (mag == NULL)
    {
        return 0U;
    }
    for (uint8_t i = 0U; i < 3U; i++)
    {
        anoprotocol_write_u16(&data[i * 2U], (uint16_t)mag[i]);
    }
    anoprotocol_write_u16(&data[6], (uint16_t)temperature_decic);
    data[8] = status;
    return anoprotocol_pack_frame(ANOPROTOCOL_SOURCE_ADDRESS, ANOPROTOCOL_DESTINATION_ADDRESS,
                          0x02U, data, sizeof(data), buffer, capacity);
}

uint16_t anoprotocol_pack_attitude(int16_t roll, int16_t pitch, int16_t yaw, uint8_t status,
                           uint8_t *buffer, uint16_t capacity)
{
    uint8_t data[7];

    anoprotocol_write_u16(&data[0], (uint16_t)roll);
    anoprotocol_write_u16(&data[2], (uint16_t)pitch);
    anoprotocol_write_u16(&data[4], (uint16_t)yaw);
    data[6] = status;
    return anoprotocol_pack_frame(ANOPROTOCOL_SOURCE_ADDRESS, ANOPROTOCOL_DESTINATION_ADDRESS,
                          0x03U, data, sizeof(data), buffer, capacity);
}

uint16_t anoprotocol_pack_battery(uint16_t battery_voltage, uint16_t battery_current,
                          uint16_t flight_voltage, uint16_t flight_current,
                          uint16_t power_state,
                          uint8_t *buffer, uint16_t capacity)
{
    uint8_t data[10];

    anoprotocol_write_u16(&data[0], battery_voltage);
    anoprotocol_write_u16(&data[2], battery_current);
    anoprotocol_write_u16(&data[4], flight_voltage);
    anoprotocol_write_u16(&data[6], flight_current);
    anoprotocol_write_u16(&data[8], power_state);
    return anoprotocol_pack_frame(ANOPROTOCOL_SOURCE_ADDRESS, ANOPROTOCOL_DESTINATION_ADDRESS,
                          0x0DU, data, sizeof(data), buffer, capacity);
}

uint16_t anoprotocol_pack_height(int32_t baro_cm, int32_t range_cm, int32_t fused_cm, uint8_t status,
                         uint8_t *buffer, uint16_t capacity)
{
    uint8_t data[13];

    anoprotocol_write_u32(&data[0], (uint32_t)baro_cm);
    anoprotocol_write_u32(&data[4], (uint32_t)range_cm);
    anoprotocol_write_u32(&data[8], (uint32_t)fused_cm);
    data[12] = status;
    return anoprotocol_pack_frame(ANOPROTOCOL_SOURCE_ADDRESS, ANOPROTOCOL_DESTINATION_ADDRESS,
                          0x05U, data, sizeof(data), buffer, capacity);
}

uint16_t anoprotocol_pack_velocity(const int16_t velocity_cms[3],
                         uint8_t *buffer, uint16_t capacity)
{
    uint8_t data[6];

    if (velocity_cms == NULL)
    {
        return 0U;
    }
    for (uint8_t i = 0U; i < 3U; i++)
    {
        anoprotocol_write_u16(&data[i * 2U], (uint16_t)velocity_cms[i]);
    }
    return anoprotocol_pack_frame(ANOPROTOCOL_SOURCE_ADDRESS, ANOPROTOCOL_DESTINATION_ADDRESS,
                          0x07U, data, sizeof(data), buffer, capacity);
}

uint16_t anoprotocol_pack_position(const int32_t position_cm[3],
                         uint8_t *buffer, uint16_t capacity)
{
    uint8_t data[12];

    if (position_cm == NULL)
    {
        return 0U;
    }
    for (uint8_t i = 0U; i < 3U; i++)
    {
        anoprotocol_write_u32(&data[i * 4U], (uint32_t)position_cm[i]);
    }
    return anoprotocol_pack_frame(ANOPROTOCOL_SOURCE_ADDRESS, ANOPROTOCOL_DESTINATION_ADDRESS,
                          0x08U, data, sizeof(data), buffer, capacity);
}

uint16_t anoprotocol_pack_target_attitude(int16_t roll, int16_t pitch, int16_t yaw,
                         uint8_t *buffer, uint16_t capacity)
{
    uint8_t data[6];

    anoprotocol_write_u16(&data[0], (uint16_t)roll);
    anoprotocol_write_u16(&data[2], (uint16_t)pitch);
    anoprotocol_write_u16(&data[4], (uint16_t)yaw);
    return anoprotocol_pack_frame(ANOPROTOCOL_SOURCE_ADDRESS, ANOPROTOCOL_DESTINATION_ADDRESS,
                          0x0AU, data, sizeof(data), buffer, capacity);
}

uint16_t anoprotocol_pack_motor(const uint16_t pwm[4],
                         uint8_t *buffer, uint16_t capacity)
{
    uint8_t data[8];

    if (pwm == NULL)
    {
        return 0U;
    }
    for (uint8_t i = 0U; i < 4U; i++)
    {
        anoprotocol_write_u16(&data[i * 2U], pwm[i]);
    }
    return anoprotocol_pack_frame(ANOPROTOCOL_SOURCE_ADDRESS, ANOPROTOCOL_DESTINATION_ADDRESS,
                          0x20U, data, sizeof(data), buffer, capacity);
}

uint16_t anoprotocol_pack_control(int16_t roll, int16_t pitch, int16_t throttle, int16_t yaw,
                         uint8_t *buffer, uint16_t capacity)
{
    uint8_t data[8];

    anoprotocol_write_u16(&data[0], (uint16_t)roll);
    anoprotocol_write_u16(&data[2], (uint16_t)pitch);
    anoprotocol_write_u16(&data[4], (uint16_t)throttle);
    anoprotocol_write_u16(&data[6], (uint16_t)yaw);
    return anoprotocol_pack_frame(ANOPROTOCOL_SOURCE_ADDRESS, ANOPROTOCOL_DESTINATION_ADDRESS,
                          0x21U, data, sizeof(data), buffer, capacity);
}

uint16_t anoprotocol_pack_flow(uint8_t type, int16_t dx, int16_t dy, uint8_t quality,
                         uint8_t *buffer, uint16_t capacity)
{
    uint8_t data[6];

    data[0] = type;
    anoprotocol_write_u16(&data[1], (uint16_t)dx);
    anoprotocol_write_u16(&data[3], (uint16_t)dy);
    data[5] = quality;
    return anoprotocol_pack_frame(ANOPROTOCOL_SOURCE_ADDRESS, ANOPROTOCOL_DESTINATION_ADDRESS,
                          0x31U, data, sizeof(data), buffer, capacity);
}

uint16_t anoprotocol_pack_range(uint8_t direction, int32_t distance_mm,
                         uint8_t *buffer, uint16_t capacity)
{
    uint8_t data[5];

    if (direction > 6U)
    {
        return 0U;
    }
    data[0] = direction;
    anoprotocol_write_u32(&data[1], (uint32_t)distance_mm);
    return anoprotocol_pack_frame(ANOPROTOCOL_SOURCE_ADDRESS, ANOPROTOCOL_DESTINATION_ADDRESS,
                          0x34U, data, sizeof(data), buffer, capacity);
}

uint16_t anoprotocol_pack_remote(const uint16_t channels[10],
                         uint8_t *buffer, uint16_t capacity)
{
    uint8_t data[20];

    if (channels == NULL)
    {
        return 0U;
    }
    for (uint8_t i = 0U; i < 10U; i++)
    {
        anoprotocol_write_u16(&data[i * 2U], channels[i]);
    }
    return anoprotocol_pack_frame(ANOPROTOCOL_SOURCE_ADDRESS, ANOPROTOCOL_DESTINATION_ADDRESS,
                          0x40U, data, sizeof(data), buffer, capacity);
}

