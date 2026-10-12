/**
 * @file ano.c
 * @brief Transport-independent ANO v8 frame packing.
 * @author 梅雨
 * @date 2026-10-12
 */
#include "ano.h"

#include <stddef.h>
#include <string.h>

#define ANO_HEADER 0xABU
#define ANO_SOURCE_ADDRESS 0x66U
#define ANO_DESTINATION_ADDRESS 0xFEU
#define ANO_FRAME_OVERHEAD 8U

static void ano_write_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

uint16_t ano_pack_frame(uint8_t source, uint8_t destination, uint8_t id,
                        const uint8_t *data, uint16_t length,
                        uint8_t *buffer, uint16_t capacity)
{
    uint8_t sc = 0U;
    uint8_t ac = 0U;

    if ((buffer == NULL) || ((data == NULL) && (length != 0U))
        || ((uint32_t)length + ANO_FRAME_OVERHEAD > capacity))
    {
        return 0U;
    }

    buffer[0] = ANO_HEADER;
    buffer[1] = source;
    buffer[2] = destination;
    buffer[3] = id;
    ano_write_u16(&buffer[4], length);
    if (length != 0U)
    {
        memcpy(&buffer[6], data, length);
    }

    /* Supplied v8 reference: checksum covers SRC through DATA, excluding AB. */
    for (uint16_t i = 1U; i < length + 6U; ++i)
    {
        sc += buffer[i];
        ac += sc;
    }
    buffer[length + 6U] = sc;
    buffer[length + 7U] = ac;

    return length + ANO_FRAME_OVERHEAD;
}

uint16_t ano_pack_imu(const int16_t acc[3], const int16_t gyro[3], uint8_t status,
                      uint8_t *buffer, uint16_t capacity)
{
    uint8_t data[13];

    if ((acc == NULL) || (gyro == NULL))
    {
        return 0U;
    }
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        ano_write_u16(&data[i * 2U], (uint16_t)acc[i]);
        ano_write_u16(&data[6U + i * 2U], (uint16_t)gyro[i]);
    }
    data[12] = status;
    return ano_pack_frame(ANO_SOURCE_ADDRESS, ANO_DESTINATION_ADDRESS,
                          0x01U, data, sizeof(data), buffer, capacity);
}

uint16_t ano_pack_mag(const int16_t mag[3], uint8_t status,
                      uint8_t *buffer, uint16_t capacity)
{
    uint8_t data[7];

    if (mag == NULL)
    {
        return 0U;
    }
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        ano_write_u16(&data[i * 2U], (uint16_t)mag[i]);
    }
    data[6] = status;
    return ano_pack_frame(ANO_SOURCE_ADDRESS, ANO_DESTINATION_ADDRESS,
                          0x02U, data, sizeof(data), buffer, capacity);
}

uint16_t ano_pack_attitude(int16_t roll, int16_t pitch, int16_t yaw, uint8_t status,
                           uint8_t *buffer, uint16_t capacity)
{
    uint8_t data[7];

    ano_write_u16(&data[0], (uint16_t)roll);
    ano_write_u16(&data[2], (uint16_t)pitch);
    ano_write_u16(&data[4], (uint16_t)yaw);
    data[6] = status;
    return ano_pack_frame(ANO_SOURCE_ADDRESS, ANO_DESTINATION_ADDRESS,
                          0x03U, data, sizeof(data), buffer, capacity);
}

uint16_t ano_pack_battery(uint16_t battery_voltage, uint16_t battery_current,
                          uint16_t flight_voltage, uint16_t flight_current,
                          uint8_t *buffer, uint16_t capacity)
{
    uint8_t data[8];

    ano_write_u16(&data[0], battery_voltage);
    ano_write_u16(&data[2], battery_current);
    ano_write_u16(&data[4], flight_voltage);
    ano_write_u16(&data[6], flight_current);
    return ano_pack_frame(ANO_SOURCE_ADDRESS, ANO_DESTINATION_ADDRESS,
                          0x0DU, data, sizeof(data), buffer, capacity);
}

