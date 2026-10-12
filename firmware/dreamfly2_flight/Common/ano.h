/**
 * @file ano.h
 * @brief Transport-independent ANO v8 frame packing.
 * @author 梅雨
 * @date 2026-10-12
 */
#ifndef DREAMFLY2_COMMON_ANO_H
#define DREAMFLY2_COMMON_ANO_H

#include <stdint.h>

/*
 * All values must already be encoded in the protocol's units and scale.
 * Caller owns the output buffer; payload and output must not overlap.
 * Returns complete frame length, or zero for invalid arguments/insufficient space.
 */
uint16_t ano_pack_frame(uint8_t source, uint8_t destination, uint8_t id,
                        const uint8_t *data, uint16_t length,
                        uint8_t *buffer, uint16_t capacity);
uint16_t ano_pack_imu(const int16_t acc[3], const int16_t gyro[3], uint8_t status,
                      uint8_t *buffer, uint16_t capacity);
uint16_t ano_pack_mag(const int16_t mag[3], uint8_t status,
                      uint8_t *buffer, uint16_t capacity);
uint16_t ano_pack_attitude(int16_t roll, int16_t pitch, int16_t yaw, uint8_t status,
                           uint8_t *buffer, uint16_t capacity);
uint16_t ano_pack_battery(uint16_t battery_voltage, uint16_t battery_current,
                          uint16_t flight_voltage, uint16_t flight_current,
                          uint8_t *buffer, uint16_t capacity);

#endif /* DREAMFLY2_COMMON_ANO_H */

