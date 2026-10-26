/**
 * @file anoprotocol.h
 * @brief Transport-independent ANO v8 frame packing.
 * @author 梅雨
 * @date 2026-10-12
 */
#ifndef DREAMFLY2_COMMON_ANOPROTOCOL_H
#define DREAMFLY2_COMMON_ANOPROTOCOL_H

#include <stdint.h>

/*
 * All values must already be encoded in the protocol's units and scale.
 * Caller owns the output buffer; payload and output must not overlap.
 * Returns complete frame length, or zero for invalid arguments/insufficient space.
 * Standard packers use source 0x66 and destination 0xFE.
 * Packing performs no physical-unit conversion or numeric saturation.
 * For every packer, buffer is the output and capacity is its size in bytes.
 * Array arguments use XYZ order unless a different order is stated below.
 */
/**
 * @brief Pack an ANO V8 frame, including length and both checksums.
 * @param source Source device address.
 * @param destination Destination device address.
 * @param id Frame type ID.
 * @param data Encoded payload; NULL is allowed only when length is zero.
 * @param length Payload length in bytes, excluding the eight frame overhead bytes.
 * @param[out] buffer Complete frame buffer; must not overlap data.
 * @param capacity Output buffer size in bytes.
 * @return Complete frame length, or 0 for invalid pointers or insufficient capacity.
 */
uint16_t anoprotocol_pack_frame(uint8_t source, uint8_t destination, uint8_t id,
                        const uint8_t *data, uint16_t length,
                        uint8_t *buffer, uint16_t capacity);
/**
 * @brief Pack 0x01: ACC in cm/s^2; GYR = degrees/s * 32768/2000; vibration status.
 * @note acc and gyro each contain three encoded XYZ values; NULL returns 0.
 */
uint16_t anoprotocol_pack_imu(const int16_t acc[3], const int16_t gyro[3], uint8_t status,
                      uint8_t *buffer, uint16_t capacity);
/**
 * @brief Pack 0x02: Magnetic XYZ, temperature in degrees C * 10, and status.
 * @note V8.10 does not specify magnetic units; caller chooses the receiver's scale.
 *       mag contains three encoded XYZ values; NULL returns 0.
 */
uint16_t anoprotocol_pack_mag(const int16_t mag[3], int16_t temperature_decic,
                      uint8_t status,
                      uint8_t *buffer, uint16_t capacity);
/** @brief Pack 0x03: Roll/Pitch/Yaw in degrees * 100, followed by fusion status. */
uint16_t anoprotocol_pack_attitude(int16_t roll, int16_t pitch, int16_t yaw, uint8_t status,
                           uint8_t *buffer, uint16_t capacity);
/**
 * @brief 0x0D: Battery voltage/current, flight voltage/current and power-state bits.
 * @note V8.10 battery-voltage table says V * 1000, prose says V * 100:
 *       confirm with the upper computer. Other voltage/current fields use V/A * 100.
 *       This function accepts encoded integers and does not choose the scale.
 */
uint16_t anoprotocol_pack_battery(uint16_t battery_voltage, uint16_t battery_current,
                          uint16_t flight_voltage, uint16_t flight_current,
                          uint16_t power_state,
                          uint8_t *buffer, uint16_t capacity);


/** @brief Pack 0x05: Barometric, auxiliary and fused ground heights in cm, then status. */
uint16_t anoprotocol_pack_height(int32_t baro_cm, int32_t range_cm, int32_t fused_cm, uint8_t status,
                         uint8_t *buffer, uint16_t capacity);

/** @brief Pack 0x07: Fused XYZ velocity in cm/s; NULL input returns 0. */
uint16_t anoprotocol_pack_velocity(const int16_t velocity_cms[3],
                         uint8_t *buffer, uint16_t capacity);

/** @brief Pack 0x08: Fused XYZ offset from takeoff position in cm; NULL input returns 0. */
uint16_t anoprotocol_pack_position(const int32_t position_cm[3],
                         uint8_t *buffer, uint16_t capacity);

/** @brief Pack 0x0A: Target Roll/Pitch/Yaw in degrees * 100. */
uint16_t anoprotocol_pack_target_attitude(int16_t roll, int16_t pitch, int16_t yaw,
                         uint8_t *buffer, uint16_t capacity);

/**
 * @brief Pack 0x20: Four motor outputs, 0..10000 = 0..100% throttle, not timer CCR.
 * @note pwm contains channels 1..4 in order; NULL returns 0.
 */
uint16_t anoprotocol_pack_motor(const uint16_t pwm[4],
                         uint8_t *buffer, uint16_t capacity);

/** @brief Pack 0x21: Roll/Pitch/Throttle/Yaw; axes +/-5000, throttle 0..10000. */
uint16_t anoprotocol_pack_control(int16_t roll, int16_t pitch, int16_t throttle, int16_t yaw,
                         uint8_t *buffer, uint16_t capacity);

/** @brief Pack 0x31: Module type, raw pixel displacement DX/DY, quality (0 = invalid). */
uint16_t anoprotocol_pack_flow(uint8_t type, int16_t dx, int16_t dy, uint8_t quality,
                         uint8_t *buffer, uint16_t capacity);

/**
 * @brief Pack 0x34: Single range in mm; 0/6 down, 1 front, 2 rear, 3 left, 4 right, 5 up.
 * @note Directions above 6 return 0.
 */
uint16_t anoprotocol_pack_range(uint8_t direction, int32_t distance_mm,
                         uint8_t *buffer, uint16_t capacity);

/**
 * @brief Pack 0x40: Roll/Pitch/Throttle/Yaw/AUX1..6, in encoded protocol RC units.
 * @note channels contains ten values; NULL returns 0. No channel scaling is performed.
 */
uint16_t anoprotocol_pack_remote(const uint16_t channels[10],
                         uint8_t *buffer, uint16_t capacity);

#endif /* DREAMFLY2_COMMON_ANOPROTOCOL_H */

