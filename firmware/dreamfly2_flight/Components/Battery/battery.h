/**
 * @file battery.h
 * @brief Public interface for battery voltage measurement.
 * @author 梅雨
 * @date 2026-10-08
 */

#ifndef DREAMFLY2_COMPONENTS_BATTERY_H
#define DREAMFLY2_COMPONENTS_BATTERY_H

/**
 * @brief Read, filter and convert the battery voltage.
 *
 * @return Filtered battery voltage in volts.
 * ADC1 must already be initialized by CubeMX before this function is called.
 */
float battery_read_voltage(void);

#endif /* DREAMFLY2_COMPONENTS_BATTERY_H */
