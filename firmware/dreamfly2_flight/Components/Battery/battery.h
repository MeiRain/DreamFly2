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
 * @note ADC1 must be initialized first. Uses polling with a 1 ms timeout argument;
 *       ADC status is not reported to the caller. Call from one task only because
 *       the moving-average filter keeps one shared window.
 */
float battery_read_voltage(void);

#endif /* DREAMFLY2_COMPONENTS_BATTERY_H */
