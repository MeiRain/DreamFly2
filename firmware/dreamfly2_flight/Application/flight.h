/**
 * @file flight.h
 * @brief Public interface for flight-state management.
 * @author 梅雨
 * @date 2026-10-08
 */

#ifndef DREAMFLY2_APPLICATION_FLIGHT_H
#define DREAMFLY2_APPLICATION_FLIGHT_H

/**
 * @brief Read and update the current battery voltage.
 * @return Latest battery voltage in volts.
 */
float flight_update_battery(void);

#endif /* DREAMFLY2_APPLICATION_FLIGHT_H */
