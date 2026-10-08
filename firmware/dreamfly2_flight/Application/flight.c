/**
 * @file flight.c
 * @brief Flight-state management implementation.
 * @author 梅雨
 * @date 2026-10-08
 */

#include "flight.h"

#include "battery.h"

static float s_battery_voltage_v;

float flight_update_battery(void)
{
    s_battery_voltage_v = battery_read_voltage();

    return s_battery_voltage_v;
}
