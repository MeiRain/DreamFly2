/**
 * @file rtos.h
 * @brief FreeRTOS task entry points for the flight controller.
 * @author 梅雨
 * @date 2026-10-08
 */

#ifndef DREAMFLY2_APPLICATION_RTOS_H
#define DREAMFLY2_APPLICATION_RTOS_H

/** @brief Run low-rate system monitoring and status indication. */
void flight_system_task_entry(void *argument);

/** @brief Run the high-rate flight-control loop. */
void flight_control_task_entry(void *argument);

/** @brief Acquire and process flight-sensor data. */
void flight_sensor_task_entry(void *argument);

/** @brief Send low-priority flight debug data. */
void flight_debug_task_entry(void *argument);

/** @brief Process the remote-control and telemetry link. */
void flight_link_task_entry(void *argument);

#endif /* DREAMFLY2_APPLICATION_RTOS_H */
