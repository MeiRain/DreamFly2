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

#endif /* DREAMFLY2_APPLICATION_RTOS_H */
