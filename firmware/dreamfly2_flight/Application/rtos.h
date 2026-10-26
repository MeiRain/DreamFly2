/**
 * @file rtos.h
 * @brief FreeRTOS task entry points for the flight controller.
 * @author 梅雨
 * @date 2026-10-08
 */

#ifndef DREAMFLY2_APPLICATION_RTOS_H
#define DREAMFLY2_APPLICATION_RTOS_H

/**
 * @brief Run the 10-tick system loop: battery every 100 ms, LED toggle every 500 ms.
 * @param argument Unused CMSIS-RTOS task argument.
 * @note Task entry; never returns. At 1 kHz kernel tick, the LED toggles every 500 ms.
 */
void flight_system_task_entry(void *argument);

/**
 * @brief Initialize flight state and run the rate PID bench loop every 2 ticks.
 * @param argument Unused CMSIS-RTOS task argument.
 * @note Never returns. Processes fresh IMU samples with a zero-rate target and
 *       integration disabled; records output for debugging without driving motors.
 */
void flight_control_task_entry(void *argument);

/**
 * @brief Initialize/calibrate sensors, acquire data and solve six-/nine-axis attitude.
 * @param argument Unused CMSIS-RTOS task argument.
 * @note Never returns. IMU loop is 2 ticks; magnetic/pressure reads run every
 *       five loops. Blocking startup calibration finishes before periodic acquisition.
 */
void flight_sensor_task_entry(void *argument);

/**
 * @brief Send USART2 attitude and USART1 debug text every 100 ticks.
 * @param argument Unused CMSIS-RTOS task argument.
 * @note Task entry; never returns. FLIGHT_DEBUG_REMOTE_ENABLED in rtos.c selects
 *       RC reception text (1) or the existing rotating sensor/PID pages (0).
 */
void flight_debug_task_entry(void *argument);

/**
 * @brief Initialize SI24R1 RX and service control/ACK frames every 5 ticks.
 * @param argument Unused CMSIS-RTOS task argument.
 * @note Sole radio owner. Valid controls are copied for debugging; 300 ms without
 *       a new control means lost input. ACK status carries six-axis attitude,
 *       battery voltage, onboard sensor status and accepted RC throttle.
 *       Attitude is unavailable after 100 ms without a solution; battery after
 *       500 ms without a system sample. Height remains explicitly unavailable;
 *       mode/arming control is not connected. MAG/BARO report initialization only.
 *       No received channel or key drives motors or changes arming state.
 *       Initialization/transport failure requires restarting this bench test.
 */
void flight_link_task_entry(void *argument);

#endif /* DREAMFLY2_APPLICATION_RTOS_H */
