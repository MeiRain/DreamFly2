/**
 * @file rtos.h
 * @brief FreeRTOS task entry points for the flight controller.
 * @author 梅雨
 * @date 2026-10-08
 */

#ifndef DREAMFLY2_APPLICATION_RTOS_H
#define DREAMFLY2_APPLICATION_RTOS_H

/**
 * @brief Run the 10-tick system loop: startup RGB, then battery and status LED.
 * @param argument Unused CMSIS-RTOS task argument.
 * @note Starts during calibration for RGB indication. Battery sampling begins
 *       after startup, every 100 ms; the status LED toggles every 500 ms.
 */
void flight_system_task_entry(void *argument);

/**
 * @brief Run centralized flight initialization, then the rate PID bench loop every 2 ticks.
 * @param argument Unused CMSIS-RTOS task argument.
 * @note Never returns. Processes fresh IMU samples with a zero-rate target and
 *       integration disabled; records output for debugging without driving motors.
 */
void flight_control_task_entry(void *argument);

/**
 * @brief Wait for flight initialization, acquire data and solve six-/nine-axis attitude.
 * @param argument Unused CMSIS-RTOS task argument.
 * @note Never returns. IMU loop is 2 ticks; magnetic/pressure reads run every
 *       5 and 25 loops respectively. flight_init() completes before acquisition.
 */
void flight_sensor_task_entry(void *argument);

/**
 * @brief Service UART2 parameters every 20 ticks and send periodic attitude/debug data.
 * @param argument Unused CMSIS-RTOS task argument.
 * @note Task entry; never returns. UART1 prints one rate/motor-preview line
 *       every 20 ticks; UART2 retains ANO attitude and parameter communication.
 */
void flight_debug_task_entry(void *argument);

/**
 * @brief Service SI24R1 RX/ACK on PB2 IRQ notifications or a 5-tick timeout.
 * @param argument Unused CMSIS-RTOS task argument.
 * @note Radio owner after flight_init() completes. Valid controls are copied for debugging; 300 ms without
 *       a new control means lost input. ACK status carries six-axis attitude,
 *       battery voltage, onboard sensor status and accepted RC throttle.
 *       Attitude is unavailable after 100 ms without a solution; battery after
 *       500 ms without a system sample. Height reports fresh relative barometric
 *       height after ground-reference collection; mode control is not connected. MAG/BARO report initialization only.
 *       Arming is owned by the control task; this task reports its snapshot.
 *       GPIO callback only sets a thread flag; all SPI work stays in this task.
 *       Initialization/transport failure requires restarting this bench test.
 */
void flight_link_task_entry(void *argument);

#endif /* DREAMFLY2_APPLICATION_RTOS_H */
