/**
 * @file rtos.h
 * @brief RTOS task entries for remote input, radio and local UI.
 * @author 梅雨
 * @date 2026-10-26
 */

#ifndef DREAMFLY2_REMOTE_APPLICATION_RTOS_H
#define DREAMFLY2_REMOTE_APPLICATION_RTOS_H

/**
 * @brief Initialize buttons and remote processing; scan/service them every 5 ms.
 * @param argument Unused CubeMX task argument.
 * @note Strong replacement for the generated weak entry. Requires a 1 kHz tick,
 *       Normal priority and at least the configured 256-word starting stack.
 *       Owns remote_init/process and therefore joystick/radio access. Do not
 *       add another joystick reader or radio caller. Handles local key/menu
 *       actions and sends controls every 10 ms. Calibration collects at most
 *       one sample per loop, only after explicit DISARMED user confirmation.
 *       Arming, mode switching and automatic flight are not connected yet.
 */
void remote_control_task_entry(void *argument);

/**
 * @brief Initialize OLED and draw the copied remote page every 100 ms.
 * @param argument Unused CubeMX task argument.
 * @note Strong replacement for the generated weak entry. Keep BelowNormal
 *       priority and the configured 512-word starting stack. This task owns
 *       OLED access and uses queue copies only, not the joystick driver.
 *       LINK requires fresh flight ACK status; ACK is hardware send success rate.
 *       UI/business rendering is in remote_draw_page(); this task owns the OLED
 *       lifecycle and never reads live remote state or calls the joystick driver.
 *       Stack/heap margins still require measurement on the target.
 */
void remote_display_task_entry(void *argument);

#endif /* DREAMFLY2_REMOTE_APPLICATION_RTOS_H */
