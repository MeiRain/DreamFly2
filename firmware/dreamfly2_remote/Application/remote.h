/**
 * @file remote.h
 * @brief Remote-controller acquisition, radio link and calibration coordination.
 * @author 梅雨
 * @date 2026-10-26
 */

#ifndef DREAMFLY2_REMOTE_APPLICATION_REMOTE_H
#define DREAMFLY2_REMOTE_APPLICATION_REMOTE_H

#include <stdbool.h>
#include <stdint.h>

#include "joystick.h"
#include "rcprotocol.h"
#include "button.h"

typedef enum
{
    REMOTE_PAGE_FLIGHT = 0, /**< Startup/main page: aircraft information. */
    REMOTE_PAGE_INPUT       /**< Live joystick channels and local calibration. */
} remote_page_t;

typedef enum
{
    REMOTE_NOTICE_NONE = 0,
    REMOTE_NOTICE_WAIT_LINK,
    REMOTE_NOTICE_LOCK_FIRST,
    REMOTE_NOTICE_MODE_PENDING,
    REMOTE_NOTICE_FLIGHT_PENDING,
    REMOTE_NOTICE_CAL_OK,
    REMOTE_NOTICE_CAL_FAILED,
    REMOTE_NOTICE_CAL_REJECTED
} remote_notice_t;

typedef enum
{
    REMOTE_ARM_UNKNOWN = 0, /**< No valid current flight status; never treat as disarmed. */
    REMOTE_ARM_DISARMED,
    REMOTE_ARM_ARMED
} remote_arm_state_t;

typedef enum
{
    REMOTE_CALIBRATION_IDLE = 0,
    REMOTE_CALIBRATION_PENDING,
    REMOTE_CALIBRATION_RUNNING,
    REMOTE_CALIBRATION_SUCCESS, /**< RAM centers changed; application may save them once. */
    REMOTE_CALIBRATION_FAILED,
    REMOTE_CALIBRATION_REJECTED /**< No fresh disarmed status, invalid input or radio fault. */
} remote_calibration_state_t;

typedef struct
{
    bool radio_ready;       /**< Radio initialized without a transport error. */
    bool joystick_ready;    /**< Joystick initialization succeeded; also check input_valid. */
    bool input_valid;       /**< Last successful joystick read is less than 50 ms old. */
    bool is_connected;      /**< Valid, advancing flight status is less than 300 ms old. */
    remote_arm_state_t arm_state;
    remote_calibration_state_t calibration;
    uint8_t link_quality_percent; /**< Hardware ACK successes among up to 10 completed sends;
                                      0 if no ACK for 300 ms. Not RSSI. */
    uint8_t keys;             /**< Caller-supplied held-key bits 0..5; not button event codes. */
    uint8_t last_tx_sequence; /**< Last successfully started control frame's sequence. */
    joystick_t joystick;     /**< Valid only while input_valid is true. */
    rcprotocol_status_t flight; /**< Cached protocol units; valid only while is_connected. */
    remote_page_t page;       /**< Local UI state, not sent to the flight controller. */
    remote_notice_t notice;   /**< Temporary feedback for rejected/unimplemented actions. */
} remote_state_t;

/**
 * @brief Initialize joystick acquisition and the NRF24L01 in fixed TX mode.
 * @return true if both initialized; inspect remote_get_state() for individual results.
 * @note Call after CubeMX GPIO/DMA/ADC1/SPI1 initialization. Radio startup blocks
 *       for about 102 ms. Creates no tasks and makes no GPIO/OLED configuration changes.
 *       Load saved joystick centers through joystick_set_calibration() if needed.
 */
bool remote_init(void);

/**
 * @brief Poll input, completed TX/ACK data, status timeouts and pending calibration.
 * @note Call every 5 ms from one task. Controls are sent at most every 10 ms,
 *       only with input less than 50 ms old. No application busy-wait for ACKs;
 *       individual SPI operations still use bounded blocking HAL transfers.
 *       Calibration advances by at most one scan per call, normally about 0.3 s
 *       total with a 1 s timeout. It pauses control-frame transmission but keeps
 *       radio polling and UI task scheduling active. It starts only with fresh
 *       disarmed feedback and no pending TX.
 *       A radio transport fault disables transmission until remote_init().
 *       All remote/joystick/radio APIs must be serialized in this same context.
 */
void remote_process(void);

/**
 * @brief Handle one button scan, page changes and a manual calibration request.
 * @param buttons Current scan result; NULL is ignored.
 * @note Call every 5 ms from the same task as remote_process(), after scanning.
 *       Short SW2 (KEY1/right) switches the two pages. Long SW2 on the main page
 *       opens the joystick page; long SW2 there requests calibration after the
 *       user centers all four axes. SW4 (KEY2/left) returns to the main page.
 *       On the main page only, SW4 mode/takeoff/landing actions show placeholders.
 *       Calibration requires fresh DISARMED feedback and changes RAM only.
 *       Direction keys stay local on the joystick page and must be released
 *       before becoming eligible for transmission again after returning.
 */
void remote_handle_buttons(const button_result_t *buttons);

/**
 * @brief Draw one copied UI snapshot into the OLED buffer.
 * @param state Snapshot from remote_get_state(); NULL is ignored.
 * @note Display task only, after oled_init(). Does not read/change live remote
 *       state or refresh the panel; call oled_update() afterward. The main page
 *       has a signal/battery/mode header. Telemetry not yet supplied by
 *       the flight application is shown as dashes, not as valid zero readings.
 */
void remote_draw_page(const remote_state_t *state);

/**
 * @brief Supply the six currently held, debounced key bits for transmission.
 * @return false if reserved bits 6 or 7 are set; previous keys remain unchanged.
 * @note Low-level override for non-UI callers. With the built-in menu, use
 *       remote_handle_buttons() instead; it filters local/menu keys each scan.
 *       Never pass short_pressed or long_pressed as held-key state.
 */
bool remote_set_keys(uint8_t keys);

/**
 * @brief Queue a manual four-axis center-calibration request.
 * @return true if queued; false if safety prerequisites are not met or busy.
 * @note Call only in response to explicit user confirmation with sticks centered.
 *       The next remote_process() rechecks fresh DISARMED feedback before running.
 *       No flash write is performed. After a SUCCESS transition, the application
 *       may export joystick_get_calibration() and save the record once.
 */
bool remote_request_calibration(void);

/**
 * @brief Copy a current snapshot, updating freshness-derived flags; NULL is ignored.
 * @note is_connected requires valid protocol status, not just hardware ACKs.
 *       The flight peer must return rcprotocol status in ACK payloads and advance
 *       status.sequence for every new status record. Duplicates do not renew age.
 *       This module does not implement pairing, arming gestures or failsafe motors.
 */
void remote_get_state(remote_state_t *state);

#endif /* DREAMFLY2_REMOTE_APPLICATION_REMOTE_H */
