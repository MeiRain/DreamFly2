/**
 * @file remote.c
 * @brief Remote ACK link, local pages and manual calibration coordination.
 * @author 梅雨
 * @date 2026-10-26
 */

#include "remote.h"

#include <stddef.h>
#include <string.h>

#include "nrf24l01.h"
#include "filter.h"
#include "oled.h"
#include "stm32f1xx_hal.h"

#define REMOTE_TX_PERIOD_MS       10U
#define REMOTE_INPUT_TIMEOUT_MS   50U
#define REMOTE_LINK_TIMEOUT_MS    300U
#define REMOTE_LINK_HISTORY_SIZE  10U
#define REMOTE_KEY_MASK           0x3FU
#define REMOTE_DIRECTION_KEYS     (KEY3 | KEY4 | KEY5 | KEY6)
#define REMOTE_NOTICE_DURATION_MS 1500U
#define REMOTE_BATTERY_TIMEOUT_MS 500U
#define REMOTE_ADC_REFERENCE_V    3.3f /* Nominal supply; calibrate against a meter later. */
#define REMOTE_ADC_FULL_SCALE     4095.0f
#define REMOTE_BATTERY_DIVIDER_GAIN 2.0f /* PA0 divider: 100 kohm / 100 kohm. */
#define REMOTE_BATTERY_HYSTERESIS_CV 3U /* 0.03 V margin when changing icon levels. */

/* Trial voltage bins for 1..4 icon segments, in 0.01 V; not measured SOC or cutoffs.
 * Keep the boards separate so their under-load thresholds can be tuned later.
 */
static const uint16_t s_flight_battery_levels_cv[4] = {350U, 370U, 385U, 400U};
static const uint16_t s_remote_battery_levels_cv[4] = {350U, 370U, 385U, 400U};
/* Display-task-only hysteresis state; UINT8_MAX means no preceding valid sample. */
static uint8_t s_flight_battery_bars = UINT8_MAX;
static uint8_t s_remote_battery_bars = UINT8_MAX;

static remote_state_t s_state;
static bool s_initialized;
static bool s_have_input;
static bool s_have_status;
static bool s_have_ack;
static bool s_tx_pending;
static uint8_t s_next_sequence;
static uint32_t s_last_input_ms;
static uint32_t s_last_status_ms;
static uint32_t s_last_ack_ms;
static uint32_t s_last_send_ms;
static uint32_t s_last_battery_ms;
static filter_moving_average_t s_battery_filter;
static uint8_t s_tx_history[REMOTE_LINK_HISTORY_SIZE];
static uint8_t s_history_index;
static uint8_t s_history_count;
static uint8_t s_history_successes;
static uint8_t s_blocked_keys; /**< Keys held on the joystick page stay local until released. */
static uint32_t s_notice_start_ms;

/** @brief Publish a temporary local message without allocating a text buffer. */
static void remote_show_notice(remote_notice_t notice)
{
    s_state.notice = notice;
    s_notice_start_ms = HAL_GetTick();
}

/** @brief Refresh derived status; missing or stale feedback means UNKNOWN. */
static void remote_refresh_state(uint32_t now)
{
    if ((uint32_t)(now - s_last_battery_ms) >= REMOTE_BATTERY_TIMEOUT_MS)
    {
        s_state.battery_cv = 0U;
    }
    if ((uint32_t)(now - s_notice_start_ms) >= REMOTE_NOTICE_DURATION_MS)
    {
        s_state.notice = REMOTE_NOTICE_NONE;
    }
    s_state.input_valid = s_state.joystick_ready && s_have_input
        && (uint32_t)(now - s_last_input_ms) < REMOTE_INPUT_TIMEOUT_MS;

    s_state.is_connected = s_state.radio_ready && s_have_status
        && (uint32_t)(now - s_last_status_ms) < REMOTE_LINK_TIMEOUT_MS;

    s_state.arm_state = REMOTE_ARM_UNKNOWN;
    if (s_state.is_connected)
    {
        s_state.arm_state = (s_state.flight.flags & RCPROTOCOL_STATUS_ARMED) != 0U
            ? REMOTE_ARM_ARMED : REMOTE_ARM_DISARMED;
    }

    s_state.link_quality_percent = 0U;
    if (s_have_ack && s_history_count != 0U
        && (uint32_t)(now - s_last_ack_ms) < REMOTE_LINK_TIMEOUT_MS)
    {
        s_state.link_quality_percent = (uint8_t)(100U * s_history_successes / s_history_count);
    }
}

/** @brief Record one completed send, not each hardware retransmission attempt. */
static void remote_record_tx(bool success, uint32_t now)
{
    s_history_successes -= s_tx_history[s_history_index];
    s_tx_history[s_history_index] = success ? 1U : 0U;
    s_history_successes += s_tx_history[s_history_index];
    s_history_index = (uint8_t)((s_history_index + 1U) % REMOTE_LINK_HISTORY_SIZE);
    if (s_history_count < REMOTE_LINK_HISTORY_SIZE)
    {
        ++s_history_count;
    }

    if (success)
    {
        s_have_ack = true;
        s_last_ack_ms = now;
    }
}

/** @brief Invalidate application feedback immediately on a radio transport error. */
static void remote_radio_fault(void)
{
    s_state.radio_ready = false;
    s_have_status = false;
    s_have_ack = false;
    s_tx_pending = false;
}

void remote_process_radio(void)
{
    if (!s_state.radio_ready)
    {
        return;
    }
    uint32_t now = HAL_GetTick();
    nrf24l01_tx_result_t result = nrf24l01_get_tx_result();
    if (result == NRF24L01_TX_ERROR)
    {
        remote_radio_fault();
        return;
    }

    s_tx_pending = result == NRF24L01_TX_PENDING;
    if (result == NRF24L01_TX_SUCCESS || result == NRF24L01_TX_FAILED)
    {
        remote_record_tx(result == NRF24L01_TX_SUCCESS, now);
    }

    if (s_tx_pending)
    {
        return;
    }

    for (uint8_t i = 0U; i < 3U; i++)
    {
        uint8_t buffer[NRF24L01_MAX_PAYLOAD_SIZE];
        uint8_t size;
        rcprotocol_status_t status;

        if (!nrf24l01_receive(buffer, sizeof(buffer), &size))
        {
            break;
        }
        if (!rcprotocol_parse_status(buffer, size, &status)
            || (status.flags & 0xF0U) != 0U
            || status.throttle < 1000U || status.throttle > 2000U)
        {
            continue;
        }

        /* A repeatedly queued old ACK payload must not keep DISARMED alive. */
        if (s_have_status && status.sequence == s_state.flight.sequence)
        {
            /* Even a duplicate may revoke DISARMED, but never renew its age. */
            if ((status.flags & RCPROTOCOL_STATUS_ARMED) != 0U)
            {
                s_state.flight.flags |= RCPROTOCOL_STATUS_ARMED;
            }
            continue;
        }

        s_state.flight = status;
        s_have_status = true;
        s_last_status_ms = now;
    }

    /* receive() returns false for both an empty FIFO and a transport error.
     * With no TX pending this checks the driver's fault latch without bus I/O.
     */
    if (nrf24l01_get_tx_result() == NRF24L01_TX_ERROR)
    {
        remote_radio_fault();
    }
}

/** @brief Check the same calibration prerequisites at request and execution time. */
static bool remote_can_calibrate(void)
{
    return s_state.radio_ready && s_state.input_valid && s_state.is_connected
        && s_state.arm_state == REMOTE_ARM_DISARMED;
}

/** @brief Build a fresh control frame; never substitute defaults for stale input. */
static void remote_send_control(uint32_t now)
{
    uint8_t buffer[RCPROTOCOL_CONTROL_FRAME_SIZE];
    rcprotocol_control_t control = {
        .sequence = s_next_sequence,
        .roll = s_state.joystick.roll,
        .pitch = s_state.joystick.pitch,
        .yaw = s_state.joystick.yaw,
        .throttle = s_state.joystick.throttle,
        .keys = s_state.keys
    };
    uint8_t size = rcprotocol_pack_control(&control, buffer, sizeof(buffer));
    if (size == 0U)
    {
        return;
    }

    if (!nrf24l01_send(buffer, size))
    {
        remote_radio_fault();
        return;
    }

    s_tx_pending = true;
    s_state.last_tx_sequence = s_next_sequence++;
    s_last_send_ms = now;
}

bool remote_init(void)
{
    memset(&s_state, 0, sizeof(s_state));
    memset(s_tx_history, 0, sizeof(s_tx_history));
    filter_moving_average_init(&s_battery_filter);
    s_last_battery_ms = 0U;
    s_initialized = false;
    s_have_input = false;
    s_have_status = false;
    s_have_ack = false;
    s_tx_pending = false;
    s_next_sequence = 0U;
    s_history_index = 0U;
    s_history_count = 0U;
    s_history_successes = 0U;
    s_blocked_keys = 0U;

    s_state.joystick = (joystick_t){1500U, 1500U, 1500U, 1500U};
    s_state.radio_ready = nrf24l01_init(NRF24L01_MODE_TX);
    s_state.joystick_ready = joystick_init();
    s_last_send_ms = HAL_GetTick();
    s_initialized = true;
    remote_refresh_state(s_last_send_ms);

    return s_state.radio_ready && s_state.joystick_ready;
}

void remote_process(void)
{
    if (!s_initialized)
    {
        return;
    }

    /* Calibration owns ADC scans until it returns DONE/FAILED. */
    if (s_state.calibration != REMOTE_CALIBRATION_RUNNING
        && s_state.joystick_ready && joystick_read(&s_state.joystick))
    {
        s_have_input = true;
        s_last_input_ms = HAL_GetTick();
        float adc = filter_moving_average(&s_battery_filter, joystick_get_battery_adc());
        float voltage_v = adc * REMOTE_ADC_REFERENCE_V
            / REMOTE_ADC_FULL_SCALE * REMOTE_BATTERY_DIVIDER_GAIN;
        s_state.battery_cv = (uint16_t)(voltage_v * 100.0f + 0.5f);
        s_last_battery_ms = s_last_input_ms;
    }

    if (s_state.radio_ready)
    {
        remote_process_radio();
    }

    uint32_t now = HAL_GetTick();
    remote_refresh_state(now);

    if (s_state.calibration == REMOTE_CALIBRATION_PENDING)
    {
        if (!remote_can_calibrate())
        {
            s_state.calibration = REMOTE_CALIBRATION_REJECTED;
            remote_show_notice(REMOTE_NOTICE_CAL_REJECTED);
            return;
        }
        if (s_tx_pending)
        {
            return;
        }
        s_state.calibration = REMOTE_CALIBRATION_RUNNING;
        s_state.keys = 0U;
        s_have_input = false;
    }

    if (s_state.calibration == REMOTE_CALIBRATION_RUNNING)
    {
        joystick_calibration_result_t result = joystick_calibrate();
        if (result != JOYSTICK_CALIBRATION_BUSY)
        {
            bool success = result == JOYSTICK_CALIBRATION_DONE;
            s_state.calibration = success
                ? REMOTE_CALIBRATION_SUCCESS : REMOTE_CALIBRATION_FAILED;
            remote_show_notice(success ? REMOTE_NOTICE_CAL_OK : REMOTE_NOTICE_CAL_FAILED);
        }
        remote_refresh_state(HAL_GetTick());
        /* Return to the RTOS after one step so the OLED can refresh normally.
         * No stale control frame is sent while the ADC is being calibrated.
         */
        return;
    }

    if (s_state.radio_ready && s_state.input_valid && !s_tx_pending
        && (uint32_t)(now - s_last_send_ms) >= REMOTE_TX_PERIOD_MS)
    {
        remote_send_control(now);
        remote_refresh_state(HAL_GetTick());
    }
}

bool remote_set_keys(uint8_t keys)
{
    if (!s_initialized || (keys & (uint8_t)~REMOTE_KEY_MASK) != 0U)
    {
        return false;
    }

    s_state.keys = keys;
    return true;
}

bool remote_request_calibration(void)
{
    if (!s_initialized)
    {
        return false;
    }

    if (s_state.calibration == REMOTE_CALIBRATION_PENDING
        || s_state.calibration == REMOTE_CALIBRATION_RUNNING)
    {
        return false;
    }

    remote_refresh_state(HAL_GetTick());
    if (!remote_can_calibrate())
    {
        s_state.calibration = REMOTE_CALIBRATION_REJECTED;
        return false;
    }

    s_state.calibration = REMOTE_CALIBRATION_PENDING;
    return true;
}

void remote_handle_buttons(const button_result_t *buttons)
{
    if (!s_initialized || buttons == NULL)
    {
        return;
    }

    remote_refresh_state(HAL_GetTick());
    s_blocked_keys &= buttons->pressed;
    if (s_state.page == REMOTE_PAGE_INPUT)
    {
        s_blocked_keys |= buttons->pressed;
    }

    if (s_state.calibration == REMOTE_CALIBRATION_PENDING
        || s_state.calibration == REMOTE_CALIBRATION_RUNNING)
    {
        s_state.keys = 0U;
        return;
    }

    if ((buttons->short_pressed & KEY1) != 0U)
    {
        s_state.page = s_state.page == REMOTE_PAGE_FLIGHT
            ? REMOTE_PAGE_INPUT : REMOTE_PAGE_FLIGHT;
        s_state.notice = REMOTE_NOTICE_NONE;
    }
    else if ((buttons->short_pressed & KEY2) != 0U)
    {
        if (s_state.page == REMOTE_PAGE_INPUT)
        {
            s_state.page = REMOTE_PAGE_FLIGHT;
            s_state.notice = REMOTE_NOTICE_NONE;
        }
        else
        {
            remote_show_notice(REMOTE_NOTICE_MODE_PENDING);
        }
    }
    else if ((buttons->long_pressed & KEY1) != 0U)
    {
        if (s_state.page == REMOTE_PAGE_FLIGHT)
        {
            /* Viewing channels does not require a flight connection. */
            s_state.page = REMOTE_PAGE_INPUT;
            s_state.notice = REMOTE_NOTICE_NONE;
        }
        else
        {
            s_state.notice = REMOTE_NOTICE_NONE;
            if (!remote_request_calibration())
            {
                remote_show_notice(s_state.arm_state == REMOTE_ARM_UNKNOWN
                    ? REMOTE_NOTICE_WAIT_LINK : s_state.arm_state == REMOTE_ARM_ARMED
                    ? REMOTE_NOTICE_LOCK_FIRST : REMOTE_NOTICE_CAL_REJECTED);
            }
        }
    }
    else if ((buttons->long_pressed & KEY2) != 0U
             && s_state.page == REMOTE_PAGE_FLIGHT)
    {
        remote_show_notice(REMOTE_NOTICE_FLIGHT_PENDING);
    }

    if (s_state.page == REMOTE_PAGE_INPUT)
    {
        s_blocked_keys |= buttons->pressed;
        s_state.keys = 0U;
    }
    else
    {
        s_state.keys = (uint8_t)(buttons->pressed & REMOTE_DIRECTION_KEYS
                                 & (uint8_t)~s_blocked_keys);
    }
}

/** @brief Center a short ASCII label using the selected fixed-width font. */
static void remote_draw_centered(int16_t y, const char *text, uint8_t font_size)
{
    int16_t width = (int16_t)(strlen(text) * font_size);
    oled_show_string((OLED_WIDTH - width) / 2, y, text, font_size);
}

/** @brief Draw the original tower plus four bars, with the tallest at title height. */
static void remote_draw_signal(uint8_t quality_percent)
{
    /* Tower cropped from the existing signal bitmap; draw the bars separately. */
    static const uint8_t tower[20] = {
        0x1C, 0x3C, 0x64, 0xC4, 0x84, 0x84, 0xC4, 0x64, 0x3C, 0x1C,
        0x00, 0x00, 0x00, 0x00, 0x3F, 0x3F, 0x00, 0x00, 0x00, 0x00
    };
    oled_show_image(0, 0, 10, 16, tower);
    /* Original three bars: x=8/11/14, width=2, heights=3/6/9. Append one bar only. */
    const uint8_t heights[4] = {3U, 6U, 9U, 11U};
    uint8_t bars = quality_percent >= 90U ? 4U
        : quality_percent >= 60U ? 3U
        : quality_percent >= 30U ? 2U
        : quality_percent > 0U ? 1U : 0U;

    for (uint8_t i = 0U; i < 4U; i++)
    {
        int16_t x = 8 + i * 3;
        if (i < bars)
        {
            oled_draw_rectangle(x, 14 - heights[i], 2, heights[i], OLED_FILLED);
        }
        else
        {
            oled_draw_line(x, 13, x + 1, 13);
        }
    }
}

/** @brief Draw a rounded phone-style battery at the right edge, with four segments. */
static void remote_draw_battery(uint16_t voltage_cv, const uint16_t levels_cv[4], uint8_t *bars)
{
    const int16_t x = 102;
    oled_draw_line(x + 2, 2, x + 20, 2);
    oled_draw_line(x + 2, 13, x + 20, 13);
    oled_draw_line(x, 4, x, 11);
    oled_draw_line(x + 22, 4, x + 22, 11);
    oled_draw_point(x + 1, 3);
    oled_draw_point(x + 21, 3);
    oled_draw_point(x + 1, 12);
    oled_draw_point(x + 21, 12);
    oled_draw_rectangle(x + 23, 5, 3, 6, OLED_FILLED);

    if (voltage_cv == 0U || levels_cv[0] == 0U)
    {
        /* Unknown data is different from a valid low-voltage, empty icon. */
        *bars = UINT8_MAX;
        oled_show_char(x + 8, 4, '?', OLED_6X8);
        return;
    }
    if (*bars > 4U)
    {
        *bars = 0U;
        for (uint8_t i = 0U; i < 4U; i++)
        {
            if (voltage_cv >= levels_cv[i])
            {
                *bars = i + 1U;
            }
        }
    }
    else
    {
        /* Both loops are bounded by four segments. Ignore small boundary jitter. */
        while (*bars < 4U && voltage_cv >= levels_cv[*bars] + REMOTE_BATTERY_HYSTERESIS_CV)
        {
            (*bars)++;
        }
        while (*bars > 0U && voltage_cv + REMOTE_BATTERY_HYSTERESIS_CV < levels_cv[*bars - 1U])
        {
            (*bars)--;
        }
    }
    for (uint8_t i = 0U; i < 4U; i++)
    {
        oled_draw_rectangle(x + 2 + i * 5, 4, 4, 8,
                            i < *bars ? OLED_FILLED : OLED_UNFILLED);
    }
}

/** @brief Round centidegrees to signed whole degrees for display only. */
static void remote_draw_angle(int16_t x, int16_t y, const char *label,
                              int16_t angle_cdeg)
{
    oled_show_string(x, y, label, OLED_8X16);
    if (angle_cdeg == RCPROTOCOL_INVALID_I16)
    {
        oled_show_string(x + 16, y, " ---", OLED_8X16);
        return;
    }

    int32_t degrees = angle_cdeg;
    degrees = degrees >= 0 ? (degrees + 50) / 100 : -((-degrees + 50) / 100);
    oled_printf(x + 16, y, OLED_8X16, "%+04ld", (long)degrees);
}

/** @brief Draw a short central popup while leaving the header and footer visible. */
static void remote_draw_popup(const char *title, const char *detail, remote_page_t page)
{
    uint8_t height = page == REMOTE_PAGE_FLIGHT ? 32U : 38U;
    oled_clear_area(4, 16, 120, height);
    oled_draw_rectangle(4, 16, 120, height, OLED_UNFILLED);
    remote_draw_centered(page == REMOTE_PAGE_FLIGHT ? 18 : 20, title, OLED_8X16);
    remote_draw_centered(page == REMOTE_PAGE_FLIGHT ? 36 : 42, detail, OLED_6X8);
}

void remote_draw_page(const remote_state_t *state)
{
    if (state == NULL)
    {
        return;
    }

    oled_clear();
    remote_draw_centered(0, state->page == REMOTE_PAGE_FLIGHT ? "FLIGHT" : "REMOTE", OLED_8X16);
    remote_draw_signal(state->link_quality_percent);
    if (state->page == REMOTE_PAGE_FLIGHT)
    {
        remote_draw_battery(state->is_connected ? state->flight.battery_cv : 0U,
                            s_flight_battery_levels_cv, &s_flight_battery_bars);
    }
    else
    {
        remote_draw_battery(state->battery_cv, s_remote_battery_levels_cv, &s_remote_battery_bars);
    }

    switch (state->page)
    {
        case REMOTE_PAGE_INPUT:
        {
            /* LH/LV are left horizontal/vertical; RH/RV are right horizontal/vertical.
             * Retain the protocol's 1000..2000 values; do not rescale the controls.
             */
            if (state->input_valid)
            {
                oled_printf(0, 18, OLED_8X16, "LH:%4u", (unsigned)state->joystick.yaw);
                oled_printf(72, 18, OLED_8X16, "RH:%4u", (unsigned)state->joystick.roll);
                oled_printf(0, 36, OLED_8X16, "LV:%4u", (unsigned)state->joystick.throttle);
                oled_printf(72, 36, OLED_8X16, "RV:%4u", (unsigned)state->joystick.pitch);
            }
            else
            {
                oled_show_string(0, 18, "LH:----", OLED_8X16);
                oled_show_string(72, 18, "RH:----", OLED_8X16);
                oled_show_string(0, 36, "LV:----", OLED_8X16);
                oled_show_string(72, 36, "RV:----", OLED_8X16);
            }
            if (state->battery_cv != 0U)
            {
                oled_printf(0, 56, OLED_6X8, "%u.%02uV",
                            (unsigned)(state->battery_cv / 100U),
                            (unsigned)(state->battery_cv % 100U));
            }
            else
            {
                oled_show_string(0, 56, "-.--V", OLED_6X8);
            }
            const char *hint = !state->joystick_ready ? "ADC ERROR"
                : !state->input_valid ? "NO INPUT" : "SW2 HOLD:CAL";
            oled_show_string(OLED_WIDTH - (int16_t)(strlen(hint) * OLED_6X8),
                              56, hint, OLED_6X8);
            break;
        }

        case REMOTE_PAGE_FLIGHT:
            /* R/P share the left column; Y/T share the right column.
             * Keep flight-frame signs; ANO sign changes remain UART2-only.
             */
            if (state->is_connected
                && (state->flight.sensor_errors & RCPROTOCOL_SENSOR_ERROR_IMU) == 0U
                && state->flight.roll_cdeg != RCPROTOCOL_INVALID_I16
                && state->flight.pitch_cdeg != RCPROTOCOL_INVALID_I16
                && state->flight.yaw_cdeg != RCPROTOCOL_INVALID_I16)
            {
                remote_draw_angle(0, 16, "R:", state->flight.roll_cdeg);
                remote_draw_angle(0, 32, "P:", state->flight.pitch_cdeg);
                remote_draw_angle(80, 16, "Y:", state->flight.yaw_cdeg);
            }
            else
            {
                remote_draw_angle(0, 16, "R:", RCPROTOCOL_INVALID_I16);
                remote_draw_angle(0, 32, "P:", RCPROTOCOL_INVALID_I16);
                remote_draw_angle(80, 16, "Y:", RCPROTOCOL_INVALID_I16);
            }
            if (state->is_connected)
            {
                oled_printf(80, 32, OLED_8X16, "T:%4u", (unsigned)state->flight.throttle);
            }
            else
            {
                oled_show_string(80, 32, "T:----", OLED_8X16);
            }
            if (state->is_connected && state->flight.height_cm != RCPROTOCOL_INVALID_I16)
            {
                if (state->flight.height_cm >= -9999 && state->flight.height_cm <= 9999)
                {
                    oled_printf(0, 48, OLED_8X16, "H:%+05dcm", (int)state->flight.height_cm);
                }
                else
                {
                    oled_show_string(0, 48, "H: OVR cm", OLED_8X16);
                }
            }
            else
            {
                oled_show_string(0, 48, "H: ----cm", OLED_8X16);
            }
            if (state->is_connected && state->flight.sensor_errors != 0U)
            {
                oled_printf(96, 48, OLED_8X16, "E:%02X", (unsigned)state->flight.sensor_errors);
            }
            else
            {
                const char *mode = "----";
                if (state->is_connected)
                {
                    if ((state->flight.flags & RCPROTOCOL_STATUS_POS_HOLD) != 0U)
                    {
                        mode = "定点";
                    }
                    else if ((state->flight.flags & RCPROTOCOL_STATUS_ALT_HOLD) != 0U)
                    {
                        mode = "定高";
                    }
                    else
                    {
                        /* Basic-mode label only; not proof of control readiness or arming. */
                        mode = "自稳";
                    }
                }
                oled_show_string(96, 48, mode, OLED_8X16);
            }
            /* Sensor warnings temporarily take precedence over the mode field. */
            break;

        default:
            break;
    }

    if (state->calibration == REMOTE_CALIBRATION_PENDING
        || state->calibration == REMOTE_CALIBRATION_RUNNING)
    {
        remote_draw_popup("CALIBRATING", "KEEP AXES CENTERED", state->page);
        return;
    }

    switch (state->notice)
    {
        case REMOTE_NOTICE_WAIT_LINK:
            remote_draw_popup("NO LINK", "WAIT FOR FLIGHT", state->page);
            break;
        case REMOTE_NOTICE_LOCK_FIRST:
            remote_draw_popup("DISARM FIRST", "CAL UNAVAILABLE", state->page);
            break;
        case REMOTE_NOTICE_MODE_PENDING:
            remote_draw_popup("FLIGHT MODE", "NOT CONNECTED YET", state->page);
            break;
        case REMOTE_NOTICE_FLIGHT_PENDING:
            remote_draw_popup("TAKEOFF/LAND", "NOT CONNECTED YET", state->page);
            break;
        case REMOTE_NOTICE_CAL_OK:
            remote_draw_popup("CAL OK (RAM)", "FLASH NOT SAVED", state->page);
            break;
        case REMOTE_NOTICE_CAL_FAILED:
            remote_draw_popup("CAL FAILED", "CENTER AND RETRY", state->page);
            break;
        case REMOTE_NOTICE_CAL_REJECTED:
            remote_draw_popup("CAL REJECTED", "CHECK LINK/STATE", state->page);
            break;
        default:
            break;
    }
}

void remote_get_state(remote_state_t *state)
{
    if (state != NULL)
    {
        remote_refresh_state(HAL_GetTick());
        *state = s_state;
    }
}
