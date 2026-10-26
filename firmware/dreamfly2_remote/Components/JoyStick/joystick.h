/**
 * @file joystick.h
 * @brief Four self-centering joystick channels using ADC1 and one-shot DMA.
 * @author 梅雨
 * @date 2026-10-24
 */

#ifndef DREAMFLY2_REMOTE_COMPONENTS_JOYSTICK_H
#define DREAMFLY2_REMOTE_COMPONENTS_JOYSTICK_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint16_t roll;     /**< RC value: 1000..2000, neutral 1500. */
    uint16_t pitch;    /**< RC value: 1000..2000, neutral 1500. */
    uint16_t yaw;      /**< RC value: 1000..2000, neutral 1500. */
    uint16_t throttle; /**< Self-centering RC value: 1000..2000, neutral 1500. */
} joystick_t;

typedef struct
{
    uint16_t roll_center_adc;     /**< Mechanical center, in raw ADC counts. */
    uint16_t pitch_center_adc;
    uint16_t yaw_center_adc;
    uint16_t throttle_center_adc;
} joystick_calibration_t;

typedef enum
{
    JOYSTICK_CALIBRATION_BUSY = 0,
    JOYSTICK_CALIBRATION_DONE,
    JOYSTICK_CALIBRATION_FAILED
} joystick_calibration_result_t;

/**
 * @brief Calibrate the ADC peripheral and start the first five-channel DMA scan.
 * @return true on success, false on invalid configuration or a HAL error.
 * @note Call after MX_DMA_Init() and MX_ADC1_Init(). This does NOT calibrate
 *       mechanical stick centers. Existing RAM calibration is preserved;
 *       without loaded calibration all four centers default to 2048.
 *       CubeMX: ranks IN0/IN1/IN2/IN8/IN9, scan enabled, continuous disabled;
 *       ADC1 DMA normal, peripheral-to-memory, halfword/halfword, memory increment.
 *       Enable DMA1 Channel1 IRQ. The driver owns ADC1 and its complete callback.
 */
bool joystick_init(void);

/**
 * @brief Filter and map one completed scan, then start the next scan.
 * @param[out] channels RC values; unchanged if the function returns false.
 * @return true for new data, false if no scan is ready or acquisition fails.
 * @note Call from one task every 5..10 ms. Each axis has an independent 10-point
 *       moving mean; its steady-state delay is about 4.5 read periods.
 *       Processing: raw ADC -> mean -> center-calibrated mapping -> center
 *       deadband (+/-10 RC units, output 1500) -> 1000..2000 clamp.
 *       Larger ADC values produce larger RC values. Nominal endpoints remain
 *       0/4095; measured endpoint calibration is not implemented.
 *       Rank 1 is battery and is not converted by this joystick interface.
 *       Throttle meaning, arming, UI and communication belong to the application.
 */
bool joystick_read(joystick_t *channels);

/**
 * @brief Start/advance mechanical-center calibration by at most one fresh scan.
 * @return BUSY while collecting, DONE after accepting centers, FAILED otherwise.
 * @note First call starts an attempt; repeat every 5 ms until DONE or FAILED,
 *       then stop calling. A later call starts a new attempt. Normally 60 samples
 *       take about 0.3 s; acquisition timeout is 1 s. No sampling loop or delay.
 *       Call only while DISARMED, with all four axes centered (throttle manually).
 *       Do not also call joystick_read() during calibration; ADC ownership stays
 *       in this same task. Failed attempts leave existing centers unchanged.
 *       Uses raw ADC means, not previously mapped or filtered RC values.
 *       No flash access is performed. Caller handles UI and persistence.
 */
joystick_calibration_result_t joystick_calibrate(void);

/**
 * @brief Load center calibration, for example after reading application flash.
 * @param calibration Four ADC centers, each within 1024..3072.
 * @return true if accepted; false for NULL/invalid data, leaving old values intact.
 * @note Resets filter history on success; may be called before joystick_init().
 *       Caller validates the persisted record/version/checksum and ensures
 *       the aircraft is disarmed. Only center values are stored here.
 */
bool joystick_set_calibration(const joystick_calibration_t *calibration);

/**
 * @brief Copy the current ADC centers for application-owned persistence.
 * @param[out] calibration Output record; NULL is ignored.
 * @note Copies RAM only, without reading or writing flash.
 */
void joystick_get_calibration(joystick_calibration_t *calibration);

#endif /* DREAMFLY2_REMOTE_COMPONENTS_JOYSTICK_H */
