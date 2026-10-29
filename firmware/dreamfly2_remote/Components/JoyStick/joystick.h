/**
 * @file joystick.h
 * @brief Three centered axes and a non-centering throttle using ADC1 one-shot DMA.
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
    uint16_t throttle; /**< Non-centering RC value: 1000 at bottom, 2000 at top. */
} joystick_t;

typedef struct
{
    uint16_t roll_center_adc;     /**< Mechanical center, in raw ADC counts. */
    uint16_t pitch_center_adc;
    uint16_t yaw_center_adc;
    uint16_t throttle_min_adc; /**< Measured bottom position, in raw ADC counts. */
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
 *       mechanical stick positions. Existing RAM calibration is preserved;
 *       defaults are 2048 for roll/pitch/yaw centers and 0 for throttle minimum.
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
 *       Processing: raw ADC -> mean -> calibrated mapping -> deadband -> clamp.
 *       Roll/pitch/yaw have a +/-10 RC center deadband (output 1500). Throttle
 *       has a 10-step bottom deadband (output 1000), with no middle deadband.
 *       Larger ADC values produce larger RC values. Attitude endpoints remain
 *       0/4095; throttle uses its measured minimum and a nominal maximum of 4095.
 *       Full travel endpoint calibration is not implemented.
 *       Rank 1 is battery; its cached raw value is available separately.
 *       Throttle meaning, arming, UI and communication belong to the application.
 */
bool joystick_read(joystick_t *channels);

/**
 * @brief Return the battery ADC count cached from the last consumed five-rank scan.
 * @return Raw 0..4095 value; 0 before a frame is consumed or after a driver fault.
 * @note Same owning task as joystick_read/calibrate. Does not start/stop ADC or
 *       consume a scan; voltage conversion, filtering and freshness belong to
 *       the application. Battery input is ADC rank 1 (PA0).
 */
uint16_t joystick_get_battery_adc(void);

/**
 * @brief Start/advance three-axis center and throttle-low calibration by one fresh scan.
 * @return BUSY while collecting, DONE after accepting calibration, FAILED otherwise.
 * @note First call starts an attempt; repeat every 5 ms until DONE or FAILED,
 *       then stop calling. A later call starts a new attempt. Normally 60 samples
 *       take about 0.3 s; acquisition timeout is 1 s. No sampling loop or delay.
 *       Call only while DISARMED, with roll/pitch/yaw centered and throttle fully down.
 *       Do not also call joystick_read() during calibration; ADC ownership stays
 *       in this same task. Failed attempts leave existing calibration unchanged.
 *       Uses raw ADC means, not previously mapped or filtered RC values.
 *       No flash access is performed. Caller handles UI and persistence.
 */
joystick_calibration_result_t joystick_calibrate(void);

/**
 * @brief Load three-axis centers and throttle minimum from application-owned storage.
 * @param calibration Roll/pitch/yaw centers in 1024..3072; throttle minimum in 0..1024.
 * @return true if accepted; false for NULL/invalid data, leaving old values intact.
 * @note Resets filter history on success; may be called before joystick_init().
 *       Caller validates the persisted record/version/checksum and ensures
 *       the aircraft is disarmed. Old four-center records are incompatible; recalibrate.
 */
bool joystick_set_calibration(const joystick_calibration_t *calibration);

/**
 * @brief Copy the current three centers and throttle minimum for application persistence.
 * @param[out] calibration Output record; NULL is ignored.
 * @note Copies RAM only, without reading or writing flash.
 */
void joystick_get_calibration(joystick_calibration_t *calibration);

#endif /* DREAMFLY2_REMOTE_COMPONENTS_JOYSTICK_H */
