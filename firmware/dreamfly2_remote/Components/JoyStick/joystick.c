/**
 * @file joystick.c
 * @brief Independent joystick acquisition, averaging and neutral/low calibration.
 * @author 梅雨
 * @date 2026-10-24
 */

#include "joystick.h"

#include <stddef.h>

#include "adc.h"
#include "filter.h"

#define JOYSTICK_ADC_COUNT                 5U
#define JOYSTICK_AXIS_COUNT                4U
#define JOYSTICK_ADC_MAX                   4095U
#define JOYSTICK_RC_DEADBAND               10
#define JOYSTICK_CALIBRATION_SAMPLES       60U
#define JOYSTICK_CALIBRATION_INTERVAL_MS   5U
#define JOYSTICK_CALIBRATION_TIMEOUT_MS    1000U
#define JOYSTICK_CENTER_MIN_ADC            1024U
#define JOYSTICK_CENTER_MAX_ADC            3072U
#define JOYSTICK_THROTTLE_MIN_MAX_ADC      1024U

/* DMA ranks: battery, roll, pitch, yaw, throttle.
 * HAL takes uint32_t*, but the configured transfer width is halfword.
 */
static _Alignas(uint32_t) volatile uint16_t s_adc_values[JOYSTICK_ADC_COUNT];
static volatile bool s_sample_ready;
static bool s_is_initialized;
static uint16_t s_battery_adc; /**< Battery rank copied with the last consumed DMA frame. */

/* Roll/pitch/yaw centers, then the non-centering throttle minimum. */
static uint16_t s_calibration_adc[JOYSTICK_AXIS_COUNT] = {2048U, 2048U, 2048U, 0U};
static filter_moving_average_t s_filters[JOYSTICK_AXIS_COUNT];

/* One bounded calibration attempt, advanced by the owning task's 5 ms calls. */
static struct
{
    uint32_t sum[JOYSTICK_AXIS_COUNT];
    uint32_t start_ms;
    uint32_t last_sample_ms;
    uint8_t count;
    bool active;
} s_calibration;

/** @brief Discard filter history after initialization or a calibration change. */
static void joystick_reset_filters(void)
{
    for (uint8_t i = 0U; i < JOYSTICK_AXIS_COUNT; i++)
    {
        filter_moving_average_init(&s_filters[i]);
    }
}

/** @brief Take one coherent raw frame and rearm normal-mode DMA. */
static bool joystick_read_adc(uint16_t values[JOYSTICK_AXIS_COUNT])
{
    if (!s_is_initialized)
    {
        return false;
    }

    if (HAL_ADC_GetError(&hadc1) != HAL_ADC_ERROR_NONE)
    {
        s_is_initialized = false;
        return false;
    }

    if (!s_sample_ready)
    {
        return false;
    }

    if (HAL_ADC_Stop_DMA(&hadc1) != HAL_OK)
    {
        s_is_initialized = false;
        return false;
    }

    s_battery_adc = s_adc_values[0];
    for (uint8_t i = 0U; i < JOYSTICK_AXIS_COUNT; i++)
    {
        values[i] = s_adc_values[i + 1U];
    }

    s_sample_ready = false;
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_adc_values,
                          JOYSTICK_ADC_COUNT) != HAL_OK)
    {
        s_is_initialized = false;
        return false;
    }

    return true;
}

/** @brief Map around a measured center, apply a continuous deadband, then clamp. */
static uint16_t joystick_convert_axis(uint16_t raw, uint16_t center)
{
    int32_t value;

    if (raw >= center)
    {
        value = 1500 + (int32_t)(raw - center) * 500 / (JOYSTICK_ADC_MAX - center);
    }
    else
    {
        value = 1500 - (int32_t)(center - raw) * 500 / center;
    }

    int32_t offset = value - 1500;
    if (offset > JOYSTICK_RC_DEADBAND)
    {
        value = 1500 + (offset - JOYSTICK_RC_DEADBAND) * 500
                       / (500 - JOYSTICK_RC_DEADBAND);
    }
    else if (offset < -JOYSTICK_RC_DEADBAND)
    {
        value = 1500 + (offset + JOYSTICK_RC_DEADBAND) * 500
                       / (500 - JOYSTICK_RC_DEADBAND);
    }
    else
    {
        value = 1500;
    }

    if (value < 1000) { value = 1000; }
    if (value > 2000) { value = 2000; }
    return (uint16_t)value;
}

/** @brief Map non-centering throttle from its measured minimum; no middle deadband. */
static uint16_t joystick_convert_throttle(uint16_t raw, uint16_t minimum)
{
    if (raw <= minimum)
    {
        return 1000U;
    }

    int32_t value = (int32_t)(raw - minimum) * 1000 / (JOYSTICK_ADC_MAX - minimum);
    if (value <= JOYSTICK_RC_DEADBAND)
    {
        return 1000U;
    }
    value = (value - JOYSTICK_RC_DEADBAND) * 1000 / (1000 - JOYSTICK_RC_DEADBAND);
    if (value > 1000) { value = 1000; }
    return (uint16_t)(1000 + value);
}

bool joystick_init(void)
{
    s_calibration.active = false;
    s_sample_ready = false;
    s_is_initialized = false;
    s_battery_adc = 0U;
    joystick_reset_filters();

    if ((hadc1.DMA_Handle == NULL)
        || (hadc1.DMA_Handle->Init.Mode != DMA_NORMAL)
        || (hadc1.DMA_Handle->Init.Direction != DMA_PERIPH_TO_MEMORY)
        || (hadc1.DMA_Handle->Init.PeriphDataAlignment != DMA_PDATAALIGN_HALFWORD)
        || (hadc1.DMA_Handle->Init.MemDataAlignment != DMA_MDATAALIGN_HALFWORD)
        || (hadc1.DMA_Handle->Init.MemInc != DMA_MINC_ENABLE)
        || (hadc1.Init.ScanConvMode != ADC_SCAN_ENABLE)
        || (hadc1.Init.ContinuousConvMode != DISABLE)
        || (hadc1.Init.NbrOfConversion != JOYSTICK_ADC_COUNT))
    {
        return false;
    }

    if ((HAL_ADC_Stop_DMA(&hadc1) != HAL_OK)
        || (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK))
    {
        return false;
    }

    s_is_initialized = HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_adc_values,
                                         JOYSTICK_ADC_COUNT) == HAL_OK;
    return s_is_initialized;
}

bool joystick_read(joystick_t *channels)
{
    uint16_t raw[JOYSTICK_AXIS_COUNT];
    uint16_t values[JOYSTICK_AXIS_COUNT];

    if ((channels == NULL) || s_calibration.active || !joystick_read_adc(raw))
    {
        return false;
    }

    for (uint8_t i = 0U; i < JOYSTICK_AXIS_COUNT; i++)
    {
        float mean = filter_moving_average(&s_filters[i], raw[i]);
        uint16_t adc = (uint16_t)(mean + 0.5f);
        values[i] = i < 3U ? joystick_convert_axis(adc, s_calibration_adc[i])
            : joystick_convert_throttle(adc, s_calibration_adc[i]);
    }

    joystick_t sample = {
        .roll = values[0],
        .pitch = values[1],
        .yaw = values[2],
        .throttle = values[3]
    };
    *channels = sample;
    return true;
}

uint16_t joystick_get_battery_adc(void)
{
    return s_is_initialized ? s_battery_adc : 0U;
}

joystick_calibration_result_t joystick_calibrate(void)
{
    uint16_t raw[JOYSTICK_AXIS_COUNT];
    uint32_t now = HAL_GetTick();

    if (!s_is_initialized)
    {
        s_calibration.active = false;
        return JOYSTICK_CALIBRATION_FAILED;
    }

    if (!s_calibration.active)
    {
        /* Discard the pre-request frame; subsequent calls consume fresh scans. */
        if (HAL_ADC_Stop_DMA(&hadc1) != HAL_OK)
        {
            s_is_initialized = false;
            return JOYSTICK_CALIBRATION_FAILED;
        }
        s_sample_ready = false;
        if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_adc_values,
                              JOYSTICK_ADC_COUNT) != HAL_OK)
        {
            s_is_initialized = false;
            return JOYSTICK_CALIBRATION_FAILED;
        }
        for (uint8_t i = 0U; i < JOYSTICK_AXIS_COUNT; i++)
        {
            s_calibration.sum[i] = 0U;
        }
        s_calibration.count = 0U;
        s_calibration.start_ms = now;
        s_calibration.last_sample_ms = now;
        s_calibration.active = true;
        return JOYSTICK_CALIBRATION_BUSY;
    }

    if ((uint32_t)(now - s_calibration.start_ms) >= JOYSTICK_CALIBRATION_TIMEOUT_MS)
    {
        s_calibration.active = false;
        return JOYSTICK_CALIBRATION_FAILED;
    }
    if ((uint32_t)(now - s_calibration.last_sample_ms) < JOYSTICK_CALIBRATION_INTERVAL_MS)
    {
        return JOYSTICK_CALIBRATION_BUSY;
    }

    if (!joystick_read_adc(raw))
    {
        if (!s_is_initialized)
        {
            s_calibration.active = false;
            return JOYSTICK_CALIBRATION_FAILED;
        }
        return JOYSTICK_CALIBRATION_BUSY;
    }
    s_calibration.last_sample_ms = now;

    for (uint8_t i = 0U; i < JOYSTICK_AXIS_COUNT; i++)
    {
        if ((i < 3U && (raw[i] < JOYSTICK_CENTER_MIN_ADC || raw[i] > JOYSTICK_CENTER_MAX_ADC))
            || (i == 3U && raw[i] > JOYSTICK_THROTTLE_MIN_MAX_ADC))
        {
            s_calibration.active = false;
            return JOYSTICK_CALIBRATION_FAILED;
        }
        s_calibration.sum[i] += raw[i];
    }

    s_calibration.count++;
    if (s_calibration.count < JOYSTICK_CALIBRATION_SAMPLES)
    {
        return JOYSTICK_CALIBRATION_BUSY;
    }

    uint8_t count = s_calibration.count;
    joystick_calibration_t calibration = {
        .roll_center_adc = (uint16_t)((s_calibration.sum[0] + count / 2U) / count),
        .pitch_center_adc = (uint16_t)((s_calibration.sum[1] + count / 2U) / count),
        .yaw_center_adc = (uint16_t)((s_calibration.sum[2] + count / 2U) / count),
        .throttle_min_adc = (uint16_t)((s_calibration.sum[3] + count / 2U) / count)
    };
    s_calibration.active = false;
    return joystick_set_calibration(&calibration)
        ? JOYSTICK_CALIBRATION_DONE : JOYSTICK_CALIBRATION_FAILED;
}

bool joystick_set_calibration(const joystick_calibration_t *calibration)
{
    if (calibration == NULL)
    {
        return false;
    }

    const uint16_t values[JOYSTICK_AXIS_COUNT] = {
        calibration->roll_center_adc,
        calibration->pitch_center_adc,
        calibration->yaw_center_adc,
        calibration->throttle_min_adc
    };

    for (uint8_t i = 0U; i < JOYSTICK_AXIS_COUNT; i++)
    {
        if ((i < 3U && (values[i] < JOYSTICK_CENTER_MIN_ADC || values[i] > JOYSTICK_CENTER_MAX_ADC))
            || (i == 3U && values[i] > JOYSTICK_THROTTLE_MIN_MAX_ADC))
        {
            return false;
        }
    }

    for (uint8_t i = 0U; i < JOYSTICK_AXIS_COUNT; i++)
    {
        s_calibration_adc[i] = values[i];
    }
    joystick_reset_filters();
    return true;
}

void joystick_get_calibration(joystick_calibration_t *calibration)
{
    if (calibration == NULL)
    {
        return;
    }

    calibration->roll_center_adc = s_calibration_adc[0];
    calibration->pitch_center_adc = s_calibration_adc[1];
    calibration->yaw_center_adc = s_calibration_adc[2];
    calibration->throttle_min_adc = s_calibration_adc[3];
}

/** @brief Mark an ADC1 scan complete; all processing stays in the caller's task. */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        s_sample_ready = true;
    }
}
