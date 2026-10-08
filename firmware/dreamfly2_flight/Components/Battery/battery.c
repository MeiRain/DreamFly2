/**
 * @file battery.c
 * @brief Battery voltage measurement using STM32 ADC1.
 * @author 梅雨
 * @date 2026-10-08
 */

#include "battery.h"

#include <stdint.h>

#include "adc.h"
#include "filter.h"

#define BATTERY_ADC_FULL_SCALE 4095.0f
#define BATTERY_ADC_REFERENCE_VOLTAGE_V 3.31f
#define BATTERY_VOLTAGE_DIVIDER_GAIN 2.0f
#define BATTERY_FILTER_WINDOW_SIZE 6U
#define BATTERY_ADC_TIMEOUT_MS 2U

float battery_read_voltage(void)
{
    uint16_t adc_raw;
    float filtered_adc;
    float voltage_v;

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, BATTERY_ADC_TIMEOUT_MS);

    adc_raw = (uint16_t)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    filter_moving_average((float)adc_raw, &filtered_adc, BATTERY_FILTER_WINDOW_SIZE);
    voltage_v = filtered_adc
                * BATTERY_ADC_REFERENCE_VOLTAGE_V
                / BATTERY_ADC_FULL_SCALE
                * BATTERY_VOLTAGE_DIVIDER_GAIN;

    return voltage_v;
}
