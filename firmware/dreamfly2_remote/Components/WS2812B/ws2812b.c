/**
 * @file ws2812b.c
 * @brief Remote WS2812B driver using TIM3 channel 1 PWM and normal DMA.
 * @author 梅雨
 * @date 2026-10-26
 */

#include "ws2812b.h"

#include <string.h>

#include "tim.h"

#define WS2812B_TIMER_CHANNEL TIM_CHANNEL_1
#define WS2812B_BITS_PER_LED 24U
#define WS2812B_RESET_SLOTS 200U
#define WS2812B_BUFFER_SIZE \
    (WS2812B_RESET_SLOTS + WS2812B_LED_COUNT * WS2812B_BITS_PER_LED + WS2812B_RESET_SLOTS)

/* TIM3: 72 MHz, PSC=0, ARR=89 -> 800 kHz / 1.25 us per bit.
 * 22/65 counts -> approximately 306/903 ns high.
 * Prefix and suffix each hold low for 250 us, including the first frame.
 * Word-sized storage matches the generated Word/Word DMA configuration.
 */
#define WS2812B_DUTY_0 22U
#define WS2812B_DUTY_1 65U
#define WS2812B_DEFAULT_BRIGHTNESS 8U
#define WS2812B_GRADIENT_RANGE 768U
#define WS2812B_GRADIENT_STEP 5U

typedef struct
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} ws2812b_pixel_t;

static ws2812b_pixel_t s_pixels[WS2812B_LED_COUNT];
static uint32_t s_pwm_buffer[WS2812B_BUFFER_SIZE]; /* Owned by DMA until completion. */
static volatile uint8_t s_transmitting; /* Shared by the sole task owner and DMA callbacks. */
static uint16_t s_gradient_phase;

/** @brief Append one byte as eight MSB-first PWM compare values. */
static void ws2812b_encode_byte(uint8_t value, uint16_t *index)
{
    for (uint8_t i = 0U; i < 8U; i++)
    {
        s_pwm_buffer[(*index)++] = (value & (0x80U >> i)) != 0U
            ? WS2812B_DUTY_1 : WS2812B_DUTY_0;
    }
}

/** @brief Stop DMA/PWM, keep the data output low and release the transfer buffer. */
static void ws2812b_finish(void)
{
    (void)HAL_TIM_PWM_Stop_DMA(&htim3, WS2812B_TIMER_CHANNEL);
    __HAL_TIM_SET_COMPARE(&htim3, WS2812B_TIMER_CHANNEL, 0U);
    s_transmitting = 0U;
}

void ws2812b_init(void)
{
    s_transmitting = 0U;
    s_gradient_phase = 0U;
    memset(s_pixels, 0, sizeof(s_pixels));
    __HAL_TIM_SET_COMPARE(&htim3, WS2812B_TIMER_CHANNEL, 0U);
}

void ws2812b_set_pixel(uint8_t index, uint8_t red, uint8_t green, uint8_t blue)
{
    if (index >= WS2812B_LED_COUNT)
    {
        return;
    }
    s_pixels[index] = (ws2812b_pixel_t){red, green, blue};
}

void ws2812b_set_all(uint8_t red, uint8_t green, uint8_t blue)
{
    for (uint8_t i = 0U; i < WS2812B_LED_COUNT; i++)
    {
        ws2812b_set_pixel(i, red, green, blue);
    }
}

void ws2812b_show(void)
{
    if (s_transmitting != 0U)
    {
        return;
    }

    memset(s_pwm_buffer, 0, sizeof(s_pwm_buffer));
    uint16_t index = WS2812B_RESET_SLOTS;
    for (uint8_t i = 0U; i < WS2812B_LED_COUNT; i++)
    {
        ws2812b_encode_byte(s_pixels[i].green, &index);
        ws2812b_encode_byte(s_pixels[i].red, &index);
        ws2812b_encode_byte(s_pixels[i].blue, &index);
    }

    s_transmitting = 1U;
    if (HAL_TIM_PWM_Start_DMA(&htim3, WS2812B_TIMER_CHANNEL,
                              s_pwm_buffer, WS2812B_BUFFER_SIZE) != HAL_OK)
    {
        ws2812b_finish();
    }
}

void ws2812b_gradient(void)
{
    if (s_transmitting != 0U)
    {
        return;
    }

    ws2812b_pixel_t color = {0U, 0U, 0U};
    uint8_t position = (uint8_t)(s_gradient_phase % 256U);
    if (s_gradient_phase < 256U)
    {
        color.red = 255U - position;
        color.green = position;
    }
    else if (s_gradient_phase < 512U)
    {
        color.green = 255U - position;
        color.blue = position;
    }
    else
    {
        color.blue = 255U - position;
        color.red = position;
    }
    color.red = (uint8_t)((uint16_t)color.red * WS2812B_DEFAULT_BRIGHTNESS / 255U);
    color.green = (uint8_t)((uint16_t)color.green * WS2812B_DEFAULT_BRIGHTNESS / 255U);
    color.blue = (uint8_t)((uint16_t)color.blue * WS2812B_DEFAULT_BRIGHTNESS / 255U);
    ws2812b_set_all(color.red, color.green, color.blue);
    ws2812b_show();

    s_gradient_phase = (s_gradient_phase + WS2812B_GRADIENT_STEP) % WS2812B_GRADIENT_RANGE;
}

void ws2812b_all_red(void)
{
    ws2812b_set_all(WS2812B_DEFAULT_BRIGHTNESS, 0U, 0U);
    ws2812b_show();
}

void ws2812b_all_orange(void)
{
    ws2812b_set_all(WS2812B_DEFAULT_BRIGHTNESS,
                    (uint8_t)(WS2812B_DEFAULT_BRIGHTNESS * 165U / 255U), 0U);
    ws2812b_show();
}

void ws2812b_all_yellow(void)
{
    ws2812b_set_all(WS2812B_DEFAULT_BRIGHTNESS, WS2812B_DEFAULT_BRIGHTNESS, 0U);
    ws2812b_show();
}

void ws2812b_all_green(void)
{
    ws2812b_set_all(0U, WS2812B_DEFAULT_BRIGHTNESS, 0U);
    ws2812b_show();
}

void ws2812b_all_cyan(void)
{
    ws2812b_set_all(0U, WS2812B_DEFAULT_BRIGHTNESS, WS2812B_DEFAULT_BRIGHTNESS);
    ws2812b_show();
}

void ws2812b_all_blue(void)
{
    ws2812b_set_all(0U, 0U, WS2812B_DEFAULT_BRIGHTNESS);
    ws2812b_show();
}

void ws2812b_all_violet(void)
{
    uint8_t intensity = (uint8_t)(WS2812B_DEFAULT_BRIGHTNESS * 205U / 255U);
    ws2812b_set_all(intensity, 0U, intensity);
    ws2812b_show();
}

void ws2812b_all_white(void)
{
    ws2812b_set_all(WS2812B_DEFAULT_BRIGHTNESS, WS2812B_DEFAULT_BRIGHTNESS,
                    WS2812B_DEFAULT_BRIGHTNESS);
    ws2812b_show();
}

void ws2812b_all_off(void)
{
    ws2812b_set_all(0U, 0U, 0U);
    ws2812b_show();
}

/** @brief HAL DMA completion hook; do not transmit, delay or call RTOS APIs here. */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        ws2812b_finish();
    }
}

/** @brief Release a failed LED transfer so a later frame may retry. */
void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        ws2812b_finish();
    }
}
