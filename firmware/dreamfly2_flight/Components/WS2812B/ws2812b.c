/**
 * @file ws2812b.c
 * @brief WS2812B RGB LED driver using TIM2 PWM and DMA.
 * @author 梅雨
 * @date 2026-10-09
 */

#include "ws2812b.h"

#include "tim.h"

#define WS2812B_TIMER_CHANNEL TIM_CHANNEL_3
#define WS2812B_BITS_PER_LED 24U
#define WS2812B_RESET_SLOTS 200U
#define WS2812B_BUFFER_SIZE \
    (WS2812B_LED_COUNT * WS2812B_BITS_PER_LED + WS2812B_RESET_SLOTS)

/* TIM2 clock: 100 MHz, period: 125 counts. */
#define WS2812B_DUTY_0 30U
#define WS2812B_DUTY_1 90U

#define WS2812B_RUN_COLOR_COUNT 7U
#define WS2812B_GRADIENT_RANGE 768U
#define WS2812B_GRADIENT_STEP 5U
#define WS2812B_GRADIENT_BRIGHTNESS 64U

typedef struct
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} ws2812b_pixel_t;

static const ws2812b_pixel_t s_run_colors[WS2812B_RUN_COLOR_COUNT] =
{
    {255U, 165U, 0U},
    {0U, 255U, 0U},
    {255U, 0U, 255U},
    {0U, 255U, 255U},
    {0U, 0U, 255U},
    {255U, 255U, 0U},
    {255U, 255U, 255U}
};

static ws2812b_pixel_t s_pixels[WS2812B_LED_COUNT];
static uint32_t s_pwm_buffer[WS2812B_BUFFER_SIZE]; /* DMA-owned until completion. */
static volatile uint8_t s_transmitting; /* Shared by the calling task and DMA callback. */

/** @brief Map a wrapped hue phase to a brightness-limited RGB color. */
static ws2812b_pixel_t ws2812b_gradient_color(uint16_t hue)
{
    ws2812b_pixel_t color = {0U, 0U, 0U};
    uint8_t position;

    hue %= WS2812B_GRADIENT_RANGE;

    if (hue < 256U)
    {
        position = (uint8_t)hue;
        color.red = (uint8_t)(255U - position);
        color.green = position;
    }
    else if (hue < 512U)
    {
        position = (uint8_t)(hue - 256U);
        color.green = (uint8_t)(255U - position);
        color.blue = position;
    }
    else
    {
        position = (uint8_t)(hue - 512U);
        color.blue = (uint8_t)(255U - position);
        color.red = position;
    }

    color.red = (uint8_t)(((uint16_t)color.red * WS2812B_GRADIENT_BRIGHTNESS) / 255U);
    color.green = (uint8_t)(((uint16_t)color.green * WS2812B_GRADIENT_BRIGHTNESS) / 255U);
    color.blue = (uint8_t)(((uint16_t)color.blue * WS2812B_GRADIENT_BRIGHTNESS) / 255U);

    return color;
}

/** @brief Append eight PWM compare values, most-significant bit first; advance index. */
static void ws2812b_encode_byte(uint8_t data, uint32_t *buffer, uint16_t *index)
{
    uint8_t mask;

    for (mask = 0x80U; mask != 0U; mask >>= 1U)
    {
        buffer[*index] = ((data & mask) != 0U) ? WS2812B_DUTY_1 : WS2812B_DUTY_0;
        (*index)++;
    }
}

void ws2812b_init(void)
{
    s_transmitting = 0U;
    ws2812b_set_all(0U, 0U, 0U);
    __HAL_TIM_SET_COMPARE(&htim2, WS2812B_TIMER_CHANNEL, 0U);
}

void ws2812b_set_pixel(uint8_t index, uint8_t red, uint8_t green, uint8_t blue)
{
    if (index >= WS2812B_LED_COUNT)
    {
        return;
    }

    s_pixels[index].red = red;
    s_pixels[index].green = green;
    s_pixels[index].blue = blue;
}

void ws2812b_set_all(uint8_t red, uint8_t green, uint8_t blue)
{
    uint8_t index;

    for (index = 0U; index < WS2812B_LED_COUNT; index++)
    {
        ws2812b_set_pixel(index, red, green, blue);
    }
}

void ws2812b_show(void)
{
    uint8_t pixel_index;
    uint16_t buffer_index = 0U;

    if (s_transmitting != 0U)
    {
        return;
    }

    for (pixel_index = 0U; pixel_index < WS2812B_LED_COUNT; pixel_index++)
    {
        ws2812b_encode_byte(s_pixels[pixel_index].green, s_pwm_buffer, &buffer_index);
        ws2812b_encode_byte(s_pixels[pixel_index].red, s_pwm_buffer, &buffer_index);
        ws2812b_encode_byte(s_pixels[pixel_index].blue, s_pwm_buffer, &buffer_index);
    }

    while (buffer_index < WS2812B_BUFFER_SIZE)
    {
        s_pwm_buffer[buffer_index] = 0U;
        buffer_index++;
    }

    s_transmitting = 1U;
    if (HAL_TIM_PWM_Start_DMA(
            &htim2,
            WS2812B_TIMER_CHANNEL,
            s_pwm_buffer,
            WS2812B_BUFFER_SIZE) != HAL_OK)
    {
        s_transmitting = 0U;
    }
}

void ws2812b_run(void)
{
    static uint8_t pixel_index;
    static uint8_t color_index;
    const ws2812b_pixel_t *color = &s_run_colors[color_index];

    ws2812b_set_all(0U, 0U, 0U);
    ws2812b_set_pixel(pixel_index, color->red, color->green, color->blue);
    ws2812b_show();

    pixel_index++;
    if (pixel_index >= WS2812B_LED_COUNT)
    {
        pixel_index = 0U;
        color_index++;
        if (color_index >= WS2812B_RUN_COLOR_COUNT)
        {
            color_index = 0U;
        }
    }
}

void ws2812b_gradient(void)
{
    static uint16_t phase;
    ws2812b_pixel_t color = ws2812b_gradient_color(phase);

    ws2812b_set_all(color.red, color.green, color.blue);
    ws2812b_show();

    phase += WS2812B_GRADIENT_STEP;
    if (phase >= WS2812B_GRADIENT_RANGE)
    {
        phase = 0U;
    }
}

void ws2812b_all_red(void)
{
    ws2812b_set_all(255U, 0U, 0U);
    ws2812b_show();
}

void ws2812b_all_orange(void)
{
    ws2812b_set_all(255U, 165U, 0U);
    ws2812b_show();
}

void ws2812b_all_yellow(void)
{
    ws2812b_set_all(255U, 255U, 0U);
    ws2812b_show();
}

void ws2812b_all_green(void)
{
    ws2812b_set_all(0U, 255U, 0U);
    ws2812b_show();
}

void ws2812b_all_cyan(void)
{
    ws2812b_set_all(0U, 255U, 255U);
    ws2812b_show();
}

void ws2812b_all_blue(void)
{
    ws2812b_set_all(0U, 0U, 255U);
    ws2812b_show();
}

void ws2812b_all_violet(void)
{
    ws2812b_set_all(205U, 0U, 205U);
    ws2812b_show();
}

void ws2812b_all_white(void)
{
    ws2812b_set_all(255U, 255U, 255U);
    ws2812b_show();
}

void ws2812b_all_off(void)
{
    ws2812b_set_all(0U, 0U, 0U);
    ws2812b_show();
}

/** @brief HAL completion hook: stop this LED transfer, hold data low and release its buffer. */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if ((htim->Instance == TIM2) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3))
    {
        (void)HAL_TIM_PWM_Stop_DMA(htim, WS2812B_TIMER_CHANNEL);
        __HAL_TIM_SET_COMPARE(htim, WS2812B_TIMER_CHANNEL, 0U);
        s_transmitting = 0U;
    }
}
