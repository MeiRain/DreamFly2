/**
 * @file oled.c
 * @brief SSD1306 128x64 drawing buffer and HAL I2C1 transport.
 * @author 梅雨
 * @date 2026-10-24
 *
 * Drawing algorithms adapted from Jiangxie Technology's OLED V2.0.
 * Original copyright: Jiangxie Technology, jiangxiekeji.com.
 * Coordinates are pixels: top-left (0,0), x right, y down.
 * Geometry uses x/y offsets, dx/dy deltas and d as the midpoint error.
 */

#include "oled.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "i2c.h"
#include "oledfont.h"

#define OLED_I2C_ADDRESS 0x78U /* 7-bit address 0x3C shifted for HAL. */
#define OLED_COMMAND     0x00U
#define OLED_DATA        0x40U
#define OLED_TIMEOUT_MS  10U

/* One byte stores eight vertically adjacent pixels, LSB at the top. */
static uint8_t s_display_buffer[8][128];

/** @brief Send a command/data block; stop on a HAL I2C error. */
static bool oled_write(uint8_t control, const uint8_t *data, uint16_t length)
{
    return HAL_I2C_Mem_Write(&hi2c1, OLED_I2C_ADDRESS, control,
                            I2C_MEMADD_SIZE_8BIT, (uint8_t *)data,
                            length, OLED_TIMEOUT_MS) == HAL_OK;
}

/** @brief Select a display RAM page and starting column. */
static bool oled_set_cursor(uint8_t page, uint8_t x)
{
    const uint8_t commands[] = {
        (uint8_t)(0xB0U | page),
        (uint8_t)(0x10U | (x >> 4U)),
        (uint8_t)(x & 0x0FU)
    };
    return oled_write(OLED_COMMAND, commands, sizeof(commands));
}

bool oled_init(void)
{
    static const uint8_t commands[] = {
        0xAE,             /* Display off while configuring. */
        0xD5, 0x80,       /* Clock divider and oscillator. */
        0xA8, 0x3F,       /* 64 multiplexed rows. */
        0xD3, 0x00, 0x40, /* Display offset and start line. */
        0x20, 0x02,       /* Page addressing mode. */
        0xA1, 0xC8,       /* Segment remap and COM scan direction. */
        0xDA, 0x12,       /* COM pin configuration for 128x64 panels. */
        0x81, 0xCF,       /* Contrast. */
        0xD9, 0xF1,       /* Pre-charge period. */
        0xDB, 0x30,       /* VCOMH level. */
        0xA4, 0xA6,       /* Follow display RAM, normal polarity. */
        0x8D, 0x14        /* Internal charge pump enabled. */
    };
    const uint8_t display_on = 0xAF;

    HAL_Delay(100U);
    if (!oled_write(OLED_COMMAND, commands, sizeof(commands)))
    {
        return false;
    }

    oled_clear();
    if (!oled_update())
    {
        return false;
    }

    return oled_write(OLED_COMMAND, &display_on, 1U);
}

/** @brief Compute an unsigned integer power for decimal scaling. */
static uint32_t oled_pow(uint32_t base, uint32_t exponent)
{
    uint32_t result = 1;
    while (exponent --)
    {
        result *= base;
    }
    return result;
}

/** @brief Test polygon inclusion using W. Randolph Franklin's crossing rule. */
static uint8_t oled_pnpoly(uint8_t nvert, int16_t *vertx, int16_t *verty, int16_t testx, int16_t testy)
{
    int16_t i, j, c = 0;

    for (i = 0, j = nvert - 1; i < nvert; j = i++)
    {
        if (((verty[i] > testy) != (verty[j] > testy)) &&
            (testx < (vertx[j] - vertx[i]) * (testy - verty[i]) / (verty[j] - verty[i]) + vertx[i]))
        {
            c = !c;
        }
    }
    return c;
}

/** @brief Test a clockwise arc angle in screen coordinates, in degrees. */
static uint8_t oled_is_in_angle(int16_t origin_x, int16_t origin_y, int16_t start_angle, int16_t end_angle)
{
    int16_t point_angle;
    point_angle = atan2f(origin_y, origin_x) * 57.2957795f;
    if (start_angle < end_angle)
    {

        if (point_angle >= start_angle && point_angle <= end_angle)
        {
            return 1;
        }
    }
    else
    {

        if (point_angle >= start_angle || point_angle <= end_angle)
        {
            return 1;
        }
    }
    return 0;
}

bool oled_update(void)
{
    return oled_update_area(0, 0, OLED_WIDTH, OLED_HEIGHT);
}

bool oled_update_area(int16_t x, int16_t y, uint8_t width, uint8_t height)
{
    int32_t right = (int32_t)x + width;
    int32_t bottom = (int32_t)y + height;

    if ((width == 0U) || (height == 0U)
        || (x >= OLED_WIDTH) || (y >= OLED_HEIGHT)
        || (right <= 0) || (bottom <= 0))
    {
        return true;
    }

    if (x < 0) { x = 0; }
    if (y < 0) { y = 0; }
    if (right > OLED_WIDTH) { right = OLED_WIDTH; }
    if (bottom > OLED_HEIGHT) { bottom = OLED_HEIGHT; }

    /* The panel refreshes complete 8-pixel-high pages. */
    for (uint8_t page = (uint8_t)(y / 8); page <= (bottom - 1) / 8; page++)
    {
        if (!oled_set_cursor(page, (uint8_t)x)
            || !oled_write(OLED_DATA, &s_display_buffer[page][x],
                           (uint16_t)(right - x)))
        {
            return false;
        }
    }

    return true;
}

void oled_clear(void)
{
    uint8_t i, j;
    for (j = 0; j < 8; j++)
    {
        for (i = 0; i < 128; i++)
        {
            s_display_buffer[j][i] = 0x00;
        }
    }
}

void oled_clear_area(int16_t origin_x, int16_t origin_y, uint8_t width, uint8_t height)
{
    int16_t i, j;

    for (j = origin_y; j < origin_y + height; j++)
    {
        for (i = origin_x; i < origin_x + width; i++)
        {
            if (i >= 0 && i <= 127 && j >=0 && j <= 63)
            {
                s_display_buffer[j / 8][i] &= ~(0x01 << (j % 8));
            }
        }
    }
}

void oled_reverse(void)
{
    uint8_t i, j;
    for (j = 0; j < 8; j++)
    {
        for (i = 0; i < 128; i++)
        {
            s_display_buffer[j][i] ^= 0xFF;
        }
    }
}

void oled_reverse_area(int16_t origin_x, int16_t origin_y, uint8_t width, uint8_t height)
{
    int16_t i, j;

    for (j = origin_y; j < origin_y + height; j++)
    {
        for (i = origin_x; i < origin_x + width; i++)
        {
            if (i >= 0 && i <= 127 && j >=0 && j <= 63)
            {
                s_display_buffer[j / 8][i] ^= 0x01 << (j % 8);
            }
        }
    }
}

void oled_show_char(int16_t x, int16_t y, char character, uint8_t font_size)
{
    if ((uint8_t)character < ' ' || (uint8_t)character > '~')
    {
        character = '?';
    }

    if (font_size == OLED_8X16)
    {
        oled_show_image(x, y, 8U, 16U, oled_font_8x16[(uint8_t)character - ' ']);
    }
    else if (font_size == OLED_6X8)
    {
        oled_show_image(x, y, 6U, 8U, oled_font_6x8[(uint8_t)character - ' ']);
    }
}

void oled_show_string(int16_t origin_x, int16_t origin_y, const char *text, uint8_t font_size)
{
    if ((text == NULL) || ((font_size != OLED_8X16) && (font_size != OLED_6X8)))
    {
        return;
    }

    uint16_t i = 0;
    char single_char[5];
    uint8_t char_length = 0;
    uint16_t x_offset = 0;
    uint16_t font_index;

    while (text[i] != '\0')
    {
        if ((int32_t)origin_x + x_offset >= OLED_WIDTH) { break; }

        if ((text[i] & 0x80) == 0x00)
        {
            char_length = 1;
            single_char[0] = text[i ++];
            single_char[1] = '\0';
        }
        else if ((text[i] & 0xE0) == 0xC0)
        {
            char_length = 2;
            single_char[0] = text[i ++];
            if (text[i] == '\0') {break;}
            single_char[1] = text[i ++];
            single_char[2] = '\0';
        }
        else if ((text[i] & 0xF0) == 0xE0)
        {
            char_length = 3;
            single_char[0] = text[i ++];
            if (text[i] == '\0') {break;}
            single_char[1] = text[i ++];
            if (text[i] == '\0') {break;}
            single_char[2] = text[i ++];
            single_char[3] = '\0';
        }
        else if ((text[i] & 0xF8) == 0xF0)
        {
            char_length = 4;
            single_char[0] = text[i ++];
            if (text[i] == '\0') {break;}
            single_char[1] = text[i ++];
            if (text[i] == '\0') {break;}
            single_char[2] = text[i ++];
            if (text[i] == '\0') {break;}
            single_char[3] = text[i ++];
            single_char[4] = '\0';
        }
        else
        {
            i ++;
            continue;
        }

        if (char_length == 1)
        {

            oled_show_char(origin_x + x_offset, origin_y, single_char[0], font_size);
            x_offset += font_size;
        }
        else
        {

            for (font_index = 0; strcmp(oled_font_chinese[font_index].index, "") != 0; font_index++)
            {

                if (strcmp(oled_font_chinese[font_index].index, single_char) == 0)
                {
                    break;
                }
            }
            if (font_size == OLED_8X16)
            {

                oled_show_image(origin_x + x_offset, origin_y, 16, 16, oled_font_chinese[font_index].data);
                x_offset += 16;
            }
            else if (font_size == OLED_6X8)
            {

                oled_show_char(origin_x + x_offset, origin_y, '?', OLED_6X8);
                x_offset += OLED_6X8;
            }
        }
    }
}

void oled_show_num(int16_t x, int16_t y, uint32_t number, uint8_t length, uint8_t font_size)
{
    if (length > 10U) { length = 10U; }
    for (uint8_t i = length; i > 0U; i--)
    {
        oled_show_char(x + (i - 1U) * font_size, y, (char)('0' + number % 10U), font_size);
        number /= 10U;
    }
}

void oled_show_signed_num(int16_t x, int16_t y, int32_t number, uint8_t length, uint8_t font_size)
{
    /* Unsigned subtraction also handles INT32_MIN without signed overflow. */
    uint32_t magnitude = number < 0 ? 0U - (uint32_t)number : (uint32_t)number;
    oled_show_char(x, y, number < 0 ? '-' : '+', font_size);
    oled_show_num(x + font_size, y, magnitude, length, font_size);
}

void oled_show_hex_num(int16_t x, int16_t y, uint32_t number, uint8_t length, uint8_t font_size)
{
    static const char digits[] = "0123456789ABCDEF";
    if (length > 8U) { length = 8U; }
    for (uint8_t i = length; i > 0U; i--)
    {
        oled_show_char(x + (i - 1U) * font_size, y, digits[number & 0x0FU], font_size);
        number >>= 4U;
    }
}

void oled_show_bin_num(int16_t x, int16_t y, uint32_t number, uint8_t length, uint8_t font_size)
{
    if (length > 32U) { length = 32U; }
    for (uint8_t i = length; i > 0U; i--)
    {
        oled_show_char(x + (i - 1U) * font_size, y, (char)('0' + (number & 1U)), font_size);
        number >>= 1U;
    }
}

void oled_show_float_num(int16_t x, int16_t y, double number, uint8_t int_length,
                         uint8_t fra_length, uint8_t font_size)
{
    if (!isfinite(number) || fabs(number) > 4294967294.0)
    {
        oled_show_string(x, y, "ERR", font_size);
        return;
    }
    if (int_length > 10U) { int_length = 10U; }
    if (fra_length > 6U) { fra_length = 6U; }

    uint32_t scale = oled_pow(10U, fra_length);
    double value = fabs(number);
    uint32_t integer = (uint32_t)value;
    uint32_t fraction = (uint32_t)round((value - integer) * scale);
    integer += fraction / scale;
    fraction %= scale;

    oled_show_char(x, y, signbit(number) ? '-' : '+', font_size);
    oled_show_num(x + font_size, y, integer, int_length, font_size);
    if (fra_length > 0U)
    {
        oled_show_char(x + (int_length + 1U) * font_size, y, '.', font_size);
        oled_show_num(x + (int_length + 2U) * font_size, y, fraction, fra_length, font_size);
    }
}

void oled_show_image(int16_t x, int16_t y, uint8_t width, uint8_t height,
                     const uint8_t *image)
{
    if ((image == NULL) || (width == 0U) || (height == 0U))
    {
        return;
    }

    /* Source layout: vertical groups of 8 pixels, low bit at the top. */
    for (uint16_t row = 0U; row < height; row++)
    {
        int32_t py = (int32_t)y + row;
        if ((py < 0) || (py >= OLED_HEIGHT)) { continue; }
        for (uint16_t col = 0U; col < width; col++)
        {
            int32_t px = (int32_t)x + col;
            if ((px < 0) || (px >= OLED_WIDTH)) { continue; }
            uint8_t mask = (uint8_t)(1U << (py % 8));
            if (image[(row / 8U) * width + col] & (1U << (row % 8U)))
            {
                s_display_buffer[py / 8][px] |= mask;
            }
            else
            {
                s_display_buffer[py / 8][px] &= (uint8_t)~mask;
            }
        }
    }
}

void oled_printf(int16_t x, int16_t y, uint8_t font_size, const char *format, ...)
{
    char text[128];
    va_list args;
    if (format == NULL) { return; }
    va_start(args, format);
    int count = vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    if (count >= 0)
    {
        oled_show_string(x, y, text, font_size);
    }
}

void oled_draw_point(int16_t origin_x, int16_t origin_y)
{
    if (origin_x >= 0 && origin_x <= 127 && origin_y >=0 && origin_y <= 63)
    {

        s_display_buffer[origin_y / 8][origin_x] |= 0x01 << (origin_y % 8);
    }
}

uint8_t oled_get_point(int16_t origin_x, int16_t origin_y)
{
    if (origin_x >= 0 && origin_x <= 127 && origin_y >=0 && origin_y <= 63)
    {

        if (s_display_buffer[origin_y / 8][origin_x] & 0x01 << (origin_y % 8))
        {
            return 1;
        }
    }

    return 0;
}

void oled_draw_line(int16_t point0_x, int16_t point0_y, int16_t point1_x, int16_t point1_y)
{
    int16_t x, y, dx, dy, d, incr_e, incr_ne, temp;
    int16_t x0 = point0_x, y0 = point0_y, x1 = point1_x, y1 = point1_y;
    uint8_t yflag = 0, xyflag = 0;

    if (y0 == y1)
    {

        if (x0 > x1) {temp = x0; x0 = x1; x1 = temp;}

        for (x = x0; x <= x1; x++)
        {
            oled_draw_point(x, y0);
        }
    }
    else if (x0 == x1)
    {

        if (y0 > y1) {temp = y0; y0 = y1; y1 = temp;}

        for (y = y0; y <= y1; y++)
        {
            oled_draw_point(x0, y);
        }
    }
    else
    {

        if (x0 > x1)
        {

            temp = x0; x0 = x1; x1 = temp;
            temp = y0; y0 = y1; y1 = temp;
        }

        if (y0 > y1)
        {

            y0 = -y0;
            y1 = -y1;

            yflag = 1;
        }

        if (y1 - y0 > x1 - x0)
        {

            temp = x0; x0 = y0; y0 = temp;
            temp = x1; x1 = y1; y1 = temp;

            xyflag = 1;
        }

        dx = x1 - x0;
        dy = y1 - y0;
        incr_e = 2 * dy;
        incr_ne = 2 * (dy - dx);
        d = 2 * dy - dx;
        x = x0;
        y = y0;

        if (yflag && xyflag){oled_draw_point(y, -x);}
        else if (yflag)        {oled_draw_point(x, -y);}
        else if (xyflag)    {oled_draw_point(y, x);}
        else                {oled_draw_point(x, y);}

        while (x < x1)
        {
            x ++;
            if (d < 0)
            {
                d += incr_e;
            }
            else
            {
                y ++;
                d += incr_ne;
            }

            if (yflag && xyflag){oled_draw_point(y, -x);}
            else if (yflag)        {oled_draw_point(x, -y);}
            else if (xyflag)    {oled_draw_point(y, x);}
            else                {oled_draw_point(x, y);}
        }
    }
}

void oled_draw_rectangle(int16_t origin_x, int16_t origin_y, uint8_t width, uint8_t height, uint8_t is_filled)
{
    if ((width == 0U) || (height == 0U)) { return; }

    int16_t i, j;
    if (!is_filled)
    {

        for (i = origin_x; i < origin_x + width; i++)
        {
            oled_draw_point(i, origin_y);
            oled_draw_point(i, origin_y + height - 1);
        }

        for (i = origin_y; i < origin_y + height; i++)
        {
            oled_draw_point(origin_x, i);
            oled_draw_point(origin_x + width - 1, i);
        }
    }
    else
    {

        for (i = origin_x; i < origin_x + width; i++)
        {

            for (j = origin_y; j < origin_y + height; j++)
            {

                oled_draw_point(i, j);
            }
        }
    }
}

void oled_draw_triangle(int16_t point0_x, int16_t point0_y, int16_t point1_x, int16_t point1_y, int16_t point2_x, int16_t point2_y, uint8_t is_filled)
{
    int16_t minx = point0_x, miny = point0_y, maxx = point0_x, maxy = point0_y;
    int16_t i, j;
    int16_t vx[] = {point0_x, point1_x, point2_x};
    int16_t vy[] = {point0_y, point1_y, point2_y};

    if (!is_filled)
    {

        oled_draw_line(point0_x, point0_y, point1_x, point1_y);
        oled_draw_line(point0_x, point0_y, point2_x, point2_y);
        oled_draw_line(point1_x, point1_y, point2_x, point2_y);
    }
    else
    {

        if (point1_x < minx) {minx = point1_x;}
        if (point2_x < minx) {minx = point2_x;}
        if (point1_y < miny) {miny = point1_y;}
        if (point2_y < miny) {miny = point2_y;}

        if (point1_x > maxx) {maxx = point1_x;}
        if (point2_x > maxx) {maxx = point2_x;}
        if (point1_y > maxy) {maxy = point1_y;}
        if (point2_y > maxy) {maxy = point2_y;}

        for (i = minx; i <= maxx; i++)
        {

            for (j = miny; j <= maxy; j++)
            {

                if (oled_pnpoly(3, vx, vy, i, j)) {oled_draw_point(i, j);}
            }
        }
    }
}

void oled_draw_circle(int16_t origin_x, int16_t origin_y, uint8_t radius, uint8_t is_filled)
{
    int16_t x, y, d, j;

    d = 1 - radius;
    x = 0;
    y = radius;

    oled_draw_point(origin_x + x, origin_y + y);
    oled_draw_point(origin_x - x, origin_y - y);
    oled_draw_point(origin_x + y, origin_y + x);
    oled_draw_point(origin_x - y, origin_y - x);

    if (is_filled)
    {

        for (j = -y; j < y; j++)
        {

            oled_draw_point(origin_x, origin_y + j);
        }
    }

    while (x < y)
    {
        x ++;
        if (d < 0)
        {
            d += 2 * x + 1;
        }
        else
        {
            y --;
            d += 2 * (x - y) + 1;
        }

        oled_draw_point(origin_x + x, origin_y + y);
        oled_draw_point(origin_x + y, origin_y + x);
        oled_draw_point(origin_x - x, origin_y - y);
        oled_draw_point(origin_x - y, origin_y - x);
        oled_draw_point(origin_x + x, origin_y - y);
        oled_draw_point(origin_x + y, origin_y - x);
        oled_draw_point(origin_x - x, origin_y + y);
        oled_draw_point(origin_x - y, origin_y + x);

        if (is_filled)
        {

            for (j = -y; j < y; j++)
            {

                oled_draw_point(origin_x + x, origin_y + j);
                oled_draw_point(origin_x - x, origin_y + j);
            }

            for (j = -x; j < x; j++)
            {

                oled_draw_point(origin_x - y, origin_y + j);
                oled_draw_point(origin_x + y, origin_y + j);
            }
        }
    }
}

void oled_draw_ellipse(int16_t origin_x, int16_t origin_y, uint8_t radius_x, uint8_t radius_y, uint8_t is_filled)
{
    int16_t x, y, j;
    int16_t a = radius_x, b = radius_y;
    if ((a == 0) || (b == 0))
    {
        oled_draw_line(origin_x - a, origin_y - b, origin_x + a, origin_y + b);
        return;
    }
    float d1, d2;

    x = 0;
    y = b;
    d1 = b * b + a * a * (-b + 0.5);

    if (is_filled)
    {

        for (j = -y; j < y; j++)
        {

            oled_draw_point(origin_x, origin_y + j);
            oled_draw_point(origin_x, origin_y + j);
        }
    }

    oled_draw_point(origin_x + x, origin_y + y);
    oled_draw_point(origin_x - x, origin_y - y);
    oled_draw_point(origin_x - x, origin_y + y);
    oled_draw_point(origin_x + x, origin_y - y);

    while (b * b * (x + 1) < a * a * (y - 0.5))
    {
        if (d1 <= 0)
        {
            d1 += b * b * (2 * x + 3);
        }
        else
        {
            d1 += b * b * (2 * x + 3) + a * a * (-2 * y + 2);
            y --;
        }
        x ++;

        if (is_filled)
        {

            for (j = -y; j < y; j++)
            {

                oled_draw_point(origin_x + x, origin_y + j);
                oled_draw_point(origin_x - x, origin_y + j);
            }
        }

        oled_draw_point(origin_x + x, origin_y + y);
        oled_draw_point(origin_x - x, origin_y - y);
        oled_draw_point(origin_x - x, origin_y + y);
        oled_draw_point(origin_x + x, origin_y - y);
    }

    d2 = b * b * (x + 0.5) * (x + 0.5) + a * a * (y - 1) * (y - 1) - a * a * b * b;

    while (y > 0)
    {
        if (d2 <= 0)
        {
            d2 += b * b * (2 * x + 2) + a * a * (-2 * y + 3);
            x ++;

        }
        else
        {
            d2 += a * a * (-2 * y + 3);
        }
        y --;

        if (is_filled)
        {

            for (j = -y; j < y; j++)
            {

                oled_draw_point(origin_x + x, origin_y + j);
                oled_draw_point(origin_x - x, origin_y + j);
            }
        }

        oled_draw_point(origin_x + x, origin_y + y);
        oled_draw_point(origin_x - x, origin_y - y);
        oled_draw_point(origin_x - x, origin_y + y);
        oled_draw_point(origin_x + x, origin_y - y);
    }
}

void oled_draw_arc(int16_t origin_x, int16_t origin_y, uint8_t radius, int16_t start_angle, int16_t end_angle, uint8_t is_filled)
{
    int16_t x, y, d, j;

    d = 1 - radius;
    x = 0;
    y = radius;

    if (oled_is_in_angle(x, y, start_angle, end_angle))    {oled_draw_point(origin_x + x, origin_y + y);}
    if (oled_is_in_angle(-x, -y, start_angle, end_angle)) {oled_draw_point(origin_x - x, origin_y - y);}
    if (oled_is_in_angle(y, x, start_angle, end_angle)) {oled_draw_point(origin_x + y, origin_y + x);}
    if (oled_is_in_angle(-y, -x, start_angle, end_angle)) {oled_draw_point(origin_x - y, origin_y - x);}

    if (is_filled)
    {

        for (j = -y; j < y; j++)
        {

            if (oled_is_in_angle(0, j, start_angle, end_angle)) {oled_draw_point(origin_x, origin_y + j);}
        }
    }

    while (x < y)
    {
        x ++;
        if (d < 0)
        {
            d += 2 * x + 1;
        }
        else
        {
            y --;
            d += 2 * (x - y) + 1;
        }

        if (oled_is_in_angle(x, y, start_angle, end_angle)) {oled_draw_point(origin_x + x, origin_y + y);}
        if (oled_is_in_angle(y, x, start_angle, end_angle)) {oled_draw_point(origin_x + y, origin_y + x);}
        if (oled_is_in_angle(-x, -y, start_angle, end_angle)) {oled_draw_point(origin_x - x, origin_y - y);}
        if (oled_is_in_angle(-y, -x, start_angle, end_angle)) {oled_draw_point(origin_x - y, origin_y - x);}
        if (oled_is_in_angle(x, -y, start_angle, end_angle)) {oled_draw_point(origin_x + x, origin_y - y);}
        if (oled_is_in_angle(y, -x, start_angle, end_angle)) {oled_draw_point(origin_x + y, origin_y - x);}
        if (oled_is_in_angle(-x, y, start_angle, end_angle)) {oled_draw_point(origin_x - x, origin_y + y);}
        if (oled_is_in_angle(-y, x, start_angle, end_angle)) {oled_draw_point(origin_x - y, origin_y + x);}

        if (is_filled)
        {

            for (j = -y; j < y; j++)
            {

                if (oled_is_in_angle(x, j, start_angle, end_angle)) {oled_draw_point(origin_x + x, origin_y + j);}
                if (oled_is_in_angle(-x, j, start_angle, end_angle)) {oled_draw_point(origin_x - x, origin_y + j);}
            }

            for (j = -x; j < x; j++)
            {

                if (oled_is_in_angle(-y, j, start_angle, end_angle)) {oled_draw_point(origin_x - y, origin_y + j);}
                if (oled_is_in_angle(y, j, start_angle, end_angle)) {oled_draw_point(origin_x + y, origin_y + j);}
            }
        }
    }
}
