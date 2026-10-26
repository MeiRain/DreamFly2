/**
 * @file oled.h
 * @brief SSD1306 128x64 OLED display interface.
 * @author 梅雨
 * @date 2026-10-24
 *
 * Drawing operations change only the private 1024-byte buffer.
 * Call oled_update() or oled_update_area() to send it to the display.
 * Pixel coordinates: top-left (0,0), x right, y down.
 * Use one task for this non-reentrant driver. It contains no UI or key logic.
 */

#ifndef DREAMFLY2_REMOTE_COMPONENTS_OLED_H
#define DREAMFLY2_REMOTE_COMPONENTS_OLED_H

#include <stdbool.h>
#include <stdint.h>

#define OLED_WIDTH    128
#define OLED_HEIGHT   64
#define OLED_8X16     8U
#define OLED_6X8      6U
#define OLED_UNFILLED 0U
#define OLED_FILLED   1U

/**
 * @brief Initialize the display and clear its RAM.
 * @return true on success, false on an I2C error.
 * @note Call after MX_I2C1_Init(); uses I2C1 and address 0x3C (7-bit).
 *       Includes a blocking 100 ms power-up delay. No GPIO configuration is changed.
 */
bool oled_init(void);

/**
 * @brief Send the complete drawing buffer to the panel.
 * @return true on success, false on the first I2C error.
 * @note Uses blocking HAL transfers; call from a display task, not an ISR.
 */
bool oled_update(void);

/**
 * @brief Refresh a clipped rectangular region, rounded vertically to whole pages.
 * @param x Left pixel coordinate.
 * @param y Top pixel coordinate.
 * @param width Region width in pixels.
 * @param height Region height in pixels.
 * @return true on success or an empty region, false on an I2C error.
 */
bool oled_update_area(int16_t x, int16_t y, uint8_t width, uint8_t height);

/** @brief Clear the drawing buffer without refreshing the panel. */
void oled_clear(void);

/** @brief Clear a rectangle in the buffer; dimensions are pixels. */
void oled_clear_area(int16_t x, int16_t y, uint8_t width, uint8_t height);

/** @brief Invert every pixel in the drawing buffer. */
void oled_reverse(void);

/** @brief Invert a rectangle in the buffer; dimensions are pixels. */
void oled_reverse_area(int16_t x, int16_t y, uint8_t width, uint8_t height);

/**
 * @brief Draw a printable ASCII character at (x,y); invalid characters become '?'.
 * @param font_size OLED_6X8 or OLED_8X16.
 */
void oled_show_char(int16_t x, int16_t y, char character, uint8_t font_size);

/**
 * @brief Draw a UTF-8 string at (x,y), without line wrapping.
 * @param text Null-terminated text; NULL is ignored.
 * @param font_size OLED_6X8 or OLED_8X16.
 * @note Chinese glyphs must exist in oledfont.c. They occupy 16x16 pixels with
 *       OLED_8X16; OLED_6X8 uses '?' instead. Missing glyphs use a placeholder.
 */
void oled_show_string(int16_t x, int16_t y, const char *text, uint8_t font_size);

/**
 * @brief Draw zero-padded decimal digits; excess high digits are discarded.
 * @param length Digit count, clamped to 0..10.
 */
void oled_show_num(int16_t x, int16_t y, uint32_t number, uint8_t length, uint8_t font_size);

/** @brief Draw a sign followed by 0..10 zero-padded decimal digits. */
void oled_show_signed_num(int16_t x, int16_t y, int32_t number, uint8_t length, uint8_t font_size);

/** @brief Draw 0..8 zero-padded hexadecimal digits, using uppercase A..F. */
void oled_show_hex_num(int16_t x, int16_t y, uint32_t number, uint8_t length, uint8_t font_size);

/** @brief Draw 0..32 zero-padded binary digits. */
void oled_show_bin_num(int16_t x, int16_t y, uint32_t number, uint8_t length, uint8_t font_size);

/**
 * @brief Draw a sign and a rounded decimal value.
 * @param int_length Integer digit count, clamped to 0..10.
 * @param fra_length Fractional digit count, clamped to 0..6.
 * @note Invalid or out-of-range values show "ERR". This function does not
 *       require printf floating-point support; high integer digits are discarded.
 */
void oled_show_float_num(int16_t x, int16_t y, double number, uint8_t int_length,
                         uint8_t fra_length, uint8_t font_size);

/**
 * @brief Copy a monochrome bitmap into the drawing buffer, clipped to the screen.
 * @param image At least width * ((height + 7) / 8) bytes; NULL is ignored.
 * @note Layout is page-major, vertical groups of eight pixels, LSB at the top.
 */
void oled_show_image(int16_t x, int16_t y, uint8_t width, uint8_t height, const uint8_t *image);

/**
 * @brief Draw formatted UTF-8 text; output is limited to 127 bytes.
 * @note Uses a 128-byte local buffer plus the C library's formatting stack.
 *       %f requires floating-point printf support in the linker configuration.
 */
void oled_printf(int16_t x, int16_t y, uint8_t font_size, const char *format, ...);

/** @brief Set one buffer pixel; off-screen coordinates are ignored. */
void oled_draw_point(int16_t x, int16_t y);

/** @brief Return a buffer pixel (0 or 1); off-screen coordinates return 0. */
uint8_t oled_get_point(int16_t x, int16_t y);

/** @brief Draw a line between two pixel coordinates using Bresenham's algorithm. */
void oled_draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1);

/** @brief Draw a rectangle; dimensions are pixels, fill is OLED_FILLED/UNFILLED. */
void oled_draw_rectangle(int16_t x, int16_t y, uint8_t width, uint8_t height, uint8_t is_filled);

/** @brief Draw a triangle; fill is OLED_FILLED or OLED_UNFILLED. */
void oled_draw_triangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                        int16_t x2, int16_t y2, uint8_t is_filled);

/** @brief Draw a circle centered at (x,y); radius is pixels. */
void oled_draw_circle(int16_t x, int16_t y, uint8_t radius, uint8_t is_filled);

/** @brief Draw an ellipse centered at (x,y); radii are pixels. */
void oled_draw_ellipse(int16_t x, int16_t y, uint8_t radius_x, uint8_t radius_y, uint8_t is_filled);

/**
 * @brief Draw an arc centered at (x,y); filled arcs are sectors.
 * @param start_angle Starting angle in degrees, -180..180.
 * @param end_angle Ending angle in degrees, -180..180.
 * @note Zero degrees points right; positive angles rotate clockwise.
 */
void oled_draw_arc(int16_t x, int16_t y, uint8_t radius,
                   int16_t start_angle, int16_t end_angle, uint8_t is_filled);

#endif /* DREAMFLY2_REMOTE_COMPONENTS_OLED_H */
