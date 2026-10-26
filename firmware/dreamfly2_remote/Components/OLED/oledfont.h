/**
 * @file oledfont.h
 * @brief Read-only glyph and bitmap data used by the OLED driver.
 * @date 2026-10-24
 * @note Font data adapted from Jiangxie Technology; original copyright retained.
 */

#ifndef DREAMFLY2_REMOTE_COMPONENTS_OLEDFONT_H
#define DREAMFLY2_REMOTE_COMPONENTS_OLEDFONT_H

#include <stdint.h>

typedef struct
{
    char index[5];     /**< One UTF-8 character plus the null terminator. */
    uint8_t data[32];  /**< 16x16 page-major monochrome glyph. */
} oled_glyph_t;

extern const uint8_t oled_font_8x16[][16];
extern const uint8_t oled_font_6x8[][6];
extern const oled_glyph_t oled_font_chinese[];

extern const uint8_t oled_icon_diode[];
extern const uint8_t oled_icon_signal_0[];
extern const uint8_t oled_icon_signal_1[];
extern const uint8_t oled_icon_signal_2[];
extern const uint8_t oled_icon_signal_3[];

#endif /* DREAMFLY2_REMOTE_COMPONENTS_OLEDFONT_H */
