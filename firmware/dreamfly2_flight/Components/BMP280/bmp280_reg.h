/**
 * @file bmp280_reg.h
 * @brief BMP280 register addresses and configuration values.
 * @author 梅雨
 * @date 2026-10-11
 */

#ifndef DREAMFLY2_COMPONENTS_BMP280_REG_H
#define DREAMFLY2_COMPONENTS_BMP280_REG_H

/* Device constants */
#define BMP280_ADDR                    (0x76U << 1U) /* HAL 8-bit I2C address, SDO low. */
#define BMP280_DEVID                   0x58U
#define BMP280_RESET_VALUE             0xB6U

/* ======================== Register addresses ======================== */
/* The values in this section are register addresses. */

/* Factory calibration data */
#define DIG_T1_LSB                     0x88U
#define DIG_T1_MSB                     0x89U
#define DIG_T2_LSB                     0x8AU
#define DIG_T2_MSB                     0x8BU
#define DIG_T3_LSB                     0x8CU
#define DIG_T3_MSB                     0x8DU
#define DIG_P1_LSB                     0x8EU
#define DIG_P1_MSB                     0x8FU
#define DIG_P2_LSB                     0x90U
#define DIG_P2_MSB                     0x91U
#define DIG_P3_LSB                     0x92U
#define DIG_P3_MSB                     0x93U
#define DIG_P4_LSB                     0x94U
#define DIG_P4_MSB                     0x95U
#define DIG_P5_LSB                     0x96U
#define DIG_P5_MSB                     0x97U
#define DIG_P6_LSB                     0x98U
#define DIG_P6_MSB                     0x99U
#define DIG_P7_LSB                     0x9AU
#define DIG_P7_MSB                     0x9BU
#define DIG_P8_LSB                     0x9CU
#define DIG_P8_MSB                     0x9DU
#define DIG_P9_LSB                     0x9EU
#define DIG_P9_MSB                     0x9FU

/* Identification, control and measurement data */
#define ID                             0xD0U
#define RESET_REG                      0xE0U
#define STATUS                         0xF3U
#define CTRL_MEAS                      0xF4U
#define CONFIG                         0xF5U
#define PRESS_MSB                      0xF7U
#define PRESS_LSB                      0xF8U
#define PRESS_XLSB                     0xF9U
#define TEMP_MSB                       0xFAU
#define TEMP_LSB                       0xFBU
#define TEMP_XLSB                      0xFCU

/* ==================== Register configuration values ==================== */
/* These values configure register fields; they are not register addresses. */

/* STATUS */
#define MEASURING                      0x08U /* Bit 3: a conversion is running. */
#define IM_UPDATE                      0x01U /* Bit 0: calibration data are being copied. */

/* CTRL_MEAS bits 7:5: temperature oversampling. */
#define OSRS_T_SKIPPED                 0x00U
#define OSRS_T_X1                      0x20U
#define OSRS_T_X2                      0x40U
#define OSRS_T_X4                      0x60U
#define OSRS_T_X8                      0x80U
#define OSRS_T_X16                     0xA0U

/* CTRL_MEAS bits 4:2: pressure oversampling. */
#define OSRS_P_SKIPPED                 0x00U
#define OSRS_P_X1                      0x04U
#define OSRS_P_X2                      0x08U
#define OSRS_P_X4                      0x0CU
#define OSRS_P_X8                      0x10U
#define OSRS_P_X16                     0x14U

/* CTRL_MEAS bits 1:0: power mode. */
#define SLEEP_MODE                     0x00U
#define FORCED_MODE                    0x01U
#define NORMAL_MODE                    0x03U

/* CONFIG bits 7:5: inactive duration in normal mode. */
#define T_SB_0_5_MS                    0x00U
#define T_SB_62_5_MS                   0x20U
#define T_SB_125_MS                    0x40U
#define T_SB_250_MS                    0x60U
#define T_SB_500_MS                    0x80U
#define T_SB_1000_MS                   0xA0U
#define T_SB_2000_MS                   0xC0U
#define T_SB_4000_MS                   0xE0U

/* CONFIG bits 4:2: internal IIR filter coefficient. */
#define FILTER_OFF                     0x00U
#define FILTER_COEFF_2                 0x04U
#define FILTER_COEFF_4                 0x08U
#define FILTER_COEFF_8                 0x0CU
#define FILTER_COEFF_16                0x10U

/* CONFIG bit 0: 3-wire SPI mode. */
#define SPI3W_DISABLE                  0x00U
#define SPI3W_ENABLE                   0x01U

#endif /* DREAMFLY2_COMPONENTS_BMP280_REG_H */
