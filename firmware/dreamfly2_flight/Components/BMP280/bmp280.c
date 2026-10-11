/**
 * @file bmp280.c
 * @brief BMP280 driver using STM32 HAL I2C.
 * @author 梅雨
 * @date 2026-10-11
 */

/*
 * Compensation notation follows the BMP280 datasheet:
 * t1-t3 and p1-p9 are factory calibration coefficients;
 * var1 and var2 are intermediate values used by the compensation formulas.
 */

#include "bmp280.h"

#include <stddef.h>

#include "bmp280_reg.h"
#include "i2c.h"

#define BMP280_I2C_TIMEOUT_MS 2U
#define BMP280_RESET_DELAY_MS 2U
#define BMP280_STARTUP_TIMEOUT_MS 10U

typedef struct
{
    uint16_t t1;
    int16_t t2;
    int16_t t3;
    uint16_t p1;
    int16_t p2;
    int16_t p3;
    int16_t p4;
    int16_t p5;
    int16_t p6;
    int16_t p7;
    int16_t p8;
    int16_t p9;
} bmp280_calibration_t;

static bmp280_calibration_t s_calibration;
static int32_t s_temperature_fine;

static bool bmp280_write_register(uint8_t register_address, uint8_t value)
{
    return HAL_I2C_Mem_Write(&hi2c1,
                             BMP280_ADDR,
                             register_address,
                             I2C_MEMADD_SIZE_8BIT,
                             &value,
                             1U,
                             BMP280_I2C_TIMEOUT_MS) == HAL_OK;
}

static bool bmp280_read_register(uint8_t register_address, uint8_t *value)
{
    return HAL_I2C_Mem_Read(&hi2c1,
                            BMP280_ADDR,
                            register_address,
                            I2C_MEMADD_SIZE_8BIT,
                            value,
                            1U,
                            BMP280_I2C_TIMEOUT_MS) == HAL_OK;
}

static bool bmp280_read_registers(uint8_t register_address,
                                  uint8_t *data,
                                  uint16_t size)
{
    return HAL_I2C_Mem_Read(&hi2c1,
                            BMP280_ADDR,
                            register_address,
                            I2C_MEMADD_SIZE_8BIT,
                            data,
                            size,
                            BMP280_I2C_TIMEOUT_MS) == HAL_OK;
}

static bool bmp280_read_uint16(uint8_t register_address, uint16_t *value)
{
    uint8_t data[2];

    if ((value == NULL)
        || !bmp280_read_registers(register_address, data, sizeof(data)))
    {
        return false;
    }

    *value = (uint16_t)(((uint16_t)data[1] << 8U) | data[0]);
    return true;
}

static bool bmp280_read_int16(uint8_t register_address, int16_t *value)
{
    uint16_t raw_value;

    if ((value == NULL) || !bmp280_read_uint16(register_address, &raw_value))
    {
        return false;
    }

    *value = (int16_t)raw_value;
    return true;
}

static bool bmp280_read_calibration(void)
{
    return bmp280_read_uint16(DIG_T1_LSB, &s_calibration.t1)
           && bmp280_read_int16(DIG_T2_LSB, &s_calibration.t2)
           && bmp280_read_int16(DIG_T3_LSB, &s_calibration.t3)
           && bmp280_read_uint16(DIG_P1_LSB, &s_calibration.p1)
           && bmp280_read_int16(DIG_P2_LSB, &s_calibration.p2)
           && bmp280_read_int16(DIG_P3_LSB, &s_calibration.p3)
           && bmp280_read_int16(DIG_P4_LSB, &s_calibration.p4)
           && bmp280_read_int16(DIG_P5_LSB, &s_calibration.p5)
           && bmp280_read_int16(DIG_P6_LSB, &s_calibration.p6)
           && bmp280_read_int16(DIG_P7_LSB, &s_calibration.p7)
           && bmp280_read_int16(DIG_P8_LSB, &s_calibration.p8)
           && bmp280_read_int16(DIG_P9_LSB, &s_calibration.p9)
           && (s_calibration.p1 != 0U);
}

static int32_t bmp280_compensate_temperature(int32_t raw_temperature)
{
    int32_t delta;
    int32_t var1;
    int32_t var2;

    var1 = ((((raw_temperature >> 3) - ((int32_t)s_calibration.t1 << 1))
             * (int32_t)s_calibration.t2) >> 11);
    delta = (raw_temperature >> 4) - (int32_t)s_calibration.t1;
    var2 = (((delta * delta) >> 12)
            * (int32_t)s_calibration.t3) >> 14;

    s_temperature_fine = var1 + var2;
    return (s_temperature_fine * 5 + 128) >> 8;
}

static uint32_t bmp280_compensate_pressure(int32_t raw_pressure)
{
    int64_t var1;
    int64_t var2;
    int64_t pressure;

    var1 = (int64_t)s_temperature_fine - 128000;
    var2 = var1 * var1 * (int64_t)s_calibration.p6;
    var2 += (var1 * (int64_t)s_calibration.p5)
            * (((int64_t)1) << 17);
    var2 += (int64_t)s_calibration.p4 * (((int64_t)1) << 35);
    var1 = ((var1 * var1 * (int64_t)s_calibration.p3) >> 8)
           + ((var1 * (int64_t)s_calibration.p2)
              * (((int64_t)1) << 12));
    var1 = (((((int64_t)1) << 47) + var1)
            * (int64_t)s_calibration.p1) >> 33;

    if (var1 == 0)
    {
        return 0U;
    }

    pressure = 1048576 - raw_pressure;
    pressure = (((pressure << 31) - var2) * 3125) / var1;
    var1 = ((int64_t)s_calibration.p9
            * (pressure >> 13) * (pressure >> 13)) >> 25;
    var2 = ((int64_t)s_calibration.p8 * pressure) >> 19;
    pressure = ((pressure + var1 + var2) >> 8)
               + ((int64_t)s_calibration.p7 * 16);

    return (uint32_t)pressure;
}

uint8_t bmp280_read_id(void)
{
    uint8_t device_id = 0U;

    (void)bmp280_read_register(ID, &device_id);
    return device_id;
}

bool bmp280_init(void)
{
    uint8_t status;
    uint8_t elapsed_ms = 0U;

    if ((bmp280_read_id() != BMP280_DEVID)
        || !bmp280_write_register(RESET_REG, BMP280_RESET_VALUE))
    {
        return false;
    }

    HAL_Delay(BMP280_RESET_DELAY_MS);

    do
    {
        if (!bmp280_read_register(STATUS, &status))
        {
            return false;
        }

        if ((status & IM_UPDATE) == 0U)
        {
            break;
        }

        HAL_Delay(1U);
        ++elapsed_ms;
    }
    while (elapsed_ms < BMP280_STARTUP_TIMEOUT_MS);

    if (((status & IM_UPDATE) != 0U) || !bmp280_read_calibration())
    {
        return false;
    }

    if (!bmp280_write_register(CONFIG,
                               T_SB_0_5_MS
                               | FILTER_COEFF_16
                               | SPI3W_DISABLE)
        || !bmp280_write_register(CTRL_MEAS,
                                  OSRS_T_X1
                                  | OSRS_P_X4
                                  | NORMAL_MODE))
    {
        return false;
    }

    return true;
}

bool bmp280_read(bmp280_t *baro_data)
{
    uint8_t buffer[6];
    int32_t raw_pressure;
    int32_t raw_temperature;
    int32_t temperature_centi_c;
    uint32_t pressure_q24_8;

    if ((baro_data == NULL)
        || !bmp280_read_registers(PRESS_MSB, buffer, sizeof(buffer)))
    {
        return false;
    }

    raw_pressure = ((int32_t)buffer[0] << 12)
                   | ((int32_t)buffer[1] << 4)
                   | ((int32_t)buffer[2] >> 4);
    raw_temperature = ((int32_t)buffer[3] << 12)
                      | ((int32_t)buffer[4] << 4)
                      | ((int32_t)buffer[5] >> 4);

    temperature_centi_c = bmp280_compensate_temperature(raw_temperature);
    pressure_q24_8 = bmp280_compensate_pressure(raw_pressure);

    if (pressure_q24_8 == 0U)
    {
        return false;
    }

    baro_data->temp_c = (float)temperature_centi_c / 100.0f;
    baro_data->press_pa = (float)pressure_q24_8 / 256.0f;
    return true;
}
