/**
 * @file ak09916.c
 * @brief AK09916 driver through the ICM20948 auxiliary I2C master.
 * @author 梅雨
 * @date 2026-10-11
 */

#include "ak09916.h"

#include "stm32f4xx_hal.h"

#include "ak09916_reg.h"
#include "icm20948.h"

#define AK09916_RESET_DELAY_MS 1U
#define AK09916_FIRST_SAMPLE_DELAY_MS 10U
#define AK09916_SAMPLE_SIZE 9U

uint8_t ak09916_read_id(void)
{
    uint8_t device_id = 0U;

    (void)icm20948_aux_read(AK09916_ADDR, WIA2, &device_id, 1U);
    return device_id;
}

bool ak09916_init(void)
{
    if (!icm20948_aux_write(AK09916_ADDR, CNTL3, SOFT_RESET))
    {
        return false;
    }

    HAL_Delay(AK09916_RESET_DELAY_MS);

    if (ak09916_read_id() != AK09916_DEVID)
    {
        return false;
    }

    if (!icm20948_aux_write(AK09916_ADDR,
                            CNTL2,
                            CONTINUOUS_MODE_100HZ))
    {
        return false;
    }

    HAL_Delay(AK09916_FIRST_SAMPLE_DELAY_MS);
    return true;
}

bool ak09916_read(ak09916_t *mag_data)
{
    uint8_t buffer[AK09916_SAMPLE_SIZE];
    int16_t raw_mag_x;
    int16_t raw_mag_y;
    int16_t raw_mag_z;

    if ((mag_data == 0)
        || !icm20948_aux_read(AK09916_ADDR,
                              ST1,
                              buffer,
                              sizeof(buffer))
        || ((buffer[0] & DATA_READY) == 0U)
        || ((buffer[8] & MAGNETIC_OVERFLOW) != 0U))
    {
        return false;
    }

    raw_mag_x = (int16_t)(((uint16_t)buffer[2] << 8U) | buffer[1]);
    raw_mag_y = (int16_t)(((uint16_t)buffer[4] << 8U) | buffer[3]);
    raw_mag_z = (int16_t)(((uint16_t)buffer[6] << 8U) | buffer[5]);

    mag_data->mag_x_ut = (float)raw_mag_x * AK09916_SENSITIVITY_UT;
    mag_data->mag_y_ut = (float)raw_mag_y * AK09916_SENSITIVITY_UT;
    mag_data->mag_z_ut = (float)raw_mag_z * AK09916_SENSITIVITY_UT;

    return true;
}
