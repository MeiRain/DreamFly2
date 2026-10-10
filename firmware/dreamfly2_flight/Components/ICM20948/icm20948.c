/**
 * @file icm20948.c
 * @brief ICM-20948 driver using STM32 HAL I2C.
 * @author 梅雨
 * @date 2026-10-10
 */

#include "icm20948.h"

#include "i2c.h"
#include "icm20948_reg.h"

#define ICM20948_I2C_TIMEOUT_MS 2U

static uint8_t s_current_bank = ICM20948_INVALID_BANK;

static bool icm20948_write_register(uint8_t register_address, uint8_t value)
{
    return HAL_I2C_Mem_Write(&hi2c1,
                             ICM20948_ADDR,
                             register_address,
                             I2C_MEMADD_SIZE_8BIT,
                             &value,
                             1U,
                             ICM20948_I2C_TIMEOUT_MS) == HAL_OK;
}

static bool icm20948_read_register(uint8_t register_address, uint8_t *value)
{
    return HAL_I2C_Mem_Read(&hi2c1,
                            ICM20948_ADDR,
                            register_address,
                            I2C_MEMADD_SIZE_8BIT,
                            value,
                            1U,
                            ICM20948_I2C_TIMEOUT_MS) == HAL_OK;
}

static bool icm20948_read_registers(uint8_t register_address,
                                    uint8_t *data,
                                    uint16_t size)
{
    return HAL_I2C_Mem_Read(&hi2c1,
                            ICM20948_ADDR,
                            register_address,
                            I2C_MEMADD_SIZE_8BIT,
                            data,
                            size,
                            ICM20948_I2C_TIMEOUT_MS) == HAL_OK;
}

static bool icm20948_select_bank(uint8_t bank)
{
    if (bank == s_current_bank)
    {
        return true;
    }

    if (!icm20948_write_register(REG_BANK_SEL, bank))
    {
        return false;
    }

    s_current_bank = bank;
    return true;
}

uint8_t icm20948_read_id(void)
{
    uint8_t device_id = 0U;

    if (!icm20948_select_bank(USER_BANK_0))
    {
        return 0U;
    }

    (void)icm20948_read_register(WHO_AM_I, &device_id);
    return device_id;
}

bool icm20948_init(void)
{
    s_current_bank = ICM20948_INVALID_BANK;

    if (!icm20948_select_bank(USER_BANK_0)
        || !icm20948_write_register(PWR_MGMT_1, DEVICE_RESET))
    {
        return false;
    }

    HAL_Delay(100U);
    s_current_bank = ICM20948_INVALID_BANK;

    if (icm20948_read_id() != ICM20948_DEVID)
    {
        return false;
    }

    if (!icm20948_write_register(PWR_MGMT_1, CLKSEL_AUTO))
    {
        return false;
    }

    HAL_Delay(10U);

    if (!icm20948_write_register(USER_CTRL, I2C_MST_EN)
        || !icm20948_select_bank(USER_BANK_2)
        || !icm20948_write_register(GYRO_SMPLRT_DIV, 0x01U)
        || !icm20948_write_register(GYRO_CONFIG_1,
                                    GYRO_DLPFCFG_3
                                    | GYRO_FS_SEL_2000DPS
                                    | GYRO_FCHOICE_DLPF)
        || !icm20948_write_register(ACCEL_SMPLRT_DIV_1, 0x00U)
        || !icm20948_write_register(ACCEL_SMPLRT_DIV_2, 0x01U)
        || !icm20948_write_register(ACCEL_CONFIG,
                                    ACCEL_DLPFCFG_3
                                    | ACCEL_FS_SEL_4G
                                    | ACCEL_FCHOICE_DLPF)
        || !icm20948_write_register(ODR_ALIGN_EN, 0x01U)
        || !icm20948_select_bank(USER_BANK_3)
        || !icm20948_write_register(I2C_MST_CTRL, I2C_MST_CLK_400KHZ)
        || !icm20948_select_bank(USER_BANK_0))
    {
        return false;
    }

    return true;
}

bool icm20948_read(icm20948_t *data)
{
    uint8_t buffer[12];

    if ((data == 0)
        || !icm20948_select_bank(USER_BANK_0)
        || !icm20948_read_registers(ACCEL_XOUT_H,
                                    buffer,
                                    sizeof(buffer)))
    {
        return false;
    }

    data->accel_x = (int16_t)(((uint16_t)buffer[0] << 8U) | buffer[1]);
    data->accel_y = (int16_t)(((uint16_t)buffer[2] << 8U) | buffer[3]);
    data->accel_z = (int16_t)(((uint16_t)buffer[4] << 8U) | buffer[5]);
    data->gyro_x = (int16_t)(((uint16_t)buffer[6] << 8U) | buffer[7]);
    data->gyro_y = (int16_t)(((uint16_t)buffer[8] << 8U) | buffer[9]);
    data->gyro_z = (int16_t)(((uint16_t)buffer[10] << 8U) | buffer[11]);

    return true;
}
