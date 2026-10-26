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
#define ICM20948_AUX_TRANSFER_DELAY_MS 1U
#define ICM20948_ACC_LSB_PER_G 8192.0f
#define ICM20948_GYRO_LSB_PER_DPS 16.4f

/* Cached bank selection; all register access must remain serialized. */
static uint8_t s_current_bank = ICM20948_INVALID_BANK;

/** @brief Write one byte in the currently selected bank using blocking I2C. */
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

/** @brief Read one byte in the currently selected bank; value must be writable. */
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

/** @brief Read consecutive registers into a buffer with at least size bytes. */
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

/** @brief Select a register bank, updating the cache only after a successful write. */
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

bool icm20948_read(icm20948_t *imu_data)
{
    uint8_t buffer[12];
    int16_t raw_acc_x;
    int16_t raw_acc_y;
    int16_t raw_acc_z;
    int16_t raw_gyro_x;
    int16_t raw_gyro_y;
    int16_t raw_gyro_z;

    if ((imu_data == 0)
        || !icm20948_select_bank(USER_BANK_0)
        || !icm20948_read_registers(ACCEL_XOUT_H,
                                    buffer,
                                    sizeof(buffer)))
    {
        return false;
    }

    raw_acc_x = (int16_t)(((uint16_t)buffer[0] << 8U) | buffer[1]);
    raw_acc_y = (int16_t)(((uint16_t)buffer[2] << 8U) | buffer[3]);
    raw_acc_z = (int16_t)(((uint16_t)buffer[4] << 8U) | buffer[5]);
    raw_gyro_x = (int16_t)(((uint16_t)buffer[6] << 8U) | buffer[7]);
    raw_gyro_y = (int16_t)(((uint16_t)buffer[8] << 8U) | buffer[9]);
    raw_gyro_z = (int16_t)(((uint16_t)buffer[10] << 8U) | buffer[11]);

    imu_data->acc_x_g = (float)raw_acc_x / ICM20948_ACC_LSB_PER_G;
    imu_data->acc_y_g = (float)raw_acc_y / ICM20948_ACC_LSB_PER_G;
    imu_data->acc_z_g = (float)raw_acc_z / ICM20948_ACC_LSB_PER_G;
    imu_data->gyro_x_dps = (float)raw_gyro_x / ICM20948_GYRO_LSB_PER_DPS;
    imu_data->gyro_y_dps = (float)raw_gyro_y / ICM20948_GYRO_LSB_PER_DPS;
    imu_data->gyro_z_dps = (float)raw_gyro_z / ICM20948_GYRO_LSB_PER_DPS;

    return true;
}

bool icm20948_aux_write(uint8_t device_address,
                        uint8_t register_address,
                        uint8_t value)
{
    if ((device_address > 0x7FU)
        || !icm20948_select_bank(USER_BANK_3)
        || !icm20948_write_register(I2C_SLV0_ADDR, device_address)
        || !icm20948_write_register(I2C_SLV0_REG, register_address)
        || !icm20948_write_register(I2C_SLV0_DO, value)
        || !icm20948_write_register(I2C_SLV0_CTRL,
                                    I2C_SLV_ENABLE | 1U))
    {
        return false;
    }

    HAL_Delay(ICM20948_AUX_TRANSFER_DELAY_MS);

    return icm20948_write_register(I2C_SLV0_CTRL, 0U)
           && icm20948_select_bank(USER_BANK_0);
}

bool icm20948_aux_read(uint8_t device_address,
                       uint8_t register_address,
                       uint8_t *data,
                       uint8_t size)
{
    bool read_success;

    if ((data == 0)
        || (device_address > 0x7FU)
        || (size == 0U)
        || (size > I2C_SLV_LENGTH_MASK)
        || !icm20948_select_bank(USER_BANK_3)
        || !icm20948_write_register(I2C_SLV0_ADDR,
                                    device_address | I2C_SLV_READ)
        || !icm20948_write_register(I2C_SLV0_REG, register_address)
        || !icm20948_write_register(I2C_SLV0_CTRL,
                                    I2C_SLV_ENABLE | size))
    {
        return false;
    }

    HAL_Delay(ICM20948_AUX_TRANSFER_DELAY_MS);

    read_success = icm20948_select_bank(USER_BANK_0)
                   && icm20948_read_registers(EXT_SLV_SENS_DATA_00,
                                              data,
                                              size);

    if (!icm20948_select_bank(USER_BANK_3)
        || !icm20948_write_register(I2C_SLV0_CTRL, 0U)
        || !icm20948_select_bank(USER_BANK_0))
    {
        return false;
    }

    return read_success;
}
