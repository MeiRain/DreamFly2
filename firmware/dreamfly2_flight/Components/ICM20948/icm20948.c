//
// Created by 梅雨 on 2026/10/2.
//

#include "icm20948.h"
#include "icm20948_reg.h"

uint8_t ICM20948_Check(void)
{
    uint8_t ID;

    ICM20948_SelectBank(USER_BANK_0);
    ICM20948_ReadByte(B0_WHO_AM_I, &ID);

    if (ID == ICM20948_DEVID)
        return 1;
    else
        return 0;
}

// 设置低通滤波器带宽（带宽 <= 采样率 / 2）// 最低分辨率 = 2 ^ 15 / 4g = 8192LSB/g ）+-2000°/s（最低分辨率 = 2 ^ 15 / 2000 = 16.384°/s）
void ICM20948_Init(void)
{
    ICM20948_SelectBank(USER_BANK_0); // 选择 USER BANK 0 Register
    ICM20948_WriteByte(B0_PWR_MGMT_1, 0x80); // 复位 ICM20948
    Delay_ms(100);

    ICM20948_Check();

    ICM20948_WriteByte(B0_PWR_MGMT_1, 0x01); // 唤醒 ICM20948，并时选择钟源
    Delay_ms(100);
    ICM20948_WriteByte(B0_USER_CTRL, 0x20); // 使能 ICM20948 的 I2C 主模式

    ICM20948_SelectBank(USER_BANK_2); // 选择 USER BANK 2 Register
    ICM20948_WriteByte(B0_INT_ENABLE, 0x00); // 关闭 ICM20948 所有中断
    ICM20948_WriteByte(B2_GYRO_SMPLRT_DIV, 0x02); // 陀螺仪采样频率 1.1 KHz / (1 + GYRO_SMPLRT_DIV)
    ICM20948_WriteByte(B2_GYRO_CONFIG_1, BIT_GYRO_DLPCFG_3 | BIT_GYRO_FS_2000DPS | BIT_GYRO_DLPF); // 设置低通滤波器带宽和陀螺仪满量程
    ICM20948_WriteByte(B2_ACCEL_SMPLRT_DIV_2, 0x02); // 加速度计采样频率 1.125 kHz/(1+ACCEL_SMPLRT_DIV)
    ICM20948_WriteByte(B2_ACCEL_CONFIG, BIT_ACCEL_DLPCFG_3 | BIT_ACCEL_FS_4g | BIT_ACCEL_DLPF); // 设置低通滤波器带宽和加速度满量程
    ICM20948_WriteByte(B2_ODR_ALIGN_EN, 0x01); // 启用 ODR 开始时间对齐

    ICM20948_SelectBank(USER_BANK_3); // 选择 USER BANK 0 Register
    ICM20948_WriteByte(B3_I2C_MST_CTRL, 0x07); // 设置 I2C 主模式速度为 400 kHz
}

/******************************************************************************
 *函  数：void ICM20948_GetAccel(int16_t *accData)
 *功  能：读取加速度的原始数据
 *参  数：*accData 原始数据的指针
 *返回值：无
 *备  注：无
 *******************************************************************************/
void ICM20948_GetAccel(int16_t* ax, int16_t* ay, int16_t* az)
{
    uint8_t Buffer[6];
    ICM20948_SelectBank(USER_BANK_0);
    ICM20948_ReadBuffer(B0_ACCEL_XOUT_H, Buffer, 6);
    *ax = (int16_t)((Buffer[0] << 8) | Buffer[1]);
    *ay = (int16_t)((Buffer[2] << 8) | Buffer[3]);
    *az = (int16_t)((Buffer[4] << 8) | Buffer[5]);
}

/******************************************************************************
 *函  数：void ICM20948_GetGyro(int16_t *gyroData)
 *功  能：读取陀螺仪的原始数据
 *参  数：*gyroData 原始数据的指针
 *返回值：无
 *备  注：无
 *******************************************************************************/
void ICM20948_GetGyro(int16_t* gx, int16_t* gy, int16_t* gz)
{
    uint8_t Buffer[6];
    ICM20948_SelectBank(USER_BANK_0);
    ICM20948_ReadBuffer(B0_GYRO_XOUT_H, Buffer, 6);
    *gx = (int16_t)((Buffer[0] << 8) | Buffer[1]);
    *gy = (int16_t)((Buffer[2] << 8) | Buffer[3]);
    *gz = (int16_t)((Buffer[4] << 8) | Buffer[5]);
}

void ICM20948_ReadByte(uint8_t reg_address, uint8_t* pByte)
{
    I2C1_ReadByte(ICM20948_ADDR, reg_address, pByte);
}

void ICM20948_WriteByte(uint8_t reg_address, uint8_t Byte)
{
    I2C1_WriteByte(ICM20948_ADDR, reg_address, Byte);
}

void ICM20948_ReadBuffer(uint8_t reg_address, uint8_t* pBuffer, uint16_t NumByteToRead)
{
    I2C1_ReadBuffer(ICM20948_ADDR, reg_address, pBuffer, NumByteToRead);
}

void ICM20948_WriteBuffer(uint8_t reg_address, uint8_t* pBuffer, uint16_t NumByteToWrite)
{
    I2C1_WriteBuffer(ICM20948_ADDR, reg_address, pBuffer, NumByteToWrite);
}

void ICM20948_SelectBank(uint8_t bank)
{
    ICM20948_WriteByte(REG_BANK_SEL, bank);
}
