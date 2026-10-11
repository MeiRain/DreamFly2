/**
 * @file icm20948_reg.h
 * @brief ICM-20948 register addresses and bit definitions.
 * @author 梅雨
 * @date 2026-10-10
 */

#ifndef DREAMFLY2_COMPONENTS_ICM20948_REG_H
#define DREAMFLY2_COMPONENTS_ICM20948_REG_H

/* Device constants */
#define ICM20948_ADDR                  (0x69U << 1U) /* HAL 8-bit I2C address, AD0 high. */
#define ICM20948_DEVID                 0xEAU         /* Expected WHO_AM_I value. */
#define ICM20948_INVALID_BANK          0xFFU         /* Software-only invalid bank state. */

/* ======================== Register addresses ======================== */
/* The values in this section are register addresses. */

/* Register bank selection */
#define REG_BANK_SEL                   0x7FU

/* Values written to REG_BANK_SEL; these are not register addresses. */
#define USER_BANK_0                    0x00U
#define USER_BANK_1                    0x10U
#define USER_BANK_2                    0x20U
#define USER_BANK_3                    0x30U

/* User Bank 0 */
#define WHO_AM_I                       0x00U
#define USER_CTRL                      0x03U
#define LP_CONFIG                      0x05U
#define PWR_MGMT_1                     0x06U
#define PWR_MGMT_2                     0x07U
#define INT_PIN_CFG                    0x0FU
#define INT_ENABLE                     0x10U
#define INT_ENABLE_1                   0x11U
#define INT_ENABLE_2                   0x12U
#define INT_ENABLE_3                   0x13U
#define I2C_MST_STATUS                 0x17U
#define INT_STATUS                     0x19U
#define INT_STATUS_1                   0x1AU
#define INT_STATUS_2                   0x1BU
#define INT_STATUS_3                   0x1CU
#define DELAY_TIMEH                    0x28U
#define DELAY_TIMEL                    0x29U
#define ACCEL_XOUT_H                   0x2DU
#define ACCEL_XOUT_L                   0x2EU
#define ACCEL_YOUT_H                   0x2FU
#define ACCEL_YOUT_L                   0x30U
#define ACCEL_ZOUT_H                   0x31U
#define ACCEL_ZOUT_L                   0x32U
#define GYRO_XOUT_H                    0x33U
#define GYRO_XOUT_L                    0x34U
#define GYRO_YOUT_H                    0x35U
#define GYRO_YOUT_L                    0x36U
#define GYRO_ZOUT_H                    0x37U
#define GYRO_ZOUT_L                    0x38U
#define TEMP_OUT_H                     0x39U
#define TEMP_OUT_L                     0x3AU
#define EXT_SLV_SENS_DATA_00           0x3BU
#define EXT_SLV_SENS_DATA_01           0x3CU
#define EXT_SLV_SENS_DATA_02           0x3DU
#define EXT_SLV_SENS_DATA_03           0x3EU
#define EXT_SLV_SENS_DATA_04           0x3FU
#define EXT_SLV_SENS_DATA_05           0x40U
#define EXT_SLV_SENS_DATA_06           0x41U
#define EXT_SLV_SENS_DATA_07           0x42U
#define EXT_SLV_SENS_DATA_08           0x43U
#define EXT_SLV_SENS_DATA_09           0x44U
#define EXT_SLV_SENS_DATA_10           0x45U
#define EXT_SLV_SENS_DATA_11           0x46U
#define EXT_SLV_SENS_DATA_12           0x47U
#define EXT_SLV_SENS_DATA_13           0x48U
#define EXT_SLV_SENS_DATA_14           0x49U
#define EXT_SLV_SENS_DATA_15           0x4AU
#define EXT_SLV_SENS_DATA_16           0x4BU
#define EXT_SLV_SENS_DATA_17           0x4CU
#define EXT_SLV_SENS_DATA_18           0x4DU
#define EXT_SLV_SENS_DATA_19           0x4EU
#define EXT_SLV_SENS_DATA_20           0x4FU
#define EXT_SLV_SENS_DATA_21           0x50U
#define EXT_SLV_SENS_DATA_22           0x51U
#define EXT_SLV_SENS_DATA_23           0x52U
#define FIFO_EN_1                      0x66U
#define FIFO_EN_2                      0x67U
#define FIFO_RST                       0x68U
#define FIFO_MODE                      0x69U
#define FIFO_COUNTH                    0x70U
#define FIFO_COUNTL                    0x71U
#define FIFO_R_W                       0x72U
#define DATA_RDY_STATUS                0x74U
#define FIFO_CFG                       0x76U

/* User Bank 1 */
#define SELF_TEST_X_GYRO               0x02U
#define SELF_TEST_Y_GYRO               0x03U
#define SELF_TEST_Z_GYRO               0x04U
#define SELF_TEST_X_ACCEL              0x0EU
#define SELF_TEST_Y_ACCEL              0x0FU
#define SELF_TEST_Z_ACCEL              0x10U
#define XA_OFFS_H                      0x14U
#define XA_OFFS_L                      0x15U
#define YA_OFFS_H                      0x17U
#define YA_OFFS_L                      0x18U
#define ZA_OFFS_H                      0x1AU
#define ZA_OFFS_L                      0x1BU
#define TIMEBASE_CORRECTION_PLL        0x28U

/* User Bank 2 */
#define GYRO_SMPLRT_DIV                0x00U
#define GYRO_CONFIG_1                  0x01U
#define GYRO_CONFIG_2                  0x02U
#define XG_OFFS_USRH                   0x03U
#define XG_OFFS_USRL                   0x04U
#define YG_OFFS_USRH                   0x05U
#define YG_OFFS_USRL                   0x06U
#define ZG_OFFS_USRH                   0x07U
#define ZG_OFFS_USRL                   0x08U
#define ODR_ALIGN_EN                   0x09U
#define ACCEL_SMPLRT_DIV_1             0x10U
#define ACCEL_SMPLRT_DIV_2             0x11U
#define ACCEL_INTEL_CTRL               0x12U
#define ACCEL_WOM_THR                  0x13U
#define ACCEL_CONFIG                   0x14U
#define ACCEL_CONFIG_2                 0x15U
#define FSYNC_CONFIG                   0x52U
#define TEMP_CONFIG                    0x53U
#define MOD_CTRL_USR                   0x54U

/* User Bank 3 */
#define I2C_MST_ODR_CONFIG             0x00U
#define I2C_MST_CTRL                   0x01U
#define I2C_MST_DELAY_CTRL             0x02U
#define I2C_SLV0_ADDR                  0x03U
#define I2C_SLV0_REG                   0x04U
#define I2C_SLV0_CTRL                  0x05U
#define I2C_SLV0_DO                    0x06U
#define I2C_SLV1_ADDR                  0x07U
#define I2C_SLV1_REG                   0x08U
#define I2C_SLV1_CTRL                  0x09U
#define I2C_SLV1_DO                    0x0AU
#define I2C_SLV2_ADDR                  0x0BU
#define I2C_SLV2_REG                   0x0CU
#define I2C_SLV2_CTRL                  0x0DU
#define I2C_SLV2_DO                    0x0EU
#define I2C_SLV3_ADDR                  0x0FU
#define I2C_SLV3_REG                   0x10U
#define I2C_SLV3_CTRL                  0x11U
#define I2C_SLV3_DO                    0x12U
#define I2C_SLV4_ADDR                  0x13U
#define I2C_SLV4_REG                   0x14U
#define I2C_SLV4_CTRL                  0x15U
#define I2C_SLV4_DO                    0x16U
#define I2C_SLV4_DI                    0x17U

/* ==================== Register configuration values ==================== */
/*
 * The values below configure fields inside registers; they are not
 * register addresses. Values for different fields in the same register
 * may be combined with the bitwise OR operator.
 */

/* PWR_MGMT_1: power management and clock source. */
#define DEVICE_RESET                   0x80U /* Bit 7: reset all registers. */
#define CLKSEL_AUTO                    0x01U /* Bits 2:0: automatically select the best clock. */

/* USER_CTRL: internal I2C master control. */
#define I2C_MST_EN                     0x20U /* Bit 5: enable internal I2C master. */

/* GYRO_CONFIG_1 bits 5:3: gyroscope digital low-pass filter setting. */
#define GYRO_DLPFCFG_0                 0x00U
#define GYRO_DLPFCFG_1                 0x08U
#define GYRO_DLPFCFG_2                 0x10U
#define GYRO_DLPFCFG_3                 0x18U
#define GYRO_DLPFCFG_4                 0x20U
#define GYRO_DLPFCFG_5                 0x28U
#define GYRO_DLPFCFG_6                 0x30U
#define GYRO_DLPFCFG_7                 0x38U

/* GYRO_CONFIG_1 bits 2:1: gyroscope full-scale range. */
#define GYRO_FS_SEL_250DPS             0x00U
#define GYRO_FS_SEL_500DPS             0x02U
#define GYRO_FS_SEL_1000DPS            0x04U
#define GYRO_FS_SEL_2000DPS            0x06U
#define GYRO_FCHOICE_DLPF              0x01U /* Bit 0: use the gyroscope DLPF path. */

/* ACCEL_CONFIG bits 5:3: accelerometer digital low-pass filter setting. */
#define ACCEL_DLPFCFG_0                0x00U
#define ACCEL_DLPFCFG_1                0x08U
#define ACCEL_DLPFCFG_2                0x10U
#define ACCEL_DLPFCFG_3                0x18U
#define ACCEL_DLPFCFG_4                0x20U
#define ACCEL_DLPFCFG_5                0x28U
#define ACCEL_DLPFCFG_6                0x30U
#define ACCEL_DLPFCFG_7                0x38U

/* ACCEL_CONFIG bits 2:1: accelerometer full-scale range. */
#define ACCEL_FS_SEL_2G                0x00U
#define ACCEL_FS_SEL_4G                0x02U
#define ACCEL_FS_SEL_8G                0x04U
#define ACCEL_FS_SEL_16G               0x06U
#define ACCEL_FCHOICE_DLPF             0x01U /* Bit 0: use the accelerometer DLPF path. */

/* I2C_MST_CTRL bits 3:0: internal I2C master clock setting. */
#define I2C_MST_CLK_400KHZ             0x07U

/* I2C_SLVx_ADDR: auxiliary device transfer direction. */
#define I2C_SLV_READ                   0x80U

/* I2C_SLVx_CTRL: transfer enable and data length. */
#define I2C_SLV_ENABLE                 0x80U
#define I2C_SLV_LENGTH_MASK            0x0FU

#endif /* DREAMFLY2_COMPONENTS_ICM20948_REG_H */
