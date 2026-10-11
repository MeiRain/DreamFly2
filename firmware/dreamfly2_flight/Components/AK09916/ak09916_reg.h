/**
 * @file ak09916_reg.h
 * @brief AK09916 register addresses and configuration values.
 * @author 梅雨
 * @date 2026-10-11
 */

#ifndef DREAMFLY2_COMPONENTS_AK09916_REG_H
#define DREAMFLY2_COMPONENTS_AK09916_REG_H

/* Device constants */
#define AK09916_ADDR                   0x0CU /* 7-bit auxiliary I2C address. */
#define AK09916_DEVID                  0x09U /* Expected WIA2 value. */
#define AK09916_SENSITIVITY_UT         0.15f /* Microtesla per LSB. */

/* ======================== Register addresses ======================== */
/* The values in this section are register addresses. */

#define WIA1                           0x00U
#define WIA2                           0x01U
#define RSV1                           0x02U
#define RSV2                           0x03U
#define ST1                            0x10U
#define HXL                            0x11U
#define HXH                            0x12U
#define HYL                            0x13U
#define HYH                            0x14U
#define HZL                            0x15U
#define HZH                            0x16U
#define TMPS                           0x17U
#define ST2                            0x18U
#define CNTL2                          0x31U
#define CNTL3                          0x32U
#define TS1                            0x33U
#define TS2                            0x34U

/* ==================== Register configuration values ==================== */
/* These values configure register fields; they are not register addresses. */

/* ST1 */
#define DATA_READY                     0x01U
#define DATA_OVERRUN                   0x02U

/* ST2 */
#define MAGNETIC_OVERFLOW              0x08U

/* CNTL2 operating modes */
#define POWER_DOWN_MODE                0x00U
#define SINGLE_MEASUREMENT_MODE        0x01U
#define CONTINUOUS_MODE_10HZ           0x02U
#define CONTINUOUS_MODE_20HZ           0x04U
#define CONTINUOUS_MODE_50HZ           0x06U
#define CONTINUOUS_MODE_100HZ          0x08U
#define SELF_TEST_MODE                 0x10U

/* CNTL3 */
#define SOFT_RESET                     0x01U

#endif /* DREAMFLY2_COMPONENTS_AK09916_REG_H */
