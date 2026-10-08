//
// Created by 梅雨 on 2026/10/2.
//

#ifndef __ICM20948_REGISTER_H
#define __ICM20948_REGISTER_H	

#define ICM20948_ADDR 					0xD2		//110100X -> 0x69（AD0 High，7 BitAddress）
#define ICM20948_DEVID				    0xEA

/************************ ICM20948 寄存器地址 ************************/
#define REG_BANK_SEL					0x7F
#define USER_BANK_0						0x00
#define USER_BANK_1						0x10
#define USER_BANK_2						0x20
#define USER_BANK_3						0x30

// USER BANK 0
#define B0_WHO_AM_I						0x00		
#define B0_USER_CTRL					0x03
#define B0_LP_CONFIG					0x05
#define B0_PWR_MGMT_1					0x06
#define B0_PWR_MGMT_2					0x07
#define B0_INT_PIN_CFG					0x0F		
#define B0_INT_ENABLE					0x10
#define B0_INT_ENABLE_1					0x11
#define B0_INT_ENABLE_2					0x12
#define B0_INT_ENABLE_3					0x13
#define B0_I2C_MST_STATUS				0x17		
#define B0_INT_STATUS					0x19		
#define B0_INT_STATUS_1					0x1A
#define B0_INT_STATUS_2					0x1B
#define B0_INT_STATUS_3					0x1C
#define B0_DELAY_TIMEH					0x28
#define B0_DELAY_TIMEL					0x29
#define B0_ACCEL_XOUT_H					0x2D		
#define B0_ACCEL_XOUT_L					0x2E		
#define B0_ACCEL_YOUT_H					0x2F		
#define B0_ACCEL_YOUT_L					0x30		
#define B0_ACCEL_ZOUT_H					0x31		
#define B0_ACCEL_ZOUT_L					0x32	
#define B0_GYRO_XOUT_H					0x33	
#define B0_GYRO_XOUT_L					0x34
#define B0_GYRO_YOUT_H					0x35
#define B0_GYRO_YOUT_L					0x36
#define B0_GYRO_ZOUT_H					0x37
#define B0_GYRO_ZOUT_L					0x38
#define B0_TEMP_OUT_H					0x39		
#define B0_TEMP_OUT_L					0x3A
#define B0_EXT_SLV_SENS_DATA_00			0x3B
#define B0_EXT_SLV_SENS_DATA_01			0x3C
#define B0_EXT_SLV_SENS_DATA_02			0x3D
#define B0_EXT_SLV_SENS_DATA_03			0x3E
#define B0_EXT_SLV_SENS_DATA_04			0x3F
#define B0_EXT_SLV_SENS_DATA_05			0x40
#define B0_EXT_SLV_SENS_DATA_06			0x41
#define B0_EXT_SLV_SENS_DATA_07			0x42
#define B0_EXT_SLV_SENS_DATA_08			0x43
#define B0_EXT_SLV_SENS_DATA_09			0x44
#define B0_EXT_SLV_SENS_DATA_10			0x45
#define B0_EXT_SLV_SENS_DATA_11			0x46
#define B0_EXT_SLV_SENS_DATA_12			0x47
#define B0_EXT_SLV_SENS_DATA_13			0x48
#define B0_EXT_SLV_SENS_DATA_14			0x49
#define B0_EXT_SLV_SENS_DATA_15			0x4A
#define B0_EXT_SLV_SENS_DATA_16			0x4B
#define B0_EXT_SLV_SENS_DATA_17			0x4C
#define B0_EXT_SLV_SENS_DATA_18			0x4D
#define B0_EXT_SLV_SENS_DATA_19			0x4E
#define B0_EXT_SLV_SENS_DATA_20			0x4F
#define B0_EXT_SLV_SENS_DATA_21			0x50
#define B0_EXT_SLV_SENS_DATA_22			0x51
#define B0_EXT_SLV_SENS_DATA_23			0x52
#define B0_FIFO_EN_1					0x66	
#define B0_FIFO_EN_2					0x67
#define B0_FIFO_RST						0x68
#define B0_FIFO_MODE					0x69
#define B0_FIFO_COUNTH					0X70
#define B0_FIFO_COUNTL					0X71
#define B0_FIFO_R_W						0x72
#define B0_DATA_RDY_STATUS				0x74
#define B0_FIFO_CFG						0x76	

// USER BANK 1
#define B1_SELF_TEST_X_GYRO				0x02	
#define B1_SELF_TEST_Y_GYRO				0x03
#define B1_SELF_TEST_Z_GYRO				0x04
#define B1_SELF_TEST_X_ACCEL			0x0E	
#define B1_SELF_TEST_Y_ACCEL			0x0F
#define B1_SELF_TEST_Z_ACCEL			0x10
#define B1_XA_OFFS_H					0x14	
#define B1_XA_OFFS_L					0x15
#define B1_YA_OFFS_H					0x17
#define B1_YA_OFFS_L					0x18
#define B1_ZA_OFFS_H					0x1A
#define B1_ZA_OFFS_L					0x1B
#define B1_TIMEBASE_CORRECTION_PLL		0x28	

// USER BANK 2
#define B2_GYRO_SMPLRT_DIV				0x00	
#define B2_GYRO_CONFIG_1				0x01
#define BIT_GYRO_DLPCFG_0   			0x00 			/* bit[5:3] */
#define BIT_GYRO_DLPCFG_1   			0x08 			/* bit[5:3] */
#define BIT_GYRO_DLPCFG_2   			0x10 			/* bit[5:3] */
#define BIT_GYRO_DLPCFG_3   			0x18 			/* bit[5:3] */
#define BIT_GYRO_DLPCFG_4   			0x20 			/* bit[5:3] */
#define BIT_GYRO_DLPCFG_5   			0x28 			/* bit[5:3] */
#define BIT_GYRO_DLPCFG_6   			0x30 			/* bit[5:3] */
#define BIT_GYRO_DLPCFG_7   			0x38 			/* bit[5:3] */
#define BIT_GYRO_FS_250DPS  			0x00 			/* bit[2:1] */
#define BIT_GYRO_FS_500DPS  			0x02 			/* bit[2:1] */
#define BIT_GYRO_FS_1000DPS 			0x04 			/* bit[2:1] */
#define BIT_GYRO_FS_2000DPS 			0x06 			/* bit[2:1] */    
#define BIT_GYRO_DLPF       			0x01 			/* bit[0]   */	
#define B2_GYRO_CONFIG_2				0x02			
#define B2_XG_OFFS_USRH					0x03				
#define B2_XG_OFFS_USRL 				0x04			
#define B2_YG_OFFS_USRH					0x05			
#define B2_YG_OFFS_USRL					0x06			
#define B2_ZG_OFFS_USRH					0x07			
#define B2_ZG_OFFS_USRL					0x08			
#define B2_ODR_ALIGN_EN					0x09				
#define B2_ACCEL_SMPLRT_DIV_1			0x10				
#define B2_ACCEL_SMPLRT_DIV_2			0x11					
#define B2_ACCEL_INTEL_CTRL				0x12					
#define B2_ACCEL_WOM_THR				0x13			
#define B2_ACCEL_CONFIG					0x14
#define BIT_ACCEL_DLPCFG_0  			0x00 			/* bit[5:3] */
#define BIT_ACCEL_DLPCFG_1  			0x08 			/* bit[5:3] */
#define BIT_ACCEL_DLPCFG_2  			0x10 			/* bit[5:3] */
#define BIT_ACCEL_DLPCFG_3  			0x18 			/* bit[5:3] */
#define BIT_ACCEL_DLPCFG_4  			0x20 			/* bit[5:3] */
#define BIT_ACCEL_DLPCFG_5  			0x28 			/* bit[5:3] */
#define BIT_ACCEL_DLPCFG_6  			0x30 			/* bit[5:3] */
#define BIT_ACCEL_DLPCFG_7  			0x38 			/* bit[5:3] */
#define BIT_ACCEL_FS_2g     			0x00 			/* bit[2:1] */
#define BIT_ACCEL_FS_4g     			0x02 			/* bit[2:1] */
#define BIT_ACCEL_FS_8g     			0x04 			/* bit[2:1] */
#define BIT_ACCEL_FS_16g    			0x06 			/* bit[2:1] */    
#define BIT_ACCEL_DLPF      			0x01 			/* bit[0]   */
#define B2_ACCEL_CONFIG_2				0x15
#define B2_FSYNC_CONFIG					0x52
#define B2_TEMP_CONFIG					0x53
#define B2_MOD_CTRL_USR					0X54

// USER BANK 3
#define B3_I2C_MST_ODR_CONFIG			0x00
#define B3_I2C_MST_CTRL					0x01
#define B3_I2C_MST_DELAY_CTRL			0x02	
#define B3_I2C_SLV0_ADDR				0x03
#define B3_I2C_SLV0_REG					0x04		
#define B3_I2C_SLV0_CTRL				0x05
#define B3_I2C_SLV0_DO					0x06
#define B3_I2C_SLV1_ADDR				0x07		
#define B3_I2C_SLV1_REG					0x08		
#define B3_I2C_SLV1_CTRL				0x09
#define B3_I2C_SLV1_DO					0x0A
#define B3_I2C_SLV2_ADDR				0x0B		
#define B3_I2C_SLV2_REG					0x0C		
#define B3_I2C_SLV2_CTRL				0x0D
#define B3_I2C_SLV2_DO					0x0E
#define B3_I2C_SLV3_ADDR				0x0F		
#define B3_I2C_SLV3_REG					0x10		
#define B3_I2C_SLV3_CTRL				0x11
#define B3_I2C_SLV3_DO					0x12
#define B3_I2C_SLV4_ADDR				0x13	
#define B3_I2C_SLV4_REG					0x14		
#define B3_I2C_SLV4_CTRL				0x15
#define B3_I2C_SLV4_DO					0x16
#define B3_I2C_SLV4_DI					0x17

#endif
