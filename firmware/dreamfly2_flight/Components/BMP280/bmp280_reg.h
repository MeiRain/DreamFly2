#ifndef __BMP280_REGISTER_H
#define __BMP280_REGISTER_H

#define BMP280_ADDR        				0x76<<1    //address pin low (GND)
#define BMP280_DEVID        			0x58
#define BMP280_RESET_VALUE   			0xB6

//状态寄存器转换标志
#define	BMP280_MEASURING					0x01
#define	BMP280_IM_UPDATE					0x08

/********************* BMP280 Register **********************/
#define ID      					0xD0
#define RESET   					0xE0
#define STATUS      				0xF3
#define CONTROL_MEAS     		0xF4
#define CONFIG      				0xF5
#define PRESS_MSB        		0xF7		/*Pressure MSB Register     */ 
#define PRESS_LSB        		0xF8		/*Pressure LSB Register     */
#define PRESS_XLSB       		0xF9		/*Pressure XLSB  Register   */
#define TEMP_MSB         		0xFA        /*Temperature LSB Register  */  
#define TEMP_LSB         		0xFB        /*Temperature LSB Register  */ 
#define TEMP_XLSB        		0xFC      	/*Temperature XLSB Register */
#define DIG_T1_LSB		0x88  
#define DIG_T1_MSB		0x89  
#define DIG_T2_LSB		0x8A  
#define DIG_T2_MSB		0x8B  
#define DIG_T3_LSB		0x8C  
#define DIG_T3_MSB		0x8D  
#define DIG_P1_LSB		0x8E  
#define DIG_P1_MSB		0x8F  
#define DIG_P2_LSB		0x90  
#define DIG_P2_MSB		0x91  
#define DIG_P3_LSB		0x92  
#define DIG_P3_MSB		0x93  
#define DIG_P4_LSB		0x94  
#define DIG_P4_MSB		0x95  
#define DIG_P5_LSB		0x96  
#define DIG_P5_MSB		0x97  
#define DIG_P6_LSB		0x98  
#define DIG_P6_MSB		0x99  
#define DIG_P7_LSB		0x9A  
#define DIG_P7_MSB		0x9B  
#define DIG_P8_LSB		0x9C  
#define DIG_P8_MSB		0x9D  
#define DIG_P9_LSB		0x9E  
#define DIG_P9_MSB		0x9F
 
#endif
