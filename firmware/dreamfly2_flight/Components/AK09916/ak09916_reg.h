#ifndef __AK09916_REGISTER_H
#define __AK09916_REGISTER_H

#define AK09916_ADDR					0x0C        //1101001X -> 0x69£¨AD0 High£¬7 BitAddress£©
#define AK09916_DEVID				    0x09
#define AK09916_READ                    0x80
#define AK09916_WRITE                   0x00

/************* AK09916 ¼Ä´æÆ÷µØÖ· *************/
#define WIA2					0x01
#define ST1						0x10
#define HXL						0x11
#define HXH						0x12
#define HYL						0x13
#define HYH						0x14
#define HZL						0x15
#define HZH						0x16
#define ST2						0x18
#define CNTL2					0x31
#define BIT_POWER_DOWN_MODE     0x00
#define BIT_SINGLE_MODE         0x01
#define BIT_CM_MODE_1           0x02
#define BIT_CM_MODE_2           0x04
#define BIT_CM_MODE_3           0x06
#define BIT_CM_MODE_4           0x08
#define BIT_SELF_TEST_MODE      0x0F
#define CNTL3					0x32
#define TS1						0x33
#define TS2						0x34

#endif
