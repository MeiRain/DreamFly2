#ifndef __PMW3901MB_REGISTER_H
#define __PMW3901MB_REGISTER_H

#define	PRODUCT_ID 					0x00			// RO 		0x49
#define REVISION_ID 				0x01			// RO 		0x00
#define MOTION 						0x02			// R/W 		0x00
#define DELTA_X_L 					0x03			// RO 		0x00
#define DELTA_X_H 					0x04			// RO 		0x00
#define DELTA_Y_L 					0x05			// RO 		0x00
#define DELTA_Y_H 					0x06			// RO 		0x00
#define SQUAL 						0x07			// RO 		0x00
#define RAWDATA_SUM 				0x08			// RO 		0x00
#define MAXIMUM_RAWDATA 			0x09			// RO 		0x00
#define MINIMUM_RAWDATA 			0x0A			// RO 		0x00
#define SHUTTER_LOWER 				0x0B			// RO 		0x00
#define SHUTTER_UPPER 				0x0C			// RO 		0x00
#define OBSERVATION 				0x15			// R/W 		0x00
#define MOTION_BURST 				0x16			// RO 		0x00
#define POWER_UP_RESET 				0x3A			// WO 		N/A
#define SHUTDOWN 					0x3B			// WO 		N/A
#define RAWDATA_GRAB 				0x58			// R/W 		0x00
#define RAWDATA_GRAB_STATUS 		0x59			// RO 		0x00
#define INVERSE_PRODUCT_ID 			0x5F			// RO 		0xB6

#endif
