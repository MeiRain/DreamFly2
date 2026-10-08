#include "Delay.h"
#include "USART.h"

#include "VL53L1X.h"

VL53L1_Dev_t VL53L1_dev[2];	//device param, include I2C
//VL53L1_DeviceInfo_t VL53L1_dev_info[2];//device ID version info
uint8_t Ajusted[]={0,0};//adjusted sign, 0-not, 1-had
vu16 Distance = 0;//保存测距数据
VL53L1_RangingMeasurementData_t VL53L1_data[2];//ranging result struct, distance, max distance,etc.

//VL53L1X mode param
//0：default, 1: high accuracy, 2: long distance, 3: high speed
modedata_t g_ModeData[]=
{
	{(FixPoint1616_t)(16384), //its uint32_t, uint16_.uint16_t, 0.25*65536
	 (FixPoint1616_t)(1179648),	 //18*65536
	 45000,
	 14,
	 10},//default
		
	{(FixPoint1616_t)(16384),	//0.25*65536
	 (FixPoint1616_t)(1179648),		//18*65536
	 200000, 
	 14,
	 10},//high accuracy
		
  {(FixPoint1616_t)(6554),		//0.1*65536
	 (FixPoint1616_t)(3932160),		//60*65536
	 33000,
	 18,
	 14},//long distance
	
  {(FixPoint1616_t)(16384),	//0.25*65536
	 (FixPoint1616_t)(2097152),		//32*65536
	 20000,
	 14,
	 10},//high speed
};

//config VL53L1X I2C addr
//dev: I2C param struct
//newaddr: device new I2C addr
VL53L1_Error VL53L1_addr_set(VL53L1_Dev_t *dev, uint8_t newaddr)
{
	uint16_t Id;
	VL53L1_Error Status = VL53L1_Error_NONE;
	
	if(newaddr == dev->I2cDevAddr) //if new addr == old addr, exit
		return VL53L1_Error_NONE;
	
	//try to access a reg with old addr(default 0x52) 
	Status = VL53L1_RdWord(dev, VL53L1_IDENTIFICATION__MODEL_ID, &Id);
	if(Status!=VL53L1_Error_NONE) 
		return Status; 

	if(Id == 0xEACC)
	{
		//set new I2C addr
		Status = VL53L1_SetDeviceAddress(dev,newaddr);
		if(Status!=VL53L1_Error_NONE) 
			return Status; 
		
		//update struct I2C addr
		dev->I2cDevAddr = newaddr;
		//check new I2C addr
		Status = VL53L1_RdWord(dev, VL53L1_IDENTIFICATION__MODEL_ID, &Id);
	}

	return Status;
}


VL53L1_Error VL53L1_reset(VL53L1_Dev_t *pDev)
{
	return VL53L1X_Init(pDev);
}


//init vl53l1x
//dev: I2C param struct	
//before call this func, set:
//    pDev->I2cDevAddr = DevAddr;	---VL53L1X I2C addr (default 0x52 at power on)
// 		pDev->comms_type = 1;        	//I2C comm mode
//		pDev->comms_speed_khz = 400; 	//I2C bps
//if resetting device, func(pDev),DevAddr=0x52 
VL53L1_Error VL53L1X_Init(VL53L1_Dev_t *pDev)
{
	VL53L1_Error Status = VL53L1_Error_NONE;
	VL53L1_Dev_t *pMyDevice = pDev;
	pMyDevice->I2cDevAddr = 0x52;//I2C地址(上电默认0x52)
	pMyDevice->comms_type = 1;           //I2C通信模式
	pMyDevice->comms_speed_khz = 400;    //I2C通信速率
	
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	GPIO_InitStructure.GPIO_Pin = XShut_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 			//选择模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 			//开漏输出类型  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
//	GPIO_ResetBits(GPIOA,XShut2_Pin | XShut_Pin);
	
	XShut_Off;
	Delay_ms(30);
	XShut_On;
	Delay_ms(30);
	
	Status = VL53L1_WaitDeviceBooted(pMyDevice);
    if(Status != VL53L1_Error_NONE)
	{
		printf("Error: Device not booted!\r\n");
		return Status;
	}
	
//	Status = VL53L1_addr_set(pMyDevice, 0x54);//set provious VL53L1X I2C addr
//	if(Status!=VL53L1_Error_NONE)
//	{
//		printf("set addr failed!\r\n");
//		return Status;
//	}
	
	Status = VL53L1_DataInit(pMyDevice);//device init
	if(Status!=VL53L1_Error_NONE) 
	{
		printf("datainit failed!\r\n");
		return Status;
	}
	
	Status = VL53L1_StaticInit(pMyDevice);
	if(Status!=VL53L1_Error_NONE) 
	{
		printf("static init failed!\r\n");
		return Status;
	}

	Status = VL53L1_SetDistanceMode(pMyDevice, VL53L1_DISTANCEMODE_LONG);	//short,medium,long
	if(Status!=VL53L1_Error_NONE)
	{
		printf("set discance mode failed!\r\n");
		return Status;
	}
	
	Status = VL53L1_SetLimitCheckEnable(pMyDevice,VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE,1);//sigma--standard deviation, enable SIGMA limit check
	if(Status!=VL53L1_Error_NONE) 
		return Status;
	
	Status = VL53L1_SetLimitCheckEnable(pMyDevice,VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,1);//signal--amplitude of the signal-																																				//-reflected. enable signal rate limit check
	if(Status!=VL53L1_Error_NONE) 
		return Status;
	
	Status = VL53L1_SetLimitCheckValue(pMyDevice,VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE,g_ModeData[0].sigmaLimit);//set SIGMA limit
	if(Status!=VL53L1_Error_NONE) 
		return Status;
	
	Status = VL53L1_SetLimitCheckValue(pMyDevice,VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,g_ModeData[0].signalLimit);//set signal rate limit
	if(Status!=VL53L1_Error_NONE) 
		return Status;
	
	Status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(pMyDevice,g_ModeData[0].timingBudget);//set the max interval for a whole diatance test
	if(Status!=VL53L1_Error_NONE) 
		return Status;

	Status = VL53L1_SetInterMeasurementPeriodMilliSeconds(pMyDevice, 50);//the Delay between two ranging operations,0.5ms
	if(Status!=VL53L1_Error_NONE) 
	{
		printf("SetInterMeasurementPeriodMilliSeconds failed!\r\n");
		return Status;
	}
	
	Status = VL53L1_StartMeasurement(pMyDevice);
	if(Status!=VL53L1_Error_NONE) 
	{
		printf("start measurement failed!\r\n");
		return Status;
	}

	return Status;
}


//VL53L1X test mode config
//dev: device I2C param struct
//mode: 0:default, 1:high accuracy, 2:long distance
VL53L1_Error VL53L1_set_mode(VL53L1_Dev_t *dev, uint8_t mode)
{
	VL53L1_Error status = VL53L1_Error_NONE;
	//VL53L1_reset(dev);//reset vl53l1x (to avoid the distance result error due to frequently changing running mode)
	//status = VL53L1_StaticInit(dev);
	 
	Delay_ms(2);
	status = VL53L1_PerformRefSpadManagement(dev);//perform ref SPAD management
	if(status!=VL53L1_Error_NONE) 
	{
		printf("refspad failed!\r\n");
		return status;
	}
	Delay_ms(2);
	/*status = VL53L1_PerformOffsetSimpleCalibration(dev,140);
	if(status!=VL53L1_Error_NONE) 
	{
		printf("offset calibration failed!\r\n");
		//return status;
	}
	Delay_ms(2);
	status = VL53L1_SetXTalkCompensationEnable(dev,1);
	if(status!=VL53L1_Error_NONE) 
	{
		printf("XTalk enable failed!\r\n");
		//return status;
	}
	Delay_ms(2);
	status = VL53L1_PerformSingleTargetXTalkCalibration(dev,140);
	if(status!=VL53L1_Error_NONE) 
	{
		printf("XTalk calibration failed!\r\n");
		//return status;
	}*/
	return status;
}

//VL53L1X single ranging
//dev: device I2C param struct
//pdata: result struct
VL53L1_Error VL53L1_single_test(VL53L1_Dev_t *dev, VL53L1_RangingMeasurementData_t *pdata)
{
	VL53L1_Error Status = VL53L1_Error_NONE;
	u8 isDataReady = 0;

	Status = VL53L1_GetMeasurementDataReady(dev,&isDataReady);
	if(Status != VL53L1_Error_NONE)
		return Status;
	
    if(1 == isDataReady)
	{
		Status = VL53L1_GetRangingMeasurementData(dev, pdata);
		Distance = pdata->RangeMilliMeter;
	}
	Status = VL53L1_ClearInterruptAndStartMeasurement(dev);
	return Status;
}
   
//	Distance_data = pdata->RangeMilliMeter;//output the final distance data
//start general test. repeatly. not used
//dev:device I2C param struct
//mode0:default, 1:high accuracy, 2:long distance
void VL53L1_general_start(VL53L1_Dev_t *dev,uint8_t mode)
{
	VL53L1_Error Status=VL53L1_Error_NONE;
	uint8_t i=0;
	
	while(VL53L1_set_mode(dev,mode)) //config test mode
	{
		i++;
		if(i==2) 
			return;
	}
	
	while(Status==VL53L1_Error_NONE)
	{
		Status = VL53L1_single_test(dev,&VL53L1_data[0]);//perform a test
//		Distance_data=VL53L1_data[0].RangeMilliMeter;
	}
	Delay_ms(50);
}
