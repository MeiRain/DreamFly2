#include "USART.h"

#include "ANOAssistant.h"

#define ST_DEV_ADDR 0x66	// 本机设备地址
#define ST_USART1_ADDR 0x01 // 本机USART1地址
#define ST_USART2_ADDR 0x02 // 本机USART2地址
#define ANOASS_ADDR 0xFE	// 上位机（匿名助手）地址
#define ANOASS_HEADER 0xAB	// 上位机（匿名助手）协议帧头

// 数据拆分宏定义，在发送大于1字节的数据类型时，把数据拆分成单独字节进行发送
#define BYTE0(dwTemp) (*((char *)(&dwTemp)))
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))

uint8_t ANO_TxBuffer[50]; // 发送数据缓存
uint8_t ANO_RxBuffer[50]; // 接收数据缓存

//static u8 cnt = 0;
//	static u8 senser_cnt 	= 10;
//	static u8 user_cnt 	  = 10;
//	static u8 status_cnt 	= 15;
//	static u8 rcdata_cnt 	= 20;
//	static u8 motopwm_cnt	= 20;
//	static u8 power_cnt		=	50;
//	static u8 senser2_cnt = 50;

// /////////////////////////////////////////////////////////////////////////////////////
// // Data_Receive_Prepare函数是协议预解析，根据协议的格式，将收到的数据进行一次格式性解析，格式正确的话再进行数据解析
// // 移植时，此函数应由用户根据自身使用的通信方式自行调用，比如串口每收到一字节数据，则调用此函数一次
// // 此函数解析出符合格式的数据帧后，会自行调用数据解析函数
// void ANO_ReceiveDataPrepare(uint8_t data)
// {
// 	static uint8_t _data_len = 0, _data_cnt = 0;
// 	static uint8_t state = 0;

// 	if (state == 0 && data == 0xAB)
// 	{
// 		state = 1;
// 		ANO_RxBuffer[0] = data;
// 	}
// 	else if (state == 1 && data == 0xFE)
// 	{
// 		state = 2;
// 		ANO_RxBuffer[1] = data;
// 	}
// 	else if (state == 2 && data < 0xC8)
// 	{
// 		state = 3;
// 		ANO_RxBuffer[2] = data;
// 	}
// 	else if (state == 3 && data < 50)
// 	{
// 		state = 4;
// 		ANO_RxBuffer[3] = data;
// 		_data_len = data;
// 		_data_cnt = 0;
// 	}
// 	else if (state == 4 && _data_len > 0)
// 	{
// 		_data_len--;
// 		RxBuffer[4 + _data_cnt++] = data;
// 		if (_data_len == 0)
// 			state = 5;
// 	}
// 	else if (state == 5)
// 	{
// 		state = 0;
// 		RxBuffer[4 + _data_cnt] = data;
// 		ANO_DT_Data_Receive_Anl(RxBuffer, _data_cnt + 5);
// 	}
// 	else
// 		state = 0;
// }

/*****************************************************************************
 *函  数：void USART_SendFlightData(uint8_t *data, uint8_t length)
 *功  能：Usart发送指定长度数据
 *参  数：*data 要发送数据的地址
 *        length 要发送数据的长度
 *返回值：无
 *备  注：宏定义WIFI_DEBUG，在structconfig.h 中定义和取消;
 *        如果开启了WiFi调参功能，则数据从USART2传到ESP8266,然后经ESP8266再传到上位机;
 *        如果未开启WiFi调参功能，则数据从USART1的Tx,Rx经线连接eLink32的Rx,Tx传到上位机;
 *****************************************************************************/
static void ANO_SendBuffer(uint8_t *pBuffer, uint8_t BufferLength)
{
#ifdef WIFI_DEBUG // 开启WiFi（无线）调参

	USART2_SendArray(pBuffer, BufferLength);

#else // 有线调参

	USART2_SendArray(pBuffer, BufferLength);

#endif
}

void ANO_SendFrame0x01(int16_t acc_x, int16_t acc_y, int16_t acc_z, int16_t gyr_x, int16_t gyr_y, int16_t gyr_z, uint8_t shock_sta)
{
	uint8_t cnt = 0, Sum_Check = 0, Add_Check = 0;
	uint16_t len;

	ANO_TxBuffer[cnt++] = ANOASS_HEADER;
	ANO_TxBuffer[cnt++] = ST_DEV_ADDR;
	ANO_TxBuffer[cnt++] = ANOASS_ADDR;
	ANO_TxBuffer[cnt++] = 0x01;
	ANO_TxBuffer[cnt++] = 0x0D;
	ANO_TxBuffer[cnt++] = 0x00;

	ANO_TxBuffer[cnt++] = BYTE0(acc_x);
	ANO_TxBuffer[cnt++] = BYTE1(acc_x);
	ANO_TxBuffer[cnt++] = BYTE0(acc_y);
	ANO_TxBuffer[cnt++] = BYTE1(acc_y);
	ANO_TxBuffer[cnt++] = BYTE0(acc_z);
	ANO_TxBuffer[cnt++] = BYTE1(acc_z);
	ANO_TxBuffer[cnt++] = BYTE0(gyr_x);
	ANO_TxBuffer[cnt++] = BYTE1(gyr_x);
	ANO_TxBuffer[cnt++] = BYTE0(gyr_y);
	ANO_TxBuffer[cnt++] = BYTE1(gyr_y);
	ANO_TxBuffer[cnt++] = BYTE0(gyr_z);
	ANO_TxBuffer[cnt++] = BYTE1(gyr_z);
	ANO_TxBuffer[cnt++] = shock_sta;

	len = ANO_TxBuffer[4] + ANO_TxBuffer[5] * 256;

	for (uint8_t i = 0; i < (len + 6); i++)
	{
		Sum_Check += ANO_TxBuffer[i];
		Add_Check += Sum_Check;
	}

	ANO_TxBuffer[cnt++] = Sum_Check;
	ANO_TxBuffer[cnt++] = Add_Check;

	ANO_SendBuffer(ANO_TxBuffer, cnt);
}

void ANO_SendFrame0x02(int16_t mag_x, int16_t mag_y, int16_t mag_z, int16_t tmp, uint8_t mag_sta)
{
	uint8_t cnt = 0, Sum_Check = 0, Add_Check = 0;
	uint16_t len;

	ANO_TxBuffer[cnt++] = ANOASS_HEADER;
	ANO_TxBuffer[cnt++] = ST_DEV_ADDR;
	ANO_TxBuffer[cnt++] = ANOASS_ADDR;
	ANO_TxBuffer[cnt++] = 0x02;
	ANO_TxBuffer[cnt++] = 0x09;
	ANO_TxBuffer[cnt++] = 0x00;

	ANO_TxBuffer[cnt++] = BYTE0(mag_x);
	ANO_TxBuffer[cnt++] = BYTE1(mag_x);
	ANO_TxBuffer[cnt++] = BYTE0(mag_y);
	ANO_TxBuffer[cnt++] = BYTE1(mag_y);
	ANO_TxBuffer[cnt++] = BYTE0(mag_z);
	ANO_TxBuffer[cnt++] = BYTE1(mag_z);
	ANO_TxBuffer[cnt++] = BYTE0(tmp);
	ANO_TxBuffer[cnt++] = BYTE1(tmp);
	ANO_TxBuffer[cnt++] = mag_sta;

	len = ANO_TxBuffer[4] + ANO_TxBuffer[5] * 256;

	for (uint8_t i = 0; i < (len + 6); i++)
	{
		Sum_Check += ANO_TxBuffer[i];
		Add_Check += Sum_Check;
	}

	ANO_TxBuffer[cnt++] = Sum_Check;
	ANO_TxBuffer[cnt++] = Add_Check;

	ANO_SendBuffer(ANO_TxBuffer, cnt);
}

void ANO_SendFrame0x03(float rol, float pit, float yaw, uint8_t fusion_sta)
{
	uint8_t cnt = 0, Sum_Check = 0, Add_Check = 0;
	uint16_t len;
	int16_t temp;
	
	ANO_TxBuffer[cnt++] = ANOASS_HEADER;
	ANO_TxBuffer[cnt++] = ST_DEV_ADDR;
	ANO_TxBuffer[cnt++] = ANOASS_ADDR;
	ANO_TxBuffer[cnt++] = 0x03;
	ANO_TxBuffer[cnt++] = 0x07;
	ANO_TxBuffer[cnt++] = 0x00;

	temp = (int)(rol * 100);
	ANO_TxBuffer[cnt++] = BYTE0(temp);
	ANO_TxBuffer[cnt++] = BYTE1(temp);
	temp = (int)(pit * 100);
	ANO_TxBuffer[cnt++] = BYTE0(temp);
	ANO_TxBuffer[cnt++] = BYTE1(temp);
	temp = (int)(yaw * 100);
	ANO_TxBuffer[cnt++] = BYTE0(temp);
	ANO_TxBuffer[cnt++] = BYTE1(temp);
	ANO_TxBuffer[cnt++] = fusion_sta;

	len = ANO_TxBuffer[4] + ANO_TxBuffer[5] * 256;

	for (uint8_t i = 0; i < (len + 6); i++)
	{
		Sum_Check += ANO_TxBuffer[i];
		Add_Check += Sum_Check;
	}

	ANO_TxBuffer[cnt++] = Sum_Check;
	ANO_TxBuffer[cnt++] = Add_Check;

	ANO_SendBuffer(ANO_TxBuffer, cnt);
}

void ANO_SendFrame0x05(int32_t alt_bar, int32_t alt_add, int32_t alt_fu, uint8_t alt_sta)
{
	uint8_t cnt = 0, Sum_Check = 0, Add_Check = 0;
	uint16_t len;

	ANO_TxBuffer[cnt++] = ANOASS_HEADER;
	ANO_TxBuffer[cnt++] = ST_DEV_ADDR;
	ANO_TxBuffer[cnt++] = ANOASS_ADDR;
	ANO_TxBuffer[cnt++] = 0x05;
	ANO_TxBuffer[cnt++] = 0x0D;
	ANO_TxBuffer[cnt++] = 0x00;

	ANO_TxBuffer[cnt++] = BYTE0(alt_bar);
	ANO_TxBuffer[cnt++] = BYTE1(alt_bar);
	ANO_TxBuffer[cnt++] = BYTE2(alt_bar);
	ANO_TxBuffer[cnt++] = BYTE3(alt_bar);
	ANO_TxBuffer[cnt++] = BYTE0(alt_add);
	ANO_TxBuffer[cnt++] = BYTE1(alt_add);
	ANO_TxBuffer[cnt++] = BYTE2(alt_add);
	ANO_TxBuffer[cnt++] = BYTE3(alt_add);
	ANO_TxBuffer[cnt++] = BYTE0(alt_fu);
	ANO_TxBuffer[cnt++] = BYTE1(alt_fu);
	ANO_TxBuffer[cnt++] = BYTE2(alt_fu);
	ANO_TxBuffer[cnt++] = BYTE3(alt_fu);
	ANO_TxBuffer[cnt++] = alt_sta;

	len = ANO_TxBuffer[4] + ANO_TxBuffer[5] * 256;

	for (uint8_t i = 0; i < (len + 6); i++)
	{
		Sum_Check += ANO_TxBuffer[i];
		Add_Check += Sum_Check;
	}

	ANO_TxBuffer[cnt++] = Sum_Check;
	ANO_TxBuffer[cnt++] = Add_Check;

	ANO_SendBuffer(ANO_TxBuffer, cnt);
}

void ANO_SendFrame0x06(uint8_t mode, uint8_t sflag, uint8_t cid, uint8_t cmd0, uint8_t cmd01)
{
	uint8_t cnt = 0, Sum_Check = 0, Add_Check = 0;
	uint16_t len;

	ANO_TxBuffer[cnt++] = ANOASS_HEADER;
	ANO_TxBuffer[cnt++] = ST_DEV_ADDR;
	ANO_TxBuffer[cnt++] = ANOASS_ADDR;
	ANO_TxBuffer[cnt++] = 0x06;
	ANO_TxBuffer[cnt++] = 0x05;
	ANO_TxBuffer[cnt++] = 0x00;

	ANO_TxBuffer[cnt++] = mode;
	ANO_TxBuffer[cnt++] = sflag;
	ANO_TxBuffer[cnt++] = cid;
	ANO_TxBuffer[cnt++] = cmd0;
	ANO_TxBuffer[cnt++] = cmd01;

	len = ANO_TxBuffer[4] + ANO_TxBuffer[5] * 256;

	for (uint8_t i = 0; i < (len + 6); i++)
	{
		Sum_Check += ANO_TxBuffer[i];
		Add_Check += Sum_Check;
	}

	ANO_TxBuffer[cnt++] = Sum_Check;
	ANO_TxBuffer[cnt++] = Add_Check;

	ANO_SendBuffer(ANO_TxBuffer, cnt);
}

void ANO_SendFrame0x0D(uint8_t voltage, uint8_t current)
{
	uint8_t cnt = 0, Sum_Check = 0, Add_Check = 0;
	uint16_t len, temp;

	ANO_TxBuffer[cnt++] = ANOASS_HEADER;
	ANO_TxBuffer[cnt++] = ST_DEV_ADDR;
	ANO_TxBuffer[cnt++] = ANOASS_ADDR;
	ANO_TxBuffer[cnt++] = 0x0D;
	ANO_TxBuffer[cnt++] = 0x04;
	ANO_TxBuffer[cnt++] = 0x00;

	temp = voltage * 100;
	ANO_TxBuffer[cnt++] = BYTE0(temp);
	ANO_TxBuffer[cnt++] = BYTE1(temp);
	temp = current * 100;
	ANO_TxBuffer[cnt++] = BYTE0(temp);
	ANO_TxBuffer[cnt++] = BYTE1(temp);

	len = ANO_TxBuffer[4] + ANO_TxBuffer[5] * 256;

	for (uint8_t i = 0; i < (len + 6); i++)
	{
		Sum_Check += ANO_TxBuffer[i];
		Add_Check += Sum_Check;
	}

	ANO_TxBuffer[cnt++] = Sum_Check;
	ANO_TxBuffer[cnt++] = Add_Check;

	ANO_SendBuffer(ANO_TxBuffer, cnt);
}

void ANOSendData_Remote(int16_t rol, int16_t pit, int16_t thr, int16_t yaw, int16_t aux1,
					    int16_t aux2, int16_t aux3, int16_t aux4, int16_t aux5, int16_t aux6)
{
	uint8_t cnt = 0, Sum_Check = 0, Add_Check = 0;
	uint16_t len;

	ANO_TxBuffer[cnt++] = ANOASS_HEADER;
	ANO_TxBuffer[cnt++] = ST_DEV_ADDR;
	ANO_TxBuffer[cnt++] = ANOASS_ADDR;
	ANO_TxBuffer[cnt++] = 0x41;
	ANO_TxBuffer[cnt++] = 0x32;
	ANO_TxBuffer[cnt++] = 0x00;

	ANO_TxBuffer[cnt++] = BYTE0(rol);
	ANO_TxBuffer[cnt++] = BYTE1(rol);
	ANO_TxBuffer[cnt++] = BYTE0(pit);
	ANO_TxBuffer[cnt++] = BYTE1(pit);
	ANO_TxBuffer[cnt++] = BYTE0(thr);
	ANO_TxBuffer[cnt++] = BYTE1(thr);
	ANO_TxBuffer[cnt++] = BYTE0(yaw);
	ANO_TxBuffer[cnt++] = BYTE1(yaw);
	ANO_TxBuffer[cnt++] = BYTE0(aux1);
	ANO_TxBuffer[cnt++] = BYTE1(aux1);
	ANO_TxBuffer[cnt++] = BYTE0(aux2);
	ANO_TxBuffer[cnt++] = BYTE1(aux2);
	ANO_TxBuffer[cnt++] = BYTE0(aux3);
	ANO_TxBuffer[cnt++] = BYTE1(aux3);
	ANO_TxBuffer[cnt++] = BYTE0(aux4);
	ANO_TxBuffer[cnt++] = BYTE1(aux4);
	ANO_TxBuffer[cnt++] = BYTE0(aux5);
	ANO_TxBuffer[cnt++] = BYTE1(aux5);
	ANO_TxBuffer[cnt++] = BYTE0(aux6);
	ANO_TxBuffer[cnt++] = BYTE1(aux6);

	len = ANO_TxBuffer[4] + ANO_TxBuffer[5] * 256;

	for (uint8_t i = 0; i < (len + 6); i++)
	{
		Sum_Check += ANO_TxBuffer[i];
		Add_Check += Sum_Check;
	}

	ANO_TxBuffer[cnt++] = Sum_Check;
	ANO_TxBuffer[cnt++] = Add_Check;

	ANO_SendBuffer(ANO_TxBuffer, cnt);
}

//void ANO_SendRemoteControlData(int16_t corl_rol, int16_t corl_pit, int16_t corl_thr, int16_t corl_yaw, int16_t corl_aux1,
//					           int16_t corl_aux2, int16_t corl_aux3, int16_t corl_aux4, int16_t corl_aux5, int16_t corl_aux6)
//{
//	uint8_t cnt = 0, Sum_Check = 0, Add_Check = 0;
//	uint16_t len;

//	ANO_TxBuffer[cnt++] = ANOASS_HEADER;
//	ANO_TxBuffer[cnt++] = ST_DEV_ADDR;
//	ANO_TxBuffer[cnt++] = ANOASS_ADDR;
//	ANO_TxBuffer[cnt++] = 0x41;
//	ANO_TxBuffer[cnt++] = 0x32;
//	ANO_TxBuffer[cnt++] = 0x00;

//	ANO_TxBuffer[cnt++] = BYTE0(rol);
//	ANO_TxBuffer[cnt++] = BYTE1(rol);
//	ANO_TxBuffer[cnt++] = BYTE0(pit);
//	ANO_TxBuffer[cnt++] = BYTE1(pit);
//	ANO_TxBuffer[cnt++] = BYTE0(thr);
//	ANO_TxBuffer[cnt++] = BYTE1(thr);
//	ANO_TxBuffer[cnt++] = BYTE0(yaw);
//	ANO_TxBuffer[cnt++] = BYTE1(yaw);
//	ANO_TxBuffer[cnt++] = BYTE0(aux1);
//	ANO_TxBuffer[cnt++] = BYTE1(aux1);
//	ANO_TxBuffer[cnt++] = BYTE0(aux2);
//	ANO_TxBuffer[cnt++] = BYTE1(aux2);
//	ANO_TxBuffer[cnt++] = BYTE0(aux3);
//	ANO_TxBuffer[cnt++] = BYTE1(aux3);
//	ANO_TxBuffer[cnt++] = BYTE0(aux4);
//	ANO_TxBuffer[cnt++] = BYTE1(aux4);
//	ANO_TxBuffer[cnt++] = BYTE0(aux5);
//	ANO_TxBuffer[cnt++] = BYTE1(aux5);
//	ANO_TxBuffer[cnt++] = BYTE0(aux6);
//	ANO_TxBuffer[cnt++] = BYTE1(aux6);

//	len = ANO_TxBuffer[4] + ANO_TxBuffer[5] * 256;

//	for (uint8_t i = 0; i < (len + 6); i++)
//	{
//		Sum_Check += ANO_TxBuffer[i];
//		Add_Check += Sum_Check;
//	}

//	ANO_TxBuffer[cnt++] = Sum_Check;
//	ANO_TxBuffer[cnt++] = Add_Check;

//	ANO_SendBuffer(ANO_TxBuffer, cnt);
//}

/***************************************自定义帧**********************************************/

void ANO_SendFrame0xF1(int16_t filter_acc_x, int16_t filter_acc_y, int16_t filter_acc_z,
					   int16_t filter_gyr_x, int16_t filter_gyr_y, int16_t filter_gyr_z,
					   int16_t filter_bar)
{
	uint8_t cnt = 0, Sum_Check = 0, Add_Check = 0;
	uint16_t len;

	ANO_TxBuffer[cnt++] = ANOASS_HEADER;
	ANO_TxBuffer[cnt++] = ST_DEV_ADDR;
	ANO_TxBuffer[cnt++] = ANOASS_ADDR;
	ANO_TxBuffer[cnt++] = 0x06;
	ANO_TxBuffer[cnt++] = 0x05;
	ANO_TxBuffer[cnt++] = 0x00;

	// ANO_TxBuffer[cnt ++] = BYTE0(filter_acc_x);
	// ANO_TxBuffer[cnt ++] = BYTE1(filter_acc_x);
	// ANO_TxBuffer[cnt ++] = BYTE0(filter_acc_y);
	// ANO_TxBuffer[cnt ++] = BYTE1(filter_acc_y);
	// ANO_TxBuffer[cnt ++] = BYTE0(filter_acc_z);
	// ANO_TxBuffer[cnt ++] = BYTE1(filter_acc_z);
	// ANO_TxBuffer[cnt ++] = BYTE0(filter_gyr_x);
	// ANO_TxBuffer[cnt ++] = BYTE1(filter_gyr_x);
	// ANO_TxBuffer[cnt ++] = BYTE0(filter_gyr_y);
	// ANO_TxBuffer[cnt ++] = BYTE1(filter_gyr_y);
	// ANO_TxBuffer[cnt ++] = BYTE0(filter_gyr_z);
	// ANO_TxBuffer[cnt ++] = BYTE1(filter_gyr_z);
	// ANO_TxBuffer[cnt ++] = BYTE0(aux3);
	// ANO_TxBuffer[cnt ++] = BYTE1(aux3);
	// ANO_TxBuffer[cnt ++] = BYTE0(aux4);
	// ANO_TxBuffer[cnt ++] = BYTE1(aux4);
	// ANO_TxBuffer[cnt ++] = BYTE0(aux5);
	// ANO_TxBuffer[cnt ++] = BYTE1(aux5);
	// ANO_TxBuffer[cnt ++] = BYTE0(aux6);
	// ANO_TxBuffer[cnt ++] = BYTE1(aux6);

	len = ANO_TxBuffer[4] + ANO_TxBuffer[5] * 256;

	for (uint8_t i = 0; i < (len + 6); i++)
	{
		Sum_Check += ANO_TxBuffer[i];
		Add_Check += Sum_Check;
	}

	ANO_TxBuffer[cnt++] = Sum_Check;
	ANO_TxBuffer[cnt++] = Add_Check;

	ANO_SendBuffer(ANO_TxBuffer, cnt);
}

// ////滤波效果观察函数调试
// void Data_Send_Filter(void)
// {
// 	u8 _cnt = 0, sum = 0, i;
// 	float _temp;
// 	vs16 temp;
// 	ANO_TxBuffer[_cnt++] = 0xAA;
// 	ANO_TxBuffer[_cnt++] = 0xAA;
// 	ANO_TxBuffer[_cnt++] = 0xF2;
// 	ANO_TxBuffer[_cnt++] = 0;

// 	_temp = FBM.Altitude;
// 	ANO_TxBuffer[_cnt++] = BYTE3(_temp);
// 	ANO_TxBuffer[_cnt++] = BYTE2(_temp);
// 	ANO_TxBuffer[_cnt++] = BYTE1(_temp);
// 	ANO_TxBuffer[_cnt++] = BYTE0(_temp);

// 	_temp = nav.z;
// 	ANO_TxBuffer[_cnt++] = BYTE3(_temp);
// 	ANO_TxBuffer[_cnt++] = BYTE2(_temp);
// 	ANO_TxBuffer[_cnt++] = BYTE1(_temp);
// 	ANO_TxBuffer[_cnt++] = BYTE0(_temp);

// 	_temp = nav.vz;
// 	ANO_TxBuffer[_cnt++] = BYTE3(_temp);
// 	ANO_TxBuffer[_cnt++] = BYTE2(_temp);
// 	ANO_TxBuffer[_cnt++] = BYTE1(_temp);
// 	ANO_TxBuffer[_cnt++] = BYTE0(_temp);
// 	_temp = nav.az;
// 	ANO_TxBuffer[_cnt++] = BYTE3(_temp);
// 	ANO_TxBuffer[_cnt++] = BYTE2(_temp);
// 	ANO_TxBuffer[_cnt++] = BYTE1(_temp);
// 	ANO_TxBuffer[_cnt++] = BYTE0(_temp);
// 	_temp = Acc_filt.Z;
// 	ANO_TxBuffer[_cnt++] = BYTE3(_temp);
// 	ANO_TxBuffer[_cnt++] = BYTE2(_temp);
// 	ANO_TxBuffer[_cnt++] = BYTE1(_temp);
// 	ANO_TxBuffer[_cnt++] = BYTE0(_temp);

// 	//_temp = Gyr_rad.X*RadtoDeg;
// 	_temp = FBM.AltitudeFilter;
// 	ANO_TxBuffer[_cnt++] = BYTE3(_temp);
// 	ANO_TxBuffer[_cnt++] = BYTE2(_temp);
// 	ANO_TxBuffer[_cnt++] = BYTE1(_temp);
// 	ANO_TxBuffer[_cnt++] = BYTE0(_temp);
// 	//_temp = Gyr_rad.Y*RadtoDeg;
// 	_temp = (float)ICM20948_ACC.Y * 0.011964f;
// 	ANO_TxBuffer[_cnt++] = BYTE3(_temp);
// 	ANO_TxBuffer[_cnt++] = BYTE2(_temp);
// 	ANO_TxBuffer[_cnt++] = BYTE1(_temp);
// 	ANO_TxBuffer[_cnt++] = BYTE0(_temp);
// 	//_temp = Gyr_rad.Z*RadtoDeg;
// 	_temp = (float)ICM20948_ACC.Z * 0.011964f;
// 	ANO_TxBuffer[_cnt++] = BYTE3(_temp);
// 	ANO_TxBuffer[_cnt++] = BYTE2(_temp);
// 	ANO_TxBuffer[_cnt++] = BYTE1(_temp);
// 	ANO_TxBuffer[_cnt++] = BYTE0(_temp);

// 	ANO_TxBuffer[3] = _cnt - 4;

// 	for (i = 0; i < _cnt; i++)
// 		sum += ANO_TxBuffer[i];

// 	ANO_TxBuffer[_cnt++] = sum;

// 	ANO_SendBuffer(ANO_TxBuffer, _cnt);
// }

// 角速度环调试,波形显示
void Data_Send_AngleRate(float data1, float data2, float data3, float data4, float data5, float data6, float data7, float data8)
{
	u8 _cnt = 0, sum = 0, i;
	float _temp;
	ANO_TxBuffer[_cnt++] = 0xAA;
	ANO_TxBuffer[_cnt++] = 0xAA;
	ANO_TxBuffer[_cnt++] = 0xF1;
	ANO_TxBuffer[_cnt++] = 0;

	_temp = data1; // RadtoDeg
	ANO_TxBuffer[_cnt++] = BYTE3(_temp);
	ANO_TxBuffer[_cnt++] = BYTE2(_temp);
	ANO_TxBuffer[_cnt++] = BYTE1(_temp);
	ANO_TxBuffer[_cnt++] = BYTE0(_temp);
	_temp = data2;
	ANO_TxBuffer[_cnt++] = BYTE3(_temp);
	ANO_TxBuffer[_cnt++] = BYTE2(_temp);
	ANO_TxBuffer[_cnt++] = BYTE1(_temp);
	ANO_TxBuffer[_cnt++] = BYTE0(_temp);
	_temp = data3;
	ANO_TxBuffer[_cnt++] = BYTE3(_temp);
	ANO_TxBuffer[_cnt++] = BYTE2(_temp);
	ANO_TxBuffer[_cnt++] = BYTE1(_temp);
	ANO_TxBuffer[_cnt++] = BYTE0(_temp);
	_temp = data4;
	ANO_TxBuffer[_cnt++] = BYTE3(_temp);
	ANO_TxBuffer[_cnt++] = BYTE2(_temp);
	ANO_TxBuffer[_cnt++] = BYTE1(_temp);
	ANO_TxBuffer[_cnt++] = BYTE0(_temp);
	_temp = data5;
	ANO_TxBuffer[_cnt++] = BYTE3(_temp);
	ANO_TxBuffer[_cnt++] = BYTE2(_temp);
	ANO_TxBuffer[_cnt++] = BYTE1(_temp);
	ANO_TxBuffer[_cnt++] = BYTE0(_temp);
	_temp = data6;
	ANO_TxBuffer[_cnt++] = BYTE3(_temp);
	ANO_TxBuffer[_cnt++] = BYTE2(_temp);
	ANO_TxBuffer[_cnt++] = BYTE1(_temp);
	ANO_TxBuffer[_cnt++] = BYTE0(_temp);
	_temp = data7;
	ANO_TxBuffer[_cnt++] = BYTE3(_temp);
	ANO_TxBuffer[_cnt++] = BYTE2(_temp);
	ANO_TxBuffer[_cnt++] = BYTE1(_temp);
	ANO_TxBuffer[_cnt++] = BYTE0(_temp);
	_temp = data8;
	ANO_TxBuffer[_cnt++] = BYTE3(_temp);
	ANO_TxBuffer[_cnt++] = BYTE2(_temp);
	ANO_TxBuffer[_cnt++] = BYTE1(_temp);
	ANO_TxBuffer[_cnt++] = BYTE0(_temp);

	ANO_TxBuffer[3] = _cnt - 4;

	for (i = 0; i < _cnt; i++)
		sum += ANO_TxBuffer[i];

	ANO_TxBuffer[_cnt++] = sum;

	ANO_SendBuffer(ANO_TxBuffer, _cnt);
}
