#include "Position.h"

void Position_Update(int16_t pixel_delta_x, int16_t pixel_delta_y, float dT)
{
//	// 设置陀螺仪滤波参数
//	static _1_lpf_filter LPF_Filter = {0.80, 0, 0};
//	
//	float cpi = ((5*0.01f) / 11.914f) *2.54f ;
//	////////////////////////*积分位移处理*////////////////////////////
//	//低通滤波
//	g_OpticalFlow.Filter_POS_X += ((g_PMW3901MB_Raw.POS_X - g_OpticalFlow.Filter_POS_X) * 0.2f);
//	g_OpticalFlow.Filter_POS_Y += ((g_PMW3901MB_Raw.POX_Y - g_OpticalFlow.Filter_POS_Y) * 0.2f);
//	
//	//传感器倾角参数  用姿态角去补偿积分位移（#define  angle_to_rad  0.0174f  //角度转弧度）
//	g_OpticalFlow.Angle_X += (600.0f * tan(-g_ATT_Angle.PIT * DEGTORAD) - g_OpticalFlow.Angle_X) * 0.2f;
//	g_OpticalFlow.Angle_Y += (600.0f * tan(-g_ATT_Angle.ROL * DEGTORAD) - g_OpticalFlow.Angle_Y) * 0.2f;

//	//位移与角度互补融合
//	g_OpticalFlow.Fusion_POS_X = g_OpticalFlow.Filter_POS_X - g_OpticalFlow.Angle_X;  
//	g_OpticalFlow.Fusion_POS_Y = g_OpticalFlow.Filter_POS_Y - g_OpticalFlow.Angle_Y;
//	
//	////////////////////////*微分位移处理*////////////////////////////
//	//对积分位移进行微分处理，得到速度。
//	//求微分速度
//	g_OpticalFlow.Raw_Speed_X = (g_OpticalFlow.Fusion_POS_X - g_OpticalFlow.out_x_i_o)/dT;
//	g_OpticalFlow.out_x_i_o = g_OpticalFlow.Fusion_POS_X;
//	g_OpticalFlow.Raw_Speed_Y = (g_OpticalFlow.Fusion_POS_Y - g_OpticalFlow.out_y_i_o)/dT;
//	g_OpticalFlow.out_y_i_o = g_OpticalFlow.Fusion_POS_Y;
//	
//	//低通滤波
//	g_OpticalFlow.Filter_Speed_X += (g_OpticalFlow.Raw_Speed_X - g_OpticalFlow.Filter_Speed_X ) * 0.1f;
//	g_OpticalFlow.Filter_Speed_Y += (g_OpticalFlow.Raw_Speed_Y - g_OpticalFlow.Filter_Speed_Y ) * 0.1f;
//	
//	///////////////////*光流数据与高度数据融合*//////////////////////////
//		
////	//式中HIGH为实际高度，单位：米
////	cpi = ((FlightData.High.bara_height*0.01f) / 11.914f) *2.54f ;
////	pixel_flow.fix_High = cpi;
//	 
////	//积分位移值单位转换为：厘米
////	pixel_flow.loc_x = pixel_flow.out_x_i * cpi;
////	pixel_flow.loc_y = pixel_flow.out_y_i * cpi;
////	 
////	//微分速度值单位转换为：厘米/秒
////	pixel_flow.loc_xs = pixel_flow.fix_x * cpi; 
////	pixel_flow.loc_ys = pixel_flow.fix_y * cpi;
//	
//	static uint8_t cnt = 0;
//	cnt++;
//	if(cnt==5)
//	{
//	printf("===========\r\n");
//	printf(" Raw POSX: %f\r\n", g_OpticalFlow.Fusion_POS_X* cpi);
//	printf(" Raw POSY: %f\r\n", g_OpticalFlow.Fusion_POS_Y* cpi);
//	printf(" Raw VX: %f\r\n", g_OpticalFlow.Filter_Speed_X* cpi);
//	printf(" Raw VY: %f\r\n", g_OpticalFlow.Filter_Speed_Y* cpi);
//	}

}
