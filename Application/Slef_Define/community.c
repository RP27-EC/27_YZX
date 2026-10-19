/*********************************************
File Name  //community.c

Descripition: //change information between two boards

Function list://void board_init(void)结构体初始化
//void board_update_send(void)通信发送函数
//void board_rx(board_t *board, uint8_t *rxBuf)接收云台发送的数据
**********************************************/
#include "community.h"
#include "rc_sensor.h"
#include "imu_sensor.h"
#include "drv_can.h"

uint8_t fail_tx_gimb = 0;
uint8_t tx_gimb_buff[8];//电机的发送数组
uint8_t rx_gimb_buff[8];//电机的接收数组

board_t board = {.set_back_flag = 0};
extern imu_sensor_t imu_sensor;

//结构体初始化
void board_init(void)
{
	board.last_yaw = imu_sensor.info->base_info.yaw;
	board.rc_stata = DEV_OFFLINE;
	board.yaw_msg = imu_sensor.info->base_info.yaw;	
}

//通信发送函数
void board_update_send(void)
{
	
	//遥控器心跳
	if ( rc_sensor.work_state == DEV_ONLINE)
	{
		board.rc_stata = DEV_ONLINE;
	}
	else
	{
		board.rc_stata = DEV_OFFLINE;
	}
	
	//模式判断
	//通过拨杆判断模式
	if (rc_sensor.info->s1 == 1)
	{
		board.control_modle = imu;
	}
	else
	{
		board.control_modle = chass;
	}
	
	memset(tx_gimb_buff, 0, 8);
	
	//机械模式发送底盘位置偏移量
	if (board.control_modle == chass)
	{
	
		board.current_yaw = imu_sensor.info->base_info.yaw;
	
		board.rc_stata = rc_sensor.work_state;
		
		//更新底盘对应角度
		board.yaw_msg = board.current_yaw - board.last_yaw;
		
		while( board.yaw_msg >180.0f)
		{
			board.yaw_msg = board.yaw_msg - 360.0f;
		
		}
		
		while (board.yaw_msg < -180.0f)
		{
			board.yaw_msg = board.yaw_msg + 360.0f;
		
		}
		
		if (rc_sensor.info->thumbwheel.step[1] == 0)
		{
			board.set_back_flag = 0;
		}
		else
		{
			board.set_back_flag = 1;
		}
		
		int16_t yaw_increment_int = (int16_t)(board.yaw_msg * 100.0f);
		memcpy(&tx_gimb_buff[0], &yaw_increment_int, 2);
		board.last_yaw = board.current_yaw;
	}
	
	//陀螺仪模式发送ch0
	if (board.control_modle == imu)
	{
		int16_t ch0_val = rc_sensor.info->ch0;
        memcpy(&tx_gimb_buff[0], &ch0_val, 2);
	}
	
    tx_gimb_buff[3] = board.control_modle;
	tx_gimb_buff[4] = board.rc_stata;
	tx_gimb_buff[5] = 0;
	tx_gimb_buff[6] = 0;
	tx_gimb_buff[7] = board.set_back_flag;

    if (CAN_SendData(&hfdcan2, 0x123, tx_gimb_buff) != HAL_OK)
	{
		fail_tx_gimb ++;
	}
	
	
}

//接收云台发送的数据
void board_rx(board_t *board, uint8_t *rxBuf)
{
	board->gimb_forward = rxBuf[0];
	int16_t temp_int = (uint16_t)((rxBuf[1] << 8) | rxBuf[2]);
	
	int16_t mec_fb_int = (int16_t)temp_int;//符号转化
	
	board->relative_mec_angle = (float)mec_fb_int / 10000.0f;
}
