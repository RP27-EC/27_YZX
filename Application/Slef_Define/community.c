/*********************************************
板间通讯函数



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

void board_init(void)
{
	board.last_yaw = imu_sensor.info->base_info.yaw;
	board.rc_stata = DEV_OFFLINE;
	board.yaw_msg = imu_sensor.info->base_info.yaw;	
}

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
	
	
	
	
	/*
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
	
	
	int16_t yaw_increment_int = (int16_t)(board.yaw_msg * 100.0f);
	
	int16_t ch0_yaw = rc_sensor.info->ch0;   // 控制云台Yaw
	
	if (rc_sensor.info->s1 == 2)
	{
		board.set0_flag = 1;
	}
	else
	{
		board.set0_flag = 0;
	}
	
	uint8_t status_flag = 1;
	
	if (board.yaw_msg == DEV_ONLINE)
	{
		status_flag = 0;
	}
	memset(tx_gimb_buff, 0, 8);
	
    memcpy(&tx_gimb_buff[0], &ch0_yaw, 2);

    memcpy(&tx_gimb_buff[2], &ch1_pitch, 2);
	
	memcpy(&tx_gimb_buff[4], &yaw_increment_int, 2);
	
	tx_gimb_buff[7] = board.set0_flag;

    if (CAN_SendData(&hfdcan2, 0x123, tx_gimb_buff) != HAL_OK)
	{
		fail_tx_gimb ++;
	}
	
	board.last_yaw = board.current_yaw;
	
	
	*/
}

void board_rx(board_t *board, uint8_t *rxBuf)
{
	board->gimb_forward = rxBuf[0];
	int16_t temp_int = (uint16_t)((rxBuf[1] << 8) | rxBuf[2]);
	
	int16_t mec_fb_int = (int16_t)temp_int;
	
	board->relative_mec_angle = (float)mec_fb_int / 10000.0f;
}
