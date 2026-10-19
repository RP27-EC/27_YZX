/**
  ******************************************************************************
  * @file    control_task.c
  * @brief   
  ******************************************************************************
  */
#include "control_task.h"
#include "community.h"
#include "rc_sensor.h"
#include "GIMB.h"


float t;
void StartControlTask(void const * argument)
{
	//初始化电机
	gimb_init(&Yaw_Motor);

	for(;;) 
	{
		static uint8_t last_online = 0;
		uint8_t now_online = (rx_data.rc_stata == 0 && rx_data.offline_cnt < 100);
		//判断是否重连
		if (now_online && !last_online)
		{
			//遥控器重连重新初始化
			gimb_reinit();
		}
		last_online = now_online;
		if ((imu_sensor.work_state.err_code == IMU_NONE_ERR) || 
				(imu_sensor.work_state.err_code == IMU_DATA_CALI))
		{
			imu_sensor.update(&imu_sensor);
			
		}
		//掉线计数器
		if (rx_data.offline_cnt < 1000)
		{
			rx_data.offline_cnt ++;
		}
		//判断在线工作
		if ( rx_data.rc_stata == 0 && rx_data.offline_cnt < 100)
		{
			
			work(&Yaw_Motor);
		
		}
		else
		{
			//断控保护
			Yaw_Motor.tx_info->target_speed = 0.0f;
			Yaw_Motor.tx_info->Kp = 0;
			Yaw_Motor.tx_info->Kd = 0;
			Yaw_Motor.single_set_speed(&Yaw_Motor); // 内部会发送 0xFC 使能命令
		
		}
		
		osDelay(1);
	}
}



