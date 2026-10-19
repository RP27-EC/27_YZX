/**
  ******************************************************************************
  * @file    control_task.c
  * @brief   ��������,����������̨�ͷ������
  ******************************************************************************
  */
#include "control_task.h"
#include "gimbal_motor.h"
#include "Judge.h"
#include "chassis.h"


extern Motor_RM_Group_t chassis_Group;

void StartCtrlTask(void const * argument)
{
	
	for(;;)
	{
		
		Yaw_Motor.single_sleep(&Yaw_Motor);
		Yaw_Motor.tx_info->torque = 0.f;
		Yaw_Motor.single_set_torque(&Yaw_Motor);
		chassis_work(&chassis_Group);
		osDelay(1);
	}
}
