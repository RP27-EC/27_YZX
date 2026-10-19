/*********************************************************************************
File Name  //down_car.c

Descripition: // use it to control revieve remote control data and control chassis

Function list: //void chasiss_init()
//void chasiss_update(Motor_RM_Group_t* chassis)
//void chasiss_test(Motor_RM_Group_t* chassis)
//void chasiss_to_motor_caculate(chassis_t* chassis_data,rc_sensor_t* rc_sensor)
//void chassis_work(Motor_RM_Group_t* group)

**********************************************************************************/




#include "chassis.h"
#include "PID.h"
#include "community.h"


//电机数据结构体用于做逆解算和赋值目标值
chassis_t chassis_data;
extern rc_sensor_t	rc_sensor;


//初始化底盘电机
void chassis_init(Motor_RM_Group_t* chassis)
{
	RM_Group_Motor_Init(chassis);
	chassis_data.gimb_forward = head;
}


//电机测试函数
void chasiss_test(Motor_RM_Group_t* chassis)
{
	chassis -> motor[0] -> ctrl -> speed_ctrl -> target = 1000.0f;
	chassis -> motor[0] ->single_set_speed(chassis -> motor[0]);
	chassis->motor[0]->single_set_torque(chassis->motor[0]);
	chassis->motor[1]->single_set_torque(chassis->motor[1]);
	chassis->motor[2]->single_set_torque(chassis->motor[2]);
	chassis->motor[3]->single_set_torque(chassis->motor[3]);
}




//逆解算函数，将遥控器的数据映射转速，然后算出电机应该输出的转速
void chasiss_to_motor_caculate(chassis_t* chassis_data,rc_sensor_t* rc_sensor)
{
	//用于计算各种运动的占比
	int x_move = abs(rc_sensor->info->ch3 );
	int y_move = abs(rc_sensor->info->ch2);
	int turn_move = abs(rc_sensor->info->ch0);
	
	if (board.control_modle == chass)
	{
		if (chassis_data->gimb_forward == head)
		{
			//将遥控器数据赋值给目标值
			chassis_data->x_target  = (rc_sensor->info->ch3)/rc_MAX_OUT * MAX_OUT;
			chassis_data->y_target  = -(rc_sensor->info->ch2)/rc_MAX_OUT * MAX_OUT;
			chassis_data->turn_target  = (rc_sensor->info->ch0)/rc_MAX_OUT * MAX_OUT;
		}
		else
		{
			chassis_data->x_target  =-(rc_sensor->info->ch3)/rc_MAX_OUT * MAX_OUT;
			chassis_data->y_target  = (rc_sensor->info->ch2)/rc_MAX_OUT * MAX_OUT;
			chassis_data->turn_target  = (rc_sensor->info->ch0)/rc_MAX_OUT * MAX_OUT;
		
		}
	
	}
	else
	{
		chassis_data->x_target  =-(rc_sensor->info->ch3)/rc_MAX_OUT * MAX_OUT;
		chassis_data->y_target  = (rc_sensor->info->ch2)/rc_MAX_OUT * MAX_OUT;
		

		float turn = -board.relative_mec_angle * GIMB_FOLLOW_KP;
		chassis_data->turn_target = constrain(turn, -MAX_OUT, MAX_OUT);
	
	}
	float total_out = x_move + y_move + turn_move;
	
	if (total_out == 0)
	{
		chassis_data->LF_wheel_out = 0;
		chassis_data->RF_wheel_out = 0;
		chassis_data->LB_wheel_out = 0;
		chassis_data->RB_wheel_out = 0;
		
	}
	//根据占比对输出进行赋值
	else
	{
		chassis_data->LF_wheel_out = (x_move/total_out* chassis_data->x_target) - 
								(y_move/total_out* chassis_data->y_target) + 
								(turn_move/total_out* chassis_data->turn_target);
		chassis_data->RF_wheel_out = (x_move/total_out* chassis_data->x_target) + 
								(y_move/total_out* chassis_data->y_target) - 
								(turn_move/total_out* chassis_data->turn_target);
		chassis_data->LB_wheel_out = (x_move/total_out* chassis_data->x_target) + 
								(y_move/total_out* chassis_data->y_target) + 
								(turn_move/total_out* chassis_data->turn_target);
		chassis_data->RB_wheel_out = (x_move/total_out* chassis_data->x_target) - 
								(y_move/total_out* chassis_data->y_target) - 
								(turn_move/total_out* chassis_data->turn_target);
	 
	}
}	

//电机工作函数
void chassis_work(Motor_RM_Group_t* group)
{
	//断控保护
	if (rc_sensor.work_state == DEV_ONLINE)
	{
			chassis_data.gimb_forward = board.gimb_forward;
			chasiss_to_motor_caculate(&chassis_data,&rc_sensor);
			group->motor[0] -> ctrl -> speed_ctrl -> target = chassis_data.LF_wheel_out;
			group->motor[1] -> ctrl -> speed_ctrl -> target = -chassis_data.RF_wheel_out;
			group->motor[2] -> ctrl -> speed_ctrl -> target = chassis_data.LB_wheel_out;
			group->motor[3] -> ctrl -> speed_ctrl -> target = -chassis_data.RB_wheel_out;
			group -> motor[0] ->single_set_speed(group -> motor[0]);
			group -> motor[1] ->single_set_speed(group -> motor[1]);
			group -> motor[2] ->single_set_speed(group -> motor[2]);
			group -> motor[3] ->single_set_speed(group -> motor[3]);
			group -> group_set_torque (group);
		
		
	}
	//断控全部输出为0
	else
	{
		group->motor[0]->single_sleep(group->motor[0]);
		group->motor[1]->single_sleep(group->motor[1]);
		group->motor[2]->single_sleep(group->motor[2]);
		group->motor[3]->single_sleep(group->motor[3]);
		
	}
}

