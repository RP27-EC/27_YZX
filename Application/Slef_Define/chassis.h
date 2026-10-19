#ifndef __CHASSIS_H_
#define __CHASSIS_H_


#include "Chassis_motor.h"
#include "rc_sensor.h"

#define rc_MAX_OUT 660.0f
#define MAX_OUT 3000.0f//以rpm为单位，限制最大转速
#define GIMB_FOLLOW_KP   (MAX_OUT / PI)      /* 955 rpm/rad，180° 对应满速 */
void chassis_init(Motor_RM_Group_t* chassis);
void chasiss_test(Motor_RM_Group_t* chassis);
void chassis_work(Motor_RM_Group_t* group);

enum chassis_state{ON,OFF};

//定义结构体用于底盘相关计算
typedef struct{
	float x_target;
	float y_target;
	float turn_target;
	
	float LF_wheel_out;
	float RF_wheel_out;
	float LB_wheel_out;
	float RB_wheel_out;
	
	float calculate[3];
	uint8_t  gimb_forward;
	
}chassis_t;

extern chassis_t chassis;
void chasiss_to_motor_caculate(chassis_t* chassis,rc_sensor_t* rc_sensor);
#endif 
