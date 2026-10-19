#ifndef __GIMB_H_
#define __GIMB_H_

#include "motor.h"
#include "PID.h"

enum gimb_init_stata{set_zero,unset_zero};
enum gimb_forward{head,back};

typedef struct
{
	float yaw_target;
	float yaw_current;
	float yaw_out;
	uint8_t ZeroPosSensor_flag;
	uint8_t last_ZeroPosSensor_flag;
	uint8_t is_init;
	bool is_ready;
	pid_ctrl_t Angle_pid;
	pid_ctrl_t speed_pid;
	float current_yaw;
	uint8_t gimb_forward;
	pid_ctrl_t chass;
	
	//陀螺仪模式数据
	int16_t ch0_data;
	float max_speed;
	float target_speed;
	float target_angle;
	float current_motor_angle;
	float relative_mec_angle;
	pid_ctrl_t imu;
}gimb_t;



#define offest -0.38049f

void gimb_init(Motor_DM_t* Yaw_Motor);
void send_gimb_turn(void);
void work( Motor_DM_t* Yaw_Motor);
float normalize_angle(float angle);
extern gimb_t gimb_data;
void gimb_reinit(void);
#endif

