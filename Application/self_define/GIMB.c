/******************************************************************
云台工作函数
1. 结构体数据初始化
2.float normalize_angle(float angle) 云台限幅函数，将角度限制在+-360
3，void gimb_init(Motor_DM_t* Yaw_Motor)/电机初始化归位
4.void send_gimb_turn(void)/发送电机的朝向和相对角度给底板
5.// 重连初始化 void gimb_reinit(void)
************************************************************************/
#include "GIMB.h"
#include "imu_sensor.h"
#include "community.h"
#include "DM_Motor.h"
#include <stdbool.h>
#include "drv_can.h"

gimb_t gimb_data;
uint8_t chassis_stop_t = 0;
uint8_t last_tw_step_change = 0; // 记录上一次的波轮跳变状态
uint16_t turn_cooldown_timer = 0; // 掉头冷却计时器 	
uint8_t last_offline_state = 0;
uint8_t current_offline_state = 0;
float target_ramp = 0.0f;
float v_cmd = 0.0f;
uint8_t is_first_frame = 0;
uint8_t last_control_model = 0;
//云台限幅函数，将角度限制在+-360
float normalize_angle(float angle) 
{
    while (angle > PI) 
	{
		angle -= 2.0f * PI;
	}		
    while (angle < -PI) 
	{
		angle += 2.0f * PI;
	}
    return angle;
}

//电机初始化归位
void gimb_init(Motor_DM_t* Yaw_Motor)
{
	gimb_data.yaw_out = 0;
	
	gimb_data.last_ZeroPosSensor_flag = 0;
	
	gimb_data.ZeroPosSensor_flag  = 0;
	
	gimb_data.is_init = unset_zero;
	
	gimb_data.is_ready = false;
	
	gimb_data.gimb_forward = head;
	
	Yaw_Motor->rx_info->data_recieve = 0;
	
	gimb_data.max_speed = 1.03;
	
	gimb_data.target_speed = 0.0f;
	
	gimb_data.Angle_pid.kp = 10;
	gimb_data.Angle_pid.ki = 0;
	gimb_data.Angle_pid.target = 0;
	gimb_data.Angle_pid.measure = 0;
	gimb_data.Angle_pid.err = 0;
	gimb_data.Angle_pid.integral_max = 100;
	gimb_data.Angle_pid.out_max = 30;
	
	
	gimb_data.speed_pid.kp = 0;
	gimb_data.speed_pid.ki = 0;
	gimb_data.speed_pid.target = 0;
	gimb_data.speed_pid.measure = 0;
	gimb_data.speed_pid.err = 0;
	gimb_data.speed_pid.integral_max = 100;
	gimb_data.speed_pid.out_max = 30;
	
	gimb_data.is_ready = true;                    // 电机成功初始化
    gimb_data.is_init  = set_zero;                // 0
}

//云台工作函数
void work( Motor_DM_t* Yaw_Motor)
{
	//机械模式
	 if (rx_data.control_modle == chass)
	 {
		 //模式切换检测
		 if (last_control_model != rx_data.control_modle)
		 {
			gimb_data.Angle_pid.kp = 10;
			gimb_data.Angle_pid.ki = 0;
			gimb_data.Angle_pid.target = 0;
			gimb_data.Angle_pid.measure = 0;
			gimb_data.Angle_pid.err = 0;
			gimb_data.Angle_pid.integral_max = 100;
			gimb_data.Angle_pid.out_max = 30;
			
			
			gimb_data.speed_pid.kp = 0;
			gimb_data.speed_pid.ki = 0;
			gimb_data.speed_pid.target = 0;
			gimb_data.speed_pid.measure = 0;
			gimb_data.speed_pid.err = 0;
			gimb_data.speed_pid.integral_max = 100;
			gimb_data.speed_pid.out_max = 30;
			 
		 }
	 
	
		//抓第一帧,云台自动回正
		if (is_first_frame == 0)
		{
			 if (Yaw_Motor->rx_info->data_recieve == 0)
			 {
				return;
			 }
			 float current_real_angle = normalize_angle(Yaw_Motor->rx_info->motor_angle - offest);
			 gimb_data.Angle_pid.target  = 0.0f;
			 gimb_data.Angle_pid.measure = current_real_angle;
			 gimb_data.Angle_pid.integral = 0.0f;
			 target_ramp = current_real_angle;
			 is_first_frame = 1;
		 }
		//超时判断
		if (rx_data.offline_cnt > 100) 
		{
			current_offline_state = 0;
			chassis_stop_t = 0;
		}
		else
		{
			current_offline_state = 1;
			
		}
		
		last_offline_state = current_offline_state;
		//yaw电机当前多圈角度
		gimb_data.yaw_current = Yaw_Motor->rx_info->motor_angle_sum;
		
		if (turn_cooldown_timer > 0)
		{
			turn_cooldown_timer --;
		}
		
		
		
		//正式工作函数
		if (gimb_data.is_init == set_zero  )
		{
			
			
			if (gimb_data.is_ready == true)
			{
				
				// 判断上升沿（之前是0现在是1）且处于非冷却期（做波轮控制掉头）
				if (rx_data.set_back_flag == 1 && last_tw_step_change == 0)
				{
					float real_angle = normalize_angle(Yaw_Motor->rx_info->motor_angle - offest); 
					
					// 1. 翻转朝向状态
					if (gimb_data.gimb_forward  == head )
					{
						gimb_data.gimb_forward = back;
						
					}
					else
					{
						gimb_data.gimb_forward = head;
					}
					
					if (gimb_data.gimb_forward == back) 
					{
						if (real_angle >= 0.0f) 
						{
							// 如果当前角度是正的（比如 170°）
							gimb_data.Angle_pid.target = PI;   // 目标设为 +180°
						} 
						else 
						{
							// 如果当前角度是负的（比如 -170°）
							gimb_data.Angle_pid.target = -PI;  // 目标设为 -180°
						}
					} 
					else 
					{
						gimb_data.Angle_pid.target = 0.0f;
					}
					
					//积分清零
					gimb_data.Angle_pid.integral = 0.0f;
					
					gimb_data.Angle_pid.target = normalize_angle(gimb_data.Angle_pid.target); 
					
					//制底盘停止计时器清零
					chassis_stop_t = 0; 
					
					//冷却防止云台短时间多次旋转
					turn_cooldown_timer = 500; 
				}
				// 更新波轮记录状态
				last_tw_step_change = rx_data.set_back_flag;
				
				//	接收到新数据		
				if (rx_data.yaw_msg_new)
				{
					rx_data.yaw_msg_new = 0;
					gimb_data.Angle_pid.target -= rx_data.yaw_msg*0.017453f;//单位转化
					gimb_data.Angle_pid.target = normalize_angle(gimb_data.Angle_pid.target);
				}
				
				//跳变死区
				if ( fabs(rx_data.yaw_msg )< 0.05f)
				{
					 chassis_stop_t ++;
					if (chassis_stop_t > 200)
					{
						chassis_stop_t = 200;
					}
				}
				else
				{
					chassis_stop_t = 0;
				}
			 
			
			//自动掉头
			//判断冷却和底盘电机静止
			if ( turn_cooldown_timer == 0 && fabs(Yaw_Motor->rx_info->speed) < 0.2f)
			{
				
				float real_angle = normalize_angle(Yaw_Motor->rx_info->motor_angle - offest);
				float d0 = fabsf(normalize_angle(0.0f - real_angle));
				float d1 = fabsf(normalize_angle(PI - real_angle));
				const float hyst = 0.035f;//延迟带防止抖动
				
				//	记录云台朝向			
				uint8_t new_state = gimb_data.gimb_forward;
				
				//判断靠近哪一头
				if (d1 + hyst < d0) 
				{
					new_state = back;
				}
					
				 else if (d0 + hyst < d1) 
				{
					new_state = head;
				}
				
				//输出目标值
				float new_target;
				if (new_state == head)
				{
					new_target = 0.0f;
				}
				else
				{
					if (real_angle < 0)
					{
						new_target = -PI;
					}
					else 
					{
						new_target = PI;
					}
				}
				
				//判断发生转头
				if (new_state != gimb_data.gimb_forward)
				{
					turn_cooldown_timer = 500;
					gimb_data.Angle_pid.integral = 0.0f;
				}
				//阶梯算法防止突然给大输出
				if (fabsf(normalize_angle(new_target - gimb_data.Angle_pid.target)) > 0.01f)
				{
					target_ramp = real_angle;              // 目标变了，从当前角度起坡
					gimb_data.Angle_pid.integral = 0.0f;
				}
				gimb_data.gimb_forward = new_state;
				gimb_data.Angle_pid.target = new_target;
			
							
				
			}
				
			//目标斜坡防止误差一开始给过大
			float target_diff = normalize_angle(gimb_data.Angle_pid.target - target_ramp);
			const float max_step = 0.005f; //步长
			
			if (target_diff > max_step)  
			{
				target_diff = max_step;
			}
			if (target_diff < -max_step) 
			{
				target_diff = -max_step;
			}
			target_ramp = normalize_angle(target_ramp + target_diff);
			
			//位置环pid
			float true_angle_fb = Yaw_Motor->rx_info->motor_angle - offest;
			true_angle_fb = normalize_angle(Yaw_Motor->rx_info->motor_angle - offest);
			gimb_data.Angle_pid.measure = true_angle_fb ;
			gimb_data.Angle_pid.err = normalize_angle(target_ramp 
								- gimb_data.Angle_pid.measure);
			
			single_pid_ctrl(&gimb_data.Angle_pid);
			
			gimb_data.speed_pid.target = gimb_data.Angle_pid.out;
			
			//输出限幅
			if (gimb_data.Angle_pid.out > 10.0f) 
			{
				gimb_data.Angle_pid.out = 10.0f;
			}
			if (gimb_data.Angle_pid.out < -10.0f) 
			{	
				gimb_data.Angle_pid.out = -10.0f;
			}
			
			

			float v_target = gimb_data.Angle_pid.out;   // 位置环输出，rad/s
			const float max_dv = 0.05f;                 // rad/s per 1ms => 50 rad/s2
			float dv = v_target - v_cmd;
			//限幅
			if (dv >  max_dv) 
			{
				dv =  max_dv;
			}
			if (dv < -max_dv) 
			{
				dv = -max_dv;
			}
			v_cmd += dv;
			
			Yaw_Motor->tx_info->target_speed = v_cmd;
			
			Yaw_Motor->tx_info->Kd = 0.5f;
			Yaw_Motor->single_set_speed(Yaw_Motor);
			//
			last_control_model = rx_data.control_modle;
			}
		}
	}
	 //陀螺仪模式
	 else if(rx_data.control_modle == imu)
	 {
		 //模式改变就将机械模式数据清零
		 if (last_control_model != rx_data.control_modle)
		 {
			send_gimb_turn();
			 
		 }
		 //接收遥控器通道数据
		gimb_data.ch0_data = rx_data.ch0;
		if (abs(gimb_data.ch0_data) < 15)
		{
			gimb_data.ch0_data = 0;
		}
		gimb_data.target_speed = ((float)gimb_data.ch0_data / 660.0f) * gimb_data.max_speed;
		
		
		// 积分
		gimb_data.target_angle += gimb_data.target_speed * 0.001f;
		
		// 限幅 ±PI
		gimb_data.target_angle = normalize_angle(gimb_data.target_angle);
		
		
		//位置环控制
		gimb_data.Angle_pid.target = gimb_data.target_angle;
		gimb_data.Angle_pid.measure = imu_sensor.info->base_info.yaw * 1.0466f; // 转弧度
		gimb_data.Angle_pid.err = normalize_angle( gimb_data.Angle_pid.target - gimb_data.Angle_pid.measure);
		single_pid_ctrl(&gimb_data.Angle_pid);
		
		//输出
		Yaw_Motor->tx_info->target_speed = gimb_data.Angle_pid.out;
 		Yaw_Motor->tx_info->Kd = 2.0f;
		Yaw_Motor->single_set_speed(Yaw_Motor);
		
		last_control_model = rx_data.control_modle;
	 }
}

//发送电机的朝向和相对角度给底板
void send_gimb_turn(void)
{
	uint8_t send_tx_buff[8] = {0};
	send_tx_buff[0] = gimb_data.gimb_forward;

	//发送单圈角度
	gimb_data.current_motor_angle = Yaw_Motor.rx_info->motor_angle;
	gimb_data.relative_mec_angle = gimb_data.current_motor_angle - offest;

	//限幅
	gimb_data.relative_mec_angle = normalize_angle(gimb_data.relative_mec_angle); 

	//压缩打包
	int16_t mec_fb_int = (int16_t)(gimb_data.relative_mec_angle * 10000.0f); 
	send_tx_buff[1] = mec_fb_int >> 8;
	send_tx_buff[2] = mec_fb_int & 0xFF;
	send_tx_buff[7] = 0xA5; // 校验位
		
	CAN2_SendData(0x456,send_tx_buff);
}

// 重连初始化 
void gimb_reinit(void)
{
	gimb_data.gimb_forward = head;
	gimb_data.Angle_pid.target = 0.0f;
	gimb_data.Angle_pid.integral = 0.0f;
	gimb_data.Angle_pid.out = 0.0f;
	turn_cooldown_timer = 0;
	chassis_stop_t = 0;
	last_tw_step_change = rx_data.set_back_flag;
	target_ramp = normalize_angle(Yaw_Motor.rx_info->motor_angle - offest);;
    v_cmd = 0.0f;
	is_first_frame = 0;
}
