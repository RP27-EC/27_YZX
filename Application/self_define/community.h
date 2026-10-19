#ifndef __COMMUNITY_H_
#define __COMMUNITY_H_

#include <stdint.h>

//接收数据有关结构体
typedef struct
{
	//机械
	float yaw_msg;
	uint8_t rc_stata;
	uint16_t offline_cnt;
	uint8_t set_back_flag;
	volatile uint8_t yaw_msg_new;
	//陀螺仪
	uint8_t control_modle;
	uint16_t ch0;
}rx_msg_t;

enum control_modle{chass,imu};


void rc_rx_update(rx_msg_t *rx_data,uint8_t *rxBuf);

extern rx_msg_t rx_data;
#endif 
