#ifndef __COMMUNITY_H_
#define __COMMUNITY_H_



#include <stdint.h>

enum gimb_forward{head,back};
enum work_modle{chass,imu};
//通信相关数据
typedef struct
{
	//机械模式
	float yaw_msg;
	uint8_t rc_stata;
	float current_yaw;
	float last_yaw;
	uint8_t gimb_forward;
	uint8_t set_back_flag;
	uint8_t control_modle;
	float relative_mec_angle;
}board_t;

void board_init(void);
void board_update_send(void);
extern board_t board;
void board_rx(board_t *board, uint8_t *rxBuf);
#endif
