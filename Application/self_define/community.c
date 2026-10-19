#include "community.h"
#include "string.h"


rx_msg_t rx_data = {.rc_stata = 0, .set_back_flag = 0};//保存底板发送的数据


//接收底盘数据
void rc_rx_update(rx_msg_t *rx_data,uint8_t *rxBuf)
{
		if (rx_data == NULL || rxBuf == NULL) 
		{
			return; 
		}
		
		rx_data->control_modle = rxBuf[3];
		//机械模式
		if (rx_data->control_modle == chass)
		{
			int16_t yaw_increment_int;
			memcpy(&yaw_increment_int, &rxBuf[0], 2);
			rx_data->yaw_msg = yaw_increment_int/100.0f;
			rx_data->yaw_msg_new = 1;
		}
        //陀螺仪模式
		else if (rx_data->control_modle == imu)
		{
			memcpy(&rx_data->ch0, &rxBuf[0], 2);
		
		}
        rx_data->rc_stata = rxBuf[4];
        
	
		rx_data->set_back_flag = rxBuf[7];
         
		
		rx_data->offline_cnt = 0;
		
		
}


