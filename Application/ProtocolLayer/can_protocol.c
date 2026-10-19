#include "can_protocol.h"
#include "community.h"

/**
 *  @brief  CAN1 接收数据
 */
uint32_t rx_id_can1;
uint32_t rx_id_can2;
void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	rx_id_can1 = rxId;
	switch (rxId)
	{
		
		case 0x0b:
		{
			L_Wheel.rx(&L_Wheel, rxBuf);
			
			break;
		}
		
		/*
		case 0x011://接收ID
		{
			Yaw_Motor.rx(&Yaw_Motor, rxBuf);
			break;
		}
		*/
		case 0x205:
		{
			R_Fric.rx(&R_Fric, rxBuf);
			break;
		}
		

		default:
			break;
	}
}
/**
 *  @brief  CAN2 接收数据
 */
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
	rx_id_can2 = canId;
	
	switch (canId)
	{
		case 0x12://接收ID
		{
			Yaw_Motor.rx(&Yaw_Motor, rxBuf);
			Yaw_Motor.rx_info->data_recieve = 1;
			break;
		}
		case 0x123:
		{
			rc_rx_update(&rx_data,rxBuf);
			break;
		}
		
		
		default:
			break;
	}
}
