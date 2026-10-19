#include "can_protocol.h"
#include "Chassis_Motor.h"
#include "gimbal_Motor.h"
#include "cap_protocol.h"
#include "judge.h"
#include "cap.h"
#include "community.h"

/**
 *  @brief  CAN1 接收数据
 */
 
 uint32_t can1_rx_ID;
 uint32_t can2_rx_ID;
 
void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	can1_rx_ID = rxId;
	switch (rxId)
	{
		case 0x201:
			chassis_Group.motor[0]->rx(chassis_Group.motor[0],rxBuf);
			break;
		
		case 0x202:
			chassis_Group.motor[2]->rx(chassis_Group.motor[2],rxBuf);
			break;
		
		case 0x203:
			chassis_Group.motor[1]->rx(chassis_Group.motor[1],rxBuf);
			break;
		
		case 0x204:
			chassis_Group.motor[3]->rx(chassis_Group.motor[3],rxBuf);
			break;
		case 0x012:
		Yaw_Motor.rx(&Yaw_Motor, rxBuf);
		break;
		default:
			break;
	}
}



/**
 *  @brief  CAN2 接收数据
 */
void CAN2_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	can2_rx_ID = rxId;
	switch (rxId)
	{
		case 0x203:
		chassis_Group.motor[2]->rx(chassis_Group.motor[2], rxBuf);
		break;
		case 0x202:
		chassis_Group.motor[3]->rx(chassis_Group.motor[3], rxBuf);
		break;
		case 0x001:
		Yaw_Motor.rx(&Yaw_Motor, rxBuf);
		break;
		case 0x456:
		board_rx(&board, rxBuf);
		break;
		default:
			break;
	}
}


/**
 *  @brief  CAN3 接收数据
 */
void CAN3_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		default:
			break;
	}
}
