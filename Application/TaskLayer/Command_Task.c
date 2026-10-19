/**
  ******************************************************************************
  * @file    Command_task.c
  * @brief   指令更新任务
  *          更新整车标志位和模式
  ******************************************************************************
  */
#include "Command_Task.h"
#include "rc_sensor.h"
#include "rc_protocol.h"
#include "community.h"
void StartCommandTask(void const * argument)
{
	 board_init();
	
	for(;;)
	{
//		keyboard_update(rc_sensor.info);
		board_update_send();
		osDelay(1);
	}
}
