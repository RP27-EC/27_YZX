#include "community_task.h"
#include "GIMB.h"

void StartCommunityTask(void const *argument)
{


	for (;;)
	{
		send_gimb_turn();
		osDelay(5);
	}
}
