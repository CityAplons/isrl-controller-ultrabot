/*
 * spin.cpp
 *
 *  Created on: Jan 23, 2021
 *      Author: Nikita Mikhailovskiy
 */

extern "C"
{
	#include "FreeRTOS.h"
	#include "task.h"
	#include "cmsis_os.h"
}

#include "spin.h"

static ros::NodeHandle *nh_;

void RosSpinTask(void * argument)
{
  for(;;)
  {
	  nh_->spinOnce();
	 // osDelay(1);
  }
}

/*
 * Create spin task
 */
uint32_t RosSpinTaskCreate(ros::NodeHandle *nh)
{
	nh_ = nh;

	osThreadId_t RosSpinHandle;
	const osThreadAttr_t rosSpin_attributes = {
	  	name : "rosSpin",
		.attr_bits = osThreadDetached,
		.cb_mem = NULL,
		.cb_size = 0,
		.stack_mem = NULL,
		.stack_size = 128 * 4,
	  	.priority = (osPriority_t) osPriorityNormal,
		.tz_module = 0,
		.reserved = 0
	 };

	RosSpinHandle = osThreadNew(RosSpinTask, NULL, &rosSpin_attributes);

	if (NULL == RosSpinHandle){ return 1; }
	return 0;

}


