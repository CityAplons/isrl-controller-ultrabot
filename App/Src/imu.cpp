/*
 * imu.cpp
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

#include "imu.h"

static ros::NodeHandle *nh_;

void imuTask(void * argument)
{
  for(;;)
  {
	 osDelay(500);
  }
}

/*
 * Create task
 */
uint32_t IMUManagerTaskCreate(ros::NodeHandle *nh)
{
	nh_ = nh;

	osThreadId_t imuManagerHandle;
	const osThreadAttr_t imu_attributes = {
	  	name : "IMU",
		.attr_bits = osThreadDetached,
		.cb_mem = NULL,
		.cb_size = 0,
		.stack_mem = NULL,
		.stack_size = 128 * 4,
	  	.priority = (osPriority_t) osPriorityNormal,
		.tz_module = 0,
		.reserved = 0
	 };

	imuManagerHandle = osThreadNew(imuTask, NULL, &imu_attributes);

	if (NULL == imuManagerHandle){ return 1; }
	return 0;

}
