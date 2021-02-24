/*
 * uvc_control.h
 *
 *  Created on: Jan 23, 2021
 *      Author: Nikita Mikhailovskiy
 */

#ifndef INC_UVC_CONTROL_H_
#define INC_UVC_CONTROL_H_

#include "stdint.h"
#include "ros.h"

// Used pins, set-up: [UVC left1, UVC left2, UVC right1, UVC right2, Fans]
#define USED_PINS RL0_Pin, RL1_Pin, RL2_Pin, RL3_Pin, RL4_Pin
//Turn-off timeouts *100 ms
#define TIMEOUTS 30, 30, 30, 30, 0

typedef struct {
	uint8_t timeout_arr[5];
	bool current_status[5];
} uvc_task_t;

uint32_t UVCManagerTaskCreate(ros::NodeHandle *nh);
void disableAll();

#endif /* INC_UVC_CONTROL_H_ */
