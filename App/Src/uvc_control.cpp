/*
 * uvc.cpp
 *
 *  Created on: Jan 23, 2021
 *      Author: Nikita Mikhailovskiy
 */

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
}

#include "runbot_stm/relay.h"
#include "uvc_control.h"

static uvc_task_t *uvc_data;

extern __IO uint8_t ros_synced;

void disableAll() {
	HAL_GPIO_WritePin(GPIOE,
			RL0_Pin | RL1_Pin | RL2_Pin | RL3_Pin | RL4_Pin | RL5_Pin | RL6_Pin,
			GPIO_PIN_RESET);
}

static void updateStates() {
	for (uint8_t i = 0; i < 7; i++) {
		if (uvc_data->timeout_arr[i] > 0)
			uvc_data->timeout_arr[i]--;
	}
}

static void toggle_uvc(bool *states) {
	static uint16_t pins[7] = { USED_PINS };
	static uint8_t times[7] = { TIMEOUTS };
	for (uint8_t i = 0; i < 7; i++) {
		if (!states[i] && uvc_data->current_status[i]
				&& (uvc_data->timeout_arr[i] <= 0)) {
			HAL_GPIO_WritePin(GPIOE, pins[i], GPIO_PIN_RESET);
			uvc_data->current_status[i] = false;
		}
		if (states[i] && !uvc_data->current_status[i]) {
			HAL_GPIO_WritePin(GPIOE, pins[i], GPIO_PIN_SET);
			uvc_data->current_status[i] = true;
			uvc_data->timeout_arr[i] = times[i];
		}
	}
}

static void uvc_cb(const runbot_stm::relayRequest &req,
		runbot_stm::relayResponse &resp) {
	// Turn-off timer handling
	bool reqs[7] = { req.relay1, req.relay2, req.relay3, req.relay4, req.relay5,
			req.relay6, req.relay7 };
	toggle_uvc(reqs);
	memcpy(resp.status, uvc_data->current_status,
			sizeof(uvc_data->current_status));
	memcpy(resp.timeout, uvc_data->timeout_arr, sizeof(uvc_data->timeout_arr));
}

static ros::NodeHandle *nh_;

ros::ServiceServer<runbot_stm::relayRequest, runbot_stm::relayResponse> uvc_service(
		"stm/relay", &uvc_cb);

void uvcControlTask(void *argument) {
	//Initializing data structure
	uvc_task_t td = { .timeout_arr = { 0, 0, 0, 0, 0, 0, 0 }, .current_status =
			{ 0, 0, 0, 0, 0, 0, 0 } };
	uvc_data = &td;

	for (;;) {
		if (ros_synced) {
			updateStates();
			osDelay(100);
		} else {
			disableAll();
			osDelay(1000);
		}
	}
}

/*
 * Create task
 */
uint32_t UVCManagerTaskCreate(ros::NodeHandle *nh) {
	nh_ = nh;
	nh_->advertiseService<runbot_stm::relayRequest, runbot_stm::relayResponse>(
			uvc_service);

	osThreadId_t UVCManagerHandle;
	const osThreadAttr_t uvc_control_attributes =
			{ name : "uvc_control", .attr_bits = osThreadDetached, .cb_mem =
					NULL, .cb_size = 0, .stack_mem = NULL,
					.stack_size = 200 * 4, .priority =
							(osPriority_t) osPriorityHigh, .tz_module = 0,
					.reserved = 0 };

	UVCManagerHandle = osThreadNew(uvcControlTask, NULL,
			&uvc_control_attributes);

	if (NULL == UVCManagerHandle) {
		return 1;
	}
	return 0;

}

