/*
 * led.cpp
 *
 *  Created on: Jan 23, 2021
 *      Author: Nikita Mikhailovskiy
 */

extern "C"
{
	#include "tim.h"
	#include "FreeRTOS.h"
	#include "task.h"
	#include "cmsis_os.h"
}

#include "led.h"
#include "ultrabot_stm/led.h"

static ros::NodeHandle *nh_;

void led_set_color(uint8_t r, uint8_t g, uint8_t b)
{
	TIM4->CCR1 = r * 0xFF - 2;
	TIM4->CCR2 = g * 0xFF - 2;
	TIM4->CCR3 = b * 0xFF - 2;
}

static void color_cb(const ultrabot_stm::ledRequest& req,
							ultrabot_stm::ledResponse& resp)
{
   //usb_lock();
   TIM4->CCR1 = (uint32_t)(req.color.a * req.color.r * 0xFFFE);
   TIM4->CCR2 = (uint32_t)(req.color.a * req.color.g * 0xFFFE);
   TIM4->CCR3 = (uint32_t)(req.color.a * req.color.b * 0xFFFE);
   resp.result = true;
   //usb_unlock();
}

ros::ServiceServer<ultrabot_stm::ledRequest,
					ultrabot_stm::ledResponse> rgb_service("stm/led", &color_cb);

void ledControlTask(void * argument)
{
	MX_TIM4_Init();
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); // RED
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2); // GREEN
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // BLUE
	led_set_color(0xFF,0xFF,0xFF);

	uint8_t breath_value = 10;
	int8_t direction = 0;
	for(;;)
	{
		if (!nh_->connected()){
			if (breath_value <= 20) direction = 5;
			if (breath_value >= 240) direction = -5;
			breath_value += direction;
			led_set_color(breath_value,breath_value,breath_value);
			osDelay(30);
		} else {
			osDelay(1000);
		}

	}
}

/*
 * Create task
 */
uint32_t LEDManagerTaskCreate(ros::NodeHandle *nh)
{
	nh_ = nh;
	nh_->advertiseService<ultrabot_stm::ledRequest,
	ultrabot_stm::ledResponse>(rgb_service);

	osThreadId_t LEDManagerHandle;
	const osThreadAttr_t led_control_attributes = {
	  	name : "led_control",
		.attr_bits = osThreadDetached,
		.cb_mem = NULL,
		.cb_size = 0,
		.stack_mem = NULL,
		.stack_size = 128 * 4,
	  	.priority = (osPriority_t) osPriorityHigh,
		.tz_module = 0,
		.reserved = 0
	 };

	LEDManagerHandle = osThreadNew(ledControlTask, NULL, &led_control_attributes);

	if (NULL == LEDManagerHandle){ return 1; }
	return 0;

}


