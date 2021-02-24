/*
 * led_control.h
 *
 *  Created on: Jan 23, 2021
 *      Author: Nikita Mikhailovskiy
 */

#ifndef INC_LED_H_
#define INC_LED_H_

#include "stdint.h"
#include "ros.h"

uint32_t LEDManagerTaskCreate(ros::NodeHandle *nh);
void led_set_color(uint8_t r, uint8_t g, uint8_t b);

#endif /* INC_LED_H_ */
