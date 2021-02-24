/*
 * power.h
 *
 *  Created on: Jan 23, 2021
 *      Author: Nikita Mikhailovskiy
 */

#ifndef INC_POWER_H_
#define INC_POWER_H_

#include "stdint.h"
#include "ros.h"

#define MAX_VOLTAGE 29.4f
#define MIN_VOLTAGE 22.0f
#define EM_VOLTAGE 21.0f
#define CONVERSION_MAX 4096
#define CS_I2C_ADDRESS (uint16_t)0x30
#define DATA_REG 0x0A

/*
 * battery_state: [0..100]% - or battery charge
 * voltage: voltage value [V]
 * current: instant current value in [mA]
 * power_consumption: calculated instant power consumption in mWh/sec
 */
typedef struct {
	uint8_t battery_state;
	float voltage;
	uint16_t voltage_raw;
	uint16_t current_raw;
	int16_t current;
	float power_consumption;
} power_task_t;

uint32_t PowerManagerTaskCreate(ros::NodeHandle *nh);

#endif /* INC_POWER_H_ */
