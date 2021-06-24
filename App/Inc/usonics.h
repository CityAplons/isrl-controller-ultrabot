/*
 * usonics.h
 *
 *  Created on: Jan 23, 2021
 *      Author: Nikita Mikhailovskiy
 */

#ifndef INC_USONICS_H_
#define INC_USONICS_H_

#include "stdint.h"
#include "ros.h"

#define NUMBER_OF_SENSORS 		8
#define SENSORS_PER_CHANNEL 	4
#define RX_BUFFER_LENGTH 		4

typedef enum us_err {
	OK,
	BAD_CONNECTION,
	READ_ERROR,
	BAD_CRC
} us_err_t;

typedef struct {
		uint16_t data[NUMBER_OF_SENSORS];
		uint8_t *front_rx_buffer;
        uint8_t *rear_rx_buffer;
        uint8_t front_counter;
        uint8_t rear_counter;
        uint8_t req;
        uint8_t state;
        us_err_t front_err;
        us_err_t rear_err;
} us_task_t;

extern __IO osEventFlagsId_t us_event_flag;

uint32_t UsonicManagerTaskCreate(ros::NodeHandle *nh);

#endif /* INC_USONICS_H_ */
