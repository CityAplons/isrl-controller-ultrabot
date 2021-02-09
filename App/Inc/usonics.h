/*
 * usonics.h
 *
 *  Created on: Jan 23, 2021
 *      Author: nmiha
 */

#ifndef INC_USONICS_H_
#define INC_USONICS_H_

#include "stdint.h"
#include "ros.h"

#define NUMBER_OF_SENSORS 		10
#define SENSORS_PER_CHANNEL 	5
#define RX_BUFFER_LENGTH 		4

typedef struct {
		uint16_t data[NUMBER_OF_SENSORS];
		uint8_t *front_rx_buffer;
        uint8_t *rear_rx_buffer;
        uint8_t front_counter;
        uint8_t rear_counter;
        osEventFlagsId_t us_flag;
        uint8_t req;
} us_task_t;

uint32_t UsonicManagerTaskCreate(ros::NodeHandle *nh);
void DMA1_Stream2_IRQHandler(void);
void DMA1_Stream5_IRQHandler(void);

#endif /* INC_USONICS_H_ */
