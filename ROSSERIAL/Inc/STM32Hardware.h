#ifndef ROS_STM32_HARDWARE_H_
#define ROS_STM32_HARDWARE_H_

#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"

extern RingBuffer rb;

class STM32Hardware
{
public:
	STM32Hardware() {}
	void init() { RingBuffer_Init(&rb); }
	// Read a byte of data from ROS connection.
	// If no data , returns -1
	int read(){ return RingBuffer_ReadByte(&rb); }
	// Send a byte of data to ROS connection
	void write(uint8_t* data, int length){ while(CDC_Transmit_FS(data, length) != USBD_OK); }
	// Returns milliseconds since start of program
	unsigned long time(){ return HAL_GetTick(); }
};

#endif
