/*
 * usonics.cpp
 *
 *  Created on: Jan 23, 2021
 *      Author: Nikita Mikhailovskiy
 */

extern "C" {
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
}

#include "usonics.h"
#include "std_msgs/UInt16MultiArray.h"

__IO osEventFlagsId_t us_event_flag;

static ros::NodeHandle *nh_;
static us_task_t *thread_data;

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart2;

std_msgs::UInt16MultiArray msg;
ros::Publisher usonics("stm/rangefinders", &msg);

static uint8_t check_crc(uint8_t *data) {
	if (data[0] == 0xFF) {
		uint8_t crc = (uint8_t) (data[1] + data[2]);
		if (crc == data[3])
			return 1;
	}
	return 0;
}

void UsonicManagerTask(void *argument) {
	//Initializing data structure
	us_task_t td;
	td.front_rx_buffer = new uint8_t[RX_BUFFER_LENGTH];
	td.rear_rx_buffer = new uint8_t[RX_BUFFER_LENGTH];
	td.front_counter = 0;
	td.rear_counter = 0;
	td.req = 0x55; // Ultrasonic request message
	td.state = 0;
	thread_data = &td;

	uint32_t flags = 0b0;

	msg.layout.dim_length = 1;
	msg.layout.data_offset = 0;
	msg.layout.dim = new std_msgs::MultiArrayDimension;
	msg.layout.dim[0].label = "mm";
	msg.layout.dim[0].size = NUMBER_OF_SENSORS;
	msg.layout.dim[0].stride = 1 * NUMBER_OF_SENSORS;
	msg.data_length = NUMBER_OF_SENSORS;
	msg.data = (uint16_t*) malloc(sizeof(uint16_t) * NUMBER_OF_SENSORS);

	// Hardware peripheral initialization
	MX_DMA_Init();
	MX_USART2_UART_Init();
	MX_UART4_Init();

	for (;;) {
		if (nh_->connected()) {

			if (td.state == 0) {
				//Registering buffers to receive sensors data
				if (HAL_UART_Receive_DMA(&huart2, td.front_rx_buffer,
				RX_BUFFER_LENGTH) != HAL_OK) {
					Error_Handler();
				}
				if (HAL_UART_Receive_DMA(&huart4, td.rear_rx_buffer,
				RX_BUFFER_LENGTH) != HAL_OK) {
					Error_Handler();
				}
				// Sending first requests
				HAL_UART_Transmit_IT(&huart2, &td.req, 1);
				HAL_UART_Transmit_IT(&huart4, &td.req, 1);

				td.state = 1;
			}

			//Wait for interrupt response
			flags = osEventFlagsWait(us_event_flag, 0b11, osFlagsWaitAll, 1000);

			// Waiting for flags from both interrupts
			usb_lock();
			if (flags == 0b11) {
				memcpy(msg.data, td.data, sizeof(uint16_t) * NUMBER_OF_SENSORS);
				usonics.publish(&msg);
				flags = osEventFlagsClear(us_event_flag, 0b11);
			} else if (flags == 0b01) {
				char msg_arr[16 + sizeof(td.front_counter)];
				strncpy(msg_arr, "Front US stuck: ", sizeof(msg_arr) - 1);
				msg_arr[sizeof(msg_arr) - 1] = '\0';
				sprintf(&msg_arr[15], "%d", td.front_counter);
				nh_->logwarn(msg_arr);
				flags = osEventFlagsClear(us_event_flag, 0b01);
			} else if (flags == 0b10) {
				char msg_arr[15 + sizeof(td.rear_counter)];
				strncpy(msg_arr, "Rear US stuck: ", sizeof(msg_arr) - 1);
				msg_arr[sizeof(msg_arr) - 1] = '\0';
				sprintf(&msg_arr[14], "%d", td.rear_counter);
				nh_->logwarn(msg_arr);
				flags = osEventFlagsClear(us_event_flag, 0b10);
			} else if (flags == (uint32_t) osErrorTimeout) {
				nh_->logerror("Ultrasonic data read timeout!");
				osDelay(2000);
				flags = osEventFlagsClear(us_event_flag,
						(uint32_t) osErrorTimeout);
			} else {
				nh_->logerror("[UsonicManagerTask] task error!");
				osDelay(2000);
			}
			nh_->spinOnce();
			usb_unlock();
		} else {
			td.state = 0;
			osDelay(100);
		}
	}
}

uint32_t UsonicManagerTaskCreate(ros::NodeHandle *nh) {
	nh_ = nh;
	nh_->advertise(usonics);

	us_event_flag = osEventFlagsNew(NULL);
	if (us_event_flag == NULL) {
		; // Event Flags object not created, handle failure
	}

	osThreadId_t usonicManagerHandle;
	const osThreadAttr_t usonicManager_attributes = { name : "usonicManager",
			.attr_bits = osThreadDetached, .cb_mem = NULL, .cb_size = 0,
			.stack_mem = NULL, .stack_size = 256 * 4, .priority =
					(osPriority_t) osPriorityAboveNormal, .tz_module = 0,
			.reserved = 0 };

	usonicManagerHandle = osThreadNew(UsonicManagerTask, NULL,
			&usonicManager_attributes);

	if (usonicManagerHandle == NULL) {
		return 1;
	}
	return 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		if (check_crc(thread_data->front_rx_buffer)) {
			thread_data->data[thread_data->front_counter] =
					thread_data->front_rx_buffer[1] << 8
							| thread_data->front_rx_buffer[2];
			//if (thread_data->data[thread_data->front_counter] == 0)
			//	thread_data->data[thread_data->front_counter] = 5000;
			thread_data->front_counter++;
			// Updating sensor address 0-0b1,1-0b10, 2-0b11, 3-0b100, 4-0b101
			if (thread_data->front_counter >= SENSORS_PER_CHANNEL) {
				thread_data->front_counter = 0;
				// Front sensor data collected
				osEventFlagsSet(us_event_flag, 0b10);
			}
			uint16_t pinMask = thread_data->front_counter;
			// Turn off unused pins
			HAL_GPIO_WritePin(GPIOE, ((0b111 ^ pinMask) << 7), GPIO_PIN_RESET);
			// Turn on necessary pins
			HAL_GPIO_WritePin(GPIOE, (pinMask << 7), GPIO_PIN_SET);
		}

		if (thread_data->state == 1) {
			HAL_UART_Receive_DMA(&huart2, thread_data->front_rx_buffer,
			RX_BUFFER_LENGTH);
			HAL_UART_Transmit_IT(&huart2, &thread_data->req, 1);
		}
	}
	if (huart->Instance == UART4) {
		uint8_t offset = 5;

		if (check_crc(thread_data->rear_rx_buffer)) {

			uint8_t id = thread_data->rear_counter + offset;
			thread_data->data[id] = thread_data->rear_rx_buffer[1] << 8
					| thread_data->rear_rx_buffer[2];
			//if (thread_data->data[id] == 0)
			//	thread_data->data[id] = 5000;
			thread_data->rear_counter++;
			// Updating sensor address 0-0b1,1-0b10, 2-0b11, 3-0b100, 4-0b101
			if (thread_data->rear_counter >= SENSORS_PER_CHANNEL) {
				thread_data->rear_counter = 0;
				// Front sensor data collected
				osEventFlagsSet(us_event_flag, 0b01);
			}
			uint16_t pinMask = thread_data->rear_counter;
			// Turn off unused pins
			HAL_GPIO_WritePin(GPIOE, ((0b111 ^ pinMask) << 10), GPIO_PIN_RESET);
			// Turn on necessary pins
			HAL_GPIO_WritePin(GPIOE, (pinMask << 10), GPIO_PIN_SET);
		}

		if (thread_data->state == 1) {
			HAL_UART_Receive_DMA(&huart4, thread_data->rear_rx_buffer,
			RX_BUFFER_LENGTH);
			HAL_UART_Transmit_IT(&huart4, &thread_data->req, 1);
		}

	}

}

