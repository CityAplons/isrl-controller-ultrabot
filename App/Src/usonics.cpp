// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: http://www.viva64.com
/*
 * usonics.cpp
 *
 *  Created on: Jan 23, 2021
 *      Author: Nikita Mikhailovskiy
 */

extern "C"
{
	#include "dma.h"
	#include "usart.h"
	#include "gpio.h"
	#include "FreeRTOS.h"
	#include "task.h"
	#include "cmsis_os.h"
}

#include "usonics.h"
#include "std_msgs/UInt16MultiArray.h"

static ros::NodeHandle *nh_;
static us_task_t * thread_data;

extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart2;

std_msgs::UInt16MultiArray msg;
ros::Publisher usonics("rangefinders", &msg);

static uint8_t check_crc(uint8_t *data)
{
	uint8_t crc = (uint8_t)(data[0] + data[1] + data[2]);
	if(crc == data[3]) return 1;
	return 0;
}

void UsonicManagerTask(void * argument)
{
	//Initializing data structure
	us_task_t td;
	td.front_rx_buffer =  new uint8_t[RX_BUFFER_LENGTH];
	td.rear_rx_buffer =  new uint8_t[RX_BUFFER_LENGTH];
	td.us_flag = osEventFlagsNew(NULL);
	td.req = 0x55; // Ultrasonic request message
	thread_data = &td;

	// Hardware peripheral initialization
	MX_USART2_UART_Init();
	MX_UART4_Init();
	MX_DMA_Init();

	//Registering buffers to receive sensors data
	HAL_UART_Receive_DMA(&huart2, td.front_rx_buffer, RX_BUFFER_LENGTH);
	HAL_UART_Receive_DMA(&huart4, td.rear_rx_buffer, RX_BUFFER_LENGTH);

	uint32_t flags = 0b0;
	for(;;)
	{
		if (nh_->connected()){
			//Wait for interrupt response
			flags = osEventFlagsWait(td.us_flag, 0b11, osFlagsWaitAll, 1000);
			// Waiting for flags from both interrupts
			if (flags == 0b11) {
				msg.data = td.data;
				msg.data_length = NUMBER_OF_SENSORS;
				usonics.publish(&msg);
			}
			else if(flags == 0b01){
				char msg_arr[16+sizeof(td.front_counter)];
				strncpy(msg_arr, "Front US stuck: ", sizeof(msg_arr) - 1);
				msg_arr[sizeof(msg_arr) - 1] = '\0';
				sprintf(&msg_arr[15],"%d",td.front_counter);
				nh_->logwarn(msg_arr);
			}
			else if(flags == 0b10){
				char msg_arr[15+sizeof(td.rear_counter)];
				strncpy(msg_arr, "Rear US stuck: ", sizeof(msg_arr) - 1);
				msg_arr[sizeof(msg_arr) - 1] = '\0';
				sprintf(&msg_arr[14],"%d",td.rear_counter);
				nh_->logwarn(msg_arr);
			}
			else if (flags == (uint32_t)osErrorTimeout) {
				nh_->logerror("Ultrasonic data read timeout!");
				osDelay(1000);
			} else {
				nh_->logerror("[UsonicManagerTask] task error!");
				osDelay(1000);
			}
		} else {
			osDelay(100);
		}
	}
}

uint32_t UsonicManagerTaskCreate(ros::NodeHandle *nh)
{
  nh_ = nh;
  nh_->advertise(usonics);

  osThreadId_t usonicManagerHandle;
  const osThreadAttr_t usonicManager_attributes = {
  	 name : "usonicManager",
	 .attr_bits = osThreadDetached,
	 .cb_mem = NULL,
	 .cb_size = 0,
	 .stack_mem = NULL,
	 .stack_size = 128 * 4,
  	 .priority = (osPriority_t) osPriorityNormal,
	 .tz_module = 0,
	 .reserved = 0
  };

  usonicManagerHandle = osThreadNew(UsonicManagerTask, NULL, &usonicManager_attributes);

  if (usonicManagerHandle == NULL){ return 1; }
  return 0;
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
  if(hdma_uart4_rx.State==HAL_DMA_STATE_READY){
	  if(check_crc(thread_data->rear_rx_buffer)){
		  thread_data->data[thread_data->rear_counter] = thread_data->rear_rx_buffer[1] << 8 | thread_data->rear_rx_buffer[2];
	      if(thread_data->data[thread_data->rear_counter] == 0) thread_data->data[thread_data->rear_counter] = 65000;
		  thread_data->rear_counter++;
	      // Updating sensor address 0-0b1,1-0b10, 2-0b11, 3-0b100, 4-0b101
	      if(thread_data->rear_counter > SENSORS_PER_CHANNEL){
	    	  thread_data->rear_counter = 0;
	    	  // Front sensor data collected
	    	  osEventFlagsSet(thread_data->us_flag, 0b01);
	      }
	      uint16_t pinMask = thread_data->rear_counter+1;
	      // Turn off unused pins
	      HAL_GPIO_WritePin(GPIOE, ((0b111^pinMask) << 10), GPIO_PIN_RESET);
	      // Turn on necessary pins
	      HAL_GPIO_WritePin(GPIOE, (pinMask << 10), GPIO_PIN_SET);
	  }
	  HAL_UART_Transmit(&huart4,&thread_data->req,1,2);

	  HAL_DMA_IRQHandler(&hdma_uart4_rx);

	  hdma_uart4_rx.State=HAL_DMA_STATE_BUSY;
  }
}

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  if(hdma_usart2_rx.State==HAL_DMA_STATE_READY){

  	  if(check_crc(thread_data->front_rx_buffer)){
  		  thread_data->data[thread_data->front_counter] = thread_data->front_rx_buffer[1] << 8 | thread_data->front_rx_buffer[2];
  		  if(thread_data->data[thread_data->front_counter] == 0) thread_data->data[thread_data->front_counter] = 65000;
  		  thread_data->front_counter++;
  		  // Updating sensor address 0-0b1,1-0b10, 2-0b11, 3-0b100, 4-0b101
  		  if(thread_data->front_counter > SENSORS_PER_CHANNEL){
  			  thread_data->front_counter = 0;
  			  // Front sensor data collected
  			  osEventFlagsSet(thread_data->us_flag, 0b10);
  		  }
  		  uint16_t pinMask = thread_data->front_counter+1;
  		  // Turn off unused pins
  		  HAL_GPIO_WritePin(GPIOE, ((0b111^pinMask) << 7), GPIO_PIN_RESET);
  		  // Turn on necessary pins
  		  HAL_GPIO_WritePin(GPIOE, (pinMask << 7), GPIO_PIN_SET);
  	  }
  	  HAL_UART_Transmit(&huart2,&thread_data->req,1,2);

  	  HAL_DMA_IRQHandler(&hdma_usart2_rx);

  	  hdma_usart2_rx.State=HAL_DMA_STATE_BUSY;
  }
}


