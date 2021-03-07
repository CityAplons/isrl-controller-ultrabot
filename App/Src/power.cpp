/*
 * power.cpp
 *
 *  Created on: Jan 23, 2021
 *      Author: Nikita Mikhailovskiy
 */

extern "C"
{
	#include "FreeRTOS.h"
	#include "task.h"
	#include "cmsis_os.h"
	#include "i2c1.h"
	#include "adc.h"
	#include "gpio.h"
}

#include "power.h"
#include "ultrabot_stm/power.h"
#include "std_msgs/Bool.h"

extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;

static ros::NodeHandle *nh_;
static power_task_t * thread_data;

ultrabot_stm::power msg1;
std_msgs::Bool msg2;
ros::Publisher power("power", &msg1);
ros::Publisher emergency("emergency", &msg2);

static HAL_StatusTypeDef getVoltage(power_task_t *data)
{
	HAL_StatusTypeDef returnValue;
	returnValue = HAL_ADC_Start(&hadc1);
	if (returnValue != HAL_OK) return returnValue;

	returnValue = HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	if(returnValue == HAL_OK)
		data->voltage_raw = HAL_ADC_GetValue(&hadc1);
	else
		return returnValue;
	returnValue = HAL_ADC_Stop(&hadc1);
	return returnValue;
}

static HAL_StatusTypeDef getCurrent(power_task_t *data)
{
	HAL_StatusTypeDef returnValue;
	uint8_t reg = DATA_REG;
	returnValue = HAL_I2C_Master_Transmit(&hi2c1, CS_I2C_ADDRESS, &reg, 1, HAL_MAX_DELAY);
	if(returnValue != HAL_OK) return returnValue;
	// Strange I2C Behavior: SCL pulled down by Slave (possible stretch conflict)
	osDelay(1);
	//
	uint8_t arr[2];
	returnValue = HAL_I2C_Master_Receive(&hi2c1, CS_I2C_ADDRESS, arr, 2, HAL_MAX_DELAY);
	if(returnValue == HAL_OK) data->current_raw = arr[0] << 8 | arr[1];
	return returnValue;
}

void powerManagerTask(void * argument)
{
	// Initializing data structure
	power_task_t td ={
		.battery_state = 100,	// For safety reason
		.voltage = 29.3,		// to prevent dangling relay switch
		.voltage_raw = 0,
		.current_raw = 0,
		.current = 0,
		.power_consumption = 0.
	};
	thread_data = &td;

	// Initialize peripherals
	MX_I2C1_Init();
	MX_ADC1_Init();

	HAL_StatusTypeDef currentStatus;
	for(;;)
	{
		// Receiving voltage level
		currentStatus = getVoltage(&td);
		if(currentStatus != HAL_OK)
		{
			if(nh_->connected()){
				usb_lock();
				nh_->logerror("Voltage ADC Error!");
				usb_unlock();
			}
			osDelay(1000);
		} else {
			td.voltage = td.voltage_raw * 33.482 / CONVERSION_MAX;
			td.battery_state = static_cast<uint8_t>((td.voltage-EM_VOLTAGE)*100./(MAX_VOLTAGE-EM_VOLTAGE));
		}

		// Receiving current level
		currentStatus = getCurrent(&td);
		if (currentStatus == HAL_I2C_ERROR_NONE){
			td.current = static_cast<int16_t>((td.current_raw-570)/52.2*1000);
			td.power_consumption = td.current * td.voltage / 1000.f;
		} else if (currentStatus == HAL_TIMEOUT) {
			td.current = 0;
			td.power_consumption= 0;
			if(nh_->connected()) {
				usb_lock();
				nh_->logwarn("Current sensor connection timed out!");
				usb_unlock();
			}
			osDelay(1000);
		} else {
			if(nh_->connected()) {
				usb_lock();
				nh_->logerror("Current sensor I2C connection error!");
				usb_unlock();
			}
			osDelay(1000);
		}

		if(nh_->connected()){
			msg1.voltage = td.voltage;
			msg1.battery_state = td.battery_state;
			msg1.current = td.current;
			msg1.power_consumption = td.power_consumption;

			if(HAL_GPIO_ReadPin(BTN1_GPIO_Port,BTN1_Pin))
				msg2.data = true;
			else
				msg2.data = false;

			usb_lock();
			power.publish(&msg1);
			emergency.publish(&msg2);
			usb_unlock();
		}
		osDelay(100);
	}
}

/*
 * Create task
 */
uint32_t PowerManagerTaskCreate(ros::NodeHandle *nh)
{
	nh_ = nh;
	nh_->advertise(power);
	nh_->advertise(emergency);

	osThreadId_t PowerManagerHandle;
	const osThreadAttr_t power_manager_attributes = {
	  	name : "power",
		.attr_bits = osThreadDetached,
		.cb_mem = NULL,
		.cb_size = 0,
		.stack_mem = NULL,
		.stack_size = 128 * 4,
	  	.priority = (osPriority_t) osPriorityNormal,
		.tz_module = 0,
		.reserved = 0
	 };

	PowerManagerHandle = osThreadNew(powerManagerTask, NULL, &power_manager_attributes);

	if (NULL == PowerManagerHandle){ return 1; }
	return 0;

}
