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

static uint8_t getVoltage(power_task_t *data)
{
	HAL_ADC_PollForConversion(&hadc1, 100);
	data->voltage_raw = HAL_ADC_GetValue(&hadc1);
	data->voltage = data->voltage_raw / CONVERSION_MAX * MAX_VOLTAGE;
	data->battery_state = static_cast<uint8_t>((data->voltage-EM_VOLTAGE)*100./(MAX_VOLTAGE-EM_VOLTAGE));
	return static_cast<uint8_t>(data->voltage);
}

static HAL_StatusTypeDef getCurrent(power_task_t *data)
{
	HAL_StatusTypeDef returnValue;
	uint8_t reg = DATA_REG;
	returnValue = HAL_I2C_Master_Transmit(&hi2c1, CS_I2C_ADDRESS, &reg, 1, HAL_MAX_DELAY);
	if(returnValue != HAL_OK)
		return returnValue;
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
	HAL_ADC_Start(&hadc1);

	HAL_StatusTypeDef currentStatus;
	for(;;)
	{
		// Receiving voltage level
		msg1.voltage = getVoltage(&td);
		msg1.battery_state = td.battery_state;

		// Receiving current level
		currentStatus = getCurrent(&td);
		if (currentStatus == HAL_OK){
			td.current = static_cast<int16_t>(td.current_raw*CURRENT_MAX/4096.*1000);
			td.power_consumption = (td.current_raw*CURRENT_MAX/4096.) * td.voltage;
		} else if (currentStatus == HAL_TIMEOUT) {
			td.current = 0;
			td.power_consumption= 0;
			nh_->logwarn("Current sensor connection timed out!");
			osDelay(1000);
		} else {
			nh_->logerror("Current sensor connection error!");
			osDelay(5000);
		}
		msg1.current = td.current;
		msg1.power_consumption = td.power_consumption;

		power.publish(&msg1);

		if(HAL_GPIO_ReadPin(BTN1_GPIO_Port,BTN1_Pin))
			msg2.data = true;
		else
			msg2.data = false;
		emergency.publish(&msg2);
		osDelay(500);
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