/*
 * imu.cpp
 *
 *  Created on: Jan 23, 2021
 *      Author: Nikita Mikhailovskiy
 */

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "i2c1.h"
#include "tim.h"
#include "MadgwickAHRS.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "HMC5883L.h"
}

#include "imu.h"

__IO uint8_t measure_flag;

static ros::NodeHandle *nh_;

void imuTask(void *argument) {

	MX_I2C2_Init();
	MX_TIM3_Init();

	I2Cdev_init(&hi2c2);
	MPU6050_initialize();
	while (!MPU6050_testConnection()) {

	}
	MPU6050_setI2CMasterModeEnabled(false);
	MPU6050_setI2CBypassEnabled(true);
	MPU6050_setDLPFMode (MPU6050_DLPF_BW_256);
	HMC5883L_initialize();
	while (!HMC5883L_testConnection()) {

	}
	int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
	measure_flag = 0;
	HAL_TIM_Base_Start_IT(&htim3);
	for (;;) {
		if (measure_flag) {
			MPU6050_getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
			HMC5883L_getHeading(&mx, &my, &mz);
			float gx_f = gx / 16384.;
			float gy_f = gy / 16384.;
			float gz_f = gz / 16384.;
			float ax_f = ax / 16384.;
			float ay_f = ay / 16384.;
			float az_f = az / 16384.;
			float mx_f = mx / 16384.;
			float my_f = my / 16384.;
			float mz_f = mz / 16384.;
			MadgwickAHRSupdate(gx_f, gy_f, gz_f, ax_f, ay_f, az_f, mx_f, my_f,
					mz_f);
			measure_flag = 0;
		}
	}
}

/*
 * Create task
 */
uint32_t IMUManagerTaskCreate(ros::NodeHandle *nh) {
	nh_ = nh;

	osThreadId_t imuManagerHandle;
	const osThreadAttr_t imu_attributes = { name : "IMU", .attr_bits =
	osThreadDetached, .cb_mem = NULL, .cb_size = 0, .stack_mem = NULL,
			.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal,
			.tz_module = 0, .reserved = 0 };

	imuManagerHandle = osThreadNew(imuTask, NULL, &imu_attributes);

	if (NULL == imuManagerHandle) {
		return 1;
	}
	return 0;

}
