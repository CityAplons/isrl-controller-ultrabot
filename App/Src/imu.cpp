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
#include "std_msgs/Empty.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"

__IO uint8_t measure_flag;

static ros::NodeHandle *nh_;

/*
 * MPU6050 calibration procedure
 */
static void calibrate(uint8_t cycles) {
	if (nh_->connected())
		nh_->loginfo("IMU [MPU6050] gyroscope calibration started!");
	MPU6050_setXGyroOffset(0);
	MPU6050_setYGyroOffset(0);
	MPU6050_setZGyroOffset(0);

	size_t counter;
	for (counter = 0; counter < cycles; counter++) {
		uint8_t average_counter = 1;
		int16_t cx, cy, cz;
		int gcx = 0;
		int gcy = 0;
		int gcz = 0;
		while (average_counter <= 100) {
			if (measure_flag) {
				MPU6050_getRotation(&cx, &cy, &cz);
				gcx += cx;
				gcy += cy;
				gcz += cz;
				average_counter++;
				measure_flag = 0;
			}
		}
		gcx /= average_counter * (-1);
		gcy /= average_counter * (-1);
		gcz /= average_counter * (-1);

		cx = static_cast<int16_t>(gcx);
		cy = static_cast<int16_t>(gcy);
		cz = static_cast<int16_t>(gcz);
		// Swapping bytes because of Cortex LSB Memory
		cx = __bswap16(cx);
		cy = __bswap16(cy);
		cz = __bswap16(cz);
		MPU6050_setXGyroOffset(cx);
		MPU6050_setYGyroOffset(cy);
		MPU6050_setZGyroOffset(cz);
	}
	if (nh_->connected())
		nh_->loginfo("IMU [MPU6050] gyroscope calibration finished!");
}

/*
 * ROS Calibration subscriber callback
 */
static void calibrate_cb(const std_msgs::Empty &toggle_msg) {
	calibrate(3);
}

sensor_msgs::Imu imu_msg;
ros::Publisher imu("stm32_imu", &imu_msg);
ros::Subscriber<std_msgs::Empty> cal("stm32_imu_calibrate", &calibrate_cb);

void imuTask(void *argument) {

	int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
	geometry_msgs::Vector3 acc;
	geometry_msgs::Vector3 gyro;
	geometry_msgs::Quaternion q;

	MX_I2C2_Init();
	MX_TIM3_Init();

	I2Cdev_init(&hi2c2);
	MPU6050_initialize();

	while (!MPU6050_testConnection()) {
		if (nh_->connected())
			nh_->logerror("IMU [MPU6050] sensor connection error!");
		osDelay(1000);
		MPU6050_initialize();
	}

	MPU6050_setI2CMasterModeEnabled(false);
	MPU6050_setI2CBypassEnabled(true);

	HMC5883L_initialize();
	while (!HMC5883L_testConnection()) {
		if (nh_->connected())
			nh_->logerror("IMU [HMC5883L] sensor connection error!");
		osDelay(1000);
		HMC5883L_initialize();
	}

	double cov[9] = { 0.0096f, 0.f, 0.f, 0.f, 0.0096f, 0.f, 0.f, 0.f, 0.0096f };
	memcpy(imu_msg.linear_acceleration_covariance, cov, sizeof(cov));
	cov[0] = 0.0033;
	cov[4] = 0.0033;
	cov[8] = 0.0033;
	memcpy(imu_msg.angular_velocity_covariance, cov, sizeof(cov));

	measure_flag = 0;
	HAL_TIM_Base_Start_IT(&htim3);
	calibrate(5); // Cycles is equals to seconds for 100Hz sample freq
	for (;;) {
		if (measure_flag) {
			MPU6050_getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
			HMC5883L_getHeading(&mx, &my, &mz);
			// deg/s to rad/s: 7510 = 2^15/(250*PI/180), where 250 is a gyro range
			float gx_f = gx / 7510.1f;
			float gy_f = gy / 7510.1f;
			float gz_f = gz / 7510.1f;
			float ax_f = ax / 16384.f;
			float ay_f = ay / 16384.f;
			float az_f = az / 16384.f;
			float mx_f = mx / 1090.;
			float my_f = my / 1090.;
			float mz_f = mz / 1090.;
			MadgwickAHRSupdate(gx_f, gy_f, gz_f, ax_f, ay_f, az_f, mx_f, my_f,
					mz_f);
			measure_flag = 0;

			if (nh_->connected()) {
				q.w = q0;
				q.x = q1;
				q.y = q2;
				q.z = q3;
				acc.x = ax_f * 9.81;
				acc.y = ay_f * 9.81;
				acc.z = az_f * 9.81;
				gyro.x = gx_f;
				gyro.y = gy_f;
				gyro.z = gz_f;
				imu_msg.orientation = q;
				imu_msg.angular_velocity = gyro;
				imu_msg.linear_acceleration = acc;
				imu.publish(&imu_msg);
			}
		}
	}
}

/*
 * Create task
 */
uint32_t IMUManagerTaskCreate(ros::NodeHandle *nh) {
	nh_ = nh;
	nh_->advertise(imu);
	nh_->subscribe(cal);

	osThreadId_t imuManagerHandle;
	const osThreadAttr_t imu_attributes = { name : "IMU", .attr_bits =
	osThreadDetached, .cb_mem = NULL, .cb_size = 0, .stack_mem = NULL,
			.stack_size = 256 * 4, .priority = (osPriority_t) osPriorityNormal,
			.tz_module = 0, .reserved = 0 };

	imuManagerHandle = osThreadNew(imuTask, NULL, &imu_attributes);

	if (NULL == imuManagerHandle) {
		return 1;
	}
	return 0;

}
