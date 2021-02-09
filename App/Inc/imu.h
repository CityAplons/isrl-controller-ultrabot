/*
 * imu.h
 *
 *  Created on: Jan 23, 2021
 *      Author: nmiha
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "stdint.h"
#include "ros.h"

uint32_t IMUManagerTaskCreate(ros::NodeHandle *nh);

#endif /* INC_IMU_H_ */
