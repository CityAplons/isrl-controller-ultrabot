/*
 * spin.h
 *
 *  Created on: Jan 23, 2021
 *      Author: nmiha
 */

#ifndef INC_SPIN_H_
#define INC_SPIN_H_

#include "stdint.h"
#include "ros.h"

uint32_t RosSpinTaskCreate(ros::NodeHandle *nh);

#endif /* INC_SPIN_H_ */
