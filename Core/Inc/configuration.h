/*
 * configuration.h
 *
 *  Created on: Aug 19, 2024
 *      Author: yahya
 */

#ifndef INC_CONFIGURATION_H_
#define INC_CONFIGURATION_H_
#include <stdint.h>

//************************************   Card Choice   ************************************
//Comment if payload is being coded.
#define ROCKET_CARD
//#define ROCKET_IGNITER_TEST

//************************************   Frequency Choice   ************************************
//freq = freq_val + 410.125
#define ROCKET_TELEM_FREQ			25;					//435.125 MHz
#define PAYLOAD_TELEM_FREQ			30;					//440.125 MHz

//************************************   Algorithms Choices   ************************************
//Uncomment if the algorithm used.
#define ALGORITHM_1							//Only pressure sensor (vertical velocity, altitude)
#define ALGORITHM_2							//Gyro, accelerometer, pressure sensor.

//************************************   Algorithms Thresholds   ************************************
#define SECOND_DEPLOY_ALTITUDE 		(float)570.0			//meters		570.0

#define ARMING_ALTITUDE_1			(float)1000.0		//m				1000
#define RISING_VELOCITY_TRESHOLD	(float)30.0			//m/sn			30.0
#define ALGORITHM_1_LOCKOUT_TIME	(uint32_t)13000		//ms			13000
#define FALLING_VELOCITY_TRESHOLD	(float)3.0			//m/sn			3.0

//#define ARMING_ALTITUDE_2			(float)-10.0		//m				1000
#define QUATERNION_ZERO_TIME		(uint32_t)12000		//ms			12000
#define ALGORITHM_2_LOCKOUT_TIME	(uint32_t)13000		//ms			13000
#define RISING_G_TRESHOLD 			(float)3000.0		//mG			3000.0
#define BURNOUT_THRESHOLD			(float)-2000.0		//mG			-2000.0
#define ANGLE_THRESHOLD				(float)80.0			//degree		80.0

#define IGNITER_TIME				(uint32_t)100		//ms			100
//#define Q_SET_ZERO_ACTIVATE
//************************************   Battery Settings   ************************************
#define LOW_BAT						(float)7.0			//V

//************************************   Print Settings   ************************************
//Uncomment if print datas decoded.
#define PRINT_DECODED

#endif /* INC_CONFIGURATION_H_ */
