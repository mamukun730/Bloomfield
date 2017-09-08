/*
 * constant.hpp
 *
 *  Created on: 2017/09/03
 *      Author: mmkn
 */

#ifndef INCLUDE_CONSTANT_HPP_
#define INCLUDE_CONSTANT_HPP_

#include "flash/r_typedefs.h"

const static uint32_t SYSTEM_CLOCK					= 48000000;

const static uint32_t CTRL_INTERVAL				= 1000;				// [us]
const static uint32_t CMT0_INTERVAL				= 1;				// [ms]
const static uint32_t CMT1_INTERVAL				= 10;				// [us]

const static float BATT_VOLTAGE_ERROR			= 4.0;

const static uint16_t MOTOR_OPCYCLE				= 370;				// 130kHz

// RSPI Gyro
const static uint8_t RSPI_ADDRESS_GYRO_DEVICEID		= 0x75;

const static uint8_t RSPI_DATA_GYRO_DEVICEID		= 0x12;

// RSPI LED-Driver
const static uint8_t RSPI_ADDRESS_LED_CONFIG		= 0x10;
const static uint8_t RSPI_ADDRESS_LED_RED1			= 0x00;
const static uint8_t RSPI_ADDRESS_LED_GREEN1		= 0x01;
const static uint8_t RSPI_ADDRESS_LED_BLUE1			= 0x02;
const static uint8_t RSPI_ADDRESS_LED_RED2			= 0x03;
const static uint8_t RSPI_ADDRESS_LED_GREEN2		= 0x04;
const static uint8_t RSPI_ADDRESS_LED_BLUE2			= 0x05;

const static uint8_t RSPI_DATA_LED_ENABLECS			= 0x02;

#endif /* INCLUDE_CONSTANT_HPP_ */
