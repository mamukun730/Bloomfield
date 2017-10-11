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

const static uint32_t CTRL_INTERVAL					= 1000;				// [us]
const static uint32_t CMT0_INTERVAL					= 1;				// [ms]
const static uint32_t CMT1_INTERVAL					= 10;				// [us]

const static float BATT_VOLTAGE_ERROR				= 3.80;
const static uint16_t LOGSIZE						= 1000;

// Math
const static float M_PI								= 3.14159265358979;
const static float DEGREE_RAD						= M_PI / 180.0;
const static float RAD_DEGREE						= 180.0 / M_PI;
const static float SQRT2							= 1.41421356237310;

// Machine Specs
const static float BODY_WEIGHT						= 0.0196;			// 機体重量 [kg]
//const static float BODY_MOMENT_INERTIA			= 0.00015319;		// 慣性モーメント [kg/m^2] 0.00005000
const static float BODY_WIDTH						= 0.0350;			// トレッド幅(機体幅-タイヤ幅) [m]
const static float GEAR_RATIO						= 1.0000;
const static float WHEEL_D							= 14.000;			// ホイール径 [mm]
//const static uint32_t SLIP_RATE						= 65000;			// スリップ定数 [mm/s2]

const static uint8_t MODE_NUMBER					= 2;
const static uint8_t NUMBER_SLALOM_PARAM			= 0;
const static uint16_t WAITING_SENSORSTART			= 100;

const static float SEARCH_SPEED						= 240.0;
const static float SEARCH_ACCEL						= 3000.0;

const static float TURN_ANGLE_SPEED					= 360.0;
const static float TURN_ANGLE_ACCEL					= 3000.0;

const static float TURN_ANGLE_RIGHT					= 90.0;
const static float TURN_ANGLE_OPPOSITE				= 180.0;

// Maps
const static uint32_t INF							= 1000000000;
const static float SECTION_STRAIGHT					= 90.000;
const static float SECTION_SLANT					= 63.640;

const static uint8_t MAPSIZE_X						= 16;
const static uint8_t MAPSIZE_Y						= 16;
const static uint8_t START_X						= 0;
const static uint8_t START_Y						= 0;
const static uint8_t GOAL_X							= 5;
const static uint8_t GOAL_Y							= 6;
const static bool ENABLE_FULLGOAL					= true;

const static uint8_t FLAG_EAST						= 0x80;
const static uint8_t FLAG_WEST						= 0x40;
const static uint8_t FLAG_SOUTH						= 0x20;
const static uint8_t FLAG_NORTH						= 0x10;

const static uint8_t WALL_EAST						= 0x08;
const static uint8_t WALL_WEST						= 0x04;
const static uint8_t WALL_SOUTH						= 0x02;
const static uint8_t WALL_NORTH						= 0x01;

const static int8_t DIR_NORTH						= 0;
const static int8_t DIR_N_EAST						= 1;
const static int8_t DIR_EAST						= 2;
const static int8_t DIR_S_EAST						= 3;
const static int8_t DIR_SOUTH						= 4;
const static int8_t DIR_S_WEST						= -3;
const static int8_t DIR_WEST						= -2;
const static int8_t DIR_N_WEST						= -1;

const static int8_t SIDE_FORWARD					= 0;
const static int8_t SIDE_F_RIGHT					= 1;
const static int8_t SIDE_RIGHT						= 2;
const static int8_t SIDE_REAR						= 4;
const static int8_t SIDE_LEFT						= -2;
const static int8_t SIDE_F_LEFT						= -1;

const static int8_t SIDE_F_FORWARD					= 126;
const static int8_t SIDE_NONE						= 127;

const static int8_t SLALOM_RIGHT					= -1;
const static int8_t SLALOM_LEFT						= 1;

// Motor
const static uint16_t MOTOR_OPCYCLE					= 240;				// 200kHz
//const static uint16_t MOTOR_DUTY_MIN				= 921;				// 4%
const static uint16_t MOTOR_DUTY_MAX				= 72;				// 70%

//const static float MOTOR_K_E						= (0.207 / 1000.0);	// 逆起電圧定数 [V/rpm]
//const static float MOTOR_K_M						= (1.980 / 1000.0);	// トルク定数 [Nm/A]
const static float MOTOR_R							= 1.700;			// 端子間抵抗 [Ω]

// Encoder
const static uint16_t ENCODER_INIT_VAL				= 32767;
const static uint16_t ENCODER_MAX					= 4096;
const static float GAIN_P_ENCODER					= 0.276363634;
const static float GAIN_I_ENCODER					= 0.004242424;
const static float GAIN_D_ENCODER					= 0.130909091;

// Gyro
const static float GYRO_RESOLUTION					= (1.0 / 16.4);		// deg/s
const static uint16_t GYRO_REFERENCE_SAMPLE			= 500;
const static float GAIN_P_GYRO						= 0.06302453;
const static float GAIN_I_GYRO						= 0.01289976;
const static float GAIN_D_GYRO						= 0.08956118;

// Accelerometer
const static float ACCEL_RESOLUTION					= (1.0 / 2048.0);		// deg/s
const static float GRAVITY_METRIC					= 9.80665;

// ADC
const static uint16_t ADC_BATTERY					= 0x001;			// 0B 00 0000 0001
const static uint16_t ADC_WALLSENSOR_P1				= 0x24A;			// 0B 10 0100 1010
const static uint16_t ADC_WALLSENSOR_P2				= 0x114;			// 0B 01 0001 0100
//																	   Ch 98 7654 3210

// Sensor
const static float GAIN_P_WALL						= 0.100;
const static float CTRL_WALL_LIMIT					= 720.0;			// deg/s

const static uint8_t SENSOR_AMOUNT					= 7;
const static uint8_t SENSOR_LED_WAIT				= 127;

const static uint16_t SENSOR_TARGET_L				= 320;				// 壁制御目標
const static uint16_t SENSOR_TARGET_R				= 420;

const static uint16_t SENSOR_CTRL_THRESHOLD_L		= 240;				// 壁制御閾値
const static uint16_t SENSOR_CTRL_THRESHOLD_R		= 315;

const static uint16_t SENSOR_DIFF_THRESHOLD			= 25;

const static uint16_t SENSOR_WALL_EXIST_L			= 110;				// 壁有無閾値
const static uint16_t SENSOR_WALL_EXIST_F			= 80;
const static uint16_t SENSOR_WALL_EXIST_R			= 200;

// RSPI Gyro
const static uint8_t RSPI_ADDRESS_GYRO_SMPRT_DIV	= 0x19;
const static uint8_t RSPI_ADDRESS_GYRO_CONFIG		= 0x1A;
const static uint8_t RSPI_ADDRESS_GYRO_GYROCONFIG	= 0x1B;
const static uint8_t RSPI_ADDRESS_GYRO_ACCELCONFIG	= 0x1C;
const static uint8_t RSPI_ADDRESS_GYRO_ACC_ZOUT_H	= 0x3F;
const static uint8_t RSPI_ADDRESS_GYRO_ACC_ZOUT_L	= 0x40;
const static uint8_t RSPI_ADDRESS_GYRO_YOUT_H		= 0x45;
const static uint8_t RSPI_ADDRESS_GYRO_YOUT_L		= 0x46;
const static uint8_t RSPI_ADDRESS_GYRO_ZOUT_H		= 0x47;
const static uint8_t RSPI_ADDRESS_GYRO_ZOUT_L		= 0x48;
const static uint8_t RSPI_ADDRESS_GYRO_SIGNAL_RESET	= 0x68;
const static uint8_t RSPI_ADDRESS_GYRO_PWR_MGMT1	= 0x6B;
const static uint8_t RSPI_ADDRESS_GYRO_PWR_MGMT2	= 0x6C;
const static uint8_t RSPI_ADDRESS_GYRO_DEVICEID		= 0x75;

const static uint8_t RSPI_DATA_GYRO_SMPRT_DIV		= 0x00;
const static uint8_t RSPI_DATA_GYRO_CONFIG			= 0x01;
const static uint8_t RSPI_DATA_GYRO_GYROCONFIG		= 0x18;
const static uint8_t RSPI_DATA_GYRO_ACCELCONFIG		= 0x18;
const static uint8_t RSPI_DATA_GYRO_SIGNAL_RESET	= 0x03;
const static uint8_t RSPI_DATA_GYRO_PWR_MGMT1		= 0x00;
const static uint8_t RSPI_DATA_GYRO_PWR_MGMT2		= 0x00;
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
