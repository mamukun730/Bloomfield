/*
 * common.hpp
 *
 *  Created on: 2017/09/03
 *      Author: mmkn
 */

#ifndef INCLUDE_COMMON_HPP_
#define INCLUDE_COMMON_HPP_

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "../iodefine.h"
#include "flash/r_typedefs.h"

#include "constant.hpp"
#include "system.hpp"
#include "util.hpp"

#include "status.hpp"
#include "pwm.hpp"

#ifdef __cplusplus
extern "C" uint16_t log_cnt;
extern "C" uint16_t log_sen_ls[LOGSIZE], log_sen_lc[LOGSIZE], log_sen_rc[LOGSIZE], log_sen_rs[LOGSIZE];
extern "C" float log_v_actual[LOGSIZE], log_v_target[LOGSIZE], log_a_v_actual[LOGSIZE], log_a_v_target[LOGSIZE];
#endif

extern Status::Value Velocity;
extern Status::Value2 VelocityDiff;
extern Status::Value2 TargetVelocity;
extern Status::Value Accel;

extern Status::Value A_Velocity;
extern Status::Value2 A_VelocityDiff;
extern Status::Value2 TargetA_Velocity;
extern Status::Value A_Accel;

extern Status::Value2 Degree;
extern Status::Value2 Distance;
extern Status::Value2 GyroRef;

extern Status::Flag ExecuteFlag;
extern Status::Flag WallPFlag;
extern Status::Flag WallEdgeFlag;
extern Status::Flag GyroCtrlFlag;

#endif /* INCLUDE_COMMON_HPP_ */
