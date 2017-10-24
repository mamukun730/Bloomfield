/*
 * util.hpp
 *
 *  Created on: 2017/09/03
 *      Author: mmkn
 */

#ifndef INCLUDE_UTIL_HPP_
#define INCLUDE_UTIL_HPP_

#include "flash/r_typedefs.h"

#ifdef __cplusplus
extern "C" void clrpsw_i();
extern "C" void setpsw_i();
extern "C" void nop();
extern "C" void xchg(int32_t *data1, int32_t *data2);
extern "C" float mysin(float degree);
extern "C" float mycos(float degree);
extern "C" float mysin_rad(float rad);
extern "C" float mycos_rad(float rad);
#endif

void clrpsw_i();
void setpsw_i();
void nop();
void xchg(int32_t *data1, int32_t *data2);

float mysin(float degree);
float mycos(float degree);

float mysin_rad(float rad);
float mycos_rad(float rad);

#endif /* INCLUDE_UTIL_HPP_ */
