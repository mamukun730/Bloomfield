/*
 * util.cpp
 *
 *  Created on: 2017/09/03
 *      Author: mmkn
 */

#include "common.hpp"
#include "util.hpp"
#include "flash/r_typedefs.h"

void clrpsw_i() {
	__builtin_rx_clrpsw('I');
}

void setpsw_i() {
	__builtin_rx_setpsw('I');
}

void nop () {
	__asm("nop");
}

void xchg(int32_t *data1, int32_t *data2) {
	__builtin_rx_xchg((int *)data1, (int *)data2);
}

float mysin(float degree) {
	return mysin_rad(degree * DEGREE_RAD);
}

float mycos(float degree) {
	return mycos_rad(degree * DEGREE_RAD);
}

float mysin_rad(float rad) {
	return (rad - ((rad * rad * rad) / 6.0) + ((rad * rad * rad * rad * rad) / 120.0) - ((rad * rad * rad * rad * rad * rad * rad) / 5040.0));
}

float mycos_rad(float rad) {
	return (1.0 - ((rad * rad) / 2.0) + ((rad * rad * rad * rad) / 24.0) - ((rad * rad * rad * rad * rad * rad) / 720.0));
}
