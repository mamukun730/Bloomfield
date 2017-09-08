/*
 * util.cpp
 *
 *  Created on: 2017/09/03
 *      Author: mmkn
 */

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
