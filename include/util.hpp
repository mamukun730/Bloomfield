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
#endif

void clrpsw_i();
void setpsw_i();
void nop();
void xchg(int32_t *data1, int32_t *data2);

#endif /* INCLUDE_UTIL_HPP_ */
