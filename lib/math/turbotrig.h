/*
 * turbotrig.h
 *
 *  Created on: Sep 8, 2016
 *      Author: James Jackson
 */

#ifndef INCLUDE_LIB_TURBOTRIG_TURBOTRIG_H_
#define INCLUDE_LIB_TURBOTRIG_TURBOTRIG_H_

#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// float-based wrappers
float atan2_approx(float y, float x);
float asin_approx(float x);
float sat(float value, float min, float max);
float sgn(float x);
float turboInvSqrt(float x);


#ifdef __cplusplus
}
#endif




#endif /* INCLUDE_LIB_TURBOTRIG_TURBOTRIG_H_ */
