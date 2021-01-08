/*
 * math_tool.h
 *
 *  Created on: 4 nov. 2020
 *      Author: Patrick
 */

#ifndef INC_MATH_TOOL_H_
#define INC_MATH_TOOL_H_

#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

int32_t constrain(int32_t x, int32_t min, int32_t max);
float fconstrain(float x, float min, float max);
uint32_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max);
float fmap(float x, float in_min, float in_max, float out_min, float out_max);

#endif /* INC_MATH_TOOL_H_ */
