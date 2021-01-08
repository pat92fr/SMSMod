/*
 * control_algo.h
 *
 *  Created on: 6 nov. 2020
 *      Author: Patrick
 */

#ifndef INC_CONTROL_ALGO_H_
#define INC_CONTROL_ALGO_H_

#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void APP_Control_Init();
void APP_Control_Process();

#ifdef __cplusplus
}
#endif


#endif /* INC_CONTROL_ALGO_H_ */
