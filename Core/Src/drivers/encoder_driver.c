/*
 * encoder_driver.c
 *
 *  Created on: Aug 5, 2025
 *      Author: lapchong
 */

#include "stm32g4xx_hal.h"
#include "stm32g4xx_it.h"
#include "main.h"
#include "encoder_driver.h"

void encoder_init(void) {
	HAL_TIM_Encoder_Start_IT(&htim1, TIM_CHANNEL_ALL);
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
}

uint32_t encoder_position = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM1) {
		if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)) {
			encoder_position -= 0x10000;
		}
//		else if (__HAL_TIM_IS_TIM_COUNTING_UP(htim)){
//			encoder_position += 0x10000;
//		}
	}
}

int32_t encoder_get_position() {
	//return HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);
	return (encoder_position + HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1));
}
