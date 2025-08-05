/*
 * encoder_driver.h
 *
 *  Created on: Aug 5, 2025
 *      Author: lapchong
 */

#ifndef SRC_DRIVERS_ENCODER_DRIVER_H_
#define SRC_DRIVERS_ENCODER_DRIVER_H_

extern uint32_t encoder_position;

void encoder_init(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
int32_t encoder_get_position();

#endif /* SRC_DRIVERS_ENCODER_DRIVER_H_ */
