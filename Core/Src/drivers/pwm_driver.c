/*
 * pwm_driver.c
 *
 *  Created on: Aug 6, 2025
 *      Author: lapchong
 */

#include "stm32g4xx_hal.h"
#include "main.h"
#include "math.h"

// Define period if known, or get it dynamically
#define HRTIM_PERIOD 65280  // Adjust this to your actual period

void pwm_init() {
    // Start the outputs (you're enabling both outputs per timer - that's fine for complementary PWM)
//    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 + HRTIM_OUTPUT_TA2
//                                  + HRTIM_OUTPUT_TC1 + HRTIM_OUTPUT_TC2
//                                  + HRTIM_OUTPUT_TD1 + HRTIM_OUTPUT_TD2);

	HAL_HRTIM_WaveformOutputStart(&hhrtim1,
	    HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
	    HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TC2 |
	    HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2);
	// Start the timers
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_C | HRTIM_TIMERID_TIMER_D);
}

void pwm_write(uint32_t duty_a, uint32_t duty_b, uint32_t duty_c) {
	hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = duty_a;
	hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR = duty_b;
	hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR = duty_c;
}

#define PI_F       3.14159265f

void pwm_spin_test(void) {
    static float angle = 0.0f;
    const float step = 0.02f; // speed adjust

    uint32_t period = __HAL_HRTIM_GETPERIOD(&hhrtim1,
        HRTIM_TIMERINDEX_TIMER_A);

    float sa = sinf(angle), sb = sinf(angle - 2*PI_F/3), sc = sinf(angle + 2*PI_F/3);
    uint32_t da = (uint32_t)((sa*0.5f + 0.5f)*period);
    uint32_t db = (uint32_t)((sb*0.5f + 0.5f)*period);
    uint32_t dc = (uint32_t)((sc*0.5f + 0.5f)*period);

    pwm_write(da, db, dc);

    angle += step;
    if (angle >= 2*PI_F) angle -= 2*PI_F;
}

