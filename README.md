# FOC Firmware Repository

* hardware used: https://github.com/CleanRoboCore/hardware_pcb_drv8323h

## Other infos:

* please write clean and portable code

## Firmware writing plan

* use arm cmsis dsp for fir or iir low pass filter for encoder and current reading, use the pid function also
* use hrtim, or basic tim1 or tim8 to test first
* make sure foc implementation is abstract from hardware, write hardware related code seperately

## Hardware usage:

### read encoder value

* use tim1, hall sensor/ xor mode
* setup: ```HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);```
* read value: ```volatile uint32_t ccr1 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);```, its 16 bit value anyways

## Repo structure planning

### app/
* app/motor_app.c/h - motor control loop
* app/can_app.c/h - can communication logic

### foc/
* foc/foc.c/h - main foc logic, transform, pid, pwm
* foc/pid_controller.c/h - use cmsis dsp pid
* foc/filters.c/h - use cmsis dsp low pass filter
* foc/transform.c/h - put clark, park and their inverse here

### drivers/
* drivers/pwm_driver.c/h - hrtim or tim implementation here
* drivers/adc_driver.c/h - put adc and calibration
* drivers/encoder_driver.c/h - put encoder reading
* drivers/board_config.c/h - put motor driver config, motor characteristics
* drivers/can_driver.c/h - put canopen init and function wrapper

### utils/
* utils/math_helpers.c/h - put normalise, clamp math functions, define pi here
