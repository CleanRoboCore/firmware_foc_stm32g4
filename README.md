# FOC Firmware Repository

* hardware used: https://github.com/CleanRoboCore/hardware_pcb_drv8323h

# Other infos:

* please write clean and portable code

# Firmware writing plan

* use arm cmsis dsp for fir or iir low pass filter for encoder and current reading, use the pid function also
* use hrtim, or basic tim1 or tim8 to test first
* make sure foc implementation is abstract from hardware, write hardware related code seperately
