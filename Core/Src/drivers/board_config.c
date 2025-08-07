#include "main.h"
#include "stm32g4xx_hal.h"
#include "board_config.h"

void drv8323_init() {
	// 0.5v for idrive
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 621);

	// simply put for vds protection
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 4090);

	// lmao i forgot enable pin
	HAL_DAC_Start(&hdac2, DAC_CHANNEL_1);
	HAL_DAC_SetValue(&hdac2, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 4090);
}
