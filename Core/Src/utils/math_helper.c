/*
 * math_helper.c
 *
 *  Created on: Aug 5, 2025
 *      Author: lapchong
 */

#include <stdio.h>
#include "math_helper.h"
#include "math.h"

/*
 * function to return normalised angle to avoid electrical angle to overflow after a few spins
 */
float math_normalise(float angle) {
	// fmod will return the remainder of angle / 2pi in float
	float normal = fmod(angle, TWO_PI);

	if (normal >= 0) {
		return normal;
	} else {
		return (normal + TWO_PI);
	}
}

/*
 * function to return clamp value so that its always between min and max
 */
float math_clamp(float min, float value, float max) {
	if (value < min) {
		return min;
	} else if (value > max) {
		return max;
	} else {
		return value;
	}
}
