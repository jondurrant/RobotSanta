/*
 * Eyes.cpp
 *
 *  Created on: 6 Oct 2023
 *      Author: jondurrant
 */

#include "Eyes.h"
#include <cstdio>
#include <math.h>

Eyes::Eyes(){
}

Eyes::Eyes(uint8_t gpRed, uint8_t gpGrn, uint8_t gpBlu){
	setGP(gpRed, gpGrn, gpBlu);

}

Eyes::~Eyes() {
	// TODO Auto-generated destructor stub
}

void Eyes::setColour(uint8_t red, uint8_t grn, uint8_t blu){
	if (xGPRed > 28){
		return;
	}


	uint16_t r = ceil( ((float)(255 - red) / 255.0) * (float)xRedMax);
	uint16_t g = ceil( ((float)(255 - grn) / 255.0) * (float)xGrnMax);
	uint16_t b = ceil( ((float)(255 - blu) / 255.0) * (float)xBluMax);

	pwm_set_gpio_level(xGPRed,  r);
	pwm_set_gpio_level(xGPGrn, g);
	pwm_set_gpio_level(xGPBlu, b);

	//printf("Set Colour %u, %u, %u\n", red, grn, blu);
}

void Eyes:: setGP(uint8_t gpRed, uint8_t gpGrn, uint8_t gpBlu){
	uint sliceRed, sliceGrn, sliceBlu;
	xGPRed = gpRed;
	xGPGrn = gpGrn;
	xGPBlu = gpBlu;

	gpio_init(xGPRed);
	gpio_set_function(xGPRed, GPIO_FUNC_PWM);
	pwm_set_gpio_level(xGPRed, 0xFFFF);
	sliceRed = pwm_gpio_to_slice_num(xGPRed);
	pwm_set_enabled(sliceRed, true);

	gpio_init(xGPGrn);
	gpio_set_function(xGPGrn, GPIO_FUNC_PWM);
	pwm_set_gpio_level(xGPGrn, 0xFFFF);
	sliceGrn = pwm_gpio_to_slice_num(xGPGrn);
	pwm_set_enabled(sliceGrn, true);

	gpio_init(xGPBlu);
	gpio_set_function(xGPBlu, GPIO_FUNC_PWM);
	pwm_set_gpio_level(xGPBlu, 0xFFFF);
	sliceBlu = pwm_gpio_to_slice_num(xGPBlu);
	pwm_set_enabled(sliceBlu, true);

	printf("Eye slicers %u, %u, %u\n",  sliceRed, sliceGrn, sliceBlu);
}
