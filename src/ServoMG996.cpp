/*
 * ServoMG996.cpp
 *
 *  Created on: 9 Nov 2023
 *      Author: jondurrant
 */

#include "ServoMG996.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include <stdio.h>

ServoMG996::ServoMG996() {
	xGP=0xFF;

}

ServoMG996::ServoMG996(uint8_t gp){
	setGP(gp);
}

ServoMG996::~ServoMG996(){
	//NOP
}


/***
* Set GPIO Pad
* @param gp
*/
void ServoMG996::setGP(uint8_t gp){
	xGP = gp;

	gpio_init(gp);

	//Setup up PWM t
	gpio_set_function(gp, GPIO_FUNC_PWM);
	pwm_set_gpio_level(gp, 0);
	uint slice_num = pwm_gpio_to_slice_num(gp);

	// Get clock speed and compute divider for 50 hz
	uint32_t clk = clock_get_hz(clk_sys);
	uint32_t div = clk / (20000 * 50);

	// Check div is in range
	if ( div < 1 ){
		div = 1;
	}
	if ( div > 255 ){
		div = 255;
	}

	pwm_config config = pwm_get_default_config();
	pwm_config_set_clkdiv(&config, (float)div);

	// Set wrap so the period is 20 ms
	pwm_config_set_wrap(&config, 20000);

	// Load the configuration
	pwm_init(slice_num, &config, false);

	pwm_set_enabled(slice_num, true);

	printf("Servo(%u) slicer %u\n", xGP, slice_num);

	goRad(xRad);
}

/**
* move to angle
* @param radians  -PI/2 to PI/2
*/
void ServoMG996::goRad(double radians){
	int duty = xDutyZero;
	if (xGP > 28){
		return;
	}
	if (radians > M_PI/2){
		return;
	}
	if (radians < M_PI/(-2)){
		return;
	}


	if (radians > 0.0){
		duty = ((xDutyPos - xDutyZero)/ (M_PI/2)) * radians + xDutyZero;
	}
	if (radians < 0.0){
		double rad = radians + M_PI/2;
		duty = ((xDutyZero - xDutyNeg)/ (M_PI/2)) * rad + xDutyNeg;
	}

	//printf("PWM for %f rad is %d duty\n", radians, duty);
	pwm_set_gpio_level(xGP, duty);
	xRad = radians;
}

/***
 * Get current angle
 * @return
 */
double ServoMG996::getRad(){
	return xRad;
}

/***
* Turn servo off
*/
void ServoMG996::off(){
	pwm_set_gpio_level(xGP, 0);
}

/***
 * Adjust the duty cycle for the set points by delta
 * @param neg - delta adjustment for RAD PI/(-2)
 * @param zero - delta adjustment for RAD 0.0
 * @param pos - delta adjustment for RAD PI/2
 */
void ServoMG996::adjustDuty(int neg, int zero, int pos){
	if (( neg + xDutyNeg) > 0){
		xDutyNeg+=neg;
	}
	if (( zero + xDutyZero) > 0){
		xDutyZero+=zero;
	}
	if (( pos + xDutyPos) > 0){
		xDutyPos+=pos;
	}
}

