/*
 * ServoMG996.h
 *
 *  Created on: 9 Nov 2023
 *      Author: jondurrant
 */

#ifndef FIRMWARE_SRC_SERVOMG996_H_
#define FIRMWARE_SRC_SERVOMG996_H_

#include "pico/stdlib.h"
#include <math.h>

class ServoMG996 {
public:
	ServoMG996();
	ServoMG996(uint8_t gp);
	virtual ~ServoMG996();

	/***
	 * Set GPIO Pad
	 * @param gp
	 */
	void setGP(uint8_t gp);

	/**
	 * move to angle
	 * @param radians  -PI/2 to PI/2
	 */
	void goRad(double radians);


	/***
	 * Get current angle
	 * @return
	 */
	double getRad();


	/***
	 * Turn servo off
	 */
	void off();


	/***
	 * Adjust the duty cycle for the set points by delta
	 * @param neg - delta adjustment for RAD PI/(-2)
	 * @param zero - delta adjustment for RAD 0.0
	 * @param pos - delta adjustment for RAD PI/2
	 */
	void adjustDuty(int neg, int zero, int pos);

private:
	uint8_t xGP = 0;
	double xRad = 0.0;

	uint xDutyZero = 1360;
	uint xDutyPos  = 2200;
	uint xDutyNeg = 680;

};

#endif /* FIRMWARE_SRC_SERVOMG996_H_ */
