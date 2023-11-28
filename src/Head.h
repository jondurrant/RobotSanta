/*
 * Head.h
 *
 *  Created on: 6 Oct 2023
 *      Author: jondurrant
 */

#ifndef SRC_HEAD_H_
#define SRC_HEAD_H_

#include "Agent.h"
#include "ServoMG996.h"


class Head : public Agent{
public:
	Head(uint8_t gpPit, uint8_t gpYaw);
	virtual ~Head();

	void moveRad(double pitch, double yaw, uint ticks);

protected:
	/***
	 * Task main run loop
	 */
	virtual void run();

	/***
	 * Get the static depth required in words
	 * @return - words
	 */
	virtual configSTACK_DEPTH_TYPE getMaxStackSize();

private:
	ServoMG996 xPitchServo;
	ServoMG996 xYawServo ;

	double xPitchCurrent 	= 0.0;
	double xPitchTarget 	= 0.0;
	double xPitchDelta		= 0.0;
	double xYawCurrent 	= 0.0;
	double xYawTarget 		= 0.0;
	double xYawDelta 		= 0.0;

	double xTotalSin = 0.0;

	int xSteps = 0;
	int xStep = 0;
};

#endif /* SRC_HEAD_H_ */
