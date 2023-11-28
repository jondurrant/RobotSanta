/*
 * Head.cpp
 *
 *  Created on: 6 Oct 2023
 *      Author: jondurrant
 */

#include "Head.h"
#include "math.h"
#include <cstdio>

Head::Head(uint8_t gpPit, uint8_t gpYaw){
	xPitchServo.setGP(gpPit);
	xYawServo.setGP(gpYaw);
	xPitchServo.goRad(xPitchCurrent);
	xYawServo.goRad(xYawCurrent);
}

Head::~Head() {
	// TODO Auto-generated destructor stub
}


void Head::run(){

	for (;;){

		if (xStep > 0){
			float s =  sin( (M_PI / (float) xSteps) * (float) (xStep -1)) / xTotalSin;
			float d = s  * xPitchDelta ;
			xPitchCurrent += d;
			xPitchServo.goRad(xPitchCurrent);

			d = s * xYawDelta;
			xYawCurrent += d;
			xYawServo.goRad(xYawCurrent);

			xStep --;
		}

		vTaskDelay(10);
	}
}

void Head::moveRad(double pitch, double yaw, uint ticks){
	int t = ticks/10;
	float f = 0;

	if ((pitch > M_PI/2.0) || pitch < M_PI/(-2.0)){
		return;
	}
	if ((yaw > M_PI/2.0) || yaw < M_PI/(-2.0)){
		return;
	}

	//Compute sin total
	for (int i=0; i < t; i++){
		f += sin( M_PI / (float) t * (float) i);
	}
	xTotalSin= f;

	xPitchTarget = pitch;
	xPitchDelta = (xPitchTarget + M_PI/2) - (xPitchCurrent + M_PI/2);

	xYawTarget = yaw;
	xYawDelta = (xYawTarget + M_PI/2) - (xYawCurrent + M_PI/2);

	xSteps = t;
	xStep = xSteps;
#if 0 //Debug code
	printf("SETUP PITCH to %f delta %f Sin Total %f\n",
			xPitchTarget,
			xPitchDelta,
			xTotalSin);
	printf("SETUP YAW to %f delta %f Sin Total %f\n",
				xYawTarget,
				xYawDelta,
				xTotalSin);
#endif

}

/***
* Get the static depth required in words
* @return - words
*/
configSTACK_DEPTH_TYPE Head::getMaxStackSize(){
	return 1024;
}
