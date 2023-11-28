/*
 * EyesAgent.cpp
 *
 *  Created on: 20 Nov 2023
 *      Author: jondurrant
 */

#include "EyesAgent.h"

EyesAgent::EyesAgent() {
	// TODO Auto-generated constructor stub

}

EyesAgent::EyesAgent(uint8_t gpRed, uint8_t gpGrn, uint8_t gpBlu){
	setGP(gpRed, gpGrn, gpBlu);
}

EyesAgent::~EyesAgent() {
	// TODO Auto-generated destructor stub
}


void EyesAgent::setGP(uint8_t gpRed, uint8_t gpGrn, uint8_t gpBlu){
	xEyes.setGP(gpRed, gpGrn, gpBlu);
	xEyes.setColour(0xFF, 0, 0);
}


/***
 * Task main run loop
 */
void EyesAgent::run(){
	uint8_t rgb[3] = {0, 0, 0};
	bool change;

	for (;;){
		change = false;
		for (uint8_t i=0; i < 3; i++){
			if (rgb[i] > xTargetRGB[i]){
				rgb[i]++;
				change = true;
			} else if (rgb[i] < xTargetRGB[i]){
				rgb[i]--;
				change = true;
			}
		}

		if (change){
			xEyes.setColour(rgb[0], rgb[1], rgb[2]);
			vTaskDelay(10);
		}

	}


	//Original Test - not reached
	for (;;){
		for (uint8_t i=0; i < 3; i++){
			for (uint8_t j=0; j < 0xFF; j++){
				rgb[i]= j;
				xEyes.setColour(rgb[0], rgb[1], rgb[2]);
				vTaskDelay(10);
			}
			for (uint8_t j=0xFF; j > 0;  j--){
				rgb[i]= j;
				xEyes.setColour(rgb[0], rgb[1], rgb[2]);
				vTaskDelay(5);
			}
		}
	}
}

/***
 * Get the static depth required in words
 * @return - words
 */
configSTACK_DEPTH_TYPE EyesAgent::getMaxStackSize(){
	return 500;
}


void EyesAgent::setColour(uint8_t r, uint8_t g, uint8_t b){
	xTargetRGB[0] = r;
	xTargetRGB[1] = g;
	xTargetRGB[2] = b;
}
