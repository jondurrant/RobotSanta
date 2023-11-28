/*
 * EyesAgent.h
 *
 *  Created on: 20 Nov 2023
 *      Author: jondurrant
 */

#ifndef FIRMWARE_SRC_EYESAGENT_H_
#define FIRMWARE_SRC_EYESAGENT_H_

#include "Agent.h"
#include "Eyes.h"
#include "pico/stdlib.h"

class EyesAgent : public Agent  {
public:
	EyesAgent();
	EyesAgent(uint8_t gpRed, uint8_t gpGrn, uint8_t gpBlu);
	virtual ~EyesAgent();

	void setGP(uint8_t gpRed, uint8_t gpGrn, uint8_t gpBlu);

	void setColour(uint8_t r, uint8_t g, uint8_t b);

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
	Eyes xEyes;

	bool xtest = true;

	uint8_t xTargetRGB[3];

};

#endif /* FIRMWARE_SRC_EYESAGENT_H_ */
