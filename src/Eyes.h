/*
 * Eyes.h
 *
 *  Created on: 6 Oct 2023
 *      Author: jondurrant
 */

#ifndef SRC_EYES_H_
#define SRC_EYES_H_

#include "pico/stdlib.h"
#include "hardware/pwm.h"

class Eyes {
public:
	Eyes();
	Eyes(uint8_t gpRed, uint8_t gpGrn, uint8_t gpBlu);

	virtual ~Eyes();

	void setGP(uint8_t gpRed, uint8_t gpGrn, uint8_t gpBlu);
	void setColour(uint8_t red, uint8_t grn, uint8_t blu);

private:
	uint8_t xGPRed = 0xFF;
	uint8_t xGPGrn = 0xFF;
	uint8_t xGPBlu = 0xFF;

	uint16_t xRedMax = 0xFFFF;
	uint16_t xGrnMax = 0xFFFF;
	uint16_t xBluMax = 20000;
};

#endif /* SRC_EYES_H_ */
