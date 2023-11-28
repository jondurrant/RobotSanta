/*
 * servo.cpp
 *
 *  Created on: 9 Nov 2023
 *      Author: jondurrant
 */

#include "pico/stdlib.h"
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

#include "ServoMG996.h"
#include <math.h>

#include "CppUTest/TestHarness.h"


TEST_GROUP(ServoGroup){

};

TEST(ServoGroup, ServoPitch){
	ServoMG996 servo(20);

	servo.goRad(0.0);
	CHECK_EQUAL(0.0, servo.getRad());
	vTaskDelay(5000);
	servo.goRad(M_PI/2);
	CHECK_EQUAL(M_PI/2, servo.getRad());
	vTaskDelay(5000);
	servo.goRad(M_PI/(-2));
	CHECK_EQUAL(M_PI/-2.0, servo.getRad());
	vTaskDelay(5000);
	servo.goRad(0.0);
	CHECK_EQUAL(0.0, servo.getRad());
	vTaskDelay(5000);
}


TEST(ServoGroup, ServoYaw){
	ServoMG996 servo(19);

	servo.goRad(0.0);
	CHECK_EQUAL(0.0, servo.getRad());
	vTaskDelay(5000);
	servo.goRad(M_PI/2);
	CHECK_EQUAL(M_PI/2, servo.getRad());
	vTaskDelay(5000);
	servo.goRad(M_PI/(-2));
	CHECK_EQUAL(M_PI/-2.0, servo.getRad());
	vTaskDelay(5000);
	servo.goRad(0.0);
	CHECK_EQUAL(0.0, servo.getRad());
	vTaskDelay(5000);
}


TEST(ServoGroup, TurnYaw){
	ServoMG996 servo(19);


	for (double d = M_PI/(-2); d < M_PI/2; d+=0.2){
		servo.goRad(d);
		vTaskDelay(500);
	}
}





