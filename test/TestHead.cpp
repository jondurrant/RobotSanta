/*
 * TestHead.cpp
 *
 *  Created on: 3 Nov 2023
 *      Author: jondurrant
 */

#include "pico/stdlib.h"
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <math.h>

#include "Head.h"

#include "CppUTest/TestHarness.h"

#define TASK_PRIORITY      ( tskIDLE_PRIORITY + 1UL )

TEST_GROUP(HeadGroup){

};

TEST(HeadGroup, HeadTest){
	Head head(20,19);
	head.start("Head", TASK_PRIORITY);

	head.moveRad(0.0, 0.0 , 1);
	vTaskDelay(5000);
	head.moveRad(0.0, M_PI/2.0  , 2000);
	vTaskDelay(5000);
	head.moveRad(0.0, M_PI/(-2.0)  , 2000);
	vTaskDelay(5000);
	head.moveRad(0.0, 0.0 , 2000);
	vTaskDelay(5000);
	head.moveRad(0.2, 0.0 , 2000);
	vTaskDelay(5000);
	head.moveRad(-0.2, 0.0 , 2000);
	vTaskDelay(5000);
	head.moveRad(0.0, 0.0 , 2000);
	vTaskDelay(5000);
}



