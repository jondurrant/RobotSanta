/**
 * Unit Test Main
 */

#include "pico/stdlib.h"
#include <stdio.h>
#include "CppUTest/TestHarness.h"
#include "CppUTest/CommandLineTestRunner.h"

#include "FreeRTOS.h"
#include "task.h"

#define TASK_PRIORITY      ( tskIDLE_PRIORITY + 1UL )

void main_task(void* params){
	printf("RUNNING TESTS\n");

	int ac=3;
	const char * av[3] = {"test", "-g", "Head"};
	CommandLineTestRunner::RunAllTests(ac, av);
	for(;;){
		vTaskDelay(1000);
	}
}

void vLaunch(void) {
  TaskHandle_t task;

  xTaskCreate(main_task, "MainThread", 2048, NULL, TASK_PRIORITY, &task);

  /* Start the tasks and timer running. */
  vTaskStartScheduler();
}


int main(int ac, char** av) {

	//Initialise IO as we are using printf for debug
	stdio_init_all();

	sleep_ms(2000);
	printf("Starting FreeRTOS on core0\n");

	vLaunch();

	for (;;){
		sleep_ms(1000);
	}
}
