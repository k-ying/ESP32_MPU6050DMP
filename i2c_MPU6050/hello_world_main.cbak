#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "MPU6050.h"
#include "inv_mpu.h"

void app_main()
{
	//MPU_Init();
	uint8_t flag = 1;
	float pitch, roll, yaw;
	flag = mpu_dmp_init();

    for(;;)
    {
    	printf("hello world %d\n", flag);
    	while(mpu_dmp_get_data(&pitch, &roll, &yaw) != 0);
    	printf("\n pitch %f \n roll %f \n yaw %f\n\n", pitch, roll, yaw);
    	vTaskDelay(1000 / portTICK_RATE_MS);

    }
}
