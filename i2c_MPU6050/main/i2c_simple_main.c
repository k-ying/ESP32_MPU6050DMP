/* i2c - Simple example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#include "MAX30205.h"
#include "MPU6050/MPU6050.h"
#include "MPU6050/inv_mpu.h"
#include "MPU6050/inv_mpu_dmp_motion_driver.h"

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}


// void app_main(void)
// {
//     int ret;
//     ESP_ERROR_CHECK(i2c_master_init());
//     ret = MAX30205_Init();
//     if(ret==ESP_OK){
//         printf("MAX30205_Init Successful!\n");
//     }else{
//         printf("MAX30205_Init Error!\n");
//     }
    
//     while (1)
//     {

    
//         double MAX30205Res = GetTemperature();
//         printf("sensor val: %.02f\n", MAX30205Res);

//         vTaskDelay(1000/ portTICK_RATE_MS);
//     }
    
// }


void app_main()
{
	// //MPU_Init();
    // ESP_ERROR_CHECK(i2c_master_init());
	// uint8_t flag = 1;
	// float pitch, roll, yaw;
    
	// flag = mpu_dmp_init();

    // for(;;)
    // {
    // 	printf("hello world %d\n", flag);
    // 	while(mpu_dmp_get_data(&pitch, &roll, &yaw) != 0);
    // 	printf("\n pitch %f \n roll %f \n yaw %f\n\n", pitch, roll, yaw);
    // 	vTaskDelay(1000 / portTICK_RATE_MS);

    // }
    ESP_ERROR_CHECK(i2c_master_init());
    // uint8_t ifwork = MPU_Init();
    // printf("%d\n",ifwork);
	uint8_t flag = 1;
	float pitch, roll,yaw; 
	int  ax, ay, az;
	int  gx,gy,gz;
	int ret,ret1;
	uint8_t res = mpu_dmp_init();
    unsigned long step_count;
    //dmp_set_pedometer_step_count(0);

    while(1)
    {
        //MPU_Get_Accelerometer(&ax, &ay, &az);
	    //MPU_Get_Gyroscope(&gx, &gy, &gz);
        // while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){}
        // printf("%f,%f,%f\n",pitch,roll,yaw);

        // //mpu_dmp_get_data(&pitch,&roll,&yaw);
        // printf("pitch   = %f\n",pitch);
        // printf("roll    = %f\n",roll);
        // printf("yaw     = %f\n",yaw);
        // printf("----------------------\n");
        
        dmp_get_pedometer_step_count(&step_count);
        printf("%ld\n",step_count);
        printf("---\n");
        
        // uint8_t re = mpu_dmp_get_data(&pitch, &roll, &yaw);
        
		
        // 串口数据读取
        
        // printf("ax = %d\n",ax);
        // printf("ay = %d\n",ay);
        // printf("az = %d\n",az);
        // printf("Gx = %d\n",gx);
        // printf("Gy = %d\n",gy);
        // printf("Gz = %d\n",gy);

        // printf("pitch   = %f\n",pitch);
        // printf("roll    = %f\n",roll);
        // printf("yaw     = %f\n",yaw);
        // printf("\n");    
   
        vTaskDelay(1000 / portTICK_RATE_MS);
    }

    
}