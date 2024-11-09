#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_gpio.h"
#include "nvs_flash.h"
#include "motor.h"


#define STACK_SIZE 2048
#define TASK_PRIORITY_DEFAULT 5

void movement_test(void *pvParameters){
    move_forward();
    vTaskDelay(pdMS_TO_TICKS(1000));

    move_backward();
    vTaskDelay(pdMS_TO_TICKS(1000));

    control_motor_stop();
    vTaskDelay(pdMS_TO_TICKS(100));

    move_right();
    vTaskDelay(pdMS_TO_TICKS(1000));

    control_motor_stop();
    vTaskDelay(pdMS_TO_TICKS(100));

    move_left();
    vTaskDelay(pdMS_TO_TICKS(1000));

}

void app_main()
{
    setup_motor_gpio();
    setup_mcpwm();

    xTaskCreate(movement_test,
                "Movement Test",
                STACK_SIZE,
                NULL,
                TASK_PRIORITY_DEFAULT,
                NULL);





}