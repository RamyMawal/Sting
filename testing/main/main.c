#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "motor.h"

#define CONFIG_FREERTOS_HZ 100
#define GPIO_MOTOR_FORWARD_PIN 5
#define GPIO_MOTOR_BACKWARD_PIN 6

#define STACK_SIZE 2048
#define TASK1_PRIORITY 5
#define TASK2_PRIORITY 5

//Run 2 motors on diffferent tasks

void task1(void *pvParameters) {
    while (1) {
        printf("Task 1 is running\n");
        
        control_motor_forward(1);

        for (size_t i = 10; i <= 100; i += 5)
        {
            update_speed_A(i);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        control_motor_stop(1);
        vTaskDelay(pdMS_TO_TICKS(1000));  

        control_motor_backward(1);

        for (size_t i = 10; i <= 100; i += 5)
        {
            update_speed_A(i);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));  
    }
}

void task2(void *pvParameters) {
    while (1) {
        printf("Task 2 is running\n");
        
        control_motor_forward(2);
        for (size_t i = 10; i <= 100; i += 5)
        {
            update_speed_B(i);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        
        vTaskDelay(pdMS_TO_TICKS(2000));  

        control_motor_stop(2);
        vTaskDelay(pdMS_TO_TICKS(2000));  

        control_motor_backward(2);
        for (size_t i = 10; i <= 100; i += 5)
        {
            update_speed_B(i);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        vTaskDelay(pdMS_TO_TICKS(2000));  
    }
}

void app_main() {
    // Create Task 1
    setup_mcpwm();
    setup_motor_gpio();


    xTaskCreate(task1,         // Task function
                "Task 1",      // Task name (for debugging purposes)
                STACK_SIZE,    // Stack size in words
                NULL,          // Parameter to pass to the task
                TASK1_PRIORITY,// Task priority
                NULL);         // Task handle (not used here)
    
    // Create Task 2
    xTaskCreate(task2,         // Task function
                "Task 2",      // Task name (for debugging purposes)
                STACK_SIZE,    // Stack size in words
                NULL,          // Parameter to pass to the task
                TASK2_PRIORITY,// Task priority
                NULL);         // Task handle (not used here)
}
