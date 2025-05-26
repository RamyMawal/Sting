#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/pcnt.h"
#include "esp_log.h"
#include <math.h>

#include "motor.h"

#define TAG "ENCODER"

// Configuration
#define PCNT_UNIT              PCNT_UNIT_0
#define ENCODER_GPIO_A         0
#define ENCODER_GPIO_B         2
#define ENCODER_PPR            880     // Pulses Per Revolution
#define SAMPLE_PERIOD_MS       100     // Speed sampling period (ms)

static void setup_encoder_pcnt()
{
    // Configure PCNT
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = ENCODER_GPIO_A,
        .ctrl_gpio_num = ENCODER_GPIO_B,
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_UNIT,
        .pos_mode = PCNT_COUNT_INC,    // Count up on rising edge
        .neg_mode = PCNT_COUNT_DEC,    // Count down on falling edge
        .lctrl_mode = PCNT_MODE_KEEP,  // Don't change on low ctrl
        .hctrl_mode = PCNT_MODE_REVERSE, // Reverse on high ctrl
        .counter_h_lim = 32767,
        .counter_l_lim = -32768,
    };
    pcnt_unit_config(&pcnt_config);

    // Set filter (debouncing) to remove glitches
    pcnt_set_filter_value(PCNT_UNIT, 1000); // 1000 APB cycles = ~12us at 80MHz
    pcnt_filter_enable(PCNT_UNIT);

    // Initialize counter
    pcnt_counter_pause(PCNT_UNIT);
    pcnt_counter_clear(PCNT_UNIT);
    pcnt_counter_resume(PCNT_UNIT);

    ESP_LOGI(TAG, "PCNT configured for encoder");
}

static void encoder_task(void *arg)
{
    int16_t pulse_count = 0;
    const float dt = SAMPLE_PERIOD_MS / 1000.0f;

    // move_motor(1, false, 100);
    // move_motor(2, false, 100);
    move_motor(3, false, 100);
    // move_motor(4, false, 100);
    vTaskDelay(pdMS_TO_TICKS(500)); 

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));

        pcnt_get_counter_value(PCNT_UNIT, &pulse_count);
        pcnt_counter_clear(PCNT_UNIT); // Reset for next interval

        // Compute angular velocity (rad/s)
        float omega = (2.0f * M_PI * pulse_count) / (ENCODER_PPR * dt);

        // Compute RPM
        float rpm = (60.0f * pulse_count) / (ENCODER_PPR * dt);

        ESP_LOGI(TAG, "Pulses: %d, Omega: %.2f rad/s, RPM: %.2f", pulse_count, omega, rpm);
    }
}

void app_main(void)
{
    setup_mcpwm();
    setup_motor_gpio();

    setup_encoder_pcnt();
    xTaskCreate(encoder_task, "encoder_task", 2048, NULL, 5, NULL);
}

