#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/pcnt.h"
#include "esp_log.h"
#include <math.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "stdio.h"
#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "driver/mcpwm.h"

#include "motor.h"

#define TAG "ENCODER"

#define MOTOR_1_A 46
#define MOTOR_1_B 3
#define MOTOR_1_PWM 8


// Configuration
#define PCNT_UNIT PCNT_UNIT_0
#define ENCODER_GPIO_A 5
#define ENCODER_GPIO_B 6
#define ENCODER_PPR 880      // Pulses Per Revolution
#define SAMPLE_PERIOD_MS 100 // Speed sampling period (ms)

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
  float motor_duty = 100.0f;  // Test speed (adjust as needed)

  gpio_set_level(MOTOR_1_A, true);
  gpio_set_level(MOTOR_1_B, false);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, motor_duty);

  ESP_LOGI(TAG, "Motor 1 running at duty: %.2f", motor_duty);

  while (1)
  {
    vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));

    pcnt_get_counter_value(PCNT_UNIT, &pulse_count);
    pcnt_counter_clear(PCNT_UNIT); // Reset for next interval

    // Compute angular velocity (rad/s)
    float omega = (2.0f * M_PI * pulse_count) / (ENCODER_PPR * dt);

    // Compute RPM
    float rpm = (60.0f * pulse_count) / (ENCODER_PPR * dt);

    ESP_LOGI(TAG, "Speed: %.2f, Pulses: %d, Omega: %.2f rad/s, RPM: %.2f",
             motor_duty, pulse_count, omega, rpm);
  }
}

void app_main(void)
{
  setup_mcpwm();
  setup_motor_gpio();

  setup_encoder_pcnt();
  xTaskCreate(encoder_task, "encoder_task", 2048, NULL, 5, NULL);
  // uint8_t mac[6];
  // esp_err_t ret = esp_read_mac(mac, ESP_MAC_WIFI_STA); // Read Station MAC address

  // if (ret == ESP_OK)
  // {
  //   printf("ESP32-S3 MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
  //          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  // }
  // else
  // {
  //   printf("Failed to read MAC address, error code: %d\n", ret);
  // }
}
