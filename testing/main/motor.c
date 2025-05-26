#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "driver/mcpwm.h"
#include "esp_log.h"
#include "motor.h"
#include <math.h>

#define TAG "MOTORS"

#define MOTOR_1_A 15
#define MOTOR_1_B 16
#define MOTOR_1_PWM 5
#define MOTOR_2_A 37
#define MOTOR_2_B 38
#define MOTOR_2_PWM 11
#define MOTOR_3_A 7
#define MOTOR_3_B 6
#define MOTOR_3_PWM 4
#define MOTOR_4_A 36
#define MOTOR_4_B 35
#define MOTOR_4_PWM 10

// PWM Configuration
#define PWM_FREQUENCY 1000    
#define PWM_DUTY_CYCLE_DEF 50 
#define PWM_DUTY_CYCLE_MIN 30
#define PWM_DUTY_CYCLE_MAX 80

void setup_motor_gpio()
{
    esp_rom_gpio_pad_select_gpio(MOTOR_1_A);
    esp_rom_gpio_pad_select_gpio(MOTOR_1_B);
    esp_rom_gpio_pad_select_gpio(MOTOR_2_A);
    esp_rom_gpio_pad_select_gpio(MOTOR_2_B);
    esp_rom_gpio_pad_select_gpio(MOTOR_3_A);
    esp_rom_gpio_pad_select_gpio(MOTOR_3_B);
    esp_rom_gpio_pad_select_gpio(MOTOR_4_A);
    esp_rom_gpio_pad_select_gpio(MOTOR_4_B);

    gpio_set_direction(MOTOR_1_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_1_B, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_2_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_2_B, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_3_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_3_B, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_4_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_4_B, GPIO_MODE_OUTPUT);
}

void setup_mcpwm()
{
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR_1_PWM);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MOTOR_2_PWM);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, MOTOR_3_PWM);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, MOTOR_4_PWM);

    mcpwm_config_t pwm_config_1;
    pwm_config_1.frequency = PWM_FREQUENCY;   // Set PWM frequency
    pwm_config_1.cmpr_a = PWM_DUTY_CYCLE_DEF; // Set duty cycle for PWM0A (0-100%)
    pwm_config_1.cmpr_b = PWM_DUTY_CYCLE_DEF; // Set duty cycle for PWM0A (0-100%)
    pwm_config_1.counter_mode = MCPWM_UP_COUNTER;
    pwm_config_1.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config_1);

    mcpwm_config_t pwm_config_2;
    pwm_config_2.frequency = PWM_FREQUENCY;   // Set PWM frequency
    pwm_config_2.cmpr_a = PWM_DUTY_CYCLE_DEF; // Set duty cycle for PWM0A (0-100%)
    pwm_config_2.cmpr_b = PWM_DUTY_CYCLE_DEF; // Set duty cycle for PWM0A (0-100%)
    pwm_config_2.counter_mode = MCPWM_UP_COUNTER;
    pwm_config_2.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config_2);

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 0);
}

void move_motor(int motor, bool forward, float speed)
{
    switch (motor)
    {
    case 1:
        ESP_LOGI(TAG, "Motor 1 speed: %f", speed);
        gpio_set_level(MOTOR_1_A, forward);
        gpio_set_level(MOTOR_1_B, !forward);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, speed);
        break;
    case 2:
        ESP_LOGI(TAG, "Motor 2 speed: %f", speed);
        gpio_set_level(MOTOR_2_A, forward);
        gpio_set_level(MOTOR_2_B, !forward);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, speed);
        break;
    case 3:
        ESP_LOGI(TAG, "Motor 3 speed: %f", speed);
        gpio_set_level(MOTOR_3_A, forward);
        gpio_set_level(MOTOR_3_B, !forward);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, speed);
        break;
    case 4:
        ESP_LOGI(TAG, "Motor 4 speed: %f", speed);
        gpio_set_level(MOTOR_4_A, forward);
        gpio_set_level(MOTOR_4_B, !forward);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, speed);
        break;
    default:
        break;
    }

}


/// @brief Update speed value of (X,Y)
/// @param x_duty_cycle X duty cycle value, between 0,1 (any out of range is set to 1)
/// @param y_duty_cycle Y duty cycle value, between 0,1 (any out of range is set to 1)
void update_speed(float x_duty_cycle, float y_duty_cycle)
{
    x_duty_cycle = fabs(x_duty_cycle);
    y_duty_cycle = fabs(y_duty_cycle);

    // Clamp to Duty cycle min/max range
    if(x_duty_cycle > 0.1f)
        x_duty_cycle = x_duty_cycle < PWM_DUTY_MIN
                           ? PWM_DUTY_MIN
                        : x_duty_cycle > PWM_DUTY_MAX
                           ? PWM_DUTY_MAX
                           : x_duty_cycle * 100;
    else 
        x_duty_cycle = 0;

    if(y_duty_cycle > 0.1f)
        y_duty_cycle = y_duty_cycle < PWM_DUTY_MIN
                           ? PWM_DUTY_MIN
                        : y_duty_cycle > PWM_DUTY_MAX
                           ? PWM_DUTY_MAX
                           : y_duty_cycle * 100;
    else
        y_duty_cycle = 0;

    ESP_LOGI(TAG, "x_duty_cycle value: %f", x_duty_cycle);
    ESP_LOGI(TAG, "y_duty_cycle value: %f", y_duty_cycle);

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, x_duty_cycle);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, y_duty_cycle);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, x_duty_cycle);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, y_duty_cycle);
}

/// @brief Update movement direction to a vector of (X,Y)
/// @param x X vector value, between -1,1 (any value above 1 is reduced to 1)
/// @param y Y vector value, between -1,1 (any value above 1 is reduced to 1)
void update_direction(float x, float y)
{
    ESP_LOGI(TAG, "x value: %f\n", x);
    ESP_LOGI(TAG, "y value: %f\n", y);

    // Rotate and normalize the vector
    x = (x / sqrt(2)) + (y / sqrt(2));
    y = (-x / sqrt(2)) + (y / sqrt(2));

    ESP_LOGI(TAG, "x norm value: %f\n", x);
    ESP_LOGI(TAG, "y norm value: %f\n", y);

    update_speed(x, y);

    if (x > 0)
    {
        gpio_set_level(MOTOR_1_A, 1);
        gpio_set_level(MOTOR_1_B, 0);
        gpio_set_level(MOTOR_4_A, 1);
        gpio_set_level(MOTOR_4_B, 0);
    }
    else
    {
        gpio_set_level(MOTOR_1_A, 0);
        gpio_set_level(MOTOR_1_B, 1);
        gpio_set_level(MOTOR_4_A, 0);
        gpio_set_level(MOTOR_4_B, 1);
    }

    if (y > 0)
    {
        gpio_set_level(MOTOR_2_A, 1);
        gpio_set_level(MOTOR_2_B, 0);
        gpio_set_level(MOTOR_3_A, 1);
        gpio_set_level(MOTOR_3_B, 0);
    }
    else
    {
        gpio_set_level(MOTOR_2_A, 0);
        gpio_set_level(MOTOR_2_B, 1);
        gpio_set_level(MOTOR_3_A, 0);
        gpio_set_level(MOTOR_3_B, 1);
    }
}

void control_motor_stop()
{
    gpio_set_level(MOTOR_1_A, 0);
    gpio_set_level(MOTOR_1_B, 0);
    gpio_set_level(MOTOR_4_A, 0);
    gpio_set_level(MOTOR_4_B, 0);

    gpio_set_level(MOTOR_2_A, 0);
    gpio_set_level(MOTOR_2_B, 0);
    gpio_set_level(MOTOR_3_A, 0);
    gpio_set_level(MOTOR_3_B, 0);
}


void x_forward()
{
    gpio_set_level(MOTOR_1_A, 1);
    gpio_set_level(MOTOR_1_B, 0);
    gpio_set_level(MOTOR_4_A, 1);
    gpio_set_level(MOTOR_4_B, 0);
}
void x_backward()
{
    gpio_set_level(MOTOR_1_A, 0);
    gpio_set_level(MOTOR_1_B, 1);
    gpio_set_level(MOTOR_4_A, 0);
    gpio_set_level(MOTOR_4_B, 1);
}
void y_forward()
{
    gpio_set_level(MOTOR_2_A, 1);
    gpio_set_level(MOTOR_2_B, 0);
    gpio_set_level(MOTOR_3_A, 1);
    gpio_set_level(MOTOR_3_B, 0);
}
void y_backward()
{
    gpio_set_level(MOTOR_2_A, 0);
    gpio_set_level(MOTOR_2_B, 1);
    gpio_set_level(MOTOR_3_A, 0);
    gpio_set_level(MOTOR_3_B, 1);
}

