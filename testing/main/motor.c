#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "driver/mcpwm.h"
#include "motor.h"
#include <math.h>

// X Motors pin definition
#define MOTOR_X_1_A 6
#define MOTOR_X_1_B 7
#define MOTOR_X_2_A 15
#define MOTOR_X_2_B 16
#define MOTOR_PWM_X 5

// Y Motors pin definition
#define MOTOR_Y_1_A 35
#define MOTOR_Y_1_B 36
#define MOTOR_Y_2_A 37
#define MOTOR_Y_2_B 38
#define MOTOR_PWM_Y 10

// PWM Configuration
#define PWM_FREQUENCY 1000 // 1kHz PWM frequency
#define PWM_DUTY_CYCLE 50  // 75% duty cycle for speed control

void setup_motor_gpio()
{

    esp_rom_gpio_pad_select_gpio(MOTOR_X_1_A);
    esp_rom_gpio_pad_select_gpio(MOTOR_X_1_B);
    esp_rom_gpio_pad_select_gpio(MOTOR_X_2_A);
    esp_rom_gpio_pad_select_gpio(MOTOR_X_2_B);
    esp_rom_gpio_pad_select_gpio(MOTOR_Y_1_A);
    esp_rom_gpio_pad_select_gpio(MOTOR_Y_1_B);
    esp_rom_gpio_pad_select_gpio(MOTOR_Y_2_A);
    esp_rom_gpio_pad_select_gpio(MOTOR_Y_2_B);

    gpio_set_direction(MOTOR_X_1_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_X_1_B, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_X_2_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_X_2_B, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_Y_1_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_Y_1_B, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_Y_2_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_Y_2_B, GPIO_MODE_OUTPUT);
}

void setup_mcpwm()
{
    // Initialize MCPWM on unit 0, timer 0
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR_PWM_X);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MOTOR_PWM_Y);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = PWM_FREQUENCY; // Set PWM frequency
    pwm_config.cmpr_a = PWM_DUTY_CYCLE;   // Set duty cycle for PWM0A (0-100%)
    pwm_config.cmpr_b = PWM_DUTY_CYCLE;   // Set duty cycle for PWM0A (0-100%)
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config); // Initialize MCPWM
}

/// @brief Update speed value of (X,Y)
/// @param x_duty_cycle X duty cycle value, between 0,1 (any out of range is reduced to 1)
/// @param y_duty_cycle Y duty cycle value, between 0,1 (any out of range 1 is reduced to 1)
void update_speed(float x_duty_cycle, float y_duty_cycle)
{
    x_duty_cycle = fabs(x_duty_cycle);
    y_duty_cycle = fabs(y_duty_cycle);

    // Scale to [0,100] range
    x_duty_cycle = x_duty_cycle > 1 || x_duty_cycle < 0 ? 100 : x_duty_cycle * 100;
    y_duty_cycle = y_duty_cycle > 1 || y_duty_cycle < 0 ? 100 : y_duty_cycle * 100;

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, x_duty_cycle);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, y_duty_cycle);
}

/// @brief Update movement direction to a vector of (X,Y)
/// @param x X vector value, between -1,1 (any value above 1 is reduced to 1)
/// @param y Y vector value, between -1,1 (any value above 1 is reduced to 1)
void update_direction(float x, float y)
{
    // Scale down to [-1,1] range
    x = x > 1 ? 1 : x;
    y = y > 1 ? 1 : y;
    x = x < -1 ? -1 : x;
    y = y < -1 ? -1 : y;

    // Rotate and normalize the vector
    // Multiplying by 1/sqrt(2) should correct the vector's length

    x = (x * 1 / sqrt(2)) + (y * 1 / sqrt(2)) * 1 / sqrt(2);
    y = (x * -1 / sqrt(2)) + (y * 1 / sqrt(2)) * 1 / sqrt(2);

    update_speed(x, y);

    if (x > 0)
    {
        gpio_set_level(MOTOR_X_1_A, 1);
        gpio_set_level(MOTOR_X_1_B, 0);
        gpio_set_level(MOTOR_X_2_A, 1);
        gpio_set_level(MOTOR_X_2_B, 0);
    }
    else
    {
        gpio_set_level(MOTOR_X_1_A, 0);
        gpio_set_level(MOTOR_X_1_B, 1);
        gpio_set_level(MOTOR_X_2_A, 0);
        gpio_set_level(MOTOR_X_2_B, 1);
    }

    if (y > 0)
    {
        gpio_set_level(MOTOR_Y_1_A, 1);
        gpio_set_level(MOTOR_Y_1_B, 0);
        gpio_set_level(MOTOR_Y_2_A, 1);
        gpio_set_level(MOTOR_Y_2_B, 0);
    }
    else
    {
        gpio_set_level(MOTOR_Y_1_A, 0);
        gpio_set_level(MOTOR_Y_1_B, 1);
        gpio_set_level(MOTOR_Y_2_A, 0);
        gpio_set_level(MOTOR_Y_2_B, 1);
    }
}

void control_motor_stop()
{
    gpio_set_level(MOTOR_X_1_A, 0);
    gpio_set_level(MOTOR_X_1_B, 0);
    gpio_set_level(MOTOR_X_2_A, 0);
    gpio_set_level(MOTOR_X_2_B, 0);

    gpio_set_level(MOTOR_Y_1_A, 0);
    gpio_set_level(MOTOR_Y_1_B, 0);
    gpio_set_level(MOTOR_Y_2_A, 0);
    gpio_set_level(MOTOR_Y_2_B, 0);
}
