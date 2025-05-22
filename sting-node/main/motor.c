#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "driver/mcpwm.h"
#include "esp_log.h"
#include "motor.h"
#include <math.h>

#define TAG "MOTORS"

#define MOTOR_1_A 36
#define MOTOR_1_B 35
#define MOTOR_1_PWM 5
#define MOTOR_2_A 7
#define MOTOR_2_B 6
#define MOTOR_2_PWM 4
#define MOTOR_3_A 15
#define MOTOR_3_B 16
#define MOTOR_3_PWM 10
#define MOTOR_4_A 37
#define MOTOR_4_B 38
#define MOTOR_4_PWM 11


#define PWM_FREQUENCY 1000    // 1kHz PWM frequency
#define PWM_DUTY_CYCLE_DEF 50 // 75% duty cycle for speed control
#define PWM_DUTY_CYCLE_MIN 35
#define PWM_DUTY_CYCLE_MAX 80

#define PID_DT 0.1
#define PID_KP 1.3
#define PID_KI 0.00005
#define PID_KD 0.01

float x_error_prev = 0;
float y_error_prev = 0;
float rot_error_prev = 0;
float pid_x_integral = 0;
float pid_y_integral = 0;
float pid_rot_integral = 0;

void setup_motor_gpio()
{
    esp_rom_gpio_pad_select_gpio(MOTOR_1_B);
    esp_rom_gpio_pad_select_gpio(MOTOR_1_A);
    esp_rom_gpio_pad_select_gpio(MOTOR_4_A);
    esp_rom_gpio_pad_select_gpio(MOTOR_4_B);
    esp_rom_gpio_pad_select_gpio(MOTOR_2_A);
    esp_rom_gpio_pad_select_gpio(MOTOR_2_B);
    esp_rom_gpio_pad_select_gpio(MOTOR_3_B);
    esp_rom_gpio_pad_select_gpio(MOTOR_3_A);

    gpio_set_direction(MOTOR_1_B, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_1_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_4_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_4_B, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_2_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_2_B, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_3_B, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_3_A, GPIO_MODE_OUTPUT);
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

/// @brief Update speed value of (X,Y)
/// @param x_duty_cycle X duty cycle value, between 0,1 (any out of range is set to 1)
/// @param y_duty_cycle Y duty cycle value, between 0,1 (any out of range is set to 1)
void update_speed(float x_duty_cycle, float y_duty_cycle)
{
    x_duty_cycle = fabs(x_duty_cycle);
    y_duty_cycle = fabs(y_duty_cycle);

    // Clamp to Duty cycle min/max range
    if(x_duty_cycle > 0.1f)
        x_duty_cycle = x_duty_cycle < PWM_DUTY_CYCLE_MIN
                           ? PWM_DUTY_CYCLE_MIN
                        : x_duty_cycle > PWM_DUTY_CYCLE_MAX
                           ? PWM_DUTY_CYCLE_MAX
                           : x_duty_cycle * 100;
    else 
        x_duty_cycle = 0;

    if(y_duty_cycle > 0.1f)
        y_duty_cycle = y_duty_cycle < PWM_DUTY_CYCLE_MIN
                           ? PWM_DUTY_CYCLE_MIN
                        : y_duty_cycle > PWM_DUTY_CYCLE_MAX
                           ? PWM_DUTY_CYCLE_MAX
                           : y_duty_cycle * 100;
    else
        y_duty_cycle = 0;

    ESP_LOGI(TAG, "x_duty_cycle value: %f", x_duty_cycle);
    ESP_LOGI(TAG, "y_duty_cycle value: %f", y_duty_cycle);

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, x_duty_cycle);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, y_duty_cycle);
}


void move_motor(int motor, double speed)
{
    bool forward = speed > 0 ? true : false;

    speed = fabs(speed) * 100;

    // if(speed > 0.1f)
    //     speed = speed < PWM_DUTY_CYCLE_MIN
    //                ? PWM_DUTY_CYCLE_MIN
    //             : speed > PWM_DUTY_CYCLE_MAX
    //                ? PWM_DUTY_CYCLE_MAX
    //                : speed * 100;
    // else
    //     speed = 0;

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


void update_movement(float x_error, float y_error, float rot_error)
{
    //float array of 4 values
    double motor_speed[4] = {0, 0, 0, 0};

    ESP_LOGI(TAG, "x_error value: %f", x_error);
    ESP_LOGI(TAG, "y_error value: %f", y_error);
    ESP_LOGI(TAG, "rot_error value: %f", rot_error);

    motor_speed[0] = (x_error * -sin(45)) + (y_error * cos(45)) + (rot_error);
    motor_speed[1] = (x_error * -sin(135)) + (y_error * cos(135)) + (rot_error);
    motor_speed[2] = (x_error * -sin(225)) + (y_error * cos(225)) + (rot_error);
    motor_speed[3] = (x_error * -sin(315)) + (y_error * cos(315)) + (rot_error);

    for (size_t i = 0; i <= 3; i++)
    {
        move_motor(i + 1, motor_speed[i]);
    }
}

/// @brief update PID loop values and move motors
/// @param x_error
/// @param y_error
/// @param rot_error

void update_pid(float x_error, float y_error, float rot_error)
{

    // Calculate PID values
    pid_x_integral = pid_x_integral + (x_error * PID_DT);
    pid_y_integral = pid_y_integral + (y_error * PID_DT);
    pid_rot_integral = pid_rot_integral + (rot_error * PID_DT);

    float x_pid = (x_error * PID_KP) + (pid_x_integral * PID_KI) + ((x_error - x_error_prev) / PID_DT * PID_KD);
    float y_pid = (y_error * PID_KP) + (pid_y_integral * PID_KI) + ((y_error - y_error_prev) / PID_DT * PID_KD);
    float rot_pid = (rot_error * PID_KP) + (pid_rot_integral * PID_KI) + ((rot_error - rot_error_prev) / PID_DT * PID_KD);
    ESP_LOGI(TAG, "x_pid value: %f", x_pid);
    ESP_LOGI(TAG, "y_pid value: %f", y_pid);
    ESP_LOGI(TAG, "rot_pid value: %f", rot_pid);


    // Update PID loop values
    x_error_prev = x_error;
    y_error_prev = y_error;
    rot_error_prev = rot_error;


    double motor_speed[4] = {0, 0, 0, 0};

    motor_speed[0] = (x_pid * -sin(45)) + (y_pid * cos(45)) + (rot_pid);
    motor_speed[1] = (x_pid * -sin(135)) + (y_pid * cos(135)) + (rot_pid);
    motor_speed[2] = (x_pid * -sin(225)) + (y_pid * cos(225)) + (rot_pid);
    motor_speed[3] = (x_pid * -sin(315)) + (y_pid * cos(315)) + (rot_pid);

    for (size_t i = 0; i <= 3; i++)
    {
        move_motor(i + 1, motor_speed[i]);
    }
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
        gpio_set_level(MOTOR_1_B, 1);
        gpio_set_level(MOTOR_1_A, 0);
        gpio_set_level(MOTOR_4_A, 1);
        gpio_set_level(MOTOR_4_B, 0);
    }
    else
    {
        gpio_set_level(MOTOR_1_B, 0);
        gpio_set_level(MOTOR_1_A, 1);
        gpio_set_level(MOTOR_4_A, 0);
        gpio_set_level(MOTOR_4_B, 1);
    }

    if (y > 0)
    {
        gpio_set_level(MOTOR_2_A, 1);
        gpio_set_level(MOTOR_2_B, 0);
        gpio_set_level(MOTOR_3_B, 1);
        gpio_set_level(MOTOR_3_A, 0);
    }
    else
    {
        gpio_set_level(MOTOR_2_A, 0);
        gpio_set_level(MOTOR_2_B, 1);
        gpio_set_level(MOTOR_3_B, 0);
        gpio_set_level(MOTOR_3_A, 1);
    }
}

void control_motor_stop()
{
    gpio_set_level(MOTOR_1_B, 0);
    gpio_set_level(MOTOR_1_A, 0);
    gpio_set_level(MOTOR_4_A, 0);
    gpio_set_level(MOTOR_4_B, 0);

    gpio_set_level(MOTOR_2_A, 0);
    gpio_set_level(MOTOR_2_B, 0);
    gpio_set_level(MOTOR_3_B, 0);
    gpio_set_level(MOTOR_3_A, 0);
}
