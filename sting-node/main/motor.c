#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "driver/mcpwm.h"
#include "esp_log.h"
#include "motor.h"
#include <math.h>


#define TAG "MOTORS"

#define MOTOR_4_PWM 11
#define MOTOR_4_A 10
#define MOTOR_4_B 9
#define MOTOR_1_A 46
#define MOTOR_1_B 3
#define MOTOR_1_PWM 8

#define MOTOR_2_PWM 5
#define MOTOR_2_A 6
#define MOTOR_2_B 7
#define MOTOR_3_A 15
#define MOTOR_3_B 16
#define MOTOR_3_PWM 17

#define PWM_FREQUENCY 1000 // 1kHz PWM frequency
#define PWM_DUTY_DEAD 2.0f
#define PWM_DUTY_CYCLE_MIN 50.0f
#define PWM_DUTY_CYCLE_MAX 100.0f


#define R 0.135  // meters
#define w_max 29 // rad per second

// Macros for motor speed
#define calculate_motor_speeds(motor_speed, x, y, yaw)                    \
{                                                                         \
    motor_speed[0] = (x * 1 / sqrt(2)) + (y * 1 / sqrt(2)) + (yaw * R);     \
    motor_speed[1] = (x * 1 / sqrt(2)) + (y * -1 / sqrt(2)) + (yaw * R);    \
    motor_speed[2] = (x * -1 / sqrt(2)) + (y * -1 / sqrt(2)) + (yaw * R);   \
    motor_speed[3] = (x * -1 / sqrt(2)) + (y * 1 / sqrt(2)) + (yaw * R);    \
}

//PID_PARAMS

#define PID_DT 0.05f

#if ID == 0
#define PID_X_KP 0.6f
#define PID_Y_KP 0.6f
#define PID_YAW_KP 0.85f
#else
#define PID_X_KP 0.9f
#define PID_Y_KP 0.9f
#define PID_YAW_KP 0.7f
#endif /* if ID == 0 */

#define PID_X_KI 0.05f
#define PID_X_KD 0f
#define PID_X_ALPHA 0.7f

#define PID_Y_KI 0.05f
#define PID_Y_KD 0f
#define PID_Y_ALPHA 0.7f

#define PID_YAW_KI 0.05f
#define PID_YAW_KD 0f
#define PID_YAW_ALPHA 0.7f



float x_error_prev = 0;
float y_error_prev = 0;
float yaw_error_prev = 0;

float pid_x_integral = 0;
float pid_y_integral = 0;
float pid_yaw_integral = 0;

float pid_x_derivative = 0;
float pid_y_derivative = 0;
float pid_yaw_derivative = 0;

void setup_motor_gpio()
{
    // Use modern GPIO API instead of deprecated esp_rom_gpio_pad_select_gpio()
    gpio_reset_pin(MOTOR_1_B);
    gpio_reset_pin(MOTOR_1_A);
    gpio_reset_pin(MOTOR_4_A);
    gpio_reset_pin(MOTOR_4_B);
    gpio_reset_pin(MOTOR_2_A);
    gpio_reset_pin(MOTOR_2_B);
    gpio_reset_pin(MOTOR_3_B);
    gpio_reset_pin(MOTOR_3_A);

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
    pwm_config_1.frequency = PWM_FREQUENCY;
    pwm_config_1.cmpr_a = 0;
    pwm_config_1.cmpr_b = 0;
    pwm_config_1.counter_mode = MCPWM_UP_COUNTER;
    pwm_config_1.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config_1);

    mcpwm_config_t pwm_config_2;
    pwm_config_2.frequency = PWM_FREQUENCY;
    pwm_config_2.cmpr_a = 0;
    pwm_config_2.cmpr_b = 0;
    pwm_config_2.counter_mode = MCPWM_UP_COUNTER;
    pwm_config_2.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config_2);

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 0);
}

/// @brief Update speed of a single motor
/// @param motor motor number (1-4)
/// @param speed in m/s up to the maximum speed of the motor, sign represents direction
void move_motor(int motor, double speed)
{
    // ESP_LOGI(TAG, "Motor %d speed: %f", motor, speed);
    bool forward = speed > 0 ? true : false;
    float duty = fabs(speed) / (R * w_max) * 100;

    if(duty < PWM_DUTY_DEAD)
        duty = 0;
    else
    {
        duty = ((duty / 100) * (PWM_DUTY_CYCLE_MAX - PWM_DUTY_CYCLE_MIN)) + PWM_DUTY_CYCLE_MIN;

        if(duty > PWM_DUTY_CYCLE_MAX)
            duty = PWM_DUTY_CYCLE_MAX;
    }
    ESP_LOGI(TAG, "Motor %d duty: %f", motor, duty);

    switch (motor)
    {
        case 1:
            gpio_set_level(MOTOR_1_A, forward);
            gpio_set_level(MOTOR_1_B, !forward);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty);
            break;
        case 2:
            gpio_set_level(MOTOR_2_A, forward);
            gpio_set_level(MOTOR_2_B, !forward);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, duty);
            break;
        case 3:
            gpio_set_level(MOTOR_3_A, forward);
            gpio_set_level(MOTOR_3_B, !forward);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, duty);
            break;
        case 4:
            gpio_set_level(MOTOR_4_A, forward);
            gpio_set_level(MOTOR_4_B, !forward);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, duty);
            break;
        default:
            break;
    }
}

void update_movement(float x_error, float y_error, float yaw_error)
{
    double motor_speeds[4] = {0, 0, 0, 0};

    ESP_LOGI(TAG, "x_error value: %f", x_error);
    ESP_LOGI(TAG, "y_error value: %f", y_error);
    ESP_LOGI(TAG, "yaw_error value: %f", yaw_error);

    calculate_motor_speeds(motor_speeds, x_error, y_error, yaw_error);

    for (size_t i = 0; i <= 3; i++)
    {
        move_motor(i + 1, motor_speeds[i]);
    }
}

/// @brief update PID loop values and move motors
/// @param x_error
/// @param y_error
/// @param yaw_error

void update_pid(float x_error, float y_error, float yaw_error)
{

    //Raw derivatives
    // float x_raw_d = (x_error - x_error_prev) / PID_DT;
    // float y_raw_d = (y_error - y_error_prev) / PID_DT;
    // float yaw_raw_d = (yaw_error - yaw_error_prev) / PID_DT;
    //
    //Low pass fiter
    // float x_d = PID_X_ALPHA * pid_x_derivative + (1 - PID_X_ALPHA) * x_raw_d;
    // float y_d = PID_Y_ALPHA * pid_y_derivative + (1 - PID_Y_ALPHA) * y_raw_d;
    // float yaw_d = PID_YAW_ALPHA * pid_yaw_derivative + (1 - PID_YAW_ALPHA) * yaw_raw_d;

    float x_pid = PID_X_KP * x_error;
    float y_pid = PID_Y_KP * y_error;
    float yaw_pid = PID_YAW_KP * yaw_error;

    //store the errors for the next iteration
    x_error_prev = x_error;
    y_error_prev = y_error;
    yaw_error_prev = yaw_error;

    double motor_speed[4] = {0, 0, 0, 0};

    calculate_motor_speeds(motor_speed, x_pid, y_pid, yaw_pid);

    for (size_t i = 0; i <= 3; i++)
    {
        move_motor(i + 1, motor_speed[i]);
    }
}

void control_motor_stop()
{
    gpio_set_level(MOTOR_1_B, 0);
    gpio_set_level(MOTOR_1_A, 0);
    gpio_set_level(MOTOR_2_A, 0);
    gpio_set_level(MOTOR_2_B, 0);
    gpio_set_level(MOTOR_3_B, 0);
    gpio_set_level(MOTOR_3_A, 0);
    gpio_set_level(MOTOR_4_A, 0);
    gpio_set_level(MOTOR_4_B, 0);
}
