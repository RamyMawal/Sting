#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "driver/mcpwm.h"
#include "motor.h"

#define MOTOR_FW_R_A 6
#define MOTOR_FW_R_B 7
#define MOTOR_FW_L_A 15
#define MOTOR_FW_L_B 16
#define MOTOR_BW_R_A 35
#define MOTOR_BW_R_B 36
#define MOTOR_BW_L_A 37
#define MOTOR_BW_L_B 38
#define MOTOR_PWM 9

// #define MOTOR_FW_R_PWM 9
// #define MOTOR_FW_L_PWM 10
// #define MOTOR_BW_R_PWM 40
// #define MOTOR_BW_L_PWM 41


// PWM Configuration
#define PWM_FREQUENCY 1000 // 1kHz PWM frequency
#define PWM_DUTY_CYCLE 50  // 75% duty cycle for speed control

void setup_motor_gpio(){

    esp_rom_gpio_pad_select_gpio(MOTOR_FW_L_A);
    esp_rom_gpio_pad_select_gpio(MOTOR_FW_L_B);
    esp_rom_gpio_pad_select_gpio(MOTOR_FW_R_A);
    esp_rom_gpio_pad_select_gpio(MOTOR_FW_R_B);
    esp_rom_gpio_pad_select_gpio(MOTOR_BW_L_A);
    esp_rom_gpio_pad_select_gpio(MOTOR_BW_L_B);
    esp_rom_gpio_pad_select_gpio(MOTOR_BW_R_A);
    esp_rom_gpio_pad_select_gpio(MOTOR_BW_R_B);

    gpio_set_direction(MOTOR_FW_R_B, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_FW_R_B , GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_FW_L_A , GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_FW_L_B , GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_BW_R_A , GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_BW_R_B , GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_BW_L_A , GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_BW_L_B , GPIO_MODE_OUTPUT);
}

void setup_mcpwm() {
    // Initialize MCPWM on unit 0, timer 0
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR_PWM );
    
    mcpwm_config_t pwm_config;
    pwm_config.frequency = PWM_FREQUENCY;  // Set PWM frequency
    pwm_config.cmpr_a = PWM_DUTY_CYCLE;    // Set duty cycle for PWM0A (0-100%)
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);  // Initialize MCPWM
}

void update_speed(float dutyCycle)
{
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, dutyCycle);
}

void control_direction(int x, int y){
    
}

void control_motor_stop() {
    gpio_set_level(MOTOR_BW_L_A, 0);
    gpio_set_level(MOTOR_BW_L_B, 0);
    gpio_set_level(MOTOR_BW_R_A, 0);
    gpio_set_level(MOTOR_BW_R_B, 0);
    gpio_set_level(MOTOR_FW_L_A, 0);
    gpio_set_level(MOTOR_FW_L_B, 0);
    gpio_set_level(MOTOR_FW_R_A, 0);
    gpio_set_level(MOTOR_FW_R_B, 0);
 }

