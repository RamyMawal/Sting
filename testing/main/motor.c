
#include "motor.h"

#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "driver/mcpwm.h"


#define GPIO_MOTOR_1_FW 5
#define GPIO_MOTOR_1_BW 6
#define GPIO_MOTOR_2_FW 9
#define GPIO_MOTOR_2_BW 10
#define PWM_GPIO_PIN_A 18 // PWM pin for controlling motor speed
#define PWM_GPIO_PIN_B 17 // PWM pin for controlling motor speed

// PWM Configuration
#define PWM_FREQUENCY 1000 // 1kHz PWM frequency
#define PWM_DUTY_CYCLE 50  // 75% duty cycle for speed control

void setup_motor_gpio(){
    esp_rom_gpio_pad_select_gpio(GPIO_MOTOR_1_FW);
    esp_rom_gpio_pad_select_gpio(GPIO_MOTOR_1_BW);
    gpio_set_direction(GPIO_MOTOR_1_FW, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_MOTOR_1_BW, GPIO_MODE_OUTPUT);
    
    esp_rom_gpio_pad_select_gpio(GPIO_MOTOR_2_FW);
    esp_rom_gpio_pad_select_gpio(GPIO_MOTOR_2_BW);
    gpio_set_direction(GPIO_MOTOR_2_FW, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_MOTOR_2_BW, GPIO_MODE_OUTPUT);
}


void update_speed_A(float dutyCycle)
{
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, dutyCycle);
}

void update_speed_B(float dutyCycle)
{
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, dutyCycle);
}
// void update_speed(float dutyCycle){
//     update_speed(dutyCycle);
// }


void setup_mcpwm() {
    // Initialize MCPWM on unit 0, timer 0
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_GPIO_PIN_A);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, PWM_GPIO_PIN_B);
    
    mcpwm_config_t pwm_config;
    pwm_config.frequency = PWM_FREQUENCY;  // Set PWM frequency
    pwm_config.cmpr_a = PWM_DUTY_CYCLE;    // Set duty cycle for PWM0A (0-100%)
    pwm_config.cmpr_b = PWM_DUTY_CYCLE;    // Set duty cycle for PWM0A (0-100%)
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);  // Initialize MCPWM
}

void control_motor_forward(int motor) {
    if(motor == 1){
        gpio_set_level(GPIO_MOTOR_1_FW, 1);
        gpio_set_level(GPIO_MOTOR_1_BW, 0);
    }
    else{
        gpio_set_level(GPIO_MOTOR_2_FW, 1);
        gpio_set_level(GPIO_MOTOR_2_BW, 0);
    }
}

void control_motor_backward(int motor) {
    if(motor == 1){
        gpio_set_level(GPIO_MOTOR_1_FW, 0);
        gpio_set_level(GPIO_MOTOR_1_BW, 1);
    }
    else{
        gpio_set_level(GPIO_MOTOR_2_FW, 0);
        gpio_set_level(GPIO_MOTOR_2_BW, 1);
    }
}

void control_motor_stop(int motor) {
    if(motor == 1){
        gpio_set_level(GPIO_MOTOR_1_FW, 0);
        gpio_set_level(GPIO_MOTOR_1_BW, 0);
    }
    else{
        gpio_set_level(GPIO_MOTOR_2_FW, 0);
        gpio_set_level(GPIO_MOTOR_2_BW, 0);
    }
}

