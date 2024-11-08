#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "driver/mcpwm.h"

void setup_motor_gpio();

void setup_mcpwm();

void update_speed_A(float dutyCycle);

void update_speed_B(float dutyCycle);

void control_motor_forward(int motor);

void control_motor_backward(int motor);

void control_motor_stop(int motor);
