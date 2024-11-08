#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "driver/mcpwm.h"

void setup_motor_gpio();

void setup_mcpwm();

void update_speed(float dutyCycle);

void control_direction(int x, int y);

void control_motor_stop();
