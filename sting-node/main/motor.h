#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "driver/mcpwm.h"

void setup_motor_gpio();

void setup_mcpwm();

/// @brief Update speed value of (X,Y)
/// @param x_duty_cycle X duty cycle value, between 0,1 (any out of range is reduced to 1)
/// @param y_duty_cycle Y duty cycle value, between 0,1 (any out of range 1 is reduced to 1)
void update_speed(float x_duty_cycle, float y_duty_cycle);

/// @brief Update movement direction to a vector of (X,Y)
/// @param x X vector value, between -1,1 (any value above 1 is reduced to 1)
/// @param y Y vector value, between -1,1 (any value above 1 is reduced to 1)
void update_direction(float x, float y);


/// @brief Update motor speed of a specific motor
/// @param motor 
/// @param speed 
void move_motor(int motor, double speed);

/// @brief update PID loop values and move motors
/// @param x_error 
/// @param y_error 
/// @param rot_error 
void update_pid(float x_error, float y_error, float rot_error);

/// @brief Update movement direction based on error values
/// @param x_error
/// @param y_error
/// @param rot_error
void update_movement(float x_error, float y_error, float rot_error);

void control_motor_stop();
