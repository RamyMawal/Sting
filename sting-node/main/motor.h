
void setup_motor_gpio();

void setup_mcpwm();

/// @brief Update motor speed of a specific motor
/// @param motor Motor number (1-4)
/// @param speed Speed in m/s (sign indicates direction)
void move_motor(int motor, double speed);

/// @brief Update PID loop values and move motors
/// @param x_error Position error in X direction
/// @param y_error Position error in Y direction
/// @param yaw_error Yaw angle error
void update_pid(float x_error, float y_error, float yaw_error);

/// @brief Update movement direction based on error values
/// @param x_error Position error in X direction
/// @param y_error Position error in Y direction
/// @param rot_error Rotation error
void update_movement(float x_error, float y_error, float rot_error);

/// @brief Stop all motors immediately
void control_motor_stop();
