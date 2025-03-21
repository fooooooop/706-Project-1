#ifndef MOVING_LOGIC_H
#define MOVING_LOGIC_H

#include "globals.h"
#include "utilities.h"
#include "sensors.h"

// Motor control functions
void enable_motors();
void disable_motors();
void stop_motors();
void forward();
void reverse();
void ccw();
void cw();
void strafe_left();
void strafe_right();

void find_corner();
void turn_angle(double target);
void forward_target(double target_sidewall, double target);
void reverse_target(double target_sidewall, double target);
void strafe_target(double target, int left_right);
void loop_one();

#endif  // MOVING_LOGIC_H
