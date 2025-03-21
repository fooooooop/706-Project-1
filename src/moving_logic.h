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

#endif  // MOVING_LOGIC_H
