#ifndef UTILITIES_H
#define UTILITIES_H

#include "globals.h"
#include "sensors.h"

void fast_flash_double_LED_builtin();
void slow_flash_LED_builtin();
void speed_change_smooth();
double GYRO_controller(double gyro_target);
double IR_controller(double IR_target, int IR_mode);

#endif  // UTILITIES_H
