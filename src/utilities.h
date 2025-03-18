#ifndef UTILITIES_H
#define UTILITIES_H

#include "globals.h"
#include "sensors.h"

void fast_flash_double_LED_builtin();
void slow_flash_LED_builtin();
void speed_change_smooth();
void GYRO_controller();
void IR_controller();

#endif  // UTILITIES_H
