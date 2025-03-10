#ifndef SENSORS_H
#define SENSORS_H

#include "globals.h"

#ifndef NO_HC_SR04
void HC_SR04_range();
#endif

void Analog_Range_A4();

#ifndef NO_READ_GYRO
void GYRO_reading();
#endif

#ifndef NO_BATTERY_V_OK
boolean is_battery_voltage_OK();
#endif

#endif  // SENSORS_H
