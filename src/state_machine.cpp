#include "state_machine.h"

#include "moving_logic.h"
#include "sensors.h"
#include "serial_command.h"
#include "utilities.h"

STATE initialising() {
  SerialCom->println("INITIALISING....");
  delay(1000);
  SerialCom->println("Enabling Motors...");
  enable_motors();
  SerialCom->println("RUNNING STATE...");
  return RUNNING;
}

STATE running() {
  static unsigned long previous_millis = 0;

  read_serial_command();
  fast_flash_double_LED_builtin();

  if (millis() - previous_millis > 500) {
    previous_millis = millis();
    SerialCom->println("RUNNING---------");
    speed_change_smooth();
    Analog_Range_A4();

#ifndef NO_READ_GYRO
    GYRO_reading();
#endif

#ifndef NO_HC_SR04
    HC_SR04_range();
#endif

#ifndef NO_BATTERY_V_OK
    if (!is_battery_voltage_OK()) return STOPPED;
#endif

    turret_motor.write(pos);
    pos = (pos == 0) ? 45 : 0;
  }

  return RUNNING;
}

STATE stopped() {
  static byte counter_lipo_voltage_ok = 0;
  static unsigned long previous_millis = 0;

  disable_motors();
  slow_flash_LED_builtin();

  if (millis() - previous_millis > 500) {
    previous_millis = millis();
    SerialCom->println("STOPPED---------");

#ifndef NO_BATTERY_V_OK
    if (is_battery_voltage_OK()) {
      SerialCom->print("Lipo OK waiting of voltage Counter 10 < ");
      SerialCom->println(counter_lipo_voltage_ok);
      counter_lipo_voltage_ok++;
      if (counter_lipo_voltage_ok > 10) {
        counter_lipo_voltage_ok = 0;
        enable_motors();
        SerialCom->println("Lipo OK returning to RUN STATE");
        return RUNNING;
      }
    } else {
      counter_lipo_voltage_ok = 0;
    }
#endif
  }
  return STOPPED;
}
