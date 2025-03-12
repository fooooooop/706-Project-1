#include "state_machine.h"

#include "moving_logic.h"
#include "sensors.h"
#include "serial_command.h"
#include "utilities.h"

STATE initialising() {
  Serial.println("INITIALISING....");
  delay(1000);  // Allow time to view message
  Serial.println("Enabling Motors...");
  enable_motors();
  Serial.println("RUNNING STATE...");
  return RUNNING;
}

STATE running() {
  static unsigned long previous_millis = 0;

  // Read wireless command from HC‑12 (Serial1)
  read_serial_command();

  // Blink the built-in LED quickly to indicate running state
  fast_flash_double_LED_builtin();

  // Perform periodic tasks every 500ms
  if (millis() - previous_millis > 500) {
    previous_millis = millis();
    Serial.println("RUNNING---------");
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

    // Update turret position as an example
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

  if (millis() - previous_millis > 500) {  // Print every 500ms
    previous_millis = millis();
    Serial.println("STOPPED---------");

#ifndef NO_BATTERY_V_OK
    if (is_battery_voltage_OK()) {
      Serial.print("Lipo OK waiting, counter: ");
      Serial.println(counter_lipo_voltage_ok);
      counter_lipo_voltage_ok++;
      if (counter_lipo_voltage_ok > 10) {  // Ensure voltage is stable
        counter_lipo_voltage_ok = 0;
        enable_motors();
        Serial.println("Lipo OK returning to RUN STATE");
        return RUNNING;
      }
    } else {
      counter_lipo_voltage_ok = 0;
    }
#endif
  }
  return STOPPED;
}