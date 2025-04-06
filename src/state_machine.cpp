#include "state_machine.h"

#include "dual_serial.h"  // <-- Add this line
#include "moving_logic.h"
#include "positioning_system.h"
#include "sensors.h"
#include "serial_command.h"
#include "utilities.h"

STATE initialising() {
  dualPrintln("INITIALISING....");
  delay(1000);  // Allow time to view message
  dualPrintln("Enabling Motors...");
  enable_motors();
  dualPrintln("RUNNING STATE...");
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
    static unsigned long last_log_time = 0;
    // if (is_logging && millis() - last_log_time > 50 &&
    //     log_index < SENSOR_LOG_SIZE) {
    //   frontLeftIR_log[log_index] = FRONT_LEFT_shortIR_reading();
    //   frontRightIR_log[log_index] = FRONT_RIGHT_shortIR_reading();
    //   backLeftIR_log[log_index] = BACK_LEFT_longIR_reading();
    //   backRightIR_log[log_index] = BACK_RIGHT_longIR_reading();
    //   ultrasonic_log[log_index] = HC_SR04_range();
    //   gyro_log[log_index] = currentAngle;
    //   log_index++;

    //   last_log_time = millis();
    // }

#ifndef NO_READ_GYRO
    // GYRO_reading(500);
    // Can't trust the readings from this cuz the current angle is now a global
    // variable Running this will mess with the GYRO controller
#endif

#ifndef NO_HC_SR04
    // HC_SR04_range();
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
    dualPrintln("STOPPED---------");

#ifndef NO_BATTERY_V_OK
    if (is_battery_voltage_OK()) {
      dualPrint("Lipo OK waiting, counter: ");
      dualPrintln(counter_lipo_voltage_ok);
      counter_lipo_voltage_ok++;
      if (counter_lipo_voltage_ok > 10) {  // Ensure voltage is stable
        counter_lipo_voltage_ok = 0;
        enable_motors();
        dualPrintln("Lipo OK returning to RUN STATE");
        return RUNNING;
      }
    } else {
      counter_lipo_voltage_ok = 0;
    }
#endif
  }
  return STOPPED;
}
