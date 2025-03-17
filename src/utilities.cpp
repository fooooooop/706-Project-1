#include "utilities.h"

void fast_flash_double_LED_builtin() {
  static byte indexer = 0;
  static unsigned long fast_flash_millis;
  if (millis() > fast_flash_millis) {
    indexer++;
    if (indexer > 4) {
      fast_flash_millis = millis() + 700;
      digitalWrite(LED_BUILTIN, LOW);
      indexer = 0;
    } else {
      fast_flash_millis = millis() + 100;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
}

void slow_flash_LED_builtin() {
  static unsigned long slow_flash_millis;
  if (millis() - slow_flash_millis > 2000) {
    slow_flash_millis = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void speed_change_smooth() {
  speed_val += speed_change;
  if (speed_val > 1000) speed_val = 1000;
  speed_change = 0;
}

void GYRO_controller() {
  //Setup!------------------------//

  // Time variables
  double t_current = 0;
  double t_previous = 0;

  // General error variables
  double gyro_currentSensor;
  double gyro_err_current;
  double gyro_target = 500;
  double gyro_err_previous = 0;

  // K variables for controller
  double kp = 5.5;
  double ki = 1.2;
  double kd = 0;

  // For Derivative controller
  double dt;
  double de;
  double dedt;

  // For Integral controller
  double gyro_err_mem = 0;

  //Main Code Start!------------------------//
  // To get dt for Integral and Derivative controllers
  t_current = millis();
  dt = (t_current - t_previous) / 1000;
  t_previous = t_current;

  // Proportional controller
  gyro_currentSensor = analogRead(A3); // Gyro reading
  gyro_err_current = gyro_target - gyro_currentSensor;

  // Integral controller
  if (gyro_u < 400) { // Anti-integral windup
    gyro_err_mem += gyro_err_current;
  }
  
  // Derivative controller
  de = (gyro_err_current - gyro_err_previous);
  gyro_err_previous = gyro_err_current;

  dedt = (de / dt);

  // PID controller
  gyro_u = kp*gyro_err_current + ki+gyro_err_mem + kd*dedt;

  return;
}