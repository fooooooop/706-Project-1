#include "sensors.h"

#define MEDIAN_WINDOW 3

#ifndef NO_HC_SR04
float HC_SR04_range() {
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;

  // Trigger the sensor
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for echo to start
  t1 = micros();
  while (digitalRead(ECHO_PIN) == 0) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000)) {
      dualPrintln("HC-SR04: NOT found");
      return 0;
    }
  }

  // Measure the length of the echo pulse
  t1 = micros();
  while (digitalRead(ECHO_PIN) == 1) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000)) {
      dualPrintln("HC-SR04: Out of range");
      return 0;
    }
  }
  t2 = micros();
  pulse_width = t2 - t1;

  cm = pulse_width / 58.0;

  if (pulse_width > MAX_DIST) {
    dualPrintln("HC-SR04: Out of range");
  } else {
    // dualPrintln("HC-SR04:");
    // dualPrintln(cm);
    // dualPrintln("cm");
  }

  return cm;
}
#endif

// Return IR sensor readings
uint16_t FRONT_LEFT_shortIR_reading() { return FrontLeftIR.getDist(); }

uint16_t FRONT_RIGHT_shortIR_reading() { return FrontRightIR.getDist(); }

uint16_t BACK_LEFT_longIR_reading() { return BackLeftIR.getDist(); }

uint16_t BACK_RIGHT_longIR_reading() { return BackRightIR.getDist(); }

#ifndef NO_READ_GYRO
void GYRO_reading(int T) {
  // T is delay for loop

  // convert the 0-1023 signal to 0-5v
  gyroRate = (analogRead(A3)*gyroSupplyVoltage)/1023;
  // find the voltage offset the value of voltage when gyro is zero (still)
  gyroRate -= (gyroZeroVoltage/1023*5);
  // read out voltage divided the gyro sensitivity to calculate the angular velocity
  float angularVelocity = gyroRate/ gyroSensitivity;
  // if the angular velocity is less than the threshold, ignore it
  if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold)
  {
    // we are running a loop in T. one second will run (1000/T).
    float angleChange = angularVelocity/(1000/T);
    currentAngle += angleChange;
  }

  if (abs(angularVelocity) > maxGyroDrift) maxGyroDrift = abs(angularVelocity);

  // keep the angle between 0-360
  // 183 threshold so we can do 180 degree turns
  if (currentAngle < -183) {currentAngle += 360;}
  else if (currentAngle > 183) {currentAngle -= 360;}
  
  delay(T);
  return;
}
#endif

#ifndef NO_BATTERY_V_OK
boolean is_battery_voltage_OK() {
  static byte Low_voltage_counter;
  int Lipo_level_cal;
  int raw_lipo;

  raw_lipo = analogRead(A0);
  Lipo_level_cal = (raw_lipo - 717);
  Lipo_level_cal = Lipo_level_cal * 100;
  Lipo_level_cal = Lipo_level_cal / 143;

  if (Lipo_level_cal > 0 && Lipo_level_cal < 160) {
    Serial.print("Lipo level:");
    Serial.print(Lipo_level_cal);
    Serial.print("%");
    Serial.println("");
    Low_voltage_counter = 0;
    return true;
  } else {
    if (Lipo_level_cal < 0)
      Serial.println("Lipo is Disconnected or Power Switch is turned OFF!!!");
    else if (Lipo_level_cal > 160)
      Serial.println("!Lipo is Overcharged!!!");
    else {
      Serial.println(
          "Lipo voltage too LOW, any lower and the lipo will be damaged");
      Serial.print("Please Re-charge Lipo:");
      Serial.print(Lipo_level_cal);
      Serial.println("%");
    }
    Low_voltage_counter++;
    if (Low_voltage_counter > 5)
      return false;
    else
      return true;
  }
}
#endif
