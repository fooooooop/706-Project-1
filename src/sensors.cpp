#include "sensors.h"

#ifndef NO_HC_SR04
void HC_SR04_range() {
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
      Serial.println("HC-SR04: NOT found");
      return;
    }
  }

  // Measure the length of the echo pulse
  t1 = micros();
  while (digitalRead(ECHO_PIN) == 1) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000)) {
      Serial.println("HC-SR04: Out of range");
      return;
    }
  }
  t2 = micros();
  pulse_width = t2 - t1;

  cm = pulse_width / 58.0;

  if (pulse_width > MAX_DIST) {
    Serial.println("HC-SR04: Out of range");
  } else {
    Serial.print("HC-SR04:");
    Serial.print(cm);
    Serial.println("cm");
  }
}
#endif

// void Analog_Range_A4() {
//   Serial.print("Analog Range A4:");
//   Serial.println(analogRead(A4));
// }

uint8_t FRONT_LEFT_IR_reading() {
  uint8_t distance = FrontLeftIR.getDistance();
  return distance;
}

uint8_t FRONT_RIGHT_IR_reading() {
  uint8_t distance = FrontRightIR.getDistance();
  return distance;
}

uint8_t BACK_LEFT_IR_reading() {
  uint8_t distance = BackLeftIR.getDistance();
  return distance;
}

uint8_t BACK_RIGHT_IR_reading() {
  uint8_t distance = BackRightIR.getDistance();
  return distance;
}

#ifndef NO_READ_GYRO
void GYRO_reading() {
  Serial.print("GYRO A3:");
  Serial.println(analogRead(A3));
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
