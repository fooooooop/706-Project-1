#include <Arduino.h>

#include "globals.h"
#include "state_machine.h"

// Instantiate servo objects and global variables
Servo left_front_motor;
Servo left_rear_motor;
Servo right_rear_motor;
Servo right_front_motor;
Servo turret_motor;

int speed_val = 100;
int speed_change = 0;
int pos = 0;
HardwareSerial *SerialCom;

void setup() {
  turret_motor.attach(11);
  pinMode(LED_BUILTIN, OUTPUT);

  // Setup ultrasonic sensor trigger pin
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Initialize Serial communication
  SerialCom = &Serial;
  SerialCom->begin(115200);
  SerialCom->println("MECHENG706_Base_Code_25/01/2018");
  delay(1000);
  SerialCom->println("Setup....");
  delay(1000);
}

void loop() {
  static STATE machine_state = INITIALISING;
  switch (machine_state) {
    case INITIALISING:
      machine_state = initialising();
      break;
    case RUNNING:
      machine_state = running();
      break;
    case STOPPED:
      machine_state = stopped();
      break;
  }
}
