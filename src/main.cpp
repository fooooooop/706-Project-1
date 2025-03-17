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
int fl_change = 0;
int fr_change = -60;
int bl_change = 0;
int br_change = 0;
int pos = 0;
double gyro_u = 0;

void setup(void) {
  // Attach turret servo (example pin 11)
  turret_motor.attach(11);
  pinMode(LED_BUILTIN, OUTPUT);

  // Setup ultrasonic trigger pin
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Initialize USB Serial for debugging and Serial1 for wireless commands
  Serial.begin(115200);  // Debug output
  Serial1.begin(9600);   // HC‑12 wireless commands

  // Debug startup messages
  Serial.println("MECHENG706_Base_Code_25/01/2018");
  delay(1000);
  Serial.println("Setup....");
  delay(1000);
}

void loop(void) {
  static STATE machine_state = INITIALISING;
  // Finite-state machine
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