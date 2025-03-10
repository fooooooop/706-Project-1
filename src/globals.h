#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <Servo.h>

// State machine states
enum STATE { INITIALISING, RUNNING, STOPPED };

// Motor control pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;

// Ultrasonic sensor pins and parameters
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;
const unsigned int MAX_DIST = 23200;

// Declare servo objects
extern Servo left_front_motor;
extern Servo left_rear_motor;
extern Servo right_rear_motor;
extern Servo right_front_motor;
extern Servo turret_motor;

// Global variables for speed control and turret position
extern int speed_val;
extern int speed_change;
extern int pos;

// Serial port pointer
extern HardwareSerial *SerialCom;

#endif  // GLOBALS_H
