#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <Servo.h>

#include "SharpIR.h"

// State machine states
enum STATE { INITIALISING, RUNNING, STOPPED };

// Motor control pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;

// IR sensor pins
const int FRONT_LEFT_IR = A4;
SharpIR FrontLeftIR(SharpIR::GP2Y0A41SK0F, FRONT_LEFT_IR);

const int FRONT_RIGHT_IR = A14;
SharpIR FrontRightIR(SharpIR::GP2Y0A41SK0F, FRONT_RIGHT_IR);

const int BACK_LEFT_IR = A6;
SharpIR BackLeftIR(SharpIR::GP2Y0A21YK0F, BACK_LEFT_IR);

const int BACK_RIGHT_IR = A15;
SharpIR BackRightIR(SharpIR::GP2Y0A21YK0F, BACK_RIGHT_IR);

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

extern int fl_change;
extern int fr_change;
extern int bl_change;
extern int br_change;

// Controller variables
extern double gyro_u;

// Serial port pointer
extern HardwareSerial *SerialCom;

#endif  // GLOBALS_H
