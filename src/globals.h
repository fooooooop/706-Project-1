#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <Servo.h>

#include "SharpDistSensor.h"
#include "SharpIR.h"

#define MEDIAN_WINDOW_FR 5
#define MEDIAN_WINDOW_FL 5
#define MEDIAN_WINDOW_BR 5
#define MEDIAN_WINDOW_BL 5
// State machine states
enum STATE { INITIALISING, RUNNING, STOPPED };

// Motor control pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;

// IR sensor pins
const int FRONT_LEFT_IR = A4;

const int FRONT_RIGHT_IR = A14;

const int BACK_LEFT_IR = A5;

const int BACK_RIGHT_IR = A15;

extern SharpDistSensor FrontLeftIR;
extern SharpDistSensor FrontRightIR;
extern SharpDistSensor BackLeftIR;
extern SharpDistSensor BackRightIR;

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
extern double IR_u;

// Serial port pointer
extern HardwareSerial *SerialCom;

#endif  // GLOBALS_H
