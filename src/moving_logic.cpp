#include "moving_logic.h"

void enable_motors() {
  left_front_motor.attach(left_front);
  left_rear_motor.attach(left_rear);
  right_rear_motor.attach(right_rear);
  right_front_motor.attach(right_front);
}

void disable_motors() {
  left_front_motor.detach();
  left_rear_motor.detach();
  right_rear_motor.detach();
  right_front_motor.detach();

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void stop_motors() {
  left_front_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_front_motor.writeMicroseconds(1500);
}

void forward() {
  while ((Serial.read() != 'c') || (Serial1.read() != 'c')) {
    GYRO_controller(0);
    IR_controller(500, 1);
    left_front_motor.writeMicroseconds(1500 + speed_val + fl_change + gyro_u -
                                       IR_u);
    left_rear_motor.writeMicroseconds(1500 + speed_val + bl_change + gyro_u +
                                      IR_u);
    right_rear_motor.writeMicroseconds(1500 - speed_val + br_change + gyro_u +
                                       IR_u);
    right_front_motor.writeMicroseconds(1500 - speed_val + fr_change + gyro_u -
                                        IR_u);
    // IR_u based on strafe_left function
  }

  left_front_motor.writeMicroseconds(0);
  left_rear_motor.writeMicroseconds(0);
  right_rear_motor.writeMicroseconds(0);
  right_front_motor.writeMicroseconds(0);
}

void reverse() {
  left_front_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val - 100);
  right_rear_motor.writeMicroseconds(1500 + speed_val + 20);
  right_front_motor.writeMicroseconds(1500 + speed_val);
}

void ccw() {
  left_front_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_front_motor.writeMicroseconds(1500 - speed_val);
}

void cw() {
  left_front_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_front_motor.writeMicroseconds(1500 + speed_val);
}

void strafe_left() {
  left_front_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_front_motor.writeMicroseconds(1500 - speed_val);
}

void strafe_right() {
  left_front_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_front_motor.writeMicroseconds(1500 + speed_val);
}

void turn_angle(double target) {
  bool gyro_exit = false;
  bool gyro_timestart = false;
  double gyro_timer = 0;
  double gyro_err_pos;
  double gyro_bounds = 10;

  while (gyro_exit == false) {
    gyro_err_pos = GYRO_controller(target);
    if (gyro_u > 800) gyro_u = 800;  // Clamp
    Serial.println(gyro_u);
    left_front_motor.writeMicroseconds(1500 + gyro_u);
    left_rear_motor.writeMicroseconds(1500 + gyro_u);
    right_rear_motor.writeMicroseconds(1500 + gyro_u);
    right_front_motor.writeMicroseconds(1500 + gyro_u);

    // Exit Condition-----//
    if ((abs(gyro_err_pos) < gyro_bounds) && (gyro_timestart != true)) {
      // Checks to see if yss is within exit threshold
      gyro_timestart = true;
      gyro_timer = millis();
    }
    if ((abs(gyro_err_pos) > gyro_bounds) && (gyro_timestart == true)) {
      // Checks to see if yss falls outside of exit threshold
      // If it does, then restart timer
      gyro_timestart = false;
    } else if ((millis() - gyro_timer > 3000.0) &&
               (abs(gyro_err_pos) < gyro_bounds) && (gyro_timestart == true)) {
      // Else, if yss is within threshold for a certain amount of time (check
      // first condition), exit controller
      gyro_exit = true;
    }
  }
}

void forward_target(double target_wall) {
  do {
    GYRO_controller(0);
    IR_controller(target_wall, 1);
    left_front_motor.writeMicroseconds(1500 + speed_val + fl_change + gyro_u -
                                       IR_u);
    left_rear_motor.writeMicroseconds(1500 + speed_val + bl_change + gyro_u +
                                      IR_u);
    right_rear_motor.writeMicroseconds(1500 - speed_val + br_change + gyro_u +
                                       IR_u);
    right_front_motor.writeMicroseconds(1500 - speed_val + fr_change + gyro_u -
                                        IR_u);
    // IR_u based on strafe_left function

    // Exit Condition-----//
    // Make a controller for the ultrasonic sensor?
  } while (HC_SR04_range() > 12);

  left_front_motor.writeMicroseconds(0);
  left_rear_motor.writeMicroseconds(0);
  right_rear_motor.writeMicroseconds(0);
  right_front_motor.writeMicroseconds(0);
}

void find_corner() {
  bool strafe_exit = false;
  double strafe_timer = 0;
  bool strafe_timestart = false;
  double strafe_bounds = 60;
  double IR_err_Fpos;
  double IR_err_Bpos;

  // Strafe left and orient onto wall-----//
  while (strafe_exit == false) {
    // Start Strafing------------//
    IR_err_Fpos = IR_controller(110, 2);
    IR_err_Bpos = IR_controller(110, 3);
    left_front_motor.writeMicroseconds(1500 - speed_val - IRFront_u);
    left_rear_motor.writeMicroseconds(1500 + speed_val + IRBack_u);
    right_rear_motor.writeMicroseconds(1500 + speed_val + IRBack_u);
    right_front_motor.writeMicroseconds(1500 - speed_val - IRFront_u);

    // Exit Condition-----//
    if (((abs(IR_err_Fpos) < strafe_bounds) &&
         (abs(IR_err_Bpos) < strafe_bounds)) &&
        (strafe_timestart != true)) {
      // Checks to see if yss is within exit threshold
      strafe_timestart = true;
      strafe_timer = millis();
    }
    if (((abs(IR_err_Fpos) > strafe_bounds) &&
         (abs(IR_err_Bpos) > strafe_bounds)) &&
        (strafe_timestart == true)) {
      // Checks to see if yss falls outside of exit threshold
      // If it does, then restart timer
      strafe_timestart = false;
    } else if ((millis() - strafe_timer > 3000.0) &&
               ((abs(IR_err_Fpos) < strafe_bounds) &&
                (abs(IR_err_Bpos) < strafe_bounds)) &&
               (strafe_timestart == true)) {
      // Else, if yss is within threshold for a certain amount of time (check
      // first condition), exit controller
      strafe_exit = true;
    }
  }

  // Quick Stop//
  delay(10);
  left_front_motor.writeMicroseconds(0);
  left_rear_motor.writeMicroseconds(0);
  right_rear_motor.writeMicroseconds(0);
  right_front_motor.writeMicroseconds(0);
  delay(1000);
  //----------//

  // Take an angle reading and "zero" the robot---//
  for (int i = 1; i < 50; i++) {
    GYRO_reading(50);
    delay(10);
  }
  currentAngle = 0;

  // Drive straight to shortest wall----------//
  do {
    GYRO_controller(0);
    IR_controller(135, 1);
    left_front_motor.writeMicroseconds(1500 + speed_val + fl_change + gyro_u -
                                       IR_u);
    left_rear_motor.writeMicroseconds(1500 + speed_val + bl_change + gyro_u +
                                      IR_u);
    right_rear_motor.writeMicroseconds(1500 - speed_val + br_change + gyro_u +
                                       IR_u);
    right_front_motor.writeMicroseconds(1500 - speed_val + fr_change + gyro_u -
                                        IR_u);
    // IR_u based on strafe_left function
  } while (HC_SR04_range() > 12);

  // Quick Stop//
  delay(10);
  left_front_motor.writeMicroseconds(0);
  left_rear_motor.writeMicroseconds(0);
  right_rear_motor.writeMicroseconds(0);
  right_front_motor.writeMicroseconds(0);
  delay(1000);
  //----------//

  // Take an angle reading and "zero" the robot---//
  for (int i = 1; i < 50; i++) {
    GYRO_reading(50);
    delay(10);
  }
  currentAngle = 0;

  turn_angle(90);
  currentAngle = 0;
  turn_angle(90);
}
