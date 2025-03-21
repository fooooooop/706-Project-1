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
  while ((Serial.read() != 's') || (Serial1.read() != 's')) {
    GYRO_controller(0);
    IR_controller(500, 1);
    left_front_motor.writeMicroseconds(1500 + speed_val + fl_change + gyro_u - IR_u);
    left_rear_motor.writeMicroseconds(1500 + speed_val + bl_change + gyro_u + IR_u);
    right_rear_motor.writeMicroseconds(1500 - speed_val + br_change + gyro_u + IR_u);
    right_front_motor.writeMicroseconds(1500 - speed_val + fr_change + gyro_u - IR_u);
    // IR_u based on strafe_left function
  }
  left_front_motor.writeMicroseconds(1500 + speed_val + fl_change + gyro_u - IR_u);
  left_rear_motor.writeMicroseconds(1500 + speed_val + bl_change + gyro_u + IR_u);
  right_rear_motor.writeMicroseconds(1500 - speed_val + br_change + gyro_u + IR_u);
  right_front_motor.writeMicroseconds(1500 - speed_val + fr_change + gyro_u - IR_u);
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
  while ((Serial.read() != 'c') || (Serial1.read() != 'c')) {
    GYRO_controller(target);
    Serial.println(gyro_u);
    left_front_motor.writeMicroseconds(1500 + gyro_u);
    left_rear_motor.writeMicroseconds(1500 + gyro_u);
    right_rear_motor.writeMicroseconds(1500 + gyro_u);
    right_front_motor.writeMicroseconds(1500 + gyro_u);
  }
}

void find_corner() {
  double target_dist = 140;

  // until one of the IR sensors is less than target_dist, drive Forward--------------------------//
  do {
    GYRO_controller(0);
    left_front_motor.writeMicroseconds(1500 + speed_val + fl_change + gyro_u);
    left_rear_motor.writeMicroseconds(1500 + speed_val + bl_change + gyro_u);
    right_rear_motor.writeMicroseconds(1500 - speed_val + br_change + gyro_u);
    right_front_motor.writeMicroseconds(1500 - speed_val + fr_change + gyro_u);
  } while ( ((double)FRONT_LEFT_shortIR_reading() > target_dist) && ((double)FRONT_RIGHT_shortIR_reading() > target_dist) );

  //Quick Stop//
  delay(10);
  left_front_motor.writeMicroseconds(0);
  left_rear_motor.writeMicroseconds(0);
  right_rear_motor.writeMicroseconds(0);
  right_front_motor.writeMicroseconds(0);
  delay(1000);
  //----------//

  // Re-orient itself straight/parallel along the wall (target_dist)--------------------------//
  if ((double)FRONT_LEFT_shortIR_reading() < target_dist) {
    Serial.println("Front LEFT");
    Serial1.println("Front LEFT");
    //Move Back Wheels
    do {
      left_front_motor.writeMicroseconds(1500 + speed_val);
      left_rear_motor.writeMicroseconds(1500 + speed_val);
      right_rear_motor.writeMicroseconds(1500 + speed_val);
      right_front_motor.writeMicroseconds(1500 + speed_val);
    } while ((double)BACK_LEFT_longIR_reading() > target_dist);

    //Quick Stop//
    delay(10);
    left_front_motor.writeMicroseconds(0);
    left_rear_motor.writeMicroseconds(0);
    right_rear_motor.writeMicroseconds(0);
    right_front_motor.writeMicroseconds(0);
    delay(1000);
    //----------//

    // Strafe left and orient onto wall
    while ((Serial.read() != 's') || (Serial1.read() != 's')) {
      IR_controller(110, 2);
      IR_controller(110, 3);
      left_front_motor.writeMicroseconds(1500 - speed_val - IRFront_u);
      left_rear_motor.writeMicroseconds(1500 + speed_val + IRBack_u);
      right_rear_motor.writeMicroseconds(1500 + speed_val + IRBack_u);
      right_front_motor.writeMicroseconds(1500 - speed_val - IRFront_u);
    }

  } else if ((double)FRONT_RIGHT_shortIR_reading() < target_dist) {
    Serial.println("Front RIGHT");
    Serial1.println("Front RIGHT");
    //Move Back Wheels
    do {
      left_front_motor.writeMicroseconds(1500 - speed_val);
      left_rear_motor.writeMicroseconds(1500 - speed_val);
      right_rear_motor.writeMicroseconds(1500 - speed_val);
      right_front_motor.writeMicroseconds(1500 - speed_val);
    } while ((double)BACK_RIGHT_longIR_reading( ) > target_dist);

    //Quick Stop//
    delay(10);
    left_front_motor.writeMicroseconds(0);
    left_rear_motor.writeMicroseconds(0);
    right_rear_motor.writeMicroseconds(0);
    right_front_motor.writeMicroseconds(0);
    delay(1000);
    //----------//

    // Strafe right and orient onto wall
    while ((Serial.read() != 's') || (Serial1.read() != 's')) {
      IR_controller(110, 2);
      IR_controller(110, 3);
      left_front_motor.writeMicroseconds(1500 + speed_val + IRFront_u);
      left_rear_motor.writeMicroseconds(1500 - speed_val - IRBack_u);
      right_rear_motor.writeMicroseconds(1500 - speed_val - IRBack_u);
      right_front_motor.writeMicroseconds(1500 + speed_val + IRFront_u);
    }
    
  }

  //Quick Stop//
  delay(10);
  left_front_motor.writeMicroseconds(0);
  left_rear_motor.writeMicroseconds(0);
  right_rear_motor.writeMicroseconds(0);
  right_front_motor.writeMicroseconds(0);
  delay(1000);
  //----------//

  // Take an angle reading and "zero" the robot---//
  for (int i = 1; i < 50; i++){
    GYRO_reading(50); 
    delay(10);
  }
  currentAngle = 0;

  // Drive straight to shortest wall----------//
  do {
    GYRO_controller(0);
    IR_controller(135, 1);
    left_front_motor.writeMicroseconds(1500 + speed_val + fl_change + gyro_u - IR_u);
    left_rear_motor.writeMicroseconds(1500 + speed_val + bl_change + gyro_u + IR_u);
    right_rear_motor.writeMicroseconds(1500 - speed_val + br_change + gyro_u + IR_u);
    right_front_motor.writeMicroseconds(1500 - speed_val + fr_change + gyro_u - IR_u);
    // IR_u based on strafe_left function
  } while ( HC_SR04_range() > 12 );

  //Reset your coordinate system here//
}
