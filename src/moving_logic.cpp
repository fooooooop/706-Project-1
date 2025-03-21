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
    IR_controller(500);
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
  // left_front_motor.writeMicroseconds(1500 - speed_val);
  // left_rear_motor.writeMicroseconds(1500 - speed_val);
  // right_rear_motor.writeMicroseconds(1500 - speed_val);
  // right_front_motor.writeMicroseconds(1500 - speed_val);

  while ((Serial.read() != 'c') || (Serial1.read() != 'c')) {
    GYRO_controller(30);
    Serial.println(gyro_u);
    left_front_motor.writeMicroseconds(1500 + gyro_u);
    left_rear_motor.writeMicroseconds(1500 + gyro_u);
    right_rear_motor.writeMicroseconds(1500 + gyro_u);
    right_front_motor.writeMicroseconds(1500 + gyro_u);
  }
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
  // && ((double)BACK_LEFT_longIR_reading() > target_dist) && ((double)BACK_RIGHT_longIR_reading() > target_dist) ); // To also use the back sensors

  delay(10);
  left_front_motor.writeMicroseconds(0);
  left_rear_motor.writeMicroseconds(0);
  right_rear_motor.writeMicroseconds(0);
  right_front_motor.writeMicroseconds(0);
  delay(1000);

  // Re-orient itself straight along the wall (target_dist)--------------------------//
  if ((double)FRONT_LEFT_shortIR_reading() < target_dist) {
    Serial.println("Front LEFT");
    Serial1.println("Front LEFT");
    do {
      left_front_motor.writeMicroseconds(1500 + speed_val);
      left_rear_motor.writeMicroseconds(1500 + speed_val);
      right_rear_motor.writeMicroseconds(1500 + speed_val);
      right_front_motor.writeMicroseconds(1500 + speed_val);
    } while ((double)BACK_LEFT_longIR_reading() > target_dist);

    delay(10);
    left_front_motor.writeMicroseconds(0);
    left_rear_motor.writeMicroseconds(0);
    right_rear_motor.writeMicroseconds(0);
    right_front_motor.writeMicroseconds(0);
    delay(1000);

    while ((Serial.read() != 's') || (Serial1.read() != 's')) {
      IR_controller(135);
      left_front_motor.writeMicroseconds(1500 - speed_val - IR_u);
      left_rear_motor.writeMicroseconds(1500 + speed_val + IR_u);
      right_rear_motor.writeMicroseconds(1500 + speed_val + IR_u);
      right_front_motor.writeMicroseconds(1500 - speed_val - IR_u);
      // IR_u based on strafe_left function
    }

  } else if ((double)FRONT_RIGHT_shortIR_reading() < target_dist) {
    Serial.println("Front RIGHT");
    Serial1.println("Front RIGHT");
    do {
      left_front_motor.writeMicroseconds(1500 - speed_val);
      left_rear_motor.writeMicroseconds(1500 - speed_val);
      right_rear_motor.writeMicroseconds(1500 - speed_val);
      right_front_motor.writeMicroseconds(1500 - speed_val);
    } while ((double)BACK_RIGHT_longIR_reading( ) > target_dist);

    delay(10);
    left_front_motor.writeMicroseconds(0);
    left_rear_motor.writeMicroseconds(0);
    right_rear_motor.writeMicroseconds(0);
    right_front_motor.writeMicroseconds(0);
    delay(1000);

    while ((Serial.read() != 's') || (Serial1.read() != 's')) {
      IR_controller(135);
      left_front_motor.writeMicroseconds(1500 + speed_val + IR_u);
      left_rear_motor.writeMicroseconds(1500 - speed_val - IR_u);
      right_rear_motor.writeMicroseconds(1500 - speed_val - IR_u);
      right_front_motor.writeMicroseconds(1500 + speed_val + IR_u);
      // IR_u based on strafe_left function
    }
    
  }
  // } else if ((double)BACK_LEFT_longIR_reading() < target_dist) {
  //   Serial.println("Back LEFT");
  //   Serial1.println("Back LEFT");
  //   do {
  //     left_front_motor.writeMicroseconds(1500 + speed_val);
  //     left_rear_motor.writeMicroseconds(1500 + speed_val);
  //     right_rear_motor.writeMicroseconds(1500 + speed_val);
  //     right_front_motor.writeMicroseconds(1500 + speed_val);
  //   } while ((double)FRONT_LEFT_shortIR_reading() > target_dist);
    

  // } else if ((double)BACK_RIGHT_longIR_reading() < target_dist) {
  //   Serial.println("Back RIGHT");
  //   Serial1.println("Back RIGHT");
  //   do {
  //     left_front_motor.writeMicroseconds(1500 - speed_val);
  //     left_rear_motor.writeMicroseconds(1500 - speed_val);
  //     right_rear_motor.writeMicroseconds(1500 - speed_val);
  //     right_front_motor.writeMicroseconds(1500 - speed_val);
  //   } while ((double)FRONT_RIGHT_shortIR_reading() > target_dist);

  // }  

  // Take an angle reading. "Zero" the robot.
  // for (int i = 1; i < 50; i++){
  //   GYRO_reading(50); 
  //   delay(50);
  // }
  // currentAngle = 0;

  // Drive to shortest wall
}
