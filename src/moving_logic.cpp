#include "moving_logic.h"

#define FORWARD_BOUND 5
#define BACKWARD_BOUND 166
#define SAMPLING_TIME 125

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
  while (Serial1.read() != 'v') {
    GYRO_controller(0, 20.5, 0, 0);
    // dualPrintln(currentAngle);
    dualPrintln(IR_controller(405, AWD, RIGHT, 2.05, 0.02, 0.08));
    left_front_motor.writeMicroseconds(1500 + speed_val + gyro_u - IR_u);
    left_rear_motor.writeMicroseconds(1500 + speed_val + gyro_u + IR_u);
    right_rear_motor.writeMicroseconds(1500 - speed_val + gyro_u + IR_u);
    right_front_motor.writeMicroseconds(1500 - speed_val + gyro_u - IR_u);
    // IR_u based on strafe_left function
  }

  // Stop Motor ----//
  left_front_motor.writeMicroseconds(0);
  left_rear_motor.writeMicroseconds(0);
  right_rear_motor.writeMicroseconds(0);
  right_front_motor.writeMicroseconds(0);
}

void reverse() {
  while (Serial1.read() != 'v') {
    GYRO_controller(0, 20.5, 0, 0);
    dualPrintln(IR_controller(405, AWD, RIGHT, 2.05, 0.02, 0.08));
    left_front_motor.writeMicroseconds(1500 - speed_val + gyro_u - IR_u);
    left_rear_motor.writeMicroseconds(1500 - speed_val + gyro_u + IR_u);
    right_rear_motor.writeMicroseconds(1500 + speed_val + gyro_u + IR_u);
    right_front_motor.writeMicroseconds(1500 + speed_val + gyro_u - IR_u);
    // IR_u based on strafe_left function
  }

  // Stop Motor ----//
  left_front_motor.writeMicroseconds(0);
  left_rear_motor.writeMicroseconds(0);
  right_rear_motor.writeMicroseconds(0);
  right_front_motor.writeMicroseconds(0);
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
  // left_front_motor.writeMicroseconds(1500 - speed_val);
  // left_rear_motor.writeMicroseconds(1500 + speed_val);
  // right_rear_motor.writeMicroseconds(1500 + speed_val);
  // right_front_motor.writeMicroseconds(1500 - speed_val);
  
  // strafe_target(180, RIGHT);
  // dualPrintln("Strafe 6 done");
  // strafe_target(360, RIGHT);
  // dualPrintln("Strafe 5 done");
  strafe_target(200, RIGHT, SLOW);
  dualPrintln("Strafe 4 done");
  // strafe_target(600, LEFT);
  // dualPrintln("Strafe 3 done");
  // strafe_target(400, LEFT);
  // dualPrintln("Strafe 2 done");
  // strafe_target(200, LEFT);
  // dualPrintln("Strafe 1 done");
}

void strafe_right() {
  // left_front_motor.writeMicroseconds(1500 + speed_val);
  // left_rear_motor.writeMicroseconds(1500 - speed_val);
  // right_rear_motor.writeMicroseconds(1500 - speed_val);
  // right_front_motor.writeMicroseconds(1500 + speed_val);

  // strafe_target(200, LEFT);
  // dualPrintln("Strafe 1 done");
  // strafe_target(400, LEFT);
  // dualPrintln("Strafe 2 done");
  strafe_target(200, LEFT, SLOW);
  dualPrintln("Strafe 3 done");
  // strafe_target(500, RIGHT);
  // dualPrintln("Strafe 4 done");
  // strafe_target(360, RIGHT);
  // dualPrintln("Strafe 5 done");
  // strafe_target(180, RIGHT);
  // dualPrintln("Strafe 6 done");
  
}


//-------------------------------------------------//
//------------Custom Functions---------------------//
//-------------------------------------------------//


void turn_angle(double target) {
  bool gyro_exit = false;
  bool gyro_timestart = false;
  double gyro_timer = 0;
  double gyro_err_pos;
  double gyro_bounds = 9;

  while (gyro_exit == false) {
    gyro_err_pos = GYRO_controller(target, 4.75, 0, 0);

    // Send Power to Motors-----//
    left_front_motor.writeMicroseconds(1500 + gyro_u);
    left_rear_motor.writeMicroseconds(1500 + gyro_u);
    right_rear_motor.writeMicroseconds(1500 + gyro_u);
    right_front_motor.writeMicroseconds(1500 + gyro_u);

    // Exit Condition-----//
    if ((abs(gyro_err_pos) < gyro_bounds) && (gyro_timestart != true)) {
      // Checks to see if yss is within exit threshold, start timer
      gyro_timestart = true;
      gyro_timer = millis();
    }
    if ((abs(gyro_err_pos) > gyro_bounds) && (gyro_timestart == true)) {
      // Checks to see if yss falls outside of exit threshold
      // If it does, then restart timer
      gyro_timestart = false;
    } else if ((millis() - gyro_timer > 750.0) &&
               (abs(gyro_err_pos) < gyro_bounds) && (gyro_timestart == true)) {
      // Else, if yss is within threshold for a certain amount of time (check
      // first condition), exit controller
      gyro_exit = true;
    }
  }
}

void forward_target(double target_sidewall, double target, enum DIRECTION left_right, enum SPEED boostit) {
  boostit == FAST ? speed_val = 350 : speed_val = 165;

  do {
    GYRO_controller(0, 20.25, 0, 0);
    IR_controller(target_sidewall, AWD, left_right, 2.05, 0.01, 0.08);
    if (millis() - global_timesnap > SAMPLING_TIME) {
      k += 1; 
      global_timesnap = millis();
    }
    US_value[k] = HC_SR04_range();

    // Send Power to Motors-----//
    left_front_motor.writeMicroseconds(1500 + speed_val + gyro_u - IR_u);
    left_rear_motor.writeMicroseconds(1500 + speed_val + gyro_u + IR_u);
    right_rear_motor.writeMicroseconds(1500 - speed_val + gyro_u + IR_u);
    right_front_motor.writeMicroseconds(1500 - speed_val + gyro_u - IR_u);
    // IR_u based on strafe_left function

    // Exit Condition - when ultrasonic sensor reaches target//
  } while (US_value[k] > target);

  // Stop Motor ----//
  left_front_motor.writeMicroseconds(0);
  left_rear_motor.writeMicroseconds(0);
  right_rear_motor.writeMicroseconds(0);
  right_front_motor.writeMicroseconds(0);

  IR_u = 0;
  IR_err_mem = 0;
  IR_err_mem_back = 0;
  IR_err_mem_front = 0;
  IR_err_previous = 0;
}

void reverse_target(double target_sidewall, double target, enum DIRECTION left_right, enum SPEED boostit) {
  boostit == FAST ? speed_val = 350 : speed_val = 165;

  do {
    GYRO_controller(0, 20.25, 0, 0);
    IR_controller(target_sidewall, AWD, left_right, 2.05, 0.01, 0.08);
    if (millis() - global_timesnap > SAMPLING_TIME) {
      k += 1; 
      global_timesnap = millis();
    }
    US_value[k] = HC_SR04_range();
    

    // Send Power to Motors-----//
    left_front_motor.writeMicroseconds(1500 - speed_val + gyro_u - IR_u);
    left_rear_motor.writeMicroseconds(1500 - speed_val + gyro_u + IR_u);
    right_rear_motor.writeMicroseconds(1500 + speed_val + gyro_u + IR_u);
    right_front_motor.writeMicroseconds(1500 + speed_val + gyro_u - IR_u);
    // IR_u based on strafe_left function

    // Exit Condition - when ultrasonic sensor reaches target//
  } while (US_value[k] < target);

  // Stop Motor ----//
  left_front_motor.writeMicroseconds(0);
  left_rear_motor.writeMicroseconds(0);
  right_rear_motor.writeMicroseconds(0);
  right_front_motor.writeMicroseconds(0);

  IR_u = 0;
  IR_err_mem = 0;
  IR_err_mem_back = 0;
  IR_err_mem_front = 0;
  IR_err_previous = 0;
}

void strafe_target(double target, enum DIRECTION left_right, enum SPEED boostit) {
  bool strafe_exit = false;
  double strafe_timer = 0;
  bool strafe_timestart = false;
  double strafe_bounds = 20;
  double IR_err_pos = 0;
  double IR_err_Fpos = 0;
  double IR_err_Bpos = 0;
  double gyro_err_pos;
  double gyro_bounds = 10;

  if (boostit == SLOW){
    // Strafe to a target by "pushing" off a wall-----//
    while (strafe_exit == false) {
      // Start Strafing------------//
      gyro_err_pos = GYRO_controller(0, 6, 0, 0);
      IR_err_pos = IR_controller(target, AWD, left_right, 2.75, 0.0015, 0.0);
      // IR_err_Fpos = IR_controller(250, FWD, left_right, 3.0, 0.0015, 0);
      // IR_err_Bpos = IR_controller(250, RWD, left_right, 3.0, 0.0015, 0);  

      if (millis() - global_timesnap > SAMPLING_TIME) {
        US_value[k] = HC_SR04_range();
        k += 1; 
        global_timesnap = millis();
      }
      
      left_front_motor.writeMicroseconds(1500 - 100 + gyro_u - IR_u);
      left_rear_motor.writeMicroseconds(1500 + 100 + gyro_u + IR_u);
      right_rear_motor.writeMicroseconds(1500 + 100 + gyro_u + IR_u);
      right_front_motor.writeMicroseconds(1500 - 100 + gyro_u - IR_u);

      // Exit Condition-----//
      if (((abs(IR_err_Fpos) < strafe_bounds) &&
          (abs(gyro_err_pos) < gyro_bounds) &&
          (abs(IR_err_Bpos) < strafe_bounds)) &&
          (strafe_timestart != true)) {
        // Checks to see if yss is within exit threshold, start timer
        strafe_timestart = true;
        strafe_timer = millis();
      }
      if (((abs(IR_err_Fpos) > strafe_bounds) ||
          (abs(gyro_err_pos) > gyro_bounds) ||
          (abs(IR_err_Bpos) > strafe_bounds)) &&
          (strafe_timestart == true)) {
        // Checks to see if yss falls outside of exit threshold
        // If it does, then restart timer
        strafe_timestart = false;

      } else if ((millis() - strafe_timer > 2000.0) &&
                ((abs(IR_err_Fpos) < strafe_bounds) &&
                  (abs(gyro_err_pos) < gyro_bounds) &&
                  (abs(IR_err_Bpos) < strafe_bounds)) &&
                (strafe_timestart == true)) {
        // Else, if yss is within threshold for a certain amount of time (check
        // first condition), exit controller
        strafe_exit = true;
      }
    }

    // Stop Motor ----//
    left_front_motor.writeMicroseconds(0);
    left_rear_motor.writeMicroseconds(0);
    right_rear_motor.writeMicroseconds(0);
    right_front_motor.writeMicroseconds(0);

    IR_u = 0;
    IR_err_mem = 0;
    IR_err_mem_back = 0;
    IR_err_mem_front = 0;
    IR_err_previous = 0;

    return;
  } else if (boostit == FAST) {
    do {
      // Start Strafing------------//
      gyro_err_pos = GYRO_controller(0, 6, 0, 0);
      IR_err_pos = IR_controller(target, AWD, left_right, 2.75, 0.0, 0.0);

      if (millis() - global_timesnap > SAMPLING_TIME) {
        US_value[k] = HC_SR04_range();
        k += 1; 
        global_timesnap = millis();
      }

      left_front_motor.writeMicroseconds(1500 - 100 + gyro_u - IR_u);
      left_rear_motor.writeMicroseconds(1500 + 100 + gyro_u + IR_u);
      right_rear_motor.writeMicroseconds(1500 + 100 + gyro_u + IR_u);
      right_front_motor.writeMicroseconds(1500 - 100 + gyro_u - IR_u);

    } while (abs(IR_err_pos) > strafe_bounds);

    // Stop Motor ----//
    left_front_motor.writeMicroseconds(0);
    left_rear_motor.writeMicroseconds(0);
    right_rear_motor.writeMicroseconds(0);
    right_front_motor.writeMicroseconds(0);

    IR_u = 0;
    IR_err_mem = 0;
    IR_err_mem_back = 0;
    IR_err_mem_front = 0;
    IR_err_previous = 0;

    return;
  }
}

void forward_right() {
  for (int i = 0; i < 5; i++){
    FRONT_LEFT_shortIR_reading();
    FRONT_RIGHT_shortIR_reading();
    BACK_LEFT_longIR_reading();
    BACK_RIGHT_longIR_reading();
  }
  strafe_target(135, LEFT, SLOW);
  forward_target(135, FORWARD_BOUND, LEFT, SLOW);
  for (int i = 0; i < 5; i++){
    FRONT_LEFT_shortIR_reading();
    FRONT_RIGHT_shortIR_reading();
    BACK_LEFT_longIR_reading();
    BACK_RIGHT_longIR_reading();
  }
  strafe_target(260, LEFT, FAST);
  reverse_target(260, BACKWARD_BOUND, LEFT, FAST);
  for (int i = 0; i < 5; i++){
    FRONT_LEFT_shortIR_reading();
    FRONT_RIGHT_shortIR_reading();
    BACK_LEFT_longIR_reading();
    BACK_RIGHT_longIR_reading();
  }
  strafe_target(460, LEFT, FAST);
  forward_target(460, FORWARD_BOUND, LEFT, FAST);
  for (int i = 0; i < 5; i++){
    FRONT_LEFT_shortIR_reading();
    FRONT_RIGHT_shortIR_reading();
    BACK_LEFT_longIR_reading();
    BACK_RIGHT_longIR_reading();
  }
  strafe_target(670, LEFT, FAST);
  reverse_target(670, BACKWARD_BOUND, LEFT, FAST);

  for (int i = 0; i < 5; i++){
    FRONT_LEFT_shortIR_reading();
    FRONT_RIGHT_shortIR_reading();
    BACK_LEFT_longIR_reading();
    BACK_RIGHT_longIR_reading();
  }
  strafe_target(530, RIGHT, FAST);
  forward_target(530, FORWARD_BOUND, RIGHT, FAST);
  for (int i = 0; i < 5; i++){
    FRONT_LEFT_shortIR_reading();
    FRONT_RIGHT_shortIR_reading();
    BACK_LEFT_longIR_reading();
    BACK_RIGHT_longIR_reading();
  }
  strafe_target(390, RIGHT, FAST);
  reverse_target(390, BACKWARD_BOUND, RIGHT, FAST);
  for (int i = 0; i < 5; i++){
    FRONT_LEFT_shortIR_reading();
    FRONT_RIGHT_shortIR_reading();
    BACK_LEFT_longIR_reading();
    BACK_RIGHT_longIR_reading();
  }
  strafe_target(200, RIGHT, FAST);
  forward_target(200, FORWARD_BOUND, RIGHT, FAST);
  for (int i = 0; i < 5; i++){
    FRONT_LEFT_shortIR_reading();
    FRONT_RIGHT_shortIR_reading();
    BACK_LEFT_longIR_reading();
    BACK_RIGHT_longIR_reading();
  }
  strafe_target(90, RIGHT, SLOW);
  reverse_target(90, BACKWARD_BOUND, RIGHT, SLOW);
}

void forward_left() {
  for (int i = 0; i < 5; i++){
    FRONT_LEFT_shortIR_reading();
    FRONT_RIGHT_shortIR_reading();
    BACK_LEFT_longIR_reading();
    BACK_RIGHT_longIR_reading();
  }
  strafe_target(90, RIGHT, SLOW);
  forward_target(90, FORWARD_BOUND, RIGHT, SLOW);
  for (int i = 0; i < 5; i++){
    FRONT_LEFT_shortIR_reading();
    FRONT_RIGHT_shortIR_reading();
    BACK_LEFT_longIR_reading();
    BACK_RIGHT_longIR_reading();
  }
  strafe_target(200, RIGHT, FAST);
  reverse_target(200, BACKWARD_BOUND, RIGHT, FAST);
  for (int i = 0; i < 5; i++){
    FRONT_LEFT_shortIR_reading();
    FRONT_RIGHT_shortIR_reading();
    BACK_LEFT_longIR_reading();
    BACK_RIGHT_longIR_reading();
  }
  strafe_target(390, RIGHT, FAST);
  forward_target(390, FORWARD_BOUND, RIGHT, FAST);
  for (int i = 0; i < 5; i++){
    FRONT_LEFT_shortIR_reading();
    FRONT_RIGHT_shortIR_reading();
    BACK_LEFT_longIR_reading();
    BACK_RIGHT_longIR_reading();
  }
  strafe_target(530, RIGHT, FAST);
  reverse_target(530, BACKWARD_BOUND, RIGHT, FAST);

  for (int i = 0; i < 5; i++){
    FRONT_LEFT_shortIR_reading();
    FRONT_RIGHT_shortIR_reading();
    BACK_LEFT_longIR_reading();
    BACK_RIGHT_longIR_reading();
  }
  strafe_target(670, LEFT, FAST);
  forward_target(670, FORWARD_BOUND, LEFT, FAST);
  for (int i = 0; i < 5; i++){
    FRONT_LEFT_shortIR_reading();
    FRONT_RIGHT_shortIR_reading();
    BACK_LEFT_longIR_reading();
    BACK_RIGHT_longIR_reading();
  }
  strafe_target(460, LEFT, FAST);
  reverse_target(460, BACKWARD_BOUND, LEFT, FAST);
  for (int i = 0; i < 5; i++){
    FRONT_LEFT_shortIR_reading();
    FRONT_RIGHT_shortIR_reading();
    BACK_LEFT_longIR_reading();
    BACK_RIGHT_longIR_reading();
  }
  strafe_target(260, LEFT, FAST);
  forward_target(260, FORWARD_BOUND, LEFT, FAST);
  for (int i = 0; i < 5; i++){
    FRONT_LEFT_shortIR_reading();
    FRONT_RIGHT_shortIR_reading();
    BACK_LEFT_longIR_reading();
    BACK_RIGHT_longIR_reading();
  }
  strafe_target(135, LEFT, SLOW);
  reverse_target(135, BACKWARD_BOUND, LEFT, SLOW);
}

void find_corner() {
  bool strafe_exit = false;
  double strafe_timer = 0;
  bool strafe_timestart = false;
  double strafe_bounds = 30;
  double IR_err_Fpos;
  double IR_err_Bpos;

  // Strafe left and orient onto wall-----//
  while (strafe_exit == false) {
    // Start Strafing------------//
    IR_err_Fpos = IR_controller(200, FWD, LEFT, 3.0, 0.0015, 0);
    IR_err_Bpos = IR_controller(200, RWD, LEFT, 3.0, 0.0015, 0);
    left_front_motor.writeMicroseconds(1500 - 100 - IRFront_u);
    left_rear_motor.writeMicroseconds(1500 + 100 + IRBack_u);
    right_rear_motor.writeMicroseconds(1500 + 100 + IRBack_u);
    right_front_motor.writeMicroseconds(1500 - 100 - IRFront_u);

    // Exit Condition-----//
    if (((abs(IR_err_Fpos) < strafe_bounds) &&
         (abs(IR_err_Bpos) < strafe_bounds)) &&
        (strafe_timestart != true)) {
      // Checks to see if yss is within exit threshold, start timer
      strafe_timestart = true;
      strafe_timer = millis();
    }
    if (((abs(IR_err_Fpos) > strafe_bounds) ||
         (abs(IR_err_Bpos) > strafe_bounds)) &&
        (strafe_timestart == true)) {
      // Checks to see if yss falls outside of exit threshold
      // If it does, then restart timer
      strafe_timestart = false;

    } else if ((millis() - strafe_timer > 1500.0) &&
               ((abs(IR_err_Fpos) < strafe_bounds) &&
                (abs(IR_err_Bpos) < strafe_bounds)) &&
               (strafe_timestart == true)) {
      // Else, if yss is within threshold for a certain amount of time (check
      // first condition), exit controller
      strafe_exit = true;

      IRFront_u = 0;
      IRBack_u = 0; 
    }
  }

  // Quick Stop//
  delay(10);
  left_front_motor.writeMicroseconds(0);
  left_rear_motor.writeMicroseconds(0);
  right_rear_motor.writeMicroseconds(0);
  right_front_motor.writeMicroseconds(0);
  delay(500);
  //----------//

  // Take an angle reading and "zero" the robot---//
  // Set Gyro zero voltage
  int i;
  float sum = 0;
  int sensorValue = 0;
  for (i = 0; i < 100; i++)  // read 100 values of voltage when gyro is at
                             // still, to calculate the zero-drift
  {
    sensorValue = analogRead(A3);
    sum += sensorValue;
    delay(5);
  }
  gyroZeroVoltage = sum / 100;  // average the sum as the zero drifting
  for (int i = 1; i < 10; i++) {
    GYRO_reading(100);
  }
  currentAngle = 0;

  // Drive straight to shortest wall----------//
  do {
    GYRO_controller(0, 20.25, 0, 0);
    IR_controller(130, AWD, LEFT, 1.0, 0, 0);
    left_front_motor.writeMicroseconds(1500 + speed_val + gyro_u - IR_u);
    left_rear_motor.writeMicroseconds(1500 + speed_val + gyro_u + IR_u);
    right_rear_motor.writeMicroseconds(1500 - speed_val + gyro_u + IR_u);
    right_front_motor.writeMicroseconds(1500 - speed_val + gyro_u - IR_u);
    // IR_u based on strafe_left function

  } while (HC_SR04_range() > 12);

  // Quick Stop//
  delay(10);
  left_front_motor.writeMicroseconds(0);
  left_rear_motor.writeMicroseconds(0);
  right_rear_motor.writeMicroseconds(0);
  right_front_motor.writeMicroseconds(0);
  delay(500);
  //----------//

  // Find Long Wall //
  turn_angle(90);
  float first_reading = HC_SR04_range();
  turn_angle(179.5);
  float second_reading = HC_SR04_range();

  
  // Align along long wall and zero robot //
  if (first_reading > second_reading) {
    turn_angle(90);

    // Stop Motor ----//
    left_front_motor.writeMicroseconds(0);
    left_rear_motor.writeMicroseconds(0);
    right_rear_motor.writeMicroseconds(0);
    right_front_motor.writeMicroseconds(0);

    // Reset Things
    currentAngle = 0;
    IR_u = 0;
    IR_err_mem = 0;
    IR_err_mem_back = 0;
    IR_err_mem_front = 0;
    IR_err_previous = 0;

    forward_right();  // Start Tilling
  } else {


    // Stop Motor ----//
    left_front_motor.writeMicroseconds(0);
    left_rear_motor.writeMicroseconds(0);
    right_rear_motor.writeMicroseconds(0);
    right_front_motor.writeMicroseconds(0);

    // Reset things
    currentAngle = 0;
    IR_u = 0; //Idk, it's to not make the IR controller crazy.
    IR_err_mem = 0;
    IR_err_mem_back = 0;
    IR_err_mem_front = 0;
    IR_err_previous = 0;

    forward_left();  // Start Tilling
  }

  return;
}
