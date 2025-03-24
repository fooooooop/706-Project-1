#include "utilities.h"

void fast_flash_double_LED_builtin() {
  static byte indexer = 0;
  static unsigned long fast_flash_millis;
  if (millis() > fast_flash_millis) {
    indexer++;
    if (indexer > 4) {
      fast_flash_millis = millis() + 700;
      digitalWrite(LED_BUILTIN, LOW);
      indexer = 0;
    } else {
      fast_flash_millis = millis() + 100;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
}

void slow_flash_LED_builtin() {
  static unsigned long slow_flash_millis;
  if (millis() - slow_flash_millis > 2000) {
    slow_flash_millis = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void speed_change_smooth() {
  speed_val += speed_change;
  if (speed_val > 500) speed_val = 500;
  delay(50);
  speed_change = 0;
}

double GYRO_controller(double gyro_target, double kp, double ki, double kd) {
  //Setup!------------------------//

  // Time variables
  double t_current = 0;
  double t_previous = 0;
  int gyro_t = 50;

  // General error variables
  double gyro_currentSensor;
  double gyro_err_current;
  double gyro_err_previous = 0;

  // For Derivative controller
  double dt;
  double de;
  double dedt;

  // For Integral controller
  double gyro_err_mem = 0;

  //Main Code Start!------------------------//
  // To get dt for Integral and Derivative controllers
  t_current = millis();
  dt = (t_current - t_previous) / 1000;
  t_previous = t_current;

  // Gyro reading
  GYRO_reading(gyro_t); 
  gyro_currentSensor = currentAngle;

  // Proportional controller
  gyro_err_current = gyro_target - gyro_currentSensor;

  // Integral controller
  if (gyro_u < 400) { // Anti-integral windup
    gyro_err_mem += gyro_err_current;
  }
  
  // Derivative controller
  de = (gyro_err_current - gyro_err_previous);
  gyro_err_previous = gyro_err_current;

  dedt = (de / dt);

  // PID controller
  gyro_u = kp*gyro_err_current + ki+gyro_err_mem + kd*dedt;

  return gyro_err_current;
}

double IR_controller(double IR_target, enum DRIVE IR_mode, enum DIRECTION left_right, double kp, double ki, double kd) {
  //Setup!------------------------//
  //IR_mode changes what efforts are given to the motor
  //This is needed for the corner finding function so that the front and back wheels have
  //their own controllers

  //left_right states which side IR sensor to use

  dualPrintln(IR_u);
  dualPrintln(IRFront_u);
  dualPrintln(IRBack_u);
  dualPrint("IR Sensor Front Right Short: ");
  dualPrintln(FRONT_RIGHT_shortIR_reading());
  dualPrint("IR Sensor Front Left Short: ");
  dualPrintln(FRONT_LEFT_shortIR_reading());
  dualPrint("IR Sensor Back Left Long: ");
  dualPrintln(BACK_LEFT_longIR_reading());
  dualPrint("IR Sensor Back Right  Long: ");
  dualPrintln(BACK_RIGHT_longIR_reading());

  // Time variables
  double t_current = 0;
  double t_previous = 0;

  // General error variables
  double IR_currentSensor;
  double IR_err_current;
  double IR_err_previous = 0;

  // For Derivative controller
  double dt;
  double de;
  double dedt;

  // For Integral controller
  double IR_err_mem = 0;

  //Main Code Start!------------------------//
  // To get dt for Integral and Derivative controllers
  t_current = millis();
  dt = (t_current - t_previous) / 1000;
  t_previous = t_current;

  // IR reading + Proportional Controller
  if (IR_mode == FWD) {

    // Uses only Front IR Sensors //
    if (left_right == LEFT) {
      // Less than 13cm from LEFT wall
      IR_currentSensor = (double)FRONT_LEFT_shortIR_reading();
      IR_err_current = (IR_target - IR_currentSensor) * -1; 
  
    } else if (left_right == RIGHT) {
      // Less than 13cm from RIGHT wall
      IR_currentSensor = (double)FRONT_RIGHT_shortIR_reading();
      IR_err_current = IR_target - IR_currentSensor; 
  
    }
    //---------------------------//

  } else if (IR_mode == RWD) {

    // Uses only the Back IR Sensors //
    if (left_right == LEFT){
      // More than 32cm from LEFT wall
      IR_currentSensor = (double)BACK_LEFT_longIR_reading();
      IR_err_current = (IR_target - IR_currentSensor) * -1; 
  
    } else if (left_right == RIGHT){
      // More than 32cm from RIGHT wall
      IR_currentSensor = (double)BACK_RIGHT_longIR_reading();
      IR_err_current = (IR_target - IR_currentSensor); 

    } 
    //------------------------------//

  } else if (IR_mode == AWD) {

    // Will use all sensors
    // USE FOR THE STRAIGHT LINE
    // Honestly we could do two separate controllers for the front and back wheels and see how that goes?
    // But that's highkey kinda hard, and also, the short IR sensors will be useless in the middle anyways
    if (IR_target < 300) {
      if ((double)FRONT_LEFT_shortIR_reading() < 320){
        IR_currentSensor = (double)FRONT_LEFT_shortIR_reading();
        IR_err_current = (IR_target - IR_currentSensor) * -1; 
    
      } else if ((double)FRONT_RIGHT_shortIR_reading() < 320){
        IR_currentSensor = (double)FRONT_RIGHT_shortIR_reading();
        IR_err_current = IR_target - IR_currentSensor; 
    
      }
    } else {
      if (left_right == LEFT){
        IR_currentSensor = (double)BACK_LEFT_longIR_reading();
        IR_err_current = (IR_target - IR_currentSensor) * -1; 
    
      } else if (left_right == RIGHT){
        IR_currentSensor = (double)BACK_RIGHT_longIR_reading();
        IR_err_current = (IR_target - IR_currentSensor); 
      }
    }
    //------------------------------//

  }

  // Integral controller
  if (IR_u < 100)  { // Anti-integral windup
    IR_err_mem += IR_err_current;
  }
  
  // Derivative controller
  de = (IR_err_current - IR_err_previous);
  IR_err_previous = IR_err_current;

  dedt = (de / dt);

  // PID controller
  if (IR_mode == FWD) {
    // Add clamps
    ((kp*IR_err_current + ki+IR_err_mem + kd*dedt) > 500) ? IRFront_u = 500 : IRFront_u = (kp*IR_err_current + ki+IR_err_mem + kd*dedt);
  } else if (IR_mode == RWD) {
    // Add clamps
    ((kp*IR_err_current + ki+IR_err_mem + kd*dedt) > 500) ? IRBack_u = 500 : IRBack_u = (kp*IR_err_current + ki+IR_err_mem + kd*dedt) ;
  } else {
    IR_u = (kp*IR_err_current + ki+IR_err_mem + kd*dedt);
  }

  return IR_err_current;
}