#include "serial_command.h"

#include "moving_logic.h"

// --- Wireless Command Parsing ---
// This function now listens on Serial1 (the HC‑12)
void read_serial_command() {
  if (Serial.available()) {
    char val = Serial.read();
    Serial.print("Received command: ");
    Serial.println(val);
    Serial.print("Speed: ");
    Serial.print(speed_val);
    Serial.println(" ms");

    switch (val) {
      case 'w':  // Move Forward
      case 'W':
        forward();
        Serial.println("Forward executed");
        Serial1.println("Forward executed");
        break;
      case 's':  // Move Backward
      case 'S':
        reverse();
        Serial.println("Reverse executed");
        Serial1.println("Reverse executed");
        break;
      case 'q':  // Strafe Left
      case 'Q':
        strafe_left();
        Serial.println("Strafe Left executed");
        Serial1.println("Strafe Left executed");
        break;
      case 'e':  // Strafe Right
      case 'E':
        strafe_right();
        Serial.println("Strafe Right executed");
        Serial1.println("Strafe Right executed");
        break;
      case 'a':  // Rotate Counter-Clockwise
      case 'A':
        ccw();
        Serial.println("Rotate CCW executed");
        Serial1.println("Rotate CCW executed");
        break;
      case 'd':  // Rotate Clockwise
      case 'D':
        cw();
        Serial.println("Rotate CW executed");
        Serial1.println("Rotate CW executed");
        break;
      case '-':  // Decrease Speed
      case '_':
        speed_change = -100;
        Serial.println("Speed decreased");
        Serial1.println("Speed decreased");
        break;
      case '=':
      case '+':  // Increase Speed
        speed_change = 100;
        Serial.println("Speed increased");
        Serial1.println("Speed increased");
        break;

      case 'y':
      case 'Y': //FL UP
        fl_change += 10;
        Serial.print("Front Left Increased: ");
        Serial1.print("Front Left Increased: ");
        Serial.println(fl_change);
        Serial1.println(fl_change);
        break;

      case 'h':
      case 'H': //FL DOWN
        fl_change -= 10;
        Serial.print("Front Left Decreased: ");
        Serial1.print("Front Left Decreased: ");
        Serial.println(fl_change);
        Serial1.println(fl_change);
        break;

      case 'u':
      case 'U': //FR UP
        fr_change += 10;  
        Serial.print("Front Right Increased: ");
        Serial1.print("Front Right Increased: ");
        Serial.println(fr_change);
        Serial1.println(fr_change);
        break;

      case 'j':
      case 'J': //FR DOWN
        fr_change -= 10;
        Serial.print("Front Right Decreased: ");
        Serial1.print("Front Right Decreased: ");
        Serial.println(fr_change);
        Serial1.println(fr_change);
        break;

      case 'i':
      case 'I': //BL UP
        bl_change += 10;
        Serial.print("Back Left Increased: ");
        Serial1.print("Back Left Increased: ");
        Serial.println(bl_change);
        Serial1.println(bl_change);
        break;

      case 'k':
      case 'K': //BL DOWN
        bl_change -= 10;
        Serial.print("Back Left Decreased: ");
        Serial1.print("Back Left Decreased: ");
        Serial.println(bl_change);
        Serial1.println(bl_change);
        break;

      case 'o':
      case 'O': //BR UP
        br_change += 10;
        Serial.print("Back Right Increased: ");
        Serial1.print("Back Right Increased: ");
        Serial.println(br_change);
        Serial1.println(br_change);
        break;

      case 'l':
      case 'L': //BR DOWN
        br_change -= 10;
        Serial.print("Back Right Decreased: ");
        Serial1.print("Back Right Decreased: ");
        Serial.println(br_change);
        Serial1.println(br_change);
        break;


      case 'r':  // Request for status report
      case 'R':
        Serial.println("Status report requested");
        // Sending a status report over Serial1 (HC-12)
        Serial1.println("=== Mega Status Report ===");
        Serial1.print("Speed value: ");
        Serial1.println(speed_val);
        // (Optionally add more debug info here, e.g., sensor values)
        Serial.println(gyro_u);
        break;

      default:
        stop_motors();
        Serial.println("Stop executed");
        Serial1.println("Stop executed");
        break;
    }
  }
}
