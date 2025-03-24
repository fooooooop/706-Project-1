#include "serial_command.h"

#include "dual_serial.h"
#include "moving_logic.h"

// woah im adding such big change - test for git pushing

// --- Wireless Command Parsing ---
// This function now listens on Serial1 (the HC‑12)
void read_serial_command() {
  if (Serial.available()) {
    char val = Serial.read();
    dualPrint("Received command: ");
    dualPrintln(String(val));
    dualPrint("Speed: ");
    dualPrint(speed_val);
    dualPrintln(" ms");

    switch (val) {
      case 'w':  // Move Forward
      case 'W':
        forward();
        dualPrintln("Forward executed");
        break;
      case 's':  // Move Backward
      case 'S':
        reverse();
        dualPrintln("Reverse executed");
        break;
      case 'q':  // Strafe Left
      case 'Q':
        strafe_left();
        dualPrintln("Strafe Left executed");
        break;
      case 'e':  // Strafe Right
      case 'E':
        strafe_right();
        dualPrintln("Strafe Right executed");
        break;
      case 'a':  // Rotate Counter-Clockwise
      case 'A':
        ccw();
        dualPrintln("Rotate CCW executed");
        break;
      case 'd':  // Rotate Clockwise
      case 'D':
        cw();
        dualPrintln("Rotate CW executed");
        break;
      case '-':  // Decrease Speed
      case '_':
        speed_change = -50;
        dualPrintln("Speed decreased");
        break;
      case '=':
      case '+':  // Increase Speed
        speed_change = 50;
        dualPrintln("Speed increased");
        break;

      case 'r':  // Request for status report
      case 'R':
        dualPrintln("Status report requested");
        Serial1.println(
            "=== Mega Status Report ===");  // Keeping this HC-12 specific line
        Serial1.print("Speed value: ");
        Serial1.println(speed_val);
        Serial.println(gyro_u);  // Keeping gyro_u only on Serial
        break;

      case 'z':  // Find Corner
      case 'Z':
        dualPrintln("Find corner initiated");
        find_corner();
        dualPrintln("Find corner executed");
        break;

      case 'x':  // Initiate forward_left()
      case 'X':
        dualPrintln("Loop One initiated");
        forward_left();
        dualPrintln("Loop One done!");
        break;

      case 'c':  // Initiate forward_right()
      case 'C':
        dualPrintln("Loop One initiated");
        forward_right();
        dualPrintln("Loop One done!");
        break;

      default:
        stop_motors();
        dualPrintln("Stop executed");
        break;
    }
  }
}
