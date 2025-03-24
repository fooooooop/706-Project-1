#include "serial_command.h"

#include "dual_serial.h"
#include "moving_logic.h"
#include "positioning_system.h"

// woah im adding such big change - test for git pushing

// --- Wireless Command Parsing ---
// This function now listens on Serial1 (the HC‑12)
void read_serial_command() {
  if (Serial1.available()) {
    char val = Serial1.read();
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
        speed_change = -100;
        dualPrintln("Speed decreased");
        break;
      case '=':
      case '+':  // Increase Speed
        speed_change = 100;
        dualPrintln("Speed increased");
        break;

      case 'y':
      case 'Y':  // FL UP
        fl_change += 10;
        dualPrint("Front Left Increased: ");
        dualPrintln(fl_change);
        break;

      case 'h':
      case 'H':  // FL DOWN
        fl_change -= 10;
        dualPrint("Front Left Decreased: ");
        dualPrintln(fl_change);
        break;

      case 'u':
      case 'U':  // FR UP
        fr_change += 10;
        dualPrint("Front Right Increased: ");
        dualPrintln(fr_change);
        break;

      case 'j':
      case 'J':  // FR DOWN
        fr_change -= 10;
        dualPrint("Front Right Decreased: ");
        dualPrintln(fr_change);
        break;

      case 'i':
      case 'I':  // BL UP
        bl_change += 10;
        dualPrint("Back Left Increased: ");
        dualPrintln(bl_change);
        break;

      case 'k':
      case 'K':  // BL DOWN
        bl_change -= 10;
        dualPrint("Back Left Decreased: ");
        dualPrintln(bl_change);
        break;

      case 'o':
      case 'O':  // BR UP
        br_change += 10;
        dualPrint("Back Right Increased: ");
        dualPrintln(br_change);
        break;

      case 'l':
      case 'L':  // BR DOWN
        br_change -= 10;
        dualPrint("Back Right Decreased: ");
        dualPrintln(br_change);
        break;

      case 'r':  // Request for status report
      case 'R':
        dualPrintln("Status report requested");
        dualPrintln(
            "=== Mega Status Report ===");  // Keeping this HC-12 specific line
        dualPrint("Speed value: ");
        dualPrintln(speed_val);
        dualPrintln(gyro_u);  // Keeping gyro_u only on Serial
        break;

      case 'z':  // Find Corner
      case 'Z':
        find_corner();
        dualPrintln("Find corner executed");
        break;
      case 'P': {
        dualPrintln("Position test mode: Press any key to exit.");
        while (!Serial1.available()) {
          Position pos = updatePosition();
          dualPrint("X: ");
          dualPrint(pos.x);
          dualPrint(" cm, Y: ");
          dualPrint(pos.y);
          dualPrint(" cm, Theta: ");
          dualPrintln(pos.theta);
          delay(500);
        }
        while (Serial1.available()) Serial1.read();
        dualPrintln("Exiting position test mode.");
        break;
      }

      default:
        stop_motors();
        dualPrintln("Stop executed");
        break;
    }
  }
}
