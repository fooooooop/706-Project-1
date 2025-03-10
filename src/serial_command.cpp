#include "serial_command.h"

#include "moving_logic.h"

void read_serial_command() {
  if (SerialCom->available()) {
    char val = SerialCom->read();
    SerialCom->print("Speed:");
    SerialCom->print(speed_val);
    SerialCom->print(" ms ");

    switch (val) {
      case 'w':
      case 'W':
        forward();
        SerialCom->println("Forward");
        break;
      case 's':
      case 'S':
        reverse();
        SerialCom->println("Backwards");
        break;
      case 'q':
      case 'Q':
        strafe_left();
        SerialCom->println("Strafe Left");
        break;
      case 'e':
      case 'E':
        strafe_right();
        SerialCom->println("Strafe Right");
        break;
      case 'a':
      case 'A':
        ccw();
        SerialCom->println("ccw");
        break;
      case 'd':
      case 'D':
        cw();
        SerialCom->println("cw");
        break;
      case '-':
      case '_':
        speed_change = -100;
        SerialCom->println("-100");
        break;
      case '=':
      case '+':
        speed_change = 100;
        SerialCom->println("+");
        break;
      default:
        stop_motors();
        SerialCom->println("stop");
        break;
    }
  }
}
