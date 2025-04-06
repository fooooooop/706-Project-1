#ifndef DUAL_SERIAL_H
#define DUAL_SERIAL_H

#include <Arduino.h>

// Generic print
template <typename T>
void dualPrint(const T& value) {
  Serial.print(value);
  Serial1.print(value);
}

// Generic println
template <typename T>
void dualPrintln(const T& value) {
  Serial.println(value);
  Serial1.println(value);
}

// Optional: println() with no arguments
inline void dualPrintln() {
  Serial.println();
  Serial1.println();
}

inline void dualPrint(float value, int digits) {
  Serial.print(value, digits);
  Serial1.print(value, digits);
}

inline void dualPrintln(float value, int digits) {
  Serial.println(value, digits);
  Serial1.println(value, digits);
}
#endif
