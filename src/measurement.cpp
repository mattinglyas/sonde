#include <Arduino.h>
#include "measurement.h"

Measurement::Measurement() {
    reset();
}

// Fully reset a measurement back to zeros.
void Measurement::reset() {
  memset(data, 0, sizeof(data));
  memset(timestamp, 0, sizeof(timestamp));
  memset(voltage, 0, sizeof(voltage));
  memset(lastPowerOff, 0, sizeof(lastPowerOff));
}
