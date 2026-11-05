// Sensor ToF emulado: la distancia la calcula el simulador segun la pose del brazo y los cubos de la mesa.
#pragma once
#include "Wire.h"

inline uint16_t (*ganchoDistancia)() = nullptr;

class VL53L0X {
 public:
  void setBus(TwoWire *) {}
  void setTimeout(uint16_t) {}
  bool init() { return true; }
  uint16_t readRangeSingleMillimeters() {
    delay(33);   // lo que tarda una medicion real
    return ganchoDistancia ? ganchoDistancia() : 8190;
  }
  bool timeoutOccurred() { return false; }
};
