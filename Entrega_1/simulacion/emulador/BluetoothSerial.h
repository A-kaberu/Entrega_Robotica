// Bluetooth emulado: es un puerto serie mas, sin cliente conectado salvo que el simulador lo indique.
#pragma once
#include "Arduino.h"

class BluetoothSerial : public Stream {
 public:
  bool cliente = false;
  void begin(const char *) {}
  bool hasClient() { return cliente; }
};
