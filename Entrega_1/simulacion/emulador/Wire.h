// Bus I2C emulado. En la direccion 0x40 hay un PCA9685: sus registros quedan guardados como en el chip real,
// con auto-incremento, asi el simulador lee lo que de verdad le llegaria a cada servo.
#pragma once
#include "Arduino.h"

class TwoWire {
 public:
  uint8_t registros[256] = {0};
  std::vector<uint8_t> direcciones;   // dispositivos conectados a este bus
  explicit TwoWire(std::vector<uint8_t> d) : direcciones(std::move(d)) {
    registros[0x00] = 0x11;           // MODE1 al encender: dormido
    registros[0xFE] = 30;             // PRESCALE al encender
    for (int canal = 0; canal < 16; canal++) registros[0x06 + 4 * canal + 3] = 0x10;   // salidas sin pulso
  }
  void begin(int, int) {}
  void setClock(uint32_t) {}
  void beginTransmission(uint8_t a) { dir = a; envio.clear(); }
  size_t write(uint8_t b) { envio.push_back(b); return 1; }
  size_t write(const uint8_t *d, size_t n) { envio.insert(envio.end(), d, d + n); return n; }
  uint8_t endTransmission(bool = true) {
    if (std::find(direcciones.begin(), direcciones.end(), dir) == direcciones.end()) return 2;   // nadie responde
    if (!envio.empty()) {
      puntero = envio[0];
      for (size_t i = 1; i < envio.size(); i++) registros[(uint8_t)(puntero + i - 1)] = envio[i];
    }
    return 0;
  }
  int requestFrom(int, int n) { leido = 0; pedido = n; return n; }
  int read() { return registros[(uint8_t)(puntero + leido++)]; }

 private:
  uint8_t dir = 0, puntero = 0;
  int leido = 0, pedido = 0;
  std::vector<uint8_t> envio;
};
inline TwoWire Wire({0x40, 0x70});
inline TwoWire Wire1({0x29});
