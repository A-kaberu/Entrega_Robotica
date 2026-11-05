// Memoria flash emulada: guarda los valores por nombre mientras dura la simulacion.
#pragma once
#include "Arduino.h"

class Preferences {
 public:
  void begin(const char *, bool) {}
  size_t getBytesLength(const char *k) { return datos.count(k) ? datos[k].size() : 0; }
  size_t getBytes(const char *k, void *d, size_t n) {
    if (!datos.count(k)) return 0;
    n = min(n, datos[k].size());
    memcpy(d, datos[k].data(), n);
    return n;
  }
  size_t putBytes(const char *k, const void *d, size_t n) {
    datos[k].assign((const uint8_t *)d, (const uint8_t *)d + n);
    return n;
  }
  template <typename T> T leer(const char *k, T defecto) {
    T v = defecto;
    getBytes(k, &v, sizeof(T));
    return v;
  }
  uint32_t getUInt(const char *k, uint32_t d = 0) { return leer(k, d); }
  void putUInt(const char *k, uint32_t v) { putBytes(k, &v, sizeof(v)); }
  float getFloat(const char *k, float d = 0) { return leer(k, d); }
  void putFloat(const char *k, float v) { putBytes(k, &v, sizeof(v)); }
  int16_t getShort(const char *k, int16_t d = 0) { return leer(k, d); }
  void putShort(const char *k, int16_t v) { putBytes(k, &v, sizeof(v)); }
  bool getBool(const char *k, bool d = false) { return leer(k, d); }
  void putBool(const char *k, bool v) { putBytes(k, &v, sizeof(v)); }

 private:
  std::map<std::string, std::vector<uint8_t>> datos;
};
