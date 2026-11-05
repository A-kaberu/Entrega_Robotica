// Emulador minimo del entorno Arduino/ESP32 para compilar el firmware real en el PC, sin hardware.
//
// Reemplaza solo lo que usa firmware/src: reloj virtual, puerto serie, I2C con un PCA9685 emulado (guarda
// sus registros igual que el chip), sensor ToF, memoria flash (Preferences) y tareas de FreeRTOS.
// El firmware se compila SIN cambios: ver simulacion/simular_firmware.cpp.
#pragma once
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <initializer_list>
#include <map>
#include <string>
#include <utility>
#include <vector>

using std::max;
using std::min;

#define radians(d) ((d) * 0.017453292519943295f)
#define degrees(r) ((r) * 57.29577951308232f)
#define sq(x) ((x) * (x))
#define constrain(x, l, h) ((x) < (l) ? (l) : ((x) > (h) ? (h) : (x)))

// ---------------- Reloj virtual: delay() no espera de verdad, solo avanza el tiempo simulado ----------------
inline uint32_t relojMs = 1000;
inline void (*ganchoTiempo)() = nullptr;   // el simulador actualiza aqui el mundo (cubos, registro, ordenes)
inline uint32_t millis() { return relojMs; }
inline void delay(uint32_t ms) {
  for (uint32_t i = 0; i < ms; i++) {
    relojMs++;
    if (ganchoTiempo) ganchoTiempo();
  }
}

// ---------------- String de Arduino, sobre std::string ----------------
class String {
 public:
  std::string s;
  String(const char *c = "") : s(c) {}
  String(const std::string &c) : s(c) {}
  size_t length() const { return s.size(); }
  const char *c_str() const { return s.c_str(); }
  char operator[](size_t i) const { return i < s.size() ? s[i] : 0; }
  String &operator+=(char c) { s += c; return *this; }
  bool operator==(const char *c) const { return s == c; }
  bool operator==(const String &o) const { return s == o.s; }
  bool operator!=(const String &o) const { return s != o.s; }
  bool startsWith(const char *c) const { return s.rfind(c, 0) == 0; }
  int indexOf(char c) const { auto p = s.find(c); return p == std::string::npos ? -1 : (int)p; }
  int indexOf(const char *c) const { auto p = s.find(c); return p == std::string::npos ? -1 : (int)p; }
  String substring(size_t a) const { return a < s.size() ? String(s.substr(a)) : String(""); }
  String substring(size_t a, size_t b) const { return a < s.size() ? String(s.substr(a, b - a)) : String(""); }
  float toFloat() const { return (float)atof(s.c_str()); }
  void trim() {
    size_t a = s.find_first_not_of(" \t\r\n"), b = s.find_last_not_of(" \t\r\n");
    s = a == std::string::npos ? "" : s.substr(a, b - a + 1);
  }
};

// ---------------- Puertos serie: la entrada la pone el simulador y la salida queda guardada ----------------
class Stream {
 public:
  std::deque<char> entrada;
  std::string salida;
  int available() { return (int)entrada.size(); }
  int peek() { return entrada.empty() ? -1 : entrada.front(); }
  int read() {
    if (entrada.empty()) return -1;
    char c = entrada.front();
    entrada.pop_front();
    return c;
  }
  void print(const char *t) { salida += t; }
  void println(const char *t) { salida += t; salida += "\n"; }
  void enviar(const std::string &t) { entrada.insert(entrada.end(), t.begin(), t.end()); }
};
class HardwareSerial : public Stream {
 public:
  void begin(int) {}
};
inline HardwareSerial Serial;

// ---------------- FreeRTOS: mutex sin efecto y una tarea que el simulador ejecuta cuando quiere ----------------
typedef int SemaphoreHandle_t;
#define portMAX_DELAY 0xFFFFFFFFu
#define pdMS_TO_TICKS(ms) (ms)
inline SemaphoreHandle_t xSemaphoreCreateMutex() { return 1; }
inline bool xSemaphoreTake(SemaphoreHandle_t, uint32_t) { return true; }
inline void xSemaphoreGive(SemaphoreHandle_t) {}
inline void (*tareaGuardada)(void *) = nullptr;
inline int vueltasTarea = 0;
struct FinTarea {};
inline void vTaskDelay(uint32_t ms) {   // sale de la tarea (que es un bucle infinito) al acabar sus vueltas
  if (vueltasTarea-- <= 0) throw FinTarea{};
  delay(ms);
}
inline void xTaskCreatePinnedToCore(void (*f)(void *), const char *, int, void *, int, void *, int) { tareaGuardada = f; }
