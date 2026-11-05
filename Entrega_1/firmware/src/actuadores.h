// Servos MG90S en el PCA9685: salida por ticks, seguimiento suave con limite de aceleracion,
// calibracion por dos puntos (guardada en la flash) y trayectorias quinticas.
//
// MG90S (Tower Pro): 4,8-6 V, senal de 50 Hz (periodo 20 ms), banda muerta 5 us,
// 0,10 s/60 deg a 4,8 V (0,08 s/60 deg a 6 V), torque de bloqueo 1,8 kgf.cm (4,8 V) / 2,2 kgf.cm (6 V).
// En la practica ~600-2400 us cubren ~180 deg; por debajo o por encima el servo empuja contra su
// tope interno (zumbido "tap tap", se calienta). Por eso US_ABS_MIN/US_ABS_MAX.
// Mapa de este archivo:
//   1. Datos de calibracion y conversion angulo <-> pulso (qDesdeUs, usDesdeQ).
//   2. Manejo directo del PCA9685 por registros I2C (pcaEscribir, pcaCanal, iniciarPCA).
//   3. Salida a los servos (escribirUs, apagar).
//   4. Seguimiento suave para sliders y jog (servicioSeguimiento).
//   5. Calibracion en la flash (guardar y cargar, con el recorte a Q_SEGURO).
//   6. Trayectorias quinticas (trayectoria, moverArticular, moverPinza).
// Lo usa main.cpp; toma sus constantes de config.h.
#pragma once
#include <Wire.h>
#include <Preferences.h>
#include "config.h"

// Relacion lineal angulo <-> pulso definida por dos puntos medidos (A y B) y limites seguros.
// Para la pinza el "angulo" es el porcentaje de cierre: 0 = abierta, 100 = cerrada.
struct CalServo {
  float qa, qb;
  int16_t usa, usb;
  int16_t usMin, usMax;
};

// Valores de partida (~11 us/deg) hasta calibrar cada servo con los comandos "cal".
const CalServo CAL_DEFECTO[NUM_SERVOS] = {
    {0.0f, 90.0f, 1500, 2500, 600, 2400},     // base: 0 = al frente, +90 = a la izquierda
    {90.0f, 0.0f, 1500, 500, 600, 2400},      // hombro: 90 = vertical, 0 = horizontal hacia adelante
    {-90.0f, 0.0f, 1500, 2500, 600, 2400},    // codo: -90 = eslabon 3 a 90 deg del 2, 0 = alineados
    {0.0f, -90.0f, 1500, 500, 600, 2400},     // muneca: 0 = alineada con eslabon 3, -90 = doblada hacia abajo
    {0.0f, 100.0f, 1200, 1800, 1100, 1900},   // pinza: 0 = abierta, 100 = cerrada (rango corto: el MG90S aprieta fuerte)
};

CalServo cal[NUM_SERVOS];
uint8_t calHecho[NUM_SERVOS] = {0};   // pasos de calibracion hechos por canal (bits CAL_A, CAL_B, CAL_MIN, CAL_MAX)
enum { CAL_A = 1, CAL_B = 2, CAL_MIN = 4, CAL_MAX = 8, CAL_TODO = 15 };
Preferences prefs;

int usActual[NUM_SERVOS] = {0};       // ultimo pulso aplicado [us]; 0 = sin senal (posicion desconocida)
int usObjetivo[NUM_SERVOS] = {0};     // destino del seguimiento (sliders, jog de calibracion, "us")
float usPos[NUM_SERVOS] = {0};        // posicion del seguimiento con decimales [us]
float usVel[NUM_SERVOS] = {0};        // velocidad del seguimiento [us por periodo]
uint16_t ticksActual[NUM_SERVOS] = {0};
float velSeguimiento = VEL_SEGUIMIENTO_US_S;   // [us/s] sliders, jog y "us" (comando "vel"); el MG90S llega a ~7000
float usPorTick = 0.0f;               // duracion de un tick del PCA9685 (~4,9 us a 50 Hz)
uint32_t erroresI2C = 0;

// Recta por los puntos A y B: de pulso a angulo D-H, y al reves en usDesdeQ().
inline float qDesdeUs(int ch, float us) {
  const CalServo &c = cal[ch];
  return c.qa + (us - c.usa) * (c.qb - c.qa) / (c.usb - c.usa);
}
inline float usDesdeQ(int ch, float q) {
  const CalServo &c = cal[ch];
  return c.usa + (q - c.qa) * (c.usb - c.usa) / (c.qb - c.qa);
}
// Rango de angulos de cada canal segun sus limites; qValido() permite una tolerancia de 0,05 grados.
inline float qMin(int ch) { return min(qDesdeUs(ch, cal[ch].usMin), qDesdeUs(ch, cal[ch].usMax)); }
inline float qMax(int ch) { return max(qDesdeUs(ch, cal[ch].usMin), qDesdeUs(ch, cal[ch].usMax)); }
inline bool qValido(int ch, float q) { return q >= qMin(ch) - 0.05f && q <= qMax(ch) + 0.05f; }

// true si los cuatro angulos estan dentro de los limites calibrados.
bool articulacionesValidas(const float q[4]) {
  for (int ch = 0; ch < 4; ch++)
    if (!qValido(ch, q[ch])) return false;
  return true;
}

// ---------------- Salida PWM: PCA9685 manejado directo por registros ----------------
// No se usa la libreria de Adafruit:
//  - pca.begin() pone el chip a 1000 Hz un instante SIN apagar las salidas (los servos reciben pulsos
//    absurdos y dan un tiron) y despues habia que apagar todo (el brazo caia) en cada reinicio de la ESP32;
//  - writeMicroseconds() relee el prescaler por I2C en cada escritura: si esa lectura falla, el pulso
//    sale absurdo y el servo salta.
// Aqui: si el PCA9685 ya estaba funcionando (la ESP32 se reinicio con los 6 V conectados: subida de
// firmware, boton EN), se LEEN los pulsos que tenia y se mantienen: el brazo no cae ni salta y el firmware
// sabe donde esta cada servo. Solo tras un corte de energia real los servos parten sin senal.
constexpr uint8_t PCA_MODE1 = 0x00, PCA_MODE2 = 0x01, PCA_LED0 = 0x06, PCA_PRESCALE = 0xFE;
constexpr uint8_t MODE1_SLEEP = 0x10, MODE1_AI = 0x20, MODE1_ALLCALL = 0x01, MODE2_OUTDRV = 0x04;
const uint8_t PRESCALE_SERVO = (uint8_t)(lroundf(OSC_PCA_HZ / (4096.0f * FREQ_SERVO_HZ)) - 1);   // 131 a 50 Hz

// Envia bytes al chip; reintenta una vez y cuenta los errores (se ven con "scan").
bool pcaEscribir(const uint8_t *datos, size_t n) {
  for (int intento = 0; intento < 2; intento++) {
    Wire.beginTransmission(PCA_ADDR);
    Wire.write(datos, n);
    if (Wire.endTransmission() == 0) return true;
    erroresI2C++;
  }
  return false;
}

// Lee n bytes desde un registro del chip.
bool pcaLeer(uint8_t reg, uint8_t *datos, size_t n) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom((int)PCA_ADDR, (int)n) != (int)n) return false;
  for (size_t i = 0; i < n; i++) datos[i] = Wire.read();
  return true;
}

// Escribe un solo registro.
bool pcaReg(uint8_t reg, uint8_t valor) {
  uint8_t d[2] = {reg, valor};
  return pcaEscribir(d, 2);
}

// Escribe un canal: ticks = ancho del pulso (0-4095) o 4096 = sin pulso (servo libre).
bool pcaCanal(int ch, uint16_t ticks) {
  uint8_t d[5] = {(uint8_t)(PCA_LED0 + 4 * ch), 0, 0, (uint8_t)(ticks & 0xFF), (uint8_t)(ticks >> 8)};   // ON = 0
  return pcaEscribir(d, 5);
}

// Devuelve cuantos servos recupero con pulso (0 si el chip partio de cero), o -1 si no responde.
int iniciarPCA() {
  usPorTick = (PRESCALE_SERVO + 1) * 1e6f / OSC_PCA_HZ;
  uint8_t modo, pre;
  if (!pcaLeer(PCA_MODE1, &modo, 1) || !pcaLeer(PCA_PRESCALE, &pre, 1)) return -1;
  bool funcionando = !(modo & MODE1_SLEEP) && pre == PRESCALE_SERVO;
  int recuperados = 0;
  if (funcionando) {
    for (int ch = 0; ch < NUM_SERVOS; ch++) {
      uint8_t r[4];
      if (!pcaLeer(PCA_LED0 + 4 * CANAL_PCA[ch], r, 4)) continue;
      uint16_t off = r[2] | (r[3] & 0x0F) << 8;
      bool apagado = r[3] & 0x10;
      float us = off * usPorTick;
      if (apagado || r[0] || r[1] || us < US_ABS_MIN - 5 || us > US_ABS_MAX + 5) continue;
      ticksActual[ch] = off;
      usPos[ch] = us;
      usActual[ch] = usObjetivo[ch] = lroundf(us);
      recuperados++;
    }
    pcaReg(PCA_MODE1, MODE1_AI | MODE1_ALLCALL);   // asegura el auto-incremento, sin tocar las salidas
  } else {
    // Arranque en frio: todas las salidas sin pulso ANTES de despertar el oscilador, luego 50 Hz.
    pcaReg(PCA_MODE1, MODE1_SLEEP | MODE1_AI | MODE1_ALLCALL);
    for (int ch = 0; ch < 16; ch++) pcaCanal(ch, 4096);
    pcaReg(PCA_PRESCALE, PRESCALE_SERVO);   // el prescaler solo se puede cambiar dormido
    pcaReg(PCA_MODE2, MODE2_OUTDRV);
    pcaReg(PCA_MODE1, MODE1_AI | MODE1_ALLCALL);
    delay(1);   // el oscilador necesita 500 us para estabilizarse
  }
  return recuperados;
}

// Aplica un pulso (rango absoluto). Solo escribe por I2C si cambia el tick; reintenta una vez si falla.
// Si la escritura no llega, NO se registra nada (ni ticks ni posicion): la telemetria y el D-H siguen
// mostrando el pulso que el servo realmente tiene, y el seguimiento lo vuelve a intentar en el proximo ciclo.
bool escribirPulso(int ch, float us) {
  us = constrain(us, (float)US_ABS_MIN, (float)US_ABS_MAX);
  uint16_t ticks = (uint16_t)lroundf(us / usPorTick);
  if (ticks != ticksActual[ch]) {
    if (!pcaCanal(CANAL_PCA[ch], ticks)) return false;
    ticksActual[ch] = ticks;
  }
  usPos[ch] = us;
  usActual[ch] = lroundf(us);
  return true;
}

// Pulso directo (rango absoluto) que ademas fija el objetivo: el seguimiento queda quieto ahi.
// Si no llega, el objetivo no se toca: el seguimiento lo reintenta.
void salidaUs(int ch, float us) {
  if (!escribirPulso(ch, us)) return;
  usObjetivo[ch] = usActual[ch];
  usVel[ch] = 0.0f;
}

// Escritura normal (trayectorias): respeta los limites calibrados.
void escribirUs(int ch, float us) {
  salidaUs(ch, constrain(us, (float)cal[ch].usMin, (float)cal[ch].usMax));
}

// Devuelve false si el PCA9685 no acepto la orden (el servo sigue con fuerza y su estado no cambia).
bool apagar(int ch) {
  bool ok = false;
  for (int intento = 0; intento < 3 && !ok; intento++) ok = pcaCanal(CANAL_PCA[ch], 4096);   // bit FULL_OFF: servo libre
  if (!ok) return false;
  usActual[ch] = usObjetivo[ch] = 0;
  usPos[ch] = usVel[ch] = 0.0f;
  ticksActual[ch] = 0;
  return true;
}

// ---------------- Seguimiento suave (sliders, jog, "us") ----------------
// Fija el destino sin bloquear; el lazo principal lo alcanza con servicioSeguimiento().
// calibrado = true recorta a los limites calibrados (uso normal); false, al rango absoluto (calibrar).
void fijarObjetivo(int ch, int us, bool calibrado) {
  usObjetivo[ch] = calibrado ? constrain(us, (int)cal[ch].usMin, (int)cal[ch].usMax) : constrain(us, US_ABS_MIN, US_ABS_MAX);
}

// Perfil trapezoidal: acelera hasta VEL_SEGUIMIENTO y frena justo a tiempo (v = sqrt(2 a e)), sin pasarse.
// Asi un salto del slider no se convierte en un golpe: el servo arranca y se detiene suave.
// Un servo sin senal recibe el objetivo de golpe (no se sabe donde esta). Devuelve true si algo se movio.
bool servicioSeguimiento() {
  const float dt = PERIODO_MS / 1000.0f;
  const float vMax = velSeguimiento * dt, a = velSeguimiento / T_ACEL_SEGUIMIENTO_S * dt * dt;
  bool movido = false;
  for (int ch = 0; ch < NUM_SERVOS; ch++) {
    if (!usObjetivo[ch]) continue;
    if (!usActual[ch]) {
      salidaUs(ch, usObjetivo[ch]);
      movido = true;
      continue;
    }
    float e = usObjetivo[ch] - usPos[ch];
    if (fabsf(e) < 0.5f && fabsf(usVel[ch]) <= a) {
      usVel[ch] = 0.0f;
      continue;
    }
    float vDes = copysignf(min(vMax, sqrtf(2.0f * a * fabsf(e))), e);
    usVel[ch] += constrain(vDes - usVel[ch], -a, a);
    if (usVel[ch] * e > 0.0f && fabsf(usVel[ch]) >= fabsf(e)) {   // el paso llegaria o pasaria: queda en el objetivo
      escribirPulso(ch, usObjetivo[ch]);
      usVel[ch] = 0.0f;
    } else {
      escribirPulso(ch, usPos[ch] + usVel[ch]);
    }
    movido = true;
  }
  return movido;
}

// true si el canal ya llego a su destino y esta detenido.
bool enObjetivo(int ch) { return !usObjetivo[ch] || (usActual[ch] == usObjetivo[ch] && usVel[ch] == 0.0f); }

// Congela todos los servos donde estan (orden de parada).
void detenerSeguimiento() {
  for (int ch = 0; ch < NUM_SERVOS; ch++) {
    usObjetivo[ch] = usActual[ch];
    usVel[ch] = 0.0f;
  }
}

// Bloquea hasta que los canales de la mascara llegan a su objetivo (o parada / tiempo agotado).
bool esperarSeguimiento(uint8_t mascara, uint32_t timeoutMs = 6000) {
  for (uint32_t t0 = millis(); millis() - t0 < timeoutMs;) {
    bool listo = true;
    for (int ch = 0; ch < NUM_SERVOS; ch++)
      if ((mascara & (1 << ch)) && !enObjetivo(ch)) listo = false;
    if (listo) return true;
    servicioSeguimiento();
    if (comprobarParada()) {
      detenerSeguimiento();
      return false;
    }
    delay(PERIODO_MS);
  }
  return false;
}

// Lleva un canal a un pulso (rango absoluto) con el perfil suave y espera a que llegue.
bool rampaUs(int ch, int us, bool calibrado = false) {
  fijarObjetivo(ch, us, calibrado);
  return esperarSeguimiento(1 << ch);
}

// Los limites min/max ya fueron medidos (asistente) o fijados (cal rapida): no se debe pasar de ellos.
inline bool limitesCalibrados(int ch) { return (calHecho[ch] & (CAL_MIN | CAL_MAX)) == (CAL_MIN | CAL_MAX); }

// Recorta los limites de base, hombro, codo y muneca a Q_SEGURO (se queda con lo mas estrecho entre lo
// calibrado y lo seguro). Se aplica al cargar y al guardar cualquier calibracion.
void limitarASeguro() {
  for (int ch = 0; ch < 4; ch++) {
    if ((calHecho[ch] & (CAL_A | CAL_B)) != (CAL_A | CAL_B)) continue;   // sin recta pulso-angulo aun
    float u1 = usDesdeQ(ch, Q_SEGURO[ch][0]), u2 = usDesdeQ(ch, Q_SEGURO[ch][1]);
    int lo = (int)ceilf(min(u1, u2)), hi = (int)floorf(max(u1, u2));
    cal[ch].usMin = max((int)cal[ch].usMin, max(lo, US_ABS_MIN));
    cal[ch].usMax = min((int)cal[ch].usMax, min(hi, US_ABS_MAX));
    if (cal[ch].usMin > cal[ch].usMax) cal[ch].usMin = cal[ch].usMax = (int16_t)lroundf(usDesdeQ(ch, Q_REFERENCIA[ch]));
  }
  // Pinza: nunca mas alla de "abierta" ni de "cerrada sobre el cubo" (sus puntos A y B). Pasarse de abierta
  // empuja su tope; pasarse de cerrada aprieta el cubo con toda la fuerza del servo.
  if ((calHecho[CH_PINZA] & (CAL_A | CAL_B)) == (CAL_A | CAL_B)) {
    cal[CH_PINZA].usMin = max(cal[CH_PINZA].usMin, min(cal[CH_PINZA].usa, cal[CH_PINZA].usb));
    cal[CH_PINZA].usMax = min(cal[CH_PINZA].usMax, max(cal[CH_PINZA].usa, cal[CH_PINZA].usb));
  }
}

// Sin calibrar, "us" solo recorre +-50 grados alrededor del centro: lo justo para montar horns y encontrar
// referencias, sin llegar a los topes del servo.
constexpr int US_SIN_CAL_MIN = 1000, US_SIN_CAL_MAX = 2000;

// Angulos actuales [deg D-H]; false si algun servo del brazo aun no tiene posicion conocida.
bool leerQ(float q[4]) {
  for (int ch = 0; ch < 4; ch++) {
    if (!usActual[ch]) return false;
    q[ch] = qDesdeUs(ch, usPos[ch]);
  }
  return true;
}

// ---------------- Calibracion ----------------
void cargarCalibracion() {
  prefs.begin("brazo", false);
  bool hayCal = prefs.getBytesLength("cal") == sizeof(cal);
  if (hayCal) prefs.getBytes("cal", cal, sizeof(cal));
  else memcpy(cal, CAL_DEFECTO, sizeof(cal));
  if (prefs.getBytesLength("calok") == sizeof(calHecho)) prefs.getBytes("calok", calHecho, sizeof(calHecho));
  else memset(calHecho, hayCal ? CAL_TODO : 0, sizeof(calHecho));   // calibracion de una version anterior
  velSeguimiento = prefs.getFloat("vel", VEL_SEGUIMIENTO_US_S);
  limitarASeguro();
}

void guardarCalibracion() {
  limitarASeguro();
  prefs.putBytes("cal", cal, sizeof(cal));
  prefs.putBytes("calok", calHecho, sizeof(calHecho));
}

// true si todos los canales de la mascara tienen sus puntos A y B; si no, avisa cuales faltan.
bool exigirCalibracion(uint8_t mascara) {
  bool ok = true;
  for (int ch = 0; ch < NUM_SERVOS; ch++) {
    if (!(mascara & (1 << ch))) continue;
    uint8_t h = calHecho[ch];
    if ((h & (CAL_A | CAL_B)) != (CAL_A | CAL_B)) {
      responder("ERR,falta calibrar %s (canal %d):%s%s%s%s\n", NOMBRES[ch], ch, h & CAL_A ? "" : " a", h & CAL_B ? "" : " b",
                h & CAL_MIN ? "" : " min", h & CAL_MAX ? "" : " max");
      ok = false;
    } else if ((h & (CAL_MIN | CAL_MAX)) != (CAL_MIN | CAL_MAX)) {
      responder("AVISO,%s sin limites min/max propios: usa los de partida\n", NOMBRES[ch]);
    }
  }
  return ok;
}

// ---------------- Trayectorias ----------------
// Trayectoria quintica sincronizada para los canales de la mascara (bit ch = 1).
// Todos llegan a la vez; la duracion sale de la articulacion que mas recorre.
// Un servo sin posicion conocida se lleva directo a su destino, de a uno.
bool trayectoria(const float qf[NUM_SERVOS], uint8_t mascara, uint32_t tFijoMs = 0) {
  float q0[NUM_SERVOS] = {0};
  float dmax = 0.0f;
  for (int ch = 0; ch < NUM_SERVOS; ch++) {
    if (!(mascara & (1 << ch))) continue;
    if (!qValido(ch, qf[ch])) {
      responder("ERR,%s q=%.1f fuera de limites [%.1f, %.1f]\n", NOMBRES[ch], qf[ch], qMin(ch), qMax(ch));
      return false;
    }
    if (!usActual[ch]) {
      escribirUs(ch, usDesdeQ(ch, qf[ch]));
      delay(400);
    }
    q0[ch] = qDesdeUs(ch, usPos[ch]);
    dmax = max(dmax, fabsf(qf[ch] - q0[ch]));
  }
  // Pico de velocidad del perfil quintico = 1.875 * dq / T
  uint32_t T = tFijoMs ? tFijoMs : max(T_MIN_MS, (uint32_t)(1875.0f * dmax / VEL_MAX_DEG_S));
  uint32_t t0 = millis();
  for (;;) {
    float tau = min(1.0f, (millis() - t0) / (float)T);
    float s = tau * tau * tau * (10.0f - 15.0f * tau + 6.0f * tau * tau);
    for (int ch = 0; ch < NUM_SERVOS; ch++)
      if (mascara & (1 << ch)) escribirUs(ch, usDesdeQ(ch, q0[ch] + (qf[ch] - q0[ch]) * s));
    if (tau >= 1.0f) return true;
    if (comprobarParada()) return false;
    delay(PERIODO_MS);
  }
}

bool moverArticular(const float q[4]) {
  float qf[NUM_SERVOS] = {q[0], q[1], q[2], q[3], 0.0f};
  return trayectoria(qf, 0x0F);
}

bool moverPinza(float cierre) {
  float qf[NUM_SERVOS] = {0};
  qf[CH_PINZA] = cierre;
  // Si quedo suelta (se suelta al abrir), sigue fisicamente abierta: se retoma desde ahi y no de golpe.
  if (!usActual[CH_PINZA]) {
    escribirUs(CH_PINZA, usDesdeQ(CH_PINZA, PINZA_ABIERTA));
    delay(150);
  }
  if (!trayectoria(qf, 1 << CH_PINZA, T_PINZA_MS)) return false;
  if (cierre <= PINZA_ABIERTA + 0.5f) {
    // Abierta no necesita fuerza; si "abierta" queda contra su tope, sostenerla la calentaria: se suelta.
    delay(200);
    apagar(CH_PINZA);
    return true;
  }
  if (cierre < PINZA_CERRADA - 0.5f) return true;
  // Cerrada sobre el cubo: el servo empuja con fuerza proporcional a lo que le falta para llegar. Si el cubo
  // lo frena antes, quedaria con toda su fuerza (y calentandose) mientras lo sostiene: se afloja un poco.
  qf[CH_PINZA] = cierre - PINZA_ALIVIO;
  return trayectoria(qf, 1 << CH_PINZA, 150);
}
