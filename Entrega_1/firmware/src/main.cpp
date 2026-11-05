// Firmware del brazo robotico 4 GDL - Lab. 1 Fundamentos de Robotica (USACH 2026).
//
// Hardware: ESP32 DevKit + PCA9685 (Wire, GPIO21/22) + VL53L0X (Wire1, GPIO18/19) + 5 servos MG90S.
// Comandos por USB (monitor serie, 115200) o Bluetooth clasico ("Brazo_Robotico_ESP32"), una linea por comando.
//
// Protocolo de la app (sliders y modo automatico):
//   B<0-180> H<0-180> C<0-180> M<0-180> P<0-180>   base, hombro, codo, muneca, pinza (grados de servo)
//   A                                               inicia la rutina automatica
//   a  (o S / stop)                                 detiene el brazo (tambien en medio de un movimiento)
//
// Comandos de texto:
//   home                       lleva el brazo a la pose HOME con la pinza abierta
//   move <x> <y> <z> [phi]     mueve la punta a (x, y, z) [mm, z sobre la mesa], pinza a phi [deg] (defecto -90)
//   q <q1> <q2> <q3> <q4>      mueve las articulaciones [deg D-H]
//   pinza <0-100>              cierre de la pinza (0 abierta, 100 cerrada); tambien "pinza abrir" / "pinza cerrar"
//   estado                     angulos, pulsos y posiciones D-H de las articulaciones
//   us <canal> <us>            pulso directo, recortado a los limites calibrados (sin calibrar, 1000-2000 us)
//   cal                        muestra la calibracion
//   cal <canal> a|b <grados>   guarda el pulso actual del canal como el angulo dado (dos puntos por servo)
//   cal <canal> lim            guarda el pulso actual como limite seguro (min o max se deduce de A y B)
//   cal reset                  vuelve a la calibracion de partida
//   aprender                   mide la distancia del ToF en cada punto de recogida VACIO (referencia)
//   verificar                  revisa, sin mover el brazo, que todas las poses de la tarea quepan en los limites
//   prueba [1|2]               prueba de movimiento sin cubos: 1 = articulaciones, 2 = puntos de la mesa
//   dh                         tabla D-H con los angulos actuales y matriz T0_4
//   telem 1|0                  telemetria tambien por USB (para el panel del PC)
//   tof cal <mm>               offset del ToF con una superficie a distancia conocida
//   corregir 1|0               usa (o no) el error de altura medido por el ToF en 'aprender' (lazo externo)
//   off | soltar | X           SUELTA todos los servos al instante, tambien en medio de un movimiento
//   reposo [s] | vel [us/s]    tiempo de reposo (0 = nunca) y velocidad del seguimiento
//   bt 1|0                     activa o apaga el envio de respuestas y telemetria por Bluetooth
//   pca | pwm <0-15> <us>      diagnostico del PCA9685 y prueba de una salida libre
//   tof | scan | ayuda
//
// Telemetria D-H calculada en paralelo (tarea en el nucleo 0, cada 100 ms si algo cambio, y cada 1 s):
// T,q1,q2,q3,q4,pinza,x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4 (O1..O4 de las matrices D-H).
// D,<mm>: distancia del ToF de la mano, cada 200 ms.
// Sliders: 0-180 recorre el rango calibrado de cada servo, con arranque y frenado suaves.
// obj <canal> <us>: destino sin esperar (jog del asistente de calibracion).
// Estado de la rutina automatica y de la prueba: E,<ESTADO>[,datos]

// Mapa de este archivo (de arriba hacia abajo):
//   1. Comunicacion: responder() escribe por USB y Bluetooth; Entrada arma lineas con lo que llega.
//   2. Parada y soltar: se atienden al instante, incluso en medio de un movimiento (comprobarParada).
//   3. Telemetria: tarea paralela que calcula el D-H y envia las lineas T y D.
//   4. Movimientos cartesianos: usan la IK de cinematica.h y las trayectorias de actuadores.h.
//   5. Sensor ToF: lectura con mediana y correccion de altura.
//   6. Rutina automatica: detectar con el sensor, tomar, trasladar y apilar.
//   7. Verificacion, pruebas y diagnostico: verificar, prueba, dh, scan, cal.
//   8. Protecciones de los servos: protegerServos().
//   9. Interprete de comandos: ejecutar() decide que hacer con cada linea.
//  10. setup() y loop(): arranque y lazo principal.
// Archivos relacionados: config.h (medidas y constantes), cinematica.h (D-H e IK), actuadores.h (servos).
#include <Arduino.h>
#include <Wire.h>
#include <BluetoothSerial.h>
#include <VL53L0X.h>
#include <stdarg.h>
#include "config.h"
#include "cinematica.h"
#include "actuadores.h"

// Objetos y estado general del programa.
BluetoothSerial SerialBT;
VL53L0X tof;
bool tofOk = false;
uint16_t baseTof[N_RECOGIDA] = {0};   // distancia medida con cada punto de recogida vacio
bool tofAprendido = false;
bool modoAuto = false;
bool pararPedido = false;
volatile bool telemUSB = false;
// Envio por Bluetooth (respuestas y telemetria T/D). Apagado por defecto: la app de App Inventor solo envia
// (SendText) y nunca lee; si la ESP32 le escribe, el buffer del telefono se llena, el Bluetooth deja de
// aceptar datos y la ESP32 queda esperando para escribir (con el mutex tomado), lo que congela el control.
// La app puede activarlo con "bt 1" si algun dia lee la telemetria para mostrar el D-H.
volatile bool ecoBT = false;
SemaphoreHandle_t mtxSalida;   // las dos tareas escriben por los mismos puertos: una linea a la vez
SemaphoreHandle_t mtxTof;      // y leen el mismo sensor ToF
uint16_t leerTof(bool crudo);
bool pausa(uint32_t ms);

// ---------------- Comunicacion ----------------
// Igual que printf, pero la salida va por USB y, si esta activado, tambien por Bluetooth.
// El mutex evita que la tarea de telemetria mezcle sus lineas con estas.
void responder(const char *fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  xSemaphoreTake(mtxSalida, portMAX_DELAY);
  Serial.print(buf);
  if (ecoBT && SerialBT.hasClient()) SerialBT.print(buf);
  xSemaphoreGive(mtxSalida);
}

// Junta caracteres en lineas. Una linea termina con '\n' / '\r' o tras 100 ms sin datos
// (la app puede enviar "A" o "B90" sin fin de linea). Si llegan sliders pegados ("B90B91H45"), cada
// letra nueva despues de un numero de slider cierra el comando anterior, asi no se pierde el ultimo valor.
struct Entrada {
  Stream &s;
  String buf;
  uint32_t tUltimo = 0;
  explicit Entrada(Stream &st) : s(st) {}
  // true si lo acumulado ya es un slider completo, por ejemplo "B90".
  bool esSlider() const {
    return buf.length() > 1 && String("BHCMP").indexOf(buf[0]) >= 0 && isdigit(buf[1]) && isdigit(buf[buf.length() - 1]);
  }
  // Lee sin bloquear. Devuelve true cuando hay un comando completo en "linea";
  // si todavia falta parte, lo deja en buf y devuelve false.
  bool leer(String &linea) {
    while (s.available()) {
      char c = (char)s.peek();
      if (esSlider() && String("BHCMPAaX").indexOf(c) >= 0) {   // empieza otro comando: cierra el slider
        linea = buf;
        buf = "";
        return true;
      }
      s.read();
      tUltimo = millis();
      // Coma y punto y coma separan comandos escritos en una misma linea ("us 3 1500, us 3 1550"):
      // antes solo se ejecutaba el primero y el resto se perdia sin aviso. Ningun comando lleva comas.
      if (c == '\n' || c == '\r' || c == ',' || c == ';') {
        if (buf.length()) {
          linea = buf;
          buf = "";
          return true;
        }
      } else if (buf.length() < 64) {
        buf += c;
      }
    }
    if (buf.length() && millis() - tUltimo > 100) {
      linea = buf;
      buf = "";
      return true;
    }
    return false;
  }
};
Entrada entradaUSB(Serial), entradaBT(SerialBT);

// Ordenes de parada: detienen el movimiento, pero los servos mantienen la fuerza.
bool esParada(String l) {
  l.trim();
  return l == "a" || l == "S" || l == "stop";
}

// Soltar: corta el pulso de todos los servos (quedan libres, sin fuerza). Se atiende AL INSTANTE,
// tambien en medio de un movimiento o de la rutina. Ojo: sin fuerza el brazo cae por su peso.
bool esSoltar(String l) {
  l.trim();
  return l == "off" || l == "soltar" || l == "X";
}

// Suelta todos los servos y avisa si alguno no recibio la orden (sigue con fuerza: cortar los 6 V).
void soltarTodo() {
  int fallidos = 0;
  for (int ch = 0; ch < NUM_SERVOS; ch++)
    if (!apagar(ch)) fallidos++;
  for (int s = 0; s < 16; s++) {   // tambien las salidas sin articulacion (servos de prueba con "pwm")
    bool usada = false;
    for (int ch = 0; ch < NUM_SERVOS; ch++) usada |= CANAL_PCA[ch] == s;
    if (!usada && !pcaCanal(s, 4096)) fallidos++;
  }
  if (fallidos) responder("ERR,%d servo(s) no se soltaron (I2C): corta la fuente de 6 V\n", fallidos);
  else responder("OK,servos libres\n");
}

// Llamada durante los movimientos: solo atiende la orden de parada.
// Los demas comandos quedan en una fila y se ejecutan al terminar el movimiento (antes se perdian).
constexpr int MAX_PENDIENTES = 8;
String pendientes[MAX_PENDIENTES];
int nPendientes = 0;

// Los sliders y el jog llegan muchas veces por segundo: de ellos solo importa el ultimo valor de cada
// canal, asi que reemplazan al pendiente del mismo canal en vez de acumularse (si no, al terminar el
// movimiento se ejecutarian valores viejos uno tras otro y el servo "saltaria").
String claveConsigna(const String &l) {
  if (l.length() > 1 && String("BHCMP").indexOf(l[0]) >= 0 && (isdigit(l[1]) || l[1] == ' ')) return l.substring(0, 1);
  if (l.startsWith("obj ")) return l.substring(0, 5);
  return "";
}

bool comprobarParada() {
  String l;
  for (Entrada *e : {&entradaUSB, &entradaBT})
    while (e->leer(l)) {
      if (esParada(l) || esSoltar(l)) {
        pararPedido = true;
        modoAuto = false;
        nPendientes = 0;
        if (esSoltar(l)) {
          soltarTodo();
        }
        continue;
      }
      String k = claveConsigna(l);
      int i = 0;
      while (k.length() && i < nPendientes && claveConsigna(pendientes[i]) != k) i++;
      if (k.length() && i < nPendientes) pendientes[i] = l;
      else if (nPendientes < MAX_PENDIENTES) pendientes[nPendientes++] = l;
    }
  return pararPedido;
}

// ---------------- Telemetria D-H en paralelo ----------------
// Tarea FreeRTOS en el nucleo 0. El nucleo 1 (loop) ejecuta comandos y trayectorias; esta tarea toma
// los pulsos actuales, los pasa a angulos D-H, multiplica las matrices D-H (fkOrigenes) y envia las
// posiciones O1..O4 a la app, tambien en medio de la rutina automatica.
// Cada 200 ms tambien mide con el ToF de la mano y envia D,<mm> (lo que "ve" la pinza).
void tareaTelemetria(void *) {
  int us[NUM_SERVOS] = {0};
  uint32_t tEnvio = 0, tTof = 0;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(PERIODO_TELEM_MS));
    bool hayCliente = telemUSB || (ecoBT && SerialBT.hasClient());
    if (tofOk && hayCliente && millis() - tTof >= 200) {
      tTof = millis();
      char d[16];
      snprintf(d, sizeof(d), "D,%u", leerTof(false));
      xSemaphoreTake(mtxSalida, portMAX_DELAY);
      if (telemUSB) Serial.println(d);
      if (ecoBT && SerialBT.hasClient()) SerialBT.println(d);
      xSemaphoreGive(mtxSalida);
    }
    bool cambio = memcmp(us, usActual, sizeof(us)) != 0;
    if (!cambio && millis() - tEnvio < 1000) continue;
    memcpy(us, usActual, sizeof(us));
    bool conocido = true;
    float q[4], O[5][3];
    for (int ch = 0; ch < 4; ch++) {
      conocido &= us[ch] != 0;
      q[ch] = us[ch] ? qDesdeUs(ch, us[ch]) : 0.0f;
    }
    if (!conocido) continue;   // algun servo sin senal: la pose no se conoce
    fkOrigenes(q, O);
    float pinza = us[CH_PINZA] ? qDesdeUs(CH_PINZA, us[CH_PINZA]) : 0.0f;
    char buf[200];
    int n = snprintf(buf, sizeof(buf), "T,%.1f,%.1f,%.1f,%.1f,%.0f", q[0], q[1], q[2], q[3], pinza);
    for (int i = 1; i <= 4; i++) n += snprintf(buf + n, sizeof(buf) - n, ",%.1f,%.1f,%.1f", O[i][0], O[i][1], O[i][2]);
    tEnvio = millis();
    xSemaphoreTake(mtxSalida, portMAX_DELAY);
    if (telemUSB) Serial.println(buf);
    if (ecoBT && SerialBT.hasClient()) SerialBT.println(buf);
    xSemaphoreGive(mtxSalida);
  }
}

// ---------------- Movimientos cartesianos ----------------
// Lleva la punta a una pose: la IK da los angulos y la trayectoria quintica los recorre.
// Entre el inicio y el destino la punta NO va en linea recta; para eso esta moverLineal().
bool moverCart(const Pose &p) {
  float q[4];
  if (!ik(p, q) || !articulacionesValidas(q)) {
    responder("ERR,inalcanzable x=%.0f y=%.0f z=%.0f phi=%.0f\n", p.x, p.y, p.z, p.phi);
    return false;
  }
  return moverArticular(q);
}

// Recta en el espacio (subir y bajar en vertical): interpola la pose y resuelve la IK en cada paso.
bool moverLineal(const Pose &p1) {
  float q[4];
  if (!leerQ(q)) return moverCart(p1);
  Pose p0 = fk(q);
  if (!ik(p1, q) || !articulacionesValidas(q)) {
    responder("ERR,inalcanzable x=%.0f y=%.0f z=%.0f\n", p1.x, p1.y, p1.z);
    return false;
  }
  float dist = sqrtf(sq(p1.x - p0.x) + sq(p1.y - p0.y) + sq(p1.z - p0.z));
  uint32_t T = max(T_MIN_MS, (uint32_t)(1875.0f * dist / VEL_LINEAL_MM_S));
  uint32_t t0 = millis();
  for (;;) {
    float tau = min(1.0f, (millis() - t0) / (float)T);
    float s = tau * tau * tau * (10.0f - 15.0f * tau + 6.0f * tau * tau);
    Pose p = {p0.x + (p1.x - p0.x) * s, p0.y + (p1.y - p0.y) * s, p0.z + (p1.z - p0.z) * s,
              p0.phi + (p1.phi - p0.phi) * s};
    if (!ik(p, q) || !articulacionesValidas(q)) {
      responder("ERR,trayectoria recta sale del espacio de trabajo\n");
      return false;
    }
    for (int ch = 0; ch < 4; ch++) escribirUs(ch, usDesdeQ(ch, q[ch]));
    if (tau >= 1.0f) return true;
    if (comprobarParada()) return false;
    delay(PERIODO_MS);
  }
}

// Convierte un punto de la mesa (radio y angulo) a pose x, y, z con la pinza vertical.
Pose poseMesa(const PuntoMesa &pm, float z) {
  return {pm.r * cosf(radians(pm.q1)), pm.r * sinf(radians(pm.q1)), z, PHI_AGARRE};
}

// Pose para medir el punto con el ToF: la punta se corre hacia la base para que el haz
// del sensor (que esta TOF_LATERAL mm mas afuera) caiga justo sobre el punto.
Pose poseObservacion(const PuntoMesa &pm) {
  return poseMesa({pm.r - TOF_LATERAL, pm.q1}, Z_OBSERVACION);
}

// Traslado seguro entre puntos de la mesa (retirar - trasladar - aproximar):
// 1) si la punta esta baja, sube en vertical hasta la altura de traslado;
// 2) se traslada a esa altura (interpolacion articular entre dos poses a la misma altura);
// 3) baja en vertical al destino.
// Asi el cubo agarrado o la pinza nunca pasan bajos sobre los otros cubos. Si solo cambia el angulo de la
// base (misma distancia, altura y orientacion), se mueve directo: la punta no cambia de altura.
bool irSeguro(const Pose &p) {
  float q[4];
  if (!leerQ(q)) return moverCart(p);
  Pose a = fk(q);
  bool soloBase = fabsf(hypotf(a.x, a.y) - hypotf(p.x, p.y)) < 3.0f && fabsf(a.z - p.z) < 3.0f && fabsf(a.phi - p.phi) < 3.0f;
  if (soloBase) return moverCart(p);
  float zSeguro = max(Z_TRASLADO, p.z);   // desde HOME (mas alto) se baja directo a esta altura
  bool pinzaVertical = fabsf(a.phi - PHI_AGARRE) < 5.0f;
  if (pinzaVertical && a.z < zSeguro - 1.0f && !moverLineal({a.x, a.y, zSeguro, a.phi})) return false;
  if (!moverCart({p.x, p.y, zSeguro, p.phi})) return false;
  return fabsf(p.z - zSeguro) < 1.0f || moverLineal(p);
}

// HOME: si la pinza esta abajo (vertical y baja), primero sube en vertical para no rozar la mesa al girarla.
bool irHome() {
  float q[4];
  if (leerQ(q)) {
    Pose a = fk(q);
    if (fabsf(a.phi - PHI_AGARRE) < 5.0f && a.z < Z_TRASLADO - 1.0f && !moverLineal({a.x, a.y, Z_TRASLADO, a.phi}))
      return false;
  }
  return moverArticular(Q_HOME) && moverPinza(PINZA_ABIERTA);
}

// ---------------- Sensor ToF ----------------
// El sensor lo usan las dos tareas (la rutina y la telemetria): el mutex evita lecturas cruzadas en Wire1.
// "crudo" = sin la correccion de offset (solo para calibrar el offset).
int16_t tofOffset = 0;   // mm que se suman a cada lectura (comando "tof cal <mm>")
uint16_t leerTof(bool crudo) {
  xSemaphoreTake(mtxTof, portMAX_DELAY);
  uint16_t d = tof.readRangeSingleMillimeters();
  bool fallo = tof.timeoutOccurred() || d >= 8000;
  xSemaphoreGive(mtxTof);
  if (fallo) return 8190;
  return crudo ? d : (uint16_t)max(0, d + tofOffset);
}

// Mediana de LECTURAS_TOF lecturas: una lectura mala aislada no cambia el resultado.
uint16_t medirTof(bool crudo) {
  uint16_t v[LECTURAS_TOF];
  for (int i = 0; i < LECTURAS_TOF; i++) v[i] = leerTof(crudo);
  for (int i = 1; i < LECTURAS_TOF; i++)   // orden por insercion para la mediana
    for (int j = i; j > 0 && v[j] < v[j - 1]; j--) std::swap(v[j], v[j - 1]);
  return v[LECTURAS_TOF / 2];
}

// ---------------- Realimentacion de altura con el ToF ----------------
// El modelo D-H + calibracion predice la altura de la punta, pero el peso, la holgura de los engranajes y
// los errores de calibracion la desvian unos milimetros. En la pose de observacion (pinza vertical, punta a
// Z_OBSERVACION) el sensor deberia ver la mesa a Z_OBSERVACION + TOF_ATRAS. La diferencia con lo medido
// es el error real de altura en ese punto (dz > 0: la punta esta mas arriba de lo que cree el modelo).
// Con la correccion activa, en ese punto se ordena z - dz: el error medido se compensa (lazo externo).
float dzPunto[N_RECOGIDA + 1] = {0};   // N_RECOGIDA puntos de recogida + la pila (ultimo)
bool corregirZ = false;

float zCorregida(int punto, float z) {
  return corregirZ && fabsf(dzPunto[punto]) <= DZ_MAX ? z - dzPunto[punto] : z;
}

// Recupera de la flash lo aprendido con el sensor: referencia de mesa vacia, dz y offset.
void cargarBaseTof() {
  tofAprendido = prefs.getBytesLength("tof") == sizeof(baseTof);
  if (tofAprendido) prefs.getBytes("tof", baseTof, sizeof(baseTof));
  if (prefs.getBytesLength("dz") == sizeof(dzPunto)) prefs.getBytes("dz", dzPunto, sizeof(dzPunto));
  tofOffset = prefs.getShort("tofoff", 0);
  corregirZ = prefs.getBool("corrz", false);
}

// Mide la mesa vacia en un punto y devuelve el error de altura dz.
bool medirPuntoVacio(const PuntoMesa &pm, const char *nombre, uint16_t &d, float &dz) {
  if (!irSeguro(poseObservacion(pm)) || !pausa(300)) return false;
  d = medirTof(false);
  dz = d - (Z_OBSERVACION + TOF_ATRAS);
  responder("OK,%s vacio: %u mm (esperado %.0f) -> error de altura dz = %+.1f mm%s\n", nombre, d,
            Z_OBSERVACION + TOF_ATRAS, dz, fabsf(dz) > DZ_MAX ? "  [demasiado grande: se ignora; revisa TOF_ATRAS y la calibracion]" : "");
  return true;
}

// Comando "aprender": recorre los puntos con la mesa VACIA y guarda la distancia de cada uno.
// Esa distancia es la referencia con la que la rutina distingue CUBO de VACIO.
bool aprenderTof() {
  if (!tofOk) {
    responder("ERR,sensor ToF no disponible\n");
    return false;
  }
  char nombre[12];
  for (int i = 0; i < N_RECOGIDA; i++) {
    snprintf(nombre, sizeof(nombre), "punto %d", i + 1);
    if (!medirPuntoVacio(PUNTOS_RECOGIDA[i], nombre, baseTof[i], dzPunto[i])) return false;
  }
  uint16_t dPila;
  if (!medirPuntoVacio(PUNTO_APILADO, "pila", dPila, dzPunto[N_RECOGIDA])) return false;
  prefs.putBytes("tof", baseTof, sizeof(baseTof));
  prefs.putBytes("dz", dzPunto, sizeof(dzPunto));
  tofAprendido = true;
  responder("Correccion de altura por ToF: %s (comando 'corregir 1' / 'corregir 0')\n", corregirZ ? "ACTIVA" : "inactiva");
  return irHome();
}

// ---------------- Rutina automatica ----------------
// Desde la pose de observacion: avanza en horizontal hasta quedar sobre el cubo, baja en vertical,
// cierra y sube a la altura de traslado. Las alturas de contacto usan la correccion del ToF si esta activa.
bool tomar(int i) {
  const PuntoMesa &pm = PUNTOS_RECOGIDA[i];
  return moverPinza(PINZA_ABIERTA) && moverLineal(poseMesa(pm, Z_OBSERVACION)) &&
         moverLineal(poseMesa(pm, zCorregida(i, TIP_CLEAR))) && moverPinza(PINZA_CERRADA) &&
         moverLineal(poseMesa(pm, Z_TRASLADO));
}

// Lleva el cubo en alto hasta sobre la pila, baja en vertical, suelta y sube.
bool dejar(int nivel) {
  float zDejar = zCorregida(N_RECOGIDA, nivel * CUBO + TIP_CLEAR);
  return irSeguro(poseMesa(PUNTO_APILADO, zDejar + MARGEN_APROX)) && moverLineal(poseMesa(PUNTO_APILADO, zDejar)) &&
         moverPinza(PINZA_ABIERTA) && moverLineal(poseMesa(PUNTO_APILADO, zDejar + MARGEN_APROX));
}

// Rutina automatica (comando "A"), lo que pide el enunciado:
//   1. Va a la pose de observacion de cada punto de recogida y mide con el ToF.
//   2. Si la distancia es al menos media arista menor que la de la mesa vacia, hay un cubo (CUBO).
//   3. Si hay cubo: lo toma y lo apila en el punto final. Si no: sigue con el punto siguiente (VACIO).
//   4. Repite hasta dar una pasada sin encontrar cubos o llenar la pila, y vuelve a HOME.
// Cada paso se informa con una linea E,... que muestran el panel y la app de PC.
// PARAR deja el brazo donde esta; un error o el final lo llevan a HOME.
void rutinaAutomatica() {
  if (!exigirCalibracion(0x1F)) return;
  if (!tofOk) {
    responder("ERR,sensor ToF no disponible\n");
    return;
  }
  if (!tofAprendido) {
    responder("ERR,primero ejecuta 'aprender' con los puntos de recogida vacios\n");
    return;
  }
  responder("E,INICIO\n");
  int nivel = 0;
  bool ok = moverPinza(PINZA_ABIERTA);
  bool hallado = true;
  // Recorre los puntos de recogida hasta que una pasada completa no encuentra cubos o la pila se llena.
  while (ok && modoAuto && hallado && nivel < MAX_PILA) {
    hallado = false;
    for (int i = 0; ok && modoAuto && i < N_RECOGIDA && nivel < MAX_PILA; i++) {
      responder("E,BUSCANDO,%d\n", i + 1);
      ok = irSeguro(poseObservacion(PUNTOS_RECOGIDA[i]));
      if (!ok) break;
      delay(150);
      uint16_t d = medirTof(false);
      bool hay = d + CUBO / 2 < baseTof[i];
      responder("E,DISTANCIA,%d,%u,%u,%s\n", i + 1, d, baseTof[i], hay ? "CUBO" : "VACIO");
      if (!hay) continue;
      hallado = true;
      responder("E,TOMANDO,%d\n", i + 1);
      ok = tomar(i);
      if (!ok) break;
      responder("E,APILANDO,%d\n", nivel + 1);
      ok = dejar(nivel);
      if (ok) nivel++;
    }
  }
  if (pararPedido) {
    responder("E,PARADO\n");   // queda detenido donde esta
  } else {
    responder(ok ? "E,FIN,%d\n" : "E,ERROR,%d\n", nivel);
    irHome();
  }
  modoAuto = false;
}

// ---------------- Verificacion de la tarea (no mueve el brazo) ----------------
// Resuelve la IK de todas las poses que usa la rutina (y de puntos intermedios de cada recta) y las
// compara con los limites calibrados. Asi se sabe antes de mover si un servo quedo montado sin recorrido.
// Acumula el resultado de verificarTarea(): cuantas poses revisa, cuantas fallan
// y el rango de angulos que usa cada articulacion.
struct Revision {
  int poses = 0, fallas = 0;
  float qLo[4] = {1e9f, 1e9f, 1e9f, 1e9f}, qHi[4] = {-1e9f, -1e9f, -1e9f, -1e9f};

  void q(const char *nombre, const float qq[4]) {
    poses++;
    for (int ch = 0; ch < 4; ch++) {
      qLo[ch] = min(qLo[ch], qq[ch]);
      qHi[ch] = max(qHi[ch], qq[ch]);
    }
    for (int ch = 0; ch < 4; ch++)
      if (!qValido(ch, qq[ch])) {
        responder("  FALLA %-12s %s q=%.1f fuera de [%.1f, %.1f]\n", nombre, NOMBRES[ch], qq[ch], qMin(ch), qMax(ch));
        fallas++;
        return;
      }
  }
  void pose(const char *nombre, const Pose &p) {
    float qq[4];
    if (!ik(p, qq)) {
      poses++;
      fallas++;
      responder("  FALLA %-12s fuera del alcance (x=%.0f y=%.0f z=%.0f)\n", nombre, p.x, p.y, p.z);
      return;
    }
    q(nombre, qq);
  }
  void recta(const char *nombre, const Pose &a, const Pose &b, int n = 6) {
    for (int k = 0; k <= n; k++) {
      float s = k / (float)n;
      pose(nombre, {a.x + (b.x - a.x) * s, a.y + (b.y - a.y) * s, a.z + (b.z - a.z) * s, a.phi + (b.phi - a.phi) * s});
    }
  }
};

// Comando "verificar": recorre todas las poses y rectas que usa la rutina, en el mismo orden,
// y compara los angulos con los limites calibrados.
void verificarTarea() {
  Revision rv;
  char nombre[16];
  rv.q("HOME", Q_HOME);
  for (int i = 0; i < N_RECOGIDA; i++) {
    const PuntoMesa &pm = PUNTOS_RECOGIDA[i];
    snprintf(nombre, sizeof(nombre), "observar %d", i + 1);
    rv.pose(nombre, poseObservacion(pm));
    snprintf(nombre, sizeof(nombre), "traslado %d", i + 1);   // altura de paso de irSeguro()
    rv.pose(nombre, poseMesa({pm.r - TOF_LATERAL, pm.q1}, Z_TRASLADO));
    snprintf(nombre, sizeof(nombre), "tomar %d", i + 1);
    rv.recta(nombre, poseObservacion(pm), poseMesa(pm, Z_OBSERVACION));
    rv.recta(nombre, poseMesa(pm, Z_OBSERVACION), poseMesa(pm, TIP_CLEAR));
    rv.recta(nombre, poseMesa(pm, TIP_CLEAR), poseMesa(pm, Z_TRASLADO));
  }
  rv.pose("observar pila", poseObservacion(PUNTO_APILADO));   // la usa 'aprender' para la correccion de altura
  rv.pose("traslado obs pila", poseMesa({PUNTO_APILADO.r - TOF_LATERAL, PUNTO_APILADO.q1}, Z_TRASLADO));
  for (int nivel = 0; nivel < MAX_PILA; nivel++) {
    float z = nivel * CUBO + TIP_CLEAR;
    snprintf(nombre, sizeof(nombre), "apilar %d", nivel + 1);
    rv.pose(nombre, poseMesa(PUNTO_APILADO, max(Z_TRASLADO, z + MARGEN_APROX)));
    rv.recta(nombre, poseMesa(PUNTO_APILADO, z + MARGEN_APROX), poseMesa(PUNTO_APILADO, z), 3);
  }
  responder("Rango que usa la tarea vs rango calibrado [deg D-H]:\n");
  for (int ch = 0; ch < 4; ch++) {
    bool ok = rv.qLo[ch] >= qMin(ch) - 0.05f && rv.qHi[ch] <= qMax(ch) + 0.05f;
    responder("  %-7s necesita [%7.1f, %7.1f]   calibrado [%7.1f, %7.1f]   %s\n", NOMBRES[ch], rv.qLo[ch], rv.qHi[ch],
              qMin(ch), qMax(ch), ok ? "OK" : "FALTA RECORRIDO");
  }
  responder(rv.fallas ? "VERIFICAR: %d de %d poses fallan. Revisa la calibracion o remonta el horn del servo indicado.\n"
                      : "VERIFICAR: las %d poses son alcanzables.\n",
            rv.fallas ? rv.fallas : rv.poses, rv.poses);
}

// ---------------- Prueba de movimiento (sin cubos) ----------------
// Espera ms milisegundos atendiendo PARAR y SOLTAR. Devuelve false si llego una parada.
bool pausa(uint32_t ms) {
  for (uint32_t t0 = millis(); millis() - t0 < ms; delay(10))
    if (comprobarParada()) return false;
  return true;
}

// Fase 1: desde HOME, cada articulacion se mueve +-20 deg (dentro de sus limites) y vuelve, de a una.
// Sirve para ver que cada servo gira hacia el lado correcto segun la convencion D-H.
bool pruebaArticular() {
  responder("E,PRUEBA,articulaciones\n");
  if (!irHome()) return false;
  for (int ch = 0; ch < 4; ch++) {
    for (float dq : {20.0f, -20.0f}) {
      float q[4];
      memcpy(q, Q_HOME, sizeof(q));
      q[ch] = constrain(Q_HOME[ch] + dq, qMin(ch) + 1.0f, qMax(ch) - 1.0f);
      responder("E,PRUEBA,%s q=%.0f\n", NOMBRES[ch], q[ch]);
      if (!moverArticular(q) || !pausa(400)) return false;
    }
    if (!moverArticular(Q_HOME)) return false;
  }
  responder("E,PRUEBA,pinza\n");
  return moverPinza(PINZA_CERRADA) && moverPinza(PINZA_ABIERTA);
}

// Fase 2: la punta visita cada punto de la mesa con la cinematica inversa, baja a la altura de agarre
// (TIP_CLEAR sobre la mesa) y espera 2 s para medir con regla; en la pose de observacion lee el ToF.
bool pruebaPuntos() {
  if (!irHome()) return false;
  for (int i = 0; i < N_RECOGIDA; i++) {
    const PuntoMesa &pm = PUNTOS_RECOGIDA[i];
    if (!irSeguro(poseObservacion(pm)) || !pausa(200)) return false;
    if (tofOk) responder("E,PRUEBA,observar %d,ToF %u mm\n", i + 1, medirTof(false));
    responder("E,PRUEBA,punto %d,la punta debe quedar %.0f mm sobre la marca\n", i + 1, TIP_CLEAR);
    if (!moverLineal(poseMesa(pm, Z_OBSERVACION)) || !moverLineal(poseMesa(pm, zCorregida(i, TIP_CLEAR))) || !pausa(2000) ||
        !moverPinza(PINZA_CERRADA) || !moverPinza(PINZA_ABIERTA) || !moverLineal(poseMesa(pm, Z_TRASLADO)))
      return false;
  }
  responder("E,PRUEBA,pila,la punta debe quedar %.0f mm sobre la marca\n", TIP_CLEAR);
  return irSeguro(poseMesa(PUNTO_APILADO, Z_TRASLADO)) && moverLineal(poseMesa(PUNTO_APILADO, zCorregida(N_RECOGIDA, TIP_CLEAR))) &&
         pausa(2000) && moverLineal(poseMesa(PUNTO_APILADO, Z_TRASLADO)) && irHome();
}

// Tabla D-H con los angulos actuales y la matriz homogenea T0_4 (sistema {0}, sin H_BASE).
void mostrarDH() {
  float q[4], tabla[4][4];
  if (!leerQ(q)) {
    responder("ERR,posicion desconocida: usa 'home' primero\n");
    return;
  }
  tablaDH(q, tabla);
  responder(" i | theta [deg] | d [mm] | a [mm] | alpha [deg]\n");
  Mat4 T = identidad();
  for (int i = 0; i < 4; i++) {
    responder(" %d |  %9.1f  | %6.1f | %6.1f | %6.0f\n", i + 1, tabla[i][0], tabla[i][1], tabla[i][2], tabla[i][3]);
    T = multiplicar(T, matrizDH(tabla[i][0], tabla[i][1], tabla[i][2], tabla[i][3]));
  }
  responder("T0_4 =\n");
  for (int r = 0; r < 4; r++)
    responder("  [%8.3f %8.3f %8.3f %8.1f]\n", T.m[r][0], T.m[r][1], T.m[r][2], T.m[r][3]);
  responder("Punta sobre la mesa: x=%.1f y=%.1f z=%.1f mm, phi=%.1f deg\n", T.m[0][3], T.m[1][3], T.m[2][3] + H_BASE,
            q[1] + q[2] + q[3]);
}

// ---------------- Comandos ----------------
// Comando "scan": busca dispositivos en un bus I2C, para comprobar el cableado.
void escanearBus(TwoWire &bus, const char *nombre) {
  responder("Bus %s:\n", nombre);
  int n = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    bus.beginTransmission(addr);
    if (bus.endTransmission() == 0) {
      const char *disp = addr == 0x40 ? "PCA9685" : addr == 0x29 ? "VL53L0X" : addr == 0x70 ? "PCA9685 (all-call)" : "?";
      responder("  0x%02X  %s\n", addr, disp);
      n++;
    }
  }
  responder("  %d dispositivo(s)\n", n);
}

// Comando "cal": tabla con los puntos A y B, limites y pendiente de cada canal.
// La columna "hecho" muestra a, b, m y M por cada paso de calibracion guardado.
void mostrarCalibracion() {
  responder("canal  nombre   A(q, us)        B(q, us)        limites us     limites q        us/deg  hecho\n");
  for (int ch = 0; ch < NUM_SERVOS; ch++) {
    const CalServo &c = cal[ch];
    uint8_t h = calHecho[ch];
    responder("  %d   %-7s (%6.1f,%5d)  (%6.1f,%5d)  [%4d, %4d]  [%6.1f, %6.1f]  %6.2f  %c%c%c%c\n", ch, NOMBRES[ch], c.qa,
              c.usa, c.qb, c.usb, c.usMin, c.usMax, qMin(ch), qMax(ch), (c.usb - c.usa) / (c.qb - c.qa),
              h & CAL_A ? 'a' : '-', h & CAL_B ? 'b' : '-', h & CAL_MIN ? 'm' : '-', h & CAL_MAX ? 'M' : '-');
  }
}

// Comando "estado": pulso y angulo de cada servo y posicion de los origenes D-H.
void mostrarEstado() {
  for (int ch = 0; ch < NUM_SERVOS; ch++) {
    if (usActual[ch]) responder("  %-7s %5d us  q = %6.1f\n", NOMBRES[ch], usActual[ch], qDesdeUs(ch, usActual[ch]));
    else responder("  %-7s sin senal\n", NOMBRES[ch]);
  }
  float q[4], O[5][3];
  if (!leerQ(q)) return;
  fkOrigenes(q, O);
  for (int i = 0; i <= 4; i++) responder("  O%d = (%6.1f, %6.1f, %6.1f) mm\n", i, O[i][0], O[i][1], O[i][2]);
  responder("  phi = %.1f deg\n", q[1] + q[2] + q[3]);
}

// Calibracion rapida desde la pose de referencia (hombro, codo y muneca, o solo el canal indicado): el brazo
// debe estar en Q_REFERENCIA, sostenido por los servos (llevado con "us" o los sliders). El pulso actual de
// cada uno es su punto A; el punto B esta 45 grados mas alla con la pendiente nominal, y los limites son
// el rango seguro (Q_SEGURO). La base conserva su calibracion; la pinza, si no esta calibrada, toma los
// valores de partida. "cal rapida 3" calibra solo la muneca y no toca hombro ni codo.
void calibracionRapida(int soloCanal) {
  int desde = soloCanal >= 0 ? soloCanal : CH_HOMBRO, hasta = soloCanal >= 0 ? soloCanal : CH_MUNECA;
  if (desde < CH_BASE || hasta > CH_MUNECA) {
    responder("ERR,cal rapida es para base (0), hombro (1), codo (2) o muneca (3)\n");
    return;
  }
  for (int ch = desde; ch <= hasta; ch++)
    if (!usActual[ch]) {
      responder("ERR,%s sin senal: llevalo a la pose de referencia con 'us %d <pulso>' (o el slider) y repite\n",
                NOMBRES[ch], ch);
      return;
    }
  for (int ch = desde; ch <= hasta; ch++) {
    int usa = usActual[ch];
    // Limites: todo el rango del servo; guardarCalibracion() los recorta al rango seguro (Q_SEGURO).
    cal[ch] = {Q_REFERENCIA[ch], Q_REFERENCIA[ch] + 45.0f, (int16_t)usa,
               (int16_t)lroundf(usa + 45.0f * SENTIDO_SERVO[ch] * US_POR_GRADO_NOMINAL), US_ABS_MIN, US_ABS_MAX};
    calHecho[ch] = CAL_TODO;
  }
  if (calHecho[CH_PINZA] != CAL_TODO) {
    cal[CH_PINZA] = CAL_DEFECTO[CH_PINZA];
    calHecho[CH_PINZA] = CAL_TODO;
  }
  guardarCalibracion();
  mostrarCalibracion();
  responder("OK,calibracion rapida guardada. Sigue: 'verificar' y 'prueba 1' (si una articulacion gira al reves: "
            "'cal <canal> invertir')\n");
}

// Interpreta los comandos "cal ...". El punto A o B guarda el pulso ACTUAL del servo junto con el
// angulo real de la pieza; dos puntos definen la recta pulso <-> angulo que usa qDesdeUs() y usDesdeQ().
void comandoCal(const String &l) {
  char a1[16] = {0}, a2[16] = {0};
  float grados = 0;
  int n = sscanf(l.c_str(), "cal %15s %15s %f", a1, a2, &grados);
  if (n <= 0) {
    mostrarCalibracion();
    return;
  }
  if (!strcmp(a1, "reset")) {
    memcpy(cal, CAL_DEFECTO, sizeof(cal));
    memset(calHecho, 0, sizeof(calHecho));
    guardarCalibracion();
    responder("OK,calibracion de partida restaurada\n");
    return;
  }
  if (!strcmp(a1, "rapida")) {
    calibracionRapida(n >= 2 && isdigit(a2[0]) ? atoi(a2) : -1);
    return;
  }
  int ch = atoi(a1);
  if (ch < 0 || ch >= NUM_SERVOS || !isdigit(a1[0])) {
    responder("ERR,canal invalido\n");
    return;
  }
  if (!strcmp(a2, "invertir")) {   // cambia el sentido de giro sin mover A (los limites no cambian)
    cal[ch].usb = 2 * cal[ch].usa - cal[ch].usb;
    guardarCalibracion();
    mostrarCalibracion();
    return;
  }
  if (!usActual[ch]) {
    responder("ERR,primero lleva el canal %d a la posicion con 'us %d <pulso>'\n", ch, ch);
    return;
  }
  esperarSeguimiento(1 << ch, 3000);   // si el jog aun se mueve, se guarda donde realmente queda
  CalServo nueva = cal[ch];
  uint8_t paso;
  if (!strcmp(a2, "a") && n == 3) nueva.qa = grados, nueva.usa = usActual[ch], paso = CAL_A;
  else if (!strcmp(a2, "b") && n == 3) nueva.qb = grados, nueva.usb = usActual[ch], paso = CAL_B;
  else if (!strcmp(a2, "min") || !strcmp(a2, "max") || !strcmp(a2, "lim")) {
    // Con A y B guardados, el lado del limite se deduce solo; asi da igual si el usuario escribe min o max.
    //  - Primer limite: bajo el punto medio de A y B es el minimo; sobre el, el maximo.
    //  - Segundo limite: es el que falta (los dos pueden quedar al mismo lado del punto medio; el orden se
    //    arregla abajo con el intercambio).
    //  - Con ambos guardados: reemplaza al mas cercano, para poder corregir uno de ellos.
    bool conAB = (calHecho[ch] & (CAL_A | CAL_B)) == (CAL_A | CAL_B);
    bool tieneMin = calHecho[ch] & CAL_MIN, tieneMax = calHecho[ch] & CAL_MAX;
    // Repetir la captura sin mover el servo dejaria min = max: el servo quedaria sin recorrido.
    if (conAB && tieneMin != tieneMax && abs(usActual[ch] - (tieneMin ? nueva.usMin : nueva.usMax)) < 50) {
      responder("ERR,este limite esta muy cerca del otro: mueve el servo hacia el lado contrario y repite\n");
      return;
    }
    bool esMin = !strcmp(a2, "min");
    if (conAB) {
      if (tieneMin && tieneMax) esMin = abs(usActual[ch] - nueva.usMin) < abs(usActual[ch] - nueva.usMax);
      else if (tieneMin || tieneMax) esMin = tieneMax;
      else esMin = usActual[ch] < (nueva.usa + nueva.usb) / 2;
    }
    if (esMin) nueva.usMin = usActual[ch], paso = CAL_MIN;
    else nueva.usMax = usActual[ch], paso = CAL_MAX;
  } else {
    responder("ERR,uso: cal <canal> a|b <grados>  o  cal <canal> lim (tambien min|max)\n");
    return;
  }
  if (nueva.usMin > nueva.usMax) std::swap(nueva.usMin, nueva.usMax);
  // El otro punto puede seguir con su valor de fabrica e igualar al que se acaba de guardar (la muneca trae
  // B = -90 y su A tambien es -90). Si todavia no se calibro, queda provisional a 45 grados con pendiente nominal.
  if (paso == CAL_A && !(calHecho[ch] & CAL_B) && (nueva.usa == nueva.usb || nueva.qa == nueva.qb)) {
    nueva.qb = nueva.qa + 45.0f;
    nueva.usb = nueva.usa + 45.0f * US_POR_GRADO_NOMINAL;
  } else if (paso == CAL_B && !(calHecho[ch] & CAL_A) && (nueva.usa == nueva.usb || nueva.qa == nueva.qb)) {
    nueva.qa = nueva.qb - 45.0f;
    nueva.usa = nueva.usb - 45.0f * US_POR_GRADO_NOMINAL;
  }
  if (nueva.usa == nueva.usb || nueva.qa == nueva.qb) {
    responder("ERR,los puntos A y B deben tener pulsos y angulos distintos\n");
    return;
  }
  cal[ch] = nueva;
  calHecho[ch] |= paso;
  guardarCalibracion();
  mostrarCalibracion();
}

// Slider de la app: 0-180 recorre TODO el rango calibrado de ese servo (0 = limite min, 180 = limite max).
// Antes 0-180 iba a 500-2500 us y se recortaba a los limites: en los extremos del slider el servo no se
// movia (zona muerta) y al salir de ella parecia saltar. El seguimiento suave hace el resto.
void slider(int ch, int valor) {
  const CalServo &c = cal[ch];
  fijarObjetivo(ch, c.usMin + (c.usMax - c.usMin) * constrain(valor, 0, 180) / 180, true);
}

// Lista de comandos. Tambien se muestra si llega un comando desconocido.
void ayuda() {
  // Una llamada por linea: responder() tiene un buffer de 256 caracteres y un solo texto largo se cortaba.
  responder("App: B/H/C/M/P<0-180> | A auto | a parar\n");
  responder("home | move x y z [phi] | q q1 q2 q3 q4 | pinza 0-100|abrir|cerrar | estado | dh\n");
  responder("us <ch> <us> | obj <ch> <us> | off | cal | cal <ch> a|b <deg> | cal <ch> lim | cal reset\n");
  responder("verificar | prueba [1|2] | aprender | tof | tof cal <mm> | corregir 1|0 | scan | pca\n");
  responder("pwm <0-15> <us> | vel [us/s] | reposo [s] | bt 1|0 | telem 1|0 | ayuda\n");
  responder("SOLTAR servos (al instante, incluso en movimiento): off | soltar | X\n");
}

// ---------------- Proteccion de los servos ----------------
// El MG90S no informa corriente ni posicion, asi que no se puede "ver" un servo trabado. Se evita lo que
// los quema:
//  1. "us" no pasa de los limites calibrados (ver ejecutar) y la pinza afloja tras cerrar (moverPinza).
//  2. Un servo QUIETO pegado a su limite calibrado mas de T_LIMITE_MS suele estar empujando la estructura
//     o su tope: retrocede RETROCESO_LIMITE_US hacia adentro.
//  3. Reposo: sin ordenes ni movimiento durante reposoS segundos, se sueltan todos (un servo sosteniendo
//     carga por minutos se calienta). Ojo: el brazo baja por su peso; dejarlo apoyado.
uint32_t tUltimaActividad = 0, tPegado[NUM_SERVOS] = {0};
bool pwmActivo = false;   // hay un servo de prueba con pulso en una salida libre ("pwm"): tambien entra al reposo
uint32_t reposoS = REPOSO_DEFECTO_S;

// Se llama en cada ciclo del lazo principal: retrocede los servos pegados al limite y aplica el reposo.
void protegerServos() {
  uint32_t ahora = millis();
  bool alguno = false;
  for (int ch = 0; ch < NUM_SERVOS; ch++) {
    if (!usActual[ch]) {
      tPegado[ch] = 0;
      continue;
    }
    alguno = true;
    bool enMin = usActual[ch] <= cal[ch].usMin + MARGEN_LIMITE_US, enMax = usActual[ch] >= cal[ch].usMax - MARGEN_LIMITE_US;
    if (!limitesCalibrados(ch) || !enObjetivo(ch) || !(enMin || enMax)) {
      tPegado[ch] = 0;
      continue;
    }
    if (!tPegado[ch]) tPegado[ch] = ahora;
    else if (ahora - tPegado[ch] > T_LIMITE_MS) {
      tPegado[ch] = 0;
      if (ch == CH_PINZA && abs(usActual[ch] - cal[ch].usa) <= MARGEN_LIMITE_US) {
        apagar(ch);   // pinza abierta: no necesita fuerza, se suelta en vez de empujar su tope
        continue;
      }
      fijarObjetivo(ch, usActual[ch] + (enMin ? RETROCESO_LIMITE_US : -RETROCESO_LIMITE_US), true);
      responder("AVISO,%s quieto en su limite (%d us): retrocede %d us para no forzar\n", NOMBRES[ch], usActual[ch],
                RETROCESO_LIMITE_US);
    }
  }
  if ((alguno || pwmActivo) && reposoS && !modoAuto && ahora - tUltimaActividad > reposoS * 1000UL) {
    pwmActivo = false;
    soltarTodo();
    responder("AVISO,reposo: %u s sin ordenes, servos liberados para que no se calienten ('reposo 0' lo desactiva)\n",
              (unsigned)reposoS);
    tUltimaActividad = ahora;
  }
}

// Interprete de comandos: recibe una linea y hace lo que pide. Primero revisa los comandos
// de la app (A, parada, sliders) y despues separa la palabra y hasta 4 numeros para los de texto.
// Los movimientos bloquean hasta terminar; mientras tanto solo se atiende PARAR y SOLTAR.
void ejecutar(String l) {
  l.trim();
  if (!l.length()) return;
  pararPedido = false;
  tUltimaActividad = millis();

  if (l == "A") {
    modoAuto = true;   // loop() ejecuta la rutina
    return;
  }
  if (esParada(l)) {
    modoAuto = false;
    detenerSeguimiento();   // los sliders y el jog dejan de seguir su objetivo
    responder("OK,parado\n");
    return;
  }
  int idx = String("BHCMP").indexOf(l[0]);
  if (idx >= 0 && l.length() > 1 && (isdigit(l[1]) || l[1] == ' ')) {
    slider(idx, (int)lroundf(l.substring(1).toFloat()));   // App Inventor envia la posicion con decimales ("B90.0")
    return;
  }

  char cmd[16] = {0};
  float v[4] = {0};
  int n = sscanf(l.c_str(), "%15s %f %f %f %f", cmd, &v[0], &v[1], &v[2], &v[3]);
  String k(cmd);
  bool ok = true;
  if (k == "home") {
    ok = exigirCalibracion(0x1F) && irHome();
  } else if (k == "move" && n >= 4) {
    ok = exigirCalibracion(0x0F) && moverCart({v[0], v[1], v[2], n >= 5 ? v[3] : PHI_AGARRE});
  } else if (k == "q" && n == 5) {
    ok = exigirCalibracion(0x0F) && moverArticular(v);
  } else if (k == "pinza") {
    float cierre = l.indexOf("abrir") > 0 ? PINZA_ABIERTA : l.indexOf("cerrar") > 0 ? PINZA_CERRADA : v[0];
    ok = exigirCalibracion(1 << CH_PINZA) && moverPinza(cierre);
  } else if (k == "estado") {
    mostrarEstado();
  } else if (k == "dh") {
    mostrarDH();
  } else if (k == "us" && n == 3 && v[0] >= 0 && v[0] < NUM_SERVOS) {
    int ch = (int)v[0];
    bool primero = !usActual[ch];
    // Proteccion: con los limites ya calibrados, "us" no pasa de ellos (empujar la estructura quema el servo).
    // Para buscar limites nuevos se usa el asistente ("obj"), que si recorre todo el rango.
    int lo = limitesCalibrados(ch) ? cal[ch].usMin : US_SIN_CAL_MIN, hi = limitesCalibrados(ch) ? cal[ch].usMax : US_SIN_CAL_MAX;
    bool recortar = v[1] < lo || v[1] > hi;
    ok = rampaUs(ch, constrain((int)v[1], lo, hi), limitesCalibrados(ch));
    responder("%s -> %d us (q = %.1f)%s%s\n", NOMBRES[ch], usActual[ch], qDesdeUs(ch, usActual[ch]),
              primero ? "  [primer pulso: aplicado de golpe]" : "",
              recortar ? "  [recortado a su limite seguro]" : "");
  } else if (k == "obj" && n == 3 && v[0] >= 0 && v[0] < NUM_SERVOS) {
    fijarObjetivo((int)v[0], (int)v[1], false);   // jog sin bloquear (asistente de calibracion), sin respuesta
    return;
  } else if (k == "bt") {
    ecoBT = n >= 2 && v[0] != 0;
    responder("OK,envio por Bluetooth (respuestas y telemetria) %s\n", ecoBT ? "activado" : "desactivado");
  } else if (k == "reposo") {
    if (n >= 2) {
      reposoS = (uint32_t)max(0.0f, v[0]);
      prefs.putUInt("reposo", reposoS);
    }
    if (reposoS) responder("OK,reposo: se sueltan los servos tras %u s sin ordenes\n", (unsigned)reposoS);
    else responder("OK,reposo desactivado\n");
  } else if (k == "vel") {
    // Velocidad del seguimiento (sliders, jog del asistente y "us"). No cambia la rutina (trayectorias quinticas).
    if (n >= 2) {
      velSeguimiento = constrain(v[0], 100.0f, 4000.0f);
      prefs.putFloat("vel", velSeguimiento);
    }
    responder("OK,velocidad de seguimiento %.0f us/s (~%.0f grados/s)%s\n", velSeguimiento, velSeguimiento / 11.0f,
              n >= 2 ? ", guardada" : "");
  } else if (k == "pwm" && n == 3 && v[0] >= 0 && v[0] < 16) {
    // Prueba directa de una salida del PCA9685 (0-15), sin calibracion ni movimiento suave: sirve para
    // saber si una salida funciona o si el problema es el servo. "pwm <salida> 0" la deja sin pulso.
    int s = (int)v[0], us = (int)v[1];
    for (int ch = 0; ch < NUM_SERVOS; ch++)
      if (CANAL_PCA[ch] == s) {
        responder("ERR,la salida %d es de %s: usa 'us %d <us>'\n", s, NOMBRES[ch], ch);
        return;
      }
    us = us <= 0 ? 0 : constrain(us, US_ABS_MIN, US_ABS_MAX);
    ok = pcaCanal(s, us ? (uint16_t)lroundf(us / usPorTick) : 4096);
    if (ok && us) pwmActivo = true;
    if (ok) {
      if (us) responder("salida %d -> %d us\n", s, us);
      else responder("salida %d -> sin pulso\n", s);
    }
  } else if (k == "verificar") {
    verificarTarea();
  } else if (k == "prueba") {
    int fase = n >= 2 ? (int)v[0] : 0;
    ok = exigirCalibracion(0x1F) && (fase == 2 || pruebaArticular()) && (fase == 1 || pruebaPuntos());
    if (ok) responder("E,PRUEBA,OK\n");
  } else if (k == "telem") {
    telemUSB = n >= 2 && v[0] != 0;
    responder("OK,telemetria USB %s\n", telemUSB ? "activada" : "desactivada");
  } else if (esSoltar(l)) {
    soltarTodo();
  } else if (k == "cal") {
    comandoCal(l);
  } else if (k == "aprender") {
    ok = exigirCalibracion(0x1F) && aprenderTof();
  } else if (k == "tof" && !tofOk) {
    responder("ERR,sensor ToF no disponible\n");
  } else if (k == "tof" && l.indexOf("cal") > 0) {
    // Offset del sensor: pon una superficie plana a una distancia conocida (regla) frente al sensor.
    float real = 0;
    if (sscanf(l.c_str(), "tof cal %f", &real) != 1 || real < 30 || real > 500) {
      responder("ERR,uso: tof cal <mm reales, 30-500>\n");
    } else {
      uint16_t crudo = medirTof(true);
      tofOffset = (int16_t)lroundf(real - crudo);
      prefs.putShort("tofoff", tofOffset);
      responder("OK,ToF mide %u mm crudos; offset guardado = %+d mm\n", crudo, tofOffset);
    }
  } else if (k == "tof") {
    responder("ToF: %u mm (mediana de %d, offset %+d mm)\n", medirTof(false), LECTURAS_TOF, tofOffset);
  } else if (k == "corregir") {
    corregirZ = n >= 2 && v[0] != 0;
    prefs.putBool("corrz", corregirZ);
    responder("OK,correccion de altura por ToF %s | dz: p1 %+.1f  p2 %+.1f  p3 %+.1f  pila %+.1f mm\n",
              corregirZ ? "ACTIVA" : "inactiva", dzPunto[0], dzPunto[1], dzPunto[2], dzPunto[N_RECOGIDA]);
  } else if (k == "scan") {
    escanearBus(Wire, "PCA9685 (GPIO21 SDA / GPIO22 SCL)");
    xSemaphoreTake(mtxTof, portMAX_DELAY);
    escanearBus(Wire1, "ToF (GPIO18 SDA / GPIO19 SCL)");
    xSemaphoreGive(mtxTof);
    responder("Errores I2C al escribir servos desde el arranque: %u\n", erroresI2C);
  } else if (k == "pca") {
    // Diagnostico del driver: relee del chip la configuracion y el pulso de cada canal y lo compara
    // con lo que el firmware cree haber enviado. "OK" = el chip tiene exactamente lo escrito.
    uint8_t m1, m2, pre;
    if (!pcaLeer(PCA_MODE1, &m1, 1) || !pcaLeer(PCA_MODE2, &m2, 1) || !pcaLeer(PCA_PRESCALE, &pre, 1)) {
      responder("ERR,el PCA9685 no responde\n");
      return;
    }
    responder("MODE1 0x%02X (%s) | MODE2 0x%02X | PRESCALE %u (esperado %u -> %.1f Hz)\n", m1,
              (m1 & MODE1_SLEEP) ? "DORMIDO" : "activo", m2, pre, PRESCALE_SERVO, OSC_PCA_HZ / (4096.0f * (pre + 1)));
    for (int ch = 0; ch < NUM_SERVOS; ch++) {
      uint8_t r[4];
      if (!pcaLeer(PCA_LED0 + 4 * CANAL_PCA[ch], r, 4)) {
        responder("  canal %d  ERR lectura\n", ch);
        continue;
      }
      bool libre = r[3] & 0x10;
      uint16_t off = r[2] | (r[3] & 0x0F) << 8;
      bool coincide = libre ? usActual[ch] == 0 : (r[0] == 0 && r[1] == 0 && off == ticksActual[ch]);
      if (libre) responder("  canal %d %-7s chip: sin pulso        firmware: %s  %s\n", ch, NOMBRES[ch],
                           usActual[ch] ? "con pulso" : "sin pulso", coincide ? "OK" : "DISTINTO");
      else responder("  canal %d %-7s chip: %4u ticks = %6.1f us  firmware: %4u ticks  %s\n", ch, NOMBRES[ch], off,
                     off * usPorTick, ticksActual[ch], coincide ? "OK" : "DISTINTO");
    }
  } else {
    ayuda();
    return;
  }
  if (pararPedido) responder("OK,parado\n");
  else if (!ok) responder("ERR,comando no completado\n");
}

// ---------------- Arranque y lazo principal ----------------
// Arranque: inicia comunicaciones e I2C, recupera los servos si el PCA9685 ya estaba activo, carga la
// calibracion de la flash, inicia el sensor y lanza la tarea de telemetria en el nucleo 0.
void setup() {
  mtxSalida = xSemaphoreCreateMutex();
  mtxTof = xSemaphoreCreateMutex();
  Serial.begin(115200);
  SerialBT.begin(NOMBRE_BT);
  delay(300);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(I2C_HZ);
  Wire1.begin(PIN_TOF_SDA, PIN_TOF_SCL);
  Wire1.setClock(I2C_HZ);

  int recuperados = iniciarPCA();
  bool pcaOk = recuperados >= 0;
  cargarCalibracion();
  reposoS = prefs.getUInt("reposo", REPOSO_DEFECTO_S);
  tUltimaActividad = millis();
  tof.setBus(&Wire1);
  tof.setTimeout(500);
  tofOk = tof.init();
  cargarBaseTof();

  responder("\n=== Brazo robotico 4 GDL ===\nBluetooth: %s | PCA9685: %s (%.2f us/tick) | ToF: %s | referencia ToF: %s\n",
            NOMBRE_BT, pcaOk ? "OK" : "NO responde", usPorTick, tofOk ? "OK" : "NO responde",
            tofAprendido ? "aprendida" : "falta 'aprender'");
  // Un servo recuperado que quedo FUERA de su rango seguro (por ejemplo empujando la estructura antes del
  // reinicio) se suelta en vez de seguir sosteniendolo ahi.
  for (int ch = 0; ch < NUM_SERVOS; ch++) {
    int us = usActual[ch];
    if (us && (us < cal[ch].usMin || us > cal[ch].usMax) && apagar(ch)) {
      recuperados--;
      responder("AVISO,%s estaba fuera de su rango seguro (%d us): se suelta\n", NOMBRES[ch], us);
    }
  }
  if (recuperados > 0)
    responder("PCA9685 ya estaba activo: se mantienen %d servo(s) en su posicion (no se sueltan al reiniciar).\n",
              recuperados);
  else
    responder("Los servos estan libres. Sujeta el brazo cerca de HOME y escribe 'home'.\n");
  exigirCalibracion(0x1F);
  ayuda();
  xTaskCreatePinnedToCore(tareaTelemetria, "telemetria", 4096, nullptr, 1, nullptr, 0);
}

// Lazo principal (nucleo 1): ejecuta un comando pendiente y lo nuevo que llegue por USB o Bluetooth,
// corre la rutina si esta activa y, cada PERIODO_MS, actualiza el seguimiento suave y las protecciones.
void loop() {
  String l;
  if (nPendientes) {
    l = pendientes[0];
    for (int i = 1; i < nPendientes; i++) pendientes[i - 1] = pendientes[i];
    nPendientes--;
    ejecutar(l);
  }
  if (entradaUSB.leer(l)) ejecutar(l);
  if (entradaBT.leer(l)) ejecutar(l);

  if (modoAuto) {
    pararPedido = false;
    rutinaAutomatica();
    modoAuto = false;   // tambien si la rutina no pudo partir: si no, se llamaria sin parar
  }

  static uint32_t tServo = 0;
  if (millis() - tServo >= PERIODO_MS) {
    tServo = millis();
    if (servicioSeguimiento() || modoAuto) tUltimaActividad = millis();
    protegerServos();
  }
}
