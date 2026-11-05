// Simulacion del firmware REAL: compila firmware/src/main.cpp sin cambios, sobre el hardware emulado de la
// carpeta emulador/, y le envia los mismos comandos que recibiria por USB o Bluetooth.
//
// El "mundo" simulado es ideal: servos que llegan exacto, sin holguras, peso ni deslizamiento de los horns.
// Por eso comprueba la LOGICA (calibracion, D-H, deteccion, rutina, seguridad), no el comportamiento mecanico.
//
// Salida: simulacion/resultados/resumen.txt (cada prueba), rutina_3_cubos.csv (movimiento) y consola.txt.
// Compilar y ejecutar: ver simulacion/README.md

#include "../firmware/src/main.cpp"

#include <fstream>

// ---------------- Mundo: los cubos sobre la mesa y lo que le llega a cada servo ----------------
struct Cubo { float x, y, z; bool tomado; };
std::vector<Cubo> cubos;
float zMinPunta = 1e9f, qMinVisto[4], qMaxVisto[4];
int choquesCubos = 0;
std::ofstream csv;
uint32_t proximaFila = 0;
std::vector<std::pair<uint32_t, std::string>> ordenesProgramadas;

// Pulso que el chip entrega al canal, leido de sus registros; 0 si esta sin pulso.
float pulsoEnChip(int ch) {
  const uint8_t *r = &Wire.registros[PCA_LED0 + 4 * CANAL_PCA[ch]];
  if (r[3] & 0x10) return 0.0f;
  return (r[2] | (r[3] & 0x0F) << 8) * usPorTick;
}

bool poseDelChip(float q[4]) {
  for (int ch = 0; ch < 4; ch++) {
    float us = pulsoEnChip(ch);
    if (us == 0.0f) return false;
    q[ch] = qDesdeUs(ch, us);
  }
  return true;
}

float cierrePinza() {
  float us = pulsoEnChip(CH_PINZA);
  return us == 0.0f ? 0.0f : qDesdeUs(CH_PINZA, us);
}

// Se llama en cada milisegundo simulado: actualiza los cubos, registra el movimiento y entrega ordenes.
void alPasarElTiempo() {
  for (size_t i = 0; i < ordenesProgramadas.size();) {
    if (relojMs >= ordenesProgramadas[i].first) {
      Serial.enviar(ordenesProgramadas[i].second);
      ordenesProgramadas.erase(ordenesProgramadas.begin() + i);
    } else {
      i++;
    }
  }
  if (!cal[0].usb) return;   // todavia no se carga la calibracion
  float q[4];
  if (!poseDelChip(q)) {
    for (auto &c : cubos) c.tomado = false;
    return;
  }
  Pose punta = fk(q);
  float cierre = cierrePinza();
  zMinPunta = min(zMinPunta, punta.z);
  for (int j = 0; j < 4; j++) {
    qMinVisto[j] = min(qMinVisto[j], q[j]);
    qMaxVisto[j] = max(qMaxVisto[j], q[j]);
  }
  Cubo *llevado = nullptr;
  for (auto &c : cubos) {
    if (!c.tomado && cierre > 50.0f && hypotf(c.x - punta.x, c.y - punta.y) < 8.0f && fabsf(c.z + TIP_CLEAR - punta.z) < 5.0f) c.tomado = true;
    if (c.tomado && cierre <= 50.0f) {   // al soltarlo, cae sobre la mesa o sobre el cubo que tenga debajo
      c.tomado = false;
      float apoyo = 0.0f;
      for (auto &o : cubos)
        if (&o != &c && hypotf(o.x - c.x, o.y - c.y) < CUBO && o.z + CUBO <= c.z + 3.0f) apoyo = max(apoyo, o.z + CUBO);
      c.z = apoyo;
    }
    if (c.tomado) {
      c.x = punta.x;
      c.y = punta.y;
      c.z = punta.z - TIP_CLEAR;
      llevado = &c;
    }
  }
  if (llevado)
    for (auto &o : cubos)
      if (&o != llevado && hypotf(o.x - llevado->x, o.y - llevado->y) < CUBO - 1.0f && llevado->z < o.z + CUBO - 1.0f && llevado->z + CUBO > o.z) choquesCubos++;
  if (csv.is_open() && relojMs >= proximaFila) {
    proximaFila = relojMs + 50;
    csv << relojMs << ',' << q[0] << ',' << q[1] << ',' << q[2] << ',' << q[3] << ',' << cierre << ',' << punta.x << ',' << punta.y << ',' << punta.z;
    for (size_t i = 0; i < 3; i++) {
      if (i < cubos.size()) csv << ',' << cubos[i].x << ',' << cubos[i].y << ',' << cubos[i].z;
      else csv << ",,,";
    }
    csv << '\n';
  }
}

// Distancia que veria el sensor: va TOF_ATRAS detras de la punta y TOF_LATERAL mas afuera que ella.
uint16_t distanciaSensor() {
  float q[4];
  if (!poseDelChip(q)) return 8190;
  Pose p = fk(q);
  float r = hypotf(p.x, p.y), ux = r > 1.0f ? p.x / r : 1.0f, uy = r > 1.0f ? p.y / r : 0.0f;
  float x = p.x + ux * TOF_LATERAL, y = p.y + uy * TOF_LATERAL, z = p.z + TOF_ATRAS, superficie = 0.0f;
  for (auto &c : cubos)
    if (!c.tomado && fabsf(c.x - x) < CUBO / 2 && fabsf(c.y - y) < CUBO / 2) superficie = max(superficie, c.z + CUBO);
  return (uint16_t)lroundf(z - superficie);
}

// ---------------- Herramientas de las pruebas ----------------
std::ofstream resumen;
int aprobadas = 0, falladas = 0;

void prueba(const std::string &nombre, bool ok, const std::string &detalle = "") {
  (ok ? aprobadas : falladas)++;
  std::string linea = std::string(ok ? "[OK ] " : "[MAL] ") + nombre + (detalle.empty() ? "" : "  (" + detalle + ")");
  printf("%s\n", linea.c_str());
  resumen << linea << "\n";
}

std::string num(float v, const char *f = "%.1f") {
  char t[32];
  snprintf(t, sizeof(t), f, v);
  return t;
}

// Ejecuta el lazo principal hasta que no quede nada que hacer: sin ordenes, sin rutina y con los servos quietos.
void correrHastaTerminar(uint32_t maxMs = 400000) {
  uint32_t fin = relojMs + maxMs, quieto = 0;
  while (relojMs < fin && quieto < 300) {
    loop();
    delay(1);
    bool ocupado = Serial.available() || nPendientes || modoAuto || !ordenesProgramadas.empty();
    for (int ch = 0; ch < NUM_SERVOS; ch++) ocupado |= !enObjetivo(ch);
    quieto = ocupado ? 0 : quieto + 1;
  }
}

// Envia un comando y devuelve lo que respondio el firmware.
std::string orden(const std::string &c) {
  size_t desde = Serial.salida.size();
  Serial.enviar(c + "\n");
  correrHastaTerminar();
  return Serial.salida.substr(desde);
}

bool contiene(const std::string &t, const std::string &b) { return t.find(b) != std::string::npos; }
int veces(const std::string &t, const std::string &b) {
  int n = 0;
  for (size_t p = t.find(b); p != std::string::npos; p = t.find(b, p + 1)) n++;
  return n;
}

Cubo cuboEnPunto(const PuntoMesa &pm) { return {pm.r * cosf(radians(pm.q1)), pm.r * sinf(radians(pm.q1)), 0.0f, false}; }
void reiniciarMarcas() {
  zMinPunta = 1e9f;
  choquesCubos = 0;
  for (int j = 0; j < 4; j++) qMinVisto[j] = 1e9f, qMaxVisto[j] = -1e9f;
}
bool enHome() {
  float q[4];
  if (!poseDelChip(q)) return false;
  for (int j = 0; j < 4; j++)
    if (fabsf(q[j] - Q_HOME[j]) > 0.5f) return false;
  return true;
}

// Calibra como lo haria el usuario: lleva cada servo al pulso y guarda A, B y los dos limites.
// Son los valores que tiene hoy la ESP32 del brazo (salida del comando "cal", 05-11-2026).
void calibrar() {
  struct { int ch, a; float qa; int b; float qb; int lo, hi; } t[] = {
      {0, 1400, 0, 1850, 45, 850, 2350},   {1, 1200, 90, 750, 135, 1100, 1700}, {2, 700, -90, 1150, -45, 650, 1450},
      {3, 1500, -90, 1050, -45, 1150, 1850}, {4, 1200, 0, 1800, 100, 1200, 1800}};
  for (auto &c : t) {
    char b[48];
    auto mandar = [&](const char *f, auto... v) { snprintf(b, sizeof(b), f, v...); orden(b); };
    mandar("us %d 1500", c.ch);
    mandar("obj %d %d", c.ch, c.a);
    mandar("cal %d a %.0f", c.ch, c.qa);
    mandar("obj %d %d", c.ch, c.b);
    mandar("cal %d b %.0f", c.ch, c.qb);
    mandar("obj %d %d", c.ch, c.lo);
    mandar("cal %d lim", c.ch);
    mandar("obj %d %d", c.ch, c.hi);
    mandar("cal %d lim", c.ch);
    mandar("obj %d %d", c.ch, c.a);
  }
}

int main() {
  ganchoTiempo = alPasarElTiempo;
  ganchoDistancia = distanciaSensor;
  resumen.open("simulacion/resultados/resumen.txt");
  setup();
  prueba("arranca y encuentra PCA9685 y sensor", contiene(Serial.salida, "PCA9685: OK") && contiene(Serial.salida, "ToF: OK"));

  // --- Calibracion y verificacion ---
  std::string r = orden("q 0 88 -88 -90");
  prueba("sin calibrar no se mueve", contiene(r, "falta calibrar"));
  calibrar();
  r = orden("cal");
  prueba("calibracion completa en los 5 canales", veces(r, "abmM") == NUM_SERVOS);
  r = orden("verificar");
  prueba("verificar: poses de la tarea alcanzables", contiene(r, "son alcanzables"));

  // --- Rutina sin haber aprendido la mesa: error una sola vez (correccion del 05-11-2026) ---
  r = orden("A");
  prueba("A sin 'aprender' da un solo error y se apaga", veces(r, "primero ejecuta") == 1 && !modoAuto, std::to_string(veces(r, "primero ejecuta")) + " error(es)");

  // --- HOME y D-H ---
  orden("home");
  prueba("home lleva el brazo a Q_HOME", enHome());
  r = orden("dh");
  prueba("dh muestra la matriz T0_4", contiene(r, "T0_4") && contiene(r, "Punta sobre la mesa"));

  // --- Telemetria: la tarea paralela en el nucleo 0 ---
  telemUSB = true;
  vueltasTarea = 1;
  size_t desde = Serial.salida.size();
  try { tareaGuardada(nullptr); } catch (FinTarea &) {}
  telemUSB = false;
  std::string t = Serial.salida.substr(desde);
  size_t pt = t.find("T,");
  prueba("telemetria T con 17 valores", pt != std::string::npos && veces(t.substr(pt, t.find('\n', pt) - pt), ",") == 17);
  prueba("telemetria D con la distancia", contiene(t, "D,"));

  // --- Aprender la mesa vacia ---
  cubos.clear();
  r = orden("aprender");
  prueba("aprender mide la mesa vacia", tofAprendido && fabsf((float)baseTof[0] - (Z_OBSERVACION + TOF_ATRAS)) < 2.0f, "referencia " + std::to_string(baseTof[0]) + " mm");
  prueba("error de altura nulo en mundo ideal", fabsf(dzPunto[0]) < 2.0f, "dz " + num(dzPunto[0]) + " mm");

  // --- Rutina sin cubos: vuelve a la posicion inicial ---
  r = orden("A");
  prueba("sin cubos: los 3 puntos VACIO, E,FIN,0 y vuelve a HOME", veces(r, "VACIO") == N_RECOGIDA && contiene(r, "E,FIN,0") && enHome());

  // --- Cubo fuera de su punto: no intenta agarrarlo y vuelve ---
  cubos = {cuboEnPunto(PUNTOS_RECOGIDA[1])};
  cubos[0].x += 30.0f;
  r = orden("A");
  prueba("cubo corrido 30 mm: no lo detecta y vuelve a HOME", contiene(r, "E,FIN,0") && enHome() && cubos[0].z == 0.0f);

  // --- Un cubo ---
  cubos = {cuboEnPunto(PUNTOS_RECOGIDA[1])};
  r = orden("A");
  Cubo pilaEsperada = cuboEnPunto(PUNTO_APILADO);
  prueba("un cubo: lo detecta, lo lleva a la pila y vuelve a HOME",
         contiene(r, "CUBO") && contiene(r, "E,FIN,1") && enHome() && hypotf(cubos[0].x - pilaEsperada.x, cubos[0].y - pilaEsperada.y) < 2.0f && fabsf(cubos[0].z) < 6.0f,
         "queda en x=" + num(cubos[0].x) + " y=" + num(cubos[0].y) + " z=" + num(cubos[0].z));

  // --- Rutina completa: tres cubos apilados, con registro del movimiento ---
  cubos = {cuboEnPunto(PUNTOS_RECOGIDA[0]), cuboEnPunto(PUNTOS_RECOGIDA[1]), cuboEnPunto(PUNTOS_RECOGIDA[2])};
  reiniciarMarcas();
  csv.open("simulacion/resultados/rutina_3_cubos.csv");
  csv << "t_ms,q1,q2,q3,q4,pinza,x,y,z,c1x,c1y,c1z,c2x,c2y,c2z,c3x,c3y,c3z\n";
  uint32_t t0 = relojMs;
  r = orden("A");
  csv.close();
  std::ofstream("simulacion/resultados/eventos_rutina.txt") << r;
  bool apilados = true;
  std::string alturas;
  std::vector<float> zs;
  for (auto &c : cubos) {
    apilados &= hypotf(c.x - pilaEsperada.x, c.y - pilaEsperada.y) < 2.0f;
    zs.push_back(c.z);
    alturas += num(c.z) + " ";
  }
  std::sort(zs.begin(), zs.end());
  // Cada cubo se suelta a 0,5 mm o menos de su apoyo: se revisa cada nivel con 2 mm de tolerancia.
  for (int i = 0; i < 3; i++) apilados &= fabsf(zs[i] - i * CUBO) < 2.0f;
  prueba("tres cubos: quedan apilados en el punto final", contiene(r, "E,FIN,3") && apilados, "alturas " + alturas + "mm");
  prueba("termina en HOME", enHome());
  prueba("la punta nunca baja de la mesa", zMinPunta >= 0.0f, "minimo " + num(zMinPunta) + " mm");
  prueba("el cubo llevado no choca con otros", choquesCubos == 0);
  bool dentro = true;
  std::string rangos;
  for (int j = 0; j < 4; j++) {
    dentro &= qMinVisto[j] >= Q_SEGURO[j][0] - 0.5f && qMaxVisto[j] <= Q_SEGURO[j][1] + 0.5f;
    rangos += "q" + std::to_string(j + 1) + " [" + num(qMinVisto[j]) + ", " + num(qMaxVisto[j]) + "] ";
  }
  prueba("todas las articulaciones dentro del rango seguro", dentro, rangos);
  resumen << "      duracion de la rutina de 3 cubos: " << num((relojMs - t0) / 1000.0f) << " s\n";
  printf("      duracion de la rutina de 3 cubos: %s s\n", num((relojMs - t0) / 1000.0f).c_str());

  // --- Repetible: sin cubos en los puntos, la pila no se altera ---
  r = orden("A");
  prueba("repetir sin cubos en recogida no altera la pila", contiene(r, "E,FIN,0"));

  // --- PARAR y SOLTAR en medio de la rutina ---
  cubos = {cuboEnPunto(PUNTOS_RECOGIDA[1])};
  ordenesProgramadas.push_back({relojMs + 3000, "a\n"});
  r = orden("A");
  float qA[4], qB[4];
  bool hayA = poseDelChip(qA);
  delay(1000);
  bool hayB = poseDelChip(qB);
  prueba("PARAR detiene la rutina y mantiene la fuerza", contiene(r, "E,PARADO") && !modoAuto && hayA && hayB && fabsf(qA[1] - qB[1]) < 0.01f);
  orden("home");
  ordenesProgramadas.push_back({relojMs + 2000, "X\n"});
  r = orden("A");
  bool todoLibre = true;
  for (int ch = 0; ch < NUM_SERVOS; ch++) todoLibre &= pulsoEnChip(ch) == 0.0f;
  prueba("SOLTAR corta los pulsos en el chip, a mitad de la rutina", contiene(r, "OK,servos libres") && todoLibre && !modoAuto);

  // --- Limites ---
  cubos.clear();
  orden("home");
  r = orden("q 0 88 -88 0");
  prueba("rechaza una pose fuera de limites", contiene(r, "fuera de limites") && enHome());
  r = orden("move 200 0 5");
  prueba("rechaza un punto fuera de alcance", contiene(r, "inalcanzable") && enHome());
  r = orden("us 2 2400");
  prueba("us se recorta al limite seguro", contiene(r, "recortado") && pulsoEnChip(2) < cal[2].usMax + 6.0f, "codo a " + num(pulsoEnChip(2), "%.0f") + " us");
  orden("home");

  // --- Sliders de la app: recorren el rango calibrado y solo vale el ultimo valor ---
  orden("C0");
  float bajo = pulsoEnChip(2);
  orden("C180");
  float alto = pulsoEnChip(2);
  prueba("slider 0-180 recorre el rango del codo", fabsf(bajo - cal[2].usMin) < 6.0f && fabsf(alto - cal[2].usMax) < 6.0f, num(bajo, "%.0f") + " a " + num(alto, "%.0f") + " us");
  orden("C90B90");   // sliders pegados, sin fin de linea
  prueba("comandos pegados de la app se separan", fabsf(pulsoEnChip(2) - (cal[2].usMin + cal[2].usMax) / 2.0f) < 6.0f && fabsf(pulsoEnChip(0) - (cal[0].usMin + cal[0].usMax) / 2.0f) < 6.0f);
  orden("C45.0");    // App Inventor envia decimales
  prueba("slider con decimales de App Inventor", fabsf(pulsoEnChip(2) - (cal[2].usMin + (cal[2].usMax - cal[2].usMin) / 4.0f)) < 6.0f);
  orden("home");

  // --- Protecciones de los servos ---
  r = orden("pinza cerrar");
  prueba("la pinza afloja tras cerrar sin soltar", !contiene(r, "ERR") && pulsoEnChip(CH_PINZA) > 0.0f && fabsf(cierrePinza() - (PINZA_CERRADA - PINZA_ALIVIO)) < 1.5f, "cierre " + num(cierrePinza()) + " %");
  r = orden("pinza abrir");
  prueba("la pinza se suelta al quedar abierta", pulsoEnChip(CH_PINZA) == 0.0f);
  orden("us 2 650");
  float antes = pulsoEnChip(2);
  desde = Serial.salida.size();
  for (uint32_t fin = relojMs + 4500; relojMs < fin;) { loop(); delay(1); }
  prueba("servo pegado a su limite retrocede", contiene(Serial.salida.substr(desde), "retrocede") && pulsoEnChip(2) > antes + 10.0f, num(antes, "%.0f") + " a " + num(pulsoEnChip(2), "%.0f") + " us");
  orden("home");
  desde = Serial.salida.size();
  for (uint32_t fin = relojMs + (REPOSO_DEFECTO_S + 2) * 1000; relojMs < fin;) { loop(); delay(1); }
  bool libres = true;
  for (int ch = 0; ch < NUM_SERVOS; ch++) libres &= pulsoEnChip(ch) == 0.0f;
  prueba("reposo: sin ordenes suelta todos los servos", contiene(Serial.salida.substr(desde), "AVISO,reposo") && libres);

  // --- Repetir la captura de un limite no deja al servo sin recorrido ---
  orden("cal reset");
  orden("us 1 1500");
  orden("obj 1 1200");
  orden("cal 1 a 90");
  orden("obj 1 750");
  orden("cal 1 b 135");
  orden("obj 1 1100");
  orden("cal 1 lim");
  r = orden("cal 1 lim");
  prueba("limite repetido: se rechaza y sigue incompleto", contiene(r, "muy cerca") && !limitesCalibrados(1));
  orden("obj 1 1700");
  orden("cal 1 lim");
  prueba("limites al mismo lado del punto medio quedan bien", limitesCalibrados(1) && cal[1].usMin == 1100 && cal[1].usMax == 1700, std::to_string(cal[1].usMin) + " a " + std::to_string(cal[1].usMax) + " us");

  std::ofstream("simulacion/resultados/consola.txt") << Serial.salida;
  std::string total = "RESULTADO: " + std::to_string(aprobadas) + " de " + std::to_string(aprobadas + falladas) + " pruebas correctas";
  printf("\n%s\n", total.c_str());
  resumen << "\n" << total << "\n";
  return falladas ? 1 : 0;
}
