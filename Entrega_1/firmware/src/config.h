// Parametros del brazo: pines, geometria D-H, tarea de apilado y movimiento.
// Es la unica fuente de estas medidas: analisis/dh_brazo.py y la app de PC las leen de aqui, y
// panel/sincronizar_panel.py las copia al panel. Si cambia algo, correr ese script.
#pragma once
#include <Arduino.h>

// ---------------- Pines (ESP32 DevKit) y direcciones ----------------
constexpr int PIN_SDA = 21;            // bus I2C 1 (Wire): PCA9685
constexpr int PIN_SCL = 22;
constexpr int PIN_TOF_SDA = 18;        // bus I2C 2 (Wire1): VL53L0X
constexpr int PIN_TOF_SCL = 19;
constexpr uint8_t PCA_ADDR = 0x40;
constexpr const char *NOMBRE_BT = "Brazo_Robotico_ESP32";   // Bluetooth clasico (app de App Inventor)

// ---------------- Canales del PCA9685 ----------------
enum Canal { CH_BASE = 0, CH_HOMBRO, CH_CODO, CH_MUNECA, CH_PINZA, NUM_SERVOS };
const char *const NOMBRES[NUM_SERVOS] = {"base", "hombro", "codo", "muneca", "pinza"};
// Salida del PCA9685 (0-15) de cada articulacion. Si una salida falla, se conecta el servo en otra,
// se cambia aqui y se recalibra esa articulacion. Para probar salidas sueltas: comando "pwm <salida> <us>".
constexpr uint8_t CANAL_PCA[NUM_SERVOS] = {0, 1, 2, 3, 4};

// ---------------- Geometria D-H [mm] (ver 01_Analisis_Cinematico_Dinamico_y_Plan.md) ----------------
constexpr float L1 = 32.0f;     // cara superior de la base giratoria -> eje del hombro
constexpr float L2 = 67.0f;     // hombro -> codo
constexpr float L3 = 60.4f;     // codo -> muneca
constexpr float L4 = 85.0f;     // muneca -> punta de la pinza
constexpr float A1_DH = -1.2f;     // desplazamiento horizontal del hombro respecto del eje de la base
constexpr float H_BASE = 35.0f; // mesa -> sistema {0}

// Pose de referencia [grados D-H]: base al frente, eslabon 2 vertical, eslabon 3 horizontal (L) y pinza
// hacia ABAJO. Es la pose que se usa para montar los horns centrados y para "cal rapida".
constexpr float Q_REFERENCIA[4] = {0.0f, 90.0f, -90.0f, -90.0f};
// HOME = la referencia, 2 grados hacia adentro en hombro y codo: sus horns quedaron montados con la
// referencia justo en el extremo del recorrido (600 us), y asi el servo no queda empujando su tope.
constexpr float Q_HOME[4] = {0.0f, 88.0f, -88.0f, -90.0f};

// Calibracion rapida ("cal rapida [canal]"): con el brazo en la pose de referencia, el pulso que tiene cada
// servo es su punto A, y el resto sale de la pendiente nominal del MG90S/SG90 (~180 grados en 600-2400 us).
// Sentido: +1 si el angulo D-H crece al subir el pulso (hombro y codo, de su calibracion anterior; muneca
// por confirmar). Si en "prueba 1" una articulacion gira al reves: "cal <canal> invertir".
constexpr float US_POR_GRADO_NOMINAL = 10.0f;
constexpr int8_t SENTIDO_SERVO[4] = {1, -1, 1, -1};

// Limites SEGUROS [grados D-H] de cada articulacion: lo que usa la tarea ("dh_brazo.py tarea") y "prueba 1"
// mas ~10 grados de margen. Ningun comando, slider ni calibracion los puede pasar: fuera de esto la pieza
// choca con la estructura (con +-80 grados la muneca llego a -167 y quedo empujando el eslabon 3).
constexpr float Q_SEGURO[4][2] = {{-55.0f, 95.0f}, {40.0f, 100.0f}, {-95.0f, -15.0f}, {-125.0f, -55.0f}};

// ---------------- Pinza (0 = abierta, 100 = cerrada) ----------------
constexpr float PINZA_ABIERTA = 0.0f;
constexpr float PINZA_CERRADA = 100.0f;
constexpr uint32_t T_PINZA_MS = 600;

// ---------------- Proteccion de los servos ----------------
// El MG90S no informa ni corriente ni posicion: el firmware no puede "ver" un servo trabado, pero si evita
// lo que los quema (ver protegerServos() en main.cpp).
constexpr int MARGEN_LIMITE_US = 15;        // "pegado al limite": a menos de esto de su limite calibrado
constexpr uint32_t T_LIMITE_MS = 3000;      // quieto pegado al limite mas de esto -> retrocede
constexpr int RETROCESO_LIMITE_US = 30;     // cuanto retrocede hacia adentro (~3 grados)
constexpr float PINZA_ALIVIO = 5.0f;        // tras cerrar, afloja este % para no apretar el cubo con toda su fuerza
constexpr uint32_t REPOSO_DEFECTO_S = 90;   // sin ordenes ni movimiento este tiempo -> suelta todo (0 = nunca)

// ---------------- Tarea de apilado ----------------
struct PuntoMesa { float r, q1; };   // radio desde el eje de la base [mm] y angulo de base [deg]
// Todos a 96 mm del eje de la base: asi el hombro no pasa de ~76 grados ni el codo baja de ~-86 grados en
// ninguna pose (antes, la pila a 70 mm pedia 97 y -110, fuera del recorrido de sus horns). El maximo es
// 99 mm: mas lejos, la altura de traslado (Z_TRASLADO) queda fuera de alcance. Comprobado con dh_brazo.py.
constexpr PuntoMesa PUNTOS_RECOGIDA[] = {{96.0f, -45.0f}, {96.0f, 0.0f}, {96.0f, 45.0f}};
constexpr int N_RECOGIDA = sizeof(PUNTOS_RECOGIDA) / sizeof(PUNTOS_RECOGIDA[0]);
constexpr PuntoMesa PUNTO_APILADO = {96.0f, 90.0f};
constexpr int MAX_PILA = 3;
constexpr float CUBO = 20.0f;           // arista del cubo [mm]
constexpr float TIP_CLEAR = 5.0f;       // punta de la pinza sobre la cara inferior del cubo al agarrar
constexpr float MARGEN_APROX = 10.0f;   // altura de aproximacion sobre el punto de agarre o dejada
constexpr float Z_TRASLADO = 60.0f;     // altura de la punta al levantar y trasladar un cubo
constexpr float PHI_AGARRE = -90.0f;    // pinza vertical hacia abajo

// ---------------- Sensor ToF montado en la mano (MEDIR con pie de metro) ----------------
// El VL53L0X va sobre la mano, detras de la pinza, mirando en la misma direccion que ella.
// Con la pinza vertical hacia abajo, su cara "de arriba" mira hacia afuera (lejos de la base),
// asi que el haz cae TOF_LATERAL mm mas lejos de la base que la punta de la pinza.
constexpr float TOF_ATRAS = 45.0f;      // desde la punta de la pinza hacia atras, a lo largo de ella, hasta la cara del sensor
constexpr float TOF_LATERAL = 20.0f;    // del centro del sensor a la linea central de la pinza
constexpr float DIST_MIN_TOF = 40.0f;   // distancia minima sensor -> cara superior del cubo para medir bien
constexpr float HOLGURA_OBS = 15.0f;    // la punta queda al menos esto sobre la cara superior del cubo
// Altura de la punta al observar: el sensor queda a >= DIST_MIN_TOF del cubo y la pinza no lo toca
constexpr float Z_OBSERVACION = (HOLGURA_OBS > DIST_MIN_TOF - TOF_ATRAS) ? CUBO + HOLGURA_OBS : CUBO + DIST_MIN_TOF - TOF_ATRAS;
constexpr int LECTURAS_TOF = 5;         // mediana de N lecturas
constexpr float DZ_MAX = 15.0f;         // correccion de altura por ToF: errores mayores se consideran mala medida

// ---------------- Movimiento ----------------
constexpr uint32_t PERIODO_MS = 20;      // periodo de actualizacion de los servos
constexpr float VEL_MAX_DEG_S = 60.0f;   // velocidad pico articular en trayectorias
constexpr float VEL_LINEAL_MM_S = 40.0f; // velocidad pico en movimientos rectos
constexpr uint32_t T_MIN_MS = 400;       // duracion minima de una trayectoria
// Seguimiento de sliders y jog: perfil trapezoidal (arranque y frenado suaves)
constexpr float VEL_SEGUIMIENTO_US_S = 600.0f;     // de partida, ~55 deg/s; se cambia en vivo con "vel" y se guarda
constexpr float T_ACEL_SEGUIMIENTO_S = 0.2f;       // tiempo para alcanzar la velocidad maxima (aceleracion = vel / 0,2 s)

// ---------------- Servos MG90S y PCA9685 ----------------
constexpr float FREQ_SERVO_HZ = 50.0f;      // MG90S: un pulso cada 20 ms
constexpr float OSC_PCA_HZ = 27000000.0f;   // oscilador interno del PCA9685 (nominal 25 MHz, real ~23-27 MHz);
                                            // el error solo escala los pulsos y la calibracion lo absorbe
constexpr uint32_t I2C_HZ = 100000;         // 100 kHz: mas robusto que 400 kHz con cables largos y ruido de servos
constexpr int US_ABS_MIN = 600;             // rango absoluto: fuera de el, el MG90S empuja su tope interno
constexpr int US_ABS_MAX = 2400;
constexpr uint32_t PERIODO_TELEM_MS = 100;   // la tarea paralela recalcula la FK D-H cada 100 ms

// Definidas en main.cpp
void responder(const char *fmt, ...);
bool comprobarParada();
