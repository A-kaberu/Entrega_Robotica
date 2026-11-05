// Cinematica directa (producto de matrices de Denavit-Hartenberg) e inversa (geometrica).
//
//  i | theta | d  | a  | alpha
//  1 |  q1   | L1 | A1 |  90
//  2 |  q2   | 0  | L2 |   0
//  3 |  q3   | 0  | L3 |   0
//  4 |  q4   | 0  | L4 |   0
// Lo usan main.cpp (movimientos, telemetria y verificacion) y, con las mismas formulas,
// analisis/dh_brazo.py, que permite comparar el PC con el firmware (comando "comparar").
// Convencion: q2 = 0 es el brazo horizontal y q2 = 90 vertical; q3 y q4 son relativos al eslabon anterior.
// phi = q2 + q3 + q4 es la inclinacion de la pinza (-90 = vertical hacia abajo).
#pragma once
#include <math.h>
#include "config.h"

struct Pose { float x, y, z, phi; };   // punta de la pinza; z sobre la mesa [mm]; phi [deg]
struct Mat4 { float m[4][4]; };

// Matriz identidad 4x4: el punto de partida del producto de matrices.
inline Mat4 identidad() {
  Mat4 T = {};
  for (int i = 0; i < 4; i++) T.m[i][i] = 1.0f;
  return T;
}

// Producto de matrices homogeneas: C = A * B.
inline Mat4 multiplicar(const Mat4 &A, const Mat4 &B) {
  Mat4 C = {};
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      for (int k = 0; k < 4; k++) C.m[i][j] += A.m[i][k] * B.m[k][j];
  return C;
}

// Matriz homogenea D-H estandar (angulos en grados)
inline Mat4 matrizDH(float thetaDeg, float d, float a, float alphaDeg) {
  float ct = cosf(radians(thetaDeg)), st = sinf(radians(thetaDeg));
  float ca = cosf(radians(alphaDeg)), sa = sinf(radians(alphaDeg));
  return {{{ct, -st * ca, st * sa, a * ct},
           {st, ct * ca, -ct * sa, a * st},
           {0.0f, sa, ca, d},
           {0.0f, 0.0f, 0.0f, 1.0f}}};
}

// Filas (theta, d, a, alpha) de la tabla D-H para los angulos q
inline void tablaDH(const float q[4], float tabla[4][4]) {
  const float t[4][4] = {{q[0], L1, A1_DH, 90.0f}, {q[1], 0.0f, L2, 0.0f}, {q[2], 0.0f, L3, 0.0f}, {q[3], 0.0f, L4, 0.0f}};
  memcpy(tabla, t, sizeof(t));
}

// Origenes O0..O4 de los sistemas D-H en el sistema de la mesa [mm]
inline void fkOrigenes(const float q[4], float O[5][3]) {
  float tabla[4][4];
  tablaDH(q, tabla);
  Mat4 T = identidad();
  O[0][0] = 0.0f; O[0][1] = 0.0f; O[0][2] = H_BASE;
  for (int i = 0; i < 4; i++) {
    T = multiplicar(T, matrizDH(tabla[i][0], tabla[i][1], tabla[i][2], tabla[i][3]));
    O[i + 1][0] = T.m[0][3];
    O[i + 1][1] = T.m[1][3];
    O[i + 1][2] = T.m[2][3] + H_BASE;
  }
}

// Cinematica directa: de los cuatro angulos a la posicion de la punta (O4) y su inclinacion.
inline Pose fk(const float q[4]) {
  float O[5][3];
  fkOrigenes(q, O);
  return {O[4][0], O[4][1], O[4][2], q[1] + q[2] + q[3]};
}

// Desacople de muneca, solucion codo arriba. Devuelve false si la pose esta fuera de alcance.
inline bool ik(const Pose &p, float q[4]) {
  float phi = radians(p.phi);
  float r = hypotf(p.x, p.y);
  // Paso 1: se resta el eslabon L4 para obtener el centro de la muneca,
  // en el plano vertical del brazo (rw = distancia horizontal, zw = altura desde el hombro).
  float rw = r - A1_DH - L4 * cosf(phi);
  float zw = (p.z - H_BASE) - L1 - L4 * sinf(phi);
  // Paso 2: ley del coseno en el triangulo hombro-codo-muneca. D = cos(q3).
  // Si |D| > 1 el triangulo no existe: el punto esta fuera de alcance.
  float D = (rw * rw + zw * zw - L2 * L2 - L3 * L3) / (2.0f * L2 * L3);
  if (fabsf(D) > 1.0f) return false;
  // Seno negativo: se elige la solucion de codo arriba.
  float s3 = -sqrtf(1.0f - D * D);
  float q3 = atan2f(s3, D);
  // Paso 3: angulo hacia la muneca menos el angulo interior del triangulo.
  float q2 = atan2f(zw, rw) - atan2f(L3 * s3, L2 + L3 * D);
  // Paso 4: la base apunta hacia el punto y q4 completa la inclinacion pedida.
  q[0] = degrees(atan2f(p.y, p.x));
  q[1] = degrees(q2);
  q[2] = degrees(q3);
  q[3] = p.phi - q[1] - q[2];
  return true;
}
