# Firmware del brazo

Programa de la ESP32. Recibe órdenes por USB o Bluetooth, calcula el modelo D-H, mueve los 5 servos y lee el sensor de distancia.

## Archivos

| Archivo | Para qué sirve |
|---|---|
| `src/config.h` | Medidas del brazo, límites seguros y puntos de la tarea |
| `src/cinematica.h` | Cálculos del modelo D-H |
| `src/actuadores.h` | Control de los servos y calibración |
| `src/main.cpp` | Órdenes, sensor, rutina automática y protecciones |
| `platformio.ini` | Configuración para compilar |

## Cómo se usa
1. Abrir esta carpeta con PlatformIO y subir el programa a la ESP32.
2. Controlar el brazo con el panel del PC o con la app Android.

## Órdenes principales

| Orden | Qué hace |
|---|---|
| `home` | Lleva el brazo a su posición inicial |
| `B/H/C/M/P<0-180>` | Mueve base, hombro, codo, muñeca o pinza |
| `aprender` | Mide la mesa vacía con el sensor |
| `A` | Inicia la rutina automática |
| `a` | Detiene el brazo |
| `X` | Suelta todos los servos |

## Rutina automática
El brazo revisa cada punto con el sensor. Si detecta un cubo, lo toma, lo traslada y lo apila. Cuando no quedan cubos, vuelve a su posición inicial.
