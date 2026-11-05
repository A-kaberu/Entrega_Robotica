# Firmware del brazo 4 GDL

Programa de la ESP32: recibe comandos por USB o Bluetooth, calcula el modelo D-H y mueve 5 servos MG90S mediante un PCA9685. También lee el sensor VL53L0X.

## Archivos

| Archivo | Función | Se conecta con |
|---|---|---|
| `src/config.h` | Medidas D-H, HOME, límites seguros, puntos de la tarea y constantes. Es la única fuente de estos números | Todos los demás; también lo leen `analisis/dh_brazo.py` y el panel |
| `src/cinematica.h` | Matrices D-H, cinemática directa e inversa | Lo usan `main.cpp` y `actuadores.h` |
| `src/actuadores.h` | Manejo del PCA9685, calibración, seguimiento suave y trayectorias quínticas | Lo usa `main.cpp` |
| `src/main.cpp` | Intérprete de comandos, telemetría, sensor, rutina automática y protecciones | Punto de entrada: `setup()` y `loop()` |
| `platformio.ini` | Configuración de compilación y librerías | PlatformIO |

## Cómo se usa
1. Abrir esta carpeta con PlatformIO y subir a la ESP32. Si falla el arranque, mantener **BOOT** durante la subida.
2. Controlar el brazo desde `panel/panel_brazo.html` (PC) o la app Android.

## Comandos principales

| Comando | Qué hace |
|---|---|
| `cal`, `cal <canal> a\|b <grados>`, `cal <canal> lim` | Muestra o guarda la calibración |
| `verificar` | Revisa, sin mover, si las poses de la tarea son alcanzables |
| `home`, `q q1 q2 q3 q4`, `move x y z [phi]`, `pinza 0-100` | Movimientos |
| `B/H/C/M/P<0-180>` | Sliders de la app |
| `aprender`, `A` | Aprende la mesa vacía; inicia la rutina con sensor |
| `a` / `X` | PARAR / SOLTAR, también durante un movimiento |

La lista completa está al inicio de `src/main.cpp`.

## Rutina automática (`A`)
En cada punto de recogida, el sensor mide la distancia. Si es al menos 10 mm menor que la de la mesa vacía, hay un cubo: lo toma, lo traslada a una altura segura y lo apila. Si no quedan cubos, vuelve a HOME.

La comprobación sin hardware está en `simulacion/`.
