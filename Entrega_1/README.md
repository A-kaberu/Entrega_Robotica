# Entrega 1: manipulador robótico de 4 GDL

| Carpeta o archivo | Qué contiene |
|---|---|
| `firmware/` | Programa de la ESP32 (explicado en su README) |
| `panel/` | Control desde el PC, por USB |
| `simulacion/` | Firmware compilado en el PC con hardware emulado, pruebas y visor (`resultados/simulaciones.html`) |
| `analisis/` | Modelo D-H en Python y MATLAB, y análisis de diseño |
| `esquematicos/` | Esquema eléctrico y proyecto KiCad |
| `Avances_Brazo.md` | Lo observado en el brazo físico y sus limitaciones |
| `Evidencias/` | Fotos y videos |
| `Informe_y_anexos/` | Informe, instalador de la app y archivos del informe |

Cómo se conecta todo: el panel o la app Android envían comandos a la ESP32 (USB o Bluetooth). El firmware calcula el modelo D-H y mueve los servos con el PCA9685, usa el sensor VL53L0X para detectar cubos y devuelve la posición de cada articulación. Las medidas están en `firmware/src/config.h`, y las usan también el análisis y el panel.
