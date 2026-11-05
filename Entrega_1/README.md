# Entrega 1: manipulador robótico de 4 GDL

Brazo robótico que se controla de forma manual o automática para detectar cubos con un sensor y apilarlos.

| Carpeta o archivo | Para qué sirve |
|---|---|
| `firmware/` | Programa de la ESP32, que mueve el brazo |
| `panel/` | Control del brazo desde el PC |
| `simulacion/` | Prueba del firmware en el PC, sin el brazo |
| `analisis/` | Cálculos del modelo D-H y del diseño |
| `esquematicos/` | Conexiones eléctricas |
| `Avances_Brazo.md` | Resultados de las pruebas |
| `Evidencias/` | Fotos y videos |
| `Codigos_Python/` | Código de conexión con la app e instalador de la app |
| `Informe_y_anexos/` | Informe e instalador de la app |

## Cómo funciona
1. El panel o la app Android envían órdenes a la ESP32, por USB o Bluetooth.
2. El firmware calcula los ángulos con el modelo D-H.
3. La placa PCA9685 genera la señal que mueve cada servo.
4. El sensor VL53L0X detecta si hay un cubo en cada punto.
