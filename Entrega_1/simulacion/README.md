# Simulación del firmware

Compila el firmware real (`firmware/src/main.cpp`) en el PC, **sin cambios**, sobre un hardware emulado, y le envía los mismos comandos que recibiría por USB.

**Qué comprueba:** la lógica (calibración, D-H, detección, rutina, límites y protecciones).
**Qué no comprueba:** la mecánica. El mundo simulado es ideal: sin holguras, peso ni deslizamiento de los servos.

## Archivos

| Archivo | Función |
|---|---|
| `emulador/` | Reemplaza Arduino, I2C con un PCA9685 emulado, sensor, Bluetooth y memoria |
| `simular_firmware.cpp` | Crea el mundo simulado (cubos, sensor) y ejecuta las pruebas |
| `generar_visor.py` | Compara con el modelo Python de `analisis/dh_brazo.py` y crea el visor |
| `resultados/simulaciones.html` | Visor: animación, ángulos en el tiempo y resultados. Se abre en el navegador |
| `resultados/resumen.txt` | Resultado de cada prueba |
| `resultados/rutina_3_cubos.csv` | Movimiento registrado cada 50 ms |
| `resultados/eventos_rutina.txt` | Mensajes del firmware durante la rutina |

## Cómo se ejecuta
Desde la carpeta que contiene `firmware/` y `simulacion/`, con g++ y Python:

```bash
g++ -std=gnu++17 -O1 -w -Isimulacion/emulador -Ifirmware/src simulacion/simular_firmware.cpp -o simulacion/simular_firmware.exe
./simulacion/simular_firmware.exe
python simulacion/generar_visor.py
```

Para guardar una imagen, usa el botón **Guardar imagen** del visor. Las figuras del informe salen de `resultados/q_t.csv` y `resultados/punta.csv`.
