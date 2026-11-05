# Simulación del firmware

Ejecuta el firmware del brazo en el PC, con un hardware virtual, para probarlo sin el brazo físico.

**Qué comprueba:** la lógica del firmware (calibración, D-H, detección, rutina, límites y protecciones).

**Qué no comprueba:** la mecánica. El mundo simulado es ideal, sin holguras, peso ni deslizamiento.

## Archivos

| Archivo | Para qué sirve |
|---|---|
| `emulador/` | Reemplaza por software la ESP32, los servos y el sensor |
| `simular_firmware.cpp` | Crea el mundo simulado y ejecuta las pruebas |
| `generar_visor.py` | Crea el visor de resultados |
| `resultados/simulaciones.html` | Muestra la animación y los resultados en el navegador |
| `resultados/resumen.txt` | Resultado de cada prueba |

## Cómo se ejecuta
Desde la carpeta `Entrega_1`, con g++ y Python:

```bash
g++ -std=gnu++17 -O1 -w -Isimulacion/emulador -Ifirmware/src simulacion/simular_firmware.cpp -o simulacion/simular_firmware.exe
./simulacion/simular_firmware.exe
python simulacion/generar_visor.py
```
