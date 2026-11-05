# Panel del PC

`panel_brazo.html` controla el brazo desde Chrome o Edge por USB:

- **Control manual:** un slider por articulación.
- **Estado:** estado del brazo, medir con el sensor, HOME, pinza y ajuste del sensor.
- **Rutina con sensor (ToF):** aprende la mesa vacía y ejecuta el comando `A` del firmware. Muestra CUBO o VACIO por punto.
- **Grabar rutina:** guarda poses reales con su pinza y las repite. La rutina queda guardada en el navegador y se puede simular.
- **Vista D-H en vivo, consola y, plegados, mover a un punto y calibración.**

**Uso:** abrir el archivo con Chrome o Edge, pulsar **Conectar** y elegir el puerto de la ESP32. Antes hay que cerrar cualquier otro programa que use ese puerto.

| Archivo | Función |
|---|---|
| `panel_brazo.html` | Interfaz. Envía los comandos del firmware y dibuja la telemetría `T` |
| `sincronizar_panel.py` | Copia al panel las constantes de `firmware/src/config.h` y revisa que los comandos y mensajes coincidan con el firmware |

Después de cambiar el firmware: `python panel/sincronizar_panel.py`. El bloque `// <config.h>` del panel no se edita a mano.
