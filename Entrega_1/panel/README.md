# Panel del PC

`panel_brazo.html` controla el brazo desde Chrome o Edge, conectado por USB.

## Qué permite hacer
- **Control manual:** mover cada articulación con un slider.
- **Estado:** ver el estado del brazo, medir con el sensor, ir a HOME y mover la pinza.
- **Rutina con sensor:** detectar cubos y apilarlos de forma automática.
- **Grabar rutina:** guardar posiciones del brazo y repetirlas.
- **Vista D-H:** ver la posición de cada articulación en vivo.
- **Calibración:** ajustar cada servo.

## Cómo se usa
Abrir el archivo en Chrome o Edge, pulsar **Conectar** y elegir el puerto de la ESP32.

## Archivos

| Archivo | Para qué sirve |
|---|---|
| `panel_brazo.html` | Interfaz del panel |
| `sincronizar_panel.py` | Mantiene el panel al día con el firmware |
