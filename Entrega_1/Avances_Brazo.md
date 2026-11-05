# Avances observados en el brazo robótico

Registro de lo que se ha visto en el brazo físico. Las evidencias (fotos y videos) se agregan en la carpeta `Evidencias/`.

## Estado general (05-11-2026)
El brazo **no ejecuta bien los movimientos**. Tiene problemas de agarre en ciertos puntos clave de los eslabones, donde el servo se une a la pieza. Por eso, muchas veces los servos se sobrecalientan o no ejecutan los movimientos que deberían hacer para la rutina. **Por lo tanto, no se cumple lo esperado en el brazo físico.**

Las piezas no se alcanzaron a reimprimir ni rediseñar.

## Lo que funcionó

| Prueba | Resultado |
|---|---|
| Detección de PCA9685 y sensor | Correcta, 0 errores I2C |
| Lectura del sensor VL53L0X | Responde (85 y 99 mm en dos lecturas, sin ajuste de offset) |
| Calibración guardada | Los 5 canales completos (`abmM`) |
| `verificar` | Las 87 poses de la tarea son alcanzables con esa calibración |
| Comando `home` | El firmware lo ejecutó sin errores |
| Panel con el brazo real | Se usó para mover el brazo y recalibrar |
| Pinza | Abrir y cerrar estaban invertidos por una mala calibración; se recalibró y quedó bien |

## Lo que falló

| Observación | Detalle |
|---|---|
| Codo demasiado cerrado en HOME | Con q3 = −88° (720 µs) se vio muy cerrado y costaba mantener la pose |
| Codo forzando | Con 1170 µs se observó choque contra la estructura |
| Codo bloqueado | Al volver a 720 µs quedó bloqueado, sin moverse |
| Sobrecalentamiento | Se percibió en los servos durante las pruebas (sin medición de temperatura) |
| Rutina completa | No se ejecutó en el brazo físico |

Causa probable: agarre deficiente entre servo y eslabón. No se alcanzó a confirmar en cada articulación, ni a comprobar si la calibración del codo coincide con la pose real.

## Lo que sí está validado
La lógica del firmware se comprobó en simulación: ver `simulacion/resultados/resumen.txt`. Esa simulación usa un mundo ideal y **no reemplaza** la prueba mecánica.

## Pendiente
- Agregar fotos y videos a `Evidencias/`.
- Revisar el juego de cada unión servo–eslabón, con la fuente apagada.
- Recalibrar el codo y repetir las pruebas si se corrige el agarre.
