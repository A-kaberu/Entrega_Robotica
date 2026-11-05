# Resultados en el brazo robótico

Resume cómo se comportó el brazo físico en las pruebas y qué se comprobó en simulación.

## Resultado general
El brazo físico no ejecutó bien los movimientos. Se observó un agarre deficiente en algunas uniones entre servo y eslabón, y los servos se sobrecalentaron o no completaron algunos movimientos de la rutina. Por eso, el brazo físico no cumplió lo esperado.

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

La causa probable es el agarre deficiente entre servo y eslabón. No se alcanzó a confirmar en cada articulación, ni a comprobar si la calibración del codo coincide con la pose real.

## Resultado en simulación
Se validó la lógica del firmware en una simulación virtual: 34 de 34 pruebas correctas (`simulacion/resultados/resumen.txt`). La simulación usa un mundo ideal, sin holguras, peso ni deslizamiento, así que no permitió comprobar la efectividad mecánica del brazo.
