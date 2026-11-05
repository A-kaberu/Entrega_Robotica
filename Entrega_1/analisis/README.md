# Análisis en el PC

| Archivo | Función | Uso |
|---|---|---|
| `dh_brazo.py` | Tabla D-H, matrices, cinemática directa e inversa. Lee las medidas de `firmware/src/config.h`. Puede comparar con el brazo en vivo | `python dh_brazo.py tabla 0 90 -90 0` · `ik x y z` · `tarea` · `formulas` · `comparar COM13` |
| `dh_brazo.m` | Lo mismo en MATLAB, con forma simbólica y dibujo | Abrir en MATLAB y ejecutar por secciones. No se ha ejecutado |
| `kinematics.py` | Alcance, singularidades, torques y estimación de masas (análisis de diseño) | `python kinematics.py` |
| `stl_tools.py` | Medidas y vistas de los archivos STL | `python stl_tools.py info pieza.stl` |

`dh_brazo.py` también lo usa `simulacion/generar_visor.py` para comparar el firmware con un modelo independiente.
