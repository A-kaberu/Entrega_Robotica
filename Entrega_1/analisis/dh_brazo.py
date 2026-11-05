"""Instancia D-H del brazo 4 GDL en el PC (equivalente a un script de MATLAB, sin librerias externas).

Lee las medidas y los puntos de la tarea directamente de firmware/src/config.h, asi el PC y la ESP32
usan exactamente los mismos numeros.

Uso (desde la carpeta analisis):
  python dh_brazo.py tabla [q1 q2 q3 q4]   tabla D-H, matrices A1..A4, T0_4 y origenes O0..O4 (defecto: HOME)
  python dh_brazo.py ik x y z [phi]        cinematica inversa (z sobre la mesa, phi defecto -90) y comprobacion con FK
  python dh_brazo.py tarea                 angulos D-H de todas las poses de la rutina y rango que usa cada articulacion
  python dh_brazo.py formulas              ecuaciones cerradas de la cinematica directa (para el informe)
  python dh_brazo.py comparar [COM13] [s]  lee la telemetria T,... del brazo y compara sus O1..O4 con la FK de aqui

'comparar' necesita pyserial: usa el Python de PlatformIO
  %USERPROFILE%\\.platformio\\penv\\Scripts\\python.exe dh_brazo.py comparar COM13
y cierra antes el panel o el Serial Monitor (el puerto no se comparte).
"""
import math
import os
import re
import sys

CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "firmware", "src", "config.h")


def leer_config():
    texto = open(CONFIG, encoding="utf-8").read()
    c = {k: float(v) for k, v in re.findall(r"constexpr float (\w+) = (-?[\d.]+)f?;", texto)}
    par = r"\{(-?[\d.]+)f?, (-?[\d.]+)f?\}"
    lista = re.search(r"PUNTOS_RECOGIDA\[\] = \{(.*?)\};", texto).group(1)
    c["RECOGIDA"] = [(float(r), float(a)) for r, a in re.findall(par, lista)]
    c["PILA"] = tuple(float(x) for x in re.search(r"PUNTO_APILADO = " + par, texto).groups())
    c["Q_HOME"] = [float(x) for x in re.search(r"Q_HOME\[4\] = \{(.*?)\};", texto).group(1).replace("f", "").split(",")]
    c["MAX_PILA"] = int(re.search(r"MAX_PILA = (\d+)", texto).group(1))
    c.update({k: int(v) for k, v in re.findall(r"constexpr (?:int|uint32_t) (\w+) = (\d+);", texto)})
    seguro = [float(x) for x in re.findall(r"-?[\d.]+", re.search(r"Q_SEGURO\[4\]\[2\] = \{(.*?)\};", texto).group(1))]
    c["Q_SEGURO"] = [tuple(seguro[2 * i:2 * i + 2]) for i in range(4)]
    return c


C = leer_config()
L1, L2, L3, L4, A1, H = C["L1"], C["L2"], C["L3"], C["L4"], C["A1_DH"], C["H_BASE"]


# ---------------- Matrices D-H (convencion estandar) ----------------
def tabla_dh(q):
    """Filas (theta, d, a, alpha) en grados y mm."""
    return [(q[0], L1, A1, 90.0), (q[1], 0.0, L2, 0.0), (q[2], 0.0, L3, 0.0), (q[3], 0.0, L4, 0.0)]


def matriz_dh(theta, d, a, alpha):
    ct, st = math.cos(math.radians(theta)), math.sin(math.radians(theta))
    ca, sa = math.cos(math.radians(alpha)), math.sin(math.radians(alpha))
    return [[ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0]]


def mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def fk(q):
    """Devuelve (lista de A_i, T0_4, origenes O0..O4 sobre la mesa)."""
    T = [[float(i == j) for j in range(4)] for i in range(4)]
    A, O = [], [(0.0, 0.0, H)]
    for fila in tabla_dh(q):
        Ai = matriz_dh(*fila)
        A.append(Ai)
        T = mul(T, Ai)
        O.append((T[0][3], T[1][3], T[2][3] + H))
    return A, T, O


def ik(x, y, z, phi=-90.0):
    """Desacople de muneca, codo arriba (igual que firmware/src/cinematica.h). None si no alcanza."""
    p = math.radians(phi)
    r = math.hypot(x, y)
    rw = r - A1 - L4 * math.cos(p)
    zw = (z - H) - L1 - L4 * math.sin(p)
    D = (rw * rw + zw * zw - L2 * L2 - L3 * L3) / (2 * L2 * L3)
    if abs(D) > 1:
        return None
    s3 = -math.sqrt(1 - D * D)
    q3 = math.atan2(s3, D)
    q2 = math.atan2(zw, rw) - math.atan2(L3 * s3, L2 + L3 * D)
    q1 = math.degrees(math.atan2(y, x))
    return [q1, math.degrees(q2), math.degrees(q3), phi - math.degrees(q2) - math.degrees(q3)]


def imprimir_matriz(nombre, M):
    print(f"{nombre} =")
    for fila in M:
        print("   [" + "  ".join(f"{v:9.3f}" for v in fila) + "]")


# ---------------- Comandos ----------------
def cmd_tabla(args):
    q = [float(v) for v in args] if len(args) == 4 else C["Q_HOME"]
    A, T, O = fk(q)
    print(f"q = {q} deg   (L1={L1}, L2={L2}, L3={L3}, L4={L4}, a1={A1}, H_BASE={H} mm)\n")
    print(" i | theta [deg] | d [mm] | a [mm] | alpha [deg]")
    for i, f in enumerate(tabla_dh(q), 1):
        print(f" {i} | {f[0]:11.2f} | {f[1]:6.1f} | {f[2]:6.1f} | {f[3]:6.0f}")
    print()
    for i, Ai in enumerate(A, 1):
        imprimir_matriz(f"A{i}", Ai)
    imprimir_matriz("T0_4 = A1*A2*A3*A4", T)
    print("\nOrigenes sobre la mesa [mm]:")
    for i, o in enumerate(O):
        print(f"  O{i} = ({o[0]:7.1f}, {o[1]:7.1f}, {o[2]:7.1f})")
    print(f"  phi = q2+q3+q4 = {q[1] + q[2] + q[3]:.1f} deg")


def cmd_ik(args):
    x, y, z = (float(v) for v in args[:3])
    phi = float(args[3]) if len(args) > 3 else -90.0
    q = ik(x, y, z, phi)
    if q is None:
        print("Fuera de alcance")
        return
    O = fk(q)[2]
    print("q = [" + ", ".join(f"{v:.2f}" for v in q) + "] deg")
    print(f"FK de vuelta: ({O[4][0]:.3f}, {O[4][1]:.3f}, {O[4][2]:.3f}) mm  ->  error "
          f"{math.dist(O[4], (x, y, z)):.2e} mm")


def poses_tarea():
    """Mismas poses que verificarTarea() del firmware."""
    cubo, tip, mar, zt = C["CUBO"], C["TIP_CLEAR"], C["MARGEN_APROX"], C["Z_TRASLADO"]
    tl, zo = C["TOF_LATERAL"], max(cubo + C["HOLGURA_OBS"], cubo + C["DIST_MIN_TOF"] - C["TOF_ATRAS"])
    mesa = lambda r, a, z: (r * math.cos(math.radians(a)), r * math.sin(math.radians(a)), z)
    poses = []
    for i, (r, a) in enumerate(C["RECOGIDA"], 1):
        poses += [(f"observar {i}", mesa(r - tl, a, zo)), (f"sobre {i}", mesa(r, a, zo)),
                  (f"agarrar {i}", mesa(r, a, tip)), (f"levantar {i}", mesa(r, a, zt))]
    r, a = C["PILA"]
    poses.append(("observar pila", mesa(r - tl, a, zo)))
    for n in range(C["MAX_PILA"]):
        z = n * cubo + tip
        poses += [(f"apilar {n + 1} sobre", mesa(r, a, z + mar)), (f"apilar {n + 1}", mesa(r, a, z))]
    return poses


def cmd_tarea(args):
    lo, hi = [1e9] * 4, [-1e9] * 4
    print(f"{'pose':18s} {'x':>7s} {'y':>7s} {'z':>6s} |" + "".join(f"{n:>8s}" for n in ("q1", "q2", "q3", "q4")))
    for nombre, (x, y, z) in poses_tarea():
        q = ik(x, y, z)
        if q is None:
            print(f"{nombre:18s} {x:7.1f} {y:7.1f} {z:6.1f} | FUERA DE ALCANCE")
            continue
        for j in range(4):
            lo[j], hi[j] = min(lo[j], q[j]), max(hi[j], q[j])
        print(f"{nombre:18s} {x:7.1f} {y:7.1f} {z:6.1f} |" + "".join(f"{v:8.1f}" for v in q))
    print("\nRango que usa la tarea [deg D-H]:")
    for j, n in enumerate(("base q1", "hombro q2", "codo q3", "muneca q4")):
        print(f"  {n:10s} [{lo[j]:7.1f}, {hi[j]:7.1f}]  recorrido {hi[j] - lo[j]:6.1f}")


def cmd_formulas(args):
    print("Cinematica directa en forma cerrada (c1 = cos q1, c23 = cos(q2+q3), c234 = cos(q2+q3+q4), ...):")
    print(f"  r   = a1 + L2 c2 + L3 c23 + L4 c234        (a1 = {A1})")
    print("  x   = c1 * r")
    print("  y   = s1 * r")
    print(f"  z   = H_BASE + L1 + L2 s2 + L3 s23 + L4 s234   (H_BASE = {H}, L1 = {L1})")
    print("  phi = q2 + q3 + q4")
    print("Inversa (codo arriba): rw = r - a1 - L4 cos(phi), zw = z - H_BASE - L1 - L4 sin(phi)")
    print("  cos q3 = (rw^2 + zw^2 - L2^2 - L3^2) / (2 L2 L3),  sin q3 = -sqrt(1 - cos^2 q3)")
    print("  q2 = atan2(zw, rw) - atan2(L3 sin q3, L2 + L3 cos q3),  q4 = phi - q2 - q3,  q1 = atan2(y, x)")
    print(f"Jacobiano: det J = r * L2 * L3 * sin q3 -> singular con el codo estirado (q3 = 0) o r = 0")


def cmd_comparar(args):
    try:
        import serial
    except ImportError:
        sys.exit("Falta pyserial: usa %USERPROFILE%\\.platformio\\penv\\Scripts\\python.exe")
    import time
    puerto = args[0] if args else "COM13"
    segundos = float(args[1]) if len(args) > 1 else 10
    s = serial.Serial()
    s.port, s.baudrate, s.timeout, s.dtr, s.rts = puerto, 115200, 0.2, False, False
    s.open()
    s.write(b"telem 1\n")
    resto, n, peor = "", 0, 0.0
    t0 = time.time()
    while time.time() - t0 < segundos:
        resto += s.read(4096).decode("utf-8", "replace")
        *lineas, resto = resto.split("\n")
        for l in lineas:
            if not l.startswith("T,"):
                continue
            v = [float(x) for x in l.strip().split(",")[1:]]
            O_fw = [tuple(v[5 + 3 * i: 8 + 3 * i]) for i in range(4)]
            O_pc = fk(v[:4])[2][1:]
            err = max(math.dist(a, b) for a, b in zip(O_fw, O_pc))
            peor, n = max(peor, err), n + 1
            print(f"q = {v[0]:6.1f} {v[1]:6.1f} {v[2]:6.1f} {v[3]:6.1f}  punta firmware {O_fw[3]}  "
                  f"PC ({O_pc[3][0]:.1f}, {O_pc[3][1]:.1f}, {O_pc[3][2]:.1f})  dif {err:.2f} mm")
    s.write(b"telem 0\n")
    s.close()
    if n:
        print(f"\n{n} lineas comparadas. Diferencia maxima firmware vs PC: {peor:.2f} mm "
              f"({'OK: misma cinematica' if peor < 0.2 else 'REVISAR: el firmware y el modelo no coinciden'})")
    else:
        print("No llego telemetria: el brazo necesita una pose conocida (envia 'home' o mueve un slider).")


if __name__ == "__main__":
    ordenes = {"tabla": cmd_tabla, "ik": cmd_ik, "tarea": cmd_tarea, "formulas": cmd_formulas, "comparar": cmd_comparar}
    if len(sys.argv) < 2 or sys.argv[1] not in ordenes:
        print(__doc__)
        sys.exit(0)
    ordenes[sys.argv[1]](sys.argv[2:])
