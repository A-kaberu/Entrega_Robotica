"""
Analisis cinematico y estatico/dinamico - brazo 4 GDL tipo KUKA a escala.
Laboratorio 1 - Fundamentos de Robotica (USACH, S2-2026).

Uso:  python kinematics.py
Solo usa la libreria estandar (matplotlib es opcional, para graficar el espacio de trabajo).

Convencion: Denavit-Hartenberg estandar (Spong / Craig distal).
  i | theta_i | d_i | a_i | alpha_i
  1 |  q1     | L1  | A1  |  +90 deg   (base, giro vertical)
  2 |  q2     |  0  | L2  |   0        (hombro)
  3 |  q3     |  0  | L3  |   0        (codo)
  4 |  q4     |  0  | L4  |   0        (muneca pitch -> punta de pinza)
{0}: sobre la cara superior de la base giratoria, en su eje de giro. L1 = de ahi al eje del hombro.
{M} (mesa): {0} esta H_BASE mm sobre la mesa -> z_mesa = z0 + H_BASE.
TCP = punta de la pinza (L4 medido hasta la punta).
q2 = 0 -> brazo horizontal hacia adelante; q2 = 90 deg -> vertical.
q3, q4 relativos al eslabon anterior; phi = q2+q3+q4 = inclinacion de la pinza.
"""
import math
import random

# ---------------- Parametros geometricos [mm] ----------------
# Distancias eje a eje obtenidas de los STL ensamblados (stl_tools.py); L4 medido hasta la punta.
# Medidas con regla: L1=30, L2=69, L3=62 (mismo orden de magnitud).
L1, L2, L3, L4 = 32.0, 67.0, 60.4, 85.0
A1 = -1.2       # eje del hombro 1.2 mm detras del eje de giro de la base (STL) -> practicamente 0
H_BASE = 35.0   # altura mesa -> cara superior de la base fija = inicio de la base giratoria (STL)
# Nota: el eje de la muneca esta 2.8 mm por encima de la linea del codo (angulo constante de 2.6 deg
# en el eslabon 3). Se absorbe como offset constante de q3 en la calibracion del servo.
CUBE = 20.0     # arista del cubo [mm]
TIP_CLEAR = 5.0 # la punta de la pinza queda 5 mm sobre la cara inferior del cubo al agarrar
G = 9.81

# Limites articulares PROVISIONALES [deg] -> corregir tras calibrar cada servo
LIM = {1: (-90.0, 90.0), 2: (0.0, 180.0), 3: (-150.0, 30.0), 4: (-120.0, 60.0)}

# ---------------- Modelo de masas ---------
# PLA: volumen y centro de masa calculados de los STL, masa = solida x 0.6 (paredes + relleno ~20 %).
# Pesar las piezas reales y corregir si difiere.
# (nombre, masa [kg], eslabon al que pertenece, fraccion de la longitud del eslabon)
MASSES = [
    ("eslabon 2 (PLA)",          0.024,  2, 0.53),
    ("servo codo MG90S",         0.0134, 2, 1.0),
    ("eslabon 3 (PLA)",          0.021,  3, 0.46),
    ("servo muneca MG90S",       0.0134, 3, 1.0),
    ("eslabon 4 (PLA)",          0.0054, 4, 0.21),
    ("servo pinza MG90S",        0.0134, 4, 0.25),
    ("dedos+engranajes (PLA)",   0.0055, 4, 0.78),
    ("carga: cubo 20 mm",        0.010,  4, 0.90),
]
# Torque de bloqueo segun datasheet a 4.8 V (1.8 kgf*cm = 0.177 N*m). MG90S a 6 V: 2.2 kgf*cm.
STALL = {2: ("MG90S", 0.177), 3: ("MG90S", 0.177), 4: ("MG90S", 0.177)}


# ======================= Cinematica directa =======================
def matmul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def dh(theta, d, a, alpha):
    ct, st, ca, sa = math.cos(theta), math.sin(theta), math.cos(alpha), math.sin(alpha)
    return [[ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0]]


def dh_rows(q):
    q1, q2, q3, q4 = q
    return [(q1, L1, A1, math.pi / 2), (q2, 0.0, L2, 0.0), (q3, 0.0, L3, 0.0), (q4, 0.0, L4, 0.0)]


def fk_frames(q):
    """Devuelve [T00, T01, T02, T03, T04] (matrices homogeneas)."""
    T = [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]
    frames = [T]
    for row in dh_rows(q):
        T = matmul(T, dh(*row))
        frames.append(T)
    return frames


def fk(q):
    """Forma cerrada: posicion de la punta (x, y, z) [mm] y phi [rad]."""
    q1, q2, q3, q4 = q
    r = A1 + L2 * math.cos(q2) + L3 * math.cos(q2 + q3) + L4 * math.cos(q2 + q3 + q4)
    z = L1 + L2 * math.sin(q2) + L3 * math.sin(q2 + q3) + L4 * math.sin(q2 + q3 + q4)
    return r * math.cos(q1), r * math.sin(q1), z, q2 + q3 + q4


# ======================= Cinematica inversa =======================
def ik(x, y, z, phi, elbow_up=True):
    """Solucion geometrica (desacople de muneca). Devuelve (q1..q4) [rad] o None."""
    q1 = math.atan2(y, x)
    r = math.hypot(x, y)
    rw = r - A1 - L4 * math.cos(phi)     # centro de muneca en el plano del brazo
    zw = z - L1 - L4 * math.sin(phi)
    D = (rw * rw + zw * zw - L2 * L2 - L3 * L3) / (2 * L2 * L3)
    if abs(D) > 1.0:
        return None                       # fuera de alcance
    s3 = math.sqrt(1.0 - D * D) * (-1.0 if elbow_up else 1.0)
    q3 = math.atan2(s3, D)
    q2 = math.atan2(zw, rw) - math.atan2(L3 * s3, L2 + L3 * D)
    q4 = phi - q2 - q3
    return q1, q2, q3, q4


def within_limits(q):
    return all(LIM[i + 1][0] - 1e-6 <= math.degrees(qi) <= LIM[i + 1][1] + 1e-6 for i, qi in enumerate(q))


def det_jacobian(q):
    """det J (x,y,z,phi respecto a q1..q4) = r * L2 * L3 * sin(q3)."""
    r = math.hypot(*fk(q)[:2])
    return r * L2 * L3 * math.sin(q[2])


# ======================= Estatica (vector de gravedad G(q)) =======================
def planar_chain(q2, q3, q4):
    ang = (q2, q2 + q3, q2 + q3 + q4)
    P = [(A1, L1)]
    for a, L in zip(ang, (L2, L3, L4)):
        P.append((P[-1][0] + L * math.cos(a), P[-1][1] + L * math.sin(a)))
    return P, ang


def gravity_torques(q2, q3, q4, masses=MASSES):
    """Torque estatico [N*m] que debe sostener cada servo de pitch (2, 3, 4)."""
    P, ang = planar_chain(q2, q3, q4)
    lens = (L2, L3, L4)
    tau = {2: 0.0, 3: 0.0, 4: 0.0}
    for _, m, link, f in masses:
        k = link - 2
        px = P[k][0] + f * lens[k] * math.cos(ang[k])
        for j in (2, 3, 4):
            if link >= j:
                tau[j] += m * G * (px - P[j - 2][0]) / 1000.0
    return tau


def inertia_about_shoulder_stretched(masses=MASSES):
    """Inercia [kg*m^2] respecto al hombro con el brazo extendido horizontal (peor caso)."""
    lens = (L2, L3, L4)
    I = 0.0
    for _, m, link, f in masses:
        d = sum(lens[: link - 2]) + f * lens[link - 2]
        I += m * (d / 1000.0) ** 2
    return I


# ======================= Reportes =======================
def deg(v):
    return tuple(round(math.degrees(a), 2) for a in v)


def report():
    print("=" * 70)
    print("1) TABLA D-H (L1, L2, L3, L4 = %.0f, %.0f, %.0f, %.0f mm)" % (L1, L2, L3, L4))
    print("   i | theta | d   | a   | alpha")
    for i, (th, d, a, al) in enumerate([("q1", L1, A1, 90),("q2", 0, L2, 0), ("q3", 0, L3, 0), ("q4", 0, L4, 0)], 1):
        print("   %d | %-5s | %-3g | %-3g | %g" % (i, th, d, a, al))

    print("\n2) VALIDACION FK (matrices D-H) vs forma cerrada, e IK ida y vuelta")
    random.seed(1)
    max_fk, max_ik, n = 0.0, 0.0, 0
    for _ in range(5000):
        q = [math.radians(random.uniform(*LIM[i])) for i in (1, 2, 3, 4)]
        T = fk_frames(q)[-1]
        x, y, z, phi = fk(q)
        max_fk = max(max_fk, abs(T[0][3] - x), abs(T[1][3] - y), abs(T[2][3] - z))
        sol = ik(x, y, z, phi, elbow_up=q[2] < 0)
        if sol and abs(math.hypot(x, y)) > 1:
            x2, y2, z2, _ = fk(sol)
            max_ik = max(max_ik, math.dist((x, y, z), (x2, y2, z2)))
            n += 1
    print("   error max FK matricial vs cerrada: %.2e mm" % max_fk)
    print("   error max IK->FK (%d poses):       %.2e mm" % (n, max_ik))

    home = [0, math.radians(90), math.radians(-90), math.radians(0)]
    print("\n   Pose HOME q=(0,90,-90,0) deg -> punta (x,y,z,phi):",
          tuple(round(v, 1) for v in fk(home)[:3]), round(math.degrees(fk(home)[3]), 1), "deg")
    for name, Ti in zip(("O0", "O1", "O2", "O3", "O4"), fk_frames(home)):
        print("     %s = (%.1f, %.1f, %.1f)" % (name, Ti[0][3], Ti[1][3], Ti[2][3]))

    print("\n3) ALCANCE")
    print("   radio max (brazo estirado, desde eje de base): %.0f mm" % (L2 + L3 + L4))
    print("   centro de muneca: anillo entre |L2-L3| = %.0f y L2+L3 = %.0f mm desde el hombro"
          % (abs(L2 - L3), L2 + L3))

    print("\n4) ZONAS ALCANZABLES (z desde la MESA, cubos de %.0f mm, TCP = punta)" % CUBE)
    print("   Para tomar/dejar el cubo n la punta queda a (n-1)*%.0f + %.0f mm sobre la mesa." % (CUBE, TIP_CLEAR))
    for h in sorted({0.0, 30.0, 50.0, 70.0, H_BASE}):
        print("   -- H_BASE = %.0f mm (eje del hombro a %.0f mm de la mesa) --" % (h, h + L1))
        for phi_deg in (-90, -60, -45):
            cells = []
            for n_cube in range(1, 5):
                z0 = (n_cube - 1) * CUBE + TIP_CLEAR - h      # z de la punta en el sistema {0}
                rs = []
                for r10 in range(0, 3000):
                    sol = ik(r10 / 10.0, 0.0, z0, math.radians(phi_deg))
                    if sol and within_limits(sol):
                        rs.append(r10 / 10.0)
                cells.append("%3.0f-%3.0f" % (min(rs), max(rs)) if rs else "   ---   ")
            print("     phi=%4d | cubo1 %s | cubo2 %s | cubo3 %s | cubo4 %s  [r, mm]" % (phi_deg, *cells))

    print("\n5) SINGULARIDADES: det J = r*L2*L3*sin(q3) -> q3 = 0 (estirado), r = 0 (sobre el eje)")

    print("\n6) ESTATICA - torque de gravedad maximo en el rango articular")
    worst = {2: (0, None), 3: (0, None), 4: (0, None)}
    for a2 in range(int(LIM[2][0]), int(LIM[2][1]) + 1, 3):
        for a3 in range(int(LIM[3][0]), int(LIM[3][1]) + 1, 3):
            for a4 in range(int(LIM[4][0]), int(LIM[4][1]) + 1, 3):
                t = gravity_torques(math.radians(a2), math.radians(a3), math.radians(a4))
                for j in (2, 3, 4):
                    if abs(t[j]) > worst[j][0]:
                        worst[j] = (abs(t[j]), (a2, a3, a4))
    for j in (2, 3, 4):
        tmax, pose = worst[j]
        servo, ts = STALL[j]
        print("   q%d (%s): tau_max = %.3f N*m = %.2f kgf*cm  en (q2,q3,q4)=%s  FS = %.2f"
              % (j, servo, tmax, tmax / 0.0981, pose, ts / tmax))

    print("\n7) DINAMICA - termino inercial en el hombro (peor caso, brazo extendido)")
    I = inertia_about_shoulder_stretched()
    for dth, T in ((60, 1.0), (90, 1.0), (90, 0.5)):
        a_max = 5.7735 * math.radians(dth) / T ** 2     # pico de aceleracion perfil quintico
        w_max = 1.875 * math.radians(dth) / T           # pico de velocidad perfil quintico
        print("   d_theta=%2d deg en T=%.1f s: w_max=%.2f rad/s, a_max=%.1f rad/s^2 -> I*a = %.4f N*m"
              % (dth, T, w_max, a_max, I * a_max))
    print("   I_hombro = %.3e kg*m^2" % I)

    print("\n8) MASAS USADAS (estimadas):")
    for name, m, link, f in MASSES:
        print("   %-22s %5.1f g  eslabon %d  a %.0f%% de su longitud" % (name, m * 1000, link, f * 100))


def plot_workspace(path="espacio_trabajo.png"):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\n(matplotlib no instalado: se omite el grafico. pip install matplotlib)")
        return
    random.seed(2)
    rs, zs = [], []
    for _ in range(40000):
        q = [0.0] + [math.radians(random.uniform(*LIM[i])) for i in (2, 3, 4)]
        x, _, z, _ = fk(q)
        rs.append(x)
        zs.append(z + H_BASE)
    fig, ax = plt.subplots(figsize=(7, 6))
    ax.scatter(rs, zs, s=0.3, alpha=0.3)
    ax.axhline(0, color="k", lw=1)
    ax.set_xlabel("r [mm]")
    ax.set_ylabel("z sobre la mesa [mm]")
    ax.set_title("Espacio de trabajo (plano vertical del brazo)")
    ax.set_aspect("equal")
    fig.savefig(path, dpi=150)
    print("\nGrafico guardado en", path)


if __name__ == "__main__":
    report()
    plot_workspace()
