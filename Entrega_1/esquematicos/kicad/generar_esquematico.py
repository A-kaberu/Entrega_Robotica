"""Genera el proyecto KiCad del brazo (esquematico con etiquetas de red)."""
import json
import os
import sys
import uuid

OUT = sys.argv[1]
PROJ = "brazo_robotico"
G = 2.54
U = lambda: str(uuid.uuid4())
ROOT = U()


def f(v):
    return ("%.3f" % v).rstrip("0").rstrip(".")


def eff(size=1.27, hide=False, justify=None):
    s = "(effects (font (size %s %s))" % (f(size), f(size))
    if justify:
        s += " (justify %s)" % justify
    if hide:
        s += " hide"
    return s + ")"


# ---------- definiciones de simbolos ----------
# pins: (nombre, numero, x, y, angulo)  coordenadas de libreria (y hacia arriba)
SYMS = {}


def sym(name, rect, pins, ref, texts=(), extra_graphics=""):
    SYMS[name] = dict(rect=rect, pins=pins, ref=ref, texts=texts, extra=extra_graphics)


def side(names, x, y0, ang, start_num):
    return [(n, str(start_num + i), x, y0 - i * G, ang) for i, n in enumerate(names)]


sym("ESP32_DevKit", (-12.7, 10.16, 12.7, -15.24),
    side(["3V3", "GND", "GPIO21_SDA", "GPIO22_SCL", "GPIO18_SDA_TOF", "GPIO19_SCL_TOF"], 15.24, 7.62, 180, 1)
    + [("VIN", "7", 15.24, -10.16, 180)], "U",
    texts=[("USB -> PC", -10.16, -12.7)])
sym("PCA9685_Module", (-12.7, 10.16, 12.7, -20.32),
    side(["GND", "OE", "SCL", "SDA", "VCC", "V+"], -15.24, 7.62, 0, 1)
    + side(["PWM0", "PWM1", "PWM2", "PWM3", "PWM4"], 15.24, 7.62, 180, 7)
    + [("V+_SERVOS", "12", 15.24, -7.62, 180), ("GND_SERVOS", "13", 15.24, -10.16, 180),
       ("V+_BORNERA", "14", -7.62, -22.86, 90), ("GND_BORNERA", "15", -2.54, -22.86, 90)], "U")
sym("VL53L0X_Module", (-10.16, 7.62, 10.16, -10.16),
    side(["VIN", "GND", "SCL", "SDA", "GPIO1", "XSHUT"], 12.7, 5.08, 180, 1), "U")
sym("Servo", (-7.62, 5.08, 7.62, -5.08),
    side(["PWM", "V+", "GND"], -10.16, 2.54, 0, 1), "M")
sym("Fuente_Lab", (-10.16, 5.08, 10.16, -5.08),
    [("+", "1", -2.54, 7.62, 270), ("-", "2", 2.54, 7.62, 270)], "PS",
    texts=[("6,0 V / 3 A", 0, -2.54)])
sym("C_Polarizado", None,
    [("+", "1", 0, 5.08, 270), ("-", "2", 0, -5.08, 90)], "C",
    extra_graphics=(
        "(rectangle (start -2.032 1.27) (end 2.032 0.762) (stroke (width 0.254) (type default)) (fill (type none)))"
        "(arc (start -2.032 -1.27) (mid 0 -0.508) (end 2.032 -1.27) (stroke (width 0.254) (type default)) (fill (type none)))"
        "(polyline (pts (xy 0 1.27) (xy 0 2.54)) (stroke (width 0) (type default)) (fill (type none)))"
        "(polyline (pts (xy 0 -0.508) (xy 0 -2.54)) (stroke (width 0) (type default)) (fill (type none)))"
        "(text \"+\" (at -2.54 2.032 0) (effects (font (size 1.27 1.27))))"))
# Ajuste de pines del condensador para que lleguen al cuerpo
SYMS["C_Polarizado"]["pin_len"] = 2.54


def sym_def(name, prefix):
    s = SYMS[name]
    out = ['(symbol "%s%s" (pin_names (offset 1.016)) (exclude_from_sim no) (in_bom yes) (on_board yes)' % (prefix, name)]
    top = s["rect"][1] if s["rect"] else 5.08
    out.append('(property "Reference" "%s" (at 0 %s 0) %s)' % (s["ref"], f(top + 2.54), eff()))
    out.append('(property "Value" "%s" (at 0 %s 0) %s)' % (name, f(top + 5.08), eff()))
    for p in ("Footprint", "Datasheet", "Description"):
        out.append('(property "%s" "" (at 0 0 0) %s)' % (p, eff(hide=True)))
    out.append('(symbol "%s_0_1"' % name)
    if s["rect"]:
        x1, y1, x2, y2 = s["rect"]
        out.append("(rectangle (start %s %s) (end %s %s) (stroke (width 0.254) (type default)) (fill (type background)))"
                   % (f(x1), f(y1), f(x2), f(y2)))
    out.append(s["extra"])
    for t, x, y in s["texts"]:
        out.append('(text "%s" (at %s %s 0) %s)' % (t, f(x), f(y), eff()))
    out.append(")")
    out.append('(symbol "%s_1_1"' % name)
    for n, num, x, y, a in s["pins"]:
        out.append('(pin passive line (at %s %s %d) (length %s) (name "%s" %s) (number "%s" %s))'
                   % (f(x), f(y), a, f(s.get("pin_len", 2.54)), n, eff(), num, eff()))
    out.append("))")
    return "\n".join(out)


# ---------- instancias ----------
items = []
PIN_POS = {}


def place(name, ref, value, X, Y, props="top"):
    s = SYMS[name]
    top = s["rect"][1] if s["rect"] else 5.08
    pins = "".join('(pin "%s" (uuid "%s"))' % (num, U()) for _, num, *_ in s["pins"])
    if props == "top":
        (rx, ry), (vx, vy), just = (X, Y - top - 5.08), (X, Y - top - 2.54), None
    elif props == "right":
        (rx, ry), (vx, vy), just = (X + 3.81, Y - 1.27), (X + 3.81, Y + 1.27), "left"
    else:  # below
        bot = -s["rect"][3]
        (rx, ry), (vx, vy), just = (X, Y + bot + 2.54), (X, Y + bot + 5.08), None
    items.append(
        '(symbol (lib_id "Brazo:%s") (at %s %s 0) (unit 1) (exclude_from_sim no) (in_bom yes) (on_board yes) (dnp no) (uuid "%s")'
        '(property "Reference" "%s" (at %s %s 0) %s)'
        '(property "Value" "%s" (at %s %s 0) %s)'
        '(property "Footprint" "" (at %s %s 0) %s)(property "Datasheet" "" (at %s %s 0) %s)(property "Description" "" (at %s %s 0) %s)'
        '%s(instances (project "%s" (path "/%s" (reference "%s") (unit 1)))))'
        % (name, f(X), f(Y), U(), ref, f(rx), f(ry), eff(justify=just), value, f(vx), f(vy), eff(justify=just),
           f(X), f(Y), eff(hide=True), f(X), f(Y), eff(hide=True), f(X), f(Y), eff(hide=True),
           pins, PROJ, ROOT, ref))
    for n, num, x, y, a in s["pins"]:
        PIN_POS[(ref, n)] = (X + x, Y - y, a)


def wire(x1, y1, x2, y2):
    items.append('(wire (pts (xy %s %s) (xy %s %s)) (stroke (width 0) (type default)) (uuid "%s"))'
                 % (f(x1), f(y1), f(x2), f(y2), U()))


def net(ref, pin, label):
    """Cable corto desde el pin hacia afuera y etiqueta de red en su extremo."""
    x, y, a = PIN_POS[(ref, pin)]
    dx, dy = {0: (-1, 0), 180: (1, 0), 90: (0, 1), 270: (0, -1)}[a]
    x2, y2 = x + dx * 2 * G, y + dy * 2 * G
    wire(x, y, x2, y2)
    ang, just = {(1, 0): (0, "left bottom"), (-1, 0): (180, "right bottom"),
                 (0, 1): (270, "right bottom"), (0, -1): (90, "left bottom")}[(dx, dy)]
    items.append('(label "%s" (at %s %s %d) (fields_autoplaced yes) %s (uuid "%s"))'
                 % (label, f(x2), f(y2), ang, eff(justify=just), U()))


def nc(ref, pin):
    x, y, _ = PIN_POS[(ref, pin)]
    items.append('(no_connect (at %s %s) (uuid "%s"))' % (f(x), f(y), U()))


def text(t, x, y, size=1.5):
    t = t.replace('"', "'")
    items.append('(text "%s" (exclude_from_sim no) (at %s %s 0) %s (uuid "%s"))'
                 % (t, f(x), f(y), eff(size, justify="left bottom"), U()))


# Ubicacion (mm, A4 horizontal)
place("ESP32_DevKit", "U1", "ESP32 DevKit (WROOM-32)", 63.5, 76.2)
place("PCA9685_Module", "U2", "PCA9685 (I2C 0x40)", 152.4, 76.2)
place("VL53L0X_Module", "U3", "VL53L0X V2 (I2C 0x29)", 63.5, 134.62)
place("C_Polarizado", "C1", "1000uF 16V", 124.46, 124.46, props="right")
place("Fuente_Lab", "PS1", "Fuente laboratorio", 152.4, 137.16, props="below")
servos = [("M1", "Base q1 MG90S"), ("M2", "Hombro q2 MG90S"), ("M3", "Codo q3 MG90S"),
          ("M4", "Muneca q4 MG90S"), ("M5", "Pinza MG90S")]
for i, (ref, val) in enumerate(servos):
    place("Servo", ref, val, 238.76, 45.72 + i * 22.86)

# Redes
net("U1", "3V3", "+3V3"); net("U1", "GND", "GND"); net("U1", "GPIO21_SDA", "SDA"); net("U1", "GPIO22_SCL", "SCL")
net("U1", "GPIO18_SDA_TOF", "SDA_TOF"); net("U1", "GPIO19_SCL_TOF", "SCL_TOF")
nc("U1", "VIN")
for p, l in (("GND", "GND"), ("SCL", "SCL"), ("SDA", "SDA"), ("VCC", "+3V3")):
    net("U2", p, l)
nc("U2", "OE"); nc("U2", "V+")
for i in range(5):
    net("U2", "PWM%d" % i, "PWM%d" % i)
net("U2", "V+_SERVOS", "+6V"); net("U2", "GND_SERVOS", "GND")
net("U2", "V+_BORNERA", "+6V"); net("U2", "GND_BORNERA", "GND")
for p, l in (("VIN", "+3V3"), ("GND", "GND"), ("SCL", "SCL_TOF"), ("SDA", "SDA_TOF")):
    net("U3", p, l)
nc("U3", "GPIO1"); nc("U3", "XSHUT")
net("C1", "+", "+6V"); net("C1", "-", "GND")
net("PS1", "+", "+6V"); net("PS1", "-", "GND")
for i, (ref, _) in enumerate(servos):
    net(ref, "PWM", "PWM%d" % i); net(ref, "V+", "+6V"); net(ref, "GND", "GND")

# Notas
text("Brazo robotico 4 GDL - Lab. 1 Fundamentos de Robotica USACH 2026", 25.4, 25.4, 2.5)
text("Redes con el mismo nombre de etiqueta estan conectadas (GND, +3V3, +6V, SDA, SCL, SDA_TOF, SCL_TOF, PWM0..PWM4).", 25.4, 31.75)
text("Dos buses I2C: Wire (GPIO21/22) -> PCA9685  |  Wire1 (GPIO18/19) -> VL53L0X", 25.4, 36.83)
notes = [
    "NOTAS:",
    "1. Fuente ajustada a 6,0 V y limite 3 A con la salida APAGADA antes de conectar. Mas de 6 V quema los servos.",
    "2. U1 se alimenta solo por USB (PC, cargador o power bank). Su pin VIN queda libre.",
    "3. GND unica para todo el sistema: ESP32, PCA9685 (logica y bornera), VL53L0X, servos y fuente.",
    "4. U2: V+_BORNERA = bornera azul; V+_SERVOS / GND_SERVOS = pines rojo/cafe de cada conector de servo (misma pista interna).",
    "5. U2 V+ del conector logico, OE, U3 GPIO1 y XSHUT quedan sin conectar.",
    "6. C1 electrolitico en la bornera: pata larga (+) a +6V.",
]
for i, t in enumerate(notes):
    text(t, 25.4, 163.83 + i * 3.81, 1.3)

os.makedirs(OUT, exist_ok=True)
sch = ['(kicad_sch (version 20231120) (generator "eeschema") (generator_version "8.0") (uuid "%s") (paper "A4")' % ROOT,
       '(title_block (title "Brazo robotico 4 GDL - Conexiones") (date "2026-10-30") (rev "1") (company "USACH - Fundamentos de Robotica"))',
       "(lib_symbols"]
sch += [sym_def(n, "Brazo:") for n in SYMS]
sch.append(")")
sch += items
sch.append('(sheet_instances (path "/" (page "1"))))')
with open(os.path.join(OUT, PROJ + ".kicad_sch"), "w", encoding="utf-8") as fh:
    fh.write("\n".join(sch))

lib = ['(kicad_symbol_lib (version 20231120) (generator "kicad_symbol_editor") (generator_version "8.0")']
lib += [sym_def(n, "") for n in SYMS]
lib.append(")")
with open(os.path.join(OUT, "Brazo.kicad_sym"), "w", encoding="utf-8") as fh:
    fh.write("\n".join(lib))

with open(os.path.join(OUT, "sym-lib-table"), "w", encoding="utf-8") as fh:
    fh.write('(sym_lib_table (version 7)\n  (lib (name "Brazo")(type "KiCad")(uri "${KIPRJMOD}/Brazo.kicad_sym")(options "")(descr "Simbolos del brazo robotico"))\n)\n')

with open(os.path.join(OUT, PROJ + ".kicad_pro"), "w", encoding="utf-8") as fh:
    json.dump({"meta": {"filename": PROJ + ".kicad_pro", "version": 1}, "sheets": [[ROOT, "Root"]]}, fh, indent=2)
print("ok")
