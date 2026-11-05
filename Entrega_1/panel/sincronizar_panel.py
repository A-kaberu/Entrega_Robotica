#!/usr/bin/env python3
"""
Mantiene panel_brazo.html al dia con el firmware.

  python panel/sincronizar_panel.py            reescribe el bloque <config.h> del panel y revisa
  python panel/sincronizar_panel.py --revisar  solo revisa (no escribe); sale con error si algo no calza

Que hace:
  1. Copia al panel las constantes de firmware/src/config.h que el panel necesita (geometria D-H, HOME,
     limites seguros, puntos de la tarea, rango de pulsos). Van entre las marcas // <config.h> y
     // </config.h> del panel: ese bloque no se edita a mano.
  2. Revisa que cada comando que el panel envia exista en el firmware (ejecutar() en main.cpp) y lista
     los comandos del firmware que el panel no tiene en un boton (se usan desde la consola).
  3. Revisa que el firmware siga imprimiendo los mensajes que el panel interpreta (T, D, E, AVISO, la
     respuesta de "us" y la de "estado"). Si alguno cambio, hay que actualizar procesarLinea() del panel.

Solo usa la libreria estandar. Correrlo despues de cada cambio en el firmware, antes del commit.
"""
import hashlib
import re
import sys
from pathlib import Path

RAIZ = Path(__file__).resolve().parent.parent
CONFIG = RAIZ / "firmware" / "src" / "config.h"
MAIN = RAIZ / "firmware" / "src" / "main.cpp"
PANEL = RAIZ / "panel" / "panel_brazo.html"
INI, FIN = "// <config.h>", "// </config.h>"

# Mensajes del firmware que procesarLinea() del panel interpreta: (para que sirve, regex sobre main.cpp)
MENSAJES = [
    ("telemetria T (q1..q4, pinza y O1..O4: 17 valores)",
     r'"T,%\.1f,%\.1f,%\.1f,%\.1f,%\.0f"[\s\S]{0,200}?i <= 4[\s\S]{0,120}?",%\.1f,%\.1f,%\.1f"'),
    ("distancia del ToF D,<mm>", r'"D,%u"'),
    ("eventos de rutina y pruebas E,...", r'"E,'),
    ("avisos de proteccion AVISO,...", r'"AVISO,'),
    ("servos sueltos (OK,servos libres)", r'"OK,servos libres'),
    ("respuesta de us: '<nombre> -> <us> us'", r'"%s -> %d us'),
    ("respuesta de estado: '  <nombre> <us> us' o 'sin senal'", r'"  %-7s %5d us[\s\S]{0,200}?sin senal'),
    ("respuesta de 'telem 1' (barrera de la rutina de la app)", r'"OK,telemetria USB %s[\s\S]{0,80}?"activada"'),
]
APP = RAIZ / "app_pc" / "brazo_control.py"


def leer(path):
    return path.read_text(encoding="utf-8")


def numeros(cfg, nombre):
    """Numeros del inicializador de una constante de config.h (escalar o arreglo)."""
    m = re.search(rf"\b{nombre}\b[^=;]*=\s*([^;]+);", cfg)
    if not m:
        sys.exit(f"config.h: no encuentro {nombre}")
    return [float(x) for x in re.findall(r"-?\d+(?:\.\d+)?", m.group(1))]


def js(v):
    return f"{v:g}"


def pares(vals):
    return "[" + ", ".join(f"[{js(a)}, {js(b)}]" for a, b in zip(vals[::2], vals[1::2])) + "]"


def bloque_config():
    cfg = leer(CONFIG)
    huella = hashlib.sha1(cfg.encode()).hexdigest()[:8]
    L = {n: js(numeros(cfg, n)[0]) for n in ("L1", "L2", "L3", "L4", "A1_DH", "H_BASE")}
    nombres = re.search(r"NOMBRES\[NUM_SERVOS\]\s*=\s*\{([^}]*)\}", cfg).group(1)
    lineas = [
        f"{INI} generado por panel/sincronizar_panel.py desde firmware/src/config.h (huella {huella}): no editar a mano",
        f"const L1 = {L['L1']}, L2 = {L['L2']}, L3 = {L['L3']}, L4 = {L['L4']}, A1 = {L['A1_DH']}, H_BASE = {L['H_BASE']};",
        f"const Q_HOME = [{', '.join(js(v) for v in numeros(cfg, 'Q_HOME'))}];",
        f"const Q_SEGURO = {pares(numeros(cfg, 'Q_SEGURO'))};   // [min, max] en grados D-H de q1..q4",
        f"const RECOGIDA = {pares(numeros(cfg, 'PUNTOS_RECOGIDA'))}, PILA = [{', '.join(js(v) for v in numeros(cfg, 'PUNTO_APILADO'))}];   // [r mm, q1 grados]",
        f"const US_ABS_MIN = {js(numeros(cfg, 'US_ABS_MIN')[0])}, US_ABS_MAX = {js(numeros(cfg, 'US_ABS_MAX')[0])};",
        f"const NOMBRES = [{', '.join(n.strip() for n in nombres.split(','))}];",
        f"const CONFIG_H = '{huella}';",
        FIN,
    ]
    return "\n".join(lineas)


def comandos_firmware(main):
    cuerpo = main[main.index("void ejecutar(String l)"):]
    cmds = set(re.findall(r'k == "(\w+)"', cuerpo))
    cmds |= set(re.findall(r'l == "(\w+)"', main))   # A, y las de esParada / esSoltar
    cmds |= set("BHCMP")                              # sliders B/H/C/M/P<0-180>
    if re.search(r"else \{\s*ayuda\(\);", cuerpo):    # un comando desconocido muestra la ayuda
        cmds.add("ayuda")
    return cmds


def comandos_panel(html):
    """Primera palabra de cada comando que el panel envia (botones, asistente y codigo)."""
    textos = re.findall(r'data-cmd="([^"]+)"', html)
    textos += re.findall(r"""(?:enviar|enc\.encode)\(\s*([`'"])(.+?)\1""", html)
    textos = [t[1] if isinstance(t, tuple) else t for t in textos]
    textos += re.findall(r"""['`]((?:cal|us|obj) [^'`]*)['`]""", html)   # cmds: [...] del asistente
    usados = set()
    for t in textos:
        t = re.sub(r"\$\{[^}]*\}", "", t).replace("\\n", "").strip()
        if t:
            usados.add(t.split()[0])
    return usados


def comandos_app(py):
    """Primera palabra de cada comando que la app de Python envia."""
    textos = re.findall(r"""\benviar\(\s*f?(["'])(.+?)\1""", py)
    textos = [t[1] for t in textos]
    textos += re.findall(r"""^BARRERA = (["'])(.+?)\1""", py, re.M)
    textos = [t[1] if isinstance(t, tuple) else t for t in textos]
    textos += re.findall(r"""\(["'][^"']+["'], ["']((?:cal|verificar|prueba)[^"']*)["']\)""", py)   # botones (texto, cmd)
    usados = set()
    for t in textos:
        t = re.sub(r"\{[^}]*\}", "", t).strip()
        if t:
            usados.add(t.split()[0])
    return usados


def main():
    revisar = "--revisar" in sys.argv
    html, mainc = leer(PANEL), leer(MAIN)
    errores = []

    nuevo = bloque_config()
    m = re.search(re.escape(INI) + r"[\s\S]*?" + re.escape(FIN), html)
    if not m:
        sys.exit(f"panel: no encuentro las marcas {INI} ... {FIN}")
    if m.group(0) != nuevo:
        if revisar:
            errores.append("las constantes del panel no coinciden con config.h: corre sin --revisar")
        else:
            html = html[:m.start()] + nuevo + html[m.end():]
            PANEL.write_text(html, encoding="utf-8", newline="\n")
            print("panel: bloque <config.h> actualizado")
    else:
        print("panel: constantes al dia con config.h")

    fw, pn = comandos_firmware(mainc), comandos_panel(html)
    for c in sorted(pn - fw):
        errores.append(f"el panel envia '{c}', que el firmware no reconoce")
    # Los sliders se envian como k + valor (no es un texto fijo) y S/stop/off/soltar son alias de a y X
    sin_boton = sorted(fw - pn - set("BHCMP") - {"S", "stop", "off", "soltar"})
    print("comandos del firmware sin boton en el panel (solo consola):", ", ".join(sin_boton) or "ninguno")
    if APP.exists():   # la app de Python lee config.h sola (dh_brazo.py); aqui se revisan sus comandos
        app = comandos_app(leer(APP))
        for c in sorted(app - fw):
            errores.append(f"la app de Python envia '{c}', que el firmware no reconoce")
        print("comandos que usa la app de Python:", ", ".join(sorted(app)))

    for desc, rx in MENSAJES:
        if not re.search(rx, mainc):
            errores.append(f"el firmware ya no imprime: {desc} -> revisar procesarLinea() del panel")

    for e in errores:
        print("ERROR:", e)
    print("OK: panel sincronizado con el firmware" if not errores else f"{len(errores)} problema(s)")
    sys.exit(1 if errores else 0)


if __name__ == "__main__":
    main()
