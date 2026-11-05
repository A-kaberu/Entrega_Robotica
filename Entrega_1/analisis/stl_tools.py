"""
Herramientas para inspeccionar los STL del brazo (solo libreria estandar).

Uso:
  python stl_tools.py info  <archivo.stl> [...]       -> caja envolvente, volumen, masa estimada
  python stl_tools.py views <archivo.stl> <salida_dir> -> PNG de vistas ortogonales (X, Y, Z)
"""
import math
import os
import struct
import sys
import zlib

PLA_DENSITY = 1.24e-3  # g/mm^3


def load_stl(path):
    with open(path, "rb") as f:
        data = f.read()
    n = struct.unpack_from("<I", data, 80)[0] if len(data) >= 84 else 0
    tris = []
    if len(data) == 84 + 50 * n:
        for i in range(n):
            v = struct.unpack_from("<12f", data, 84 + 50 * i)
            tris.append((v[3:6], v[6:9], v[9:12]))
    else:  # STL ASCII
        verts = []
        for line in data.decode("utf-8", "ignore").splitlines():
            p = line.split()
            if p and p[0] == "vertex":
                verts.append(tuple(float(c) for c in p[1:4]))
        tris = [tuple(verts[i:i + 3]) for i in range(0, len(verts) - 2, 3)]
    return tris


def info(path):
    tris = load_stl(path)
    xs = [p[0] for t in tris for p in t]
    ys = [p[1] for t in tris for p in t]
    zs = [p[2] for t in tris for p in t]
    vol = 0.0
    for a, b, c in tris:
        vol += (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
                + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.0
    vol = abs(vol)
    print("%-28s tris=%6d  X[%7.1f,%7.1f] Y[%7.1f,%7.1f] Z[%7.1f,%7.1f]  dim=(%.1f x %.1f x %.1f) mm  "
          "vol=%.1f cm3  masa solida=%.1f g"
          % (os.path.basename(path), len(tris), min(xs), max(xs), min(ys), max(ys), min(zs), max(zs),
             max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs), vol / 1000, vol * PLA_DENSITY))


def write_png(path, w, h, rows):
    raw = b"".join(b"\x00" + bytes(r) for r in rows)

    def chunk(tag, payload):
        c = struct.pack(">I", len(payload)) + tag + payload
        return c + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF)

    png = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b"")
    with open(path, "wb") as f:
        f.write(png)


def render(tris, axis, path, scale=4.0):
    """Proyeccion ortogonal mirando a lo largo de -axis, sombreado por normal, con rejilla cada 10 mm."""
    u, v = [(1, 2), (0, 2), (0, 1)][axis]   # ejes de imagen (horizontal, vertical)
    pts = [p for t in tris for p in t]
    umin, umax = min(p[u] for p in pts), max(p[u] for p in pts)
    vmin, vmax = min(p[v] for p in pts), max(p[v] for p in pts)
    W, H = int((umax - umin) * scale) + 1, int((vmax - vmin) * scale) + 1
    depth = [[-1e9] * W for _ in range(H)]
    img = [[255] * W for _ in range(H)]
    for gx in range(math.ceil(umin / 10) * 10, int(umax) + 1, 10):   # rejilla
        X = int((gx - umin) * scale)
        for y in range(H):
            img[y][X] = 0 if gx == 0 else 200
    for gy in range(math.ceil(vmin / 10) * 10, int(vmax) + 1, 10):
        Y = H - 1 - int((gy - vmin) * scale)
        for x in range(W):
            img[Y][x] = 0 if gy == 0 else 200
    for a, b, c in tris:
        e1 = [b[i] - a[i] for i in range(3)]
        e2 = [c[i] - a[i] for i in range(3)]
        n = [e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]]
        ln = math.sqrt(sum(k * k for k in n)) or 1.0
        shade = int(60 + 150 * abs(n[axis]) / ln)
        P = [((p[u] - umin) * scale, (vmax - p[v]) * scale, p[axis]) for p in (a, b, c)]
        x0, x1 = max(int(min(p[0] for p in P)), 0), min(int(max(p[0] for p in P)) + 1, W - 1)
        y0, y1 = max(int(min(p[1] for p in P)), 0), min(int(max(p[1] for p in P)) + 1, H - 1)
        (ax, ay, az), (bx, by, bz), (cx, cy, cz) = P
        den = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy)
        if abs(den) < 1e-12:
            continue
        for y in range(y0, y1 + 1):
            for x in range(x0, x1 + 1):
                l1 = ((by - cy) * (x - cx) + (cx - bx) * (y - cy)) / den
                l2 = ((cy - ay) * (x - cx) + (ax - cx) * (y - cy)) / den
                l3 = 1 - l1 - l2
                if l1 >= -1e-6 and l2 >= -1e-6 and l3 >= -1e-6:
                    d = l1 * az + l2 * bz + l3 * cz
                    if d > depth[y][x]:
                        depth[y][x] = d
                        img[y][x] = shade
    write_png(path, W, H, img)
    print("vista %s -> %s  (horizontal=%s [%.0f..%.0f], vertical=%s [%.0f..%.0f], rejilla 10 mm, linea negra = 0)"
          % ("XYZ"[axis], path, "XYZ"[u], umin, umax, "XYZ"[v], vmin, vmax))


if __name__ == "__main__":
    cmd = sys.argv[1]
    if cmd == "info":
        for p in sys.argv[2:]:
            info(p)
    elif cmd == "views":
        tris = load_stl(sys.argv[2])
        base = os.path.splitext(os.path.basename(sys.argv[2]))[0].replace(" ", "_")
        os.makedirs(sys.argv[3], exist_ok=True)
        for ax in range(3):
            render(tris, ax, os.path.join(sys.argv[3], "%s_vista%s.png" % (base, "XYZ"[ax])))
