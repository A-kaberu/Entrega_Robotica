"""Toma el movimiento que registro la simulacion del firmware y prepara lo que se ve y se usa en el informe.

  1. Compara cada muestra con el modelo D-H de analisis/dh_brazo.py (dos implementaciones independientes).
  2. Crea resultados/simulaciones.html: visor con animacion, grafico q(t) y resultados de las pruebas.
  3. Crea resultados/q_t.csv y resultados/punta.csv: datos reducidos para las figuras del informe.

Uso desde la raiz del repo, despues de simular_firmware:   python simulacion/generar_visor.py
"""
import csv
import json
import math
import sys
from pathlib import Path

RAIZ = Path(__file__).resolve().parent
sys.path.insert(0, str(RAIZ.parent / "analisis"))
import dh_brazo as dh  # noqa: E402

RES = RAIZ / "resultados"
filas = list(csv.DictReader((RES / "rutina_3_cubos.csv").open(encoding="utf-8")))
t0 = float(filas[0]["t_ms"])

# 1. Firmware (C++) contra modelo (Python)
peor = 0.0
for f in filas:
    q = [float(f[k]) for k in ("q1", "q2", "q3", "q4")]
    punta = dh.fk(q)[2][4]
    peor = max(peor, math.dist(punta, (float(f["x"]), float(f["y"]), float(f["z"]))))
linea = f"Firmware vs modelo Python: {len(filas)} muestras, diferencia maxima en la punta {peor:.3f} mm"
print(linea)
(RES / "comparacion_modelo.txt").write_text(linea + "\n", encoding="utf-8")

# 2. Datos reducidos: una muestra cada 200 ms
reducidas = filas[::4]
with (RES / "q_t.csv").open("w", newline="", encoding="utf-8") as s:
    e = csv.writer(s)
    e.writerow(["t", "q1", "q2", "q3", "q4", "z"])
    for f in reducidas:
        e.writerow([round((float(f["t_ms"]) - t0) / 1000, 2), *(round(float(f[k]), 1) for k in ("q1", "q2", "q3", "q4", "z"))])
with (RES / "punta.csv").open("w", newline="", encoding="utf-8") as s:
    e = csv.writer(s)
    e.writerow(["x", "y"])
    for f in reducidas:
        e.writerow([round(float(f["x"]), 1), round(float(f["y"]), 1)])

# 3. Visor: una sola pagina, sin archivos externos
muestras = []
for f in filas[::2]:
    cubos = [[round(float(f[f"c{i}{e}"]), 1) for e in "xyz"] for i in (1, 2, 3) if f[f"c{i}x"]]
    muestras.append({"t": round((float(f["t_ms"]) - t0) / 1000, 2), "q": [round(float(f[k]), 1) for k in ("q1", "q2", "q3", "q4")],
                     "p": round(float(f["pinza"])), "c": cubos})
datos = {"muestras": muestras, "L": [dh.L1, dh.L2, dh.L3, dh.L4], "a1": dh.A1, "H": dh.H,
         "recogida": dh.C["RECOGIDA"], "pila": dh.C["PILA"], "cubo": dh.C["CUBO"],
         "resumen": (RES / "resumen.txt").read_text(encoding="utf-8"), "comparacion": linea,
         "eventos": (RES / "eventos_rutina.txt").read_text(encoding="utf-8")}

PAGINA = r"""<!doctype html><html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>Simulaciones del brazo</title>
<style>
:root{--f:#fff;--t:#1f2937;--s:#64748b;--l:#d7dce3;--a:#2563eb}
@media(prefers-color-scheme:dark){:root{--f:#151a22;--t:#e5e7eb;--s:#94a3b8;--l:#2c3440;--a:#60a5fa}}
*{box-sizing:border-box}body{margin:0;padding:16px;background:var(--f);color:var(--t);font:15px/1.45 system-ui,"Segoe UI",sans-serif}
h1{font-size:20px;margin:0 0 4px}h2{font-size:15px;margin:18px 0 6px}p{margin:4px 0;color:var(--s)}
.vistas{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:12px;margin-top:10px}
canvas{width:100%;border:1px solid var(--l);border-radius:8px;display:block}
.controles{display:flex;flex-wrap:wrap;gap:10px;align-items:center;margin:10px 0}
button{font:inherit;padding:6px 14px;border-radius:6px;border:1px solid var(--a);background:var(--a);color:#fff;cursor:pointer}
input[type=range]{flex:1;min-width:180px}pre{background:rgba(127,127,127,.1);padding:10px;border-radius:8px;overflow:auto;font-size:12px;max-height:340px;margin:0}
</style></head><body>
<h1>Simulaciones del firmware del brazo 4 GDL</h1>
<p>Rutina con 3 cubos: el firmware real se compiló en el PC con el hardware emulado. El mundo simulado es ideal (sin holguras ni deslizamiento): valida la lógica, no la mecánica.</p>
<div class="controles"><button id="play">▶ Reproducir</button><button id="png">Guardar imagen</button>
<input id="t" type="range" min="0" value="0"><span id="txt"></span></div>
<div class="vistas"><canvas id="lado" width="640" height="440"></canvas><canvas id="arriba" width="640" height="440"></canvas></div>
<h2>Ángulos de las articulaciones en el tiempo</h2><canvas id="graf" width="1300" height="300"></canvas>
<h2>Resultados de las pruebas</h2><pre id="res"></pre><h2>Eventos de la rutina (mensajes del firmware)</h2><pre id="ev"></pre>
<script>
const D=__DATOS__,M=D.muestras,rad=d=>d*Math.PI/180,$=i=>document.getElementById(i);
const COL=['#2563eb','#ea580c','#059669','#9333ea'],css=n=>getComputedStyle(document.body).color;
function fk(q){const[L1,L2,L3,L4]=D.L,c=rad(q[1]),d=rad(q[1]+q[2]),e=rad(q[1]+q[2]+q[3]);
 let r=[0,D.a1,D.a1+L2*Math.cos(c),D.a1+L2*Math.cos(c)+L3*Math.cos(d)];r.push(r[3]+L4*Math.cos(e));
 let z=[D.H,D.H+L1,D.H+L1+L2*Math.sin(c),D.H+L1+L2*Math.sin(c)+L3*Math.sin(d)];z.push(z[3]+L4*Math.sin(e));return{r,z,b:rad(q[0])}}
function ejes(g,w,h,f,x0,y0,e,tit){g.clearRect(0,0,w,h);g.strokeStyle='rgba(127,127,127,.2)';g.lineWidth=1;
 for(let v=-200;v<=300;v+=20){let[a]=f(v,0);g.beginPath();g.moveTo(a,0);g.lineTo(a,h);g.stroke();let[,b]=f(0,v);g.beginPath();g.moveTo(0,b);g.lineTo(w,b);g.stroke()}
 g.fillStyle=css();g.font='15px system-ui';g.fillText(tit,10,20)}
function brazo(g,P){g.lineCap='round';g.lineWidth=7;const c=['#64748b',COL[1],COL[2],COL[3]];for(let i=0;i<4;i++){g.strokeStyle=c[i];g.beginPath();g.moveTo(...P[i]);g.lineTo(...P[i+1]);g.stroke()}
 g.fillStyle='#fbbf24';P.slice(1,4).forEach(p=>{g.beginPath();g.arc(p[0],p[1],5,0,7);g.fill()});g.fillStyle='#dc2626';g.beginPath();g.arc(P[4][0],P[4][1],5,0,7);g.fill()}
function pintar(k){const m=M[k],a=fk(m.q);$('txt').textContent=`t = ${m.t.toFixed(1)} s · q = ${m.q.join(', ')}° · pinza ${m.p} %`;
 let cv=$('lado'),g=cv.getContext('2d'),W=cv.width,H=cv.height,s=2.3,f=(x,y)=>[60+x*s,H-30-y*s];
 ejes(g,W,H,f,0,0,0,'Vista lateral (radio r, altura z) [mm]');g.fillStyle='rgba(127,127,127,.5)';g.fillRect(...f(-20,D.H),40*s,D.H*s);g.strokeStyle=css();g.lineWidth=2;g.beginPath();g.moveTo(0,f(0,0)[1]);g.lineTo(W,f(0,0)[1]);g.stroke();
 m.c.forEach(c=>{let r=Math.hypot(c[0],c[1]),p=f(r-D.cubo/2,c[2]+D.cubo);g.fillStyle='rgba(234,88,12,.55)';g.fillRect(p[0],p[1],D.cubo*s,D.cubo*s)});
 brazo(g,a.r.map((r,i)=>f(r,a.z[i])));
 cv=$('arriba');g=cv.getContext('2d');s=1.6;f=(x,y)=>[cv.width/2-60+x*s,cv.height/2+y*-s+40];ejes(g,cv.width,cv.height,f,0,0,0,'Vista superior (x, y) [mm]');
 const cua=(x,y,col,n)=>{const p=f(x-D.cubo/2,y+D.cubo/2);g.strokeStyle=col;g.lineWidth=2;g.strokeRect(p[0],p[1],D.cubo*s,D.cubo*s);g.fillStyle=col;g.font='13px system-ui';g.fillText(n,p[0],p[1]-3)};
 D.recogida.forEach(([r,q],i)=>cua(r*Math.cos(rad(q)),r*Math.sin(rad(q)),COL[0],'P'+(i+1)));cua(D.pila[0]*Math.cos(rad(D.pila[1])),D.pila[0]*Math.sin(rad(D.pila[1])),COL[1],'Pila');
 m.c.forEach(c=>{const p=f(c[0]-D.cubo/2,c[1]+D.cubo/2);g.fillStyle='rgba(234,88,12,.55)';g.fillRect(p[0],p[1],D.cubo*s,D.cubo*s)});
 brazo(g,a.r.map(r=>f(r*Math.cos(a.b),r*Math.sin(a.b))));graf(k)}
function graf(k){const cv=$('graf'),g=cv.getContext('2d'),W=cv.width,H=cv.height,T=M[M.length-1].t,x=t=>50+(W-70)*t/T,y=q=>H-20-(H-40)*(q+130)/240;g.clearRect(0,0,W,H);g.font='13px system-ui';g.fillStyle=css();g.strokeStyle='rgba(127,127,127,.3)';
 for(let q=-120;q<=100;q+=40){g.beginPath();g.moveTo(50,y(q));g.lineTo(W-20,y(q));g.stroke();g.fillText(q+'°',8,y(q)+4)}
 for(let j=0;j<4;j++){g.strokeStyle=COL[j];g.lineWidth=2;g.beginPath();M.forEach((m,i)=>{i?g.lineTo(x(m.t),y(m.q[j])):g.moveTo(x(m.t),y(m.q[j]))});g.stroke();g.fillStyle=COL[j];g.fillText('q'+(j+1),60+j*40,16)}
 g.strokeStyle='#dc2626';g.lineWidth=1.5;g.beginPath();g.moveTo(x(M[k].t),0);g.lineTo(x(M[k].t),H);g.stroke()}
const S=$('t');S.max=M.length-1;S.oninput=()=>pintar(+S.value);let id=null;
$('play').onclick=()=>{if(id){clearInterval(id);id=null;$('play').textContent='▶ Reproducir';return}$('play').textContent='⏸ Pausa';id=setInterval(()=>{let k=(+S.value+1)%M.length;S.value=k;pintar(k)},100)};
$('png').onclick=()=>{const a=document.createElement('a');a.download=`simulacion_t${M[+S.value].t}.png`;a.href=$('lado').toDataURL();a.click()};
$('res').textContent=D.resumen+'\n'+D.comparacion;$('ev').textContent=D.eventos;pintar(0);
</script></body></html>"""
(RES / "simulaciones.html").write_text(PAGINA.replace("__DATOS__", json.dumps(datos, ensure_ascii=False)), encoding="utf-8")
print("Visor creado:", RES / "simulaciones.html")
