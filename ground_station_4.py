import tkinter as tk
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText
import serial, serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from collections import deque
import cv2
import numpy as np
from PIL import Image, ImageTk
import csv, os, time, json, io, struct, threading
from datetime import datetime

# ════════════════════════════════════════════════
#  COLORES — Estilo HUD aeroespacial oscuro
# ════════════════════════════════════════════════
C = {
    "bg":       "#050b14",
    "panel":    "#0a1628",
    "border":   "#0e3a6e",
    "accent":   "#00d4ff",
    "accent2":  "#ff6b35",
    "green":    "#00ff9f",
    "yellow":   "#ffd60a",
    "red":      "#ff2d55",
    "text":     "#c8dff5",
    "dim":      "#4a6a8a",
    "grid":     "#0d2137",
}

# ════════════════════════════════════════════════
#  SERIAL — detecta automáticamente el puerto
# ════════════════════════════════════════════════
def detectar_puerto():
    puertos = list(serial.tools.list_ports.comports())
    return puertos[0].device if puertos else None

PUERTO = detectar_puerto()
try:
    ser = serial.Serial(PUERTO, 115200, timeout=0.1) if PUERTO else None
except Exception:
    ser = None

# ════════════════════════════════════════════════
#  DATOS TELEMETRÍA
# ════════════════════════════════════════════════
MAX_LEN = 100
KEYS    = ["temp", "pres", "alt", "yaw", "pitch", "roll"]
data    = {k: deque([0.0] * MAX_LEN, maxlen=MAX_LEN) for k in KEYS}
log     = []

session_start = datetime.now().strftime("%Y%m%d_%H%M%S")
csv_path      = f"cansat_log_{session_start}.csv"

with open(csv_path, "w", newline="") as f:
    csv.writer(f).writerow(["timestamp", "temp", "pres", "alt", "yaw", "pitch", "roll"])

def guardar_fila(vals):
    with open(csv_path, "a", newline="") as f:
        csv.writer(f).writerow([datetime.now().isoformat()] + vals)

# ════════════════════════════════════════════════
#  ESTADO RECEPCIÓN DE IMAGEN
# ════════════════════════════════════════════════
# Buffer binario acumulado para detectar paquetes de imagen
_img_buffer   = bytearray()
_img_fragments = {}   # cam_id → {idx: bytes}
_img_totales   = {}   # cam_id → n_frags esperados
_img_sizes     = {}   # cam_id → size_total en bytes
_ultima_imagen = None  # ImageTk para mostrar en pantalla

PKT_TELEMETRY   = 0x01
PKT_FOTO_INICIO = 0x02
PKT_FOTO_FRAG   = 0x03
PKT_FOTO_FIN    = 0x04
MAGIC           = bytes([0xDE, 0xAD, 0xBE, 0xEF])
FRAG_SIZE       = 24  # debe coincidir con el C3

# ════════════════════════════════════════════════
#  VENTANA PRINCIPAL
# ════════════════════════════════════════════════
root = tk.Tk()
root.title("🛰  MouseRat CanSat · Ground Station")
root.geometry("1400x860")
root.configure(bg=C["bg"])
root.resizable(True, True)

FONT_MONO  = ("Courier New", 10)
FONT_TITLE = ("Courier New", 11, "bold")
FONT_BIG   = ("Courier New", 22, "bold")
FONT_MED   = ("Courier New", 13, "bold")
FONT_SM    = ("Courier New", 9)

# ════════════════════════════════════════════════
#  HELPERS UI
# ════════════════════════════════════════════════
def panel(parent, row, col, rowspan=1, colspan=1, padx=6, pady=6):
    outer = tk.Frame(parent, bg=C["border"], bd=0)
    outer.grid(row=row, column=col, rowspan=rowspan, columnspan=colspan,
               sticky="nsew", padx=padx, pady=pady)
    inner = tk.Frame(outer, bg=C["panel"], bd=0)
    inner.pack(fill="both", expand=True, padx=1, pady=1)
    return inner

def sec_title(parent, text, color=None):
    color = color or C["accent"]
    tk.Label(parent, text=f"◈ {text}", fg=color, bg=C["panel"],
             font=FONT_TITLE, anchor="w").pack(fill="x", padx=8, pady=(6, 2))
    tk.Frame(parent, bg=color, height=1).pack(fill="x", padx=8)

# ════════════════════════════════════════════════
#  LAYOUT GRID
# ════════════════════════════════════════════════
root.grid_rowconfigure(0, weight=0)
root.grid_rowconfigure(1, weight=4)
root.grid_rowconfigure(2, weight=2)
root.grid_columnconfigure(0, weight=3)
root.grid_columnconfigure(1, weight=2)
root.grid_columnconfigure(2, weight=3)

# ────────────────────────────────────────────────
#  BARRA SUPERIOR
# ────────────────────────────────────────────────
topbar = tk.Frame(root, bg=C["bg"], height=46)
topbar.grid(row=0, column=0, columnspan=3, sticky="ew", padx=6, pady=(6, 0))

tk.Label(topbar, text="▶  MOUSERATCAN · GROUND STATION",
         fg=C["accent"], bg=C["bg"], font=("Courier New", 14, "bold")).pack(side="left", padx=14)

status_var = tk.StringVar(value="● SIN SEÑAL")
status_lbl = tk.Label(topbar, textvariable=status_var,
                      fg=C["red"], bg=C["bg"], font=FONT_TITLE)
status_lbl.pack(side="left", padx=20)

mission_time = tk.StringVar(value="T+ 00:00:00")
tk.Label(topbar, textvariable=mission_time,
         fg=C["yellow"], bg=C["bg"], font=FONT_MED).pack(side="right", padx=14)

port_lbl = tk.Label(topbar, text=f"PORT: {PUERTO or 'N/A'}",
                    fg=C["dim"], bg=C["bg"], font=FONT_SM)
port_lbl.pack(side="right", padx=14)

t_inicio    = time.time()
_after_ids  = {}

def tick_clock():
    try:
        elapsed = int(time.time() - t_inicio)
        h, r = divmod(elapsed, 3600)
        m, s = divmod(r, 60)
        mission_time.set(f"T+ {h:02d}:{m:02d}:{s:02d}")
        _after_ids["clock"] = root.after(1000, tick_clock)
    except tk.TclError:
        pass

# ────────────────────────────────────────────────
#  PANEL IMAGEN (col 0, row 1)
# ────────────────────────────────────────────────
img_pnl = panel(root, 1, 0)
sec_title(img_pnl, "IMAGEN ESTEREOSCÓPICA", C["accent"])

img_label = tk.Label(img_pnl, bg="black",
                     text="Esperando imagen...",
                     fg=C["dim"], font=FONT_MONO)
img_label.pack(fill="both", expand=True, padx=4, pady=4)

img_status_var = tk.StringVar(value="Sin imagen recibida")
tk.Label(img_pnl, textvariable=img_status_var,
         fg=C["dim"], bg=C["panel"], font=FONT_SM).pack(pady=(0, 4))

def guardar_imagen_actual():
    """Guarda la imagen de disparidad mostrada actualmente."""
    global _ultima_imagen_raw
    if _ultima_imagen_raw is None:
        log_msg("No hay imagen para guardar.")
        return
    nombre = f"disparidad_{datetime.now().strftime('%H%M%S')}.jpg"
    with open(nombre, "wb") as f:
        f.write(_ultima_imagen_raw)
    log_msg(f"Imagen guardada: {nombre}")

_ultima_imagen_raw = None  # bytes del JPEG de disparidad

tk.Button(img_pnl, text="⬇  GUARDAR IMAGEN", bg=C["accent"], fg=C["bg"],
          font=FONT_TITLE, relief="flat", bd=0, cursor="hand2",
          command=guardar_imagen_actual, pady=4).pack(fill="x", padx=10, pady=(0, 6))

def mostrar_imagen(jpg_bytes: bytes):
    """Muestra el JPEG de disparidad en el panel."""
    global _ultima_imagen, _ultima_imagen_raw
    try:
        _ultima_imagen_raw = jpg_bytes
        arr  = np.frombuffer(jpg_bytes, np.uint8)
        img  = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        img  = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        h = max(img_label.winfo_height(), 200)
        w = max(img_label.winfo_width(),  300)
        img = cv2.resize(img, (w, h))

        photo = ImageTk.PhotoImage(Image.fromarray(img))
        img_label.config(image=photo, text="")
        img_label.img = photo
        _ultima_imagen = photo
        img_status_var.set(f"Última imagen: {datetime.now().strftime('%H:%M:%S')}  ({len(jpg_bytes)} bytes)")
    except Exception as e:
        log_msg(f"Error mostrando imagen: {e}")

def procesar_disparidad(buf1: bytes, buf2: bytes) -> bytes | None:
    """Genera mapa de disparidad a partir de dos JPEGs."""
    try:
        arr1 = np.frombuffer(buf1, np.uint8)
        arr2 = np.frombuffer(buf2, np.uint8)
        im1  = cv2.imdecode(arr1, cv2.IMREAD_COLOR)
        im2  = cv2.imdecode(arr2, cv2.IMREAD_COLOR)
        if im1 is None or im2 is None:
            return None

        W, H = 160, 120
        im1 = cv2.resize(im1, (W, H))
        im2 = cv2.resize(im2, (W, H))

        g1 = cv2.cvtColor(im1, cv2.COLOR_BGR2GRAY)
        g2 = cv2.cvtColor(im2, cv2.COLOR_BGR2GRAY)

        stereo    = cv2.StereoBM_create(numDisparities=32, blockSize=11)
        disp      = stereo.compute(g1, g2)
        disp_norm = cv2.normalize(disp, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        color_map = cv2.applyColorMap(disp_norm, cv2.COLORMAP_JET)

        ok, buf = cv2.imencode('.jpg', color_map, [cv2.IMWRITE_JPEG_QUALITY, 60])
        return buf.tobytes() if ok else None
    except Exception as e:
        log_msg(f"Error en disparidad: {e}")
        return None

# ────────────────────────────────────────────────
#  TELEMETRÍA EN VIVO (col 1, row 1)
# ────────────────────────────────────────────────
telem_pnl = panel(root, 1, 1)
sec_title(telem_pnl, "TELEMETRÍA", C["green"])

DISPLAY = [
    ("temp",  "TEMPERATURA", "°C",  C["accent2"]),
    ("pres",  "PRESIÓN",     "hPa", C["accent"]),
    ("alt",   "ALTITUD",     "m",   C["green"]),
    ("yaw",   "YAW",         "°",   C["yellow"]),
    ("pitch", "PITCH",       "°",   C["yellow"]),
    ("roll",  "ROLL",        "°",   C["yellow"]),
]

value_vars = {}
for key, label, unit, color in DISPLAY:
    row_f = tk.Frame(telem_pnl, bg=C["panel"])
    row_f.pack(fill="x", padx=10, pady=3)
    tk.Label(row_f, text=label, fg=C["dim"], bg=C["panel"],
             font=FONT_SM, width=12, anchor="w").pack(side="left")
    var = tk.StringVar(value="--.-")
    value_vars[key] = var
    tk.Label(row_f, textvariable=var, fg=color, bg=C["panel"],
             font=FONT_MED, width=9, anchor="e").pack(side="right")
    tk.Label(row_f, text=unit, fg=C["dim"], bg=C["panel"],
             font=FONT_SM, width=5, anchor="w").pack(side="right")
    tk.Frame(telem_pnl, bg=C["grid"], height=1).pack(fill="x", padx=10)

# Calibración BNO055
cal_frame = tk.Frame(telem_pnl, bg=C["panel"])
cal_frame.pack(fill="x", padx=10, pady=(8, 2))
tk.Label(cal_frame, text="CALIB", fg=C["dim"], bg=C["panel"],
         font=FONT_SM).pack(side="left")
cal_vars = {}
for c_name in ["sys", "gyro", "accel", "mag"]:
    tk.Label(cal_frame, text=f" {c_name.upper()}:", fg=C["dim"],
             bg=C["panel"], font=FONT_SM).pack(side="left")
    v = tk.StringVar(value="-")
    cal_vars[c_name] = v
    tk.Label(cal_frame, textvariable=v, fg=C["green"],
             bg=C["panel"], font=FONT_SM, width=2).pack(side="left")

pk_var = tk.StringVar(value="PAQUETES: 0")
tk.Label(telem_pnl, textvariable=pk_var, fg=C["dim"],
         bg=C["panel"], font=FONT_SM).pack(side="bottom", pady=6)
paquetes = [0]

# ────────────────────────────────────────────────
#  GRÁFICAS (col 2, row 1)
# ────────────────────────────────────────────────
graph_pnl = panel(root, 1, 2)
sec_title(graph_pnl, "GRÁFICAS", C["accent"])

GRAPH_KEYS   = ["temp", "pres", "alt", "pitch", "roll"]
GRAPH_COLORS = [C["accent2"], C["accent"], C["green"], C["yellow"], C["red"]]
GRAPH_LABELS = ["Temp (°C)", "Pres (hPa)", "Alt (m)", "Pitch (°)", "Roll (°)"]

fig, axs = plt.subplots(len(GRAPH_KEYS), 1, figsize=(4, 6))
fig.patch.set_facecolor(C["panel"])
fig.subplots_adjust(hspace=0.7, left=0.15, right=0.97, top=0.97, bottom=0.04)

lines = []
for i, ax in enumerate(axs):
    ax.set_facecolor(C["bg"])
    ax.tick_params(colors=C["dim"], labelsize=6)
    ax.set_title(GRAPH_LABELS[i], color=GRAPH_COLORS[i], fontsize=7,
                 loc="left", pad=2, fontfamily="monospace")
    for spine in ax.spines.values():
        spine.set_edgecolor(C["border"])
    ax.yaxis.set_major_locator(mticker.MaxNLocator(nbins=3))
    line, = ax.plot(list(data[GRAPH_KEYS[i]]), color=GRAPH_COLORS[i], linewidth=1.2)
    ax.fill_between(range(MAX_LEN), list(data[GRAPH_KEYS[i]]),
                    alpha=0.08, color=GRAPH_COLORS[i])
    lines.append(line)

canvas_graph = FigureCanvasTkAgg(fig, master=graph_pnl)
canvas_graph.get_tk_widget().pack(fill="both", expand=True, padx=4, pady=(4, 0))
canvas_graph.get_tk_widget().configure(bg=C["panel"])

def guardar_graficas():
    """Guarda las gráficas actuales como PNG."""
    nombre = f"graficas_{datetime.now().strftime('%H%M%S')}.png"
    fig.savefig(nombre, dpi=150, facecolor=C["panel"], bbox_inches="tight")
    log_msg(f"Gráficas guardadas: {nombre}")

tk.Button(graph_pnl, text="⬇  GUARDAR GRÁFICAS", bg=C["accent"], fg=C["bg"],
          font=FONT_TITLE, relief="flat", bd=0, cursor="hand2",
          command=guardar_graficas, pady=4).pack(fill="x", padx=10, pady=(2, 6))

# ────────────────────────────────────────────────
#  TERMINAL + CONTROLES (row 2)
# ────────────────────────────────────────────────
bottom = tk.Frame(root, bg=C["bg"])
bottom.grid(row=2, column=0, columnspan=3, sticky="nsew", padx=6, pady=6)
bottom.grid_columnconfigure(0, weight=1)
bottom.grid_columnconfigure(1, weight=0)
bottom.grid_rowconfigure(0, weight=1)

term_outer = tk.Frame(bottom, bg=C["border"])
term_outer.grid(row=0, column=0, sticky="nsew", padx=(0, 4))
term_inner = tk.Frame(term_outer, bg=C["panel"])
term_inner.pack(fill="both", expand=True, padx=1, pady=1)
sec_title(term_inner, "CONSOLA SERIAL", C["dim"])

terminal = ScrolledText(term_inner, height=7, bg=C["bg"], fg=C["green"],
                        insertbackground=C["green"], font=FONT_MONO,
                        bd=0, relief="flat")
terminal.pack(fill="both", expand=True, padx=6, pady=4)

def log_msg(msg):
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    terminal.insert(tk.END, f"[{ts}] {msg}\n")
    terminal.see(tk.END)

# Panel de controles
ctrl_outer = tk.Frame(bottom, bg=C["border"], width=220)
ctrl_outer.grid(row=0, column=1, sticky="ns")
ctrl_outer.pack_propagate(False)
ctrl_inner = tk.Frame(ctrl_outer, bg=C["panel"])
ctrl_inner.pack(fill="both", expand=True, padx=1, pady=1)
sec_title(ctrl_inner, "CONTROL", C["accent2"])

def make_btn(parent, text, color, command):
    btn = tk.Button(parent, text=text, bg=color, fg=C["bg"],
                    font=FONT_TITLE, relief="flat", bd=0,
                    activebackground=C["text"], cursor="hand2",
                    command=command, pady=8)
    btn.pack(fill="x", padx=10, pady=5)
    return btn

def guardar_manual():
    snap = f"snapshot_{datetime.now().strftime('%H%M%S')}.csv"
    with open(snap, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["timestamp", "temp", "pres", "alt", "yaw", "pitch", "roll"])
        w.writerows(log)
    log_msg(f"Snapshot guardado: {snap}")

def borrar():
    log.clear()
    for k in data:
        data[k].clear()
        data[k].extend([0.0] * MAX_LEN)
    paquetes[0] = 0
    pk_var.set("PAQUETES: 0")
    terminal.delete("1.0", tk.END)
    log_msg("Buffer limpiado.")

make_btn(ctrl_inner, "⬇  SNAPSHOT CSV",    C["green"], guardar_manual)
make_btn(ctrl_inner, "⬇  GUARDAR GRÁFICAS", C["accent"], guardar_graficas)
make_btn(ctrl_inner, "⬇  GUARDAR IMAGEN",   C["accent2"], guardar_imagen_actual)
make_btn(ctrl_inner, "✕  LIMPIAR BUFFER",   C["red"], borrar)

tk.Label(ctrl_inner, text=f"Log: {os.path.basename(csv_path)}",
         fg=C["dim"], bg=C["panel"], font=FONT_SM,
         wraplength=200).pack(padx=10, pady=(4, 2))
tk.Label(ctrl_inner, text="(auto-guardado por paquete)",
         fg=C["dim"], bg=C["panel"], font=FONT_SM).pack(padx=10)

# ════════════════════════════════════════════════
#  PARSEO DE TELEMETRÍA JSON
# ════════════════════════════════════════════════
def procesar_json(line: str):
    """Parsea una línea JSON de telemetría y actualiza la UI."""
    try:
        d = json.loads(line)

        # Campos requeridos
        temp  = float(d.get("temp",     0))
        pres  = float(d.get("pressure", 0))
        alt   = float(d.get("altitude", 0))
        yaw   = float(d.get("yaw",      0))
        pitch = float(d.get("pitch",    0))
        roll  = float(d.get("roll",     0))

        vals = [temp, pres, alt, yaw, pitch, roll]

        for k, val in zip(KEYS, vals):
            data[k].append(val)

        for key, _, _, _ in DISPLAY:
            idx = KEYS.index(key)
            value_vars[key].set(f"{vals[idx]:.2f}")

        # Calibración
        calib = d.get("calib", {})
        for c_name in ["sys", "gyro", "accel", "mag"]:
            cal_vars[c_name].set(str(calib.get(c_name, "-")))

        log.append(vals)
        guardar_fila(vals)

        paquetes[0] += 1
        pk_var.set(f"PAQUETES: {paquetes[0]}")

        status_var.set("● EN LÍNEA")
        status_lbl.config(fg=C["green"])

        # Actualizar gráficas
        for i, gk in enumerate(GRAPH_KEYS):
            y = list(data[gk])
            lines[i].set_ydata(y)
            lines[i].set_xdata(range(len(y)))
            axs[i].relim()
            axs[i].autoscale_view()
            # Compatible con todas las versiones de matplotlib
            for coll in axs[i].collections[:]:
                coll.remove()
            axs[i].fill_between(range(len(y)), y, alpha=0.08, color=GRAPH_COLORS[i])

        canvas_graph.draw_idle()

    except (json.JSONDecodeError, KeyError, ValueError):
        pass  # No era JSON de telemetría, ignorar

# ════════════════════════════════════════════════
#  PROCESAMIENTO DE FRAGMENTOS DE IMAGEN
# ════════════════════════════════════════════════
def intentar_reconstruir(cam_id: int):
    """Si tenemos todos los fragmentos de una cámara, reconstruye la foto."""
    if cam_id not in _img_totales:
        return
    n_total   = _img_totales[cam_id]
    frags     = _img_fragments.get(cam_id, {})
    recibidos = len(frags)

    # Mostrar progreso
    if n_total > 0:
        perdidos = [i for i in range(n_total) if i not in frags]
        pct      = recibidos / n_total * 100
        msg      = f"CAM{cam_id} paq {recibidos}/{n_total} ({pct:.0f}%)"
        if perdidos and recibidos < n_total:
            msg += f" — perdidos: {perdidos[:5]}{'...' if len(perdidos) > 5 else ''}"
        log_msg(msg)

    # Reconstruir si llegaron todos, o si ya llegó PKT_FIN y tenemos al menos 80%
    if recibidos < n_total and recibidos < n_total * 0.8:
        return  # Esperar más fragmentos

    if recibidos < n_total:
        log_msg(f"CAM{cam_id} reconstruyendo con {recibidos}/{n_total} frags (faltan {n_total - recibidos})")
    
    # Ensamblar en orden — rellenar huecos con ceros
    partes = []
    for i in range(n_total):
        partes.append(frags.get(i, bytes(FRAG_SIZE)))  # ceros si falta
    jpg_bytes = b"".join(partes)
    log_msg(f"CAM{cam_id} reconstruida: {len(jpg_bytes)} bytes")

    # Si tenemos ambas cámaras, generar disparidad
    otras = [c for c in _img_fragments if c != cam_id and c in _img_totales]
    for otra in otras:
        if len(_img_fragments[otra]) >= _img_totales[otra]:
            buf_otra = b"".join(_img_fragments[otra][i] for i in range(_img_totales[otra]))
            log_msg("Generando mapa de disparidad...")
            resultado = procesar_disparidad(jpg_bytes if cam_id == 1 else buf_otra,
                                            buf_otra   if cam_id == 1 else jpg_bytes)
            if resultado:
                root.after(0, lambda r=resultado: mostrar_imagen(r))
                log_msg(f"Disparidad lista: {len(resultado)} bytes")
            # Limpiar fragmentos usados
            _img_fragments.pop(1, None)
            _img_fragments.pop(2, None)
            _img_totales.pop(1, None)
            _img_totales.pop(2, None)
            return

    # Si solo llegó una cámara, mostrar esa sola
    root.after(0, lambda b=jpg_bytes: mostrar_imagen(b))

# ════════════════════════════════════════════════
#  BUCLE PRINCIPAL DE LECTURA SERIAL
#
#  PROTOCOLO:
#  - Telemetría llega como JSON texto terminado en \n
#  - Paquetes de imagen llegan con magic header:
#      0xDE 0xAD 0xBE 0xEF + tipo(1) + payload
#    Esto evita confundir bytes de JPEG con tipos de paquete
# ════════════════════════════════════════════════

# Tamaños de payload después del magic+tipo
SIZEOF_INICIO_PAYLOAD = 7
SIZEOF_FRAG_PAYLOAD   = 31
SIZEOF_FIN_PAYLOAD    = 1

_raw_buf  = bytearray()
_text_buf = ""

def procesar_chunk(chunk: bytes):
    """Procesa bytes del serial separando JSON de paquetes binarios."""
    global _text_buf, _raw_buf

    _raw_buf.extend(chunk)

    while _raw_buf:
        # Buscar magic header en el buffer
        magic_pos = _raw_buf.find(MAGIC)

        if magic_pos == -1:
            # No hay magic → todo es texto (o basura no-JSON)
            texto = _raw_buf.decode('utf-8', errors='ignore')
            _raw_buf.clear()
            for ch in texto:
                if ch == '\n':
                    line = _text_buf.strip()
                    _text_buf = ""
                    if line:
                        log_msg(line)
                        procesar_json(line)
                else:
                    _text_buf += ch
            return

        # Hay texto antes del magic → procesarlo como JSON
        if magic_pos > 0:
            texto_previo = _raw_buf[:magic_pos].decode('utf-8', errors='ignore')
            for ch in texto_previo:
                if ch == '\n':
                    line = _text_buf.strip()
                    _text_buf = ""
                    if line:
                        log_msg(line)
                        procesar_json(line)
                else:
                    _text_buf += ch
            _raw_buf = _raw_buf[magic_pos:]

        # Ahora _raw_buf empieza con el magic
        # Necesitamos al menos magic(4) + tipo(1) = 5 bytes para saber qué es
        if len(_raw_buf) < 5:
            return  # Esperar más bytes

        tipo = _raw_buf[4]

        if tipo == PKT_FOTO_INICIO:
            needed = 4 + 1 + SIZEOF_INICIO_PAYLOAD
            if len(_raw_buf) < needed:
                return
            cam_id     = _raw_buf[5]
            size_total = struct.unpack_from("<I", _raw_buf, 6)[0]
            n_frags    = struct.unpack_from("<H", _raw_buf, 10)[0]
            # Validar valores razonables
            if size_total > 200_000 or n_frags > 10_000:
                log_msg(f"PKT_INICIO inválido, descartando")
                _raw_buf = _raw_buf[1:]
                continue
            _img_totales[cam_id]   = n_frags
            _img_sizes[cam_id]     = size_total
            _img_fragments[cam_id] = {}
            log_msg(f"CAM{cam_id} inicio: {size_total}B en {n_frags} frags")
            _raw_buf = _raw_buf[needed:]

        elif tipo == PKT_FOTO_FRAG:
            needed = 4 + 1 + SIZEOF_FRAG_PAYLOAD
            if len(_raw_buf) < needed:
                return
            cam_id    = _raw_buf[5]
            frag_idx  = struct.unpack_from("<H", _raw_buf, 6)[0]
            frag_tot  = struct.unpack_from("<H", _raw_buf, 8)[0]
            datos     = bytes(_raw_buf[10:10 + FRAG_SIZE])
            if cam_id not in _img_fragments:
                _img_fragments[cam_id] = {}
            _img_fragments[cam_id][frag_idx] = datos
            _raw_buf = _raw_buf[needed:]
            intentar_reconstruir(cam_id)

        elif tipo == PKT_FOTO_FIN:
            needed = 4 + 1 + SIZEOF_FIN_PAYLOAD
            if len(_raw_buf) < needed:
                return
            cam_id = _raw_buf[5]
            log_msg(f"CAM{cam_id} fin recibido — forzando reconstrucción")
            _raw_buf = _raw_buf[needed:]
            # Forzar reconstrucción aunque falten fragmentos
            if cam_id in _img_totales and cam_id in _img_fragments:
                frags    = _img_fragments[cam_id]
                n_total  = _img_totales[cam_id]
                partes   = []
                for i in range(n_total):
                    partes.append(frags.get(i, bytes(FRAG_SIZE)))
                jpg_bytes = b"".join(partes)
                recibidos = len(frags)
                perdidos  = n_total - recibidos
                log_msg(f"CAM{cam_id} ensamblada: {recibidos}/{n_total} frags, {perdidos} perdidos")

                # Intentar mostrar o procesar disparidad
                otras = [c for c in _img_fragments if c != cam_id and c in _img_totales]
                if otras:
                    otra     = otras[0]
                    n_otra   = _img_totales[otra]
                    fr_otra  = _img_fragments[otra]
                    buf_otra = b"".join(fr_otra.get(i, bytes(FRAG_SIZE)) for i in range(n_otra))
                    log_msg("Generando mapa de disparidad...")
                    resultado = procesar_disparidad(
                        jpg_bytes if cam_id == 1 else buf_otra,
                        buf_otra  if cam_id == 1 else jpg_bytes
                    )
                    if resultado:
                        root.after(0, lambda r=resultado: mostrar_imagen(r))
                        log_msg(f"Disparidad lista: {len(resultado)} bytes")
                    _img_fragments.pop(1, None)
                    _img_fragments.pop(2, None)
                    _img_totales.pop(1, None)
                    _img_totales.pop(2, None)
                else:
                    root.after(0, lambda b=jpg_bytes: mostrar_imagen(b))

        else:
            # Tipo desconocido — descartar el magic y seguir buscando
            _raw_buf = _raw_buf[1:]

def loop():
    global ser

    if ser is None and PUERTO:
        try:
            ser = serial.Serial(PUERTO, 115200, timeout=0.1)
        except Exception:
            pass

    if ser and ser.in_waiting:
        try:
            chunk = ser.read(ser.in_waiting)
            procesar_chunk(chunk)
        except Exception as e:
            log_msg(f"Serial error: {e}")

    _after_ids["loop"] = root.after(30, loop)

# ════════════════════════════════════════════════
#  CIERRE LIMPIO
# ════════════════════════════════════════════════
def cerrar():
    for aid in _after_ids.values():
        try:
            root.after_cancel(aid)
        except Exception:
            pass
    if ser and ser.is_open:
        ser.close()
    root.destroy()

root.protocol("WM_DELETE_WINDOW", cerrar)

# ════════════════════════════════════════════════
#  ARRANQUE
# ════════════════════════════════════════════════
log_msg(f"MouseRat CanSat Ground Station — {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
log_msg(f"Puerto: {PUERTO or 'No detectado'}")
log_msg(f"Log: {csv_path}")

_after_ids["clock"] = root.after(0, tick_clock)
_after_ids["loop"]  = root.after(0, loop)
root.mainloop()
