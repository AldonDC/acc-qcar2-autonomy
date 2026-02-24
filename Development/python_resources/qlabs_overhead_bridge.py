#!/usr/bin/env python3
"""
QLabs Overhead HTTP Bridge
==========================
Captura la vista overhead de QLabs (birds-eye) y la sirve como JPEG en HTTP.
Corre donde tenga acceso al Quanser SDK (host o container quanser/virtual-qcar2).

Uso rápido (en el host o en el container con --network host):
    python3 qlabs_overhead_bridge.py

El dashboard lo consume automáticamente en http://localhost:8086/frame.jpg

Parámetros (como variables de entorno o argumentos):
    QLABS_HOST          IP de QLabs (default: localhost)
    BRIDGE_PORT         Puerto HTTP del bridge (default: 8086)
    CAM_X, CAM_Y, CAM_Z  Posición de la cámara (default: 0.15, 1.7, 5.0)
    CAM_RX, CAM_RY, CAM_RZ  Rotación en grados (default: 0, 90, 0)
    IMG_W, IMG_H        Resolución de captura (default: 1280, 720)
    FPS                 Frames por segundo (default: 20)
"""

import io
import os
import sys
import time
import struct
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

# ── Intentar importar qvl ───────────────────────────────────────
_here = os.path.dirname(os.path.abspath(__file__))
for _p in [_here, os.path.join(_here, "qvl"), "/home/qcar2_scripts/python",
           "/opt/quanser/python", "/usr/local/lib/python3/dist-packages"]:
    if os.path.isdir(os.path.join(_p, "qvl" if _p != _here else "qvl")) and _p not in sys.path:
        sys.path.insert(0, _p)

try:
    from qvl.qlabs import QuanserInteractiveLabs
    from qvl.free_camera import QLabsFreeCamera
except ImportError as e:
    print("[bridge] ERROR: no se puede importar qvl:", e)
    print("[bridge] Asegúrate de correr este script donde está instalado quanser-sdk.")
    sys.exit(1)

import cv2
import numpy as np

# ── Configuración ──────────────────────────────────────────────
QLABS_HOST = os.getenv("QLABS_HOST", "localhost")
BRIDGE_PORT = int(os.getenv("BRIDGE_PORT", "8086"))
CAM_LOC = [
    float(os.getenv("CAM_X", "0.15")),
    float(os.getenv("CAM_Y", "1.7")),
    float(os.getenv("CAM_Z", "5.0")),
]
CAM_ROT = [
    float(os.getenv("CAM_RX", "0")),
    float(os.getenv("CAM_RY", "90")),
    float(os.getenv("CAM_RZ", "0")),
]
IMG_W = int(os.getenv("IMG_W", "1280"))
IMG_H = int(os.getenv("IMG_H", "720"))
FPS   = int(os.getenv("FPS",   "20"))

# ── Estado compartido ──────────────────────────────────────────
_lock   = threading.Lock()
_latest_jpg: bytes = b""
_running = True


def _no_signal_jpg() -> bytes:
    img = np.zeros((IMG_H, IMG_W, 3), dtype=np.uint8)
    img[:] = (28, 33, 45)
    cv2.putText(img, "CONNECTING TO QLABS...", (IMG_W//2 - 200, IMG_H//2),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (160, 170, 190), 2, cv2.LINE_AA)
    _, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 75])
    return buf.tobytes()


# ── Capture loop (background thread) ─────────────────────────
def _capture_loop():
    global _latest_jpg, _running

    print("[bridge] Conectando a QLabs @ %s ..." % QLABS_HOST)
    conn = QuanserInteractiveLabs()
    if not conn.open(QLABS_HOST):
        print("[bridge] ERROR: no se pudo conectar a QLabs en %s" % QLABS_HOST)
        _running = False
        return

    cam = QLabsFreeCamera(conn)
    ok = cam.spawn_degrees(location=CAM_LOC, rotation=CAM_ROT)
    if not ok:
        print("[bridge] WARN: spawn_degrees devolvió False — la cámara puede que ya exista, continuando.")

    cam.set_image_capture_resolution(IMG_W, IMG_H)
    print("[bridge] Cámara overhead activa en %s → http://localhost:%d/frame.jpg" % (CAM_LOC, BRIDGE_PORT))

    dt = 1.0 / max(5, FPS)
    consecutive_errors = 0
    while _running:
        try:
            ok, img = cam.get_image()
            if ok and img is not None:
                # img ya es BGR (cv2.imdecode en QLabsFreeCamera.get_image)
                _, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 75])
                with _lock:
                    _latest_jpg = buf.tobytes()
                consecutive_errors = 0
        except Exception as e:
            consecutive_errors += 1
            if consecutive_errors % 30 == 1:
                print("[bridge] WARN: error de captura: %s" % e)
        time.sleep(dt)

    conn.close()


# ── HTTP handler ───────────────────────────────────────────────
class _Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path in ("/frame.jpg", "/frame", "/frame/overhead"):
            with _lock:
                data = _latest_jpg if _latest_jpg else _no_signal_jpg()
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-cache, no-store")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(data)
        elif self.path == "/status":
            with _lock:
                has_frame = bool(_latest_jpg)
            body = b'{"ok": true, "has_frame": ' + (b"true" if has_frame else b"false") + b"}"
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_error(404)

    def log_message(self, fmt, *args):
        pass  # silenciar logs por request


# ── Main ───────────────────────────────────────────────────────
def main():
    global _running

    # Llenar con placeholder hasta que lleguen frames reales
    with _lock:
        _latest_jpg = _no_signal_jpg()  # type: ignore[assignment]

    # Hilo de captura
    t = threading.Thread(target=_capture_loop, daemon=True)
    t.start()

    # Servidor HTTP
    server = HTTPServer(("0.0.0.0", BRIDGE_PORT), _Handler)
    print("[bridge] HTTP bridge listo en http://0.0.0.0:%d/frame.jpg" % BRIDGE_PORT)
    print("[bridge] Ctrl+C para salir.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        _running = False
        print("\n[bridge] Saliendo.")


if __name__ == "__main__":
    main()
