# -*- coding: utf-8 -*-
import math
import queue
import subprocess
import sys
import threading
import time
from collections import deque
import locale

import tkinter as tk
from tkinter import ttk, messagebox

def _ensure_package(pkg_name, import_name=None):
    name = import_name or pkg_name
    try:
        __import__(name)
        return True
    except Exception:
        try:
            subprocess.check_call([sys.executable, "-m", "pip", "install", pkg_name])
        except Exception:
            return False
    try:
        __import__(name)
        return True
    except Exception:
        return False


if _ensure_package("pyserial", "serial"):
    import serial
    from serial.tools import list_ports
else:
    serial = None
    list_ports = None

if _ensure_package("matplotlib"):
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
else:
    Figure = None
    FigureCanvasTkAgg = None

APP_TITLE = "AIRGO AF1000X IMU Viewer"
BAUD_DEFAULT = 115200
STREAM_START_CMD = "1510\n"
STREAM_STOP_CMD = "1511\n"
MAX_POINTS = 400  # about 20s at 20 Hz

def _is_korean_locale():
    loc = locale.getdefaultlocale()
    if loc and loc[0]:
        if loc[0].lower().startswith("ko"):
            return True
    loc2 = locale.getlocale()
    if loc2 and loc2[0]:
        if loc2[0].lower().startswith("ko"):
            return True
    return False


STR = {
    "en": {
        "title": "AIRGO AF1000X IMU Viewer",
        "copyright": "© SYUBEA",
        "error_title": "Error",
        "port": "Port",
        "baud": "Baud",
        "refresh": "Refresh",
        "connect": "Connect",
        "disconnect": "Disconnect",
        "disconnected": "Disconnected",
        "connected": "Connected: {port}",
        "select_port": "Select a port",
        "invalid_baud": "Invalid baud",
        "open_failed": "Open failed: {err}",
        "need_pyserial": "pyserial not installed",
        "need_mpl": "matplotlib not installed",
        "chart_title": "Roll / Pitch / Yaw",
        "chart_xlabel": "time (s)",
        "chart_ylabel": "deg",
        "att_title": "Attitude (3D)",
        "line_roll": "Roll",
        "line_pitch": "Pitch",
        "line_yaw": "Yaw",
        "axis_x": "X",
        "axis_y": "Y",
        "axis_z": "Z",
        "last_fmt": "R:{r:.2f} P:{p:.2f} Y:{y:.2f} VBAT:{v}",
        "att_fmt": "R:{r:.1f} P:{p:.1f} Y:{y:.1f}  VBAT:{v}",
    },
    "ko": {
        "title": "AIRGO AF1000X IMU 뷰어",
        "copyright": "© SYUBEA",
        "error_title": "오류",
        "port": "포트",
        "baud": "보드레이트",
        "refresh": "새로고침",
        "connect": "연결",
        "disconnect": "연결 해제",
        "disconnected": "연결 안됨",
        "connected": "연결됨: {port}",
        "select_port": "포트를 선택하세요",
        "invalid_baud": "보드레이트가 올바르지 않습니다",
        "open_failed": "열기 실패: {err}",
        "need_pyserial": "pyserial이 설치되어 있지 않습니다",
        "need_mpl": "matplotlib이 설치되어 있지 않습니다",
        "chart_title": "롤 / 피치 / 요",
        "chart_xlabel": "시간 (s)",
        "chart_ylabel": "도",
        "att_title": "자세 (3D)",
        "line_roll": "롤",
        "line_pitch": "피치",
        "line_yaw": "요",
        "axis_x": "X",
        "axis_y": "Y",
        "axis_z": "Z",
        "last_fmt": "R:{r:.2f} P:{p:.2f} Y:{y:.2f} VBAT:{v}",
        "att_fmt": "R:{r:.1f} P:{p:.1f} Y:{y:.1f}  VBAT:{v}",
    },
}

def _set_mpl_font(lang):
    if Figure is None:
        return
    try:
        import matplotlib
        from matplotlib import font_manager
    except Exception:
        return
    if lang == "ko":
        preferred = [
            "Malgun Gothic",
            "맑은 고딕",
            "NanumGothic",
            "Noto Sans CJK KR",
            "AppleGothic",
        ]
    else:
        preferred = ["Segoe UI", "Arial", "DejaVu Sans"]

    available = {f.name for f in font_manager.fontManager.ttflist}
    for name in preferred:
        if name in available:
            matplotlib.rcParams["font.family"] = name
            break
    matplotlib.rcParams["axes.unicode_minus"] = False


class IMUViewer:
    def __init__(self, root):
        self.root = root
        self.lang = "ko" if _is_korean_locale() else "en"
        self.t = STR[self.lang]
        _set_mpl_font(self.lang)
        self.root.title(self.t["title"])
        self.root.resizable(False, False)

        self.ser = None
        self.reader_thread = None
        self.stop_event = threading.Event()
        self.queue = queue.Queue()

        self.t0 = None
        self.ts = deque(maxlen=MAX_POINTS)
        self.roll = deque(maxlen=MAX_POINTS)
        self.pitch = deque(maxlen=MAX_POINTS)
        self.yaw = deque(maxlen=MAX_POINTS)

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value=str(BAUD_DEFAULT))
        self.status_var = tk.StringVar(value=self.t["disconnected"])
        self.last_var = tk.StringVar(value=self.t["last_fmt"].format(r=0.0, p=0.0, y=0.0, v="--.-"))

        self._build_ui()
        self.refresh_ports()
        self._poll_queue()

    def _build_ui(self):
        main = ttk.Frame(self.root, padding=12)
        main.grid(row=0, column=0, sticky="nsew")

        title = ttk.Label(main, text=self.t["title"], font=("Segoe UI", 14, "bold"))
        title.grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 8))

        ttk.Label(main, text=self.t["port"]).grid(row=1, column=0, sticky="w")
        self.port_combo = ttk.Combobox(main, textvariable=self.port_var, width=28, state="readonly")
        self.port_combo.grid(row=1, column=1, sticky="w", padx=(6, 6))
        btn_frame = ttk.Frame(main)
        btn_frame.grid(row=1, column=2, sticky="w")
        self.refresh_btn = ttk.Button(btn_frame, text=self.t["refresh"], command=self.refresh_ports)
        self.refresh_btn.pack(side="left")
        self.connect_btn = ttk.Button(btn_frame, text=self.t["connect"], command=self.connect)
        self.connect_btn.pack(side="left", padx=(6, 0))
        self.disconnect_btn = ttk.Button(btn_frame, text=self.t["disconnect"], command=self.disconnect, state="disabled")
        self.disconnect_btn.pack(side="left", padx=(6, 0))

        main.grid_columnconfigure(3, weight=1)
        ttk.Label(main, text=self.t["baud"]).grid(row=1, column=4, sticky="e")
        self.baud_combo = ttk.Combobox(main, textvariable=self.baud_var,
                                       values=["115200", "230400", "460800", "921600"],
                                       width=10, state="readonly")
        self.baud_combo.grid(row=1, column=5, sticky="w", padx=(6, 0))

        if Figure is None or FigureCanvasTkAgg is None:
            ttk.Label(main, text=self.t["need_mpl"], foreground="#a00").grid(row=3, column=0, columnspan=6, sticky="w")
            ttk.Label(main, text=self.t["copyright"], foreground="#666").grid(
                row=4, column=0, columnspan=6, sticky="e", pady=(2, 0)
            )
            main.columnconfigure(1, weight=1)
            return

        fig = Figure(figsize=(8.6, 3.2), dpi=100)
        self.ax = fig.add_subplot(121)
        self.ax.set_title(self.t["chart_title"])
        self.ax.set_xlabel(self.t["chart_xlabel"])
        self.ax.set_ylabel(self.t["chart_ylabel"])
        self.ax.grid(True, alpha=0.3)

        (self.line_roll,) = self.ax.plot([], [], label=self.t["line_roll"])
        (self.line_pitch,) = self.ax.plot([], [], label=self.t["line_pitch"])
        (self.line_yaw,) = self.ax.plot([], [], label=self.t["line_yaw"])
        self.ax.legend(loc="upper right")

        self.ax3d = fig.add_subplot(122, projection="3d")
        self.ax3d.set_title(self.t["att_title"])
        self.ax3d.set_xlim(-1.2, 1.2)
        self.ax3d.set_ylim(-1.2, 1.2)
        self.ax3d.set_zlim(-1.2, 1.2)
        self.ax3d.set_xlabel(self.t["axis_x"])
        self.ax3d.set_ylabel(self.t["axis_y"])
        self.ax3d.set_zlabel(self.t["axis_z"])
        self.ax3d.view_init(elev=20, azim=35)
        self.ax3d.grid(True, alpha=0.3)

        (self.axis_x,) = self.ax3d.plot([0, 1], [0, 0], [0, 0], color="#ff4d4d", label=self.t["axis_x"])
        (self.axis_y,) = self.ax3d.plot([0, 0], [0, 1], [0, 0], color="#4dff4d", label=self.t["axis_y"])
        (self.axis_z,) = self.ax3d.plot([0, 0], [0, 0], [0, 1], color="#4d7bff", label=self.t["axis_z"])
        self.att_text = self.ax3d.text2D(0.02, 0.95, "", transform=self.ax3d.transAxes)

        # cube model
        s = 0.5
        self.cube_vertices = [
            (-s, -s, -s), (s, -s, -s), (s, s, -s), (-s, s, -s),
            (-s, -s, s),  (s, -s, s),  (s, s, s),  (-s, s, s),
        ]
        self.cube_edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        ]
        self.cube_lines = []
        for _ in self.cube_edges:
            line, = self.ax3d.plot([0, 0], [0, 0], [0, 0], color="#999999", alpha=0.8)
            self.cube_lines.append(line)

        self.canvas = FigureCanvasTkAgg(fig, master=main)
        self.canvas.get_tk_widget().grid(row=3, column=0, columnspan=6, pady=(4, 8))

        status_frame = ttk.Frame(main)
        status_frame.grid(row=4, column=0, columnspan=6, sticky="we")
        ttk.Label(status_frame, textvariable=self.status_var, foreground="#005a9e").pack(side="left")
        ttk.Label(status_frame, textvariable=self.last_var).pack(side="right")

        ttk.Label(main, text=self.t["copyright"], foreground="#666").grid(
            row=5, column=0, columnspan=6, sticky="e", pady=(2, 0)
        )

        main.columnconfigure(1, weight=1)

    def refresh_ports(self):
        if list_ports is None:
            return
        ports = list(list_ports.comports())
        values = []
        self.port_map = {}
        for p in ports:
            label = f"{p.device} - {p.description}" if p.description else p.device
            values.append(label)
            self.port_map[label] = p.device
        self.port_combo["values"] = values
        if values and not self.port_var.get():
            self.port_var.set(values[0])

    def _selected_port(self):
        label = self.port_var.get().strip()
        if not label:
            return ""
        return self.port_map.get(label, label)

    def connect(self):
        if serial is None:
            messagebox.showerror(self.t["error_title"], self.t["need_pyserial"])
            return
        port = self._selected_port()
        if not port:
            messagebox.showerror(self.t["error_title"], self.t["select_port"])
            return
        try:
            baud = int(self.baud_var.get().strip())
        except ValueError:
            messagebox.showerror(self.t["error_title"], self.t["invalid_baud"])
            return
        try:
            self.ser = serial.Serial(port, baud, timeout=0.2)
        except Exception as exc:
            messagebox.showerror(self.t["error_title"], self.t["open_failed"].format(err=exc))
            return

        self.stop_event.clear()
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()

        self._send(STREAM_START_CMD)
        self.t0 = time.time()
        self.ts.clear(); self.roll.clear(); self.pitch.clear(); self.yaw.clear()

        self.status_var.set(self.t["connected"].format(port=port))
        self.connect_btn.configure(state="disabled")
        self.disconnect_btn.configure(state="normal")

    def disconnect(self):
        if self.ser:
            self._send(STREAM_STOP_CMD)
        self.stop_event.set()
        if self.reader_thread:
            self.reader_thread.join(timeout=0.5)
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.status_var.set(self.t["disconnected"])
        self.connect_btn.configure(state="normal")
        self.disconnect_btn.configure(state="disabled")

    def _send(self, text):
        try:
            if self.ser:
                self.ser.write(text.encode())
        except Exception:
            pass

    def _reader_loop(self):
        while not self.stop_event.is_set():
            try:
                line = self.ser.readline()
            except Exception:
                break
            if not line:
                continue
            try:
                s = line.decode(errors="ignore").strip()
            except Exception:
                continue
            if not s.startswith("IMU "):
                continue
            parts = s.split()
            if len(parts) not in (4, 5):
                continue
            try:
                r = float(parts[1])
                p = float(parts[2])
                y = float(parts[3])
                v = float(parts[4]) if len(parts) == 5 else None
            except ValueError:
                continue
            self.queue.put((time.time(), r, p, y, v))

    def _poll_queue(self):
        updated = False
        while True:
            try:
                t, r, p, y, v = self.queue.get_nowait()
            except queue.Empty:
                break
            if self.t0 is None:
                self.t0 = t
            self.ts.append(t - self.t0)
            self.roll.append(r)
            self.pitch.append(p)
            self.yaw.append(y)
            if v is None:
                self.last_var.set(self.t["last_fmt"].format(r=r, p=p, y=y, v="--.-"))
                self._last_vbat = None
            else:
                self.last_var.set(self.t["last_fmt"].format(r=r, p=p, y=y, v=f"{v:.2f}"))
                self._last_vbat = v
            updated = True

        if updated and Figure is not None:
            self._redraw()
        self.root.after(50, self._poll_queue)

    def _redraw(self):
        if not self.ts:
            return
        xs = list(self.ts)
        self.line_roll.set_data(xs, list(self.roll))
        self.line_pitch.set_data(xs, list(self.pitch))
        self.line_yaw.set_data(xs, list(self.yaw))
        self.ax.relim()
        self.ax.autoscale_view()
        self._update_3d()
        self.canvas.draw_idle()

    def _update_3d(self):
        if not self.roll:
            return
        roll = math.radians(self.roll[-1])
        pitch = math.radians(self.pitch[-1])
        yaw = math.radians(self.yaw[-1])

        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)

        # R = Rz(yaw) * Ry(pitch) * Rx(roll)
        r00 = cy * cp
        r01 = cy * sp * sr - sy * cr
        r02 = cy * sp * cr + sy * sr
        r10 = sy * cp
        r11 = sy * sp * sr + cy * cr
        r12 = sy * sp * cr - cy * sr
        r20 = -sp
        r21 = cp * sr
        r22 = cp * cr

        # basis vectors
        x = (r00, r10, r20)
        y = (r01, r11, r21)
        z = (r02, r12, r22)

        self.axis_x.set_data([0, x[0]], [0, x[1]])
        self.axis_x.set_3d_properties([0, x[2]])
        self.axis_y.set_data([0, y[0]], [0, y[1]])
        self.axis_y.set_3d_properties([0, y[2]])
        self.axis_z.set_data([0, z[0]], [0, z[1]])
        self.axis_z.set_3d_properties([0, z[2]])

        # update cube model
        verts = []
        for vx, vy, vz in self.cube_vertices:
            rx = r00 * vx + r01 * vy + r02 * vz
            ry = r10 * vx + r11 * vy + r12 * vz
            rz = r20 * vx + r21 * vy + r22 * vz
            verts.append((rx, ry, rz))
        for line, (a, b) in zip(self.cube_lines, self.cube_edges):
            x0, y0, z0 = verts[a]
            x1, y1, z1 = verts[b]
            line.set_data([x0, x1], [y0, y1])
            line.set_3d_properties([z0, z1])

        vbat = getattr(self, "_last_vbat", None)
        if vbat is None:
            self.att_text.set_text(self.t["att_fmt"].format(
                r=math.degrees(roll), p=math.degrees(pitch), y=math.degrees(yaw), v="--.-V"
            ))
        else:
            self.att_text.set_text(self.t["att_fmt"].format(
                r=math.degrees(roll), p=math.degrees(pitch), y=math.degrees(yaw), v=f"{vbat:.2f}V"
            ))


def main():
    root = tk.Tk()
    app = IMUViewer(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.disconnect(), root.destroy()))
    root.mainloop()


if __name__ == "__main__":
    main()
