# -*- coding: utf-8 -*-
import json
import os
import queue
import subprocess
import sys
import threading
import time
import locale
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from tkinter.scrolledtext import ScrolledText

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


_ensure_package("pyserial", "serial")
_ensure_package("esptool")

from flash_logic import list_ports, run_esptool

APP_TITLE = "AIRGO AF1000X Firmware Updater"
DEFAULT_BAUD = 460800
GRAPH_WINDOW_SEC = 120
CONFIG_FILE = os.path.join(os.path.dirname(__file__), "config.json")

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
        "title": "AIRGO AF1000X Firmware Updater",
        "copyright": "© SYUBEA",
        "error_title": "Error",
        "status_idle": "Idle",
        "status_uploading": "Uploading...",
        "status_done": "Upload complete",
        "status_failed": "Upload failed (exit code: {code})",
        "status_stopping": "Stopping...",
        "port": "Port",
        "refresh": "Refresh",
        "firmware": "Firmware",
        "browse": "Browse",
        "baud": "Baud",
        "upload": "Upload",
        "stop": "Stop",
        "graph": "Upload Graph",
        "log": "Log",
        "help": "Hold BOOT, press RESET, then release BOOT to enter download mode.",
        "select_firmware": "Select firmware",
        "select_port": "Select a port.",
        "invalid_firmware": "Select a valid firmware (.bin).",
        "invalid_baud": "Check the Baud value.",
        "graph_zero": "0%",
        "graph_full": "100%",
    },
    "ko": {
        "title": "AIRGO AF1000X 펌웨어 업로더",
        "copyright": "© SYUBEA",
        "error_title": "오류",
        "status_idle": "대기 중",
        "status_uploading": "업로드 중...",
        "status_done": "업로드 완료",
        "status_failed": "업로드 실패 (exit code: {code})",
        "status_stopping": "중지 요청 중...",
        "port": "포트",
        "refresh": "찾기",
        "firmware": "펌웨어",
        "browse": "찾아보기",
        "baud": "Baud",
        "upload": "업로드",
        "stop": "중지",
        "graph": "업로드 그래프",
        "log": "로그",
        "help": "BOOT 버튼을 누른 채 RESET을 누른 뒤 BOOT를 떼면 다운로드 모드로 진입합니다.",
        "select_firmware": "펌웨어 선택",
        "select_port": "포트를 선택하세요.",
        "invalid_firmware": "유효한 펌웨어(.bin)를 선택하세요.",
        "invalid_baud": "Baud 값을 확인하세요.",
        "graph_zero": "0%",
        "graph_full": "100%",
    },
}


class App:
    def __init__(self, root):
        self.root = root
        self.lang = "ko" if _is_korean_locale() else "en"
        self.t = STR[self.lang]
        self.root.title(self.t["title"])
        self.root.resizable(False, False)

        self.queue = queue.Queue()
        self.stop_event = threading.Event()
        self.upload_thread = None
        self.port_items = []
        self.progress_points = []
        self.start_time = None

        self.port_var = tk.StringVar()
        self.fw_var = tk.StringVar()
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUD))
        self.status_var = tk.StringVar(value=self.t["status_idle"])
        self.percent_var = tk.StringVar(value="0%")

        self._build_ui()
        self._load_config()
        self.refresh_ports()
        self._poll_queue()

    def _build_ui(self):
        main = ttk.Frame(self.root, padding=12)
        main.grid(row=0, column=0, sticky="nsew")

        title = ttk.Label(main, text=self.t["title"], font=("Segoe UI", 14, "bold"))
        title.grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 8))

        ttk.Label(main, text=self.t["port"]).grid(row=1, column=0, sticky="w")
        self.port_combo = ttk.Combobox(main, textvariable=self.port_var, width=36, state="readonly")
        self.port_combo.grid(row=1, column=1, sticky="we", padx=(6, 6))
        self.refresh_btn = ttk.Button(main, text=self.t["refresh"], command=self.refresh_ports)
        self.refresh_btn.grid(row=1, column=2, sticky="e")

        ttk.Label(main, text=self.t["firmware"]).grid(row=2, column=0, sticky="w", pady=(6, 0))
        self.fw_entry = ttk.Entry(main, textvariable=self.fw_var, width=36)
        self.fw_entry.grid(row=2, column=1, sticky="we", padx=(6, 6), pady=(6, 0))
        self.browse_btn = ttk.Button(main, text=self.t["browse"], command=self.browse_firmware)
        self.browse_btn.grid(row=2, column=2, sticky="e", pady=(6, 0))

        ttk.Label(main, text=self.t["baud"]).grid(row=3, column=0, sticky="w", pady=(6, 0))
        self.baud_combo = ttk.Combobox(
            main,
            textvariable=self.baud_var,
            values=["115200", "230400", "460800", "921600"],
            width=12,
            state="readonly",
        )
        self.baud_combo.grid(row=3, column=1, sticky="w", padx=(6, 6), pady=(6, 0))

        btn_frame = ttk.Frame(main)
        btn_frame.grid(row=4, column=0, columnspan=3, sticky="we", pady=(10, 6))
        self.upload_btn = ttk.Button(btn_frame, text=self.t["upload"], command=self.start_upload)
        self.upload_btn.pack(side="left")
        self.stop_btn = ttk.Button(btn_frame, text=self.t["stop"], command=self.stop_upload, state="disabled")
        self.stop_btn.pack(side="left", padx=(6, 0))

        progress_frame = ttk.Frame(main)
        progress_frame.grid(row=5, column=0, columnspan=3, sticky="we")
        self.progress = ttk.Progressbar(progress_frame, maximum=100, length=360)
        self.progress.pack(side="left", fill="x", expand=True)
        self.percent_label = ttk.Label(progress_frame, textvariable=self.percent_var, width=5)
        self.percent_label.pack(side="left", padx=(6, 0))

        ttk.Label(main, text=self.t["graph"]).grid(row=6, column=0, columnspan=3, sticky="w", pady=(8, 0))
        self.graph = tk.Canvas(main, width=420, height=140, bg="#0f1115", highlightthickness=1, highlightbackground="#333")
        self.graph.grid(row=7, column=0, columnspan=3, sticky="we", pady=(4, 8))

        ttk.Label(main, text=self.t["log"]).grid(row=8, column=0, columnspan=3, sticky="w")
        self.log_text = ScrolledText(main, width=58, height=9, state="disabled")
        self.log_text.grid(row=9, column=0, columnspan=3, sticky="we", pady=(4, 8))

        ttk.Label(main, text=self.t["help"], foreground="#444").grid(row=10, column=0, columnspan=3, sticky="w")

        self.status_label = ttk.Label(main, textvariable=self.status_var, foreground="#005a9e")
        self.status_label.grid(row=11, column=0, columnspan=3, sticky="w", pady=(6, 0))

        ttk.Label(main, text=self.t["copyright"], foreground="#666").grid(
            row=12, column=0, columnspan=3, sticky="e", pady=(2, 0)
        )

        main.columnconfigure(1, weight=1)

    def _log(self, message):
        self.log_text.configure(state="normal")
        self.log_text.insert("end", message + "\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _set_status(self, message):
        self.status_var.set(message)

    def _set_progress(self, percent):
        self.progress["value"] = percent
        self.percent_var.set(f"{percent}%")
        self._record_progress(percent)

    def _record_progress(self, percent):
        now = time.time()
        if self.start_time is None:
            self.start_time = now
        self.progress_points.append((now, percent))
        if len(self.progress_points) > 400:
            self.progress_points = self.progress_points[-400:]
        self._draw_graph()

    def _draw_graph(self):
        w = int(self.graph["width"])
        h = int(self.graph["height"])
        self.graph.delete("all")
        self.graph.create_rectangle(1, 1, w - 1, h - 1, outline="#333")
        self.graph.create_text(6, h - 10, text=self.t["graph_zero"], anchor="w", fill="#7f8c8d", font=("Segoe UI", 8))
        self.graph.create_text(6, 8, text=self.t["graph_full"], anchor="w", fill="#7f8c8d", font=("Segoe UI", 8))

        if not self.progress_points:
            return

        t_last = self.progress_points[-1][0]
        t0 = t_last - GRAPH_WINDOW_SEC
        if t_last - t0 < 10:
            t0 = t_last - 10
        if t_last == t0:
            t0 = t_last - 1

        points = []
        for t, p in self.progress_points:
            if t < t0:
                continue
            x = 10 + (t - t0) / (t_last - t0) * (w - 20)
            y = h - 10 - (p / 100.0) * (h - 20)
            points.append((x, y))

        if len(points) == 1:
            x, y = points[0]
            self.graph.create_oval(x - 2, y - 2, x + 2, y + 2, fill="#00c2ff", outline="")
            return

        for i in range(1, len(points)):
            x1, y1 = points[i - 1]
            x2, y2 = points[i]
            self.graph.create_line(x1, y1, x2, y2, fill="#00c2ff", width=2)

    def _load_config(self):
        if not os.path.isfile(CONFIG_FILE):
            return
        try:
            with open(CONFIG_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception:
            return
        self.last_port = data.get("last_port")
        self.fw_var.set(data.get("last_firmware", ""))
        if data.get("last_baud"):
            self.baud_var.set(str(data.get("last_baud")))

    def _save_config(self, port, firmware, baud):
        data = {
            "last_port": port,
            "last_firmware": firmware,
            "last_baud": baud,
        }
        try:
            with open(CONFIG_FILE, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
        except Exception:
            pass

    def refresh_ports(self):
        ports = list_ports()
        self.port_items = []
        values = []
        for p in ports:
            display = f"{p['device']} - {p['desc']}" if p.get("desc") else p["device"]
            self.port_items.append((display, p["device"]))
            values.append(display)
        self.port_combo["values"] = values

        selected = None
        if hasattr(self, "last_port") and self.last_port:
            for display, dev in self.port_items:
                if dev == self.last_port:
                    selected = display
                    break
        if selected is None and len(values) == 1:
            selected = values[0]
        if selected:
            self.port_var.set(selected)

    def _selected_port(self):
        display = self.port_var.get().strip()
        if not display:
            return ""
        for d, dev in self.port_items:
            if d == display:
                return dev
        return display

    def browse_firmware(self):
        file_path = filedialog.askopenfilename(
            title=self.t["select_firmware"],
            filetypes=[("Firmware", "*.bin"), ("All Files", "*.*")],
        )
        if file_path:
            self.fw_var.set(file_path)

    def start_upload(self):
        port = self._selected_port()
        firmware = self.fw_var.get().strip()
        baud = self.baud_var.get().strip()

        if not port:
            messagebox.showerror(self.t["error_title"], self.t["select_port"])
            return
        if not firmware or not os.path.isfile(firmware):
            messagebox.showerror(self.t["error_title"], self.t["invalid_firmware"])
            return
        try:
            baud_int = int(baud)
        except ValueError:
            messagebox.showerror(self.t["error_title"], self.t["invalid_baud"])
            return

        self._save_config(port, firmware, baud_int)
        self.progress_points = []
        self.start_time = time.time()
        self._set_progress(0)
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")
        self._set_status(self.t["status_uploading"])

        self.stop_event.clear()
        self._set_busy(True)

        def worker():
            def on_log(msg):
                self.queue.put(("log", msg))

            def on_progress(pct):
                self.queue.put(("progress", pct))

            exit_code = run_esptool(port, firmware, baud_int, on_progress, on_log, self.stop_event)
            self.queue.put(("done", exit_code))

        self.upload_thread = threading.Thread(target=worker, daemon=True)
        self.upload_thread.start()

    def stop_upload(self):
        if self.upload_thread and self.upload_thread.is_alive():
            self.stop_event.set()
            self._set_status(self.t["status_stopping"])

    def _set_busy(self, is_busy):
        state = "disabled" if is_busy else "readonly"
        self.port_combo.configure(state=state)
        self.refresh_btn.configure(state="disabled" if is_busy else "normal")
        self.browse_btn.configure(state="disabled" if is_busy else "normal")
        self.baud_combo.configure(state=state)
        self.upload_btn.configure(state="disabled" if is_busy else "normal")
        self.stop_btn.configure(state="normal" if is_busy else "disabled")

    def _poll_queue(self):
        try:
            while True:
                kind, payload = self.queue.get_nowait()
                if kind == "log":
                    self._log(payload)
                elif kind == "progress":
                    self._set_progress(payload)
                elif kind == "done":
                    self._on_done(payload)
        except queue.Empty:
            pass
        self.root.after(100, self._poll_queue)

    def _on_done(self, exit_code):
        if exit_code == 0:
            self._set_status(self.t["status_done"])
        else:
            self._set_status(self.t["status_failed"].format(code=exit_code))
        self._set_busy(False)


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
