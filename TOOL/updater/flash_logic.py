# -*- coding: utf-8 -*-
import re
import subprocess
import sys
import time


def list_ports():
    try:
        from serial.tools import list_ports as lp
    except Exception:
        return []
    ports = []
    for p in lp.comports():
        desc = p.description or ""
        ports.append({"device": p.device, "desc": desc})
    return ports


def run_esptool(port, firmware_path, baud, on_progress, on_log, stop_event):
    cmd = [
        sys.executable,
        "-m",
        "esptool",
        "--chip",
        "esp32s3",
        "--port",
        port,
        "--baud",
        str(baud),
        "--before",
        "default_reset",
        "--after",
        "hard_reset",
        "write_flash",
        "-z",
        "0x0",
        firmware_path,
    ]

    on_log(" ".join(cmd))

    try:
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
    except Exception as exc:
        on_log(f"Failed to start esptool: {exc}")
        return 1

    last_percent = -1
    while True:
        if stop_event.is_set():
            try:
                proc.terminate()
            except Exception:
                pass
            on_log("Canceled by user.")
            return 1

        line = proc.stdout.readline()
        if not line:
            if proc.poll() is not None:
                break
            time.sleep(0.05)
            continue

        line = line.rstrip()
        if line:
            on_log(line)

        m = re.search(r"\((\d+)\s*%\)", line)
        if not m:
            m = re.search(r"\b(\d{1,3})%\b", line)
        if m:
            pct = int(m.group(1))
            if pct != last_percent:
                last_percent = pct
                on_progress(pct)

    code = proc.wait()
    if code == 0 and last_percent < 100:
        on_progress(100)
    return code
