# AF1000X Serial Command Reference (Drone)

KO: SYUBEA AF1000X 드론(FC) 시리얼 명령
EN: SYUBEA AF1000X drone (FC) serial commands

KO: Baudrate = `115200`, Line ending = `\r` or `\n`
EN: Baudrate = `115200`, Line ending = `\r` or `\n`

---

## KO: Flow 비트 안내 | EN: Flow Bit Notes

KO: Flow ON/OFF는 드론 시리얼이 아니라 **조종기 AUX2 비트**로 제어됨  
EN: Flow ON/OFF is controlled via **controller AUX2 bit**, not drone serial

KO: AUX2 비트  
EN: AUX2 bits  
- `bit0` = Headless  
- `bit1` = Optical Flow Enable

KO: 실제 코드 위치  
EN: Code locations  
- `C:\Users\sklee\Documents\Arduino\AR1000X_Main\AR1000X_Main.ino` (AUX2 비트 생성/전송)
- `C:\Users\sklee\Documents\Arduino\AF1000X_Main\AF1000X_CORE.h` (AUX2 비트 수신/해석)

---

## KO: 1) 기본 명령 | EN: Basic Commands

| Command | KO | EN |
|---|---|---|
| `T` | Hover 학습 기반 이륙 | Takeoff with hover learn |
| `L` | 착륙 | Land |
| `K` | 긴급 모터 컷 | Emergency motor cut |
| `Y0` | Yaw 리셋 | Yaw reset |

---

## KO: 2) 플로우 캘리브레이션 | EN: Flow Calibration

| Command | KO | EN |
|---|---|---|
| `C` | 플로우 자동 캘리브레이션 시작 | Start flow auto calibration |
| `D<cm>` | 실제 이동거리 입력(보정) | Refine with actual distance |

KO: 예시 `D60.0` (cm 단위)
EN: Example `D60.0` (cm)

---

## KO: 3) 상태/튜닝 출력 | EN: Status/Tuning

| Command | KO | EN |
|---|---|---|
| `B` | 배터리 전압 출력 | Print battery voltage |
| `PID?` | 튜닝 값 덤프 | Dump tuning values |
| `TUNE?` | `PID?`와 동일 | Same as `PID?` |
| `PIDDEF` | PID 기본값 스니펫 출력 | Print PID.h defaults snippet |

---

## KO: 설정 백업 | EN: Settings Backup

KO: 별도의 “NVS 전체 백업” 명령은 없음  
EN: There is no dedicated “full NVS backup” command

KO: 대신 `PID?`, `PIDDEF` 출력으로 주요 튜닝 값을 백업 가능  
EN: Use `PID?` / `PIDDEF` outputs to back up key tuning values

---

## KO: 출력 예시 | EN: Output Example

```
Battery: 3.85V (filt 3.82V)
ALT_KP=0.1800 ALT_KD=0.0600 YAW_KP=1.5000
FLOW_K=0.002200 HOVER_PWM=120
```

---

## KO: 4) 예제 (터미널/파이썬) | EN: Examples (Terminal/Python)

### KO: Python (pyserial)
### EN: Python (pyserial)

```python
import serial, time

ser = serial.Serial("COM6", 115200, timeout=0.2)
time.sleep(0.5)

def send(cmd):
    ser.write((cmd + "\n").encode())
    time.sleep(0.05)
    print(ser.read_all().decode(errors="ignore"))

send("B")
send("PID?")
send("PIDDEF")
send("Y0")
```

### KO: Windows PowerShell (pyserial miniterm)
### EN: Windows PowerShell (pyserial miniterm)

```powershell
python -m serial.tools.miniterm COM6 115200
```

KO: 연결 후 직접 명령어 입력
EN: After connecting, type commands directly
