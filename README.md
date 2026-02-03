# AF1000X (Educational Drone) Fly Controller
AF1000X 교육용 마이크로 드론 펌웨어

AF1000X는 교육 및 연구 목적의 **마이크로 드론 펌웨어 프로젝트**입니다.  
PCB 하드웨어부터 펌웨어, 비행 제어 로직까지 **드론 시스템 전체를 이해**할 수 있도록 구성했습니다.

AF1000X is an **educational micro-drone firmware project**.  
It is designed to help learners understand the **entire drone system**, from PCB hardware to firmware and flight control logic.

한국 주식회사 슈베아에서 개발한 FC이며, 제품 구입 문의는 hello@1510.co.kr 로 연락 부탁드립니다.  
This FC is developed by SYUBEA Co., Ltd. (Korea). For purchase inquiries, please contact hello@1510.co.kr.

---

## 주요 기능 (KR)
1. IMU 기반 자세 안정화 (Roll / Pitch / Yaw)
2. ToF + 기압계(SPL06) 융합 고도 추정 및 호버링
3. 옵티컬 플로우(PMW3901) 기반 위치 보정
4. nRF24L01 2.4GHz 무선 통신 + 바인딩 + 홉 테이블
5. 배터리 전압 모니터링 및 경고/크리티컬 상태 처리
6. LED 상태 표시(부팅/바인딩/저전압/자이로/헤드리스/오토튠)
7. 시리얼 명령 지원 및 IMU 스트리밍 (`1510` 시작, `1511` 중지)

## Key Features (EN)
1. IMU-based attitude stabilization (Roll / Pitch / Yaw)
2. Fused altitude estimation using ToF + barometer (SPL06) with hovering
3. Optical flow (PMW3901) position correction
4. nRF24L01 2.4GHz wireless link + binding + hop table
5. Battery voltage monitoring with warning/critical handling
6. LED state indication (boot/bind/low-batt/gyro/headless/auto-tune)
7. Serial command support and IMU streaming (`1510` start, `1511` stop)

---

## 자동화된 기능 (KR)
1. 자동 이륙 / 자동 착륙
2. Hover 학습으로 `hoverThrottle` 자동 보정 및 저장
3. 옵티컬 플로우 자동 캘리브레이션(FlowK) 시퀀스
4. 센서 POST 및 자동 초기화(IMU/ToF/Baro/Flow)
5. 링크 끊김 시 페일세이프: 유지 후 착륙
6. 저전압 시 경고 및 비상 착륙 처리
7. Hover 안정 후 자동 튜닝(고도 PD + 요 P)

## Automated Features (EN)
1. Auto takeoff / auto landing
2. Hover learning with auto-adjust and persistence of `hoverThrottle`
3. Optical-flow auto calibration (FlowK) sequence
4. Sensor POST and auto initialization (IMU/ToF/Baro/Flow)
5. Failsafe on link loss: hold then land
6. Low-battery warning and emergency landing handling
7. Auto tuning after hover ready (altitude PD + yaw P)

---

## 특징 (KR)
1. 교육/연구용에 최적화된 모듈 구조 (`CORE`, `BINDING`, `HOVER`, `EasyCommander`, `AutoTune`, `GPIO`, `PID`)
2. PC 도구 제공
   - 펌웨어 업로더 (PYW + 자동 의존성 설치)
   - IMU 뷰어 (PYW + 자동 의존성 설치, 그래프/3D 자세 표시)
3. 시리얼 출력/로그 기반 디버깅 용이
4. UI 한글/영문 자동 전환 지원

## Highlights (EN)
1. Modular structure optimized for education/research (`CORE`, `BINDING`, `HOVER`, `EasyCommander`, `AutoTune`, `GPIO`, `PID`)
2. PC tools included
   - Firmware updater (PYW + auto dependency install)
   - IMU viewer (PYW + auto dependency install, graph/3D attitude)
3. Easy debugging via serial logs/output
4. Automatic UI language switch (Korean/English)

---

## 주요 칩셋 스펙 (KR)
1. MCU: ESP32-S3 (메인 컨트롤러)
2. IMU: ICM45686 (6축 가속/자이로, I2C)
3. 기압계: SPL06 (압력/고도, I2C)
4. ToF: VL53L1X (거리/고도, I2C, XSHUT 제어)
5. 옵티컬 플로우: PMW3901 (SPI)
6. RF: nRF24L01 (2.4GHz, SPI)
7. 배터리 ADC: 1S LiPo 전압 모니터링 (ADC)
8. 제품 사이즈: 35mm x 35mm

## Key Chipset Specs (EN)
1. MCU: ESP32-S3 (main controller)
2. IMU: ICM45686 (6-axis accel/gyro, I2C)
3. Barometer: SPL06 (pressure/altitude, I2C)
4. ToF: VL53L1X (range/altitude, I2C, XSHUT control)
5. Optical flow: PMW3901 (SPI)
6. RF: nRF24L01 (2.4GHz, SPI)
7. Battery ADC: 1S LiPo voltage monitor (ADC)
8. Board size: 35mm x 35mm

Note: For detailed electrical specs, refer to each chip's datasheet.

---

## 파일 설명 (KR)
- `AF1000X_Main.ino`: 스케치 엔트리, setup/loop 및 모듈 연결
- `AF1000X_CORE.h`: 센서 초기화, 자세/고도/포지션 추정, 제어 루프, 상태/LED, 시리얼 명령 등 핵심 로직
- `AF1000X_BINDING.h`: RF 바인딩/홉테이블/패킷 처리
- `AF1000X_Hover.h`: Hover 학습 및 hoverThrottle 보정/저장
- `AF1000X_EasyCommander.h`: Takeoff/Land 등 간단 커맨드 래퍼
- `AF1000X_AutoTune.h`: 자동 튜닝 루틴
- `AF1000X_GPIO.h`: 핀맵 정의 (I2C/SPI/RF/모터/LED/ADC)
- `AF1000X_PID.h`: PID/제어 파라미터 정의

## File Descriptions (EN)
- `AF1000X_Main.ino`: Sketch entry, setup/loop and module wiring
- `AF1000X_CORE.h`: Sensor init, attitude/altitude/position estimation, control loop, status/LED, serial commands
- `AF1000X_BINDING.h`: RF binding, hop table, packet handling
- `AF1000X_Hover.h`: Hover learning and hoverThrottle adjustment/persistence
- `AF1000X_EasyCommander.h`: Convenience commands (takeoff/land)
- `AF1000X_AutoTune.h`: Auto-tuning routines
- `AF1000X_GPIO.h`: Pin map (I2C/SPI/RF/motors/LED/ADC)
- `AF1000X_PID.h`: PID and control parameters

---

## 프로젝트 구조 / Project Structure
```
AF1000X/
├─ AF1000X_Main.ino
├─ AF1000X_CORE.h
├─ AF1000X_BINDING.h
├─ AF1000X_Hover.h
├─ AF1000X_EasyCommander.h
├─ AF1000X_AutoTune.h
├─ AF1000X_GPIO.h
├─ AF1000X_PID.h
├─ README.md
```

---

© SYUBEA
