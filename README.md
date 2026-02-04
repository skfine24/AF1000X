# AF1000X FC for Smart Education Drone

## Purpose
This document describes the AF1000X Flight Controller (FC) as a smart education drone reference platform. It specifies supported features, startup behavior, indicators, and hardware guidelines in a product manual format.

## Key Features (Summary)
- Auto takeoff and auto landing
- Altitude hold hover and position hold hover
- External micro servo support
- External LED support and AUX power output (for FPV camera)
- Headless mode
- Safety cut off (emergency stop and failsafe)
- Control mode 1 and 2 supported (default: mode 2)
- DIY auto setup: hover learning + auto tuning

## Automatic Setup (2)
1. Hover learning: automatically adjusts `hoverThrottle` based on stable hover and saves it.
2. Auto tuning: runs altitude PD and yaw P tuning after hover ready and stores the results.

## Accessory I/O (Servo + User LED)
- Micro servo PWM output at 50 Hz (GPIO36), 900 to 2000 us range.
- User LED output on GPIO17. Default targets WS2812C (single wire NeoPixel class).
- WS2801C power supply pads are supported; control requires a dedicated SPI build.
- AUX power output is reserved for FPV camera power.

## Remote Accessory Control
- AR1000X can trigger accessory actions via AUX1 commands.
- LED step: cycles user LED color on each command.
- Servo toggle: switches between min and max angle positions.

## Power On Sequence
1. Power on and wait 1 second.
2. Serial banner prints once.
3. Sensor POST runs (IMU, BARO, ToF, Flow).
4. LED POST display: all LEDs ON, failed sensors blink at 0.5 second steps.
5. If any sensor failed, the status is held for 3 seconds.
6. If IMU or BARO fails, the system enters HARD LOCK (flight disabled) and keeps error LEDs on.
7. If ToF fails, altitude hold continues using barometer only.
8. If Flow fails, position correction is disabled.

POST message examples:
```
POST OK (IMU=1 BARO=1 TOF=1 FLOW=1)
POST FAIL -> FLIGHT LOCK (IMU=0 BARO=1 TOF=1 FLOW=1)
```

## LED Rules (Power On)
1. Sensor LEDs map to IMU, BARO, ToF, and Flow (LED1 to LED4).
2. Boot sequence has top priority over all other LED states.
3. If the board is detected inverted at boot, LEDs run an inverted chase (200 ms per step).
4. Otherwise, all LEDs blink together (1 s ON / 1 s OFF) during boot.
5. When binding completes, all LEDs stay ON for 1 s, then switch to normal logic.
6. During POST, all LEDs turn ON and only failed sensors blink at 0.5 s steps. If any failure occurs, the status is held for 3 s.

## LED Rules (Runtime)
1. Priority order: Boot Bind > Low Battery > Gyro Reset > Headless > Auto Tune > Normal.
2. Low battery: all LEDs blink together (1 s ON / 1 s OFF).
3. Gyro reset animation: all LEDs repeat 0.5 s ON / 1 s OFF for 3 cycles.
4. Headless mode: LED3 LED4 blink 2 s ON / 1 s OFF, LED1 LED2 stay OFF.
5. Auto tune: LED1 LED2 ON while LED3 LED4 OFF, then swap every 1.5 s.
6. Normal state: LED1 to LED3 show sensor OK. LED4 is ON only if Flow is OK and Flow is enabled by AUX2.

## Sensor Policy (POST)
- Required sensors: IMU + BARO
- Optional sensors: ToF + Flow
- IMU or BARO failure triggers HARD LOCK (flight disabled)
- ToF failure falls back to barometer only
- Flow failure disables position correction
- ToF runtime failover: 30 consecutive read failures disable ToF and fall back to barometer

## RF Link (nRF24L01 Standard)
- Recommended RF module: nRF24L01 2.4 GHz (standard)
- The controller scans nearby spectrum and selects low energy channels to build a hop table (FHSS).
- Binding creates a unique ID (TX address) and hop table, then stores them in NVS
- Binding mode: power on AF1000X upside down, then pair with AR1000X

### Identification Code
- Binding ID is a 5 byte TX address (nRF24L01 address format)
- Example format: `0xE7 0xE7 0xE7 0xE7 0xE7`
- Generated per pairing session and stored on both sides
- Used to lock the drone to the controller during normal operation

### Frequency Channel Table (Default)
Default hop table used before pairing overwrites it:
`63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74`

## Processor and Connectivity
- MCU: ESP32-S3FN8
- Wireless: Wi-Fi, Bluetooth
- RF link: nRF24L01 (2.4 GHz)

## Onboard Sensors
- IMU: ICM45686
- Barometer: SPL06
- ToF: VL53L1X
- Optical flow: PMW3901

## DIY Support Specs (Recommended)
- Motor: 720 to 8050 class
- Battery: 1S LiPo, 25C or higher
- All up weight: about 70 g

## Reference Hardware (Current Build)
These values describe the current reference build and may vary with your frame or parts.
- Weight (including battery): about 65 g
- Motor: 720 class, about 52,000 rpm at 1S
- Propeller: 58 mm
- Battery: 1S LiPo (4.15 V to 3.3 V), 600 mAh, 25C
- Motor to motor distance: 120.208 mm

## Flight Controller (AF1000X FC)
- Board version: 01.10
- Size: 35 mm x 35 mm
- Mounting hole: 2 mm diameter, 1.5 mm from board edge

See `IMG/README.md` for board images and mechanical details.

## Recommended Controller
- AR1000X Remote Controller

## Quick Start (Binding)
1. Power on AF1000X and flip it upside down. LEDs 1 to 4 will light sequentially.
2. Power on AR1000X while holding GPIO15 (bind button). GPIO25 LED will blink.
3. When pairing completes, LEDs remain on.

## Repository Layout
- `AF1000X_Main.ino` - main entry
- `AF1000X_AutoTune.h` - auto tuning logic
- `AF1000X_BINDING.h` - binding and FHSS helpers
- `AF1000X_CORE.h` - core flight control logic
- `AF1000X_EasyCommander.h` - high level command helpers
- `AF1000X_GPIO.h` - pin definitions
- `AF1000X_Hover.h` - hover and altitude control
- `AF1000X_PID.h` - PID defaults and tuning constants

## Safety / Warnings
- Always remove propellers before firmware updates or bench testing.
- Perform first flights in a safe, open area with clear line of sight.
- Do not operate the drone when battery voltage is below the recommended range.
- If POST indicates IMU/BARO failure, do not attempt to fly. Power down and inspect sensors.
- Keep hands and loose objects away from rotating parts.

---

# AF1000X FC (스마트 교육용 드론)

## 목적 (Purpose)
본 문서는 스마트 교육용 드론의 레퍼런스 플랫폼인 AF1000X 비행 컨트롤러(FC)에 대해 설명합니다. 제품 매뉴얼 형식으로 지원 기능, 시작 동작, 상태 표시등 및 하드웨어 가이드라인을 상세히 규정합니다.

## 주요 기능 (요약)
- 자동 이륙 및 자동 착륙 지원
- 고도 유지(Altitude Hold) 및 위치 유지(Position Hold) 호버링
- 외부 마이크로 서보 지원
- 외부 LED 지원 및 FPV 카메라용 AUX 전원 출력
- 헤드리스 모드 (Headless Mode)
- 안전 차단 기능 (비상 정지 및 페일세이프)
- 조종 모드 1, 2 지원 (기본값: 모드 2)
- DIY 자동 설정: 호버링 학습 + 오토 튜닝

## 자동 설정 (Automatic Setup)
1. **호버 학습 (Hover learning):** 안정적인 호버링 상태를 기반으로 `hoverThrottle` 값을 자동 조정하고 저장합니다.
2. **오토 튜닝 (Auto tuning):** 호버링 준비가 완료되면 고도 PD 및 요(Yaw) P 제어 값을 튜닝하고 결과를 저장합니다.

## 액세서리 I/O (서보 + 사용자 LED)
- **마이크로 서보:** PWM 출력 50Hz (GPIO36), 900~2000us 범위.
- **사용자 LED:** GPIO17 출력. 기본 타겟은 WS2812C (Single wire NeoPixel 클래스)입니다.
- **WS2801C 지원:** 전원 공급 패드가 지원되며, 제어를 위해서는 전용 SPI 빌드가 필요합니다.
- **AUX 전원 출력:** FPV 카메라 전원용으로 예약되어 있습니다.

## 리모트 액세서리 제어
- AR1000X 조종기의 **AUX1** 명령을 통해 액세서리 동작을 트리거할 수 있습니다.
- **LED 스텝:** 명령 시마다 사용자 LED 색상을 순차적으로 전환합니다.
- **서보 토글:** 서보의 최소/최대 각도 위치를 전환합니다.

## 전원 가동 시퀀스 (Power On Sequence)
1. 전원 On 후 1초 대기.
2. 시리얼 배너(Serial banner) 1회 출력.
3. **센서 POST 실행:** IMU, BARO, ToF, Flow 순서로 자가 진단 수행.
4. **LED POST 표시:** 모든 LED가 켜진 후, 실패한 센서만 0.5초 간격으로 점멸.
5. 센서 실패 시, 해당 상태를 3초간 유지.
6. **IMU 또는 BARO 실패 시:** 시스템 **HARD LOCK** (비행 불가) 진입 및 에러 LED 점등 유지.
7. **ToF 실패 시:** 기압계(Barometer)만을 사용하여 고도 유지를 계속합니다.
8. **Flow 실패 시:** 위치 보정 기능이 비활성화됩니다.

**POST 메시지 예시:**
```
POST OK (IMU=1 BARO=1 TOF=1 FLOW=1)
POST FAIL -> FLIGHT LOCK (IMU=0 BARO=1 TOF=1 FLOW=1)
```

## LED 규칙 (부팅 시)
1. 센서 LED 매핑: LED1(IMU), LED2(BARO), LED3(ToF), LED4(Flow).
2. 부트 시퀀스는 다른 모든 LED 상태보다 최우선 순위를 가집니다.
3. 부팅 시 보드가 뒤집힌 채로 감지되면, LED가 역방향으로 순차 점멸합니다 (200ms 간격).
4. 그 외의 경우, 부팅 중에는 모든 LED가 동시에 점멸합니다 (1초 ON / 1초 OFF).
5. 바인딩 완료 시 모든 LED가 1초간 점등된 후 정상 로직으로 전환됩니다.
6. POST 중에는 모든 LED가 켜지며 실패한 센서만 0.5초 간격으로 점멸합니다 (상태 3초 유지).

## LED 규칙 (런타임)
1. **우선순위:** 부트/바인딩 > 배터리 부족 > 자이로 리셋 > 헤드리스 > 오토 튜닝 > 정상 상태.
2. **배터리 부족:** 모든 LED 동시 점멸 (1초 ON / 1초 OFF).
3. **자이로 리셋 애니메이션:** 모든 LED가 0.5초 ON / 1초 OFF 패턴으로 3회 반복.
4. **헤드리스 모드:** LED3, 4번 2초 ON / 1초 OFF (LED1, 2번은 OFF).
5. **오토 튜닝:** LED1, 2번과 LED3, 4번이 1.5초 간격으로 교차 점멸.
6. **정상 상태:** LED1~3은 센서 정상 표시. LED4는 Flow 정상 및 AUX2에 의해 활성화된 경우에만 점등.

## 센서 정책 (POST)
- **필수 센서:** IMU + BARO
- **선택 센서:** ToF + Flow
- IMU 또는 BARO 실패 시 **HARD LOCK** (비행 차단).
- ToF 실패 시 기압계(Barometer) 모드로 전환.
- Flow 실패 시 위치 보정 비활성화.
- **ToF 런타임 페일오버:** 30회 연속 읽기 실패 시 ToF를 비활성화하고 기압계로 대체합니다.

## RF 링크 (nRF24L01 표준)
- 컨트롤러가 주변 스펙트럼을 스캔하여 저에너지 채널로 주파수 호핑 테이블(FHSS)을 구축합니다.
- 바인딩 시 고유 ID(TX 주소)와 호핑 테이블을 생성하여 NVS에 저장합니다.
- **바인딩 모드:** AF1000X를 뒤집은 상태로 전원을 켜고 AR1000X와 페어링합니다.

## 하드웨어 사양
- **MCU:** ESP32-S3FN8 (Wi-Fi, Bluetooth 지원)
- **온보드 센서:** ICM45686(IMU), SPL06(BARO), VL53L1X(ToF), PMW3901(Flow).
- **레퍼런스 기체:** 무게 약 65g, 720 클래스 모터, 58mm 프롭, 1S LiPo (600mAh).

## 빠른 시작 (바인딩)
1. AF1000X 전원을 켜고 **기체를 뒤집습니다**. LED 1~4가 순차적으로 점등됩니다.
2. AR1000X 조종기의 **GPIO15(바인드 버튼)**를 누른 채 전원을 켭니다.
3. 페어링 완료 시 LED가 점등된 상태를 유지합니다.

## 리포지토리 구조 (Repository Layout)
- `AF1000X_Main.ino`: 메인 엔트리
- `AF1000X_AutoTune.h`: 오토 튜닝 로직
- `AF1000X_BINDING.h`: 바인딩 및 FHSS 헬퍼
- `AF1000X_CORE.h`: 핵심 비행 제어 로직
- `AF1000X_EasyCommander.h`: 상위 레벨 명령 헬퍼
- `AF1000X_GPIO.h`: 핀 정의
- `AF1000X_Hover.h`: 호버링 및 고도 제어
- `AF1000X_PID.h`: PID 기본값 및 튜닝 상수

## 안전 및 주의사항 (Safety / Warnings)
- 펌웨어 업데이트나 벤치 테스트 전에는 **반드시 프로펠러를 제거**하십시오.
- 첫 비행은 시야가 확보된 안전하고 개방된 장소에서 수행하십시오.
- 배터리 전압이 권장 범위 미만일 때는 기체를 운용하지 마십시오.
- POST 결과가 IMU/BARO 실패를 나타내면 비행을 시도하지 말고 센서를 점검하십시오.
- 회전하는 부품에 손이나 물체가 닿지 않도록 주의하십시오.

---

# AF1000X FC (スマート教育用ドローン)

## 目的 (Purpose)
本ドキュメントは、スマート教育用ドローンのリファレンスプラットフォームである AF1000X フライトコントローラー(FC)について説明します。製品マニュアルの形式で、サポート機能、起動時の動作、インジケーター、およびハードウェアガイドラインを規定します。

## 主な機能 (概要)
- 自動離陸および自動着陸
- 高度維持 (Altitude Hold) および位置維持 (Position Hold) ホバリング
- 外部マイクロサーボ対応
- 外部LED対応およびFPVカメラ用AUX電源出力
- ヘッドレスモード (Headless Mode)
- 安全遮断機能 (緊急停止およびフェイルセーフ)
- 操作モード1および2対応 (デフォルト: モード2)
- DIY自動セットアップ: ホバリング学習 + オートチューニング

## 自動セットアップ (Automatic Setup)
1. **ホバリング学習:** 安定したホバリングに基づいて `hoverThrottle` 値を自動的に調整し、保存します。
2. **オートチューニング:** ホバリング準備完了後、高度PDおよびヨー(Yaw) P制御のチューニングを行い、結果を保存します。

## アクセサリ I/O (サーボ + ユーザーLED)
- **マイクロサーボ:** PWM出力 50Hz (GPIO36)、900〜2000us範囲。
- **ユーザーLED:** GPIO17出力。デフォルトはWS2812C (Single wire NeoPixelクラス) をターゲットとしています。
- **WS2801C対応:** 電源パッドがサポートされており、制御には専用のSPIビルドが必要です。
- **AUX電源出力:** FPVカメラの電源供給用に予約されています。

## リモートアクセサリ制御
- AR1000Xコントローラーの **AUX1** コマンドでアクセサリのアクションをトリガーできます。
- **LEDステップ:** コマンドごとにユーザーLEDの色を順次切り替えます。
- **サーボトグル:** サーボの最小/最大角度位置を切り替えます。

## 電源投入シーケンス (Power On Sequence)
1. 電源投入後、1秒間待機。
2. シリアルバナーを1回表示。
3. **センサーPOST (自己診断) 実行:** IMU、BARO、ToF、Flowの順。
4. **LED POST表示:** 全LED点灯後、失敗したセンサーのみ0.5秒間隔で点滅。
5. センサー失敗時、その状態を3秒間保持。
6. **IMUまたはBARO失敗時:** システムは **HARD LOCK** (飛行禁止) に入り、エラーLEDを維持。
7. **ToF失敗時:** 気圧計 (Barometer) のみを使用して高度維持を継続。
8. **Flow失敗時:** 位置補正機能を無効化。

**POSTメッセージ例:**
```
POST OK (IMU=1 BARO=1 TOF=1 FLOW=1)
POST FAIL -> FLIGHT LOCK (IMU=0 BARO=1 TOF=1 FLOW=1)
```

## LEDルール (起動時)
1. センサーLEDマップ: LED1(IMU), LED2(BARO), LED3(ToF), LED4(Flow)。
2. ブートシーケンスは他のすべてのLED状態より優先されます。
3. 起動時にボードが**反転 (逆さま)** 状態で検出された場合、LEDは反転チェイス点滅を行います (200ms間隔)。
4. それ以外の場合、起動中はすべてのLEDが同時に点滅します (1秒ON / 1秒OFF)。
5. バインド完了時、すべてのLEDが1秒間点灯し、その後通常のロジックに切り替わります。
6. POST中はすべてのLEDが点灯し、失敗したセンサーのみ0.5秒間隔で点滅します (3秒保持)。

## LEDルール (動作時)
1. **優先順位:** ブート/バインド > 低バッテリー > ジャイロリセット > ヘッドレス > オートチューニング > 通常。
2. **低バッテリー:** 全LED同時点滅 (1秒ON / 1秒OFF)。
3. **ジャイロリセット:** 全LEDが0.5秒ON / 1秒OFFのパターンを3回繰り返す。
4. **ヘッドレスモード:** LED3, 4が2秒ON / 1秒OFF (LED1, 2はOFF)。
5. **オートチューニング:** LED1, 2とLED3, 4が1.5秒ごとに交互に点滅。
6. **通常状態:** LED1〜3はセンサー正常を表示。LED4はFlow正常かつAUX2で有効化されている場合のみ点灯。

## センサーポリシー (POST)
- **必須センサー:** IMU + BARO
- **オプションセンサー:** ToF + Flow
- IMUまたはBAROの失敗は **HARD LOCK** (飛行不可) をトリガー。
- ToF失敗時は気圧計にフォールバック。
- Flow失敗時は位置補正を無効化。
- **ToFランタイムフェイルオーバー:** 30回連続で読み取り失敗した場合、ToFを無効化し気圧計に切り替えます。

## ハードウェア仕様
- **MCU:** ESP32-S3FN8 (Wi-Fi, Bluetooth対応)
- **搭載センサー:** ICM45686(IMU), SPL06(BARO), VL53L1X(ToF), PMW3901(Flow)。
- **リファレンス機体:** 重量約65g、720クラスモーター、58mmプロップ、1S LiPo (600mAh)。

## クイックスタート (バインド)
1. AF1000Xの電源を入れ、**機体を裏返します (逆さま)**。LED 1〜4が順次点灯します。
2. AR1000Xの **GPIO15 (バインドボタン)** を押しながら電源を入れます。
3. ペアリングが完了すると、LEDは点灯したままになります。

## リポジトリ構成 (Repository Layout)
- `AF1000X_Main.ino`: メインエントリ
- `AF1000X_AutoTune.h`: オートチューニングロジック
- `AF1000X_BINDING.h`: バインドおよびFHSSヘルパー
- `AF1000X_CORE.h`: コアフライト制御ロジック
- `AF1000X_EasyCommander.h`: 上位レベルコマンドヘルパー
- `AF1000X_GPIO.h`: ピン定義
- `AF1000X_Hover.h`: ホバリングおよび高度制御
- `AF1000X_PID.h`: PIDデフォルト値およびチューニング定数

## 安全上の注意 (Safety / Warnings)
- ファームウェアの更新やベンチテストの前には、**必ずプロペラを取り外してください**。
- 初飛行は、視界が確保された安全で広い場所で行ってください。
- バッテリー電圧が推奨範囲を下回っている場合は、機体を操作しないでください。
- POSTの結果がIMU/BAROの失敗を示している場合は、飛行を試みずセンサーを点検してください。
- 回転部には手や物を近づけないでください。

