#ifndef AF1000X_H
#define AF1000X_H


#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <Preferences.h>
#include <VL53L1X.h>
#include "ICM45686.h"

// KO: GPIO / 핀 맵
// EN: GPIO / pin map
#include "AF1000X_GPIO.h"

// KO: 비행 튜닝 / PID 상수
// EN: Flight tuning / PID constants
#include "AF1000X_PID.h"

// KO: 실제 루프 dt(초), 메인 루프에서 갱신
// EN: Actual loop dt (seconds), updated in main loop
extern float g_loopDt;


// KO: Hover 학습 헤더
// EN: Hover learning header
#include "AF1000X_Hover.h"
#include "AF1000X_AutoTune.h"

#ifndef HOP_MAX
#define HOP_MAX 12
#endif

#ifndef HOP_CH_MIN
#define HOP_CH_MIN 5
#endif
#ifndef HOP_CH_MAX
#define HOP_CH_MAX 80
#endif

#include "AF1000X_BINDING.h"

/* ============================================================================
 * KO: AF1000X 코어 (ESP32-S3)
 * EN: AF1000X core (ESP32-S3)
 * KO: 센서 POST, 고도 융합(ToF+SPL06), 옵티컬 플로우, 바인딩/호버/오토튠 포함
 * EN: Sensor POST, altitude fusion (ToF+SPL06), optical flow, binding/hover/auto-tune
 * ========================================================================== */

// ============================================================================
// KO: 모드 상수 (u8)
// EN: Mode constants (u8)
// ============================================================================
static constexpr uint8_t MODE_READY     = 0;
static constexpr uint8_t MODE_TAKEOFF   = 1;
static constexpr uint8_t MODE_HOVERING  = 2;
static constexpr uint8_t MODE_LANDING   = 3;
static constexpr uint8_t MODE_EMERGENCY = 4;

// ============================================================================
// KO: RF 구조체
// EN: RF structs
// ============================================================================
struct Signal {
  uint8_t throttle, roll, pitch, yaw, aux1, aux2;
  uint8_t speed; // KO: 1~3 / EN: 1~3
  uint8_t hop;   // KO: FHSS 홉 인덱스 / EN: FHSS hop index
};

// KO: AUX2 비트 정의
// EN: AUX2 bit definitions
static const uint8_t AUX2_HEADLESS = 0x01;
static const uint8_t AUX2_FLOW     = 0x02;

struct Telemetry {
  float vbat, alt, posX, posY;
  int rssi;
};

// ============================================================================
// KO: 핀 정의 (AF1000X_GPIO.h로 이동)
// EN: Pins (moved to AF1000X_GPIO.h)
// ============================================================================

// KO: FHSS 기본 테이블 (페어링으로 교체)
// EN: FHSS default table (replaced by pairing)
static const uint8_t HOP_DEFAULT[HOP_MAX] = {63,64,65,66,67,68,69,70,71,72,73,74};
static const uint32_t HOP_SLOT_MS = 20;
static const uint32_t HOP_SCAN_MS = 20;

// ============================================================================
// KO: 전역 (Hover/EasyCommander에서 참조)
// EN: Globals (referenced by Hover/EasyCommander)
// ============================================================================
#ifdef AF1000X_IMPLEMENTATION

// KO: RF
// EN: RF
RF24 radio(PIN_RF_CE, PIN_RF_CSN);
Preferences prefs;
Signal receiverData{};
Telemetry telemetryData{};
uint32_t lastRxMs = 0;
bool failsafe = false;
bool linkReady = false;

uint8_t hopTable[HOP_MAX] = {63,64,65,66,67,68,69,70,71,72,73,74};
uint8_t hopLen = HOP_MAX;

uint8_t hopSeed = 1;
uint32_t hopStartMs = 0;
bool hopSynced = false;
uint8_t hopIdxLast = 0xFF;
uint8_t hopScanIdx = 0;
uint32_t hopScanMs = 0;

float currentRoll  = 0.0f;
float currentPitch = 0.0f;


// KO: 모드 (Hover 헤더는 uint8_t extern 기대)
// EN: Mode (Hover header expects uint8_t extern)
volatile uint8_t currentMode = MODE_READY;

// KO: 타깃 / 상태 (meters)
// EN: Targets / states (meters)
float targetAltitude = TAKEOFF_TARGET_M;
float currentAltitude = 0.0f;

float targetPosX = 0.0f, targetPosY = 0.0f;
float currentPosX = 0.0f, currentPosY = 0.0f;

float targetYaw = 0.0f;   // KO: deg / EN: deg
float currentYaw = 0.0f;  // KO: deg / EN: deg

int speedLevel = 1;
float moveSpeedSteps[3]  = {MOVE_SPEED_STEPS[0], MOVE_SPEED_STEPS[1], MOVE_SPEED_STEPS[2]};
float climbSpeedSteps[3] = {CLIMB_SPEED_STEPS[0], CLIMB_SPEED_STEPS[1], CLIMB_SPEED_STEPS[2]};
float yawSpeedSteps[3]   = {YAW_SPEED_STEPS[0], YAW_SPEED_STEPS[1], YAW_SPEED_STEPS[2]};

float trimRoll = 0.0f, trimPitch = 0.0f;

// KO: 튜닝 게인 (NVS 로드, 기본값은 AF1000X_PID.h)
// EN: Tunable gains (loaded from NVS, defaults from AF1000X_PID.h)
float altKp = ALT_KP_DEFAULT;
float altKd = ALT_KD_DEFAULT;
float yawKp = YAW_KP_DEFAULT;

// KO: 필터 파라미터 (NVS 로드, 기본값은 AF1000X_PID.h)
// EN: Tunable filter parameters (loaded from NVS, defaults from AF1000X_PID.h)
float attTau = ATT_TAU;
float accZMin = ACC_Z_MIN;
float tofEmaA = TOF_EMA_A;
float baroEmaA = BARO_EMA_A;
float altEmaA = ALT_EMA_A;
float tofJumpRejectM = TOF_JUMP_REJECT_M;

// KO: 고도 제어 기반 PWM (Hover 학습이 자동 산출)
// EN: Altitude control PWM (auto-learned by hover)
int hoverThrottle = HOVER_PWM_DEFAULT;

// KO: 배터리 상태 (1S LiPo)
// EN: Battery state (1S LiPo)
float batteryVoltage = 4.2f;      // KO: 1S 완충 전압 / EN: 1S full voltage
float batteryVoltageFilt = 4.2f;  // KO: EMA 필터 / EN: EMA filtered
bool batteryLowWarning = false;
bool batteryCritical = false;

// KO: 센서 OK/락
// EN: Sensor OK/lock
bool ok_imu=false, ok_baro=false, ok_tof=false, ok_flow=false;
bool flightLock = false;
bool calibrated = false;
bool flowUserEnabled = true;

// KO: 모터 시동 / 아이들 스핀 상태
// EN: Motor arm / idle spin state
bool motorArmedIdle = false;
static bool armAwaitSecond = false;
static bool armHighPrev = false;
static bool armPulsing = false;
static uint32_t armFirstUpMs = 0;
static uint32_t armPulseStartMs = 0;
static uint32_t armDisarmStartMs = 0;
static bool autoTakeoffPending = false;
static uint32_t autoTakeoffAtMs = 0;

// KO: 고도 필터
// EN: Altitude filters
bool tofFiltInit=false;
float tof_m_filt=0.0f;
float tof_m_last_raw=0.0f;

bool baroInit=false;
float baro_alt_m_filt=0.0f;

bool altInit=false;
float alt_m_filt=0.0f;
float alt_m_prev=0.0f;

// KO: flowK (m/count/m)
// EN: flowK (m/count/m)
float flowK = FLOW_K_DEFAULT;

// KO: ToF
// EN: ToF
VL53L1X tof;

// KO: IMU (ICM45686)
// EN: IMU (ICM45686)
static ICM456xx IMU0(Wire, 0);
static ICM456xx IMU1(Wire, 1);
static ICM456xx* IMU = nullptr;
float gyroX_bias = 0.0f;
float gyroY_bias = 0.0f;
float gyroZ_bias = 0.0f;
float yaw_internal = 0.0f;
uint32_t lastYawUs = 0;

// KO: 모터 LEDC 채널
// EN: Motors LEDC channels
int ch1=-1,ch2=-1,ch3=-1,ch4=-1;

// KO: 기압계 기준값
// EN: Baro baseline
float basePressurePa = 101325.0f;

// KO: 플로우 캘리브레이션 상태
// EN: Flow calibration state
enum FlowCalState : uint8_t { FC_IDLE, FC_TAKE_HOVER, FC_FORWARD, FC_PAUSE1, FC_BACK, FC_PAUSE2, FC_DONE, FC_ABORT };
FlowCalState flowCal = FC_IDLE;
uint32_t flowCal_t0 = 0;
float flowCal_dist_m = 0.50f;
float flowCal_hoverAlt_m = 0.80f;
float flowCal_pwr = 35.0f;

int32_t flowCal_sum_dx = 0;
uint32_t flowCal_samples = 0;
float flowCal_height_acc = 0.0f;
uint32_t flowCal_height_n = 0;
float flowCal_suggestK = 0.0f;
bool flowCal_reported = false;

#else
extern RF24 radio;
extern Preferences prefs;
extern Signal receiverData;
extern Telemetry telemetryData;
extern uint32_t lastRxMs;
extern bool failsafe;

extern volatile uint8_t currentMode;

extern float targetAltitude, currentAltitude;
extern float targetPosX, targetPosY, currentPosX, currentPosY;
extern float targetYaw, currentYaw;

extern int speedLevel;
extern float moveSpeedSteps[3], climbSpeedSteps[3], yawSpeedSteps[3];

extern float trimRoll, trimPitch;
extern int hoverThrottle;
extern float altKp, altKd, yawKp;
extern float attTau, accZMin, tofEmaA, baroEmaA, altEmaA, tofJumpRejectM;

extern float batteryVoltage;
extern bool batteryLowWarning, batteryCritical;

extern bool ok_imu, ok_baro, ok_tof, ok_flow;
extern bool flightLock;
extern bool calibrated;

extern bool tofFiltInit;
extern float tof_m_filt, tof_m_last_raw;
extern bool baroInit;
extern float baro_alt_m_filt;
extern bool altInit;
extern float alt_m_filt, alt_m_prev;

extern float flowK;
extern VL53L1X tof;

extern float gyroX_bias, gyroY_bias, gyroZ_bias, yaw_internal;
extern uint32_t lastYawUs;

extern int ch1,ch2,ch3,ch4;

extern float basePressurePa;

extern enum FlowCalState : uint8_t { FC_IDLE, FC_TAKE_HOVER, FC_FORWARD, FC_PAUSE1, FC_BACK, FC_PAUSE2, FC_DONE, FC_ABORT } flowCal;
extern uint32_t flowCal_t0;
extern float flowCal_dist_m, flowCal_hoverAlt_m, flowCal_pwr;
extern int32_t flowCal_sum_dx;
extern uint32_t flowCal_samples;
extern float flowCal_height_acc;
extern uint32_t flowCal_height_n;
extern float flowCal_suggestK;
extern bool flowCal_reported;
#endif

// ============================================================================
// KO: 헬퍼
// EN: Helpers
// ============================================================================
static inline float clampf(float v,float lo,float hi){ return (v<lo)?lo:((v>hi)?hi:v); }
static inline void ledsInit(){
  pinMode(PIN_LED_IMU, OUTPUT);
  pinMode(PIN_LED_BARO, OUTPUT);
  pinMode(PIN_LED_TOF, OUTPUT);
  pinMode(PIN_LED_FLOW, OUTPUT);
  digitalWrite(PIN_LED_IMU, LOW);
  digitalWrite(PIN_LED_BARO, LOW);
  digitalWrite(PIN_LED_TOF, LOW);
  digitalWrite(PIN_LED_FLOW, LOW);
}
static inline void ledSet(bool imu_ok2,bool baro_ok2,bool tof_ok2,bool flow_ok2){
  digitalWrite(PIN_LED_IMU,  imu_ok2  ? HIGH : LOW);
  digitalWrite(PIN_LED_BARO, baro_ok2 ? HIGH : LOW);
  digitalWrite(PIN_LED_TOF,  tof_ok2  ? HIGH : LOW);
  digitalWrite(PIN_LED_FLOW, flow_ok2 ? HIGH : LOW);
}
static inline void ledAll(bool on){
  digitalWrite(PIN_LED_IMU,  on?HIGH:LOW);
  digitalWrite(PIN_LED_BARO, on?HIGH:LOW);
  digitalWrite(PIN_LED_TOF,  on?HIGH:LOW);
  digitalWrite(PIN_LED_FLOW, on?HIGH:LOW);
}
static inline void ledFailPattern(){
  for(int i=0;i<6;i++){
    ledAll(true);  delay(80);
    ledAll(false); delay(80);
  }
}

// ============================================================================
// KO: LED 상태 머신 (부팅/바인딩/저전압/자이로/헤드리스/오토튠)
// EN: LED state machine (boot/bind/low batt/gyro/headless/auto-tune)
// KO: 부팅/바인딩 시퀀스는 최우선, 이후 저전압/자이로/헤드리스/오토튠 순
// EN: Boot/bind has top priority; then low battery, gyro, headless, auto-tune
// ============================================================================

static bool g_ledInvertedAtBoot = false;

static bool g_ledLowBattery = false;
static bool g_ledHeadless   = false;
static bool g_ledBound      = false;
static bool g_ledAutoTune   = false;

// KO: 자이로 리셋 애니메이션 (논블로킹)
// EN: Gyro reset animation (non-blocking)
static bool     s_ledGyroAnimActive = false;
static uint8_t  s_ledGyroAnimCount  = 0;   // KO: ON 단계 완료 횟수 / EN: ON phases completed
static bool     s_ledGyroAnimOn     = false;
static uint32_t s_ledGyroNextMs     = 0;

static inline void ledStartGyroResetAnim(){
  s_ledGyroAnimActive = true;
  s_ledGyroAnimCount  = 0;
  s_ledGyroAnimOn     = true;
  s_ledGyroNextMs     = millis();
}

// KO: 부팅/바인딩 시퀀스
// EN: Boot/binding sequence
enum LedBootState : uint8_t {
  LED_BOOT_BLINK_ALL = 0,
  LED_BOOT_CHASE_INVERTED = 1,
  LED_BOOT_BOUND_SOLID = 2,
  LED_BOOT_DONE = 3
};
static LedBootState s_ledBootState = LED_BOOT_BLINK_ALL;
static uint32_t s_ledBootStartMs = 0;
static bool s_ledPrevBound = false;

static inline bool _blink(uint32_t now, uint32_t onMs, uint32_t offMs){
  uint32_t period = onMs + offMs;
  uint32_t t = now % period;
  return (t < onMs);
}

static inline void _ledOneHot(uint8_t idx){
  digitalWrite(PIN_LED_IMU,  idx==0 ? HIGH : LOW);
  digitalWrite(PIN_LED_BARO, idx==1 ? HIGH : LOW);
  digitalWrite(PIN_LED_TOF,  idx==2 ? HIGH : LOW);
  digitalWrite(PIN_LED_FLOW, idx==3 ? HIGH : LOW);
}

// KO: 초기화 시 1회 호출 (IMU 체크 후 inverted 플래그 유효)
// EN: Call once during init (after IMU check so inverted flag is valid)
static inline void ledBootBegin(bool invertedAtBoot){
  g_ledInvertedAtBoot = invertedAtBoot;
  s_ledBootStartMs = millis();
  s_ledPrevBound = g_ledBound;
  s_ledBootState = invertedAtBoot ? LED_BOOT_CHASE_INVERTED : LED_BOOT_BLINK_ALL;
}

// KO: 내부 부팅/바인딩 업데이트 (이 tick에서 LED 처리 시 true)
// EN: Internal boot/bind updater (returns true if it handled LEDs this tick)
static inline bool ledBootTick(){
  uint32_t now = millis();

  // KO: 바인딩 상승 에지 감지
  // EN: Detect rising edge: not bound -> bound
  if (!s_ledPrevBound && g_ledBound) {
    s_ledBootState = LED_BOOT_BOUND_SOLID;
    s_ledBootStartMs = now;
  }
  s_ledPrevBound = g_ledBound;

  if (s_ledBootState == LED_BOOT_BOUND_SOLID) {
    ledAll(true);
    if (now - s_ledBootStartMs >= 1000) {
      s_ledBootState = LED_BOOT_DONE;
    }
    return true;
  }

  if (s_ledBootState == LED_BOOT_DONE) return false;

  if (s_ledBootState == LED_BOOT_BLINK_ALL) {
    bool on = _blink(now, 1000, 1000);
    ledAll(on);
    return true;
  }

  // KO: 뒤집힘 체이스 (200ms 스텝)
  // EN: inverted chase (200ms step)
  if (s_ledBootState == LED_BOOT_CHASE_INVERTED) {
    uint8_t step = (uint8_t)((now / 200) % 4);
    _ledOneHot(step);
    return true;
  }
  return false;
}

// KO: 메인 LED 업데이트 (매 루프 호출)
// EN: Main LED update (call every loop)
static inline void ledTick(){
  uint32_t now = millis();

  // KO: 0) 부팅/바인딩 우선
  // EN: 0) Boot/bind sequence has top priority
  if (ledBootTick()) return;

  // KO: A) 저전압
  // EN: A) Low battery
  if (g_ledLowBattery) {
    bool on = _blink(now, 1000, 1000); // KO: 1s on / 1s off / EN: 1s on / 1s off
    ledAll(on);
    return;
  }

  // KO: B) 자이로 리셋 애니메이션
  // EN: B) Gyro reset animation
  if (s_ledGyroAnimActive) {
    if (now >= s_ledGyroNextMs) {
      if (s_ledGyroAnimOn) {
        s_ledGyroAnimOn = false;
        s_ledGyroNextMs = now + 500;
      } else {
        s_ledGyroAnimOn = true;
        s_ledGyroNextMs = now + 1000;
        s_ledGyroAnimCount++;
        if (s_ledGyroAnimCount >= 3) {
          s_ledGyroAnimActive = false;
        }
      }
    }
    ledAll(s_ledGyroAnimOn);
    return;
  }

  // KO: C) 헤드리스 (LED3/4 2s on / 1s off)
  // EN: C) Headless (LED3/4 2s on / 1s off)
  if (g_ledHeadless) {
    bool on34 = (now % 3000) < 2000;
    digitalWrite(PIN_LED_IMU,  LOW);
    digitalWrite(PIN_LED_BARO, LOW);
    digitalWrite(PIN_LED_TOF,  on34 ? HIGH : LOW);
    digitalWrite(PIN_LED_FLOW, on34 ? HIGH : LOW);
    return;
  }

  // KO: D) 오토튠 (LED1/2 on, LED3/4 off 1.5s 교대)
  // EN: D) Auto tune (LED1/2 on, LED3/4 off 1.5s swap)
  if (g_ledAutoTune) {
    bool phase = ((now / 1500) % 2) == 0;
    digitalWrite(PIN_LED_IMU,  phase ? HIGH : LOW);
    digitalWrite(PIN_LED_BARO, phase ? HIGH : LOW);
    digitalWrite(PIN_LED_TOF,  phase ? LOW  : HIGH);
    digitalWrite(PIN_LED_FLOW, phase ? LOW  : HIGH);
    return;
  }

  // KO: E) 일반 상태 (기존 센서 LED 유지)
  // EN: E) Normal (keep existing sensor LED logic)
}

static inline float readBatteryVoltage(){
  int raw = analogRead(PIN_BAT_ADC);
  float vadc = (raw / 4095.0f) * 3.3f;
  float divider = (BATTERY_RTOP_KOHM + BATTERY_RBOT_KOHM) / BATTERY_RBOT_KOHM;
  float vbat = vadc * divider;
  if(BATTERY_CAL_ACTUAL_V > 0.0f && BATTERY_CAL_MEASURED_V > 0.0f){
    vbat *= (BATTERY_CAL_ACTUAL_V / BATTERY_CAL_MEASURED_V);
  }
  return vbat;
}

// KO: 배터리 체크
// EN: Battery check
static inline void checkBattery(){
  batteryVoltage = readBatteryVoltage();

  // KO: EMA 로우패스 (지터 방지)
  // EN: EMA low-pass to prevent jitter
  const float alpha = 0.20f; // KO: 0..1 (클수록 빠름) / EN: 0..1 (higher=faster)
  batteryVoltageFilt = batteryVoltageFilt + alpha * (batteryVoltage - batteryVoltageFilt);

  if(batteryVoltageFilt < BATTERY_MIN_VOLTAGE){
    if(!batteryCritical){
      batteryCritical = true;
      Serial.printf("⚠️ CRITICAL: Battery %.2fV - Emergency landing!\n", batteryVoltageFilt);
    }
    // KO: 비행 중이면 강제 착륙
    // EN: Force landing if in flight
    if(currentMode == MODE_HOVERING || currentMode == MODE_TAKEOFF){
      currentMode = MODE_LANDING;
    }
  } else if(batteryVoltageFilt < BATTERY_WARNING_VOLTAGE){
    if(!batteryLowWarning){
      batteryLowWarning = true;
      Serial.printf("⚠️ WARNING: Battery low %.2fV\n", batteryVoltageFilt);
    }
  } else {
    batteryLowWarning = false;
    batteryCritical = false;
  }
}



// ============================================================================
// KO: 바인딩 LED 훅 (AF1000X_BINDING.h 사용)
// EN: Binding LED hooks (used by AF1000X_BINDING.h)
// KO: 실제 패턴은 ledTick()/ledBootBegin()/g_ledBound로 제어
// EN: Actual patterns controlled by ledTick()/ledBootBegin()/g_ledBound
// ============================================================================
inline void binding_ledAll(bool on) { ledAll(on); }

// KO: 페어링/바인딩 대기 중 반복 호출
// EN: Called repeatedly while waiting for pairing/binding
inline void binding_ledPairingTick() {
  ledTick();
}

// KO: 바인딩 완료 시 1회 호출
// EN: Called once when binding completes
inline void binding_ledBoundOnce() {
  ledAll(true);
}

// KO: 바인딩 에러 시 호출
// EN: Called on binding error
inline void binding_ledErrorOnce() {
  // KO: 짧은 에러 플래시
  // EN: quick error flash
  for(int i=0;i<3;i++){ ledAll(true); delay(120); ledAll(false); delay(120); }
}

static inline void motorsInit(){
  pinMode(PIN_M1, OUTPUT);
  pinMode(PIN_M2, OUTPUT);
  pinMode(PIN_M3, OUTPUT);
  pinMode(PIN_M4, OUTPUT);

  ch1 = ledcAttach(PIN_M1, PWM_FREQ, PWM_RES);
  ch2 = ledcAttach(PIN_M2, PWM_FREQ, PWM_RES);
  ch3 = ledcAttach(PIN_M3, PWM_FREQ, PWM_RES);
  ch4 = ledcAttach(PIN_M4, PWM_FREQ, PWM_RES);

  ledcWrite(ch1, 0);
  ledcWrite(ch2, 0);
  ledcWrite(ch3, 0);
  ledcWrite(ch4, 0);
}
static inline void motorsOff(){
  ledcWrite(ch1, 0);
  ledcWrite(ch2, 0);
  ledcWrite(ch3, 0);
  ledcWrite(ch4, 0);
}
static inline void motorsIdle(int pwm){
  int v = constrain(pwm, 0, 255);
  ledcWrite(ch1, v);
  ledcWrite(ch2, v);
  ledcWrite(ch3, v);
  ledcWrite(ch4, v);
}
static inline void motorControl(int thr, float r, float p, float ycmd){
  if(currentMode==MODE_READY || currentMode==MODE_EMERGENCY){ motorsOff(); return; }

  float fr = r + trimRoll;
  float fp = p + trimPitch;

  int m1 = constrain((int)(thr - fr + fp + ycmd), 0, 255);
  int m2 = constrain((int)(thr - fr - fp - ycmd), 0, 255);
  int m3 = constrain((int)(thr + fr - fp + ycmd), 0, 255);
  int m4 = constrain((int)(thr + fr + fp - ycmd), 0, 255);

  ledcWrite(ch1, m1);
  ledcWrite(ch2, m2);
  ledcWrite(ch3, m3);
  ledcWrite(ch4, m4);
}

// ============================================================================
// KO: RF
// EN: RF
// ============================================================================
static inline void radioInit(){
  radio.begin();
  radio.setChannel(PAIR_CHANNEL);
  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.openReadingPipe(1, RF_ADDR);
  radio.startListening();
  lastRxMs = millis();
}

static inline uint8_t hopIndexForMs(uint32_t now){
  if(HOP_SLOT_MS == 0 || hopLen == 0) return 0;
  uint32_t slots = (now - hopStartMs) / HOP_SLOT_MS;
  return (uint8_t)((slots + hopSeed) % hopLen);
}

static inline void fhssSetChannelIdx(uint8_t idx){
  if(hopLen == 0) return;
  if(idx >= hopLen) idx = 0;
  if(idx == hopIdxLast) return;
  hopIdxLast = idx;
  radio.stopListening();
  radio.setChannel(hopTable[idx]);
  radio.startListening();
}

static inline void fhssScanTick(uint32_t now){
  if(hopLen == 0) return;
  if(now - hopScanMs >= HOP_SCAN_MS){
    hopScanMs = now;
    hopScanIdx = (uint8_t)((hopScanIdx + 1) % hopLen);
    fhssSetChannelIdx(hopScanIdx);
  }
}

static inline void fhssResync(uint8_t rxHop, uint32_t now){
  if(hopLen == 0) return;
  if(rxHop >= hopLen) return;
  uint8_t seedMod = (uint8_t)(hopSeed % hopLen);
  uint8_t offset = (uint8_t)((rxHop + hopLen - seedMod) % hopLen);
  hopStartMs = now - (uint32_t)offset * HOP_SLOT_MS;
  hopSynced = true;
}

static inline void fhssUpdate(uint32_t now){
  if(hopLen == 0){
    fhssSetChannelIdx(0);
    return;
  }
  if(hopSynced){
    uint8_t idx = hopIndexForMs(now);
    fhssSetChannelIdx(idx);
  } else {
    fhssScanTick(now);
  }
}

static inline void fhssInit(){
  if(hopLen == 0 || hopLen > HOP_MAX){
    hopLen = HOP_MAX;
    for(uint8_t i = 0; i < HOP_MAX; i++) hopTable[i] = HOP_DEFAULT[i];
  }
  hopSeed = binding_getHopSeed();
  if(hopSeed == 0) hopSeed = 1;
  hopStartMs = millis();
  hopSynced = false;
  hopIdxLast = 0xFF;
  hopScanIdx = 0;
  hopScanMs = hopStartMs;
}
static inline void updateRadio(){
  uint32_t now = millis();
  fhssUpdate(now);

  if(radio.available()){
    radio.read(&receiverData, sizeof(Signal));
    lastRxMs = now;
    failsafe = false;
    fhssResync(receiverData.hop, now);

    if(receiverData.speed >= 1 && receiverData.speed <= 3){
      speedLevel = receiverData.speed - 1;
    }

    telemetryData.vbat = batteryVoltage;
    telemetryData.alt  = currentAltitude;
    telemetryData.posX = currentPosX;
    telemetryData.posY = currentPosY;
    telemetryData.rssi = 1;

    radio.writeAckPayload(1, &telemetryData, sizeof(Telemetry));
  }

  if(now - lastRxMs > FAILSAFE_MS){
    failsafe = true;
    hopSynced = false;
    if(currentMode != MODE_READY && currentMode != MODE_EMERGENCY){

      // KO: 페일세이프 시 현재 위치/요 유지 후 착륙
      // EN: On failsafe, hold current pos/yaw then land
      targetPosX = currentPosX;
      targetPosY = currentPosY;
      targetYaw  = currentYaw;

      currentMode = MODE_LANDING;
    }
  }
}

// ============================================================================
// KO: IMU (ICM45686) 요 적분
// EN: IMU (ICM45686) yaw integration
// ============================================================================
static inline float safeDt(float dt){ return (dt<=0.0f || dt>0.1f) ? 0.0f : dt; }

static inline bool imu_init_auto(){
  int ret = IMU0.begin();
  if(ret == 0) IMU = &IMU0;
  else {
    ret = IMU1.begin();
    if(ret == 0) IMU = &IMU1;
  }
  if(IMU == nullptr) return false;

  IMU->startAccel(200, 16);
  IMU->startGyro (200, 2000);

  gyroX_bias = 0.0f;
  gyroY_bias = 0.0f;
  gyroZ_bias = 0.0f;
  yaw_internal = 0.0f;
  currentRoll = 0.0f;
  currentPitch = 0.0f;
  lastYawUs = micros();
  return true;
}
static inline bool imu_calibrate_gyroZ(uint16_t samples=800, uint16_t delay_ms=2){
  if(!ok_imu || IMU == nullptr) return false;

  float sumX=0.0f, sumY=0.0f, sumZ=0.0f; uint16_t got=0;
  inv_imu_sensor_data_t d{};
  for(uint16_t i=0;i<samples;i++){
    if(IMU->getDataFromRegisters(d) == 0){
      sumX += (float)d.gyro_data[0];
      sumY += (float)d.gyro_data[1];
      sumZ += (float)d.gyro_data[2];
      got++;
    }
    delay(delay_ms);
  }
  if(got < (uint16_t)(samples*0.6f)) return false;

  gyroX_bias = sumX / (float)got;
  gyroY_bias = sumY / (float)got;
  gyroZ_bias = sumZ / (float)got;
  yaw_internal = 0.0f;
  currentRoll = 0.0f;
  currentPitch = 0.0f;
  lastYawUs = micros();
  return true;
}
static inline void imu_updateYaw(){
  if(!ok_imu || IMU == nullptr) return;
  inv_imu_sensor_data_t d{};
  if(IMU->getDataFromRegisters(d) != 0) return;

  float ax = (float)d.accel_data[0];
  float ay = (float)d.accel_data[1];
  float az = (float)d.accel_data[2];

  uint32_t now = micros();
  float dt = (now - lastYawUs) * 1e-6f;
  lastYawUs = now;
  dt = safeDt(dt);
  if(dt <= 0.0f) return;

  float gx = (float)d.gyro_data[0] - gyroX_bias;
  float gy = (float)d.gyro_data[1] - gyroY_bias;
  float gz = (float)d.gyro_data[2] - gyroZ_bias;

  // KO: 롤/피치 상보 필터 (자이로 + 가속도)
  // EN: Complementary filter for roll/pitch (gyro + accel)
  float accRoll = currentRoll;
  float accPitch = currentPitch;
  if (fabsf(az) >= accZMin) {
    // KO: 단위/스케일이 거칠어도 뒤집힘 감지에는 충분
    // EN: Scale need not be perfect for inverted detection
    accRoll  = atan2f(ay, az) * 57.2957795f;
    accPitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 57.2957795f;
  }
  float alpha = attTau / (attTau + dt);
  currentRoll  = alpha * (currentRoll  + gx * dt) + (1.0f - alpha) * accRoll;
  currentPitch = alpha * (currentPitch + gy * dt) + (1.0f - alpha) * accPitch;

  yaw_internal += gz * dt;

  while(yaw_internal > 180.0f) yaw_internal -= 360.0f;
  while(yaw_internal < -180.0f) yaw_internal += 360.0f;

  currentYaw = yaw_internal;
}
static inline void yawReset(){
  yaw_internal = 0.0f;
  lastYawUs = micros();
  currentYaw = 0.0f;
  targetYaw = 0.0f;
}

// ============================================================================
// KO: SPL06 최소 드라이버 (I2C -> 압력 -> 고도)
// EN: SPL06 minimal driver (I2C -> pressure -> altitude)
// ============================================================================
static uint8_t  spl_addr = 0;
static bool     spl_ok = false;
static int16_t  spl_c0, spl_c1;
static int32_t  spl_c00, spl_c10;
static int16_t  spl_c01, spl_c11, spl_c20, spl_c21;
static int16_t  spl_c30 = 0;
static uint32_t kT = 524288;
static uint32_t kP = 253952;

static inline bool i2cProbe(uint8_t addr){
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}
static inline bool i2cRead(uint8_t addr, uint8_t reg, uint8_t* buf, size_t n){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if(Wire.endTransmission(false) != 0) return false;
  if(Wire.requestFrom((int)addr, (int)n) != (int)n) return false;
  for(size_t i=0;i<n;i++) buf[i]=Wire.read();
  return true;
}
static inline bool i2cWrite1(uint8_t addr, uint8_t reg, uint8_t val){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}
static inline int32_t getTwosComplement(uint32_t raw, uint8_t len){
  if(raw & ((uint32_t)1 << (len-1))){
    return (int32_t)(raw - ((uint32_t)1 << len));
  }
  return (int32_t)raw;
}
static inline bool spl_init(){
  const uint8_t addrs[]={0x76,0x77};
  for(auto a : addrs){
    if(i2cProbe(a)){
      spl_addr = a;
      break;
    }
  }
  if(!spl_addr) return false;

  uint8_t id=0;
  if(!i2cRead(spl_addr, 0x0D, &id,1)) return false;
  if((id&0xF0)!=0x10) return false;

  uint8_t coef[18];
  if(!i2cRead(spl_addr, 0x10, coef, 18)) return false;

  spl_c0  = (int16_t)(((uint16_t)coef[0] << 4) | (((uint16_t)coef[1] >> 4) & 0x0F));
  spl_c0  = (int16_t)getTwosComplement((uint32_t)spl_c0, 12);

  spl_c1  = (int16_t)((((uint16_t)coef[1] & 0x0F) << 8) | (uint16_t)coef[2]);
  spl_c1  = (int16_t)getTwosComplement((uint32_t)spl_c1, 12);

  spl_c00 = (int32_t)(((uint32_t)coef[3] << 12) | ((uint32_t)coef[4] << 4) | (((uint32_t)coef[5] >> 4) & 0x0F));
  spl_c00 = getTwosComplement((uint32_t)spl_c00, 20);

  spl_c10 = (int32_t)((((uint32_t)coef[5] & 0x0F) << 16) | ((uint32_t)coef[6] << 8) | (uint32_t)coef[7]);
  spl_c10 = getTwosComplement((uint32_t)spl_c10, 20);

  spl_c01 = (int16_t)(((uint16_t)coef[8] << 8) | (uint16_t)coef[9]);
  spl_c11 = (int16_t)(((uint16_t)coef[10] << 8) | (uint16_t)coef[11]);
  spl_c20 = (int16_t)(((uint16_t)coef[12] << 8) | (uint16_t)coef[13]);
  spl_c21 = (int16_t)(((uint16_t)coef[14] << 8) | (uint16_t)coef[15]);
  spl_c30 = (int16_t)(((uint16_t)coef[16] << 8) | (uint16_t)coef[17]);

  uint8_t prs_cfg = (0x01 << 4) | 0x03;
  uint8_t tmp_cfg = (0x80) | (0x01 << 4) | 0x03;
  if(!i2cWrite1(spl_addr, 0x06, prs_cfg)) return false;
  if(!i2cWrite1(spl_addr, 0x07, tmp_cfg)) return false;

  uint8_t cfg = 0x00 | 0x04;
  if(!i2cWrite1(spl_addr, 0x08, cfg)) return false;

  uint8_t meas = 0x07;
  if(!i2cWrite1(spl_addr, 0x09, meas)) return false;

  spl_ok = true;
  return true;
}
static inline bool spl_readPressurePa(float &out){
  if(!spl_ok) return false;
  uint8_t buf[6];
  if(!i2cRead(spl_addr, 0x00, buf,6)) return false;

  int32_t praw = (int32_t)((((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[2]));
  praw = getTwosComplement((uint32_t)praw, 24);

  int32_t traw = (int32_t)((((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 8) | (uint32_t)buf[5]));
  traw = getTwosComplement((uint32_t)traw, 24);

  float praw_sc = (float)praw / (float)kP;
  float traw_sc = (float)traw / (float)kT;

  float pComp = (float)spl_c00
               + praw_sc * ((float)spl_c10 + praw_sc * ((float)spl_c20 + praw_sc * (float)spl_c30))
               + traw_sc * ((float)spl_c01)
               + traw_sc * praw_sc * ((float)spl_c11 + praw_sc * (float)spl_c21);

  out = pComp;
  return true;
}

// ============================================================================
// KO: ToF (VL53L1X)
// EN: ToF (VL53L1X)
// ============================================================================
static inline bool tof_init(){
  pinMode(PIN_VL53_XSHUT, OUTPUT);
  digitalWrite(PIN_VL53_XSHUT, LOW);
  delay(10);
  digitalWrite(PIN_VL53_XSHUT, HIGH);
  delay(10);

  tof.setTimeout(500);
  if(!tof.init()) return false;

  tof.setDistanceMode(VL53L1X::Short);
  tof.setMeasurementTimingBudget(20000);
  tof.startContinuous(20);
  return true;
}
static inline bool tof_read_m(float &out){
  uint16_t mm = tof.read(false);
  if(tof.timeoutOccurred()) return false;
  out = (float)mm * 0.001f;
  return true;
}

// ============================================================================
// KO: PMW3901 최소 드라이버 (SPI 옵티컬 플로우)
// EN: PMW3901 minimal (SPI) optical flow
// ============================================================================
static uint8_t pmw_read(uint8_t reg){
  digitalWrite(PIN_FLOW_CS, LOW);
  delayMicroseconds(2);
  SPI.transfer(reg & 0x7F);
  delayMicroseconds(160);
  uint8_t val = SPI.transfer(0);
  digitalWrite(PIN_FLOW_CS, HIGH);
  delayMicroseconds(1);
  return val;
}
static void pmw_write(uint8_t reg, uint8_t val){
  digitalWrite(PIN_FLOW_CS, LOW);
  delayMicroseconds(2);
  SPI.transfer(reg | 0x80);
  SPI.transfer(val);
  digitalWrite(PIN_FLOW_CS, HIGH);
  delayMicroseconds(1);
}
static inline bool pmw_init(){
  pinMode(PIN_FLOW_CS, OUTPUT);
  digitalWrite(PIN_FLOW_CS, HIGH);
  pinMode(PIN_FLOW_MOTION, INPUT);

  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));

  pmw_write(0x3A, 0x5A);
  delay(5);

  uint8_t pid = pmw_read(0x00);
  uint8_t inv_pid = pmw_read(0x5F);
  if(pid != 0x49 || inv_pid != 0xB6) return false;

  pmw_write(0x7F, 0x00);
  pmw_write(0x55, 0x01);
  pmw_write(0x50, 0x07);
  pmw_write(0x7F, 0x0E);
  pmw_write(0x43, 0x10);

  pmw_write(0x7F, 0x00);
  pmw_write(0x51, 0x7B);
  pmw_write(0x50, 0x00);
  pmw_write(0x55, 0x00);
  pmw_write(0x7F, 0x0E);
  pmw_write(0x43, 0x00);
  pmw_write(0x7F, 0x00);

  delay(10);
  return true;
}
struct PMWData {
  int16_t dx, dy;
  uint8_t squal;
  bool valid;
};
static inline PMWData pmw_burst(){
  PMWData r{0,0,0,false};

  pmw_write(0x7F, 0x00);
  digitalWrite(PIN_FLOW_CS, LOW);
  delayMicroseconds(2);
  SPI.transfer(0x16);
  delayMicroseconds(35);

  uint8_t buf[12];
  for(int i=0;i<12;i++) buf[i] = SPI.transfer(0);
  digitalWrite(PIN_FLOW_CS, HIGH);

  r.dx = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
  r.dy = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));
  r.squal = buf[6];
  r.valid = (buf[0] & 0x80) ? true : false;

  return r;
}

// ============================================================================
// KO: 고도 융합 (ToF + Baro)
// EN: Altitude fusion (ToF + Baro)
// ============================================================================
static inline void updateAltitudeFusion(){
  float tofNew;
  if(ok_tof && tof_read_m(tofNew)){
    if(!tofFiltInit){
      tof_m_filt = tofNew;
      tof_m_last_raw = tofNew;
      tofFiltInit = true;
    } else {
      if(fabsf(tofNew - tof_m_last_raw) > tofJumpRejectM){
        // KO: 급변 reject
        // EN: reject sudden jump
      } else {
        tof_m_filt += tofEmaA * (tofNew - tof_m_filt);
        tof_m_last_raw = tofNew;
      }
    }
  }

  float pPa=0.0f;
  if(ok_baro && spl_readPressurePa(pPa)){
    float h_m = (1.0f - powf(pPa/basePressurePa, 0.190295f)) * 44307.7f;
    if(!baroInit){
      baro_alt_m_filt = h_m;
      baroInit = true;
    } else {
      baro_alt_m_filt += baroEmaA * (h_m - baro_alt_m_filt);
    }
  }

  float fused = 0.0f;
  if(tofFiltInit && baroInit){
    if(tof_m_filt < ALT_SWITCH_M){
      fused = tof_m_filt;
    } else {
      fused = baro_alt_m_filt;
    }
  } else if(tofFiltInit){
    fused = tof_m_filt;
  } else if(baroInit){
    fused = baro_alt_m_filt;
  } else {
    fused = 0.0f;
  }

  if(!altInit){
    alt_m_filt = fused;
    alt_m_prev = fused;
    altInit = true;
  } else {
    alt_m_prev = alt_m_filt;
    alt_m_filt += altEmaA * (fused - alt_m_filt);
  }

  currentAltitude = alt_m_filt;
}

// ============================================================================
// KO: 플로우 + 요 회전으로 위치 추정
// EN: Position from flow + yaw rotation
// ============================================================================
static inline void updatePositionFromFlow(){
  if(!ok_flow || !flowUserEnabled) return;

  PMWData f = pmw_burst();
  if(!f.valid || f.squal < FLOW_SQUAL_MIN) return;

  // KO: 플로우 캘리브레이션 누적
  // EN: Flow calibration accumulation
  if(flowCal == FC_FORWARD || flowCal == FC_BACK){
    if(currentAltitude > 0.20f){
      flowCal_sum_dx += f.dx;
      flowCal_samples++;
      flowCal_height_acc += currentAltitude;
      flowCal_height_n++;
    }
  }

  float h = currentAltitude;
  if(h < 0.10f) h = 0.10f;

  float dX_body = (float)f.dx * flowK * h;
  float dY_body = (float)f.dy * flowK * h;

  // KO: 롤/피치 기반 틸트 보정 (1m 근처에서만)
  // EN: Tilt compensation using roll/pitch estimate (only near 1m)
  if(fabsf(currentAltitude - FLOW_TILT_COMP_ALT_M) <= FLOW_TILT_COMP_WINDOW_M){
    float rollRad = currentRoll * DEG_TO_RAD;
    float pitchRad = currentPitch * DEG_TO_RAD;
    float cosR = cosf(rollRad);
    float cosP = cosf(pitchRad);
    if(fabsf(cosR) < 0.35f) cosR = (cosR >= 0.0f) ? 0.35f : -0.35f;
    if(fabsf(cosP) < 0.35f) cosP = (cosP >= 0.0f) ? 0.35f : -0.35f;
    dX_body /= cosR;
    dY_body /= cosP;
  }

  if(fabsf(dX_body) > FLOW_MAX_STEP_M) dX_body = (dX_body>0) ? FLOW_MAX_STEP_M : -FLOW_MAX_STEP_M;
  if(fabsf(dY_body) > FLOW_MAX_STEP_M) dY_body = (dY_body>0) ? FLOW_MAX_STEP_M : -FLOW_MAX_STEP_M;

  float rad = currentYaw * DEG_TO_RAD;
  float c = cosf(rad);
  float s = sinf(rad);
  float dX_world = dX_body * c - dY_body * s;
  float dY_world = dX_body * s + dY_body * c;

  currentPosX += dX_world;
  currentPosY += dY_world;
}

// ============================================================================
// KO: 플로우 자동 캘리브레이션
// EN: Flow auto calibration
// ============================================================================
static inline void startFlowAutoCal(float dist_m, float hoverAlt_m, float pwr){
  if(currentMode != MODE_READY){
    Serial.println("FLOW CAL: must be in READY mode.");
    return;
  }

  flowCal_dist_m = dist_m;
  flowCal_hoverAlt_m = hoverAlt_m;
  flowCal_pwr = pwr;
  flowCal_sum_dx = 0;
  flowCal_samples = 0;
  flowCal_height_acc = 0.0f;
  flowCal_height_n = 0;
  flowCal_suggestK = 0.0f;
  flowCal_reported = false;

  flowCal = FC_TAKE_HOVER;
  flowCal_t0 = millis();

  hover_startLearn();

  Serial.printf("FLOW CAL START: dist=%.2fm hover=%.2fm pwr=%.0f%%\n", dist_m, hoverAlt_m, pwr);
}

static inline void flowCalTick(){
  if(flowCal == FC_IDLE) return;

  switch(flowCal){
    case FC_TAKE_HOVER: {
      if(hover_isReady()){
        targetAltitude = flowCal_hoverAlt_m;
        if(fabsf(currentAltitude - flowCal_hoverAlt_m) < 0.10f){
          flowCal = FC_FORWARD;
          flowCal_t0 = millis();
          Serial.println("FLOW CAL: hovering stable -> forward");
        }
      }
      if((millis() - flowCal_t0) > 15000){
        flowCal = FC_ABORT;
        Serial.println("FLOW CAL: ABORT (timeout hover)");
      }
    } break;

    case FC_FORWARD: {
      float step_m = (moveSpeedSteps[speedLevel] * (flowCal_pwr/100.0f)) * g_loopDt;
      if(step_m < 0.002f) step_m = 0.002f;
      targetPosX += step_m;

      if(fabsf(currentPosX - targetPosX) > flowCal_dist_m){
        flowCal_t0 = millis();
        flowCal = FC_PAUSE1;
      }
    } break;

    case FC_PAUSE1: {
      if(millis() - flowCal_t0 > 500){
        flowCal_t0 = millis();
        flowCal = FC_BACK;
      }
    } break;

    case FC_BACK: {
      float step_m = (moveSpeedSteps[speedLevel] * (flowCal_pwr/100.0f)) * g_loopDt;
      if(step_m < 0.002f) step_m = 0.002f;
      targetPosX -= step_m;

      if(fabsf(currentPosX - targetPosX) < 0.10f || (millis() - flowCal_t0) > 6000){
        flowCal_t0 = millis();
        flowCal = FC_PAUSE2;
      }
    } break;

    case FC_PAUSE2: {
      if(millis() - flowCal_t0 > 500){
        if(flowCal_samples < 50 || flowCal_height_n < 50){
          flowCal = FC_ABORT;
          Serial.println("FLOW CAL: ABORT (not enough samples)");
          break;
        }
        float avgH = flowCal_height_acc / (float)flowCal_height_n;
        float counts = (float)abs(flowCal_sum_dx);
        if(avgH < 0.10f || counts < 5.0f){
          flowCal = FC_ABORT;
          Serial.println("FLOW CAL: ABORT (bad data)");
          break;
        }

        flowCal_suggestK = flowCal_dist_m / (avgH * counts);
        flowK = flowCal_suggestK;
        prefs.putFloat("flowK", flowK);

        flowCal = FC_DONE;
        Serial.println("FLOW CAL: DONE");
      }
    } break;

    default: break;
  }

  if(flowCal == FC_DONE && !flowCal_reported){
    flowCal_reported = true;
    float avgH = (flowCal_height_n ? flowCal_height_acc/(float)flowCal_height_n : 0.0f);
    Serial.printf("FLOW CAL RESULT: avgH=%.2fm dxCounts=%ld flowK=%.6f (saved)\n",
                  avgH, (long)flowCal_sum_dx, flowK);
    Serial.println("If measured actual distance, send: D60.0  (cm)");
  }
}

// ============================================================================
// KO: 튜닝 덤프 헬퍼 (Serial)
// EN: Tuning dump helpers (Serial)
// ============================================================================
static inline void printTuning(bool asPidDefaults){
  if(asPidDefaults){
    Serial.println("---- PID.h defaults (copy) ----");
    Serial.printf("static const float ALT_KP_DEFAULT = %.4ff;\n", altKp);
    Serial.printf("static const float ALT_KD_DEFAULT = %.4ff;\n", altKd);
    Serial.printf("static const float YAW_KP_DEFAULT = %.4ff;\n", yawKp);
    Serial.printf("static const float FLOW_K_DEFAULT = %.6ff;\n", flowK);
    Serial.printf("static const int   HOVER_PWM_DEFAULT = %d;\n", hoverThrottle);
    Serial.printf("static const float ATT_TAU = %.3ff;\n", attTau);
    Serial.printf("static const float ACC_Z_MIN = %.3ff;\n", accZMin);
    Serial.printf("static const float TOF_EMA_A = %.3ff;\n", tofEmaA);
    Serial.printf("static const float BARO_EMA_A = %.3ff;\n", baroEmaA);
    Serial.printf("static const float ALT_EMA_A = %.3ff;\n", altEmaA);
    Serial.printf("static const float TOF_JUMP_REJECT_M = %.3ff;\n", tofJumpRejectM);
    Serial.println("---- end ----");
    return;
  }

  Serial.println("---- Tuning dump ----");
  Serial.printf("ALT_KP=%.4f ALT_KD=%.4f YAW_KP=%.4f\n", altKp, altKd, yawKp);
  Serial.printf("FLOW_K=%.6f HOVER_PWM=%d\n", flowK, hoverThrottle);
  Serial.printf("ATT_TAU=%.3f ACC_Z_MIN=%.3f\n", attTau, accZMin);
  Serial.printf("TOF_EMA_A=%.3f BARO_EMA_A=%.3f ALT_EMA_A=%.3f TOF_JUMP_REJECT_M=%.3f\n",
                tofEmaA, baroEmaA, altEmaA, tofJumpRejectM);
  Serial.printf("GYRO_BIAS X=%.4f Y=%.4f Z=%.4f\n", gyroX_bias, gyroY_bias, gyroZ_bias);
  Serial.println("---- end ----");
}

// ============================================================================
// KO: 시리얼 명령 (테스트/캘리브레이션/호버학습)
// EN: Serial commands (test/calibration/hover learn)
// ============================================================================
static bool imuStreamEnabled = false;
static uint32_t imuStreamNextMs = 0;
static const uint32_t IMU_STREAM_PERIOD_MS = 50; // 20 Hz

static inline void serialCommands(){
  // KO: 논블로킹 라인 리더 (제어 루프 스톨 방지)
  // EN: Non-blocking line reader (prevents control-loop stalls)
  static char cmdBuf[96];
  static uint8_t cmdLen = 0;

  while(Serial.available()){
    char c = (char)Serial.read();
    if(c == '\r') continue;

    if(c == '\n'){
      cmdBuf[cmdLen] = 0;
      cmdLen = 0;

      String s(cmdBuf);
      s.trim();
      if(s.length()==0) continue;

      if(s == "C"){ startFlowAutoCal(0.50f, 0.80f, 35.0f); continue; }
      if(s == "Y0"){ yawReset(); Serial.println("Yaw reset."); continue; }
      if(s == "1510"){ // start IMU stream
        imuStreamEnabled = true;
        imuStreamNextMs = 0;
        Serial.println("IMU STREAM ON");
        continue;
      }
      if(s == "1511"){ // stop IMU stream
        imuStreamEnabled = false;
        Serial.println("IMU STREAM OFF");
        continue;
      }

      if(s == "T"){ // KO: 이륙(hover 학습) / EN: takeoff (hover learn)
        if(currentMode == MODE_READY) {
          hover_startLearn();
          Serial.println("Takeoff: hover learn started.");
        }
        continue;
      }
      if(s == "L"){
        if(currentMode != MODE_READY) currentMode = MODE_LANDING;
        Serial.println("Landing.");
        continue;
      }
      if(s == "K"){
        currentMode = MODE_EMERGENCY;
        motorsOff();
        Serial.println("KILL MOTORS!");
        continue;
      }

      if(s.startsWith("D")){
        float actual_cm = s.substring(1).toFloat();
        if(actual_cm > 5.0f && actual_cm < 300.0f){
          float avgH = (flowCal_height_n ? flowCal_height_acc/(float)flowCal_height_n : currentAltitude);
          float counts = (float)abs(flowCal_sum_dx);
          if(avgH > 0.10f && counts > 5.0f){
            float actual_m = actual_cm * 0.01f;
            flowK = actual_m / (avgH * counts);
            prefs.putFloat("flowK", flowK);
            Serial.printf("FLOW CAL REFINE: actual=%.1fcm => flowK=%.6f (saved)\n", actual_cm, flowK);
          }
        }
        continue;
      }

      // KO: 배터리 체크 명령 (raw + filtered)
      // EN: Battery check command (raw + filtered)
      if(s == "B"){
        Serial.printf("Battery: %.2fV (filt %.2fV)\n", batteryVoltage, batteryVoltageFilt);
        continue;
      }

      // KO: 튜닝 덤프
      // EN: Tuning dump
      if(s == "PID?" || s == "TUNE?"){
        printTuning(false);
        continue;
      }
      // KO: AF1000X_PID.h 붙여넣기 스니펫 출력
      // EN: Print snippet to paste into AF1000X_PID.h
      if(s == "PIDDEF"){
        printTuning(true);
        continue;
      }

      // KO: 알 수 없는 명령 무시
      // EN: Unknown command -> ignore
      continue;
    }

    // KO: 문자 누적
    // EN: Accumulate characters
    if(cmdLen < (sizeof(cmdBuf)-1)){
      cmdBuf[cmdLen++] = c;
    } else {
      // KO: 오버플로우 -> 버퍼 리셋
      // EN: overflow -> reset buffer
      cmdLen = 0;
    }
  }
}

static inline void imuStreamTick(uint32_t now){
  if(!imuStreamEnabled) return;
  if((int32_t)(now - imuStreamNextMs) < 0) return;
  imuStreamNextMs = now + IMU_STREAM_PERIOD_MS;
  Serial.printf("IMU %.3f %.3f %.3f %.2f\n", currentRoll, currentPitch, currentYaw, batteryVoltageFilt);
}


// ============================================================================
// KO: POST + 캘리브레이션
// EN: POST + calibration
// ============================================================================
static inline bool postSensors(){
  ok_imu  = imu_init_auto();
  ok_baro = spl_init();
  ok_tof  = tof_init();
  ok_flow = pmw_init();

  ledSet(ok_imu, ok_baro, ok_tof, ok_flow);

  // KO: 필수 = Baro + ToF + Flow (IMU는 실패해도 비행 가능하지만 권장)
  // EN: Required = Baro + ToF + Flow (IMU failure allowed but not recommended)
  flightLock = !(ok_baro && ok_tof && ok_flow);

  if(flightLock){
    Serial.printf("POST FAIL -> FLIGHT LOCK (IMU=%d BARO=%d TOF=%d FLOW=%d)\n",
                  ok_imu, ok_baro, ok_tof, ok_flow);
    ledFailPattern();
    return false;
  }
  Serial.printf("POST OK (IMU=%d BARO=%d TOF=%d FLOW=%d)\n",
                ok_imu, ok_baro, ok_tof, ok_flow);
  return true;
}

static inline bool calibrateSensors(){
  // KO: 기압 기준값
  // EN: Baro baseline
  float sumP=0; int got=0;
  for(int i=0;i<200;i++){
    float p;
    if(spl_readPressurePa(p)){ sumP += p; got++; }
    delay(10);
  }
  if(got < 120) return false;
  basePressurePa = sumP / got;

  // KO: 필터 리셋
  // EN: reset filters
  altInit=false;
  baroInit=false;
  tofFiltInit=false;

  // KO: IMU 자이로 바이어스 (옵션)
  // EN: IMU gyro bias (optional)
  if(ok_imu){
    if(!imu_calibrate_gyroZ()){
      ok_imu = false;
    } else {
      tune_saveGyroBias();
    }
  }
  yawReset();

  calibrated = true;
  return true;
}

// ============================================================================
// KO: 공개 API
// EN: Public APIs
// ============================================================================
static inline void calibrate(){
  if(currentMode == MODE_READY){
    Serial.println("AF: calibrate (ground)");
    if(!calibrateSensors()){
      Serial.println("AF: calibrate FAIL -> lock");
      flightLock = true;
      ledFailPattern();
    } else {
      Serial.println("AF: calibrate OK");
    }
  }
}

static inline void autoTakeoff(){
  if(flightLock){
    Serial.println("AF: FLIGHT LOCK - takeoff blocked");
    currentMode = MODE_READY;
    motorsOff();
    return;
  }
  if(currentMode == MODE_READY){
    hover_startLearn(); // KO: A 방식 시작 / EN: start A method
  }
}

static inline void autoLanding(){
  if(currentMode != MODE_READY){
    currentMode = MODE_LANDING;
  }
}

static inline void emergencyStop(){
  currentMode = MODE_EMERGENCY;
  motorsOff();
  motorArmedIdle = false;
  armAwaitSecond = false;
  armHighPrev = false;
  armPulsing = false;
  autoTakeoffPending = false;
}

// ============================================================================
// KO: updateSystem / updateFlight
// EN: updateSystem / updateFlight
// ============================================================================
static inline void updateSystem(){
  // KO: 바인딩 상태 우선 업데이트
  // EN: Always update binding state first
  binding_update();
  g_ledBound = binding_isBound();

  static bool lastBound = false;
  if(g_ledBound && !lastBound){
    fhssInit();
  }
  lastBound = g_ledBound;

  static bool lastLink = false;
  if(linkReady && !lastLink){
    fhssInit();
  }
  lastLink = linkReady;

  // KO: 배터리 체크 (10Hz, 바인딩 전에도)
  // EN: Battery check (10Hz) even before binding completes
  static uint32_t lastBatMs = 0;
  uint32_t nowBatMs = millis();
  if(nowBatMs - lastBatMs >= 100){
    lastBatMs = nowBatMs;
    checkBattery();
  }
  g_ledLowBattery = (batteryLowWarning || batteryCritical);
  bool batteryOkForArm = (batteryVoltageFilt >= BATTERY_ARM_MIN_VOLTAGE);

  // KO: 헤드리스 플래그 (최신 RX 데이터 이후 갱신)
  // EN: Headless flag (updated after we have fresh RX data)
  g_ledHeadless = false;
  g_ledAutoTune = tune_isActive();

  // KO: 부팅/바인딩 중에도 LED 구동
  // EN: Drive LEDs during boot/binding too
  ledTick();

  if (!g_ledBound || !linkReady) {
    // KO: 미바운드/링크 미준비 -> 제어/비행 금지
    // EN: Not bound or link not ready -> no control / no flight
    currentMode = MODE_READY;
    motorsOff();
    motorArmedIdle = false;
    armAwaitSecond = false;
    armHighPrev = false;
    armPulsing = false;
    autoTakeoffPending = false;
    return;
  }
  updateRadio();

  // KO: AUX1=2 자이로 초기화 요청 (READY + 스로틀 낮음)
  // EN: Gyro init request via AUX1=2 (READY + low throttle)
  {
    static uint32_t gyroReqStart = 0;
    static bool gyroReqLatched = false;
    if(receiverData.aux1 == 2 && currentMode == MODE_READY && receiverData.throttle < 20){
      if(gyroReqStart == 0) gyroReqStart = millis();
      if(!gyroReqLatched && (millis() - gyroReqStart >= 200)){
        if(imu_calibrate_gyroZ()){
          tune_saveGyroBias();
        }
        yawReset();
        ledStartGyroResetAnim();
        gyroReqLatched = true;
      }
    } else {
      gyroReqStart = 0;
      gyroReqLatched = false;
    }
  }

  // KO: 헤드리스/플로우 (TX aux2 비트)
  // EN: Headless/flow (TX aux2 bits)
  g_ledHeadless = (receiverData.aux2 & AUX2_HEADLESS) != 0;
  flowUserEnabled = (receiverData.aux2 & AUX2_FLOW) != 0;

  serialCommands();  

  if(ok_imu) imu_updateYaw();
  else currentYaw = 0.0f;
  imuStreamTick(millis());

  // KO: 모터 시동(아이들 스핀) - 좌 스틱 더블업
  // EN: Motor arm (idle spin) - left stick double-up within window
  {
    if(currentMode != MODE_READY || flightLock || failsafe || !batteryOkForArm){
      motorArmedIdle = false;
      armAwaitSecond = false;
      armHighPrev = false;
      armPulsing = false;
      armDisarmStartMs = 0;
      autoTakeoffPending = false;
    } else {
      bool high = receiverData.throttle >= ARM_THROTTLE_HIGH;

      if(armAwaitSecond && (millis() - armFirstUpMs > ARM_WINDOW_MS)){
        armAwaitSecond = false;
      }

      if(high && !armHighPrev){
        if(armAwaitSecond){
          motorArmedIdle = true;
          armAwaitSecond = false;
          armPulsing = true;
          armPulseStartMs = millis();
        } else {
          armAwaitSecond = true;
          armFirstUpMs = millis();
        }
      }

      armHighPrev = high;

      // KO: 스로틀 하단 홀드 시 시동 해제
      // EN: Disarm if throttle fully low for hold time
      if(motorArmedIdle && receiverData.throttle <= ARM_THROTTLE_LOW){
        if(armDisarmStartMs == 0) armDisarmStartMs = millis();
        if(millis() - armDisarmStartMs >= ARM_DISARM_HOLD_MS){
          motorArmedIdle = false;
          armPulsing = false;
          autoTakeoffPending = false;
        }
      } else {
        armDisarmStartMs = 0;
      }
    }
  }

  // KO: 자동 이착륙 요청 (AUX1 펄스)
  // EN: Auto takeoff / landing request (AUX1 pulse)
  {
    static bool aux1Prev = false;
    bool aux1Now = (receiverData.aux1 == 1);
    if(aux1Now && !aux1Prev){
  if(currentMode == MODE_READY && !flightLock && !failsafe && batteryOkForArm){
        motorArmedIdle = true;
        armPulsing = true;
        armPulseStartMs = millis();
        autoTakeoffPending = true;
        autoTakeoffAtMs = millis() + AUTO_TAKEOFF_DELAY_MS;
        targetAltitude = currentAltitude;
        targetYaw = currentYaw;
        targetPosX = currentPosX;
        targetPosY = currentPosY;
      } else if(currentMode == MODE_TAKEOFF || currentMode == MODE_HOVERING){
        autoTakeoffPending = false;
        motorArmedIdle = false;
        armPulsing = false;
        autoLanding();
      }
    }
    aux1Prev = aux1Now;
  }

  if(autoTakeoffPending && currentMode == MODE_READY && !flightLock && !failsafe && batteryOkForArm){
    if(millis() >= autoTakeoffAtMs){
      autoTakeoffPending = false;
      motorArmedIdle = false;
      armPulsing = false;
      targetAltitude = AUTO_TAKEOFF_ALT_M;
      targetYaw = currentYaw;
      targetPosX = currentPosX;
      targetPosY = currentPosY;
      currentMode = MODE_TAKEOFF;
    }
  }

  // KO: 이륙 시작 (아이들 시동 상태 + 스로틀 상승)
  // EN: Takeoff start: if armed idle and throttle pushed up
  if(motorArmedIdle && currentMode == MODE_READY && !flightLock && !failsafe && batteryOkForArm){
    if(receiverData.throttle >= ARM_TAKEOFF_THRESH){
      motorArmedIdle = false;
      armPulsing = false;
      autoTakeoffPending = false;
      targetAltitude = currentAltitude;
      targetYaw = currentYaw;
      currentMode = MODE_TAKEOFF;
    }
  }

  // KO: 스로틀 하강 -> TAKEOFF에서 READY로 복귀
  // EN: Throttle down -> TAKEOFF back to READY
  if(currentMode == MODE_TAKEOFF && receiverData.throttle <= ARM_THROTTLE_LOW){
    currentMode = MODE_READY;
    motorsOff();
    motorArmedIdle = false;
    armPulsing = false;
    autoTakeoffPending = false;
  }

  // KO: 틸트 킬 (롤/피치 임계값 초과 시 긴급 정지)
  // EN: Tilt kill: emergency stop if roll/pitch exceeds threshold
  {
    static uint32_t tiltStartMs = 0;
    if(ok_imu && currentMode != MODE_READY && currentMode != MODE_EMERGENCY){
      float absRoll = fabsf(currentRoll);
      float absPitch = fabsf(currentPitch);
      if(absRoll > TILT_KILL_DEG || absPitch > TILT_KILL_DEG){
        if(tiltStartMs == 0) tiltStartMs = millis();
        if(millis() - tiltStartMs >= TILT_KILL_HOLD_MS){
          emergencyStop();
          tiltStartMs = 0;
        }
      } else {
        tiltStartMs = 0;
      }
    } else {
      tiltStartMs = 0;
    }
  }

  updateAltitudeFusion();
  updatePositionFromFlow();
  flowCalTick();

  // KO: Hover 상태머신 tick (A 방식)
  // EN: Hover state machine tick (A method)
  hover_update();

  // KO: Hover 준비 후 오토튠 (ALT PD + YAW P)
  // EN: Auto tune after hover ready (ALT PD + YAW P)
  tune_update();
  g_ledAutoTune = tune_isActive();

  // ==========================================================================
  // KO: 스틱 입력 처리 (중복 제거)
  // EN: Stick input handling (duplicate removed)
  // ==========================================================================
  if ((currentMode == MODE_HOVERING || currentMode == MODE_TAKEOFF) && !failsafe && !tune_isActive()) {
    // KO: 1) 데드존 설정 (중앙 127 기준 ±10 무시)
    // EN: 1) Deadzone (ignore ±10 around 127)
    int rawRoll = receiverData.roll;
    int rawPitch = receiverData.pitch;
    int rawThr = receiverData.throttle;
    int rawYaw = receiverData.yaw;

    float vX_cmd = 0.0f; // KO: 좌우 이동 속도 / EN: lateral velocity cmd
    float vY_cmd = 0.0f; // KO: 전후 이동 속도 / EN: forward velocity cmd

    // KO: Roll (좌우)
    // EN: Roll (lateral)
    if (abs(rawRoll - 127) > 10) {
      float input = (float)(rawRoll - 127) / 127.0f;
      vX_cmd = input * moveSpeedSteps[speedLevel]; 
    }

    // KO: Pitch (전후)
    // EN: Pitch (forward/back)
    if (abs(rawPitch - 127) > 10) {
      float input = (float)(rawPitch - 127) / 127.0f;
      vY_cmd = input * moveSpeedSteps[speedLevel];
    }

    // KO: Yaw (회전)
    // EN: Yaw (rotation)
    if (abs(rawYaw - 127) > 10) {
      float input = (float)(rawYaw - 127) / 127.0f;
      targetYaw += input * yawSpeedSteps[speedLevel] * g_loopDt;
      // KO: 각도 래핑
      // EN: Wrap angle
      while(targetYaw > 180.0f) targetYaw -= 360.0f;
      while(targetYaw < -180.0f) targetYaw += 360.0f;
    }

    // KO: Throttle (고도 미세 조정)
    // EN: Throttle (altitude trim)
    if (abs(rawThr - 127) > 20) {
      float input = (float)(rawThr - 127) / 127.0f;
      targetAltitude += input * climbSpeedSteps[speedLevel] * g_loopDt;
      // KO: 고도 안전 제한 (0.1m ~ 2.5m)
      // EN: Altitude limits (0.1m ~ 2.5m)
      if (targetAltitude < 0.1f) targetAltitude = 0.1f;
      if (targetAltitude > 2.5f) targetAltitude = 2.5f;
    }

    // KO: 2) 좌표계 변환 (드론 헤딩 기준)
    // EN: 2) Frame transform (drone heading)
    if (vX_cmd != 0.0f || vY_cmd != 0.0f) {
      float rad = currentYaw * DEG_TO_RAD;
      float c = cosf(rad);
      float s = sinf(rad);

      float dX_world = (vY_cmd * c - vX_cmd * s) * g_loopDt; 
      float dY_world = (vY_cmd * s + vX_cmd * c) * g_loopDt;

      // KO: 안전 제한
      // EN: Safety clamp
      const float MAX_TARGET_STEP = 0.12f * g_loopDt;
      dX_world = clampf(dX_world, -MAX_TARGET_STEP, MAX_TARGET_STEP);
      dY_world = clampf(dY_world, -MAX_TARGET_STEP, MAX_TARGET_STEP);

      targetPosX += dX_world;
      targetPosY += dY_world;
    }
  }
  // ============================================================


  // KO: LED 업데이트
  // EN: LED update
  ledSet(ok_imu, ok_baro, ok_tof, ok_flow && flowUserEnabled);
  // KO: 최종 LED tick (상태 업데이트 후)
  // EN: Final LED tick (after state updates)
  ledTick();

}

static inline void updateFlight(){
  if(currentMode == MODE_READY || currentMode == MODE_EMERGENCY){
    if(currentMode == MODE_READY && motorArmedIdle){
      uint32_t now = millis();
      int pwm = ARM_IDLE_PWM_LOW;
      if(armPulsing){
        uint32_t dt = now - armPulseStartMs;
        if(dt < ARM_PULSE_MS){
          pwm = ARM_IDLE_PWM_LOW;
        } else if(dt < (ARM_PULSE_MS * 2)){
          pwm = ARM_IDLE_PWM_HIGH;
        } else if(dt < (ARM_PULSE_MS * 3)){
          pwm = ARM_IDLE_PWM_LOW;
        } else {
          armPulsing = false;
          pwm = ARM_IDLE_PWM_LOW;
        }
      }
      motorsIdle(pwm);
      return;
    }
    motorsOff();
    return;
  }

  // KO: TAKEOFF -> HOVERING 전환은 hover header가 안정 시 수행
  // EN: TAKEOFF -> HOVERING handled by hover header when stable
  // KO: LANDING
  // EN: LANDING
  if(currentMode == MODE_LANDING){
    targetAltitude -= LAND_DESCEND_MPS * g_loopDt;
    targetAltitude = clampf(targetAltitude, 0.0f, 3.0f);
    if(currentAltitude < READY_ALT_M){
      currentMode = MODE_READY;
      motorsOff();
      return;
    }
  }

  // KO: Position P
  // EN: Position P
  float ex = (targetPosX - currentPosX);
  float ey = (targetPosY - currentPosY);
  float r_cmd = ex * POS_KP;
  float p_cmd = ey * POS_KP;

  // KO: Yaw P
  // EN: Yaw P
  float eyaw = (targetYaw - currentYaw);
  while(eyaw > 180.0f) eyaw -= 360.0f;
  while(eyaw < -180.0f) eyaw += 360.0f;
  float y_cmd = eyaw * yawKp;

  // KO: Altitude PD
  // EN: Altitude PD
  float zErr = (targetAltitude - currentAltitude);
  float zVel = (alt_m_filt - alt_m_prev) / g_loopDt;
  float zCmd = (zErr * altKp) - (zVel * altKd);

  // KO: Hover 학습이 hoverThrottle을 자동 보정
  // EN: Hover learning auto-adjusts hoverThrottle
  hover_onZcmd(zCmd);

  int thr = hoverThrottle + (int)(zCmd * 100.0f);
  thr = constrain(thr, 0, 255);

  motorControl(thr, r_cmd, p_cmd, y_cmd);
}

// ============================================================================
// KO: 초기화
// EN: init
// ============================================================================
static inline void initAF1000X(){
  Serial.println("AF1000X: init (FIXED VERSION)");

  ledsInit();
  motorsInit();

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);

  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

  radioInit();

  prefs.begin("af1000x", false);
  flowK = prefs.getFloat("flowK", flowK);
  Serial.printf("flowK=%.6f (loaded)\n", flowK);
  tune_loadFromNVS();
  tune_saveFilterParamsIfMissing();

  // KO: POST + 캘리브레이션
  // EN: POST + calibration
  postSensors();
  if(!flightLock){
    if(!calibrateSensors()){
      flightLock = true;
      ledFailPattern();
    }
  }

  
  // ============================================================================
  // KO: LED 부팅 시퀀스 초기화 (전체 점멸 또는 inverted chase)
  // EN: LED boot sequence init (blink-all or inverted chase)
  // ============================================================================
  g_ledBound = false;
  g_ledHeadless = false;
  g_ledLowBattery = false;

  bool invertedAtBoot = false;
  if(ok_imu && IMU != nullptr){
    inv_imu_sensor_data_t d{};
    if(IMU->getDataFromRegisters(d) == 0){
      float az = (float)d.accel_data[2];
      invertedAtBoot = (az < 0.0f);
    }
  }
  ledBootBegin(invertedAtBoot);

binding_begin();
  fhssInit();

  hover_begin(); // KO: hoverPWM 로드 / EN: load hoverPWM

  currentMode = MODE_READY;
  Serial.printf("AF1000X: READY (lock=%d)\n", (int)flightLock);
  Serial.println("✅ Fixed: Duplicate stick input removed");
  Serial.println("✅ Added: Battery protection");
}

#endif // KO: AF1000X_H / EN: AF1000X_H
