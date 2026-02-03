#ifndef AF1000X_EASY_COMMANDER_H
#define AF1000X_EASY_COMMANDER_H

#include <Arduino.h>

/* ============================================================================
 * KO: EasyCommander (AF1000X 코어 호환)
 * EN: EasyCommander (AF1000X core compatible)
 * KO: currentMode 기반, 모드 상수는 코어와 동일해야 함
 * EN: Uses currentMode; mode constants must match core
 * ============================================================================
 */

// ============================================================================
// KO: 코어 extern (AF1000X_CORE.h 제공)
// EN: Core externs (from AF1000X_CORE.h)
// ============================================================================
extern volatile uint8_t currentMode;    // KO: 코어 상태 / EN: core state
extern bool flightLock;                 // KO: 센서 실패 비행 금지 / EN: flight lock

extern float targetAltitude, currentAltitude;
extern float targetPosX, targetPosY, targetYaw;
extern float currentPosX, currentPosY, currentYaw;

extern float moveSpeedSteps[3];
extern float climbSpeedSteps[3];
extern float yawSpeedSteps[3];
extern int   speedLevel;

extern void updateSystem();
extern void updateFlight();

extern void autoTakeoff();
extern void autoLanding();
extern void emergencyStop();

// ============================================================================
// KO: MODE_* 값은 코어와 동일해야 함
// EN: MODE_* values must match core
// ============================================================================
#ifndef READY
  #define READY      0
  #define TAKEOFF    1
  #define HOVERING   2
  #define LANDING    3
  #define EMERGENCY  4
#endif

// ============================================================================
// KO: 내부 유틸 (제어 루프를 멈추지 않는 대기)
// EN: Internal util (wait without stopping control loop)
// ============================================================================
static inline void _airgo_yield(unsigned long ms = 0) {
  if (ms == 0) {
    updateSystem();
    updateFlight();
    delay(20);
    return;
  }

  unsigned long start = millis();
  while (millis() - start < ms) {
    updateSystem();
    updateFlight();
    delay(20);
    if (currentMode == EMERGENCY) return;
  }
}

// ============================================================================
// KO: 안전 가드
// EN: Safety guard
// ============================================================================
static inline bool _airgo_readyToFly() {
  if (flightLock) return false;
  if (currentMode == EMERGENCY) return false;
  return true;
}

// ============================================================================
// KO: 1) 기본 비행
// EN: 1) Core flight
// ============================================================================
static inline void takeoff() {
  if (!_airgo_readyToFly()) return;
  if (currentMode != READY) return;

  Serial.println("[CMD] Takeoff");
  autoTakeoff();                // KO: hover 학습 takeoff / EN: hover-learn takeoff

  // KO: TAKEOFF 종료까지 대기
  // EN: Wait until TAKEOFF completes
  while (currentMode == TAKEOFF) _airgo_yield(0);

  // KO: HOVERING 진입 실패 시 종료
  // EN: Exit if not HOVERING
}

static inline void land() {
  if (currentMode == READY) return;

  Serial.println("[CMD] Land");
  autoLanding();

  while (currentMode == LANDING) _airgo_yield(0);
}

static inline void stay(float sec) {
  if (sec <= 0) return;
  Serial.printf("[CMD] Stay %.2f sec\n", sec);
  _airgo_yield((unsigned long)(sec * 1000.0f));
}

static inline void killMotors() {
  Serial.println("[CMD] KILL MOTORS!");
  emergencyStop();
}

// ============================================================================
// KO: 2) CM 이동 명령 (정확도는 센서/융합 성능에 의존)
// EN: 2) CM moves (accuracy depends on sensors/fusion)
// ============================================================================
static inline void forwardCm(float cm, float pwr) {
  if (currentMode != HOVERING) return;
  if (cm <= 0) return;

  float p = constrain(pwr, 1.0f, 100.0f) / 100.0f;
  float dM = cm * 0.01f;

  Serial.printf("[CMD] Forward %.1f cm (pwr=%.0f%%)\n", cm, pwr);

  // KO: 헤딩(yaw) 기준 전진
  // EN: Forward in heading frame
  float rad = targetYaw * DEG_TO_RAD;
  float moved = 0.0f;

  while (moved < dM) {
    float step = (moveSpeedSteps[speedLevel] * p) * 0.02f;  // KO: 50Hz 기준 / EN: 50Hz base
    if (step < 0.001f) step = 0.001f;

    targetPosX += step * cosf(rad);
    targetPosY += step * sinf(rad);

    moved += step;
    _airgo_yield(0);
    if (currentMode == EMERGENCY) break;
  }
}

static inline void backwardCm(float cm, float pwr) {
  if (currentMode != HOVERING) return;
  if (cm <= 0) return;

  float p = constrain(pwr, 1.0f, 100.0f) / 100.0f;
  float dM = cm * 0.01f;

  Serial.printf("[CMD] Backward %.1f cm (pwr=%.0f%%)\n", cm, pwr);

  float rad = targetYaw * DEG_TO_RAD;
  float moved = 0.0f;

  while (moved < dM) {
    float step = (moveSpeedSteps[speedLevel] * p) * 0.02f;
    if (step < 0.001f) step = 0.001f;

    targetPosX -= step * cosf(rad);
    targetPosY -= step * sinf(rad);

    moved += step;
    _airgo_yield(0);
    if (currentMode == EMERGENCY) break;
  }
}

static inline void rightCm(float cm, float pwr) {
  if (currentMode != HOVERING) return;
  if (cm <= 0) return;

  float p = constrain(pwr, 1.0f, 100.0f) / 100.0f;
  float dM = cm * 0.01f;

  Serial.printf("[CMD] Right %.1f cm (pwr=%.0f%%)\n", cm, pwr);

  float rad = (targetYaw + 90.0f) * DEG_TO_RAD; // KO: 오른쪽 / EN: right
  float moved = 0.0f;

  while (moved < dM) {
    float step = (moveSpeedSteps[speedLevel] * p) * 0.02f;
    if (step < 0.001f) step = 0.001f;

    targetPosX += step * cosf(rad);
    targetPosY += step * sinf(rad);

    moved += step;
    _airgo_yield(0);
    if (currentMode == EMERGENCY) break;
  }
}

static inline void leftCm(float cm, float pwr) {
  if (currentMode != HOVERING) return;
  if (cm <= 0) return;

  float p = constrain(pwr, 1.0f, 100.0f) / 100.0f;
  float dM = cm * 0.01f;

  Serial.printf("[CMD] Left %.1f cm (pwr=%.0f%%)\n", cm, pwr);

  float rad = (targetYaw - 90.0f) * DEG_TO_RAD; // KO: 왼쪽 / EN: left
  float moved = 0.0f;

  while (moved < dM) {
    float step = (moveSpeedSteps[speedLevel] * p) * 0.02f;
    if (step < 0.001f) step = 0.001f;

    targetPosX += step * cosf(rad);
    targetPosY += step * sinf(rad);

    moved += step;
    _airgo_yield(0);
    if (currentMode == EMERGENCY) break;
  }
}

static inline void upCm(float cm, float pwr) {
  if (currentMode != HOVERING) return;
  if (cm <= 0) return;

  float p = constrain(pwr, 1.0f, 100.0f) / 100.0f;
  float dM = cm * 0.01f;

  Serial.printf("[CMD] Up %.1f cm (pwr=%.0f%%)\n", cm, pwr);

  float startAlt = targetAltitude;
  while (targetAltitude < startAlt + dM) {
    targetAltitude += (climbSpeedSteps[speedLevel] * p) * 0.02f;
    _airgo_yield(0);
    if (currentMode == EMERGENCY) break;
  }
}

static inline void downCm(float cm, float pwr) {
  if (currentMode != HOVERING) return;
  if (cm <= 0) return;

  float p = constrain(pwr, 1.0f, 100.0f) / 100.0f;
  float dM = cm * 0.01f;

  Serial.printf("[CMD] Down %.1f cm (pwr=%.0f%%)\n", cm, pwr);

  float startAlt = targetAltitude;
  while (targetAltitude > startAlt - dM) {
    targetAltitude -= (climbSpeedSteps[speedLevel] * p) * 0.02f;
    if (targetAltitude < 0.1f) targetAltitude = 0.1f;
    _airgo_yield(0);
    if (currentMode == EMERGENCY) break;
  }
}

// ============================================================================
// KO: 3) 회전
// EN: 3) Rotation
// ============================================================================
static inline void turnAngle(float deg, float pwr) {
  if (currentMode != HOVERING) return;
  if (deg == 0) return;

  float p = constrain(pwr, 1.0f, 100.0f) / 100.0f;

  Serial.printf("[CMD] TurnAngle %.1f deg (pwr=%.0f%%)\n", deg, pwr);

  float start = targetYaw;
  float goal  = start + deg;

  // KO: 각도 래핑
  // EN: wrap
  while (goal > 180.0f) goal -= 360.0f;
  while (goal < -180.0f) goal += 360.0f;

  float dir = (deg > 0) ? 1.0f : -1.0f;

  while (fabsf(targetYaw - goal) > 1.0f) {
    targetYaw += dir * (yawSpeedSteps[speedLevel] * p) * 0.02f;

    // KO: 각도 래핑
    // EN: wrap
    while (targetYaw > 180.0f) targetYaw -= 360.0f;
    while (targetYaw < -180.0f) targetYaw += 360.0f;

    _airgo_yield(0);
    if (currentMode == EMERGENCY) break;
  }

  targetYaw = goal;
  _airgo_yield(200);
}

static inline void turnTime(float sec, float pwr) {
  if (currentMode != HOVERING) return;
  if (sec <= 0) return;

  float p = constrain(pwr, 1.0f, 100.0f) / 100.0f;

  Serial.printf("[CMD] TurnTime %.2f sec (pwr=%.0f%%)\n", sec, pwr);

  unsigned long ms = (unsigned long)(sec * 1000.0f);
  unsigned long start = millis();

  // KO: 지정 시간 동안 한 방향 회전 예시
  // EN: Example: rotate in one direction for duration
  while (millis() - start < ms) {
    targetYaw += (yawSpeedSteps[speedLevel] * p) * 0.02f;
    while (targetYaw > 180.0f) targetYaw -= 360.0f;
    while (targetYaw < -180.0f) targetYaw += 360.0f;

    _airgo_yield(0);
    if (currentMode == EMERGENCY) break;
  }
}

#endif // KO: AF1000X_EASY_COMMANDER_H / EN: AF1000X_EASY_COMMANDER_H
