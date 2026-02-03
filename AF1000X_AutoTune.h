#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include "AF1000X_PID.h"

// ============================================================================
// KO: AF1000X 오토튠 (ALT PD + YAW P)
// EN: AF1000X auto tune (ALT PD + YAW P)
// KO: Hover 학습 완료 후 1회 자동 실행, 결과 NVS 저장
// EN: Runs once after hover learning, stores results to NVS
// ============================================================================

// ============================================================================
// KO: Core extern
// EN: Externs (from core)
// ============================================================================
extern volatile uint8_t currentMode;
extern bool flightLock;
extern bool failsafe;
extern float currentAltitude;
extern float targetAltitude;
extern float currentYaw;
extern float targetYaw;
extern float altKp, altKd, yawKp;
extern float attTau, accZMin, tofEmaA, baroEmaA, altEmaA, tofJumpRejectM;
extern float gyroX_bias, gyroY_bias, gyroZ_bias;
extern float flowK;
extern int hoverThrottle;
extern Preferences prefs;

// ============================================================================
// KO: Hover 상태 헬퍼 (AF1000X_Hover.h inline)
// EN: Hover status helpers (inline in AF1000X_Hover.h)
// ============================================================================
static inline bool hover_isReady();
static inline bool hover_isLearning();
static inline void hover_abort();

// ============================================================================
// KO: NVS 키
// EN: NVS keys
// ============================================================================
static constexpr const char* KEY_ALT_KP    = "altKp";
static constexpr const char* KEY_ALT_KD    = "altKd";
static constexpr const char* KEY_YAW_KP    = "yawKp";
static constexpr const char* KEY_TUNE_DONE = "tuneDone";
static constexpr const char* KEY_FLOW_K    = "flowK";
static constexpr const char* KEY_GX_BIAS   = "gxb";
static constexpr const char* KEY_GY_BIAS   = "gyb";
static constexpr const char* KEY_GZ_BIAS   = "gzb";
static constexpr const char* KEY_ATT_TAU   = "attTau";
static constexpr const char* KEY_ACC_ZMIN  = "accZMin";
static constexpr const char* KEY_TOF_EMA   = "tofA";
static constexpr const char* KEY_BARO_EMA  = "baroA";
static constexpr const char* KEY_ALT_EMA   = "altA";
static constexpr const char* KEY_TOF_JUMP  = "tofJ";

enum AutoTuneState : uint8_t {
  AT_IDLE = 0,
  AT_ALT_UP,
  AT_ALT_DOWN,
  AT_YAW_STEP,
  AT_YAW_BACK,
  AT_DONE,
  AT_ABORT
};

static AutoTuneState at_state = AT_IDLE;
static bool at_active = false;
static bool at_done = false;
static uint32_t at_t0 = 0;

static float at_baseAlt = 0.0f;
static float at_stepAlt = 0.0f;
static float at_maxAlt = 0.0f;
static bool  at_riseRecorded = false;
static uint32_t at_riseMs = 0;

static float at_baseYaw = 0.0f;
static float at_maxYaw = 0.0f;
static bool  at_yawRiseRecorded = false;
static uint32_t at_yawRiseMs = 0;

static inline float _clampf(float v, float lo, float hi){
  if(v < lo) return lo;
  if(v > hi) return hi;
  return v;
}

static inline float _wrapDeg(float d){
  while(d > 180.0f) d -= 360.0f;
  while(d < -180.0f) d += 360.0f;
  return d;
}

static inline void tune_loadFromNVS(){
  altKp = prefs.getFloat(KEY_ALT_KP, ALT_KP_DEFAULT);
  altKd = prefs.getFloat(KEY_ALT_KD, ALT_KD_DEFAULT);
  yawKp = prefs.getFloat(KEY_YAW_KP, YAW_KP_DEFAULT);
  at_done = prefs.getBool(KEY_TUNE_DONE, false);

  // KO: 필터 파라미터
  // EN: Filter params
  attTau = prefs.getFloat(KEY_ATT_TAU, ATT_TAU);
  accZMin = prefs.getFloat(KEY_ACC_ZMIN, ACC_Z_MIN);
  tofEmaA = prefs.getFloat(KEY_TOF_EMA, TOF_EMA_A);
  baroEmaA = prefs.getFloat(KEY_BARO_EMA, BARO_EMA_A);
  altEmaA = prefs.getFloat(KEY_ALT_EMA, ALT_EMA_A);
  tofJumpRejectM = prefs.getFloat(KEY_TOF_JUMP, TOF_JUMP_REJECT_M);

  // KO: 자이로 바이어스 (마지막 캘리브레이션 저장값)
  // EN: Gyro bias (stored from last calibration)
  gyroX_bias = prefs.getFloat(KEY_GX_BIAS, gyroX_bias);
  gyroY_bias = prefs.getFloat(KEY_GY_BIAS, gyroY_bias);
  gyroZ_bias = prefs.getFloat(KEY_GZ_BIAS, gyroZ_bias);
}

static inline void tune_saveToNVS(){
  prefs.putFloat(KEY_ALT_KP, altKp);
  prefs.putFloat(KEY_ALT_KD, altKd);
  prefs.putFloat(KEY_YAW_KP, yawKp);
  prefs.putBool(KEY_TUNE_DONE, true);
  at_done = true;
}

static inline void tune_saveFilterParamsIfMissing(){
  bool missing =
    !prefs.isKey(KEY_ATT_TAU) ||
    !prefs.isKey(KEY_ACC_ZMIN) ||
    !prefs.isKey(KEY_TOF_EMA) ||
    !prefs.isKey(KEY_BARO_EMA) ||
    !prefs.isKey(KEY_ALT_EMA) ||
    !prefs.isKey(KEY_TOF_JUMP);

  if(missing){
    prefs.putFloat(KEY_ATT_TAU, attTau);
    prefs.putFloat(KEY_ACC_ZMIN, accZMin);
    prefs.putFloat(KEY_TOF_EMA, tofEmaA);
    prefs.putFloat(KEY_BARO_EMA, baroEmaA);
    prefs.putFloat(KEY_ALT_EMA, altEmaA);
    prefs.putFloat(KEY_TOF_JUMP, tofJumpRejectM);
  }
}

static inline void tune_saveGyroBias(){
  prefs.putFloat(KEY_GX_BIAS, gyroX_bias);
  prefs.putFloat(KEY_GY_BIAS, gyroY_bias);
  prefs.putFloat(KEY_GZ_BIAS, gyroZ_bias);
}

static inline void tune_factoryReset(){
  prefs.remove(KEY_ALT_KP);
  prefs.remove(KEY_ALT_KD);
  prefs.remove(KEY_YAW_KP);
  prefs.remove(KEY_TUNE_DONE);
  prefs.remove("hoverPWM");
  prefs.remove(KEY_FLOW_K);
  prefs.remove(KEY_GX_BIAS);
  prefs.remove(KEY_GY_BIAS);
  prefs.remove(KEY_GZ_BIAS);
  prefs.remove(KEY_ATT_TAU);
  prefs.remove(KEY_ACC_ZMIN);
  prefs.remove(KEY_TOF_EMA);
  prefs.remove(KEY_BARO_EMA);
  prefs.remove(KEY_ALT_EMA);
  prefs.remove(KEY_TOF_JUMP);

  altKp = ALT_KP_DEFAULT;
  altKd = ALT_KD_DEFAULT;
  yawKp = YAW_KP_DEFAULT;

  hoverThrottle = HOVER_PWM_DEFAULT;
  flowK = FLOW_K_DEFAULT;

  gyroX_bias = 0.0f;
  gyroY_bias = 0.0f;
  gyroZ_bias = 0.0f;
  attTau = ATT_TAU;
  accZMin = ACC_Z_MIN;
  tofEmaA = TOF_EMA_A;
  baroEmaA = BARO_EMA_A;
  altEmaA = ALT_EMA_A;
  tofJumpRejectM = TOF_JUMP_REJECT_M;

  at_done = false;
  at_active = false;
  at_state = AT_IDLE;
}

static inline bool tune_isActive(){ return at_active; }
static inline bool tune_isDone(){ return at_done; }

static inline void tune_start(){
  at_active = true;
  at_state = AT_ALT_UP;
  at_t0 = millis();

  at_baseAlt = currentAltitude;
  float step = ATUNE_ALT_STEP_M;
  if(at_baseAlt + step > ATUNE_MAX_ALT_M){
    step = ATUNE_MAX_ALT_M - at_baseAlt;
  }
  if(step < 0.05f) step = 0.05f;
  at_stepAlt = step;
  targetAltitude = at_baseAlt + at_stepAlt;

  at_maxAlt = currentAltitude;
  at_riseRecorded = false;
  at_riseMs = 0;

  at_baseYaw = currentYaw;
  at_maxYaw = currentYaw;
  at_yawRiseRecorded = false;
  at_yawRiseMs = 0;
}

static inline void tune_abort(){
  at_active = false;
  at_state = AT_ABORT;
  at_done = false;
  // KO: 현재 게인 유지, 다음 비행에서 재시도
  // EN: keep current gains, retry next flight
}

static inline void tune_update(){
  if(at_done) return;

  // KO: Hover 준비 + 안정 모드 대기
  // EN: Wait for hover ready and stable mode
  if(!at_active){
    if(hover_isReady() && currentMode == 2 /*HOVERING*/ && !flightLock && !failsafe){
      tune_start();
    }
    return;
  }

  if(flightLock || failsafe || currentMode != 2 /*HOVERING*/){
    tune_abort();
    return;
  }

  uint32_t now = millis();

  switch(at_state){
    case AT_ALT_UP: {
      if(currentAltitude > at_maxAlt) at_maxAlt = currentAltitude;

      float target = at_baseAlt + at_stepAlt;
      float riseThresh = at_baseAlt + (at_stepAlt * 0.9f);
      if(!at_riseRecorded && currentAltitude >= riseThresh){
        at_riseRecorded = true;
        at_riseMs = now - at_t0;
      }

      if(now - at_t0 >= ATUNE_ALT_UP_MS){
        targetAltitude = at_baseAlt;
        at_state = AT_ALT_DOWN;
        at_t0 = now;
      } else {
        targetAltitude = target;
      }
    } break;

    case AT_ALT_DOWN: {
      if(now - at_t0 >= ATUNE_ALT_DOWN_MS){
        // KO: ALT 응답 평가
        // EN: Evaluate ALT response
        float overshoot = at_maxAlt - (at_baseAlt + at_stepAlt);
        if(overshoot < 0.0f) overshoot = 0.0f;
        uint32_t riseMs = at_riseRecorded ? at_riseMs : ATUNE_ALT_UP_MS;

        // KO: 휴리스틱 보정
        // EN: Heuristic adjustments
        if(riseMs > 2000){
          altKp *= 1.15f;
        } else if(riseMs < 900){
          altKp *= 0.92f;
        }

        if(overshoot > 0.08f){
          altKd *= 1.25f;
          altKp *= 0.90f;
        } else if(overshoot < 0.02f){
          altKd *= 0.92f;
        }

        altKp = _clampf(altKp, ALT_KP_MIN, ALT_KP_MAX);
        altKd = _clampf(altKd, ALT_KD_MIN, ALT_KD_MAX);

        at_state = AT_YAW_STEP;
        at_t0 = now;
        at_baseYaw = currentYaw;
        at_maxYaw = currentYaw;
        at_yawRiseRecorded = false;
        at_yawRiseMs = 0;
        targetYaw = _wrapDeg(at_baseYaw + ATUNE_YAW_STEP_DEG);
      }
    } break;

    case AT_YAW_STEP: {
      float dy = _wrapDeg(currentYaw - at_baseYaw);
      if(fabsf(dy) > fabsf(at_maxYaw - at_baseYaw)) at_maxYaw = currentYaw;
      float riseThresh = ATUNE_YAW_STEP_DEG * 0.9f;
      if(!at_yawRiseRecorded && fabsf(dy) >= fabsf(riseThresh)){
        at_yawRiseRecorded = true;
        at_yawRiseMs = now - at_t0;
      }
      if(now - at_t0 >= (ATUNE_YAW_PHASE_MS/2)){
        targetYaw = at_baseYaw;
        at_state = AT_YAW_BACK;
        at_t0 = now;
      }
    } break;

    case AT_YAW_BACK: {
      if(now - at_t0 >= (ATUNE_YAW_PHASE_MS/2)){
        uint32_t riseMs = at_yawRiseRecorded ? at_yawRiseMs : (ATUNE_YAW_PHASE_MS/2);

        if(riseMs > 1500){
          yawKp *= 1.20f;
        } else if(riseMs < 700){
          yawKp *= 0.88f;
        }
        yawKp = _clampf(yawKp, YAW_KP_MIN, YAW_KP_MAX);

        at_state = AT_DONE;
      }
    } break;

    case AT_DONE:
      tune_saveToNVS();
      at_active = false;
      break;

    case AT_ABORT:
    default:
      at_active = false;
      break;
  }
}
