#define AF1000X_IMPLEMENTATION
#include "AF1000X_CORE.h"
#include "AF1000X_EasyCommander.h"
float g_loopDt = 0.02f;  // KO: 글로벌 루프 dt(초) / EN: global loop dt (seconds)

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5); // KO: 레거시 Serial 응답 빠르게 / EN: keep legacy Serial reads snappy
  delay(300);

  Serial.println();
  Serial.println("SYUBEA Co., LTD");
  Serial.println("www.1510.co.kr");
  Serial.print("Model : AF1000X FC - Ver.");
  Serial.println(FC_VERSION);
  Serial.println();
  initAF1000X();
  Serial.println("Use `PID?` / `PIDDEF` outputs to back up key tuning values");  
}

void loop() {
  static uint32_t lastUs = micros();
  uint32_t nowUs = micros();
  uint32_t elapsedUs = nowUs - lastUs;
  lastUs = nowUs;

  float dt = elapsedUs * 1e-6f;
  if (dt < 0.005f) dt = 0.005f;
  if (dt > 0.050f) dt = 0.050f;
  g_loopDt = dt;

  updateSystem();
  updateFlight();

  delay(20);
}
