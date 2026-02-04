#define AF1000X_IMPLEMENTATION
#include "AF1000X_CORE.h"
#include "AF1000X_EasyCommander.h"
float g_loopDt = 0.02f;  // KO: Í∏Î°úÎ≤å Î£®ÌîÑ dt(Ï¥) / EN: global loop dt (seconds)

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5); // KO: †àÍ±∞Ïãú Serial ùëãµ Îπ†Î•¥Í≤ / EN: keep legacy Serial reads snappy
  delay(1000);

  LOG_PRINTLN();
  LOG_PRINTLN("SYUBEA Co., LTD");
  LOG_PRINTLN("www.1510.co.kr");
  LOG_PRINT("Model : AF1000X FC - Ver.");
  LOG_PRINTLN(FC_VERSION);
  LOG_PRINTLN();
  initAF1000X();
  LOG_PRINTLN("Use `PID?` / `PIDDEF` outputs to back up key tuning values");  
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