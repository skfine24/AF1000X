#pragma once
#include <Arduino.h>
#include <RF24.h>
#include <Preferences.h>

// ============================================================================
// KO: AF1000X 바인딩 (드론)
// EN: AF1000X binding (drone-side)
// KO: 부팅 5초 내 기체 뒤집힘 -> 페어링 수신
// EN: Inverted within 5s after boot -> pairing listen
// KO: TX 주소(5바이트) 수신 후 NVS 저장, 이후 해당 주소만 수신
// EN: Receive TX address (5 bytes), store to NVS, then listen only to it
// KO: 바인딩 없으면 flightLock=true (READY 고정, 비행 불가)
// EN: If not bound => flightLock=true (READY only, no flight)
// ============================================================================

// ============================================================================
// KO: AF1000X_CORE.h 제공 extern
// EN: Externs provided by AF1000X_CORE.h
// ============================================================================
extern RF24 radio;
extern Preferences prefs;

extern volatile uint8_t currentMode;
extern bool flightLock;

extern float currentRoll;   // KO: deg / EN: deg
extern float currentPitch;  // KO: deg / EN: deg

extern uint8_t hopTable[HOP_MAX];
extern uint8_t hopLen;
extern bool linkReady;

// KO: 오토튠 리셋 (AF1000X_AutoTune.h)
// EN: Auto-tune reset (AF1000X_AutoTune.h)
static inline void tune_factoryReset();

// ============================================================================
// KO: 타이밍
// EN: Timing
// ============================================================================
static constexpr uint32_t BOOT_WINDOW_MS  = 5000;
static constexpr uint32_t PAIR_LISTEN_MS  = 4000;
static constexpr uint32_t WAIT_LINK_MS    = 5000;
static constexpr uint32_t LED_BLINK_MS    = 120;

// ============================================================================
// KO: 페어링 주소 (AR1000X 페어링 파이프와 동일)
// EN: Pairing address (must match AR1000X pairing pipe)
// ============================================================================
static const uint8_t PAIR_ADDR[5] = {0xE8, 0xE8, 0xF0, 0xF0, 0xE1};
static const uint8_t PAIR_CHANNEL = 76;
static const uint8_t LINK_FLAG_FACTORY_RESET = 0x01;

// ============================================================================
// KO: NVS 키
// EN: NVS keys
// ============================================================================
static constexpr const char* NVS_KEYB = "bound";
static constexpr const char* NVS_KEY0 = "tx0";
static constexpr const char* NVS_KEY1 = "tx1";
static constexpr const char* NVS_KEY2 = "tx2";
static constexpr const char* NVS_KEY3 = "tx3";
static constexpr const char* NVS_KEY4 = "tx4";

static constexpr const char* NVS_KEYHOP_LEN = "hopL";
static constexpr const char* NVS_KEYHOP[HOP_MAX] = {
  "h0","h1","h2","h3","h4","h5","h6","h7",
  "h8","h9","hA","hB"
};

// ============================================================================
// KO: 로컬 상태
// EN: Local state
// ============================================================================
static uint8_t g_boundAddr[5] = {0xB1,0xB2,0xB3,0xB4,0x01};
static bool    g_isBound = false;

// KO: 전방 선언 (정의 전 사용)
// EN: Forward decl (used before definition)
static inline void binding_applyHop(const uint8_t* table, uint8_t len);

struct LinkPayload {
  char name[8];
  uint8_t addr[5];
  uint8_t hopLen;
  uint8_t hopTable[HOP_MAX];
  uint8_t flags;
  uint8_t crc;
};

enum BindState : uint8_t {
  BS_NORMAL = 0,
  BS_WAIT_WINDOW,
  BS_PAIRING_LISTEN,
  BS_WAIT_LINK,
  BS_BOUND_OK,
  BS_ERROR
};

static BindState g_bs = BS_WAIT_WINDOW;
static uint32_t  g_bootMs  = 0;
static uint32_t  g_stateMs = 0;

// ============================================================================
// KO: LED 훅 (AF1000X_CORE.h에서 구현)
// EN: LED hooks (implemented in AF1000X_CORE.h)
// ============================================================================
void binding_ledAll(bool on);
void binding_ledPairingTick();
void binding_ledBoundOnce();
void binding_ledErrorOnce();

// ============================================================================
// KO: 뒤집힘 감지
// EN: Inverted detect
// ============================================================================
static inline bool binding_isInverted() {
  return (fabsf(currentRoll) > 120.0f) || (fabsf(currentPitch) > 120.0f);
}

// ============================================================================
// KO: NVS 로드/저장
// EN: NVS load/save
// ============================================================================
static inline void binding_loadFromNVS() {
  g_isBound = prefs.getBool(NVS_KEYB, false);
  if (!g_isBound) return;
  g_boundAddr[0] = (uint8_t)prefs.getUChar(NVS_KEY0, g_boundAddr[0]);
  g_boundAddr[1] = (uint8_t)prefs.getUChar(NVS_KEY1, g_boundAddr[1]);
  g_boundAddr[2] = (uint8_t)prefs.getUChar(NVS_KEY2, g_boundAddr[2]);
  g_boundAddr[3] = (uint8_t)prefs.getUChar(NVS_KEY3, g_boundAddr[3]);
  g_boundAddr[4] = (uint8_t)prefs.getUChar(NVS_KEY4, g_boundAddr[4]);
}

static inline void binding_saveToNVS(const uint8_t addr[5]) {
  prefs.putUChar(NVS_KEY0, addr[0]);
  prefs.putUChar(NVS_KEY1, addr[1]);
  prefs.putUChar(NVS_KEY2, addr[2]);
  prefs.putUChar(NVS_KEY3, addr[3]);
  prefs.putUChar(NVS_KEY4, addr[4]);
  prefs.putBool(NVS_KEYB, true);
}

static inline void binding_loadHopFromNVS() {
  uint8_t len = (uint8_t)prefs.getUChar(NVS_KEYHOP_LEN, 0);
  if(len >= 1 && len <= HOP_MAX){
    uint8_t tmp[HOP_MAX] = {0};
    for(uint8_t i = 0; i < len; i++){
      tmp[i] = (uint8_t)prefs.getUChar(NVS_KEYHOP[i], 0);
    }
    binding_applyHop(tmp, len);
  }
}

static inline void binding_saveHopToNVS(const uint8_t* table, uint8_t len) {
  if(len < 1 || len > HOP_MAX) return;
  prefs.putUChar(NVS_KEYHOP_LEN, len);
  for(uint8_t i = 0; i < len; i++){
    prefs.putUChar(NVS_KEYHOP[i], table[i]);
  }
}

static inline void binding_applyHop(const uint8_t* table, uint8_t len) {
  if(len < 1 || len > HOP_MAX) return;
  uint8_t out = 0;
  for(uint8_t i = 0; i < len; i++){
    uint8_t ch = table[i];
    if(ch < HOP_CH_MIN || ch > HOP_CH_MAX) continue;
    hopTable[out++] = ch;
    if(out >= HOP_MAX) break;
  }
  if(out == 0) return;
  hopLen = out;
}

static inline uint8_t linkCrc(const LinkPayload &p){
  uint8_t c = 0x6A;
  for(uint8_t i = 0; i < 8; i++) c ^= (uint8_t)(p.name[i] + (i * 13));
  for(uint8_t i = 0; i < 5; i++) c ^= (uint8_t)(p.addr[i] + (i * 7));
  c ^= p.hopLen;
  for(uint8_t i = 0; i < p.hopLen && i < HOP_MAX; i++) c ^= (uint8_t)(p.hopTable[i] + (i * 11));
  c ^= p.flags;
  return c;
}

static inline uint8_t binding_getHopSeed() {
  uint8_t s = (uint8_t)(g_boundAddr[0] ^ g_boundAddr[1] ^ g_boundAddr[2] ^ g_boundAddr[3] ^ g_boundAddr[4]);
  return (s == 0) ? 1 : s;
}

// ============================================================================
// KO: 리스닝 파이프 설정
// EN: Apply listening pipe
// ============================================================================
static inline void binding_listenOn(const uint8_t addr[5]) {
  radio.stopListening();
  radio.openReadingPipe(1, addr);
  radio.startListening();
}

static inline void binding_listenOnPair() {
  radio.stopListening();
  radio.setChannel(PAIR_CHANNEL);
  radio.openReadingPipe(1, PAIR_ADDR);
  radio.startListening();
}

static inline void binding_listenOnBoundPair() {
  radio.stopListening();
  radio.setChannel(PAIR_CHANNEL);
  radio.openReadingPipe(1, g_boundAddr);
  radio.startListening();
}

// ============================================================================
// KO: 공개 헬퍼
// EN: Public helpers
// ============================================================================
static inline bool binding_isBound() { return g_isBound; }

// ============================================================================
// KO: 시작 (radioInit + prefs.begin 이후 호출)
// EN: Begin (call after radioInit + prefs.begin)
// ============================================================================
static inline void binding_begin() {
  g_bootMs  = millis();
  g_stateMs = g_bootMs;

  binding_loadFromNVS();
  binding_loadHopFromNVS();

  if (g_isBound) {
    // KO: 기억된 조종기 링크 셋업 대기 (페어링 채널)
    // EN: Wait for remembered controller link setup on pair channel
    binding_listenOnBoundPair();
    linkReady = false;
    flightLock = true;
    g_bs = BS_WAIT_LINK;
    binding_ledBoundOnce();
  } else {
    // KO: 미바운드면 비행 락
    // EN: If unbound, keep flight lock
    flightLock = true;
    currentMode = 0;
    g_bs = BS_WAIT_WINDOW;
  }
}

// ============================================================================
// KO: 주기 업데이트 (updateSystem 내부 호출)
// EN: Tick (call inside updateSystem)
// ============================================================================
static inline void binding_update() {
  const uint32_t now = millis();

  switch (g_bs) {
    case BS_NORMAL:
      return;

    case BS_WAIT_WINDOW: {
      // KO: 부팅 5초 내 뒤집힘 -> 페어링 모드
      // EN: Inverted within 5s after boot -> pairing mode
      if ((now - g_bootMs) <= BOOT_WINDOW_MS) {
        if (binding_isInverted()) {
          g_bs = BS_PAIRING_LISTEN;
          g_stateMs = now;

          binding_listenOnPair();

          flightLock = true;
          currentMode = 0;

          binding_ledAll(false);
          return;
        }
      } else {
        // KO: 창 종료, 미바운드면 에러 상태 고정 (재부팅 필요)
        // EN: Window expired; if unbound, stay in error until reboot
        binding_listenOnPair();
        flightLock = true;
        currentMode = 0;
        g_bs = BS_ERROR;
        binding_ledErrorOnce();
      }
    } break;

    case BS_PAIRING_LISTEN: {
      binding_ledPairingTick();

      if (now - g_stateMs > PAIR_LISTEN_MS) {
        g_bs = BS_ERROR;
        binding_ledErrorOnce();
        flightLock = true;
        currentMode = 0;
        return;
      }

      if (radio.available()) {
        LinkPayload payload = {};
        radio.read(&payload, sizeof(payload));

        if(payload.crc != linkCrc(payload)) return;
        if(payload.flags & LINK_FLAG_FACTORY_RESET){
          tune_factoryReset();
        }

        // KO: sanity 체크 (모두 00/FF 방지)
        // EN: sanity check (not all 00 / not all FF)
        uint8_t orv = payload.addr[0] | payload.addr[1] | payload.addr[2] | payload.addr[3] | payload.addr[4];
        if (orv == 0x00 || orv == 0xFF) return;

        memcpy(g_boundAddr, payload.addr, 5);
        binding_saveToNVS(g_boundAddr);

        if(payload.hopLen >= 1 && payload.hopLen <= HOP_MAX){
          binding_applyHop(payload.hopTable, payload.hopLen);
          binding_saveHopToNVS(hopTable, hopLen);
        }

        g_isBound = true;
        linkReady = true;

        binding_listenOn(g_boundAddr);

        flightLock = false;
        g_bs = BS_BOUND_OK;
        binding_ledBoundOnce();
        return;
      }
    } break;

    case BS_WAIT_LINK: {
      // KO: 뒤집힘 시 수동 페어링 허용
      // EN: Allow manual pairing if inverted
      if(binding_isInverted()){
        g_bs = BS_PAIRING_LISTEN;
        g_stateMs = now;
        binding_listenOnPair();
        flightLock = true;
        currentMode = 0;
        binding_ledAll(false);
        return;
      }

      if(now - g_stateMs >= WAIT_LINK_MS){
        // KO: 타임아웃 -> 저장된 홉 테이블로 복귀, 제어 허용
        // EN: Timeout -> fall back to stored hop table and allow control
        linkReady = true;
        binding_listenOn(g_boundAddr);
        flightLock = false;
        g_bs = BS_NORMAL;
        return;
      }

      if (radio.available()) {
        LinkPayload payload = {};
        radio.read(&payload, sizeof(payload));
        if(payload.crc != linkCrc(payload)) return;

        // KO: 바인딩 주소와 일치해야 함
        // EN: must match bound addr
        if(memcmp(payload.addr, g_boundAddr, 5) != 0) return;
        if(payload.flags & LINK_FLAG_FACTORY_RESET){
          tune_factoryReset();
        }

        if(payload.hopLen >= 1 && payload.hopLen <= HOP_MAX){
          binding_applyHop(payload.hopTable, payload.hopLen);
          binding_saveHopToNVS(hopTable, hopLen);
        }

        linkReady = true;
        binding_listenOn(g_boundAddr);
        flightLock = false;
        g_bs = BS_NORMAL;
        return;
      }
    } break;

    case BS_BOUND_OK:
      g_bs = BS_NORMAL;
      return;

    case BS_ERROR:
      flightLock = true;
      currentMode = 0;
      return;
  }
}
