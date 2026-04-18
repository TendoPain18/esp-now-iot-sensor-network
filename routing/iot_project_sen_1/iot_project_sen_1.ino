#include <ESP8266WiFi.h>
#include <espnow.h>
extern "C" {
  #include "user_interface.h"
}

#define US_ECHO_PIN D5   // GPIO14 on many NodeMCU? (Actually D5=GPIO14)
#define US_TRIG_PIN D6   // D6=GPIO12

// ===================== Fixed parameters =====================
#define WIFI_CH                 1
static const uint16_t FRAME_MS     = 1000;
static const uint8_t  MAX_NODES    = 10;
static const uint8_t  M_SUBSLOTS   = 2;
static const uint8_t  G_MS         = 3;   // Gf=Gpre=Gpost=3ms
#define CONTROL_EVERY_FRAMES    5
#define OWN_DATA_MS             500

// Routing/neighbor
#define MAX_NEIGHBORS           25
#define NBR_TIMEOUT_MS          15000
#define NBR_FRESH_MS            6000
#define DIRECT_RSSI_TH          (-55)
#define LOOP_MARGIN_DBM         3
#define MAX_LEVEL               20

// Buffer
#define BUF_SIZE                35

// De-dup
#define DEDUP_SIZE              40
#define DEDUP_TTL_MS            8000

// Node identity (set per device 1..10, or 0 = auto)
#define NODE_ID_CFG             3

// Protocol keys
static const uint32_t GW_KEY      = 0xDEADBEEF;
static const uint32_t HELLO_MAGIC = 0xA1B2C3D4;
static const uint32_t DATA_KEY    = 0xD00DFEED;

uint8_t BCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// ============================================================
//                      TYPES FIRST
// ============================================================

struct Schedule {
  uint16_t dataSlots;
  uint16_t totalSlots;
  uint16_t slotMs;
  uint16_t remainderMs;
  uint16_t beaconMs;
  uint16_t slackMs;   // Gf
  uint16_t txSlotMs;  // slotMs - 2G
  bool ok;
};

#pragma pack(push, 1)
struct GwBeacon {
  uint32_t key;
  uint8_t  gwMac[6];
  uint32_t frameSeq;
  uint16_t epoch;
};

struct HelloMsg {
  uint32_t magic;
  uint16_t epoch;
  uint8_t  senderMac[6];
  uint8_t  nodeId;
  uint8_t  level;
  int8_t   rssiToGw;
  int8_t   linkRssiToPar;
  int16_t  pathSum;
  uint8_t  parent[6];
  uint32_t uptimeSec;
};

struct DataMsg {
  uint32_t key;
  uint16_t seq;
  uint8_t  srcId;
  uint8_t  srcMac[6];

  uint8_t  hopCount;
  uint8_t  pathLen;
  uint8_t  pathIds[10];

  uint8_t  nextHopMac[6];

  char     text[32];
};
#pragma pack(pop)

// promisc minimal
struct wifi_pkt_rx_ctrl_t {
  signed rssi : 8;
  unsigned : 24;
  unsigned channel : 4;
  unsigned : 12;
};
struct wifi_promiscuous_pkt_t {
  wifi_pkt_rx_ctrl_t rx_ctrl;
  uint8_t payload[256];
};

struct NeighborEntry {
  bool used=false;
  uint8_t mac[6]={0};
  uint8_t nodeId=0;
  uint8_t level=255;

  int8_t  linkRssi=-127;
  int8_t  neighRssiToGw=-127;
  int16_t neighPathSum=-32768;
  uint8_t neighParent[6]={0};

  uint16_t lastEpoch=0;
  uint32_t lastSeenMs=0;
};

struct DedupEntry {
  bool used=false;
  uint8_t srcId=0;
  uint16_t seq=0;
  uint32_t lastSeenMs=0;
};

// ============================================================
//                   GLOBALS (after types)
// ============================================================

static Schedule SCH;

NeighborEntry nbr[MAX_NEIGHBORS];

DataMsg buf[BUF_SIZE];
volatile uint8_t head=0, tail=0, count=0;
volatile uint32_t drops=0;

DedupEntry dedup[DEDUP_SIZE];
uint8_t dedupPos=0;

uint8_t MY_MAC[6];
uint8_t NODE_ID=0;

volatile uint16_t gwEpoch=0;
volatile bool gwEpochValid=false;

uint8_t GW_MAC[6]={0};
volatile bool gwKnown=false;

volatile int32_t gwSum=0;
volatile uint16_t gwCnt=0;
int8_t avgRssiToGw=-127;

volatile uint32_t lastGwBeaconRxMs=0;
volatile uint32_t lastGwFrameSeq=0;
volatile bool gwSyncValid=false;

// Anchor first-beacon-per-frameSeq
volatile uint32_t syncBaseMs=0;
volatile uint32_t anchoredFrameSeq=0xFFFFFFFF;

// Routing
uint8_t parentMac[6]={0};
int8_t  linkRssiToParent=-127;
int16_t myPathSum=-32768;
uint8_t myLevel=1;

// sensor generation
uint16_t myDataSeq=0;
uint32_t lastOwn=0;

// ============================================================
//                      HELPERS / FUNCTIONS
// ============================================================

static bool macEq(const uint8_t* a, const uint8_t* b){ return memcmp(a,b,6)==0; }

static String macToString(const uint8_t *m){
  char b[18];
  snprintf(b,sizeof(b),"%02X:%02X:%02X:%02X:%02X:%02X",m[0],m[1],m[2],m[3],m[4],m[5]);
  return String(b);
}

static void ensurePeer(const uint8_t* mac){
  esp_now_add_peer((uint8_t*)mac, ESP_NOW_ROLE_COMBO, WIFI_CH, NULL, 0);
}

// -------- Schedule build (Option A) --------
static Schedule buildScheduleA(uint16_t frameMs, uint8_t N, uint8_t m, uint8_t G){
  Schedule s{};
  s.dataSlots  = (uint16_t)N * (uint16_t)m;
  s.totalSlots = s.dataSlots + 1; // beacon is a slot
  s.slackMs    = G;               // Gf

  if (s.totalSlots == 0) { s.ok=false; return s; }
  if (frameMs <= G)      { s.ok=false; return s; }

  uint16_t usable = (uint16_t)(frameMs - G);         // only subtract end slack
  s.slotMs = (uint16_t)(usable / s.totalSlots);      // include beacon in division
  if (s.slotMs == 0) { s.ok=false; return s; }

  uint16_t used = (uint16_t)(s.slotMs * s.totalSlots);
  s.remainderMs = (uint16_t)(usable - used);
  s.beaconMs    = (uint16_t)(s.slotMs + s.remainderMs); // remainder added to beacon

  int16_t tx = (int16_t)s.slotMs - (int16_t)(2 * G);  // Gpre + Gpost
  s.txSlotMs = (tx > 0) ? (uint16_t)tx : 0;

  uint32_t total =
    (uint32_t)s.beaconMs + (uint32_t)s.dataSlots * (uint32_t)s.slotMs + (uint32_t)s.slackMs;

  s.ok = (s.txSlotMs > 0) && (total == frameMs);
  return s;
}

// -------- Path helpers --------
static void appendIdToPath(DataMsg &m, uint8_t id){
  if(m.pathLen >= 10) return;
  if(m.pathLen > 0 && m.pathIds[m.pathLen-1] == id) return;
  m.pathIds[m.pathLen] = id;
  m.pathLen++;
}

static bool pathContains(const DataMsg &m, uint8_t id){
  for(uint8_t i=0;i<m.pathLen && i<10;i++){
    if(m.pathIds[i] == id) return true;
  }
  return false;
}

// -------- Buffer ops --------
static inline void bufPush(const DataMsg& m){
  if(count >= BUF_SIZE){ drops++; return; }
  buf[head] = m;
  head = (head + 1) % BUF_SIZE;
  count++;
}
static inline bool bufPop(DataMsg& out){
  if(count == 0) return false;
  out = buf[tail];
  tail = (tail + 1) % BUF_SIZE;
  count--;
  return true;
}

// -------- De-dup --------
static bool seenRecently(uint8_t srcId, uint16_t seq){
  uint32_t now = millis();
  for(int i=0;i<DEDUP_SIZE;i++){
    if(!dedup[i].used) continue;
    if(now - dedup[i].lastSeenMs > DEDUP_TTL_MS) dedup[i].used = false;
  }
  for(int i=0;i<DEDUP_SIZE;i++){
    if(!dedup[i].used) continue;
    if(dedup[i].srcId == srcId && dedup[i].seq == seq){
      dedup[i].lastSeenMs = now;
      return true;
    }
  }
  dedup[dedupPos].used = true;
  dedup[dedupPos].srcId = srcId;
  dedup[dedupPos].seq = seq;
  dedup[dedupPos].lastSeenMs = now;
  dedupPos = (dedupPos + 1) % DEDUP_SIZE;
  return false;
}

// -------- Neighbor table --------
int findOrAllocNbr(const uint8_t* mac){
  for(int i=0;i<MAX_NEIGHBORS;i++) if(nbr[i].used && macEq(nbr[i].mac,mac)) return i;
  for(int i=0;i<MAX_NEIGHBORS;i++){
    if(!nbr[i].used){ nbr[i].used=true; memcpy(nbr[i].mac,mac,6); return i; }
  }
  return -1;
}

void cleanupNbr(uint32_t now){
  for(int i=0;i<MAX_NEIGHBORS;i++){
    if(!nbr[i].used) continue;
    if(now - nbr[i].lastSeenMs > NBR_TIMEOUT_MS) nbr[i].used=false;
  }
}

// -------- Sensors (SIMULATED; replace later) --------
static float sensor_temperatureC(){
  uint32_t t = millis();
  return 27.0f + 8.0f * sinf((float)t / 12000.0f);
}
static float sensor_humidityPct(){
  uint32_t t = millis();
  return 57.0f + 22.0f * sinf((float)t / 20000.0f);
}
static int sensor_ultrasonicCm(){
  // Optional: stop sniffing during timing-critical measurement
  sniffOff();

  // Trigger: 10us HIGH pulse
  digitalWrite(US_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG_PIN, LOW);

  // Read echo pulse width (timeout protects you from long blocking)
  // 15000us ~= 258cm max (since cm ≈ us/58)
  uint32_t dur = pulseIn(US_ECHO_PIN, HIGH, 15000);

  sniffOn();

  if(dur == 0) return -1;          // no echo / out of range
  int cm = (int)(dur / 58UL);      // HC-SR04 conversion
  return cm;
}

static uint8_t sensor_pirMotion(){
  uint32_t t = millis() % 7000;
  return (t < 1000) ? 1 : 0;
}
static void buildSensorPayload(char out[32]){
  int us = sensor_ultrasonicCm();
  float hu = sensor_humidityPct();
  float tc = sensor_temperatureC();
  uint8_t pir = sensor_pirMotion();
  snprintf(out, 32, "US=%d H=%.0f T=%.1f PIR=%u", us, hu, tc, (unsigned)pir);
}

// -------- Find structs in raw payload --------
bool findGwBeacon(const uint8_t* p, uint16_t len, GwBeacon &out){
  uint8_t v[4] = {(uint8_t)(GW_KEY),(uint8_t)(GW_KEY>>8),(uint8_t)(GW_KEY>>16),(uint8_t)(GW_KEY>>24)};
  for(uint16_t i=0; i + sizeof(GwBeacon) <= len; i++){
    if(p[i]==v[0] && p[i+1]==v[1] && p[i+2]==v[2] && p[i+3]==v[3]){
      memcpy(&out, p+i, sizeof(GwBeacon));
      return out.key==GW_KEY;
    }
  }
  return false;
}

bool findHello(const uint8_t* p, uint16_t len, HelloMsg &out){
  uint8_t v[4] = {(uint8_t)(HELLO_MAGIC),(uint8_t)(HELLO_MAGIC>>8),(uint8_t)(HELLO_MAGIC>>16),(uint8_t)(HELLO_MAGIC>>24)};
  for(uint16_t i=0; i + sizeof(HelloMsg) <= len; i++){
    if(p[i]==v[0] && p[i+1]==v[1] && p[i+2]==v[2] && p[i+3]==v[3]){
      memcpy(&out, p+i, sizeof(HelloMsg));
      return out.magic==HELLO_MAGIC;
    }
  }
  return false;
}

bool findDataMsg(const uint8_t* p, uint16_t len, DataMsg &out){
  uint8_t v[4] = {(uint8_t)(DATA_KEY),(uint8_t)(DATA_KEY>>8),(uint8_t)(DATA_KEY>>16),(uint8_t)(DATA_KEY>>24)};
  for(uint16_t i=0; i + sizeof(DataMsg) <= len; i++){
    if(p[i]==v[0] && p[i+1]==v[1] && p[i+2]==v[2] && p[i+3]==v[3]){
      memcpy(&out, p+i, sizeof(DataMsg));
      return out.key==DATA_KEY;
    }
  }
  return false;
}

// -------- Sniff control --------
static inline void sniffOn(){ wifi_promiscuous_enable(1); }
static inline void sniffOff(){ wifi_promiscuous_enable(0); }

// -------- Promiscuous callback --------
void ICACHE_FLASH_ATTR sniffer_cb(uint8_t *bufp, uint16_t len){
  wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t*)bufp;
  uint16_t scanLen = len;
  if(scanLen > sizeof(pkt->payload)) scanLen = sizeof(pkt->payload);

  const uint8_t* frame = pkt->payload;
  int8_t rssi = pkt->rx_ctrl.rssi;

  GwBeacon b;
  if(findGwBeacon(frame, scanLen, b)){
    memcpy(GW_MAC, b.gwMac, 6);
    gwKnown = true;

    gwSum += rssi;
    gwCnt++;

    gwEpoch = b.epoch;
    gwEpochValid = true;

    uint32_t now = millis();
    lastGwBeaconRxMs = now;
    lastGwFrameSeq = b.frameSeq;

    // First beacon per frameSeq anchors the frame start
    if(b.frameSeq != anchoredFrameSeq){
      anchoredFrameSeq = b.frameSeq;
      syncBaseMs = now;
      gwSyncValid = true;
    }
    return;
  }

  HelloMsg h;
  if(findHello(frame, scanLen, h)){
    if(macEq(h.senderMac, MY_MAC)) return;
    int idx = findOrAllocNbr(h.senderMac);
    if(idx >= 0){
      nbr[idx].nodeId = h.nodeId;
      nbr[idx].level  = h.level;
      nbr[idx].linkRssi = rssi;
      nbr[idx].neighRssiToGw = h.rssiToGw;
      nbr[idx].neighPathSum  = h.pathSum;
      memcpy(nbr[idx].neighParent, h.parent, 6);
      nbr[idx].lastEpoch = h.epoch;
      nbr[idx].lastSeenMs = millis();
    }
    return;
  }

  DataMsg m;
  if(findDataMsg(frame, scanLen, m)){
    if(macEq(m.nextHopMac, MY_MAC)){
      if(pathContains(m, NODE_ID)) return;
      if(seenRecently(m.srcId, m.seq)) return;   // de-dup
      bufPush(m);
    }
    return;
  }
}

void updateAvgGw(){
  noInterrupts();
  int32_t sum = gwSum;
  uint16_t cnt = gwCnt;
  gwSum=0; gwCnt=0;
  interrupts();
  if(cnt>0) avgRssiToGw = (int8_t)(sum / (int32_t)cnt);
}

void chooseParent(){
  if(!gwKnown) return;

  memcpy(parentMac, GW_MAC, 6);
  linkRssiToParent = avgRssiToGw;
  myLevel = 1;
  myPathSum = (int16_t)avgRssiToGw;

  if(avgRssiToGw > DIRECT_RSSI_TH) return;

  bool found=false;
  int best=-1;
  int16_t bestMetric=-32768;
  uint32_t now=millis();

  for(int i=0;i<MAX_NEIGHBORS;i++){
    if(!nbr[i].used) continue;
    if(now - nbr[i].lastSeenMs > NBR_FRESH_MS) continue;
    if(nbr[i].neighRssiToGw <= (avgRssiToGw + LOOP_MARGIN_DBM)) continue;
    if(nbr[i].neighPathSum <= -30000) continue;

    int16_t cand = (int16_t)nbr[i].linkRssi + nbr[i].neighPathSum;
    if(!found || cand > bestMetric){
      found=true; best=i; bestMetric=cand;
    }
  }

  if(found){
    memcpy(parentMac, nbr[best].mac, 6);
    linkRssiToParent = nbr[best].linkRssi;
    myPathSum = bestMetric;
    uint8_t lvl = nbr[best].level + 1;
    myLevel = (lvl > MAX_LEVEL) ? MAX_LEVEL : lvl;
  }
}

void sendHello(){
  if(!gwKnown) return;

  uint16_t e=0; bool eValid=false;
  noInterrupts(); e=gwEpoch; eValid=gwEpochValid; interrupts();
  if(!eValid) e=0;

  HelloMsg h;
  h.magic = HELLO_MAGIC;
  h.epoch = e;
  memcpy(h.senderMac, MY_MAC, 6);
  h.nodeId = NODE_ID;
  h.level  = myLevel;
  h.rssiToGw = avgRssiToGw;
  h.linkRssiToPar = linkRssiToParent;
  h.pathSum = myPathSum;
  memcpy(h.parent, parentMac, 6);
  h.uptimeSec = millis()/1000;

  ensurePeer(BCAST);
  esp_now_send(BCAST, (uint8_t*)&h, sizeof(h));
  ensurePeer(GW_MAC);
  esp_now_send(GW_MAC, (uint8_t*)&h, sizeof(h));
}

void genOwnData(){
  if(!gwKnown) return;

  DataMsg m;
  memset(&m, 0, sizeof(m));
  m.key = DATA_KEY;
  m.seq = myDataSeq++;
  m.srcId = NODE_ID;
  memcpy(m.srcMac, MY_MAC, 6);

  m.hopCount = 0;
  m.pathLen = 1;
  m.pathIds[0] = NODE_ID;

  memset(m.nextHopMac, 0, 6);
  buildSensorPayload(m.text);

  bufPush(m);
}

// ---------------- Slot timing ----------------
static inline bool syncOk(){
  uint32_t now = millis();
  return gwKnown && gwSyncValid && (now - (uint32_t)lastGwBeaconRxMs <= 3000);
}

// slot 0: beacon [0..beaconMs)
// slots 1..dataSlots: each slotMs
// slack: rest
static inline uint8_t currentSlot(uint32_t now, uint32_t &slotStart, uint32_t &slotEnd){
  uint32_t base = (uint32_t)syncBaseMs;
  uint32_t dt = (now >= base) ? (now - base) : 0;
  uint32_t pos = dt % FRAME_MS;

  uint32_t dataStart = SCH.beaconMs;
  uint32_t dataEnd   = SCH.beaconMs + (uint32_t)SCH.dataSlots * (uint32_t)SCH.slotMs; // == FRAME_MS - Gf

  uint32_t frameAbsStart = now - pos;

  if(pos < SCH.beaconMs){
    slotStart = frameAbsStart;
    slotEnd   = frameAbsStart + SCH.beaconMs;
    return 0;
  }
  if(pos >= dataEnd){
    slotStart = frameAbsStart + dataEnd;
    slotEnd   = frameAbsStart + FRAME_MS;
    return 255; // slack
  }

  uint32_t pos2 = pos - dataStart;
  uint16_t idx = (uint16_t)(pos2 / SCH.slotMs); // 0..dataSlots-1
  uint8_t s = (uint8_t)(1 + idx);

  slotStart = frameAbsStart + dataStart + (uint32_t)idx * (uint32_t)SCH.slotMs;
  slotEnd   = slotStart + SCH.slotMs;
  return s;
}

static inline bool isControlFrame(uint32_t frameSeq){
  if(CONTROL_EVERY_FRAMES <= 1) return false;
  return (frameSeq % CONTROL_EVERY_FRAMES) == 0;
}

// ===================== ROUND-ROBIN OWNED SLOTS =====================
// slot = 1 + k*MAX_NODES + (nodeId-1), for k=0..M_SUBSLOTS-1
static inline uint8_t nodeSlotK(uint8_t nodeId, uint8_t k){
  return (uint8_t)(1 + (uint16_t)k * (uint16_t)MAX_NODES + (uint16_t)(nodeId - 1));
}
static inline bool nodeOwnsSlot(uint8_t nodeId, uint8_t slot){
  if(slot < 1 || slot > SCH.dataSlots) return false;
  for(uint8_t k=0; k<M_SUBSLOTS; k++){
    if(slot == nodeSlotK(nodeId, k)) return true;
  }
  return false;
}

// ---------------- Print routing table (on epoch change) ----------------
void printNodeTable(uint16_t epoch){
  Serial.println();
  Serial.println(F("=============== NODE TABLE (every epoch) ==============="));
  Serial.printf("Me: nodeId=%u MAC=%s CH=%d\n",
                (unsigned)NODE_ID, WiFi.macAddress().c_str(), wifi_get_channel());

  Serial.printf("Schedule: FRAME_MS=%u N=%u m=%u G=%u | dataSlots=%u totalSlots=%u\n",
                (unsigned)FRAME_MS, (unsigned)MAX_NODES, (unsigned)M_SUBSLOTS, (unsigned)G_MS,
                (unsigned)SCH.dataSlots, (unsigned)SCH.totalSlots);

  Serial.printf("Derived: slotMs=%u beaconMs=%u slackMs=%u txSlotMs=%u | ownedSlots=",
                (unsigned)SCH.slotMs, (unsigned)SCH.beaconMs, (unsigned)SCH.slackMs,
                (unsigned)SCH.txSlotMs);
  for(uint8_t k=0;k<M_SUBSLOTS;k++){
    Serial.printf("%u", (unsigned)nodeSlotK(NODE_ID,k));
    if(k+1<M_SUBSLOTS) Serial.print(",");
  }
  Serial.println();

  uint32_t now=millis();
  Serial.printf("GW: mac=%s avgRSSI=%d lastBeaconAgo=%lu ms frameSeq=%lu epoch=%u\n",
                gwKnown ? macToString(GW_MAC).c_str() : "??",
                (int)avgRssiToGw,
                gwKnown ? (unsigned long)(now-(uint32_t)lastGwBeaconRxMs) : 0UL,
                (unsigned long)(uint32_t)lastGwFrameSeq,
                (unsigned)epoch);

  Serial.printf("PARENT: %s | link2Par=%d | pathSum=%d | level=%u\n",
                macToString(parentMac).c_str(),
                (int)linkRssiToParent,
                (int)myPathSum,
                (unsigned)myLevel);

  Serial.printf("RelayBuf: count=%u dropped=%lu\n", (unsigned)count, (unsigned long)drops);

  Serial.println(F("Type | ID  | MAC                | linkRSSI | neighGW | neighPathSum   | lastEpoch | neighLvl | neighParent | age(ms)"));
  Serial.println(F("---------------------------------------------------------------------------------------------------------------"));

  if(gwKnown){
    Serial.printf("GW   | --  | %-18s | %8d | %6d | %13d | %8u | %8u | %-18s | %7lu\n",
      macToString(GW_MAC).c_str(),
      (int)avgRssiToGw,
      (int)avgRssiToGw,
      (int)0,
      (unsigned)epoch,
      (unsigned)0,
      macToString(GW_MAC).c_str(),
      (unsigned long)(now-(uint32_t)lastGwBeaconRxMs)
    );
  }

  for(int i=0;i<MAX_NEIGHBORS;i++){
    if(!nbr[i].used) continue;
    uint32_t age = now - nbr[i].lastSeenMs;
    Serial.printf("NBR  | %3u | %-18s | %8d | %6d | %13d | %8u | %8u | %-18s | %7lu\n",
      (unsigned)nbr[i].nodeId,
      macToString(nbr[i].mac).c_str(),
      (int)nbr[i].linkRssi,
      (int)nbr[i].neighRssiToGw,
      (int)nbr[i].neighPathSum,
      (unsigned)nbr[i].lastEpoch,
      (unsigned)nbr[i].level,
      macToString(nbr[i].neighParent).c_str(),
      (unsigned long)age
    );
  }
  Serial.println(F("==============================================================================================================="));
}

// ---------------- Flush buffered DATA in a TX window ----------------
void flushDataInWindow(uint32_t txStartMs, uint32_t txEndMs){
  if(!gwKnown) return;

  while(millis() < txStartMs) delay(0);

  sniffOff();
  ensurePeer(parentMac);

  while(millis() < txEndMs){
    DataMsg m;
    noInterrupts();
    bool ok = bufPop(m);
    interrupts();
    if(!ok) break;

    if(!pathContains(m, NODE_ID)){
      if(m.srcId != NODE_ID || m.hopCount > 0) {
        appendIdToPath(m, NODE_ID);
      }
    }
    m.hopCount++;
    memcpy(m.nextHopMac, parentMac, 6);

    esp_now_send(parentMac, (uint8_t*)&m, sizeof(m));
    delay(0);
  }

  sniffOn();

  // keep TX printing commented
  // Serial.printf("TX: sent...\n");
}

void setup(){
  Serial.begin(115200);

  pinMode(US_TRIG_PIN, OUTPUT);
  pinMode(US_ECHO_PIN, INPUT);
  digitalWrite(US_TRIG_PIN, LOW);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  wifi_set_channel(WIFI_CH);
  delay(10);

  SCH = buildScheduleA(FRAME_MS, MAX_NODES, M_SUBSLOTS, G_MS);

  WiFi.macAddress(MY_MAC);

  if(NODE_ID_CFG >= 1 && NODE_ID_CFG <= MAX_NODES){
    NODE_ID = NODE_ID_CFG;
  }else{
    NODE_ID = (uint8_t)((MY_MAC[5] % MAX_NODES) + 1);
  }

  Serial.println();
  Serial.printf("BOOT: NODE MAC=%s nodeId=%u CH=%d\n",
                WiFi.macAddress().c_str(), (unsigned)NODE_ID, wifi_get_channel());

  if(!SCH.ok){
    Serial.println("BOOT: Schedule invalid! Check N,m,G.");
    Serial.printf("BOOT: dataSlots=%u totalSlots=%u slotMs=%u beaconMs=%u txSlotMs=%u\n",
                  (unsigned)SCH.dataSlots, (unsigned)SCH.totalSlots,
                  (unsigned)SCH.slotMs, (unsigned)SCH.beaconMs, (unsigned)SCH.txSlotMs);
    while(1) delay(100);
  }

  Serial.printf("BOOT: slotMs=%u remainderMs=%u beaconMs=%u slackMs=%u txSlotMs=%u\n",
                (unsigned)SCH.slotMs, (unsigned)SCH.remainderMs, (unsigned)SCH.beaconMs,
                (unsigned)SCH.slackMs, (unsigned)SCH.txSlotMs);

  if(esp_now_init()!=0){
    Serial.println("BOOT: ESP-NOW init FAIL");
    while(1) delay(100);
  }
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);

  wifi_set_promiscuous_rx_cb(sniffer_cb);
  sniffOn();

  lastOwn = millis();
  
}

void loop(){
  uint32_t now = millis();

  // Generate sensor data every 0.5s
  if(now - lastOwn >= OWN_DATA_MS){
    lastOwn = now;
    genOwnData();
  }

  // Print routing table when epoch changes (like before)
  static uint16_t lastEpochPrinted=0xFFFF;
  if(gwEpochValid){
    uint16_t e; noInterrupts(); e=gwEpoch; interrupts();
    if(e != lastEpochPrinted){
      lastEpochPrinted = e;
      updateAvgGw();
      cleanupNbr(now);
      chooseParent();
      printNodeTable(e);
    }
  }

  // sync validity
  if(gwKnown && (now - (uint32_t)lastGwBeaconRxMs > 3000)){
    gwSyncValid = false;
  }
  if(!syncOk()){
    delay(2);
    return;
  }

  // Determine current slot
  uint32_t slotStart=0, slotEnd=0;
  uint8_t s = currentSlot(now, slotStart, slotEnd);
  if(s == 255){
    delay(1);
    return;
  }

  // Round-robin ownership
  if(!nodeOwnsSlot(NODE_ID, s)){
    delay(1);
    return;
  }

  // Once-per-slot action
  static uint32_t lastHandledFrame=0xFFFFFFFF;
  static uint8_t  lastHandledSlot=0xFF;

  uint32_t frameSeq=0;
  uint16_t epoch=0;
  noInterrupts();
  frameSeq = lastGwFrameSeq;
  epoch = gwEpoch;
  interrupts();

  if(frameSeq == lastHandledFrame && s == lastHandledSlot){
    delay(1);
    return;
  }
  lastHandledFrame = frameSeq;
  lastHandledSlot  = s;

  updateAvgGw();
  cleanupNbr(now);
  chooseParent();

  bool ctrl = isControlFrame(frameSeq);

  // ONLY THIS SYNC LINE is uncommented (as requested)
  Serial.printf("SYNC: frameSeq=%lu epoch=%u slot=%u ctrl=%u txSlotMs=%u buf=%u\n",
                (unsigned long)frameSeq, (unsigned)epoch, (unsigned)s,
                (unsigned)ctrl, (unsigned)SCH.txSlotMs, (unsigned)count);

  // Send HELLO once per frame (on k=0 slot)
  if(s == nodeSlotK(NODE_ID, 0)){
    sendHello();
  }

  if(ctrl){
    delay(1);
    return;
  }

  // TX window inside this slot
  uint32_t txStart = slotStart + G_MS;                          // Gpre
  uint32_t txEnd   = (slotEnd > G_MS) ? (slotEnd - G_MS) : slotEnd; // Gpost

  flushDataInWindow(txStart, txEnd);

  delay(1);
}
