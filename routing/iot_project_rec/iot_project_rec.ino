#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Wire.h>
extern "C" {
  #include "user_interface.h"
}

#define WIFI_CH                 1
static const uint16_t FRAME_MS     = 1000;
static const uint8_t  MAX_NODES    = 10;
static const uint8_t  M_SUBSLOTS   = 2;
static const uint8_t  G_MS         = 3;

#define BEACON_REPEATS          4
#define BEACON_SPACING_MS       4

#define PRINT_TABLE_MS          5000
#define EPOCH_STEP_MS           5000

static const uint32_t GW_KEY      = 0xDEADBEEF;
static const uint32_t HELLO_MAGIC = 0xA1B2C3D4;
static const uint32_t DATA_KEY    = 0xD00DFEED;

uint8_t BCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// ===================== LED SETTINGS =====================
#define LED_PIN   14   // D5 = GPIO14

// ===================== I2C SETTINGS =====================
#define I2C_ADDR      0x08
#define I2C_SDA_PIN   4   // D2 = GPIO4
#define I2C_SCL_PIN   5   // D1 = GPIO5
#define I2C_CLOCK     100000

static const uint16_t I2C_PREAMBLE = 0xBEEF;

// ===================== TYPES FIRST =====================

struct Schedule {
  uint16_t dataSlots;
  uint16_t totalSlots;
  uint16_t slotMs;
  uint16_t remainderMs;
  uint16_t beaconMs;
  uint16_t slackMs;
  uint16_t txSlotMs;
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

// Packet we send over I2C to ESP32
struct I2CPacket {
  uint16_t preamble;   // 0xBEEF
  uint16_t length;     // payload length below (not including preamble/length/crc)
  uint16_t seq;
  uint8_t  srcId;
  uint8_t  hopCount;
  char     text[32];
  uint16_t crc;        // CRC16 over all fields except crc
};
#pragma pack(pop)

// ===================== helpers =====================

static uint16_t crc16_ccitt(const uint8_t* data, size_t len){
  uint16_t crc = 0xFFFF;
  for(size_t i=0;i<len;i++){
    crc ^= (uint16_t)data[i] << 8;
    for(int b=0;b<8;b++){
      if(crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else            crc = (crc << 1);
    }
  }
  return crc;
}

static String macToString(const uint8_t *m){
  char b[18];
  snprintf(b,sizeof(b),"%02X:%02X:%02X:%02X:%02X:%02X",m[0],m[1],m[2],m[3],m[4],m[5]);
  return String(b);
}

static Schedule buildScheduleA(uint16_t frameMs, uint8_t N, uint8_t m, uint8_t G){
  Schedule s{};
  s.dataSlots  = (uint16_t)N * (uint16_t)m;
  s.totalSlots = s.dataSlots + 1;
  s.slackMs    = G;

  if (s.totalSlots == 0) { s.ok=false; return s; }
  if (frameMs <= G)      { s.ok=false; return s; }

  uint16_t usable = (uint16_t)(frameMs - G);
  s.slotMs = (uint16_t)(usable / s.totalSlots);
  if (s.slotMs == 0) { s.ok=false; return s; }

  uint16_t used = (uint16_t)(s.slotMs * s.totalSlots);
  s.remainderMs = (uint16_t)(usable - used);
  s.beaconMs    = (uint16_t)(s.slotMs + s.remainderMs);

  int16_t tx = (int16_t)s.slotMs - (int16_t)(2 * G);
  s.txSlotMs = (tx > 0) ? (uint16_t)tx : 0;

  uint32_t total =
    (uint32_t)s.beaconMs + (uint32_t)s.dataSlots * (uint32_t)s.slotMs + (uint32_t)s.slackMs;

  s.ok = (s.txSlotMs > 0) && (total == frameMs);
  return s;
}

static void sendI2CFromDataMsg(const DataMsg& m){
  I2CPacket p{};
  p.preamble = I2C_PREAMBLE;
  p.length   = (uint16_t)(sizeof(I2CPacket) - sizeof(p.preamble) - sizeof(p.length) - sizeof(p.crc));
  p.seq      = m.seq;
  p.srcId    = m.srcId;
  p.hopCount = m.hopCount;
  memset(p.text, 0, sizeof(p.text));
  strncpy(p.text, m.text, sizeof(p.text)-1);

  p.crc = 0;
  p.crc = crc16_ccitt((const uint8_t*)&p, sizeof(I2CPacket) - sizeof(p.crc));

  Wire.beginTransmission(I2C_ADDR);
  Wire.write((const uint8_t*)&p, sizeof(p));
  uint8_t err = Wire.endTransmission();
  if(err != 0){
    Serial.printf("I2C: TX fail err=%u\n", err);
  }
}

// ===================== NEW: Extract US Value =====================
static int extractUS(const char* text){
  // Example: "US=69 H=79 T=30.8 PIR=0"
  const char* p = strstr(text, "US=");
  if(!p) return -1;
  p += 3; // move after "US="
  return atoi(p); // stops automatically at space
}

// ===================== globals =====================
static Schedule SCH;

// ---------------- Routing table ----------------
struct RouteEntry {
  bool used=false;
  uint8_t mac[6]={0};
  uint8_t nodeId=0;
  uint16_t epoch=0;
  uint8_t level=0;
  int8_t  rssiToGw=-127;
  int8_t  link2Par=-127;
  int16_t pathSum=-32768;
  uint8_t parent[6]={0};
  uint32_t lastSeenMs=0;
};
static const int MAX_RT=80;
RouteEntry rt[MAX_RT];

int findOrAlloc(const uint8_t *mac){
  for(int i=0;i<MAX_RT;i++) if(rt[i].used && memcmp(rt[i].mac,mac,6)==0) return i;
  for(int i=0;i<MAX_RT;i++){
    if(!rt[i].used){ rt[i].used=true; memcpy(rt[i].mac,mac,6); return i; }
  }
  return -1;
}

void printRoutingTable(){
  Serial.println();
  Serial.println(F("TBL: ================= GATEWAY ROUTING TABLE ================="));
  Serial.println(F("TBL: NodeID | Node MAC            | Epoch | Lvl | RSSI2GW | link2Par | pathSum | Parent MAC          | Age(ms)"));
  Serial.println(F("TBL: -----------------------------------------------------------------------------------------------------------"));
  uint32_t now = millis();
  for(int i=0;i<MAX_RT;i++){
    if(!rt[i].used) continue;
    uint32_t age = now - rt[i].lastSeenMs;
    Serial.printf("TBL: %6u | %-18s | %5u | %3u | %7d | %8d | %7d | %-18s | %7u\n",
      rt[i].nodeId,
      macToString(rt[i].mac).c_str(),
      rt[i].epoch,
      rt[i].level,
      (int)rt[i].rssiToGw,
      (int)rt[i].link2Par,
      (int)rt[i].pathSum,
      macToString(rt[i].parent).c_str(),
      (unsigned)age
    );
  }
  Serial.println(F("TBL: =========================================================="));
}

// ---------------- Frame/beacon ----------------
uint32_t frameSeq=0;
uint16_t epoch=0;
uint32_t lastPrint=0;
uint32_t lastEpochTick=0;

static inline uint32_t frameStart(uint32_t now){
  return (now / FRAME_MS) * FRAME_MS;
}

void sendBeaconOnce(){
  GwBeacon b;
  b.key = GW_KEY;
  WiFi.macAddress(b.gwMac);
  b.frameSeq = frameSeq;
  b.epoch = epoch;
  esp_now_send(BCAST, (uint8_t*)&b, sizeof(b));
}

uint8_t beaconSentCount=0;
uint32_t lastBeaconSendMs=0;
uint32_t lastFrameStartSeen=0;

void onRecv(uint8_t *mac, uint8_t *data, uint8_t len){
  if(len == sizeof(HelloMsg)){
    HelloMsg h; memcpy(&h, data, sizeof(h));
    if(h.magic == HELLO_MAGIC){
      int idx = findOrAlloc(h.senderMac);
      if(idx >= 0){
        rt[idx].nodeId   = h.nodeId;
        rt[idx].epoch    = h.epoch;
        rt[idx].level    = h.level;
        rt[idx].rssiToGw = h.rssiToGw;
        rt[idx].link2Par = h.linkRssiToPar;
        rt[idx].pathSum  = h.pathSum;
        memcpy(rt[idx].parent, h.parent, 6);
        rt[idx].lastSeenMs = millis();
      }
      return;
    }
  }

  if(len == sizeof(DataMsg)){
    DataMsg m; memcpy(&m, data, sizeof(m));
    if(m.key != DATA_KEY) return;

    String route="";
    for(uint8_t i=0;i<m.pathLen && i<10;i++){
      route += String(m.pathIds[i]);
      if(i+1<m.pathLen) route += "->";
    }
    if(route.length()==0) route = String(m.srcId);
    route += "->GW";

    Serial.printf("RX: from=%s | route=%s | seq=%u hops=%u | \"%s\"\n",
      macToString(mac).c_str(),
      route.c_str(),
      (unsigned)m.seq,
      (unsigned)m.hopCount,
      m.text
    );

    // ✅ NEW: If nodeId = 3, check US and control LED on D5
    if(m.srcId == 3){
      int usVal = extractUS(m.text);
      if(usVal >= 0){
        if(usVal < 20){
          digitalWrite(LED_PIN, HIGH);   // LED ON
          Serial.printf("LED: ON  (Node 3 US=%d < 20)\n", usVal);
        }else{
          digitalWrite(LED_PIN, LOW);    // LED OFF
          Serial.printf("LED: OFF (Node 3 US=%d >= 20)\n", usVal);
        }
      }else{
        Serial.println("WARN: Could not extract US value!");
      }
    }

    // Forward to ESP32 over I2C
    sendI2CFromDataMsg(m);
  }
}

void setup(){
  Serial.begin(115200);

  // ✅ LED output
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // I2C master on NodeMCU v2: D2(SDA)=GPIO4, D1(SCL)=GPIO5
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  wifi_set_channel(WIFI_CH);
  delay(10);

  SCH = buildScheduleA(FRAME_MS, MAX_NODES, M_SUBSLOTS, G_MS);
  if(!SCH.ok){
    Serial.println("BOOT: Schedule invalid! Check N,m,G.");
    while(1) delay(100);
  }

  Serial.println();
  Serial.print("BOOT: GATEWAY MAC: "); Serial.println(WiFi.macAddress());
  Serial.printf("BOOT: CH=%d FRAME_MS=%u N=%u m=%u G=%u\n",
                wifi_get_channel(), (unsigned)FRAME_MS,
                (unsigned)MAX_NODES, (unsigned)M_SUBSLOTS, (unsigned)G_MS);

  if(esp_now_init()!=0){
    Serial.println("BOOT: ESP-NOW init FAIL");
    while(1) delay(100);
  }
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_recv_cb(onRecv);
  esp_now_add_peer(BCAST, ESP_NOW_ROLE_COMBO, WIFI_CH, NULL, 0);

  lastPrint = lastEpochTick = millis();
  lastFrameStartSeen = frameStart(millis());
  beaconSentCount = 0;

  Serial.println("BOOT: GATEWAY READY");
}

void loop(){
  uint32_t now = millis();

  if(now - lastEpochTick >= EPOCH_STEP_MS){
    lastEpochTick += EPOCH_STEP_MS;
    epoch++;
  }

  uint32_t fs = frameStart(now);
  if(fs != lastFrameStartSeen){
    lastFrameStartSeen = fs;
    frameSeq++;
    beaconSentCount = 0;
    lastBeaconSendMs = 0;
  }

  if(now >= fs && now < (fs + SCH.beaconMs)){
    if(beaconSentCount < BEACON_REPEATS){
      if(beaconSentCount == 0 || (now - lastBeaconSendMs >= BEACON_SPACING_MS)){
        sendBeaconOnce();
        lastBeaconSendMs = now;
        beaconSentCount++;
      }
    }
  }

  if(now - lastPrint >= PRINT_TABLE_MS){
    lastPrint = now;
    printRoutingTable();
  }

  delay(1);
}
