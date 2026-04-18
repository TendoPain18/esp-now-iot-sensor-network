#include <WiFi.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ===================== WiFi =====================
const char* WIFI_SSID = "";
const char* WIFI_PASS = "";

// ===================== ThingsBoard (demo) =====================
const char* TB_HOST  = "";
const int   TB_PORT  = 0;
const char* TB_TOKEN = "";

// MQTT Topics
static const char* TOPIC_TELEMETRY       = "";
static const char* TOPIC_RPC_REQ         = "";
static const char* TOPIC_RPC_RESP_PREFIX = "";

// ===================== I2C (ESP32 Slave) =====================
#define I2C_ADDR    0x08
#define SDA_PIN     21
#define SCL_PIN     22
static const uint16_t I2C_PREAMBLE = 0xBEEF;

#pragma pack(push, 1)
struct I2CPacket {
  uint16_t preamble;
  uint16_t length;
  uint16_t seq;
  uint8_t  srcId;
  uint8_t  hopCount;
  char     text[32];
  uint16_t crc;
};
#pragma pack(pop)

// ===================== CRC16 =====================
static uint16_t crc16_ccitt(const uint8_t* data, size_t len){
  uint16_t crc = 0xFFFF;
  for(size_t i=0;i<len;i++){
    crc ^= (uint16_t)data[i] << 8;
    for(int b=0;b<8;b++){
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

// ===================== Globals =====================
volatile bool gotPkt = false;
I2CPacket pkt;

const int LED_PIN = 2;   // ESP32 Dev Module often uses GPIO2 LED
bool ledState = false;

WiFiClient espClient;
PubSubClient mqtt(espClient);

// ===================== Parse "US=124 H=78 T=27.2 PIR=0" =====================
static bool parseSensorText(const char* s,
                            int &us, int &h, float &t, int &pir,
                            bool &hasUS, bool &hasH, bool &hasT, bool &hasPIR) {
  hasUS = hasH = hasT = hasPIR = false;
  us = h = pir = 0;
  t = 0.0f;

  const char* p = strstr(s, "US=");
  if(p){ us = atoi(p + 3); hasUS = true; }

  p = strstr(s, "H=");
  if(p){ h = atoi(p + 2); hasH = true; }

  p = strstr(s, "T=");
  if(p){ t = atof(p + 2); hasT = true; }

  p = strstr(s, "PIR=");
  if(p){ pir = atoi(p + 4); hasPIR = true; }

  return hasUS || hasH || hasT || hasPIR;
}

// Make key: "id3_US"
static void makeKey(char* out, size_t outSize, uint8_t srcId, const char* suffix){
  snprintf(out, outSize, "id%u_%s", (unsigned)srcId, suffix);
}

// ===================== I2C Receive Callback =====================
void onI2CReceive(int bytes){
  if(bytes < (int)sizeof(I2CPacket)){
    while(Wire.available()) Wire.read();
    return;
  }
  uint8_t *p = (uint8_t*)&pkt;
  for(size_t i=0; i<sizeof(I2CPacket) && Wire.available(); i++){
    p[i] = (uint8_t)Wire.read();
  }
  while(Wire.available()) Wire.read();
  gotPkt = true;
}

// ===================== Validate Packet =====================
bool validatePacket(I2CPacket& p){
  if(p.preamble != I2C_PREAMBLE) return false;

  uint16_t rx = p.crc;
  p.crc = 0;
  uint16_t calc = crc16_ccitt((const uint8_t*)&p, sizeof(I2CPacket) - sizeof(p.crc));
  p.crc = rx;

  if(calc != rx) return false;

  p.text[31] = '\0';
  return true;
}

// ===================== WiFi =====================
void ensureWiFi(){
  if(WiFi.status() == WL_CONNECTED) return;

  Serial.printf("WiFi: connecting to %s...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t start = millis();
  while(WiFi.status() != WL_CONNECTED){
    delay(300);
    Serial.print(".");
    if(millis() - start > 20000){
      Serial.println("\nWiFi: timeout, retry...");
      WiFi.disconnect();
      delay(500);
      WiFi.begin(WIFI_SSID, WIFI_PASS);
      start = millis();
    }
  }

  Serial.print("\nWiFi OK. IP=");
  Serial.println(WiFi.localIP());
}

// ===================== MQTT RPC Callback =====================
void mqttCallback(char* topic, byte* payload, unsigned int length){
  String t(topic);
  int lastSlash = t.lastIndexOf('/');
  String reqId = (lastSlash >= 0) ? t.substring(lastSlash + 1) : "0";

  StaticJsonDocument<256> doc;
  if(deserializeJson(doc, payload, length)){
    Serial.println("RPC: bad JSON");
    return;
  }

  const char* method = doc["method"] | "";
  JsonVariant params = doc["params"];

  if(String(method) == "setLed"){
    bool on = false;
    if(params.is<bool>()) on = params.as<bool>();
    else if(params.is<int>()) on = (params.as<int>() != 0);

    ledState = on;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);

    StaticJsonDocument<64> resp;
    resp["ok"] = true;
    resp["led"] = ledState;

    char out[128];
    size_t n = serializeJson(resp, out);

    String respTopic = String(TOPIC_RPC_RESP_PREFIX) + reqId;
    mqtt.publish(respTopic.c_str(), out, n);
  }
}

// ===================== MQTT Connect =====================
void ensureMQTT(){
  if(mqtt.connected()) return;

  mqtt.setServer(TB_HOST, TB_PORT);
  mqtt.setCallback(mqttCallback);

  String clientId = "esp32-" + String((uint32_t)ESP.getEfuseMac(), HEX);

  Serial.println("MQTT: connecting...");
  while(!mqtt.connected()){
    if(mqtt.connect(clientId.c_str(), TB_TOKEN, NULL)){
      Serial.println("MQTT OK");
      mqtt.subscribe(TOPIC_RPC_REQ);
    } else {
      Serial.printf("MQTT fail rc=%d, retry...\n", mqtt.state());
      delay(1500);
    }
  }
}

// ===================== Publish Per-Node Telemetry =====================
void publishTelemetryPerNode(const I2CPacket& p){
  int us, h, pir;
  float temp;
  bool hasUS, hasH, hasT, hasPIR;

  parseSensorText(p.text, us, h, temp, pir, hasUS, hasH, hasT, hasPIR);

  // IMPORTANT: Use larger doc so keys don't get dropped
  StaticJsonDocument<1024> doc;
  JsonObject obj = doc.to<JsonObject>();

  char key[24];

  // Always publish per-node seq/hops/text (debug)
  makeKey(key, sizeof(key), p.srcId, "seq");
  obj[key] = p.seq;

  makeKey(key, sizeof(key), p.srcId, "hops");
  obj[key] = p.hopCount;

  makeKey(key, sizeof(key), p.srcId, "text");
  obj[key] = p.text;

  // Publish only values that exist
  if(hasUS && us >= 0){ // ignore US=-1
    makeKey(key, sizeof(key), p.srcId, "US");
    obj[key] = us;
  }
  if(hasH){
    makeKey(key, sizeof(key), p.srcId, "H");
    obj[key] = h;
  }
  if(hasT){
    makeKey(key, sizeof(key), p.srcId, "T");
    obj[key] = temp;
  }
  if(hasPIR){
    makeKey(key, sizeof(key), p.srcId, "PIR");
    obj[key] = pir;
  }

  // Optional global LED state
  obj["esp32_led"] = ledState;

  char out[1024];
  size_t n = serializeJson(doc, out);

  // Debug: see EXACT JSON that is sent
  Serial.print("TB JSON: ");
  Serial.println(out);

  mqtt.publish(TOPIC_TELEMETRY, out, n);
}

// ===================== Setup =====================
void setup(){
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // I2C slave
  Wire.begin(I2C_ADDR, SDA_PIN, SCL_PIN, 100000);
  Wire.onReceive(onI2CReceive);

  ensureWiFi();
  ensureMQTT();

  Serial.println("Ready: I2C -> WiFi -> ThingsBoard (per-node keys)");
}

// ===================== Loop =====================
void loop(){
  ensureWiFi();
  ensureMQTT();
  mqtt.loop();

  if(gotPkt){
    gotPkt = false;

    I2CPacket local = pkt;
    if(!validatePacket(local)){
      Serial.println("I2C: bad packet");
      return;
    }

    Serial.printf("I2C: seq=%u srcId=%u hops=%u text=%s\n",
                  (unsigned)local.seq, local.srcId, local.hopCount, local.text);

    publishTelemetryPerNode(local);
  }
}
