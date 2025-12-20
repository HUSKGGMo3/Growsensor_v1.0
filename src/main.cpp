#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <Wire.h>
#include <BH1750.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_MLX90614.h>
#include <MHZ19.h>
#include <math.h>
#include <ArduinoJson.h>
#include <cstring>
#include <time.h>
#include <sys/time.h>
#include <vector>
#include <esp_system.h>

// ----------------------------
// Pin and peripheral settings
// ----------------------------
static constexpr int DEFAULT_I2C_SDA_PIN = 21;
static constexpr int DEFAULT_I2C_SCL_PIN = 22;
static constexpr int DEFAULT_CO2_RX_PIN = 16; // ESP32 RX <- sensor TX
static constexpr int DEFAULT_CO2_TX_PIN = 17; // ESP32 TX -> sensor RX
static constexpr int PIN_MIN = 0;
static constexpr int PIN_MAX = 39;

int pinI2C_SDA = DEFAULT_I2C_SDA_PIN;
int pinI2C_SCL = DEFAULT_I2C_SCL_PIN;
int pinCO2_RX = DEFAULT_CO2_RX_PIN;
int pinCO2_TX = DEFAULT_CO2_TX_PIN;

// Access point settings
static const char *AP_SSID = "GrowSensor-Setup";
static const char *AP_PASSWORD = "growcontrol"; // keep non-empty for stability
static constexpr byte DNS_PORT = 53;

// Wi-Fi connect timeout (ms)
static constexpr unsigned long WIFI_TIMEOUT = 15000;

// Sensor refresh interval (ms)
static constexpr unsigned long SENSOR_INTERVAL = 2000;
static constexpr uint64_t HISTORY_BUCKET_5M_MS = 300000;      // 5-minute buckets
static constexpr uint64_t HISTORY_BUCKET_15M_MS = 900000;     // 15-minute buckets
static constexpr size_t HISTORY_6H_CAPACITY = 80;             // 6h @5min + buffer
static constexpr size_t HISTORY_24H_CAPACITY = 110;           // 24h @15min + buffer
static constexpr uint64_t HISTORY_LIVE_WINDOW_MS = 2ULL * 60ULL * 60ULL * 1000ULL; // last 2h
static constexpr unsigned long LEAF_STALE_MS = 300000;         // 5 minutes stale threshold
static constexpr float LEAF_DIFF_THRESHOLD = 5.0f;              // °C difference to mark IR sensor unhealthy
static constexpr size_t LOG_CAPACITY = 720;                     // ~6h if we log every 30s
static constexpr size_t LOG_JSON_LIMIT = 180;                   // cap JSON output to latest entries
static constexpr size_t LOG_HEAP_THRESHOLD = 24000;             // prune logs when free heap drops below ~24KB
static constexpr unsigned long STALL_LIMIT_MS = 4UL * 60UL * 60UL * 1000UL; // 4h stall watchdog
static constexpr unsigned long TIME_SYNC_RETRY_MS = 60000;      // retry NTP every 60s until synced
static constexpr unsigned long TIME_SYNC_REFRESH_MS = 300000;   // refresh offset every 5 minutes
static constexpr time_t TIME_VALID_AFTER = 1672531200;          // 2023-01-01 UTC safeguard

static const char *NTP_SERVER_1 = "pool.ntp.org";
static const char *NTP_SERVER_2 = "time.nist.gov";
static const char *NTP_SERVER_3 = "time.google.com";

String timezoneName = "Europe/Berlin";
bool timeSynced = false;
uint64_t epochOffsetMs = 0; // epoch_ms - millis() for stable mapping
unsigned long lastTimeSyncAttempt = 0;
unsigned long lastTimeSyncOk = 0;
// Auth
// Lux to PPFD conversion factors (approximate for common horticulture spectra)
enum class LightChannel {
  FullSpectrum,
  White,
  Bloom
};

enum class ClimateSensorType {
  SHT31,
  DHT22,
  BME280,
  BME680,
  SHT30,
  DS18B20
};

enum class Co2SensorType {
  MHZ19,
  MHZ14,
  SENSEAIR_S8,
  SCD30,
  SCD40,
  SCD41
};

float luxToPPFD(float lux, LightChannel channel) {
  // Factors derived from typical LED spectral profiles; adjust in UI if needed.
  switch (channel) {
  case LightChannel::FullSpectrum:
    return lux * 0.014f; // full-spectrum horticulture LEDs
  case LightChannel::White:
    return lux * 0.0157f; // cool/neutral white channels
  case LightChannel::Bloom:
    return lux * 0.0175f; // red-heavy bloom channels
  default:
    return lux * 0.015f;
  }
}

struct Telemetry {
  float lux = NAN;
  float ppfd = NAN;
  float ppfdFactor = NAN;
  float ambientTempC = NAN;
  float humidity = NAN;
  float leafTempC = NAN;
  int co2ppm = -1;
  float vpd = NAN;
  float vpdTargetLow = NAN;
  float vpdTargetHigh = NAN;
  int vpdStatus = 0; // -1 under, 0 in range, 1 over
};

struct SensorHealth {
  bool present = false;
  bool enabled = true;
  bool healthy = false;
  unsigned long lastUpdate = 0;
};

struct SensorStall {
  float lastValue = NAN;
  unsigned long lastChange = 0;
};

struct SensorSlot {
  String id;
  String type;
  String category;
  bool enabled = true;
  bool healthy = false;
  bool present = false;
  SensorHealth *health = nullptr;
  bool *enabledFlag = nullptr;
};

struct SensorTemplate {
  const char *type;
  const char *name;
  const char *category;
  const char *interfaceType; // i2c, uart, gpio
  int sda;
  int scl;
  int rx;
  int tx;
};

struct SensorConfig {
  String id;
  String type;
  String category;
  String name;
  String interfaceType;
  bool enabled = true;
  bool healthy = false;
  bool present = false;
  bool advancedPins = false;
  int sda = -1;
  int scl = -1;
  int rx = -1;
  int tx = -1;
};

const SensorTemplate SENSOR_TEMPLATES[] = {
    {"BH1750", "BH1750", "light", "i2c", DEFAULT_I2C_SDA_PIN,
     DEFAULT_I2C_SCL_PIN, -1, -1},
    {"TSL2591", "TSL2591", "light", "i2c", DEFAULT_I2C_SDA_PIN,
     DEFAULT_I2C_SCL_PIN, -1, -1},
    {"SHT31", "SHT31", "climate", "i2c", DEFAULT_I2C_SDA_PIN,
     DEFAULT_I2C_SCL_PIN, -1, -1},
    {"BME280", "BME280", "climate", "i2c", DEFAULT_I2C_SDA_PIN,
     DEFAULT_I2C_SCL_PIN, -1, -1},
    {"DHT22", "DHT22", "climate", "gpio", -1, -1, -1, -1},
    {"MLX90614", "MLX90614", "leaf", "i2c", DEFAULT_I2C_SDA_PIN,
     DEFAULT_I2C_SCL_PIN, -1, -1},
    {"MHZ19", "MH-Z19", "co2", "uart", -1, -1, DEFAULT_CO2_RX_PIN,
     DEFAULT_CO2_TX_PIN},
    {"MHZ14", "MH-Z14", "co2", "uart", -1, -1, DEFAULT_CO2_RX_PIN,
     DEFAULT_CO2_TX_PIN},
    {"SCD30", "SCD30", "co2", "i2c", DEFAULT_I2C_SDA_PIN,
     DEFAULT_I2C_SCL_PIN, -1, -1},
    {"SCD40", "SCD40", "co2", "i2c", DEFAULT_I2C_SDA_PIN,
     DEFAULT_I2C_SCL_PIN, -1, -1},
};
const size_t SENSOR_TEMPLATE_COUNT = sizeof(SENSOR_TEMPLATES) / sizeof(SENSOR_TEMPLATES[0]);

std::vector<SensorConfig> sensorConfigs;

struct Partner {
  String id;
  String name;
  String description;
  String url;
  String logo;
  bool enabled = true;
};

struct VpdProfile {
  const char *id;
  const char *label;
  float targetLow;   // recommended lower bound (kPa)
  float targetHigh;  // recommended upper bound (kPa)
};

struct HistoryPoint {
  uint64_t ts;
  float value;
};

struct HistorySeries {
  HistoryPoint midTerm[HISTORY_6H_CAPACITY];
  size_t midStart = 0;
  size_t midCount = 0;
  HistoryPoint longTerm[HISTORY_24H_CAPACITY];
  size_t longStart = 0;
  size_t longCount = 0;
  uint64_t bucket5mStart = 0;
  float bucket5mSum = 0.0f;
  uint16_t bucket5mCount = 0;
  uint64_t bucket15mStart = 0;
  float bucket15mSum = 0.0f;
  uint16_t bucket15mCount = 0;
};

// Globals
BH1750 lightMeter;
Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
HardwareSerial co2Serial(2);
MHZ19 co2Sensor;
Preferences prefs;
DNSServer dnsServer;
WebServer server(80);

// Wi-Fi config
String savedSsid;
String savedWifiPass;
bool apMode = false;
LightChannel channel = LightChannel::FullSpectrum;
ClimateSensorType climateType = ClimateSensorType::SHT31;
Co2SensorType co2Type = Co2SensorType::MHZ19;
Telemetry latest;
unsigned long lastSensorMillis = 0;

bool staticIpEnabled = false;
IPAddress staticIp;
IPAddress staticGateway;
IPAddress staticSubnet;

// Sensor health tracking
SensorHealth lightHealth;
SensorHealth climateHealth;
SensorHealth leafHealth;
SensorHealth co2Health;
std::vector<SensorSlot> sensors;
std::vector<Partner> partners;

void loadPartners();

// Logging buffer
String logBuffer[LOG_CAPACITY];
size_t logStart = 0;
size_t logCount = 0;
bool logPruneNoted = false;

// Forward declarations
void pruneLogsIfLowMemory(bool allowNotice = true);
void appendLogLine(const String &line);

// Sensor toggles (persisted)
bool enableLight = true;
bool enableClimate = true;
bool enableLeaf = true;
bool enableCo2 = true;

// VPD stage
String vpdStageId = "seedling";

// Stall tracking
SensorStall stallLight;
SensorStall stallClimateTemp;
SensorStall stallClimateHum;
SensorStall stallLeaf;
SensorStall stallCo2;

const VpdProfile VPD_PROFILES[] = {
    {"seedling", "Steckling/Sämling", 0.4f, 0.8f},
    {"veg", "Vegitativ", 0.8f, 1.2f},
    {"bloom", "Blütephase", 1.0f, 1.4f},       // early flower
    {"late_bloom", "Späteblüte", 1.2f, 1.6f},  // late flower
};

struct MetricDef {
  const char *id;
  const char *unit;
  uint8_t decimals;
  HistorySeries series;
};

MetricDef HISTORY_METRICS[] = {
    {"lux", "Lux", 1, HistorySeries()},
    {"ppfd", "µmol/m²/s", 1, HistorySeries()},
    {"co2", "ppm", 0, HistorySeries()},
    {"temp", "°C", 1, HistorySeries()},
    {"humidity", "%", 1, HistorySeries()},
    {"leaf", "°C", 1, HistorySeries()},
    {"vpd", "kPa", 3, HistorySeries()},
};
const size_t HISTORY_METRIC_COUNT = sizeof(HISTORY_METRICS) / sizeof(HISTORY_METRICS[0]);

// ----------------------------
// Helpers
// ----------------------------
String lightChannelName() {
  switch (channel) {
  case LightChannel::FullSpectrum:
    return "full_spectrum";
  case LightChannel::White:
    return "white";
  case LightChannel::Bloom:
    return "bloom";
  default:
    return "full_spectrum";
  }
}

LightChannel lightChannelFromString(const String &value) {
  if (value == "white")
    return LightChannel::White;
  if (value == "bloom")
    return LightChannel::Bloom;
  return LightChannel::FullSpectrum;
}

String climateSensorName(ClimateSensorType t) {
  switch (t) {
  case ClimateSensorType::SHT31:
    return "sht31";
  case ClimateSensorType::DHT22:
    return "dht22";
  case ClimateSensorType::BME280:
    return "bme280";
  case ClimateSensorType::BME680:
    return "bme680";
  case ClimateSensorType::SHT30:
    return "sht30";
  case ClimateSensorType::DS18B20:
    return "ds18b20";
  default:
    return "sht31";
  }
}

ClimateSensorType climateFromString(const String &v) {
  String lower = v;
  lower.toLowerCase();
  if (lower == "dht22") return ClimateSensorType::DHT22;
  if (lower == "bme280") return ClimateSensorType::BME280;
  if (lower == "bme680") return ClimateSensorType::BME680;
  if (lower == "sht30") return ClimateSensorType::SHT30;
  if (lower == "ds18b20") return ClimateSensorType::DS18B20;
  return ClimateSensorType::SHT31;
}

String co2SensorName(Co2SensorType t) {
  switch (t) {
  case Co2SensorType::MHZ19:
    return "mhz19";
  case Co2SensorType::MHZ14:
    return "mhz14";
  case Co2SensorType::SENSEAIR_S8:
    return "senseair_s8";
  case Co2SensorType::SCD30:
    return "scd30";
  case Co2SensorType::SCD40:
    return "scd40";
  case Co2SensorType::SCD41:
    return "scd41";
  default:
    return "mhz19";
  }
}

Co2SensorType co2FromString(const String &v) {
  String lower = v;
  lower.toLowerCase();
  if (lower == "mhz14") return Co2SensorType::MHZ14;
  if (lower == "senseair_s8") return Co2SensorType::SENSEAIR_S8;
  if (lower == "scd30") return Co2SensorType::SCD30;
  if (lower == "scd40") return Co2SensorType::SCD40;
  if (lower == "scd41") return Co2SensorType::SCD41;
  return Co2SensorType::MHZ19;
}

const VpdProfile &vpdProfileById(const String &id) {
  for (const auto &p : VPD_PROFILES) {
    if (id == p.id)
      return p;
  }
  return VPD_PROFILES[0];
}

MetricDef *historyById(const String &id) {
  for (size_t i = 0; i < HISTORY_METRIC_COUNT; i++) {
    if (id == HISTORY_METRICS[i].id)
      return &HISTORY_METRICS[i];
  }
  return nullptr;
}

void addPoint(HistoryPoint *buffer, size_t capacity, size_t &start, size_t &count, uint64_t ts, float value) {
  if (capacity == 0)
    return;
  if (count < capacity) {
    size_t idx = (start + count) % capacity;
    buffer[idx].ts = ts;
    buffer[idx].value = value;
    count++;
  } else {
    buffer[start].ts = ts;
    buffer[start].value = value;
    start = (start + 1) % capacity;
  }
}

void flushBucket(HistorySeries &series, bool fiveMinuteBucket, uint64_t ts) {
  const uint64_t bucketSize = fiveMinuteBucket ? HISTORY_BUCKET_5M_MS : HISTORY_BUCKET_15M_MS;
  uint64_t &bucketStart = fiveMinuteBucket ? series.bucket5mStart : series.bucket15mStart;
  float &bucketSum = fiveMinuteBucket ? series.bucket5mSum : series.bucket15mSum;
  uint16_t &bucketCount = fiveMinuteBucket ? series.bucket5mCount : series.bucket15mCount;
  HistoryPoint *targetBuf = fiveMinuteBucket ? series.midTerm : series.longTerm;
  size_t &targetStart = fiveMinuteBucket ? series.midStart : series.longStart;
  size_t &targetCount = fiveMinuteBucket ? series.midCount : series.longCount;
  const size_t targetCapacity = fiveMinuteBucket ? HISTORY_6H_CAPACITY : HISTORY_24H_CAPACITY;

  if (bucketStart == 0)
    return;

  const uint64_t maxAdvance = bucketSize * (fiveMinuteBucket ? HISTORY_6H_CAPACITY : HISTORY_24H_CAPACITY);
  if (ts > bucketStart + maxAdvance) {
    bucketStart = (ts / bucketSize) * bucketSize;
    bucketSum = 0.0f;
    bucketCount = 0;
    return;
  }

  while (ts >= bucketStart + bucketSize) {
    float avg = bucketCount > 0 ? (bucketSum / bucketCount) : NAN;
    addPoint(targetBuf, targetCapacity, targetStart, targetCount, bucketStart, avg);
    bucketStart += bucketSize;
    bucketSum = 0.0f;
    bucketCount = 0;
  }
}

void pushHistoryValue(const char *metric, float value, uint64_t ts) {
  MetricDef *def = historyById(metric);
  if (!def)
    return;
  float normalized = (!isnan(value) && value != -1.0f) ? value : NAN;
  uint64_t now = ts;
  if (def->series.bucket5mStart == 0) {
    def->series.bucket5mStart = (now / HISTORY_BUCKET_5M_MS) * HISTORY_BUCKET_5M_MS;
  }
  if (def->series.bucket15mStart == 0) {
    def->series.bucket15mStart = (now / HISTORY_BUCKET_15M_MS) * HISTORY_BUCKET_15M_MS;
  }
  flushBucket(def->series, true, now);
  flushBucket(def->series, false, now);
  if (!isnan(normalized)) {
    def->series.bucket5mSum += normalized;
    def->series.bucket5mCount++;
    def->series.bucket15mSum += normalized;
    def->series.bucket15mCount++;
  }
}

float safeFloat(float v, float fallback = 0.0f) { return isnan(v) ? fallback : v; }
int safeInt(int v, int fallback = 0) { return v < 0 ? fallback : v; }

float currentPpfdFactor() {
  return luxToPPFD(1.0f, channel);
}

bool enforceAuth() {
  return true;
}

void persistSensorFlags() {
  prefs.begin("grow-sensor", false);
  prefs.putBool("en_light", enableLight);
  prefs.putBool("en_climate", enableClimate);
  prefs.putBool("en_leaf", enableLeaf);
  prefs.putBool("en_co2", enableCo2);
  prefs.end();
}

bool validPin(int pin) { return pin >= PIN_MIN && pin <= PIN_MAX; }

bool pinsValid(int sda, int scl, int co2Rx, int co2Tx) {
  return validPin(sda) && validPin(scl) && validPin(co2Rx) && validPin(co2Tx) && sda != scl && co2Rx != co2Tx;
}

void loadPinsFromPrefs() {
  prefs.begin("grow-sensor", true);
  pinI2C_SDA = prefs.getInt("i2c_sda", DEFAULT_I2C_SDA_PIN);
  pinI2C_SCL = prefs.getInt("i2c_scl", DEFAULT_I2C_SCL_PIN);
  pinCO2_RX = prefs.getInt("co2_rx", DEFAULT_CO2_RX_PIN);
  pinCO2_TX = prefs.getInt("co2_tx", DEFAULT_CO2_TX_PIN);
  prefs.end();

  if (!pinsValid(pinI2C_SDA, pinI2C_SCL, pinCO2_RX, pinCO2_TX)) {
    pinI2C_SDA = DEFAULT_I2C_SDA_PIN;
    pinI2C_SCL = DEFAULT_I2C_SCL_PIN;
    pinCO2_RX = DEFAULT_CO2_RX_PIN;
    pinCO2_TX = DEFAULT_CO2_TX_PIN;
  }
}

void appendLogLine(const String &line) {
  size_t idx = (logStart + logCount) % LOG_CAPACITY;
  if (logCount == LOG_CAPACITY) {
    logStart = (logStart + 1) % LOG_CAPACITY;
    idx = (logStart + logCount - 1) % LOG_CAPACITY;
  } else {
    logCount++;
  }
  logBuffer[idx] = line;
}

void pruneLogsIfLowMemory(bool allowNotice) {
  static bool pruning = false;
  if (pruning)
    return;
  pruning = true;
  size_t freeHeap = ESP.getFreeHeap();
  bool lowHeap = freeHeap < LOG_HEAP_THRESHOLD;
  bool nearFull = logCount >= LOG_CAPACITY - 8;
  if (lowHeap || nearFull) {
    size_t drop = 0;
    if (lowHeap) {
      drop = logCount > 1 ? (logCount / 2) : (logCount > 0 ? 1 : 0);
    } else if (nearFull && logCount > 0) {
      drop = (logCount / 8) + 1;
    }
    if (drop > 0) {
      logStart = (logStart + drop) % LOG_CAPACITY;
      logCount -= drop;
    }
    if (allowNotice && lowHeap && !logPruneNoted) {
      logPruneNoted = true;
      char buf[96];
      snprintf(buf, sizeof(buf), "%lus: Log pruned due to low heap", millis() / 1000);
      appendLogLine(String(buf));
    }
  } else if (!lowHeap) {
    logPruneNoted = false;
  }
  pruning = false;
}

void logEvent(const String &msg) {
  pruneLogsIfLowMemory(true);
  String line;
  line.reserve(msg.length() + 12);
  line = String(millis() / 1000);
  line += "s: ";
  line += msg;
  appendLogLine(line);
}

void applyTimezoneEnv() {
  const String tz = timezoneName.length() > 0 ? timezoneName : "UTC";
  setenv("TZ", tz.c_str(), 1);
  tzset();
}

bool captureTimeOffset() {
  struct timeval tv;
  if (gettimeofday(&tv, nullptr) != 0) return false;
  if (tv.tv_sec < TIME_VALID_AFTER) return false;
  uint64_t nowMs = (uint64_t)tv.tv_sec * 1000ULL + (tv.tv_usec / 1000ULL);
  uint64_t mono = millis();
  epochOffsetMs = nowMs > mono ? (nowMs - mono) : 0;
  timeSynced = true;
  lastTimeSyncOk = millis();
  return true;
}

void startNtpSync() {
  if (WiFi.status() != WL_CONNECTED)
    return;
  applyTimezoneEnv();
  configTzTime(timezoneName.c_str(), NTP_SERVER_1, NTP_SERVER_2, NTP_SERVER_3);
  lastTimeSyncAttempt = millis();
}

void maintainTimeSync() {
  if (WiFi.status() != WL_CONNECTED)
    return;
  unsigned long now = millis();
  bool needKick = (!timeSynced && (now - lastTimeSyncAttempt >= TIME_SYNC_RETRY_MS)) || lastTimeSyncAttempt == 0;
  if (needKick) {
    startNtpSync();
  }
  bool shouldRefresh = (!timeSynced && now - lastTimeSyncAttempt >= 1500) || (timeSynced && (now - lastTimeSyncOk >= TIME_SYNC_REFRESH_MS));
  if (shouldRefresh) {
    bool wasSynced = timeSynced;
    bool ok = captureTimeOffset();
    if (ok && !wasSynced) {
      logEvent("Time synced via NTP");
    }
  }
}

uint64_t mapTimestampToEpochMs(uint64_t monotonicTs) {
  if (!timeSynced) return monotonicTs;
  return epochOffsetMs + monotonicTs;
}

uint64_t currentEpochMs() {
  return mapTimestampToEpochMs(millis());
}

void loadPartners() {
  partners.clear();
  prefs.begin("grow-sensor", true);
  String raw = prefs.getString("partners", "[]");
  prefs.end();
  DynamicJsonDocument doc(1024);
  if (deserializeJson(doc, raw) != DeserializationError::Ok) return;
  JsonArray arr = doc.as<JsonArray>();
  for (JsonObject obj : arr) {
    Partner p;
    p.id = obj["id"] | "";
    p.name = obj["name"] | "";
    p.description = obj["desc"] | "";
    p.url = obj["url"] | "";
    p.logo = obj["logo"] | "";
    p.enabled = obj["en"] | true;
    if (p.id.length() > 0) partners.push_back(p);
  }
}

void rebuildSensorList() {
  sensors.clear();
  auto addSensor = [&](const String &id, const String &type, const String &cat, bool enabled, SensorHealth &h, bool &flag) {
    SensorSlot s;
    s.id = id;
    s.type = type;
    s.category = cat;
    s.enabled = enabled;
    s.healthy = h.healthy;
    s.present = h.present;
    s.health = &h;
    s.enabledFlag = &flag;
    sensors.push_back(s);
  };

  addSensor("lux", "BH1750", "light", enableLight, lightHealth, enableLight);
  addSensor("climate", climateSensorName(climateType), "climate", enableClimate, climateHealth, enableClimate);
  addSensor("leaf", "MLX90614", "leaf", enableLeaf, leafHealth, enableLeaf);
  addSensor("co2", co2SensorName(co2Type), "co2", enableCo2, co2Health, enableCo2);
}

SensorSlot *findSensor(const String &id) {
  for (auto &s : sensors) {
    if (s.id == id) return &s;
  }
  return nullptr;
}

const SensorTemplate *findTemplate(const String &type) {
  for (size_t i = 0; i < SENSOR_TEMPLATE_COUNT; i++) {
    if (type.equalsIgnoreCase(SENSOR_TEMPLATES[i].type)) return &SENSOR_TEMPLATES[i];
  }
  return nullptr;
}

void saveSensorConfigs() {
  DynamicJsonDocument doc(2048);
  JsonArray arr = doc.to<JsonArray>();
  for (const auto &cfg : sensorConfigs) {
    JsonObject o = arr.createNestedObject();
    o["id"] = cfg.id;
    o["type"] = cfg.type;
    o["category"] = cfg.category;
    o["name"] = cfg.name;
    o["iface"] = cfg.interfaceType;
    o["enabled"] = cfg.enabled;
    o["advanced"] = cfg.advancedPins;
    if (cfg.sda >= 0) o["sda"] = cfg.sda;
    if (cfg.scl >= 0) o["scl"] = cfg.scl;
    if (cfg.rx >= 0) o["rx"] = cfg.rx;
    if (cfg.tx >= 0) o["tx"] = cfg.tx;
  }
  String out;
  serializeJson(arr, out);
  prefs.begin("grow-sensor", false);
  prefs.putString("sensors_cfg", out);
  prefs.end();
}

void applyPinsFromConfig() {
  bool appliedI2C = false;
  bool appliedUart = false;
  for (const auto &cfg : sensorConfigs) {
    if (cfg.advancedPins) {
      if (cfg.interfaceType == "i2c" && cfg.sda >= 0 && cfg.scl >= 0) {
        pinI2C_SDA = cfg.sda;
        pinI2C_SCL = cfg.scl;
        appliedI2C = true;
      }
      if (cfg.interfaceType == "uart" && cfg.rx >= 0 && cfg.tx >= 0) {
        pinCO2_RX = cfg.rx;
        pinCO2_TX = cfg.tx;
        appliedUart = true;
      }
    }
  }
  if (!appliedI2C) {
    pinI2C_SDA = DEFAULT_I2C_SDA_PIN;
    pinI2C_SCL = DEFAULT_I2C_SCL_PIN;
  }
  if (!appliedUart) {
    pinCO2_RX = DEFAULT_CO2_RX_PIN;
    pinCO2_TX = DEFAULT_CO2_TX_PIN;
  }
}

void loadSensorConfigs() {
  sensorConfigs.clear();
  prefs.begin("grow-sensor", true);
  String raw = prefs.getString("sensors_cfg", "");
  prefs.end();
  if (raw.length() == 0) {
    SensorConfig l; l.id="lux"; l.type="BH1750"; l.category="light"; l.name="BH1750"; l.interfaceType="i2c"; l.enabled=enableLight; l.sda=pinI2C_SDA; l.scl=pinI2C_SCL;
    SensorConfig c; c.id="climate"; c.type=climateSensorName(climateType); c.category="climate"; c.name="Klima"; c.interfaceType="i2c"; c.enabled=enableClimate; c.sda=pinI2C_SDA; c.scl=pinI2C_SCL;
    SensorConfig lf; lf.id="leaf"; lf.type="MLX90614"; lf.category="leaf"; lf.name="Leaf"; lf.interfaceType="i2c"; lf.enabled=enableLeaf; lf.sda=pinI2C_SDA; lf.scl=pinI2C_SCL;
    SensorConfig co; co.id="co2"; co.type=co2SensorName(co2Type); co.category="co2"; co.name="CO₂"; co.interfaceType="uart"; co.enabled=enableCo2; co.rx=pinCO2_RX; co.tx=pinCO2_TX;
    sensorConfigs.push_back(l); sensorConfigs.push_back(c); sensorConfigs.push_back(lf); sensorConfigs.push_back(co);
    saveSensorConfigs();
    return;
  }
  DynamicJsonDocument doc(4096);
  if (deserializeJson(doc, raw) != DeserializationError::Ok) return;
  for (JsonObject obj : doc.as<JsonArray>()) {
    SensorConfig cfg;
    cfg.id = obj["id"] | "";
    cfg.type = obj["type"] | "";
    cfg.category = obj["category"] | "";
    cfg.name = obj["name"] | cfg.type;
    cfg.interfaceType = obj["iface"] | "";
    cfg.enabled = obj["enabled"] | true;
    cfg.advancedPins = obj["advanced"] | false;
    cfg.sda = obj["sda"] | -1;
    cfg.scl = obj["scl"] | -1;
    cfg.rx = obj["rx"] | -1;
    cfg.tx = obj["tx"] | -1;
    sensorConfigs.push_back(cfg);
    if (cfg.id == "lux") enableLight = cfg.enabled;
    if (cfg.id == "climate") enableClimate = cfg.enabled;
    if (cfg.id == "leaf") enableLeaf = cfg.enabled;
    if (cfg.id == "co2") enableCo2 = cfg.enabled;
  }
  applyPinsFromConfig();
}

// ----------------------------
// Wi-Fi handling
// ----------------------------
void startAccessPoint() {
  apMode = true;
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  delay(100); // give the AP time to start

  IPAddress apIP = WiFi.softAPIP();
  dnsServer.start(DNS_PORT, "*", apIP);
  Serial.printf("[AP] Started %s (%s)\n", AP_SSID, apIP.toString().c_str());
  logEvent("AP mode active: " + String(AP_SSID));
}

bool connectToWiFi() {
  prefs.begin("grow-sensor", false);
  savedSsid = prefs.getString("ssid", "");
  savedWifiPass = prefs.getString("wifi_pass", "");
  bool hasWifiPassKey = prefs.isKey("wifi_pass");
  bool hasLegacyPassKey = prefs.isKey("pass");
  String legacyPass = hasLegacyPassKey ? prefs.getString("pass", "") : "";
  channel = lightChannelFromString(prefs.getString("channel", "full_spectrum"));
  climateType = climateFromString(prefs.getString("climate_type", "sht31"));
  co2Type = co2FromString(prefs.getString("co2_type", "mhz19"));
  staticIpEnabled = prefs.getBool("static", false);
  staticIp.fromString(prefs.getString("ip", ""));
  staticGateway.fromString(prefs.getString("gw", ""));
  staticSubnet.fromString(prefs.getString("sn", ""));
  timezoneName = prefs.getString("timezone", timezoneName);
  enableLight = prefs.getBool("en_light", true);
  enableClimate = prefs.getBool("en_climate", true);
  enableLeaf = prefs.getBool("en_leaf", true);
  enableCo2 = prefs.getBool("en_co2", true);
  vpdStageId = prefs.getString("vpd_stage", "seedling");
  bool migrateLegacyWifiPass = !hasWifiPassKey && hasLegacyPassKey && savedWifiPass.isEmpty() && !legacyPass.isEmpty() && !savedSsid.isEmpty();
  if (migrateLegacyWifiPass) {
    savedWifiPass = legacyPass;
    prefs.putString("wifi_pass", savedWifiPass);
  }
  prefs.end();
  applyTimezoneEnv();
  loadPartners();
  loadSensorConfigs();
  rebuildSensorList();

  if (savedSsid.isEmpty()) {
    Serial.println("[WiFi] No stored credentials, starting AP");
    startAccessPoint();
    return false;
  }

  WiFi.mode(WIFI_STA);
  if (staticIpEnabled && staticIp != IPAddress((uint32_t)0) && staticGateway != IPAddress((uint32_t)0) && staticSubnet != IPAddress((uint32_t)0)) {
    WiFi.config(staticIp, staticGateway, staticSubnet);
  }
  WiFi.begin(savedSsid.c_str(), savedWifiPass.c_str());
  Serial.printf("[WiFi] Connecting to %s ...\n", savedSsid.c_str());

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_TIMEOUT) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    apMode = false;
    Serial.printf("[WiFi] Connected, IP: %s\n", WiFi.localIP().toString().c_str());
    logEvent("WiFi connected: " + WiFi.localIP().toString());
    startNtpSync();
    return true;
  }

  Serial.println("[WiFi] Connection failed, starting AP");
  logEvent("WiFi connection failed, switching to AP");
  startAccessPoint();
  return false;
}

void saveWifiCredentials(const String &ssid, const String &pass) {
  prefs.begin("grow-sensor", false);
  prefs.putString("ssid", ssid);
  prefs.putString("wifi_pass", pass);
  prefs.end();
  savedSsid = ssid;
  savedWifiPass = pass;
}

void clearPreferences() {
  prefs.begin("grow-sensor", false);
  prefs.clear();
  prefs.end();
  logEvent("Preferences cleared");
}

void savePartners() {
  DynamicJsonDocument doc(1024);
  JsonArray arr = doc.to<JsonArray>();
  for (const auto &p : partners) {
    JsonObject obj = arr.createNestedObject();
    obj["id"] = p.id;
    obj["name"] = p.name;
    obj["desc"] = p.description;
    obj["url"] = p.url;
    obj["logo"] = p.logo;
    obj["en"] = p.enabled;
  }
  String out;
  serializeJson(arr, out);
  prefs.begin("grow-sensor", false);
  prefs.putString("partners", out);
  prefs.end();
}

// ----------------------------
// Sensor handling
// ----------------------------
void reinitLightSensor() {
  lightHealth.healthy = false;
  lightHealth.enabled = true;
  lightHealth.present = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);
}

void reinitClimateSensor() {
  climateHealth.healthy = false;
  climateHealth.enabled = true;
  if (climateType == ClimateSensorType::SHT31) {
    climateHealth.present = sht31.begin(0x44);
  } else {
    climateHealth.present = true;
  }
}

void reinitLeafSensor() {
  leafHealth.healthy = false;
  leafHealth.enabled = true;
  leafHealth.present = mlx.begin();
}

void reinitCo2Sensor() {
  co2Health.healthy = false;
  co2Health.enabled = true;
  if (co2Type == Co2SensorType::MHZ19 || co2Type == Co2SensorType::MHZ14) {
    co2Serial.begin(9600, SERIAL_8N1, pinCO2_RX, pinCO2_TX);
    co2Sensor.begin(co2Serial);
    co2Sensor.autoCalibration(false);
    co2Health.present = true;
  } else {
    co2Health.present = true;
  }
}

void initSensors() {
  Wire.begin(pinI2C_SDA, pinI2C_SCL);

  reinitLightSensor();
  Serial.println(lightHealth.present ? "[Sensor] BH1750 initialized" : "[Sensor] BH1750 not detected");
  reinitClimateSensor();
  Serial.println(climateHealth.present ? "[Sensor] Climate initialized" : "[Sensor] Climate not detected");
  reinitLeafSensor();
  Serial.println(leafHealth.present ? "[Sensor] MLX90614 initialized" : "[Sensor] MLX90614 not detected");
  reinitCo2Sensor();
  Serial.println(co2Health.present ? "[Sensor] CO2 initialized" : "[Sensor] CO2 not detected");

  rebuildSensorList();
}

void readSensors() {
  if (!enableLight) {
    lightHealth.healthy = false;
    lightHealth.enabled = false;
    latest.lux = NAN;
    latest.ppfd = NAN;
  }
  if (!enableClimate) {
    climateHealth.healthy = false;
    climateHealth.enabled = false;
    latest.ambientTempC = NAN;
    latest.humidity = NAN;
  }
  if (!enableLeaf) {
    leafHealth.healthy = false;
    leafHealth.enabled = false;
    latest.leafTempC = NAN;
  }
  if (!enableCo2) {
    co2Health.healthy = false;
    co2Health.enabled = false;
    latest.co2ppm = -1;
  }

  latest.ppfdFactor = currentPpfdFactor();

  if (enableLight && lightHealth.present && lightMeter.measurementReady()) {
    latest.lux = lightMeter.readLightLevel();
    latest.ppfdFactor = currentPpfdFactor();
    latest.ppfd = luxToPPFD(latest.lux, channel);
    lightHealth.healthy = !isnan(latest.lux);
    lightHealth.lastUpdate = millis();
    if (!isnan(latest.lux)) {
      if (isnan(stallLight.lastValue) || fabsf(stallLight.lastValue - latest.lux) > 0.01f) {
        stallLight.lastValue = latest.lux;
        stallLight.lastChange = millis();
      }
    }
  }

  float temp = NAN;
  float humidity = NAN;
  if (enableClimate && climateHealth.present) {
    if (climateType == ClimateSensorType::SHT31) {
      temp = sht31.readTemperature();
      humidity = sht31.readHumidity();
    } else {
      // Other climate sensors not implemented yet
      temp = NAN;
      humidity = NAN;
    }
  }
  if (!isnan(temp) && !isnan(humidity)) {
    latest.ambientTempC = temp;
    latest.humidity = humidity;
    climateHealth.healthy = true;
    climateHealth.lastUpdate = millis();
    if (isnan(stallClimateTemp.lastValue) || fabsf(stallClimateTemp.lastValue - temp) > 0.01f) {
      stallClimateTemp.lastValue = temp;
      stallClimateTemp.lastChange = millis();
    }
    if (isnan(stallClimateHum.lastValue) || fabsf(stallClimateHum.lastValue - humidity) > 0.01f) {
      stallClimateHum.lastValue = humidity;
      stallClimateHum.lastChange = millis();
    }
  } else {
    climateHealth.healthy = false;
  }

  double objTemp = enableLeaf && leafHealth.present ? mlx.readObjectTempC() : NAN;
  if (!isnan(objTemp)) {
    latest.leafTempC = objTemp;
    leafHealth.healthy = true;
    leafHealth.lastUpdate = millis();
    if (isnan(stallLeaf.lastValue) || fabsf(stallLeaf.lastValue - objTemp) > 0.01f) {
      stallLeaf.lastValue = objTemp;
      stallLeaf.lastChange = millis();
    }
  } else {
    leafHealth.healthy = false;
  }

  if (enableCo2 && co2Health.present) {
    if (co2Type == Co2SensorType::MHZ19 || co2Type == Co2SensorType::MHZ14) {
      int ppm = co2Sensor.getCO2();
      if (ppm > 0 && ppm < 5000) {
        latest.co2ppm = ppm;
        co2Health.healthy = true;
        co2Health.lastUpdate = millis();
        if (stallCo2.lastValue != ppm) {
          stallCo2.lastValue = ppm;
          stallCo2.lastChange = millis();
        }
      } else {
        co2Health.healthy = false;
      }
    } else {
      co2Health.healthy = false; // unsupported sensor type
    }
  }

  // Compute VPD when data is valid and IR sensor is not drifting excessively
  float leafForVpd = latest.leafTempC;
  if (!leafHealth.healthy && !isnan(latest.ambientTempC)) {
    // Fallback: approximate leaf temp as ambient -2°C if IR sensor failed
    leafForVpd = latest.ambientTempC - 2.0f;
    latest.leafTempC = leafForVpd;
  }

  bool leafFresh = leafHealth.healthy || (!isnan(leafForVpd) && !isnan(latest.ambientTempC));
  bool leafAligned = !isnan(leafForVpd) && !isnan(latest.ambientTempC) && fabs(leafForVpd - latest.ambientTempC) <= LEAF_DIFF_THRESHOLD;
  if (leafFresh && leafAligned && !isnan(latest.humidity) && !isnan(latest.ambientTempC)) {
    float satVP = 0.6108f * expf((17.27f * latest.ambientTempC) / (latest.ambientTempC + 237.3f)); // kPa
    latest.vpd = (1.0f - (latest.humidity / 100.0f)) * satVP;
  } else {
    latest.vpd = NAN;
  }

  const VpdProfile &profile = vpdProfileById(vpdStageId);
  latest.vpdTargetLow = profile.targetLow;
  latest.vpdTargetHigh = profile.targetHigh;
  if (isnan(latest.vpd)) {
    latest.vpdStatus = 0;
  } else if (latest.vpd < latest.vpdTargetLow) {
    latest.vpdStatus = -1;
  } else if (latest.vpd > latest.vpdTargetHigh) {
    latest.vpdStatus = 1;
  } else {
    latest.vpdStatus = 0;
  }

  uint64_t sampleTs = currentEpochMs();
  pushHistoryValue("lux", latest.lux, sampleTs);
  pushHistoryValue("ppfd", latest.ppfd, sampleTs);
  pushHistoryValue("co2", static_cast<float>(latest.co2ppm), sampleTs);
  pushHistoryValue("temp", latest.ambientTempC, sampleTs);
  pushHistoryValue("humidity", latest.humidity, sampleTs);
  pushHistoryValue("leaf", latest.leafTempC, sampleTs);
  pushHistoryValue("vpd", latest.vpd, sampleTs);

  unsigned long now = millis();
  auto stallCheck = [&](SensorStall &stall, SensorHealth &health, bool &enabled, const char *name) {
    if (stall.lastChange == 0)
      stall.lastChange = now;
    if (now - stall.lastChange > STALL_LIMIT_MS && enabled) {
      enabled = false;
      health.healthy = false;
      persistSensorFlags();
      logEvent(String(name) + " disabled (stalled >4h)");
    }
  };

  stallCheck(stallLight, lightHealth, enableLight, "Light");
  stallCheck(stallClimateTemp, climateHealth, enableClimate, "Climate");
  stallCheck(stallLeaf, leafHealth, enableLeaf, "Leaf");
  stallCheck(stallCo2, co2Health, enableCo2, "CO2");
  rebuildSensorList();
}

// ----------------------------
// Web server and API
// ----------------------------
static const char INDEX_HTML[] PROGMEM = R"HTML(
  <!doctype html>
  <html lang="de">
  <head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1" />
    <title>GrowSensor v0.2.6</title>
    <style>
      :root { color-scheme: light dark; }
      html, body { background: #0f172a; color: #e2e8f0; min-height: 100%; }
      body { font-family: system-ui, sans-serif; margin: 0; padding: 0; }
      header { padding: 16px; background: #111827; box-shadow: 0 2px 6px rgba(0,0,0,0.25); position: sticky; top: 0; z-index: 10; }
      h1 { margin: 0; font-size: 1.2rem; position:relative; }
      .header-row { display:flex; justify-content: space-between; align-items: center; gap:12px; flex-wrap: wrap; }
      .conn { display:flex; align-items:center; gap:8px; }
      .led { width:12px; height:12px; border-radius:50%; background:#ef4444; box-shadow:0 0 0 3px rgba(239,68,68,0.25); display:inline-block; }
      .badge { display:inline-block; padding:4px 8px; border-radius:999px; font-size:0.8rem; background:#1f2937; color:#cbd5e1; }
      .badge-dev { background:#10b981; color:#0f172a; }
      .badge-warn { background:#f59e0b; color:#0f172a; }
      .ghost { background:transparent; border:1px solid #1f2937; color:#e2e8f0; }
      main { padding: 16px; display: grid; gap: 12px; }
      .card { background: #111827; border: 1px solid #1f2937; border-radius: 12px; padding: 16px; box-shadow: 0 4px 8px rgba(0,0,0,0.25); position: relative; overflow:hidden; }
      .grid { display: grid; gap: 12px; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); }
      .value { font-size: 1.6rem; font-variant-numeric: tabular-nums; }
      label { display: block; margin-top: 8px; font-size: 0.9rem; }
      input, select, button { width: 100%; padding: 10px; margin-top: 4px; border-radius: 8px; border: 1px solid #1f2937; background: #0b1220; color: #e2e8f0; }
      button { cursor: pointer; border: none; background: linear-gradient(120deg, #22d3ee, #6366f1); color: #0b1220; font-weight: 600; }
      button:disabled { opacity: 0.6; cursor: not-allowed; }
      .row { display: flex; gap: 8px; align-items: center; flex-wrap: wrap; }
      .row > * { flex: 1; }
      .status { font-weight: 600; }
      .status.ok { color: #34d399; }
      .status.err { color: #f87171; }
      .logbox { width: 100%; min-height: 180px; font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace; white-space: pre-wrap; }
      .chart { position: relative; height: 200px; }
      canvas { width: 100%; height: 200px; }
      footer { text-align: center; padding: 12px; font-size: 0.85rem; color: #94a3b8; }
      .card-header { display:flex; align-items:center; justify-content: space-between; gap:8px; }
      .status-dot { width:12px; height:12px; border-radius:50%; background:#6b7280; box-shadow:0 0 0 3px rgba(107,114,128,0.25); }
      .tile-title { display:flex; align-items:center; gap:8px; }
      .tile-header { display:flex; align-items:center; justify-content:space-between; gap:8px; margin-bottom:6px; }
      .metric-tile { cursor: pointer; position:relative; transition: max-height 220ms ease, transform 180ms ease, opacity 180ms ease, border-color 120ms ease; overflow:hidden; max-height:720px; padding-bottom:48px; }
      .metric-tile:hover { transform: translateY(-2px); border-color: #334155; }
      .tile-body { position:relative; transition:max-height 240ms ease, opacity 200ms ease, transform 200ms ease; max-height:720px; opacity:1; transform:translateY(0); overflow:hidden; min-height:156px; display:flex; flex-direction:column; gap:6px; }
      .metric-tile.collapsed { max-height:96px; min-height:64px; padding-bottom:16px; opacity:0.98; transform:translateY(0); }
      .metric-tile.collapsed .tile-body { max-height:0; opacity:0; transform:translateY(-6px); display:none; }
      .metric-tile.collapsed .hover-chart, .metric-tile.collapsed:hover .hover-chart { display:none !important; pointer-events:none; }
      .metric-tile.collapsed .tile-header { margin-bottom:0; }
      .metric-tile.collapsed:hover { transform:none; }
      .hover-chart { position:absolute; inset:0; padding:12px; background:rgba(15,23,42,0.96); border:1px solid #1f2937; border-radius:10px; display:flex; align-items:stretch; justify-content:stretch; pointer-events:none; z-index:4; opacity:0; visibility:hidden; transition:opacity 140ms ease; }
      .metric-tile:hover .hover-chart { opacity:1; visibility:visible; }
      .tile-eye { position:absolute; left:12px; bottom:12px; width:26px; height:26px; border-radius:50%; border:1px solid #1f2937; background:#0b1220; color:#e2e8f0; display:inline-flex; align-items:center; justify-content:center; gap:2px; padding:0; box-shadow:0 4px 8px rgba(0,0,0,0.18); transition:border-color 140ms ease, background 140ms ease, transform 140ms ease, box-shadow 160ms ease; }
      .tile-eye:hover { border-color:#334155; background:#111827; transform:translateY(-1px); box-shadow:0 6px 14px rgba(0,0,0,0.3); }
      .tile-eye:active { transform:translateY(0); }
      .tile-eye svg { width:18px; height:18px; display:block; }
      .tile-eye .eye-closed { display:none; }
      .metric-tile.collapsed .tile-eye .eye-open { display:none; }
      .metric-tile.collapsed .tile-eye .eye-closed { display:block; }
      .hover-chart canvas { width:100%; height:100%; display:block; flex:1 1 auto; }
      .dev-note { color:#f59e0b; font-size:0.9rem; margin-top:6px; }
      #devModal { position:fixed; inset:0; display:none; align-items:center; justify-content:center; background:rgba(0,0,0,0.6); z-index:60; }
      .hover-hint { font-size:0.85rem; color:#94a3b8; margin-top:6px; }
      nav { display:flex; gap:8px; flex-wrap:wrap; margin-top:10px; }
      nav button { width:auto; padding:8px 12px; position:relative; overflow:hidden; }
      nav button::before, nav button::after { content:""; position:absolute; inset:-30%; background:radial-gradient(circle, rgba(34,197,94,0.4) 0, rgba(34,197,94,0.0) 60%); opacity:0; transform:scale(0.6); transition:opacity 180ms ease, transform 220ms ease; pointer-events:none; mix-blend-mode:screen; }
      nav button::after { inset:-50%; animation: drift 3s linear infinite; }
      nav button:hover::before, nav button:hover::after { opacity:1; transform:scale(1); }
      @keyframes drift { from { transform: translate3d(-8px,-6px,0) scale(1); } 50% { transform: translate3d(8px,8px,0) scale(1.05);} to { transform: translate3d(-6px,6px,0) scale(1);} }
      .view { display:none; flex-direction:column; gap:12px; }
      .view.active { display:flex; }
      .menu-btn { background:#1f2937; color:#e2e8f0; border:1px solid #1f2937; }
      .wifi-status { display:flex; align-items:center; gap:8px; margin-bottom:8px; }
      .pulse { animation:pulse 1.8s ease-in-out infinite; }
      @keyframes pulse { 0% { box-shadow:0 0 0 0 rgba(52,211,153,0.45); } 70% { box-shadow:0 0 0 10px rgba(52,211,153,0); } 100% { box-shadow:0 0 0 0 rgba(52,211,153,0); } }
      .hidden { display:none; }
      .vpd-chart { position:relative; height:180px; border-radius:12px; overflow:hidden; background:#0b1220; border:1px solid #1f2937; }
      .vpd-canvas { position:absolute; inset:0; }
      .vpd-overlay { position:absolute; inset:0; pointer-events:none; display:flex; flex-direction:column; justify-content:space-between; padding:8px; font-size:0.9rem; color:#e2e8f0; }
      .vpd-legend { display:flex; gap:8px; align-items:center; flex-wrap:wrap; }
      .legend-swatch { width:16px; height:10px; border-radius:4px; display:inline-block; }
      .vpd-marker { width:12px; height:12px; border-radius:50%; border:2px solid #0f172a; box-shadow:0 0 0 4px rgba(14,165,233,0.35); background:#fbbf24; }
      .vpd-no-data { position:absolute; inset:0; display:flex; align-items:center; justify-content:center; color:#94a3b8; backdrop-filter:blur(1px); background:rgba(15,23,42,0.65); }
      .sensor-card { border:1px solid #1f2937; border-radius:10px; padding:10px; margin-bottom:8px; background:#0b1220; }
      .sensor-card .row { align-items:flex-start; }
      .sensor-desc { color:#94a3b8; font-size:0.9rem; }
      .sensor-grid { display:grid; grid-template-columns:repeat(auto-fit,minmax(220px,1fr)); gap:12px; }
      .sensor-tile { border:1px solid #1f2937; border-radius:12px; padding:12px; background:#0b1220; display:flex; flex-direction:column; gap:6px; cursor:pointer; transition: transform 120ms ease, border-color 120ms ease; }
      .sensor-tile:hover { transform: translateY(-2px); border-color:#334155; }
      .sensor-status { display:flex; align-items:center; gap:8px; font-size:0.95rem; color:#cbd5e1; }
      .sensor-status .status-dot { width:10px; height:10px; }
      .sensor-actions { display:flex; gap:8px; flex-wrap:wrap; }
      .tag { display:inline-block; padding:3px 8px; border-radius:999px; font-size:0.8rem; background:#1f2937; color:#cbd5e1; }
      #chartModal { position:fixed; inset:0; background:rgba(0,0,0,0.6); display:none; align-items:center; justify-content:center; z-index:80; }
      #chartModal .modal-card { background:#0b1220; border:1px solid #1f2937; border-radius:14px; padding:16px; width:90%; max-width:520px; box-shadow:0 10px 28px rgba(0,0,0,0.45); display:flex; flex-direction:column; gap:12px; }
      #sensorWizard { position:fixed; inset:0; display:none; align-items:center; justify-content:center; background:rgba(15,23,42,0.55); backdrop-filter:blur(2px); z-index:80; transition:opacity 140ms ease; opacity:0; }
      #sensorWizard.active { display:flex; opacity:1; }
      #sensorWizard .modal-card { background:#0b1220; border:1px solid #1f2937; border-radius:14px; padding:16px; width:92%; max-width:640px; box-shadow:0 16px 32px rgba(0,0,0,0.5); transform:scale(0.94); opacity:0; transition:transform 160ms ease, opacity 160ms ease; }
      #sensorWizard.active .modal-card { transform:scale(1); opacity:1; }
      .modal-tabs { display:flex; gap:8px; }
      .modal-tabs button { flex:1; padding:10px; background:#1f2937; border:1px solid #1f2937; color:#e2e8f0; }
      .modal-tabs button.active { background:linear-gradient(120deg,#22d3ee,#6366f1); color:#0b1220; border-color:transparent; }
      .modal-close { position:absolute; top:10px; right:12px; background:transparent; color:#e2e8f0; border:none; font-size:1.2rem; cursor:pointer; }
      .wizard-step { display:none; flex-direction:column; gap:10px; }
      .wizard-step.active { display:flex; }
      .pill { padding:4px 8px; border-radius:999px; background:#1f2937; color:#cbd5e1; font-size:0.85rem; display:inline-flex; gap:6px; align-items:center; }
      .chart-empty { position:absolute; inset:0; display:flex; align-items:center; justify-content:center; color:#94a3b8; background:rgba(15,23,42,0.6); }
      .axis-label { font-size:0.9rem; color:#94a3b8; }
      .wifi-bars { display:flex; gap:4px; align-items:flex-end; height:16px; }
      .wifi-bar { width:4px; background:#334155; border-radius:2px; }
      .wifi-bar.active { background:#22c55e; }
      .error-banner { position: sticky; top: 0; z-index: 90; background: #7f1d1d; color: #fecdd3; padding: 8px 12px; border-bottom: 1px solid #b91c1c; display: none; align-items: center; gap: 12px; font-size: 0.9rem; }
      .error-banner ul { margin: 0; padding-left: 18px; }
      .error-banner button { background: transparent; border: 1px solid #fca5a5; color: #fecdd3; border-radius: 6px; padding: 4px 8px; cursor: pointer; }
      .time-row { margin-top:10px; display:flex; gap:10px; align-items:center; flex-wrap:wrap; }
      .tz-select { width:auto; min-width:160px; }
      .time-text { font-size:0.9rem; color:#cbd5e1; }
      .sr-only { position:absolute; width:1px; height:1px; padding:0; margin:-1px; overflow:hidden; clip:rect(0,0,0,0); white-space:nowrap; border:0; }
      .row.hidden { display:none !important; }
      .chart-legend { display:flex; gap:10px; flex-wrap:wrap; align-items:center; font-size:0.9rem; color:#cbd5e1; }
      .legend-item { display:inline-flex; align-items:center; gap:6px; padding:4px 8px; border-radius:999px; background:#0b1220; border:1px solid #1f2937; }
      .color-select { width:auto; min-width:150px; }
      .legend-color-select { min-width:120px; padding:6px 8px; }
    </style>
  </head>
  <body>
    <header>
      <div class="header-row">
        <div>
          <h1>GrowSensor – v0.2.6</h1>
          <div class="hover-hint">Live Monitoring</div>
        </div>
        <div class="header-row" style="gap:10px;">
          <div class="conn">
            <span class="led" id="connLed"></span>
            <span id="connText">offline</span>
            <span class="badge" id="wifiMode">Offline</span>
          </div>
          <button id="openDev" class="ghost" style="width:auto;">Dev-Modus</button>
          <span id="devStatus" class="badge badge-dev" style="display:none;">Dev aktiv</span>
        </div>
      </div>
      <div class="time-row">
        <label for="timezoneSelect" style="width:auto;">Zeitzone</label>
        <select id="timezoneSelect" class="tz-select">
          <option value="Europe/Berlin">Europe/Berlin</option>
          <option value="UTC">UTC</option>
          <option value="Europe/London">Europe/London</option>
          <option value="America/New_York">America/New_York</option>
          <option value="Australia/Sydney">Australia/Sydney</option>
          <option value="Asia/Tokyo">Asia/Tokyo</option>
        </select>
        <span class="badge badge-warn" id="timeBadge">Zeit nicht synchron</span>
        <span class="time-text" id="localTimeText">–</span>
      </div>
      <nav>
        <button id="navDashboard" class="menu-btn">Dashboard</button>
        <button id="navSensors" class="menu-btn">Sensoren</button>
      </nav>
    </header>
    <div id="errorBanner" class="error-banner">
      <strong>Fehler</strong>
      <ul id="errorList"></ul>
      <button id="clearErrors">Schließen</button>
    </div>
    <main>
      <div id="view-dashboard" class="view active">
        <section class="grid metrics">
        <article class="card metric-tile" data-metric="lux">
          <div class="card-header tile-header">
            <div class="tile-title"><div>Licht (Lux)</div><span class="status-dot" id="luxDot"></span></div>
          </div>
          <div class="tile-body">
            <div class="value" id="lux">–</div>
            <div class="hover-chart"><canvas class="hover-canvas" data-metric="lux" width="320" height="140"></canvas></div>
          </div>
        </article>
        <article class="card metric-tile" data-metric="ppfd">
          <div class="card-header tile-header">
            <div class="tile-title"><div>PPFD (µmol/m²/s)</div><span class="status-dot" id="ppfdDot"></span></div>
          </div>
          <div class="tile-body">
            <div class="value" id="ppfd">–</div>
            <div style="font-size:0.85rem;margin-top:6px;">Spektrum: <span id="ppfdSpectrum">–</span><br/>Faktor: <span id="ppfdFactor">–</span></div>
            <div class="hover-chart"><canvas class="hover-canvas" data-metric="ppfd" width="320" height="140"></canvas></div>
          </div>
        </article>
        <article class="card metric-tile" data-metric="co2">
          <div class="card-header tile-header">
            <div class="tile-title"><div>CO₂ (ppm)</div><span class="status-dot" id="co2Dot"></span></div>
          </div>
          <div class="tile-body">
            <div class="value" id="co2">–</div>
            <div class="hover-chart"><canvas class="hover-canvas" data-metric="co2" width="320" height="140"></canvas></div>
          </div>
        </article>
        <article class="card metric-tile" data-metric="temp">
          <div class="card-header tile-header">
            <div class="tile-title"><div>Umgebungstemperatur (°C)</div><span class="status-dot" id="tempDot"></span></div>
          </div>
          <div class="tile-body">
            <div class="value" id="temp">–</div>
            <div class="hover-chart"><canvas class="hover-canvas" data-metric="temp" width="320" height="140"></canvas></div>
          </div>
        </article>
        <article class="card metric-tile" data-metric="humidity">
          <div class="card-header tile-header">
            <div class="tile-title"><div>Luftfeuchte (%)</div><span class="status-dot" id="humidityDot"></span></div>
          </div>
          <div class="tile-body">
            <div class="value" id="humidity">–</div>
            <div class="hover-chart"><canvas class="hover-canvas" data-metric="humidity" width="320" height="140"></canvas></div>
          </div>
        </article>
        <article class="card metric-tile" data-metric="leaf">
          <div class="card-header tile-header">
            <div class="tile-title"><div>Leaf-Temp (°C)</div><span class="status-dot" id="leafDot"></span></div>
          </div>
          <div class="tile-body">
            <div class="value" id="leaf">–</div>
            <div class="hover-chart"><canvas class="hover-canvas" data-metric="leaf" width="320" height="140"></canvas></div>
          </div>
        </article>
        <article class="card metric-tile" data-metric="vpd">
          <div class="card-header tile-header">
            <div class="tile-title"><div>VPD (kPa)</div><span class="status-dot" id="vpdDot"></span></div>
          </div>
          <div class="tile-body">
            <div class="value" id="vpd">–</div>
            <div id="vpdStatus" style="font-size:0.85rem;margin-top:6px;"></div>
            <div class="vpd-chart" id="vpdTileChart">
              <canvas id="vpdTileCanvas" class="vpd-canvas"></canvas>
              <div class="vpd-overlay">
                <div id="vpdTileLegend" class="vpd-legend"></div>
                <div style="display:flex; justify-content:space-between; align-items:center;">
                  <div id="vpdTileTarget" class="vpd-legend"></div>
                  <div id="vpdTileStatus" style="font-weight:600;"></div>
                </div>
              </div>
              <div id="vpdTileNoData" class="vpd-no-data" style="display:none;">keine Daten</div>
            </div>
            <div class="hover-chart"><canvas class="hover-canvas" data-metric="vpd" width="320" height="140"></canvas></div>
          </div>
        </article>
      </section>

        <section class="card" id="avgCard">
        <h3 style="margin-top:0">Gestern Ø (24h)</h3>
        <div class="grid">
          <div><strong>Lux:</strong> <span id="avgLux">–</span></div>
          <div><strong>CO₂:</strong> <span id="avgCo2">–</span></div>
          <div><strong>Temp:</strong> <span id="avgTemp">–</span></div>
          <div><strong>Feuchte:</strong> <span id="avgHum">–</span></div>
          <div><strong>VPD:</strong> <span id="avgVpd">–</span></div>
        </div>
      </section>

        <section class="card">
        <div class="card-header" style="padding:0; margin-bottom:8px; align-items:center; gap:10px;">
          <h3 style="margin:0;">24h Verlauf (15m)</h3>
          <label for="chartMetricSelect" style="display:flex; align-items:center; gap:6px; font-size:0.9rem;">
            Metrik
            <select id="chartMetricSelect" style="width:auto; min-width:150px;">
              <option value="temp">Temperatur</option>
              <option value="humidity">Luftfeuchte</option>
              <option value="vpd">VPD</option>
              <option value="co2">CO₂</option>
              <option value="lux">Lux</option>
              <option value="all">Alle Metriken</option>
            </select>
          </label>
        </div>
        <div class="row" style="justify-content:space-between; align-items:center; gap:8px; flex-wrap:wrap; margin-bottom:6px;">
          <div id="mainChartLegend" class="chart-legend"></div>
          <label for="chartColorSelect" style="display:flex; align-items:center; gap:6px; font-size:0.9rem;">
            Farbe
            <select id="chartColorSelect" class="color-select"></select>
          </label>
        </div>
        <div class="chart"><canvas id="chart"></canvas></div>
      </section>

        <section class="grid">
        <article class="card">
          <h3 style="margin-top:0">Spektrum wählen</h3>
          <label for="channel">LED Kanal</label>
          <select id="channel">
            <option value="full_spectrum">Vollspectrum</option>
            <option value="white">White</option>
            <option value="bloom">Blütekanal</option>
          </select>
          <button id="saveChannel">Speichern</button>
          <label for="vpdStage" style="margin-top:12px; display:block;">Pflanzenstadium (VPD)</label>
          <select id="vpdStage">
            <option value="seedling">Steckling / Sämling (1 Woche)</option>
            <option value="veg">Vegitativ</option>
            <option value="bloom">Blütephase</option>
            <option value="late_bloom">Späteblüte</option>
          </select>
          <button id="applyStage" style="margin-top:8px;">Apply changes</button>
          <p id="vpdStageStatus" class="status" style="margin-top:6px;"></p>
          <p id="vpdTarget" style="margin-top:8px; color:#a5b4fc"></p>
          <p class="hover-hint">Hover über einer Kachel für den 6h-Mini-Graph.</p>
        </article>

      <article class="card" id="wifiCard">
          <h3 style="margin-top:0">Wi-Fi Setup</h3>
          <div id="wifiConnectedBlock" class="hidden">
            <div class="wifi-status">
              <span class="led pulse" style="background:#34d399;" id="wifiPulse"></span>
              <strong>Verbunden</strong>
            </div>
            <div class="row" style="margin-top:4px;">
              <div><strong>SSID:</strong> <span id="wifiSsid">–</span></div>
              <div><strong>IP:</strong> <span id="wifiIp">–</span></div>
            </div>
            <div class="row" style="margin-top:4px; align-items:center;">
              <div><strong>Signal:</strong></div>
              <div class="wifi-bars" id="wifiBars"></div>
            </div>
            <button id="toggleWifiForm" class="ghost" style="width:auto; margin-top:8px;">Wi-Fi ändern</button>
          </div>
          <div id="wifiForm">
          <div class="row">
            <div style="flex:2">
              <label for="ssid">SSID</label>
              <select id="ssid"></select>
            </div>
            <button id="scanWifi" style="flex:1; margin-top: 28px;">Netzwerke suchen</button>
          </div>
          <label for="pass">Passwort</label>
          <input id="pass" type="password" placeholder="WLAN Passwort" />
          <div class="row">
            <label style="flex:1; display:flex; gap:6px; align-items:center;">
              <input type="checkbox" id="staticIpToggle" style="width:auto;"> Static IP
            </label>
          </div>
          <div class="row hidden" id="staticIpRow">
            <input id="ip" placeholder="IP (optional)" class="dev-only" />
            <input id="gw" placeholder="Gateway (optional)" class="dev-only" />
            <input id="sn" placeholder="Subnet (optional)" class="dev-only" />
          </div>
          <button id="saveWifi" class="dev-only">Verbinden & Speichern</button>
          <button id="resetWifi" class="dev-only" style="margin-top:8px;background:#ef4444;color:#fff;">WLAN Reset</button>
          <p id="wifiStatus" class="status" style="margin-top:8px;"></p>
          <p class="dev-note" id="wifiDevNote">Wi-Fi Änderungen nur im Dev-Modus.</p>
          </div>
        </article>
      </section>
      </div>

      <div id="view-sensors" class="view">
        <section class="card">
          <h3 style="margin-top:0">Sensoren</h3>
          <div class="sensor-grid" id="activeSensors"></div>
          <h4 style="margin:12px 0 6px;">Verfügbare Sensoren</h4>
          <div class="sensor-grid" id="availableSensors"></div>
          <div style="margin-top:12px;">
            <button id="addSensorBtn" style="width:auto;">+ Sensor hinzufügen</button>
          </div>
        </section>
      </div>

      <section class="card">
        <h3 style="margin-top:0">Logs (letzte 6h)</h3>
        <button id="refreshLogs">Aktualisieren</button>
        <button id="downloadLogs">Download</button>
        <pre class="logbox" id="logBox"></pre>
      </section>

      <section class="card" id="partnerCard" style="display:none;">
        <h3 style="margin-top:0">Partner & Supporter</h3>
        <div id="partnerList"></div>
        <div class="row" style="margin-top:12px;">
          <input id="partnerId" placeholder="ID" />
          <input id="partnerName" placeholder="Name" />
        </div>
        <input id="partnerDesc" placeholder="Beschreibung" />
        <input id="partnerUrl" placeholder="URL (optional)" />
        <input id="partnerLogo" placeholder="Logo (optional)" />
        <label style="display:flex;align-items:center;gap:6px;margin-top:8px;"><input type="checkbox" id="partnerEnabled" checked style="width:auto;" /> Aktiv</label>
        <button id="savePartner">Partner speichern</button>
      </section>
    </main>
    <footer>Growcontroller v0.2.6 • Sensorgehäuse v0.3</footer>

    <div id="devModal">
      <div class="card" style="max-width:420px;width:90%;">
        <h3 style="margin-top:0">Dev-Modus aktivieren</h3>
        <label for="devCode">Code</label>
        <input id="devCode" type="password" placeholder="Dev-Code" />
        <div class="row">
          <button id="activateDev">Aktivieren</button>
          <button id="cancelDev" class="ghost">Abbrechen</button>
        </div>
        <p id="devStatusMsg" class="status" style="margin-top:8px;"></p>
      </div>
    </div>

    <div id="chartModal">
        <div class="modal-card" id="chartModalCard">
          <button class="modal-close" id="chartClose">&times;</button>
          <div class="card-header" style="padding:0;">
            <div id="chartModalTitle">Detailansicht</div>
            <span class="badge" id="chartModalBadge">–</span>
          </div>
        <div class="modal-tabs" id="detailScopeTabs">
          <button id="tabLive" class="active">Live</button>
          <button id="tabLast6h">Letzte 6h</button>
          <button id="tabLast24h">Letzte 24h</button>
        </div>
        <div class="row" style="align-items:center; gap:8px; flex-wrap:wrap; justify-content:space-between;">
          <label for="detailColorSelect" style="display:flex; align-items:center; gap:6px; font-size:0.9rem;">
            Farbe
            <select id="detailColorSelect" class="color-select"></select>
          </label>
          <div id="detailLegend" class="chart-legend"></div>
        </div>
        <div style="position:relative;">
          <div class="modal-tabs" id="vpdViewTabs" style="display:none;">
            <button id="tabHeatmap" class="active">Heatmap</button>
            <button id="tabTimeline">Zeitverlauf</button>
          </div>
          <canvas id="vpdHeatmap" style="width:100%; height:260px; display:none;"></canvas>
          <canvas id="detailChart" style="width:100%; height:260px;"></canvas>
          <div id="chartEmpty" class="chart-empty" style="display:none;">Keine Daten</div>
          <div id="chartDebugText" class="dev-note" style="display:none; position:absolute; top:8px; left:12px;"></div>
          <div class="row axis-label" style="justify-content:space-between; align-items:center; margin-top:6px;">
            <div id="yAxisLabel">Wert</div>
            <div id="xAxisLabel">Zeit</div>
          </div>
        </div>
      </div>
    </div>

    <div id="sensorWizard">
      <div class="modal-card" style="position:relative;">
        <button class="modal-close" id="sensorWizardClose">&times;</button>
        <h3 style="margin-top:0">Sensor hinzufügen</h3>
        <div class="wizard-step active" data-step="1">
          <label for="sensorCategorySelect">Kategorie</label>
          <select id="sensorCategorySelect"></select>
          <label for="sensorTypeSelect">Sensor</label>
          <select id="sensorTypeSelect"></select>
        </div>
        <div class="wizard-step" data-step="2">
          <div class="pill" id="defaultPinsInfo">Standardpins werden genutzt.</div>
          <div class="row">
            <div>
              <label>SDA</label>
              <input id="wizSda" type="number" min="0" max="39" disabled />
            </div>
            <div>
              <label>SCL</label>
              <input id="wizScl" type="number" min="0" max="39" disabled />
            </div>
            <div>
              <label>RX</label>
              <input id="wizRx" type="number" min="0" max="39" disabled />
            </div>
            <div>
              <label>TX</label>
              <input id="wizTx" type="number" min="0" max="39" disabled />
            </div>
          </div>
          <button id="advancedPinsBtn" class="ghost" style="width:auto;">Advanced Pin Config</button>
          <p class="sensor-desc" id="advancedPinsWarn" style="display:none;">Wirklich Pinbelegung ändern? Falsche Pins können den Sensor unbrauchbar machen.</p>
        </div>
        <div class="wizard-step" data-step="3">
          <p style="margin:0;">Bitte ESP einmal aus- und einstecken (Neustart), damit Pins aktiv werden.</p>
          <button id="restartNow" class="ghost" style="width:auto;">Neustart jetzt</button>
        </div>
        <div class="row" style="margin-top:12px;">
          <button id="wizBack" class="ghost">Zurück</button>
          <button id="wizNext">Weiter</button>
          <button id="wizSave" style="display:none;">Speichern</button>
        </div>
        <p id="wizStatus" class="status" style="margin-top:8px;"></p>
      </div>
    </div>

    <script>
      const errorMessages = new Set();
      const errorBanner = document.getElementById('errorBanner');
      const errorList = document.getElementById('errorList');
      const clearErrorsBtn = document.getElementById('clearErrors');

      function renderErrors() {
        if (!errorBanner || !errorList) return;
        errorList.innerHTML = '';
        errorMessages.forEach(msg => {
          const li = document.createElement('li');
          li.textContent = msg;
          errorList.appendChild(li);
        });
        errorBanner.style.display = errorMessages.size ? 'flex' : 'none';
      }

      function pushError(message) {
        if (!message) return;
        errorMessages.add(message);
        console.warn(message);
        renderErrors();
      }

      function clearErrors(matchPrefix = null) {
        if (matchPrefix === null) {
          errorMessages.clear();
        } else {
          Array.from(errorMessages).forEach(msg => {
            if (msg.startsWith(matchPrefix)) errorMessages.delete(msg);
          });
        }
        renderErrors();
      }

      if (clearErrorsBtn) {
        clearErrorsBtn.addEventListener('click', () => clearErrors());
      } else {
        pushError('Missing DOM element: clearErrors');
      }

      window.addEventListener('error', (event) => {
        const msg = event?.message || 'Uncaught JS error';
        pushError(`JS Error: ${msg}`);
      });

      window.addEventListener('unhandledrejection', (event) => {
        const reason = event?.reason;
        const msg = (reason && reason.message) ? reason.message : (typeof reason === 'string' ? reason : 'Unhandled promise rejection');
        pushError(`JS Promise: ${msg}`);
      });

      function getEl(id) {
        const el = document.getElementById(id);
        if (!el) pushError(`Missing DOM element: ${id}`);
        return el;
      }

      function setText(id, value) {
        const el = getEl(id);
        if (el) el.textContent = value;
      }

      function setValue(id, value) {
        const el = getEl(id);
        if (el) el.value = value;
      }

      function setDisplay(id, visible, displayStyle = 'block') {
        const el = getEl(id);
        if (el) el.style.display = visible ? displayStyle : 'none';
      }

      const TILE_DEFS = [
        { id:'lux', label:'Licht', unit:'Lux', defaultVisible:true, defaultOrder:0 },
        { id:'ppfd', label:'PPFD', unit:'µmol/m²/s', defaultVisible:true, defaultOrder:1 },
        { id:'co2', label:'CO₂', unit:'ppm', defaultVisible:true, defaultOrder:2 },
        { id:'temp', label:'Umgebung', unit:'°C', defaultVisible:true, defaultOrder:3 },
        { id:'humidity', label:'Luftfeuchte', unit:'%', defaultVisible:true, defaultOrder:4 },
        { id:'leaf', label:'Leaf-Temp', unit:'°C', defaultVisible:true, defaultOrder:5 },
        { id:'vpd', label:'VPD', unit:'kPa', defaultVisible:true, defaultOrder:6 },
      ];
      const TILE_VISIBILITY_KEY = 'tile_visibility_v026';
      let tileVisibility = {};
      const tileOrder = TILE_DEFS.map(t => t.id);
      const TILE_EYE_MARKUP = `<button class=\"tile-eye\" type=\"button\" aria-pressed=\"true\" title=\"Kachel einklappen\" aria-label=\"Kachel einklappen\"><span class=\"sr-only\">Kachel ein-/ausblenden</span><svg class=\"eye-open\" viewBox=\"0 0 24 24\" aria-hidden=\"true\"><path d=\"M12 5c-5 0-9 7-9 7s4 7 9 7 9-7 9-7-4-7-9-7zm0 12a5 5 0 1 1 0-10 5 5 0 0 1 0 10zm0-8a3 3 0 1 0 0 6 3 3 0 0 0 0-6z\" fill=\"currentColor\"/></svg><svg class=\"eye-closed\" viewBox=\"0 0 24 24\" aria-hidden=\"true\"><path d=\"m3 3 18 18-1.4 1.4-3.1-3.1C15 20.3 13.5 21 12 21c-5 0-9-7-9-7 0-.9 1.1-3 3-4.9L1.6 4.4 3 3zm5 5.6 2 2A2 2 0 0 0 10 12a2 2 0 0 0 2 2c.4 0 .7-.1 1-.3l2 2c-.9.5-1.9.8-3 .8-2.8 0-5-2.2-5-5 0-.9.2-1.7.5-2.5zm3.9-3.1c5 0 9 7 9 7 0 1.1-1.3 3.7-3.8 5.5l-1.5-1.5c1.7-1.2 2.9-3 3.3-3.9-.7-1.1-3.4-5.1-7-5.1-.7 0-1.3.1-1.9.3l-1.6-1.6c.9-.3 1.8-.5 2.8-.5z\" fill=\"currentColor\"/></svg></button>`;

      function loadTileVisibility() {
        let saved = {};
        try {
          saved = JSON.parse(localStorage.getItem(TILE_VISIBILITY_KEY) || '{}') || {};
        } catch (err) {
          console.warn('Failed to parse tile visibility', err);
        }
        tileVisibility = {};
        const setDefault = (id, fallback = true) => {
          tileVisibility[id] = saved[id] !== undefined ? saved[id] !== false : fallback !== false;
        };
        TILE_DEFS.forEach(def => setDefault(def.id, true));
        Object.keys(saved || {}).forEach(id => {
          if (!Object.prototype.hasOwnProperty.call(tileVisibility, id)) {
            tileVisibility[id] = saved[id] !== false;
          }
        });
      }

      function persistTileVisibility() {
        try {
          localStorage.setItem(TILE_VISIBILITY_KEY, JSON.stringify(tileVisibility));
        } catch (err) {
          console.warn('Failed to store tile visibility', err);
        }
      }

      function tileIsExpanded(metric) {
        if (!(metric in tileVisibility)) tileVisibility[metric] = true;
        return tileVisibility[metric] !== false;
      }

      function updateEyeVisual(btn, expanded) {
        if (!btn) return;
        const label = expanded ? 'Kachel einklappen' : 'Kachel anzeigen';
        btn.setAttribute('aria-pressed', expanded ? 'true' : 'false');
        btn.setAttribute('aria-label', label);
        btn.setAttribute('title', label);
      }

      function applyTileVisibility() {
        document.querySelectorAll('.metric-tile').forEach(tile => {
          const metric = tile.dataset.metric;
          if (!metric) return;
          const expanded = tileIsExpanded(metric);
          tile.classList.toggle('collapsed', !expanded);
          updateEyeVisual(tile.querySelector('.tile-eye'), expanded);
        });
      }

      function setTileExpanded(metric, expanded, persist = true) {
        tileVisibility[metric] = expanded !== false;
        applyTileVisibility();
        if (persist) persistTileVisibility();
      }

      function ensureTileEyes() {
        document.querySelectorAll('.metric-tile').forEach(tile => {
          const metric = tile.dataset.metric || '';
          if (!tile.querySelector('.tile-eye')) {
            tile.insertAdjacentHTML('beforeend', TILE_EYE_MARKUP);
          }
          const eye = tile.querySelector('.tile-eye');
          if (eye) eye.dataset.metric = metric;
        });
      }

      loadTileVisibility();
      ensureTileEyes();
      applyTileVisibility();

      const chartCanvas = getEl('chart');
      const ctx = chartCanvas && chartCanvas.getContext ? chartCanvas.getContext('2d') : null;
      const metrics = Array.from(new Set(tileOrder));
      const BUCKET_5M = 5 * 60 * 1000;
      const BUCKET_15M = 15 * 60 * 1000;
      const SIX_H_MS = 6 * 60 * 60 * 1000;
      const DAY_MS = 24 * 60 * 60 * 1000;
      const LIVE_WINDOW_MS = 2 * 60 * 60 * 1000;
      const historyStore = {};
      metrics.forEach(m => historyStore[m] = { mid: [], long: [], agg5:{ bucket:-1, sum:0, count:0 }, agg15:{ bucket:-1, sum:0, count:0 }, synced:false });
      const MAX_6H_POINTS = 80; // 6h @5min + buffer
      const MAX_24H_POINTS = 110; // 24h @15min + buffer
      let lastTelemetryAt = 0;
      const clockState = { synced:false, timezone:'Europe/Berlin', epochRef:null, epochCapturedAt:null };
      let devMode = false;
      const DEV_CODE = "Test1234#";
      const hoverCanvases = {};
      document.querySelectorAll('.hover-canvas').forEach(c => hoverCanvases[c.dataset.metric] = c.getContext ? c.getContext('2d') : null);
      const detailChartCanvas = getEl('detailChart');
      const detailCtx = detailChartCanvas && detailChartCanvas.getContext ? detailChartCanvas.getContext('2d') : null;
      const vpdHeatmapCanvas = getEl('vpdHeatmap');
      const vpdHeatmapCtx = vpdHeatmapCanvas && vpdHeatmapCanvas.getContext ? vpdHeatmapCanvas.getContext('2d') : null;
      const vpdTileCanvas = getEl('vpdTileCanvas');
      const vpdTileCtx = vpdTileCanvas && vpdTileCanvas.getContext ? vpdTileCanvas.getContext('2d') : null;
      const chartEmpty = getEl('chartEmpty');
      const xAxisLabel = getEl('xAxisLabel');
      const yAxisLabel = getEl('yAxisLabel');
      const wifiBars = getEl('wifiBars');
      const timezoneSelect = getEl('timezoneSelect');
      const chartMetricSelect = getEl('chartMetricSelect');
      const chartColorSelect = getEl('chartColorSelect');
      const detailColorSelect = getEl('detailColorSelect');
      const mainChartLegend = getEl('mainChartLegend');
      const detailLegend = getEl('detailLegend');
      const timeBadge = getEl('timeBadge');
      const localTimeText = getEl('localTimeText');
      let detailMetric = null;
      let detailMode = 'live';
      let detailVpdView = 'heatmap';
      let clickDebug = false;
      let lastVpdTargets = { low: null, high: null };
      let wifiFormOpen = false;
      let wizardOpenedAt = 0;
      const detailCache = {};
      const METRIC_META = {
        lux: { unit:'Lux', label:'Lux', decimals:1 },
        ppfd:{ unit:'µmol/m²/s', label:'PPFD', decimals:1 },
        co2: { unit:'ppm', label:'CO₂', decimals:0 },
        temp:{ unit:'°C', label:'Temperatur', decimals:1 },
        humidity:{ unit:'%', label:'Luftfeuchte', decimals:1 },
        leaf:{ unit:'°C', label:'Leaf-Temp', decimals:1 },
        vpd:{ unit:'kPa', label:'VPD', decimals:3 },
      };
      const COLOR_PALETTE = [
        { id:'cyan', name:'Cyan', value:'#22d3ee' },
        { id:'indigo', name:'Indigo', value:'#6366f1' },
        { id:'amber', name:'Amber', value:'#f59e0b' },
        { id:'emerald', name:'Emerald', value:'#10b981' },
        { id:'pink', name:'Pink', value:'#ec4899' },
        { id:'red', name:'Rot', value:'#ef4444' },
        { id:'lime', name:'Lime', value:'#84cc16' },
        { id:'sky', name:'Sky', value:'#38bdf8' },
        { id:'violet', name:'Violett', value:'#a855f7' },
        { id:'slate', name:'Slate', value:'#94a3b8' },
        { id:'rose', name:'Rose', value:'#fb7185' },
        { id:'teal', name:'Teal', value:'#14b8a6' },
      ];
      const COLOR_PREF_KEY = 'metric_colors_v026';
      let colorPrefs = {};
      function loadColorPrefs() {
        try {
          const raw = localStorage.getItem(COLOR_PREF_KEY);
          if (raw) colorPrefs = JSON.parse(raw) || {};
        } catch (err) {
          console.warn('color prefs parse failed', err);
          colorPrefs = {};
        }
      }
      function persistColorPrefs() {
        try { localStorage.setItem(COLOR_PREF_KEY, JSON.stringify(colorPrefs)); } catch (err) { console.warn('color prefs persist failed', err); }
      }
      function hashString(str) {
        let h = 0;
        for (let i = 0; i < str.length; i++) {
          h = Math.imul(31, h) + str.charCodeAt(i) | 0;
        }
        return Math.abs(h);
      }
      function colorForDevice(id) {
        if (!id) return COLOR_PALETTE[0].value;
        if (colorPrefs[id]) return colorPrefs[id];
        const idx = hashString(id) % COLOR_PALETTE.length;
        return COLOR_PALETTE[idx].value;
      }
      function setColorForDevice(id, color) {
        if (!id || !color) return;
        colorPrefs[id] = color;
        persistColorPrefs();
      }
      function colorKey(metric, deviceId) {
        if (!metric && !deviceId) return '';
        return metric ? `${metric}:${deviceId || metric}` : `${deviceId}`;
      }
      function colorForMetricDevice(metric, deviceId) {
        const key = colorKey(metric, deviceId);
        if (key && colorPrefs[key]) return colorPrefs[key];
        if (deviceId && colorPrefs[deviceId]) return colorPrefs[deviceId];
        return colorForDevice(deviceId || key || metric);
      }
      function setColorForMetricDevice(metric, deviceId, color) {
        const key = colorKey(metric, deviceId);
        if (!key || !color) return;
        colorPrefs[key] = color;
        persistColorPrefs();
      }
      loadColorPrefs();
      const metricDeviceIds = {
        lux: 'BH1750',
        ppfd: 'BH1750',
        co2: 'MHZ19',
        temp: 'SHT31',
        humidity: 'SHT31',
        leaf: 'MLX90614',
        vpd: 'VPD',
      };
      function setDeviceId(metric, id) {
        if (!metric || !id) return;
        metricDeviceIds[metric] = id;
      }
      function deviceIdForMetric(metric) {
        return metricDeviceIds[metric] || metric.toUpperCase();
      }
      function applyDeviceIdsFromTelemetry(data = {}) {
        const map = data.device_ids || {};
        const co2Dev = data.co2_device || map.co2;
        const lightDev = data.lux_device || map.lux;
        const climateDev = data.climate_device || map.climate;
        const leafDev = data.leaf_device || map.leaf;
        if (lightDev) { setDeviceId('lux', lightDev); setDeviceId('ppfd', lightDev); }
        if (climateDev) { setDeviceId('temp', climateDev); setDeviceId('humidity', climateDev); }
        if (leafDev) setDeviceId('leaf', leafDev);
        if (co2Dev) setDeviceId('co2', co2Dev);
        if ((metricDeviceIds.temp || climateDev) && (metricDeviceIds.leaf || leafDev)) {
          setDeviceId('vpd', `${metricDeviceIds.temp || climateDev || 'climate'}+${metricDeviceIds.leaf || leafDev || 'leaf'}`);
        }
      }
      function applyDeviceIdsFromSensors(active = []) {
        const byCat = {};
        active.forEach(s => {
          if (!s || s.enabled === false) return;
          byCat[s.category] = s.type || s.name || s.id;
        });
        if (byCat.light) { setDeviceId('lux', byCat.light); setDeviceId('ppfd', byCat.light); }
        if (byCat.climate) { setDeviceId('temp', byCat.climate); setDeviceId('humidity', byCat.climate); }
        if (byCat.leaf) setDeviceId('leaf', byCat.leaf);
        if (byCat.co2) setDeviceId('co2', byCat.co2);
        if (metricDeviceIds.temp || metricDeviceIds.humidity || metricDeviceIds.leaf) {
          setDeviceId('vpd', `${metricDeviceIds.temp || 'climate'}+${metricDeviceIds.leaf || 'leaf'}`);
        }
      }
      function renderLegend(el, entries = [], opts = {}) {
        if (!el) return;
        el.innerHTML = '';
        const allowColorPickers = opts.allowColorPickers === true;
        entries.forEach(item => {
          const div = document.createElement('div');
          div.className = 'legend-item';
          if (item.title) div.title = item.title;
          const swatch = document.createElement('span');
          swatch.className = 'legend-swatch';
          swatch.style.background = item.color;
          const label = document.createElement('span');
          label.textContent = item.label || item.id;
          div.appendChild(swatch);
          div.appendChild(label);
          if (allowColorPickers) {
            const select = document.createElement('select');
            select.className = 'color-select legend-color-select';
            COLOR_PALETTE.forEach(c => {
              const opt = document.createElement('option');
              opt.value = c.value;
              opt.textContent = `● ${c.name}`;
              opt.style.color = c.value;
              if (c.value === item.color) opt.selected = true;
              select.appendChild(opt);
            });
            select.addEventListener('change', () => {
              if (typeof opts.onColorChange === 'function') opts.onColorChange(item, select.value);
            });
            div.appendChild(select);
          }
          el.appendChild(div);
        });
      }
      function renderColorSelect(selectEl, currentColor) {
        if (!selectEl) return;
        const selected = currentColor || COLOR_PALETTE[0].value;
        selectEl.innerHTML = '';
        COLOR_PALETTE.forEach(c => {
          const opt = document.createElement('option');
          opt.value = c.value;
          opt.textContent = `● ${c.name}`;
          opt.style.color = c.value;
          if (c.value === selected) opt.selected = true;
          selectEl.appendChild(opt);
        });
      }
      const KNOWN_TIMEZONES = ['Europe/Berlin','UTC','Europe/London','America/New_York','Asia/Tokyo','Australia/Sydney'];
      let chartMetric = 'temp';
      if (chartMetricSelect && chartMetricSelect.value) chartMetric = chartMetricSelect.value;

      function authedFetch(url, options = {}) { return fetch(url, options); }
      function flag(val, fallback = false) { if (val === undefined || val === null) return fallback; return val === true || val === 1 || val === "1"; }

      async function fetchJson(url, options = {}) {
        let res;
        try {
          res = await authedFetch(url, options);
        } catch (err) {
          pushError(`API ${url} failed: ${err.message}`);
          throw err;
        }
        let text = '';
        try {
          text = await res.text();
        } catch (err) {
          pushError(`API ${url} failed: ${err.message}`);
          throw err;
        }
        if (!res.ok) {
          const msg = `API ${url} failed: ${res.status}${text ? ` ${text}` : ''}`;
          pushError(msg);
          throw new Error(msg);
        }
        if (!text) return {};
        try {
          return JSON.parse(text);
        } catch (err) {
          pushError(`JSON parse failed (${url}): ${err.message}`);
          throw err;
        }
      }

      function populateTimezoneSelect() {
        if (!timezoneSelect) return;
        const current = timezoneSelect.value || clockState.timezone;
        timezoneSelect.innerHTML = '';
        KNOWN_TIMEZONES.forEach(tz => {
          const opt = document.createElement('option');
          opt.value = tz;
          opt.textContent = tz;
          timezoneSelect.appendChild(opt);
        });
        if (current && !KNOWN_TIMEZONES.includes(current)) {
          const opt = document.createElement('option');
          opt.value = current;
          opt.textContent = current;
          timezoneSelect.appendChild(opt);
        }
        if (current) timezoneSelect.value = current;
        if (timezoneSelect.value) clockState.timezone = timezoneSelect.value;
      }

      function parseMs(val) {
        if (typeof val === 'number') return val;
        if (typeof val === 'string') {
          const num = Number(val);
          return Number.isFinite(num) ? num : null;
        }
        return null;
      }

      function getApproxEpochMs() {
        if (clockState.synced && clockState.epochRef !== null && clockState.epochCapturedAt !== null) {
          return clockState.epochRef + (Date.now() - clockState.epochCapturedAt);
        }
        return null;
      }

      function renderTimeStatus() {
        if (timeBadge) {
          if (clockState.synced) {
            timeBadge.textContent = 'Zeit synchron';
            timeBadge.classList.remove('badge-warn');
          } else {
            timeBadge.textContent = 'Zeit nicht synchron';
            timeBadge.classList.add('badge-warn');
          }
        }
        if (localTimeText) {
          const nowMs = getApproxEpochMs();
          if (clockState.synced && nowMs !== null) {
            localTimeText.textContent = new Intl.DateTimeFormat(undefined, { hour:'2-digit', minute:'2-digit', second:'2-digit', timeZone: clockState.timezone }).format(new Date(nowMs));
          } else {
            localTimeText.textContent = 'Zeitquelle unsynchron';
          }
        }
      }

      function updateClockState(payload = {}) {
        if (payload.timezone) {
          clockState.timezone = payload.timezone;
          if (timezoneSelect) timezoneSelect.value = payload.timezone;
        }
        const epochMs = parseMs(payload.epoch_ms ?? payload.now_epoch_ms);
        const syncedFlag = flag(payload.time_synced);
        if (syncedFlag && epochMs !== null) {
          clockState.synced = true;
          clockState.epochRef = epochMs;
          clockState.epochCapturedAt = Date.now();
        } else if (payload.time_synced !== undefined && !syncedFlag) {
          clockState.synced = false;
          if (payload.time_synced === 0) {
            clockState.epochRef = null;
            clockState.epochCapturedAt = null;
          }
        }
        renderTimeStatus();
      }

      function setModalVisible(el, visible) {
        if (!el) return;
        const animate = el.id === 'sensorWizard';
        if (animate) {
          if (visible) {
            el.style.display = 'flex';
            requestAnimationFrame(() => el.classList.add('active'));
          } else {
            el.classList.remove('active');
            setTimeout(() => { el.style.display = 'none'; }, 180);
          }
        } else {
          el.style.display = visible ? 'flex' : 'none';
        }
        el.style.pointerEvents = visible ? 'auto' : 'none';
      }

      document.addEventListener('click', (e) => {
        if (clickDebug || devMode) {
          console.log('CLICK', e.target, getComputedStyle(e.target).pointerEvents);
          ['devModal','chartModal','sensorWizard'].forEach(id => {
            const el = getEl(id);
            if (el) console.log(id, 'display', getComputedStyle(el).display, 'pointer', getComputedStyle(el).pointerEvents, 'z', getComputedStyle(el).zIndex);
          });
        }
      }, true);

      document.addEventListener('keydown', (e) => {
        if (e.key === 'Escape') {
          ['sensorWizard','chartModal','devModal'].forEach(id => setModalVisible(getEl(id), false));
        }
      });

      function setDot(id, ok, present, enabled) {
        const el = getEl(id);
        if (!el) return;
        let color = '#6b7280';
        if (enabled && present) color = ok ? '#34d399' : '#fbbf24';
        else if (enabled && !present) color = '#9ca3af';
        el.style.background = color;
        el.style.boxShadow = `0 0 0 3px ${color}33`;
      }

      let lastTelemetryPayload = {};

      function metricStatus(metric, telemetry = lastTelemetryPayload) {
        const t = telemetry || {};
        const num = (val) => (typeof val === 'number' && !Number.isNaN(val)) ? val : null;
        const map = {
          lux: { ok: flag(t.lux_ok), present: flag(t.lux_present), enabled: flag(t.lux_enabled, true), value: num(t.lux) },
          ppfd: { ok: flag(t.lux_ok), present: flag(t.lux_present), enabled: flag(t.lux_enabled, true), value: num(t.ppfd) },
          co2: { ok: flag(t.co2_ok), present: flag(t.co2_present), enabled: flag(t.co2_enabled, true), value: (typeof t.co2 === 'number' && t.co2 > 0 && !Number.isNaN(t.co2)) ? t.co2 : null },
          temp: { ok: flag(t.climate_ok), present: flag(t.climate_present), enabled: flag(t.climate_enabled, true), value: num(t.temp) },
          humidity: { ok: flag(t.climate_ok), present: flag(t.climate_present), enabled: flag(t.climate_enabled, true), value: num(t.humidity) },
          leaf: { ok: flag(t.leaf_ok), present: flag(t.leaf_present), enabled: flag(t.leaf_enabled, true), value: num(t.leaf) },
          vpd: {
            ok: flag(t.vpd_ok) && num(t.vpd) !== null,
            present: flag(t.climate_present) && flag(t.leaf_present),
            enabled: flag(t.climate_enabled) || flag(t.leaf_enabled),
            value: num(t.vpd)
          },
        };
        const state = map[metric] || { ok:false, present:false, enabled:false, value:null };
        const valid = state.value !== null;
        return { ...state, valid, healthy: state.ok && state.present && state.enabled && valid };
      }

      function metricIsHealthy(metric, telemetry = lastTelemetryPayload) {
        return metricStatus(metric, telemetry).healthy;
      }

      function updateTileHeaderStates(telemetry = {}) {
        if (telemetry && typeof telemetry === 'object') {
          lastTelemetryPayload = { ...lastTelemetryPayload, ...telemetry };
        }
        document.querySelectorAll('.metric-tile').forEach(tile => {
          const metric = tile.dataset.metric;
          if (!metric) return;
          const state = metricStatus(metric, lastTelemetryPayload);
          const expanded = tileIsExpanded(metric);
          const dotId = `${metric}Dot`;
          setDot(dotId, state.ok, state.present, state.enabled);
          tile.classList.toggle('collapsed', !expanded);
          updateEyeVisual(tile.querySelector('.tile-eye'), expanded);
        });
      }

      function updateConnectionStatus(apMode = false, wifiConnected = false) {
        const online = (Date.now() - lastTelemetryAt) < 10000;
        const dot = getEl('connLed');
        const text = getEl('connText');
        const badge = getEl('wifiMode');
        if (dot) {
          dot.style.background = online ? '#34d399' : '#ef4444';
          dot.style.boxShadow = online ? '0 0 0 3px rgba(52,211,153,0.25)' : '0 0 0 3px rgba(239,68,68,0.25)';
        }
        if (text) text.textContent = online ? 'online' : 'offline';
        if (badge) {
          badge.textContent = apMode ? 'AP' : (wifiConnected ? 'WLAN' : 'Offline');
          badge.style.background = apMode ? '#f59e0b' : (wifiConnected ? '#22c55e' : '#ef4444');
          badge.style.color = '#0f172a';
        }
      }

      function resizeCanvas(canvas, ctxTarget) {
        if (!canvas || !ctxTarget) return { width: 0, height: 0 };
        const ratio = window.devicePixelRatio || 1;
        const rect = canvas.getBoundingClientRect();
        const cssW = Math.max(1, Math.round(rect.width));
        const cssH = Math.max(1, Math.round(rect.height));
        const w = Math.round(cssW * ratio);
        const h = Math.round(cssH * ratio);
        if (canvas.width !== w || canvas.height !== h) {
          canvas.width = w;
          canvas.height = h;
        }
        ctxTarget.setTransform(1,0,0,1,0,0);
        ctxTarget.scale(ratio,ratio);
        return { width: cssW, height: cssH };
      }

      function normalizeMode(mode) {
        if (mode === '24h') return '24h';
        if (mode === '6h') return '6h';
        return 'live';
      }

      function pushBucket(list, cap, point) {
        if (!point) return;
        list.push(point);
        if (list.length > cap) list.splice(0, list.length - cap);
      }

      function addBucketSample(store, entry, ts, bucketMs, targetKey, cap) {
        const agg = bucketMs === BUCKET_5M ? store.agg5 : store.agg15;
        const list = targetKey === 'mid' ? store.mid : store.long;
        const bucketIdx = Math.floor(ts / bucketMs);
        if (agg.bucket === -1) agg.bucket = bucketIdx;
        if (bucketIdx < agg.bucket || bucketIdx - agg.bucket > cap * 2) {
          agg.bucket = bucketIdx;
          agg.sum = 0;
          agg.count = 0;
        }
        while (bucketIdx > agg.bucket) {
          const value = agg.count > 0 ? agg.sum / agg.count : null;
          pushBucket(list, cap, { t: agg.bucket * bucketMs, v: value });
          agg.bucket += 1;
          agg.sum = 0;
          agg.count = 0;
        }
        if (entry !== null) {
          agg.sum += entry;
          agg.count += 1;
        }
      }

      function recordMetric(metric, value, ts, syncedFlag = clockState.synced) {
        const store = historyStore[metric];
        if (!store) return;
        const tsMs = parseMs(ts);
        const nowTs = tsMs !== null ? tsMs : (clockState.synced ? (getApproxEpochMs() ?? Date.now()) : Date.now());
        const entry = (typeof value === 'number' && !Number.isNaN(value)) ? value : null;
        store.synced = syncedFlag ? true : store.synced;
        addBucketSample(store, entry, nowTs, BUCKET_5M, 'mid', MAX_6H_POINTS);
        addBucketSample(store, entry, nowTs, BUCKET_15M, 'long', MAX_24H_POINTS);
      }

      function pendingPoint(store, mode) {
        const normalized = normalizeMode(mode);
        const agg = normalized === '24h' ? store.agg15 : store.agg5;
        const bucketMs = normalized === '24h' ? BUCKET_15M : BUCKET_5M;
        if (agg.bucket === -1) return null;
        const value = agg.count > 0 ? agg.sum / agg.count : null;
        return { t: agg.bucket * bucketMs, v: value };
      }

      function getSeriesData(metric, mode) {
        const normalized = normalizeMode(mode);
        const store = historyStore[metric] || { mid: [], long: [] };
        const windowMs = normalized === '24h' ? DAY_MS : (normalized === '6h' ? SIX_H_MS : LIVE_WINDOW_MS);
        const lastPointTs = normalized === '24h' ? (store.long[store.long.length - 1]?.t) : (store.mid[store.mid.length - 1]?.t);
        const nowTs = (clockState.synced || store.synced) ? (getApproxEpochMs() ?? Date.now()) : (lastPointTs ?? Date.now());
        const minTs = Math.max(0, nowTs - windowMs);
        const list = normalized === '24h' ? store.long : store.mid;
        const cap = normalized === '24h' ? MAX_24H_POINTS : MAX_6H_POINTS;
        const data = list.filter(p => p && typeof p.t === 'number' && p.t >= minTs).slice(-cap);
        const pending = pendingPoint(store, normalized);
        if (pending && pending.t >= minTs) data.push(pending);
        return data;
      }

      function mergeHistory(metric, points, mode, syncedFlag = clockState.synced) {
        const store = historyStore[metric];
        if (!store) return;
        store.synced = syncedFlag ? true : store.synced;
        const normalized = normalizeMode(mode);
        const sanitized = points.map(p => ({ t: p.t, v: (p && typeof p.v === 'number' && !Number.isNaN(p.v)) ? p.v : null }));
        if (normalized === '6h') {
          store.mid = sanitized.slice(-MAX_6H_POINTS);
          store.agg5.bucket = store.mid.length ? Math.floor(store.mid[store.mid.length - 1].t / BUCKET_5M) : -1;
          store.agg5.sum = 0;
          store.agg5.count = 0;
        } else if (normalized === '24h') {
          store.long = sanitized.slice(-MAX_24H_POINTS);
          store.agg15.bucket = store.long.length ? Math.floor(store.long[store.long.length - 1].t / BUCKET_15M) : -1;
          store.agg15.sum = 0;
          store.agg15.count = 0;
        } else {
          store.mid = sanitized.slice(-MAX_6H_POINTS);
          store.agg5.bucket = store.mid.length ? Math.floor(store.mid[store.mid.length - 1].t / BUCKET_5M) : -1;
          store.agg5.sum = 0;
          store.agg5.count = 0;
        }
      }

      function collectPaired(mode) {
        const temps = getSeriesData('temp', mode);
        const hums = getSeriesData('humidity', mode);
        const vpds = getSeriesData('vpd', mode);
        const len = Math.min(temps.length, hums.length, vpds.length);
        const out = [];
        for (let i = 0; i < len; i++) {
          out.push({ temp: temps[temps.length - len + i]?.v, hum: hums[hums.length - len + i]?.v, vpd: vpds[vpds.length - len + i]?.v });
        }
        return out.filter(p => p.temp !== null && p.hum !== null && p.vpd !== null);
      }

      function formatTimeLabel(ts, mode, firstTs, lastTs, synced = clockState.synced, tz = clockState.timezone, includeDate = false, anchorDate = '') {
        const normalized = normalizeMode(mode);
        if (!synced) {
          const delta = Math.max(0, ts - firstTs);
          if (normalized === '6h' || normalized === '24h') {
            const hh = Math.floor(delta / 3600000);
            const mm = Math.floor((delta % 3600000) / 60000);
            return `${String(hh).padStart(2,'0')}:${String(mm).padStart(2,'0')}`;
          }
          const mm = Math.floor(delta / 60000);
          const ss = Math.floor((delta % 60000) / 1000);
          return `${String(mm).padStart(2,'0')}:${String(ss).padStart(2,'0')}`;
        }
        const opts = { hour:'2-digit', minute:'2-digit', timeZone: tz || 'UTC' };
        if (normalized === 'live') opts.second = '2-digit';
        const d = new Date(ts);
        const timeLabel = new Intl.DateTimeFormat(undefined, opts).format(d);
        if (includeDate) {
          const dateLabel = new Intl.DateTimeFormat(undefined, { timeZone: tz || 'UTC', month:'2-digit', day:'2-digit' }).format(d);
          if (!anchorDate || dateLabel !== anchorDate) return `${dateLabel} ${timeLabel}`;
        }
        return timeLabel;
      }

      function dayStamp(ts, tz = clockState.timezone) {
        return new Intl.DateTimeFormat('en-CA', { timeZone: tz || 'UTC', year:'numeric', month:'2-digit', day:'2-digit' }).format(new Date(ts));
      }

      function computeYRange(values, decimals = 1) {
        const maxVal = Math.max(...values);
        const minVal = Math.min(...values);
        if (!Number.isFinite(maxVal) || !Number.isFinite(minVal)) return { min:0, max:1, span:1 };
        let paddedMin = minVal;
        let paddedMax = maxVal;
        if (maxVal === minVal) {
          const pad = Math.max(Math.abs(maxVal) * 0.02, decimals > 0 ? Math.pow(10, -decimals) : 1);
          paddedMin = maxVal - pad;
          paddedMax = maxVal + pad;
        } else {
          const pad = Math.max((maxVal - minVal) * 0.05, Math.abs(maxVal) * 0.01, Math.abs(minVal) * 0.01);
          paddedMin = minVal - pad;
          paddedMax = maxVal + pad;
        }
        return { min: paddedMin, max: paddedMax, span: Math.max(paddedMax - paddedMin, 0.0001) };
      }

      function planTimeTicks(ctxDraw, { mode, startTs, endTs, plotWidth, synced, tz, minTicks = 5, maxTicks = 10, targetTicks = null, labelPadding = 14 }) {
        const normalized = normalizeMode(mode);
        const span = Math.max(1, endTs - startTs);
        const includeDate = normalized === '24h' && dayStamp(startTs, tz) !== dayStamp(endTs, tz);
        const anchorDate = includeDate ? dayStamp(startTs, tz) : '';
        const desiredBase = targetTicks ? Math.max(2, targetTicks) : Math.max(minTicks, Math.round(plotWidth / 90));
        const desired = Math.min(maxTicks, Math.max(2, desiredBase));
        const minSpacing = 48;
        const segments = Math.max(1, desired - 1);
        const rawTicks = [];
        for (let i = 0; i <= segments; i++) {
          rawTicks.push(Math.round(startTs + (span / segments) * i));
        }
        const labelFor = (ts) => formatTimeLabel(ts, normalized, startTs, endTs, synced, tz, includeDate, anchorDate);
        const sampleLabels = rawTicks.map(labelFor);
        const maxLabelWidth = sampleLabels.reduce((m, l) => Math.max(m, ctxDraw.measureText(l).width), 0);
        const spacingNeeded = Math.max(maxLabelWidth + labelPadding, minSpacing);
        const maxCountBySpace = Math.max(2, Math.floor(plotWidth / Math.max(1, spacingNeeded)) + 1);
        const allowed = Math.max(2, Math.min(maxTicks, Math.min(desired, maxCountBySpace)));
        const step = Math.max(1, Math.ceil(rawTicks.length / allowed));
        const ticks = [];
        for (let i = 0; i < rawTicks.length; i += step) ticks.push(rawTicks[i]);
        if (ticks[ticks.length - 1] !== rawTicks[rawTicks.length - 1]) ticks.push(rawTicks[rawTicks.length - 1]);
        const labels = ticks.map(labelFor);
        const finalMaxWidth = labels.reduce((m,l)=>Math.max(m, ctxDraw.measureText(l).width), 0);
        return { ticks, labels, maxLabelWidth: finalMaxWidth };
      }

      function normalizeSeries(points, decimals = 1) {
        const values = (points || []).map(p => p?.v).filter(v => v !== null && typeof v === 'number' && !Number.isNaN(v));
        if (!values.length) return { points: [], min: NaN, max: NaN, last: null, span: 0 };
        const min = Math.min(...values);
        const max = Math.max(...values);
        const spanRaw = max - min;
        const span = spanRaw === 0 ? 1 : spanRaw;
        const normalized = (points || []).map(p => {
          if (!p || p.v === null || Number.isNaN(p.v)) return { t: p?.t ?? 0, v: null };
          const norm = spanRaw === 0 ? 0.5 : (p.v - min) / span;
          return { t: p.t, v: norm };
        });
        const last = values[values.length - 1];
        return { points: normalized, min, max, last: last ?? null, span };
      }

      function drawLineChart(canvas, ctxDraw, points, mode, meta, opts = {}) {
        if (!canvas || !ctxDraw) return false;
        const normalized = normalizeMode(mode);
        const decimals = opts.decimals ?? meta?.decimals ?? 1;
        const yTicks = opts.yTicks ?? 4;
        const fontSize = opts.fontSize || 12;
        const synced = opts.synced ?? clockState.synced;
        const tz = opts.timezone || clockState.timezone;
        const { width, height } = resizeCanvas(canvas, ctxDraw);
        ctxDraw.clearRect(0,0,canvas.width, canvas.height);
        const seriesList = Array.isArray(opts.series) && opts.series.length ? opts.series : [{ id: opts.seriesId || meta?.label || 'series', label: opts.seriesLabel || meta?.label, color: opts.color, points }];
        const prepared = seriesList.map(s => {
          const pts = (s.points || []).filter(p => p && typeof p.t === 'number' && p.v !== null && !Number.isNaN(p.v));
          return { ...s, points: pts };
        }).filter(s => s.points.length);
        if (!prepared.length) return false;
        const allPoints = prepared.flatMap(s => s.points);
        const sorted = [...allPoints].sort((a,b) => a.t - b.t);
        const values = sorted.map(p => p.v);
        const range = computeYRange(values, decimals);
        const firstTs = sorted[0]?.t ?? 0;
        const lastTs = sorted[sorted.length - 1]?.t ?? (firstTs + 1);
        const targetSpan = opts.windowMs ?? (normalized === '24h' ? DAY_MS : (normalized === '6h' ? SIX_H_MS : LIVE_WINDOW_MS));
        const approxNow = getApproxEpochMs();
        const anchorEndTs = opts.anchorEndTs ?? ((synced && approxNow !== null) ? approxNow : null);
        const domainEnd = Math.max(lastTs, anchorEndTs ?? lastTs);
        const domainStart = Math.max(0, domainEnd - targetSpan);
        const timeSpan = Math.max(1, domainEnd - domainStart);
        ctxDraw.strokeStyle = '#1f2937';
        ctxDraw.fillStyle = '#94a3b8';
        ctxDraw.font = `${fontSize}px system-ui`;

        const yLabels = [];
        for (let i=0;i<=yTicks;i++){
          const val = (range.max - (range.span*(i/yTicks))).toFixed(decimals);
          yLabels.push(val);
        }
        const maxYLabelWidth = yLabels.reduce((m,l) => Math.max(m, ctxDraw.measureText(l).width), 0);
        const paddingLeft = Math.max(opts.paddingLeft ?? 44, Math.ceil(maxYLabelWidth + 12));
        const paddingRight = opts.paddingRight ?? 12;
        const paddingTop = opts.paddingTop ?? 12;
        let paddingBottom = Math.max(opts.paddingBottom ?? 32, Math.round(fontSize * 1.4));
        const labelPadding = opts.labelPadding ?? 14;
        let plotWidth = Math.max(1, width - paddingLeft - paddingRight);
        let plotHeight = Math.max(1, height - paddingTop - paddingBottom);
        const tickPlan = planTimeTicks(ctxDraw, { mode: normalized, startTs: domainStart, endTs: domainEnd, plotWidth, synced, tz, minTicks: opts.minXTicks ?? 5, maxTicks: opts.maxXTicks ?? 10, targetTicks: opts.xTicks ? Math.max(2, opts.xTicks + 1) : null, labelPadding });
        const plotBottom = paddingTop + plotHeight;

        for (let i=0;i<=yTicks;i++){
          const y = paddingTop + (plotHeight / yTicks) * i;
          ctxDraw.beginPath(); ctxDraw.moveTo(paddingLeft, y); ctxDraw.lineTo(paddingLeft + plotWidth, y); ctxDraw.stroke();
          const label = yLabels[i];
          ctxDraw.textAlign = 'right';
          ctxDraw.textBaseline = 'middle';
          ctxDraw.fillText(label, paddingLeft - 8, Math.min(plotBottom - 2, y));
        }

        tickPlan.ticks.forEach(ts => {
          const x = paddingLeft + ((ts - domainStart) / timeSpan) * plotWidth;
          ctxDraw.beginPath(); ctxDraw.moveTo(x, paddingTop); ctxDraw.lineTo(x, plotBottom); ctxDraw.stroke();
        });

        tickPlan.ticks.forEach((ts, idx) => {
          const x = paddingLeft + ((ts - domainStart) / timeSpan) * plotWidth;
          const label = tickPlan.labels[idx];
          ctxDraw.textAlign = 'center';
          ctxDraw.textBaseline = 'top';
          ctxDraw.fillText(label, x, plotBottom + 2);
        });

        if (opts.unitLabel) {
          const unitWidth = ctxDraw.measureText(opts.unitLabel).width;
          ctxDraw.textAlign = 'right';
          ctxDraw.textBaseline = 'top';
          ctxDraw.fillText(opts.unitLabel, width - paddingRight, paddingTop + 2);
        }
        prepared.forEach((serie) => {
          const color = serie.color || opts.color || '#22d3ee';
          ctxDraw.strokeStyle = color;
          ctxDraw.lineWidth = serie.lineWidth || opts.lineWidth || 2;
          ctxDraw.beginPath();
          let started=false;
          serie.points.forEach((pt)=>{
            if (!pt || pt.v === null || Number.isNaN(pt.v) || typeof pt.t !== 'number') { started=false; return; }
            const x = paddingLeft + ((pt.t - domainStart)/timeSpan)*plotWidth;
            const y = plotBottom - ((pt.v-range.min)/range.span)*plotHeight;
            if(!started){ctxDraw.moveTo(x,y); started=true;} else {ctxDraw.lineTo(x,y);}
          });
          ctxDraw.stroke();
        });
        return true;
      }

      function drawChart() {
        if (!chartCanvas || !ctx) { pushError('Missing DOM element: chart'); return; }
        const isAllMetrics = chartMetric === 'all';
        const colorSelectWrapper = chartColorSelect ? chartColorSelect.closest('label') : null;
        if (colorSelectWrapper) colorSelectWrapper.style.display = isAllMetrics ? 'none' : 'flex';
        let ok = false;
        if (isAllMetrics) {
          const series = metrics.map(m => {
            const meta = METRIC_META[m] || { label: m.toUpperCase(), unit:'', decimals:1 };
            const data = getSeriesData(m, '24h');
            const deviceId = deviceIdForMetric(m);
            const color = colorForMetricDevice(m, deviceId);
            const normalized = normalizeSeries(data, 2);
            if (!normalized.points.length) return null;
            const rangeText = Number.isFinite(normalized.min) && Number.isFinite(normalized.max) ? `Range: ${normalized.min.toFixed(meta.decimals ?? 1)}–${normalized.max.toFixed(meta.decimals ?? 1)} ${meta.unit}` : '';
            const lastText = normalized.last !== null ? `Letzter Wert: ${normalized.last.toFixed(meta.decimals ?? 1)} ${meta.unit}` : 'Keine gültigen Werte';
            return {
              id: `${m}-${deviceId}`,
              metric: m,
              deviceId,
              label: `${meta.label || m.toUpperCase()} (${meta.unit || ''}) – GeräteID: ${deviceId}`,
              title: `${lastText}${rangeText ? ` • ${rangeText}` : ''}`,
              color,
              points: normalized.points,
            };
          }).filter(Boolean);
          ok = drawLineChart(chartCanvas, ctx, [], '24h', { unit:'normiert', decimals:2 }, {
            series,
            unitLabel: 'normiert (0..1)',
            decimals: 2,
            minXTicks: 6,
            maxXTicks: 10,
            yTicks: 5,
            windowMs: DAY_MS,
            anchorEndTs: getApproxEpochMs() ?? null,
            synced: clockState.synced,
            timezone: clockState.timezone
          });
          if (mainChartLegend) {
            renderLegend(mainChartLegend, ok ? series : [], {
              allowColorPickers: true,
              onColorChange: (item, color) => {
                setColorForMetricDevice(item.metric, item.deviceId, color);
                drawChart();
                tileOrder.forEach(m => { if (tileIsExpanded(m)) drawHover(m); });
              }
            });
          }
        } else {
          const meta = METRIC_META[chartMetric] || { unit:'', decimals:1 };
          const data = getSeriesData(chartMetric, '24h');
          const deviceId = deviceIdForMetric(chartMetric);
          const color = colorForMetricDevice(chartMetric, deviceId);
          const series = [{ id: deviceId, label: `GeräteID: ${deviceId}`, color, points: data }];
          ok = drawLineChart(chartCanvas, ctx, data, '24h', meta, { series, unitLabel: meta.unit, decimals: meta.decimals ?? 1, minXTicks:6, maxXTicks:10, yTicks:5, synced: historyStore[chartMetric]?.synced ?? clockState.synced, timezone: clockState.timezone });
          if (mainChartLegend) renderLegend(mainChartLegend, ok ? series : []);
          if (chartColorSelect) renderColorSelect(chartColorSelect, color);
        }
        if (!ok) {
          const { width } = resizeCanvas(chartCanvas, ctx);
          ctx.clearRect(0,0,chartCanvas.width, chartCanvas.height);
          ctx.fillStyle = '#94a3b8';
          ctx.font = '12px system-ui';
          ctx.fillText('Keine Daten', 6, 18);
        }
      }

      function drawHover(metric) {
        const ctxHover = hoverCanvases[metric];
        if (!ctxHover || !ctxHover.canvas) { pushError(`Missing DOM element: hover canvas for ${metric}`); return; }
        const canvas = ctxHover.canvas;
        const data = getSeriesData(metric, '6h');
        const meta = METRIC_META[metric] || { unit:'', decimals:1 };
        const deviceId = deviceIdForMetric(metric);
        const color = colorForMetricDevice(metric, deviceId);
        const ok = drawLineChart(canvas, ctxHover, data, '6h', meta, {
          series:[{ id: deviceId, color, points: data }],
          minXTicks:6,
          maxXTicks:10,
          yTicks:3,
          fontSize:10,
          lineWidth:1,
          decimals: meta.decimals,
          unitLabel: meta.unit,
          synced: historyStore[metric]?.synced ?? clockState.synced,
          timezone: clockState.timezone,
          windowMs: SIX_H_MS,
          anchorEndTs: getApproxEpochMs() ?? null
        });
        if (!ok) {
          resizeCanvas(canvas, ctxHover);
          ctxHover.clearRect(0,0,canvas.width, canvas.height);
        }
      }

      function drawDetailChart(metric, mode, points, meta) {
        if (!detailCtx || !metric || !detailChartCanvas) return false;
        const deviceId = deviceIdForMetric(metric);
        const color = colorForMetricDevice(metric, deviceId);
        const tickBounds = mode === 'live' ? { minXTicks:5, maxXTicks:8, windowMs: LIVE_WINDOW_MS } : (mode === '6h' ? { minXTicks:6, maxXTicks:10, windowMs: SIX_H_MS } : { minXTicks:6, maxXTicks:10, windowMs: DAY_MS });
        const ok = drawLineChart(detailChartCanvas, detailCtx, points, mode, meta, {
          series:[{ id: deviceId, label:`GeräteID: ${deviceId}`, color, points }],
          unitLabel: meta?.unit,
          decimals: meta?.decimals ?? 1,
          synced: historyStore[metric]?.synced ?? clockState.synced,
          timezone: clockState.timezone,
          minXTicks: tickBounds.minXTicks,
          maxXTicks: tickBounds.maxXTicks,
          yTicks:5,
          windowMs: tickBounds.windowMs,
          anchorEndTs: getApproxEpochMs() ?? null
        });
        if (detailLegend) renderLegend(detailLegend, ok ? [{ id: deviceId, label:`GeräteID: ${deviceId}`, color }] : []);
        if (detailColorSelect) renderColorSelect(detailColorSelect, color);
        const debugBox = getEl('chartDebugText');
        if (devMode) {
          const values = points.map(p => p.v).filter(v => v !== null);
          const min = values.length ? Math.min(...values).toFixed(meta?.decimals ?? 2) : '–';
          const max = values.length ? Math.max(...values).toFixed(meta?.decimals ?? 2) : '–';
          debugBox.textContent = `points: ${values.length} / ${points.length} • min ${min} • max ${max}`;
          debugBox.style.display = 'block';
        } else {
          debugBox.style.display = 'none';
        }
        return ok;
      }

      function avg(arr) {
        const filtered = arr.map(p => p.v).filter(v => v !== null);
        if (!filtered.length) return NaN;
        return filtered.reduce((a,b)=>a+b,0) / filtered.length;
      }

      async function loadDetailHistory(metric, mode) {
        const normalized = normalizeMode(mode);
        const key = `${metric}-${normalized}`;
        if (!detailCache[key]) detailCache[key] = [];
        try {
          const data = await fetchJson(`/api/history?metric=${metric}&range=${normalized}`);
          const mapped = (data.points || []).map(p => {
            const val = (p[1] === null || Number.isNaN(p[1])) ? null : parseFloat(p[1]);
            const ts = parseMs(p[0]);
            return { t: ts !== null ? ts : Date.now(), v: (typeof val === 'number' && !Number.isNaN(val)) ? val : null };
          });
          detailCache[key] = mapped;
          const syncedFlag = flag(data.time_synced);
          if (normalized === 'live' || normalized === '6h' || normalized === '24h') mergeHistory(metric, mapped, normalized, syncedFlag);
          clearErrors('API /api/history');
        } catch (err) {
          console.warn('history error', err);
          pushError(`API /api/history failed: ${err.message}`);
          detailCache[key] = [];
        }
        return detailCache[key];
      }

      async function primeHistory() {
        let clockPrimed = false;
        for (const metric of metrics) {
          for (const range of ['6h','24h']) {
            try {
              const data = await fetchJson(`/api/history?metric=${metric}&range=${range}`);
              const mapped = (data.points || []).map(p => {
                const val = (p[1] === null || Number.isNaN(p[1])) ? null : parseFloat(p[1]);
                const ts = parseMs(p[0]);
                return { t: ts !== null ? ts : Date.now(), v: (typeof val === 'number' && !Number.isNaN(val)) ? val : null };
              });
              const syncedFlag = flag(data.time_synced);
              if (!clockPrimed) {
                const sampleTs = mapped[mapped.length - 1]?.t;
                updateClockState({ timezone: data.timezone, time_synced: data.time_synced, epoch_ms: sampleTs });
                clockPrimed = true;
              }
              mergeHistory(metric, mapped, range, syncedFlag);
              detailCache[`${metric}-${range}`] = mapped;
            } catch (err) {
              pushError(`API /api/history failed: ${err.message}`);
            }
          }
        }
        drawChart();
        updateAverages();
        tileOrder.forEach(m => { if (tileIsExpanded(m)) drawHover(m); });
      }

      function updateAverages() {
        const setAvg = (id, metric) => {
          const el = getEl(id);
          const meta = METRIC_META[metric] || { decimals:1 };
          const val = avg(getSeriesData(metric,'24h'));
          if (el) el.textContent = Number.isNaN(val) ? '–' : val.toFixed(meta.decimals ?? 1);
        };
        setAvg('avgLux', 'lux');
        setAvg('avgCo2', 'co2');
        setAvg('avgTemp', 'temp');
        setAvg('avgHum', 'humidity');
        setAvg('avgVpd', 'vpd');
      }

      function vpdColor(vpd) {
        if (vpd === null || Number.isNaN(vpd)) return 'rgba(15,23,42,0)';
        const stops = [
          { v: 0.0, c: [59,130,246] },
          { v: 0.6, c: [34,197,94] },
          { v: 1.1, c: [234,179,8] },
          { v: 1.6, c: [248,113,113] },
        ];
        const clamp = Math.min(Math.max(vpd, 0), 2.0);
        let idx = stops.findIndex(s => clamp <= s.v);
        if (idx <= 0) return `rgb(${stops[0].c.join(',')})`;
        const a = stops[idx-1], b = stops[idx];
        const t = (clamp - a.v) / (b.v - a.v);
        const mix = a.c.map((v,i) => Math.round(v + (b.c[i]-v)*t));
        return `rgb(${mix.join(',')})`;
      }

      function calcVpd(tempC, humidity) {
        if (tempC === null || humidity === null) return null;
        const es = 0.6108 * Math.exp((17.27 * tempC) / (tempC + 237.3));
        const ea = es * (humidity / 100.0);
        const vpd = es - ea;
        if (Number.isNaN(vpd)) return null;
        return Math.max(0, Math.min(2.5, vpd));
      }

      function humidityForVpd(tempC, vpd) {
        const es = 0.6108 * Math.exp((17.27 * tempC) / (tempC + 237.3));
        const rh = (1 - (vpd / es)) * 100;
        if (Number.isNaN(rh)) return null;
        return Math.max(30, Math.min(100, rh));
      }

      function drawVpdHeatmap(ctxDraw, canvas, mode, targets, overlayEl, markerData) {
        resizeCanvas(canvas, ctxDraw);
        const w = canvas.width;
        const h = canvas.height;
        ctxDraw.clearRect(0,0,w,h);
        const tempMin = 14, tempMax = 35;
        const humMin = 30, humMax = 100;
        const step = 8 * (window.devicePixelRatio || 1);
        for (let y = 0; y < h; y += step) {
          const temp = tempMax - (y / h) * (tempMax - tempMin);
          for (let x = 0; x < w; x += step) {
            const hum = humMin + (x / w) * (humMax - humMin);
            const vpd = calcVpd(temp, hum);
            ctxDraw.fillStyle = vpdColor(vpd);
            ctxDraw.fillRect(x, y, step, step);
          }
        }
        if (targets && targets.low !== null && targets.high !== null) {
          const samples = 32;
          ctxDraw.fillStyle = 'rgba(34,197,94,0.18)';
          ctxDraw.strokeStyle = '#a5f3fc';
          ctxDraw.lineWidth = 2;
          ctxDraw.beginPath();
          for (let i=0;i<samples;i++) {
            const t = tempMin + (i/(samples-1))*(tempMax-tempMin);
            const humHigh = humidityForVpd(t, targets.low);
            const x = ((humHigh - humMin)/(humMax-humMin))*w;
            const y = (1 - (t - tempMin)/(tempMax - tempMin))*h;
            if (i===0) ctxDraw.moveTo(x,y); else ctxDraw.lineTo(x,y);
          }
          for (let i=samples-1;i>=0;i--) {
            const t = tempMin + (i/(samples-1))*(tempMax-tempMin);
            const humLow = humidityForVpd(t, targets.high);
            const x = ((humLow - humMin)/(humMax-humMin))*w;
            const y = (1 - (t - tempMin)/(tempMax - tempMin))*h;
            ctxDraw.lineTo(x,y);
          }
          ctxDraw.closePath();
          ctxDraw.fill();
          ctxDraw.stroke();
        }
        const points = markerData || [];
        points.forEach((pt, idx) => {
          const x = ((pt.hum - humMin) / (humMax - humMin)) * w;
          const y = (1 - (pt.temp - tempMin) / (tempMax - tempMin)) * h;
          ctxDraw.fillStyle = idx === points.length -1 ? '#ef4444' : 'rgba(14,165,233,0.6)';
          ctxDraw.strokeStyle = '#0f172a';
          ctxDraw.lineWidth = 2;
          ctxDraw.beginPath();
          ctxDraw.arc(x, y, idx === points.length -1 ? 6 : 4, 0, Math.PI*2);
          ctxDraw.fill();
          ctxDraw.stroke();
        });
        if (overlayEl) {
          const legend = overlayEl.querySelector('.vpd-legend');
          if (legend) {
            legend.innerHTML = `
              <span class="legend-swatch" style="background:${vpdColor(0.3)}"></span>niedrig
              <span class="legend-swatch" style="background:${vpdColor(0.9)}"></span>ideal
              <span class="legend-swatch" style="background:${vpdColor(1.5)}"></span>hoch
            `;
          }
        }
      }

      function openDetailModal(metric) {
        detailMetric = metric;
        detailMode = 'live';
        detailVpdView = 'heatmap';
        const meta = METRIC_META[metric] || { label: metric.toUpperCase(), unit:'' };
        setText('chartModalTitle', `Detailansicht: ${meta.label}`);
        const badge = getEl('chartModalBadge');
        if (badge) badge.textContent = meta.unit || metric;
        const tabLive = getEl('tabLive');
        const tabLast6h = getEl('tabLast6h');
        const tabLast24h = getEl('tabLast24h');
        if (tabLive) tabLive.classList.add('active');
        if (tabLast6h) tabLast6h.classList.remove('active');
        if (tabLast24h) tabLast24h.classList.remove('active');
        const showVpd = metric === 'vpd';
        setDisplay('vpdViewTabs', showVpd, 'flex');
        if (detailChartCanvas) detailChartCanvas.style.display = showVpd ? 'none' : 'block';
        if (vpdHeatmapCanvas) vpdHeatmapCanvas.style.display = showVpd ? 'block' : 'none';
        setModalVisible(getEl('chartModal'), true);
        renderDetail();
      }

      function closeDetailModal() {
        setModalVisible(getEl('chartModal'), false);
      }

      async function fetchData() {
        try {
          const data = await fetchJson('/api/telemetry');
          lastTelemetryAt = Date.now();
          updateClockState(data);
          const epochMs = parseMs(data.epoch_ms);
          const monoMs = parseMs(data.monotonic_ms ?? data.now);
          const synced = flag(data.time_synced) && epochMs !== null;
          const tsForSample = synced ? epochMs : (monoMs !== null ? monoMs : Date.now());
          updateConnectionStatus(data.ap_mode === 1, data.wifi_connected === 1);
          setText('lux', (typeof data.lux === 'number' && !Number.isNaN(data.lux)) ? data.lux.toFixed(1) : '–');
          setText('ppfd', (typeof data.ppfd === 'number' && !Number.isNaN(data.ppfd)) ? data.ppfd.toFixed(1) : '–');
          setText('ppfdFactor', (typeof data.ppfd_factor === 'number' && !Number.isNaN(data.ppfd_factor)) ? data.ppfd_factor.toFixed(4) : '–');
          const channelSelect = getEl('channel');
          setText('ppfdSpectrum', channelSelect && channelSelect.selectedOptions[0] ? channelSelect.selectedOptions[0].textContent : '–');
          setText('co2', (typeof data.co2 === 'number' && data.co2 > 0) ? data.co2.toFixed(0) : '–');
          setText('temp', (typeof data.temp === 'number' && !Number.isNaN(data.temp)) ? data.temp.toFixed(1) : '–');
          setText('humidity', (typeof data.humidity === 'number' && !Number.isNaN(data.humidity)) ? data.humidity.toFixed(1) : '–');
          setText('leaf', (typeof data.leaf === 'number' && !Number.isNaN(data.leaf)) ? data.leaf.toFixed(1) : '–');
          const vpdOk = flag(data.vpd_ok) && typeof data.vpd === 'number' && !Number.isNaN(data.vpd);
          setText('vpd', vpdOk ? data.vpd.toFixed(3) : '–');
          setText('vpdTarget', `Ziel: ${typeof data.vpd_low === 'number' ? data.vpd_low.toFixed(2) : '–'} – ${typeof data.vpd_high === 'number' ? data.vpd_high.toFixed(2) : '–'} kPa`);
          const statusEl = getEl('vpdStatus');
          const status = data.vpd_status ?? 0;
          if (statusEl) {
            if (!vpdOk) { statusEl.textContent = 'keine Daten'; statusEl.style.color = '#9ca3af'; }
            else if (status < 0) { statusEl.textContent = 'unter Ziel'; statusEl.style.color = '#f59e0b'; }
            else if (status > 0) { statusEl.textContent = 'über Ziel'; statusEl.style.color = '#f87171'; }
            else { statusEl.textContent = 'im Ziel'; statusEl.style.color = '#34d399'; }
          }
          lastVpdTargets = { low: data.vpd_low ?? null, high: data.vpd_high ?? null };
          applyDeviceIdsFromTelemetry(data);
          updateTileHeaderStates(data);
          metrics.forEach(m => recordMetric(m, data[m], tsForSample, synced));
          drawChart();
          updateAverages();
          tileOrder.forEach(m => { if (tileIsExpanded(m)) drawHover(m); });
          renderVpdTile(data);
          renderDetail();
          applyWifiState(data);
        } catch (err) {
          console.warn('telemetry failed', err);
          pushError(`API /api/telemetry failed: ${err.message}`);
          updateConnectionStatus(false, false);
        }
      }

      async function loadSettings() {
        try {
          const data = await fetchJson('/api/settings');
          updateClockState(data);
          setValue('channel', data.channel ?? '');
          setValue('vpdStage', data.vpd_stage ?? '');
          setText('vpdStageStatus', '');
          if (timezoneSelect && data.timezone) timezoneSelect.value = data.timezone;
          const wifiStatus = getEl('wifiStatus');
          if (wifiStatus) {
            wifiStatus.textContent = data.wifi ?? '';
            wifiStatus.className = 'status ' + (data.connected ? 'ok' : 'err');
          }
          setValue('ip', data.ip || '');
          setValue('gw', data.gw || '');
          setValue('sn', data.sn || '');
          const staticToggle = getEl('staticIpToggle');
          if (staticToggle) staticToggle.checked = data.static || false;
          updateStaticIpVisibility();
          applyWifiState({ wifi_connected: data.connected ? 1 : 0, ap_mode: data.ap_mode ? 1 : 0, ip: data.ip, gw: data.gw, sn: data.sn });
          await loadSensors();
          clearErrors('API /api/settings');
        } catch (err) {
          pushError(`API /api/settings failed: ${err.message}`);
        }
      }

      const SENSOR_TEMPLATES = [
        { type:'BH1750', name:'BH1750', category:'light', iface:'i2c', sda:21, scl:22 },
        { type:'TSL2591', name:'TSL2591', category:'light', iface:'i2c', sda:21, scl:22 },
        { type:'SHT31', name:'SHT31', category:'climate', iface:'i2c', sda:21, scl:22 },
        { type:'BME280', name:'BME280', category:'climate', iface:'i2c', sda:21, scl:22 },
        { type:'MLX90614', name:'MLX90614', category:'leaf', iface:'i2c', sda:21, scl:22 },
        { type:'MHZ19', name:'MH-Z19', category:'co2', iface:'uart', rx:16, tx:17 },
        { type:'MHZ14', name:'MH-Z14', category:'co2', iface:'uart', rx:16, tx:17 },
        { type:'SCD30', name:'SCD30', category:'co2', iface:'i2c', sda:21, scl:22 },
        { type:'SCD40', name:'SCD40', category:'co2', iface:'i2c', sda:21, scl:22 },
      ];

      function renderSensors(active, templates) {
        const activeBox = getEl('activeSensors');
        const availBox = getEl('availableSensors');
        if (!activeBox || !availBox) return;
        activeBox.innerHTML = '';
        availBox.innerHTML = '';
        active.forEach(sensor => {
          const tile = document.createElement('div');
          tile.className = 'sensor-tile';
          const dot = document.createElement('span');
          dot.className = 'status-dot';
          dot.style.background = sensor.enabled ? (sensor.healthy ? '#34d399' : '#f59e0b') : '#ef4444';
          const status = document.createElement('div');
          status.className = 'sensor-status';
          status.appendChild(dot);
          const label = document.createElement('span');
          label.textContent = `${sensor.name || sensor.type} (${sensor.category})`;
          status.appendChild(label);
          tile.appendChild(status);
          const tags = document.createElement('div');
          tags.className = 'sensor-actions';
          const btn = document.createElement('button');
          btn.style.width = 'auto';
          btn.textContent = sensor.enabled ? 'Deaktivieren' : 'Aktivieren';
          btn.addEventListener('click', async (ev) => {
            ev.stopPropagation();
            const action = sensor.enabled ? 'disable' : 'enable';
            await authedFetch('/api/sensors/config', { method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body:`action=${action}&id=${sensor.id}`});
            await loadSensors();
          });
          tags.appendChild(btn);
          const hint = document.createElement('div');
          hint.className = 'hover-hint';
          hint.textContent = sensor.enabled ? 'Deaktivieren' : 'Aktivieren';
          tags.appendChild(hint);
          tile.appendChild(tags);
          activeBox.appendChild(tile);
        });

        templates.forEach(tpl => {
          const tile = document.createElement('div');
          tile.className = 'sensor-tile';
          tile.innerHTML = `<div class="sensor-status"><span class="status-dot" style="background:#6b7280;"></span><span>${tpl.name} (${tpl.category})</span></div><div class="tag">${tpl.iface}</div>`;
          tile.addEventListener('click', () => {
            startWizard(tpl);
          });
          availBox.appendChild(tile);
        });
      }

      async function loadSensors() {
        try {
          const data = await fetchJson('/api/sensors/config');
          const active = data.active || [];
          const templates = data.templates || SENSOR_TEMPLATES;
          renderSensors(active, templates);
          applyDeviceIdsFromSensors(active);
          drawChart();
          renderDetail();
          clearErrors('API /api/sensors/config');
        } catch (err) {
          pushError(`API /api/sensors/config failed: ${err.message}`);
        }
      }

      let wizardStep = 1;
      let wizardTemplate = null;
      let wizardAdvanced = false;

      function goWizardStep(step) {
        wizardStep = step;
        document.querySelectorAll('#sensorWizard .wizard-step').forEach(el => el.classList.remove('active'));
        const target = document.querySelector(`#sensorWizard .wizard-step[data-step=\"${step}\"]`);
        if (target) target.classList.add('active');
        const wizNext = getEl('wizNext');
        const wizSave = getEl('wizSave');
        if (wizNext) wizNext.style.display = step < 3 ? 'inline-block' : 'none';
        if (wizSave) wizSave.style.display = step === 3 ? 'inline-block' : 'none';
      }

      function fillWizardSelectors() {
        const catSelect = getEl('sensorCategorySelect');
        const typeSelect = getEl('sensorTypeSelect');
        if (!catSelect || !typeSelect) return;
        catSelect.innerHTML = '';
        typeSelect.innerHTML = '';
        const categories = Array.from(new Set(SENSOR_TEMPLATES.map(t => t.category)));
        categories.forEach(cat => {
          const opt = document.createElement('option');
          opt.value = cat;
          opt.textContent = cat;
          catSelect.appendChild(opt);
        });
        const updateTypeOptions = (cat) => {
          if (!typeSelect) return;
          typeSelect.innerHTML = '';
          SENSOR_TEMPLATES.filter(t => t.category === cat).forEach(tpl => {
            const opt = document.createElement('option');
            opt.value = tpl.type;
            opt.textContent = `${tpl.name}`;
            typeSelect.appendChild(opt);
          });
        };
        updateTypeOptions(categories[0]);
      }

      function populateWizardPins(tpl) {
        setValue('wizSda', tpl.sda ?? '');
        setValue('wizScl', tpl.scl ?? '');
        setValue('wizRx', tpl.rx ?? '');
        setValue('wizTx', tpl.tx ?? '');
      }

      function startWizard(tpl) {
        wizardTemplate = tpl || SENSOR_TEMPLATES[0];
        wizardAdvanced = false;
        fillWizardSelectors();
        setValue('sensorCategorySelect', wizardTemplate.category);
        const typeSelect = getEl('sensorTypeSelect');
        if (typeSelect) {
          typeSelect.innerHTML = '';
          SENSOR_TEMPLATES.filter(t => t.category === wizardTemplate.category).forEach(item => {
            const opt = document.createElement('option');
            opt.value = item.type;
            opt.textContent = `${item.name}`;
            typeSelect.appendChild(opt);
          });
          typeSelect.value = wizardTemplate.type;
        }
        populateWizardPins(wizardTemplate);
        setText('defaultPinsInfo', 'Standardpins werden genutzt.');
        ['wizSda','wizScl','wizRx','wizTx'].forEach(id => {
          const el = getEl(id);
          if (el) el.disabled = true;
        });
        setDisplay('advancedPinsWarn', false);
        goWizardStep(1);
        wizardOpenedAt = Date.now();
        setModalVisible(getEl('sensorWizard'), true);
      }

      const sensorCategorySelect = getEl('sensorCategorySelect');
      if (sensorCategorySelect) {
        sensorCategorySelect.addEventListener('change', () => {
          const cat = sensorCategorySelect.value;
          const typeSelect = getEl('sensorTypeSelect');
          if (!typeSelect) return;
          typeSelect.innerHTML = '';
          SENSOR_TEMPLATES.filter(t => t.category === cat).forEach(tpl => {
            const opt = document.createElement('option');
            opt.value = tpl.type;
            opt.textContent = `${tpl.name}`;
            typeSelect.appendChild(opt);
          });
          const tpl = SENSOR_TEMPLATES.find(t => t.category === cat) || SENSOR_TEMPLATES[0];
          wizardTemplate = tpl;
          typeSelect.value = tpl.type;
          populateWizardPins(tpl);
        });
      }

      const sensorTypeSelect = getEl('sensorTypeSelect');
      if (sensorTypeSelect) {
        sensorTypeSelect.addEventListener('change', () => {
          const type = sensorTypeSelect.value;
          const tpl = SENSOR_TEMPLATES.find(t => t.type === type) || wizardTemplate;
          wizardTemplate = tpl;
          setValue('sensorCategorySelect', tpl.category);
          populateWizardPins(tpl);
        });
      }

      const addSensorBtn = getEl('addSensorBtn');
      if (addSensorBtn) addSensorBtn.addEventListener('click', () => startWizard(SENSOR_TEMPLATES[0]));
      const sensorWizardClose = getEl('sensorWizardClose');
      if (sensorWizardClose) sensorWizardClose.addEventListener('click', () => setModalVisible(getEl('sensorWizard'), false));
      const sensorWizard = getEl('sensorWizard');
      if (sensorWizard) {
        sensorWizard.addEventListener('click', (e) => {
          if (e.target.id === 'sensorWizard' && (Date.now() - wizardOpenedAt) > 150) setModalVisible(getEl('sensorWizard'), false);
        });
      }
      const wizBack = getEl('wizBack');
      if (wizBack) wizBack.addEventListener('click', () => goWizardStep(Math.max(1, wizardStep - 1)));
      const wizNext = getEl('wizNext');
      if (wizNext) wizNext.addEventListener('click', () => {
        if (wizardStep === 1) {
          const type = sensorTypeSelect ? sensorTypeSelect.value : '';
          const tpl = SENSOR_TEMPLATES.find(t => t.type === type) || wizardTemplate;
          wizardTemplate = tpl;
          populateWizardPins(tpl);
        }
        goWizardStep(Math.min(3, wizardStep + 1));
      });
      const advancedPinsBtn = getEl('advancedPinsBtn');
      if (advancedPinsBtn) advancedPinsBtn.addEventListener('click', () => {
        if (!confirm('Wirklich Pinbelegung ändern?')) return;
        wizardAdvanced = true;
        setDisplay('advancedPinsWarn', true, 'block');
        ['wizSda','wizScl','wizRx','wizTx'].forEach(id => {
          const el = getEl(id);
          if (el) el.disabled = false;
        });
      });
      const wizSave = getEl('wizSave');
      if (wizSave) wizSave.addEventListener('click', async () => {
        const body = new URLSearchParams();
        body.set('action', 'add');
        const typeEl = getEl('sensorTypeSelect');
        const catEl = getEl('sensorCategorySelect');
        const typeVal = typeEl ? typeEl.value : '';
        const catVal = catEl ? catEl.value : '';
        body.set('sensor_type', typeVal);
        body.set('category', catVal);
        const id = `${typeVal.toLowerCase()}_${Date.now()}`;
        body.set('id', id);
        await authedFetch('/api/sensors/config', { method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body: body.toString() });
        if (wizardAdvanced) {
          const pins = new URLSearchParams();
          pins.set('action','set_pins');
          pins.set('id', id);
          pins.set('sda', (getEl('wizSda')?.value) || '-1');
          pins.set('scl', (getEl('wizScl')?.value) || '-1');
          pins.set('rx', (getEl('wizRx')?.value) || '-1');
          pins.set('tx', (getEl('wizTx')?.value) || '-1');
          await authedFetch('/api/sensors/config', { method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body: pins.toString() });
        }
        setText('wizStatus', 'Gespeichert. Bitte Neustart durchführen.');
        const wizStatus = getEl('wizStatus');
        if (wizStatus) wizStatus.className = 'status ok';
        goWizardStep(3);
        await loadSensors();
      });
      const restartNowBtn = getEl('restartNow');
      if (restartNowBtn) restartNowBtn.addEventListener('click', async () => {
        await authedFetch('/api/reset', { method:'POST' });
      });

      async function scanNetworks() {
        setText('wifiStatus', 'Suche Netzwerke...');
        try {
          const data = await fetchJson('/api/networks');
          const ssidSelect = getEl('ssid');
          if (ssidSelect) {
            ssidSelect.innerHTML = '';
            (data.networks || []).forEach(n => {
              const opt = document.createElement('option');
              opt.value = n.ssid;
              opt.textContent = `${n.ssid} (${n.rssi}dBm)`;
              ssidSelect.appendChild(opt);
            });
            if ((data.networks || []).length === 0) {
              const opt = document.createElement('option');
              opt.textContent = 'Keine Netzwerke gefunden';
              ssidSelect.appendChild(opt);
            }
          }
          setText('wifiStatus', 'Suche abgeschlossen');
          clearErrors('API /api/networks');
        } catch (err) {
          pushError(`API /api/networks failed: ${err.message}`);
        }
      }

      function updateStaticIpVisibility() {
        const toggle = getEl('staticIpToggle');
        const checked = toggle ? toggle.checked : false;
        const wrapper = getEl('staticIpRow');
        if (wrapper) wrapper.classList.toggle('hidden', !checked);
      }

      if (timezoneSelect) {
        timezoneSelect.addEventListener('change', async () => {
          const tz = timezoneSelect.value;
          try {
            const res = await authedFetch('/api/settings', { method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body:`timezone=${encodeURIComponent(tz)}` });
            if (!res.ok) {
              const text = await res.text().catch(() => '');
              throw new Error(`status ${res.status} ${text}`);
            }
            clockState.timezone = tz;
            updateClockState({ timezone: tz, time_synced: clockState.synced, epoch_ms: getApproxEpochMs() });
          } catch (err) {
            pushError(`API /api/settings failed: ${err.message}`);
          }
        });
      }

      const saveChannelBtn = getEl('saveChannel');
      if (saveChannelBtn) saveChannelBtn.addEventListener('click', async () => {
        const channel = getEl('channel')?.value || '';
        try {
          const res = await authedFetch('/api/settings', { method: 'POST', headers: {'Content-Type':'application/x-www-form-urlencoded'}, body: `channel=${channel}` });
          if (!res.ok) {
            const text = await res.text().catch(() => '');
            throw new Error(`status ${res.status} ${text}`);
          }
          setText('wifiStatus', 'Spektrum gespeichert');
        } catch (err) {
          pushError(`API /api/settings failed: ${err.message}`);
        }
      });

      const applyStageBtn = getEl('applyStage');
      if (applyStageBtn) applyStageBtn.addEventListener('click', async () => {
        const vpdStage = getEl('vpdStage')?.value || '';
        const channel = getEl('channel')?.value || '';
        const status = getEl('vpdStageStatus');
        if (status) {
          status.textContent = 'Speichern...';
          status.className = 'status';
        }
        try {
          const res = await authedFetch('/api/settings', { method: 'POST', headers: {'Content-Type':'application/x-www-form-urlencoded'}, body: `channel=${channel}&vpd_stage=${vpdStage}` });
          const text = await res.text().catch(() => '');
          if (res.ok) {
            if (status) { status.textContent = 'VPD Targets aktualisiert'; status.classList.add('ok'); }
            await fetchData();
          } else {
            if (status) { status.textContent = 'Speichern fehlgeschlagen'; status.classList.add('err'); }
            pushError(`API /api/settings failed: ${res.status} ${text}`);
          }
        } catch (err) {
          pushError(`API /api/settings failed: ${err.message}`);
          if (status) { status.textContent = 'Speichern fehlgeschlagen'; status.classList.add('err'); }
        }
      });

      const saveWifiBtn = getEl('saveWifi');
      if (saveWifiBtn) saveWifiBtn.addEventListener('click', async () => {
        const wifiStatus = getEl('wifiStatus');
        if (!devMode) { if (wifiStatus) { wifiStatus.textContent = 'Dev-Modus erforderlich'; wifiStatus.className='status err'; } return; }
        const ssid = getEl('ssid')?.value || '';
        const pass = getEl('pass')?.value || '';
        const staticIp = getEl('staticIpToggle')?.checked || false;
        const ip = getEl('ip')?.value || '';
        const gw = getEl('gw')?.value || '';
        const sn = getEl('sn')?.value || '';
        if (wifiStatus) wifiStatus.textContent = 'Speichern & Neustart...';
        try {
          const res = await authedFetch('/api/wifi', { method: 'POST', headers: {'Content-Type':'application/x-www-form-urlencoded'}, body: `ssid=${encodeURIComponent(ssid)}&pass=${encodeURIComponent(pass)}&static=${staticIp ? 1 : 0}&ip=${encodeURIComponent(ip)}&gw=${encodeURIComponent(gw)}&sn=${encodeURIComponent(sn)}` });
          const text = await res.text().catch(() => '');
          if (res.ok) {
            if (wifiStatus) { wifiStatus.textContent = 'Verbunden mit deiner Pflanze'; wifiStatus.className = 'status ok'; }
          } else {
            if (wifiStatus) { wifiStatus.textContent = 'Falsches Passwort'; wifiStatus.className = 'status err'; }
            pushError(`API /api/wifi failed: ${res.status} ${text}`);
          }
        } catch (err) {
          pushError(`API /api/wifi failed: ${err.message}`);
          if (wifiStatus) { wifiStatus.textContent = 'Speichern fehlgeschlagen'; wifiStatus.className = 'status err'; }
        }
      });

      const resetWifiBtn = getEl('resetWifi');
      if (resetWifiBtn) resetWifiBtn.addEventListener('click', async () => {
        const wifiStatus = getEl('wifiStatus');
        if (!devMode) { if (wifiStatus) { wifiStatus.textContent = 'Dev-Modus erforderlich'; wifiStatus.className='status err'; } return; }
        if (wifiStatus) wifiStatus.textContent = 'Werkseinstellungen...';
        await authedFetch('/api/reset', { method: 'POST' });
        setTimeout(() => location.reload(), 1500);
      });

      const staticToggle = getEl('staticIpToggle');
      if (staticToggle) staticToggle.addEventListener('change', updateStaticIpVisibility);

      async function loadLogs() {
        try {
          const data = await fetchJson('/api/logs');
          setText('logBox', (data || []).join('\n'));
          clearErrors('API /api/logs');
        } catch (err) {
          pushError(`API /api/logs failed: ${err.message}`);
        }
      }

      async function downloadLogs() {
        try {
          const data = await fetchJson('/api/logs');
          const blob = new Blob([(data || []).join('\n')], { type: 'text/plain' });
          const url = URL.createObjectURL(blob);
          const a = document.createElement('a');
          a.href = url;
          a.download = 'growsensor-log.txt';
          a.click();
          URL.revokeObjectURL(url);
        } catch (err) {
          pushError(`API /api/logs failed: ${err.message}`);
        }
      }

      const scanWifiBtn = getEl('scanWifi');
      if (scanWifiBtn) scanWifiBtn.addEventListener('click', scanNetworks);
      const refreshLogsBtn = getEl('refreshLogs');
      if (refreshLogsBtn) refreshLogsBtn.addEventListener('click', loadLogs);
      const downloadLogsBtn = getEl('downloadLogs');
      if (downloadLogsBtn) downloadLogsBtn.addEventListener('click', downloadLogs);
      async function loadPartners() {
        try {
          const data = await fetchJson('/api/partners');
          const list = getEl('partnerList');
          if (list) {
            list.innerHTML = '';
            (data.partners || []).forEach(p => {
              const card = document.createElement('div');
              card.className = 'card';
              card.style.marginTop = '8px';
              card.innerHTML = `<strong>${p.name}</strong><br/><span style=\"color:#9ca3af;\">${p.description}</span>` + (p.url ? `<br/><a href=\"${p.url}\" target=\"_blank\">Link</a>` : '');
              list.appendChild(card);
            });
          }
          clearErrors('API /api/partners');
        } catch (err) {
          pushError(`API /api/partners failed: ${err.message}`);
        }
      }

      const savePartnerBtn = getEl('savePartner');
      if (savePartnerBtn) savePartnerBtn.addEventListener('click', async () => {
        if (!devMode) { return; }
        const body = new URLSearchParams();
        body.set('id', getEl('partnerId')?.value || '');
        body.set('name', getEl('partnerName')?.value || '');
        body.set('description', getEl('partnerDesc')?.value || '');
        body.set('url', getEl('partnerUrl')?.value || '');
        body.set('logo', getEl('partnerLogo')?.value || '');
        body.set('enabled', getEl('partnerEnabled')?.checked ? '1' : '0');
        await authedFetch('/api/partners', { method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body: body.toString() });
        await loadPartners();
      });

      document.querySelectorAll('.metric-tile').forEach(card => {
        const metric = card.dataset.metric;
        card.addEventListener('mouseenter', () => {
          if (!tileIsExpanded(metric)) return;
          drawHover(metric);
        });
      });

      document.querySelectorAll('.tile-eye').forEach(btn => {
        const metric = btn.dataset.metric;
        btn.addEventListener('click', (e) => {
          e.stopPropagation();
          e.preventDefault();
          if (!metric) return;
          setTileExpanded(metric, !tileIsExpanded(metric));
        });
      });

      document.addEventListener('click', (e) => {
        if (!(e.target instanceof Element)) return;
        const tile = e.target.closest('.metric-tile');
        if (tile) {
          const metric = tile.dataset.metric;
          if (!metric) return;
          if (e.target.closest('.tile-eye')) return;
          if (!tileIsExpanded(metric)) {
            setTileExpanded(metric, true);
            return;
          }
          openDetailModal(metric);
        }
      });

      const chartModalEl = getEl('chartModal');
      if (chartModalEl) chartModalEl.addEventListener('click', (e) => {
        if (e.target.id === 'chartModal') closeDetailModal();
      });
      const chartClose = getEl('chartClose');
      if (chartClose) chartClose.addEventListener('click', closeDetailModal);
      const tabLiveBtn = getEl('tabLive');
      if (tabLiveBtn) tabLiveBtn.addEventListener('click', () => {
        detailMode = 'live';
        if (tabLiveBtn) tabLiveBtn.classList.add('active');
        const tabLast6h = getEl('tabLast6h');
        const tabLast24h = getEl('tabLast24h');
        if (tabLast6h) tabLast6h.classList.remove('active');
        if (tabLast24h) tabLast24h.classList.remove('active');
        renderDetail();
      });
      const tabLast6hBtn = getEl('tabLast6h');
      if (tabLast6hBtn) tabLast6hBtn.addEventListener('click', () => {
        detailMode = '6h';
        tabLast6hBtn.classList.add('active');
        const tabLiveEl = getEl('tabLive');
        const tabLast24h = getEl('tabLast24h');
        if (tabLiveEl) tabLiveEl.classList.remove('active');
        if (tabLast24h) tabLast24h.classList.remove('active');
        renderDetail();
      });
      const tabLast24hBtn = getEl('tabLast24h');
      if (tabLast24hBtn) tabLast24hBtn.addEventListener('click', () => {
        detailMode = '24h';
        tabLast24hBtn.classList.add('active');
        const tabLiveEl = getEl('tabLive');
        const tabLast6h = getEl('tabLast6h');
        if (tabLiveEl) tabLiveEl.classList.remove('active');
        if (tabLast6h) tabLast6h.classList.remove('active');
        renderDetail();
      });
      const tabHeatmapBtn = getEl('tabHeatmap');
      if (tabHeatmapBtn) tabHeatmapBtn.addEventListener('click', () => {
        detailVpdView = 'heatmap';
        tabHeatmapBtn.classList.add('active');
        const tabTimeline = getEl('tabTimeline');
        if (tabTimeline) tabTimeline.classList.remove('active');
        renderDetail();
      });
      const tabTimelineBtn = getEl('tabTimeline');
      if (tabTimelineBtn) tabTimelineBtn.addEventListener('click', () => {
        detailVpdView = 'timeline';
        tabTimelineBtn.classList.add('active');
        const tabHeatmap = getEl('tabHeatmap');
        if (tabHeatmap) tabHeatmap.classList.remove('active');
        renderDetail();
      });

      function switchView(target) {
        document.querySelectorAll('.view').forEach(v => v.classList.remove('active'));
        const viewEl = getEl(target);
        if (viewEl) viewEl.classList.add('active');
      }
      const navDashboard = getEl('navDashboard');
      if (navDashboard) navDashboard.addEventListener('click', () => switchView('view-dashboard'));
      const navSensors = getEl('navSensors');
      if (navSensors) navSensors.addEventListener('click', () => switchView('view-sensors'));
      if (chartMetricSelect) {
        chartMetricSelect.addEventListener('change', () => {
          chartMetric = chartMetricSelect.value || 'temp';
          drawChart();
        });
      }
      if (chartColorSelect) {
        chartColorSelect.addEventListener('change', () => {
          if (!chartMetric || chartMetric === 'all') return;
          const deviceId = deviceIdForMetric(chartMetric);
          const color = chartColorSelect.value || COLOR_PALETTE[0].value;
          setColorForMetricDevice(chartMetric, deviceId, color);
          drawChart();
          tileOrder.forEach(m => { if (tileIsExpanded(m)) drawHover(m); });
          renderDetail();
        });
      }
      if (detailColorSelect) {
        detailColorSelect.addEventListener('change', () => {
          if (!detailMetric) return;
          const deviceId = deviceIdForMetric(detailMetric);
          const color = detailColorSelect.value || COLOR_PALETTE[0].value;
          setColorForMetricDevice(detailMetric, deviceId, color);
          drawChart();
          tileOrder.forEach(m => { if (tileIsExpanded(m)) drawHover(m); });
          renderDetail();
        });
      }

      function applyWifiState(data = {}) {
        const connected = flag(data?.wifi_connected);
        const ap = flag(data?.ap_mode);
        const showConnected = connected && !ap;
        const form = getEl('wifiForm');
        const block = getEl('wifiConnectedBlock');
        if (block) {
          block.classList.toggle('hidden', !showConnected);
          if (showConnected) {
            setText('wifiIp', data.ip || data.ip_active || '–');
            setText('wifiSsid', data.ssid || data.ssid_active || '–');
            updateWifiBars(typeof data.rssi === 'number' ? data.rssi : null);
          } else {
            setText('wifiIp', '–');
            setText('wifiSsid', '–');
            updateWifiBars(null);
          }
        }
        if (form) {
          const showForm = wifiFormOpen || !showConnected;
          form.classList.toggle('hidden', !showForm);
        }
        const toggleBtn = getEl('toggleWifiForm');
        if (toggleBtn) toggleBtn.textContent = wifiFormOpen ? 'Abbrechen' : 'Wi-Fi ändern';
      }

      function updateWifiBars(rssi) {
        if (!wifiBars) return;
        wifiBars.innerHTML = '';
        const level = typeof rssi === 'number' ? (rssi >= -50 ? 4 : rssi >= -60 ? 3 : rssi >= -70 ? 2 : rssi >= -80 ? 1 : 0) : 0;
        for (let i=1;i<=4;i++) {
          const bar = document.createElement('div');
          bar.className = 'wifi-bar';
          if (i <= level) bar.classList.add('active');
          bar.style.height = `${6 + i*3}px`;
          wifiBars.appendChild(bar);
        }
      }

      const toggleWifiFormBtn = getEl('toggleWifiForm');
      if (toggleWifiFormBtn) toggleWifiFormBtn.addEventListener('click', () => {
        wifiFormOpen = !wifiFormOpen;
        applyWifiState(lastTelemetryPayload);
      });

      function setDevVisible() {
        document.querySelectorAll('.dev-only').forEach(el => el.disabled = !devMode);
        const wifiDevNote = getEl('wifiDevNote');
        const partnerCard = getEl('partnerCard');
        const devStatus = getEl('devStatus');
        if (wifiDevNote) wifiDevNote.style.display = devMode ? 'none' : 'block';
        if (partnerCard) partnerCard.style.display = devMode ? 'block' : 'none';
        if (devStatus) devStatus.style.display = devMode ? 'inline-block' : 'none';
      }

      const openDevBtn = getEl('openDev');
      if (openDevBtn) openDevBtn.addEventListener('click', () => {
        setModalVisible(getEl('devModal'), true);
        setText('devStatusMsg', '');
        setValue('devCode', '');
      });
      const cancelDevBtn = getEl('cancelDev');
      if (cancelDevBtn) cancelDevBtn.addEventListener('click', () => setModalVisible(getEl('devModal'), false));
      const activateDevBtn = getEl('activateDev');
      if (activateDevBtn) activateDevBtn.addEventListener('click', () => {
        const code = getEl('devCode')?.value || '';
        const status = getEl('devStatusMsg');
        if (!status) return;
        if (code === DEV_CODE) {
          devMode = true;
          status.textContent = 'Dev aktiviert';
          status.className = 'status ok';
          setModalVisible(getEl('devModal'), false);
          setDevVisible();
        } else {
          status.textContent = 'Falscher Code';
          status.className = 'status err';
        }
      });

      async function renderDetail() {
        if (!detailMetric) return;
        const meta = METRIC_META[detailMetric] || { unit:'', label: detailMetric.toUpperCase(), decimals:1 };
        if (yAxisLabel) yAxisLabel.textContent = `${meta.label} (${meta.unit})`;
        if (xAxisLabel) {
          if (clockState.synced) {
            if (detailMode === 'live') xAxisLabel.textContent = 'Zeit (HH:MM:SS lokal)';
            else if (detailMode === '24h') xAxisLabel.textContent = 'Zeit (HH:MM lokal, 24h)';
            else xAxisLabel.textContent = 'Zeit (HH:MM lokal)';
          } else {
            xAxisLabel.textContent = detailMode === 'live' ? 'Zeit (mm:ss seit Start)' : 'Zeit (HH:MM relativ)';
          }
        }
        const showHeatmap = detailMetric === 'vpd' && detailVpdView === 'heatmap';
        if (vpdHeatmapCanvas) vpdHeatmapCanvas.style.display = showHeatmap ? 'block' : 'none';
        if (detailChartCanvas) detailChartCanvas.style.display = showHeatmap ? 'none' : 'block';
        if (chartEmpty) chartEmpty.style.display = 'none';
        if (showHeatmap) {
          if (detailLegend) renderLegend(detailLegend, []);
          if (detailColorSelect) renderColorSelect(detailColorSelect, colorForMetricDevice(detailMetric, deviceIdForMetric(detailMetric)));
          const points = collectPaired(detailMode);
          drawVpdHeatmap(vpdHeatmapCtx, vpdHeatmapCanvas, detailMode, lastVpdTargets, getEl('chartModalCard'), points);
        } else {
          const historyPoints = await loadDetailHistory(detailMetric, detailMode);
          const ok = drawDetailChart(detailMetric, detailMode, historyPoints, meta);
          if (chartEmpty) chartEmpty.style.display = ok ? 'none' : 'flex';
        }
      }

      function renderVpdTile(data) {
        const temp = typeof data.temp === 'number' ? data.temp : null;
        const hum = typeof data.humidity === 'number' ? data.humidity : null;
        const vpd = typeof data.vpd === 'number' && !Number.isNaN(data.vpd) ? data.vpd : null;
        const tileStatus = getEl('vpdTileStatus');
        const tileNoData = getEl('vpdTileNoData');
        const points = collectPaired('live').slice(-60);
        if (!tileStatus || !tileNoData) return;
        if (temp === null || hum === null || vpd === null || Number.isNaN(temp) || Number.isNaN(hum) || points.length === 0) {
          tileNoData.style.display = 'flex';
          tileStatus.textContent = 'keine Daten';
          return;
        }
        tileNoData.style.display = 'none';
        tileStatus.textContent = `VPD ${vpd.toFixed(2)} kPa`;
        drawVpdHeatmap(vpdTileCtx, vpdTileCanvas, 'live', lastVpdTargets, getEl('vpdTileChart'), points);
        const targetText = lastVpdTargets.low && lastVpdTargets.high ? `${lastVpdTargets.low.toFixed(2)}–${lastVpdTargets.high.toFixed(2)} kPa` : '–';
        const vpdTileTarget = getEl('vpdTileTarget');
        if (vpdTileTarget) vpdTileTarget.innerHTML = `<span class=\"legend-swatch\" style=\"background:${vpdColor(lastVpdTargets.low || 0.8)}\"></span> Ziel ${targetText}`;
      }

      setInterval(fetchData, 2500);
      setInterval(renderTimeStatus, 1000);
      populateTimezoneSelect();
      setDevVisible();
      renderTimeStatus();
      loadSettings();
      scanNetworks();
      loadLogs();
      loadPartners();
      primeHistory();
      fetchData();
    </script>
  </body>
  </html>
  )HTML";


void handleRoot() {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleNotFound() {
  if (apMode) {
    // Captive portal redirect while the access point is active
    server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString(), true);
    server.send(302, "text/plain", "");
    return;
  }
  server.send(404, "text/plain", "Not found");
}

void handlePartners() {
  if (!enforceAuth())
    return;
  if (server.method() == HTTP_POST) {
    if (!server.hasArg("id")) {
      server.send(400, "text/plain", "missing id");
      return;
    }
    String id = server.arg("id");
    Partner *existing = nullptr;
    for (auto &p : partners) {
      if (p.id == id) {
        existing = &p;
        break;
      }
    }
    Partner p = existing ? *existing : Partner();
    p.id = id;
    if (server.hasArg("name")) p.name = server.arg("name");
    if (server.hasArg("description")) p.description = server.arg("description");
    if (server.hasArg("url")) p.url = server.arg("url");
    if (server.hasArg("logo")) p.logo = server.arg("logo");
    if (server.hasArg("enabled")) p.enabled = server.arg("enabled") == "1";
    if (existing) {
      *existing = p;
    } else {
      partners.push_back(p);
    }
    savePartners();
    server.send(200, "text/plain", "saved");
    return;
  }

  size_t enabledCount = 0;
  for (const auto &p : partners) {
    if (p.enabled)
      enabledCount++;
  }
  String json;
  json.reserve(48 + enabledCount * 160);
  json = "{\"partners\":[";
  size_t count = 0;
  for (size_t i = 0; i < partners.size(); i++) {
    const auto &p = partners[i];
    if (!p.enabled)
      continue;
    if (count++ > 0) json += ",";
    json += "{\"id\":\"" + p.id + "\",\"name\":\"" + p.name + "\",\"description\":\"" + p.description + "\",\"url\":\"" + p.url + "\",\"logo\":\"" + p.logo + "\"}";
  }
  json += "]}";
  server.send(200, "application/json", json);
}

void handleTelemetry() {
  pruneLogsIfLowMemory(false);
  bool wifiConnected = WiFi.status() == WL_CONNECTED;
  IPAddress activeIp = apMode ? WiFi.softAPIP() : WiFi.localIP();
  String ipStr = activeIp.toString();
  String gwStr = WiFi.gatewayIP().toString();
  String snStr = WiFi.subnetMask().toString();
  int rssi = wifiConnected ? WiFi.RSSI() : 0;
  String ssid = wifiConnected ? WiFi.SSID() : (apMode ? String(AP_SSID) : savedSsid);
  bool luxOk = enableLight && lightHealth.present && lightHealth.healthy && !isnan(latest.lux);
  bool climateOk = enableClimate && climateHealth.present && climateHealth.healthy && !isnan(latest.ambientTempC) && !isnan(latest.humidity);
  bool leafOk = enableLeaf && leafHealth.present && leafHealth.healthy && !isnan(latest.leafTempC);
  bool co2Ok = enableCo2 && co2Health.present && co2Health.healthy && latest.co2ppm > 0 && latest.co2ppm < 5000;
  bool vpdOk = !isnan(latest.vpd);
  unsigned long now = millis();
  uint64_t epochMs = mapTimestampToEpochMs(now);
  auto ageMs = [&](unsigned long t) -> unsigned long { return t == 0 ? 0UL : (now - t); };
  auto sensorLabel = [](String v) { v.toUpperCase(); return v; };
  String luxDev = sensorLabel(findSensor("lux") ? findSensor("lux")->type : "BH1750");
  String climateDev = sensorLabel(findSensor("climate") ? findSensor("climate")->type : climateSensorName(climateType));
  String leafDev = sensorLabel(findSensor("leaf") ? findSensor("leaf")->type : "MLX90614");
  String co2Dev = sensorLabel(findSensor("co2") ? findSensor("co2")->type : co2SensorName(co2Type));
  char json[1400];
  snprintf(json, sizeof(json),
           "{\"lux\":%.1f,\"ppfd\":%.1f,\"ppfd_factor\":%.4f,\"co2\":%d,\"temp\":%.1f,\"humidity\":%.1f,\"leaf\":%.1f,"
           "\"vpd\":%.3f,\"vpd_low\":%.2f,\"vpd_high\":%.2f,\"vpd_status\":%d,"
           "\"wifi_connected\":%d,\"ap_mode\":%d,\"ip\":\"%s\",\"gw\":\"%s\",\"sn\":\"%s\",\"rssi\":%d,\"ssid\":\"%s\","
           "\"lux_ok\":%d,\"co2_ok\":%d,\"climate_ok\":%d,\"leaf_ok\":%d,\"vpd_ok\":%d,"
           "\"lux_present\":%d,\"co2_present\":%d,\"climate_present\":%d,\"leaf_present\":%d,"
           "\"lux_enabled\":%d,\"co2_enabled\":%d,\"climate_enabled\":%d,\"leaf_enabled\":%d,"
           "\"lux_age_ms\":%lu,\"co2_age_ms\":%lu,\"climate_age_ms\":%lu,\"leaf_age_ms\":%lu,"
           "\"now\":%lu,\"monotonic_ms\":%lu,\"epoch_ms\":%llu,\"time_synced\":%d,\"timezone\":\"%s\","
           "\"lux_device\":\"%s\",\"climate_device\":\"%s\",\"leaf_device\":\"%s\",\"co2_device\":\"%s\"}",
           safeFloat(latest.lux), safeFloat(latest.ppfd), safeFloat(latest.ppfdFactor), safeInt(latest.co2ppm, -1), safeFloat(latest.ambientTempC),
           safeFloat(latest.humidity), safeFloat(latest.leafTempC), safeFloat(latest.vpd), safeFloat(latest.vpdTargetLow),
           safeFloat(latest.vpdTargetHigh), latest.vpdStatus,
           wifiConnected ? 1 : 0, apMode ? 1 : 0, ipStr.c_str(), gwStr.c_str(), snStr.c_str(), rssi, ssid.c_str(),
           luxOk ? 1 : 0, co2Ok ? 1 : 0, climateOk ? 1 : 0, leafOk ? 1 : 0, vpdOk ? 1 : 0,
           lightHealth.present ? 1 : 0, co2Health.present ? 1 : 0, climateHealth.present ? 1 : 0, leafHealth.present ? 1 : 0,
           enableLight ? 1 : 0, enableCo2 ? 1 : 0, enableClimate ? 1 : 0, enableLeaf ? 1 : 0,
           ageMs(lightHealth.lastUpdate), ageMs(co2Health.lastUpdate), ageMs(climateHealth.lastUpdate), ageMs(leafHealth.lastUpdate),
           now, now, (unsigned long long)epochMs, timeSynced ? 1 : 0, timezoneName.c_str(),
           luxDev.c_str(), climateDev.c_str(), leafDev.c_str(), co2Dev.c_str());
  server.send(200, "application/json", json);
}

void handleHistory() {
  if (!enforceAuth())
    return;
  String metric = server.hasArg("metric") ? server.arg("metric") : "";
  String range = server.hasArg("range") ? server.arg("range") : "live";
  range.trim();
  range.toLowerCase();
  MetricDef *def = historyById(metric);
  if (!def) {
    server.send(400, "text/plain", "invalid metric");
    return;
  }

  bool range24h = range == "24h" || range == "24hr" || range == "24hrs" || range == "day";
  bool range6h = range == "6h" || range == "6hr" || range == "6hrs";
  String normalized = range24h ? "24h" : (range6h ? "6h" : "live");
  uint64_t now = currentEpochMs();
  flushBucket(def->series, true, now);
  flushBucket(def->series, false, now);

  const uint64_t windowMs = range24h ? (24ULL * 60ULL * 60ULL * 1000ULL) : (normalized == "live" ? HISTORY_LIVE_WINDOW_MS : (6ULL * 60ULL * 60ULL * 1000ULL));
  const uint64_t minTs = now > windowMs ? now - windowMs : 0;
  pruneLogsIfLowMemory(false);
  String json;
  json.reserve(96 + 64);
  json = "{\"metric\":\"" + metric + "\",\"unit\":\"" + String(def->unit) + "\",\"points\":[";
  bool first = true;
  auto appendPoint = [&](uint64_t ts, float v) {
    uint64_t mapped = ts;
    if (!first)
      json += ",";
    first = false;
    json += "[" + String((unsigned long long)mapped) + ",";
    if (isnan(v)) {
      json += "null";
    } else {
      json += String((double)v, (unsigned int)def->decimals);
    }
    json += "]";
  };

  if (range24h) {
    for (size_t i = 0; i < def->series.longCount; i++) {
      size_t idx = (def->series.longStart + i) % HISTORY_24H_CAPACITY;
      const HistoryPoint &p = def->series.longTerm[idx];
      if (p.ts >= minTs) appendPoint(p.ts, p.value);
    }
    if (def->series.bucket15mStart != 0) {
      float pending = def->series.bucket15mCount > 0 ? (def->series.bucket15mSum / def->series.bucket15mCount) : NAN;
      if (def->series.bucket15mStart >= minTs) appendPoint(def->series.bucket15mStart, pending);
    }
  } else {
    for (size_t i = 0; i < def->series.midCount; i++) {
      size_t idx = (def->series.midStart + i) % HISTORY_6H_CAPACITY;
      const HistoryPoint &p = def->series.midTerm[idx];
      if (p.ts >= minTs) appendPoint(p.ts, p.value);
    }
    if (def->series.bucket5mStart != 0) {
      float pending = def->series.bucket5mCount > 0 ? (def->series.bucket5mSum / def->series.bucket5mCount) : NAN;
      if (def->series.bucket5mStart >= minTs) appendPoint(def->series.bucket5mStart, pending);
    }
  }

  bool syncedFlag = timeSynced;
  json += "],\"time_synced\":" + String(syncedFlag ? 1 : 0) + ",\"timezone\":\"" + timezoneName + "\",\"epoch_base_ms\":" + String((unsigned long long)(syncedFlag ? currentEpochMs() : 0)) + "}";
  server.send(200, "application/json", json);
}

void handlePins() {
  if (!enforceAuth())
    return;
  if (server.method() == HTTP_GET) {
    String json = "{";
    json += "\"sda\":" + String(pinI2C_SDA) + ",";
    json += "\"scl\":" + String(pinI2C_SCL) + ",";
    json += "\"co2_rx\":" + String(pinCO2_RX) + ",";
    json += "\"co2_tx\":" + String(pinCO2_TX) + ",";
    json += "\"default_sda\":" + String(DEFAULT_I2C_SDA_PIN) + ",";
    json += "\"default_scl\":" + String(DEFAULT_I2C_SCL_PIN) + ",";
    json += "\"default_co2_rx\":" + String(DEFAULT_CO2_RX_PIN) + ",";
    json += "\"default_co2_tx\":" + String(DEFAULT_CO2_TX_PIN);
    json += "}";
    server.send(200, "application/json", json);
    return;
  }

  if (!server.hasArg("sda") || !server.hasArg("scl") || !server.hasArg("co2_rx") || !server.hasArg("co2_tx")) {
    server.send(400, "text/plain", "missing pins");
    return;
  }
  int sda = server.arg("sda").toInt();
  int scl = server.arg("scl").toInt();
  int rx = server.arg("co2_rx").toInt();
  int tx = server.arg("co2_tx").toInt();

  if (!pinsValid(sda, scl, rx, tx)) {
    server.send(400, "text/plain", "invalid pins");
    return;
  }

  prefs.begin("grow-sensor", false);
  prefs.putInt("i2c_sda", sda);
  prefs.putInt("i2c_scl", scl);
  prefs.putInt("co2_rx", rx);
  prefs.putInt("co2_tx", tx);
  prefs.end();

  pinI2C_SDA = sda;
  pinI2C_SCL = scl;
  pinCO2_RX = rx;
  pinCO2_TX = tx;

  server.send(200, "text/plain", "saved");
  logEvent("Pins saved, restarting");
  delay(200);
  ESP.restart();
}

void handleSettings() {
  if (!enforceAuth())
    return;
  if (server.method() == HTTP_GET) {
    bool wifiConnected = WiFi.status() == WL_CONNECTED;
    int rssi = wifiConnected ? WiFi.RSSI() : 0;
    String ssidActive = wifiConnected ? WiFi.SSID() : (apMode ? String(AP_SSID) : savedSsid);
    IPAddress activeIp = apMode ? WiFi.softAPIP() : WiFi.localIP();
    String json = "{";
    json += "\"channel\":\"" + lightChannelName() + "\",";
    json += "\"vpd_stage\":\"" + vpdStageId + "\",";
    json += "\"wifi\":\"" + String(apMode ? "Access Point aktiv" : "Verbunden: " + WiFi.SSID()) + "\",";
    json += "\"connected\":" + String(wifiConnected ? 1 : 0) + ",";
    json += "\"ap_mode\":" + String(apMode ? 1 : 0) + ",";
    json += "\"ssid\":\"" + savedSsid + "\",";
    json += "\"ssid_active\":\"" + ssidActive + "\",";
    json += "\"rssi\":" + String(rssi) + ",";
    json += "\"ip_active\":\"" + activeIp.toString() + "\",";
    json += "\"timezone\":\"" + timezoneName + "\",";
    json += "\"time_synced\":" + String(timeSynced ? 1 : 0) + ",";
    json += "\"epoch_ms\":" + String((unsigned long long)currentEpochMs()) + ",";
    json += "\"static\":" + String(staticIpEnabled ? 1 : 0) + ",";
    json += "\"ip\":\"" + (staticIpEnabled ? staticIp.toString() : WiFi.localIP().toString()) + "\",";
    json += "\"gw\":\"" + (staticIpEnabled ? staticGateway.toString() : WiFi.gatewayIP().toString()) + "\",";
    json += "\"sn\":\"" + (staticIpEnabled ? staticSubnet.toString() : WiFi.subnetMask().toString()) + "\"";
    json += "}";
    server.send(200, "application/json", json);
    return;
  }

  bool hasChannel = server.hasArg("channel");
  bool hasVpdStage = server.hasArg("vpd_stage");
  bool hasTimezone = server.hasArg("timezone");
  if (!hasChannel && !hasVpdStage && !hasTimezone) {
    server.send(400, "text/plain", "channel, timezone or vpd_stage missing");
    return;
  }
  if (hasChannel) {
    LightChannel next = lightChannelFromString(server.arg("channel"));
    channel = next;
  }
  if (hasVpdStage) {
    vpdStageId = server.arg("vpd_stage");
  }
  if (hasTimezone) {
    timezoneName = server.arg("timezone");
    applyTimezoneEnv();
    timeSynced = false; // force refresh
    lastTimeSyncAttempt = 0;
    startNtpSync();
  }
  prefs.begin("grow-sensor", false);
  if (hasChannel) prefs.putString("channel", lightChannelName());
  if (hasVpdStage) prefs.putString("vpd_stage", vpdStageId);
  if (hasTimezone) prefs.putString("timezone", timezoneName);
  prefs.end();
  server.send(200, "text/plain", "saved");
}

void handleWifiSave() {
  if (!enforceAuth())
    return;
  if (!server.hasArg("ssid") || !server.hasArg("pass")) {
    server.send(400, "text/plain", "missing ssid/pass");
    return;
  }
  String ssid = server.arg("ssid");
  String pass = server.arg("pass");
  bool staticFlag = server.hasArg("static") && server.arg("static") == "1";
  IPAddress ip, gw, sn;
  if (staticFlag) {
    ip.fromString(server.arg("ip"));
    gw.fromString(server.arg("gw"));
    sn.fromString(server.arg("sn"));
  }

  saveWifiCredentials(ssid, pass);
  prefs.begin("grow-sensor", false);
  prefs.putBool("static", staticFlag);
  prefs.putString("ip", ip.toString());
  prefs.putString("gw", gw.toString());
  prefs.putString("sn", sn.toString());
  prefs.end();
  staticIpEnabled = staticFlag;
  staticIp = ip;
  staticGateway = gw;
  staticSubnet = sn;

  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  if (staticIpEnabled && staticIp != IPAddress((uint32_t)0) && staticGateway != IPAddress((uint32_t)0) && staticSubnet != IPAddress((uint32_t)0)) {
    WiFi.config(staticIp, staticGateway, staticSubnet);
  }
  WiFi.begin(ssid.c_str(), pass.c_str());

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_TIMEOUT) {
    delay(200);
  }

  if (WiFi.status() == WL_CONNECTED) {
    apMode = false;
    logEvent("WiFi saved + connected: " + WiFi.localIP().toString());
    server.send(200, "text/plain", "connected");
  } else {
    logEvent("WiFi save failed (wrong password?)");
    server.send(401, "text/plain", "wrong password");
  }
}

void handleReset() {
  if (!enforceAuth())
    return;
  clearPreferences();
  server.send(200, "text/plain", "cleared");
  delay(200);
  ESP.restart();
}

void handleNetworks() {
  if (!enforceAuth())
    return;
  int n = WiFi.scanNetworks();
  String json = "{\"networks\":[";
  for (int i = 0; i < n; i++) {
    json += "{\"ssid\":\"" + WiFi.SSID(i) + "\",\"rssi\":" + String(WiFi.RSSI(i)) + "}";
    if (i < n - 1)
      json += ",";
  }
  json += "]}";
  server.send(200, "application/json", json);
}

void handleSensorsApi() {
  if (!enforceAuth())
    return;
  if (server.method() == HTTP_POST) {
    if (!server.hasArg("id")) {
      server.send(400, "text/plain", "missing id");
      return;
    }
    String id = server.arg("id");
    if (id == "replace") {
      if (!server.hasArg("from") || !server.hasArg("to") || !server.hasArg("category")) {
        server.send(400, "text/plain", "missing replace args");
        return;
      }
      SensorSlot *src = findSensor(server.arg("from"));
      if (!src) {
        server.send(404, "text/plain", "source not found");
        return;
      }
      src->enabled = false;
      if (src->enabledFlag) *(src->enabledFlag) = false;
      if (src->health) {
        src->health->healthy = false;
        src->health->enabled = false;
      }
      SensorSlot replacement;
      replacement.id = server.arg("to");
      replacement.type = server.hasArg("type") ? server.arg("type") : src->type;
      replacement.category = server.arg("category");
      replacement.enabled = true;
      replacement.present = true;
      replacement.healthy = false;
      if (replacement.category == "light") {
        replacement.enabledFlag = &enableLight;
        replacement.health = &lightHealth;
      } else if (replacement.category == "climate") {
        replacement.enabledFlag = &enableClimate;
        replacement.health = &climateHealth;
      } else if (replacement.category == "leaf") {
        replacement.enabledFlag = &enableLeaf;
        replacement.health = &leafHealth;
      } else if (replacement.category == "co2") {
        replacement.enabledFlag = &enableCo2;
        replacement.health = &co2Health;
      }
      if (replacement.enabledFlag) *(replacement.enabledFlag) = true;
      sensors.push_back(replacement);
      if (replacement.category == "climate") {
        climateType = climateFromString(replacement.type);
        reinitClimateSensor();
      } else if (replacement.category == "co2") {
        co2Type = co2FromString(replacement.type);
        reinitCo2Sensor();
      } else if (replacement.category == "light") {
        reinitLightSensor();
      } else if (replacement.category == "leaf") {
        reinitLeafSensor();
      }
      persistSensorFlags();
      rebuildSensorList();
      server.send(200, "text/plain", "replaced");
      return;
    }
    if (id == "climate_type" && server.hasArg("value")) {
      climateType = climateFromString(server.arg("value"));
      prefs.begin("grow-sensor", false);
      prefs.putString("climate_type", climateSensorName(climateType));
      prefs.end();
      reinitClimateSensor();
      rebuildSensorList();
      server.send(200, "text/plain", "saved");
      return;
    }
    if (id == "co2_type" && server.hasArg("value")) {
      co2Type = co2FromString(server.arg("value"));
      prefs.begin("grow-sensor", false);
      prefs.putString("co2_type", co2SensorName(co2Type));
      prefs.end();
      reinitCo2Sensor();
      rebuildSensorList();
      server.send(200, "text/plain", "saved");
      return;
    }
    if (!server.hasArg("enabled")) {
      server.send(400, "text/plain", "missing enabled");
      return;
    }
    bool flag = server.arg("enabled") == "1";
    SensorSlot *target = findSensor(id);
    if (!target) {
      server.send(404, "text/plain", "not found");
      return;
    }
    if (target->enabledFlag) *(target->enabledFlag) = flag;
    if (target->health) {
      target->health->healthy = false;
      target->health->enabled = flag;
      target->health->lastUpdate = 0;
    }
    if (flag) {
      if (target->category == "light") reinitLightSensor();
      else if (target->category == "climate") reinitClimateSensor();
      else if (target->category == "leaf") reinitLeafSensor();
      else if (target->category == "co2") reinitCo2Sensor();
    } else {
      if (target->category == "light") { latest.lux = NAN; latest.ppfd = NAN; }
      if (target->category == "climate") { latest.ambientTempC = NAN; latest.humidity = NAN; }
      if (target->category == "leaf") { latest.leafTempC = NAN; }
      if (target->category == "co2") { latest.co2ppm = -1; }
    }
    persistSensorFlags();
    rebuildSensorList();
    server.send(200, "text/plain", "saved");
    return;
  }

  String json = "{\"sensors\":[";
  for (size_t i = 0; i < sensors.size(); i++) {
    const auto &s = sensors[i];
    bool healthy = s.health ? s.health->healthy : s.healthy;
    json += "{\"id\":\"" + s.id + "\",\"name\":\"" + s.type + "\",\"category\":\"" + s.category + "\",\"enabled\":" + String((s.enabledFlag ? *s.enabledFlag : s.enabled) ? 1 : 0) + ",\"healthy\":" + String(healthy ? 1 : 0) + ",\"present\":" + String((s.health ? s.health->present : s.present) ? 1 : 0) + "}";
    if (i < sensors.size() - 1) json += ",";
  }
  json += "],";
  json += "\"climate_type\":\"" + climateSensorName(climateType) + "\",";
  json += "\"co2_type\":\"" + co2SensorName(co2Type) + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

void handleSensorsConfig() {
  if (!enforceAuth())
    return;
  if (server.method() == HTTP_GET) {
    size_t activeCount = sensorConfigs.size();
    size_t tplCount = SENSOR_TEMPLATE_COUNT;
    String json;
    json.reserve(64 + activeCount * 200 + tplCount * 200);
    json = "{\"active\":[";
    for (size_t i = 0; i < sensorConfigs.size(); i++) {
      const auto &cfg = sensorConfigs[i];
      if (i) json += ",";
      json += "{\"id\":\"" + cfg.id + "\",\"type\":\"" + cfg.type + "\",\"name\":\"" + cfg.name + "\",\"category\":\"" + cfg.category + "\",\"iface\":\"" + cfg.interfaceType + "\",\"enabled\":" + String(cfg.enabled ? 1 : 0) + ",\"advanced\":" + String(cfg.advancedPins ? 1 : 0) + ",";
      json += "\"sda\":" + String(cfg.sda) + ",\"scl\":" + String(cfg.scl) + ",\"rx\":" + String(cfg.rx) + ",\"tx\":" + String(cfg.tx) + "}";
    }
    json += "],\"templates\":[";
    for (size_t i = 0; i < SENSOR_TEMPLATE_COUNT; i++) {
      const auto &t = SENSOR_TEMPLATES[i];
      if (i) json += ",";
      json += "{\"type\":\"" + String(t.type) + "\",\"name\":\"" + String(t.name) + "\",\"category\":\"" + String(t.category) + "\",\"iface\":\"" + String(t.interfaceType) + "\",\"sda\":" + String(t.sda) + ",\"scl\":" + String(t.scl) + ",\"rx\":" + String(t.rx) + ",\"tx\":" + String(t.tx) + "}";
    }
    json += "]}";
    server.send(200, "application/json", json);
    return;
  }

  if (!server.hasArg("action")) {
    server.send(400, "text/plain", "missing action");
    return;
  }
  String action = server.arg("action");
  if (action == "enable" || action == "disable") {
    if (!server.hasArg("id")) { server.send(400, "text/plain", "missing id"); return; }
    String id = server.arg("id");
    bool en = action == "enable";
    for (auto &cfg : sensorConfigs) {
      if (cfg.id == id) {
        cfg.enabled = en;
        if (id == "lux") enableLight = en;
        if (id == "climate") enableClimate = en;
        if (id == "leaf") enableLeaf = en;
        if (id == "co2") enableCo2 = en;
      }
    }
    persistSensorFlags();
    saveSensorConfigs();
    rebuildSensorList();
    server.send(200, "text/plain", "ok");
    return;
  }
  if (action == "add") {
    if (!server.hasArg("sensor_type")) { server.send(400, "text/plain", "missing sensor_type"); return; }
    String sType = server.arg("sensor_type");
    const SensorTemplate *tpl = findTemplate(sType);
    SensorConfig cfg;
    if (tpl) {
      cfg.type = tpl->type;
      cfg.name = tpl->name;
      cfg.category = tpl->category;
      cfg.interfaceType = tpl->interfaceType;
      cfg.sda = tpl->sda;
      cfg.scl = tpl->scl;
      cfg.rx = tpl->rx;
      cfg.tx = tpl->tx;
    } else {
      cfg.type = sType;
      cfg.name = sType;
      cfg.category = server.arg("category");
      cfg.interfaceType = "i2c";
    }
    cfg.id = server.hasArg("id") ? server.arg("id") : cfg.type + String(sensorConfigs.size());
    cfg.enabled = true;
    sensorConfigs.push_back(cfg);
    saveSensorConfigs();
    server.send(200, "text/plain", "added");
    return;
  }
  if (action == "set_pins") {
    if (!server.hasArg("id")) { server.send(400, "text/plain", "missing id"); return; }
    String id = server.arg("id");
    int sda = server.hasArg("sda") ? server.arg("sda").toInt() : -1;
    int scl = server.hasArg("scl") ? server.arg("scl").toInt() : -1;
    int rx = server.hasArg("rx") ? server.arg("rx").toInt() : -1;
    int tx = server.hasArg("tx") ? server.arg("tx").toInt() : -1;
    for (auto &cfg : sensorConfigs) {
      if (cfg.id == id) {
        cfg.sda = sda;
        cfg.scl = scl;
        cfg.rx = rx;
        cfg.tx = tx;
        cfg.advancedPins = true;
      }
    }
    applyPinsFromConfig();
    saveSensorConfigs();
    server.send(200, "text/plain", "pins saved");
    return;
  }

  server.send(400, "text/plain", "unsupported action");
}

void handleLogs() {
  if (!enforceAuth())
    return;
  pruneLogsIfLowMemory(false);
  size_t count = logCount > LOG_JSON_LIMIT ? LOG_JSON_LIMIT : logCount;
  size_t startIdx = (logStart + (logCount > count ? (logCount - count) : 0)) % LOG_CAPACITY;
  String json;
  json.reserve(8 + count * 32);
  json = "[";
  for (size_t i = 0; i < count; i++) {
    size_t idx = (startIdx + i) % LOG_CAPACITY;
    json += "\"" + logBuffer[idx] + "\"";
    if (i < count - 1)
      json += ",";
  }
  json += "]";
  server.send(200, "application/json", json);
}

void setupServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/telemetry", HTTP_GET, handleTelemetry);
  server.on("/api/settings", HTTP_GET, handleSettings);
  server.on("/api/settings", HTTP_POST, handleSettings);
  server.on("/api/wifi", HTTP_POST, handleWifiSave);
  server.on("/api/reset", HTTP_POST, handleReset);
  server.on("/api/networks", HTTP_GET, handleNetworks);
  server.on("/api/sensors", HTTP_GET, handleSensorsApi);
  server.on("/api/sensors", HTTP_POST, handleSensorsApi);
  server.on("/api/sensors/config", HTTP_GET, handleSensorsConfig);
  server.on("/api/sensors/config", HTTP_POST, handleSensorsConfig);
  server.on("/api/pins", HTTP_GET, handlePins);
  server.on("/api/pins", HTTP_POST, handlePins);
  server.on("/api/logs", HTTP_GET, handleLogs);
  server.on("/api/history", HTTP_GET, handleHistory);
   server.on("/api/partners", HTTP_GET, handlePartners);
   server.on("/api/partners", HTTP_POST, handlePartners);
  server.onNotFound(handleNotFound);
  server.begin();
}

// ----------------------------
// Arduino entry points
// ----------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nGrowSensor booting...");

  loadPinsFromPrefs();
  loadSensorConfigs();
  initSensors();
  connectToWiFi();
  setupServer();
}

void loop() {
  if (apMode) {
    dnsServer.processNextRequest();
  }
  pruneLogsIfLowMemory(false);
  server.handleClient();
  maintainTimeSync();

  if (millis() - lastSensorMillis >= SENSOR_INTERVAL) {
    lastSensorMillis = millis();
    readSensors();
  }
}
