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
#include <HTTPClient.h>
#include <time.h>
#include <sys/time.h>
#include <vector>
#include <algorithm>
#include <esp_system.h>

#ifndef CLOUD_DIAG
#define CLOUD_DIAG 0
#endif

#ifndef ENABLE_MDNS
#define ENABLE_MDNS 1
#endif

#if ENABLE_MDNS
#include <ESPmDNS.h>
#endif

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
static constexpr unsigned long SENSOR_MIN_INTERVAL = 30000;    // 2 samples/min max
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
static constexpr time_t TIME_VALID_AFTER = 1700000000;          // ~2023-11-14 UTC safeguard
static constexpr unsigned long CLOUD_WORKER_INTERVAL_MS = 1500;
static constexpr unsigned long CLOUD_RECORDING_INTERVAL_MS = 60000;
static constexpr unsigned long CLOUD_BACKOFF_SHORT_MS = 5000;
static constexpr unsigned long CLOUD_BACKOFF_MED_MS = 15000;
static constexpr unsigned long CLOUD_BACKOFF_LONG_MS = 60000;
static constexpr unsigned long CLOUD_TEST_TIMEOUT_MS = 8000;
static constexpr unsigned long CLOUD_HEALTH_WINDOW_MS = 60000;
static constexpr unsigned long CLOUD_PING_INTERVAL_MS = 30000;
static constexpr unsigned long DAILY_CHECKPOINT_MS = 15UL * 60UL * 1000UL;
static constexpr unsigned long CLOUD_LOG_FLUSH_INTERVAL_MS = 30000;
static constexpr size_t CLOUD_LOG_CAPACITY = 120;
static constexpr size_t CLOUD_LOG_FLUSH_LINES = 24;
static const char *FIRMWARE_VERSION = "v0.3.3";
static const char *CLOUD_ROOT_FOLDER = "GrowSensor";

static const char *NTP_SERVER_1 = "pool.ntp.org";
static const char *NTP_SERVER_2 = "time.nist.gov";
static const char *NTP_SERVER_3 = "time.google.com";

String timezoneName = "Europe/Berlin";
bool timeSynced = false;
uint64_t bootEpochMs = 0;      // epoch_ms - millis() baseline after sync
uint32_t bootMillisAtSync = 0; // millis() captured when epoch baseline set
unsigned long lastTimeSyncAttempt = 0;
unsigned long lastTimeSyncOk = 0;
bool isTimeSynced();
void onTimeSynchronized(uint64_t epochNowMs, uint32_t monotonicNowMs);
uint64_t mapTimestampToEpochMs(uint32_t monotonicTs);
uint64_t currentEpochMs();
String cloudRootPath();
String normalizeCloudBaseUrl(String url);
String buildWebDavBaseUrl();
enum class CloudProtocol : uint8_t { HTTP = 0, HTTPS = 1 };
CloudProtocol cloudProtocol();
const char *cloudProtocolLabel();
struct CloudRequestClients;
bool beginCloudRequest(HTTPClient &http, const String &fullUrl, CloudRequestClients &clients);
String ensureTrailingSlash(String s);
String joinUrl(const String &base, const String &rel);
float safeFloat(float v, float fallback = 0.0f);
int safeInt(int v, int fallback = 0);
void enqueueCloudJob(const String &path, const String &payload, const String &dayKey,
                     const String &kind = "generic", const String &contentType = "application/json",
                     bool replaceByPath = true);
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

enum class StorageMode {
  LOCAL_ONLY,
  CLOUD_PRIMARY,
  LOCAL_FALLBACK
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

struct MetricSampleInfo {
  bool everHadData = false;
  uint64_t lastEpochMs = 0;
};

struct DailyAggregate {
  String dayKey;
  uint32_t count = 0;
  float sum = 0.0f;
  float min = NAN;
  float max = NAN;
  float last = NAN;
};

struct DailyPoint {
  String dayKey;
  float avg = NAN;
  float min = NAN;
  float max = NAN;
  float last = NAN;
  uint32_t count = 0;
};

struct HourlyAggregate {
  uint8_t hour = 0;
  uint32_t count = 0;
  float sum = 0.0f;
  float min = NAN;
  float max = NAN;
  bool active = false;
};

struct CloudJob {
  String dayKey;
  String path;
  String payload;
  String contentType = "application/json";
  String kind;
  int attempts = 0;
  unsigned long nextAttemptAt = 0;
};

struct CloudConfig {
  String baseUrl;
  String username;
  String password;
  bool enabled = false;
  bool recording = false;
  bool persistCredentials = true;
  uint8_t retentionMonths = 1;
  uint8_t protocol = 0;
};

struct CloudStatus {
  bool connected = false;
  bool enabled = false;           // persisted flag
  bool runtimeEnabled = false;    // worker flag
  bool recording = false;
  bool foldersReady = true;
  uint64_t lastUploadMs = 0;
  uint64_t lastFailureMs = 0;
  uint64_t lastPingMs = 0;
  uint64_t lastTestMs = 0;
  uint64_t lastConfigSaveMs = 0;
  uint64_t lastStateChangeMs = 0;
  uint32_t failureCount = 0;
  int lastHttpCode = 0;
  String lastError;
  String lastErrorSuffix;
  String lastUrl;
  String lastUploadedPath;
  String lastStateReason;
  String deviceFolder;
  size_t queueSize = 0;
};

// Globals
BH1750 lightMeter;
Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
HardwareSerial co2Serial(2);
MHZ19 co2Sensor;
Preferences prefsWifi;
Preferences prefsSensors;
Preferences prefsUi;
Preferences prefsSystem;
DNSServer dnsServer;
WebServer server(80);

// Wi-Fi config
String savedSsid;
String savedWifiPass;
bool apMode = false;
bool wifiConnectInProgress = false;
unsigned long wifiConnectStarted = 0;
unsigned long wifiFallbackAt = 0;
String deviceHostname = "growsensor";
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
String cloudLogBuffer[CLOUD_LOG_CAPACITY];
size_t cloudLogStart = 0;
size_t cloudLogCount = 0;
unsigned long lastCloudLogFlushMs = 0;
uint32_t cloudLogSeq = 0;

// Forward declarations
void pruneLogsIfLowMemory(bool allowNotice = true);
void appendLogLine(const String &line);
void logEvent(const String &msg, const String &level = "info", const String &source = "system");
String deviceId();
String buildWebDavUrl(const String &pathRelative);
bool ensureCollection(const String &path, const char *label);
bool webdavRequestFollowRedirects(const String &method, const String &url, const String &payload, const char *contentType, int &outCode, String &outLocationShort, String &outBodyShort, String *chainOut = nullptr, bool *redirectedToHttps = nullptr, unsigned long timeoutMs = CLOUD_TEST_TIMEOUT_MS);

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

unsigned long lastLightSampleMs = 0;
unsigned long lastClimateSampleMs = 0;
unsigned long lastLeafSampleMs = 0;
unsigned long lastCo2SampleMs = 0;
String activeDayKey;

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
MetricSampleInfo SAMPLE_INFO[HISTORY_METRIC_COUNT];
DailyAggregate DAILY_AGG[HISTORY_METRIC_COUNT];
std::vector<DailyPoint> DAILY_HISTORY[HISTORY_METRIC_COUNT];
HourlyAggregate HOURLY_AGG[HISTORY_METRIC_COUNT][24];
String hourlyDayKey;

CloudConfig cloudConfig;
CloudStatus cloudStatus;
std::vector<CloudJob> cloudQueue;
Preferences prefsCloud;
StorageMode storageMode = StorageMode::LOCAL_ONLY;
unsigned long lastCloudPingMs = 0;
unsigned long lastCloudOkMs = 0;
uint64_t lastCloudOkEpochMs = 0;
unsigned long lastDailyCheckpointMs = 0;
unsigned long lastCloudWorkerTickMs = 0;
unsigned long nextCloudAttemptAfterMs = 0;
unsigned long lastRecordingEnqueueMs = 0;
String lastRecordingMinuteKey;

// ----------------------------
// Helpers
// ----------------------------
struct CloudRequestClients {
  WiFiClient http;
  WiFiClientSecure https;
};

CloudProtocol cloudProtocol() {
  return cloudConfig.protocol == 1 ? CloudProtocol::HTTPS : CloudProtocol::HTTP;
}

const char *cloudProtocolLabel() {
  return cloudProtocol() == CloudProtocol::HTTPS ? "https" : "http";
}

bool beginCloudRequest(HTTPClient &http, const String &fullUrl, CloudRequestClients &clients) {
  if (cloudProtocol() == CloudProtocol::HTTPS) {
    clients.https.setInsecure();
    return http.begin(clients.https, fullUrl);
  }
  return http.begin(clients.http, fullUrl);
}
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

MetricSampleInfo *sampleInfoById(const String &id) {
  for (size_t i = 0; i < HISTORY_METRIC_COUNT; i++) {
    if (id == HISTORY_METRICS[i].id) return &SAMPLE_INFO[i];
  }
  return nullptr;
}

int metricIndex(const String &id) {
  for (size_t i = 0; i < HISTORY_METRIC_COUNT; i++) {
    if (id == HISTORY_METRICS[i].id) return (int)i;
  }
  return -1;
}

String dayKeyForEpoch(uint64_t epochMs) {
  if (epochMs == 0) return "";
  struct tm t;
  time_t secs = epochMs / 1000ULL;
  if (localtime_r(&secs, &t) == nullptr) return "";
  char buf[16];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d", t.tm_year + 1900, t.tm_mon + 1, t.tm_mday);
  return String(buf);
}

String currentDayKey() {
  if (!isTimeSynced()) return "unsynced";
  return dayKeyForEpoch(currentEpochMs());
}

const char *storageModeName(StorageMode mode) {
  switch (mode) {
  case StorageMode::CLOUD_PRIMARY: return "cloud_primary";
  case StorageMode::LOCAL_FALLBACK: return "local_fallback";
  default: return "local_only";
  }
}

bool cloudHealthOk() {
  if (!(cloudConfig.enabled && cloudConfig.baseUrl.length() > 0) || lastCloudOkMs == 0) return false;
  if (WiFi.status() != WL_CONNECTED) return false;
  return millis() - lastCloudOkMs <= CLOUD_HEALTH_WINDOW_MS;
}

bool cloudStorageActive() {
  return cloudConfig.enabled && cloudConfig.recording && cloudHealthOk();
}

bool cloudLoggingActive() {
  return cloudConfig.enabled && cloudHealthOk();
}

void refreshStorageMode(const String &reason = "") {
  bool runtimeActive = cloudConfig.enabled && cloudConfig.baseUrl.length() > 0;
  StorageMode next = StorageMode::LOCAL_ONLY;
  if (runtimeActive) {
    next = cloudHealthOk() ? StorageMode::CLOUD_PRIMARY : StorageMode::LOCAL_FALLBACK;
  }
  if (storageMode != next && reason.length() > 0) {
    cloudStatus.lastStateReason = reason;
    cloudStatus.lastStateChangeMs = currentEpochMs();
  }
  storageMode = next;
  cloudStatus.enabled = cloudConfig.enabled;
  cloudStatus.runtimeEnabled = runtimeActive;
  cloudStatus.connected = storageMode == StorageMode::CLOUD_PRIMARY;
  cloudStatus.recording = cloudConfig.recording;
  cloudStatus.deviceFolder = cloudRootPath();
}

uint16_t retentionDays() {
  uint8_t months = cloudConfig.retentionMonths;
  if (months < 1) months = 1;
  if (months > 4) months = 4;
  return (uint16_t)(months * 30);
}

void markCloudSuccess(const String &reason = "cloud ok") {
  lastCloudOkMs = millis();
  lastCloudOkEpochMs = currentEpochMs();
  cloudStatus.lastPingMs = lastCloudOkEpochMs;
  cloudStatus.lastError = "";
  cloudStatus.lastErrorSuffix = "";
  cloudStatus.lastStateReason = reason;
  cloudStatus.lastStateChangeMs = currentEpochMs();
  refreshStorageMode(reason);
}

void markCloudFailure(const String &msg) {
  lastCloudOkMs = 0;
  cloudStatus.lastError = msg;
  cloudStatus.lastErrorSuffix = "";
  cloudStatus.failureCount++;
  cloudStatus.lastFailureMs = currentEpochMs();
  cloudStatus.connected = false;
  cloudStatus.lastStateReason = msg;
  cloudStatus.lastStateChangeMs = currentEpochMs();
  logEvent("Cloud error: " + msg, "error", "cloud");
  refreshStorageMode(msg);
}

void saveCloudConfig(const String &reason = "config save") {
  cloudConfig.baseUrl = normalizeCloudBaseUrl(cloudConfig.baseUrl);
  prefsCloud.begin("cloud", false);
  prefsCloud.putBool("persist", cloudConfig.persistCredentials);
  if (cloudConfig.persistCredentials) {
    prefsCloud.putString("base", cloudConfig.baseUrl);
    prefsCloud.putString("user", cloudConfig.username);
    prefsCloud.putString("pass", cloudConfig.password);
  } else {
    prefsCloud.remove("base");
    prefsCloud.remove("user");
    prefsCloud.remove("pass");
  }
  prefsCloud.putBool("enabled", cloudConfig.enabled);
  prefsCloud.putBool("recording", cloudConfig.recording);
  prefsCloud.putUChar("retention", cloudConfig.retentionMonths);
  prefsCloud.putUChar("protocol", cloudConfig.protocol);
  prefsCloud.end();
  cloudStatus.enabled = cloudConfig.enabled;
  cloudStatus.runtimeEnabled = cloudConfig.enabled && cloudConfig.baseUrl.length() > 0;
  cloudStatus.recording = cloudConfig.recording;
  cloudStatus.lastConfigSaveMs = currentEpochMs();
  cloudStatus.lastStateReason = reason;
  cloudStatus.lastStateChangeMs = cloudStatus.lastConfigSaveMs;
  refreshStorageMode(reason);
}

void loadCloudConfig() {
  prefsCloud.begin("cloud", true);
  bool persist = prefsCloud.getBool("persist", true);
  cloudConfig.persistCredentials = persist;
  if (cloudConfig.persistCredentials) {
    cloudConfig.baseUrl = prefsCloud.getString("base", "");
    cloudConfig.username = prefsCloud.getString("user", "");
    cloudConfig.password = prefsCloud.getString("pass", "");
  } else {
    cloudConfig.baseUrl = "";
    cloudConfig.username = "";
    cloudConfig.password = "";
  }
  cloudConfig.enabled = prefsCloud.getBool("enabled", false);
  cloudConfig.recording = prefsCloud.getBool("recording", false);
  cloudConfig.retentionMonths = prefsCloud.getUChar("retention", 1);
  cloudConfig.protocol = prefsCloud.getUChar("protocol", 0);
  if (cloudConfig.retentionMonths < 1) cloudConfig.retentionMonths = 1;
  if (cloudConfig.retentionMonths > 4) cloudConfig.retentionMonths = 4;
  if (cloudConfig.protocol > 1) cloudConfig.protocol = 0;
  prefsCloud.end();
  cloudConfig.baseUrl = normalizeCloudBaseUrl(cloudConfig.baseUrl);
  cloudStatus.enabled = cloudConfig.enabled;
  cloudStatus.recording = cloudConfig.recording;
  cloudStatus.lastStateReason = "boot load";
  cloudStatus.lastStateChangeMs = currentEpochMs();
  refreshStorageMode("boot load");
}

void setCloudEnabled(bool enabled, const String &reason) {
  cloudConfig.enabled = enabled;
  cloudStatus.enabled = enabled;
  cloudStatus.runtimeEnabled = enabled && cloudConfig.baseUrl.length() > 0;
  if (!enabled) {
    cloudConfig.recording = false;
    cloudStatus.recording = false;
  }
  cloudStatus.lastStateReason = reason;
  cloudStatus.lastStateChangeMs = currentEpochMs();
  refreshStorageMode(reason);
}

void setRecordingActive(bool active, const String &reason) {
  cloudConfig.recording = active;
  cloudStatus.recording = active;
  cloudStatus.lastStateReason = reason;
  cloudStatus.lastStateChangeMs = currentEpochMs();
}

void persistDailyHistory() {
  if (storageMode != StorageMode::LOCAL_ONLY) return;
  prefsCloud.begin("cloud", false);
  DynamicJsonDocument doc(16384);
  JsonArray arr = doc.to<JsonArray>();
  for (size_t i = 0; i < HISTORY_METRIC_COUNT; i++) {
    JsonObject obj = arr.createNestedObject();
    obj["id"] = HISTORY_METRICS[i].id;
    JsonArray series = obj.createNestedArray("days");
    for (const auto &p : DAILY_HISTORY[i]) {
      JsonObject d = series.createNestedObject();
      d["day"] = p.dayKey;
      d["avg"] = p.avg;
      d["min"] = p.min;
      d["max"] = p.max;
      d["last"] = p.last;
      d["count"] = p.count;
    }
  }
  String out;
  serializeJson(doc, out);
  prefsCloud.putString("daily", out);
  prefsCloud.end();
}

void loadDailyHistory() {
  if (storageMode != StorageMode::LOCAL_ONLY) return;
  for (size_t i = 0; i < HISTORY_METRIC_COUNT; i++) {
    DAILY_HISTORY[i].clear();
    DAILY_AGG[i] = DailyAggregate();
  }
  prefsCloud.begin("cloud", true);
  String raw = prefsCloud.getString("daily", "");
  prefsCloud.end();
  if (raw.length() == 0) return;
  DynamicJsonDocument doc(16384);
  if (deserializeJson(doc, raw) != DeserializationError::Ok) return;
  for (JsonObject obj : doc.as<JsonArray>()) {
    String id = obj["id"] | "";
    int idx = metricIndex(id);
    if (idx < 0) continue;
    JsonArray series = obj["days"].as<JsonArray>();
    for (JsonObject d : series) {
      DailyPoint p;
      p.dayKey = d["day"] | "";
      p.avg = d["avg"] | NAN;
      p.min = d["min"] | NAN;
      p.max = d["max"] | NAN;
      p.last = d["last"] | NAN;
      p.count = d["count"] | 0;
      if (p.dayKey.length() > 0) DAILY_HISTORY[idx].push_back(p);
    }
  }
}

void trimDailyHistory(size_t idx) {
  if (idx >= HISTORY_METRIC_COUNT) return;
  uint16_t days = retentionDays();
  if (DAILY_HISTORY[idx].size() > days) {
    DAILY_HISTORY[idx].erase(DAILY_HISTORY[idx].begin(), DAILY_HISTORY[idx].begin() + (DAILY_HISTORY[idx].size() - days));
  }
}

void finalizeDaily(const String &dayKey) {
  if (dayKey.length() == 0) return;
  if (storageMode != StorageMode::LOCAL_ONLY) return;
  for (size_t i = 0; i < HISTORY_METRIC_COUNT; i++) {
    if (DAILY_AGG[i].dayKey != dayKey || DAILY_AGG[i].count == 0) continue;
    DailyPoint p;
    p.dayKey = dayKey;
    p.count = DAILY_AGG[i].count;
    p.min = DAILY_AGG[i].min;
    p.max = DAILY_AGG[i].max;
    p.last = DAILY_AGG[i].last;
    p.avg = DAILY_AGG[i].count > 0 ? (DAILY_AGG[i].sum / DAILY_AGG[i].count) : NAN;
    DAILY_HISTORY[i].push_back(p);
    trimDailyHistory(i);
  }
}

void resetDailyAgg(int idx, const String &dayKey) {
  if (idx < 0 || idx >= (int)HISTORY_METRIC_COUNT) return;
  DAILY_AGG[idx].dayKey = dayKey;
  DAILY_AGG[idx].count = 0;
  DAILY_AGG[idx].sum = 0;
  DAILY_AGG[idx].min = NAN;
  DAILY_AGG[idx].max = NAN;
  DAILY_AGG[idx].last = NAN;
}

void resetHourlyAgg(const String &dayKey) {
  hourlyDayKey = dayKey;
  for (size_t i = 0; i < HISTORY_METRIC_COUNT; i++) {
    for (size_t h = 0; h < 24; h++) {
      HOURLY_AGG[i][h].hour = (uint8_t)h;
      HOURLY_AGG[i][h].count = 0;
      HOURLY_AGG[i][h].sum = 0;
      HOURLY_AGG[i][h].min = NAN;
      HOURLY_AGG[i][h].max = NAN;
      HOURLY_AGG[i][h].active = false;
    }
  }
}

void resetAllDaily(const String &dayKey) {
  for (size_t i = 0; i < HISTORY_METRIC_COUNT; i++) {
    resetDailyAgg(i, dayKey);
  }
  resetHourlyAgg(dayKey);
}

void accumulateDaily(int idx, float value, const String &dayKey, int hour) {
  if (idx < 0 || idx >= (int)HISTORY_METRIC_COUNT || dayKey.length() == 0 || isnan(value)) return;
  if (DAILY_AGG[idx].dayKey != dayKey) {
    if (DAILY_AGG[idx].dayKey.length() > 0) {
      finalizeDaily(DAILY_AGG[idx].dayKey);
    }
    resetDailyAgg(idx, dayKey);
  }
  auto &agg = DAILY_AGG[idx];
  agg.dayKey = dayKey;
  agg.count++;
  agg.sum += value;
  if (isnan(agg.min) || value < agg.min) agg.min = value;
  if (isnan(agg.max) || value > agg.max) agg.max = value;
  agg.last = value;
  if (hour >= 0 && hour < 24) {
    if (hourlyDayKey != dayKey) {
      resetHourlyAgg(dayKey);
    }
    HourlyAggregate &bin = HOURLY_AGG[idx][hour];
    bin.hour = (uint8_t)hour;
    bin.count++;
    bin.sum += value;
    if (!bin.active || isnan(bin.min) || value < bin.min) bin.min = value;
    if (!bin.active || isnan(bin.max) || value > bin.max) bin.max = value;
    bin.active = true;
  }
}

bool dailyPointFor(int idx, const String &dayKey, DailyPoint &out) {
  if (idx < 0 || idx >= (int)HISTORY_METRIC_COUNT) return false;
  for (auto it = DAILY_HISTORY[idx].rbegin(); it != DAILY_HISTORY[idx].rend(); ++it) {
    if (it->dayKey == dayKey) { out = *it; return true; }
  }
  return false;
}

int hourFromEpoch(uint64_t epochMs) {
  if (!isTimeSynced()) return -1;
  time_t seconds = (time_t)(epochMs / 1000ULL);
  struct tm local;
  if (localtime_r(&seconds, &local) == nullptr) return -1;
  return local.tm_hour;
}

String safeBaseUrl() {
  return buildWebDavBaseUrl();
}

String normalizeCloudBaseUrl(String url) {
  url.trim();
  if (url.length() == 0) return "";
  if (url.startsWith("http://")) {
    url = url.substring(strlen("http://"));
  } else if (url.startsWith("https://")) {
    url = url.substring(strlen("https://"));
  }
  String rootTag = String("/") + CLOUD_ROOT_FOLDER;
  int idx = url.lastIndexOf(rootTag);
  if (idx > 0) {
    int end = idx + rootTag.length();
    if (end == (int)url.length() || url.charAt(end) == '/') {
      url = url.substring(0, idx);
    }
  }
  while (url.endsWith("/")) url.remove(url.length() - 1);
  return url;
}

String buildWebDavBaseUrl() {
  String base = normalizeCloudBaseUrl(cloudConfig.baseUrl);
  if (base.length() == 0) return "";
  String full = String(cloudProtocolLabel()) + "://" + base;
  return ensureTrailingSlash(full);
}

String buildWebDavUrl(const String &pathRelative) {
  String base = buildWebDavBaseUrl();
  if (base.length() == 0) return "";
  String cleanedPath = pathRelative;
  while (cleanedPath.startsWith("//")) cleanedPath.remove(0, 1);
  return joinUrl(base, cleanedPath);
}

String ensureTrailingSlash(String s) {
  while (s.endsWith("/")) s.remove(s.length() - 1);
  if (s.length() == 0) return s;
  s += "/";
  return s;
}

String joinUrl(const String &base, const String &rel) {
  String cleanBase = ensureTrailingSlash(base);
  String cleanRel = rel;
  while (cleanRel.startsWith("/")) cleanRel.remove(0, 1);
  return cleanBase + cleanRel;
}

String cloudDeviceRoot() {
  return String("/") + CLOUD_ROOT_FOLDER + "/" + deviceId();
}

String cloudRootPath() {
  return cloudDeviceRoot();
}

String cloudDailyMonthPath(const String &dayKey) {
  if (dayKey.length() < 7) return "";
  return cloudRootPath() + "/daily/" + dayKey.substring(0, 7);
}

String cloudDailyPath(const String &dayKey) {
  if (dayKey.length() < 8) return "";
  return cloudDailyMonthPath(dayKey) + "/" + dayKey + ".json";
}

String cloudSamplesFolder(const String &dayKey) {
  if (dayKey.length() == 0) return "";
  return cloudDeviceRoot() + "/samples/" + dayKey;
}

String cloudMetaFolder() {
  return cloudDeviceRoot() + "/meta";
}

String fileSafeTimestamp(uint64_t epochMs) {
  time_t secs = epochMs / 1000ULL;
  struct tm t;
  if (localtime_r(&secs, &t) == nullptr) return "";
  char buf[24];
  snprintf(buf, sizeof(buf), "%04d%02d%02d_%02d%02d%02d", t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);
  return String(buf);
}

String isoTimestamp(uint64_t epochMs) {
  time_t secs = epochMs / 1000ULL;
  struct tm t;
  if (localtime_r(&secs, &t) == nullptr) return "";
  char buf[32];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d", t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);
  return String(buf);
}

String minuteKeyFromEpoch(uint64_t epochMs) {
  time_t secs = epochMs / 1000ULL;
  struct tm t;
  if (localtime_r(&secs, &t) == nullptr) return "";
  char buf[32];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d_%02d-%02d", t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min);
  return String(buf);
}

bool cloudEnsureFolders(const String &dayKey) {
  if (cloudConfig.baseUrl.length() == 0) return false;
  String monthPath = dayKey.length() >= 7 ? cloudDailyMonthPath(dayKey) : "";
  String deviceRoot = cloudRootPath();
  String baseRoot = String("/") + CLOUD_ROOT_FOLDER;
  String metaPath = deviceRoot + "/meta";
  String dailyPath = deviceRoot + "/daily";
  String samplesPath = deviceRoot + "/samples";
  String samplesDayPath = dayKey.length() > 0 ? samplesPath + "/" + dayKey : "";
  String logsPath = deviceRoot + "/logs";
  String logsChunksPath = logsPath + "/chunks";
  bool ok = true;
  if (!ensureCollection(baseRoot, "root")) ok = false;
  if (!ensureCollection(deviceRoot, "device")) ok = false;
  if (!ensureCollection(samplesPath, "samples")) ok = false;
  if (!ensureCollection(dailyPath, "daily")) ok = false;
  if (!ensureCollection(metaPath, "meta")) ok = false;
  if (!ensureCollection(logsPath, "logs")) ok = false;
  if (!ensureCollection(logsChunksPath, "logc")) ok = false;
  if (samplesDayPath.length() > 0 && !ensureCollection(samplesDayPath, "sday")) ok = false;
  if (monthPath.length() > 0 && !ensureCollection(monthPath, "month")) ok = false;
  cloudStatus.foldersReady = ok;
  return ok;
}

String serializeDailyPayload(const String &dayKey) {
  if (dayKey.length() == 0 || dayKey == "unsynced") return "";
  DynamicJsonDocument doc(8192);
  doc["date"] = dayKey;
  doc["tz"] = timezoneName;
  doc["deviceId"] = deviceId();
  doc["firmware"] = FIRMWARE_VERSION;
  JsonObject sensors = doc.createNestedObject("sensors");
  bool hasMetric = false;
  for (size_t i = 0; i < HISTORY_METRIC_COUNT; i++) {
    if (DAILY_AGG[i].dayKey != dayKey || DAILY_AGG[i].count == 0) continue;
    JsonObject m = sensors.createNestedObject(HISTORY_METRICS[i].id);
    m["unit"] = HISTORY_METRICS[i].unit;
    m["avg"] = DAILY_AGG[i].sum / DAILY_AGG[i].count;
    m["min"] = DAILY_AGG[i].min;
    m["max"] = DAILY_AGG[i].max;
    m["samples"] = DAILY_AGG[i].count;
    m["last"] = DAILY_AGG[i].last;
    hasMetric = true;
  }
  if (hourlyDayKey == dayKey) {
    JsonArray hourly = doc.createNestedArray("hourly");
    for (size_t h = 0; h < 24; h++) {
      bool any = false;
      JsonObject hourObj;
      JsonObject hourSensors;
      for (size_t i = 0; i < HISTORY_METRIC_COUNT; i++) {
        HourlyAggregate &bin = HOURLY_AGG[i][h];
        if (!bin.active || bin.count == 0) continue;
        if (!any) {
          hourObj = hourly.createNestedObject();
          hourObj["h"] = (int)h;
          hourSensors = hourObj.createNestedObject("sensors");
          any = true;
        }
        JsonObject m = hourSensors.createNestedObject(HISTORY_METRICS[i].id);
        m["avg"] = bin.sum / bin.count;
        m["min"] = bin.min;
        m["max"] = bin.max;
        m["samples"] = bin.count;
      }
    }
  }
  if (!hasMetric) return "";
  String payload;
  serializeJson(doc, payload);
  return payload;
}

String buildRecordingPayload(uint64_t epochMs) {
  DynamicJsonDocument doc(1024);
  doc["ts"] = epochMs;
  doc["iso"] = isoTimestamp(epochMs);
  doc["deviceId"] = deviceId();
  doc["firmware"] = FIRMWARE_VERSION;
  doc["mode"] = "http";
  JsonObject net = doc.createNestedObject("net");
  bool wifiConnected = WiFi.status() == WL_CONNECTED;
  net["ssid"] = wifiConnected ? WiFi.SSID() : savedSsid;
  net["ip"] = (wifiConnected ? WiFi.localIP() : WiFi.softAPIP()).toString();
  net["ap_mode"] = apMode;
  JsonObject metrics = doc.createNestedObject("metrics");
  if (isfinite(latest.lux)) {
    metrics["lux"] = latest.lux;
  } else {
    metrics["lux"] = nullptr;
  }
  if (isfinite(latest.ppfd)) {
    metrics["ppfd"] = latest.ppfd;
  } else {
    metrics["ppfd"] = nullptr;
  }
  if (isfinite(latest.ambientTempC)) {
    metrics["temp"] = latest.ambientTempC;
  } else {
    metrics["temp"] = nullptr;
  }
  if (isfinite(latest.humidity)) {
    metrics["humidity"] = latest.humidity;
  } else {
    metrics["humidity"] = nullptr;
  }
  if (isfinite(latest.leafTempC)) {
    metrics["leaf"] = latest.leafTempC;
  } else {
    metrics["leaf"] = nullptr;
  }
  if (latest.co2ppm >= 0) {
    metrics["co2"] = latest.co2ppm;
  } else {
    metrics["co2"] = nullptr;
  }
  if (isfinite(latest.vpd)) {
    metrics["vpd"] = latest.vpd;
  } else {
    metrics["vpd"] = nullptr;
  }
  String payload;
  serializeJson(doc, payload);
  return payload;
}

void enqueueRecordingSample(uint64_t epochMs) {
  if (!cloudStatus.runtimeEnabled || !cloudConfig.recording) return;
  String minuteKey = minuteKeyFromEpoch(epochMs);
  if (minuteKey.length() == 0) {
    minuteKey = String("unsynced_") + String((unsigned long)(millis() / 60000UL));
  }
  String dayKey = dayKeyForEpoch(epochMs);
  if (dayKey.length() == 0) dayKey = currentDayKey();
  if (dayKey.length() == 0) dayKey = "unsynced";
  String folder = cloudSamplesFolder(dayKey);
  if (folder.length() == 0) return;
  String path = folder + "/samples_" + minuteKey + ".json";
  enqueueCloudJob(path, buildRecordingPayload(epochMs), dayKey, "recording_sample", "application/json", true);
}

void enqueueRecordingEvent(const String &event, const String &reason) {
  if (!cloudStatus.runtimeEnabled) return;
  uint64_t nowEpoch = currentEpochMs();
  String dayKey = currentDayKey();
  String stamp = fileSafeTimestamp(nowEpoch);
  String path = cloudMetaFolder() + "/recording_" + event + "_" + stamp + ".json";
  DynamicJsonDocument doc(512);
  doc["event"] = event;
  doc["ts"] = nowEpoch;
  doc["iso"] = isoTimestamp(nowEpoch);
  doc["reason"] = reason;
  doc["deviceId"] = deviceId();
  doc["mode"] = "http";
  String payload;
  serializeJson(doc, payload);
  enqueueCloudJob(path, payload, dayKey, "recording_event", "application/json", false);
}

void enqueueCloudJob(const String &path, const String &payload, const String &dayKey, const String &kind, const String &contentType, bool replaceByPath) {
  if (!cloudConfig.enabled || cloudConfig.baseUrl.length() == 0 || payload.length() == 0) return;
  if (!cloudStatus.foldersReady) return;
  String finalPath = path.startsWith("/") ? path : (String("/") + path);
  for (auto &job : cloudQueue) {
    if (replaceByPath && job.path == finalPath) {
      job.payload = payload;
      job.dayKey = dayKey;
      job.kind = kind;
      job.contentType = contentType;
      job.attempts = 0;
      job.nextAttemptAt = 0;
      cloudStatus.queueSize = cloudQueue.size();
      return;
    }
  }
  CloudJob job;
  job.dayKey = dayKey;
  job.path = finalPath;
  job.payload = payload;
  job.kind = kind;
  job.contentType = contentType;
  if (cloudQueue.size() >= 12) {
    cloudQueue.erase(cloudQueue.begin());
  }
  cloudQueue.push_back(job);
  cloudStatus.queueSize = cloudQueue.size();
}

void enqueueDailyJob(const String &dayKey, const String &payload) {
  String path = cloudDailyPath(dayKey);
  if (path.length() == 0) return;
  enqueueCloudJob(path, payload, dayKey, "daily", "application/json", true);
}

void finalizeDayAndQueue(const String &dayKey) {
  if (dayKey.length() == 0) return;
  finalizeDaily(dayKey);
  if (storageMode == StorageMode::LOCAL_ONLY) {
    persistDailyHistory();
  }
  if (cloudLoggingActive()) {
    String payload = serializeDailyPayload(dayKey);
    enqueueDailyJob(dayKey, payload);
  }
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
    MetricSampleInfo *info = sampleInfoById(metric);
    if (info) {
      info->everHadData = true;
      info->lastEpochMs = ts;
    }
  }
}

float safeFloat(float v, float fallback) { return isfinite(v) ? v : fallback; }
int safeInt(int v, int fallback) { return v < 0 ? fallback : v; }

float currentPpfdFactor() {
  return luxToPPFD(1.0f, channel);
}

String deviceId() {
  if (deviceHostname.length() > 0) return deviceHostname;
  char buf[16];
  uint64_t chipid = ESP.getEfuseMac();
  snprintf(buf, sizeof(buf), "%04X%08X", (uint16_t)(chipid >> 32), (uint32_t)chipid);
  return String(buf);
}

bool enforceAuth() {
  return true;
}

void persistSensorFlags() {
  prefsSensors.begin("sensors", false);
  prefsSensors.putBool("en_light", enableLight);
  prefsSensors.putBool("en_climate", enableClimate);
  prefsSensors.putBool("en_leaf", enableLeaf);
  prefsSensors.putBool("en_co2", enableCo2);
  prefsSensors.end();
}

bool validPin(int pin) { return pin >= PIN_MIN && pin <= PIN_MAX; }

bool pinsValid(int sda, int scl, int co2Rx, int co2Tx) {
  return validPin(sda) && validPin(scl) && validPin(co2Rx) && validPin(co2Tx) && sda != scl && co2Rx != co2Tx;
}

bool allowSensorSample(unsigned long now, unsigned long &lastTs) {
  if (lastTs == 0 || now - lastTs >= SENSOR_MIN_INTERVAL) {
    lastTs = now;
    return true;
  }
  return false;
}

void loadPinsFromPrefs() {
  prefsSystem.begin("system", true);
  pinI2C_SDA = prefsSystem.getInt("i2c_sda", DEFAULT_I2C_SDA_PIN);
  pinI2C_SCL = prefsSystem.getInt("i2c_scl", DEFAULT_I2C_SCL_PIN);
  pinCO2_RX = prefsSystem.getInt("co2_rx", DEFAULT_CO2_RX_PIN);
  pinCO2_TX = prefsSystem.getInt("co2_tx", DEFAULT_CO2_TX_PIN);
  prefsSystem.end();

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

void appendCloudLogLine(const String &line) {
  size_t idx = (cloudLogStart + cloudLogCount) % CLOUD_LOG_CAPACITY;
  if (cloudLogCount == CLOUD_LOG_CAPACITY) {
    cloudLogStart = (cloudLogStart + 1) % CLOUD_LOG_CAPACITY;
    idx = (cloudLogStart + cloudLogCount - 1) % CLOUD_LOG_CAPACITY;
  } else {
    cloudLogCount++;
  }
  cloudLogBuffer[idx] = line;
}

String sanitizeLogField(const String &value) {
  String out = value;
  out.replace("\n", " ");
  out.replace("\r", " ");
  out.trim();
  return out;
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

void logEvent(const String &msg, const String &level, const String &source) {
  pruneLogsIfLowMemory(true);
  String line;
  line.reserve(msg.length() + 12);
  line = String(millis() / 1000);
  line += "s: ";
  line += msg;
  appendLogLine(line);

  if (cloudLoggingActive()) {
    String iso = isTimeSynced() ? isoTimestamp(currentEpochMs()) : "";
    if (iso.length() == 0) iso = String(millis() / 1000) + "s";
    String cloudLine = iso + " | " + sanitizeLogField(level) + " | " + sanitizeLogField(source) + " | " + sanitizeLogField(msg);
    appendCloudLogLine(cloudLine);
  }
}

void flushCloudLogs(bool force = false) {
  if (!cloudLoggingActive()) return;
  if (cloudLogCount == 0) return;
  unsigned long now = millis();
  if (!force && (cloudLogCount < CLOUD_LOG_FLUSH_LINES) && (lastCloudLogFlushMs != 0) && (now - lastCloudLogFlushMs < CLOUD_LOG_FLUSH_INTERVAL_MS)) {
    return;
  }
  uint64_t nowEpoch = currentEpochMs();
  String dayKey = currentDayKey();
  if (dayKey.length() == 0) dayKey = "unsynced";
  size_t count = cloudLogCount;
  String payload;
  payload.reserve(count * 64);
  for (size_t i = 0; i < count; i++) {
    size_t idx = (cloudLogStart + i) % CLOUD_LOG_CAPACITY;
    payload += cloudLogBuffer[idx];
    if (i < count - 1) payload += "\n";
  }
  cloudLogStart = 0;
  cloudLogCount = 0;
  lastCloudLogFlushMs = now;
  cloudLogSeq++;
  String stamp = fileSafeTimestamp(nowEpoch);
  String path = cloudDeviceRoot() + "/logs/chunks/log_" + stamp + "_" + String(cloudLogSeq) + ".log";
  enqueueCloudJob(path, payload, dayKey, "logs", "text/plain", false);
}

String cloudShortPath(const String &path) {
  if (path.length() == 0) return "/";
  return path.startsWith("/") ? path : (String("/") + path);
}

String cloudRequestStateText(const char *op, const String &label, int code) {
  return String(op) + " " + label + " -> " + String(code) + " (" + cloudProtocolLabel() + ")";
}

void setCloudRequestNote(const char *op, const String &label, const String &path, int code, const String &chain, bool httpsRedirected) {
  cloudStatus.lastHttpCode = code;
  cloudStatus.lastUrl = cloudShortPath(path);
  cloudStatus.lastErrorSuffix = httpsRedirected ? "redirected-to-https" : "";
  if (chain.length() > 0) {
    cloudStatus.lastError = String(op) + " " + label + " -> " + chain + " (" + cloudProtocolLabel() + ")";
  }
}

void setCloudError(const char *op, const String &label, const String &path, int code, const String &detail = "", const String &suffix = "") {
  cloudStatus.lastHttpCode = code;
  cloudStatus.lastUrl = cloudShortPath(path);
  String msg = String(op) + " " + label + " -> " + String(code);
  if (detail.length() > 0 && code <= 0) msg += String(" ") + detail;
#if CLOUD_DIAG
  if (detail.length() > 0 && code > 0) msg += String(" ") + detail;
#endif
  msg += String(" (") + cloudProtocolLabel() + ")";
  markCloudFailure(msg);
  cloudStatus.lastErrorSuffix = suffix;
}

bool webdavRequest(const String &method, const String &url, const String &body, const char *contentType, int &code, String *resp = nullptr, unsigned long timeoutMs = CLOUD_TEST_TIMEOUT_MS, String *location = nullptr) {
  HTTPClient http;
  http.setTimeout(timeoutMs);
  CloudRequestClients clients;
  if (!beginCloudRequest(http, url, clients)) {
    code = -1;
    if (resp) *resp = cloudProtocol() == CloudProtocol::HTTPS ? "HTTPS connect failed" : "HTTP connect failed";
    return false;
  }
  if (location) {
    const char *headers[] = {"Location"};
    http.collectHeaders(headers, 1);
    *location = "";
  }
  if (cloudConfig.username.length() > 0) {
    http.setAuthorization(cloudConfig.username.c_str(), cloudConfig.password.c_str());
  }
  if (contentType) http.addHeader("Content-Type", contentType);
  if (method == "PROPFIND") http.addHeader("Depth", "0");
  if (method == "MKCOL") {
    http.addHeader("Content-Length", "0");
    code = http.sendRequest(method.c_str(), (uint8_t *)nullptr, 0);
  } else if (method == "PUT") {
    code = http.sendRequest(method.c_str(), (uint8_t *)body.c_str(), body.length());
  } else {
    code = http.sendRequest(method.c_str(), body);
  }
  if (code <= 0) {
    if (resp) {
      *resp = cloudProtocol() == CloudProtocol::HTTPS ? "TLS handshake failed" : "HTTP request failed";
    }
    http.end();
    return false;
  }
  if (resp) {
    String out = http.getString();
    if (out.length() > 120) out = out.substring(0, 120);
    *resp = out;
  }
  if (location) {
    String loc = http.header("Location");
    if (loc.length() > 120) loc = loc.substring(0, 120);
    *location = loc;
  }
  http.end();
  return code > 0;
}

String redirectUrlRoot(const String &url) {
  int schemeIdx = url.indexOf("://");
  if (schemeIdx < 0) return "";
  int hostStart = schemeIdx + 3;
  int slashPos = url.indexOf('/', hostStart);
  if (slashPos < 0) return url;
  return url.substring(0, slashPos);
}

String redirectUrlBase(const String &url) {
  int slashPos = url.lastIndexOf('/');
  if (slashPos < 0) return url;
  return url.substring(0, slashPos + 1);
}

String resolveRedirectUrl(const String &currentUrl, const String &location) {
  if (location.startsWith("http")) return location;
  if (location.startsWith("/")) {
    String root = redirectUrlRoot(currentUrl);
    return root.length() > 0 ? root + location : location;
  }
  return redirectUrlBase(currentUrl) + location;
}

bool webdavRequestFollowRedirects(const String &method, const String &url, const String &payload, const char *contentType, int &outCode, String &outLocationShort, String &outBodyShort, String *chainOut, bool *redirectedToHttps, unsigned long timeoutMs) {
  String currentUrl = url;
  String chain;
  bool httpsRedirected = false;
  int redirectCount = 0;
  outLocationShort = "";
  outBodyShort = "";
  for (;;) {
    String resp;
    String location;
    int code = 0;
    bool ok = webdavRequest(method, currentUrl, payload, contentType, code, &resp, timeoutMs, &location);
    outCode = code;
    outLocationShort = location;
    outBodyShort = resp;
    if (!ok) {
      if (chainOut) *chainOut = chain;
      if (redirectedToHttps) *redirectedToHttps = httpsRedirected;
      return false;
    }
    if (code == 301 || code == 302 || code == 307 || code == 308) {
      if (chain.length() > 0) chain += "->";
      chain += String(code);
      String loc = location;
      loc.trim();
      if (loc.length() == 0) {
        if (chainOut) *chainOut = chain;
        if (redirectedToHttps) *redirectedToHttps = httpsRedirected;
        return false;
      }
      if (loc.startsWith("https://")) {
        if (cloudProtocol() != CloudProtocol::HTTPS) {
          httpsRedirected = true;
          if (chainOut) *chainOut = chain;
          if (redirectedToHttps) *redirectedToHttps = httpsRedirected;
          return false;
        }
      } else if (loc.startsWith("http://") && cloudProtocol() == CloudProtocol::HTTPS) {
        if (chainOut) *chainOut = chain;
        if (redirectedToHttps) *redirectedToHttps = httpsRedirected;
        return false;
      }
      if (redirectCount >= 3) {
        if (chainOut) *chainOut = chain + " loop";
        if (redirectedToHttps) *redirectedToHttps = httpsRedirected;
        return false;
      }
      redirectCount++;
      currentUrl = resolveRedirectUrl(currentUrl, loc);
      continue;
    }
    if (chain.length() > 0) {
      chain += "->";
      chain += String(code);
    }
    if (chainOut) *chainOut = chain;
    if (redirectedToHttps) *redirectedToHttps = httpsRedirected;
    if (code >= 200 && code < 300) return true;
    if (method == "MKCOL" && (code == 405 || code == 409)) return true;
    return false;
  }
}

bool ensureCollection(const String &path, const char *label) {
  String url = buildWebDavUrl(path);
  int code = 0;
  String resp;
  String location;
  String chain;
  bool httpsRedirected = false;
  bool ok = webdavRequestFollowRedirects("MKCOL", url, "", "text/plain", code, location, resp, &chain, &httpsRedirected);
  setCloudRequestNote("MKCOL", label, path, code, chain, httpsRedirected);
  if (!ok) {
    if (chain.length() > 0) {
      markCloudFailure(cloudStatus.lastError);
      cloudStatus.lastErrorSuffix = httpsRedirected ? "redirected-to-https" : "";
    } else {
      setCloudError("MKCOL", label, path, code, resp, httpsRedirected ? "redirected-to-https" : "");
    }
    return false;
  }
  return true;
}

bool pingCloud(bool force = false) {
  if (!cloudStatus.runtimeEnabled) return false;
  if (WiFi.status() != WL_CONNECTED) {
    markCloudFailure("WiFi disconnected");
    return false;
  }
  unsigned long now = millis();
  if (!force && lastCloudPingMs != 0 && now - lastCloudPingMs < CLOUD_PING_INTERVAL_MS) return cloudHealthOk();
  lastCloudPingMs = now;
  String url = safeBaseUrl();
  if (url.length() == 0) return false;
  int code = 0;
  String resp;
  String location;
  String chain;
  bool httpsRedirected = false;
  bool ok = webdavRequestFollowRedirects("PROPFIND", url, "", "text/plain", code, location, resp, &chain, &httpsRedirected);
  setCloudRequestNote("PROPFIND", "base", "/", code, chain, httpsRedirected);
  if (!ok) {
    if (chain.length() > 0) {
      markCloudFailure(cloudStatus.lastError);
      cloudStatus.lastErrorSuffix = httpsRedirected ? "redirected-to-https" : "";
      return false;
    }
    if (code == 401 || code == 403) {
      setCloudError("PROPFIND", "auth", "/", code, resp, httpsRedirected ? "redirected-to-https" : "");
      return false;
    }
    if (code == 404) {
      setCloudError("PROPFIND", "base", "/", code, resp, httpsRedirected ? "redirected-to-https" : "");
      return false;
    }
    setCloudError("PROPFIND", "base", "/", code, resp, httpsRedirected ? "redirected-to-https" : "");
    return false;
  }
  if (!cloudEnsureFolders("")) {
    return false;
  }
  markCloudSuccess(cloudRequestStateText("PROPFIND", "base", code));
  if (chain.length() > 0) {
    cloudStatus.lastError = String("PROPFIND base -> ") + chain;
  }
  return true;
}

bool testCloudConnection() {
  return pingCloud(true);
}

bool cloudGet(const String &path, String &resp, int &code) {
  String fullPath = path.startsWith("/") ? path : (String("/") + path);
  String url = buildWebDavUrl(fullPath);
  return webdavRequest("GET", url, "", nullptr, code, &resp, CLOUD_TEST_TIMEOUT_MS);
}

bool fetchCloudDailyPoint(const String &dayKey, const String &metric, DailyPoint &out) {
  String path = cloudDailyPath(dayKey);
  if (path.length() == 0) return false;
  int code = 0;
  String payload;
  if (!cloudGet(path, payload, code)) {
    markCloudFailure("Daily GET failed");
    return false;
  }
  if (code == 404) return false;
  if (code < 200 || code >= 300) {
    markCloudFailure("Daily GET code: " + String(code));
    return false;
  }
  DynamicJsonDocument doc(4096);
  if (deserializeJson(doc, payload) != DeserializationError::Ok) {
    markCloudFailure("Daily JSON parse failed");
    return false;
  }
  markCloudSuccess(cloudRequestStateText("GET", "daily", code));
  JsonObject sensors = doc["sensors"];
  if (!sensors.containsKey(metric)) return false;
  JsonObject m = sensors[metric];
  out.dayKey = doc["date"] | dayKey;
  out.avg = m["avg"] | NAN;
  out.min = m["min"] | NAN;
  out.max = m["max"] | NAN;
  out.last = m["last"] | NAN;
  out.count = m["samples"] | m["count"] | 0;
  return out.count > 0;
}

bool uploadCloudJob(const CloudJob &job) {
  String base = safeBaseUrl();
  if (base.length() == 0) return false;
  String fullPath = job.path;
  if (!fullPath.startsWith("/")) fullPath = "/" + fullPath;
  if (!cloudEnsureFolders(job.dayKey)) {
    return false;
  }
  String url = buildWebDavUrl(fullPath);
  int code = 0;
  String resp;
  const char *ctype = job.contentType.length() > 0 ? job.contentType.c_str() : "application/json";
  String location;
  String chain;
  bool httpsRedirected = false;
  bool ok = webdavRequestFollowRedirects("PUT", url, job.payload, ctype, code, location, resp, &chain, &httpsRedirected, CLOUD_TEST_TIMEOUT_MS);
  setCloudRequestNote("PUT", "upload", fullPath, code, chain, httpsRedirected);
  if (!ok) {
    if (chain.length() > 0) {
      markCloudFailure(cloudStatus.lastError);
      cloudStatus.lastErrorSuffix = httpsRedirected ? "redirected-to-https" : "";
    } else {
      setCloudError("PUT", "upload", fullPath, code, resp, httpsRedirected ? "redirected-to-https" : "");
    }
    return false;
  }
  cloudStatus.lastUploadMs = currentEpochMs();
  cloudStatus.lastUploadedPath = fullPath;
  cloudStatus.lastError = "";
  markCloudSuccess(cloudRequestStateText("PUT", "upload", code));
  if (chain.length() > 0) {
    cloudStatus.lastError = String("PUT upload -> ") + chain;
  }
  cloudStatus.queueSize = cloudQueue.size();
  return true;
}

void processCloudQueue() {
  unsigned long now = millis();
  if (lastCloudWorkerTickMs != 0 && now - lastCloudWorkerTickMs < CLOUD_WORKER_INTERVAL_MS) return;
  lastCloudWorkerTickMs = now;
  cloudStatus.queueSize = cloudQueue.size();
  if (!cloudStatus.runtimeEnabled || cloudQueue.empty()) {
    refreshStorageMode();
    return;
  }
  if (WiFi.status() != WL_CONNECTED) {
    refreshStorageMode();
    return;
  }
  if (nextCloudAttemptAfterMs != 0 && now < nextCloudAttemptAfterMs) return;
  CloudJob &job = cloudQueue.front();
  if (job.nextAttemptAt != 0 && now < job.nextAttemptAt) return;
  bool ok = uploadCloudJob(job);
  if (ok) {
    cloudQueue.erase(cloudQueue.begin());
    cloudStatus.queueSize = cloudQueue.size();
    nextCloudAttemptAfterMs = now + CLOUD_WORKER_INTERVAL_MS;
  } else {
    job.attempts++;
    unsigned long backoff = job.attempts <= 1 ? CLOUD_BACKOFF_SHORT_MS : (job.attempts <= 3 ? CLOUD_BACKOFF_MED_MS : CLOUD_BACKOFF_LONG_MS);
    job.nextAttemptAt = now + backoff;
    nextCloudAttemptAfterMs = job.nextAttemptAt;
    if (job.attempts > 5) {
      cloudQueue.erase(cloudQueue.begin());
      cloudStatus.queueSize = cloudQueue.size();
    }
  }
  refreshStorageMode();
}

void maintainCloudHealth() {
  if (!cloudStatus.runtimeEnabled) {
    refreshStorageMode("cloud disabled");
    return;
  }
  if (WiFi.status() != WL_CONNECTED) {
    refreshStorageMode("wifi lost");
    return;
  }
  pingCloud(false);
}

void tickCloudRecording() {
  if (!cloudStatus.runtimeEnabled || !cloudConfig.recording) return;
  uint64_t nowEpoch = currentEpochMs();
  String minuteKey = minuteKeyFromEpoch(nowEpoch);
  if (minuteKey.length() == 0) return;
  if (minuteKey == lastRecordingMinuteKey && lastRecordingEnqueueMs != 0 && millis() - lastRecordingEnqueueMs < CLOUD_RECORDING_INTERVAL_MS) return;
  enqueueRecordingSample(nowEpoch);
  lastRecordingMinuteKey = minuteKey;
  lastRecordingEnqueueMs = millis();
}

bool sendCloudTestFile(int &httpCode, size_t &bytesOut, String &pathOut, String &errorOut) {
  bytesOut = 0;
  if (!cloudStatus.runtimeEnabled) {
    errorOut = "Cloud disabled";
    return false;
  }
  uint64_t nowMs = currentEpochMs();
  String stamp = fileSafeTimestamp(nowMs);
  String dayKey = currentDayKey();
  if (dayKey.length() == 0) dayKey = "unsynced";
  pathOut = cloudDeviceRoot() + "/meta/TestCloud_" + stamp + ".txt";
  if (!cloudEnsureFolders(dayKey)) {
    errorOut = cloudStatus.lastError.length() > 0 ? cloudStatus.lastError : "MKCOL failed";
    return false;
  }
  IPAddress ip = apMode ? WiFi.softAPIP() : WiFi.localIP();
  String body = "ts=" + isoTimestamp(nowMs) + "\n";
  body += "device=" + deviceId() + "\n";
  body += "ssid=" + (WiFi.status() == WL_CONNECTED ? WiFi.SSID() : savedSsid) + "\n";
  body += "ip=" + ip.toString() + "\n";
  body += "base_path=" + cloudRootPath() + "\n";
  int code = 0;
  String resp;
  bytesOut = body.length();
  String location;
  String chain;
  bool httpsRedirected = false;
  bool ok = webdavRequestFollowRedirects("PUT", buildWebDavUrl(pathOut), body, "text/plain", code, location, resp, &chain, &httpsRedirected, CLOUD_TEST_TIMEOUT_MS);
  setCloudRequestNote("PUT", "test", pathOut, code, chain, httpsRedirected);
  httpCode = code;
  if (ok) {
    cloudStatus.lastTestMs = currentEpochMs();
    cloudStatus.lastUploadMs = cloudStatus.lastTestMs;
    cloudStatus.lastUploadedPath = pathOut;
    cloudStatus.lastError = "";
    markCloudSuccess(cloudRequestStateText("PUT", "test", code));
    if (chain.length() > 0) {
      cloudStatus.lastError = String("PUT test -> ") + chain;
    }
    return true;
  }
  if (chain.length() > 0) {
    markCloudFailure(cloudStatus.lastError);
    cloudStatus.lastErrorSuffix = httpsRedirected ? "redirected-to-https" : "";
  } else {
    setCloudError("PUT", "test", pathOut, code, resp, httpsRedirected ? "redirected-to-https" : "");
  }
  errorOut = cloudStatus.lastError;
  return false;
}

void applyTimezoneEnv() {
  const String tz = timezoneName.length() > 0 ? timezoneName : "UTC";
  setenv("TZ", tz.c_str(), 1);
  tzset();
}

bool isTimeSynced() {
  time_t now = time(nullptr);
  if (now < 0) return false;
  struct tm utcTm;
  if (gmtime_r(&now, &utcTm) == nullptr) return false;
  if (now < 1700000000 && (utcTm.tm_year + 1900) < 2024) return false;
  return true;
}

void onTimeSynchronized(uint64_t epochNowMs, uint32_t monotonicNowMs) {
  bool wasSynced = timeSynced;
  bootMillisAtSync = monotonicNowMs;
  bootEpochMs = epochNowMs > bootMillisAtSync ? (epochNowMs - bootMillisAtSync) : 0ULL;
  lastTimeSyncOk = monotonicNowMs;
  timeSynced = true;
  if (!wasSynced) {
    logEvent("Time synced: bootEpochMs baseline updated");
  }
}

bool captureTimeOffset() {
  struct timeval tv;
  if (gettimeofday(&tv, nullptr) != 0) return false;
  if (tv.tv_sec < TIME_VALID_AFTER) return false;
  if (!isTimeSynced()) return false;
  uint64_t nowMs = (uint64_t)tv.tv_sec * 1000ULL + (tv.tv_usec / 1000ULL);
  uint32_t mono = millis();
  onTimeSynchronized(nowMs, mono);
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
  unsigned long now = millis();
  bool currentlySynced = isTimeSynced();
  if (currentlySynced && !timeSynced) {
    onTimeSynchronized(currentEpochMs(), now);
  } else if (!currentlySynced) {
    timeSynced = false;
  }
  if (WiFi.status() != WL_CONNECTED)
    return;
  bool needKick = (!timeSynced && (now - lastTimeSyncAttempt >= TIME_SYNC_RETRY_MS)) || lastTimeSyncAttempt == 0;
  if (needKick) {
    startNtpSync();
  }
  bool shouldRefresh = (!timeSynced && now - lastTimeSyncAttempt >= 1500) || (timeSynced && (now - lastTimeSyncOk >= TIME_SYNC_REFRESH_MS));
  if (shouldRefresh) {
    captureTimeOffset();
  }
}

uint64_t mapTimestampToEpochMs(uint32_t monotonicTs) {
  if (bootEpochMs != 0) return bootEpochMs + monotonicTs;
  return (uint64_t)monotonicTs;
}

uint64_t currentEpochMs() {
  if (isTimeSynced()) {
    struct timeval tv;
    if (gettimeofday(&tv, nullptr) == 0) {
      return (uint64_t)tv.tv_sec * 1000ULL + (tv.tv_usec / 1000ULL);
    }
  }
  return mapTimestampToEpochMs(millis());
}

void loadPartners() {
  partners.clear();
  prefsUi.begin("ui", true);
  String raw = prefsUi.getString("partners", "[]");
  prefsUi.end();
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
  prefsSensors.begin("sensors", false);
  prefsSensors.putString("sensors_cfg", out);
  prefsSensors.end();
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
  prefsSensors.begin("sensors", true);
  String raw = prefsSensors.getString("sensors_cfg", "");
  prefsSensors.end();
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
  wifiConnectInProgress = false;
  wifiFallbackAt = 0;
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  delay(100); // give the AP time to start

  IPAddress apIP = WiFi.softAPIP();
  dnsServer.start(DNS_PORT, "*", apIP);
  Serial.printf("[AP] Started %s (%s)\n", AP_SSID, apIP.toString().c_str());
  logEvent("AP mode active: " + String(AP_SSID));
}

void startMdns() {
#if ENABLE_MDNS
  if (deviceHostname.isEmpty()) deviceHostname = "growsensor";
  if (!MDNS.begin(deviceHostname.c_str())) {
    logEvent("mDNS start failed");
    return;
  }
  MDNS.addService("http", "tcp", 80);
  logEvent("mDNS active: " + deviceHostname + ".local");
#endif
}

bool beginWifiConnect(const String &ssid, const String &pass, bool waitForResult) {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  if (staticIpEnabled && staticIp != IPAddress((uint32_t)0) && staticGateway != IPAddress((uint32_t)0) && staticSubnet != IPAddress((uint32_t)0)) {
    WiFi.config(staticIp, staticGateway, staticSubnet);
  }
  WiFi.begin(ssid.c_str(), pass.c_str());
  wifiConnectInProgress = true;
  wifiConnectStarted = millis();
  wifiFallbackAt = wifiConnectStarted + WIFI_TIMEOUT + 10000;
  if (!waitForResult) return false;

  while (WiFi.status() != WL_CONNECTED && millis() - wifiConnectStarted < WIFI_TIMEOUT) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnectInProgress = false;
    apMode = false;
    wifiFallbackAt = 0;
    logEvent("WiFi connected: " + WiFi.localIP().toString());
    startMdns();
    startNtpSync();
    return true;
  }
  wifiConnectInProgress = false;
  return false;
}

bool connectToWiFi() {
  prefsWifi.begin("wifi", false);
  savedSsid = prefsWifi.getString("ssid", "");
  savedWifiPass = prefsWifi.getString("wifi_pass", "");
  bool hasWifiPassKey = prefsWifi.isKey("wifi_pass");
  bool hasLegacyPassKey = prefsWifi.isKey("pass");
  String legacyPass = hasLegacyPassKey ? prefsWifi.getString("pass", "") : "";
  prefsWifi.end();
  prefsSystem.begin("system", false);
  channel = lightChannelFromString(prefsSystem.getString("channel", "full_spectrum"));
  climateType = climateFromString(prefsSystem.getString("climate_type", "sht31"));
  co2Type = co2FromString(prefsSystem.getString("co2_type", "mhz19"));
  timezoneName = prefsSystem.getString("timezone", timezoneName);
  vpdStageId = prefsSystem.getString("vpd_stage", "seedling");
  prefsSystem.end();
  prefsWifi.begin("wifi", false);
  staticIpEnabled = prefsWifi.getBool("static", false);
  staticIp.fromString(prefsWifi.getString("ip", ""));
  staticGateway.fromString(prefsWifi.getString("gw", ""));
  staticSubnet.fromString(prefsWifi.getString("sn", ""));
  prefsWifi.end();
  prefsSensors.begin("sensors", false);
  enableLight = prefsSensors.getBool("en_light", true);
  enableClimate = prefsSensors.getBool("en_climate", true);
  enableLeaf = prefsSensors.getBool("en_leaf", true);
  enableCo2 = prefsSensors.getBool("en_co2", true);
  prefsSensors.end();
  bool migrateLegacyWifiPass = !hasWifiPassKey && hasLegacyPassKey && savedWifiPass.isEmpty() && !legacyPass.isEmpty() && !savedSsid.isEmpty();
  if (migrateLegacyWifiPass) {
    savedWifiPass = legacyPass;
    prefsWifi.begin("wifi", false);
    prefsWifi.putString("wifi_pass", savedWifiPass);
    prefsWifi.end();
  }
  applyTimezoneEnv();
  loadPartners();
  loadSensorConfigs();
  rebuildSensorList();

  if (savedSsid.isEmpty()) {
    Serial.println("[WiFi] No stored credentials, starting AP");
    startAccessPoint();
    return false;
  }

  Serial.printf("[WiFi] Connecting to %s ...\n", savedSsid.c_str());
  bool ok = beginWifiConnect(savedSsid, savedWifiPass, true);

  if (ok) {
    apMode = false;
    Serial.printf("[WiFi] Connected, IP: %s\n", WiFi.localIP().toString().c_str());
    return true;
  }

  Serial.println("[WiFi] Connection failed, starting AP");
  logEvent("WiFi connection failed, switching to AP");
  startAccessPoint();
  return false;
}

void saveWifiCredentials(const String &ssid, const String &pass) {
  prefsWifi.begin("wifi", false);
  prefsWifi.putString("ssid", ssid);
  prefsWifi.putString("wifi_pass", pass);
  prefsWifi.end();
  savedSsid = ssid;
  savedWifiPass = pass;
}

void maintainWifiConnection() {
  if (wifiConnectInProgress) {
    if (WiFi.status() == WL_CONNECTED) {
      wifiConnectInProgress = false;
      apMode = false;
      wifiFallbackAt = 0;
      logEvent("WiFi connected: " + WiFi.localIP().toString());
      startMdns();
      startNtpSync();
    } else if (wifiFallbackAt > 0 && millis() > wifiFallbackAt) {
      logEvent("WiFi connect timeout, enabling AP fallback");
      wifiConnectInProgress = false;
      startAccessPoint();
    }
    return;
  }
  if (!apMode && WiFi.status() != WL_CONNECTED && wifiFallbackAt > 0 && millis() > wifiFallbackAt) {
    logEvent("WiFi unavailable, enabling AP fallback");
    startAccessPoint();
  }
}

void clearPreferences() {
  prefsWifi.begin("wifi", false);
  prefsWifi.clear();
  prefsWifi.end();
  prefsSensors.begin("sensors", false);
  prefsSensors.clear();
  prefsSensors.end();
  prefsSystem.begin("system", false);
  prefsSystem.clear();
  prefsSystem.end();
  prefsUi.begin("ui", false);
  prefsUi.clear();
  prefsUi.end();
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
  prefsUi.begin("ui", false);
  prefsUi.putString("partners", out);
  prefsUi.end();
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
  unsigned long nowMs = millis();

  bool lightSampled = false;
  if (enableLight && lightHealth.present && allowSensorSample(nowMs, lastLightSampleMs) && lightMeter.measurementReady()) {
    lightSampled = true;
    latest.lux = lightMeter.readLightLevel();
    latest.ppfdFactor = currentPpfdFactor();
    latest.ppfd = luxToPPFD(latest.lux, channel);
    lightHealth.healthy = !isnan(latest.lux);
    lightHealth.lastUpdate = millis();
    if (!isnan(latest.lux)) {
      stallLight.lastValue = latest.lux;
      stallLight.lastChange = millis();
    }
  }

  float temp = NAN;
  float humidity = NAN;
  bool climateSampled = false;
  if (enableClimate && climateHealth.present && allowSensorSample(nowMs, lastClimateSampleMs)) {
    if (climateType == ClimateSensorType::SHT31) {
      temp = sht31.readTemperature();
      humidity = sht31.readHumidity();
    } else {
      temp = NAN;
      humidity = NAN;
    }
    climateSampled = true;
  }
  if (!isnan(temp) && !isnan(humidity)) {
    latest.ambientTempC = temp;
    latest.humidity = humidity;
    climateHealth.healthy = true;
    climateHealth.lastUpdate = millis();
    stallClimateTemp.lastValue = temp;
    stallClimateHum.lastValue = humidity;
    stallClimateTemp.lastChange = millis();
    stallClimateHum.lastChange = millis();
  } else if (climateSampled) {
    climateHealth.healthy = false;
  }

  bool leafSampled = false;
  double objTemp = (enableLeaf && leafHealth.present && allowSensorSample(nowMs, lastLeafSampleMs)) ? mlx.readObjectTempC() : NAN;
  if (!isnan(objTemp)) {
    leafSampled = true;
    latest.leafTempC = objTemp;
    leafHealth.healthy = true;
    leafHealth.lastUpdate = millis();
    stallLeaf.lastValue = objTemp;
    stallLeaf.lastChange = millis();
  } else if (leafSampled) {
    leafHealth.healthy = false;
  }

  bool co2Sampled = false;
  if (enableCo2 && co2Health.present && allowSensorSample(nowMs, lastCo2SampleMs)) {
    if (co2Type == Co2SensorType::MHZ19 || co2Type == Co2SensorType::MHZ14) {
      co2Sampled = true;
      int ppm = co2Sensor.getCO2();
      if (ppm > 0 && ppm < 5000) {
        latest.co2ppm = ppm;
        co2Health.healthy = true;
        co2Health.lastUpdate = millis();
        stallCo2.lastValue = ppm;
        stallCo2.lastChange = millis();
      } else {
        co2Health.healthy = false;
      }
    } else {
      co2Health.healthy = false; // unsupported sensor type
    }
  }

  // Compute VPD when data is valid and IR sensor is not drifting excessively
  bool vpdUpdated = climateSampled || leafSampled;
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
  String sampleDay = isTimeSynced() ? dayKeyForEpoch(sampleTs) : String("unsynced");
  int sampleHour = hourFromEpoch(sampleTs);
  if (activeDayKey.length() == 0) {
    activeDayKey = sampleDay;
  } else if (sampleDay.length() > 0 && sampleDay != activeDayKey) {
    finalizeDayAndQueue(activeDayKey);
    activeDayKey = sampleDay;
    resetAllDaily(sampleDay);
    lastDailyCheckpointMs = 0;
  }
  if (lightSampled) {
    pushHistoryValue("lux", latest.lux, sampleTs);
    pushHistoryValue("ppfd", latest.ppfd, sampleTs);
    accumulateDaily(metricIndex("lux"), latest.lux, sampleDay, sampleHour);
    accumulateDaily(metricIndex("ppfd"), latest.ppfd, sampleDay, sampleHour);
  }
  if (co2Sampled) {
    pushHistoryValue("co2", static_cast<float>(latest.co2ppm), sampleTs);
    accumulateDaily(metricIndex("co2"), static_cast<float>(latest.co2ppm), sampleDay, sampleHour);
  }
  if (climateSampled) {
    pushHistoryValue("temp", latest.ambientTempC, sampleTs);
    pushHistoryValue("humidity", latest.humidity, sampleTs);
    accumulateDaily(metricIndex("temp"), latest.ambientTempC, sampleDay, sampleHour);
    accumulateDaily(metricIndex("humidity"), latest.humidity, sampleDay, sampleHour);
  }
  if (leafSampled) {
    pushHistoryValue("leaf", latest.leafTempC, sampleTs);
    accumulateDaily(metricIndex("leaf"), latest.leafTempC, sampleDay, sampleHour);
  }
  if (vpdUpdated && !isnan(latest.vpd)) {
    pushHistoryValue("vpd", latest.vpd, sampleTs);
    accumulateDaily(metricIndex("vpd"), latest.vpd, sampleDay, sampleHour);
  }

  if (cloudStorageActive() && sampleDay != "unsynced") {
    if (lastDailyCheckpointMs == 0 || millis() - lastDailyCheckpointMs >= DAILY_CHECKPOINT_MS) {
      String checkpointPayload = serializeDailyPayload(sampleDay);
      enqueueDailyJob(sampleDay, checkpointPayload);
      lastDailyCheckpointMs = millis();
    }
  }

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
static const uint8_t INDEX_HTML_GZ[] PROGMEM = {
  31, 139, 8, 0, 235, 12, 71, 105, 2, 255, 236, 189, 219, 114, 27, 73,
  146, 40, 248, 222, 95, 145, 197, 170, 110, 0, 85, 0, 8, 128, 23, 145,
  160, 72, 29, 137, 162, 90, 156, 166, 46, 43, 178, 164, 174, 214, 200, 164,
  36, 16, 32, 178, 153, 64, 98, 50, 19, 188, 136, 135, 102, 99, 231, 97,
  95, 215, 214, 102, 109, 119, 223, 214, 236, 88, 127, 192, 177, 53, 219, 243,
  212, 79, 167, 254, 164, 191, 100, 221, 61, 46, 25, 183, 76, 128, 148, 170,
  187, 218, 118, 167, 167, 68, 100, 92, 60, 60, 60, 60, 60, 60, 60, 60,
  60, 126, 19, 4, 15, 191, 25, 38, 131, 252, 122, 198, 130, 113, 62, 137,
  247, 126, 3, 41, 248, 35, 136, 195, 233, 217, 238, 202, 144, 173, 240, 36,
  22, 14, 241, 7, 252, 156, 176, 60, 12, 6, 227, 48, 205, 88, 190, 187,
  242, 227, 201, 179, 214, 214, 74, 176, 170, 103, 78, 195, 9, 219, 93, 185,
  136, 216, 229, 44, 73, 243, 149, 96, 144, 76, 115, 54, 133, 194, 151, 209,
  48, 31, 239, 14, 217, 69, 52, 96, 45, 250, 104, 6, 209, 52, 202, 163,
  48, 110, 101, 131, 48, 102, 187, 221, 2, 84, 30, 229, 49, 219, 251, 125,
  154, 92, 30, 179, 105, 150, 164, 193, 69, 167, 189, 214, 94, 123, 184, 202,
  51, 120, 161, 44, 191, 150, 191, 131, 160, 159, 38, 73, 30, 220, 64, 123,
  113, 146, 2, 192, 49, 155, 176, 126, 16, 71, 103, 227, 60, 24, 134, 233,
  249, 78, 112, 43, 74, 98, 15, 155, 193, 105, 50, 188, 134, 226, 167, 225,
  224, 252, 44, 77, 230, 211, 97, 63, 248, 182, 51, 234, 62, 232, 133, 59,
  28, 6, 124, 179, 30, 219, 26, 117, 118, 130, 73, 52, 109, 141, 25, 130,
  234, 7, 221, 78, 231, 183, 5, 44, 1, 101, 4, 157, 108, 141, 194, 73,
  20, 95, 247, 131, 236, 58, 203, 217, 164, 53, 143, 154, 65, 22, 78, 179,
  86, 198, 210, 104, 4, 64, 194, 244, 44, 154, 246, 3, 128, 55, 11, 135,
  195, 104, 122, 70, 191, 21, 86, 64, 100, 150, 2, 44, 149, 217, 221, 156,
  93, 237, 152, 8, 118, 187, 221, 173, 222, 3, 72, 76, 174, 90, 217, 56,
  28, 38, 151, 0, 35, 232, 205, 174, 2, 40, 27, 164, 103, 167, 97, 189,
  211, 164, 255, 181, 123, 27, 13, 104, 40, 201, 128, 192, 9, 180, 154, 229,
  209, 224, 252, 122, 39, 200, 147, 25, 53, 251, 185, 21, 77, 135, 236, 10,
  90, 233, 105, 56, 116, 161, 125, 13, 79, 234, 86, 22, 125, 6, 66, 118,
  219, 189, 148, 77, 52, 136, 41, 139, 195, 60, 186, 96, 69, 237, 54, 239,
  66, 11, 6, 13, 192, 12, 163, 108, 22, 135, 215, 253, 81, 204, 160, 23,
  127, 158, 3, 2, 163, 235, 150, 224, 6, 192, 103, 22, 2, 27, 156, 178,
  252, 146, 177, 233, 78, 16, 194, 64, 77, 91, 17, 208, 45, 235, 7, 3,
  40, 193, 210, 157, 224, 44, 156, 245, 187, 61, 36, 2, 2, 105, 93, 166,
  240, 29, 224, 191, 90, 155, 0, 112, 234, 180, 166, 67, 211, 129, 109, 33,
  44, 85, 53, 102, 67, 168, 73, 156, 40, 154, 17, 99, 204, 63, 78, 147,
  148, 122, 19, 14, 163, 121, 214, 223, 192, 81, 215, 198, 226, 91, 54, 90,
  135, 255, 51, 134, 162, 19, 224, 255, 214, 228, 72, 244, 214, 182, 155, 155,
  91, 248, 255, 98, 52, 36, 146, 209, 52, 142, 166, 208, 249, 56, 25, 104,
  108, 217, 62, 13, 135, 103, 76, 235, 139, 89, 76, 178, 197, 58, 128, 223,
  114, 241, 219, 222, 222, 38, 66, 169, 17, 235, 180, 183, 104, 192, 116, 156,
  187, 163, 222, 246, 218, 3, 201, 223, 223, 14, 78, 135, 27, 172, 107, 99,
  208, 130, 41, 106, 78, 140, 111, 187, 157, 211, 237, 173, 174, 170, 39, 167,
  137, 85, 239, 50, 76, 167, 86, 197, 209, 198, 54, 235, 156, 150, 87, 60,
  27, 39, 89, 110, 214, 201, 83, 152, 51, 179, 48, 133, 97, 147, 125, 236,
  119, 161, 207, 89, 18, 71, 195, 192, 238, 130, 156, 161, 18, 226, 36, 140,
  166, 238, 28, 146, 20, 13, 206, 210, 104, 200, 89, 33, 224, 131, 92, 240,
  81, 152, 14, 109, 113, 80, 204, 54, 194, 34, 240, 160, 97, 14, 130, 0,
  106, 207, 96, 99, 178, 138, 241, 171, 156, 172, 197, 220, 74, 46, 88, 58,
  138, 161, 230, 56, 26, 14, 113, 162, 40, 132, 51, 146, 139, 45, 129, 183,
  103, 90, 58, 85, 53, 41, 214, 221, 234, 232, 116, 225, 179, 134, 38, 217,
  48, 74, 217, 128, 32, 1, 133, 231, 19, 189, 69, 36, 158, 198, 158, 46,
  49, 49, 161, 5, 147, 14, 178, 115, 214, 226, 245, 51, 236, 205, 140, 133,
  121, 61, 156, 231, 73, 107, 20, 229, 77, 68, 100, 18, 94, 213, 123, 29,
  64, 162, 25, 116, 71, 105, 163, 161, 53, 115, 17, 198, 115, 38, 101, 170,
  20, 62, 155, 196, 203, 148, 116, 17, 166, 81, 8, 127, 167, 243, 9, 8,
  214, 65, 63, 200, 195, 211, 121, 28, 166, 152, 144, 21, 112, 226, 240, 148,
  197, 58, 186, 98, 30, 113, 241, 214, 34, 57, 184, 101, 78, 153, 160, 211,
  222, 166, 118, 36, 140, 104, 58, 155, 3, 190, 25, 139, 129, 38, 176, 100,
  204, 243, 60, 153, 42, 153, 33, 150, 130, 98, 180, 137, 166, 58, 252, 117,
  119, 158, 234, 115, 215, 207, 81, 198, 130, 116, 218, 237, 245, 58, 238, 130,
  164, 150, 31, 137, 209, 96, 158, 102, 88, 98, 150, 68, 92, 220, 201, 22,
  166, 201, 148, 153, 64, 81, 168, 0, 181, 206, 16, 33, 152, 100, 245, 110,
  175, 51, 100, 103, 205, 224, 219, 94, 111, 184, 198, 24, 252, 216, 92, 219,
  220, 28, 117, 27, 69, 179, 18, 13, 34, 213, 165, 88, 8, 55, 59, 14,
  34, 125, 32, 118, 120, 202, 229, 106, 2, 2, 62, 202, 175, 145, 168, 155,
  59, 10, 191, 105, 146, 183, 194, 24, 184, 146, 13, 181, 33, 55, 87, 140,
  128, 179, 35, 49, 22, 81, 203, 187, 56, 148, 47, 9, 8, 109, 47, 248,
  30, 89, 40, 166, 21, 78, 159, 53, 121, 152, 207, 51, 201, 93, 222, 174,
  136, 50, 237, 228, 92, 106, 19, 64, 129, 181, 245, 225, 218, 246, 182, 91,
  136, 165, 169, 86, 106, 180, 245, 160, 251, 64, 111, 46, 78, 206, 96, 246,
  219, 28, 99, 168, 19, 124, 38, 26, 10, 196, 60, 106, 77, 146, 105, 66,
  75, 100, 51, 56, 126, 246, 2, 62, 90, 111, 216, 25, 114, 121, 51, 120,
  193, 166, 113, 2, 127, 146, 105, 56, 128, 191, 251, 9, 200, 129, 56, 204,
  154, 193, 202, 81, 116, 202, 210, 16, 39, 47, 230, 38, 43, 144, 180, 159,
  204, 211, 8, 212, 138, 151, 236, 18, 62, 21, 212, 157, 224, 114, 12, 244,
  108, 209, 7, 112, 77, 202, 90, 246, 186, 10, 58, 94, 174, 11, 22, 77,
  40, 73, 220, 105, 2, 23, 117, 6, 225, 244, 34, 204, 236, 206, 150, 20,
  30, 129, 182, 70, 234, 78, 206, 174, 144, 37, 96, 132, 139, 177, 45, 166,
  84, 207, 153, 161, 91, 27, 52, 69, 37, 201, 183, 215, 195, 181, 211, 45,
  75, 144, 183, 148, 50, 181, 88, 47, 88, 164, 153, 184, 122, 3, 31, 250,
  214, 144, 212, 205, 251, 170, 15, 155, 167, 15, 122, 91, 157, 42, 245, 161,
  219, 121, 208, 236, 118, 215, 155, 221, 158, 82, 32, 60, 40, 180, 225, 191,
  214, 108, 30, 103, 12, 216, 135, 214, 210, 112, 26, 77, 66, 62, 100, 144,
  247, 26, 179, 130, 94, 123, 61, 11, 88, 8, 133, 128, 245, 146, 121, 14,
  146, 109, 132, 186, 55, 91, 4, 51, 139, 105, 106, 250, 96, 174, 183, 183,
  238, 7, 51, 26, 198, 172, 4, 230, 131, 69, 16, 255, 211, 57, 187, 30,
  165, 176, 197, 200, 138, 74, 55, 65, 231, 183, 184, 116, 219, 148, 236, 112,
  58, 110, 244, 154, 189, 110, 183, 217, 221, 88, 3, 50, 118, 137, 140, 193,
  3, 127, 13, 28, 57, 183, 18, 213, 64, 102, 94, 178, 17, 42, 175, 72,
  144, 71, 49, 107, 209, 174, 229, 158, 106, 42, 1, 248, 2, 134, 46, 227,
  103, 177, 80, 157, 38, 32, 185, 39, 253, 77, 163, 77, 216, 201, 193, 210,
  218, 194, 166, 125, 139, 139, 71, 219, 32, 181, 77, 72, 10, 88, 220, 133,
  124, 11, 96, 217, 152, 240, 49, 109, 242, 34, 163, 36, 157, 160, 208, 83,
  169, 98, 157, 48, 210, 196, 220, 161, 57, 14, 131, 34, 51, 124, 58, 141,
  106, 170, 255, 160, 215, 209, 148, 47, 217, 175, 245, 173, 178, 142, 245, 199,
  8, 13, 37, 144, 196, 171, 207, 127, 162, 246, 242, 83, 189, 5, 188, 208,
  216, 49, 80, 193, 117, 96, 109, 189, 187, 177, 97, 143, 142, 32, 181, 95,
  15, 147, 59, 173, 222, 146, 250, 150, 26, 32, 65, 238, 22, 187, 0, 216,
  89, 31, 245, 167, 66, 198, 24, 187, 80, 142, 133, 216, 139, 86, 15, 142,
  62, 54, 235, 238, 40, 128, 148, 246, 140, 87, 145, 234, 163, 184, 92, 233,
  187, 59, 26, 41, 53, 74, 226, 132, 168, 84, 70, 55, 54, 151, 86, 70,
  21, 113, 52, 170, 154, 154, 99, 107, 22, 78, 73, 241, 171, 212, 121, 182,
  184, 206, 195, 5, 237, 70, 179, 183, 214, 92, 239, 129, 120, 216, 110, 184,
  105, 15, 80, 59, 69, 96, 195, 52, 153, 129, 2, 27, 231, 168, 87, 157,
  198, 243, 180, 222, 229, 60, 2, 122, 196, 233, 121, 148, 183, 42, 11, 57,
  91, 25, 222, 206, 250, 86, 179, 187, 185, 214, 236, 110, 173, 163, 144, 239,
  53, 236, 237, 66, 23, 247, 11, 107, 29, 123, 195, 176, 222, 107, 44, 220,
  124, 96, 37, 158, 102, 237, 40, 23, 82, 120, 157, 203, 135, 171, 150, 88,
  221, 184, 158, 91, 201, 218, 38, 43, 102, 243, 83, 92, 215, 13, 45, 94,
  42, 215, 214, 230, 147, 48, 146, 156, 208, 94, 243, 207, 85, 216, 230, 199,
  113, 56, 203, 72, 179, 212, 56, 112, 155, 56, 71, 227, 165, 205, 117, 143,
  12, 224, 219, 48, 201, 166, 128, 199, 86, 5, 167, 46, 104, 222, 152, 106,
  26, 38, 29, 173, 129, 18, 232, 173, 77, 98, 4, 57, 24, 92, 51, 95,
  212, 28, 9, 169, 150, 212, 198, 138, 62, 244, 122, 75, 162, 170, 86, 14,
  83, 222, 119, 22, 85, 247, 136, 71, 11, 99, 19, 53, 197, 30, 225, 41,
  240, 247, 28, 23, 237, 104, 154, 49, 162, 140, 228, 73, 161, 25, 21, 19,
  19, 89, 55, 140, 139, 121, 57, 136, 210, 1, 44, 57, 33, 8, 39, 88,
  110, 225, 63, 49, 29, 215, 214, 105, 121, 237, 173, 161, 42, 212, 217, 240,
  204, 210, 141, 141, 134, 111, 238, 246, 26, 85, 70, 4, 115, 2, 153, 115,
  197, 93, 96, 179, 60, 101, 249, 96, 236, 89, 97, 101, 134, 87, 98, 203,
  73, 210, 213, 89, 112, 13, 134, 239, 34, 202, 162, 211, 40, 198, 4, 250,
  25, 155, 146, 218, 93, 24, 43, 215, 177, 82, 78, 217, 180, 39, 39, 187,
  102, 254, 241, 138, 217, 168, 208, 94, 249, 204, 161, 15, 46, 5, 122, 155,
  154, 142, 219, 219, 44, 211, 113, 203, 137, 173, 105, 191, 230, 190, 86, 109,
  107, 189, 114, 106, 25, 29, 71, 215, 158, 12, 41, 104, 107, 216, 94, 235,
  75, 119, 171, 97, 80, 222, 84, 63, 180, 85, 178, 232, 130, 145, 172, 41,
  54, 122, 97, 213, 110, 208, 221, 44, 134, 80, 178, 195, 154, 103, 88, 212,
  156, 51, 180, 14, 165, 116, 24, 38, 57, 97, 155, 242, 75, 154, 174, 88,
  114, 180, 158, 163, 129, 152, 22, 19, 163, 235, 107, 13, 31, 26, 225, 0,
  229, 187, 49, 247, 203, 164, 164, 226, 168, 236, 226, 172, 216, 14, 109, 233,
  219, 161, 45, 125, 90, 217, 70, 79, 85, 191, 13, 255, 180, 6, 113, 194,
  101, 252, 221, 132, 164, 9, 36, 153, 177, 233, 23, 130, 112, 240, 176, 209,
  214, 39, 155, 181, 239, 53, 182, 189, 206, 134, 159, 39, 88, 96, 201, 78,
  209, 13, 186, 1, 151, 24, 94, 49, 162, 154, 30, 178, 139, 214, 20, 54,
  207, 202, 238, 160, 172, 172, 238, 90, 171, 25, 163, 12, 5, 255, 91, 0,
  242, 34, 25, 134, 177, 46, 8, 70, 209, 21, 90, 101, 148, 212, 54, 9,
  120, 151, 105, 168, 75, 120, 157, 219, 54, 27, 5, 255, 111, 118, 28, 114,
  142, 163, 169, 173, 50, 24, 187, 125, 181, 217, 47, 235, 214, 52, 188, 112,
  182, 72, 74, 99, 44, 12, 70, 220, 212, 161, 1, 233, 118, 108, 40, 150,
  169, 79, 140, 139, 16, 42, 91, 74, 177, 90, 198, 236, 234, 130, 237, 247,
  79, 25, 76, 42, 144, 17, 122, 90, 56, 226, 6, 17, 73, 204, 149, 149,
  157, 242, 69, 181, 181, 102, 25, 20, 252, 43, 105, 177, 124, 118, 183, 31,
  52, 183, 81, 199, 92, 111, 4, 29, 79, 114, 167, 17, 108, 118, 126, 219,
  40, 209, 98, 232, 124, 174, 206, 7, 176, 114, 137, 50, 118, 13, 218, 198,
  205, 98, 105, 206, 82, 147, 232, 170, 5, 171, 222, 116, 216, 154, 36, 67,
  6, 141, 164, 172, 140, 96, 146, 56, 162, 247, 180, 212, 232, 6, 132, 52,
  26, 229, 193, 90, 38, 20, 125, 143, 225, 64, 3, 70, 204, 230, 29, 3,
  145, 35, 27, 243, 110, 108, 56, 41, 186, 13, 191, 77, 130, 240, 0, 22,
  78, 147, 73, 201, 206, 114, 109, 88, 111, 1, 3, 53, 81, 27, 108, 2,
  213, 53, 120, 193, 6, 217, 24, 74, 106, 97, 165, 45, 163, 14, 106, 66,
  59, 183, 65, 158, 84, 52, 133, 173, 216, 45, 105, 214, 9, 60, 167, 117,
  100, 101, 197, 206, 192, 58, 53, 193, 234, 109, 181, 90, 152, 19, 79, 147,
  184, 211, 121, 235, 52, 183, 207, 134, 74, 78, 114, 202, 21, 8, 5, 240,
  50, 26, 69, 45, 101, 201, 189, 131, 77, 197, 84, 129, 77, 203, 192, 76,
  216, 146, 10, 174, 226, 41, 221, 197, 102, 46, 141, 1, 102, 119, 182, 72,
  173, 87, 91, 164, 58, 95, 223, 34, 197, 5, 83, 249, 10, 121, 49, 27,
  186, 90, 189, 99, 253, 21, 134, 107, 159, 10, 237, 136, 64, 159, 234, 183,
  196, 56, 19, 34, 114, 113, 173, 218, 95, 24, 53, 176, 117, 232, 213, 162,
  45, 137, 79, 32, 45, 101, 130, 88, 96, 95, 211, 22, 137, 157, 242, 221,
  175, 125, 144, 67, 152, 199, 236, 12, 132, 97, 249, 10, 230, 227, 109, 123,
  85, 211, 78, 183, 17, 88, 43, 187, 12, 97, 107, 82, 40, 39, 186, 18,
  223, 245, 140, 224, 186, 199, 94, 96, 43, 63, 136, 43, 76, 165, 115, 18,
  146, 119, 50, 128, 243, 49, 239, 21, 99, 46, 79, 132, 29, 46, 86, 154,
  42, 90, 192, 55, 113, 71, 135, 147, 101, 109, 163, 97, 114, 211, 232, 244,
  116, 212, 91, 183, 112, 155, 38, 173, 97, 152, 135, 11, 56, 224, 238, 150,
  84, 153, 110, 105, 35, 182, 217, 135, 91, 125, 184, 6, 110, 41, 66, 218,
  214, 116, 211, 52, 232, 107, 167, 185, 226, 56, 76, 199, 136, 70, 25, 36,
  94, 154, 187, 117, 134, 44, 27, 20, 186, 160, 68, 202, 229, 60, 187, 158,
  117, 156, 43, 78, 115, 189, 71, 184, 246, 9, 174, 60, 192, 69, 251, 95,
  83, 156, 223, 122, 86, 7, 209, 144, 176, 34, 223, 109, 27, 94, 106, 50,
  80, 2, 100, 105, 123, 33, 177, 188, 176, 97, 43, 19, 182, 110, 175, 214,
  246, 111, 189, 37, 140, 208, 190, 254, 221, 195, 152, 236, 218, 146, 5, 188,
  251, 172, 108, 198, 112, 111, 248, 236, 108, 37, 173, 120, 79, 179, 58, 142,
  144, 176, 107, 135, 68, 230, 108, 121, 109, 187, 216, 240, 133, 103, 11, 125,
  92, 214, 126, 57, 31, 151, 111, 105, 101, 91, 184, 247, 169, 218, 193, 220,
  127, 95, 36, 247, 62, 91, 29, 63, 66, 237, 9, 254, 105, 185, 190, 40,
  75, 172, 154, 214, 36, 90, 55, 38, 209, 102, 97, 198, 217, 166, 13, 169,
  178, 237, 110, 244, 58, 150, 143, 138, 80, 58, 122, 142, 153, 100, 93, 247,
  97, 90, 56, 235, 76, 89, 240, 45, 231, 155, 119, 209, 103, 203, 95, 229,
  151, 219, 116, 154, 198, 193, 18, 49, 205, 231, 165, 62, 46, 190, 237, 205,
  186, 118, 2, 85, 108, 143, 188, 125, 43, 211, 134, 181, 189, 132, 159, 38,
  127, 151, 177, 239, 25, 99, 191, 185, 238, 25, 123, 180, 19, 173, 245, 236,
  177, 223, 104, 248, 182, 131, 219, 235, 158, 13, 35, 167, 157, 38, 83, 55,
  61, 135, 125, 155, 174, 56, 245, 146, 209, 164, 138, 111, 23, 230, 33, 172,
  168, 148, 195, 162, 95, 46, 160, 124, 133, 213, 174, 159, 27, 100, 52, 34,
  118, 236, 85, 200, 36, 253, 29, 60, 212, 220, 246, 10, 150, 209, 26, 40,
  241, 210, 145, 78, 58, 182, 143, 142, 197, 36, 98, 137, 49, 124, 233, 44,
  4, 200, 196, 229, 87, 145, 10, 147, 72, 170, 235, 115, 37, 62, 122, 254,
  253, 155, 216, 73, 42, 81, 45, 253, 71, 237, 117, 88, 219, 212, 225, 184,
  195, 178, 196, 102, 119, 218, 144, 154, 203, 147, 6, 101, 225, 190, 116, 22,
  197, 177, 230, 43, 88, 237, 88, 185, 196, 18, 227, 177, 91, 121, 13, 218,
  74, 39, 241, 73, 55, 211, 1, 167, 5, 154, 88, 126, 253, 247, 84, 100,
  203, 180, 85, 93, 89, 13, 175, 162, 172, 37, 253, 235, 74, 183, 56, 142,
  103, 14, 109, 219, 79, 195, 212, 63, 41, 215, 109, 138, 208, 160, 195, 22,
  166, 208, 68, 54, 173, 161, 230, 224, 148, 218, 178, 110, 143, 147, 178, 155,
  27, 3, 218, 243, 66, 241, 78, 66, 152, 109, 131, 141, 13, 93, 229, 3,
  22, 76, 166, 83, 224, 67, 117, 204, 236, 152, 17, 45, 153, 81, 200, 135,
  97, 152, 141, 217, 48, 40, 65, 203, 17, 49, 222, 69, 172, 96, 143, 56,
  153, 15, 191, 134, 14, 191, 110, 235, 240, 230, 132, 226, 237, 56, 42, 233,
  61, 90, 2, 153, 111, 182, 180, 101, 185, 75, 110, 121, 26, 22, 115, 244,
  139, 150, 67, 99, 84, 182, 220, 51, 190, 5, 27, 250, 234, 89, 74, 88,
  14, 82, 54, 132, 156, 40, 140, 51, 31, 71, 220, 115, 231, 211, 41, 219,
  249, 148, 183, 254, 75, 248, 9, 85, 180, 150, 39, 103, 103, 180, 183, 243,
  89, 201, 149, 60, 173, 216, 138, 186, 32, 179, 249, 4, 232, 119, 237, 44,
  1, 95, 193, 211, 73, 29, 23, 44, 244, 73, 168, 192, 79, 156, 252, 87,
  57, 34, 117, 133, 171, 163, 109, 62, 118, 29, 112, 58, 203, 105, 70, 46,
  26, 250, 201, 85, 41, 142, 165, 222, 9, 94, 219, 215, 125, 27, 115, 7,
  204, 90, 102, 89, 154, 194, 134, 241, 52, 4, 177, 153, 26, 14, 165, 165,
  87, 82, 182, 59, 150, 27, 244, 131, 81, 119, 216, 29, 22, 78, 159, 35,
  54, 24, 14, 215, 52, 95, 151, 226, 68, 70, 204, 40, 97, 226, 213, 29,
  172, 79, 183, 187, 131, 238, 64, 187, 12, 224, 50, 150, 113, 243, 196, 231,
  124, 106, 177, 175, 209, 181, 121, 108, 94, 158, 145, 78, 40, 116, 170, 30,
  116, 205, 185, 100, 212, 84, 106, 167, 222, 105, 223, 61, 8, 189, 59, 163,
  65, 184, 17, 110, 184, 52, 177, 156, 130, 54, 13, 159, 32, 53, 39, 29,
  151, 62, 237, 64, 118, 194, 196, 21, 30, 71, 144, 185, 171, 54, 79, 191,
  147, 97, 50, 255, 220, 226, 14, 245, 150, 220, 192, 67, 82, 105, 164, 236,
  88, 30, 144, 128, 210, 178, 254, 68, 133, 165, 34, 109, 37, 211, 184, 68,
  121, 18, 13, 233, 118, 14, 203, 109, 64, 140, 100, 171, 235, 181, 104, 15,
  226, 104, 214, 71, 125, 84, 238, 144, 26, 166, 75, 245, 52, 225, 189, 22,
  3, 215, 49, 221, 211, 253, 38, 248, 224, 155, 104, 130, 119, 230, 66, 67,
  91, 231, 122, 96, 133, 105, 152, 15, 129, 77, 110, 239, 144, 44, 77, 59,
  97, 60, 198, 234, 174, 177, 166, 218, 28, 101, 48, 220, 242, 58, 245, 29,
  142, 5, 196, 13, 191, 5, 60, 180, 209, 177, 46, 123, 81, 143, 172, 186,
  90, 121, 195, 123, 20, 123, 17, 152, 51, 246, 124, 22, 17, 125, 23, 94,
  109, 147, 196, 208, 189, 1, 12, 40, 92, 103, 93, 70, 147, 42, 189, 58,
  179, 80, 151, 226, 178, 203, 235, 189, 164, 102, 131, 46, 127, 53, 241, 219,
  237, 250, 61, 77, 244, 49, 237, 232, 102, 4, 225, 6, 90, 218, 95, 193,
  69, 95, 79, 135, 243, 9, 163, 123, 40, 113, 197, 228, 89, 194, 81, 134,
  252, 239, 53, 63, 142, 141, 77, 135, 59, 132, 132, 90, 218, 66, 182, 238,
  64, 88, 184, 167, 114, 38, 42, 214, 114, 47, 76, 117, 219, 221, 242, 251,
  82, 230, 117, 169, 251, 122, 96, 169, 169, 110, 146, 32, 229, 82, 170, 4,
  21, 113, 223, 230, 65, 231, 203, 90, 162, 86, 90, 243, 89, 113, 234, 225,
  236, 211, 120, 17, 24, 208, 169, 230, 38, 99, 223, 206, 225, 133, 70, 48,
  229, 138, 66, 108, 131, 61, 96, 167, 78, 33, 113, 183, 209, 114, 184, 177,
  10, 157, 37, 201, 112, 33, 78, 167, 161, 86, 70, 94, 34, 181, 164, 125,
  158, 36, 113, 30, 205, 60, 118, 82, 175, 234, 86, 177, 105, 220, 238, 45,
  235, 122, 188, 209, 112, 152, 193, 240, 119, 241, 157, 246, 150, 152, 142, 93,
  119, 101, 58, 183, 115, 185, 218, 239, 48, 189, 185, 132, 83, 245, 166, 97,
  183, 237, 246, 12, 129, 212, 91, 183, 246, 176, 58, 81, 23, 30, 127, 155,
  67, 208, 246, 205, 201, 173, 234, 77, 131, 1, 0, 228, 110, 50, 61, 83,
  67, 30, 77, 199, 48, 11, 115, 207, 132, 80, 222, 4, 19, 54, 140, 194,
  160, 62, 75, 217, 136, 165, 89, 11, 116, 237, 249, 128, 161, 143, 140, 188,
  135, 133, 223, 141, 224, 70, 148, 175, 186, 14, 212, 172, 184, 214, 211, 92,
  246, 122, 142, 73, 31, 254, 247, 225, 170, 186, 124, 255, 112, 85, 198, 7,
  120, 136, 27, 15, 113, 55, 159, 111, 68, 229, 229, 252, 135, 195, 232, 2,
  116, 166, 48, 203, 118, 87, 138, 91, 226, 43, 123, 170, 11, 88, 160, 248,
  194, 250, 93, 253, 246, 255, 223, 254, 253, 63, 84, 4, 0, 200, 209, 11,
  234, 144, 149, 23, 217, 202, 222, 17, 26, 115, 94, 36, 211, 40, 79, 82,
  224, 225, 135, 171, 70, 3, 246, 167, 31, 189, 128, 250, 184, 187, 162, 22,
  138, 149, 178, 150, 209, 40, 100, 100, 98, 116, 130, 89, 56, 149, 249, 49,
  27, 174, 4, 209, 144, 23, 60, 130, 143, 61, 32, 32, 228, 251, 170, 200,
  98, 39, 176, 156, 172, 236, 37, 163, 17, 202, 199, 242, 226, 162, 5, 186,
  130, 205, 219, 64, 147, 214, 139, 4, 131, 55, 188, 42, 171, 108, 245, 31,
  135, 142, 239, 65, 176, 62, 250, 79, 62, 101, 23, 43, 18, 52, 93, 210,
  86, 196, 208, 212, 173, 149, 61, 40, 214, 130, 166, 230, 217, 195, 85, 14,
  192, 128, 169, 186, 51, 100, 23, 199, 196, 106, 43, 6, 186, 129, 186, 111,
  174, 160, 27, 115, 146, 224, 7, 225, 121, 30, 93, 216, 93, 48, 58, 96,
  126, 104, 195, 34, 247, 50, 58, 163, 241, 249, 60, 74, 82, 158, 253, 25,
  26, 58, 38, 117, 208, 223, 195, 63, 177, 40, 199, 50, 15, 87, 169, 162,
  6, 72, 40, 145, 216, 63, 27, 144, 108, 94, 238, 116, 76, 190, 73, 102,
  116, 69, 147, 150, 237, 221, 149, 131, 57, 8, 55, 182, 250, 132, 165, 48,
  84, 43, 123, 198, 231, 195, 85, 94, 182, 162, 250, 143, 39, 251, 43, 123,
  240, 207, 18, 69, 5, 232, 163, 100, 58, 76, 138, 150, 248, 231, 18, 213,
  31, 147, 250, 16, 174, 190, 100, 151, 31, 127, 74, 210, 243, 149, 61, 59,
  101, 25, 32, 160, 166, 165, 176, 202, 135, 171, 199, 215, 195, 41, 187, 6,
  32, 86, 202, 50, 64, 50, 40, 125, 146, 156, 95, 39, 80, 93, 253, 118,
  43, 2, 211, 16, 249, 245, 65, 179, 167, 76, 80, 196, 46, 88, 81, 99,
  249, 132, 38, 19, 141, 125, 48, 141, 6, 227, 60, 200, 174, 167, 131, 113,
  138, 116, 178, 248, 80, 7, 168, 246, 169, 28, 82, 156, 12, 194, 248, 4,
  210, 248, 76, 6, 25, 102, 214, 46, 101, 91, 185, 213, 88, 241, 75, 40,
  177, 135, 224, 141, 192, 199, 19, 248, 93, 38, 152, 164, 254, 189, 18, 160,
  71, 76, 139, 59, 63, 3, 166, 176, 211, 176, 229, 149, 85, 139, 250, 97,
  22, 113, 11, 209, 156, 88, 217, 59, 1, 112, 65, 253, 127, 252, 183, 253,
  134, 35, 88, 124, 181, 104, 24, 65, 4, 42, 249, 0, 137, 39, 132, 81,
  65, 35, 131, 176, 74, 181, 92, 209, 139, 159, 80, 138, 94, 199, 109, 220,
  21, 116, 78, 194, 98, 106, 141, 231, 147, 104, 24, 229, 215, 95, 145, 98,
  207, 5, 200, 160, 254, 219, 251, 211, 236, 185, 194, 107, 105, 186, 201, 42,
  127, 55, 218, 13, 146, 222, 87, 36, 219, 254, 171, 191, 253, 151, 255, 2,
  170, 209, 108, 114, 127, 170, 237, 35, 70, 75, 19, 12, 74, 255, 221, 104,
  117, 49, 27, 126, 69, 90, 189, 125, 253, 52, 168, 159, 191, 14, 239, 79,
  169, 183, 179, 225, 29, 40, 5, 165, 191, 6, 165, 42, 86, 246, 105, 168,
  151, 211, 52, 22, 200, 120, 26, 102, 227, 211, 36, 76, 135, 106, 245, 149,
  110, 199, 160, 69, 200, 60, 87, 75, 177, 160, 112, 117, 51, 243, 192, 224,
  57, 108, 186, 16, 196, 62, 90, 200, 61, 0, 40, 221, 174, 253, 112, 85,
  245, 137, 171, 209, 82, 101, 166, 225, 65, 136, 100, 40, 126, 66, 118, 98,
  5, 84, 55, 30, 43, 110, 120, 200, 55, 26, 123, 207, 216, 56, 102, 41,
  234, 231, 244, 41, 115, 231, 113, 1, 238, 40, 2, 125, 14, 198, 102, 174,
  212, 25, 189, 15, 131, 152, 133, 233, 1, 150, 203, 160, 219, 131, 113, 28,
  177, 159, 255, 47, 187, 227, 218, 192, 60, 196, 152, 59, 198, 50, 134, 80,
  208, 51, 188, 53, 116, 6, 133, 252, 205, 249, 81, 235, 138, 161, 75, 145,
  121, 68, 105, 156, 120, 172, 201, 103, 69, 166, 23, 131, 125, 85, 132, 247,
  3, 165, 214, 141, 94, 33, 250, 85, 117, 205, 139, 210, 154, 90, 241, 252,
  106, 165, 122, 219, 64, 187, 54, 160, 138, 112, 56, 54, 243, 40, 205, 3,
  146, 27, 189, 118, 87, 214, 122, 157, 21, 97, 72, 222, 93, 233, 174, 119,
  144, 186, 188, 210, 94, 165, 32, 208, 111, 143, 87, 76, 125, 61, 166, 133,
  118, 169, 179, 82, 18, 20, 113, 7, 0, 25, 68, 225, 136, 20, 153, 250,
  209, 252, 74, 72, 4, 99, 62, 23, 91, 65, 161, 188, 204, 175, 158, 38,
  185, 218, 167, 44, 51, 145, 61, 237, 227, 134, 176, 18, 75, 237, 202, 182,
  83, 206, 83, 82, 225, 38, 132, 140, 79, 176, 221, 71, 228, 8, 198, 250,
  42, 172, 54, 155, 141, 134, 95, 155, 215, 8, 230, 63, 19, 179, 189, 126,
  253, 12, 86, 159, 255, 241, 255, 76, 146, 120, 117, 242, 63, 254, 239, 213,
  108, 25, 158, 195, 94, 254, 106, 153, 142, 15, 107, 25, 215, 121, 208, 16,
  119, 224, 65, 134, 206, 216, 121, 158, 206, 39, 125, 109, 71, 140, 208, 32,
  125, 128, 233, 198, 130, 121, 154, 174, 238, 61, 131, 45, 47, 30, 206, 153,
  197, 159, 129, 216, 76, 210, 5, 171, 235, 175, 128, 255, 109, 141, 239, 43,
  176, 63, 130, 252, 103, 226, 126, 71, 77, 173, 100, 123, 232, 221, 175, 150,
  235, 7, 74, 89, 254, 53, 178, 154, 179, 141, 253, 10, 188, 70, 48, 255,
  153, 152, 237, 199, 201, 25, 59, 157, 79, 207, 48, 232, 232, 12, 227, 127,
  205, 83, 125, 47, 94, 201, 122, 88, 229, 87, 203, 123, 121, 97, 18, 248,
  53, 50, 159, 215, 42, 240, 21, 24, 80, 193, 253, 167, 82, 46, 231, 163,
  124, 196, 230, 160, 97, 178, 194, 164, 81, 201, 122, 178, 155, 191, 90, 246,
  27, 155, 214, 149, 95, 35, 11, 194, 78, 109, 244, 213, 183, 53, 8, 243,
  159, 138, 245, 0, 225, 150, 109, 129, 172, 222, 219, 64, 141, 95, 239, 230,
  134, 198, 244, 87, 203, 114, 182, 141, 234, 43, 112, 28, 130, 252, 103, 98,
  56, 203, 176, 86, 201, 106, 208, 183, 95, 45, 167, 93, 204, 22, 111, 104,
  68, 57, 235, 244, 206, 220, 223, 44, 203, 168, 22, 34, 242, 150, 185, 66,
  230, 4, 160, 238, 115, 230, 113, 145, 17, 220, 164, 23, 21, 188, 164, 195,
  227, 73, 5, 207, 84, 147, 162, 184, 44, 238, 105, 209, 36, 0, 54, 120,
  68, 46, 100, 70, 131, 220, 171, 172, 132, 4, 10, 134, 117, 182, 121, 95,
  135, 37, 47, 146, 46, 154, 39, 97, 122, 198, 242, 187, 161, 233, 66, 145,
  35, 46, 112, 215, 189, 21, 48, 90, 110, 85, 151, 75, 216, 105, 49, 151,
  97, 187, 47, 147, 167, 32, 27, 12, 236, 197, 117, 238, 178, 51, 226, 115,
  22, 77, 89, 0, 181, 208, 70, 249, 11, 137, 76, 60, 73, 28, 240, 195,
  197, 82, 163, 37, 23, 150, 216, 149, 240, 226, 108, 31, 63, 52, 128, 227,
  53, 137, 189, 230, 164, 7, 210, 237, 247, 12, 212, 246, 116, 26, 252, 252,
  127, 6, 245, 222, 250, 24, 68, 202, 120, 205, 127, 234, 135, 54, 81, 71,
  242, 238, 73, 171, 239, 209, 252, 170, 175, 108, 190, 154, 225, 0, 80, 57,
  82, 214, 50, 191, 8, 50, 224, 208, 206, 181, 12, 146, 125, 114, 82, 9,
  9, 87, 228, 50, 64, 206, 97, 95, 37, 164, 103, 92, 175, 44, 3, 246,
  220, 178, 160, 84, 194, 2, 217, 93, 6, 199, 62, 237, 168, 60, 150, 88,
  146, 29, 252, 67, 169, 173, 66, 138, 169, 29, 207, 99, 35, 74, 73, 165,
  251, 162, 201, 21, 54, 167, 245, 113, 178, 190, 69, 41, 55, 31, 153, 220,
  101, 58, 67, 240, 107, 200, 180, 36, 91, 254, 16, 203, 93, 67, 247, 223,
  172, 176, 100, 22, 193, 63, 183, 188, 88, 10, 63, 138, 114, 28, 170, 156,
  124, 221, 149, 209, 116, 23, 224, 251, 200, 19, 181, 59, 246, 121, 24, 120,
  170, 21, 250, 191, 182, 185, 89, 178, 46, 45, 173, 192, 107, 75, 22, 39,
  43, 11, 77, 189, 37, 43, 144, 5, 28, 38, 182, 191, 184, 235, 247, 128,
  105, 182, 255, 74, 185, 11, 148, 238, 251, 116, 79, 191, 218, 170, 136, 91,
  90, 196, 97, 87, 149, 68, 54, 192, 3, 37, 82, 4, 172, 21, 87, 119,
  133, 247, 44, 64, 14, 55, 191, 9, 167, 103, 236, 151, 99, 102, 2, 95,
  205, 203, 6, 6, 202, 87, 172, 112, 65, 95, 196, 186, 176, 36, 172, 236,
  193, 63, 75, 178, 197, 90, 71, 35, 22, 221, 212, 73, 17, 129, 149, 189,
  238, 139, 37, 33, 108, 151, 65, 88, 91, 22, 2, 94, 84, 246, 131, 88,
  127, 241, 5, 236, 234, 25, 222, 125, 36, 228, 47, 55, 188, 207, 194, 244,
  116, 193, 240, 26, 24, 120, 135, 247, 222, 83, 145, 159, 196, 2, 245, 104,
  34, 188, 76, 64, 29, 97, 43, 230, 46, 35, 96, 105, 234, 87, 138, 22,
  184, 187, 107, 42, 200, 186, 51, 9, 11, 31, 68, 171, 117, 238, 197, 68,
  199, 216, 129, 240, 74, 12, 254, 246, 239, 255, 91, 240, 25, 180, 66, 22,
  196, 201, 121, 8, 27, 71, 226, 85, 199, 219, 208, 56, 93, 134, 250, 111,
  64, 196, 95, 47, 246, 47, 52, 110, 99, 232, 168, 211, 154, 70, 64, 60,
  199, 241, 229, 114, 205, 218, 153, 170, 97, 44, 219, 100, 46, 179, 204, 91,
  106, 153, 111, 103, 93, 189, 62, 75, 77, 80, 158, 18, 5, 151, 63, 255,
  101, 28, 163, 46, 91, 185, 86, 79, 105, 171, 119, 116, 240, 52, 248, 67,
  56, 13, 99, 223, 124, 49, 185, 117, 234, 238, 13, 173, 153, 59, 154, 199,
  241, 199, 76, 29, 74, 189, 77, 226, 88, 126, 149, 76, 92, 179, 62, 221,
  127, 90, 217, 123, 135, 127, 150, 170, 112, 26, 39, 9, 52, 244, 36, 254,
  249, 175, 57, 59, 231, 221, 240, 56, 255, 249, 166, 144, 198, 81, 89, 120,
  129, 219, 70, 222, 63, 160, 98, 52, 24, 131, 70, 237, 117, 72, 213, 72,
  200, 55, 181, 232, 50, 235, 142, 134, 25, 8, 152, 199, 166, 89, 217, 123,
  61, 138, 195, 233, 103, 54, 133, 201, 55, 140, 96, 156, 234, 176, 190, 55,
  22, 208, 93, 181, 82, 73, 135, 140, 177, 33, 76, 167, 51, 64, 63, 103,
  131, 115, 252, 25, 172, 6, 199, 63, 255, 101, 66, 63, 235, 221, 224, 93,
  2, 157, 106, 44, 69, 212, 11, 118, 134, 250, 222, 89, 148, 135, 228, 55,
  123, 215, 113, 152, 141, 195, 108, 185, 225, 195, 171, 74, 31, 69, 221, 227,
  217, 207, 127, 201, 217, 41, 65, 184, 207, 32, 134, 179, 89, 124, 93, 58,
  32, 91, 36, 165, 30, 99, 25, 124, 124, 14, 86, 19, 191, 195, 241, 204,
  32, 186, 101, 180, 200, 204, 29, 173, 117, 41, 24, 101, 192, 204, 15, 76,
  110, 168, 253, 120, 201, 107, 0, 225, 198, 233, 250, 104, 224, 3, 227, 241,
  78, 127, 78, 209, 162, 126, 254, 235, 41, 252, 139, 219, 215, 20, 102, 49,
  140, 48, 240, 230, 207, 127, 77, 3, 188, 158, 176, 57, 110, 189, 136, 166,
  81, 235, 247, 160, 57, 141, 219, 6, 80, 109, 119, 250, 155, 10, 169, 163,
  124, 193, 247, 151, 150, 65, 239, 162, 214, 179, 40, 56, 102, 249, 124, 230,
  136, 31, 185, 40, 17, 68, 30, 3, 129, 13, 159, 224, 220, 80, 20, 230,
  55, 43, 42, 204, 111, 90, 224, 70, 87, 243, 177, 252, 229, 121, 40, 69,
  69, 117, 35, 174, 3, 127, 140, 167, 232, 33, 189, 3, 226, 247, 169, 47,
  28, 163, 96, 15, 116, 10, 213, 81, 178, 154, 190, 81, 75, 217, 225, 116,
  157, 184, 106, 253, 116, 118, 156, 199, 199, 135, 254, 45, 39, 34, 126, 156,
  69, 195, 197, 39, 250, 58, 184, 195, 215, 165, 192, 14, 103, 119, 117, 189,
  91, 186, 139, 75, 153, 163, 204, 110, 67, 249, 48, 46, 112, 93, 104, 15,
  84, 209, 65, 138, 97, 125, 130, 95, 203, 118, 68, 19, 38, 60, 44, 193,
  59, 128, 240, 44, 73, 39, 75, 232, 25, 142, 168, 121, 129, 142, 223, 108,
  206, 38, 193, 75, 150, 127, 190, 100, 233, 121, 0, 19, 246, 52, 226, 236,
  227, 145, 60, 94, 219, 180, 236, 199, 27, 25, 52, 228, 53, 25, 104, 205,
  233, 18, 88, 33, 69, 254, 14, 211, 103, 235, 116, 56, 218, 218, 89, 102,
  190, 96, 135, 219, 237, 246, 242, 51, 102, 230, 118, 154, 244, 70, 159, 16,
  60, 134, 29, 53, 11, 0, 171, 75, 126, 220, 208, 38, 135, 121, 75, 126,
  150, 179, 168, 84, 228, 93, 82, 104, 172, 160, 136, 251, 106, 6, 146, 105,
  137, 27, 45, 54, 58, 193, 207, 255, 125, 52, 154, 250, 71, 189, 180, 49,
  161, 223, 250, 224, 31, 164, 192, 87, 57, 114, 83, 134, 221, 47, 129, 235,
  39, 173, 75, 67, 151, 218, 207, 194, 56, 198, 193, 94, 217, 195, 95, 89,
  192, 173, 163, 98, 44, 231, 24, 3, 64, 50, 50, 40, 56, 131, 113, 48,
  1, 78, 31, 2, 159, 147, 208, 111, 61, 126, 173, 134, 191, 184, 125, 213,
  18, 11, 130, 200, 104, 219, 43, 92, 37, 239, 211, 12, 44, 59, 181, 49,
  175, 230, 216, 246, 114, 10, 194, 213, 115, 135, 87, 211, 226, 50, 146, 159,
  40, 98, 61, 138, 152, 173, 140, 241, 194, 62, 77, 100, 161, 68, 201, 96,
  123, 128, 242, 100, 197, 192, 173, 107, 190, 246, 215, 227, 178, 67, 74, 12,
  22, 148, 143, 112, 165, 237, 98, 6, 180, 1, 117, 19, 254, 189, 76, 210,
  220, 167, 97, 210, 187, 132, 220, 49, 12, 203, 6, 248, 130, 47, 255, 13,
  53, 96, 229, 7, 189, 117, 192, 198, 73, 60, 100, 0, 238, 221, 209, 227,
  151, 129, 132, 166, 94, 216, 93, 106, 48, 56, 82, 86, 151, 221, 8, 4,
  155, 75, 175, 16, 28, 113, 142, 46, 208, 102, 112, 126, 154, 92, 113, 38,
  70, 177, 22, 13, 14, 103, 39, 36, 187, 253, 115, 39, 56, 166, 66, 193,
  225, 107, 107, 236, 92, 10, 85, 157, 21, 98, 112, 11, 161, 170, 24, 77,
  191, 113, 9, 80, 16, 58, 154, 89, 100, 61, 124, 29, 212, 185, 158, 27,
  198, 13, 139, 172, 70, 205, 179, 75, 171, 230, 239, 65, 111, 190, 12, 175,
  151, 172, 158, 77, 173, 234, 199, 243, 211, 41, 203, 203, 107, 87, 222, 237,
  195, 205, 18, 49, 242, 221, 151, 56, 67, 208, 101, 44, 55, 230, 131, 181,
  130, 122, 30, 202, 149, 87, 167, 71, 35, 24, 201, 119, 208, 92, 6, 146,
  41, 203, 89, 28, 131, 84, 42, 105, 178, 88, 77, 150, 214, 229, 183, 188,
  186, 252, 242, 199, 59, 70, 5, 175, 83, 123, 102, 221, 17, 192, 196, 10,
  95, 246, 59, 88, 0, 212, 21, 3, 143, 234, 45, 59, 94, 196, 238, 21,
  103, 77, 228, 75, 47, 239, 45, 120, 140, 162, 227, 117, 235, 60, 128, 30,
  185, 163, 48, 7, 252, 96, 0, 182, 28, 103, 160, 119, 177, 64, 107, 127,
  125, 249, 246, 47, 194, 40, 198, 55, 63, 43, 80, 208, 100, 186, 189, 203,
  94, 41, 151, 185, 225, 112, 200, 97, 62, 201, 167, 126, 121, 240, 131, 64,
  25, 102, 243, 244, 243, 28, 251, 177, 148, 180, 189, 211, 104, 15, 140, 235,
  28, 95, 109, 172, 185, 25, 173, 254, 18, 116, 35, 106, 33, 120, 199, 78,
  159, 62, 126, 219, 12, 158, 159, 156, 188, 94, 197, 127, 142, 27, 85, 124,
  80, 132, 127, 243, 44, 160, 37, 43, 166, 207, 68, 234, 145, 218, 210, 124,
  15, 35, 89, 46, 174, 169, 253, 131, 41, 61, 246, 90, 34, 172, 121, 23,
  143, 64, 158, 163, 17, 131, 110, 241, 70, 140, 216, 203, 191, 82, 59, 189,
  211, 162, 76, 105, 141, 238, 107, 169, 213, 231, 251, 101, 209, 209, 252, 135,
  253, 242, 32, 53, 193, 81, 18, 167, 201, 62, 173, 183, 196, 164, 169, 161,
  37, 151, 48, 93, 217, 12, 202, 130, 167, 249, 137, 247, 183, 255, 253, 175,
  101, 10, 103, 245, 73, 122, 105, 215, 69, 152, 46, 63, 33, 143, 69, 166,
  159, 48, 166, 65, 216, 169, 197, 237, 194, 7, 48, 200, 44, 134, 46, 229,
  254, 93, 133, 151, 104, 7, 195, 40, 95, 70, 41, 127, 194, 208, 22, 31,
  229, 229, 106, 248, 61, 169, 66, 142, 53, 222, 206, 61, 241, 185, 220, 56,
  118, 88, 172, 244, 99, 26, 227, 130, 134, 211, 55, 120, 18, 102, 44, 248,
  241, 205, 81, 9, 143, 91, 75, 187, 170, 110, 46, 240, 227, 60, 159, 245,
  87, 87, 145, 28, 171, 41, 155, 36, 57, 107, 207, 198, 179, 213, 97, 120,
  177, 58, 138, 98, 150, 173, 206, 51, 150, 174, 58, 58, 67, 9, 122, 175,
  211, 36, 79, 96, 241, 5, 141, 18, 127, 157, 39, 113, 92, 133, 157, 110,
  52, 54, 171, 251, 221, 70, 172, 51, 83, 64, 125, 101, 15, 255, 13, 234,
  176, 100, 79, 135, 32, 16, 27, 101, 7, 71, 165, 16, 50, 14, 34, 171,
  170, 232, 87, 227, 171, 182, 74, 230, 42, 72, 81, 140, 2, 46, 231, 158,
  71, 211, 75, 22, 101, 253, 128, 90, 13, 96, 73, 164, 48, 39, 121, 48,
  154, 79, 207, 17, 1, 146, 91, 67, 150, 5, 127, 130, 84, 208, 70, 206,
  67, 220, 52, 225, 171, 202, 82, 124, 215, 143, 89, 60, 106, 161, 209, 3,
  182, 222, 209, 36, 64, 157, 59, 57, 111, 6, 33, 90, 248, 176, 232, 193,
  241, 235, 96, 150, 254, 252, 215, 17, 104, 93, 115, 50, 250, 157, 177, 108,
  48, 78, 127, 254, 11, 52, 209, 104, 59, 187, 222, 50, 102, 203, 80, 126,
  225, 191, 211, 112, 194, 238, 194, 102, 88, 209, 228, 179, 2, 123, 9, 111,
  105, 158, 162, 29, 202, 227, 217, 172, 37, 247, 21, 193, 106, 112, 146, 156,
  151, 10, 119, 31, 70, 175, 23, 111, 93, 244, 22, 150, 197, 13, 246, 222,
  56, 133, 49, 194, 129, 250, 25, 212, 241, 181, 236, 156, 53, 238, 194, 249,
  26, 160, 101, 24, 183, 187, 178, 215, 165, 71, 185, 243, 59, 241, 59, 236,
  111, 123, 188, 26, 187, 83, 189, 181, 149, 189, 181, 251, 212, 91, 95, 217,
  91, 95, 162, 94, 229, 244, 186, 151, 58, 97, 7, 19, 92, 172, 94, 188,
  102, 105, 22, 149, 45, 9, 129, 182, 80, 7, 195, 112, 206, 210, 113, 8,
  115, 43, 43, 206, 127, 202, 71, 122, 177, 205, 179, 235, 245, 43, 41, 89,
  197, 142, 67, 188, 204, 90, 121, 242, 84, 81, 251, 132, 101, 246, 26, 136,
  59, 129, 33, 11, 48, 167, 10, 88, 185, 7, 158, 221, 198, 179, 196, 112,
  9, 52, 87, 90, 231, 32, 132, 151, 14, 180, 117, 242, 142, 171, 238, 66,
  91, 239, 29, 169, 238, 82, 60, 167, 83, 91, 250, 19, 160, 233, 43, 29,
  82, 140, 159, 82, 44, 93, 0, 201, 204, 33, 58, 164, 45, 6, 230, 239,
  219, 204, 192, 13, 182, 166, 47, 178, 179, 251, 110, 86, 75, 237, 244, 158,
  86, 75, 44, 130, 66, 190, 230, 227, 231, 101, 43, 31, 52, 251, 227, 44,
  78, 194, 97, 22, 196, 33, 110, 249, 131, 104, 26, 172, 22, 54, 191, 213,
  223, 197, 249, 206, 144, 93, 68, 3, 118, 56, 252, 221, 89, 190, 179, 234,
  89, 161, 92, 221, 170, 196, 56, 237, 43, 138, 145, 157, 69, 188, 1, 177,
  27, 98, 124, 59, 161, 162, 13, 240, 192, 89, 246, 110, 3, 233, 203, 228,
  65, 71, 197, 217, 194, 162, 54, 223, 204, 167, 24, 175, 165, 172, 53, 145,
  253, 213, 90, 43, 248, 170, 164, 61, 89, 224, 107, 181, 168, 206, 232, 202,
  90, 84, 5, 190, 66, 99, 255, 211, 156, 205, 75, 73, 73, 153, 199, 209,
  103, 232, 85, 231, 203, 154, 121, 22, 70, 241, 60, 197, 195, 95, 127, 75,
  50, 127, 97, 67, 11, 37, 148, 193, 206, 165, 51, 247, 142, 232, 31, 133,
  176, 17, 156, 211, 180, 43, 235, 1, 22, 225, 19, 243, 43, 140, 10, 181,
  151, 211, 42, 82, 222, 26, 173, 63, 95, 208, 150, 58, 142, 193, 8, 166,
  60, 188, 101, 63, 160, 61, 99, 111, 199, 64, 100, 6, 242, 168, 12, 17,
  222, 101, 148, 88, 10, 30, 222, 45, 104, 145, 62, 209, 167, 87, 40, 118,
  126, 105, 36, 105, 226, 149, 33, 72, 153, 111, 88, 152, 37, 211, 127, 28,
  134, 68, 70, 138, 224, 81, 53, 160, 20, 186, 227, 139, 144, 44, 91, 227,
  22, 109, 170, 180, 153, 113, 48, 153, 141, 18, 116, 99, 234, 107, 155, 36,
  99, 195, 48, 157, 231, 159, 217, 180, 41, 226, 107, 13, 195, 44, 120, 30,
  206, 103, 185, 216, 9, 228, 237, 197, 171, 226, 29, 205, 122, 11, 252, 179,
  203, 44, 117, 160, 103, 102, 65, 61, 102, 249, 231, 156, 5, 155, 142, 135,
  190, 97, 21, 31, 129, 236, 25, 99, 5, 216, 28, 157, 231, 115, 32, 123,
  38, 76, 94, 85, 65, 99, 48, 74, 41, 178, 63, 175, 248, 84, 124, 121,
  234, 204, 82, 229, 225, 17, 39, 103, 74, 89, 134, 223, 79, 224, 55, 234,
  17, 105, 197, 141, 133, 242, 251, 10, 51, 80, 163, 166, 44, 37, 63, 145,
  146, 75, 22, 139, 233, 244, 154, 3, 9, 126, 23, 28, 207, 103, 24, 198,
  26, 131, 208, 56, 151, 25, 180, 230, 100, 20, 154, 165, 220, 144, 43, 237,
  198, 198, 121, 24, 193, 62, 180, 119, 146, 135, 79, 237, 83, 18, 167, 206,
  75, 218, 0, 155, 59, 100, 123, 79, 108, 99, 235, 0, 121, 10, 155, 122,
  11, 200, 19, 218, 231, 179, 8, 239, 104, 155, 192, 156, 218, 174, 45, 232,
  199, 55, 71, 101, 39, 61, 110, 117, 96, 160, 196, 170, 143, 73, 165, 0,
  238, 188, 143, 219, 52, 247, 113, 11, 173, 196, 2, 47, 101, 39, 166, 92,
  54, 244, 109, 232, 0, 177, 224, 49, 143, 244, 104, 123, 199, 90, 199, 87,
  130, 209, 10, 142, 203, 202, 118, 94, 150, 68, 120, 184, 90, 68, 42, 122,
  56, 74, 18, 232, 21, 157, 114, 163, 107, 123, 154, 196, 49, 128, 226, 49,
  70, 131, 58, 187, 154, 177, 20, 212, 191, 105, 14, 68, 11, 254, 246, 239,
  255, 85, 156, 52, 156, 177, 241, 207, 127, 153, 103, 140, 10, 62, 92, 21,
  64, 126, 99, 134, 108, 146, 111, 107, 175, 248, 98, 251, 13, 244, 73, 86,
  132, 172, 93, 167, 24, 218, 197, 11, 109, 203, 204, 56, 21, 124, 211, 48,
  174, 27, 83, 78, 51, 142, 0, 86, 251, 20, 20, 20, 255, 117, 137, 92,
  48, 147, 44, 88, 109, 154, 193, 198, 121, 177, 213, 242, 217, 91, 234, 237,
  135, 135, 86, 176, 160, 98, 148, 209, 189, 199, 26, 242, 213, 199, 142, 131,
  112, 58, 96, 177, 27, 154, 116, 239, 241, 233, 105, 202, 252, 103, 237, 246,
  140, 157, 153, 1, 73, 239, 185, 79, 211, 195, 95, 105, 171, 76, 225, 185,
  173, 222, 244, 43, 185, 2, 84, 60, 107, 182, 98, 85, 112, 189, 245, 4,
  5, 204, 154, 248, 122, 151, 86, 117, 159, 190, 247, 126, 135, 123, 150, 108,
  199, 75, 200, 165, 47, 32, 249, 92, 50, 76, 28, 79, 248, 109, 216, 167,
  44, 199, 195, 191, 105, 134, 75, 184, 111, 211, 90, 18, 160, 182, 0, 36,
  2, 109, 58, 161, 49, 23, 56, 114, 23, 79, 168, 173, 136, 225, 68, 68,
  142, 7, 201, 140, 157, 96, 98, 25, 3, 65, 13, 140, 11, 172, 70, 92,
  6, 34, 195, 196, 69, 204, 135, 117, 65, 187, 218, 28, 67, 113, 169, 12,
  44, 89, 135, 110, 113, 136, 74, 228, 32, 191, 84, 173, 178, 203, 28, 37,
  43, 52, 94, 241, 88, 10, 238, 246, 221, 224, 174, 45, 11, 183, 244, 226,
  71, 9, 224, 245, 23, 119, 114, 222, 215, 181, 129, 59, 221, 58, 170, 190,
  193, 180, 82, 230, 133, 195, 121, 234, 31, 122, 199, 196, 131, 194, 253, 47,
  153, 24, 11, 20, 194, 189, 203, 229, 42, 223, 184, 72, 185, 225, 188, 16,
  94, 234, 238, 101, 79, 219, 139, 217, 240, 109, 196, 46, 105, 202, 46, 210,
  59, 61, 76, 247, 156, 133, 249, 36, 156, 57, 179, 89, 164, 151, 120, 214,
  153, 48, 48, 60, 47, 222, 100, 225, 193, 126, 47, 228, 53, 201, 101, 60,
  182, 204, 27, 233, 10, 25, 67, 183, 225, 175, 97, 136, 39, 35, 122, 155,
  198, 187, 21, 162, 135, 190, 187, 234, 58, 108, 193, 4, 252, 166, 252, 34,
  224, 37, 224, 12, 9, 126, 128, 79, 10, 90, 195, 206, 120, 154, 127, 12,
  254, 80, 113, 193, 218, 4, 253, 148, 157, 206, 207, 12, 127, 79, 88, 104,
  91, 83, 80, 146, 74, 238, 41, 149, 60, 62, 73, 243, 152, 222, 77, 18,
  202, 254, 66, 127, 174, 226, 73, 194, 47, 185, 185, 232, 152, 194, 253, 11,
  225, 245, 99, 104, 237, 136, 71, 89, 125, 199, 210, 188, 204, 175, 26, 203,
  94, 105, 101, 145, 197, 238, 27, 254, 84, 255, 105, 208, 252, 132, 191, 51,
  96, 13, 168, 124, 125, 64, 250, 185, 129, 126, 139, 198, 0, 81, 118, 207,
  171, 180, 232, 239, 175, 122, 117, 87, 93, 105, 89, 102, 246, 47, 210, 92,
  244, 6, 23, 41, 48, 11, 188, 166, 76, 23, 160, 178, 59, 244, 250, 27,
  163, 66, 90, 240, 80, 36, 152, 66, 71, 120, 37, 139, 1, 199, 116, 31,
  166, 192, 89, 146, 94, 11, 97, 188, 247, 7, 254, 29, 177, 5, 183, 128,
  252, 181, 189, 82, 219, 105, 243, 4, 148, 112, 89, 131, 119, 117, 169, 198,
  244, 106, 158, 48, 232, 229, 203, 172, 70, 34, 131, 54, 229, 49, 253, 184,
  249, 138, 139, 170, 81, 56, 143, 243, 215, 209, 52, 59, 156, 142, 18, 58,
  160, 33, 23, 128, 25, 164, 4, 151, 44, 197, 115, 6, 24, 160, 57, 104,
  66, 237, 69, 83, 122, 105, 191, 166, 189, 227, 167, 143, 203, 124, 138, 138,
  93, 13, 244, 235, 120, 24, 202, 77, 205, 116, 62, 57, 69, 221, 119, 18,
  77, 119, 87, 58, 43, 248, 118, 200, 238, 202, 218, 246, 10, 10, 103, 218,
  171, 58, 14, 153, 75, 30, 201, 8, 140, 246, 143, 150, 195, 104, 16, 255,
  157, 48, 122, 243, 199, 165, 16, 122, 115, 245, 119, 194, 231, 100, 57, 124,
  78, 190, 34, 62, 149, 126, 177, 225, 240, 2, 55, 151, 67, 228, 93, 242,
  55, 92, 236, 148, 244, 88, 84, 9, 160, 78, 176, 159, 76, 71, 209, 89,
  137, 19, 171, 233, 59, 57, 36, 19, 145, 221, 232, 59, 122, 128, 192, 191,
  0, 191, 139, 210, 243, 24, 157, 245, 161, 32, 144, 140, 157, 205, 167, 103,
  193, 207, 127, 153, 194, 246, 109, 250, 8, 116, 200, 56, 195, 139, 20, 8,
  37, 56, 255, 249, 191, 79, 167, 120, 34, 14, 255, 9, 201, 56, 159, 158,
  166, 225, 124, 48, 198, 103, 199, 38, 120, 203, 108, 106, 95, 39, 187, 179,
  48, 88, 91, 177, 122, 232, 70, 156, 120, 18, 229, 176, 219, 65, 175, 23,
  134, 79, 150, 197, 65, 56, 207, 90, 128, 203, 48, 224, 110, 191, 131, 115,
  192, 176, 254, 146, 193, 34, 13, 235, 85, 163, 9, 224, 241, 42, 2, 117,
  130, 204, 25, 66, 94, 56, 183, 13, 76, 55, 228, 156, 174, 39, 95, 46,
  51, 94, 178, 173, 224, 207, 184, 19, 187, 247, 222, 163, 218, 18, 169, 97,
  7, 196, 123, 18, 106, 183, 227, 132, 173, 226, 79, 243, 244, 231, 191, 14,
  206, 23, 109, 170, 160, 246, 75, 114, 172, 123, 135, 126, 111, 233, 18, 197,
  201, 13, 161, 132, 131, 42, 156, 19, 252, 6, 18, 132, 119, 111, 151, 235,
  50, 235, 72, 54, 72, 163, 153, 90, 134, 64, 47, 147, 199, 25, 47, 88,
  150, 133, 103, 44, 11, 118, 131, 41, 187, 196, 187, 40, 245, 198, 142, 91,
  140, 135, 65, 135, 66, 195, 100, 48, 71, 227, 92, 251, 140, 229, 7, 49,
  195, 159, 79, 174, 15, 135, 245, 154, 86, 172, 230, 131, 128, 54, 231, 133,
  245, 177, 144, 93, 91, 11, 138, 14, 226, 161, 10, 132, 86, 18, 129, 8,
  40, 163, 249, 148, 27, 223, 201, 145, 76, 228, 215, 245, 71, 156, 162, 81,
  80, 255, 70, 239, 230, 127, 254, 207, 193, 55, 10, 159, 6, 84, 204, 231,
  233, 116, 71, 149, 87, 89, 237, 8, 139, 63, 63, 121, 113, 4, 104, 213,
  106, 86, 9, 73, 218, 54, 232, 20, 7, 48, 251, 235, 147, 236, 44, 216,
  221, 211, 26, 150, 93, 140, 35, 189, 91, 131, 20, 54, 52, 76, 244, 172,
  94, 139, 163, 130, 34, 248, 127, 113, 212, 198, 147, 165, 125, 174, 91, 67,
  69, 0, 171, 231, 23, 216, 133, 179, 25, 116, 121, 127, 28, 197, 195, 122,
  28, 105, 64, 110, 27, 22, 170, 188, 223, 109, 98, 176, 182, 96, 96, 128,
  108, 118, 3, 55, 211, 193, 163, 160, 134, 155, 239, 90, 208, 15, 106, 200,
  225, 170, 211, 183, 14, 189, 103, 243, 108, 76, 212, 174, 79, 56, 8, 135,
  230, 42, 221, 75, 97, 213, 112, 56, 28, 42, 16, 69, 25, 36, 29, 168,
  214, 109, 124, 63, 198, 147, 109, 142, 118, 57, 150, 26, 211, 212, 39, 97,
  62, 24, 191, 78, 217, 40, 186, 194, 249, 48, 143, 99, 27, 101, 163, 196,
  174, 91, 198, 70, 157, 160, 215, 117, 210, 7, 44, 206, 152, 81, 227, 113,
  154, 134, 215, 237, 81, 154, 76, 234, 70, 229, 70, 21, 227, 8, 116, 178,
  179, 54, 73, 214, 236, 93, 148, 143, 117, 236, 26, 13, 11, 147, 33, 172,
  94, 57, 195, 26, 6, 55, 233, 172, 112, 187, 44, 241, 176, 105, 115, 90,
  234, 52, 48, 115, 112, 244, 14, 240, 201, 60, 100, 73, 6, 92, 134, 243,
  52, 26, 156, 215, 154, 1, 76, 66, 232, 149, 62, 0, 141, 162, 45, 155,
  78, 5, 55, 213, 94, 68, 89, 134, 142, 237, 79, 95, 189, 128, 82, 52,
  77, 250, 129, 53, 249, 45, 140, 47, 163, 233, 48, 185, 244, 224, 66, 68,
  66, 92, 232, 89, 191, 134, 73, 102, 62, 59, 137, 250, 1, 229, 63, 106,
  11, 70, 67, 17, 81, 251, 113, 58, 8, 231, 248, 154, 244, 191, 28, 115,
  98, 107, 18, 160, 64, 247, 19, 228, 210, 175, 126, 240, 221, 13, 192, 186,
  253, 84, 160, 87, 72, 169, 82, 4, 231, 211, 49, 104, 242, 160, 101, 165,
  236, 207, 252, 72, 165, 26, 219, 148, 78, 202, 11, 132, 249, 247, 142, 183,
  79, 117, 81, 248, 119, 191, 19, 213, 218, 106, 70, 62, 178, 82, 96, 190,
  215, 81, 29, 76, 70, 170, 5, 96, 255, 90, 150, 227, 19, 111, 53, 85,
  28, 197, 194, 143, 18, 227, 96, 6, 60, 29, 193, 40, 22, 168, 55, 202,
  40, 244, 154, 23, 173, 164, 145, 154, 179, 36, 248, 235, 209, 176, 225, 244,
  30, 118, 112, 229, 11, 4, 84, 216, 177, 132, 62, 76, 94, 13, 15, 47,
  99, 125, 119, 19, 13, 53, 124, 2, 33, 172, 160, 64, 185, 80, 201, 24,
  221, 189, 133, 22, 155, 220, 251, 179, 4, 83, 213, 17, 19, 47, 68, 139,
  197, 150, 152, 39, 56, 149, 77, 190, 197, 18, 95, 216, 38, 127, 209, 116,
  153, 214, 158, 242, 101, 130, 183, 23, 101, 17, 236, 4, 154, 210, 194, 118,
  140, 11, 9, 174, 137, 20, 193, 163, 118, 47, 76, 236, 197, 72, 180, 1,
  172, 102, 52, 82, 186, 14, 241, 134, 78, 14, 143, 14, 62, 62, 61, 120,
  118, 12, 16, 222, 171, 102, 110, 64, 211, 234, 215, 226, 249, 21, 204, 37,
  218, 11, 245, 107, 244, 70, 7, 124, 206, 167, 81, 14, 95, 148, 37, 118,
  213, 111, 121, 203, 253, 60, 157, 51, 149, 248, 138, 191, 46, 29, 220, 54,
  45, 168, 248, 96, 64, 1, 22, 95, 99, 80, 80, 181, 71, 25, 150, 130,
  222, 117, 161, 15, 146, 94, 1, 156, 226, 150, 41, 232, 179, 217, 100, 41,
  168, 61, 23, 42, 198, 105, 43, 192, 202, 176, 230, 5, 222, 255, 109, 127,
  41, 200, 107, 46, 100, 25, 202, 77, 35, 116, 17, 210, 77, 53, 240, 219,
  165, 192, 175, 187, 224, 49, 100, 175, 6, 90, 198, 35, 190, 51, 230, 27,
  46, 232, 139, 153, 54, 140, 111, 95, 23, 163, 120, 254, 58, 92, 10, 230,
  102, 1, 243, 195, 142, 203, 148, 111, 15, 143, 15, 159, 28, 30, 29, 158,
  252, 244, 241, 15, 7, 63, 225, 100, 193, 216, 174, 31, 137, 205, 163, 24,
  72, 246, 241, 162, 211, 219, 84, 124, 13, 171, 55, 197, 205, 125, 171, 242,
  161, 202, 205, 173, 9, 24, 11, 80, 227, 144, 167, 56, 191, 61, 9, 103,
  245, 28, 215, 10, 80, 88, 135, 13, 15, 42, 7, 63, 29, 124, 124, 241,
  248, 205, 31, 126, 124, 13, 245, 62, 153, 182, 195, 127, 229, 33, 103, 217,
  53, 251, 87, 97, 16, 248, 215, 21, 94, 0, 190, 241, 125, 227, 214, 12,
  54, 131, 25, 27, 98, 73, 32, 2, 150, 194, 51, 74, 248, 20, 129, 85,
  96, 215, 121, 30, 147, 46, 42, 107, 16, 81, 253, 5, 140, 152, 190, 255,
  186, 34, 94, 146, 255, 87, 180, 248, 201, 178, 173, 85, 216, 208, 2, 205,
  69, 64, 17, 238, 141, 117, 113, 166, 234, 0, 170, 173, 132, 183, 134, 151,
  6, 159, 36, 87, 144, 216, 9, 58, 65, 111, 29, 254, 95, 226, 192, 205,
  179, 10, 233, 189, 135, 232, 36, 23, 96, 47, 94, 116, 123, 193, 198, 160,
  181, 17, 116, 90, 219, 193, 3, 252, 47, 91, 15, 30, 4, 219, 248, 95,
  139, 254, 107, 173, 195, 127, 240, 247, 243, 164, 19, 116, 123, 225, 70, 0,
  101, 131, 46, 252, 175, 211, 234, 118, 2, 254, 137, 9, 248, 40, 47, 148,
  105, 109, 133, 107, 193, 154, 72, 233, 4, 155, 1, 255, 162, 255, 181, 54,
  63, 3, 70, 163, 40, 70, 122, 12, 230, 41, 104, 94, 60, 182, 217, 191,
  174, 172, 162, 249, 240, 226, 204, 237, 28, 217, 113, 135, 247, 237, 222, 4,
  91, 239, 110, 193, 255, 183, 186, 237, 245, 0, 254, 107, 173, 181, 187, 248,
  223, 126, 119, 35, 232, 117, 218, 144, 187, 214, 134, 95, 208, 161, 30, 252,
  43, 40, 193, 123, 12, 191, 218, 219, 80, 7, 202, 7, 107, 173, 245, 246,
  246, 81, 183, 189, 25, 172, 3, 28, 248, 254, 60, 129, 174, 195, 39, 212,
  122, 12, 255, 137, 46, 118, 137, 70, 197, 55, 252, 26, 64, 249, 78, 208,
  126, 208, 106, 67, 27, 173, 246, 90, 140, 105, 0, 184, 189, 1, 40, 109,
  183, 183, 0, 56, 252, 211, 107, 111, 65, 115, 27, 240, 183, 7, 255, 110,
  80, 211, 240, 179, 219, 126, 208, 198, 196, 141, 207, 147, 181, 246, 54, 226,
  61, 64, 122, 111, 139, 33, 234, 16, 118, 93, 232, 197, 26, 52, 176, 6,
  48, 54, 218, 27, 49, 36, 32, 236, 141, 1, 212, 134, 191, 208, 30, 86,
  133, 34, 107, 45, 4, 66, 137, 72, 131, 245, 214, 6, 252, 125, 64, 255,
  182, 177, 183, 0, 136, 192, 109, 3, 150, 240, 103, 19, 255, 27, 96, 13,
  160, 18, 224, 136, 116, 162, 63, 139, 71, 81, 110, 255, 63, 185, 250, 12,
  58, 157, 157, 24, 115, 219, 216, 159, 226, 212, 71, 247, 155, 161, 49, 227,
  131, 32, 79, 175, 141, 61, 129, 44, 243, 47, 199, 175, 94, 182, 103, 97,
  154, 177, 58, 5, 210, 56, 206, 147, 20, 84, 56, 84, 135, 14, 65, 210,
  215, 61, 226, 167, 65, 10, 237, 205, 109, 141, 126, 232, 141, 220, 6, 3,
  220, 85, 4, 184, 55, 105, 56, 123, 87, 181, 1, 171, 161, 3, 48, 180,
  158, 39, 1, 181, 76, 162, 40, 40, 132, 25, 72, 76, 4, 224, 219, 108,
  84, 72, 53, 41, 165, 80, 221, 224, 162, 21, 181, 86, 212, 56, 70, 34,
  216, 6, 124, 35, 135, 55, 236, 253, 145, 9, 244, 125, 52, 252, 0, 37,
  137, 64, 244, 251, 27, 80, 94, 49, 34, 209, 40, 194, 91, 99, 143, 172,
  28, 0, 158, 161, 102, 161, 26, 81, 137, 26, 254, 197, 207, 66, 210, 202,
  13, 27, 0, 70, 132, 10, 180, 49, 165, 141, 120, 19, 178, 26, 25, 94,
  157, 162, 106, 220, 62, 103, 215, 89, 157, 143, 31, 209, 191, 216, 251, 69,
  67, 187, 107, 164, 185, 138, 122, 51, 188, 35, 136, 82, 185, 61, 14, 179,
  87, 151, 211, 215, 248, 176, 108, 154, 95, 183, 97, 216, 227, 186, 73, 132,
  38, 44, 107, 141, 134, 181, 139, 92, 130, 78, 86, 199, 245, 161, 187, 173,
  216, 88, 207, 248, 29, 162, 10, 190, 182, 249, 215, 96, 214, 172, 156, 89,
  155, 156, 195, 249, 222, 35, 26, 93, 91, 253, 108, 52, 190, 132, 123, 51,
  104, 127, 121, 238, 117, 187, 141, 53, 15, 179, 131, 171, 25, 222, 237, 64,
  187, 5, 134, 212, 117, 44, 31, 34, 29, 175, 126, 216, 200, 219, 67, 194,
  75, 126, 16, 140, 238, 108, 67, 74, 74, 59, 3, 231, 98, 58, 159, 13,
  209, 206, 116, 141, 213, 231, 97, 92, 63, 205, 167, 208, 77, 129, 184, 131,
  241, 41, 110, 242, 109, 59, 141, 48, 95, 209, 201, 217, 174, 170, 139, 70,
  34, 103, 105, 39, 139, 145, 72, 197, 96, 126, 209, 25, 164, 21, 144, 0,
  60, 14, 249, 227, 28, 208, 7, 65, 201, 234, 53, 93, 183, 168, 53, 13,
  224, 72, 8, 130, 71, 61, 212, 55, 149, 37, 96, 8, 67, 169, 205, 85,
  22, 39, 221, 197, 41, 233, 18, 143, 162, 230, 85, 240, 182, 218, 131, 254,
  219, 156, 201, 131, 199, 36, 125, 12, 147, 178, 214, 214, 158, 123, 168, 21,
  19, 157, 88, 206, 107, 30, 20, 188, 178, 75, 99, 221, 198, 195, 0, 192,
  89, 64, 217, 177, 5, 131, 100, 56, 123, 168, 212, 206, 75, 210, 113, 183,
  132, 85, 119, 44, 41, 218, 38, 213, 131, 12, 138, 252, 222, 124, 29, 54,
  33, 49, 12, 42, 31, 151, 111, 20, 199, 232, 21, 109, 222, 34, 64, 6,
  41, 128, 14, 82, 177, 172, 53, 52, 190, 219, 89, 74, 184, 224, 6, 27,
  106, 91, 184, 23, 96, 154, 82, 252, 168, 245, 225, 166, 100, 189, 209, 230,
  151, 34, 141, 71, 234, 121, 199, 219, 220, 183, 138, 22, 27, 101, 146, 175,
  188, 55, 108, 154, 205, 83, 70, 29, 186, 102, 217, 63, 132, 143, 104, 253,
  175, 57, 220, 180, 96, 220, 124, 171, 73, 59, 154, 102, 176, 4, 61, 30,
  254, 57, 68, 231, 14, 180, 142, 215, 107, 167, 12, 208, 99, 160, 178, 3,
  199, 88, 91, 142, 134, 127, 105, 81, 252, 122, 205, 36, 198, 229, 120, 216,
  120, 67, 98, 3, 107, 218, 157, 220, 13, 236, 89, 227, 97, 50, 159, 54,
  38, 11, 217, 35, 37, 211, 75, 216, 195, 61, 130, 160, 123, 10, 20, 131,
  210, 179, 52, 12, 211, 240, 210, 200, 211, 165, 61, 244, 62, 203, 31, 79,
  163, 73, 136, 160, 158, 165, 225, 132, 213, 185, 5, 213, 169, 86, 61, 113,
  230, 51, 42, 252, 156, 140, 116, 105, 246, 107, 23, 91, 84, 219, 181, 139,
  78, 146, 121, 198, 200, 121, 72, 89, 146, 125, 212, 173, 150, 39, 62, 106,
  236, 152, 6, 36, 238, 227, 203, 157, 194, 164, 201, 170, 70, 137, 206, 65,
  85, 142, 167, 6, 122, 249, 223, 253, 78, 255, 68, 253, 155, 204, 121, 87,
  57, 44, 98, 254, 140, 122, 173, 55, 4, 29, 188, 79, 7, 11, 38, 120,
  241, 40, 47, 52, 161, 29, 24, 200, 51, 59, 181, 241, 111, 88, 72, 61,
  249, 113, 255, 15, 7, 39, 31, 55, 94, 64, 189, 141, 224, 251, 96, 179,
  3, 255, 116, 59, 157, 142, 183, 88, 151, 202, 117, 43, 10, 30, 31, 254,
  241, 227, 243, 143, 47, 208, 156, 182, 41, 75, 149, 20, 125, 250, 248, 39,
  94, 16, 54, 166, 213, 37, 143, 14, 223, 30, 124, 124, 119, 248, 242, 233,
  171, 119, 162, 198, 130, 10, 227, 8, 213, 180, 235, 99, 210, 213, 244, 13,
  131, 32, 82, 113, 114, 130, 124, 161, 151, 126, 63, 65, 73, 127, 19, 76,
  162, 97, 63, 120, 255, 1, 22, 250, 4, 35, 243, 225, 175, 240, 236, 108,
  163, 127, 19, 156, 206, 7, 231, 44, 239, 183, 186, 205, 32, 155, 79, 250,
  157, 38, 180, 57, 159, 230, 100, 234, 195, 50, 221, 133, 133, 178, 107, 116,
  45, 232, 243, 93, 196, 173, 53, 32, 47, 30, 255, 241, 227, 230, 243, 143,
  175, 95, 29, 190, 60, 193, 174, 110, 117, 118, 130, 213, 213, 96, 115, 28,
  252, 167, 141, 9, 168, 131, 63, 0, 236, 209, 136, 165, 78, 165, 222, 186,
  86, 171, 219, 229, 213, 122, 235, 80, 175, 235, 171, 136, 155, 198, 152, 46,
  25, 162, 33, 27, 84, 237, 199, 184, 22, 118, 236, 179, 213, 100, 112, 78,
  23, 236, 136, 40, 58, 226, 176, 85, 137, 38, 236, 115, 50, 101, 253, 218,
  193, 28, 183, 21, 171, 79, 88, 26, 71, 120, 252, 192, 102, 201, 96, 252,
  134, 141, 250, 200, 165, 226, 115, 63, 156, 193, 204, 101, 195, 199, 57, 165,
  22, 251, 35, 68, 132, 223, 203, 192, 70, 140, 101, 85, 240, 201, 193, 219,
  143, 251, 175, 158, 30, 64, 238, 10, 94, 137, 236, 246, 214, 214, 191, 93,
  177, 6, 28, 231, 40, 159, 45, 116, 86, 93, 140, 120, 149, 196, 210, 95,
  190, 210, 68, 214, 128, 152, 66, 135, 248, 126, 96, 201, 39, 100, 146, 129,
  53, 97, 203, 166, 169, 53, 192, 154, 215, 168, 45, 54, 180, 172, 154, 191,
  22, 137, 16, 23, 2, 8, 18, 39, 209, 196, 174, 42, 187, 66, 168, 20,
  238, 179, 54, 174, 69, 142, 141, 170, 86, 135, 208, 117, 96, 0, 182, 118,
  154, 137, 108, 69, 110, 53, 174, 197, 227, 83, 6, 162, 69, 178, 7, 87,
  202, 148, 136, 106, 0, 56, 150, 69, 130, 131, 162, 55, 171, 2, 191, 194,
  197, 215, 90, 40, 40, 205, 198, 172, 240, 78, 45, 74, 23, 105, 118, 233,
  107, 79, 233, 235, 210, 210, 50, 130, 111, 81, 86, 166, 216, 37, 229, 28,
  231, 243, 166, 40, 111, 166, 59, 43, 157, 253, 88, 140, 213, 97, 61, 203,
  91, 87, 123, 156, 195, 170, 170, 229, 120, 107, 106, 46, 249, 86, 77, 45,
  167, 100, 114, 249, 170, 58, 89, 118, 93, 235, 69, 148, 162, 166, 149, 225,
  111, 211, 174, 164, 167, 250, 198, 130, 110, 229, 152, 195, 64, 73, 118, 89,
  50, 149, 160, 7, 61, 30, 51, 22, 229, 141, 100, 215, 145, 166, 8, 229,
  192, 3, 165, 105, 20, 212, 242, 188, 245, 126, 76, 227, 67, 114, 15, 52,
  171, 64, 178, 183, 184, 140, 93, 229, 140, 148, 158, 233, 111, 8, 54, 15,
  222, 150, 50, 215, 177, 72, 5, 51, 242, 85, 192, 116, 111, 5, 21, 93,
  200, 139, 156, 202, 245, 55, 38, 182, 118, 62, 242, 137, 60, 111, 61, 45,
  188, 153, 85, 75, 203, 89, 84, 19, 3, 163, 149, 215, 198, 220, 69, 16,
  188, 120, 59, 249, 139, 160, 136, 232, 115, 229, 96, 68, 129, 37, 225, 152,
  44, 92, 81, 200, 207, 208, 195, 40, 231, 126, 97, 6, 43, 67, 170, 183,
  56, 143, 176, 227, 86, 224, 233, 222, 42, 234, 186, 162, 85, 69, 165, 87,
  77, 52, 169, 92, 121, 230, 25, 101, 249, 57, 84, 139, 128, 98, 179, 167,
  150, 85, 194, 221, 122, 52, 19, 135, 185, 245, 76, 255, 248, 200, 216, 36,
  246, 144, 200, 116, 111, 45, 21, 104, 196, 170, 165, 210, 253, 131, 33, 130,
  134, 216, 99, 33, 146, 189, 117, 138, 48, 29, 86, 173, 34, 163, 180, 30,
  106, 151, 158, 90, 152, 236, 151, 69, 42, 54, 134, 45, 140, 84, 70, 41,
  199, 136, 144, 21, 30, 158, 17, 57, 165, 88, 146, 51, 138, 7, 77, 74,
  47, 17, 129, 60, 222, 144, 35, 1, 121, 178, 127, 156, 139, 231, 133, 236,
  145, 46, 114, 22, 213, 244, 77, 93, 51, 183, 76, 2, 167, 215, 238, 28,
  164, 228, 162, 2, 223, 56, 224, 122, 249, 66, 90, 18, 116, 165, 75, 203,
  230, 91, 139, 90, 28, 93, 176, 154, 155, 253, 150, 223, 117, 195, 18, 99,
  161, 209, 234, 133, 200, 27, 141, 174, 79, 217, 219, 19, 185, 133, 122, 43,
  31, 34, 161, 157, 7, 172, 189, 151, 92, 253, 107, 194, 206, 242, 108, 204,
  127, 155, 251, 29, 91, 136, 189, 130, 189, 70, 26, 185, 27, 32, 44, 43,
  67, 181, 99, 152, 124, 95, 190, 10, 109, 143, 235, 122, 234, 33, 130, 42,
  240, 148, 133, 67, 122, 27, 74, 219, 231, 25, 5, 30, 211, 197, 27, 63,
  18, 232, 110, 142, 40, 224, 14, 206, 221, 40, 10, 53, 9, 205, 247, 30,
  183, 131, 23, 7, 39, 111, 14, 247, 63, 194, 159, 199, 152, 91, 28, 92,
  206, 175, 250, 64, 48, 221, 165, 70, 57, 129, 112, 255, 154, 65, 52, 1,
  84, 76, 71, 23, 116, 160, 233, 223, 120, 93, 102, 76, 167, 26, 127, 245,
  65, 210, 43, 26, 229, 62, 49, 150, 187, 140, 170, 103, 120, 239, 160, 15,
  76, 209, 44, 249, 141, 136, 122, 197, 251, 133, 165, 141, 74, 55, 23, 5,
  224, 183, 101, 14, 47, 254, 250, 232, 199, 226, 111, 92, 247, 105, 241, 215,
  133, 173, 138, 170, 202, 125, 83, 12, 167, 21, 85, 73, 243, 206, 185, 245,
  153, 150, 240, 245, 87, 101, 11, 168, 182, 168, 88, 21, 164, 81, 133, 1,
  155, 75, 235, 1, 78, 28, 36, 112, 169, 124, 228, 53, 248, 82, 40, 43,
  165, 124, 117, 83, 159, 114, 185, 146, 9, 3, 185, 8, 201, 4, 97, 106,
  23, 238, 55, 177, 90, 2, 208, 34, 131, 95, 175, 233, 94, 61, 255, 45,
  150, 21, 249, 137, 82, 31, 127, 255, 27, 46, 81, 248, 99, 36, 150, 29,
  89, 128, 59, 109, 214, 106, 218, 215, 241, 124, 52, 138, 174, 84, 26, 10,
  88, 245, 193, 165, 58, 125, 102, 252, 12, 19, 229, 82, 159, 235, 228, 31,
  209, 139, 133, 6, 3, 227, 186, 61, 163, 72, 14, 80, 212, 30, 136, 147,
  55, 7, 47, 159, 126, 220, 127, 245, 242, 217, 225, 239, 141, 153, 68, 204,
  9, 244, 226, 110, 162, 253, 96, 171, 137, 215, 113, 158, 178, 56, 15, 251,
  65, 167, 221, 217, 194, 70, 83, 50, 101, 117, 218, 107, 27, 218, 168, 7,
  126, 62, 45, 135, 181, 94, 128, 234, 182, 203, 33, 137, 105, 230, 5, 210,
  221, 40, 96, 108, 116, 116, 16, 29, 155, 113, 43, 250, 212, 213, 251, 212,
  49, 48, 89, 43, 14, 96, 125, 4, 124, 113, 240, 226, 213, 155, 159, 60,
  130, 138, 54, 138, 71, 225, 117, 50, 207, 51, 79, 54, 217, 132, 10, 230,
  196, 29, 158, 148, 244, 92, 0, 202, 47, 52, 183, 2, 26, 55, 183, 118,
  251, 226, 154, 231, 65, 108, 109, 78, 197, 85, 209, 98, 113, 83, 6, 121,
  80, 110, 207, 197, 126, 57, 156, 204, 98, 166, 142, 178, 242, 204, 227, 47,
  111, 77, 59, 110, 171, 106, 4, 37, 25, 165, 147, 210, 60, 184, 18, 126,
  189, 72, 18, 244, 233, 229, 247, 189, 106, 104, 32, 249, 230, 37, 253, 110,
  71, 217, 203, 240, 101, 29, 48, 50, 15, 124, 74, 154, 109, 99, 51, 208,
  246, 11, 152, 30, 237, 73, 120, 85, 175, 42, 71, 29, 245, 185, 133, 144,
  127, 123, 85, 3, 123, 65, 167, 180, 231, 109, 236, 181, 117, 100, 126, 107,
  9, 34, 32, 55, 122, 147, 92, 132, 49, 49, 94, 19, 47, 182, 209, 57,
  2, 199, 59, 154, 214, 85, 7, 100, 153, 6, 47, 228, 12, 226, 24, 214,
  118, 49, 196, 206, 205, 18, 197, 18, 141, 130, 59, 180, 51, 85, 188, 220,
  80, 227, 222, 90, 186, 223, 186, 125, 108, 51, 78, 46, 101, 3, 55, 92,
  190, 11, 47, 95, 84, 178, 184, 173, 150, 255, 162, 248, 4, 36, 41, 129,
  97, 167, 44, 5, 149, 31, 82, 195, 233, 96, 156, 164, 127, 148, 63, 126,
  2, 185, 108, 51, 151, 134, 167, 125, 8, 83, 224, 173, 95, 120, 249, 164,
  223, 211, 226, 23, 192, 247, 190, 187, 161, 31, 183, 252, 242, 145, 140, 236,
  254, 221, 141, 66, 245, 182, 136, 127, 230, 171, 45, 251, 33, 0, 124, 242,
  225, 192, 29, 131, 79, 41, 80, 52, 25, 110, 208, 86, 75, 127, 241, 12,
  243, 219, 94, 111, 184, 198, 88, 173, 188, 230, 192, 173, 195, 122, 108, 107,
  212, 241, 214, 41, 70, 10, 3, 130, 95, 48, 119, 176, 10, 207, 123, 50,
  38, 24, 148, 71, 240, 202, 66, 141, 33, 207, 209, 148, 248, 4, 95, 231,
  128, 133, 105, 63, 142, 32, 25, 139, 213, 29, 96, 51, 218, 223, 108, 217,
  201, 201, 104, 148, 49, 108, 165, 187, 105, 103, 161, 32, 197, 59, 119, 192,
  11, 179, 119, 77, 25, 156, 0, 191, 158, 7, 183, 56, 21, 84, 151, 150,
  68, 1, 163, 0, 96, 127, 112, 162, 212, 5, 3, 5, 63, 8, 20, 104,
  113, 206, 219, 84, 230, 7, 196, 86, 36, 80, 164, 187, 160, 69, 72, 192,
  31, 200, 112, 224, 98, 176, 87, 19, 236, 79, 188, 194, 243, 96, 53, 232,
  9, 56, 88, 72, 135, 203, 95, 46, 150, 229, 108, 192, 246, 16, 11, 212,
  63, 1, 59, 194, 175, 219, 217, 85, 5, 35, 113, 116, 160, 40, 252, 208,
  75, 58, 115, 144, 139, 150, 231, 97, 134, 210, 198, 115, 122, 43, 92, 213,
  196, 242, 81, 34, 152, 28, 255, 157, 111, 190, 169, 243, 42, 32, 110, 233,
  7, 137, 174, 114, 73, 48, 14, 51, 24, 46, 24, 52, 107, 185, 224, 171,
  232, 11, 92, 214, 54, 59, 240, 127, 95, 128, 25, 137, 3, 94, 26, 111,
  194, 113, 172, 80, 228, 74, 201, 96, 59, 71, 112, 248, 211, 228, 146, 175,
  122, 143, 103, 179, 52, 185, 58, 192, 83, 161, 23, 120, 188, 252, 232, 17,
  133, 176, 104, 67, 129, 186, 123, 143, 2, 171, 181, 130, 162, 145, 224, 225,
  174, 234, 76, 21, 21, 176, 3, 135, 83, 178, 23, 43, 34, 76, 64, 227,
  178, 61, 39, 241, 96, 7, 157, 255, 173, 14, 78, 104, 211, 136, 235, 221,
  90, 103, 88, 195, 142, 22, 41, 219, 78, 10, 70, 216, 169, 53, 60, 135,
  220, 231, 236, 154, 115, 15, 71, 225, 182, 5, 191, 160, 214, 237, 39, 253,
  8, 91, 160, 160, 109, 162, 222, 67, 189, 15, 216, 132, 142, 150, 231, 110,
  152, 168, 9, 68, 61, 102, 105, 196, 116, 214, 19, 189, 245, 95, 223, 34,
  202, 98, 229, 118, 150, 76, 88, 125, 134, 203, 218, 12, 89, 108, 214, 190,
  32, 247, 22, 218, 179, 194, 183, 88, 251, 103, 237, 220, 88, 252, 203, 249,
  79, 28, 178, 115, 149, 229, 21, 69, 183, 243, 220, 166, 116, 142, 1, 202,
  124, 199, 212, 35, 54, 138, 49, 97, 171, 17, 197, 185, 58, 193, 23, 243,
  77, 151, 34, 54, 108, 207, 209, 31, 15, 187, 167, 157, 249, 65, 130, 215,
  143, 18, 210, 219, 226, 182, 76, 153, 199, 20, 174, 189, 128, 157, 194, 20,
  22, 193, 65, 60, 31, 178, 172, 94, 84, 214, 107, 97, 170, 186, 26, 191,
  11, 243, 7, 234, 219, 249, 226, 1, 66, 39, 247, 214, 242, 41, 42, 26,
  141, 217, 244, 44, 31, 163, 210, 67, 138, 153, 7, 27, 151, 46, 28, 55,
  139, 109, 253, 165, 244, 14, 190, 239, 124, 216, 241, 215, 144, 254, 13, 110,
  125, 159, 107, 164, 78, 196, 253, 87, 71, 175, 222, 124, 124, 253, 248, 232,
  224, 228, 228, 192, 115, 23, 103, 112, 29, 226, 201, 50, 62, 133, 0, 27,
  117, 254, 65, 128, 251, 106, 69, 119, 175, 104, 128, 148, 136, 206, 18, 85,
  237, 80, 126, 202, 138, 155, 107, 155, 155, 163, 174, 167, 98, 72, 76, 46,
  235, 61, 22, 95, 178, 218, 104, 99, 155, 117, 78, 61, 213, 216, 132, 165,
  97, 60, 84, 21, 15, 212, 183, 172, 218, 237, 156, 110, 111, 249, 90, 156,
  69, 211, 115, 85, 239, 53, 255, 144, 149, 216, 96, 125, 107, 123, 219, 83,
  41, 101, 69, 91, 111, 146, 92, 175, 66, 47, 123, 121, 170, 196, 160, 64,
  169, 58, 71, 252, 67, 86, 218, 90, 31, 12, 186, 155, 158, 74, 217, 249,
  181, 170, 115, 76, 191, 101, 21, 254, 96, 165, 167, 202, 69, 148, 128, 112,
  85, 181, 222, 210, 167, 142, 97, 184, 181, 177, 49, 122, 224, 107, 12, 31,
  47, 46, 154, 19, 95, 178, 218, 246, 122, 184, 118, 234, 107, 48, 77, 50,
  166, 17, 35, 211, 43, 141, 78, 31, 116, 183, 54, 106, 190, 123, 77, 97,
  172, 42, 157, 240, 15, 53, 84, 235, 167, 91, 161, 78, 13, 235, 146, 142,
  224, 216, 55, 7, 207, 228, 253, 28, 46, 144, 62, 146, 222, 152, 185, 183,
  115, 40, 29, 111, 223, 154, 91, 75, 195, 203, 127, 95, 149, 169, 244, 132,
  22, 10, 101, 136, 114, 199, 235, 194, 111, 34, 231, 248, 191, 65, 205, 134,
  137, 143, 118, 39, 128, 50, 239, 233, 229, 207, 117, 230, 25, 1, 229, 126,
  254, 35, 242, 156, 118, 28, 164, 131, 50, 114, 184, 2, 194, 246, 24, 175,
  162, 145, 223, 71, 220, 164, 134, 227, 30, 94, 96, 210, 104, 236, 216, 93,
  173, 234, 160, 112, 33, 53, 187, 88, 142, 63, 40, 37, 227, 99, 106, 21,
  84, 186, 212, 214, 67, 198, 186, 85, 53, 192, 64, 67, 20, 219, 56, 136,
  40, 29, 254, 60, 68, 115, 135, 144, 243, 240, 253, 195, 15, 230, 64, 140,
  229, 142, 58, 154, 204, 227, 250, 90, 23, 84, 252, 6, 168, 200, 88, 7,
  101, 50, 70, 4, 125, 156, 215, 35, 24, 90, 189, 25, 71, 41, 32, 16,
  225, 105, 86, 31, 151, 175, 241, 68, 132, 103, 73, 250, 148, 172, 85, 214,
  69, 91, 90, 54, 49, 73, 0, 52, 36, 59, 44, 29, 246, 122, 64, 151,
  197, 213, 8, 224, 173, 2, 85, 213, 76, 182, 85, 131, 104, 136, 78, 27,
  26, 81, 177, 209, 223, 154, 237, 73, 114, 217, 189, 52, 145, 2, 72, 22,
  90, 183, 30, 191, 200, 125, 187, 215, 98, 51, 237, 233, 60, 233, 197, 34,
  211, 213, 107, 244, 78, 201, 125, 166, 118, 239, 217, 229, 242, 234, 129, 248,
  3, 187, 86, 42, 159, 124, 23, 162, 196, 52, 68, 170, 65, 81, 70, 16,
  67, 247, 235, 21, 73, 162, 248, 35, 77, 115, 237, 127, 119, 35, 107, 146,
  238, 203, 83, 63, 5, 125, 44, 35, 115, 110, 63, 45, 100, 26, 174, 27,
  8, 34, 86, 225, 173, 43, 208, 229, 29, 53, 25, 9, 75, 163, 87, 101,
  65, 99, 84, 164, 125, 12, 133, 233, 102, 93, 213, 59, 19, 128, 76, 246,
  66, 81, 153, 14, 5, 173, 57, 162, 147, 14, 145, 84, 20, 108, 44, 195,
  114, 149, 52, 243, 176, 225, 125, 72, 247, 141, 64, 107, 9, 190, 165, 205,
  201, 125, 24, 215, 94, 226, 188, 199, 12, 2, 189, 204, 61, 42, 170, 61,
  121, 222, 125, 176, 209, 169, 89, 103, 65, 158, 116, 178, 62, 215, 94, 60,
  255, 83, 119, 187, 102, 29, 225, 4, 181, 227, 231, 39, 107, 221, 154, 207,
  232, 237, 100, 209, 185, 11, 0, 58, 250, 227, 118, 103, 179, 187, 94, 179,
  140, 211, 252, 8, 197, 54, 53, 27, 183, 224, 69, 111, 20, 245, 163, 210,
  185, 137, 180, 47, 228, 102, 65, 87, 139, 44, 154, 241, 54, 26, 150, 50,
  143, 28, 100, 197, 60, 30, 131, 132, 49, 215, 93, 248, 138, 69, 219, 121,
  242, 227, 108, 134, 222, 144, 160, 29, 148, 243, 43, 249, 184, 43, 48, 207,
  210, 100, 162, 124, 75, 235, 98, 159, 122, 115, 235, 114, 233, 36, 68, 251,
  10, 237, 69, 57, 210, 31, 35, 24, 123, 75, 255, 16, 54, 217, 164, 7,
  240, 101, 105, 248, 250, 200, 107, 16, 170, 225, 12, 83, 28, 35, 21, 218,
  155, 180, 74, 192, 71, 86, 37, 72, 113, 218, 137, 209, 139, 158, 233, 109,
  241, 20, 187, 61, 158, 234, 26, 198, 194, 145, 222, 36, 124, 218, 109, 66,
  146, 57, 251, 36, 158, 168, 113, 232, 108, 35, 131, 16, 200, 236, 29, 51,
  87, 6, 19, 40, 178, 77, 3, 121, 209, 19, 7, 176, 184, 211, 175, 149,
  176, 96, 107, 87, 243, 141, 66, 102, 11, 162, 179, 13, 11, 107, 113, 239,
  94, 100, 218, 171, 125, 207, 173, 193, 3, 23, 136, 44, 179, 124, 221, 226,
  208, 54, 162, 142, 164, 212, 123, 7, 66, 219, 41, 135, 205, 99, 57, 137,
  134, 169, 48, 25, 205, 243, 219, 252, 106, 181, 171, 106, 139, 76, 194, 226,
  179, 118, 251, 131, 91, 197, 106, 150, 202, 19, 65, 140, 48, 33, 183, 203,
  207, 35, 241, 128, 111, 61, 148, 231, 243, 239, 63, 184, 243, 232, 244, 122,
  63, 204, 45, 85, 154, 87, 80, 70, 142, 204, 107, 226, 160, 201, 150, 181,
  197, 81, 43, 153, 121, 200, 126, 231, 179, 120, 80, 35, 239, 179, 246, 64,
  68, 204, 164, 219, 160, 109, 52, 18, 113, 32, 184, 147, 226, 191, 10, 217,
  228, 90, 45, 8, 74, 155, 152, 182, 132, 223, 245, 18, 126, 150, 55, 75,
  220, 122, 224, 139, 33, 42, 99, 124, 179, 80, 57, 239, 219, 229, 124, 77,
  225, 232, 250, 103, 128, 150, 239, 163, 1, 240, 187, 119, 30, 20, 185, 59,
  190, 3, 47, 135, 59, 237, 116, 137, 191, 47, 143, 35, 123, 255, 153, 176,
  20, 239, 223, 133, 225, 121, 52, 41, 238, 244, 90, 199, 131, 43, 54, 205,
  209, 152, 73, 140, 222, 68, 123, 88, 230, 172, 29, 42, 60, 143, 19, 26,
  44, 46, 141, 186, 38, 44, 138, 113, 156, 92, 114, 37, 36, 26, 156, 51,
  242, 130, 198, 38, 218, 158, 140, 93, 251, 82, 173, 192, 172, 184, 125, 13,
  68, 241, 223, 110, 194, 211, 171, 242, 160, 109, 144, 107, 222, 74, 131, 4,
  126, 148, 132, 15, 155, 144, 179, 18, 145, 131, 162, 32, 59, 215, 238, 48,
  177, 77, 55, 81, 27, 84, 145, 126, 162, 62, 160, 210, 61, 86, 202, 75,
  218, 213, 150, 163, 132, 129, 50, 76, 156, 120, 21, 47, 90, 60, 171, 230,
  41, 45, 78, 223, 212, 43, 243, 18, 45, 75, 89, 180, 239, 6, 47, 143,
  21, 213, 176, 98, 31, 81, 11, 28, 20, 240, 30, 125, 233, 242, 135, 147,
  87, 15, 119, 199, 145, 109, 84, 21, 177, 239, 2, 11, 131, 171, 205, 36,
  246, 21, 71, 25, 144, 64, 184, 14, 151, 246, 203, 114, 38, 151, 19, 145,
  44, 166, 6, 197, 245, 128, 242, 129, 32, 191, 158, 86, 51, 65, 152, 219,
  95, 227, 74, 203, 141, 21, 208, 85, 28, 25, 206, 170, 240, 148, 134, 242,
  29, 171, 174, 50, 109, 211, 69, 24, 107, 75, 95, 20, 49, 135, 233, 211,
  223, 254, 143, 255, 37, 248, 238, 102, 64, 203, 132, 121, 4, 34, 43, 88,
  71, 176, 126, 200, 164, 68, 200, 230, 119, 117, 6, 107, 112, 32, 68, 24,
  126, 135, 57, 117, 171, 11, 42, 235, 131, 13, 181, 172, 46, 222, 250, 135,
  198, 19, 173, 110, 140, 71, 76, 234, 146, 225, 141, 7, 87, 113, 136, 66,
  82, 38, 153, 18, 247, 236, 83, 45, 126, 166, 34, 5, 97, 173, 225, 41,
  66, 147, 189, 41, 155, 119, 79, 19, 92, 76, 29, 94, 231, 7, 44, 101,
  215, 104, 97, 50, 233, 165, 161, 118, 201, 157, 72, 175, 212, 214, 110, 70,
  136, 134, 14, 64, 128, 235, 33, 78, 28, 169, 45, 139, 149, 29, 249, 104,
  163, 167, 195, 193, 153, 189, 200, 150, 36, 65, 151, 46, 1, 75, 206, 142,
  251, 206, 140, 5, 179, 226, 78, 51, 98, 169, 217, 96, 207, 4, 73, 187,
  133, 243, 64, 81, 170, 124, 22, 184, 99, 207, 201, 242, 135, 151, 175, 222,
  189, 252, 120, 114, 248, 226, 224, 79, 175, 94, 30, 80, 224, 52, 251, 6,
  96, 237, 199, 147, 125, 248, 87, 164, 30, 37, 211, 33, 134, 37, 172, 61,
  158, 48, 80, 22, 194, 213, 151, 236, 242, 227, 79, 73, 122, 142, 73, 89,
  20, 174, 158, 36, 231, 215, 9, 126, 204, 179, 60, 13, 99, 72, 57, 190,
  30, 78, 217, 117, 237, 131, 97, 63, 55, 206, 153, 184, 26, 183, 163, 71,
  156, 116, 110, 29, 201, 251, 181, 158, 67, 175, 187, 157, 90, 169, 230, 223,
  240, 89, 235, 222, 82, 146, 77, 105, 105, 98, 88, 30, 149, 101, 244, 131,
  90, 111, 29, 215, 80, 103, 3, 48, 207, 199, 108, 248, 140, 193, 10, 85,
  159, 167, 49, 105, 64, 120, 140, 42, 149, 32, 117, 202, 238, 148, 208, 212,
  83, 5, 109, 20, 135, 103, 220, 65, 73, 139, 137, 35, 20, 252, 27, 34,
  28, 100, 18, 243, 20, 1, 111, 96, 170, 201, 68, 30, 193, 180, 56, 215,
  39, 8, 59, 50, 65, 150, 66, 246, 210, 107, 117, 245, 143, 149, 238, 202,
  78, 113, 167, 58, 196, 91, 164, 26, 122, 216, 137, 127, 201, 146, 169, 183,
  171, 134, 133, 60, 101, 89, 121, 112, 35, 126, 77, 32, 188, 12, 163, 188,
  148, 128, 75, 5, 126, 209, 226, 61, 62, 126, 125, 8, 147, 19, 32, 220,
  10, 27, 63, 70, 123, 132, 10, 50, 232, 165, 161, 222, 2, 74, 99, 124,
  239, 2, 242, 125, 230, 117, 138, 207, 198, 221, 225, 117, 129, 100, 119, 67,
  20, 225, 253, 128, 62, 145, 188, 168, 255, 35, 17, 39, 153, 141, 152, 36,
  231, 62, 7, 7, 30, 41, 212, 223, 34, 214, 226, 193, 169, 111, 191, 187,
  17, 23, 39, 63, 5, 252, 39, 89, 142, 107, 53, 83, 230, 105, 33, 129,
  173, 56, 180, 28, 69, 188, 206, 238, 203, 183, 176, 69, 248, 138, 103, 171,
  34, 98, 137, 34, 218, 225, 23, 85, 189, 35, 181, 177, 186, 113, 218, 21,
  212, 57, 41, 26, 247, 38, 187, 39, 92, 82, 50, 155, 227, 89, 232, 137,
  113, 243, 210, 245, 172, 48, 111, 102, 150, 173, 177, 98, 97, 165, 224, 11,
  122, 121, 33, 154, 200, 222, 33, 175, 126, 183, 101, 17, 141, 142, 102, 165,
  178, 213, 214, 90, 41, 138, 152, 16, 159, 127, 137, 5, 55, 255, 92, 189,
  214, 154, 249, 86, 23, 150, 89, 6, 197, 50, 35, 72, 135, 71, 41, 118,
  7, 11, 95, 11, 94, 168, 209, 248, 250, 106, 5, 135, 92, 221, 85, 79,
  161, 229, 251, 235, 235, 109, 195, 207, 39, 158, 150, 72, 223, 245, 148, 109,
  248, 56, 170, 132, 255, 42, 130, 134, 225, 44, 123, 145, 225, 146, 229, 248,
  209, 114, 37, 91, 46, 58, 202, 83, 73, 91, 170, 118, 42, 43, 136, 88,
  197, 190, 33, 3, 88, 128, 42, 119, 118, 166, 182, 119, 92, 33, 162, 92,
  161, 159, 69, 83, 208, 217, 235, 80, 7, 195, 36, 99, 85, 243, 54, 184,
  231, 208, 85, 207, 190, 245, 133, 51, 182, 93, 230, 110, 44, 43, 175, 162,
  44, 15, 209, 64, 234, 72, 145, 40, 227, 49, 24, 158, 93, 118, 126, 17,
  160, 65, 21, 107, 248, 132, 165, 15, 238, 15, 65, 189, 240, 224, 11, 90,
  21, 176, 27, 247, 166, 2, 223, 111, 156, 136, 203, 141, 115, 151, 10, 234,
  50, 116, 195, 177, 113, 58, 4, 114, 3, 19, 137, 186, 214, 68, 170, 225,
  195, 80, 20, 247, 2, 100, 245, 212, 218, 106, 23, 149, 92, 39, 96, 122,
  83, 177, 133, 62, 3, 230, 84, 246, 248, 240, 45, 106, 157, 63, 195, 124,
  7, 28, 200, 101, 188, 20, 129, 146, 137, 110, 92, 13, 247, 206, 1, 225,
  61, 234, 114, 227, 206, 34, 106, 35, 187, 241, 234, 126, 206, 10, 204, 251,
  234, 22, 21, 112, 209, 63, 156, 230, 113, 251, 169, 88, 254, 240, 54, 92,
  152, 215, 149, 186, 218, 4, 61, 118, 156, 204, 211, 126, 173, 215, 26, 70,
  103, 17, 186, 25, 77, 162, 233, 60, 103, 122, 74, 134, 183, 220, 134, 122,
  10, 210, 238, 79, 24, 192, 196, 43, 154, 120, 140, 69, 108, 8, 17, 192,
  182, 235, 212, 135, 198, 226, 241, 172, 234, 13, 141, 233, 191, 205, 89, 28,
  51, 80, 184, 189, 163, 122, 187, 132, 58, 192, 3, 168, 237, 43, 188, 235,
  179, 240, 90, 92, 122, 117, 13, 165, 34, 79, 245, 205, 26, 94, 175, 92,
  182, 235, 216, 131, 108, 235, 25, 37, 235, 67, 57, 152, 91, 59, 228, 55,
  103, 39, 170, 195, 133, 188, 172, 75, 57, 31, 39, 25, 58, 10, 203, 52,
  24, 138, 143, 50, 221, 241, 32, 231, 92, 247, 12, 54, 62, 184, 213, 193,
  253, 143, 142, 198, 71, 33, 2, 204, 213, 64, 171, 3, 220, 42, 145, 241,
  243, 171, 203, 224, 238, 6, 219, 39, 38, 119, 37, 220, 170, 130, 154, 28,
  222, 245, 250, 69, 11, 150, 179, 7, 86, 116, 203, 10, 93, 138, 90, 74,
  209, 181, 197, 221, 112, 130, 121, 150, 53, 131, 75, 102, 199, 49, 126, 122,
  59, 109, 46, 126, 11, 251, 108, 23, 191, 45, 125, 128, 66, 95, 12, 42,
  67, 154, 209, 139, 182, 34, 20, 55, 157, 48, 136, 96, 241, 75, 157, 40,
  136, 99, 3, 138, 172, 134, 44, 141, 39, 12, 67, 161, 50, 104, 207, 228,
  213, 44, 207, 220, 169, 60, 125, 178, 136, 233, 105, 90, 152, 225, 236, 136,
  246, 252, 37, 21, 147, 114, 85, 113, 222, 152, 115, 111, 136, 159, 255, 213,
  150, 16, 89, 204, 119, 147, 69, 86, 183, 45, 161, 116, 213, 56, 153, 231,
  162, 221, 27, 47, 242, 60, 234, 62, 198, 187, 234, 110, 117, 202, 150, 32,
  23, 149, 202, 200, 254, 37, 111, 203, 232, 32, 85, 253, 89, 18, 97, 244,
  55, 178, 214, 102, 38, 20, 124, 118, 170, 226, 133, 26, 165, 154, 87, 60,
  77, 98, 71, 20, 22, 135, 237, 242, 166, 56, 94, 235, 225, 49, 173, 252,
  222, 146, 113, 114, 86, 175, 237, 31, 29, 238, 255, 1, 221, 6, 219, 57,
  93, 31, 111, 6, 20, 71, 104, 50, 131, 149, 107, 72, 175, 23, 212, 101,
  86, 195, 236, 142, 65, 206, 247, 53, 249, 172, 121, 173, 89, 43, 30, 112,
  134, 15, 131, 61, 63, 84, 132, 12, 94, 244, 236, 130, 254, 244, 130, 222,
  5, 244, 129, 171, 137, 129, 170, 249, 208, 143, 27, 114, 28, 161, 160, 232,
  66, 89, 65, 163, 135, 80, 252, 115, 89, 193, 207, 135, 48, 249, 175, 22,
  62, 83, 115, 43, 162, 41, 239, 44, 49, 174, 231, 236, 122, 152, 92, 78,
  203, 70, 150, 181, 201, 151, 10, 167, 252, 65, 54, 8, 103, 204, 218, 37,
  188, 55, 73, 109, 13, 131, 26, 30, 123, 8, 108, 201, 164, 40, 223, 20,
  6, 58, 95, 175, 60, 97, 35, 241, 252, 54, 225, 175, 137, 36, 231, 77,
  244, 78, 5, 116, 242, 166, 188, 61, 125, 183, 231, 53, 188, 50, 80, 249,
  47, 227, 204, 254, 118, 243, 244, 65, 111, 75, 191, 14, 135, 217, 40, 240,
  246, 81, 132, 96, 145, 97, 146, 183, 102, 115, 232, 66, 43, 26, 198, 204,
  18, 141, 210, 211, 0, 175, 155, 112, 84, 93, 41, 105, 27, 155, 2, 173,
  249, 181, 245, 225, 218, 246, 182, 37, 24, 75, 154, 31, 133, 153, 121, 66,
  230, 149, 126, 5, 236, 209, 233, 233, 168, 183, 190, 28, 236, 44, 78, 46,
  107, 149, 130, 205, 234, 237, 55, 222, 238, 22, 141, 11, 175, 125, 29, 164,
  209, 116, 173, 242, 46, 208, 253, 224, 184, 130, 211, 56, 187, 181, 142, 109,
  139, 66, 201, 213, 241, 56, 28, 210, 205, 151, 79, 60, 112, 255, 218, 236,
  10, 143, 50, 176, 194, 237, 218, 218, 39, 163, 146, 187, 182, 88, 67, 212,
  116, 8, 219, 116, 184, 168, 225, 174, 176, 212, 165, 134, 187, 242, 21, 121,
  110, 248, 86, 59, 240, 225, 107, 77, 115, 118, 231, 22, 119, 112, 16, 90,
  134, 186, 123, 45, 235, 162, 255, 189, 7, 150, 59, 227, 200, 0, 165, 106,
  121, 189, 219, 184, 145, 129, 155, 54, 64, 56, 148, 25, 52, 220, 123, 215,
  88, 3, 77, 13, 88, 210, 54, 53, 232, 62, 118, 70, 44, 117, 30, 122,
  35, 57, 239, 115, 5, 57, 39, 207, 56, 152, 117, 74, 124, 24, 25, 146,
  113, 149, 76, 49, 114, 69, 154, 144, 183, 242, 10, 5, 118, 136, 23, 104,
  232, 55, 47, 164, 211, 230, 223, 165, 113, 108, 202, 106, 93, 4, 38, 208,
  26, 71, 71, 66, 111, 227, 152, 81, 222, 56, 230, 150, 52, 174, 174, 204,
  99, 33, 103, 244, 120, 170, 188, 169, 101, 94, 161, 39, 159, 31, 28, 76,
  94, 72, 6, 143, 105, 26, 71, 4, 60, 214, 131, 222, 3, 225, 158, 232,
  239, 133, 200, 172, 232, 137, 40, 81, 73, 74, 108, 214, 34, 165, 30, 45,
  226, 239, 140, 141, 108, 218, 194, 136, 251, 235, 154, 172, 133, 238, 151, 126,
  222, 194, 156, 10, 230, 194, 236, 106, 214, 38, 79, 42, 3, 1, 30, 179,
  194, 88, 61, 52, 92, 32, 23, 81, 33, 123, 8, 65, 128, 132, 134, 218,
  100, 54, 141, 106, 139, 8, 135, 64, 188, 29, 49, 245, 250, 106, 226, 210,
  69, 31, 79, 127, 45, 40, 70, 175, 17, 103, 125, 209, 107, 250, 222, 188,
  176, 174, 20, 135, 51, 221, 173, 152, 198, 71, 70, 104, 17, 221, 20, 159,
  86, 192, 23, 222, 176, 25, 64, 73, 197, 238, 12, 99, 84, 163, 196, 213,
  96, 110, 112, 144, 164, 116, 220, 241, 111, 130, 118, 155, 206, 162, 56, 80,
  84, 149, 198, 44, 140, 243, 49, 48, 47, 7, 144, 156, 23, 87, 172, 5,
  82, 218, 157, 235, 98, 17, 231, 205, 222, 150, 239, 55, 121, 71, 15, 179,
  231, 28, 254, 189, 22, 13, 195, 53, 187, 108, 241, 105, 180, 69, 23, 202,
  113, 129, 177, 122, 10, 107, 34, 190, 101, 236, 179, 153, 235, 135, 188, 56,
  48, 158, 101, 197, 99, 155, 21, 6, 67, 35, 242, 21, 108, 7, 46, 13,
  80, 86, 54, 70, 194, 114, 207, 147, 93, 168, 136, 209, 195, 192, 133, 173,
  170, 120, 154, 109, 185, 182, 125, 4, 179, 231, 195, 65, 63, 14, 128, 138,
  158, 18, 14, 235, 116, 22, 13, 245, 9, 90, 35, 74, 47, 252, 15, 70,
  104, 133, 210, 35, 247, 152, 83, 65, 134, 183, 217, 52, 195, 219, 24, 209,
  109, 122, 118, 156, 29, 123, 46, 84, 222, 3, 231, 129, 207, 26, 242, 222,
  116, 113, 217, 187, 228, 114, 183, 145, 117, 177, 64, 249, 128, 18, 13, 75,
  49, 19, 94, 239, 116, 31, 249, 97, 208, 43, 78, 98, 131, 97, 148, 246,
  97, 15, 31, 230, 20, 19, 41, 199, 8, 75, 42, 126, 14, 239, 183, 219,
  53, 78, 31, 50, 78, 241, 11, 235, 49, 94, 130, 105, 1, 89, 219, 60,
  203, 106, 93, 150, 95, 30, 3, 89, 227, 189, 93, 181, 21, 116, 63, 60,
  2, 10, 60, 122, 180, 16, 75, 14, 43, 216, 173, 6, 214, 190, 176, 235,
  141, 162, 212, 172, 214, 241, 20, 26, 70, 163, 17, 9, 13, 106, 163, 197,
  43, 185, 58, 222, 153, 188, 1, 136, 215, 247, 176, 142, 99, 24, 29, 95,
  195, 134, 23, 100, 91, 132, 219, 0, 164, 160, 228, 184, 224, 123, 140, 236,
  228, 4, 25, 73, 217, 197, 211, 40, 85, 236, 203, 227, 38, 233, 236, 203,
  73, 105, 110, 4, 135, 84, 195, 206, 225, 175, 227, 158, 193, 172, 212, 219,
  109, 136, 210, 212, 67, 212, 136, 30, 5, 181, 249, 140, 108, 51, 180, 31,
  215, 55, 17, 98, 63, 229, 2, 1, 138, 104, 29, 195, 139, 0, 10, 113,
  100, 93, 132, 7, 168, 26, 105, 4, 188, 33, 91, 23, 89, 218, 139, 77,
  190, 238, 34, 150, 169, 217, 85, 110, 156, 225, 36, 114, 101, 153, 124, 224,
  0, 27, 68, 175, 234, 134, 215, 77, 56, 203, 95, 167, 116, 89, 68, 151,
  213, 52, 194, 141, 29, 111, 241, 151, 180, 233, 210, 75, 115, 198, 112, 206,
  97, 20, 108, 253, 216, 79, 66, 40, 59, 137, 145, 241, 252, 98, 30, 146,
  66, 130, 104, 201, 138, 174, 81, 168, 96, 57, 62, 162, 15, 173, 17, 214,
  137, 84, 155, 178, 57, 186, 120, 89, 27, 108, 26, 92, 189, 28, 111, 95,
  48, 68, 158, 92, 162, 85, 5, 153, 34, 188, 12, 175, 107, 139, 140, 195,
  82, 247, 72, 249, 196, 219, 165, 169, 177, 199, 25, 158, 203, 84, 132, 202,
  127, 33, 212, 186, 150, 175, 120, 10, 75, 36, 163, 156, 63, 35, 132, 172,
  220, 240, 232, 20, 192, 16, 82, 140, 40, 225, 128, 172, 220, 84, 173, 55,
  181, 110, 85, 232, 12, 88, 122, 248, 56, 77, 147, 75, 24, 53, 199, 101,
  114, 168, 177, 114, 113, 107, 243, 111, 255, 243, 255, 106, 205, 175, 161, 201,
  222, 90, 201, 255, 112, 175, 120, 214, 106, 139, 206, 90, 255, 48, 139, 212,
  26, 66, 234, 85, 147, 227, 233, 174, 113, 252, 160, 236, 173, 56, 243, 225,
  222, 102, 114, 189, 194, 96, 68, 247, 223, 208, 182, 243, 228, 89, 116, 197,
  134, 117, 9, 14, 99, 171, 215, 254, 246, 239, 255, 225, 248, 245, 47, 90,
  103, 75, 214, 79, 123, 162, 146, 91, 97, 67, 189, 208, 91, 59, 159, 69,
  34, 186, 162, 214, 201, 186, 32, 7, 114, 140, 66, 172, 81, 14, 84, 93,
  39, 49, 1, 63, 47, 110, 153, 124, 1, 112, 188, 49, 98, 194, 221, 167,
  59, 36, 95, 0, 146, 75, 43, 29, 228, 91, 186, 21, 114, 7, 144, 194,
  236, 129, 252, 114, 16, 191, 112, 236, 15, 124, 19, 43, 66, 224, 9, 26,
  147, 30, 85, 107, 248, 119, 152, 69, 81, 73, 53, 79, 113, 218, 219, 23,
  37, 129, 14, 158, 66, 180, 69, 43, 10, 161, 250, 199, 11, 149, 239, 99,
  68, 55, 232, 152, 79, 118, 168, 36, 46, 146, 40, 80, 118, 140, 196, 23,
  28, 42, 244, 168, 141, 31, 37, 139, 232, 128, 38, 14, 226, 215, 162, 194,
  188, 74, 203, 183, 168, 86, 174, 50, 116, 84, 203, 91, 211, 228, 43, 49,
  58, 151, 170, 141, 178, 166, 206, 146, 100, 104, 136, 90, 181, 2, 251, 225,
  145, 104, 46, 133, 118, 26, 150, 0, 67, 26, 124, 179, 43, 245, 133, 210,
  250, 228, 200, 80, 105, 254, 244, 1, 179, 79, 39, 179, 130, 246, 197, 242,
  32, 124, 127, 18, 254, 78, 185, 159, 14, 40, 116, 22, 160, 228, 95, 139,
  4, 59, 24, 87, 58, 0, 15, 231, 116, 17, 69, 191, 68, 174, 88, 7,
  118, 28, 56, 166, 51, 1, 175, 134, 44, 164, 75, 196, 50, 111, 1, 144,
  231, 79, 194, 180, 126, 119, 227, 37, 73, 108, 16, 235, 166, 24, 95, 36,
  196, 185, 8, 119, 44, 146, 197, 218, 34, 111, 226, 93, 224, 3, 112, 18,
  9, 110, 103, 106, 26, 91, 43, 33, 148, 27, 94, 32, 218, 85, 61, 19,
  144, 50, 17, 89, 192, 10, 97, 236, 7, 200, 111, 224, 153, 176, 208, 42,
  103, 129, 33, 177, 235, 135, 192, 175, 208, 153, 16, 208, 106, 98, 65, 160,
  217, 218, 88, 52, 106, 248, 204, 200, 115, 22, 14, 69, 228, 209, 204, 24,
  63, 215, 215, 163, 200, 45, 118, 113, 90, 13, 28, 178, 132, 222, 6, 181,
  166, 71, 153, 73, 28, 237, 38, 190, 188, 38, 102, 20, 128, 111, 125, 71,
  10, 191, 218, 71, 14, 205, 80, 119, 150, 125, 197, 59, 39, 190, 240, 161,
  68, 33, 252, 147, 252, 112, 104, 132, 131, 123, 154, 228, 159, 118, 172, 155,
  152, 73, 94, 167, 130, 77, 101, 152, 106, 154, 102, 169, 166, 105, 148, 250,
  167, 121, 145, 177, 48, 180, 212, 227, 228, 146, 71, 71, 47, 185, 222, 41,
  78, 68, 184, 51, 188, 249, 6, 236, 101, 97, 94, 151, 54, 167, 114, 33,
  132, 150, 35, 20, 67, 88, 178, 239, 24, 89, 121, 120, 118, 9, 77, 153,
  168, 202, 193, 145, 5, 9, 225, 81, 89, 14, 176, 92, 107, 224, 55, 195,
  134, 228, 194, 118, 149, 147, 169, 10, 23, 39, 143, 5, 11, 36, 56, 21,
  33, 176, 158, 50, 166, 93, 202, 137, 57, 143, 117, 45, 39, 101, 209, 52,
  244, 128, 174, 148, 101, 48, 112, 83, 238, 216, 201, 159, 187, 20, 97, 250,
  222, 170, 151, 150, 64, 174, 164, 198, 213, 50, 233, 238, 201, 33, 85, 237,
  21, 28, 72, 48, 155, 7, 236, 41, 69, 51, 148, 46, 69, 85, 215, 120,
  57, 134, 231, 209, 12, 69, 93, 67, 8, 210, 183, 252, 129, 165, 250, 130,
  217, 200, 3, 183, 232, 143, 0, 72, 61, 168, 120, 20, 75, 5, 248, 223,
  213, 66, 252, 147, 157, 64, 71, 148, 92, 54, 10, 64, 13, 219, 249, 21,
  145, 226, 69, 235, 75, 61, 244, 43, 60, 244, 120, 68, 114, 124, 77, 134,
  75, 153, 112, 166, 191, 116, 214, 164, 240, 254, 250, 155, 26, 242, 102, 142,
  29, 216, 117, 42, 34, 247, 155, 142, 182, 214, 11, 110, 184, 249, 198, 215,
  240, 58, 142, 218, 153, 232, 239, 48, 64, 115, 71, 250, 67, 29, 74, 205,
  53, 159, 107, 128, 98, 230, 19, 13, 42, 212, 128, 249, 10, 17, 118, 1,
  59, 101, 31, 208, 66, 163, 38, 17, 33, 193, 119, 198, 44, 250, 246, 168,
  56, 218, 71, 117, 203, 119, 138, 173, 1, 208, 206, 159, 139, 250, 197, 65,
  116, 122, 118, 26, 214, 55, 122, 205, 94, 183, 219, 236, 110, 172, 53, 209,
  134, 218, 32, 184, 86, 153, 222, 218, 118, 115, 115, 11, 255, 159, 23, 41,
  115, 132, 231, 119, 74, 114, 215, 161, 179, 104, 157, 255, 162, 70, 146, 209,
  136, 126, 91, 119, 254, 93, 239, 228, 83, 143, 219, 175, 96, 17, 128, 248,
  248, 53, 217, 39, 76, 30, 129, 244, 119, 71, 143, 95, 82, 59, 175, 68,
  59, 134, 40, 231, 48, 61, 132, 46, 0, 171, 24, 134, 62, 232, 223, 246,
  122, 131, 141, 13, 102, 140, 66, 105, 3, 133, 215, 65, 103, 212, 125, 208,
  11, 107, 203, 204, 13, 180, 212, 125, 22, 143, 167, 213, 249, 251, 123, 77,
  124, 31, 147, 139, 52, 55, 90, 40, 127, 145, 141, 130, 19, 21, 133, 148,
  21, 70, 196, 53, 238, 20, 65, 141, 61, 6, 218, 20, 221, 229, 148, 161,
  85, 132, 154, 121, 29, 93, 177, 248, 13, 229, 0, 240, 110, 89, 196, 102,
  245, 200, 219, 50, 97, 145, 7, 89, 246, 78, 15, 110, 222, 109, 242, 223,
  52, 8, 117, 10, 86, 76, 8, 55, 124, 53, 159, 47, 168, 201, 59, 232,
  86, 189, 148, 245, 120, 89, 194, 225, 123, 222, 103, 215, 8, 236, 148, 125,
  238, 41, 75, 171, 8, 239, 56, 161, 75, 107, 7, 173, 84, 34, 149, 163,
  66, 201, 99, 203, 178, 169, 87, 131, 74, 59, 110, 158, 168, 12, 85, 189,
  14, 193, 114, 144, 49, 130, 223, 73, 26, 78, 51, 52, 60, 0, 61, 58,
  240, 63, 250, 87, 239, 84, 81, 120, 16, 198, 24, 190, 16, 58, 210, 180,
  187, 99, 51, 11, 82, 168, 224, 23, 162, 65, 133, 121, 110, 138, 118, 143,
  24, 88, 22, 231, 79, 221, 142, 34, 108, 134, 11, 198, 59, 157, 133, 233,
  141, 223, 240, 244, 151, 220, 212, 11, 110, 150, 151, 187, 67, 0, 98, 121,
  110, 8, 233, 174, 193, 207, 120, 255, 198, 237, 36, 55, 238, 60, 13, 175,
  49, 88, 216, 208, 242, 93, 21, 207, 12, 242, 67, 24, 138, 51, 249, 12,
  116, 159, 159, 24, 236, 39, 29, 6, 67, 55, 22, 17, 146, 79, 22, 126,
  1, 194, 109, 92, 199, 120, 132, 221, 70, 123, 22, 226, 11, 31, 105, 94,
  239, 53, 65, 28, 187, 203, 203, 208, 173, 79, 238, 247, 141, 170, 170, 162,
  147, 160, 90, 95, 83, 144, 101, 252, 199, 8, 71, 231, 68, 200, 10, 175,
  233, 78, 46, 198, 104, 11, 175, 51, 104, 234, 58, 243, 120, 202, 77, 135,
  226, 10, 2, 71, 193, 115, 194, 157, 230, 37, 37, 40, 15, 89, 152, 50,
  0, 82, 209, 19, 88, 186, 169, 65, 60, 21, 106, 248, 216, 20, 95, 239,
  237, 155, 67, 66, 224, 64, 241, 206, 19, 43, 131, 76, 178, 21, 220, 139,
  87, 21, 159, 208, 131, 180, 245, 56, 66, 75, 245, 32, 156, 53, 3, 114,
  183, 116, 68, 173, 72, 117, 252, 255, 112, 51, 129, 112, 234, 188, 128, 29,
  188, 42, 203, 139, 0, 196, 0, 189, 193, 43, 100, 51, 58, 171, 195, 247,
  82, 180, 18, 45, 42, 81, 142, 110, 56, 28, 114, 108, 69, 4, 115, 124,
  45, 133, 241, 160, 48, 215, 248, 26, 68, 83, 188, 174, 251, 34, 147, 198,
  117, 32, 66, 147, 55, 107, 15, 95, 120, 134, 70, 97, 89, 158, 166, 74,
  241, 226, 241, 35, 122, 136, 133, 181, 241, 97, 223, 160, 95, 124, 116, 55,
  220, 40, 98, 252, 165, 120, 217, 26, 159, 115, 147, 104, 88, 83, 64, 224,
  67, 193, 192, 55, 131, 29, 157, 137, 112, 56, 164, 152, 149, 36, 125, 71,
  113, 2, 123, 42, 208, 137, 87, 21, 126, 182, 131, 219, 217, 89, 155, 103,
  81, 123, 173, 110, 35, 208, 147, 10, 144, 150, 150, 161, 90, 122, 168, 151,
  7, 113, 81, 228, 180, 244, 28, 26, 50, 88, 2, 122, 166, 24, 95, 212,
  22, 47, 145, 145, 215, 90, 199, 78, 166, 231, 142, 205, 140, 66, 190, 95,
  142, 113, 155, 175, 33, 186, 167, 53, 230, 59, 37, 83, 33, 168, 21, 100,
  126, 52, 36, 17, 88, 213, 114, 250, 206, 5, 2, 63, 255, 223, 4, 32,
  248, 181, 62, 126, 175, 177, 213, 69, 95, 52, 105, 198, 214, 208, 74, 255,
  176, 171, 171, 11, 247, 39, 5, 247, 16, 69, 11, 138, 255, 64, 78, 194,
  133, 246, 168, 152, 31, 184, 137, 77, 149, 222, 133, 15, 38, 25, 1, 1,
  139, 163, 157, 172, 105, 94, 156, 169, 188, 52, 39, 165, 31, 127, 96, 219,
  124, 65, 187, 228, 189, 0, 200, 43, 179, 138, 231, 153, 113, 241, 39, 119,
  175, 244, 192, 134, 231, 132, 172, 183, 153, 118, 43, 7, 221, 228, 240, 187,
  239, 187, 115, 246, 40, 168, 47, 122, 107, 0, 207, 146, 180, 175, 29, 71,
  246, 115, 35, 155, 118, 112, 37, 194, 108, 84, 30, 93, 81, 72, 245, 71,
  129, 12, 242, 224, 158, 11, 219, 227, 93, 250, 196, 15, 117, 218, 88, 80,
  80, 186, 168, 187, 58, 218, 104, 61, 226, 17, 24, 250, 70, 17, 45, 14,
  92, 165, 72, 165, 102, 154, 133, 96, 108, 114, 209, 214, 52, 223, 34, 111,
  220, 15, 92, 151, 224, 161, 68, 20, 0, 139, 119, 202, 43, 214, 0, 188,
  156, 12, 235, 255, 107, 92, 108, 36, 120, 91, 237, 146, 140, 33, 84, 51,
  90, 167, 93, 61, 109, 199, 187, 34, 232, 181, 164, 210, 166, 175, 7, 93,
  99, 65, 216, 240, 11, 115, 226, 217, 18, 72, 69, 239, 1, 144, 162, 236,
  66, 249, 238, 245, 139, 250, 114, 25, 168, 212, 138, 101, 68, 94, 229, 69,
  228, 178, 103, 38, 190, 120, 92, 170, 68, 9, 63, 192, 5, 158, 236, 83,
  64, 184, 152, 188, 162, 222, 127, 40, 243, 21, 170, 26, 151, 167, 143, 127,
  250, 248, 226, 24, 37, 134, 93, 96, 147, 242, 143, 15, 255, 248, 241, 57,
  47, 113, 116, 248, 246, 224, 227, 187, 195, 151, 79, 95, 189, 131, 4, 247,
  249, 27, 124, 224, 13, 249, 243, 164, 162, 185, 122, 161, 16, 188, 47, 126,
  154, 254, 69, 57, 138, 161, 186, 210, 34, 222, 171, 95, 118, 177, 50, 153,
  232, 17, 126, 24, 130, 81, 147, 4, 141, 101, 165, 97, 93, 239, 149, 153,
  235, 104, 249, 209, 148, 90, 87, 219, 213, 142, 152, 250, 128, 175, 28, 135,
  70, 137, 34, 85, 57, 253, 144, 66, 106, 246, 77, 244, 56, 110, 98, 155,
  76, 135, 201, 37, 32, 76, 9, 3, 96, 12, 25, 86, 226, 54, 71, 218,
  233, 93, 188, 227, 48, 109, 111, 151, 147, 160, 161, 60, 210, 52, 173, 86,
  121, 80, 113, 57, 134, 107, 155, 71, 162, 21, 125, 176, 52, 63, 89, 13,
  155, 226, 63, 245, 6, 185, 31, 28, 215, 198, 121, 174, 187, 131, 192, 34,
  85, 126, 139, 160, 200, 62, 231, 51, 77, 205, 99, 210, 236, 51, 62, 159,
  255, 17, 170, 192, 189, 87, 183, 123, 10, 156, 112, 26, 229, 162, 56, 239,
  58, 112, 241, 140, 143, 127, 157, 36, 37, 140, 50, 201, 197, 250, 236, 238,
  30, 145, 128, 51, 150, 148, 254, 251, 141, 134, 223, 97, 240, 173, 244, 97,
  150, 200, 184, 108, 168, 191, 200, 131, 151, 228, 102, 117, 123, 247, 165, 65,
  42, 240, 212, 82, 237, 183, 123, 202, 21, 14, 163, 146, 213, 140, 71, 94,
  90, 81, 68, 213, 62, 72, 239, 143, 152, 28, 101, 170, 68, 160, 173, 178,
  197, 110, 195, 17, 129, 143, 244, 61, 211, 2, 81, 9, 221, 93, 45, 150,
  92, 148, 106, 173, 110, 73, 131, 30, 141, 93, 203, 245, 41, 238, 133, 227,
  130, 79, 0, 249, 8, 66, 226, 172, 132, 34, 30, 93, 200, 192, 161, 235,
  208, 68, 95, 62, 60, 68, 169, 88, 104, 116, 178, 116, 43, 233, 210, 173,
  38, 76, 183, 138, 50, 255, 159, 99, 136, 82, 25, 139, 71, 174, 108, 144,
  191, 14, 163, 20, 207, 132, 189, 74, 18, 250, 53, 100, 142, 223, 182, 244,
  139, 240, 138, 173, 241, 124, 226, 169, 161, 57, 65, 120, 107, 93, 204, 134,
  158, 90, 220, 79, 193, 91, 33, 166, 215, 167, 212, 75, 151, 132, 168, 160,
  105, 147, 112, 80, 31, 8, 90, 124, 56, 80, 146, 121, 110, 61, 173, 230,
  123, 214, 4, 42, 123, 222, 51, 129, 186, 124, 141, 187, 17, 206, 98, 132,
  195, 123, 29, 19, 60, 0, 3, 60, 127, 8, 34, 116, 211, 38, 188, 250,
  132, 220, 123, 13, 67, 187, 16, 185, 127, 33, 210, 239, 53, 204, 205, 66,
  222, 235, 196, 106, 105, 69, 196, 12, 41, 205, 35, 35, 235, 78, 182, 51,
  116, 62, 177, 82, 160, 181, 66, 142, 47, 178, 194, 226, 5, 251, 35, 12,
  64, 91, 47, 86, 100, 242, 9, 198, 157, 21, 29, 250, 169, 221, 186, 111,
  121, 134, 221, 252, 103, 51, 93, 198, 223, 104, 6, 34, 70, 213, 83, 238,
  1, 33, 78, 33, 249, 59, 139, 34, 173, 86, 251, 18, 117, 158, 47, 241,
  158, 40, 59, 166, 103, 177, 174, 54, 230, 153, 116, 106, 63, 201, 28, 47,
  102, 159, 158, 142, 231, 229, 11, 197, 175, 154, 49, 99, 211, 228, 198, 49,
  88, 13, 214, 232, 29, 196, 142, 117, 205, 93, 40, 183, 19, 179, 142, 168,
  244, 91, 85, 9, 234, 111, 122, 106, 23, 86, 104, 97, 192, 30, 143, 13,
  163, 53, 218, 172, 241, 37, 21, 145, 59, 153, 184, 185, 159, 202, 2, 181,
  250, 49, 147, 189, 113, 177, 17, 106, 78, 86, 210, 147, 77, 217, 143, 174,
  93, 209, 233, 132, 15, 205, 162, 19, 89, 86, 217, 9, 219, 47, 91, 250,
  1, 44, 17, 178, 167, 8, 208, 3, 236, 140, 94, 110, 24, 86, 212, 118,
  222, 181, 25, 129, 95, 118, 17, 110, 15, 20, 243, 7, 89, 90, 2, 245,
  157, 53, 40, 219, 189, 199, 238, 148, 203, 137, 184, 92, 32, 34, 108, 85,
  197, 13, 178, 181, 122, 109, 230, 121, 103, 6, 164, 223, 161, 169, 27, 63,
  121, 80, 90, 76, 241, 217, 241, 130, 140, 195, 240, 186, 248, 212, 226, 26,
  13, 157, 185, 246, 141, 38, 7, 208, 31, 66, 97, 132, 130, 171, 200, 107,
  104, 44, 162, 202, 220, 6, 252, 121, 94, 254, 241, 169, 66, 128, 170, 82,
  149, 135, 51, 192, 79, 147, 25, 137, 191, 50, 89, 230, 185, 48, 87, 70,
  183, 26, 155, 182, 246, 31, 215, 42, 168, 118, 205, 66, 224, 70, 80, 145,
  49, 68, 237, 93, 168, 168, 115, 79, 133, 92, 31, 240, 48, 22, 63, 241,
  7, 73, 201, 212, 146, 217, 78, 242, 238, 211, 40, 87, 111, 201, 245, 82,
  9, 203, 118, 155, 95, 116, 204, 124, 219, 113, 189, 44, 172, 217, 190, 178,
  52, 200, 118, 164, 58, 222, 12, 93, 5, 117, 243, 8, 108, 67, 59, 109,
  135, 148, 126, 135, 222, 185, 238, 227, 29, 181, 89, 56, 237, 27, 14, 245,
  184, 184, 131, 56, 24, 178, 225, 139, 8, 117, 8, 14, 193, 155, 31, 94,
  209, 229, 140, 171, 183, 246, 245, 61, 217, 241, 93, 89, 221, 55, 95, 248,
  19, 200, 138, 52, 234, 26, 138, 236, 14, 94, 105, 234, 244, 52, 26, 115,
  123, 25, 149, 155, 37, 151, 245, 46, 244, 162, 165, 95, 43, 232, 26, 243,
  193, 232, 3, 199, 135, 158, 21, 246, 148, 209, 250, 193, 31, 37, 94, 16,
  167, 193, 131, 125, 93, 53, 33, 59, 252, 189, 120, 213, 222, 223, 175, 174,
  158, 161, 215, 168, 232, 4, 231, 144, 251, 116, 194, 53, 36, 34, 23, 20,
  208, 57, 59, 20, 144, 4, 95, 20, 221, 43, 218, 104, 233, 181, 0, 223,
  14, 96, 92, 121, 120, 25, 135, 83, 10, 165, 22, 13, 206, 179, 250, 32,
  191, 122, 154, 134, 151, 56, 143, 133, 153, 2, 87, 29, 84, 134, 216, 116,
  136, 127, 102, 113, 146, 191, 195, 35, 126, 169, 30, 161, 248, 160, 181, 133,
  0, 64, 23, 55, 8, 89, 249, 133, 76, 192, 143, 245, 100, 10, 191, 233,
  71, 209, 255, 95, 3, 166, 220, 122, 211, 93, 15, 110, 191, 130, 141, 19,
  136, 98, 57, 120, 16, 218, 252, 125, 227, 212, 212, 126, 196, 35, 119, 134,
  182, 230, 53, 122, 161, 159, 155, 20, 151, 138, 28, 249, 103, 126, 175, 92,
  229, 8, 250, 64, 186, 27, 217, 74, 211, 253, 244, 246, 30, 149, 192, 237,
  123, 94, 213, 24, 178, 12, 247, 57, 79, 194, 140, 169, 131, 82, 78, 209,
  71, 69, 127, 123, 6, 173, 17, 144, 202, 146, 3, 100, 184, 188, 168, 193,
  4, 45, 101, 171, 227, 218, 80, 68, 163, 186, 208, 147, 67, 219, 52, 90,
  213, 176, 243, 26, 50, 143, 103, 225, 128, 15, 244, 250, 150, 27, 142, 254,
  108, 34, 66, 72, 233, 3, 39, 219, 110, 25, 98, 67, 61, 204, 41, 185,
  105, 193, 222, 103, 87, 129, 247, 236, 128, 36, 24, 190, 13, 210, 232, 34,
  70, 3, 195, 110, 18, 75, 173, 42, 40, 40, 4, 162, 70, 163, 92, 251,
  34, 190, 126, 70, 30, 91, 184, 96, 225, 254, 197, 183, 227, 40, 24, 205,
  157, 98, 250, 188, 210, 216, 69, 223, 68, 120, 140, 109, 104, 103, 34, 248,
  72, 21, 213, 51, 180, 184, 73, 148, 220, 129, 9, 175, 168, 198, 59, 225,
  72, 164, 3, 105, 3, 237, 231, 3, 6, 114, 19, 230, 42, 117, 163, 96,
  37, 114, 37, 67, 49, 209, 158, 176, 48, 155, 167, 20, 134, 177, 30, 55,
  132, 223, 21, 136, 29, 223, 204, 68, 6, 120, 201, 216, 144, 25, 114, 217,
  196, 225, 7, 67, 46, 52, 53, 214, 241, 33, 191, 143, 230, 130, 39, 215,
  88, 130, 233, 48, 123, 77, 93, 33, 215, 153, 92, 103, 48, 3, 165, 6,
  119, 157, 241, 190, 101, 99, 34, 44, 129, 151, 76, 6, 107, 158, 97, 41,
  193, 201, 77, 27, 229, 70, 163, 225, 57, 165, 97, 51, 159, 139, 218, 128,
  69, 113, 93, 141, 170, 216, 83, 175, 74, 252, 60, 247, 194, 150, 155, 30,
  129, 5, 18, 19, 127, 216, 37, 44, 26, 28, 6, 159, 27, 178, 216, 251,
  232, 67, 195, 14, 69, 140, 201, 185, 142, 22, 154, 132, 72, 54, 170, 90,
  54, 226, 88, 194, 15, 223, 91, 114, 199, 55, 199, 232, 176, 122, 49, 143,
  131, 50, 31, 198, 176, 46, 74, 30, 143, 109, 238, 142, 27, 187, 123, 247,
  100, 237, 226, 212, 143, 143, 56, 135, 221, 52, 231, 85, 223, 66, 97, 25,
  47, 56, 110, 67, 170, 203, 3, 131, 106, 37, 150, 235, 160, 40, 111, 120,
  121, 254, 170, 125, 163, 48, 182, 207, 30, 181, 47, 84, 112, 130, 11, 76,
  241, 62, 65, 191, 196, 125, 35, 75, 207, 229, 45, 75, 219, 84, 65, 14,
  142, 7, 63, 75, 36, 61, 6, 170, 11, 13, 134, 126, 161, 125, 69, 198,
  37, 224, 122, 140, 199, 159, 116, 18, 77, 23, 41, 219, 74, 16, 44, 167,
  193, 99, 83, 111, 232, 73, 231, 9, 233, 73, 0, 182, 68, 129, 80, 37,
  49, 6, 39, 44, 176, 93, 60, 26, 225, 105, 149, 167, 34, 101, 35, 224,
  60, 9, 71, 239, 124, 201, 147, 14, 111, 112, 16, 126, 200, 161, 159, 43,
  195, 48, 230, 120, 100, 216, 161, 51, 19, 59, 100, 139, 142, 143, 167, 7,
  157, 54, 30, 184, 35, 84, 222, 113, 180, 75, 96, 25, 143, 89, 194, 56,
  154, 33, 112, 183, 254, 96, 237, 197, 49, 45, 94, 151, 39, 154, 191, 55,
  88, 130, 102, 175, 103, 186, 72, 254, 208, 23, 193, 137, 80, 114, 37, 119,
  16, 88, 21, 23, 130, 198, 165, 98, 230, 12, 97, 198, 30, 193, 182, 125,
  31, 195, 2, 234, 14, 207, 92, 153, 53, 79, 222, 38, 44, 15, 171, 174,
  75, 152, 222, 208, 79, 233, 185, 238, 226, 125, 142, 140, 125, 233, 193, 152,
  54, 157, 201, 136, 162, 190, 161, 187, 136, 219, 35, 35, 197, 113, 154, 190,
  150, 170, 15, 85, 22, 95, 80, 208, 137, 46, 49, 130, 29, 246, 49, 96,
  34, 139, 170, 111, 116, 197, 238, 249, 99, 250, 202, 178, 210, 199, 231, 145,
  107, 251, 116, 150, 154, 207, 178, 146, 138, 110, 188, 232, 121, 3, 94, 81,
  248, 10, 75, 47, 97, 216, 202, 237, 150, 58, 174, 211, 40, 24, 62, 201,
  36, 164, 7, 49, 11, 83, 114, 19, 71, 207, 101, 221, 47, 186, 105, 122,
  66, 187, 194, 128, 68, 236, 17, 63, 46, 127, 156, 166, 225, 53, 76, 60,
  250, 91, 23, 150, 45, 204, 111, 20, 23, 124, 232, 187, 56, 29, 209, 18,
  97, 98, 189, 167, 231, 238, 245, 68, 245, 136, 51, 12, 167, 122, 56, 173,
  198, 243, 106, 98, 169, 48, 42, 28, 201, 66, 90, 29, 241, 242, 176, 40,
  71, 191, 37, 51, 7, 183, 31, 60, 209, 68, 102, 33, 87, 215, 139, 222,
  145, 16, 202, 252, 87, 254, 248, 12, 168, 103, 109, 83, 108, 221, 237, 160,
  222, 92, 75, 42, 99, 215, 88, 193, 162, 154, 74, 16, 32, 34, 134, 144,
  145, 56, 16, 226, 18, 63, 247, 16, 132, 203, 82, 209, 109, 123, 29, 242,
  206, 85, 80, 152, 94, 243, 206, 238, 42, 122, 181, 241, 50, 243, 11, 73,
  38, 217, 154, 203, 49, 73, 202, 111, 16, 189, 7, 228, 21, 156, 15, 109,
  76, 175, 215, 195, 230, 41, 105, 201, 97, 27, 3, 200, 156, 122, 188, 73,
  212, 58, 205, 1, 105, 235, 115, 251, 194, 179, 195, 17, 239, 32, 85, 90,
  181, 26, 222, 104, 55, 39, 69, 35, 239, 59, 31, 228, 170, 225, 147, 219,
  122, 73, 129, 149, 233, 18, 131, 53, 235, 18, 168, 79, 65, 230, 219, 205,
  99, 190, 108, 18, 151, 42, 23, 33, 172, 250, 213, 157, 132, 92, 13, 157,
  60, 110, 120, 200, 150, 170, 200, 249, 250, 86, 252, 128, 108, 2, 242, 85,
  74, 45, 9, 81, 174, 23, 33, 245, 11, 216, 133, 47, 225, 35, 45, 181,
  111, 158, 53, 21, 119, 193, 38, 97, 52, 61, 152, 26, 219, 6, 121, 166,
  100, 53, 199, 147, 75, 64, 28, 11, 223, 119, 253, 40, 167, 0, 222, 210,
  136, 239, 53, 171, 31, 187, 214, 16, 189, 182, 214, 134, 71, 178, 226, 133,
  253, 115, 70, 49, 129, 233, 242, 81, 119, 212, 219, 94, 123, 80, 115, 11,
  194, 76, 141, 139, 98, 219, 235, 225, 218, 233, 150, 175, 88, 194, 223, 99,
  251, 238, 70, 46, 67, 183, 179, 43, 88, 116, 96, 171, 49, 105, 205, 163,
  79, 42, 48, 168, 90, 230, 212, 102, 214, 191, 135, 217, 237, 236, 68, 15,
  119, 249, 242, 183, 131, 91, 123, 175, 87, 51, 202, 55, 154, 75, 109, 174,
  243, 137, 15, 212, 39, 190, 175, 71, 171, 188, 58, 236, 195, 220, 48, 40,
  186, 216, 18, 184, 240, 189, 138, 249, 60, 201, 173, 171, 141, 254, 100, 108,
  170, 175, 143, 220, 29, 199, 23, 111, 167, 103, 124, 131, 124, 196, 70, 6,
  127, 16, 67, 235, 121, 168, 22, 172, 235, 91, 72, 11, 61, 152, 210, 189,
  70, 25, 244, 55, 226, 10, 145, 14, 149, 167, 161, 90, 210, 43, 169, 117,
  146, 204, 172, 58, 152, 98, 213, 144, 54, 106, 200, 125, 146, 228, 121, 50,
  41, 235, 133, 200, 133, 234, 107, 61, 195, 116, 165, 148, 153, 239, 131, 110,
  123, 189, 225, 223, 35, 22, 246, 69, 130, 105, 164, 33, 70, 235, 22, 70,
  202, 82, 96, 206, 26, 126, 209, 170, 101, 80, 189, 101, 80, 169, 225, 2,
  122, 46, 111, 96, 233, 144, 132, 166, 211, 210, 137, 213, 50, 41, 225, 221,
  202, 191, 142, 105, 42, 87, 218, 106, 251, 62, 163, 82, 95, 159, 230, 194,
  194, 212, 47, 196, 192, 66, 123, 174, 80, 61, 224, 243, 143, 74, 209, 212,
  12, 188, 50, 59, 188, 42, 178, 45, 139, 175, 40, 114, 229, 51, 86, 234,
  57, 184, 186, 200, 43, 228, 230, 232, 185, 123, 14, 68, 90, 241, 141, 70,
  202, 31, 52, 202, 107, 2, 229, 78, 66, 227, 218, 6, 89, 215, 70, 115,
  85, 104, 223, 100, 6, 52, 54, 95, 98, 6, 159, 178, 179, 104, 250, 58,
  196, 187, 88, 59, 197, 180, 78, 46, 216, 73, 82, 215, 216, 167, 25, 92,
  107, 249, 120, 195, 212, 204, 23, 61, 17, 3, 163, 151, 229, 114, 185, 222,
  40, 127, 233, 87, 8, 156, 247, 209, 7, 31, 130, 120, 19, 246, 113, 28,
  157, 33, 59, 213, 82, 236, 84, 173, 172, 24, 90, 115, 197, 165, 104, 244,
  87, 55, 227, 140, 155, 11, 0, 151, 88, 92, 101, 53, 103, 201, 150, 102,
  8, 211, 134, 173, 21, 244, 176, 95, 134, 32, 213, 94, 63, 227, 28, 223,
  230, 246, 29, 21, 187, 162, 68, 141, 189, 42, 70, 76, 208, 174, 94, 39,
  143, 7, 125, 133, 131, 161, 147, 107, 34, 14, 158, 162, 238, 93, 7, 241,
  170, 169, 113, 135, 59, 134, 87, 77, 141, 57, 43, 135, 77, 139, 51, 95,
  218, 99, 50, 21, 71, 195, 171, 198, 47, 221, 113, 131, 129, 20, 50, 177,
  96, 164, 225, 213, 98, 86, 26, 48, 122, 122, 96, 25, 94, 202, 147, 217,
  82, 140, 100, 208, 18, 58, 215, 43, 163, 30, 197, 179, 71, 81, 50, 159,
  70, 57, 113, 191, 239, 56, 19, 51, 165, 116, 247, 45, 184, 22, 4, 190,
  250, 126, 197, 41, 180, 176, 219, 38, 2, 206, 178, 67, 11, 77, 211, 148,
  77, 189, 202, 147, 136, 107, 238, 30, 102, 70, 8, 40, 86, 3, 39, 25,
  23, 4, 235, 220, 82, 202, 43, 55, 153, 163, 227, 166, 3, 98, 158, 84,
  62, 136, 70, 134, 146, 111, 86, 34, 23, 181, 122, 42, 191, 71, 218, 231,
  155, 162, 166, 17, 205, 5, 183, 185, 125, 181, 147, 211, 243, 236, 37, 81,
  15, 35, 98, 81, 234, 234, 89, 146, 114, 71, 127, 113, 98, 243, 21, 38,
  149, 88, 76, 0, 242, 91, 17, 220, 73, 132, 157, 55, 132, 96, 189, 206,
  195, 228, 10, 21, 149, 219, 229, 10, 21, 85, 194, 118, 86, 180, 98, 235,
  42, 37, 5, 145, 162, 68, 78, 200, 16, 2, 84, 70, 4, 20, 128, 189,
  126, 97, 84, 32, 219, 196, 183, 189, 222, 112, 141, 249, 101, 188, 185, 27,
  112, 159, 154, 215, 132, 160, 58, 76, 162, 198, 138, 20, 217, 160, 145, 210,
  91, 36, 126, 141, 208, 227, 226, 54, 48, 27, 238, 58, 175, 40, 241, 214,
  132, 157, 64, 17, 101, 150, 55, 118, 247, 110, 156, 128, 154, 223, 204, 232,
  182, 38, 62, 250, 88, 97, 128, 133, 92, 114, 217, 144, 214, 143, 188, 205,
  47, 227, 23, 110, 222, 55, 22, 62, 210, 227, 222, 240, 42, 171, 18, 214,
  4, 211, 228, 172, 85, 197, 87, 223, 123, 69, 181, 161, 166, 152, 172, 132,
  24, 183, 10, 78, 90, 213, 248, 232, 123, 157, 139, 76, 114, 224, 117, 1,
  234, 68, 227, 198, 89, 234, 80, 241, 144, 93, 164, 247, 183, 164, 183, 133,
  179, 234, 65, 201, 219, 146, 231, 98, 130, 5, 139, 160, 35, 197, 199, 208,
  124, 170, 217, 157, 138, 196, 118, 158, 89, 158, 246, 46, 179, 143, 145, 208,
  124, 70, 215, 141, 154, 126, 148, 194, 11, 91, 153, 146, 57, 44, 71, 139,
  242, 211, 16, 182, 124, 239, 215, 155, 235, 31, 26, 139, 39, 134, 214, 158,
  54, 175, 216, 6, 123, 192, 78, 107, 119, 96, 117, 107, 32, 198, 150, 210,
  81, 50, 243, 100, 73, 77, 255, 88, 106, 16, 74, 250, 252, 193, 241, 94,
  211, 186, 71, 51, 205, 63, 70, 148, 101, 93, 226, 40, 113, 28, 69, 38,
  22, 34, 178, 238, 66, 176, 29, 71, 61, 6, 7, 63, 193, 61, 130, 108,
  33, 201, 139, 2, 97, 58, 32, 58, 142, 175, 155, 1, 236, 157, 59, 66,
  129, 125, 125, 72, 55, 184, 203, 113, 42, 131, 103, 91, 82, 156, 48, 46,
  101, 18, 180, 219, 222, 168, 130, 88, 111, 148, 58, 178, 138, 114, 41, 35,
  223, 250, 186, 87, 73, 224, 234, 65, 91, 173, 125, 252, 199, 142, 157, 95,
  172, 96, 226, 151, 231, 60, 7, 31, 96, 64, 241, 208, 148, 58, 71, 213,
  25, 39, 11, 17, 45, 50, 158, 2, 192, 35, 170, 80, 231, 245, 208, 81,
  217, 57, 141, 17, 32, 241, 52, 198, 52, 211, 11, 4, 133, 161, 190, 228,
  106, 39, 46, 26, 167, 60, 112, 184, 63, 227, 41, 63, 1, 56, 156, 142,
  208, 31, 239, 218, 233, 191, 176, 251, 203, 37, 69, 88, 252, 173, 101, 86,
  36, 219, 118, 116, 81, 7, 159, 35, 182, 159, 3, 187, 219, 50, 228, 11,
  180, 167, 71, 172, 214, 227, 146, 139, 53, 197, 146, 119, 69, 192, 234, 224,
  161, 234, 183, 61, 43, 3, 157, 34, 88, 118, 199, 147, 45, 99, 22, 206,
  48, 76, 158, 56, 234, 232, 139, 95, 109, 124, 246, 65, 28, 89, 136, 20,
  119, 90, 154, 167, 150, 38, 231, 222, 150, 172, 13, 98, 108, 177, 253, 114,
  214, 26, 135, 83, 216, 163, 62, 71, 105, 112, 136, 155, 145, 144, 146, 235,
  236, 66, 242, 37, 133, 177, 224, 87, 177, 222, 176, 44, 137, 161, 32, 63,
  19, 44, 190, 120, 24, 178, 103, 83, 247, 152, 93, 169, 211, 244, 244, 24,
  231, 219, 236, 189, 2, 108, 223, 193, 227, 25, 101, 247, 241, 85, 176, 69,
  19, 27, 251, 230, 153, 136, 179, 200, 159, 123, 32, 118, 20, 9, 168, 135,
  148, 197, 96, 148, 151, 56, 233, 16, 115, 55, 96, 23, 109, 241, 134, 52,
  143, 226, 243, 117, 98, 48, 81, 222, 31, 57, 248, 60, 153, 15, 198, 140,
  2, 195, 23, 95, 197, 73, 89, 145, 134, 177, 247, 101, 205, 62, 33, 198,
  63, 252, 208, 127, 186, 55, 244, 159, 116, 232, 63, 217, 208, 175, 200, 217,
  153, 99, 209, 162, 238, 3, 180, 145, 67, 151, 107, 85, 236, 39, 89, 12,
  118, 112, 230, 248, 92, 209, 83, 26, 52, 48, 186, 106, 7, 195, 117, 69,
  175, 99, 56, 57, 63, 168, 68, 101, 106, 132, 194, 215, 14, 24, 220, 221,
  81, 198, 158, 39, 195, 0, 194, 149, 58, 55, 38, 177, 98, 76, 17, 74,
  5, 20, 76, 88, 133, 105, 169, 228, 167, 178, 152, 226, 121, 12, 84, 93,
  191, 179, 0, 240, 208, 125, 38, 136, 161, 140, 64, 232, 5, 162, 21, 196,
  176, 150, 153, 22, 231, 223, 46, 63, 142, 134, 160, 189, 36, 113, 30, 205,
  234, 214, 121, 33, 159, 142, 117, 207, 41, 98, 249, 206, 87, 120, 113, 8,
  42, 233, 103, 30, 70, 18, 41, 220, 110, 164, 12, 187, 38, 63, 133, 249,
  1, 152, 1, 29, 53, 235, 87, 5, 24, 109, 100, 27, 150, 115, 153, 61,
  204, 13, 82, 36, 186, 184, 161, 51, 221, 46, 100, 32, 78, 90, 21, 233,
  98, 194, 130, 245, 209, 146, 16, 162, 252, 255, 207, 0, 150, 120, 13, 139,
  97, 164, 47, 152, 77, 47, 14, 78, 222, 28, 238, 127, 132, 63, 143, 205,
  192, 7, 104, 115, 233, 215, 106, 218, 19, 48, 93, 117, 66, 207, 11, 194,
  212, 255, 113, 54, 99, 233, 126, 152, 97, 112, 41, 255, 59, 76, 122, 80,
  95, 114, 111, 225, 67, 131, 158, 52, 164, 24, 168, 20, 181, 70, 202, 18,
  226, 100, 95, 15, 248, 208, 55, 22, 38, 179, 205, 187, 12, 175, 142, 153,
  231, 177, 143, 187, 12, 114, 5, 168, 138, 161, 246, 215, 242, 141, 171, 105,
  133, 196, 81, 43, 60, 39, 124, 195, 224, 61, 96, 63, 225, 33, 54, 101,
  72, 25, 73, 225, 69, 206, 109, 170, 28, 221, 44, 255, 244, 221, 141, 74,
  80, 39, 131, 132, 144, 225, 153, 211, 192, 43, 57, 148, 140, 28, 68, 74,
  78, 237, 246, 83, 27, 48, 157, 212, 75, 159, 81, 200, 197, 179, 229, 120,
  55, 208, 114, 12, 214, 24, 70, 178, 46, 57, 44, 185, 226, 168, 233, 8,
  183, 166, 55, 48, 192, 163, 42, 71, 158, 166, 255, 254, 162, 214, 116, 113,
  104, 223, 44, 92, 213, 125, 216, 248, 46, 15, 185, 74, 67, 50, 5, 78,
  154, 50, 114, 216, 161, 176, 26, 92, 231, 64, 75, 214, 52, 63, 192, 32,
  171, 211, 188, 8, 128, 216, 88, 172, 140, 100, 227, 228, 82, 138, 141, 155,
  64, 152, 141, 21, 27, 52, 21, 173, 203, 38, 156, 129, 145, 60, 155, 255,
  99, 95, 170, 7, 50, 229, 167, 190, 210, 4, 170, 34, 48, 135, 121, 14,
  234, 190, 192, 231, 57, 41, 163, 105, 225, 196, 244, 69, 42, 168, 230, 151,
  86, 170, 82, 194, 20, 20, 141, 34, 109, 25, 15, 38, 255, 85, 117, 98,
  247, 14, 115, 104, 180, 217, 88, 16, 89, 188, 92, 217, 253, 181, 44, 89,
  197, 123, 15, 141, 95, 108, 249, 210, 23, 15, 62, 1, 220, 231, 146, 39,
  201, 60, 163, 119, 92, 233, 162, 156, 34, 114, 99, 137, 170, 164, 13, 187,
  85, 241, 148, 120, 22, 102, 89, 116, 193, 248, 102, 221, 60, 86, 173, 68,
  132, 198, 25, 157, 214, 180, 241, 94, 26, 21, 124, 249, 163, 164, 170, 154,
  66, 33, 10, 36, 211, 155, 147, 123, 114, 186, 115, 0, 147, 247, 13, 7,
  77, 180, 195, 162, 99, 198, 65, 154, 98, 12, 245, 23, 17, 116, 114, 122,
  22, 60, 125, 245, 34, 96, 92, 166, 244, 249, 158, 173, 214, 240, 89, 104,
  53, 19, 57, 15, 196, 254, 46, 13, 113, 141, 145, 59, 189, 253, 34, 7,
  86, 6, 59, 9, 54, 25, 73, 6, 18, 165, 94, 35, 233, 83, 107, 120,
  3, 146, 185, 208, 27, 158, 22, 23, 190, 79, 143, 54, 139, 228, 92, 94,
  42, 119, 124, 155, 46, 128, 177, 49, 96, 189, 154, 105, 153, 116, 163, 155,
  224, 196, 228, 105, 207, 67, 17, 92, 202, 241, 230, 86, 245, 219, 226, 210,
  73, 86, 231, 175, 108, 139, 249, 128, 14, 81, 170, 136, 244, 181, 51, 102,
  123, 81, 26, 195, 104, 201, 178, 176, 47, 179, 39, 186, 86, 146, 83, 160,
  17, 56, 73, 109, 25, 143, 75, 203, 89, 168, 241, 233, 42, 158, 86, 175,
  92, 207, 243, 189, 244, 119, 77, 241, 173, 122, 235, 160, 171, 111, 118, 228,
  63, 93, 35, 232, 55, 61, 195, 198, 99, 27, 189, 255, 224, 68, 134, 79,
  243, 55, 220, 125, 79, 15, 46, 107, 167, 111, 151, 164, 139, 64, 179, 198,
  161, 29, 111, 41, 188, 12, 65, 203, 64, 61, 138, 135, 73, 151, 241, 133,
  180, 142, 54, 53, 112, 141, 202, 75, 146, 222, 7, 13, 151, 128, 228, 248,
  50, 99, 152, 103, 122, 116, 65, 254, 132, 93, 139, 8, 128, 168, 179, 207,
  206, 111, 252, 103, 82, 244, 87, 85, 121, 74, 48, 76, 60, 36, 220, 18,
  23, 94, 28, 2, 238, 125, 43, 11, 42, 221, 253, 211, 239, 89, 250, 243,
  95, 114, 118, 248, 180, 15, 138, 154, 204, 190, 253, 36, 212, 129, 194, 255,
  148, 136, 225, 58, 212, 242, 131, 8, 88, 216, 236, 37, 8, 166, 130, 149,
  212, 214, 94, 9, 50, 102, 205, 35, 103, 253, 242, 191, 32, 157, 82, 79,
  44, 63, 246, 66, 216, 145, 143, 88, 147, 16, 213, 71, 70, 58, 179, 223,
  8, 98, 52, 3, 117, 126, 220, 15, 148, 102, 170, 191, 109, 233, 106, 177,
  228, 236, 195, 221, 119, 250, 155, 228, 218, 35, 62, 208, 145, 135, 187, 187,
  244, 55, 164, 119, 80, 223, 84, 51, 245, 57, 182, 64, 215, 148, 122, 97,
  223, 175, 117, 18, 149, 154, 42, 176, 89, 63, 240, 206, 163, 71, 124, 122,
  126, 191, 214, 9, 250, 254, 25, 37, 75, 108, 251, 74, 208, 220, 82, 69,
  224, 11, 227, 83, 114, 95, 84, 125, 53, 36, 25, 251, 205, 55, 48, 36,
  214, 233, 20, 189, 96, 155, 162, 249, 73, 88, 247, 28, 241, 39, 236, 130,
  82, 83, 33, 75, 105, 81, 65, 14, 23, 103, 150, 126, 96, 240, 57, 73,
  39, 125, 208, 98, 109, 24, 213, 70, 72, 171, 82, 190, 41, 245, 205, 119,
  16, 170, 57, 115, 145, 44, 139, 153, 138, 121, 196, 133, 71, 236, 140, 226,
  31, 115, 85, 134, 127, 217, 185, 77, 36, 217, 163, 160, 112, 135, 183, 111,
  137, 217, 203, 166, 4, 167, 37, 57, 101, 196, 44, 181, 151, 41, 124, 222,
  249, 110, 29, 11, 150, 80, 33, 75, 53, 56, 227, 170, 130, 231, 138, 130,
  53, 69, 237, 99, 53, 251, 134, 66, 81, 92, 93, 83, 208, 146, 156, 187,
  10, 28, 198, 2, 23, 91, 81, 136, 59, 216, 2, 143, 235, 126, 181, 53,
  31, 44, 254, 232, 220, 31, 24, 58, 166, 224, 109, 210, 41, 44, 137, 48,
  243, 187, 91, 75, 189, 187, 129, 50, 138, 54, 20, 229, 47, 242, 230, 87,
  207, 133, 232, 36, 202, 243, 222, 21, 26, 180, 53, 162, 170, 184, 80, 232,
  158, 243, 243, 60, 177, 219, 209, 149, 187, 79, 94, 229, 142, 139, 105, 97,
  240, 70, 95, 63, 245, 250, 207, 167, 42, 141, 79, 26, 200, 173, 22, 189,
  22, 120, 165, 53, 137, 30, 35, 166, 99, 158, 118, 56, 229, 190, 249, 234,
  49, 96, 140, 219, 102, 191, 109, 226, 191, 212, 66, 205, 218, 163, 45, 112,
  185, 199, 197, 150, 197, 118, 177, 234, 87, 140, 17, 241, 157, 197, 122, 213,
  228, 238, 42, 85, 149, 146, 48, 185, 135, 126, 48, 89, 164, 26, 140, 117,
  246, 243, 237, 225, 48, 192, 209, 178, 171, 176, 53, 96, 114, 21, 70, 122,
  169, 245, 215, 117, 74, 114, 148, 18, 191, 206, 97, 184, 44, 105, 235, 176,
  158, 172, 175, 200, 186, 91, 20, 79, 92, 211, 211, 164, 91, 180, 85, 84,
  29, 43, 247, 187, 134, 219, 151, 95, 45, 208, 139, 120, 245, 9, 189, 183,
  62, 181, 96, 41, 235, 147, 241, 168, 86, 149, 110, 160, 21, 44, 212, 3,
  121, 69, 68, 207, 213, 238, 86, 244, 3, 127, 52, 86, 231, 133, 42, 28,
  209, 138, 219, 141, 95, 174, 10, 188, 255, 68, 109, 180, 10, 161, 244, 97,
  145, 98, 176, 172, 46, 48, 249, 42, 106, 128, 7, 193, 146, 181, 243, 158,
  102, 144, 219, 234, 5, 252, 239, 36, 29, 157, 5, 173, 208, 75, 196, 74,
  83, 98, 191, 211, 87, 120, 110, 147, 106, 10, 83, 151, 161, 190, 241, 36,
  105, 44, 149, 22, 12, 133, 192, 43, 122, 19, 176, 141, 33, 188, 241, 106,
  183, 177, 46, 22, 174, 2, 245, 247, 146, 3, 100, 95, 63, 88, 70, 53,
  194, 90, 46, 152, 192, 130, 206, 122, 89, 210, 11, 179, 88, 51, 112, 6,
  93, 246, 96, 98, 246, 135, 132, 28, 255, 237, 44, 252, 133, 33, 71, 253,
  42, 158, 11, 219, 95, 76, 90, 167, 96, 83, 217, 236, 84, 155, 218, 139,
  97, 86, 26, 153, 198, 245, 103, 195, 10, 151, 96, 220, 158, 83, 255, 222,
  16, 111, 189, 9, 71, 22, 183, 138, 71, 138, 92, 147, 21, 103, 70, 213,
  188, 73, 119, 19, 100, 3, 89, 109, 192, 226, 199, 83, 16, 154, 168, 25,
  61, 75, 195, 9, 179, 11, 21, 12, 232, 32, 148, 178, 127, 155, 179, 44,
  183, 234, 187, 86, 84, 156, 118, 175, 82, 232, 166, 98, 19, 178, 231, 220,
  136, 248, 11, 230, 211, 136, 248, 230, 122, 49, 78, 160, 255, 152, 174, 120,
  149, 84, 49, 30, 24, 44, 168, 105, 104, 126, 79, 139, 49, 51, 2, 148,
  107, 151, 172, 25, 62, 74, 110, 217, 237, 196, 80, 231, 87, 164, 58, 21,
  110, 13, 223, 120, 152, 165, 234, 254, 230, 223, 95, 149, 88, 202, 30, 84,
  220, 212, 161, 3, 11, 44, 95, 156, 159, 80, 192, 60, 216, 30, 221, 104,
  11, 252, 134, 190, 209, 222, 210, 247, 189, 230, 133, 71, 144, 225, 125, 221,
  209, 200, 120, 220, 200, 130, 233, 108, 222, 221, 213, 178, 28, 156, 216, 94,
  47, 9, 143, 19, 229, 251, 0, 246, 225, 165, 16, 183, 239, 5, 113, 187,
  2, 162, 220, 190, 223, 17, 36, 238, 243, 77, 152, 193, 178, 32, 148, 113,
  192, 19, 170, 197, 103, 29, 18, 167, 11, 166, 125, 136, 39, 234, 22, 34,
  241, 211, 95, 76, 118, 151, 71, 180, 125, 228, 57, 188, 88, 222, 130, 228,
  145, 176, 106, 38, 250, 195, 34, 44, 163, 201, 114, 101, 100, 73, 235, 154,
  165, 227, 90, 250, 228, 35, 71, 161, 52, 117, 82, 43, 22, 194, 63, 78,
  243, 44, 216, 69, 155, 230, 197, 189, 56, 191, 190, 110, 20, 149, 169, 30,
  21, 126, 195, 175, 227, 106, 181, 101, 226, 175, 94, 221, 45, 14, 216, 126,
  49, 21, 151, 51, 236, 61, 236, 93, 28, 181, 18, 229, 182, 244, 96, 112,
  41, 173, 150, 215, 246, 218, 202, 244, 44, 97, 40, 187, 223, 140, 130, 153,
  228, 179, 173, 137, 9, 93, 109, 92, 115, 10, 185, 214, 53, 185, 192, 158,
  206, 207, 158, 36, 87, 218, 51, 165, 72, 194, 167, 152, 108, 63, 86, 202,
  91, 191, 120, 97, 133, 222, 118, 2, 30, 216, 209, 255, 103, 229, 241, 136,
  60, 182, 55, 30, 8, 200, 12, 44, 243, 200, 23, 24, 200, 240, 212, 48,
  5, 71, 207, 227, 142, 97, 198, 15, 242, 195, 55, 226, 9, 221, 25, 190,
  164, 165, 245, 252, 232, 39, 105, 2, 248, 238, 198, 104, 245, 54, 88, 133,
  36, 35, 214, 197, 109, 240, 183, 127, 255, 175, 68, 1, 80, 212, 163, 169,
  248, 4, 132, 225, 51, 188, 50, 67, 25, 171, 214, 156, 99, 196, 83, 20,
  109, 181, 5, 211, 165, 172, 242, 52, 49, 158, 89, 117, 163, 117, 159, 87,
  120, 68, 92, 156, 213, 195, 52, 117, 45, 132, 124, 240, 41, 154, 6, 228,
  223, 149, 51, 72, 163, 148, 32, 236, 176, 31, 47, 195, 151, 142, 223, 176,
  42, 43, 47, 225, 99, 180, 142, 221, 189, 240, 135, 211, 38, 69, 104, 182,
  96, 45, 58, 159, 118, 143, 224, 190, 238, 83, 61, 231, 232, 192, 160, 61,
  98, 14, 187, 180, 2, 132, 62, 232, 186, 106, 13, 251, 2, 246, 30, 106,
  194, 134, 209, 78, 241, 156, 82, 58, 129, 55, 228, 49, 165, 157, 190, 93,
  146, 46, 142, 41, 209, 145, 10, 159, 199, 153, 15, 143, 245, 39, 211, 105,
  103, 90, 164, 14, 212, 235, 183, 102, 58, 127, 46, 13, 163, 251, 217, 54,
  253, 202, 14, 20, 143, 176, 88, 229, 124, 108, 138, 79, 129, 221, 88, 215,
  200, 38, 120, 222, 62, 116, 160, 250, 162, 93, 223, 147, 46, 190, 235, 46,
  244, 30, 165, 27, 99, 84, 104, 222, 116, 164, 85, 247, 54, 243, 40, 160,
  211, 44, 0, 237, 141, 165, 46, 99, 197, 120, 31, 220, 244, 94, 25, 208,
  207, 147, 71, 44, 31, 140, 255, 37, 75, 166, 245, 79, 171, 225, 44, 90,
  165, 225, 89, 29, 2, 93, 175, 31, 101, 108, 154, 37, 233, 174, 98, 196,
  223, 225, 171, 153, 240, 201, 111, 150, 225, 199, 237, 239, 242, 68, 37, 228,
  9, 90, 220, 141, 6, 21, 165, 235, 252, 77, 157, 197, 241, 209, 44, 87,
  92, 249, 100, 221, 236, 125, 231, 67, 99, 199, 91, 14, 132, 232, 171, 211,
  63, 99, 217, 247, 93, 110, 150, 190, 45, 45, 136, 215, 155, 213, 99, 115,
  80, 171, 13, 34, 202, 116, 13, 124, 164, 231, 184, 111, 29, 218, 129, 210,
  114, 235, 181, 60, 227, 197, 59, 249, 234, 151, 115, 229, 193, 188, 186, 227,
  138, 227, 165, 70, 74, 40, 190, 143, 248, 224, 104, 163, 68, 163, 177, 107,
  10, 141, 175, 56, 46, 34, 172, 10, 81, 187, 252, 222, 10, 228, 146, 47,
  37, 229, 246, 249, 56, 62, 3, 201, 153, 243, 172, 157, 123, 14, 249, 221,
  104, 175, 189, 44, 184, 248, 93, 65, 249, 170, 96, 224, 139, 174, 103, 15,
  154, 25, 54, 77, 60, 227, 52, 130, 63, 156, 160, 184, 133, 248, 40, 30,
  105, 112, 239, 225, 120, 67, 234, 251, 164, 73, 229, 91, 12, 222, 71, 166,
  248, 192, 90, 1, 102, 21, 142, 118, 23, 208, 142, 74, 71, 107, 89, 189,
  246, 248, 245, 97, 160, 243, 85, 173, 244, 126, 153, 71, 60, 243, 102, 117,
  21, 99, 16, 2, 183, 6, 117, 102, 46, 255, 156, 108, 9, 168, 24, 151,
  97, 58, 173, 215, 68, 91, 1, 67, 36, 106, 77, 252, 235, 81, 1, 103,
  41, 27, 69, 168, 165, 125, 45, 241, 76, 79, 191, 139, 238, 106, 2, 47,
  24, 193, 191, 108, 72, 15, 179, 219, 228, 144, 121, 246, 195, 167, 226, 100,
  18, 20, 55, 66, 242, 22, 213, 58, 134, 58, 13, 203, 178, 240, 140, 89,
  51, 111, 193, 202, 230, 104, 87, 101, 43, 92, 153, 106, 50, 75, 129, 241,
  36, 79, 232, 132, 199, 101, 143, 182, 184, 175, 177, 196, 208, 117, 34, 163,
  32, 41, 134, 195, 38, 204, 27, 225, 71, 102, 142, 160, 86, 146, 47, 61,
  80, 240, 61, 25, 138, 137, 49, 63, 216, 171, 159, 189, 2, 127, 45, 241,
  70, 127, 29, 201, 86, 168, 245, 247, 145, 112, 191, 168, 140, 91, 94, 202,
  253, 35, 229, 156, 43, 233, 238, 45, 235, 228, 1, 127, 193, 119, 141, 82,
  138, 243, 224, 211, 116, 21, 149, 15, 221, 123, 254, 199, 138, 72, 231, 82,
  106, 62, 195, 39, 50, 246, 149, 249, 166, 126, 163, 89, 117, 20, 130, 220,
  203, 72, 67, 85, 203, 251, 40, 125, 148, 24, 218, 78, 62, 78, 96, 107,
  166, 176, 185, 245, 141, 162, 49, 143, 232, 98, 188, 77, 65, 235, 187, 82,
  84, 243, 176, 22, 229, 82, 218, 148, 26, 198, 182, 64, 76, 1, 159, 4,
  174, 150, 194, 150, 244, 42, 17, 118, 213, 178, 204, 186, 186, 233, 17, 98,
  154, 35, 111, 81, 145, 143, 215, 227, 11, 150, 2, 72, 35, 70, 223, 215,
  59, 232, 0, 81, 156, 178, 108, 204, 45, 253, 175, 102, 40, 26, 181, 150,
  220, 141, 170, 141, 147, 251, 250, 34, 203, 31, 163, 166, 24, 212, 241, 138,
  171, 116, 3, 241, 186, 187, 211, 237, 21, 110, 59, 137, 134, 62, 123, 198,
  2, 151, 10, 191, 39, 133, 41, 152, 112, 99, 237, 245, 226, 224, 170, 129,
  227, 89, 143, 17, 113, 88, 108, 217, 33, 28, 185, 128, 203, 34, 218, 48,
  2, 82, 90, 171, 46, 190, 120, 189, 217, 57, 141, 234, 53, 64, 238, 104,
  126, 85, 195, 87, 137, 225, 79, 195, 91, 96, 63, 233, 97, 129, 1, 252,
  241, 23, 56, 225, 79, 180, 241, 167, 218, 252, 69, 158, 207, 39, 88, 66,
  61, 205, 230, 47, 245, 150, 222, 96, 163, 167, 216, 42, 24, 0, 178, 201,
  56, 86, 135, 31, 246, 225, 21, 62, 44, 86, 186, 6, 96, 121, 101, 123,
  168, 165, 103, 167, 97, 189, 187, 209, 236, 173, 53, 215, 123, 205, 78, 163,
  230, 121, 133, 151, 30, 164, 123, 255, 27, 253, 44, 2, 36, 120, 167, 221,
  105, 6, 131, 126, 240, 126, 99, 187, 217, 93, 235, 52, 123, 235, 155, 31,
  130, 219, 166, 91, 108, 147, 23, 91, 91, 111, 118, 183, 31, 52, 183, 215,
  125, 165, 186, 237, 46, 47, 213, 195, 98, 15, 182, 155, 91, 254, 82, 2,
  86, 111, 125, 171, 217, 237, 174, 225, 127, 70, 185, 15, 238, 197, 92, 16,
  139, 122, 84, 110, 101, 37, 3, 50, 96, 156, 194, 102, 208, 107, 119, 172,
  96, 120, 209, 240, 138, 63, 48, 56, 67, 127, 244, 233, 240, 16, 230, 239,
  21, 143, 181, 202, 225, 225, 35, 13, 70, 180, 7, 122, 203, 9, 106, 65,
  70, 167, 120, 10, 9, 104, 91, 255, 238, 134, 224, 208, 85, 223, 246, 159,
  97, 73, 175, 131, 210, 209, 184, 109, 124, 114, 156, 225, 101, 147, 24, 189,
  170, 213, 253, 208, 12, 78, 245, 20, 167, 103, 252, 62, 18, 161, 211, 10,
  66, 140, 252, 178, 26, 212, 79, 41, 90, 118, 232, 9, 207, 58, 33, 109,
  52, 4, 36, 80, 147, 168, 95, 52, 163, 34, 166, 35, 143, 79, 120, 129,
  1, 94, 78, 219, 131, 247, 209, 135, 214, 69, 227, 251, 188, 225, 222, 102,
  23, 61, 2, 88, 222, 174, 120, 158, 56, 10, 227, 1, 176, 51, 61, 22,
  184, 79, 175, 241, 17, 227, 219, 252, 74, 217, 6, 199, 202, 146, 42, 177,
  242, 33, 108, 178, 217, 2, 163, 117, 59, 91, 193, 247, 188, 79, 236, 10,
  122, 217, 125, 208, 238, 61, 128, 20, 130, 79, 4, 226, 45, 253, 16, 244,
  214, 30, 180, 215, 60, 239, 187, 227, 40, 48, 60, 130, 171, 43, 12, 232,
  57, 180, 118, 199, 247, 128, 34, 47, 220, 130, 106, 38, 43, 148, 207, 55,
  239, 35, 220, 122, 180, 84, 197, 168, 189, 246, 6, 189, 76, 88, 245, 130,
  148, 196, 17, 227, 92, 20, 84, 182, 4, 194, 87, 38, 81, 138, 129, 62,
  234, 93, 140, 223, 131, 20, 88, 13, 40, 148, 5, 29, 47, 87, 16, 33,
  29, 47, 75, 131, 53, 157, 8, 0, 20, 180, 141, 113, 21, 13, 112, 73,
  133, 206, 63, 103, 97, 142, 156, 173, 66, 76, 74, 183, 17, 126, 84, 200,
  195, 59, 194, 39, 46, 190, 113, 120, 125, 16, 55, 233, 205, 91, 134, 143,
  147, 184, 135, 255, 2, 10, 247, 239, 44, 185, 196, 182, 108, 224, 111, 254,
  12, 121, 17, 176, 224, 210, 23, 118, 107, 92, 228, 143, 173, 160, 71, 254,
  200, 225, 151, 77, 247, 109, 77, 28, 57, 254, 176, 83, 119, 189, 201, 191,
  232, 164, 96, 109, 195, 243, 100, 40, 47, 136, 212, 198, 15, 42, 215, 117,
  29, 4, 196, 235, 30, 200, 51, 117, 225, 130, 194, 79, 121, 94, 195, 66,
  27, 191, 65, 47, 16, 10, 206, 222, 240, 4, 188, 189, 230, 143, 118, 96,
  172, 128, 49, 254, 81, 207, 116, 184, 26, 8, 61, 151, 185, 171, 80, 6,
  222, 194, 121, 55, 70, 190, 170, 23, 137, 162, 127, 134, 174, 160, 90, 187,
  226, 173, 97, 128, 131, 75, 252, 227, 109, 77, 235, 63, 250, 90, 114, 42,
  252, 128, 97, 17, 86, 131, 203, 134, 152, 246, 188, 49, 158, 233, 181, 160,
  240, 153, 175, 203, 54, 34, 226, 18, 209, 127, 140, 69, 187, 188, 56, 141,
  242, 85, 51, 184, 110, 82, 31, 248, 191, 37, 198, 13, 243, 84, 79, 240,
  57, 197, 57, 226, 63, 219, 177, 22, 13, 90, 79, 31, 3, 155, 105, 97,
  162, 221, 65, 225, 59, 10, 148, 27, 107, 189, 178, 48, 128, 202, 201, 155,
  244, 8, 181, 196, 55, 59, 237, 238, 86, 99, 137, 192, 108, 181, 111, 195,
  141, 209, 218, 104, 80, 91, 20, 156, 237, 46, 113, 215, 172, 232, 169, 162,
  31, 59, 206, 107, 74, 250, 82, 42, 231, 14, 48, 67, 180, 90, 23, 85,
  90, 221, 70, 227, 123, 201, 128, 45, 31, 251, 105, 12, 245, 28, 233, 185,
  235, 200, 228, 166, 62, 16, 222, 186, 100, 38, 170, 75, 8, 138, 243, 86,
  5, 47, 182, 196, 119, 227, 251, 203, 178, 176, 106, 92, 28, 231, 218, 20,
  89, 245, 76, 155, 198, 247, 99, 215, 176, 23, 193, 2, 11, 42, 139, 55,
  144, 26, 25, 121, 125, 129, 211, 74, 172, 108, 26, 221, 21, 1, 119, 162,
  61, 28, 132, 86, 235, 151, 36, 253, 17, 5, 83, 47, 167, 60, 178, 122,
  53, 233, 17, 194, 223, 145, 242, 119, 32, 106, 33, 254, 147, 140, 149, 7,
  94, 115, 227, 121, 85, 196, 176, 179, 131, 19, 203, 247, 5, 138, 21, 145,
  155, 157, 138, 42, 110, 156, 194, 69, 97, 94, 41, 186, 31, 202, 89, 69,
  86, 84, 46, 108, 241, 138, 50, 247, 114, 231, 55, 229, 164, 197, 224, 80,
  140, 116, 92, 73, 76, 169, 163, 88, 36, 6, 64, 227, 69, 82, 138, 20,
  251, 93, 117, 188, 47, 237, 52, 93, 220, 70, 126, 203, 70, 235, 240, 127,
  100, 73, 229, 155, 34, 16, 102, 155, 184, 49, 90, 3, 105, 182, 185, 156,
  48, 243, 68, 74, 251, 82, 97, 166, 7, 121, 227, 75, 66, 69, 47, 54,
  1, 123, 35, 2, 220, 247, 189, 47, 231, 22, 59, 226, 161, 212, 160, 124,
  107, 70, 76, 78, 35, 24, 231, 78, 150, 106, 255, 219, 156, 165, 215, 220,
  133, 3, 111, 59, 227, 171, 215, 45, 94, 172, 230, 222, 167, 23, 238, 40,
  166, 168, 224, 169, 237, 104, 58, 101, 233, 243, 147, 23, 71, 120, 212, 108,
  89, 134, 30, 82, 4, 29, 216, 14, 101, 217, 238, 10, 47, 223, 202, 46,
  209, 148, 180, 18, 144, 131, 192, 238, 202, 105, 56, 56, 63, 163, 173, 78,
  255, 187, 27, 181, 16, 119, 64, 199, 189, 93, 217, 123, 184, 138, 0, 246,
  166, 17, 27, 166, 209, 217, 215, 3, 190, 173, 1, 143, 134, 44, 140, 191,
  22, 232, 110, 123, 67, 3, 61, 78, 6, 99, 3, 242, 167, 106, 101, 193,
  85, 163, 147, 25, 155, 62, 149, 110, 202, 97, 236, 185, 41, 165, 59, 54,
  171, 11, 219, 59, 118, 54, 249, 2, 138, 243, 33, 59, 19, 164, 242, 219,
  136, 161, 168, 174, 141, 185, 186, 94, 187, 235, 237, 157, 138, 24, 55, 77,
  121, 181, 199, 178, 240, 240, 187, 99, 220, 243, 29, 187, 118, 18, 229, 49,
  250, 75, 127, 226, 221, 13, 167, 89, 52, 24, 231, 125, 25, 24, 37, 230,
  143, 21, 59, 202, 245, 105, 56, 164, 51, 107, 221, 199, 136, 224, 61, 193,
  12, 219, 201, 136, 74, 55, 120, 37, 203, 136, 101, 132, 95, 177, 169, 40,
  223, 40, 57, 61, 2, 2, 22, 173, 137, 132, 90, 195, 87, 50, 204, 242,
  205, 177, 89, 150, 146, 74, 75, 247, 214, 221, 226, 100, 133, 179, 30, 202,
  227, 141, 54, 36, 58, 109, 98, 83, 122, 162, 39, 28, 14, 235, 53, 140,
  204, 113, 193, 124, 181, 168, 249, 70, 129, 156, 86, 51, 165, 32, 17, 11,
  42, 3, 50, 13, 13, 217, 165, 170, 11, 157, 117, 156, 224, 110, 80, 241,
  39, 63, 91, 64, 83, 154, 193, 19, 79, 185, 183, 80, 29, 115, 144, 37,
  79, 194, 83, 124, 227, 72, 212, 110, 138, 56, 6, 126, 183, 53, 221, 241,
  219, 73, 114, 188, 145, 36, 66, 143, 132, 95, 18, 46, 47, 182, 119, 147,
  48, 217, 137, 45, 172, 132, 109, 167, 84, 129, 230, 16, 17, 182, 229, 252,
  4, 125, 37, 38, 125, 27, 101, 209, 105, 204, 234, 14, 247, 214, 96, 222,
  224, 121, 64, 195, 142, 116, 196, 103, 71, 149, 9, 154, 52, 18, 93, 102,
  232, 210, 98, 169, 134, 233, 64, 207, 190, 112, 80, 233, 220, 232, 117, 109,
  44, 59, 90, 164, 243, 57, 178, 53, 235, 152, 217, 135, 123, 149, 7, 123,
  53, 58, 99, 200, 233, 230, 102, 110, 159, 48, 211, 83, 56, 50, 235, 49,
  206, 237, 226, 144, 75, 47, 231, 156, 247, 96, 99, 30, 59, 59, 227, 142,
  176, 218, 81, 27, 29, 248, 200, 51, 30, 159, 101, 62, 153, 38, 110, 5,
  76, 205, 147, 105, 52, 128, 74, 104, 253, 166, 196, 169, 181, 251, 176, 222,
  80, 243, 31, 142, 81, 12, 67, 129, 149, 220, 42, 186, 64, 242, 12, 84,
  238, 99, 210, 219, 145, 45, 133, 31, 179, 170, 216, 71, 143, 120, 194, 83,
  59, 19, 20, 41, 250, 185, 160, 143, 102, 220, 13, 11, 6, 19, 9, 55,
  23, 253, 11, 103, 31, 149, 207, 121, 151, 95, 110, 108, 95, 70, 163, 232,
  99, 225, 181, 69, 89, 222, 39, 39, 50, 124, 56, 203, 138, 49, 15, 67,
  57, 191, 234, 155, 68, 132, 148, 143, 88, 186, 209, 52, 10, 206, 102, 163,
  225, 114, 37, 7, 73, 207, 42, 8, 41, 190, 130, 168, 194, 90, 37, 49,
  201, 87, 84, 110, 113, 172, 226, 50, 217, 87, 37, 102, 225, 200, 70, 24,
  146, 124, 69, 65, 226, 88, 37, 33, 133, 23, 212, 181, 9, 157, 170, 42,
  108, 139, 60, 254, 18, 82, 119, 207, 187, 219, 227, 71, 65, 8, 217, 189,
  88, 109, 148, 138, 134, 134, 219, 211, 18, 39, 196, 152, 90, 215, 98, 141,
  210, 65, 13, 174, 176, 88, 125, 47, 240, 251, 161, 141, 35, 90, 145, 5,
  83, 248, 113, 162, 107, 84, 88, 174, 48, 149, 224, 54, 73, 4, 126, 33,
  166, 87, 231, 163, 88, 206, 227, 60, 67, 29, 170, 168, 166, 205, 31, 79,
  109, 53, 51, 181, 131, 212, 143, 12, 4, 229, 39, 60, 202, 175, 196, 5,
  39, 191, 62, 57, 225, 179, 100, 186, 153, 135, 198, 74, 107, 138, 233, 84,
  76, 158, 213, 75, 110, 95, 48, 28, 178, 24, 157, 218, 203, 15, 117, 54,
  215, 85, 142, 199, 254, 38, 113, 130, 217, 109, 98, 218, 50, 141, 98, 185,
  162, 85, 252, 186, 83, 179, 207, 66, 220, 162, 248, 26, 255, 56, 162, 172,
  101, 113, 16, 197, 77, 84, 68, 162, 194, 104, 221, 143, 145, 56, 183, 26,
  135, 32, 202, 98, 17, 197, 73, 215, 58, 49, 185, 162, 15, 199, 51, 168,
  144, 210, 57, 163, 9, 3, 253, 88, 245, 132, 118, 70, 127, 216, 80, 28,
  54, 191, 239, 124, 224, 209, 162, 42, 139, 24, 186, 108, 37, 65, 7, 116,
  94, 106, 80, 18, 146, 28, 10, 170, 12, 156, 163, 146, 92, 240, 173, 200,
  212, 89, 48, 112, 57, 63, 118, 53, 26, 226, 230, 228, 197, 99, 133, 229,
  138, 65, 194, 175, 165, 249, 69, 157, 225, 90, 77, 27, 7, 87, 11, 154,
  87, 199, 97, 10, 5, 153, 178, 52, 26, 40, 202, 157, 25, 202, 240, 158,
  228, 18, 83, 20, 202, 105, 115, 20, 190, 22, 53, 171, 204, 223, 175, 206,
  13, 149, 1, 151, 9, 188, 36, 93, 68, 215, 151, 201, 203, 224, 97, 219,
  194, 85, 231, 46, 232, 32, 156, 55, 247, 72, 129, 84, 72, 174, 45, 160,
  13, 148, 229, 1, 164, 113, 179, 247, 167, 8, 247, 141, 223, 221, 88, 248,
  125, 68, 235, 184, 229, 77, 171, 231, 169, 198, 138, 251, 18, 120, 165, 225,
  63, 60, 144, 200, 158, 94, 6, 10, 51, 189, 176, 206, 95, 135, 159, 124,
  122, 25, 169, 59, 7, 133, 99, 6, 246, 134, 235, 64, 181, 210, 226, 98,
  109, 165, 6, 69, 138, 249, 142, 38, 95, 80, 36, 236, 134, 47, 194, 59,
  81, 91, 60, 20, 66, 165, 172, 221, 107, 237, 92, 11, 194, 178, 83, 148,
  226, 155, 19, 121, 113, 181, 246, 237, 246, 32, 92, 3, 214, 180, 95, 22,
  81, 49, 11, 5, 126, 15, 113, 214, 151, 55, 54, 199, 8, 144, 1, 142,
  93, 85, 91, 163, 141, 109, 214, 57, 93, 216, 214, 94, 117, 91, 63, 255,
  245, 116, 153, 182, 182, 30, 116, 31, 116, 253, 109, 149, 195, 142, 38, 11,
  1, 175, 173, 15, 215, 182, 183, 45, 192, 183, 174, 174, 205, 77, 218, 176,
  229, 156, 158, 137, 135, 115, 243, 183, 146, 209, 117, 173, 45, 185, 108, 154,
  236, 71, 49, 183, 162, 152, 2, 208, 56, 225, 20, 3, 124, 36, 52, 190,
  126, 42, 110, 112, 101, 207, 210, 100, 162, 182, 54, 238, 126, 133, 107, 231,
  39, 0, 13, 182, 168, 67, 177, 103, 203, 220, 114, 142, 122, 136, 154, 33,
  191, 67, 33, 111, 64, 55, 133, 62, 248, 193, 208, 131, 164, 115, 153, 111,
  83, 240, 135, 89, 244, 36, 76, 221, 198, 188, 190, 91, 85, 222, 91, 95,
  247, 162, 58, 223, 54, 227, 88, 64, 61, 23, 59, 255, 174, 218, 58, 183,
  147, 195, 138, 11, 163, 97, 110, 147, 86, 140, 34, 71, 25, 211, 118, 53,
  115, 154, 61, 165, 171, 46, 236, 11, 143, 179, 226, 236, 60, 171, 243, 39,
  49, 128, 77, 206, 163, 25, 246, 66, 49, 74, 233, 233, 4, 113, 205, 59,
  216, 123, 149, 108, 108, 171, 221, 218, 150, 247, 186, 86, 59, 112, 233, 212,
  236, 58, 94, 251, 156, 3, 237, 106, 213, 238, 129, 37, 123, 78, 178, 82,
  184, 198, 138, 219, 37, 238, 92, 29, 179, 60, 143, 166, 103, 217, 151, 26,
  32, 50, 1, 167, 118, 71, 187, 2, 10, 7, 188, 176, 87, 104, 142, 66,
  40, 136, 79, 92, 31, 106, 181, 146, 42, 124, 197, 57, 99, 181, 166, 177,
  172, 156, 49, 127, 45, 181, 232, 82, 37, 177, 86, 53, 237, 130, 124, 98,
  113, 239, 214, 66, 51, 53, 156, 94, 27, 129, 89, 64, 197, 213, 52, 74,
  185, 139, 224, 165, 96, 67, 90, 8, 197, 170, 89, 164, 185, 104, 20, 121,
  246, 180, 41, 114, 44, 65, 174, 76, 13, 156, 4, 59, 101, 181, 200, 96,
  249, 18, 230, 26, 138, 118, 177, 252, 212, 240, 128, 83, 168, 183, 210, 76,
  1, 64, 18, 110, 183, 3, 174, 44, 191, 190, 80, 140, 73, 52, 147, 163,
  17, 205, 120, 248, 238, 146, 193, 59, 187, 148, 5, 207, 46, 43, 11, 102,
  83, 89, 48, 155, 122, 10, 22, 250, 69, 52, 56, 73, 206, 206, 98, 205,
  42, 205, 83, 15, 103, 60, 221, 37, 176, 94, 171, 97, 192, 0, 254, 99,
  131, 115, 90, 191, 120, 211, 148, 135, 205, 59, 239, 164, 113, 38, 63, 22,
  77, 145, 57, 49, 138, 65, 69, 174, 187, 171, 87, 33, 135, 110, 2, 211,
  30, 212, 15, 28, 194, 119, 129, 234, 157, 102, 32, 108, 73, 162, 128, 180,
  44, 169, 236, 104, 214, 151, 228, 110, 6, 103, 151, 125, 73, 82, 88, 159,
  166, 125, 69, 54, 107, 37, 85, 81, 79, 143, 233, 54, 154, 181, 230, 248,
  175, 176, 248, 102, 120, 185, 112, 244, 201, 58, 9, 97, 9, 81, 231, 136,
  46, 62, 202, 199, 7, 47, 143, 95, 189, 249, 120, 114, 240, 226, 245, 209,
  227, 147, 131, 99, 195, 235, 243, 134, 84, 251, 126, 237, 201, 243, 238, 131,
  255, 183, 189, 103, 89, 138, 35, 73, 242, 190, 95, 145, 104, 100, 93, 149,
  219, 80, 128, 164, 214, 180, 208, 3, 147, 0, 141, 180, 134, 36, 22, 104,
  245, 218, 72, 109, 144, 84, 37, 69, 46, 69, 102, 109, 102, 22, 15, 97,
  101, 54, 167, 253, 128, 157, 181, 249, 130, 249, 130, 61, 236, 169, 111, 253,
  39, 253, 37, 235, 30, 175, 140, 135, 71, 102, 86, 1, 234, 105, 179, 61,
  65, 69, 134, 123, 68, 120, 120, 120, 120, 120, 120, 184, 127, 183, 2, 60,
  147, 2, 123, 107, 63, 161, 171, 241, 48, 203, 175, 214, 58, 35, 150, 96,
  18, 104, 7, 7, 94, 168, 145, 60, 232, 163, 105, 125, 16, 173, 61, 88,
  133, 191, 253, 209, 218, 131, 7, 186, 207, 166, 68, 188, 191, 183, 253, 224,
  187, 39, 171, 10, 115, 245, 251, 198, 168, 247, 222, 236, 63, 172, 16, 203,
  95, 21, 218, 254, 8, 247, 199, 120, 118, 196, 175, 222, 109, 61, 248, 94,
  35, 134, 252, 121, 11, 168, 223, 109, 255, 219, 147, 149, 199, 171, 143, 20,
  114, 173, 64, 35, 8, 63, 255, 205, 138, 251, 205, 159, 87, 159, 84, 136,
  223, 44, 241, 159, 90, 175, 217, 225, 93, 96, 157, 96, 36, 234, 197, 32,
  191, 92, 91, 125, 12, 74, 26, 252, 249, 163, 15, 233, 35, 19, 233, 163,
  27, 35, 221, 219, 216, 124, 88, 209, 87, 254, 242, 224, 108, 205, 14, 27,
  155, 143, 12, 164, 143, 230, 70, 250, 147, 27, 115, 136, 43, 122, 114, 233,
  243, 139, 42, 238, 141, 55, 66, 37, 217, 117, 205, 228, 85, 140, 8, 6,
  188, 72, 224, 112, 111, 185, 88, 164, 106, 19, 64, 198, 174, 38, 96, 120,
  172, 108, 213, 8, 250, 52, 74, 4, 174, 87, 163, 170, 103, 92, 183, 235,
  91, 157, 132, 245, 87, 96, 40, 180, 4, 100, 216, 35, 218, 81, 4, 85,
  108, 220, 3, 178, 254, 4, 35, 99, 246, 250, 57, 168, 162, 177, 72, 172,
  208, 237, 12, 146, 243, 142, 163, 175, 91, 155, 43, 195, 190, 132, 31, 136,
  72, 9, 131, 172, 172, 65, 143, 87, 231, 38, 126, 168, 79, 237, 221, 75,
  80, 222, 177, 235, 241, 3, 92, 117, 59, 207, 206, 98, 216, 25, 245, 164,
  124, 61, 16, 131, 239, 129, 126, 62, 42, 79, 174, 152, 159, 137, 56, 232,
  225, 166, 47, 79, 174, 204, 30, 32, 29, 80, 234, 78, 247, 109, 233, 84,
  80, 106, 8, 167, 20, 255, 212, 33, 106, 227, 115, 156, 116, 176, 113, 146,
  140, 6, 93, 24, 96, 77, 194, 238, 246, 4, 101, 16, 118, 52, 137, 251,
  215, 130, 44, 184, 0, 145, 31, 197, 79, 92, 157, 211, 160, 171, 62, 203,
  21, 105, 120, 203, 147, 221, 101, 205, 184, 156, 162, 87, 225, 80, 196, 160,
  64, 91, 157, 137, 182, 88, 159, 164, 44, 207, 84, 81, 16, 19, 120, 84,
  166, 53, 45, 28, 77, 202, 50, 179, 232, 6, 16, 130, 193, 46, 132, 207,
  80, 39, 154, 148, 89, 199, 174, 99, 82, 214, 225, 191, 206, 102, 28, 157,
  194, 106, 76, 98, 144, 74, 252, 145, 104, 245, 211, 198, 229, 198, 150, 131,
  205, 171, 127, 10, 210, 143, 159, 108, 100, 134, 14, 83, 87, 142, 207, 123,
  248, 72, 97, 39, 207, 198, 209, 144, 29, 50, 187, 228, 93, 11, 167, 14,
  217, 201, 65, 82, 224, 255, 92, 15, 102, 197, 150, 102, 205, 53, 42, 32,
  192, 73, 60, 120, 141, 71, 35, 117, 44, 98, 194, 110, 25, 90, 56, 78,
  134, 29, 60, 186, 194, 105, 239, 36, 27, 172, 117, 118, 62, 236, 237, 67,
  193, 9, 51, 79, 20, 107, 215, 29, 65, 165, 165, 125, 96, 178, 206, 90,
  7, 21, 198, 164, 207, 250, 187, 124, 185, 116, 113, 113, 177, 132, 185, 109,
  150, 38, 249, 40, 78, 251, 160, 2, 14, 58, 211, 197, 224, 40, 27, 92,
  173, 29, 242, 158, 63, 191, 127, 205, 255, 153, 126, 147, 12, 158, 43, 30,
  77, 6, 211, 67, 251, 197, 93, 131, 2, 56, 117, 217, 73, 103, 84, 152,
  10, 130, 75, 197, 85, 85, 107, 46, 197, 250, 38, 151, 242, 240, 137, 88,
  222, 113, 42, 222, 22, 27, 57, 131, 113, 110, 196, 156, 117, 137, 32, 166,
  250, 172, 118, 32, 163, 90, 98, 92, 142, 153, 185, 232, 229, 222, 170, 246,
  156, 114, 60, 250, 218, 27, 14, 171, 105, 184, 168, 61, 3, 108, 210, 181,
  203, 16, 190, 247, 94, 24, 110, 95, 213, 30, 67, 249, 124, 253, 225, 241,
  209, 31, 65, 169, 124, 170, 124, 189, 24, 236, 139, 251, 215, 48, 70, 38,
  64, 153, 196, 196, 31, 149, 184, 148, 53, 151, 161, 7, 47, 244, 110, 0,
  177, 239, 9, 80, 166, 222, 76, 121, 149, 67, 119, 134, 188, 178, 160, 75,
  200, 0, 150, 113, 249, 199, 228, 75, 148, 15, 144, 244, 53, 220, 174, 148,
  135, 134, 185, 109, 99, 90, 17, 75, 235, 166, 150, 21, 67, 132, 16, 43,
  143, 243, 163, 60, 169, 138, 95, 150, 123, 172, 254, 142, 129, 113, 162, 178,
  86, 168, 2, 128, 176, 207, 87, 174, 121, 208, 175, 53, 214, 155, 106, 77,
  184, 54, 86, 81, 191, 57, 210, 119, 62, 245, 209, 105, 214, 83, 170, 142,
  103, 158, 179, 42, 58, 152, 95, 48, 94, 219, 227, 143, 83, 86, 159, 58,
  95, 246, 5, 221, 44, 255, 161, 170, 194, 203, 193, 121, 36, 61, 95, 184,
  197, 193, 86, 227, 135, 217, 143, 170, 145, 174, 253, 146, 196, 104, 31, 63,
  106, 174, 142, 82, 180, 24, 126, 175, 47, 71, 160, 164, 255, 129, 15, 158,
  227, 13, 122, 28, 201, 18, 130, 119, 170, 128, 186, 49, 147, 92, 160, 52,
  249, 157, 222, 8, 191, 62, 180, 30, 235, 114, 205, 116, 186, 61, 244, 183,
  252, 9, 217, 148, 253, 247, 252, 243, 61, 124, 172, 24, 143, 167, 159, 239,
  253, 116, 232, 56, 230, 97, 11, 161, 104, 169, 217, 19, 80, 90, 228, 190,
  188, 231, 137, 227, 148, 57, 142, 21, 144, 53, 247, 34, 221, 227, 81, 20,
  216, 231, 24, 129, 32, 148, 168, 93, 167, 56, 156, 147, 103, 193, 67, 220,
  181, 146, 20, 157, 174, 151, 188, 206, 113, 2, 33, 182, 19, 202, 30, 208,
  8, 209, 200, 222, 2, 165, 235, 35, 135, 142, 214, 130, 143, 196, 100, 80,
  111, 181, 97, 1, 217, 55, 253, 124, 194, 54, 132, 72, 231, 95, 9, 135,
  78, 80, 102, 104, 200, 125, 245, 197, 57, 10, 86, 173, 225, 81, 176, 66,
  65, 228, 105, 147, 53, 189, 103, 189, 10, 218, 91, 69, 13, 17, 71, 194,
  211, 223, 240, 180, 216, 24, 141, 169, 155, 198, 23, 193, 94, 92, 118, 109,
  217, 200, 158, 169, 178, 52, 212, 165, 218, 216, 204, 144, 166, 10, 161, 90,
  58, 80, 68, 239, 250, 217, 184, 78, 119, 202, 216, 181, 132, 41, 250, 179,
  177, 150, 206, 41, 42, 237, 79, 166, 190, 100, 85, 168, 168, 166, 111, 114,
  0, 230, 113, 170, 231, 125, 20, 87, 106, 64, 79, 113, 77, 194, 243, 13,
  150, 206, 118, 203, 230, 176, 110, 214, 90, 205, 74, 224, 236, 70, 50, 138,
  157, 69, 116, 158, 32, 8, 250, 81, 167, 92, 205, 79, 104, 147, 212, 168,
  152, 96, 231, 221, 26, 206, 17, 82, 169, 63, 135, 102, 101, 109, 236, 126,
  242, 91, 121, 187, 237, 80, 15, 218, 44, 116, 43, 62, 211, 195, 158, 184,
  43, 125, 156, 141, 39, 184, 229, 240, 213, 190, 147, 0, 40, 170, 66, 166,
  75, 172, 176, 185, 163, 172, 25, 68, 160, 76, 225, 32, 138, 65, 228, 92,
  168, 152, 85, 251, 35, 89, 181, 63, 170, 175, 186, 123, 41, 106, 230, 151,
  245, 21, 247, 101, 197, 210, 174, 232, 142, 204, 214, 238, 156, 125, 80, 219,
  109, 145, 55, 8, 77, 199, 200, 168, 230, 219, 129, 213, 51, 50, 74, 102,
  82, 227, 32, 101, 228, 162, 213, 169, 74, 122, 220, 134, 240, 212, 215, 221,
  245, 93, 174, 56, 223, 32, 212, 42, 76, 64, 55, 188, 171, 101, 136, 184,
  219, 174, 67, 86, 247, 150, 22, 162, 1, 36, 59, 99, 81, 194, 236, 214,
  84, 123, 161, 230, 172, 63, 19, 50, 36, 94, 133, 12, 226, 227, 104, 50,
  42, 177, 246, 219, 244, 56, 195, 235, 202, 189, 50, 74, 7, 0, 54, 134,
  178, 224, 34, 206, 7, 49, 232, 130, 113, 58, 41, 191, 148, 61, 157, 102,
  159, 212, 42, 86, 107, 84, 174, 64, 177, 192, 126, 170, 230, 106, 48, 107,
  224, 21, 45, 4, 138, 176, 142, 184, 129, 123, 166, 33, 253, 166, 33, 18,
  43, 11, 71, 245, 99, 148, 227, 253, 158, 125, 119, 109, 40, 183, 186, 159,
  52, 39, 217, 7, 152, 174, 120, 224, 245, 104, 247, 248, 246, 235, 58, 166,
  243, 172, 192, 186, 106, 162, 150, 110, 91, 229, 135, 93, 45, 18, 53, 76,
  89, 235, 126, 167, 14, 181, 204, 223, 130, 60, 213, 42, 181, 69, 25, 68,
  44, 108, 140, 63, 137, 35, 224, 140, 82, 229, 255, 55, 244, 219, 223, 208,
  213, 92, 224, 192, 40, 186, 164, 3, 47, 85, 26, 183, 47, 122, 215, 107,
  16, 98, 20, 29, 60, 26, 67, 173, 33, 68, 95, 61, 251, 51, 113, 90,
  181, 106, 246, 201, 61, 204, 254, 54, 207, 106, 193, 17, 170, 229, 178, 111,
  81, 97, 174, 233, 225, 24, 159, 243, 199, 0, 108, 110, 76, 226, 207, 50,
  45, 141, 138, 131, 110, 68, 11, 111, 103, 166, 128, 136, 220, 54, 243, 138,
  93, 1, 200, 219, 58, 173, 212, 156, 33, 253, 75, 104, 64, 55, 154, 228,
  116, 53, 141, 96, 224, 234, 236, 164, 243, 16, 175, 191, 129, 47, 185, 108,
  38, 210, 62, 81, 92, 164, 125, 14, 93, 100, 205, 189, 109, 183, 135, 240,
  141, 171, 166, 235, 116, 175, 253, 29, 118, 89, 158, 151, 215, 117, 56, 38,
  207, 96, 160, 138, 112, 99, 72, 34, 188, 156, 141, 14, 176, 247, 40, 213,
  230, 25, 44, 89, 91, 107, 24, 188, 8, 86, 191, 91, 9, 103, 35, 4,
  205, 113, 150, 33, 229, 85, 212, 63, 53, 12, 41, 88, 96, 82, 68, 20,
  134, 178, 122, 227, 116, 25, 10, 131, 138, 213, 179, 186, 168, 91, 194, 150,
  130, 213, 208, 158, 170, 102, 11, 16, 105, 213, 105, 111, 126, 22, 224, 202,
  26, 199, 222, 125, 181, 23, 77, 44, 7, 40, 37, 173, 208, 182, 211, 249,
  205, 69, 86, 179, 224, 161, 85, 58, 21, 64, 233, 161, 49, 67, 223, 226,
  12, 185, 41, 176, 164, 168, 170, 212, 70, 75, 90, 25, 31, 108, 129, 101,
  124, 12, 109, 52, 51, 78, 229, 2, 179, 11, 231, 103, 221, 206, 143, 73,
  126, 10, 21, 79, 2, 64, 116, 4, 19, 51, 156, 164, 195, 224, 151, 191,
  163, 213, 58, 93, 239, 132, 174, 150, 228, 28, 43, 77, 109, 185, 65, 67,
  230, 174, 175, 226, 33, 237, 111, 172, 230, 91, 231, 225, 169, 119, 202, 154,
  173, 166, 164, 137, 179, 241, 162, 215, 154, 24, 113, 137, 157, 13, 208, 30,
  138, 6, 187, 31, 118, 183, 247, 226, 40, 239, 159, 236, 68, 121, 116, 102,
  156, 206, 177, 86, 15, 104, 205, 141, 194, 25, 146, 22, 183, 59, 218, 116,
  169, 191, 111, 168, 211, 145, 149, 46, 238, 2, 180, 177, 144, 126, 84, 81,
  212, 183, 88, 56, 90, 246, 15, 185, 204, 85, 67, 28, 132, 183, 184, 206,
  255, 146, 0, 213, 112, 121, 119, 14, 16, 57, 178, 19, 111, 150, 36, 140,
  84, 49, 184, 207, 145, 89, 139, 119, 128, 61, 127, 60, 228, 79, 76, 62,
  178, 184, 150, 219, 217, 133, 140, 115, 48, 61, 184, 127, 93, 109, 44, 186,
  182, 92, 53, 145, 160, 131, 180, 193, 107, 191, 233, 21, 58, 239, 89, 153,
  237, 149, 121, 146, 14, 49, 229, 163, 107, 214, 215, 214, 46, 37, 191, 217,
  113, 188, 145, 251, 2, 86, 207, 228, 63, 152, 153, 242, 0, 139, 59, 158,
  138, 46, 177, 244, 175, 5, 51, 212, 117, 181, 5, 6, 5, 225, 58, 231,
  6, 38, 222, 59, 75, 171, 62, 212, 5, 51, 221, 233, 192, 80, 208, 22,
  56, 191, 52, 97, 65, 230, 180, 5, 45, 45, 208, 253, 38, 208, 223, 150,
  61, 88, 199, 61, 236, 49, 117, 173, 54, 72, 71, 229, 94, 254, 167, 184,
  24, 199, 176, 79, 196, 121, 217, 11, 94, 37, 37, 108, 163, 239, 227, 9,
  83, 134, 131, 193, 4, 184, 228, 248, 151, 159, 79, 242, 56, 237, 209, 151,
  78, 142, 191, 248, 23, 215, 93, 92, 10, 81, 225, 43, 174, 254, 37, 221,
  188, 51, 61, 178, 131, 177, 41, 63, 116, 150, 35, 233, 34, 98, 11, 249,
  60, 102, 131, 121, 159, 93, 24, 187, 114, 85, 108, 74, 123, 163, 122, 104,
  66, 207, 44, 249, 125, 108, 33, 176, 186, 252, 64, 231, 99, 180, 174, 239,
  139, 126, 148, 190, 143, 203, 139, 44, 63, 45, 236, 160, 17, 114, 134, 149,
  219, 62, 154, 228, 38, 48, 189, 48, 171, 229, 23, 16, 130, 167, 113, 175,
  103, 204, 229, 28, 151, 254, 169, 104, 156, 116, 117, 47, 146, 129, 115, 170,
  134, 50, 194, 189, 93, 213, 180, 223, 15, 84, 95, 106, 204, 53, 129, 120,
  13, 32, 59, 35, 35, 189, 75, 109, 34, 245, 231, 179, 152, 199, 42, 99,
  218, 101, 210, 30, 246, 145, 170, 225, 216, 101, 120, 85, 230, 101, 146, 246,
  114, 248, 127, 58, 120, 117, 22, 30, 218, 176, 218, 152, 235, 204, 52, 110,
  172, 118, 150, 220, 135, 164, 132, 136, 108, 133, 186, 244, 74, 120, 235, 164,
  176, 158, 232, 241, 228, 242, 138, 201, 96, 234, 129, 95, 7, 150, 127, 211,
  108, 3, 245, 191, 230, 240, 115, 121, 116, 52, 140, 139, 254, 9, 156, 161,
  65, 244, 118, 90, 56, 102, 80, 188, 60, 155, 75, 134, 34, 251, 28, 206,
  24, 218, 37, 81, 54, 222, 141, 197, 3, 139, 215, 35, 80, 77, 4, 60,
  227, 123, 189, 19, 185, 172, 245, 82, 58, 214, 88, 250, 46, 151, 98, 162,
  210, 126, 114, 22, 195, 24, 216, 216, 241, 255, 108, 82, 218, 31, 159, 186,
  168, 217, 7, 39, 120, 140, 250, 188, 9, 123, 22, 222, 227, 179, 144, 161,
  70, 187, 162, 211, 161, 57, 73, 106, 96, 44, 159, 28, 70, 83, 231, 181,
  232, 211, 133, 1, 177, 19, 241, 87, 87, 230, 249, 221, 115, 201, 102, 18,
  208, 142, 74, 123, 183, 52, 81, 179, 97, 30, 155, 40, 154, 105, 102, 141,
  111, 131, 199, 43, 70, 94, 217, 70, 66, 248, 14, 90, 181, 4, 23, 139,
  99, 152, 103, 23, 194, 57, 17, 192, 65, 69, 183, 182, 2, 25, 206, 112,
  196, 34, 206, 147, 59, 154, 56, 102, 90, 195, 166, 108, 238, 6, 201, 213,
  232, 191, 249, 70, 31, 254, 11, 151, 62, 206, 94, 80, 59, 46, 46, 117,
  94, 166, 184, 2, 149, 254, 242, 49, 206, 143, 146, 116, 128, 7, 222, 113,
  254, 203, 207, 199, 113, 26, 128, 190, 148, 163, 159, 196, 100, 188, 244, 114,
  39, 192, 219, 40, 75, 163, 105, 181, 178, 170, 120, 203, 190, 151, 102, 110,
  230, 19, 61, 85, 171, 182, 155, 118, 59, 39, 101, 57, 94, 91, 94, 182,
  231, 132, 137, 20, 80, 232, 76, 101, 241, 79, 91, 168, 43, 178, 151, 86,
  157, 62, 190, 79, 32, 183, 1, 204, 189, 105, 39, 91, 231, 71, 123, 22,
  121, 24, 241, 11, 39, 98, 111, 235, 29, 58, 253, 82, 141, 100, 174, 201,
  56, 177, 188, 28, 36, 195, 52, 203, 89, 218, 110, 32, 141, 135, 104, 206,
  18, 195, 57, 23, 235, 18, 153, 113, 17, 45, 125, 43, 180, 195, 1, 126,
  239, 250, 172, 199, 10, 49, 90, 13, 45, 237, 79, 251, 98, 43, 128, 38,
  80, 232, 160, 153, 193, 38, 51, 15, 229, 93, 5, 86, 52, 191, 139, 52,
  164, 135, 177, 107, 70, 215, 50, 198, 33, 193, 66, 23, 211, 12, 35, 161,
  4, 108, 93, 234, 112, 255, 187, 67, 199, 50, 82, 182, 126, 23, 41, 131,
  187, 200, 199, 143, 2, 114, 93, 252, 163, 158, 69, 174, 209, 185, 196, 47,
  114, 212, 56, 114, 183, 165, 93, 253, 20, 32, 14, 44, 188, 110, 40, 129,
  52, 159, 61, 222, 24, 166, 178, 26, 160, 130, 179, 24, 44, 136, 118, 93,
  62, 116, 159, 235, 26, 254, 182, 230, 59, 221, 154, 27, 35, 159, 48, 22,
  4, 252, 130, 180, 32, 30, 253, 62, 157, 81, 52, 81, 7, 88, 241, 174,
  242, 142, 30, 7, 200, 94, 63, 7, 165, 137, 125, 251, 97, 247, 237, 70,
  118, 54, 134, 34, 80, 64, 203, 47, 225, 244, 144, 20, 117, 11, 62, 89,
  39, 29, 137, 153, 209, 156, 143, 10, 171, 98, 65, 55, 236, 49, 89, 37,
  30, 240, 119, 92, 189, 182, 60, 129, 85, 201, 140, 36, 66, 201, 19, 135,
  210, 251, 215, 136, 131, 255, 152, 98, 236, 18, 192, 86, 155, 27, 39, 160,
  178, 75, 227, 36, 125, 49, 97, 106, 83, 25, 149, 95, 172, 252, 69, 110,
  98, 107, 45, 129, 17, 145, 15, 218, 206, 58, 232, 151, 212, 55, 124, 16,
  107, 196, 72, 245, 222, 186, 70, 231, 152, 164, 28, 223, 205, 27, 66, 76,
  43, 183, 238, 157, 12, 128, 208, 66, 48, 167, 25, 86, 190, 220, 119, 99,
  66, 9, 3, 15, 127, 196, 221, 116, 62, 158, 125, 213, 4, 246, 178, 9,
  110, 98, 242, 57, 20, 221, 134, 117, 35, 254, 115, 22, 74, 205, 50, 185,
  193, 34, 185, 193, 18, 105, 115, 136, 27, 199, 167, 24, 126, 11, 38, 71,
  153, 165, 190, 238, 147, 238, 106, 43, 19, 215, 58, 248, 88, 128, 133, 100,
  48, 47, 117, 84, 177, 117, 159, 163, 87, 15, 77, 232, 57, 57, 86, 134,
  132, 176, 99, 25, 97, 211, 30, 158, 157, 147, 213, 173, 231, 146, 86, 107,
  180, 97, 175, 32, 34, 64, 20, 84, 236, 7, 156, 91, 54, 161, 41, 158,
  58, 26, 95, 91, 218, 207, 44, 167, 191, 191, 5, 249, 141, 138, 248, 241,
  156, 69, 158, 102, 84, 116, 150, 233, 220, 107, 209, 175, 241, 27, 243, 226,
  153, 141, 143, 59, 155, 129, 136, 122, 20, 68, 167, 229, 36, 26, 129, 138,
  134, 171, 237, 169, 49, 31, 213, 35, 133, 12, 143, 155, 214, 38, 167, 29,
  104, 120, 156, 216, 198, 44, 183, 109, 186, 166, 24, 5, 48, 159, 140, 184,
  61, 7, 40, 151, 122, 187, 198, 131, 127, 88, 125, 107, 41, 16, 102, 144,
  90, 119, 38, 124, 190, 22, 89, 188, 66, 14, 119, 86, 140, 254, 225, 236,
  203, 88, 232, 110, 202, 162, 106, 168, 195, 205, 41, 220, 218, 7, 157, 169,
  204, 203, 182, 97, 185, 86, 154, 141, 129, 36, 21, 0, 254, 106, 33, 254,
  240, 76, 224, 63, 143, 172, 171, 115, 134, 27, 112, 69, 92, 64, 106, 208,
  201, 184, 161, 193, 225, 69, 85, 121, 120, 209, 212, 59, 125, 130, 82, 111,
  101, 39, 62, 143, 63, 38, 79, 71, 152, 75, 216, 189, 192, 211, 134, 40,
  60, 70, 124, 181, 186, 35, 225, 124, 98, 26, 27, 191, 67, 17, 141, 236,
  66, 31, 52, 240, 75, 56, 253, 6, 217, 131, 174, 128, 95, 160, 2, 103,
  5, 124, 190, 44, 185, 68, 132, 186, 153, 126, 147, 140, 105, 200, 100, 12,
  112, 195, 11, 250, 227, 240, 2, 145, 166, 158, 78, 165, 225, 215, 218, 45,
  102, 229, 149, 224, 12, 26, 78, 227, 73, 124, 166, 76, 254, 51, 179, 79,
  205, 14, 209, 182, 59, 175, 97, 241, 193, 90, 44, 130, 29, 104, 10, 77,
  128, 141, 61, 192, 176, 103, 78, 244, 65, 215, 248, 222, 145, 8, 165, 5,
  209, 86, 131, 41, 137, 207, 2, 91, 125, 133, 157, 197, 106, 167, 126, 87,
  105, 75, 202, 154, 221, 101, 102, 138, 18, 244, 212, 108, 178, 22, 254, 54,
  74, 56, 144, 50, 46, 157, 13, 74, 149, 58, 55, 184, 170, 114, 104, 128,
  222, 249, 30, 101, 19, 220, 79, 238, 31, 97, 197, 20, 113, 2, 232, 227,
  209, 8, 168, 18, 91, 90, 177, 208, 224, 185, 55, 151, 180, 29, 194, 233,
  254, 108, 12, 103, 38, 23, 216, 184, 170, 95, 15, 246, 147, 241, 56, 14,
  118, 183, 246, 182, 246, 131, 47, 112, 154, 122, 21, 23, 229, 47, 127, 47,
  19, 108, 198, 142, 41, 199, 238, 15, 141, 134, 120, 20, 179, 94, 153, 39,
  103, 32, 83, 140, 36, 33, 60, 54, 56, 67, 140, 151, 82, 51, 12, 248,
  229, 209, 48, 62, 202, 51, 96, 49, 100, 41, 97, 85, 118, 82, 41, 205,
  234, 36, 37, 250, 141, 99, 18, 157, 106, 118, 217, 225, 113, 164, 175, 150,
  24, 103, 212, 238, 54, 119, 229, 178, 163, 217, 183, 57, 199, 73, 251, 112,
  47, 143, 209, 167, 1, 147, 177, 24, 38, 111, 87, 97, 155, 41, 160, 92,
  67, 48, 185, 26, 187, 163, 207, 138, 235, 245, 77, 192, 238, 111, 103, 55,
  143, 216, 56, 202, 236, 104, 141, 85, 204, 230, 108, 248, 42, 99, 14, 58,
  3, 149, 105, 43, 20, 9, 85, 63, 167, 86, 46, 100, 250, 210, 215, 198,
  62, 155, 252, 69, 232, 121, 46, 123, 45, 82, 13, 178, 139, 244, 238, 200,
  37, 214, 211, 40, 59, 18, 235, 233, 21, 252, 219, 253, 228, 161, 217, 79,
  139, 50, 186, 23, 38, 96, 190, 44, 151, 199, 163, 40, 73, 59, 164, 242,
  1, 220, 14, 40, 97, 121, 10, 111, 129, 15, 71, 255, 14, 98, 30, 126,
  119, 177, 53, 42, 2, 68, 141, 135, 65, 100, 249, 81, 245, 64, 128, 29,
  67, 125, 104, 196, 44, 151, 212, 66, 89, 82, 221, 157, 44, 193, 192, 123,
  229, 165, 25, 146, 37, 234, 49, 169, 110, 158, 70, 177, 191, 121, 124, 158,
  157, 106, 253, 133, 70, 190, 58, 19, 136, 245, 219, 143, 82, 247, 192, 37,
  10, 173, 133, 91, 85, 13, 117, 184, 154, 205, 76, 119, 16, 114, 46, 147,
  88, 64, 89, 228, 57, 107, 47, 85, 229, 246, 110, 170, 3, 132, 22, 130,
  154, 78, 72, 214, 182, 58, 160, 115, 189, 209, 3, 253, 131, 217, 5, 11,
  36, 180, 113, 212, 116, 66, 175, 169, 112, 18, 34, 11, 246, 152, 18, 0,
  111, 188, 14, 199, 2, 15, 181, 22, 71, 208, 59, 253, 36, 202, 106, 98,
  151, 137, 60, 110, 80, 234, 100, 113, 195, 115, 125, 179, 59, 148, 236, 129,
  237, 14, 53, 246, 187, 67, 245, 249, 59, 144, 214, 65, 125, 248, 35, 253,
  124, 96, 106, 131, 88, 210, 33, 171, 241, 144, 16, 103, 81, 62, 76, 210,
  253, 12, 143, 198, 157, 239, 199, 151, 116, 93, 51, 2, 16, 102, 211, 75,
  135, 47, 238, 95, 143, 249, 211, 182, 103, 203, 162, 228, 217, 81, 190, 44,
  162, 0, 241, 144, 63, 159, 239, 177, 80, 224, 107, 34, 112, 250, 211, 207,
  247, 24, 212, 0, 244, 204, 60, 97, 94, 76, 83, 17, 213, 231, 16, 227,
  200, 142, 123, 40, 197, 214, 161, 9, 134, 40, 10, 80, 236, 176, 8, 30,
  236, 203, 244, 243, 61, 17, 170, 3, 202, 14, 142, 70, 81, 122, 10, 8,
  183, 147, 244, 244, 217, 114, 4, 24, 214, 168, 203, 35, 54, 65, 186, 75,
  19, 14, 168, 214, 121, 107, 218, 184, 81, 81, 252, 52, 155, 156, 82, 252,
  48, 191, 172, 138, 206, 99, 177, 62, 28, 251, 144, 40, 119, 77, 68, 21,
  64, 104, 33, 152, 89, 9, 103, 247, 24, 131, 248, 28, 115, 227, 161, 206,
  121, 91, 186, 35, 243, 91, 54, 87, 227, 91, 219, 154, 68, 2, 34, 43,
  58, 160, 184, 12, 218, 0, 107, 28, 233, 224, 216, 132, 111, 109, 112, 0,
  135, 58, 176, 63, 228, 163, 54, 160, 176, 109, 101, 14, 236, 54, 22, 182,
  0, 22, 17, 205, 28, 248, 45, 81, 174, 89, 198, 214, 131, 206, 42, 11,
  240, 178, 210, 70, 37, 87, 108, 254, 27, 248, 207, 87, 174, 196, 213, 38,
  64, 232, 221, 53, 113, 137, 122, 60, 76, 63, 143, 105, 22, 106, 225, 84,
  242, 1, 117, 156, 148, 57, 158, 184, 192, 67, 161, 13, 196, 237, 57, 153,
  12, 233, 71, 117, 103, 217, 164, 136, 99, 76, 235, 64, 190, 35, 229, 111,
  161, 173, 192, 251, 60, 21, 37, 229, 163, 165, 197, 227, 231, 149, 124, 175,
  102, 90, 16, 1, 91, 93, 138, 175, 116, 10, 176, 184, 141, 53, 4, 192,
  168, 137, 190, 241, 215, 71, 84, 36, 222, 20, 198, 245, 113, 20, 227, 222,
  24, 212, 63, 64, 182, 201, 99, 22, 184, 57, 4, 22, 100, 206, 78, 151,
  78, 236, 192, 54, 138, 45, 146, 46, 6, 62, 82, 183, 37, 227, 12, 3,
  100, 61, 84, 143, 38, 3, 60, 252, 227, 99, 143, 236, 56, 16, 27, 53,
  49, 193, 70, 204, 62, 245, 222, 146, 229, 29, 44, 74, 155, 111, 173, 64,
  29, 24, 83, 142, 208, 126, 212, 220, 177, 48, 119, 190, 201, 107, 36, 168,
  241, 0, 180, 234, 80, 197, 67, 62, 32, 47, 115, 187, 94, 131, 212, 132,
  89, 169, 26, 155, 29, 250, 60, 121, 93, 91, 216, 172, 170, 84, 141, 91,
  35, 42, 239, 169, 185, 103, 234, 181, 67, 3, 118, 86, 38, 113, 222, 213,
  234, 77, 18, 73, 39, 125, 110, 103, 12, 204, 122, 217, 92, 149, 17, 189,
  23, 111, 153, 171, 255, 107, 122, 110, 119, 195, 106, 91, 100, 76, 53, 212,
  13, 39, 135, 171, 150, 99, 149, 169, 25, 213, 255, 51, 184, 181, 213, 166,
  221, 157, 51, 159, 102, 109, 215, 90, 134, 126, 187, 235, 196, 180, 191, 69,
  138, 89, 79, 98, 84, 98, 238, 89, 151, 156, 217, 183, 6, 111, 12, 69,
  145, 89, 254, 154, 155, 7, 160, 137, 91, 224, 0, 163, 43, 51, 204, 57,
  124, 208, 197, 69, 83, 226, 226, 217, 82, 17, 163, 104, 81, 255, 254, 195,
  205, 56, 32, 165, 166, 220, 24, 142, 217, 31, 125, 210, 249, 207, 185, 103,
  29, 91, 185, 189, 105, 23, 157, 185, 187, 121, 111, 148, 11, 183, 53, 235,
  179, 11, 137, 153, 230, 252, 225, 202, 128, 154, 115, 40, 38, 231, 156, 87,
  15, 77, 232, 185, 231, 28, 91, 185, 189, 57, 23, 157, 105, 156, 243, 79,
  106, 114, 23, 181, 169, 91, 212, 217, 93, 253, 120, 2, 61, 84, 63, 86,
  31, 192, 47, 231, 181, 60, 253, 70, 94, 188, 140, 103, 249, 135, 71, 236,
  210, 198, 162, 96, 109, 128, 84, 51, 53, 215, 76, 19, 250, 132, 158, 208,
  39, 158, 9, 125, 98, 78, 232, 147, 155, 77, 232, 147, 91, 157, 208, 39,
  119, 48, 161, 15, 111, 125, 66, 159, 220, 241, 132, 98, 39, 169, 25, 101,
  157, 167, 166, 84, 0, 132, 22, 130, 185, 39, 149, 53, 116, 123, 179, 42,
  187, 115, 119, 211, 250, 228, 22, 102, 85, 17, 241, 46, 166, 85, 228, 171,
  179, 103, 245, 141, 76, 130, 103, 77, 106, 85, 61, 52, 161, 103, 158, 82,
  149, 115, 175, 202, 184, 103, 204, 143, 134, 186, 253, 214, 137, 55, 169, 226,
  97, 93, 53, 22, 89, 72, 236, 106, 242, 83, 168, 3, 223, 238, 206, 38,
  177, 218, 20, 118, 123, 101, 245, 73, 210, 88, 251, 125, 19, 34, 151, 178,
  61, 131, 202, 58, 242, 246, 100, 22, 115, 83, 207, 49, 54, 207, 232, 12,
  115, 67, 18, 59, 15, 62, 47, 146, 178, 127, 130, 35, 85, 193, 181, 175,
  219, 4, 17, 239, 157, 3, 136, 102, 163, 58, 71, 26, 158, 207, 18, 45,
  28, 17, 104, 186, 154, 104, 221, 36, 1, 175, 19, 138, 186, 205, 100, 214,
  164, 218, 89, 148, 164, 142, 76, 227, 119, 150, 189, 211, 248, 170, 232, 106,
  117, 209, 24, 82, 84, 99, 57, 197, 177, 216, 159, 63, 157, 254, 36, 208,
  133, 13, 50, 210, 140, 132, 149, 70, 231, 155, 81, 113, 114, 148, 25, 81,
  194, 244, 82, 147, 141, 245, 47, 161, 1, 221, 28, 204, 172, 154, 203, 14,
  82, 108, 105, 80, 53, 97, 173, 46, 192, 43, 34, 62, 24, 125, 114, 210,
  54, 137, 30, 137, 242, 80, 131, 155, 189, 55, 133, 68, 238, 246, 101, 99,
  148, 77, 76, 234, 176, 18, 167, 31, 172, 52, 84, 16, 179, 247, 161, 207,
  209, 18, 54, 35, 158, 188, 222, 121, 91, 230, 124, 156, 49, 32, 97, 5,
  142, 150, 106, 7, 89, 117, 85, 192, 18, 136, 27, 25, 81, 26, 120, 217,
  187, 67, 123, 146, 45, 184, 161, 218, 212, 224, 119, 177, 255, 158, 177, 107,
  223, 230, 24, 58, 131, 150, 35, 215, 81, 85, 3, 55, 143, 140, 95, 117,
  220, 27, 120, 215, 233, 25, 183, 246, 109, 182, 113, 243, 248, 97, 213, 76,
  83, 38, 88, 113, 5, 46, 114, 103, 224, 165, 177, 248, 247, 117, 150, 139,
  140, 196, 58, 6, 23, 84, 38, 108, 118, 186, 170, 232, 186, 241, 97, 251,
  195, 238, 193, 206, 203, 237, 173, 253, 253, 173, 79, 43, 63, 185, 47, 25,
  139, 152, 67, 170, 54, 121, 46, 15, 189, 229, 69, 213, 177, 69, 222, 100,
  155, 156, 30, 183, 159, 205, 216, 205, 15, 66, 79, 42, 223, 185, 61, 179,
  234, 124, 156, 99, 90, 245, 108, 200, 115, 206, 171, 129, 194, 63, 177, 110,
  111, 111, 97, 102, 245, 182, 127, 23, 83, 107, 107, 43, 68, 158, 103, 160,
  213, 245, 148, 200, 101, 161, 114, 172, 62, 15, 142, 71, 209, 144, 213, 93,
  239, 153, 249, 88, 221, 140, 134, 99, 179, 186, 72, 196, 26, 62, 165, 177,
  39, 233, 208, 172, 95, 149, 187, 47, 46, 78, 178, 139, 13, 173, 83, 85,
  7, 225, 212, 178, 16, 141, 237, 250, 120, 9, 107, 250, 201, 194, 108, 158,
  185, 58, 37, 139, 96, 97, 86, 84, 205, 188, 178, 163, 91, 88, 47, 225,
  89, 48, 12, 19, 214, 10, 148, 97, 233, 99, 218, 248, 90, 4, 179, 168,
  127, 216, 64, 120, 173, 187, 14, 207, 100, 4, 46, 227, 89, 62, 235, 103,
  104, 141, 169, 254, 149, 57, 53, 73, 136, 147, 145, 210, 28, 24, 43, 170,
  197, 102, 206, 43, 44, 79, 18, 189, 240, 232, 208, 235, 214, 70, 232, 120,
  107, 37, 88, 22, 255, 30, 84, 121, 159, 58, 191, 254, 229, 175, 182, 243,
  140, 249, 26, 180, 96, 158, 25, 60, 57, 48, 62, 247, 145, 104, 240, 71,
  3, 34, 238, 175, 202, 60, 228, 162, 188, 96, 233, 6, 178, 99, 14, 141,
  161, 151, 248, 133, 88, 58, 57, 59, 138, 243, 78, 176, 174, 125, 88, 179,
  53, 100, 207, 235, 4, 119, 176, 109, 199, 211, 162, 187, 110, 23, 60, 76,
  132, 107, 140, 186, 157, 197, 121, 122, 205, 215, 159, 92, 120, 24, 179, 2,
  169, 213, 181, 38, 28, 23, 175, 103, 198, 17, 123, 35, 235, 32, 110, 47,
  147, 251, 150, 219, 45, 36, 26, 191, 139, 135, 76, 83, 50, 32, 133, 121,
  122, 103, 69, 63, 18, 210, 140, 237, 32, 18, 32, 172, 96, 173, 222, 25,
  243, 177, 142, 254, 240, 71, 176, 244, 79, 68, 62, 192, 119, 206, 115, 154,
  224, 92, 140, 38, 173, 73, 143, 100, 241, 15, 114, 178, 29, 245, 104, 225,
  66, 124, 165, 34, 190, 242, 47, 13, 169, 135, 70, 241, 121, 44, 3, 128,
  194, 98, 162, 214, 17, 107, 57, 120, 241, 60, 88, 250, 110, 5, 126, 62,
  130, 33, 169, 146, 199, 88, 242, 80, 47, 249, 35, 150, 60, 208, 75, 190,
  95, 145, 239, 168, 48, 149, 171, 22, 14, 9, 88, 49, 232, 98, 218, 177,
  228, 249, 234, 211, 228, 217, 243, 71, 79, 147, 111, 191, 165, 120, 234, 40,
  202, 103, 113, 97, 132, 234, 38, 87, 32, 45, 150, 160, 180, 99, 51, 95,
  18, 60, 123, 206, 73, 16, 86, 80, 117, 71, 121, 142, 156, 251, 60, 158,
  196, 152, 112, 155, 199, 129, 123, 28, 124, 27, 36, 255, 252, 112, 58, 190,
  60, 52, 67, 24, 139, 73, 48, 114, 88, 70, 121, 171, 168, 97, 92, 21,
  97, 71, 74, 206, 234, 30, 205, 66, 86, 136, 85, 82, 74, 109, 247, 151,
  101, 250, 118, 91, 1, 228, 147, 20, 205, 72, 50, 33, 159, 248, 121, 160,
  240, 0, 35, 96, 156, 183, 227, 36, 101, 94, 95, 4, 228, 154, 214, 148,
  5, 238, 105, 18, 182, 196, 124, 192, 117, 20, 14, 85, 21, 212, 52, 167,
  42, 25, 13, 202, 82, 186, 169, 113, 156, 23, 220, 45, 151, 59, 207, 242,
  159, 7, 192, 62, 176, 240, 202, 36, 26, 21, 117, 77, 74, 104, 189, 65,
  2, 133, 227, 118, 162, 161, 0, 205, 33, 143, 134, 177, 48, 118, 235, 251,
  149, 248, 194, 244, 56, 17, 32, 157, 185, 175, 169, 125, 203, 168, 176, 230,
  65, 74, 54, 74, 234, 152, 61, 191, 118, 9, 44, 95, 254, 48, 70, 23,
  57, 30, 215, 119, 12, 252, 26, 191, 227, 204, 214, 195, 143, 7, 19, 246,
  245, 224, 172, 32, 97, 119, 160, 219, 62, 72, 12, 117, 229, 131, 123, 13,
  26, 246, 36, 143, 125, 160, 199, 252, 179, 15, 122, 63, 46, 74, 31, 104,
  9, 223, 44, 184, 138, 60, 213, 112, 1, 246, 61, 19, 114, 189, 164, 120,
  157, 164, 9, 168, 238, 6, 45, 66, 152, 11, 147, 56, 134, 244, 178, 112,
  238, 112, 150, 166, 48, 10, 10, 73, 124, 146, 96, 117, 216, 4, 113, 60,
  8, 43, 210, 73, 156, 26, 49, 235, 208, 34, 213, 60, 56, 5, 65, 37,
  66, 73, 95, 31, 182, 255, 152, 196, 19, 155, 169, 89, 217, 65, 145, 124,
  137, 73, 85, 76, 251, 188, 198, 103, 237, 45, 136, 111, 251, 19, 234, 126,
  152, 113, 126, 117, 133, 158, 63, 193, 23, 133, 213, 118, 85, 76, 180, 172,
  62, 218, 237, 170, 15, 77, 173, 34, 73, 152, 243, 181, 20, 38, 140, 209,
  98, 86, 98, 63, 229, 38, 160, 246, 38, 199, 199, 201, 165, 11, 123, 80,
  240, 15, 245, 40, 118, 34, 150, 228, 218, 94, 144, 241, 224, 96, 140, 95,
  80, 187, 119, 1, 188, 216, 118, 227, 168, 96, 15, 14, 43, 124, 168, 53,
  197, 7, 57, 255, 224, 162, 227, 16, 110, 174, 47, 73, 123, 126, 100, 63,
  56, 206, 70, 24, 136, 207, 144, 102, 160, 136, 186, 85, 66, 189, 1, 254,
  229, 181, 128, 37, 106, 91, 103, 61, 132, 20, 46, 201, 12, 129, 64, 166,
  23, 217, 25, 8, 221, 13, 18, 84, 180, 171, 184, 16, 57, 42, 59, 68,
  11, 187, 124, 43, 211, 91, 208, 139, 252, 45, 200, 125, 177, 185, 5, 185,
  119, 25, 109, 24, 133, 53, 173, 168, 237, 176, 177, 29, 237, 100, 103, 254,
  246, 99, 175, 118, 144, 70, 236, 255, 138, 11, 119, 15, 214, 173, 192, 174,
  126, 91, 216, 133, 115, 182, 45, 64, 66, 2, 165, 16, 101, 133, 192, 40,
  127, 54, 34, 148, 107, 153, 194, 137, 11, 226, 13, 102, 219, 14, 140, 159,
  126, 18, 24, 92, 185, 30, 28, 242, 61, 160, 0, 209, 136, 74, 123, 144,
  164, 193, 253, 107, 79, 245, 233, 50, 123, 66, 226, 66, 44, 255, 41, 207,
  46, 248, 61, 197, 242, 51, 105, 230, 122, 177, 76, 145, 117, 91, 10, 14,
  209, 97, 245, 219, 223, 227, 74, 66, 201, 211, 179, 7, 47, 239, 24, 121,
  192, 44, 163, 179, 177, 139, 85, 236, 150, 174, 92, 96, 187, 9, 33, 125,
  96, 98, 136, 240, 39, 103, 99, 58, 80, 21, 251, 180, 41, 242, 17, 199,
  23, 44, 130, 168, 168, 255, 148, 170, 30, 247, 81, 246, 179, 36, 30, 44,
  9, 120, 215, 204, 36, 35, 90, 90, 6, 81, 110, 132, 149, 212, 204, 105,
  67, 158, 236, 4, 240, 60, 11, 190, 67, 30, 31, 198, 57, 136, 211, 32,
  62, 226, 7, 182, 67, 76, 96, 223, 47, 166, 5, 84, 205, 172, 88, 206,
  22, 25, 221, 136, 208, 106, 56, 44, 49, 65, 63, 26, 197, 242, 97, 2,
  139, 18, 141, 207, 114, 204, 248, 208, 164, 37, 162, 190, 25, 107, 122, 253,
  166, 4, 133, 6, 167, 202, 53, 142, 17, 243, 233, 153, 163, 6, 162, 147,
  136, 232, 41, 208, 123, 100, 13, 107, 174, 41, 169, 39, 31, 209, 202, 108,
  196, 227, 244, 71, 129, 33, 150, 98, 85, 80, 191, 22, 119, 196, 214, 236,
  91, 138, 172, 38, 223, 89, 245, 253, 144, 151, 212, 163, 222, 165, 54, 106,
  253, 172, 225, 54, 91, 89, 144, 182, 179, 116, 200, 174, 188, 10, 122, 107,
  132, 29, 155, 220, 10, 204, 114, 181, 1, 57, 186, 185, 68, 207, 172, 188,
  220, 166, 76, 54, 210, 93, 32, 155, 193, 172, 203, 84, 59, 148, 84, 103,
  87, 1, 239, 179, 18, 4, 169, 37, 206, 172, 175, 78, 222, 106, 167, 151,
  192, 113, 199, 163, 248, 146, 74, 132, 45, 30, 109, 90, 16, 161, 39, 220,
  166, 208, 173, 22, 124, 219, 41, 191, 83, 206, 142, 143, 153, 235, 6, 54,
  167, 246, 124, 216, 35, 162, 211, 50, 57, 239, 184, 1, 49, 237, 1, 237,
  179, 132, 80, 84, 169, 43, 144, 120, 159, 166, 193, 175, 127, 249, 239, 224,
  75, 156, 128, 240, 27, 101, 167, 32, 150, 130, 7, 143, 78, 14, 91, 175,
  131, 42, 206, 172, 241, 83, 207, 204, 179, 208, 165, 103, 154, 164, 69, 232,
  89, 18, 147, 226, 93, 49, 68, 168, 122, 130, 223, 21, 185, 205, 126, 220,
  128, 152, 22, 34, 111, 104, 21, 251, 226, 129, 38, 70, 221, 153, 2, 213,
  237, 60, 30, 36, 57, 27, 244, 82, 153, 45, 97, 16, 226, 162, 67, 44,
  9, 223, 192, 58, 123, 113, 126, 14, 138, 78, 133, 38, 40, 179, 224, 205,
  254, 254, 206, 222, 211, 96, 2, 189, 98, 255, 6, 160, 91, 96, 139, 152,
  132, 140, 161, 227, 17, 76, 134, 189, 206, 221, 13, 124, 193, 89, 169, 40,
  58, 252, 131, 233, 37, 105, 127, 52, 25, 196, 69, 215, 154, 250, 208, 16,
  151, 117, 112, 46, 131, 180, 135, 245, 211, 177, 19, 134, 179, 76, 200, 140,
  36, 37, 67, 41, 214, 249, 73, 49, 148, 75, 57, 187, 87, 118, 158, 244,
  225, 235, 60, 91, 98, 90, 91, 7, 44, 173, 36, 69, 194, 46, 241, 88,
  245, 74, 110, 90, 235, 122, 193, 130, 67, 94, 174, 60, 48, 88, 188, 25,
  246, 4, 192, 239, 163, 225, 120, 98, 208, 254, 33, 30, 87, 14, 2, 158,
  118, 199, 176, 18, 173, 181, 216, 154, 164, 132, 179, 138, 133, 137, 80, 74,
  231, 202, 80, 248, 1, 56, 35, 79, 6, 68, 228, 121, 181, 57, 79, 70,
  112, 60, 27, 141, 162, 49, 123, 150, 213, 98, 27, 150, 54, 74, 37, 101,
  137, 246, 168, 141, 179, 170, 69, 109, 156, 213, 87, 226, 162, 168, 47, 58,
  136, 183, 94, 102, 151, 73, 114, 138, 144, 121, 133, 172, 164, 20, 2, 173,
  149, 117, 173, 25, 168, 95, 70, 152, 202, 79, 107, 40, 172, 31, 194, 171,
  108, 112, 229, 146, 251, 21, 123, 216, 108, 113, 177, 222, 143, 117, 193, 177,
  200, 186, 156, 137, 235, 155, 145, 209, 117, 232, 114, 107, 1, 91, 13, 253,
  250, 183, 159, 89, 59, 191, 254, 237, 127, 168, 86, 182, 6, 73, 89, 237,
  173, 226, 87, 83, 231, 197, 250, 243, 169, 45, 149, 198, 183, 55, 57, 59,
  139, 114, 27, 131, 151, 145, 40, 182, 171, 167, 140, 104, 192, 37, 141, 248,
  64, 10, 19, 217, 171, 26, 205, 171, 166, 45, 174, 0, 185, 26, 1, 108,
  86, 202, 138, 132, 255, 227, 107, 126, 37, 189, 127, 128, 130, 183, 233, 120,
  82, 234, 143, 225, 177, 208, 149, 182, 116, 139, 246, 121, 197, 24, 198, 225,
  22, 236, 25, 241, 8, 184, 1, 142, 152, 35, 140, 153, 140, 29, 152, 98,
  166, 49, 21, 237, 56, 60, 180, 82, 228, 181, 186, 253, 193, 155, 69, 250,
  238, 199, 182, 136, 25, 60, 106, 148, 245, 170, 184, 250, 250, 181, 16, 207,
  197, 153, 115, 90, 56, 31, 86, 137, 185, 248, 33, 31, 49, 26, 34, 171,
  232, 70, 192, 163, 168, 136, 15, 88, 244, 32, 205, 254, 39, 207, 76, 2,
  70, 9, 101, 3, 130, 178, 215, 228, 89, 153, 129, 0, 16, 185, 166, 172,
  166, 198, 226, 43, 213, 148, 9, 105, 54, 40, 225, 168, 81, 73, 214, 176,
  219, 82, 76, 68, 13, 75, 2, 153, 205, 72, 16, 210, 232, 135, 172, 3,
  211, 91, 13, 76, 92, 100, 137, 242, 74, 189, 214, 235, 41, 252, 194, 148,
  96, 193, 144, 6, 175, 162, 96, 93, 83, 22, 47, 241, 187, 218, 23, 169,
  133, 182, 195, 197, 128, 193, 70, 70, 153, 205, 70, 212, 157, 218, 130, 117,
  167, 214, 230, 250, 12, 150, 133, 158, 90, 199, 23, 85, 139, 69, 228, 209,
  111, 68, 155, 35, 250, 56, 201, 89, 180, 128, 62, 125, 211, 157, 183, 242,
  255, 170, 150, 157, 105, 80, 12, 136, 75, 89, 179, 134, 63, 126, 139, 171,
  224, 162, 195, 67, 173, 26, 168, 197, 114, 97, 178, 138, 171, 180, 112, 6,
  25, 136, 128, 47, 157, 167, 173, 245, 109, 42, 82, 176, 47, 5, 29, 134,
  226, 23, 182, 98, 208, 239, 187, 66, 36, 124, 72, 71, 87, 82, 121, 9,
  111, 49, 3, 106, 17, 81, 175, 18, 64, 48, 136, 60, 163, 186, 4, 17,
  146, 123, 93, 196, 47, 164, 34, 227, 58, 82, 106, 65, 235, 127, 168, 245,
  64, 202, 31, 140, 132, 199, 26, 11, 91, 73, 34, 31, 58, 41, 93, 248,
  3, 120, 143, 36, 194, 254, 226, 241, 140, 212, 105, 12, 33, 228, 107, 70,
  74, 23, 217, 140, 45, 132, 236, 0, 47, 174, 72, 176, 177, 171, 77, 223,
  18, 18, 198, 224, 120, 196, 212, 129, 26, 156, 85, 181, 73, 150, 212, 18,
  206, 149, 10, 170, 25, 82, 246, 208, 225, 103, 196, 27, 44, 222, 134, 198,
  58, 198, 54, 72, 198, 176, 169, 143, 134, 83, 97, 36, 249, 88, 89, 168,
  100, 159, 137, 27, 26, 162, 183, 117, 91, 130, 143, 80, 74, 226, 203, 166,
  232, 109, 2, 57, 96, 181, 46, 9, 164, 47, 80, 15, 23, 135, 119, 18,
  165, 135, 200, 94, 49, 163, 44, 212, 115, 153, 182, 19, 123, 152, 108, 180,
  85, 24, 222, 121, 122, 83, 19, 81, 247, 6, 18, 57, 32, 118, 184, 166,
  61, 17, 189, 29, 24, 64, 151, 212, 12, 103, 25, 19, 238, 109, 1, 51,
  153, 51, 39, 187, 54, 118, 135, 27, 166, 110, 96, 77, 44, 227, 16, 106,
  211, 165, 54, 6, 199, 214, 43, 162, 71, 153, 210, 150, 157, 36, 73, 242,
  19, 67, 180, 30, 252, 203, 222, 135, 247, 61, 230, 20, 208, 45, 217, 153,
  98, 13, 129, 108, 102, 209, 240, 153, 113, 221, 120, 154, 77, 244, 116, 230,
  225, 183, 153, 121, 138, 233, 58, 88, 136, 42, 180, 82, 172, 69, 129, 19,
  164, 70, 250, 110, 15, 148, 2, 201, 254, 95, 23, 190, 11, 184, 91, 108,
  216, 5, 7, 178, 74, 21, 134, 218, 69, 57, 214, 220, 6, 164, 163, 128,
  101, 88, 226, 34, 51, 207, 209, 218, 38, 15, 6, 121, 94, 157, 5, 40,
  255, 134, 185, 120, 11, 6, 15, 39, 36, 100, 173, 37, 188, 56, 74, 48,
  5, 12, 114, 91, 137, 119, 98, 56, 152, 233, 253, 107, 214, 71, 168, 21,
  252, 250, 159, 255, 21, 240, 159, 83, 126, 94, 154, 178, 115, 19, 3, 183,
  150, 155, 4, 103, 157, 76, 151, 163, 206, 148, 69, 192, 195, 241, 0, 38,
  17, 15, 15, 126, 85, 136, 106, 217, 154, 117, 83, 147, 31, 8, 228, 13,
  132, 141, 100, 192, 128, 235, 174, 74, 120, 93, 249, 209, 175, 241, 148, 142,
  186, 95, 210, 154, 150, 36, 81, 124, 49, 28, 58, 214, 248, 204, 77, 239,
  74, 171, 236, 16, 116, 252, 250, 98, 139, 53, 247, 146, 253, 223, 229, 186,
  224, 45, 234, 148, 2, 225, 63, 250, 22, 200, 187, 41, 143, 150, 152, 51,
  90, 233, 98, 179, 28, 232, 205, 164, 164, 30, 158, 112, 239, 71, 125, 252,
  65, 244, 106, 93, 191, 55, 25, 242, 252, 214, 113, 105, 221, 167, 96, 121,
  54, 30, 151, 157, 167, 117, 109, 53, 166, 5, 55, 67, 116, 249, 25, 235,
  182, 87, 197, 203, 83, 113, 196, 188, 189, 117, 225, 61, 198, 102, 248, 170,
  120, 195, 50, 52, 57, 251, 247, 130, 8, 128, 46, 238, 50, 150, 180, 186,
  193, 69, 146, 159, 2, 223, 157, 4, 163, 95, 254, 23, 179, 36, 164, 235,
  29, 111, 140, 184, 249, 143, 102, 188, 163, 191, 19, 93, 82, 158, 249, 124,
  70, 39, 207, 38, 166, 14, 80, 94, 171, 142, 7, 112, 54, 27, 203, 156,
  86, 22, 39, 73, 109, 205, 181, 130, 87, 14, 24, 246, 254, 26, 27, 191,
  124, 29, 79, 154, 222, 111, 108, 124, 119, 172, 236, 51, 91, 192, 233, 171,
  249, 27, 25, 234, 45, 147, 252, 124, 162, 67, 95, 152, 195, 152, 175, 200,
  127, 132, 67, 202, 182, 144, 13, 95, 127, 179, 215, 159, 218, 200, 119, 53,
  173, 158, 220, 84, 15, 110, 52, 176, 208, 197, 52, 83, 154, 94, 237, 113,
  206, 243, 96, 65, 255, 173, 69, 138, 53, 31, 113, 114, 55, 38, 124, 98,
  2, 18, 111, 39, 186, 98, 62, 115, 222, 140, 13, 156, 156, 145, 21, 212,
  80, 149, 90, 241, 20, 181, 202, 161, 1, 218, 28, 57, 192, 178, 204, 113,
  91, 156, 29, 219, 17, 43, 160, 142, 231, 118, 6, 75, 137, 206, 136, 202,
  161, 1, 218, 216, 25, 237, 224, 73, 245, 96, 15, 149, 4, 130, 30, 76,
  163, 32, 8, 34, 170, 135, 38, 116, 99, 47, 116, 61, 82, 168, 43, 158,
  238, 100, 99, 170, 55, 217, 152, 236, 76, 54, 214, 251, 146, 141, 231, 232,
  10, 98, 118, 81, 251, 28, 105, 230, 161, 246, 12, 215, 63, 141, 175, 202,
  109, 214, 98, 231, 84, 162, 37, 82, 80, 183, 95, 140, 173, 183, 175, 219,
  221, 188, 110, 123, 235, 186, 225, 198, 117, 187, 219, 214, 212, 157, 166, 246,
  120, 103, 152, 58, 25, 10, 224, 54, 46, 242, 103, 227, 3, 137, 161, 222,
  91, 97, 253, 119, 173, 202, 204, 163, 174, 16, 206, 175, 117, 68, 97, 143,
  26, 239, 136, 34, 126, 213, 172, 37, 65, 116, 127, 134, 121, 232, 242, 115,
  135, 142, 219, 108, 63, 47, 96, 71, 154, 170, 63, 234, 119, 227, 74, 240,
  157, 218, 168, 248, 85, 113, 185, 25, 159, 179, 140, 74, 163, 184, 219, 54,
  120, 213, 32, 62, 95, 202, 210, 209, 149, 230, 145, 133, 111, 115, 95, 96,
  156, 56, 221, 207, 82, 166, 75, 112, 179, 97, 178, 16, 247, 27, 70, 76,
  39, 173, 208, 189, 45, 1, 76, 246, 91, 109, 85, 100, 139, 79, 13, 81,
  168, 55, 229, 204, 164, 232, 30, 26, 13, 44, 231, 48, 19, 161, 106, 41,
  172, 250, 81, 135, 140, 118, 56, 243, 168, 158, 60, 180, 247, 185, 177, 227,
  139, 50, 115, 191, 175, 42, 134, 26, 208, 12, 98, 17, 230, 154, 69, 185,
  150, 179, 93, 209, 81, 132, 229, 118, 94, 136, 170, 0, 2, 106, 216, 160,
  131, 219, 153, 219, 160, 210, 71, 60, 70, 178, 90, 104, 0, 54, 43, 56,
  1, 189, 49, 94, 252, 200, 30, 177, 42, 181, 116, 28, 173, 114, 104, 128,
  54, 107, 160, 205, 163, 37, 149, 82, 246, 148, 26, 20, 107, 187, 135, 90,
  185, 149, 107, 219, 0, 8, 45, 4, 115, 108, 91, 131, 216, 96, 114, 70,
  210, 217, 114, 101, 27, 211, 229, 56, 58, 10, 86, 182, 77, 48, 124, 255,
  18, 175, 124, 55, 183, 62, 30, 108, 124, 216, 220, 50, 247, 49, 201, 227,
  238, 9, 158, 206, 21, 12, 36, 8, 152, 103, 42, 79, 233, 220, 42, 217,
  182, 109, 99, 107, 61, 143, 22, 144, 46, 214, 26, 162, 166, 212, 228, 244,
  204, 3, 70, 254, 150, 93, 183, 252, 133, 221, 67, 152, 101, 94, 51, 99,
  248, 184, 201, 102, 234, 162, 37, 169, 68, 8, 120, 221, 243, 110, 107, 127,
  247, 237, 198, 1, 252, 121, 249, 73, 7, 251, 9, 249, 229, 58, 152, 164,
  73, 185, 214, 193, 108, 88, 209, 81, 60, 90, 11, 244, 42, 102, 102, 69,
  12, 107, 212, 79, 206, 96, 240, 107, 171, 193, 212, 100, 142, 171, 151, 151,
  73, 177, 141, 24, 194, 160, 250, 223, 117, 61, 199, 62, 245, 88, 75, 236,
  33, 17, 251, 137, 61, 48, 158, 19, 33, 194, 75, 13, 33, 97, 65, 232,
  159, 138, 7, 35, 64, 51, 247, 29, 67, 21, 176, 234, 157, 122, 154, 206,
  2, 231, 135, 193, 165, 175, 115, 157, 63, 199, 73, 25, 116, 223, 188, 89,
  123, 247, 110, 109, 111, 143, 187, 196, 135, 150, 25, 88, 57, 121, 219, 216,
  185, 255, 111, 43, 228, 28, 243, 34, 58, 219, 147, 232, 103, 64, 98, 194,
  147, 143, 136, 188, 216, 72, 250, 224, 14, 197, 91, 57, 59, 91, 43, 10,
  88, 43, 240, 63, 59, 197, 134, 108, 179, 210, 123, 144, 199, 163, 8, 150,
  111, 216, 248, 12, 169, 114, 226, 172, 34, 116, 234, 92, 198, 219, 63, 31,
  15, 248, 75, 91, 51, 68, 232, 115, 50, 18, 43, 11, 96, 57, 30, 8,
  124, 27, 81, 122, 30, 129, 212, 178, 75, 72, 151, 77, 217, 135, 230, 141,
  157, 69, 9, 99, 65, 224, 4, 126, 167, 168, 169, 129, 122, 215, 92, 196,
  179, 117, 54, 46, 175, 132, 227, 55, 251, 191, 81, 39, 229, 212, 60, 137,
  10, 12, 252, 187, 201, 111, 116, 125, 115, 9, 181, 118, 227, 62, 204, 247,
  94, 116, 54, 30, 217, 65, 202, 30, 175, 224, 203, 54, 232, 30, 84, 67,
  68, 111, 83, 230, 122, 238, 132, 50, 147, 184, 29, 197, 43, 75, 69, 84,
  91, 95, 251, 60, 21, 201, 219, 2, 72, 50, 42, 79, 174, 44, 204, 164,
  117, 202, 242, 161, 51, 210, 180, 188, 225, 253, 52, 227, 188, 49, 215, 117,
  157, 28, 248, 155, 247, 204, 149, 28, 28, 114, 59, 30, 130, 112, 149, 119,
  158, 252, 151, 241, 105, 17, 243, 210, 145, 238, 250, 45, 230, 203, 209, 253,
  7, 48, 198, 50, 230, 48, 219, 209, 85, 54, 41, 11, 17, 24, 218, 19,
  155, 209, 19, 53, 186, 38, 58, 163, 189, 1, 152, 47, 1, 52, 174, 180,
  247, 236, 59, 239, 217, 77, 169, 78, 196, 29, 20, 87, 213, 85, 145, 91,
  73, 196, 221, 107, 23, 164, 207, 19, 64, 48, 36, 28, 28, 198, 89, 146,
  150, 236, 205, 33, 156, 63, 89, 68, 180, 4, 14, 81, 93, 114, 141, 240,
  71, 26, 31, 149, 76, 210, 5, 86, 121, 185, 232, 72, 43, 125, 169, 241,
  197, 1, 176, 251, 44, 116, 111, 177, 232, 38, 231, 225, 103, 161, 69, 209,
  37, 139, 27, 224, 92, 86, 148, 47, 211, 132, 167, 75, 126, 157, 131, 42,
  34, 242, 248, 126, 205, 62, 213, 235, 86, 66, 144, 37, 248, 230, 243, 106,
  71, 18, 182, 202, 117, 198, 85, 159, 55, 252, 123, 27, 161, 100, 120, 178,
  224, 56, 55, 43, 121, 237, 133, 95, 52, 123, 176, 200, 180, 166, 121, 215,
  62, 247, 188, 80, 82, 223, 146, 4, 77, 110, 242, 72, 218, 100, 164, 66,
  47, 82, 226, 145, 169, 108, 68, 72, 70, 140, 112, 107, 197, 18, 225, 69,
  68, 28, 17, 246, 97, 205, 90, 193, 98, 50, 38, 103, 22, 22, 40, 73,
  6, 73, 121, 69, 98, 82, 31, 105, 108, 192, 76, 22, 54, 86, 162, 35,
  66, 63, 65, 21, 199, 229, 125, 244, 190, 43, 171, 133, 178, 13, 4, 161,
  209, 99, 76, 76, 251, 220, 127, 206, 73, 232, 158, 253, 43, 144, 247, 153,
  216, 52, 77, 16, 94, 236, 130, 120, 86, 188, 80, 39, 123, 197, 8, 229,
  202, 210, 227, 21, 251, 36, 165, 245, 14, 55, 164, 170, 101, 250, 112, 165,
  102, 11, 71, 138, 16, 108, 42, 180, 223, 146, 116, 242, 183, 65, 53, 132,
  14, 157, 82, 64, 193, 10, 249, 16, 122, 163, 56, 29, 162, 47, 21, 32,
  89, 49, 183, 129, 170, 115, 45, 118, 179, 106, 96, 182, 122, 122, 26, 163,
  54, 128, 110, 65, 105, 167, 221, 230, 84, 215, 176, 165, 246, 120, 155, 61,
  252, 184, 179, 25, 220, 191, 6, 250, 192, 105, 229, 117, 114, 9, 147, 243,
  32, 156, 6, 167, 59, 145, 118, 162, 112, 165, 30, 206, 185, 20, 121, 236,
  127, 33, 239, 248, 196, 122, 101, 157, 172, 125, 194, 174, 104, 8, 233, 59,
  155, 236, 189, 221, 94, 16, 65, 241, 17, 110, 159, 123, 27, 154, 184, 122,
  163, 236, 130, 189, 80, 54, 75, 79, 146, 33, 115, 95, 187, 127, 237, 86,
  215, 233, 251, 235, 95, 254, 234, 84, 65, 88, 103, 14, 216, 131, 49, 226,
  21, 191, 24, 2, 135, 117, 86, 35, 47, 182, 205, 19, 198, 199, 208, 68,
  97, 167, 229, 197, 188, 187, 236, 24, 254, 252, 243, 189, 17, 83, 47, 150,
  138, 11, 188, 149, 254, 124, 79, 229, 227, 61, 138, 250, 167, 67, 22, 9,
  98, 141, 49, 16, 83, 32, 186, 4, 157, 96, 17, 173, 244, 190, 15, 167,
  159, 239, 189, 16, 41, 122, 131, 63, 39, 241, 8, 216, 174, 162, 240, 244,
  208, 49, 226, 21, 113, 249, 22, 243, 96, 158, 71, 163, 46, 187, 110, 70,
  78, 135, 131, 223, 119, 90, 64, 9, 189, 14, 223, 6, 246, 69, 124, 158,
  73, 177, 104, 198, 158, 24, 103, 227, 9, 28, 185, 98, 172, 240, 5, 86,
  135, 80, 126, 116, 84, 148, 97, 195, 198, 90, 125, 193, 29, 101, 47, 46,
  49, 8, 166, 86, 170, 167, 230, 54, 235, 242, 52, 240, 178, 196, 127, 129,
  78, 103, 42, 29, 231, 208, 5, 185, 157, 155, 88, 152, 86, 47, 138, 128,
  192, 44, 251, 236, 139, 127, 194, 255, 209, 85, 134, 255, 119, 82, 158, 141,
  240, 191, 255, 3, 74, 97, 219, 74, 39, 179, 2, 0
};
static constexpr size_t INDEX_HTML_GZ_LEN = sizeof(INDEX_HTML_GZ);


void handleRoot() {
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Content-Encoding", "gzip");
  server.send_P(200, "text/html", reinterpret_cast<PGM_P>(INDEX_HTML_GZ), INDEX_HTML_GZ_LEN);
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
  uint64_t epochMs = currentEpochMs();
  bool syncedFlag = isTimeSynced();
  auto ageMs = [&](unsigned long t) -> unsigned long { return t == 0 ? 0UL : (now - t); };
  auto sensorLabel = [](String v) { v.toUpperCase(); return v; };
  String luxDev = sensorLabel(findSensor("lux") ? findSensor("lux")->type : "BH1750");
  String climateDev = sensorLabel(findSensor("climate") ? findSensor("climate")->type : climateSensorName(climateType));
  String leafDev = sensorLabel(findSensor("leaf") ? findSensor("leaf")->type : "MLX90614");
  String co2Dev = sensorLabel(findSensor("co2") ? findSensor("co2")->type : co2SensorName(co2Type));
  auto sampleTsFor = [&](const String &id) -> unsigned long long {
    MetricSampleInfo *info = sampleInfoById(id);
    return info ? (unsigned long long)info->lastEpochMs : 0ULL;
  };
  auto everFor = [&](const String &id) -> int {
    MetricSampleInfo *info = sampleInfoById(id);
    return info && info->everHadData ? 1 : 0;
  };

  char json[1800];
  snprintf(json, sizeof(json),
           "{\"lux\":%.1f,\"ppfd\":%.1f,\"ppfd_factor\":%.4f,\"co2\":%d,\"temp\":%.1f,\"humidity\":%.1f,\"leaf\":%.1f,"
           "\"vpd\":%.3f,\"vpd_low\":%.2f,\"vpd_high\":%.2f,\"vpd_status\":%d,"
           "\"wifi_connected\":%d,\"connecting\":%d,\"ap_mode\":%d,\"ip\":\"%s\",\"gw\":\"%s\",\"sn\":\"%s\",\"rssi\":%d,\"ssid\":\"%s\","
           "\"lux_ok\":%d,\"co2_ok\":%d,\"climate_ok\":%d,\"leaf_ok\":%d,\"vpd_ok\":%d,"
           "\"lux_present\":%d,\"co2_present\":%d,\"climate_present\":%d,\"leaf_present\":%d,"
           "\"lux_enabled\":%d,\"co2_enabled\":%d,\"climate_enabled\":%d,\"leaf_enabled\":%d,"
           "\"lux_age_ms\":%lu,\"co2_age_ms\":%lu,\"climate_age_ms\":%lu,\"leaf_age_ms\":%lu,"
           "\"now\":%lu,\"monotonic_ms\":%lu,\"epoch_ms\":%llu,\"time_synced\":%d,\"timezone\":\"%s\","
           "\"lux_device\":\"%s\",\"climate_device\":\"%s\",\"leaf_device\":\"%s\",\"co2_device\":\"%s\","
           "\"lux_last\":%llu,\"co2_last\":%llu,\"temp_last\":%llu,\"humidity_last\":%llu,\"leaf_last\":%llu,\"vpd_last\":%llu,"
           "\"lux_ever\":%d,\"co2_ever\":%d,\"temp_ever\":%d,\"humidity_ever\":%d,\"leaf_ever\":%d,\"vpd_ever\":%d}",
           safeFloat(latest.lux), safeFloat(latest.ppfd), safeFloat(latest.ppfdFactor), safeInt(latest.co2ppm, -1), safeFloat(latest.ambientTempC),
           safeFloat(latest.humidity), safeFloat(latest.leafTempC), safeFloat(latest.vpd), safeFloat(latest.vpdTargetLow),
           safeFloat(latest.vpdTargetHigh), latest.vpdStatus,
           wifiConnected ? 1 : 0, wifiConnectInProgress ? 1 : 0, apMode ? 1 : 0, ipStr.c_str(), gwStr.c_str(), snStr.c_str(), rssi, ssid.c_str(),
           luxOk ? 1 : 0, co2Ok ? 1 : 0, climateOk ? 1 : 0, leafOk ? 1 : 0, vpdOk ? 1 : 0,
           lightHealth.present ? 1 : 0, co2Health.present ? 1 : 0, climateHealth.present ? 1 : 0, leafHealth.present ? 1 : 0,
           enableLight ? 1 : 0, enableCo2 ? 1 : 0, enableClimate ? 1 : 0, enableLeaf ? 1 : 0,
           ageMs(lightHealth.lastUpdate), ageMs(co2Health.lastUpdate), ageMs(climateHealth.lastUpdate), ageMs(leafHealth.lastUpdate),
           now, now, (unsigned long long)epochMs, syncedFlag ? 1 : 0, timezoneName.c_str(),
           luxDev.c_str(), climateDev.c_str(), leafDev.c_str(), co2Dev.c_str(),
           sampleTsFor("lux"), sampleTsFor("co2"), sampleTsFor("temp"), sampleTsFor("humidity"), sampleTsFor("leaf"), sampleTsFor("vpd"),
           everFor("lux"), everFor("co2"), everFor("temp"), everFor("humidity"), everFor("leaf"), everFor("vpd"));
  server.send(200, "application/json", json);
}

void handlePing() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", "{\"ok\":1}");
}

void handleStatus() {
  bool wifiConnected = WiFi.status() == WL_CONNECTED;
  IPAddress activeIp = apMode ? WiFi.softAPIP() : WiFi.localIP();
  String ipStr = activeIp.toString();
  String ssid = wifiConnected ? WiFi.SSID() : savedSsid;
  int rssi = wifiConnected ? WiFi.RSSI() : 0;
  bool connecting = wifiConnectInProgress && !wifiConnected;
  String json = "{";
  json += "\"wifi_connected\":" + String(wifiConnected ? 1 : 0) + ",";
  json += "\"connecting\":" + String(connecting ? 1 : 0) + ",";
  json += "\"ap_mode\":" + String(apMode ? 1 : 0) + ",";
  json += "\"ssid\":\"" + ssid + "\",";
  json += "\"rssi\":" + String(rssi) + ",";
  json += "\"ip\":\"" + ipStr + "\",";
  json += "\"hostname\":\"" + deviceHostname + "\"";
  json += "}";
  server.sendHeader("Access-Control-Allow-Origin", "*");
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

  bool syncedFlag = isTimeSynced();
  json += "],\"time_synced\":" + String(syncedFlag ? 1 : 0) + ",\"timezone\":\"" + timezoneName + "\",\"epoch_base_ms\":" + String((unsigned long long)(syncedFlag ? currentEpochMs() : 0)) + "}";
  server.send(200, "application/json", json);
}

time_t parseDayKey(const String &dayKey) {
  int y, m, d;
  if (sscanf(dayKey.c_str(), "%d-%d-%d", &y, &m, &d) != 3) return 0;
  struct tm t = {};
  t.tm_year = y - 1900;
  t.tm_mon = m - 1;
  t.tm_mday = d;
  return mktime(&t);
}

std::vector<String> recentDayKeys(int days) {
  std::vector<String> keys;
  if (!isTimeSynced()) return keys;
  if (days < 1) return keys;
  uint64_t now = currentEpochMs();
  const uint64_t dayMs = 24ULL * 60ULL * 60ULL * 1000ULL;
  for (int i = days - 1; i >= 0; --i) {
    uint64_t ts = now > (uint64_t)i * dayMs ? now - ((uint64_t)i * dayMs) : 0;
    keys.push_back(dayKeyForEpoch(ts));
  }
  return keys;
}

String formatDayKey(time_t ts) {
  struct tm t;
  if (localtime_r(&ts, &t) == nullptr) return "";
  char buf[16];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d", t.tm_year + 1900, t.tm_mon + 1, t.tm_mday);
  return String(buf);
}

std::vector<String> dayKeysBetween(const String &from, const String &to) {
  std::vector<String> keys;
  if (!isTimeSynced()) return keys;
  time_t start = parseDayKey(from);
  time_t end = parseDayKey(to);
  if (start == 0 || end == 0 || start > end) return keys;
  const time_t daySec = 24 * 60 * 60;
  for (time_t t = start; t <= end; t += daySec) {
    String key = formatDayKey(t);
    if (key.length() > 0) keys.push_back(key);
  }
  return keys;
}

void handleDailyHistory() {
  if (!enforceAuth()) return;
  if (!cloudStorageActive()) {
    server.send(403, "text/plain", "cloud inactive");
    return;
  }
  String metric = server.hasArg("metric") ? server.arg("metric") : "";
  int days = server.hasArg("days") ? server.arg("days").toInt() : 30;
  if (days < 1) days = 1;
  if (days > 120) days = 120;
  days = std::min(days, (int)retentionDays());
  int idx = metricIndex(metric);
  if (idx < 0) { server.send(400, "text/plain", "invalid metric"); return; }
  pruneLogsIfLowMemory(false);
  bool cloudOk = cloudStorageActive();
  std::vector<DailyPoint> points;
  DAILY_HISTORY[idx].clear();
  if (cloudOk) {
    auto keys = recentDayKeys(days);
    for (const auto &k : keys) {
      DailyPoint p;
      if (fetchCloudDailyPoint(k, metric, p)) {
        DAILY_HISTORY[idx].push_back(p);
      }
    }
    points = DAILY_HISTORY[idx];
  }
  String json;
  json.reserve(256);
  json = "{\"metric\":\"" + metric + "\",\"points\":[";
  bool first = true;
  auto appendPoint = [&](const DailyPoint &p) {
    if (p.dayKey.length() == 0 || p.count == 0) return;
    time_t ts = parseDayKey(p.dayKey);
    if (!first) json += ",";
    first = false;
    json += "[" + String((unsigned long long)ts * 1000ULL) + ",";
    json += "{\"avg\":" + String((double)p.avg, (unsigned int)HISTORY_METRICS[idx].decimals) + ",";
    json += "\"min\":" + String((double)p.min, (unsigned int)HISTORY_METRICS[idx].decimals) + ",";
    json += "\"max\":" + String((double)p.max, (unsigned int)HISTORY_METRICS[idx].decimals) + ",";
    json += "\"last\":" + String((double)p.last, (unsigned int)HISTORY_METRICS[idx].decimals) + ",";
    json += "\"count\":" + String(p.count) + "}";
    json += "]";
  };
  for (const auto &p : points) appendPoint(p);
  json += "],\"time_synced\":" + String((isTimeSynced() && cloudOk) ? 1 : 0) + ",\"cloud\":"
      + String(cloudOk ? 1 : 0) + "}";
  server.send(200, "application/json", json);
}

void handleCloudDaily() {
  if (!enforceAuth()) return;
  if (!cloudStorageActive()) {
    server.send(403, "text/plain", "cloud inactive");
    return;
  }
  String sensor = server.hasArg("sensor") ? server.arg("sensor") : "";
  if (sensor.length() == 0 && server.hasArg("metric")) sensor = server.arg("metric");
  String from = server.hasArg("from") ? server.arg("from") : "";
  String to = server.hasArg("to") ? server.arg("to") : "";
  if (sensor.length() == 0 || from.length() == 0 || to.length() == 0) {
    server.send(400, "text/plain", "missing sensor/from/to");
    return;
  }
  int idx = metricIndex(sensor);
  if (idx < 0) {
    server.send(400, "text/plain", "invalid sensor");
    return;
  }
  static String lastQuery;
  static String lastResponse;
  static unsigned long lastQueryAt = 0;
  String queryKey = sensor + "|" + from + "|" + to;
  unsigned long now = millis();
  if (queryKey == lastQuery && lastResponse.length() > 0 && now - lastQueryAt < 20000) {
    server.send(200, "application/json", lastResponse);
    return;
  }
  auto keys = dayKeysBetween(from, to);
  std::vector<DailyPoint> points;
  points.reserve(keys.size());
  for (const auto &k : keys) {
    DailyPoint p;
    if (fetchCloudDailyPoint(k, sensor, p)) {
      points.push_back(p);
    }
  }
  String json;
  json.reserve(256 + points.size() * 64);
  json = "{\"sensor\":\"" + sensor + "\",\"points\":[";
  bool first = true;
  for (const auto &p : points) {
    if (p.dayKey.length() == 0 || p.count == 0) continue;
    time_t ts = parseDayKey(p.dayKey);
    if (!first) json += ",";
    first = false;
    json += "[" + String((unsigned long long)ts * 1000ULL) + ",";
    json += "{\"avg\":" + String((double)p.avg, (unsigned int)HISTORY_METRICS[idx].decimals) + ",";
    json += "\"min\":" + String((double)p.min, (unsigned int)HISTORY_METRICS[idx].decimals) + ",";
    json += "\"max\":" + String((double)p.max, (unsigned int)HISTORY_METRICS[idx].decimals) + ",";
    json += "\"last\":" + String((double)p.last, (unsigned int)HISTORY_METRICS[idx].decimals) + ",";
    json += "\"count\":" + String(p.count) + "}]";
  }
  json += "],\"time_synced\":" + String(isTimeSynced() ? 1 : 0) + ",\"cloud\":1}";
  lastQuery = queryKey;
  lastResponse = json;
  lastQueryAt = now;
  server.send(200, "application/json", json);
}

void handleCloudTest() {
  if (!enforceAuth()) return;
  int httpCode = 0;
  size_t bytes = 0;
  String path, err;
  bool ok = sendCloudTestFile(httpCode, bytes, path, err);
  (void)bytes;
  String payload = "{\"ok\":" + String(ok ? 1 : 0) + ",\"code\":" + String(httpCode) + ",\"path\":\"" + path + "\",\"err\":\"" + err + "\"}";
  server.send(ok ? 200 : 500, "application/json", payload);
}

void handleCloud() {
  if (!enforceAuth()) return;
  if (server.method() == HTTP_GET) {
    cloudStatus.queueSize = cloudQueue.size();
    refreshStorageMode();
    DynamicJsonDocument doc(1024);
    doc["enabled"] = cloudConfig.enabled;
    doc["runtime_enabled"] = cloudStatus.runtimeEnabled;
    doc["recording"] = cloudStatus.recording;
    doc["base_url"] = cloudConfig.baseUrl;
    doc["username"] = cloudConfig.username;
    doc["persist_credentials"] = cloudConfig.persistCredentials;
    doc["retention"] = cloudConfig.retentionMonths;
    doc["protocol"] = cloudProtocolLabel();
    doc["connected"] = cloudStatus.connected;
    doc["last_upload_ms"] = (uint64_t)cloudStatus.lastUploadMs;
    doc["last_uploaded_path"] = cloudStatus.lastUploadedPath;
    doc["last_ping_ms"] = (uint64_t)cloudStatus.lastPingMs;
    doc["last_failure_ms"] = (uint64_t)cloudStatus.lastFailureMs;
    doc["last_test_ms"] = (uint64_t)cloudStatus.lastTestMs;
    doc["last_config_ms"] = (uint64_t)cloudStatus.lastConfigSaveMs;
    doc["last_state_ms"] = (uint64_t)cloudStatus.lastStateChangeMs;
    doc["queue_size"] = (uint32_t)cloudQueue.size();
    doc["failures"] = cloudStatus.failureCount;
    doc["last_error"] = cloudStatus.lastError;
    doc["last_error_suffix"] = cloudStatus.lastErrorSuffix;
    doc["last_http_code"] = cloudStatus.lastHttpCode;
    doc["last_url"] = cloudStatus.lastUrl;
    doc["last_state_reason"] = cloudStatus.lastStateReason;
    doc["device_folder"] = cloudStatus.deviceFolder;
    doc["device_id"] = deviceId();
    doc["storage_mode"] = storageModeName(storageMode);
    String payload;
    serializeJson(doc, payload);
    server.send(200, "application/json", payload);
    return;
  }

  String action = server.hasArg("action") ? server.arg("action") : "";
  if (action == "save") {
    if (server.hasArg("base_url")) cloudConfig.baseUrl = server.arg("base_url");
    if (server.hasArg("username")) cloudConfig.username = server.arg("username");
    if (server.hasArg("password") && server.arg("password").length() > 0) cloudConfig.password = server.arg("password");
    if (server.hasArg("enabled")) cloudConfig.enabled = server.arg("enabled") == "1";
    if (server.hasArg("recording")) cloudConfig.recording = server.arg("recording") == "1";
    if (server.hasArg("persist_credentials")) cloudConfig.persistCredentials = server.arg("persist_credentials") == "1";
    if (server.hasArg("protocol")) {
      cloudConfig.protocol = server.arg("protocol") == "https" ? 1 : 0;
    }
    if (server.hasArg("retention")) {
      int r = server.arg("retention").toInt();
      cloudConfig.retentionMonths = (uint8_t)std::max(1, std::min(4, r));
    }
    saveCloudConfig("ui save");
    server.send(200, "text/plain", "saved");
    return;
  }
  if (action == "forget") {
    cloudConfig.baseUrl = "";
    cloudConfig.username = "";
    cloudConfig.password = "";
    cloudConfig.persistCredentials = false;
    saveCloudConfig("ui forget");
    server.send(200, "application/json", "{\"forgot\":1}");
    return;
  }
  if (action == "test") {
    int httpCode = 0;
    size_t bytes = 0;
    String path, err;
    bool ok = sendCloudTestFile(httpCode, bytes, path, err);
    (void)bytes;
    String payload = "{\"ok\":" + String(ok ? 1 : 0) + ",\"code\":" + String(httpCode) + ",\"path\":\"" + path + "\",\"err\":\"" + err + "\"}";
    server.send(ok ? 200 : 500, "application/json", payload);
    return;
  }
  if (action == "start") {
    setCloudEnabled(true, "ui start");
    setRecordingActive(true, "ui start");
    refreshStorageMode("ui start");
    enqueueRecordingEvent("start", "UI toggle");
    enqueueRecordingSample(currentEpochMs());
    saveCloudConfig("ui start");
    server.send(200, "application/json", "{\"enabled\":1,\"recording\":1}");
    return;
  }
  if (action == "stop") {
    setRecordingActive(false, "ui stop");
    saveCloudConfig("ui stop");
    enqueueRecordingEvent("stop", "UI toggle");
    server.send(200, "application/json", "{\"recording\":0,\"enabled\":" + String(cloudConfig.enabled ? 1 : 0) + "}");
    return;
  }
  server.send(400, "text/plain", "unsupported");
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

  prefsSystem.begin("system", false);
  prefsSystem.putInt("i2c_sda", sda);
  prefsSystem.putInt("i2c_scl", scl);
  prefsSystem.putInt("co2_rx", rx);
  prefsSystem.putInt("co2_tx", tx);
  prefsSystem.end();

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
    bool syncedFlag = isTimeSynced();
    timeSynced = syncedFlag;
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
    json += "\"time_synced\":" + String(syncedFlag ? 1 : 0) + ",";
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
  prefsSystem.begin("system", false);
  if (hasChannel) prefsSystem.putString("channel", lightChannelName());
  if (hasVpdStage) prefsSystem.putString("vpd_stage", vpdStageId);
  if (hasTimezone) prefsSystem.putString("timezone", timezoneName);
  prefsSystem.end();
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
  prefsWifi.begin("wifi", false);
  prefsWifi.putBool("static", staticFlag);
  prefsWifi.putString("ip", ip.toString());
  prefsWifi.putString("gw", gw.toString());
  prefsWifi.putString("sn", sn.toString());
  prefsWifi.end();
  staticIpEnabled = staticFlag;
  staticIp = ip;
  staticGateway = gw;
  staticSubnet = sn;

  apMode = false;
  wifiFallbackAt = millis() + WIFI_TIMEOUT + 10000;
  beginWifiConnect(ssid, pass, false);
  server.send(200, "text/plain", "connecting");
}

void handleRestart() {
  if (!enforceAuth())
    return;
  logEvent("Soft restart requested — NO ERASE performed");
  server.send(200, "text/plain", "restarting");
  delay(150);
  ESP.restart();
}

void handleFactoryReset() {
  if (!enforceAuth())
    return;
  if (!server.hasArg("confirm") || server.arg("confirm") != "RESET") {
    server.send(400, "text/plain", "confirm RESET");
    return;
  }
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
      prefsSystem.begin("system", false);
      prefsSystem.putString("climate_type", climateSensorName(climateType));
      prefsSystem.end();
      reinitClimateSensor();
      rebuildSensorList();
      server.send(200, "text/plain", "saved");
      return;
    }
    if (id == "co2_type" && server.hasArg("value")) {
      co2Type = co2FromString(server.arg("value"));
      prefsSystem.begin("system", false);
      prefsSystem.putString("co2_type", co2SensorName(co2Type));
      prefsSystem.end();
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
  server.on("/api/ping", HTTP_GET, handlePing);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/settings", HTTP_GET, handleSettings);
  server.on("/api/settings", HTTP_POST, handleSettings);
  server.on("/api/wifi", HTTP_POST, handleWifiSave);
  server.on("/api/restart", HTTP_POST, handleRestart);
  server.on("/api/factory-reset", HTTP_POST, handleFactoryReset);
  server.on("/api/reset", HTTP_POST, handleFactoryReset);
  server.on("/api/networks", HTTP_GET, handleNetworks);
  server.on("/api/sensors", HTTP_GET, handleSensorsApi);
  server.on("/api/sensors", HTTP_POST, handleSensorsApi);
  server.on("/api/sensors/config", HTTP_GET, handleSensorsConfig);
  server.on("/api/sensors/config", HTTP_POST, handleSensorsConfig);
  server.on("/api/cloud", HTTP_GET, handleCloud);
  server.on("/api/cloud", HTTP_POST, handleCloud);
  server.on("/api/cloud/test", HTTP_POST, handleCloudTest);
  server.on("/api/pins", HTTP_GET, handlePins);
  server.on("/api/pins", HTTP_POST, handlePins);
  server.on("/api/logs", HTTP_GET, handleLogs);
  server.on("/api/history", HTTP_GET, handleHistory);
  server.on("/api/history/daily", HTTP_GET, handleDailyHistory);
  server.on("/api/cloud/daily", HTTP_GET, handleCloudDaily);
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
#if defined(LED_BUILTIN)
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
#endif

  loadCloudConfig();
  loadDailyHistory();
  activeDayKey = currentDayKey();
  resetAllDaily(activeDayKey);
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
  maintainWifiConnection();
  maintainTimeSync();
  maintainCloudHealth();
  tickCloudRecording();
  flushCloudLogs();
  processCloudQueue();

  if (millis() - lastSensorMillis >= SENSOR_INTERVAL) {
    lastSensorMillis = millis();
    readSensors();
  }
}
