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
static constexpr unsigned long CLOUD_TEST_TIMEOUT_MS = 5000;
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
String recordingStartDay;
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

void closeCloudClients(HTTPClient &http, CloudRequestClients &clients) {
  http.end();
  clients.http.stop();
  clients.https.stop();
}

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

bool cloudConnected() {
  return cloudConfig.enabled && cloudConfig.baseUrl.length() > 0 && cloudHealthOk();
}

bool cloudArchiveActive() {
  return cloudConnected() && cloudConfig.recording;
}

bool cloudLoggingActive() {
  return false;
}

void refreshStorageMode(const String &reason = "") {
  bool runtimeActive = cloudConfig.enabled && cloudConfig.baseUrl.length() > 0;
  if (reason.length() > 0) {
    cloudStatus.lastStateReason = reason;
    cloudStatus.lastStateChangeMs = currentEpochMs();
  }
  storageMode = StorageMode::LOCAL_ONLY;
  cloudStatus.enabled = cloudConfig.enabled;
  cloudStatus.runtimeEnabled = runtimeActive;
  cloudStatus.connected = runtimeActive && cloudHealthOk();
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
  recordingStartDay = prefsCloud.getString("record_start", "");
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

void saveRecordingStartDay(const String &dayKey) {
  recordingStartDay = dayKey;
  prefsCloud.begin("cloud", false);
  prefsCloud.putString("record_start", recordingStartDay);
  prefsCloud.end();
}

void clearRecordingStartDay() {
  saveRecordingStartDay("");
}

void persistDailyHistory() {
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
  uint16_t days = std::min<uint16_t>(retentionDays(), 1);
  if (DAILY_HISTORY[idx].size() > days) {
    DAILY_HISTORY[idx].erase(DAILY_HISTORY[idx].begin(), DAILY_HISTORY[idx].begin() + (DAILY_HISTORY[idx].size() - days));
  }
}

void finalizeDaily(const String &dayKey) {
  if (dayKey.length() == 0) return;
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

String cloudReportsFolder() {
  return cloudDeviceRoot() + "/reports";
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
  String reportsPath = cloudReportsFolder();
  bool ok = true;
  if (!ensureCollection(baseRoot, "root")) ok = false;
  if (!ensureCollection(deviceRoot, "device")) ok = false;
  if (!ensureCollection(dailyPath, "daily")) ok = false;
  if (!ensureCollection(metaPath, "meta")) ok = false;
  if (!ensureCollection(reportsPath, "reports")) ok = false;
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

String formatMetricValue(float value, int decimals) {
  if (isnan(value)) return "–";
  return String((double)value, decimals);
}

bool dailyAggregateFor(const String &dayKey, const char *metricId, DailyAggregate &out) {
  int idx = metricIndex(metricId);
  if (idx < 0) return false;
  if (DAILY_AGG[idx].dayKey != dayKey || DAILY_AGG[idx].count == 0) return false;
  out = DAILY_AGG[idx];
  return true;
}

String dailySummaryAssessment(float tempMin, float tempMax, float vpdAvg, float vpdMin, float vpdMax) {
  if (!isnan(tempMin) && !isnan(tempMax) && (tempMax - tempMin) > 6.0f) {
    return "Auffällige Temperaturschwankungen";
  }
  if (!isnan(vpdAvg) && vpdAvg > 1.6f) {
    return "VPD meist zu hoch → Gefahr für Stress";
  }
  if (!isnan(vpdAvg) && vpdAvg < 0.6f) {
    return "VPD eher niedrig → erhöhtes Schimmelrisiko";
  }
  if (!isnan(vpdMin) && !isnan(vpdMax) && (vpdMax - vpdMin) > 0.8f) {
    return "VPD schwankend – Klima prüfen";
  }
  return "Klima heute stabil und unauffällig";
}

String buildDailySummaryText(const String &dayKey) {
  if (dayKey.length() == 0 || dayKey == "unsynced") return "";
  DailyAggregate tempAgg;
  DailyAggregate humAgg;
  DailyAggregate vpdAgg;
  bool hasTemp = dailyAggregateFor(dayKey, "temp", tempAgg);
  bool hasHum = dailyAggregateFor(dayKey, "humidity", humAgg);
  bool hasVpd = dailyAggregateFor(dayKey, "vpd", vpdAgg);
  if (!hasTemp && !hasHum && !hasVpd) return "";
  float tempAvg = hasTemp && tempAgg.count > 0 ? tempAgg.sum / tempAgg.count : NAN;
  float humAvg = hasHum && humAgg.count > 0 ? humAgg.sum / humAgg.count : NAN;
  float vpdAvg = hasVpd && vpdAgg.count > 0 ? vpdAgg.sum / vpdAgg.count : NAN;
  String text;
  text.reserve(256);
  text += "Datum: " + dayKey + "\n";
  if (hasTemp) {
    text += "Durchschnittstemperatur: " + formatMetricValue(tempAvg, 1) + " °C\n";
    text += "Temp Min / Max: " + formatMetricValue(tempAgg.min, 1) + " / " + formatMetricValue(tempAgg.max, 1) + " °C\n";
  }
  if (hasHum) {
    text += "Durchschnitt Luftfeuchte: " + formatMetricValue(humAvg, 1) + " %\n";
  }
  if (hasVpd) {
    text += "Durchschnitt VPD: " + formatMetricValue(vpdAvg, 2) + " kPa\n";
    text += "VPD Min / Max: " + formatMetricValue(vpdAgg.min, 2) + " / " + formatMetricValue(vpdAgg.max, 2) + " kPa\n";
  }
  String assessment = dailySummaryAssessment(tempAgg.min, tempAgg.max, vpdAvg, vpdAgg.min, vpdAgg.max);
  text += "Bewertung: " + assessment + "\n";
  return text;
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
  (void)epochMs;
  return;
}

void enqueueRecordingEvent(const String &event, const String &reason) {
  (void)event;
  (void)reason;
  return;
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

void enqueueDailySummary(const String &dayKey) {
  if (!cloudArchiveActive()) return;
  String payload = buildDailySummaryText(dayKey);
  if (payload.length() == 0) return;
  String path = cloudRootPath() + "/daily/GrowSummary_" + dayKey + ".txt";
  enqueueCloudJob(path, payload, dayKey, "daily_summary", "text/plain", true);
}

void finalizeDayAndQueue(const String &dayKey) {
  if (dayKey.length() == 0) return;
  finalizeDaily(dayKey);
  persistDailyHistory();
  if (cloudArchiveActive()) {
    String payload = serializeDailyPayload(dayKey);
    enqueueDailyJob(dayKey, payload);
    enqueueDailySummary(dayKey);
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
  if (detail.length() > 0) msg += String(" ") + detail;
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
    closeCloudClients(http, clients);
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
    closeCloudClients(http, clients);
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
  closeCloudClients(http, clients);
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

bool fetchCloudDailyMetrics(const String &dayKey, DailyPoint &temp, DailyPoint &humidity, DailyPoint &vpd) {
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
  auto readMetric = [&](const char *id, DailyPoint &out) {
    if (!sensors.containsKey(id)) return false;
    JsonObject m = sensors[id];
    out.dayKey = doc["date"] | dayKey;
    out.avg = m["avg"] | NAN;
    out.min = m["min"] | NAN;
    out.max = m["max"] | NAN;
    out.last = m["last"] | NAN;
    out.count = m["samples"] | m["count"] | 0;
    return out.count > 0;
  };
  bool any = false;
  if (readMetric("temp", temp)) any = true;
  if (readMetric("humidity", humidity)) any = true;
  if (readMetric("vpd", vpd)) any = true;
  return any;
}

struct ReportMetricStats {
  uint32_t samples = 0;
  double sum = 0.0;
  float min = NAN;
  float max = NAN;
};

void updateReportStats(ReportMetricStats &stats, const DailyPoint &p) {
  if (p.count == 0) return;
  stats.samples += p.count;
  stats.sum += static_cast<double>(p.avg) * static_cast<double>(p.count);
  if (isnan(stats.min) || p.min < stats.min) stats.min = p.min;
  if (isnan(stats.max) || p.max > stats.max) stats.max = p.max;
}

String reportAssessment(float tempMin, float tempMax, float vpdAvg, float vpdMin, float vpdMax) {
  if (!isnan(tempMin) && !isnan(tempMax) && (tempMax - tempMin) <= 4.0f && !isnan(vpdMin) && !isnan(vpdMax) && (vpdMax - vpdMin) <= 0.4f) {
    return "Grow klimatisch sehr stabil – ideal";
  }
  if (!isnan(tempMin) && !isnan(tempMax) && (tempMax - tempMin) > 8.0f) {
    return "Viele Peaks – ggf. Lüftung oder Heizung prüfen";
  }
  if (!isnan(vpdAvg) && vpdAvg > 1.6f) {
    return "VPD meist zu hoch → Gefahr für Stress";
  }
  if (!isnan(vpdAvg) && vpdAvg < 0.6f) {
    return "VPD meist zu niedrig – ggf. zu feucht";
  }
  if (!isnan(vpdMin) && !isnan(vpdMax) && (vpdMax - vpdMin) > 1.0f) {
    return "Viele Peaks – ggf. Lüftung oder Heizung prüfen";
  }
  return "Klima insgesamt solide mit kleinen Schwankungen";
}

std::vector<String> dayKeysBetween(const String &from, const String &to);

bool buildGrowReport(String &pathOut, String &payloadOut, String &errorOut) {
  if (!cloudConnected()) {
    errorOut = "cloud inactive";
    return false;
  }
  if (!isTimeSynced()) {
    errorOut = "time not synced";
    return false;
  }
  String startDay = recordingStartDay;
  if (startDay.length() == 0) {
    errorOut = "recording start unknown";
    return false;
  }
  String endDay = currentDayKey();
  if (endDay.length() == 0) {
    errorOut = "invalid date";
    return false;
  }
  auto keys = dayKeysBetween(startDay, endDay);
  if (keys.empty()) {
    errorOut = "no days in range";
    return false;
  }

  ReportMetricStats tempStats;
  ReportMetricStats humStats;
  ReportMetricStats vpdStats;
  uint32_t logDays = 0;
  for (const auto &dayKey : keys) {
    DailyPoint temp;
    DailyPoint hum;
    DailyPoint vpd;
    if (!fetchCloudDailyMetrics(dayKey, temp, hum, vpd)) continue;
    bool dayHasData = false;
    if (temp.count > 0) { updateReportStats(tempStats, temp); dayHasData = true; }
    if (hum.count > 0) { updateReportStats(humStats, hum); dayHasData = true; }
    if (vpd.count > 0) { updateReportStats(vpdStats, vpd); dayHasData = true; }
    if (dayHasData) logDays++;
  }

  if (logDays == 0 || (tempStats.samples == 0 && humStats.samples == 0 && vpdStats.samples == 0)) {
    errorOut = "no data";
    return false;
  }

  float tempAvg = tempStats.samples > 0 ? static_cast<float>(tempStats.sum / tempStats.samples) : NAN;
  float humAvg = humStats.samples > 0 ? static_cast<float>(humStats.sum / humStats.samples) : NAN;
  float vpdAvg = vpdStats.samples > 0 ? static_cast<float>(vpdStats.sum / vpdStats.samples) : NAN;
  String assessment = reportAssessment(tempStats.min, tempStats.max, vpdAvg, vpdStats.min, vpdStats.max);
  String finishStamp = isoTimestamp(currentEpochMs());

  String text;
  text.reserve(512);
  text += "Zeitraum: " + startDay + " bis " + endDay + "\n";
  text += "Anzahl Log-Tage: " + String(logDays) + "\n";
  if (tempStats.samples > 0) {
    text += "Temp avg: " + formatMetricValue(tempAvg, 1) + " °C\n";
    text += "Temp min / max gesamt: " + formatMetricValue(tempStats.min, 1) + " / " + formatMetricValue(tempStats.max, 1) + " °C\n";
  }
  if (humStats.samples > 0) {
    text += "Luftfeuchte avg: " + formatMetricValue(humAvg, 1) + " %\n";
    text += "Luftfeuchte min / max: " + formatMetricValue(humStats.min, 1) + " / " + formatMetricValue(humStats.max, 1) + " %\n";
  }
  if (vpdStats.samples > 0) {
    text += "VPD avg: " + formatMetricValue(vpdAvg, 2) + " kPa\n";
    text += "VPD min / max: " + formatMetricValue(vpdStats.min, 2) + " / " + formatMetricValue(vpdStats.max, 2) + " kPa\n";
  }
  text += "Statistik: " + assessment + "\n";
  text += "Abschlusszeitpunkt: " + finishStamp + "\n";
  text += "Firmware Version am Ende: " + String(FIRMWARE_VERSION) + "\n";

  pathOut = cloudReportsFolder() + "/GrowReport_" + startDay + "_bis_" + endDay + ".txt";
  payloadOut = text;
  return true;
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
  if (!cloudStatus.runtimeEnabled || !cloudConfig.enabled || !cloudStatus.connected || cloudQueue.empty()) {
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
    unsigned long backoff = job.kind == "daily" ? (24UL * 60UL * 60UL * 1000UL)
        : (job.attempts <= 1 ? CLOUD_BACKOFF_SHORT_MS : (job.attempts <= 3 ? CLOUD_BACKOFF_MED_MS : CLOUD_BACKOFF_LONG_MS));
    job.nextAttemptAt = now + backoff;
    nextCloudAttemptAfterMs = job.nextAttemptAt;
    if (job.kind != "daily" && job.attempts > 5) {
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
  return;
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

  (void)lastDailyCheckpointMs;

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
  31, 139, 8, 0, 187, 35, 71, 105, 2, 255, 236, 189, 219, 114, 27, 73, 146, 40, 248, 222, 95, 145, 197, 170, 110, 0, 85, 0, 8, 128, 23, 145, 160, 72, 29, 137, 162, 90, 156, 166, 46, 43, 178, 164, 174, 214, 200, 164, 36, 16, 32, 178, 153, 64, 98, 50, 19, 188, 136, 135, 102, 99, 231, 97, 95, 215, 214, 102, 109, 119, 223, 214, 236, 88, 127, 192, 177, 53, 219, 243, 212, 79, 167, 254, 164, 191, 100, 221, 61, 46, 25, 183, 76, 128, 148, 170, 187, 218, 118, 167, 167, 68, 100, 92, 60, 60, 60, 60, 60, 60, 60, 60, 60, 126, 19, 4, 15, 191, 25, 38, 131, 252, 122, 198, 130, 113, 62, 137, 247, 126, 3, 41, 248, 35, 136, 195, 233, 217, 238, 202, 144, 173, 240, 36, 22, 14, 241, 7, 252, 156, 176, 60, 12, 6, 227, 48, 205, 88, 190, 187, 242, 227, 201, 179, 214, 214, 74, 176, 170, 103, 78, 195, 9, 219, 93, 185, 136, 216, 229, 44, 73, 243, 149, 96, 144, 76, 115, 54, 133, 194, 151, 209, 48, 31, 239, 14, 217, 69, 52, 96, 45, 250, 104, 6, 209, 52, 202, 163, 48, 110, 101, 131, 48, 102, 187, 221, 2, 84, 30, 229, 49, 219, 251, 125, 154, 92, 30, 179, 105, 150, 164, 193, 69, 167, 189, 214, 94, 123, 184, 202, 51, 120, 161, 44, 191, 150, 191, 131, 160, 159, 38, 73, 30, 220, 64, 123, 113, 146, 2, 192, 49, 155, 176, 126, 16, 71, 103, 227, 60, 24, 134, 233, 249, 78, 112, 43, 74, 98, 15, 155, 193, 105, 50, 188, 134, 226, 167, 225, 224, 252, 44, 77, 230, 211, 97, 63, 248, 182, 51, 234, 62, 232, 133, 59, 28, 6, 124, 179, 30, 219, 26, 117, 118, 130, 73, 52, 109, 141, 25, 130, 234, 7, 221, 78, 231, 183, 5, 44, 1, 101, 4, 157, 108, 141, 194, 73, 20, 95, 247, 131, 236, 58, 203, 217, 164, 53, 143, 154, 65, 22, 78, 179, 86, 198, 210, 104, 4, 64, 194, 244, 44, 154, 246, 3, 128, 55, 11, 135, 195, 104, 122, 70, 191, 21, 86, 64, 100, 150, 2, 44, 149, 217, 221, 156, 93, 237, 152, 8, 118, 187, 221, 173, 222, 3, 72, 76, 174, 90, 217, 56, 28, 38, 151, 0, 35, 232, 205, 174, 2, 40, 27, 164, 103, 167, 97, 189, 211, 164, 255, 181, 123, 27, 13, 104, 40, 201, 128, 192, 9, 180, 154, 229, 209, 224, 252, 122, 39, 200, 147, 25, 53, 251, 185, 21, 77, 135, 236, 10, 90, 233, 105, 56, 116, 161, 125, 13, 79, 234, 86, 22, 125, 6, 66, 118, 219, 189, 148, 77, 52, 136, 41, 139, 195, 60, 186, 96, 69, 237, 54, 239, 66, 11, 6, 13, 192, 12, 163, 108, 22, 135, 215, 253, 81, 204, 160, 23, 127, 158, 3, 2, 163, 235, 150, 224, 6, 192, 103, 22, 2, 27, 156, 178, 252, 146, 177, 233, 78, 16, 194, 64, 77, 91, 17, 208, 45, 235, 7, 3, 40, 193, 210, 157, 224, 44, 156, 245, 187, 61, 36, 2, 2, 105, 93, 166, 240, 29, 224, 191, 90, 155, 0, 112, 234, 180, 166, 67, 211, 129, 109, 33, 44, 85, 53, 102, 67, 168, 73, 156, 40, 154, 17, 99, 204, 63, 78, 147, 148, 122, 19, 14, 163, 121, 214, 223, 192, 81, 215, 198, 226, 91, 54, 90, 135, 255, 51, 134, 162, 19, 224, 255, 214, 228, 72, 244, 214, 182, 155, 155, 91, 248, 255, 98, 52, 36, 146, 209, 52, 142, 166, 208, 249, 56, 25, 104, 108, 217, 62, 13, 135, 103, 76, 235, 139, 89, 76, 178, 197, 58, 128, 223, 114, 241, 219, 222, 222, 38, 66, 169, 17, 235, 180, 183, 104, 192, 116, 156, 187, 163, 222, 246, 218, 3, 201, 223, 223, 14, 78, 135, 27, 172, 107, 99, 208, 130, 41, 106, 78, 140, 111, 187, 157, 211, 237, 173, 174, 170, 39, 167, 137, 85, 239, 50, 76, 167, 86, 197, 209, 198, 54, 235, 156, 150, 87, 60, 27, 39, 89, 110, 214, 201, 83, 152, 51, 179, 48, 133, 97, 147, 125, 236, 119, 161, 207, 89, 18, 71, 195, 192, 238, 130, 156, 161, 18, 226, 36, 140, 166, 238, 28, 146, 20, 13, 206, 210, 104, 200, 89, 33, 224, 131, 92, 240, 81, 152, 14, 109, 113, 80, 204, 54, 194, 34, 240, 160, 97, 14, 130, 0, 106, 207, 96, 99, 178, 138, 241, 171, 156, 172, 197, 220, 74, 46, 88, 58, 138, 161, 230, 56, 26, 14, 113, 162, 40, 132, 51, 146, 139, 45, 129, 183, 103, 90, 58, 85, 53, 41, 214, 221, 234, 232, 116, 225, 179, 134, 38, 217, 48, 74, 217, 128, 32, 1, 133, 231, 19, 189, 69, 36, 158, 198, 158, 46, 49, 49, 161, 5, 147, 14, 178, 115, 214, 226, 245, 51, 236, 205, 140, 133, 121, 61, 156, 231, 73, 107, 20, 229, 77, 68, 100, 18, 94, 213, 123, 29, 64, 162, 25, 116, 71, 105, 163, 161, 53, 115, 17, 198, 115, 38, 101, 170, 20, 62, 155, 196, 203, 148, 116, 17, 166, 81, 8, 127, 167, 243, 9, 8, 214, 65, 63, 200, 195, 211, 121, 28, 166, 152, 144, 21, 112, 226, 240, 148, 197, 58, 186, 98, 30, 113, 241, 214, 34, 57, 184, 101, 78, 153, 160, 211, 222, 166, 118, 36, 140, 104, 58, 155, 3, 190, 25, 139, 129, 38, 176, 100, 204, 243, 60, 153, 42, 153, 33, 150, 130, 98, 180, 137, 166, 58, 252, 117, 119, 158, 234, 115, 215, 207, 81, 198, 130, 116, 218, 237, 245, 58, 238, 130, 164, 150, 31, 137, 209, 96, 158, 102, 88, 98, 150, 68, 92, 220, 201, 22, 166, 201, 148, 153, 64, 81, 168, 0, 181, 206, 16, 33, 152, 100, 245, 110, 175, 51, 100, 103, 205, 224, 219, 94, 111, 184, 198, 24, 252, 216, 92, 219, 220, 28, 117, 27, 69, 179, 18, 13, 34, 213, 165, 88, 8, 55, 59, 14, 34, 125, 32, 118, 120, 202, 229, 106, 2, 2, 62, 202, 175, 145, 168, 155, 59, 10, 191, 105, 146, 183, 194, 24, 184, 146, 13, 181, 33, 55, 87, 140, 128, 179, 35, 49, 22, 81, 203, 187, 56, 148, 47, 9, 8, 109, 47, 248, 30, 89, 40, 166, 21, 78, 159, 53, 121, 152, 207, 51, 201, 93, 222, 174, 136, 50, 237, 228, 92, 106, 19, 64, 129, 181, 245, 225, 218, 246, 182, 91, 136, 165, 169, 86, 106, 180, 245, 160, 251, 64, 111, 46, 78, 206, 96, 246, 219, 28, 99, 168, 19, 124, 38, 26, 10, 196, 60, 106, 77, 146, 105, 66, 75, 100, 51, 56, 126, 246, 2, 62, 90, 111, 216, 25, 114, 121, 51, 120, 193, 166, 113, 2, 127, 146, 105, 56, 128, 191, 251, 9, 200, 129, 56, 204, 154, 193, 202, 81, 116, 202, 210, 16, 39, 47, 230, 38, 43, 144, 180, 159, 204, 211, 8, 212, 138, 151, 236, 18, 62, 21, 212, 157, 224, 114, 12, 244, 108, 209, 7, 112, 77, 202, 90, 246, 186, 10, 58, 94, 174, 11, 22, 77, 40, 73, 220, 105, 2, 23, 117, 6, 225, 244, 34, 204, 236, 206, 150, 20, 30, 129, 182, 70, 234, 78, 206, 174, 144, 37, 96, 132, 139, 177, 45, 166, 84, 207, 153, 161, 91, 27, 52, 69, 37, 201, 183, 215, 195, 181, 211, 45, 75, 144, 183, 148, 50, 181, 88, 47, 88, 164, 153, 184, 122, 3, 31, 250, 214, 144, 212, 205, 251, 170, 15, 155, 167, 15, 122, 91, 157, 42, 245, 161, 219, 121, 208, 236, 118, 215, 155, 221, 158, 82, 32, 60, 40, 180, 225, 191, 214, 108, 30, 103, 12, 216, 135, 214, 210, 112, 26, 77, 66, 62, 100, 144, 247, 26, 179, 130, 94, 123, 61, 11, 88, 8, 133, 128, 245, 146, 121, 14, 146, 109, 132, 186, 55, 91, 4, 51, 139, 105, 106, 250, 96, 174, 183, 183, 238, 7, 51, 26, 198, 172, 4, 230, 131, 69, 16, 255, 211, 57, 187, 30, 165, 176, 197, 200, 138, 74, 55, 65, 231, 183, 184, 116, 219, 148, 236, 112, 58, 110, 244, 154, 189, 110, 183, 217, 221, 88, 3, 50, 118, 137, 140, 193, 3, 127, 13, 28, 57, 183, 18, 213, 64, 102, 94, 178, 17, 42, 175, 72, 144, 71, 49, 107, 209, 174, 229, 158, 106, 42, 1, 248, 2, 134, 46, 227, 103, 177, 80, 157, 38, 32, 185, 39, 253, 77, 163, 77, 216, 201, 193, 210, 218, 194, 166, 125, 139, 139, 71, 219, 32, 181, 77, 72, 10, 88, 220, 133, 124, 11, 96, 217, 152, 240, 49, 109, 242, 34, 163, 36, 157, 160, 208, 83, 169, 98, 157, 48, 210, 196, 220, 161, 57, 14, 131, 34, 51, 124, 58, 141, 106, 170, 255, 160, 215, 209, 148, 47, 217, 175, 245, 173, 178, 142, 245, 199, 8, 13, 37, 144, 196, 171, 207, 127, 162, 246, 242, 83, 189, 5, 188, 208, 216, 49, 80, 193, 117, 96, 109, 189, 187, 177, 97, 143, 142, 32, 181, 95, 15, 147, 59, 173, 222, 146, 250, 150, 26, 32, 65, 238, 22, 187, 0, 216, 89, 31, 245, 167, 66, 198, 24, 187, 80, 142, 133, 216, 139, 86, 15, 142, 62, 54, 235, 238, 40, 128, 148, 246, 140, 87, 145, 234, 163, 184, 92, 233, 187, 59, 26, 41, 53, 74, 226, 132, 168, 84, 70, 55, 54, 151, 86, 70, 21, 113, 52, 170, 154, 154, 99, 107, 22, 78, 73, 241, 171, 212, 121, 182, 184, 206, 195, 5, 237, 70, 179, 183, 214, 92, 239, 129, 120, 216, 110, 184, 105, 15, 80, 59, 69, 96, 195, 52, 153, 129, 2, 27, 231, 168, 87, 157, 198, 243, 180, 222, 229, 60, 2, 122, 196, 233, 121, 148, 183, 42, 11, 57, 91, 25, 222, 206, 250, 86, 179, 187, 185, 214, 236, 110, 173, 163, 144, 239, 53, 236, 237, 66, 23, 247, 11, 107, 29, 123, 195, 176, 222, 107, 44, 220, 124, 96, 37, 158, 102, 237, 40, 23, 82, 120, 157, 203, 135, 171, 150, 88, 221, 184, 158, 91, 201, 218, 38, 43, 102, 243, 83, 92, 215, 13, 45, 94, 42, 215, 214, 230, 147, 48, 146, 156, 208, 94, 243, 207, 85, 216, 230, 199, 113, 56, 203, 72, 179, 212, 56, 112, 155, 56, 71, 227, 165, 205, 117, 143, 12, 224, 219, 48, 201, 166, 128, 199, 86, 5, 167, 46, 104, 222, 152, 106, 26, 38, 29, 173, 129, 18, 232, 173, 77, 98, 4, 57, 24, 92, 51, 95, 212, 28, 9, 169, 150, 212, 198, 138, 62, 244, 122, 75, 162, 170, 86, 14, 83, 222, 119, 22, 85, 247, 136, 71, 11, 99, 19, 53, 197, 30, 225, 41, 240, 247, 28, 23, 237, 104, 154, 49, 162, 140, 228, 73, 161, 25, 21, 19, 19, 89, 55, 140, 139, 121, 57, 136, 210, 1, 44, 57, 33, 8, 39, 88, 110, 225, 63, 49, 29, 215, 214, 105, 121, 237, 173, 161, 42, 212, 217, 240, 204, 210, 141, 141, 134, 111, 238, 246, 26, 85, 70, 4, 115, 2, 153, 115, 197, 93, 96, 179, 60, 101, 249, 96, 236, 89, 97, 101, 134, 87, 98, 203, 73, 210, 213, 89, 112, 13, 134, 239, 34, 202, 162, 211, 40, 198, 4, 250, 25, 155, 146, 218, 93, 24, 43, 215, 177, 82, 78, 217, 180, 39, 39, 187, 102, 254, 241, 138, 217, 168, 208, 94, 249, 204, 161, 15, 46, 5, 122, 155, 154, 142, 219, 219, 44, 211, 113, 203, 137, 173, 105, 191, 230, 190, 86, 109, 107, 189, 114, 106, 25, 29, 71, 215, 158, 12, 41, 104, 107, 216, 94, 235, 75, 119, 171, 97, 80, 222, 84, 63, 180, 85, 178, 232, 130, 145, 172, 41, 54, 122, 97, 213, 110, 208, 221, 44, 134, 80, 178, 195, 154, 103, 88, 212, 156, 51, 180, 14, 165, 116, 24, 38, 57, 97, 155, 242, 75, 154, 174, 88, 114, 180, 158, 163, 129, 152, 22, 19, 163, 235, 107, 13, 31, 26, 225, 0, 229, 187, 49, 247, 203, 164, 164, 226, 168, 236, 226, 172, 216, 14, 109, 233, 219, 161, 45, 125, 90, 217, 70, 79, 85, 191, 13, 255, 180, 6, 113, 194, 101, 252, 221, 132, 164, 9, 36, 153, 177, 233, 23, 130, 112, 240, 176, 209, 214, 39, 155, 181, 239, 53, 182, 189, 206, 134, 159, 39, 88, 96, 201, 78, 209, 13, 186, 1, 151, 24, 94, 49, 162, 154, 30, 178, 139, 214, 20, 54, 207, 202, 238, 160, 172, 172, 238, 90, 171, 25, 163, 12, 5, 255, 91, 0, 242, 34, 25, 134, 177, 46, 8, 70, 209, 21, 90, 101, 148, 212, 54, 9, 120, 151, 105, 168, 75, 120, 157, 219, 54, 27, 5, 255, 111, 118, 28, 114, 142, 163, 169, 173, 50, 24, 187, 125, 181, 217, 47, 235, 214, 52, 188, 112, 182, 72, 74, 99, 44, 12, 70, 220, 212, 161, 1, 233, 118, 108, 40, 150, 169, 79, 140, 139, 16, 42, 91, 74, 177, 90, 198, 236, 234, 130, 237, 247, 79, 25, 76, 42, 144, 17, 122, 90, 56, 226, 6, 17, 73, 204, 149, 149, 157, 242, 69, 181, 181, 102, 25, 20, 252, 43, 105, 177, 124, 118, 183, 31, 52, 183, 81, 199, 92, 111, 4, 29, 79, 114, 167, 17, 108, 118, 126, 219, 40, 209, 98, 232, 124, 174, 206, 7, 176, 114, 137, 50, 118, 13, 218, 198, 205, 98, 105, 206, 82, 147, 232, 170, 5, 171, 222, 116, 216, 154, 36, 67, 6, 141, 164, 172, 140, 96, 146, 56, 162, 247, 180, 212, 232, 6, 132, 52, 26, 229, 193, 90, 38, 20, 125, 143, 225, 64, 3, 70, 204, 230, 29, 3, 145, 35, 27, 243, 110, 108, 56, 41, 186, 13, 191, 77, 130, 240, 0, 22, 78, 147, 73, 201, 206, 114, 109, 88, 111, 1, 3, 53, 81, 27, 108, 2, 213, 53, 120, 193, 6, 217, 24, 74, 106, 97, 165, 45, 163, 14, 106, 66, 59, 183, 65, 158, 84, 52, 133, 173, 216, 45, 105, 214, 9, 60, 167, 117, 100, 101, 197, 206, 192, 58, 53, 193, 234, 109, 181, 90, 152, 19, 79, 147, 184, 211, 121, 235, 52, 183, 207, 134, 74, 78, 114, 202, 21, 8, 5, 240, 50, 26, 69, 45, 101, 201, 189, 131, 77, 197, 84, 129, 77, 203, 192, 76, 216, 146, 10, 174, 226, 41, 221, 197, 102, 46, 141, 1, 102, 119, 182, 72, 173, 87, 91, 164, 58, 95, 223, 34, 197, 5, 83, 249, 10, 121, 49, 27, 186, 90, 189, 99, 253, 21, 134, 107, 159, 10, 237, 136, 64, 159, 234, 183, 196, 56, 19, 34, 114, 113, 173, 218, 95, 24, 53, 176, 117, 232, 213, 162, 45, 137, 79, 32, 45, 101, 130, 88, 96, 95, 211, 22, 137, 157, 242, 221, 175, 125, 144, 67, 152, 199, 236, 12, 132, 97, 249, 10, 230, 227, 109, 123, 85, 211, 78, 183, 17, 88, 43, 187, 12, 97, 107, 82, 40, 39, 186, 18, 223, 245, 140, 224, 186, 199, 94, 96, 43, 63, 136, 43, 76, 165, 115, 18, 146, 119, 50, 128, 243, 49, 239, 21, 99, 46, 79, 132, 29, 46, 86, 154, 42, 90, 192, 55, 113, 71, 135, 147, 101, 109, 163, 97, 114, 211, 232, 244, 116, 212, 91, 183, 112, 155, 38, 173, 97, 152, 135, 11, 56, 224, 238, 150, 84, 153, 110, 105, 35, 182, 217, 135, 91, 125, 184, 6, 110, 41, 66, 218, 214, 116, 211, 52, 232, 107, 167, 185, 226, 56, 76, 199, 136, 70, 25, 36, 94, 154, 187, 117, 134, 44, 27, 20, 186, 160, 68, 202, 229, 60, 187, 158, 117, 156, 43, 78, 115, 189, 71, 184, 246, 9, 174, 60, 192, 69, 251, 95, 83, 156, 223, 122, 86, 7, 209, 144, 176, 34, 223, 109, 27, 94, 106, 50, 80, 2, 100, 105, 123, 33, 177, 188, 176, 97, 43, 19, 182, 110, 175, 214, 246, 111, 189, 37, 140, 208, 190, 254, 221, 195, 152, 236, 218, 146, 5, 188, 251, 172, 108, 198, 112, 111, 248, 236, 108, 37, 173, 120, 79, 179, 58, 142, 144, 176, 107, 135, 68, 230, 108, 121, 109, 187, 216, 240, 133, 103, 11, 125, 92, 214, 126, 57, 31, 151, 111, 105, 101, 91, 184, 247, 169, 218, 193, 220, 127, 95, 36, 247, 62, 91, 29, 63, 66, 237, 9, 254, 105, 185, 190, 40, 75, 172, 154, 214, 36, 90, 55, 38, 209, 102, 97, 198, 217, 166, 13, 169, 178, 237, 110, 244, 58, 150, 143, 138, 80, 58, 122, 142, 153, 100, 93, 247, 97, 90, 56, 235, 76, 89, 240, 45, 231, 155, 119, 209, 103, 203, 95, 229, 151, 219, 116, 154, 198, 193, 18, 49, 205, 231, 165, 62, 46, 190, 237, 205, 186, 118, 2, 85, 108, 143, 188, 125, 43, 211, 134, 181, 189, 132, 159, 38, 127, 151, 177, 239, 25, 99, 191, 185, 238, 25, 123, 180, 19, 173, 245, 236, 177, 223, 104, 248, 182, 131, 219, 235, 158, 13, 35, 167, 157, 38, 83, 55, 61, 135, 125, 155, 174, 56, 245, 146, 209, 164, 138, 111, 23, 230, 33, 172, 168, 148, 195, 162, 95, 46, 160, 124, 133, 213, 174, 159, 27, 100, 52, 34, 118, 236, 85, 200, 36, 253, 29, 60, 212, 220, 246, 10, 150, 209, 26, 40, 241, 210, 145, 78, 58, 182, 143, 142, 197, 36, 98, 137, 49, 124, 233, 44, 4, 200, 196, 229, 87, 145, 10, 147, 72, 170, 235, 115, 37, 62, 122, 254, 253, 155, 216, 73, 42, 81, 45, 253, 71, 237, 117, 88, 219, 212, 225, 184, 195, 178, 196, 102, 119, 218, 144, 154, 203, 147, 6, 101, 225, 190, 116, 22, 197, 177, 230, 43, 88, 237, 88, 185, 196, 18, 227, 177, 91, 121, 13, 218, 74, 39, 241, 73, 55, 211, 1, 167, 5, 154, 88, 126, 253, 247, 84, 100, 203, 180, 85, 93, 89, 13, 175, 162, 172, 37, 253, 235, 74, 183, 56, 142, 103, 14, 109, 219, 79, 195, 212, 63, 41, 215, 109, 138, 208, 160, 195, 22, 166, 208, 68, 54, 173, 161, 230, 224, 148, 218, 178, 110, 143, 147, 178, 155, 27, 3, 218, 243, 66, 241, 78, 66, 152, 109, 131, 141, 13, 93, 229, 3, 22, 76, 166, 83, 224, 67, 117, 204, 236, 152, 17, 45, 153, 81, 200, 135, 97, 152, 141, 217, 48, 40, 65, 203, 17, 49, 222, 69, 172, 96, 143, 56, 153, 15, 191, 134, 14, 191, 110, 235, 240, 230, 132, 226, 237, 56, 42, 233, 61, 90, 2, 153, 111, 182, 180, 101, 185, 75, 110, 121, 26, 22, 115, 244, 139, 150, 67, 99, 84, 182, 220, 51, 190, 5, 27, 250, 234, 89, 74, 88, 14, 82, 54, 132, 156, 40, 140, 51, 31, 71, 220, 115, 231, 211, 41, 219, 249, 148, 183, 254, 75, 248, 9, 85, 180, 150, 39, 103, 103, 180, 183, 243, 89, 201, 149, 60, 173, 216, 138, 186, 32, 179, 249, 4, 232, 119, 237, 44, 1, 95, 193, 211, 73, 29, 23, 44, 244, 73, 168, 192, 79, 156, 252, 87, 57, 34, 117, 133, 171, 163, 109, 62, 118, 29, 112, 58, 203, 105, 70, 46, 26, 250, 201, 85, 41, 142, 165, 222, 9, 94, 219, 215, 125, 27, 115, 7, 204, 90, 102, 89, 154, 194, 134, 241, 52, 4, 177, 153, 26, 14, 165, 165, 87, 82, 182, 59, 150, 27, 244, 131, 81, 119, 216, 29, 22, 78, 159, 35, 54, 24, 14, 215, 52, 95, 151, 226, 68, 70, 204, 40, 97, 226, 213, 29, 172, 79, 183, 187, 131, 238, 64, 187, 12, 224, 50, 150, 113, 243, 196, 231, 124, 106, 177, 175, 209, 181, 121, 108, 94, 158, 145, 78, 40, 116, 170, 30, 116, 205, 185, 100, 212, 84, 106, 167, 222, 105, 223, 61, 8, 189, 59, 163, 65, 184, 17, 110, 184, 52, 177, 156, 130, 54, 13, 159, 32, 53, 39, 29, 151, 62, 237, 64, 118, 194, 196, 21, 30, 71, 144, 185, 171, 54, 79, 191, 147, 97, 50, 255, 220, 226, 14, 245, 150, 220, 192, 67, 82, 105, 164, 236, 88, 30, 144, 128, 210, 178, 254, 68, 133, 165, 34, 109, 37, 211, 184, 68, 121, 18, 13, 233, 118, 14, 203, 109, 64, 140, 100, 171, 235, 181, 104, 15, 226, 104, 214, 71, 125, 84, 238, 144, 26, 166, 75, 245, 52, 225, 189, 22, 3, 215, 49, 221, 211, 253, 38, 248, 224, 155, 104, 130, 119, 230, 66, 67, 91, 231, 122, 96, 133, 105, 152, 15, 129, 77, 110, 239, 144, 44, 77, 59, 97, 60, 198, 234, 174, 177, 166, 218, 28, 101, 48, 220, 242, 58, 245, 29, 142, 5, 196, 13, 191, 5, 60, 180, 209, 177, 46, 123, 81, 143, 172, 186, 90, 121, 195, 123, 20, 123, 17, 152, 51, 246, 124, 22, 17, 125, 23, 94, 109, 147, 196, 208, 189, 1, 12, 40, 92, 103, 93, 70, 147, 42, 189, 58, 179, 80, 151, 226, 178, 203, 235, 189, 164, 102, 131, 46, 127, 53, 241, 219, 237, 250, 61, 77, 244, 49, 237, 232, 102, 4, 225, 6, 90, 218, 95, 193, 69, 95, 79, 135, 243, 9, 163, 123, 40, 113, 197, 228, 89, 194, 81, 134, 252, 239, 53, 63, 142, 141, 77, 135, 59, 132, 132, 90, 218, 66, 182, 238, 64, 88, 184, 167, 114, 38, 42, 214, 114, 47, 76, 117, 219, 221, 242, 251, 82, 230, 117, 169, 251, 122, 96, 169, 169, 110, 146, 32, 229, 82, 170, 4, 21, 113, 223, 230, 65, 231, 203, 90, 162, 86, 90, 243, 89, 113, 234, 225, 236, 211, 120, 17, 24, 208, 169, 230, 38, 99, 223, 206, 225, 133, 70, 48, 229, 138, 66, 108, 131, 61, 96, 167, 78, 33, 113, 183, 209, 114, 184, 177, 10, 157, 37, 201, 112, 33, 78, 167, 161, 86, 70, 94, 34, 181, 164, 125, 158, 36, 113, 30, 205, 60, 118, 82, 175, 234, 86, 177, 105, 220, 238, 45, 235, 122, 188, 209, 112, 152, 193, 240, 119, 241, 157, 246, 150, 152, 142, 93, 119, 101, 58, 183, 115, 185, 218, 239, 48, 189, 185, 132, 83, 245, 166, 97, 183, 237, 246, 12, 129, 212, 91, 183, 246, 176, 58, 81, 23, 30, 127, 155, 67, 208, 246, 205, 201, 173, 234, 77, 131, 1, 0, 228, 110, 50, 61, 83, 67, 30, 77, 199, 48, 11, 115, 207, 132, 80, 222, 4, 19, 54, 140, 194, 160, 62, 75, 217, 136, 165, 89, 11, 116, 237, 249, 128, 161, 143, 140, 188, 135, 133, 223, 141, 224, 70, 148, 175, 186, 14, 212, 172, 184, 214, 211, 92, 246, 122, 142, 73, 31, 254, 247, 225, 170, 186, 124, 255, 112, 85, 198, 7, 120, 136, 27, 15, 113, 55, 159, 111, 68, 229, 229, 252, 135, 195, 232, 2, 116, 166, 48, 203, 118, 87, 138, 91, 226, 43, 123, 170, 11, 88, 160, 248, 194, 250, 93, 253, 246, 255, 223, 254, 253, 63, 84, 4, 0, 200, 209, 11, 234, 144, 149, 23, 217, 202, 222, 17, 26, 115, 94, 36, 211, 40, 79, 82, 224, 225, 135, 171, 70, 3, 246, 167, 31, 189, 128, 250, 184, 187, 162, 22, 138, 149, 178, 150, 209, 40, 100, 100, 98, 116, 130, 89, 56, 149, 249, 49, 27, 174, 4, 209, 144, 23, 60, 130, 143, 61, 32, 32, 228, 251, 170, 200, 98, 39, 176, 156, 172, 236, 37, 163, 17, 202, 199, 242, 226, 162, 5, 186, 130, 205, 219, 64, 147, 214, 139, 4, 131, 55, 188, 42, 171, 108, 245, 31, 135, 142, 239, 65, 176, 62, 250, 79, 62, 101, 23, 43, 18, 52, 93, 210, 86, 196, 208, 212, 173, 149, 61, 40, 214, 130, 166, 230, 217, 195, 85, 14, 192, 128, 169, 186, 51, 100, 23, 199, 196, 106, 43, 6, 186, 129, 186, 111, 174, 160, 27, 115, 146, 224, 7, 225, 121, 30, 93, 216, 93, 48, 58, 96, 126, 104, 195, 34, 247, 50, 58, 163, 241, 249, 60, 74, 82, 158, 253, 25, 26, 58, 38, 117, 208, 223, 195, 63, 177, 40, 199, 50, 15, 87, 169, 162, 6, 72, 40, 145, 216, 63, 27, 144, 108, 94, 238, 116, 76, 190, 73, 102, 116, 69, 147, 150, 237, 221, 149, 131, 57, 8, 55, 182, 250, 132, 165, 48, 84, 43, 123, 198, 231, 195, 85, 94, 182, 162, 250, 143, 39, 251, 43, 123, 240, 207, 18, 69, 5, 232, 163, 100, 58, 76, 138, 150, 248, 231, 18, 213, 31, 147, 250, 16, 174, 190, 100, 151, 31, 127, 74, 210, 243, 149, 61, 59, 101, 25, 32, 160, 166, 165, 176, 202, 135, 171, 199, 215, 195, 41, 187, 6, 32, 86, 202, 50, 64, 50, 40, 125, 146, 156, 95, 39, 80, 93, 253, 118, 43, 2, 211, 16, 249, 245, 65, 179, 167, 76, 80, 196, 46, 88, 81, 99, 249, 132, 38, 19, 141, 125, 48, 141, 6, 227, 60, 200, 174, 167, 131, 113, 138, 116, 178, 248, 80, 7, 168, 246, 169, 28, 82, 156, 12, 194, 248, 4, 210, 248, 76, 6, 25, 102, 214, 46, 101, 91, 185, 213, 88, 241, 75, 40, 177, 135, 224, 141, 192, 199, 19, 248, 93, 38, 152, 164, 254, 189, 18, 160, 71, 76, 139, 59, 63, 3, 166, 176, 211, 176, 229, 149, 85, 139, 250, 97, 22, 113, 11, 209, 156, 88, 217, 59, 1, 112, 65, 253, 127, 252, 183, 253, 134, 35, 88, 124, 181, 104, 24, 65, 4, 42, 249, 0, 137, 39, 132, 81, 65, 35, 131, 176, 74, 181, 92, 209, 139, 159, 80, 138, 94, 199, 109, 220, 21, 116, 78, 194, 98, 106, 141, 231, 147, 104, 24, 229, 215, 95, 145, 98, 207, 5, 200, 160, 254, 219, 251, 211, 236, 185, 194, 107, 105, 186, 201, 42, 127, 55, 218, 13, 146, 222, 87, 36, 219, 254, 171, 191, 253, 151, 255, 2, 170, 209, 108, 114, 127, 170, 237, 35, 70, 75, 19, 12, 74, 255, 221, 104, 117, 49, 27, 126, 69, 90, 189, 125, 253, 52, 168, 159, 191, 14, 239, 79, 169, 183, 179, 225, 29, 40, 5, 165, 191, 6, 165, 42, 86, 246, 105, 168, 151, 211, 52, 22, 200, 120, 26, 102, 227, 211, 36, 76, 135, 106, 245, 149, 110, 199, 160, 69, 200, 60, 87, 75, 177, 160, 112, 117, 51, 243, 192, 224, 57, 108, 186, 16, 196, 62, 90, 200, 61, 0, 40, 221, 174, 253, 112, 85, 245, 137, 171, 209, 82, 101, 166, 225, 65, 136, 100, 40, 126, 66, 118, 98, 5, 84, 55, 30, 43, 110, 120, 200, 55, 26, 123, 207, 216, 56, 102, 41, 234, 231, 244, 41, 115, 231, 113, 1, 238, 40, 2, 125, 14, 198, 102, 174, 212, 25, 189, 15, 131, 152, 133, 233, 1, 150, 203, 160, 219, 131, 113, 28, 177, 159, 255, 47, 187, 227, 218, 192, 60, 196, 152, 59, 198, 50, 134, 80, 208, 51, 188, 53, 116, 6, 133, 252, 205, 249, 81, 235, 138, 161, 75, 145, 121, 68, 105, 156, 120, 172, 201, 103, 69, 166, 23, 131, 125, 85, 132, 247, 3, 165, 214, 141, 94, 33, 250, 85, 117, 205, 139, 210, 154, 90, 241, 252, 106, 165, 122, 219, 64, 187, 54, 160, 138, 112, 56, 54, 243, 40, 205, 3, 146, 27, 189, 118, 87, 214, 122, 157, 21, 97, 72, 222, 93, 233, 174, 119, 144, 186, 188, 210, 94, 165, 32, 208, 111, 143, 87, 76, 125, 61, 166, 133, 118, 169, 179, 82, 18, 20, 113, 7, 0, 25, 68, 225, 136, 20, 153, 250, 209, 252, 74, 72, 4, 99, 62, 23, 91, 65, 161, 188, 204, 175, 158, 38, 185, 218, 167, 44, 51, 145, 61, 237, 227, 134, 176, 18, 75, 237, 202, 182, 83, 206, 83, 82, 225, 38, 132, 140, 79, 176, 221, 71, 228, 8, 198, 250, 42, 172, 54, 155, 141, 134, 95, 155, 215, 8, 230, 63, 19, 179, 189, 126, 253, 12, 86, 159, 255, 241, 255, 76, 146, 120, 117, 242, 63, 254, 239, 213, 108, 25, 158, 195, 94, 254, 106, 153, 142, 15, 107, 25, 215, 121, 208, 16, 119, 224, 65, 134, 206, 216, 121, 158, 206, 39, 125, 109, 71, 140, 208, 32, 125, 128, 233, 198, 130, 121, 154, 174, 238, 61, 131, 45, 47, 30, 206, 153, 197, 159, 129, 216, 76, 210, 5, 171, 235, 175, 128, 255, 109, 141, 239, 43, 176, 63, 130, 252, 103, 226, 126, 71, 77, 173, 100, 123, 232, 221, 175, 150, 235, 7, 74, 89, 254, 53, 178, 154, 179, 141, 253, 10, 188, 70, 48, 255, 153, 152, 237, 199, 201, 25, 59, 157, 79, 207, 48, 232, 232, 12, 227, 127, 205, 83, 125, 47, 94, 201, 122, 88, 229, 87, 203, 123, 121, 97, 18, 248, 53, 50, 159, 215, 42, 240, 21, 24, 80, 193, 253, 167, 82, 46, 231, 163, 124, 196, 230, 160, 97, 178, 194, 164, 81, 201, 122, 178, 155, 191, 90, 246, 27, 155, 214, 149, 95, 35, 11, 194, 78, 109, 244, 213, 183, 53, 8, 243, 159, 138, 245, 0, 225, 150, 109, 129, 172, 222, 219, 64, 141, 95, 239, 230, 134, 198, 244, 87, 203, 114, 182, 141, 234, 43, 112, 28, 130, 252, 103, 98, 56, 203, 176, 86, 201, 106, 208, 183, 95, 45, 167, 93, 204, 22, 111, 104, 68, 57, 235, 244, 206, 220, 223, 44, 203, 168, 22, 34, 242, 150, 185, 66, 230, 4, 160, 238, 115, 230, 113, 145, 17, 220, 164, 23, 21, 188, 164, 195, 227, 73, 5, 207, 84, 147, 162, 184, 44, 238, 105, 209, 36, 0, 54, 120, 68, 46, 100, 70, 131, 220, 171, 172, 132, 4, 10, 134, 117, 182, 121, 95, 135, 37, 47, 146, 46, 154, 39, 97, 122, 198, 242, 187, 161, 233, 66, 145, 35, 46, 112, 215, 189, 21, 48, 90, 110, 85, 151, 75, 216, 105, 49, 151, 97, 187, 47, 147, 167, 32, 27, 12, 236, 197, 117, 238, 178, 51, 226, 115, 22, 77, 89, 0, 181, 208, 70, 249, 11, 137, 76, 60, 73, 28, 240, 195, 197, 82, 163, 37, 23, 150, 216, 149, 240, 226, 108, 31, 63, 52, 128, 227, 53, 137, 189, 230, 164, 7, 210, 237, 247, 12, 212, 246, 116, 26, 252, 252, 127, 6, 245, 222, 250, 24, 68, 202, 120, 205, 127, 234, 135, 54, 81, 71, 242, 238, 73, 171, 239, 209, 252, 170, 175, 108, 190, 154, 225, 0, 80, 57, 82, 214, 50, 191, 8, 50, 224, 208, 206, 181, 12, 146, 125, 114, 82, 9, 9, 87, 228, 50, 64, 206, 97, 95, 37, 164, 103, 92, 175, 44, 3, 246, 220, 178, 160, 84, 194, 2, 217, 93, 6, 199, 62, 237, 168, 60, 150, 88, 146, 29, 252, 67, 169, 173, 66, 138, 169, 29, 207, 99, 35, 74, 73, 165, 251, 162, 201, 21, 54, 167, 245, 113, 178, 190, 69, 41, 55, 31, 153, 220, 101, 58, 67, 240, 107, 200, 180, 36, 91, 254, 16, 203, 93, 67, 247, 223, 172, 176, 100, 22, 193, 63, 183, 188, 88, 10, 63, 138, 114, 28, 170, 156, 124, 221, 149, 209, 116, 23, 224, 251, 200, 19, 181, 59, 246, 121, 24, 120, 170, 21, 250, 191, 182, 185, 89, 178, 46, 45, 173, 192, 107, 75, 22, 39, 43, 11, 77, 189, 37, 43, 144, 5, 28, 38, 182, 191, 184, 235, 247, 128, 105, 182, 255, 74, 185, 11, 148, 238, 251, 116, 79, 191, 218, 170, 136, 91, 90, 196, 97, 87, 149, 68, 54, 192, 3, 37, 82, 4, 172, 21, 87, 119, 133, 247, 44, 64, 14, 55, 191, 9, 167, 103, 236, 151, 99, 102, 2, 95, 205, 203, 6, 6, 202, 87, 172, 112, 65, 95, 196, 186, 176, 36, 172, 236, 193, 63, 75, 178, 197, 90, 71, 35, 22, 221, 212, 73, 17, 129, 149, 189, 238, 139, 37, 33, 108, 151, 65, 88, 91, 22, 2, 94, 84, 246, 131, 88, 127, 241, 5, 236, 234, 25, 222, 125, 36, 228, 47, 55, 188, 207, 194, 244, 116, 193, 240, 26, 24, 120, 135, 247, 222, 83, 145, 159, 196, 2, 245, 104, 34, 188, 76, 64, 29, 97, 43, 230, 46, 35, 96, 105, 234, 87, 138, 22, 184, 187, 107, 42, 200, 186, 51, 9, 11, 31, 68, 171, 117, 238, 197, 68, 199, 216, 129, 240, 74, 12, 254, 246, 239, 255, 91, 240, 25, 180, 66, 22, 196, 201, 121, 8, 27, 71, 226, 85, 199, 219, 208, 56, 93, 134, 250, 111, 64, 196, 95, 47, 246, 47, 52, 110, 99, 232, 168, 211, 154, 70, 64, 60, 199, 241, 229, 114, 205, 218, 153, 170, 97, 44, 219, 100, 46, 179, 204, 91, 106, 153, 111, 103, 93, 189, 62, 75, 77, 80, 158, 18, 5, 151, 63, 255, 101, 28, 163, 46, 91, 185, 86, 79, 105, 171, 119, 116, 240, 52, 248, 67, 56, 13, 99, 223, 124, 49, 185, 117, 234, 238, 13, 173, 153, 59, 154, 199, 241, 199, 76, 29, 74, 189, 77, 226, 88, 126, 149, 76, 92, 179, 62, 221, 127, 90, 217, 123, 135, 127, 150, 170, 112, 26, 39, 9, 52, 244, 36, 254, 249, 175, 57, 59, 231, 221, 240, 56, 255, 249, 166, 144, 198, 81, 89, 120, 129, 219, 70, 222, 63, 160, 98, 52, 24, 131, 70, 237, 117, 72, 213, 72, 200, 55, 181, 232, 50, 235, 142, 134, 25, 8, 152, 199, 166, 89, 217, 123, 61, 138, 195, 233, 103, 54, 133, 201, 55, 140, 96, 156, 234, 176, 190, 55, 22, 208, 93, 181, 82, 73, 135, 140, 177, 33, 76, 167, 51, 64, 63, 103, 131, 115, 252, 25, 172, 6, 199, 63, 255, 101, 66, 63, 235, 221, 224, 93, 2, 157, 106, 44, 69, 212, 11, 118, 134, 250, 222, 89, 148, 135, 228, 55, 123, 215, 113, 152, 141, 195, 108, 185, 225, 195, 171, 74, 31, 69, 221, 227, 217, 207, 127, 201, 217, 41, 65, 184, 207, 32, 134, 179, 89, 124, 93, 58, 32, 91, 36, 165, 30, 99, 25, 124, 124, 14, 86, 19, 191, 195, 241, 204, 32, 186, 101, 180, 200, 204, 29, 173, 117, 41, 24, 101, 192, 204, 15, 76, 110, 168, 253, 120, 201, 107, 0, 225, 198, 233, 250, 104, 224, 3, 227, 241, 78, 127, 78, 209, 162, 126, 254, 235, 41, 252, 139, 219, 215, 20, 102, 49, 140, 48, 240, 230, 207, 127, 77, 3, 188, 158, 176, 57, 110, 189, 136, 166, 81, 235, 247, 160, 57, 141, 219, 6, 80, 109, 119, 250, 155, 10, 169, 163, 124, 193, 247, 151, 150, 65, 239, 162, 214, 179, 40, 56, 102, 249, 124, 230, 136, 31, 185, 40, 17, 68, 30, 3, 129, 13, 159, 224, 220, 80, 20, 230, 55, 43, 42, 204, 111, 90, 224, 70, 87, 243, 177, 252, 229, 121, 40, 69, 69, 117, 35, 174, 3, 127, 140, 167, 232, 33, 189, 3, 226, 247, 169, 47, 28, 163, 96, 15, 116, 10, 213, 81, 178, 154, 190, 81, 75, 217, 225, 116, 157, 184, 106, 253, 116, 118, 156, 199, 199, 135, 254, 45, 39, 34, 126, 156, 69, 195, 197, 39, 250, 58, 184, 195, 215, 165, 192, 14, 103, 119, 117, 189, 91, 186, 139, 75, 153, 163, 204, 110, 67, 249, 48, 46, 112, 93, 104, 15, 84, 209, 65, 138, 97, 125, 130, 95, 203, 118, 68, 19, 38, 60, 44, 193, 59, 128, 240, 44, 73, 39, 75, 232, 25, 142, 168, 121, 129, 142, 223, 108, 206, 38, 193, 75, 150, 127, 190, 100, 233, 121, 0, 19, 246, 52, 226, 236, 227, 145, 60, 94, 219, 180, 236, 199, 27, 25, 52, 228, 53, 25, 104, 205, 233, 18, 88, 33, 69, 254, 14, 211, 103, 235, 116, 56, 218, 218, 89, 102, 190, 96, 135, 219, 237, 246, 242, 51, 102, 230, 118, 154, 244, 70, 159, 16, 60, 134, 29, 53, 11, 0, 171, 75, 126, 220, 208, 38, 135, 121, 75, 126, 150, 179, 168, 84, 228, 93, 82, 104, 172, 160, 136, 251, 106, 6, 146, 105, 137, 27, 45, 54, 58, 193, 207, 255, 125, 52, 154, 250, 71, 189, 180, 49, 161, 223, 250, 224, 31, 164, 192, 87, 57, 114, 83, 134, 221, 47, 129, 235, 39, 173, 75, 67, 151, 218, 207, 194, 56, 198, 193, 94, 217, 195, 95, 89, 192, 173, 163, 98, 44, 231, 24, 3, 64, 50, 50, 40, 56, 131, 113, 48, 1, 78, 31, 2, 159, 147, 208, 111, 61, 126, 173, 134, 191, 184, 125, 213, 18, 11, 130, 200, 104, 219, 43, 92, 37, 239, 211, 12, 44, 59, 181, 49, 175, 230, 216, 246, 114, 10, 194, 213, 115, 135, 87, 211, 226, 50, 146, 159, 40, 98, 61, 138, 152, 173, 140, 241, 194, 62, 77, 100, 161, 68, 201, 96, 123, 128, 242, 100, 197, 192, 173, 107, 190, 246, 215, 227, 178, 67, 74, 12, 22, 148, 143, 112, 165, 237, 98, 6, 180, 1, 117, 19, 254, 189, 76, 210, 220, 167, 97, 210, 187, 132, 220, 49, 12, 203, 6, 248, 130, 47, 255, 13, 53, 96, 229, 7, 189, 117, 192, 198, 73, 60, 100, 0, 238, 221, 209, 227, 151, 129, 132, 166, 94, 216, 93, 106, 48, 56, 82, 86, 151, 221, 8, 4, 155, 75, 175, 16, 28, 113, 142, 46, 208, 102, 112, 126, 154, 92, 113, 38, 70, 177, 22, 13, 14, 103, 39, 36, 187, 253, 115, 39, 56, 166, 66, 193, 225, 107, 107, 236, 92, 10, 85, 157, 21, 98, 112, 11, 161, 170, 24, 77, 191, 113, 9, 80, 16, 58, 154, 89, 100, 61, 124, 29, 212, 185, 158, 27, 198, 13, 139, 172, 70, 205, 179, 75, 171, 230, 239, 65, 111, 190, 12, 175, 151, 172, 158, 77, 173, 234, 199, 243, 211, 41, 203, 203, 107, 87, 222, 237, 195, 205, 18, 49, 242, 221, 151, 56, 67, 208, 101, 44, 55, 230, 131, 181, 130, 122, 30, 202, 149, 87, 167, 71, 35, 24, 201, 119, 208, 92, 6, 146, 41, 203, 89, 28, 131, 84, 42, 105, 178, 88, 77, 150, 214, 229, 183, 188, 186, 252, 242, 199, 59, 70, 5, 175, 83, 123, 102, 221, 17, 192, 196, 10, 95, 246, 59, 88, 0, 212, 21, 3, 143, 234, 45, 59, 94, 196, 238, 21, 103, 77, 228, 75, 47, 239, 45, 148, 28, 146, 151, 108, 113, 87, 202, 5, 94, 56, 28, 114, 152, 79, 242, 169, 127, 50, 254, 16, 136, 187, 185, 176, 16, 125, 158, 195, 190, 229, 108, 41, 81, 119, 39, 82, 15, 140, 187, 20, 95, 141, 208, 220, 134, 85, 127, 9, 138, 9, 181, 16, 188, 99, 167, 79, 31, 191, 109, 6, 207, 79, 78, 94, 175, 226, 63, 199, 141, 170, 65, 40, 98, 175, 121, 86, 175, 146, 229, 202, 103, 159, 244, 136, 76, 105, 59, 135, 145, 44, 151, 149, 212, 254, 193, 148, 94, 90, 45, 145, 148, 188, 139, 71, 32, 76, 209, 130, 64, 87, 104, 35, 70, 188, 229, 95, 38, 157, 222, 105, 33, 158, 180, 70, 247, 181, 212, 234, 195, 245, 178, 208, 100, 254, 147, 118, 121, 138, 153, 224, 40, 137, 163, 92, 159, 202, 89, 98, 79, 212, 208, 146, 235, 135, 174, 233, 5, 101, 145, 203, 252, 196, 251, 219, 255, 254, 215, 50, 109, 175, 250, 24, 187, 180, 235, 34, 70, 150, 159, 144, 199, 34, 211, 79, 24, 211, 26, 235, 212, 226, 70, 217, 3, 24, 100, 22, 67, 151, 114, 191, 74, 239, 37, 218, 193, 48, 202, 151, 209, 136, 159, 48, 52, 132, 71, 121, 185, 14, 124, 79, 170, 144, 87, 139, 183, 115, 79, 124, 254, 46, 142, 17, 20, 43, 253, 152, 198, 184, 154, 224, 244, 13, 158, 132, 25, 11, 126, 124, 115, 84, 194, 227, 214, 186, 170, 170, 155, 171, 235, 56, 207, 103, 253, 213, 85, 36, 199, 106, 202, 38, 73, 206, 218, 179, 241, 108, 117, 24, 94, 172, 142, 162, 152, 101, 171, 243, 140, 165, 171, 206, 130, 93, 130, 222, 235, 52, 201, 19, 88, 249, 64, 157, 195, 95, 231, 73, 28, 87, 97, 167, 91, 108, 205, 234, 126, 159, 13, 235, 192, 18, 80, 95, 217, 195, 127, 131, 58, 172, 151, 211, 33, 8, 196, 70, 217, 169, 77, 41, 132, 140, 131, 200, 170, 42, 250, 117, 232, 170, 125, 138, 121, 36, 77, 33, 132, 2, 46, 231, 158, 71, 211, 75, 22, 101, 253, 128, 90, 13, 88, 58, 162, 24, 35, 121, 48, 154, 79, 207, 17, 1, 146, 91, 67, 150, 5, 127, 130, 84, 80, 5, 206, 67, 220, 177, 224, 147, 198, 82, 124, 215, 143, 89, 60, 106, 161, 197, 1, 246, 189, 209, 36, 64, 133, 55, 57, 111, 6, 33, 154, 215, 176, 232, 193, 241, 235, 96, 150, 254, 252, 215, 17, 168, 60, 115, 178, 184, 157, 177, 108, 48, 78, 127, 254, 11, 52, 209, 104, 59, 91, 206, 50, 102, 203, 80, 126, 225, 191, 211, 112, 194, 238, 194, 102, 88, 209, 228, 179, 2, 123, 9, 111, 105, 158, 162, 237, 193, 227, 217, 172, 37, 149, 250, 96, 53, 56, 73, 206, 75, 133, 187, 15, 163, 215, 139, 247, 13, 122, 11, 203, 226, 6, 27, 95, 156, 194, 24, 94, 64, 253, 12, 234, 248, 84, 117, 206, 26, 119, 225, 124, 13, 208, 50, 140, 219, 93, 217, 235, 210, 139, 216, 249, 157, 248, 29, 54, 151, 61, 94, 141, 221, 169, 222, 218, 202, 222, 218, 125, 234, 173, 175, 236, 173, 47, 81, 175, 114, 122, 221, 75, 157, 176, 35, 249, 45, 86, 47, 94, 179, 52, 139, 202, 150, 132, 64, 91, 168, 131, 97, 56, 103, 233, 56, 132, 185, 149, 21, 135, 47, 229, 35, 189, 216, 224, 216, 245, 58, 117, 148, 172, 98, 199, 33, 222, 36, 173, 60, 246, 169, 168, 125, 194, 50, 123, 13, 68, 53, 124, 200, 2, 204, 169, 2, 86, 238, 254, 102, 183, 241, 44, 49, 252, 241, 204, 149, 214, 57, 133, 224, 165, 3, 109, 157, 188, 227, 170, 187, 208, 208, 122, 71, 170, 187, 20, 207, 233, 200, 148, 254, 4, 104, 119, 74, 135, 20, 96, 167, 20, 75, 23, 64, 50, 115, 136, 14, 105, 247, 2, 246, 44, 154, 70, 217, 216, 6, 135, 198, 171, 224, 148, 177, 178, 205, 108, 57, 161, 102, 70, 71, 97, 147, 249, 34, 59, 187, 239, 182, 179, 212, 226, 238, 105, 181, 196, 182, 39, 132, 117, 62, 126, 94, 182, 140, 66, 179, 63, 206, 226, 36, 28, 102, 65, 28, 98, 127, 131, 104, 26, 172, 22, 214, 187, 213, 223, 197, 249, 206, 144, 93, 68, 3, 118, 56, 252, 221, 89, 190, 179, 234, 89, 238, 92, 69, 173, 196, 204, 236, 43, 138, 49, 154, 69, 228, 0, 177, 181, 98, 124, 111, 162, 226, 6, 240, 16, 88, 246, 214, 5, 233, 203, 228, 145, 69, 197, 41, 193, 162, 54, 223, 204, 167, 24, 121, 165, 172, 53, 145, 253, 213, 90, 43, 152, 180, 164, 61, 89, 224, 107, 181, 168, 78, 219, 202, 90, 84, 5, 190, 66, 99, 255, 211, 156, 205, 75, 73, 73, 153, 199, 209, 103, 232, 85, 231, 203, 154, 121, 22, 70, 241, 60, 197, 99, 92, 127, 75, 50, 127, 97, 67, 11, 197, 157, 193, 206, 165, 51, 247, 142, 232, 31, 133, 176, 171, 156, 211, 180, 43, 235, 1, 22, 225, 19, 243, 43, 140, 10, 181, 151, 211, 146, 84, 222, 26, 45, 102, 95, 208, 150, 58, 88, 193, 88, 164, 60, 80, 101, 63, 160, 13, 104, 111, 199, 64, 100, 6, 242, 168, 12, 17, 222, 101, 148, 88, 10, 30, 222, 18, 104, 145, 114, 210, 167, 247, 36, 118, 126, 105, 36, 105, 226, 149, 33, 72, 153, 111, 88, 152, 37, 211, 127, 28, 134, 68, 70, 138, 197, 81, 53, 160, 20, 132, 227, 139, 144, 44, 91, 227, 22, 237, 208, 180, 153, 113, 48, 153, 141, 18, 116, 72, 234, 107, 59, 46, 99, 247, 49, 157, 231, 159, 217, 180, 41, 34, 101, 13, 195, 44, 120, 30, 206, 103, 185, 216, 86, 228, 237, 197, 171, 226, 29, 109, 132, 11, 60, 173, 203, 204, 126, 160, 180, 102, 65, 61, 102, 249, 231, 156, 5, 155, 142, 175, 189, 97, 223, 30, 129, 236, 25, 99, 5, 216, 105, 157, 231, 115, 32, 123, 38, 236, 103, 85, 225, 95, 48, 222, 40, 178, 63, 175, 248, 84, 124, 121, 234, 204, 82, 229, 171, 17, 39, 103, 74, 243, 134, 223, 79, 224, 55, 234, 17, 105, 197, 221, 131, 242, 155, 7, 51, 208, 201, 166, 44, 37, 143, 143, 146, 235, 18, 139, 233, 244, 154, 3, 9, 126, 23, 28, 207, 103, 24, 144, 26, 195, 201, 56, 215, 18, 180, 230, 100, 60, 153, 165, 28, 138, 43, 141, 208, 198, 201, 22, 193, 62, 180, 183, 165, 135, 79, 237, 243, 14, 167, 206, 75, 218, 77, 155, 219, 109, 123, 131, 109, 99, 235, 0, 121, 202, 178, 129, 5, 228, 9, 25, 13, 88, 132, 183, 173, 77, 96, 78, 109, 215, 176, 244, 227, 155, 163, 178, 51, 27, 183, 58, 48, 80, 98, 213, 199, 164, 82, 0, 119, 222, 20, 110, 154, 155, 194, 133, 38, 103, 129, 151, 50, 58, 83, 46, 27, 250, 118, 135, 128, 88, 240, 152, 199, 108, 180, 253, 92, 173, 131, 40, 193, 104, 5, 199, 101, 101, 219, 56, 75, 34, 60, 92, 45, 98, 14, 61, 28, 37, 9, 244, 138, 84, 126, 116, 82, 79, 147, 56, 6, 80, 60, 90, 104, 80, 103, 87, 51, 150, 130, 250, 55, 205, 129, 104, 193, 223, 254, 253, 191, 138, 99, 139, 51, 54, 254, 249, 47, 243, 140, 81, 193, 135, 171, 2, 200, 111, 204, 224, 75, 242, 149, 236, 21, 95, 148, 190, 129, 62, 201, 138, 224, 179, 235, 20, 13, 187, 120, 107, 109, 153, 25, 167, 194, 104, 26, 150, 122, 99, 202, 105, 150, 22, 192, 106, 159, 194, 123, 226, 191, 46, 145, 11, 102, 146, 5, 171, 237, 60, 216, 56, 47, 182, 90, 62, 123, 75, 253, 246, 240, 248, 9, 22, 84, 140, 23, 186, 247, 88, 67, 190, 250, 0, 113, 16, 78, 7, 44, 118, 131, 140, 238, 61, 62, 61, 77, 153, 255, 212, 220, 158, 177, 51, 51, 180, 232, 61, 247, 105, 122, 32, 43, 109, 149, 41, 124, 176, 213, 235, 124, 37, 151, 121, 138, 7, 202, 86, 172, 10, 174, 223, 157, 160, 128, 89, 19, 223, 225, 210, 170, 238, 211, 247, 222, 239, 112, 207, 146, 237, 120, 9, 185, 244, 85, 34, 159, 115, 133, 137, 227, 9, 191, 215, 250, 148, 229, 160, 112, 227, 27, 37, 176, 132, 251, 54, 173, 37, 161, 102, 11, 64, 34, 100, 166, 19, 228, 114, 129, 75, 118, 241, 24, 218, 138, 24, 78, 68, 228, 120, 144, 204, 216, 9, 38, 150, 49, 16, 212, 192, 8, 191, 106, 196, 101, 72, 49, 76, 92, 196, 124, 88, 23, 180, 171, 205, 49, 20, 151, 202, 192, 146, 117, 232, 62, 134, 168, 68, 174, 238, 75, 213, 42, 187, 150, 81, 178, 66, 227, 101, 141, 165, 224, 110, 223, 13, 238, 218, 178, 112, 75, 175, 112, 148, 0, 94, 127, 113, 39, 55, 124, 93, 27, 184, 211, 253, 161, 234, 187, 72, 43, 101, 254, 52, 156, 167, 254, 161, 183, 69, 60, 40, 220, 255, 186, 136, 177, 64, 33, 220, 187, 92, 147, 242, 141, 139, 148, 27, 206, 91, 223, 165, 142, 91, 246, 180, 189, 152, 13, 223, 70, 236, 146, 166, 236, 34, 189, 211, 195, 116, 207, 89, 152, 79, 194, 153, 51, 155, 69, 122, 137, 143, 156, 9, 3, 3, 237, 226, 157, 20, 30, 182, 247, 66, 94, 120, 92, 198, 247, 202, 188, 91, 174, 144, 49, 116, 27, 254, 174, 133, 120, 252, 161, 183, 105, 188, 64, 33, 122, 232, 187, 117, 174, 195, 22, 76, 192, 239, 188, 47, 2, 94, 2, 206, 144, 224, 7, 248, 56, 160, 53, 236, 140, 167, 249, 199, 224, 15, 21, 87, 165, 77, 208, 79, 217, 233, 252, 204, 240, 220, 132, 133, 182, 53, 5, 37, 169, 228, 198, 81, 201, 51, 146, 52, 143, 233, 5, 36, 161, 236, 47, 244, 204, 42, 30, 23, 252, 146, 59, 136, 142, 93, 221, 191, 16, 94, 63, 134, 214, 142, 120, 188, 212, 119, 44, 205, 203, 60, 164, 177, 236, 149, 86, 22, 89, 236, 190, 129, 76, 245, 159, 6, 205, 79, 248, 139, 1, 214, 128, 202, 119, 4, 164, 199, 26, 232, 183, 104, 12, 16, 101, 247, 188, 74, 139, 254, 146, 170, 87, 119, 213, 149, 150, 101, 102, 255, 34, 205, 69, 111, 112, 145, 2, 179, 192, 255, 201, 244, 39, 42, 187, 13, 175, 191, 22, 42, 164, 5, 15, 42, 130, 41, 116, 30, 88, 178, 24, 112, 76, 247, 97, 10, 156, 37, 233, 181, 16, 198, 123, 127, 224, 223, 17, 91, 112, 159, 199, 95, 219, 43, 181, 157, 54, 79, 64, 9, 151, 53, 120, 87, 151, 106, 76, 175, 230, 9, 104, 94, 190, 204, 106, 36, 50, 104, 83, 30, 157, 143, 155, 175, 184, 168, 26, 133, 243, 56, 127, 29, 77, 179, 195, 233, 40, 161, 211, 30, 242, 39, 152, 65, 74, 112, 201, 82, 60, 103, 128, 1, 154, 131, 38, 212, 94, 52, 165, 151, 118, 146, 218, 59, 126, 250, 184, 204, 65, 169, 216, 213, 64, 191, 142, 135, 161, 220, 212, 76, 231, 147, 83, 212, 125, 39, 209, 116, 119, 165, 179, 130, 175, 128, 236, 174, 172, 109, 175, 160, 112, 166, 189, 170, 227, 90, 185, 228, 145, 140, 192, 104, 255, 104, 57, 140, 6, 241, 223, 9, 163, 55, 127, 92, 10, 161, 55, 87, 127, 39, 124, 78, 150, 195, 231, 228, 43, 226, 83, 233, 225, 26, 14, 47, 112, 115, 57, 68, 222, 37, 231, 197, 197, 30, 78, 143, 69, 149, 0, 234, 4, 251, 201, 116, 20, 157, 149, 184, 163, 154, 94, 152, 67, 50, 17, 217, 141, 190, 163, 167, 4, 252, 11, 240, 187, 40, 61, 143, 209, 237, 30, 10, 2, 201, 216, 217, 124, 122, 22, 252, 252, 151, 41, 108, 223, 166, 143, 64, 135, 140, 51, 188, 18, 129, 80, 130, 243, 159, 255, 251, 116, 138, 199, 235, 240, 159, 144, 140, 243, 233, 105, 26, 206, 7, 99, 124, 64, 108, 130, 247, 197, 166, 246, 197, 176, 59, 11, 131, 181, 21, 171, 135, 110, 236, 136, 39, 81, 14, 187, 29, 116, 161, 97, 248, 248, 88, 28, 132, 243, 172, 5, 184, 12, 3, 238, 192, 59, 56, 7, 12, 235, 47, 25, 44, 210, 176, 94, 53, 154, 0, 30, 47, 21, 80, 39, 200, 156, 33, 228, 133, 115, 111, 192, 116, 40, 206, 233, 162, 241, 229, 50, 227, 37, 219, 10, 254, 140, 59, 177, 123, 239, 61, 170, 45, 145, 26, 118, 64, 188, 39, 161, 118, 207, 77, 216, 42, 254, 52, 79, 127, 254, 235, 224, 124, 209, 166, 10, 106, 191, 36, 47, 189, 119, 232, 68, 151, 46, 81, 156, 124, 26, 74, 56, 168, 194, 211, 193, 111, 32, 65, 120, 247, 118, 158, 46, 179, 142, 100, 131, 52, 154, 169, 101, 8, 244, 50, 121, 156, 241, 130, 101, 89, 120, 198, 178, 96, 55, 152, 178, 75, 188, 85, 82, 111, 236, 184, 197, 120, 64, 115, 40, 52, 76, 6, 115, 52, 206, 181, 207, 88, 126, 16, 51, 252, 249, 228, 250, 112, 88, 175, 105, 197, 106, 62, 8, 104, 115, 94, 88, 31, 11, 217, 181, 181, 240, 230, 32, 30, 170, 64, 104, 37, 17, 136, 128, 50, 154, 79, 185, 241, 157, 188, 210, 68, 126, 93, 127, 142, 41, 26, 5, 245, 111, 244, 110, 254, 231, 255, 28, 124, 163, 240, 105, 64, 197, 124, 158, 78, 119, 84, 121, 149, 213, 142, 176, 248, 243, 147, 23, 71, 128, 86, 173, 102, 149, 144, 164, 109, 131, 78, 113, 0, 179, 191, 62, 201, 206, 130, 221, 61, 173, 97, 217, 197, 56, 210, 187, 53, 72, 97, 67, 195, 68, 207, 234, 181, 56, 42, 40, 130, 255, 23, 71, 109, 60, 89, 218, 231, 186, 53, 84, 4, 176, 122, 126, 129, 93, 56, 155, 65, 151, 247, 199, 81, 60, 172, 199, 145, 6, 228, 182, 97, 161, 202, 251, 221, 38, 6, 107, 11, 6, 6, 200, 102, 55, 112, 51, 29, 60, 10, 106, 184, 249, 174, 5, 253, 160, 134, 28, 174, 58, 125, 235, 208, 123, 54, 207, 198, 68, 237, 250, 132, 131, 112, 104, 174, 210, 189, 20, 86, 13, 135, 195, 161, 2, 81, 148, 65, 210, 129, 106, 221, 198, 151, 96, 60, 217, 230, 104, 151, 99, 169, 49, 77, 125, 18, 230, 131, 241, 235, 148, 141, 162, 43, 156, 15, 243, 56, 182, 81, 54, 74, 236, 186, 101, 108, 212, 9, 122, 93, 39, 125, 192, 226, 140, 25, 53, 30, 167, 105, 120, 221, 30, 165, 201, 164, 110, 84, 110, 84, 49, 142, 64, 39, 59, 107, 147, 100, 205, 222, 69, 249, 88, 199, 174, 209, 176, 48, 25, 194, 234, 149, 51, 172, 97, 112, 147, 206, 10, 183, 203, 18, 15, 155, 54, 167, 165, 78, 3, 51, 7, 71, 239, 0, 31, 191, 67, 150, 100, 192, 101, 56, 79, 163, 193, 121, 173, 25, 192, 36, 132, 94, 233, 3, 208, 40, 218, 178, 233, 84, 112, 83, 237, 69, 148, 101, 232, 37, 255, 244, 213, 11, 40, 69, 211, 164, 31, 88, 147, 223, 194, 248, 50, 154, 14, 147, 75, 15, 46, 68, 36, 196, 133, 30, 232, 107, 152, 100, 230, 179, 147, 168, 31, 80, 254, 163, 182, 96, 52, 20, 17, 181, 31, 167, 131, 112, 142, 239, 66, 255, 203, 49, 39, 182, 38, 1, 10, 116, 63, 65, 46, 253, 234, 7, 223, 221, 0, 172, 219, 79, 5, 122, 133, 148, 42, 69, 112, 62, 29, 131, 38, 15, 90, 86, 202, 254, 204, 143, 84, 170, 177, 77, 233, 164, 188, 64, 152, 127, 239, 120, 251, 84, 23, 133, 127, 247, 59, 81, 173, 173, 102, 228, 35, 43, 5, 230, 123, 29, 213, 193, 100, 164, 90, 0, 246, 175, 101, 57, 62, 214, 86, 83, 197, 81, 44, 252, 40, 49, 14, 102, 192, 211, 17, 140, 98, 129, 122, 163, 140, 66, 175, 121, 209, 74, 26, 169, 57, 75, 130, 191, 30, 13, 27, 78, 239, 97, 7, 87, 190, 64, 64, 133, 29, 75, 232, 195, 228, 213, 240, 240, 50, 214, 119, 55, 209, 80, 195, 39, 16, 194, 10, 10, 148, 11, 149, 140, 209, 45, 90, 104, 177, 201, 93, 73, 75, 48, 85, 29, 49, 241, 66, 180, 88, 108, 137, 121, 130, 83, 217, 228, 91, 44, 241, 133, 109, 242, 183, 73, 151, 105, 237, 41, 95, 38, 120, 123, 81, 22, 193, 78, 160, 41, 45, 108, 199, 184, 144, 224, 154, 72, 177, 56, 106, 247, 194, 196, 94, 140, 68, 27, 192, 106, 70, 35, 165, 235, 16, 111, 232, 228, 240, 232, 224, 227, 211, 131, 103, 199, 0, 225, 189, 106, 230, 6, 52, 173, 126, 45, 158, 95, 193, 92, 162, 189, 80, 191, 70, 175, 109, 192, 231, 124, 26, 229, 240, 69, 89, 98, 87, 253, 150, 183, 220, 207, 211, 57, 83, 137, 175, 248, 59, 209, 193, 109, 211, 130, 138, 161, 255, 11, 176, 248, 174, 130, 130, 170, 61, 175, 176, 20, 244, 174, 11, 125, 144, 244, 10, 224, 20, 129, 76, 65, 159, 205, 38, 75, 65, 237, 185, 80, 49, 226, 90, 1, 86, 6, 40, 47, 240, 254, 111, 251, 75, 65, 94, 115, 33, 203, 160, 108, 26, 161, 139, 224, 108, 170, 129, 223, 46, 5, 126, 221, 5, 143, 193, 119, 53, 208, 50, 178, 240, 157, 49, 223, 112, 65, 95, 204, 180, 97, 124, 251, 186, 24, 197, 243, 215, 225, 82, 48, 55, 11, 152, 31, 118, 92, 166, 124, 123, 120, 124, 248, 228, 240, 232, 240, 228, 167, 143, 127, 56, 248, 9, 39, 11, 70, 105, 253, 72, 108, 30, 197, 64, 178, 143, 23, 157, 222, 166, 226, 107, 88, 189, 41, 2, 238, 91, 149, 15, 85, 110, 110, 77, 192, 88, 128, 26, 135, 60, 197, 249, 237, 73, 56, 171, 231, 184, 86, 128, 194, 58, 108, 120, 80, 57, 248, 233, 224, 227, 139, 199, 111, 254, 240, 227, 107, 168, 247, 201, 180, 29, 254, 43, 15, 30, 203, 174, 217, 191, 10, 131, 192, 191, 174, 240, 2, 240, 141, 47, 21, 183, 102, 176, 25, 204, 216, 16, 75, 2, 17, 176, 20, 158, 81, 194, 167, 8, 145, 2, 187, 206, 243, 152, 116, 81, 89, 131, 136, 234, 47, 96, 68, 231, 253, 215, 21, 241, 38, 252, 191, 162, 197, 79, 150, 109, 173, 194, 134, 22, 104, 46, 66, 131, 112, 111, 172, 139, 51, 85, 7, 80, 109, 37, 188, 53, 188, 129, 248, 36, 185, 130, 196, 78, 208, 9, 122, 235, 240, 255, 18, 7, 110, 158, 85, 72, 239, 61, 68, 39, 185, 0, 123, 241, 162, 219, 11, 54, 6, 173, 141, 160, 211, 218, 14, 30, 224, 127, 217, 122, 240, 32, 216, 198, 255, 90, 244, 95, 107, 29, 254, 131, 191, 159, 39, 157, 160, 219, 11, 55, 2, 40, 27, 116, 225, 127, 157, 86, 183, 19, 240, 79, 76, 192, 231, 117, 161, 76, 107, 43, 92, 11, 214, 68, 74, 39, 216, 12, 248, 23, 253, 175, 181, 249, 25, 48, 26, 69, 49, 210, 99, 48, 79, 65, 243, 226, 81, 202, 254, 117, 101, 21, 205, 135, 23, 103, 110, 231, 200, 142, 59, 188, 111, 247, 38, 216, 122, 119, 11, 254, 191, 213, 109, 175, 7, 240, 95, 107, 173, 221, 197, 255, 246, 187, 27, 65, 175, 211, 134, 220, 181, 54, 252, 130, 14, 245, 224, 95, 65, 9, 222, 99, 248, 213, 222, 134, 58, 80, 62, 88, 107, 173, 183, 183, 143, 186, 237, 205, 96, 29, 224, 192, 247, 231, 9, 116, 29, 62, 161, 214, 99, 248, 79, 116, 177, 75, 52, 42, 190, 225, 215, 0, 202, 119, 130, 246, 131, 86, 27, 218, 104, 181, 215, 98, 76, 3, 192, 237, 13, 64, 105, 187, 189, 5, 192, 225, 159, 94, 123, 11, 154, 219, 128, 191, 61, 248, 119, 131, 154, 134, 159, 221, 246, 131, 54, 38, 110, 124, 158, 172, 181, 183, 17, 239, 1, 210, 123, 91, 12, 81, 135, 176, 235, 66, 47, 214, 160, 129, 53, 128, 177, 209, 222, 136, 33, 1, 97, 111, 12, 160, 54, 252, 133, 246, 176, 42, 20, 89, 107, 33, 16, 74, 68, 26, 172, 183, 54, 224, 239, 3, 250, 183, 141, 189, 5, 64, 4, 110, 27, 176, 132, 63, 155, 248, 223, 0, 107, 0, 149, 0, 71, 164, 19, 253, 89, 60, 138, 114, 251, 255, 201, 213, 103, 208, 233, 236, 196, 152, 219, 198, 254, 20, 167, 62, 186, 223, 12, 141, 25, 31, 4, 121, 122, 109, 236, 9, 100, 153, 127, 57, 126, 245, 178, 61, 11, 211, 140, 213, 41, 36, 198, 113, 158, 164, 160, 194, 161, 58, 116, 8, 146, 190, 238, 17, 63, 13, 82, 104, 111, 110, 107, 244, 67, 111, 228, 54, 24, 224, 174, 34, 192, 189, 73, 195, 217, 187, 170, 13, 88, 13, 29, 128, 161, 245, 60, 9, 168, 101, 18, 69, 65, 33, 204, 64, 98, 34, 0, 223, 102, 163, 66, 170, 73, 41, 133, 234, 6, 23, 173, 168, 181, 162, 198, 49, 18, 97, 51, 224, 27, 57, 188, 97, 239, 143, 76, 160, 239, 163, 225, 7, 40, 73, 4, 162, 223, 223, 128, 242, 138, 177, 133, 70, 17, 94, 65, 123, 100, 229, 0, 240, 12, 53, 11, 213, 136, 74, 212, 240, 47, 126, 22, 146, 86, 110, 216, 0, 48, 34, 84, 160, 141, 41, 109, 196, 155, 144, 213, 200, 240, 234, 20, 85, 227, 246, 57, 187, 206, 234, 124, 252, 136, 254, 197, 222, 47, 26, 218, 93, 35, 205, 85, 212, 155, 225, 133, 67, 148, 202, 237, 113, 152, 189, 186, 156, 190, 198, 39, 98, 211, 252, 186, 13, 195, 30, 215, 77, 34, 52, 97, 89, 107, 52, 172, 93, 228, 18, 116, 178, 58, 174, 15, 221, 109, 197, 198, 122, 198, 47, 36, 85, 240, 181, 205, 191, 6, 179, 102, 229, 204, 218, 228, 28, 206, 247, 30, 209, 232, 218, 234, 103, 163, 241, 37, 220, 155, 65, 251, 203, 115, 175, 219, 109, 172, 121, 152, 29, 92, 205, 240, 110, 7, 218, 45, 48, 56, 174, 99, 249, 16, 233, 120, 245, 195, 70, 222, 30, 18, 94, 242, 131, 96, 116, 103, 27, 82, 82, 218, 25, 56, 23, 211, 249, 108, 136, 118, 166, 107, 172, 62, 15, 227, 250, 105, 62, 133, 110, 10, 196, 29, 140, 79, 113, 147, 111, 219, 105, 132, 249, 138, 78, 206, 118, 85, 93, 52, 18, 57, 75, 59, 89, 140, 68, 42, 134, 229, 139, 206, 32, 173, 128, 4, 224, 113, 200, 31, 231, 128, 62, 8, 74, 86, 175, 233, 186, 69, 173, 105, 0, 71, 66, 16, 60, 234, 161, 190, 169, 44, 1, 67, 24, 74, 109, 174, 178, 56, 233, 46, 78, 73, 151, 120, 20, 255, 174, 130, 183, 213, 30, 244, 223, 230, 76, 30, 60, 38, 233, 99, 152, 148, 181, 182, 246, 112, 67, 173, 152, 232, 196, 114, 94, 243, 160, 224, 149, 93, 26, 235, 54, 30, 6, 0, 206, 2, 202, 142, 45, 24, 36, 195, 217, 67, 165, 118, 94, 146, 142, 187, 37, 172, 186, 99, 73, 209, 54, 169, 30, 100, 80, 228, 151, 240, 235, 176, 9, 137, 97, 80, 249, 184, 124, 163, 56, 70, 175, 104, 243, 22, 1, 50, 72, 1, 116, 144, 138, 101, 173, 161, 241, 221, 206, 82, 194, 5, 55, 216, 80, 219, 194, 189, 0, 211, 148, 226, 71, 173, 15, 55, 37, 235, 141, 54, 191, 20, 105, 60, 82, 207, 59, 222, 230, 190, 85, 180, 216, 40, 147, 124, 229, 189, 97, 211, 108, 158, 50, 234, 208, 53, 203, 254, 33, 124, 68, 235, 127, 205, 225, 166, 5, 227, 230, 91, 77, 218, 209, 52, 131, 37, 232, 241, 240, 207, 33, 58, 119, 160, 117, 188, 94, 59, 101, 128, 30, 94, 239, 3, 142, 177, 182, 28, 13, 255, 210, 162, 248, 245, 154, 73, 140, 203, 241, 176, 241, 134, 196, 6, 214, 180, 59, 185, 27, 216, 179, 198, 195, 100, 62, 109, 76, 22, 178, 71, 74, 166, 151, 176, 135, 123, 4, 65, 247, 20, 40, 154, 164, 103, 105, 24, 166, 225, 165, 145, 167, 75, 123, 232, 125, 150, 63, 158, 70, 147, 16, 65, 61, 75, 195, 9, 171, 115, 11, 170, 83, 173, 122, 226, 204, 103, 84, 248, 57, 25, 233, 210, 236, 215, 46, 182, 168, 182, 107, 23, 157, 36, 243, 140, 145, 243, 144, 178, 36, 251, 168, 91, 45, 79, 124, 212, 216, 49, 13, 72, 220, 199, 151, 59, 133, 73, 147, 85, 141, 18, 157, 131, 170, 28, 79, 13, 244, 242, 191, 251, 157, 254, 137, 250, 55, 153, 243, 174, 114, 88, 196, 252, 25, 245, 90, 111, 8, 58, 120, 159, 14, 22, 76, 240, 226, 121, 93, 104, 66, 59, 48, 144, 103, 118, 106, 227, 223, 176, 144, 122, 242, 227, 254, 31, 14, 78, 62, 110, 188, 128, 122, 27, 193, 247, 193, 102, 7, 254, 233, 118, 58, 29, 111, 177, 46, 149, 235, 86, 20, 60, 62, 252, 227, 199, 231, 31, 95, 160, 57, 109, 83, 150, 42, 41, 250, 244, 241, 79, 188, 32, 108, 76, 171, 75, 30, 29, 190, 61, 248, 248, 238, 240, 229, 211, 87, 239, 68, 141, 5, 21, 198, 17, 170, 105, 215, 199, 164, 171, 233, 27, 6, 65, 164, 226, 228, 4, 249, 66, 47, 253, 126, 130, 146, 254, 38, 152, 68, 195, 126, 240, 254, 3, 44, 244, 9, 198, 216, 195, 95, 225, 217, 217, 70, 255, 38, 56, 157, 15, 206, 89, 222, 111, 117, 155, 65, 54, 159, 244, 59, 77, 104, 115, 62, 205, 201, 212, 135, 101, 186, 11, 11, 101, 215, 232, 90, 208, 231, 187, 136, 91, 107, 64, 94, 60, 254, 227, 199, 205, 231, 31, 95, 191, 58, 124, 121, 130, 93, 221, 234, 236, 4, 171, 171, 193, 230, 56, 248, 79, 27, 19, 80, 7, 127, 0, 216, 163, 17, 75, 157, 74, 189, 117, 173, 86, 183, 203, 171, 245, 214, 161, 94, 215, 87, 17, 55, 141, 49, 93, 50, 68, 67, 54, 168, 218, 143, 113, 45, 236, 216, 103, 171, 201, 224, 156, 46, 216, 17, 81, 116, 196, 97, 171, 18, 77, 216, 231, 100, 202, 250, 181, 131, 57, 110, 43, 86, 159, 176, 52, 142, 240, 248, 129, 205, 146, 193, 248, 13, 27, 245, 145, 75, 197, 231, 126, 56, 131, 153, 203, 134, 143, 115, 74, 45, 246, 71, 136, 8, 191, 151, 129, 141, 24, 203, 170, 224, 147, 131, 183, 31, 247, 95, 61, 61, 128, 220, 21, 188, 18, 217, 237, 173, 173, 127, 187, 98, 13, 56, 206, 81, 62, 91, 232, 172, 186, 24, 241, 42, 137, 165, 191, 97, 165, 137, 172, 1, 49, 133, 14, 241, 253, 192, 146, 79, 200, 36, 3, 107, 194, 150, 77, 83, 107, 128, 53, 175, 81, 91, 108, 104, 89, 53, 127, 45, 18, 33, 46, 4, 16, 36, 78, 162, 137, 93, 85, 118, 133, 80, 41, 220, 103, 109, 92, 139, 28, 27, 85, 173, 14, 161, 235, 192, 0, 108, 237, 52, 19, 217, 138, 220, 106, 92, 139, 103, 164, 12, 68, 139, 100, 15, 174, 148, 41, 17, 213, 0, 112, 44, 139, 4, 7, 69, 111, 86, 5, 126, 133, 139, 175, 181, 80, 80, 154, 141, 89, 225, 157, 90, 148, 46, 210, 236, 210, 215, 158, 210, 215, 165, 165, 101, 44, 222, 162, 172, 76, 177, 75, 202, 57, 206, 231, 77, 81, 222, 76, 119, 86, 58, 251, 217, 23, 171, 195, 122, 150, 183, 174, 246, 204, 134, 85, 85, 203, 241, 214, 212, 92, 242, 173, 154, 90, 78, 201, 228, 242, 85, 117, 178, 236, 186, 214, 219, 38, 69, 77, 43, 195, 223, 166, 93, 73, 79, 245, 141, 5, 221, 202, 49, 135, 129, 146, 236, 178, 100, 42, 65, 15, 122, 60, 102, 44, 202, 27, 201, 174, 35, 77, 17, 202, 129, 71, 93, 211, 40, 168, 229, 121, 235, 253, 152, 198, 135, 228, 30, 104, 86, 129, 100, 111, 113, 25, 8, 203, 25, 41, 61, 211, 223, 16, 108, 30, 188, 45, 101, 174, 99, 145, 138, 140, 228, 171, 128, 233, 222, 10, 42, 84, 145, 23, 57, 149, 235, 111, 76, 108, 237, 124, 228, 19, 121, 222, 122, 90, 172, 52, 171, 150, 150, 179, 168, 38, 70, 89, 43, 175, 141, 185, 139, 32, 120, 241, 118, 242, 23, 65, 17, 161, 236, 202, 193, 136, 2, 75, 194, 49, 89, 184, 162, 144, 159, 161, 135, 81, 206, 253, 194, 12, 86, 134, 84, 111, 113, 30, 174, 199, 173, 192, 211, 189, 85, 212, 117, 69, 171, 138, 74, 175, 154, 104, 82, 185, 242, 204, 51, 202, 242, 115, 168, 22, 1, 197, 102, 79, 45, 171, 132, 187, 245, 104, 38, 14, 115, 235, 153, 254, 241, 145, 177, 73, 236, 33, 145, 233, 222, 90, 42, 208, 136, 85, 75, 165, 251, 7, 67, 4, 13, 177, 199, 66, 36, 123, 235, 20, 97, 58, 172, 90, 69, 70, 105, 61, 212, 46, 61, 181, 48, 217, 47, 139, 84, 108, 12, 91, 24, 169, 140, 82, 142, 17, 33, 43, 60, 60, 35, 114, 74, 177, 36, 103, 20, 15, 154, 148, 94, 34, 2, 121, 188, 33, 71, 2, 242, 100, 255, 56, 23, 15, 5, 217, 35, 93, 228, 44, 170, 233, 155, 186, 102, 110, 153, 4, 78, 175, 221, 57, 72, 201, 69, 5, 190, 113, 192, 245, 242, 133, 180, 36, 232, 74, 151, 150, 205, 183, 22, 181, 56, 186, 96, 53, 55, 251, 45, 191, 235, 134, 37, 198, 66, 163, 213, 11, 145, 55, 26, 93, 159, 178, 183, 39, 114, 11, 245, 86, 62, 41, 66, 59, 15, 88, 123, 47, 185, 250, 215, 132, 157, 229, 217, 152, 255, 54, 247, 59, 182, 16, 123, 5, 123, 141, 52, 114, 55, 64, 88, 86, 6, 93, 199, 128, 247, 190, 124, 21, 164, 30, 215, 245, 212, 67, 4, 85, 224, 41, 11, 135, 244, 202, 147, 182, 207, 51, 10, 60, 166, 139, 55, 126, 36, 208, 221, 28, 81, 192, 29, 156, 187, 81, 20, 106, 18, 154, 239, 61, 110, 7, 47, 14, 78, 222, 28, 238, 127, 132, 63, 143, 49, 183, 56, 184, 156, 95, 245, 129, 96, 186, 75, 141, 114, 2, 225, 254, 53, 131, 104, 2, 168, 152, 142, 46, 232, 64, 211, 191, 241, 186, 204, 152, 78, 53, 254, 234, 131, 164, 87, 52, 202, 125, 98, 44, 119, 25, 85, 207, 240, 222, 65, 31, 152, 162, 89, 242, 27, 17, 245, 138, 151, 8, 75, 27, 149, 110, 46, 10, 192, 111, 203, 28, 94, 252, 245, 209, 143, 197, 223, 184, 238, 211, 226, 175, 11, 91, 21, 85, 149, 251, 166, 24, 78, 43, 170, 146, 230, 157, 115, 235, 51, 45, 225, 59, 174, 202, 22, 80, 109, 81, 177, 42, 72, 163, 10, 3, 54, 151, 214, 3, 156, 56, 72, 224, 82, 249, 200, 107, 240, 165, 80, 86, 74, 249, 234, 166, 62, 229, 114, 37, 19, 6, 114, 17, 146, 9, 194, 212, 46, 220, 111, 98, 181, 4, 160, 69, 6, 191, 94, 211, 189, 122, 254, 91, 44, 43, 242, 19, 165, 62, 254, 254, 55, 92, 162, 240, 199, 72, 44, 59, 178, 0, 119, 218, 172, 213, 180, 175, 227, 249, 104, 20, 93, 169, 52, 20, 176, 234, 131, 75, 117, 250, 204, 248, 25, 38, 202, 165, 62, 215, 201, 63, 162, 23, 11, 13, 6, 198, 117, 123, 70, 145, 28, 160, 168, 61, 16, 39, 111, 14, 94, 62, 253, 184, 255, 234, 229, 179, 195, 223, 27, 51, 137, 152, 19, 232, 197, 221, 68, 251, 193, 86, 19, 175, 227, 60, 101, 113, 30, 246, 131, 78, 187, 179, 133, 141, 166, 100, 202, 234, 180, 215, 54, 180, 81, 15, 252, 124, 90, 14, 107, 189, 0, 213, 109, 151, 67, 18, 211, 204, 11, 164, 187, 81, 192, 216, 232, 232, 32, 58, 54, 227, 86, 244, 169, 171, 247, 169, 99, 96, 178, 86, 28, 192, 250, 8, 248, 226, 224, 197, 171, 55, 63, 121, 4, 21, 109, 20, 143, 194, 235, 100, 158, 103, 158, 108, 178, 9, 21, 204, 137, 59, 60, 41, 233, 185, 0, 148, 95, 104, 110, 5, 52, 110, 110, 237, 246, 197, 53, 207, 131, 216, 218, 156, 138, 171, 162, 197, 226, 166, 12, 242, 160, 220, 158, 139, 253, 114, 56, 153, 197, 76, 29, 101, 229, 153, 199, 95, 222, 154, 118, 220, 86, 213, 8, 74, 50, 74, 39, 165, 121, 112, 37, 252, 122, 145, 36, 232, 211, 203, 239, 123, 213, 208, 64, 242, 205, 75, 250, 221, 142, 178, 151, 225, 203, 58, 96, 100, 30, 248, 148, 52, 219, 198, 102, 160, 237, 23, 48, 61, 218, 147, 240, 170, 94, 85, 142, 58, 234, 115, 11, 33, 255, 246, 170, 6, 246, 130, 78, 105, 207, 219, 216, 107, 235, 200, 252, 214, 18, 68, 64, 110, 244, 38, 185, 8, 99, 98, 188, 38, 94, 108, 163, 115, 4, 142, 119, 52, 173, 171, 14, 200, 50, 13, 94, 200, 25, 196, 49, 172, 237, 98, 136, 157, 155, 37, 138, 37, 26, 5, 119, 104, 103, 170, 120, 185, 161, 198, 189, 181, 116, 191, 117, 251, 216, 102, 156, 92, 202, 6, 110, 184, 124, 23, 94, 190, 168, 100, 113, 91, 45, 255, 69, 241, 9, 72, 82, 2, 195, 78, 89, 10, 42, 63, 164, 134, 211, 193, 56, 73, 255, 40, 127, 252, 4, 114, 217, 102, 46, 13, 79, 251, 16, 166, 192, 91, 191, 240, 242, 73, 191, 167, 197, 47, 128, 239, 125, 119, 67, 63, 110, 249, 229, 35, 25, 38, 254, 187, 27, 133, 234, 109, 17, 255, 204, 87, 91, 246, 67, 0, 248, 228, 195, 129, 59, 6, 159, 82, 212, 105, 50, 220, 160, 173, 150, 254, 226, 25, 230, 183, 189, 222, 112, 141, 177, 90, 121, 205, 129, 91, 135, 245, 216, 214, 168, 227, 173, 83, 140, 20, 70, 23, 191, 96, 238, 96, 21, 158, 247, 100, 76, 48, 40, 143, 224, 149, 133, 26, 227, 167, 163, 41, 241, 9, 190, 179, 1, 11, 211, 126, 28, 65, 50, 22, 171, 59, 192, 102, 180, 191, 217, 178, 147, 147, 209, 40, 99, 216, 74, 119, 211, 206, 66, 65, 138, 119, 238, 128, 23, 102, 239, 154, 50, 56, 1, 126, 61, 15, 110, 113, 42, 168, 46, 45, 137, 2, 70, 1, 192, 254, 224, 68, 169, 11, 6, 10, 126, 16, 40, 208, 226, 156, 183, 169, 204, 15, 136, 173, 72, 160, 72, 119, 65, 139, 144, 128, 63, 144, 225, 192, 197, 200, 177, 38, 216, 159, 120, 133, 231, 193, 106, 208, 19, 112, 176, 144, 14, 151, 191, 65, 44, 203, 217, 128, 237, 33, 22, 168, 127, 2, 118, 132, 95, 183, 179, 171, 10, 70, 226, 232, 64, 81, 248, 161, 151, 116, 230, 32, 23, 45, 207, 195, 12, 165, 141, 231, 244, 86, 184, 170, 137, 229, 163, 68, 48, 57, 254, 59, 223, 124, 83, 231, 85, 64, 220, 210, 15, 18, 93, 229, 146, 96, 28, 102, 48, 92, 48, 104, 214, 114, 193, 87, 209, 23, 184, 172, 109, 118, 224, 255, 190, 0, 51, 18, 7, 188, 52, 222, 132, 227, 88, 161, 200, 149, 146, 193, 118, 142, 224, 240, 167, 201, 37, 95, 245, 30, 207, 102, 105, 114, 117, 128, 167, 66, 47, 240, 120, 249, 209, 35, 10, 97, 209, 134, 2, 117, 247, 30, 5, 86, 107, 5, 69, 35, 193, 195, 93, 213, 153, 42, 42, 96, 7, 14, 167, 100, 47, 86, 68, 152, 128, 198, 101, 123, 78, 226, 193, 14, 58, 255, 91, 29, 156, 208, 166, 17, 215, 187, 181, 206, 176, 134, 29, 45, 82, 182, 157, 20, 140, 176, 83, 107, 120, 14, 185, 207, 217, 53, 231, 30, 142, 194, 109, 11, 126, 65, 173, 219, 79, 250, 17, 182, 64, 65, 219, 68, 189, 135, 122, 31, 176, 9, 29, 45, 207, 221, 48, 81, 19, 136, 122, 204, 210, 136, 233, 172, 39, 122, 235, 191, 190, 69, 148, 197, 202, 237, 44, 153, 176, 250, 12, 151, 181, 25, 178, 216, 172, 125, 65, 238, 45, 180, 103, 133, 111, 177, 246, 207, 218, 185, 177, 248, 151, 243, 159, 56, 100, 231, 42, 203, 43, 138, 110, 231, 185, 77, 233, 28, 3, 148, 249, 142, 133, 23, 24, 199, 234, 52, 46, 24, 19, 182, 26, 81, 156, 171, 19, 124, 49, 223, 116, 41, 98, 195, 246, 28, 253, 241, 176, 123, 218, 153, 31, 36, 120, 253, 40, 33, 189, 45, 110, 203, 148, 121, 76, 225, 218, 11, 216, 41, 76, 97, 17, 28, 196, 243, 33, 203, 234, 69, 101, 189, 22, 166, 170, 171, 241, 187, 48, 127, 160, 190, 157, 47, 158, 18, 116, 114, 111, 45, 159, 162, 162, 209, 152, 77, 207, 242, 49, 42, 61, 164, 152, 121, 176, 113, 233, 194, 113, 179, 216, 214, 95, 74, 239, 224, 251, 206, 135, 29, 127, 13, 233, 223, 224, 214, 247, 185, 70, 234, 68, 220, 127, 117, 244, 234, 205, 199, 215, 143, 143, 14, 78, 78, 14, 60, 119, 113, 6, 215, 33, 158, 44, 227, 187, 10, 176, 81, 231, 31, 4, 184, 175, 86, 116, 247, 138, 6, 72, 137, 232, 44, 81, 213, 14, 229, 167, 172, 184, 185, 182, 185, 57, 234, 122, 42, 134, 196, 228, 178, 222, 99, 241, 37, 171, 141, 54, 182, 89, 231, 212, 83, 141, 77, 88, 26, 198, 67, 85, 241, 64, 125, 203, 170, 221, 206, 233, 246, 150, 175, 197, 89, 52, 61, 87, 245, 94, 243, 15, 89, 137, 13, 214, 183, 182, 183, 61, 149, 82, 86, 180, 245, 38, 201, 245, 42, 244, 70, 151, 167, 74, 12, 10, 148, 170, 115, 196, 63, 100, 165, 173, 245, 193, 160, 187, 233, 169, 148, 157, 95, 171, 58, 199, 244, 91, 86, 225, 79, 79, 122, 170, 92, 68, 9, 8, 87, 85, 235, 45, 125, 234, 24, 134, 91, 27, 27, 163, 7, 190, 198, 240, 25, 226, 162, 57, 241, 37, 171, 109, 175, 135, 107, 167, 190, 6, 211, 36, 99, 26, 49, 50, 189, 210, 232, 244, 65, 119, 107, 163, 230, 187, 215, 20, 198, 170, 210, 9, 255, 80, 67, 181, 126, 186, 21, 234, 212, 176, 46, 233, 8, 142, 125, 115, 240, 76, 222, 207, 225, 2, 233, 35, 233, 141, 153, 123, 59, 135, 210, 241, 246, 173, 185, 181, 52, 188, 252, 247, 85, 153, 74, 79, 104, 161, 80, 134, 40, 119, 188, 46, 252, 38, 114, 142, 255, 27, 212, 108, 152, 248, 104, 119, 2, 40, 243, 158, 94, 254, 92, 103, 158, 17, 80, 238, 231, 63, 34, 207, 105, 199, 65, 58, 40, 35, 135, 43, 32, 108, 143, 241, 42, 26, 249, 125, 196, 77, 106, 56, 238, 225, 5, 38, 141, 198, 142, 221, 213, 170, 14, 10, 23, 82, 179, 139, 229, 248, 131, 82, 50, 62, 166, 86, 65, 165, 75, 109, 61, 100, 172, 91, 85, 3, 12, 52, 68, 177, 141, 131, 136, 210, 225, 207, 67, 52, 119, 8, 57, 15, 223, 63, 252, 96, 14, 196, 88, 238, 168, 163, 201, 60, 174, 175, 117, 65, 197, 111, 128, 138, 140, 117, 80, 38, 99, 68, 208, 199, 121, 61, 130, 161, 213, 155, 113, 148, 2, 2, 17, 158, 102, 245, 113, 249, 26, 79, 68, 120, 150, 164, 79, 201, 90, 101, 93, 180, 165, 101, 19, 147, 4, 64, 67, 178, 195, 210, 97, 175, 7, 116, 89, 92, 141, 0, 222, 42, 80, 85, 205, 100, 91, 53, 136, 134, 232, 180, 161, 17, 21, 27, 253, 173, 217, 158, 36, 151, 221, 75, 19, 41, 128, 100, 161, 117, 235, 241, 139, 220, 183, 123, 45, 54, 211, 158, 206, 147, 94, 44, 50, 93, 189, 70, 239, 148, 220, 103, 106, 247, 158, 93, 46, 175, 30, 136, 63, 176, 107, 165, 242, 201, 119, 33, 74, 76, 67, 164, 26, 20, 101, 4, 49, 116, 191, 94, 145, 36, 138, 63, 210, 52, 215, 254, 119, 55, 178, 38, 233, 190, 60, 245, 83, 208, 199, 50, 50, 231, 246, 211, 66, 166, 225, 186, 129, 32, 98, 21, 222, 186, 2, 93, 222, 81, 147, 145, 176, 52, 122, 85, 22, 52, 70, 69, 218, 199, 80, 152, 110, 214, 85, 189, 51, 1, 200, 100, 47, 20, 149, 233, 80, 208, 154, 35, 58, 233, 16, 73, 69, 193, 198, 50, 44, 87, 73, 51, 15, 27, 222, 135, 116, 223, 8, 180, 150, 224, 91, 218, 156, 220, 135, 113, 237, 37, 206, 123, 204, 32, 208, 203, 220, 163, 162, 218, 147, 231, 221, 7, 27, 157, 154, 117, 22, 228, 73, 39, 235, 115, 237, 197, 243, 63, 117, 183, 107, 214, 17, 78, 80, 59, 126, 126, 178, 214, 173, 249, 140, 222, 78, 22, 157, 187, 0, 160, 163, 63, 110, 119, 54, 187, 235, 53, 203, 56, 205, 143, 80, 108, 83, 179, 113, 11, 94, 244, 70, 81, 63, 42, 157, 155, 72, 251, 66, 110, 22, 116, 181, 200, 162, 25, 111, 163, 97, 41, 243, 200, 65, 86, 204, 227, 49, 72, 24, 115, 221, 133, 175, 88, 180, 157, 39, 63, 206, 102, 232, 13, 9, 218, 65, 57, 191, 146, 143, 187, 2, 243, 44, 77, 38, 202, 183, 180, 46, 246, 169, 55, 183, 46, 151, 78, 66, 180, 175, 208, 94, 148, 35, 253, 49, 130, 177, 183, 244, 15, 97, 147, 77, 122, 0, 95, 150, 134, 175, 143, 188, 6, 161, 26, 206, 48, 197, 49, 82, 161, 189, 73, 171, 4, 124, 100, 85, 130, 20, 167, 157, 24, 189, 232, 153, 222, 22, 79, 177, 219, 227, 169, 174, 97, 44, 28, 233, 77, 194, 167, 221, 38, 36, 153, 179, 79, 226, 137, 26, 135, 206, 54, 50, 8, 129, 204, 222, 49, 115, 101, 48, 129, 34, 219, 52, 144, 23, 61, 113, 0, 139, 59, 253, 90, 9, 11, 182, 118, 53, 223, 40, 100, 182, 32, 58, 219, 176, 176, 22, 247, 238, 69, 166, 189, 218, 247, 220, 26, 60, 112, 129, 200, 50, 203, 215, 45, 14, 109, 35, 234, 72, 74, 189, 119, 32, 180, 157, 114, 216, 60, 150, 147, 104, 152, 10, 147, 209, 60, 191, 205, 175, 86, 187, 170, 182, 200, 36, 44, 62, 107, 183, 63, 184, 85, 172, 102, 169, 60, 17, 196, 8, 19, 114, 187, 252, 60, 18, 175, 1, 215, 67, 121, 62, 255, 254, 131, 59, 143, 78, 175, 247, 195, 220, 82, 165, 121, 5, 101, 228, 200, 188, 38, 14, 154, 108, 89, 91, 28, 181, 146, 153, 135, 236, 119, 62, 139, 7, 53, 242, 62, 107, 15, 68, 196, 76, 186, 13, 218, 70, 35, 17, 7, 130, 59, 41, 254, 171, 144, 77, 174, 213, 130, 160, 180, 137, 105, 75, 248, 93, 47, 225, 103, 121, 179, 196, 173, 7, 190, 24, 162, 50, 198, 55, 11, 149, 243, 190, 93, 206, 215, 20, 142, 174, 127, 6, 104, 249, 62, 26, 0, 191, 123, 231, 65, 145, 187, 227, 59, 240, 114, 184, 211, 78, 151, 248, 251, 242, 56, 178, 247, 159, 9, 75, 241, 254, 93, 24, 158, 71, 147, 226, 78, 175, 117, 60, 184, 98, 211, 28, 141, 153, 196, 232, 77, 180, 135, 101, 206, 218, 161, 194, 243, 56, 161, 193, 226, 210, 168, 107, 194, 162, 24, 199, 201, 37, 87, 66, 162, 193, 57, 35, 47, 104, 108, 162, 237, 201, 216, 181, 47, 213, 10, 204, 138, 219, 215, 64, 20, 255, 237, 38, 60, 189, 42, 15, 218, 6, 185, 230, 173, 52, 72, 224, 71, 73, 248, 176, 9, 57, 43, 17, 57, 40, 10, 178, 115, 237, 14, 19, 219, 116, 19, 181, 65, 21, 233, 39, 234, 3, 42, 221, 99, 165, 188, 164, 93, 109, 57, 74, 24, 40, 195, 196, 137, 87, 241, 162, 197, 179, 106, 158, 210, 226, 244, 77, 189, 23, 47, 209, 178, 148, 69, 251, 110, 240, 242, 88, 81, 13, 43, 246, 17, 181, 192, 65, 1, 239, 209, 151, 46, 127, 56, 121, 245, 112, 119, 28, 217, 70, 85, 17, 251, 46, 176, 48, 184, 218, 76, 98, 95, 113, 148, 1, 9, 132, 235, 112, 105, 191, 44, 103, 114, 57, 17, 201, 98, 106, 80, 92, 15, 40, 31, 8, 242, 235, 105, 53, 19, 132, 185, 253, 53, 174, 180, 220, 88, 1, 93, 197, 145, 225, 172, 10, 79, 105, 40, 223, 177, 234, 42, 211, 54, 93, 132, 177, 182, 244, 69, 17, 115, 152, 62, 253, 237, 255, 248, 95, 130, 239, 110, 6, 180, 76, 152, 71, 32, 178, 130, 117, 4, 235, 135, 76, 74, 132, 108, 126, 87, 103, 176, 6, 7, 66, 132, 225, 119, 152, 83, 183, 186, 160, 178, 62, 216, 80, 203, 234, 226, 173, 127, 104, 60, 209, 234, 198, 120, 196, 164, 46, 25, 222, 120, 112, 21, 135, 40, 36, 101, 146, 41, 113, 207, 62, 213, 226, 103, 42, 82, 16, 214, 26, 158, 34, 52, 217, 155, 178, 121, 247, 52, 193, 197, 212, 225, 117, 126, 192, 82, 118, 141, 22, 38, 147, 94, 26, 106, 151, 220, 137, 244, 74, 109, 237, 102, 132, 104, 232, 0, 4, 184, 30, 226, 196, 145, 218, 178, 88, 217, 145, 143, 54, 122, 58, 28, 156, 217, 139, 108, 73, 18, 116, 233, 18, 176, 228, 236, 184, 239, 204, 88, 48, 43, 238, 52, 35, 150, 154, 13, 246, 76, 144, 180, 91, 56, 15, 20, 165, 202, 103, 129, 59, 246, 156, 44, 127, 120, 249, 234, 221, 203, 143, 39, 135, 47, 14, 254, 244, 234, 229, 1, 5, 78, 179, 111, 0, 214, 126, 60, 217, 135, 127, 69, 234, 81, 50, 29, 98, 88, 194, 218, 227, 9, 3, 101, 33, 92, 125, 201, 46, 63, 254, 148, 164, 231, 152, 148, 69, 225, 234, 73, 114, 126, 157, 224, 199, 60, 203, 211, 48, 134, 148, 227, 235, 225, 148, 93, 215, 62, 24, 246, 115, 227, 156, 137, 171, 113, 59, 122, 196, 73, 231, 214, 145, 188, 95, 235, 57, 244, 186, 219, 169, 149, 106, 254, 13, 159, 181, 238, 45, 37, 217, 148, 150, 38, 134, 229, 81, 89, 70, 63, 168, 245, 214, 113, 13, 117, 54, 0, 243, 124, 204, 134, 207, 24, 172, 80, 245, 121, 26, 147, 6, 132, 199, 168, 82, 9, 82, 167, 236, 78, 9, 77, 61, 85, 208, 70, 113, 120, 198, 29, 148, 180, 152, 56, 66, 193, 191, 33, 194, 65, 38, 49, 79, 17, 240, 6, 166, 154, 76, 228, 17, 76, 139, 115, 125, 130, 176, 35, 19, 100, 41, 100, 47, 189, 86, 87, 255, 88, 233, 174, 236, 20, 119, 170, 67, 188, 69, 170, 161, 135, 157, 248, 151, 44, 153, 122, 187, 106, 88, 200, 83, 150, 149, 7, 55, 226, 215, 4, 194, 203, 48, 202, 75, 9, 184, 84, 224, 23, 45, 222, 227, 227, 215, 135, 48, 57, 1, 194, 173, 176, 241, 99, 180, 71, 168, 32, 131, 94, 26, 234, 45, 160, 52, 198, 247, 46, 32, 223, 103, 94, 167, 248, 108, 220, 29, 94, 23, 72, 118, 55, 68, 17, 222, 15, 232, 19, 201, 139, 250, 63, 18, 113, 146, 217, 136, 73, 114, 238, 115, 112, 224, 145, 66, 253, 45, 98, 45, 30, 156, 250, 246, 187, 27, 113, 113, 242, 83, 192, 127, 146, 229, 184, 86, 51, 101, 158, 22, 18, 216, 138, 67, 203, 81, 196, 235, 236, 190, 124, 11, 91, 132, 175, 120, 182, 42, 34, 150, 40, 162, 29, 126, 81, 213, 59, 82, 27, 171, 27, 167, 93, 65, 157, 147, 162, 113, 111, 178, 123, 194, 37, 37, 179, 57, 158, 133, 158, 24, 55, 47, 93, 207, 10, 243, 102, 102, 217, 26, 43, 22, 86, 10, 190, 160, 151, 23, 162, 137, 236, 29, 242, 234, 119, 91, 22, 209, 232, 104, 86, 42, 91, 109, 173, 149, 162, 136, 9, 241, 249, 151, 88, 112, 243, 207, 213, 107, 173, 153, 111, 117, 97, 153, 101, 80, 44, 51, 130, 116, 120, 148, 98, 119, 176, 240, 181, 224, 133, 26, 141, 175, 175, 86, 112, 200, 213, 93, 245, 20, 90, 190, 191, 190, 222, 54, 252, 124, 226, 105, 137, 244, 93, 79, 217, 134, 143, 163, 74, 248, 175, 34, 104, 24, 206, 178, 23, 25, 46, 89, 142, 31, 45, 87, 178, 229, 162, 163, 60, 149, 180, 165, 106, 167, 178, 130, 136, 85, 236, 27, 50, 128, 5, 168, 114, 103, 103, 106, 123, 199, 21, 34, 202, 21, 26, 95, 176, 207, 89, 29, 234, 96, 152, 100, 172, 106, 222, 6, 247, 28, 186, 234, 217, 183, 190, 112, 198, 182, 203, 220, 141, 101, 229, 85, 148, 229, 33, 26, 72, 29, 41, 18, 101, 60, 6, 195, 179, 203, 206, 47, 2, 52, 168, 98, 13, 159, 176, 244, 193, 253, 33, 168, 23, 30, 124, 65, 171, 2, 118, 227, 222, 84, 224, 251, 141, 19, 113, 185, 113, 238, 82, 65, 93, 134, 110, 56, 54, 78, 135, 64, 110, 96, 34, 81, 215, 154, 72, 53, 124, 24, 138, 226, 94, 128, 172, 158, 90, 91, 237, 162, 146, 235, 4, 76, 111, 42, 182, 208, 103, 192, 156, 202, 30, 31, 190, 69, 173, 243, 103, 152, 239, 128, 3, 185, 140, 151, 34, 80, 50, 209, 141, 171, 225, 222, 57, 32, 188, 71, 93, 110, 220, 89, 68, 109, 100, 55, 94, 221, 207, 89, 129, 121, 95, 221, 162, 2, 46, 250, 135, 211, 60, 110, 63, 21, 203, 31, 222, 134, 11, 243, 186, 82, 87, 155, 160, 199, 142, 147, 121, 218, 175, 245, 90, 195, 232, 44, 66, 55, 163, 73, 52, 157, 231, 76, 79, 201, 240, 150, 219, 80, 79, 65, 218, 253, 9, 3, 152, 120, 69, 19, 143, 177, 136, 13, 33, 2, 216, 118, 157, 250, 208, 88, 60, 158, 85, 189, 161, 49, 253, 183, 57, 139, 99, 6, 10, 183, 119, 84, 111, 151, 80, 7, 120, 0, 181, 125, 133, 119, 125, 22, 94, 139, 75, 175, 174, 161, 84, 228, 169, 190, 89, 195, 235, 149, 203, 118, 29, 123, 144, 109, 61, 163, 100, 125, 40, 7, 115, 107, 135, 252, 230, 236, 68, 117, 184, 144, 151, 117, 41, 231, 227, 36, 67, 71, 97, 153, 6, 67, 241, 81, 166, 59, 30, 228, 156, 235, 158, 193, 198, 7, 183, 58, 184, 255, 209, 209, 248, 40, 68, 128, 185, 26, 104, 117, 128, 91, 37, 50, 126, 126, 117, 25, 220, 221, 96, 251, 196, 228, 174, 132, 91, 85, 80, 147, 195, 187, 94, 191, 104, 193, 114, 246, 192, 138, 110, 89, 161, 75, 81, 75, 41, 186, 182, 184, 27, 78, 48, 207, 178, 102, 112, 201, 236, 56, 198, 79, 111, 167, 205, 197, 111, 97, 159, 237, 226, 183, 165, 15, 80, 232, 139, 65, 101, 72, 51, 122, 209, 86, 132, 226, 166, 19, 6, 17, 44, 126, 169, 19, 5, 113, 108, 64, 145, 213, 144, 165, 241, 132, 97, 40, 84, 6, 237, 153, 188, 154, 229, 153, 59, 149, 167, 79, 22, 49, 61, 77, 11, 51, 156, 29, 209, 158, 191, 164, 98, 82, 174, 42, 206, 27, 115, 238, 13, 241, 243, 191, 218, 18, 34, 139, 249, 110, 178, 200, 234, 182, 37, 148, 174, 26, 39, 243, 92, 180, 123, 227, 69, 158, 71, 221, 199, 120, 87, 221, 173, 78, 217, 18, 228, 162, 82, 25, 217, 191, 228, 109, 25, 29, 164, 170, 63, 75, 34, 140, 254, 70, 214, 218, 204, 132, 130, 207, 78, 85, 188, 80, 163, 84, 243, 138, 167, 73, 236, 136, 194, 226, 176, 93, 222, 20, 199, 107, 61, 60, 166, 149, 223, 91, 50, 78, 206, 234, 181, 253, 163, 195, 253, 63, 160, 219, 96, 59, 167, 235, 227, 205, 128, 226, 8, 77, 102, 176, 114, 13, 233, 245, 130, 186, 204, 106, 152, 221, 49, 200, 249, 190, 38, 159, 53, 175, 53, 107, 197, 3, 206, 240, 97, 176, 231, 135, 138, 144, 193, 139, 158, 93, 208, 159, 94, 208, 187, 128, 62, 112, 53, 49, 80, 53, 31, 250, 113, 67, 142, 35, 20, 20, 93, 40, 43, 104, 244, 16, 138, 127, 46, 43, 248, 249, 16, 38, 255, 213, 194, 103, 106, 110, 69, 52, 229, 157, 37, 198, 245, 156, 93, 15, 147, 203, 105, 217, 200, 178, 54, 249, 82, 225, 148, 63, 200, 6, 225, 140, 89, 187, 132, 247, 38, 169, 173, 97, 80, 195, 99, 15, 129, 45, 153, 20, 229, 155, 194, 64, 231, 235, 149, 39, 108, 36, 158, 223, 38, 252, 53, 145, 228, 188, 137, 222, 169, 128, 78, 222, 148, 183, 167, 239, 246, 188, 134, 87, 6, 42, 255, 101, 156, 217, 223, 110, 158, 62, 232, 109, 233, 215, 225, 48, 27, 5, 222, 62, 138, 16, 44, 50, 76, 242, 214, 108, 14, 93, 104, 69, 195, 152, 89, 162, 81, 122, 26, 224, 117, 19, 142, 170, 43, 37, 109, 99, 83, 160, 53, 191, 182, 62, 92, 219, 222, 182, 4, 99, 73, 243, 163, 48, 51, 79, 200, 188, 210, 175, 128, 61, 58, 61, 29, 245, 214, 151, 131, 157, 197, 201, 101, 173, 82, 176, 89, 189, 253, 198, 219, 221, 162, 113, 225, 181, 175, 131, 52, 154, 174, 85, 222, 5, 186, 31, 28, 87, 112, 26, 103, 183, 214, 177, 109, 81, 40, 185, 58, 30, 135, 67, 186, 249, 242, 137, 7, 238, 95, 155, 93, 225, 81, 6, 86, 184, 93, 91, 251, 100, 84, 114, 215, 22, 107, 136, 154, 14, 97, 155, 14, 23, 53, 220, 21, 150, 186, 212, 112, 87, 190, 34, 207, 13, 223, 106, 7, 62, 124, 173, 105, 206, 238, 220, 226, 14, 14, 66, 203, 80, 119, 175, 101, 93, 244, 191, 247, 192, 114, 103, 28, 25, 160, 84, 45, 175, 119, 27, 55, 50, 112, 211, 6, 8, 135, 50, 131, 134, 123, 239, 26, 107, 160, 169, 1, 75, 218, 166, 6, 221, 199, 206, 136, 165, 206, 67, 111, 36, 231, 125, 174, 32, 231, 228, 25, 7, 179, 78, 137, 15, 35, 67, 50, 174, 146, 41, 70, 174, 72, 19, 242, 86, 94, 161, 192, 14, 241, 2, 13, 253, 230, 133, 116, 218, 252, 187, 52, 142, 77, 89, 173, 139, 192, 4, 90, 227, 232, 72, 232, 109, 28, 51, 202, 27, 199, 220, 146, 198, 213, 149, 121, 44, 228, 140, 30, 79, 149, 55, 181, 204, 43, 244, 228, 243, 131, 131, 201, 11, 201, 224, 49, 77, 227, 136, 128, 199, 122, 208, 123, 32, 220, 19, 253, 189, 16, 153, 21, 61, 17, 37, 42, 73, 137, 205, 90, 164, 212, 163, 69, 252, 157, 177, 145, 77, 91, 24, 113, 127, 93, 147, 181, 208, 253, 210, 207, 91, 152, 83, 193, 92, 152, 93, 205, 218, 228, 73, 101, 32, 192, 99, 86, 24, 171, 135, 134, 11, 228, 34, 42, 100, 15, 33, 8, 144, 208, 80, 155, 204, 166, 81, 109, 33, 225, 76, 253, 125, 57, 34, 26, 117, 140, 190, 32, 38, 250, 82, 214, 244, 189, 100, 97, 93, 20, 14, 103, 186, 179, 48, 81, 93, 198, 93, 17, 200, 139, 79, 43, 140, 11, 111, 216, 12, 139, 164, 34, 114, 134, 49, 42, 71, 226, 194, 47, 55, 35, 72, 2, 57, 78, 246, 55, 65, 187, 77, 39, 76, 28, 40, 42, 64, 99, 22, 198, 249, 24, 88, 146, 3, 72, 206, 139, 139, 211, 2, 41, 237, 38, 117, 177, 52, 243, 102, 111, 203, 119, 145, 188, 163, 135, 217, 115, 14, 255, 94, 75, 129, 225, 112, 93, 182, 164, 52, 218, 162, 11, 229, 184, 192, 88, 61, 133, 149, 14, 95, 40, 246, 89, 194, 245, 163, 91, 28, 24, 207, 98, 225, 177, 184, 10, 51, 160, 17, 207, 10, 148, 252, 75, 3, 148, 149, 141, 241, 173, 220, 83, 98, 23, 42, 98, 244, 48, 112, 97, 171, 42, 158, 102, 91, 174, 197, 30, 193, 236, 249, 112, 208, 141, 252, 80, 209, 83, 194, 97, 157, 206, 162, 161, 62, 65, 27, 67, 233, 53, 254, 193, 8, 109, 75, 122, 60, 30, 115, 42, 200, 160, 53, 155, 102, 208, 26, 35, 102, 77, 207, 142, 158, 99, 207, 133, 202, 219, 221, 60, 156, 89, 67, 222, 134, 46, 174, 112, 151, 92, 217, 54, 178, 46, 22, 168, 20, 80, 162, 97, 169, 91, 194, 151, 157, 110, 25, 63, 12, 122, 197, 249, 106, 48, 140, 210, 62, 236, 204, 195, 156, 34, 29, 229, 24, 55, 73, 69, 197, 225, 253, 118, 187, 198, 233, 67, 38, 39, 126, 13, 61, 198, 171, 45, 45, 32, 107, 155, 103, 89, 173, 203, 242, 203, 99, 32, 107, 188, 183, 171, 182, 130, 238, 135, 71, 64, 129, 71, 143, 22, 98, 201, 97, 5, 187, 213, 192, 218, 23, 118, 189, 81, 148, 154, 213, 58, 158, 66, 195, 104, 52, 34, 161, 65, 109, 180, 120, 37, 87, 115, 59, 147, 247, 250, 240, 82, 30, 214, 113, 204, 157, 227, 107, 216, 198, 130, 108, 139, 80, 185, 71, 10, 74, 142, 11, 190, 199, 120, 77, 78, 232, 144, 148, 93, 60, 141, 82, 197, 190, 60, 26, 146, 206, 190, 156, 148, 230, 246, 110, 72, 53, 236, 28, 254, 230, 237, 25, 204, 74, 189, 221, 134, 40, 77, 61, 68, 61, 231, 81, 80, 155, 207, 200, 226, 66, 187, 108, 125, 107, 32, 118, 73, 46, 16, 160, 136, 214, 49, 116, 239, 87, 136, 35, 235, 34, 60, 64, 213, 72, 35, 224, 13, 217, 186, 200, 210, 222, 97, 242, 117, 23, 177, 76, 205, 174, 114, 147, 11, 39, 145, 43, 203, 228, 179, 5, 216, 32, 250, 74, 55, 188, 206, 191, 89, 254, 58, 165, 43, 32, 186, 172, 166, 17, 110, 236, 120, 139, 191, 164, 173, 148, 94, 154, 51, 134, 115, 186, 162, 96, 235, 135, 121, 18, 66, 217, 249, 138, 140, 210, 23, 243, 64, 19, 18, 68, 75, 86, 116, 77, 61, 5, 203, 241, 17, 125, 104, 141, 176, 78, 164, 218, 148, 205, 209, 113, 203, 218, 54, 211, 224, 234, 229, 120, 251, 130, 33, 242, 228, 18, 109, 37, 200, 20, 225, 101, 120, 93, 91, 100, 242, 149, 186, 71, 202, 39, 222, 46, 77, 141, 61, 206, 240, 92, 166, 34, 84, 254, 11, 161, 214, 181, 124, 197, 83, 88, 34, 25, 229, 252, 113, 32, 100, 229, 134, 71, 167, 0, 134, 144, 98, 68, 9, 7, 100, 229, 166, 106, 189, 169, 117, 171, 66, 103, 192, 210, 195, 199, 105, 154, 92, 194, 168, 57, 142, 144, 67, 141, 149, 139, 187, 152, 127, 251, 159, 255, 87, 107, 126, 13, 77, 246, 214, 74, 254, 135, 123, 113, 179, 86, 91, 116, 130, 250, 135, 89, 164, 214, 16, 82, 175, 154, 28, 79, 119, 141, 227, 199, 95, 111, 197, 73, 14, 247, 33, 147, 235, 21, 134, 24, 186, 255, 54, 181, 157, 39, 207, 162, 43, 54, 172, 75, 112, 24, 49, 189, 246, 183, 127, 255, 15, 199, 91, 127, 209, 58, 91, 178, 126, 218, 19, 149, 156, 5, 27, 234, 221, 221, 218, 249, 44, 18, 49, 19, 181, 78, 214, 5, 57, 144, 99, 20, 98, 141, 114, 160, 234, 146, 136, 9, 248, 121, 113, 119, 228, 11, 128, 227, 61, 16, 19, 238, 62, 221, 12, 249, 2, 144, 92, 90, 233, 32, 223, 210, 93, 143, 59, 128, 20, 198, 12, 228, 151, 131, 248, 133, 99, 85, 224, 91, 83, 17, 216, 78, 208, 152, 244, 168, 90, 195, 191, 111, 44, 138, 74, 170, 121, 138, 211, 142, 189, 40, 9, 116, 240, 20, 162, 141, 87, 81, 8, 213, 63, 94, 168, 124, 31, 35, 186, 65, 135, 119, 178, 67, 37, 209, 142, 68, 129, 178, 195, 33, 190, 224, 80, 161, 71, 109, 252, 40, 89, 68, 7, 52, 113, 16, 191, 22, 21, 230, 85, 90, 190, 69, 181, 114, 149, 161, 3, 88, 222, 154, 38, 95, 137, 209, 185, 84, 109, 148, 53, 117, 150, 36, 67, 67, 212, 170, 21, 216, 15, 143, 68, 115, 41, 180, 211, 176, 4, 24, 210, 224, 155, 93, 169, 47, 148, 214, 39, 247, 132, 74, 163, 166, 15, 152, 125, 230, 152, 21, 180, 47, 150, 7, 225, 209, 147, 240, 215, 199, 253, 116, 64, 161, 179, 0, 37, 255, 90, 36, 216, 193, 184, 168, 1, 120, 56, 103, 134, 40, 250, 37, 114, 197, 58, 176, 227, 192, 49, 93, 4, 120, 53, 100, 33, 93, 34, 150, 249, 0, 128, 60, 127, 18, 166, 245, 187, 155, 36, 73, 98, 131, 88, 55, 197, 248, 34, 33, 206, 69, 184, 99, 103, 44, 214, 22, 121, 191, 238, 2, 159, 117, 147, 72, 112, 235, 81, 211, 216, 90, 9, 161, 220, 240, 2, 209, 46, 224, 153, 128, 148, 225, 199, 2, 86, 8, 99, 63, 64, 126, 175, 206, 132, 133, 182, 54, 11, 12, 137, 93, 63, 4, 126, 49, 206, 132, 128, 86, 19, 11, 2, 205, 214, 198, 162, 81, 195, 199, 67, 158, 179, 112, 40, 226, 137, 102, 198, 248, 185, 30, 28, 69, 110, 177, 139, 211, 106, 224, 144, 37, 244, 226, 167, 53, 61, 202, 12, 221, 104, 55, 241, 229, 53, 49, 163, 0, 124, 235, 59, 40, 248, 213, 62, 93, 104, 6, 176, 179, 236, 43, 222, 57, 241, 133, 207, 31, 10, 225, 159, 228, 135, 67, 35, 200, 219, 211, 36, 255, 180, 99, 221, 175, 76, 242, 58, 21, 108, 42, 195, 84, 211, 52, 75, 53, 77, 163, 212, 63, 205, 59, 139, 133, 161, 165, 30, 39, 151, 60, 230, 121, 201, 165, 77, 113, 206, 193, 93, 220, 205, 151, 93, 47, 11, 163, 185, 180, 57, 149, 11, 33, 180, 28, 161, 24, 194, 146, 125, 199, 116, 202, 131, 174, 75, 104, 202, 68, 85, 14, 142, 44, 72, 8, 143, 202, 114, 128, 229, 90, 3, 191, 239, 53, 36, 199, 180, 171, 156, 76, 85, 184, 56, 121, 44, 88, 32, 193, 169, 8, 129, 245, 148, 49, 237, 82, 78, 36, 121, 172, 107, 185, 30, 139, 166, 161, 7, 116, 81, 44, 131, 129, 155, 114, 119, 77, 254, 136, 165, 8, 190, 247, 86, 189, 159, 4, 114, 37, 53, 46, 140, 73, 39, 78, 14, 169, 106, 175, 224, 64, 130, 217, 60, 96, 79, 41, 70, 161, 116, 20, 170, 186, 156, 203, 49, 60, 143, 102, 40, 234, 26, 66, 144, 190, 229, 207, 38, 213, 23, 204, 70, 30, 142, 69, 15, 237, 47, 245, 160, 226, 169, 43, 21, 182, 127, 87, 11, 220, 79, 118, 2, 29, 81, 114, 196, 40, 0, 53, 108, 151, 86, 68, 138, 23, 173, 47, 245, 124, 175, 240, 187, 227, 113, 198, 241, 141, 24, 46, 101, 194, 153, 254, 126, 89, 147, 130, 246, 235, 47, 101, 200, 251, 54, 118, 184, 214, 169, 136, 199, 111, 186, 207, 90, 239, 178, 225, 230, 27, 223, 184, 235, 56, 106, 103, 162, 191, 174, 0, 205, 29, 233, 207, 111, 40, 53, 215, 124, 132, 1, 138, 153, 15, 47, 168, 0, 2, 230, 219, 66, 216, 5, 236, 148, 125, 236, 10, 141, 154, 68, 132, 4, 223, 201, 177, 232, 219, 163, 226, 192, 30, 213, 45, 223, 217, 180, 6, 64, 59, 85, 46, 234, 23, 199, 203, 233, 217, 105, 88, 223, 232, 53, 123, 221, 110, 179, 187, 177, 214, 68, 27, 106, 131, 224, 90, 101, 122, 107, 219, 205, 205, 45, 252, 127, 94, 164, 204, 189, 157, 223, 20, 201, 93, 55, 205, 162, 117, 254, 139, 26, 73, 70, 35, 250, 109, 221, 228, 119, 125, 142, 79, 61, 206, 188, 130, 69, 0, 226, 227, 215, 100, 159, 48, 121, 4, 210, 223, 29, 61, 126, 73, 237, 188, 18, 237, 24, 162, 156, 195, 244, 16, 186, 0, 172, 34, 19, 250, 160, 127, 219, 235, 13, 54, 54, 152, 49, 10, 165, 13, 20, 190, 4, 157, 81, 247, 65, 47, 172, 45, 51, 55, 208, 82, 247, 89, 60, 137, 86, 231, 175, 234, 53, 241, 213, 75, 46, 210, 220, 24, 160, 252, 157, 53, 10, 57, 84, 20, 82, 86, 24, 17, 173, 184, 83, 132, 42, 246, 24, 104, 83, 116, 130, 83, 134, 86, 17, 64, 230, 117, 116, 197, 226, 55, 148, 3, 192, 187, 101, 113, 152, 213, 211, 109, 203, 4, 59, 30, 100, 217, 59, 61, 100, 121, 183, 201, 127, 211, 32, 212, 41, 4, 49, 33, 220, 240, 213, 124, 190, 160, 38, 239, 160, 91, 245, 82, 214, 227, 101, 9, 135, 239, 121, 159, 93, 35, 176, 83, 246, 185, 167, 44, 173, 34, 188, 227, 132, 46, 173, 29, 180, 82, 137, 84, 142, 10, 37, 143, 45, 203, 166, 94, 13, 42, 237, 184, 121, 162, 50, 84, 245, 186, 249, 202, 65, 198, 184, 124, 39, 105, 56, 205, 208, 240, 0, 244, 232, 192, 255, 232, 95, 189, 83, 69, 225, 65, 24, 99, 80, 66, 232, 72, 211, 238, 142, 205, 44, 72, 161, 130, 95, 136, 6, 21, 230, 185, 41, 218, 61, 98, 96, 89, 156, 63, 117, 59, 54, 176, 25, 4, 24, 111, 106, 22, 166, 55, 126, 111, 211, 95, 114, 83, 47, 184, 89, 94, 238, 14, 97, 133, 229, 185, 33, 164, 187, 6, 63, 227, 85, 27, 183, 147, 220, 184, 243, 52, 188, 198, 16, 96, 67, 203, 35, 85, 60, 30, 200, 15, 97, 40, 122, 228, 51, 208, 125, 126, 98, 176, 159, 116, 24, 12, 157, 83, 68, 160, 61, 89, 248, 5, 8, 183, 113, 29, 163, 12, 118, 27, 237, 89, 136, 239, 118, 164, 121, 189, 215, 4, 113, 236, 46, 47, 67, 183, 62, 57, 213, 55, 170, 170, 138, 78, 130, 106, 125, 77, 161, 147, 241, 31, 35, 200, 156, 19, 247, 42, 188, 166, 155, 182, 24, 121, 45, 188, 206, 160, 169, 235, 204, 227, 255, 54, 29, 138, 139, 5, 28, 5, 207, 9, 119, 154, 151, 148, 160, 60, 100, 97, 202, 0, 72, 69, 79, 96, 233, 166, 6, 241, 84, 168, 225, 99, 83, 124, 147, 183, 111, 14, 9, 129, 3, 197, 59, 79, 172, 12, 50, 201, 86, 112, 47, 94, 64, 124, 66, 207, 204, 214, 227, 8, 45, 213, 131, 112, 214, 12, 200, 137, 210, 17, 181, 34, 213, 241, 234, 195, 205, 4, 194, 169, 243, 2, 118, 72, 170, 44, 47, 194, 10, 3, 244, 6, 175, 144, 205, 232, 172, 14, 95, 65, 209, 74, 180, 168, 68, 57, 186, 225, 112, 200, 177, 21, 113, 201, 241, 13, 20, 198, 67, 189, 92, 227, 27, 15, 77, 241, 102, 238, 139, 76, 26, 215, 129, 8, 77, 222, 172, 61, 124, 225, 25, 26, 133, 101, 121, 154, 42, 197, 59, 198, 143, 232, 121, 21, 214, 198, 231, 122, 131, 126, 241, 209, 221, 112, 99, 131, 241, 247, 223, 101, 107, 124, 206, 77, 162, 97, 77, 1, 129, 15, 5, 3, 95, 2, 118, 116, 38, 194, 225, 144, 34, 81, 146, 244, 29, 197, 9, 236, 169, 64, 39, 94, 85, 248, 217, 110, 107, 103, 103, 109, 158, 69, 237, 181, 186, 141, 64, 79, 42, 64, 90, 90, 134, 106, 233, 161, 94, 30, 196, 69, 145, 211, 210, 115, 104, 200, 96, 9, 232, 153, 98, 124, 81, 91, 188, 68, 70, 190, 104, 29, 59, 153, 30, 49, 54, 51, 10, 249, 126, 57, 198, 109, 190, 134, 232, 158, 214, 152, 239, 148, 76, 5, 150, 86, 144, 249, 209, 144, 68, 96, 85, 203, 233, 59, 215, 2, 252, 252, 127, 19, 128, 224, 215, 250, 248, 189, 198, 86, 23, 125, 209, 164, 25, 49, 67, 43, 253, 195, 174, 174, 46, 220, 159, 20, 220, 239, 19, 45, 40, 254, 3, 57, 9, 23, 218, 163, 98, 126, 224, 38, 54, 85, 122, 23, 62, 131, 100, 132, 249, 43, 142, 118, 178, 166, 121, 29, 166, 242, 42, 156, 148, 126, 252, 217, 108, 243, 93, 236, 146, 87, 0, 32, 175, 204, 42, 158, 103, 198, 117, 158, 220, 189, 168, 3, 27, 158, 19, 178, 222, 102, 218, 93, 27, 116, 126, 195, 239, 190, 239, 38, 217, 163, 160, 190, 232, 5, 1, 60, 75, 210, 190, 118, 28, 217, 207, 141, 108, 218, 193, 149, 8, 158, 81, 121, 116, 69, 129, 210, 31, 5, 50, 116, 131, 123, 46, 108, 143, 119, 233, 195, 61, 212, 105, 99, 65, 65, 233, 162, 110, 224, 104, 163, 245, 136, 199, 85, 232, 27, 69, 180, 232, 110, 149, 34, 149, 154, 105, 22, 130, 177, 201, 69, 91, 211, 124, 97, 188, 113, 63, 112, 93, 130, 135, 18, 81, 0, 44, 94, 31, 175, 88, 3, 240, 202, 49, 172, 255, 175, 113, 177, 145, 224, 109, 181, 75, 50, 134, 80, 205, 104, 157, 118, 245, 180, 29, 239, 138, 160, 215, 146, 74, 155, 190, 30, 116, 141, 5, 97, 195, 47, 204, 137, 103, 75, 32, 21, 189, 7, 64, 138, 178, 11, 229, 187, 215, 47, 234, 203, 101, 160, 82, 43, 150, 17, 121, 149, 215, 139, 203, 30, 143, 248, 226, 113, 169, 18, 37, 252, 0, 23, 120, 178, 79, 97, 222, 98, 242, 138, 122, 255, 161, 204, 87, 168, 106, 92, 158, 62, 254, 233, 227, 139, 99, 148, 24, 118, 129, 77, 202, 63, 62, 252, 227, 199, 231, 188, 196, 209, 225, 219, 131, 143, 239, 14, 95, 62, 125, 245, 14, 18, 220, 71, 109, 240, 217, 54, 228, 207, 147, 138, 230, 234, 133, 66, 240, 190, 248, 105, 250, 23, 229, 40, 134, 234, 74, 139, 120, 175, 126, 217, 197, 202, 100, 162, 71, 248, 97, 96, 69, 77, 18, 52, 150, 149, 134, 117, 189, 87, 102, 174, 163, 229, 71, 83, 106, 93, 109, 87, 59, 98, 234, 3, 190, 114, 28, 26, 37, 138, 84, 229, 244, 67, 10, 169, 217, 55, 209, 163, 179, 137, 109, 50, 29, 38, 151, 128, 48, 37, 12, 128, 49, 100, 88, 137, 219, 28, 105, 167, 119, 241, 142, 195, 180, 189, 93, 78, 130, 134, 242, 72, 211, 180, 90, 229, 65, 197, 229, 24, 174, 109, 30, 137, 86, 244, 193, 210, 252, 100, 53, 108, 138, 255, 212, 27, 228, 126, 112, 92, 27, 231, 185, 238, 14, 2, 139, 84, 249, 45, 130, 34, 251, 156, 207, 52, 53, 143, 73, 179, 207, 248, 124, 254, 71, 168, 2, 247, 94, 221, 238, 41, 112, 194, 105, 148, 139, 226, 188, 235, 192, 197, 51, 62, 254, 117, 146, 148, 48, 202, 36, 23, 235, 179, 187, 123, 68, 2, 206, 88, 82, 122, 229, 55, 26, 126, 135, 193, 183, 210, 135, 89, 34, 227, 178, 161, 254, 206, 14, 94, 125, 155, 213, 237, 221, 151, 6, 169, 192, 83, 75, 181, 95, 228, 41, 87, 56, 140, 74, 86, 51, 30, 121, 105, 197, 6, 85, 251, 32, 189, 63, 98, 114, 148, 169, 18, 129, 182, 202, 22, 187, 13, 71, 4, 62, 210, 247, 76, 11, 68, 37, 116, 119, 181, 88, 114, 81, 170, 181, 186, 37, 13, 122, 52, 118, 45, 215, 167, 184, 23, 142, 11, 62, 1, 228, 35, 8, 137, 179, 18, 138, 120, 116, 33, 3, 135, 174, 67, 19, 125, 249, 240, 16, 165, 98, 161, 209, 201, 210, 173, 164, 75, 183, 154, 48, 221, 42, 202, 252, 127, 142, 33, 74, 101, 44, 30, 185, 178, 65, 254, 58, 140, 82, 60, 19, 246, 42, 73, 232, 215, 144, 57, 126, 219, 210, 47, 194, 43, 182, 198, 243, 137, 167, 134, 230, 4, 225, 173, 117, 49, 27, 122, 106, 113, 63, 5, 111, 133, 152, 222, 148, 82, 239, 87, 18, 162, 130, 166, 77, 194, 65, 125, 32, 104, 241, 225, 64, 73, 230, 185, 245, 96, 154, 239, 177, 18, 168, 236, 121, 165, 4, 234, 242, 53, 238, 70, 56, 139, 17, 14, 239, 117, 76, 240, 0, 12, 240, 252, 33, 136, 208, 77, 155, 240, 234, 19, 114, 239, 53, 12, 237, 66, 228, 254, 133, 72, 191, 215, 48, 55, 11, 121, 47, 9, 171, 165, 21, 17, 51, 164, 52, 143, 119, 172, 59, 217, 206, 208, 249, 196, 74, 129, 214, 10, 57, 190, 200, 10, 139, 215, 230, 143, 48, 172, 108, 189, 88, 145, 201, 39, 24, 119, 86, 116, 232, 167, 118, 235, 190, 229, 25, 118, 243, 159, 205, 116, 25, 85, 163, 25, 136, 200, 83, 79, 185, 7, 132, 56, 133, 228, 175, 39, 138, 180, 90, 237, 75, 212, 121, 190, 196, 123, 98, 231, 152, 158, 197, 186, 218, 152, 103, 210, 169, 253, 36, 115, 188, 152, 125, 122, 58, 158, 151, 47, 20, 191, 106, 198, 140, 77, 147, 27, 199, 96, 53, 88, 163, 215, 13, 59, 214, 229, 117, 161, 220, 78, 204, 58, 162, 210, 111, 85, 37, 168, 191, 233, 169, 93, 88, 161, 133, 1, 123, 60, 54, 140, 214, 104, 179, 198, 247, 81, 68, 238, 100, 226, 230, 126, 42, 11, 191, 234, 199, 76, 246, 198, 197, 70, 168, 57, 89, 73, 79, 54, 101, 63, 186, 118, 69, 167, 19, 62, 52, 139, 78, 100, 89, 101, 39, 108, 191, 108, 233, 7, 176, 68, 32, 158, 34, 236, 14, 176, 51, 122, 185, 97, 176, 80, 219, 121, 215, 102, 4, 126, 217, 69, 184, 61, 80, 36, 31, 100, 105, 9, 212, 119, 214, 160, 108, 247, 30, 187, 83, 46, 39, 226, 114, 225, 133, 176, 85, 21, 13, 200, 214, 234, 181, 153, 231, 157, 25, 144, 126, 135, 166, 110, 252, 228, 65, 105, 49, 197, 199, 196, 11, 50, 14, 195, 235, 226, 83, 139, 86, 52, 116, 230, 218, 55, 154, 28, 64, 127, 8, 133, 17, 10, 174, 34, 175, 161, 177, 136, 42, 115, 27, 240, 71, 119, 249, 199, 167, 10, 1, 170, 74, 85, 30, 206, 0, 63, 77, 102, 36, 254, 202, 100, 153, 231, 194, 92, 25, 221, 106, 108, 218, 218, 127, 92, 171, 160, 218, 53, 11, 129, 27, 65, 69, 198, 192, 179, 119, 161, 162, 206, 61, 21, 114, 125, 192, 131, 83, 252, 196, 159, 25, 37, 83, 75, 102, 59, 201, 187, 15, 158, 92, 189, 37, 215, 75, 37, 44, 219, 109, 126, 209, 49, 243, 109, 199, 245, 178, 176, 102, 251, 202, 210, 32, 219, 241, 231, 120, 51, 244, 146, 155, 155, 71, 96, 27, 218, 105, 59, 164, 224, 187, 246, 80, 169, 143, 119, 212, 102, 225, 180, 111, 56, 212, 227, 226, 14, 226, 96, 200, 134, 47, 34, 212, 33, 56, 4, 111, 126, 120, 69, 151, 51, 174, 222, 218, 215, 247, 100, 199, 119, 101, 117, 223, 124, 225, 15, 27, 43, 210, 168, 107, 40, 178, 59, 120, 165, 169, 211, 211, 104, 204, 237, 101, 84, 110, 150, 92, 214, 187, 208, 139, 150, 126, 173, 160, 107, 204, 7, 163, 15, 28, 31, 122, 44, 216, 83, 70, 235, 7, 127, 106, 120, 65, 244, 5, 15, 246, 117, 213, 132, 236, 240, 247, 226, 173, 122, 127, 191, 186, 122, 134, 94, 163, 162, 19, 156, 67, 238, 211, 9, 215, 144, 136, 92, 80, 64, 231, 236, 80, 64, 18, 124, 81, 116, 175, 104, 163, 165, 215, 2, 124, 59, 128, 113, 229, 225, 101, 28, 78, 41, 64, 90, 52, 56, 207, 234, 131, 252, 234, 105, 26, 94, 226, 60, 22, 102, 10, 92, 117, 80, 25, 98, 211, 33, 254, 153, 197, 73, 254, 14, 143, 248, 165, 122, 132, 226, 131, 214, 22, 2, 0, 93, 220, 32, 100, 229, 23, 50, 1, 63, 214, 147, 41, 252, 166, 31, 197, 244, 127, 13, 152, 114, 235, 77, 119, 61, 184, 253, 10, 54, 78, 32, 138, 229, 224, 65, 104, 243, 87, 139, 83, 83, 251, 17, 79, 215, 25, 218, 154, 215, 232, 133, 126, 110, 82, 92, 42, 114, 228, 159, 249, 109, 113, 149, 35, 232, 3, 233, 110, 188, 42, 77, 247, 211, 219, 123, 84, 2, 183, 239, 121, 43, 99, 200, 50, 220, 231, 60, 9, 51, 166, 14, 74, 57, 69, 31, 21, 253, 237, 25, 180, 70, 64, 42, 75, 14, 144, 225, 242, 162, 6, 19, 180, 148, 173, 142, 107, 67, 17, 141, 234, 66, 79, 14, 109, 211, 104, 85, 195, 206, 107, 200, 60, 158, 133, 3, 62, 208, 235, 91, 110, 144, 249, 179, 137, 8, 12, 165, 15, 156, 108, 187, 101, 136, 13, 245, 220, 166, 228, 166, 5, 123, 159, 93, 5, 222, 179, 3, 146, 96, 248, 54, 72, 163, 139, 24, 13, 12, 166, 73, 44, 181, 170, 160, 160, 16, 136, 26, 141, 114, 237, 139, 248, 250, 25, 121, 108, 225, 130, 133, 251, 23, 223, 142, 163, 96, 52, 119, 138, 233, 243, 74, 99, 23, 125, 19, 225, 49, 182, 161, 157, 137, 224, 35, 85, 84, 207, 208, 226, 38, 81, 114, 7, 38, 188, 162, 26, 239, 132, 35, 145, 14, 164, 13, 180, 159, 15, 24, 200, 77, 152, 171, 212, 141, 130, 149, 200, 149, 12, 197, 68, 123, 194, 194, 108, 158, 82, 112, 197, 122, 220, 16, 126, 87, 32, 118, 124, 51, 19, 25, 224, 37, 99, 67, 102, 200, 101, 19, 135, 31, 12, 185, 208, 212, 88, 199, 135, 252, 62, 154, 11, 158, 92, 99, 9, 166, 195, 236, 53, 117, 133, 92, 103, 114, 157, 193, 12, 148, 26, 220, 117, 198, 251, 66, 141, 137, 176, 4, 94, 50, 25, 172, 121, 134, 165, 4, 39, 55, 109, 148, 27, 141, 134, 231, 148, 134, 205, 124, 46, 106, 3, 22, 197, 117, 53, 170, 98, 79, 189, 42, 241, 243, 220, 11, 91, 110, 122, 4, 22, 72, 76, 252, 97, 151, 176, 104, 112, 24, 124, 110, 200, 98, 239, 163, 15, 13, 59, 192, 48, 38, 231, 58, 90, 104, 18, 34, 217, 168, 106, 217, 136, 99, 9, 63, 124, 111, 201, 29, 223, 28, 163, 195, 234, 197, 60, 14, 202, 124, 24, 195, 186, 40, 121, 60, 182, 185, 59, 110, 236, 238, 221, 147, 181, 139, 83, 63, 62, 226, 28, 118, 211, 156, 87, 125, 11, 133, 101, 188, 224, 184, 13, 169, 46, 15, 12, 170, 149, 88, 174, 131, 162, 188, 225, 229, 249, 91, 245, 141, 194, 216, 62, 123, 212, 190, 80, 193, 9, 46, 48, 197, 251, 176, 252, 18, 247, 141, 44, 61, 151, 183, 44, 109, 83, 5, 57, 56, 30, 252, 44, 145, 244, 24, 168, 46, 52, 24, 250, 133, 246, 21, 25, 151, 128, 235, 49, 30, 127, 210, 73, 52, 93, 164, 108, 43, 65, 176, 156, 6, 143, 77, 189, 161, 135, 154, 39, 164, 39, 1, 216, 18, 5, 66, 149, 196, 200, 154, 176, 192, 118, 241, 104, 132, 167, 85, 158, 138, 148, 141, 128, 243, 208, 27, 189, 222, 37, 79, 58, 188, 193, 65, 248, 33, 135, 126, 174, 12, 195, 152, 227, 145, 97, 135, 206, 76, 236, 144, 45, 58, 62, 158, 30, 116, 218, 120, 224, 142, 80, 121, 199, 209, 46, 129, 101, 60, 102, 9, 227, 104, 134, 192, 221, 250, 67, 176, 23, 199, 180, 120, 93, 158, 104, 254, 222, 96, 9, 154, 189, 158, 233, 34, 249, 67, 95, 4, 39, 66, 201, 149, 220, 65, 96, 85, 92, 8, 26, 151, 138, 153, 51, 132, 25, 123, 4, 219, 246, 125, 12, 246, 167, 59, 60, 115, 101, 214, 60, 121, 155, 176, 60, 172, 186, 46, 97, 122, 67, 63, 165, 71, 184, 139, 87, 55, 50, 246, 165, 7, 99, 218, 116, 38, 35, 138, 250, 134, 238, 34, 110, 143, 140, 20, 199, 105, 250, 90, 170, 62, 84, 89, 124, 65, 65, 39, 186, 196, 8, 118, 216, 199, 128, 137, 44, 170, 190, 209, 21, 187, 231, 143, 212, 43, 203, 74, 31, 159, 71, 174, 237, 211, 89, 106, 62, 203, 74, 42, 102, 241, 162, 71, 11, 120, 69, 225, 43, 44, 189, 132, 97, 43, 183, 91, 234, 184, 78, 163, 96, 248, 36, 147, 144, 30, 196, 44, 76, 201, 77, 28, 61, 151, 117, 191, 232, 166, 233, 9, 237, 10, 3, 18, 177, 71, 252, 184, 252, 113, 154, 134, 215, 48, 241, 232, 111, 93, 88, 182, 48, 191, 81, 92, 240, 161, 239, 226, 116, 68, 75, 132, 137, 245, 158, 30, 177, 215, 19, 213, 211, 204, 48, 156, 234, 57, 180, 26, 207, 171, 137, 165, 194, 168, 112, 36, 11, 105, 117, 196, 123, 194, 162, 28, 253, 150, 204, 28, 220, 126, 240, 68, 19, 153, 133, 92, 93, 47, 122, 71, 66, 40, 243, 95, 249, 227, 51, 160, 158, 181, 77, 177, 117, 183, 131, 122, 115, 45, 169, 140, 93, 99, 5, 139, 106, 42, 65, 128, 136, 24, 66, 70, 226, 64, 136, 75, 252, 220, 67, 16, 46, 75, 69, 183, 237, 117, 200, 59, 87, 65, 97, 122, 205, 59, 187, 171, 232, 213, 198, 203, 204, 47, 36, 153, 100, 107, 46, 199, 36, 41, 191, 65, 244, 30, 144, 87, 112, 62, 180, 49, 189, 94, 15, 155, 167, 164, 37, 135, 109, 12, 32, 115, 234, 241, 38, 81, 235, 52, 7, 164, 173, 207, 237, 11, 207, 14, 71, 188, 110, 84, 105, 213, 106, 120, 163, 221, 156, 20, 141, 188, 239, 124, 144, 171, 134, 79, 110, 235, 37, 5, 86, 166, 75, 12, 214, 172, 75, 160, 62, 5, 153, 111, 55, 143, 249, 178, 73, 92, 170, 92, 132, 176, 234, 87, 119, 18, 114, 53, 116, 242, 184, 225, 33, 91, 170, 226, 225, 235, 91, 241, 3, 178, 9, 200, 183, 38, 181, 36, 68, 185, 94, 4, 202, 47, 96, 23, 190, 132, 143, 180, 212, 190, 121, 214, 84, 220, 5, 155, 132, 209, 244, 96, 106, 108, 27, 228, 153, 146, 213, 28, 79, 46, 1, 113, 44, 124, 223, 245, 163, 156, 2, 120, 75, 35, 190, 215, 172, 126, 236, 90, 67, 244, 218, 90, 27, 30, 201, 138, 23, 246, 207, 25, 69, 250, 165, 203, 71, 221, 81, 111, 123, 237, 65, 205, 45, 8, 51, 53, 46, 138, 109, 175, 135, 107, 167, 91, 190, 98, 9, 127, 101, 237, 187, 27, 185, 12, 221, 206, 174, 96, 209, 129, 173, 198, 164, 53, 143, 62, 169, 112, 159, 106, 153, 83, 155, 89, 255, 30, 102, 183, 179, 19, 61, 220, 229, 203, 223, 14, 110, 237, 189, 94, 205, 40, 223, 104, 46, 181, 185, 206, 39, 62, 80, 159, 248, 190, 30, 173, 242, 234, 176, 15, 115, 195, 160, 232, 98, 75, 224, 194, 247, 42, 230, 163, 35, 183, 174, 54, 250, 147, 177, 169, 190, 62, 114, 119, 28, 95, 188, 157, 158, 241, 13, 242, 17, 27, 25, 252, 65, 12, 173, 231, 161, 90, 176, 174, 111, 33, 45, 244, 96, 74, 247, 26, 101, 208, 223, 136, 43, 68, 58, 84, 158, 134, 106, 73, 175, 164, 214, 73, 50, 179, 234, 96, 138, 85, 67, 218, 168, 33, 247, 73, 146, 231, 201, 164, 172, 23, 34, 23, 170, 175, 245, 12, 211, 149, 82, 102, 190, 15, 186, 237, 245, 134, 127, 143, 88, 216, 23, 9, 166, 145, 134, 24, 173, 91, 24, 41, 75, 129, 57, 107, 248, 69, 171, 150, 65, 245, 150, 65, 165, 134, 11, 232, 185, 188, 129, 165, 67, 18, 154, 78, 75, 39, 86, 203, 164, 132, 119, 43, 255, 58, 166, 169, 92, 105, 171, 237, 251, 140, 74, 125, 125, 154, 11, 11, 83, 191, 16, 3, 11, 237, 185, 66, 245, 128, 207, 63, 42, 69, 83, 51, 240, 202, 236, 240, 170, 200, 182, 44, 190, 162, 200, 149, 207, 88, 169, 231, 224, 234, 34, 175, 144, 155, 163, 231, 238, 57, 16, 105, 197, 55, 26, 41, 127, 208, 40, 175, 9, 148, 59, 9, 141, 107, 27, 100, 93, 27, 205, 85, 161, 125, 147, 25, 208, 216, 124, 137, 25, 124, 202, 206, 162, 233, 235, 16, 239, 98, 237, 20, 211, 58, 185, 96, 39, 73, 93, 99, 159, 102, 112, 173, 229, 227, 13, 83, 51, 95, 244, 68, 12, 140, 94, 150, 203, 229, 122, 163, 252, 253, 94, 33, 112, 222, 71, 31, 124, 8, 226, 77, 216, 199, 113, 116, 134, 236, 84, 75, 177, 83, 181, 178, 98, 104, 205, 21, 151, 162, 209, 95, 221, 140, 30, 110, 46, 0, 92, 98, 113, 149, 213, 156, 37, 91, 154, 33, 76, 27, 182, 86, 208, 195, 126, 25, 130, 84, 123, 211, 140, 115, 124, 155, 219, 119, 84, 236, 138, 18, 53, 246, 170, 24, 49, 65, 187, 122, 157, 60, 30, 244, 21, 14, 134, 78, 174, 137, 56, 120, 138, 186, 119, 29, 196, 171, 166, 198, 29, 238, 24, 94, 53, 53, 230, 172, 28, 54, 45, 122, 124, 105, 143, 201, 84, 28, 13, 175, 26, 191, 116, 199, 13, 6, 82, 200, 196, 130, 145, 134, 87, 139, 89, 105, 192, 232, 65, 129, 101, 120, 41, 79, 102, 75, 49, 146, 65, 75, 232, 92, 175, 140, 122, 20, 165, 30, 69, 201, 124, 26, 229, 196, 253, 190, 227, 76, 204, 148, 210, 221, 183, 224, 90, 16, 248, 234, 251, 21, 167, 208, 194, 110, 155, 8, 56, 203, 14, 45, 52, 77, 83, 54, 245, 42, 79, 34, 174, 185, 123, 152, 25, 33, 160, 88, 13, 156, 100, 92, 16, 172, 115, 75, 41, 175, 220, 100, 142, 142, 155, 14, 136, 121, 82, 249, 32, 26, 25, 74, 190, 89, 137, 92, 212, 234, 169, 252, 30, 105, 159, 111, 138, 154, 70, 52, 23, 220, 230, 246, 213, 78, 78, 207, 179, 151, 68, 61, 140, 136, 69, 169, 171, 103, 73, 202, 29, 253, 197, 137, 205, 87, 152, 84, 98, 49, 1, 200, 111, 69, 112, 39, 17, 76, 222, 16, 130, 245, 58, 15, 147, 43, 84, 84, 110, 151, 43, 84, 84, 9, 219, 89, 209, 138, 173, 171, 148, 20, 68, 138, 18, 57, 33, 67, 8, 80, 25, 17, 80, 0, 246, 250, 133, 81, 129, 108, 19, 223, 246, 122, 195, 53, 230, 151, 241, 230, 110, 192, 125, 64, 94, 19, 130, 234, 48, 137, 26, 43, 82, 100, 131, 70, 74, 111, 145, 248, 53, 2, 138, 139, 219, 192, 108, 184, 235, 188, 141, 196, 91, 19, 118, 2, 69, 148, 89, 222, 216, 221, 187, 113, 2, 106, 126, 51, 163, 219, 154, 248, 148, 99, 133, 1, 22, 114, 201, 101, 67, 90, 63, 242, 54, 191, 140, 95, 184, 121, 223, 88, 248, 72, 143, 123, 195, 171, 172, 74, 88, 19, 76, 147, 179, 86, 21, 95, 125, 239, 21, 213, 134, 154, 98, 178, 18, 98, 220, 42, 56, 105, 85, 227, 163, 239, 117, 46, 50, 201, 129, 215, 5, 168, 19, 141, 27, 103, 169, 67, 197, 67, 118, 145, 94, 213, 146, 222, 22, 206, 170, 7, 37, 111, 75, 30, 129, 9, 22, 44, 130, 142, 20, 31, 67, 243, 169, 102, 119, 42, 18, 219, 121, 102, 121, 218, 187, 204, 62, 70, 66, 243, 25, 93, 55, 106, 250, 81, 10, 47, 108, 101, 74, 230, 176, 28, 45, 202, 79, 67, 216, 242, 189, 95, 111, 174, 127, 104, 44, 158, 24, 90, 123, 218, 188, 98, 27, 236, 1, 59, 173, 221, 129, 213, 173, 129, 24, 91, 74, 71, 201, 204, 147, 37, 53, 253, 99, 169, 65, 40, 233, 243, 7, 199, 123, 77, 235, 30, 205, 52, 255, 24, 81, 150, 117, 137, 163, 196, 113, 20, 153, 88, 136, 200, 186, 11, 193, 118, 28, 245, 24, 28, 252, 4, 247, 8, 178, 133, 36, 47, 10, 132, 233, 128, 232, 56, 190, 110, 6, 176, 119, 238, 8, 5, 246, 245, 33, 221, 224, 46, 199, 169, 12, 158, 109, 73, 113, 194, 184, 148, 73, 208, 110, 123, 163, 10, 98, 189, 81, 234, 200, 42, 202, 165, 140, 124, 235, 235, 94, 37, 129, 171, 7, 109, 181, 246, 241, 31, 59, 118, 126, 177, 130, 137, 95, 158, 243, 28, 124, 86, 1, 197, 67, 83, 234, 28, 85, 103, 156, 44, 68, 180, 200, 120, 10, 0, 143, 168, 66, 157, 215, 67, 71, 101, 231, 52, 70, 128, 196, 211, 24, 211, 76, 47, 16, 20, 134, 250, 146, 171, 157, 184, 104, 156, 242, 192, 225, 254, 140, 167, 252, 4, 224, 112, 58, 66, 127, 188, 107, 167, 255, 194, 238, 47, 151, 20, 97, 241, 183, 150, 89, 145, 108, 219, 209, 69, 29, 124, 100, 216, 126, 228, 235, 110, 203, 144, 47, 208, 158, 30, 177, 90, 143, 75, 46, 214, 20, 75, 222, 21, 1, 171, 131, 135, 170, 223, 246, 172, 12, 116, 138, 96, 217, 29, 79, 182, 140, 89, 56, 195, 48, 121, 226, 168, 163, 47, 126, 181, 241, 217, 7, 113, 100, 33, 82, 220, 105, 105, 158, 90, 154, 156, 123, 91, 178, 54, 136, 177, 197, 246, 203, 89, 107, 28, 78, 97, 143, 250, 28, 165, 193, 33, 110, 70, 66, 74, 174, 179, 11, 201, 151, 20, 198, 130, 95, 197, 122, 195, 178, 36, 134, 130, 252, 76, 176, 248, 226, 97, 200, 158, 77, 221, 99, 118, 165, 78, 211, 131, 98, 156, 111, 179, 247, 10, 176, 125, 7, 143, 103, 148, 221, 199, 87, 193, 22, 77, 108, 236, 155, 103, 34, 206, 34, 127, 238, 129, 216, 81, 36, 160, 30, 82, 22, 131, 81, 94, 226, 164, 67, 204, 221, 128, 93, 180, 197, 203, 208, 60, 138, 207, 215, 137, 193, 68, 121, 127, 228, 224, 243, 100, 62, 24, 51, 10, 12, 95, 124, 21, 39, 101, 69, 26, 198, 222, 151, 53, 251, 132, 24, 255, 240, 67, 255, 233, 222, 208, 127, 210, 161, 255, 100, 67, 191, 34, 103, 103, 142, 69, 139, 186, 15, 208, 70, 14, 93, 174, 85, 177, 159, 100, 49, 216, 193, 153, 227, 115, 69, 79, 105, 208, 192, 232, 170, 29, 12, 215, 21, 189, 142, 225, 228, 252, 160, 18, 149, 169, 17, 10, 95, 59, 96, 112, 119, 71, 25, 123, 158, 12, 3, 8, 87, 234, 220, 152, 196, 138, 49, 69, 40, 21, 80, 48, 97, 21, 166, 165, 146, 159, 202, 98, 138, 231, 137, 79, 117, 253, 206, 2, 192, 67, 247, 153, 32, 134, 50, 2, 161, 23, 136, 86, 16, 195, 90, 102, 90, 156, 127, 187, 252, 56, 26, 130, 246, 146, 196, 121, 52, 171, 91, 231, 133, 124, 58, 214, 61, 167, 136, 229, 59, 95, 225, 197, 33, 168, 164, 159, 121, 24, 73, 164, 112, 187, 145, 50, 236, 154, 252, 20, 230, 7, 96, 6, 116, 212, 172, 95, 21, 96, 180, 145, 109, 88, 206, 101, 246, 48, 55, 72, 145, 232, 226, 134, 206, 116, 187, 144, 129, 56, 105, 85, 164, 139, 9, 11, 214, 71, 75, 66, 136, 242, 255, 63, 3, 88, 226, 53, 44, 134, 145, 190, 96, 54, 189, 56, 56, 121, 115, 184, 255, 17, 254, 60, 54, 3, 31, 160, 205, 165, 95, 171, 105, 79, 192, 116, 213, 9, 61, 47, 8, 83, 255, 199, 217, 140, 165, 251, 97, 134, 193, 165, 252, 239, 48, 233, 65, 125, 201, 189, 133, 15, 13, 122, 210, 144, 98, 160, 82, 212, 26, 41, 75, 136, 147, 125, 61, 224, 67, 223, 88, 152, 204, 54, 239, 50, 188, 58, 102, 158, 199, 62, 238, 50, 200, 21, 160, 42, 134, 218, 95, 203, 55, 174, 166, 21, 18, 71, 173, 240, 156, 240, 13, 131, 247, 128, 253, 132, 135, 216, 148, 33, 101, 36, 133, 23, 57, 183, 169, 114, 116, 179, 252, 211, 119, 55, 42, 65, 157, 12, 18, 66, 134, 103, 78, 3, 175, 228, 80, 50, 114, 16, 41, 57, 181, 219, 79, 109, 192, 116, 82, 47, 125, 70, 33, 23, 143, 145, 227, 221, 64, 203, 49, 88, 99, 24, 201, 186, 228, 176, 228, 138, 163, 166, 35, 220, 154, 222, 192, 0, 143, 170, 28, 121, 154, 254, 251, 139, 90, 211, 197, 161, 125, 179, 112, 85, 247, 97, 227, 187, 60, 228, 42, 13, 201, 20, 56, 105, 202, 200, 97, 135, 194, 106, 112, 157, 3, 45, 89, 211, 252, 0, 131, 172, 78, 243, 34, 0, 98, 99, 177, 50, 146, 141, 147, 75, 41, 54, 110, 2, 97, 54, 86, 108, 208, 84, 180, 46, 155, 112, 6, 70, 242, 108, 254, 143, 125, 169, 30, 200, 148, 159, 250, 74, 19, 168, 138, 192, 28, 230, 57, 168, 251, 2, 159, 231, 164, 140, 166, 133, 19, 211, 23, 169, 160, 154, 95, 90, 169, 74, 9, 83, 80, 52, 138, 180, 101, 60, 152, 252, 87, 213, 137, 221, 59, 204, 161, 209, 102, 99, 65, 100, 241, 114, 101, 247, 215, 178, 100, 21, 239, 61, 52, 126, 177, 229, 75, 95, 60, 248, 4, 112, 31, 65, 158, 36, 243, 140, 94, 103, 165, 139, 114, 138, 200, 141, 37, 170, 146, 54, 236, 86, 197, 83, 226, 89, 152, 101, 209, 5, 227, 155, 117, 243, 88, 181, 18, 17, 26, 103, 116, 90, 211, 198, 123, 105, 84, 240, 229, 143, 146, 170, 106, 10, 133, 40, 144, 76, 111, 78, 238, 201, 233, 206, 1, 76, 222, 55, 28, 52, 209, 14, 139, 142, 25, 7, 105, 138, 49, 212, 95, 68, 208, 201, 233, 89, 240, 244, 213, 139, 128, 113, 153, 210, 231, 123, 182, 90, 195, 103, 161, 213, 76, 228, 60, 16, 251, 187, 52, 196, 53, 70, 238, 244, 246, 139, 28, 88, 25, 236, 36, 216, 100, 36, 25, 72, 148, 122, 141, 164, 79, 173, 225, 13, 72, 230, 66, 111, 120, 90, 92, 248, 234, 60, 218, 44, 146, 115, 121, 169, 220, 241, 109, 186, 0, 198, 198, 128, 245, 106, 166, 101, 210, 141, 110, 130, 19, 147, 167, 61, 15, 69, 112, 41, 199, 155, 91, 213, 111, 139, 75, 39, 89, 157, 191, 157, 45, 230, 3, 58, 68, 169, 34, 210, 215, 206, 152, 237, 69, 105, 12, 163, 37, 203, 194, 190, 204, 158, 232, 90, 73, 78, 129, 70, 224, 36, 181, 101, 60, 46, 45, 103, 161, 198, 167, 171, 120, 90, 189, 114, 61, 207, 247, 210, 223, 53, 197, 183, 234, 173, 131, 174, 190, 217, 145, 255, 116, 141, 160, 223, 244, 12, 27, 143, 109, 244, 254, 131, 19, 25, 62, 205, 223, 112, 247, 61, 61, 184, 172, 157, 190, 93, 146, 46, 2, 205, 26, 135, 118, 188, 165, 240, 50, 4, 45, 3, 245, 40, 30, 38, 93, 198, 23, 210, 58, 218, 212, 192, 53, 42, 47, 73, 122, 31, 52, 92, 2, 146, 227, 203, 140, 97, 158, 233, 209, 5, 249, 19, 118, 45, 34, 0, 162, 206, 62, 59, 191, 241, 159, 73, 209, 95, 85, 229, 41, 193, 48, 241, 144, 112, 75, 92, 120, 113, 8, 184, 247, 173, 44, 168, 116, 247, 79, 191, 103, 233, 207, 127, 201, 217, 225, 211, 62, 40, 106, 50, 251, 246, 147, 80, 7, 10, 255, 83, 34, 134, 235, 80, 203, 15, 34, 96, 97, 179, 151, 32, 152, 10, 86, 82, 91, 123, 37, 200, 152, 53, 143, 156, 245, 203, 255, 46, 116, 74, 61, 177, 252, 216, 11, 97, 71, 62, 98, 77, 66, 84, 31, 25, 233, 204, 126, 35, 136, 209, 12, 212, 249, 113, 63, 80, 154, 169, 254, 182, 165, 171, 197, 146, 179, 15, 119, 223, 233, 111, 146, 107, 143, 248, 64, 71, 30, 238, 238, 210, 223, 144, 222, 65, 125, 83, 205, 212, 231, 216, 2, 93, 83, 234, 133, 125, 191, 214, 73, 84, 106, 170, 192, 102, 253, 192, 59, 143, 30, 241, 233, 249, 253, 90, 39, 232, 251, 103, 148, 44, 177, 237, 43, 65, 115, 75, 21, 129, 47, 140, 79, 201, 125, 81, 245, 213, 144, 100, 236, 55, 223, 192, 144, 88, 167, 83, 244, 130, 109, 138, 230, 39, 97, 221, 115, 196, 159, 176, 11, 74, 77, 133, 44, 165, 69, 5, 57, 92, 156, 89, 250, 129, 193, 231, 36, 157, 244, 65, 139, 181, 97, 84, 27, 33, 173, 74, 249, 166, 212, 55, 223, 65, 168, 230, 204, 69, 178, 44, 102, 42, 230, 17, 23, 30, 177, 51, 138, 127, 204, 85, 25, 254, 101, 231, 54, 145, 100, 143, 130, 194, 29, 222, 190, 37, 102, 47, 155, 18, 156, 150, 228, 148, 17, 179, 212, 94, 166, 240, 209, 230, 187, 117, 44, 88, 66, 133, 44, 213, 224, 140, 171, 10, 158, 43, 10, 214, 20, 181, 143, 213, 236, 27, 10, 69, 113, 117, 77, 65, 75, 114, 238, 42, 112, 24, 11, 92, 108, 69, 33, 238, 96, 11, 60, 174, 251, 213, 214, 124, 176, 248, 163, 115, 127, 96, 232, 152, 130, 183, 73, 167, 176, 36, 194, 204, 239, 110, 45, 245, 238, 6, 202, 40, 218, 80, 148, 191, 200, 155, 95, 61, 23, 162, 147, 40, 207, 123, 87, 104, 208, 214, 136, 170, 226, 66, 161, 123, 206, 207, 243, 196, 110, 71, 87, 238, 62, 121, 149, 59, 46, 166, 133, 193, 27, 125, 253, 212, 235, 63, 159, 170, 52, 62, 105, 32, 183, 90, 244, 90, 224, 149, 214, 36, 122, 140, 152, 142, 121, 218, 225, 148, 251, 230, 171, 199, 128, 49, 110, 155, 253, 182, 137, 255, 82, 11, 53, 107, 143, 182, 192, 229, 30, 23, 91, 22, 219, 197, 170, 95, 49, 70, 196, 119, 22, 235, 85, 147, 187, 171, 84, 85, 74, 194, 228, 30, 250, 193, 100, 145, 106, 48, 214, 217, 207, 183, 135, 195, 0, 71, 203, 174, 194, 214, 128, 201, 85, 24, 233, 165, 214, 95, 215, 41, 201, 81, 74, 252, 58, 135, 225, 178, 164, 173, 195, 122, 178, 190, 34, 235, 110, 81, 60, 113, 77, 79, 147, 110, 209, 86, 81, 117, 172, 220, 239, 26, 110, 95, 126, 181, 64, 47, 226, 213, 39, 244, 222, 250, 212, 130, 165, 172, 79, 198, 163, 90, 85, 186, 129, 86, 176, 80, 15, 228, 21, 17, 61, 87, 187, 91, 209, 15, 252, 209, 88, 157, 23, 170, 112, 68, 43, 110, 55, 126, 185, 42, 240, 254, 19, 181, 209, 42, 132, 210, 135, 69, 138, 193, 178, 186, 192, 228, 171, 168, 1, 30, 4, 75, 214, 206, 123, 154, 65, 110, 171, 23, 240, 191, 147, 116, 116, 22, 180, 66, 47, 17, 43, 77, 137, 253, 78, 95, 225, 185, 77, 170, 41, 76, 93, 134, 250, 198, 147, 164, 177, 84, 90, 48, 20, 2, 175, 232, 77, 192, 54, 134, 240, 198, 171, 221, 198, 186, 88, 184, 10, 212, 223, 75, 14, 144, 125, 253, 96, 25, 213, 8, 107, 185, 96, 2, 11, 58, 235, 101, 73, 47, 204, 98, 205, 192, 25, 116, 217, 131, 137, 217, 31, 18, 114, 252, 183, 179, 240, 23, 134, 28, 245, 171, 120, 46, 108, 127, 49, 105, 157, 130, 77, 101, 179, 83, 109, 106, 47, 134, 89, 105, 100, 26, 215, 159, 13, 43, 92, 130, 113, 123, 78, 253, 123, 67, 188, 245, 38, 28, 89, 220, 42, 30, 41, 114, 77, 86, 156, 25, 85, 243, 38, 221, 77, 144, 13, 100, 181, 1, 139, 31, 79, 65, 104, 162, 102, 244, 44, 13, 39, 204, 46, 84, 48, 160, 131, 80, 202, 254, 109, 206, 178, 220, 170, 239, 90, 81, 113, 218, 189, 74, 161, 155, 138, 77, 200, 158, 115, 35, 226, 47, 152, 79, 35, 226, 155, 235, 197, 56, 129, 254, 99, 186, 226, 85, 82, 197, 120, 96, 176, 160, 166, 161, 249, 61, 45, 198, 204, 8, 80, 174, 93, 178, 102, 248, 40, 185, 101, 183, 19, 67, 157, 95, 145, 234, 84, 184, 53, 124, 227, 97, 150, 170, 251, 155, 127, 127, 85, 98, 41, 123, 80, 113, 83, 135, 14, 44, 176, 124, 113, 126, 66, 1, 243, 96, 123, 116, 163, 45, 240, 27, 250, 70, 123, 75, 223, 247, 154, 23, 30, 65, 134, 247, 117, 71, 35, 227, 113, 35, 11, 166, 179, 121, 119, 87, 203, 114, 112, 98, 123, 189, 36, 60, 78, 148, 239, 3, 216, 135, 151, 66, 220, 190, 23, 196, 237, 10, 136, 114, 251, 126, 71, 144, 184, 207, 55, 97, 6, 203, 130, 80, 198, 1, 79, 168, 22, 159, 117, 72, 156, 46, 152, 246, 33, 158, 168, 91, 136, 196, 79, 127, 49, 217, 93, 30, 209, 246, 145, 231, 240, 98, 121, 11, 146, 71, 194, 170, 153, 232, 15, 139, 176, 140, 38, 203, 149, 145, 37, 173, 107, 150, 142, 107, 233, 147, 143, 28, 133, 210, 212, 73, 173, 88, 8, 255, 56, 205, 179, 96, 23, 109, 154, 23, 247, 226, 252, 250, 186, 81, 84, 166, 122, 84, 248, 13, 191, 142, 171, 213, 150, 137, 191, 122, 117, 183, 56, 96, 251, 197, 84, 92, 206, 176, 247, 176, 119, 113, 212, 74, 148, 219, 210, 131, 193, 165, 180, 90, 94, 219, 107, 43, 211, 179, 132, 161, 236, 126, 51, 10, 102, 146, 207, 182, 38, 38, 116, 181, 113, 205, 41, 228, 90, 215, 228, 2, 123, 58, 63, 123, 146, 92, 105, 207, 148, 34, 9, 159, 98, 178, 253, 88, 41, 111, 253, 226, 133, 21, 122, 219, 9, 120, 96, 71, 255, 159, 149, 199, 35, 242, 216, 222, 120, 32, 32, 51, 176, 204, 35, 95, 96, 32, 195, 83, 195, 20, 28, 61, 143, 59, 134, 25, 63, 200, 15, 223, 136, 39, 116, 103, 248, 146, 150, 214, 243, 163, 159, 164, 9, 224, 187, 27, 163, 213, 219, 96, 21, 146, 140, 88, 23, 183, 193, 223, 254, 253, 191, 18, 5, 64, 81, 143, 166, 226, 19, 16, 134, 207, 240, 202, 12, 101, 172, 90, 115, 142, 17, 79, 81, 180, 213, 22, 76, 151, 178, 202, 211, 196, 120, 102, 213, 141, 214, 125, 94, 225, 17, 113, 113, 86, 15, 211, 212, 181, 16, 242, 193, 167, 104, 26, 144, 127, 87, 206, 32, 141, 82, 130, 176, 195, 126, 188, 12, 95, 58, 126, 195, 170, 172, 188, 132, 143, 209, 58, 118, 247, 194, 31, 78, 155, 20, 161, 217, 130, 181, 232, 124, 218, 61, 130, 251, 186, 79, 245, 156, 163, 3, 131, 246, 136, 57, 236, 210, 10, 16, 250, 160, 235, 170, 53, 236, 11, 216, 123, 168, 9, 27, 70, 59, 197, 115, 74, 105, 199, 221, 144, 167, 148, 118, 250, 118, 73, 186, 231, 148, 146, 144, 193, 167, 114, 230, 195, 99, 253, 249, 116, 218, 165, 22, 169, 3, 245, 18, 174, 153, 206, 159, 78, 195, 72, 127, 182, 91, 186, 167, 59, 37, 22, 76, 138, 81, 94, 246, 202, 200, 140, 226, 206, 251, 227, 137, 219, 77, 248, 111, 86, 20, 191, 240, 125, 177, 27, 235, 110, 218, 4, 15, 241, 135, 22, 173, 127, 49, 106, 107, 219, 19, 223, 243, 72, 66, 157, 167, 115, 178, 186, 183, 153, 71, 1, 29, 145, 1, 104, 111, 128, 118, 25, 128, 198, 251, 138, 167, 247, 30, 130, 126, 72, 61, 98, 249, 96, 252, 47, 89, 50, 173, 127, 90, 13, 103, 209, 42, 141, 243, 234, 16, 104, 124, 253, 40, 99, 211, 44, 73, 119, 21, 119, 255, 14, 159, 226, 132, 79, 126, 93, 13, 63, 110, 127, 151, 39, 42, 33, 79, 208, 140, 111, 52, 168, 40, 93, 231, 15, 245, 44, 14, 186, 102, 249, 247, 202, 119, 240, 102, 239, 59, 31, 26, 59, 222, 114, 32, 153, 95, 157, 254, 25, 203, 190, 239, 114, 91, 247, 109, 105, 65, 188, 51, 173, 94, 176, 131, 90, 109, 144, 123, 166, 191, 225, 35, 61, 199, 125, 64, 209, 142, 190, 150, 91, 79, 240, 25, 207, 232, 201, 167, 196, 156, 123, 20, 230, 125, 32, 87, 198, 47, 53, 82, 66, 155, 126, 196, 7, 71, 27, 37, 26, 141, 93, 83, 18, 125, 197, 113, 17, 177, 90, 136, 218, 229, 151, 97, 32, 151, 28, 52, 41, 183, 207, 199, 241, 25, 136, 227, 156, 103, 237, 220, 115, 200, 239, 70, 123, 237, 185, 194, 197, 143, 21, 202, 167, 10, 3, 95, 200, 62, 123, 208, 204, 88, 108, 226, 109, 168, 17, 252, 225, 4, 197, 125, 201, 71, 241, 242, 131, 123, 185, 199, 27, 167, 223, 39, 77, 42, 31, 120, 240, 190, 92, 197, 7, 214, 138, 90, 171, 112, 180, 187, 128, 198, 89, 58, 175, 203, 234, 181, 199, 175, 15, 3, 157, 175, 106, 165, 151, 214, 60, 82, 158, 55, 171, 235, 45, 131, 16, 184, 53, 168, 51, 83, 167, 224, 100, 75, 64, 111, 185, 12, 211, 105, 189, 38, 218, 10, 24, 34, 81, 107, 226, 95, 143, 94, 57, 75, 217, 40, 66, 213, 239, 107, 137, 103, 122, 79, 94, 116, 87, 19, 120, 193, 8, 254, 101, 67, 122, 237, 221, 38, 135, 204, 179, 95, 83, 21, 199, 157, 160, 13, 18, 146, 183, 168, 43, 50, 84, 148, 88, 150, 133, 103, 204, 154, 121, 11, 214, 123, 71, 101, 43, 91, 237, 202, 244, 157, 89, 10, 140, 39, 121, 66, 39, 60, 46, 123, 180, 111, 126, 141, 37, 134, 174, 103, 26, 69, 94, 49, 188, 64, 97, 222, 8, 231, 52, 115, 4, 181, 146, 124, 233, 129, 130, 239, 201, 250, 76, 140, 249, 193, 94, 253, 236, 21, 248, 107, 137, 55, 250, 235, 72, 182, 98, 175, 112, 31, 9, 247, 139, 202, 184, 229, 165, 220, 63, 82, 206, 185, 146, 238, 222, 178, 78, 122, 13, 20, 124, 215, 40, 165, 56, 143, 104, 77, 247, 91, 249, 208, 189, 231, 127, 172, 48, 119, 46, 165, 230, 51, 124, 119, 99, 95, 217, 132, 234, 55, 154, 169, 72, 33, 200, 93, 151, 52, 84, 181, 188, 143, 210, 241, 137, 161, 65, 230, 227, 4, 246, 123, 10, 155, 91, 223, 40, 26, 243, 136, 110, 219, 219, 20, 180, 190, 43, 69, 53, 143, 149, 81, 46, 165, 77, 169, 97, 236, 53, 196, 20, 240, 73, 224, 106, 41, 108, 73, 175, 18, 97, 87, 45, 203, 172, 251, 160, 30, 33, 166, 121, 7, 23, 21, 249, 120, 61, 190, 96, 41, 128, 52, 2, 255, 125, 189, 211, 19, 16, 197, 41, 203, 198, 252, 248, 224, 213, 12, 69, 163, 214, 146, 187, 251, 181, 113, 114, 159, 116, 100, 249, 99, 212, 20, 131, 58, 222, 155, 149, 190, 37, 94, 31, 122, 186, 18, 195, 13, 50, 209, 208, 103, 36, 89, 224, 167, 225, 119, 207, 48, 5, 19, 238, 214, 189, 27, 43, 174, 26, 56, 238, 250, 24, 102, 135, 197, 150, 113, 195, 145, 11, 184, 44, 162, 97, 36, 32, 165, 181, 234, 54, 141, 215, 69, 158, 211, 168, 94, 3, 228, 142, 230, 87, 53, 124, 234, 24, 254, 52, 188, 5, 246, 147, 30, 22, 24, 192, 31, 127, 129, 19, 254, 238, 27, 127, 255, 205, 95, 228, 249, 124, 130, 37, 212, 123, 111, 254, 82, 111, 233, 97, 55, 122, 223, 173, 130, 1, 32, 155, 44, 110, 117, 248, 97, 159, 136, 225, 107, 101, 165, 107, 0, 150, 87, 6, 141, 90, 122, 118, 26, 214, 187, 27, 205, 222, 90, 115, 189, 215, 236, 52, 106, 158, 167, 125, 233, 149, 187, 247, 191, 209, 15, 56, 64, 130, 119, 218, 157, 102, 48, 232, 7, 239, 55, 182, 155, 221, 181, 78, 179, 183, 190, 249, 33, 184, 109, 186, 197, 54, 121, 177, 181, 245, 102, 119, 251, 65, 115, 123, 221, 87, 170, 219, 238, 242, 82, 61, 44, 246, 96, 187, 185, 229, 47, 37, 96, 245, 214, 183, 154, 221, 238, 26, 254, 103, 148, 251, 224, 222, 246, 5, 177, 168, 135, 250, 86, 166, 55, 32, 3, 6, 63, 108, 6, 189, 118, 199, 138, 176, 23, 13, 175, 248, 171, 133, 51, 116, 114, 159, 14, 15, 97, 254, 94, 241, 0, 174, 28, 30, 190, 252, 96, 132, 144, 160, 7, 162, 160, 22, 100, 116, 138, 247, 149, 128, 182, 245, 239, 110, 8, 14, 221, 31, 110, 255, 25, 150, 244, 58, 40, 29, 141, 219, 198, 39, 199, 195, 94, 54, 137, 33, 177, 90, 221, 15, 205, 224, 84, 79, 113, 122, 198, 47, 57, 17, 58, 173, 32, 196, 112, 50, 171, 65, 253, 148, 66, 112, 135, 158, 152, 175, 19, 210, 70, 67, 64, 2, 53, 137, 250, 69, 51, 42, 2, 69, 242, 160, 135, 23, 24, 53, 230, 180, 61, 120, 31, 125, 104, 93, 52, 190, 207, 27, 238, 21, 121, 209, 35, 128, 229, 237, 138, 231, 221, 164, 48, 30, 0, 59, 211, 11, 132, 251, 244, 196, 31, 49, 190, 205, 175, 148, 109, 112, 172, 44, 169, 18, 43, 95, 215, 38, 67, 48, 48, 90, 183, 179, 21, 124, 207, 251, 196, 174, 160, 151, 221, 7, 237, 222, 3, 72, 33, 248, 68, 32, 222, 210, 15, 65, 111, 237, 65, 123, 205, 243, 104, 60, 142, 2, 195, 115, 189, 186, 194, 128, 222, 88, 107, 119, 124, 175, 50, 242, 194, 45, 168, 102, 178, 66, 249, 124, 243, 190, 236, 173, 135, 96, 85, 140, 218, 107, 111, 208, 115, 135, 85, 207, 82, 73, 28, 49, 120, 70, 65, 101, 75, 32, 124, 101, 18, 165, 24, 61, 164, 222, 197, 160, 64, 72, 129, 213, 128, 226, 99, 208, 153, 117, 5, 17, 210, 241, 178, 52, 88, 211, 137, 0, 64, 65, 219, 24, 87, 209, 0, 151, 84, 232, 252, 115, 22, 230, 200, 217, 42, 110, 165, 244, 69, 225, 231, 143, 60, 102, 36, 124, 226, 226, 27, 135, 215, 7, 113, 147, 30, 210, 101, 248, 226, 137, 235, 81, 32, 160, 112, 167, 209, 146, 155, 113, 203, 70, 19, 231, 111, 155, 23, 81, 16, 46, 125, 177, 188, 198, 69, 254, 216, 138, 164, 228, 15, 71, 126, 217, 116, 31, 236, 196, 145, 227, 175, 69, 117, 215, 155, 252, 139, 142, 31, 214, 54, 60, 239, 144, 242, 130, 72, 109, 252, 160, 114, 93, 215, 235, 64, 60, 25, 130, 60, 83, 23, 126, 45, 252, 232, 232, 53, 44, 180, 241, 27, 116, 45, 161, 136, 239, 13, 79, 20, 221, 107, 254, 18, 8, 6, 32, 24, 227, 31, 245, 246, 135, 171, 129, 208, 27, 156, 187, 10, 101, 224, 45, 156, 119, 99, 228, 171, 122, 145, 40, 250, 103, 232, 10, 170, 181, 43, 222, 26, 70, 77, 184, 196, 63, 222, 214, 180, 254, 163, 3, 39, 167, 194, 15, 24, 107, 97, 53, 184, 108, 136, 105, 207, 27, 227, 153, 94, 11, 10, 159, 249, 186, 108, 35, 34, 46, 17, 82, 200, 88, 180, 203, 139, 211, 40, 95, 53, 131, 235, 38, 245, 129, 255, 219, 88, 100, 55, 38, 73, 202, 249, 156, 130, 39, 241, 159, 237, 88, 11, 49, 173, 167, 143, 129, 205, 180, 216, 211, 238, 160, 240, 29, 5, 202, 141, 181, 94, 89, 108, 65, 229, 57, 78, 122, 132, 90, 226, 155, 157, 118, 119, 171, 177, 68, 180, 183, 218, 183, 225, 198, 104, 109, 52, 168, 45, 138, 248, 118, 151, 96, 110, 86, 72, 86, 209, 143, 29, 231, 137, 38, 125, 41, 149, 115, 7, 152, 33, 90, 173, 139, 42, 173, 110, 163, 241, 189, 100, 192, 150, 143, 253, 52, 134, 122, 142, 244, 220, 117, 100, 114, 83, 31, 8, 111, 93, 50, 19, 213, 37, 4, 197, 121, 171, 130, 23, 91, 226, 187, 241, 253, 101, 89, 172, 54, 46, 142, 115, 109, 138, 172, 122, 166, 77, 227, 251, 177, 107, 216, 139, 96, 129, 5, 149, 197, 27, 157, 141, 140, 188, 190, 104, 108, 37, 86, 54, 141, 238, 138, 128, 59, 209, 30, 14, 66, 171, 245, 75, 146, 254, 136, 34, 180, 151, 83, 30, 89, 189, 154, 244, 8, 225, 239, 72, 249, 59, 16, 181, 16, 255, 73, 198, 202, 163, 185, 185, 65, 194, 42, 2, 227, 217, 17, 143, 229, 163, 5, 197, 138, 200, 205, 78, 69, 21, 55, 248, 225, 162, 216, 177, 20, 50, 16, 229, 172, 34, 43, 42, 23, 182, 120, 69, 153, 123, 185, 243, 155, 114, 210, 98, 196, 41, 70, 58, 174, 36, 166, 212, 81, 44, 18, 3, 160, 241, 34, 41, 69, 138, 253, 174, 242, 25, 144, 118, 154, 46, 110, 35, 191, 101, 163, 117, 248, 63, 178, 164, 242, 77, 17, 8, 179, 77, 220, 24, 173, 129, 52, 219, 92, 78, 152, 121, 194, 175, 125, 169, 48, 211, 35, 199, 241, 37, 161, 162, 23, 155, 128, 189, 17, 86, 238, 251, 222, 151, 115, 139, 29, 70, 81, 106, 80, 190, 53, 35, 38, 79, 20, 12, 158, 39, 75, 181, 255, 109, 206, 210, 107, 238, 23, 130, 87, 168, 241, 41, 237, 22, 47, 86, 115, 47, 233, 11, 31, 23, 83, 84, 240, 212, 118, 52, 157, 178, 244, 249, 201, 139, 35, 60, 191, 182, 44, 67, 15, 41, 44, 15, 108, 135, 178, 108, 119, 133, 151, 111, 101, 151, 104, 74, 90, 9, 200, 235, 96, 119, 229, 52, 28, 156, 159, 209, 86, 167, 255, 221, 141, 90, 136, 59, 160, 227, 222, 174, 236, 61, 92, 69, 0, 123, 211, 136, 13, 211, 232, 236, 235, 1, 223, 214, 128, 71, 67, 22, 198, 95, 11, 116, 183, 189, 161, 129, 30, 39, 131, 177, 1, 249, 83, 181, 178, 224, 170, 209, 201, 140, 77, 159, 74, 223, 231, 48, 246, 92, 191, 210, 189, 165, 213, 45, 240, 29, 59, 155, 28, 12, 197, 249, 144, 157, 9, 82, 249, 109, 196, 80, 84, 215, 198, 92, 93, 175, 221, 245, 74, 80, 69, 224, 156, 166, 188, 47, 100, 89, 120, 248, 133, 52, 238, 78, 143, 93, 59, 137, 242, 24, 157, 176, 63, 241, 238, 134, 211, 44, 26, 140, 243, 190, 140, 182, 18, 243, 23, 144, 29, 229, 250, 52, 28, 210, 153, 181, 238, 184, 68, 240, 158, 96, 134, 237, 185, 68, 165, 27, 188, 146, 101, 196, 50, 98, 186, 216, 84, 148, 15, 159, 156, 30, 1, 1, 139, 214, 68, 66, 173, 225, 43, 25, 102, 249, 230, 216, 44, 75, 73, 165, 165, 123, 235, 110, 113, 178, 194, 89, 175, 239, 241, 70, 27, 18, 157, 54, 177, 41, 189, 251, 19, 14, 135, 245, 26, 134, 251, 184, 96, 190, 90, 212, 124, 163, 64, 78, 171, 153, 82, 228, 137, 5, 149, 1, 153, 134, 134, 236, 82, 213, 133, 206, 58, 78, 112, 55, 168, 248, 147, 159, 45, 160, 41, 205, 224, 137, 167, 220, 5, 169, 142, 57, 200, 146, 39, 225, 41, 62, 156, 36, 106, 55, 69, 112, 4, 191, 47, 156, 238, 77, 238, 36, 57, 46, 78, 18, 161, 71, 194, 217, 9, 151, 23, 219, 101, 74, 152, 236, 196, 22, 86, 194, 182, 83, 170, 64, 115, 136, 8, 219, 242, 168, 130, 190, 18, 147, 190, 141, 178, 232, 52, 102, 117, 135, 123, 107, 48, 111, 240, 60, 160, 97, 135, 79, 226, 179, 163, 202, 4, 77, 26, 137, 46, 51, 116, 105, 177, 84, 195, 116, 160, 103, 223, 98, 168, 244, 152, 244, 250, 75, 150, 29, 45, 210, 249, 28, 217, 154, 117, 204, 236, 195, 189, 202, 131, 189, 26, 157, 49, 228, 116, 29, 52, 183, 79, 152, 233, 125, 29, 153, 245, 24, 231, 118, 113, 200, 165, 151, 115, 206, 123, 176, 49, 143, 157, 157, 113, 239, 90, 237, 168, 141, 14, 124, 228, 25, 143, 207, 50, 159, 76, 19, 183, 2, 166, 230, 201, 52, 26, 64, 37, 180, 126, 83, 226, 212, 218, 125, 88, 15, 179, 249, 15, 199, 40, 48, 162, 192, 74, 110, 21, 93, 32, 121, 6, 42, 247, 49, 233, 237, 200, 150, 194, 57, 90, 85, 236, 163, 155, 61, 225, 169, 157, 9, 138, 20, 253, 92, 208, 71, 51, 238, 207, 5, 131, 137, 132, 155, 139, 254, 133, 179, 143, 202, 145, 189, 203, 111, 76, 182, 47, 163, 81, 244, 177, 112, 255, 162, 44, 239, 59, 22, 25, 190, 198, 101, 5, 174, 135, 161, 156, 95, 245, 77, 34, 66, 202, 71, 44, 221, 104, 26, 5, 103, 179, 209, 112, 185, 146, 131, 164, 103, 21, 132, 20, 95, 65, 84, 97, 173, 146, 152, 228, 43, 42, 183, 56, 86, 113, 153, 236, 171, 18, 179, 112, 100, 35, 12, 73, 190, 162, 32, 113, 172, 146, 144, 194, 11, 234, 218, 132, 78, 85, 21, 11, 70, 30, 127, 9, 169, 187, 231, 221, 237, 241, 163, 32, 132, 236, 222, 214, 54, 74, 69, 67, 195, 237, 105, 137, 19, 98, 76, 173, 107, 1, 76, 233, 160, 6, 87, 88, 172, 190, 23, 248, 253, 208, 198, 17, 173, 200, 130, 41, 252, 56, 209, 221, 44, 44, 87, 152, 74, 112, 155, 36, 162, 201, 16, 211, 171, 243, 81, 44, 231, 113, 158, 161, 14, 85, 84, 211, 230, 143, 167, 182, 154, 153, 218, 65, 234, 71, 6, 130, 242, 19, 30, 229, 87, 226, 130, 147, 95, 159, 156, 240, 89, 50, 221, 204, 67, 99, 165, 53, 197, 116, 42, 38, 207, 234, 37, 183, 47, 24, 14, 89, 140, 78, 237, 229, 135, 58, 155, 235, 42, 111, 102, 127, 147, 56, 193, 236, 54, 49, 109, 153, 70, 177, 92, 209, 42, 126, 221, 169, 217, 103, 33, 110, 81, 124, 141, 127, 28, 81, 214, 178, 56, 136, 226, 38, 42, 34, 81, 97, 180, 238, 199, 72, 156, 91, 141, 67, 16, 101, 177, 8, 13, 165, 107, 157, 152, 92, 209, 135, 227, 25, 84, 72, 233, 156, 209, 132, 129, 14, 177, 122, 66, 59, 163, 63, 108, 40, 14, 155, 223, 119, 62, 240, 16, 84, 149, 69, 12, 93, 182, 146, 160, 3, 58, 47, 53, 40, 9, 73, 14, 5, 85, 6, 206, 81, 73, 46, 248, 86, 100, 234, 44, 24, 184, 156, 31, 187, 26, 13, 113, 115, 242, 226, 177, 194, 114, 197, 32, 225, 215, 210, 252, 162, 206, 112, 173, 166, 141, 131, 171, 5, 205, 171, 227, 48, 133, 130, 76, 89, 26, 13, 20, 229, 206, 12, 101, 120, 249, 114, 137, 41, 10, 229, 180, 57, 10, 95, 139, 154, 85, 230, 239, 87, 231, 134, 202, 128, 203, 4, 222, 188, 46, 66, 246, 203, 228, 101, 240, 176, 109, 225, 170, 115, 23, 116, 16, 206, 155, 123, 164, 64, 42, 36, 215, 22, 208, 6, 202, 242, 168, 212, 184, 217, 251, 83, 132, 251, 198, 239, 110, 44, 252, 62, 162, 117, 220, 242, 166, 213, 243, 84, 99, 197, 37, 12, 188, 39, 241, 31, 30, 72, 100, 79, 47, 3, 133, 153, 94, 88, 231, 175, 195, 79, 62, 189, 140, 212, 157, 131, 194, 49, 3, 123, 195, 117, 160, 90, 105, 113, 177, 182, 82, 131, 34, 197, 124, 156, 147, 47, 40, 18, 118, 195, 23, 54, 158, 168, 45, 94, 31, 161, 82, 214, 238, 181, 118, 174, 69, 118, 217, 41, 74, 241, 205, 137, 188, 13, 91, 251, 118, 123, 16, 174, 1, 107, 218, 207, 149, 168, 64, 136, 2, 191, 135, 56, 235, 203, 27, 155, 99, 88, 201, 0, 199, 174, 170, 173, 209, 198, 54, 235, 156, 46, 108, 107, 175, 186, 173, 159, 255, 122, 186, 76, 91, 91, 15, 186, 15, 186, 254, 182, 202, 97, 71, 147, 133, 128, 215, 214, 135, 107, 219, 219, 22, 224, 91, 87, 215, 230, 38, 109, 216, 114, 78, 207, 196, 107, 188, 249, 91, 201, 232, 186, 214, 150, 92, 54, 77, 246, 163, 64, 94, 81, 76, 81, 109, 156, 24, 141, 1, 190, 60, 26, 95, 63, 21, 215, 194, 178, 103, 105, 50, 81, 91, 27, 119, 191, 194, 181, 243, 19, 128, 6, 91, 212, 161, 216, 179, 101, 101, 229, 142, 201, 41, 31, 75, 103, 66, 141, 111, 84, 234, 144, 168, 62, 242, 27, 27, 242, 238, 117, 83, 40, 141, 31, 12, 101, 73, 122, 160, 249, 118, 14, 127, 152, 69, 79, 194, 212, 197, 200, 235, 224, 85, 229, 226, 245, 117, 175, 200, 243, 189, 53, 14, 24, 212, 115, 177, 243, 111, 189, 173, 195, 61, 57, 246, 184, 122, 26, 54, 57, 105, 234, 40, 114, 148, 197, 109, 87, 179, 185, 217, 243, 190, 42, 84, 128, 112, 75, 43, 14, 216, 179, 58, 127, 140, 3, 120, 233, 60, 154, 97, 47, 20, 55, 149, 30, 97, 16, 107, 189, 131, 13, 90, 201, 238, 183, 218, 247, 109, 121, 215, 108, 181, 77, 151, 158, 207, 174, 119, 182, 207, 131, 208, 174, 86, 237, 67, 88, 178, 49, 37, 83, 134, 107, 209, 184, 93, 226, 182, 215, 49, 203, 243, 104, 122, 150, 125, 169, 149, 34, 19, 112, 106, 119, 52, 62, 160, 4, 193, 171, 130, 133, 122, 41, 36, 135, 248, 196, 69, 164, 86, 43, 169, 194, 151, 165, 51, 86, 107, 26, 107, 207, 25, 243, 215, 82, 43, 51, 85, 18, 11, 90, 211, 46, 200, 39, 22, 119, 129, 45, 212, 87, 195, 51, 182, 17, 152, 5, 84, 68, 79, 163, 148, 187, 82, 94, 10, 54, 164, 213, 82, 44, 173, 69, 154, 139, 70, 145, 103, 79, 155, 34, 199, 146, 246, 202, 30, 193, 73, 176, 83, 86, 139, 172, 154, 47, 97, 174, 161, 252, 23, 107, 84, 13, 79, 65, 133, 14, 44, 109, 25, 0, 36, 225, 198, 61, 224, 202, 242, 59, 14, 197, 152, 68, 51, 57, 26, 209, 140, 7, 14, 47, 25, 188, 179, 75, 89, 240, 236, 178, 178, 96, 54, 149, 5, 179, 169, 167, 96, 161, 132, 68, 131, 147, 228, 236, 44, 214, 76, 215, 60, 245, 112, 198, 211, 93, 2, 235, 181, 26, 6, 12, 224, 63, 54, 56, 167, 69, 142, 55, 77, 121, 216, 188, 243, 66, 155, 88, 103, 68, 83, 100, 115, 140, 98, 208, 163, 235, 238, 18, 87, 200, 161, 155, 192, 52, 26, 245, 3, 135, 240, 93, 160, 122, 167, 25, 8, 131, 147, 40, 32, 205, 79, 42, 59, 154, 245, 37, 185, 155, 193, 217, 101, 95, 146, 20, 214, 167, 105, 95, 145, 205, 90, 110, 85, 188, 85, 190, 58, 90, 107, 142, 255, 158, 139, 111, 134, 151, 11, 71, 159, 172, 147, 16, 150, 16, 117, 142, 232, 226, 163, 124, 124, 240, 242, 248, 213, 155, 143, 39, 7, 47, 94, 31, 61, 62, 57, 56, 54, 92, 67, 111, 72, 255, 239, 215, 158, 60, 255, 127, 219, 123, 178, 229, 56, 142, 228, 222, 245, 21, 77, 154, 161, 153, 182, 128, 1, 64, 82, 90, 17, 60, 16, 36, 0, 46, 105, 131, 36, 76, 64, 146, 99, 73, 6, 208, 192, 52, 6, 109, 12, 186, 103, 187, 123, 112, 16, 49, 17, 251, 228, 15, 240, 58, 246, 11, 252, 5, 126, 240, 147, 222, 244, 39, 250, 18, 103, 214, 213, 117, 100, 245, 49, 0, 168, 149, 195, 47, 36, 166, 186, 42, 171, 42, 43, 43, 43, 43, 43, 143, 149, 63, 124, 187, 12, 52, 147, 2, 121, 107, 63, 97, 168, 241, 40, 203, 47, 87, 123, 99, 150, 218, 18, 112, 7, 183, 98, 168, 145, 220, 63, 68, 253, 251, 48, 90, 189, 191, 2, 255, 31, 142, 87, 239, 223, 215, 13, 59, 37, 224, 221, 157, 173, 251, 223, 62, 90, 81, 144, 171, 223, 215, 6, 189, 243, 106, 247, 65, 5, 88, 254, 170, 192, 30, 142, 241, 124, 140, 187, 3, 126, 241, 102, 243, 254, 247, 26, 50, 228, 207, 27, 0, 253, 102, 235, 95, 31, 45, 127, 183, 242, 80, 1, 215, 10, 52, 132, 240, 75, 98, 87, 216, 175, 254, 180, 242, 168, 2, 252, 106, 145, 255, 212, 70, 205, 110, 248, 2, 234, 20, 99, 96, 47, 4, 249, 197, 234, 202, 119, 32, 164, 193, 127, 127, 240, 1, 125, 104, 2, 125, 120, 109, 160, 59, 235, 27, 15, 42, 252, 202, 95, 30, 152, 173, 201, 97, 125, 227, 161, 1, 244, 225, 220, 64, 63, 185, 209, 142, 184, 159, 42, 63, 8, 64, 158, 93, 23, 96, 251, 18, 62, 102, 211, 102, 14, 206, 174, 37, 39, 23, 151, 95, 92, 202, 54, 150, 46, 155, 81, 255, 42, 183, 100, 95, 48, 56, 10, 163, 177, 85, 97, 160, 110, 4, 240, 99, 42, 98, 65, 38, 95, 89, 26, 108, 54, 81, 50, 190, 60, 21, 127, 191, 26, 214, 7, 57, 149, 79, 252, 192, 176, 28, 203, 213, 244, 148, 219, 144, 200, 162, 160, 12, 49, 84, 208, 142, 222, 48, 142, 78, 202, 228, 44, 137, 129, 26, 236, 84, 37, 119, 204, 76, 85, 181, 112, 82, 124, 232, 5, 145, 240, 4, 196, 26, 19, 146, 150, 104, 69, 77, 69, 8, 118, 82, 159, 106, 60, 246, 136, 148, 35, 214, 3, 33, 107, 63, 152, 128, 44, 27, 243, 252, 133, 188, 0, 53, 35, 238, 232, 196, 125, 175, 26, 29, 155, 163, 51, 63, 3, 38, 5, 71, 92, 126, 43, 56, 250, 5, 93, 135, 118, 13, 4, 17, 193, 23, 240, 40, 173, 238, 117, 2, 87, 120, 25, 114, 108, 105, 121, 33, 157, 87, 98, 152, 49, 3, 48, 168, 225, 152, 137, 112, 129, 104, 17, 106, 216, 239, 176, 119, 160, 204, 159, 153, 76, 109, 10, 6, 21, 15, 95, 56, 243, 6, 170, 220, 162, 69, 241, 240, 38, 130, 9, 88, 141, 84, 49, 222, 158, 240, 146, 211, 243, 81, 76, 251, 237, 172, 217, 68, 100, 165, 80, 2, 84, 22, 30, 50, 165, 205, 192, 200, 130, 235, 243, 254, 33, 238, 213, 186, 189, 70, 118, 56, 197, 24, 182, 38, 102, 159, 143, 65, 38, 251, 7, 254, 124, 46, 36, 143, 96, 192, 135, 191, 136, 147, 239, 85, 241, 2, 241, 39, 222, 1, 107, 214, 186, 230, 117, 152, 95, 101, 165, 112, 195, 59, 228, 70, 201, 99, 212, 21, 184, 124, 141, 87, 49, 162, 195, 24, 195, 116, 200, 64, 53, 112, 137, 65, 125, 50, 140, 134, 244, 133, 231, 53, 180, 212, 139, 216, 7, 109, 205, 198, 241, 80, 225, 243, 48, 135, 171, 112, 44, 82, 202, 244, 123, 67, 216, 181, 142, 190, 192, 18, 238, 53, 252, 58, 53, 29, 18, 149, 228, 228, 37, 218, 192, 67, 166, 178, 157, 44, 88, 19, 84, 139, 87, 7, 38, 50, 19, 1, 106, 248, 14, 244, 205, 13, 141, 139, 204, 201, 33, 213, 18, 23, 23, 182, 79, 235, 244, 146, 109, 145, 87, 80, 119, 35, 142, 62, 254, 169, 71, 212, 70, 71, 194, 116, 184, 126, 156, 140, 135, 125, 100, 14, 254, 244, 243, 237, 39, 202, 90, 216, 193, 117, 238, 93, 9, 20, 163, 84, 128, 203, 34, 126, 162, 200, 48, 11, 250, 234, 179, 92, 57, 195, 207, 135, 28, 46, 235, 198, 37, 31, 189, 10, 111, 69, 76, 10, 174, 208, 157, 112, 139, 245, 73, 204, 242, 196, 61, 5, 177, 128, 7, 101, 90, 211, 195, 193, 180, 44, 51, 11, 111, 208, 66, 176, 181, 115, 97, 237, 216, 139, 166, 101, 214, 179, 235, 152, 152, 117, 41, 119, 67, 29, 250, 120, 142, 161, 123, 123, 245, 211, 134, 229, 134, 218, 4, 105, 231, 240, 4, 78, 54, 174, 110, 145, 9, 139, 204, 11, 124, 124, 54, 64, 247, 170, 237, 60, 155, 68, 35, 166, 249, 234, 147, 175, 196, 28, 59, 228, 32, 135, 73, 129, 127, 243, 203, 57, 43, 182, 174, 251, 252, 154, 7, 8, 56, 142, 135, 47, 81, 95, 163, 116, 53, 140, 167, 45, 65, 15, 71, 201, 168, 135, 250, 52, 16, 44, 142, 179, 225, 106, 111, 251, 221, 206, 46, 20, 28, 51, 197, 106, 177, 122, 213, 19, 88, 90, 220, 5, 34, 235, 173, 246, 144, 29, 39, 135, 108, 188, 75, 23, 139, 231, 231, 231, 139, 152, 234, 107, 113, 154, 143, 227, 244, 16, 238, 165, 195, 222, 108, 33, 56, 200, 134, 151, 171, 251, 124, 228, 79, 239, 93, 241, 63, 102, 95, 39, 195, 167, 138, 70, 147, 225, 108, 223, 246, 21, 110, 184, 149, 206, 92, 114, 210, 9, 21, 150, 130, 160, 82, 241, 200, 222, 154, 74, 177, 190, 73, 165, 60, 154, 44, 150, 247, 156, 138, 55, 69, 70, 206, 100, 156, 183, 124, 103, 95, 98, 19, 243, 78, 175, 78, 29, 163, 90, 98, 61, 235, 215, 156, 167, 100, 176, 212, 90, 213, 161, 88, 165, 235, 106, 14, 13, 106, 36, 22, 145, 79, 77, 106, 98, 196, 47, 203, 70, 92, 119, 230, 97, 103, 188, 210, 198, 169, 2, 104, 97, 235, 15, 92, 245, 183, 95, 102, 168, 127, 175, 48, 219, 181, 209, 250, 251, 213, 237, 62, 253, 139, 15, 79, 93, 181, 48, 58, 156, 121, 116, 49, 232, 101, 113, 158, 124, 142, 242, 225, 14, 247, 208, 90, 121, 236, 124, 217, 21, 120, 179, 140, 232, 170, 10, 207, 135, 103, 145, 52, 255, 226, 26, 53, 91, 136, 27, 101, 63, 169, 78, 250, 182, 59, 149, 209, 63, 126, 124, 220, 74, 246, 228, 147, 231, 112, 131, 1, 7, 178, 136, 205, 53, 209, 19, 207, 234, 103, 232, 255, 237, 183, 252, 36, 140, 91, 241, 117, 68, 231, 52, 230, 149, 98, 223, 223, 243, 7, 36, 83, 246, 215, 211, 143, 119, 209, 99, 55, 158, 204, 62, 222, 253, 180, 239, 88, 167, 98, 15, 161, 232, 169, 217, 28, 86, 106, 156, 63, 191, 229, 41, 25, 149, 186, 153, 21, 144, 53, 119, 34, 221, 236, 87, 20, 216, 146, 175, 0, 16, 74, 208, 174, 101, 40, 174, 201, 147, 224, 1, 50, 192, 36, 69, 207, 131, 69, 175, 133, 168, 0, 136, 253, 132, 114, 4, 52, 64, 188, 6, 181, 0, 233, 94, 5, 208, 219, 64, 208, 145, 88, 12, 42, 96, 1, 108, 32, 219, 220, 133, 47, 152, 188, 77, 241, 175, 132, 85, 51, 156, 139, 116, 203, 93, 245, 197, 185, 60, 84, 189, 161, 143, 103, 5, 130, 184, 88, 202, 154, 222, 187, 68, 213, 218, 91, 197, 184, 157, 242, 196, 82, 60, 225, 60, 134, 36, 235, 167, 241, 121, 176, 19, 151, 125, 155, 55, 50, 95, 109, 150, 224, 189, 186, 30, 152, 193, 130, 21, 64, 181, 117, 160, 136, 190, 198, 100, 147, 186, 99, 56, 99, 207, 110, 38, 235, 207, 38, 90, 162, 180, 168, 180, 63, 153, 71, 175, 85, 161, 194, 154, 126, 22, 66, 51, 143, 103, 9, 31, 163, 120, 87, 6, 124, 138, 103, 64, 158, 201, 179, 116, 164, 55, 174, 89, 168, 89, 181, 86, 171, 18, 56, 167, 145, 140, 15, 105, 33, 157, 167, 222, 130, 113, 84, 151, 227, 201, 216, 103, 229, 216, 29, 209, 38, 170, 1, 52, 187, 80, 184, 53, 156, 219, 8, 86, 197, 171, 136, 25, 172, 211, 152, 187, 31, 253, 230, 2, 204, 236, 120, 39, 218, 42, 244, 43, 58, 211, 99, 255, 184, 59, 125, 146, 77, 166, 120, 228, 240, 221, 190, 157, 64, 83, 24, 162, 101, 23, 46, 222, 148, 144, 215, 12, 35, 144, 116, 113, 18, 197, 48, 114, 30, 12, 205, 170, 135, 99, 89, 245, 112, 92, 95, 245, 253, 133, 168, 153, 95, 212, 87, 220, 149, 21, 75, 187, 162, 59, 179, 2, 115, 209, 242, 105, 217, 83, 114, 78, 91, 164, 13, 66, 210, 49, 114, 21, 250, 78, 96, 229, 75, 73, 241, 76, 106, 30, 36, 143, 92, 176, 6, 85, 113, 143, 155, 96, 158, 250, 190, 187, 186, 205, 29, 231, 155, 132, 218, 133, 9, 200, 134, 183, 181, 13, 17, 118, 219, 125, 200, 234, 222, 208, 70, 52, 26, 201, 193, 88, 152, 48, 135, 53, 211, 220, 52, 157, 253, 103, 182, 12, 9, 215, 168, 97, 124, 20, 77, 199, 37, 214, 126, 157, 30, 101, 248, 28, 15, 119, 146, 116, 8, 205, 38, 80, 22, 156, 199, 249, 48, 6, 89, 48, 78, 167, 229, 231, 114, 160, 227, 236, 131, 218, 197, 106, 143, 202, 29, 40, 54, 216, 167, 106, 173, 134, 93, 163, 15, 105, 113, 128, 196, 69, 219, 141, 94, 53, 11, 105, 199, 158, 72, 236, 44, 156, 213, 79, 81, 142, 239, 215, 182, 109, 134, 33, 220, 234, 206, 2, 28, 101, 239, 96, 185, 226, 161, 215, 173, 195, 227, 224, 162, 203, 152, 142, 111, 141, 245, 148, 74, 109, 221, 182, 194, 15, 123, 27, 32, 106, 152, 188, 214, 253, 78, 233, 74, 152, 61, 17, 145, 150, 68, 19, 91, 212, 221, 218, 130, 198, 232, 147, 184, 2, 118, 228, 42, 255, 127, 160, 223, 252, 129, 174, 214, 2, 39, 70, 225, 37, 29, 122, 177, 210, 120, 124, 209, 167, 94, 3, 19, 163, 240, 224, 145, 24, 106, 21, 33, 250, 238, 217, 237, 68, 105, 213, 174, 217, 37, 207, 48, 251, 219, 60, 187, 5, 103, 168, 182, 203, 174, 133, 133, 185, 150, 135, 67, 124, 202, 61, 98, 216, 218, 152, 200, 239, 178, 44, 141, 130, 3, 46, 18, 33, 45, 92, 103, 165, 0, 137, 92, 55, 243, 130, 105, 147, 229, 227, 142, 86, 106, 174, 144, 254, 37, 52, 90, 215, 104, 122, 249, 114, 232, 98, 26, 65, 192, 213, 221, 73, 167, 33, 94, 127, 29, 221, 25, 109, 34, 210, 62, 81, 84, 164, 125, 14, 93, 96, 205, 163, 109, 119, 134, 240, 131, 171, 102, 232, 244, 168, 253, 3, 118, 73, 158, 151, 215, 13, 56, 38, 239, 96, 32, 138, 112, 101, 72, 34, 158, 73, 141, 1, 48, 167, 172, 234, 240, 12, 22, 173, 163, 53, 12, 158, 5, 43, 223, 46, 135, 221, 16, 65, 83, 156, 165, 72, 121, 17, 29, 158, 24, 138, 20, 44, 48, 49, 34, 10, 67, 89, 189, 113, 185, 12, 129, 65, 5, 172, 90, 89, 208, 53, 97, 139, 193, 74, 104, 47, 85, 179, 6, 136, 212, 234, 52, 13, 231, 202, 214, 225, 40, 109, 28, 115, 126, 108, 207, 154, 88, 118, 93, 138, 91, 161, 110, 167, 247, 155, 179, 172, 102, 198, 67, 139, 116, 42, 138, 216, 3, 99, 133, 190, 193, 21, 114, 147, 203, 73, 86, 85, 137, 141, 22, 183, 50, 62, 216, 12, 203, 248, 24, 218, 96, 58, 46, 229, 29, 166, 23, 206, 79, 251, 189, 159, 146, 252, 4, 42, 30, 7, 0, 232, 0, 22, 102, 52, 77, 71, 193, 47, 255, 133, 90, 235, 116, 173, 23, 186, 82, 146, 115, 173, 52, 165, 229, 6, 9, 153, 155, 118, 11, 111, 242, 223, 88, 204, 183, 238, 195, 51, 239, 146, 53, 107, 77, 73, 21, 103, 227, 155, 161, 181, 48, 226, 61, 52, 27, 162, 62, 20, 21, 118, 63, 188, 223, 218, 137, 163, 252, 240, 120, 59, 202, 163, 83, 227, 118, 142, 181, 6, 128, 107, 174, 20, 206, 16, 181, 120, 220, 209, 170, 75, 221, 201, 167, 78, 70, 86, 178, 184, 219, 160, 141, 134, 244, 71, 149, 74, 96, 147, 197, 100, 102, 127, 144, 219, 92, 117, 196, 155, 240, 30, 215, 248, 255, 100, 131, 106, 186, 124, 56, 123, 8, 28, 201, 137, 119, 75, 34, 70, 138, 24, 220, 166, 206, 172, 197, 7, 192, 124, 128, 247, 185, 159, 213, 143, 44, 184, 235, 86, 118, 46, 131, 125, 204, 246, 238, 93, 85, 7, 139, 46, 45, 87, 93, 36, 232, 0, 96, 208, 218, 111, 250, 26, 203, 71, 86, 102, 59, 101, 158, 164, 35, 76, 166, 234, 170, 245, 181, 189, 75, 241, 111, 118, 29, 111, 164, 190, 128, 213, 51, 233, 15, 86, 166, 220, 195, 226, 158, 167, 162, 139, 44, 253, 107, 193, 20, 117, 125, 109, 131, 65, 65, 184, 198, 169, 129, 177, 247, 222, 226, 138, 15, 116, 193, 84, 119, 122, 99, 40, 104, 219, 56, 191, 48, 219, 2, 207, 105, 219, 180, 180, 154, 238, 54, 53, 253, 109, 201, 131, 13, 220, 67, 30, 51, 87, 107, 131, 120, 84, 238, 19, 127, 140, 139, 73, 12, 231, 68, 156, 151, 131, 224, 69, 82, 194, 49, 250, 54, 158, 50, 97, 56, 24, 78, 129, 74, 142, 126, 249, 249, 56, 143, 211, 1, 253, 232, 228, 248, 67, 124, 118, 221, 33, 36, 19, 21, 190, 16, 234, 79, 210, 141, 33, 211, 195, 155, 24, 135, 242, 3, 103, 59, 146, 214, 6, 54, 147, 207, 99, 54, 153, 183, 217, 185, 113, 42, 87, 197, 38, 183, 55, 170, 135, 102, 235, 206, 156, 223, 71, 22, 2, 170, 75, 15, 116, 166, 83, 235, 249, 190, 56, 140, 210, 183, 113, 121, 158, 229, 39, 133, 29, 57, 69, 174, 176, 114, 75, 65, 149, 220, 20, 150, 23, 86, 181, 252, 12, 76, 240, 36, 30, 12, 140, 181, 156, 227, 209, 63, 21, 157, 147, 174, 28, 69, 50, 116, 110, 213, 80, 70, 184, 111, 168, 154, 182, 127, 76, 245, 165, 70, 93, 19, 8, 111, 23, 57, 24, 153, 238, 64, 74, 19, 169, 63, 169, 203, 60, 90, 25, 83, 47, 147, 14, 112, 140, 84, 13, 71, 47, 195, 171, 50, 19, 175, 116, 144, 195, 223, 179, 225, 139, 211, 112, 223, 110, 171, 205, 185, 78, 77, 227, 38, 44, 64, 84, 210, 152, 16, 225, 221, 80, 150, 94, 14, 111, 28, 21, 150, 159, 234, 63, 51, 219, 97, 69, 100, 176, 244, 64, 175, 67, 203, 84, 166, 219, 68, 253, 222, 74, 126, 42, 143, 14, 70, 113, 113, 120, 12, 119, 104, 96, 189, 189, 22, 134, 25, 20, 45, 119, 51, 201, 80, 104, 159, 195, 24, 67, 123, 36, 202, 38, 239, 99, 225, 64, 244, 114, 12, 162, 137, 104, 207, 232, 94, 31, 68, 46, 107, 61, 151, 134, 53, 150, 188, 203, 185, 152, 168, 180, 155, 156, 198, 48, 7, 54, 119, 252, 59, 155, 150, 246, 199, 199, 46, 104, 246, 193, 137, 160, 164, 62, 111, 192, 153, 133, 239, 248, 44, 110, 174, 209, 175, 24, 116, 104, 46, 146, 154, 24, 203, 212, 136, 41, 5, 120, 45, 250, 118, 97, 180, 216, 142, 184, 87, 161, 121, 127, 247, 60, 178, 153, 8, 180, 205, 201, 111, 23, 39, 106, 53, 204, 107, 19, 133, 51, 77, 173, 241, 77, 240, 221, 178, 145, 177, 185, 17, 17, 190, 139, 86, 45, 194, 197, 230, 24, 229, 217, 185, 176, 115, 131, 230, 32, 162, 91, 71, 129, 140, 233, 57, 102, 105, 23, 200, 19, 77, 92, 51, 173, 105, 83, 58, 119, 3, 229, 106, 246, 95, 127, 173, 79, 255, 153, 139, 31, 231, 44, 168, 157, 23, 231, 58, 207, 83, 220, 129, 74, 126, 249, 49, 206, 15, 146, 116, 136, 23, 222, 73, 254, 203, 207, 71, 113, 26, 128, 188, 148, 163, 157, 196, 116, 178, 248, 124, 59, 192, 215, 40, 75, 162, 105, 181, 179, 42, 95, 8, 159, 39, 165, 155, 254, 71, 79, 130, 172, 157, 166, 253, 222, 113, 89, 78, 86, 151, 150, 236, 53, 97, 44, 5, 4, 58, 83, 88, 252, 227, 38, 202, 138, 204, 147, 176, 119, 136, 22, 237, 228, 49, 128, 89, 109, 209, 101, 196, 102, 245, 34, 252, 54, 194, 23, 246, 168, 222, 222, 123, 116, 14, 178, 26, 206, 92, 147, 118, 101, 105, 41, 72, 70, 105, 150, 3, 193, 230, 232, 251, 226, 65, 154, 179, 197, 112, 205, 197, 190, 68, 98, 92, 64, 77, 223, 50, 109, 112, 128, 223, 251, 62, 237, 177, 2, 140, 90, 67, 75, 250, 211, 190, 216, 2, 160, 217, 40, 116, 192, 116, 208, 201, 204, 131, 121, 87, 128, 21, 221, 191, 71, 28, 210, 211, 120, 111, 134, 152, 51, 230, 33, 155, 133, 46, 164, 14, 51, 161, 24, 44, 33, 170, 218, 126, 38, 132, 95, 173, 163, 25, 41, 91, 251, 253, 202, 8, 71, 210, 185, 87, 180, 92, 19, 127, 40, 183, 223, 85, 123, 231, 138, 11, 75, 142, 18, 71, 238, 246, 244, 94, 191, 5, 136, 11, 11, 175, 27, 202, 70, 154, 205, 30, 239, 12, 243, 185, 13, 81, 192, 89, 8, 238, 136, 126, 93, 58, 116, 221, 209, 13, 123, 91, 211, 15, 189, 230, 197, 200, 199, 140, 5, 2, 63, 51, 255, 35, 215, 169, 253, 113, 71, 214, 68, 93, 96, 133, 223, 240, 45, 217, 153, 203, 81, 63, 5, 161, 137, 125, 251, 225, 253, 235, 245, 236, 116, 2, 69, 32, 128, 150, 159, 195, 217, 62, 201, 234, 238, 248, 120, 157, 52, 36, 102, 74, 115, 62, 43, 172, 138, 5, 253, 112, 192, 120, 149, 8, 80, 209, 115, 229, 218, 242, 24, 118, 37, 83, 146, 8, 33, 79, 92, 74, 239, 93, 33, 12, 254, 99, 134, 1, 124, 0, 90, 109, 130, 168, 128, 202, 219, 142, 139, 244, 217, 108, 83, 155, 207, 171, 252, 108, 37, 241, 114, 83, 198, 107, 89, 188, 136, 76, 235, 118, 234, 77, 63, 167, 190, 166, 195, 183, 17, 40, 216, 251, 234, 26, 157, 197, 235, 60, 46, 132, 193, 196, 180, 114, 235, 221, 201, 104, 16, 90, 0, 230, 84, 195, 202, 200, 20, 110, 96, 52, 161, 224, 177, 93, 166, 232, 251, 113, 247, 93, 19, 216, 219, 38, 184, 142, 202, 103, 95, 12, 27, 246, 141, 248, 203, 217, 40, 53, 219, 228, 26, 155, 228, 26, 91, 164, 205, 37, 110, 18, 159, 96, 12, 58, 88, 28, 165, 150, 250, 178, 33, 11, 170, 163, 76, 60, 235, 48, 151, 11, 12, 57, 98, 62, 234, 168, 98, 235, 61, 71, 175, 30, 154, 173, 231, 164, 88, 25, 242, 196, 14, 232, 133, 93, 123, 104, 118, 78, 82, 183, 60, 239, 172, 222, 104, 197, 94, 65, 68, 56, 41, 168, 216, 38, 184, 182, 108, 65, 83, 188, 117, 52, 58, 238, 217, 30, 123, 179, 223, 223, 134, 252, 90, 69, 180, 121, 202, 194, 175, 51, 44, 58, 219, 116, 238, 189, 232, 151, 248, 141, 117, 241, 172, 198, 143, 219, 27, 129, 8, 253, 21, 68, 39, 229, 52, 26, 131, 136, 134, 187, 237, 177, 177, 30, 149, 147, 66, 134, 215, 77, 235, 144, 211, 46, 52, 60, 88, 114, 99, 170, 231, 54, 67, 83, 132, 2, 144, 143, 199, 92, 159, 3, 152, 75, 189, 67, 227, 193, 109, 172, 177, 181, 100, 8, 29, 184, 214, 173, 49, 159, 47, 133, 22, 47, 147, 195, 147, 21, 163, 219, 56, 231, 50, 22, 186, 135, 178, 168, 26, 234, 237, 230, 100, 110, 237, 131, 42, 85, 234, 101, 91, 177, 92, 203, 205, 38, 128, 146, 170, 1, 254, 106, 193, 254, 240, 78, 224, 191, 143, 172, 169, 123, 134, 27, 80, 72, 60, 64, 106, 173, 147, 73, 67, 135, 163, 243, 170, 242, 232, 188, 105, 116, 250, 2, 165, 222, 202, 78, 252, 41, 127, 204, 169, 158, 80, 151, 176, 119, 129, 199, 13, 81, 166, 140, 32, 131, 117, 87, 194, 249, 216, 52, 118, 126, 139, 44, 26, 201, 133, 190, 104, 224, 151, 112, 246, 53, 146, 7, 93, 1, 191, 64, 5, 78, 10, 232, 9, 43, 169, 68, 132, 114, 154, 125, 157, 76, 232, 150, 201, 4, 218, 141, 206, 233, 143, 163, 115, 4, 154, 122, 6, 149, 134, 95, 234, 180, 232, 74, 43, 193, 41, 116, 156, 198, 211, 248, 84, 169, 252, 59, 147, 79, 205, 9, 209, 118, 56, 47, 97, 243, 193, 94, 44, 130, 109, 232, 10, 85, 128, 141, 35, 192, 176, 126, 78, 8, 78, 87, 249, 222, 147, 0, 165, 6, 209, 22, 131, 41, 142, 207, 2, 183, 125, 129, 147, 197, 234, 167, 254, 84, 105, 139, 202, 154, 211, 165, 51, 70, 9, 124, 106, 58, 89, 11, 126, 27, 33, 28, 99, 216, 148, 206, 1, 165, 74, 157, 23, 92, 85, 57, 52, 154, 222, 250, 25, 101, 35, 220, 143, 238, 159, 96, 199, 20, 113, 2, 224, 227, 241, 24, 176, 18, 91, 82, 177, 144, 224, 185, 53, 151, 212, 29, 194, 237, 254, 116, 2, 119, 38, 183, 177, 241, 84, 191, 22, 236, 38, 147, 73, 28, 188, 223, 220, 217, 220, 13, 62, 195, 109, 234, 69, 92, 148, 191, 252, 87, 153, 96, 55, 118, 204, 68, 246, 126, 104, 116, 196, 163, 244, 13, 202, 60, 57, 5, 158, 98, 100, 202, 225, 1, 242, 25, 96, 124, 148, 234, 48, 225, 231, 7, 163, 248, 32, 207, 128, 196, 144, 164, 132, 86, 217, 201, 39, 214, 213, 72, 74, 140, 27, 231, 36, 6, 213, 108, 178, 195, 131, 169, 95, 46, 50, 202, 168, 61, 109, 110, 203, 100, 71, 211, 111, 115, 138, 147, 250, 225, 65, 30, 163, 77, 3, 102, 36, 50, 84, 222, 174, 192, 214, 41, 96, 98, 67, 176, 196, 26, 189, 163, 79, 139, 235, 181, 77, 192, 225, 111, 101, 215, 143, 72, 58, 206, 236, 104, 164, 85, 224, 242, 108, 244, 34, 99, 6, 58, 67, 149, 110, 46, 20, 89, 133, 63, 166, 86, 66, 112, 250, 209, 215, 134, 222, 141, 255, 98, 235, 121, 30, 123, 45, 84, 13, 179, 243, 244, 246, 208, 37, 246, 211, 56, 59, 16, 251, 233, 5, 252, 217, 255, 224, 193, 217, 167, 5, 25, 189, 14, 131, 188, 93, 148, 75, 147, 113, 148, 164, 61, 82, 248, 0, 106, 7, 144, 176, 61, 133, 181, 192, 187, 131, 127, 3, 54, 15, 191, 251, 216, 27, 21, 1, 162, 198, 194, 32, 178, 236, 168, 6, 192, 192, 142, 160, 62, 116, 98, 150, 75, 108, 33, 47, 169, 222, 78, 22, 97, 226, 131, 242, 194, 140, 238, 17, 13, 24, 87, 55, 111, 163, 56, 222, 60, 62, 203, 78, 180, 241, 66, 39, 95, 156, 8, 196, 254, 61, 140, 82, 247, 194, 37, 10, 173, 141, 91, 85, 13, 245, 118, 53, 135, 153, 110, 32, 228, 60, 38, 177, 128, 201, 72, 115, 214, 89, 170, 202, 237, 211, 84, 111, 16, 90, 0, 106, 6, 33, 73, 219, 26, 128, 78, 245, 198, 8, 244, 15, 230, 16, 172, 38, 161, 13, 163, 102, 16, 122, 77, 5, 147, 96, 89, 112, 198, 148, 208, 240, 218, 251, 112, 34, 224, 80, 123, 113, 12, 163, 211, 111, 162, 172, 38, 14, 153, 72, 102, 8, 165, 78, 42, 67, 188, 215, 55, 155, 67, 201, 17, 216, 230, 80, 19, 191, 57, 212, 33, 247, 3, 105, 29, 102, 135, 59, 233, 231, 67, 83, 26, 196, 146, 30, 89, 141, 135, 132, 56, 141, 242, 81, 146, 238, 102, 120, 53, 238, 125, 63, 185, 160, 235, 26, 153, 26, 159, 96, 74, 201, 116, 244, 236, 222, 213, 132, 187, 182, 61, 89, 18, 37, 79, 14, 242, 165, 103, 60, 3, 34, 207, 117, 248, 241, 46, 143, 77, 40, 178, 7, 60, 254, 120, 151, 181, 26, 130, 156, 153, 39, 204, 138, 105, 38, 18, 30, 238, 99, 156, 228, 201, 0, 185, 216, 26, 116, 193, 0, 69, 1, 178, 29, 22, 193, 131, 125, 153, 125, 188, 43, 66, 117, 64, 217, 222, 193, 56, 74, 79, 0, 224, 86, 146, 158, 60, 89, 138, 0, 194, 42, 245, 120, 196, 22, 72, 55, 105, 194, 9, 213, 26, 111, 205, 26, 15, 42, 138, 158, 186, 241, 41, 69, 15, 243, 243, 170, 232, 44, 22, 251, 195, 209, 15, 137, 114, 87, 69, 84, 53, 8, 45, 0, 157, 133, 112, 30, 184, 49, 62, 195, 4, 145, 40, 115, 222, 148, 236, 200, 236, 150, 205, 221, 248, 218, 214, 38, 145, 13, 145, 20, 157, 166, 184, 13, 218, 52, 214, 40, 210, 129, 177, 1, 223, 218, 192, 0, 10, 117, 218, 254, 144, 143, 219, 52, 133, 99, 43, 115, 218, 110, 97, 97, 139, 198, 34, 56, 150, 211, 126, 83, 148, 107, 154, 177, 181, 160, 183, 194, 2, 188, 44, 183, 17, 201, 21, 153, 255, 6, 246, 243, 149, 41, 113, 117, 8, 16, 114, 119, 77, 92, 162, 1, 15, 244, 106, 7, 193, 228, 108, 245, 153, 39, 10, 47, 115, 143, 0, 134, 39, 67, 49, 58, 233, 60, 105, 167, 186, 211, 108, 90, 196, 49, 230, 54, 33, 253, 72, 85, 216, 84, 61, 177, 4, 15, 115, 75, 217, 104, 105, 249, 38, 120, 37, 159, 215, 76, 11, 36, 96, 175, 139, 241, 165, 142, 1, 22, 2, 176, 6, 1, 24, 128, 207, 55, 255, 250, 224, 124, 132, 79, 97, 92, 31, 146, 143, 69, 194, 69, 96, 27, 60, 102, 129, 155, 35, 195, 138, 7, 108, 223, 61, 96, 122, 22, 74, 23, 2, 31, 170, 219, 162, 177, 195, 4, 217, 8, 149, 211, 100, 128, 151, 127, 116, 246, 200, 142, 2, 113, 80, 135, 190, 192, 182, 34, 8, 169, 242, 183, 100, 201, 55, 139, 210, 166, 91, 43, 80, 135, 21, 143, 215, 89, 59, 35, 142, 168, 189, 120, 141, 8, 53, 28, 64, 171, 1, 85, 52, 228, 107, 228, 37, 110, 215, 106, 144, 90, 48, 43, 95, 105, 179, 65, 159, 39, 185, 113, 11, 157, 85, 149, 175, 116, 115, 76, 37, 255, 53, 207, 76, 189, 118, 104, 180, 237, 74, 36, 142, 95, 173, 222, 37, 145, 121, 213, 103, 118, 198, 154, 89, 158, 205, 85, 25, 49, 122, 225, 203, 92, 253, 93, 51, 114, 123, 24, 86, 223, 34, 109, 176, 33, 110, 56, 137, 140, 181, 68, 195, 76, 204, 168, 254, 238, 96, 214, 86, 155, 123, 122, 206, 164, 178, 181, 67, 107, 25, 250, 237, 182, 179, 51, 255, 22, 121, 150, 61, 217, 129, 137, 181, 103, 67, 114, 86, 223, 154, 188, 49, 21, 133, 102, 249, 107, 110, 26, 128, 46, 110, 128, 2, 140, 161, 116, 88, 115, 248, 160, 179, 139, 166, 236, 221, 221, 242, 113, 35, 107, 81, 127, 254, 221, 173, 56, 0, 165, 150, 220, 152, 142, 57, 30, 125, 209, 249, 207, 185, 87, 29, 123, 185, 185, 101, 23, 131, 185, 189, 117, 111, 228, 11, 55, 181, 234, 221, 153, 68, 167, 53, 127, 176, 60, 164, 214, 28, 138, 201, 53, 231, 213, 67, 179, 245, 220, 107, 142, 189, 220, 220, 154, 139, 193, 52, 174, 249, 7, 181, 184, 11, 218, 210, 45, 232, 228, 174, 126, 60, 130, 17, 170, 31, 43, 247, 225, 151, 227, 45, 79, 251, 200, 11, 207, 120, 150, 132, 123, 204, 30, 109, 44, 12, 214, 6, 72, 53, 83, 207, 117, 90, 208, 71, 244, 130, 62, 242, 44, 232, 35, 115, 65, 31, 93, 111, 65, 31, 221, 232, 130, 62, 186, 133, 5, 125, 112, 227, 11, 250, 232, 150, 23, 20, 7, 73, 173, 40, 27, 60, 181, 164, 162, 65, 104, 1, 152, 123, 81, 89, 71, 55, 183, 170, 114, 56, 183, 183, 172, 143, 110, 96, 85, 21, 18, 111, 112, 89, 181, 12, 66, 165, 200, 238, 216, 63, 131, 127, 22, 208, 157, 19, 159, 182, 175, 102, 161, 179, 10, 42, 13, 100, 112, 166, 178, 65, 162, 77, 58, 186, 106, 245, 88, 154, 8, 245, 99, 181, 202, 19, 73, 156, 90, 34, 9, 164, 65, 71, 175, 100, 90, 73, 162, 254, 174, 128, 107, 52, 144, 133, 196, 97, 37, 96, 133, 90, 95, 132, 203, 134, 192, 220, 66, 109, 146, 75, 7, 182, 236, 54, 212, 7, 214, 17, 122, 233, 25, 250, 29, 68, 254, 128, 103, 87, 13, 125, 107, 248, 213, 87, 52, 54, 237, 141, 233, 32, 212, 68, 142, 220, 150, 213, 207, 54, 113, 166, 36, 173, 84, 8, 114, 57, 133, 68, 138, 61, 32, 119, 193, 44, 148, 202, 33, 105, 191, 59, 141, 169, 66, 43, 225, 131, 84, 156, 39, 229, 225, 49, 171, 40, 3, 112, 183, 74, 114, 51, 64, 90, 215, 244, 88, 103, 216, 235, 89, 151, 136, 226, 8, 64, 147, 231, 68, 239, 230, 202, 243, 58, 161, 168, 219, 204, 144, 52, 206, 119, 26, 37, 169, 195, 247, 248, 187, 230, 224, 36, 190, 44, 250, 90, 93, 84, 152, 20, 213, 92, 78, 112, 46, 246, 231, 15, 39, 159, 4, 184, 176, 129, 143, 154, 209, 178, 210, 232, 108, 35, 42, 142, 15, 50, 35, 146, 152, 94, 106, 46, 188, 254, 37, 52, 90, 55, 47, 122, 181, 150, 61, 196, 216, 226, 176, 234, 194, 162, 71, 128, 43, 243, 19, 233, 99, 114, 146, 1, 137, 17, 137, 242, 80, 107, 215, 125, 52, 133, 4, 238, 142, 101, 125, 156, 77, 77, 236, 176, 18, 103, 28, 172, 52, 84, 45, 186, 143, 225, 144, 131, 37, 244, 74, 60, 43, 153, 227, 127, 230, 124, 236, 24, 180, 176, 106, 142, 218, 108, 7, 88, 245, 156, 192, 242, 198, 25, 9, 56, 26, 104, 217, 123, 138, 123, 18, 50, 184, 225, 220, 212, 228, 223, 227, 248, 61, 115, 215, 190, 205, 49, 117, 214, 90, 206, 92, 7, 85, 77, 220, 188, 86, 126, 209, 121, 175, 227, 123, 168, 103, 222, 218, 183, 110, 243, 230, 49, 198, 170, 149, 166, 212, 180, 226, 153, 92, 228, 215, 192, 135, 101, 241, 231, 203, 44, 23, 89, 185, 117, 8, 110, 83, 153, 217, 220, 25, 170, 194, 235, 250, 187, 173, 119, 239, 247, 182, 159, 111, 109, 238, 238, 110, 126, 88, 254, 228, 122, 59, 194, 25, 193, 90, 170, 62, 121, 190, 15, 189, 231, 5, 53, 176, 5, 222, 101, 155, 188, 31, 55, 159, 209, 219, 205, 33, 66, 47, 42, 151, 41, 60, 171, 234, 124, 156, 99, 89, 245, 140, 224, 115, 174, 171, 1, 194, 191, 176, 238, 104, 111, 96, 101, 245, 190, 127, 23, 75, 75, 38, 70, 52, 115, 157, 59, 66, 185, 50, 11, 21, 121, 134, 159, 6, 71, 227, 104, 196, 234, 174, 13, 204, 156, 196, 142, 76, 194, 132, 111, 173, 186, 72, 70, 28, 62, 166, 161, 39, 233, 200, 172, 95, 149, 187, 94, 25, 199, 217, 249, 186, 54, 168, 106, 128, 112, 179, 185, 19, 77, 236, 250, 248, 80, 107, 218, 210, 194, 106, 158, 186, 247, 0, 22, 229, 194, 172, 168, 186, 121, 97, 71, 192, 176, 188, 229, 89, 192, 12, 179, 173, 21, 76, 195, 146, 199, 180, 249, 181, 8, 120, 81, 239, 252, 64, 88, 182, 187, 70, 209, 100, 148, 46, 195, 117, 159, 141, 51, 180, 230, 84, 239, 137, 78, 45, 18, 194, 100, 168, 52, 39, 198, 138, 106, 161, 153, 235, 10, 219, 147, 4, 47, 172, 62, 244, 186, 181, 81, 60, 94, 91, 73, 198, 197, 159, 123, 85, 110, 168, 222, 175, 127, 249, 171, 109, 96, 99, 122, 140, 22, 204, 122, 131, 39, 200, 70, 151, 32, 9, 6, 127, 52, 0, 226, 54, 173, 204, 138, 46, 202, 11, 150, 146, 32, 59, 226, 173, 49, 60, 19, 191, 174, 165, 211, 211, 131, 56, 199, 27, 110, 245, 97, 213, 150, 144, 61, 30, 12, 238, 100, 219, 206, 167, 197, 112, 221, 33, 120, 136, 8, 247, 24, 245, 130, 139, 235, 244, 146, 239, 63, 185, 241, 48, 174, 5, 98, 171, 111, 45, 56, 110, 94, 207, 138, 35, 244, 70, 210, 65, 216, 94, 34, 247, 109, 183, 174, 54, 247, 95, 200, 217, 105, 70, 6, 173, 48, 239, 187, 172, 232, 39, 130, 155, 177, 19, 68, 54, 8, 171, 182, 214, 232, 140, 245, 88, 67, 155, 249, 3, 216, 250, 199, 34, 253, 220, 27, 199, 229, 38, 56, 19, 179, 73, 123, 77, 9, 95, 21, 253, 32, 37, 59, 137, 118, 207, 197, 87, 42, 42, 44, 255, 210, 144, 158, 104, 28, 159, 197, 50, 72, 40, 108, 38, 106, 31, 177, 158, 131, 103, 79, 131, 197, 111, 151, 225, 231, 67, 152, 146, 42, 249, 14, 75, 30, 232, 37, 127, 192, 146, 251, 122, 201, 247, 203, 210, 215, 42, 196, 127, 180, 124, 39, 32, 83, 244, 49, 53, 89, 242, 116, 229, 113, 242, 228, 233, 195, 199, 201, 55, 223, 80, 52, 117, 16, 229, 93, 204, 28, 161, 186, 73, 21, 136, 139, 69, 40, 237, 217, 196, 151, 4, 79, 158, 114, 20, 132, 85, 171, 186, 171, 60, 7, 206, 237, 34, 143, 99, 76, 187, 205, 99, 197, 125, 23, 124, 19, 36, 255, 248, 96, 54, 185, 216, 55, 195, 28, 139, 69, 48, 82, 38, 70, 121, 171, 200, 98, 92, 20, 97, 87, 74, 145, 47, 144, 150, 44, 100, 133, 88, 203, 10, 171, 78, 255, 129, 155, 132, 88, 107, 144, 79, 83, 212, 195, 200, 164, 125, 226, 231, 158, 158, 4, 25, 99, 193, 29, 37, 41, 179, 12, 35, 90, 174, 106, 93, 89, 205, 61, 93, 194, 145, 152, 15, 185, 140, 194, 91, 85, 5, 53, 221, 169, 74, 70, 135, 178, 148, 238, 106, 18, 231, 5, 55, 221, 229, 6, 182, 252, 231, 30, 144, 15, 108, 188, 50, 137, 198, 69, 93, 151, 178, 181, 222, 33, 1, 194, 49, 77, 209, 64, 128, 228, 144, 71, 163, 88, 40, 196, 245, 243, 74, 124, 97, 114, 156, 8, 162, 206, 76, 220, 212, 185, 101, 84, 88, 245, 0, 37, 59, 37, 101, 204, 129, 95, 186, 196, 36, 231, 63, 76, 208, 140, 142, 199, 254, 157, 0, 189, 198, 111, 56, 177, 13, 240, 227, 222, 148, 125, 221, 59, 45, 200, 182, 219, 48, 108, 95, 75, 12, 135, 229, 107, 247, 18, 36, 236, 105, 30, 251, 154, 30, 241, 207, 190, 214, 187, 113, 81, 250, 154, 150, 240, 205, 106, 87, 161, 167, 154, 46, 180, 125, 203, 152, 220, 32, 41, 94, 38, 105, 2, 162, 187, 129, 139, 16, 214, 194, 68, 142, 193, 189, 44, 152, 219, 156, 164, 41, 136, 2, 67, 18, 158, 68, 88, 29, 52, 129, 28, 15, 192, 10, 117, 18, 166, 134, 204, 58, 176, 136, 53, 15, 76, 129, 80, 9, 80, 226, 215, 7, 237, 207, 211, 120, 106, 19, 53, 43, 219, 43, 146, 207, 49, 41, 138, 105, 159, 87, 249, 170, 189, 6, 246, 109, 127, 66, 217, 111, 25, 228, 144, 149, 101, 122, 253, 4, 93, 20, 86, 223, 85, 49, 209, 179, 250, 104, 247, 171, 62, 52, 245, 138, 40, 97, 6, 218, 146, 153, 48, 66, 139, 89, 137, 237, 238, 77, 180, 218, 153, 30, 29, 37, 23, 110, 219, 189, 130, 127, 168, 7, 177, 29, 177, 156, 202, 246, 134, 140, 135, 123, 19, 252, 130, 210, 189, 219, 192, 11, 237, 125, 28, 21, 204, 41, 177, 130, 199, 242, 207, 239, 229, 252, 131, 11, 142, 183, 112, 243, 129, 73, 220, 243, 43, 251, 222, 81, 54, 198, 96, 125, 6, 55, 3, 65, 212, 173, 18, 234, 29, 240, 47, 47, 69, 91, 162, 182, 117, 215, 195, 150, 194, 108, 153, 1, 16, 192, 244, 34, 59, 75, 161, 123, 64, 130, 136, 118, 25, 23, 34, 143, 101, 143, 232, 225, 61, 63, 202, 244, 30, 244, 34, 127, 15, 242, 92, 108, 238, 65, 158, 93, 70, 31, 70, 97, 77, 47, 234, 56, 108, 236, 71, 187, 217, 153, 191, 253, 208, 171, 19, 164, 17, 250, 191, 224, 198, 221, 129, 125, 43, 160, 171, 223, 22, 116, 97, 192, 109, 51, 144, 144, 0, 41, 88, 89, 33, 32, 202, 159, 141, 0, 229, 94, 166, 96, 226, 134, 120, 133, 201, 157, 3, 227, 167, 31, 5, 6, 85, 174, 5, 251, 252, 12, 40, 128, 53, 162, 208, 30, 36, 105, 112, 239, 202, 83, 125, 182, 196, 220, 76, 220, 22, 75, 127, 204, 179, 115, 254, 78, 177, 244, 68, 170, 185, 158, 45, 81, 104, 221, 146, 140, 67, 12, 88, 253, 246, 143, 184, 226, 80, 242, 246, 236, 129, 203, 7, 70, 94, 48, 203, 232, 116, 226, 66, 21, 167, 165, 203, 23, 216, 105, 66, 112, 31, 88, 24, 34, 68, 202, 233, 132, 14, 102, 197, 62, 109, 136, 156, 197, 241, 57, 139, 50, 42, 234, 83, 153, 217, 139, 248, 16, 121, 63, 75, 244, 145, 103, 32, 187, 245, 205, 108, 51, 162, 167, 37, 96, 229, 70, 232, 73, 77, 157, 54, 226, 9, 81, 0, 206, 147, 224, 91, 164, 241, 81, 156, 3, 59, 13, 226, 3, 126, 97, 219, 199, 124, 233, 135, 197, 172, 128, 170, 153, 21, 239, 217, 66, 163, 27, 53, 90, 77, 135, 37, 47, 56, 140, 48, 217, 55, 119, 94, 96, 145, 164, 209, 117, 199, 140, 33, 77, 106, 34, 234, 187, 177, 150, 215, 175, 74, 80, 96, 112, 169, 92, 229, 24, 177, 158, 158, 53, 106, 64, 58, 9, 136, 94, 2, 125, 68, 78, 38, 247, 57, 150, 164, 30, 125, 68, 47, 221, 144, 199, 241, 143, 12, 67, 108, 197, 170, 160, 126, 47, 110, 139, 163, 217, 183, 21, 89, 77, 126, 178, 234, 231, 33, 47, 169, 7, 253, 158, 58, 168, 245, 187, 134, 219, 109, 165, 65, 218, 202, 210, 17, 123, 242, 42, 232, 163, 17, 78, 108, 242, 40, 48, 203, 213, 1, 228, 200, 230, 18, 60, 211, 242, 114, 157, 50, 217, 73, 255, 14, 217, 13, 102, 102, 166, 250, 161, 184, 58, 123, 10, 120, 155, 149, 192, 72, 45, 118, 102, 125, 117, 114, 91, 59, 163, 4, 138, 59, 26, 199, 23, 84, 178, 108, 225, 216, 105, 181, 8, 61, 33, 57, 133, 108, 117, 199, 119, 156, 242, 55, 229, 236, 232, 72, 153, 222, 168, 51, 31, 206, 136, 232, 164, 76, 206, 122, 110, 208, 76, 123, 66, 187, 44, 105, 20, 85, 234, 50, 36, 62, 166, 89, 240, 235, 95, 254, 51, 248, 28, 39, 192, 252, 198, 217, 9, 176, 165, 224, 254, 195, 227, 253, 214, 251, 160, 138, 69, 107, 252, 212, 179, 247, 220, 233, 211, 43, 77, 226, 34, 244, 108, 137, 105, 241, 166, 24, 97, 171, 122, 132, 223, 22, 186, 205, 113, 92, 3, 153, 22, 32, 111, 248, 21, 251, 225, 129, 70, 70, 221, 157, 2, 197, 237, 60, 30, 38, 57, 155, 244, 98, 153, 45, 98, 160, 226, 162, 71, 108, 9, 223, 196, 122, 59, 113, 126, 6, 130, 78, 5, 38, 40, 179, 224, 213, 238, 238, 246, 206, 227, 96, 10, 163, 98, 127, 6, 32, 91, 96, 143, 152, 168, 140, 129, 227, 81, 78, 70, 131, 222, 237, 77, 252, 142, 179, 83, 145, 117, 248, 39, 51, 72, 210, 195, 241, 116, 24, 23, 125, 107, 233, 67, 131, 93, 214, 181, 115, 9, 164, 125, 91, 63, 30, 123, 97, 216, 101, 65, 58, 162, 148, 12, 183, 88, 103, 39, 197, 64, 46, 230, 236, 93, 217, 113, 251, 67, 15, 62, 155, 99, 90, 71, 7, 108, 173, 36, 69, 196, 46, 242, 120, 246, 138, 111, 218, 70, 114, 86, 59, 164, 229, 202, 2, 131, 197, 164, 97, 110, 2, 126, 27, 13, 199, 18, 131, 182, 15, 241, 152, 114, 16, 237, 105, 115, 12, 43, 25, 91, 139, 163, 73, 114, 56, 171, 88, 168, 8, 37, 119, 174, 20, 133, 239, 128, 50, 242, 100, 72, 68, 167, 87, 135, 243, 116, 12, 215, 179, 241, 56, 154, 48, 215, 173, 22, 199, 176, 212, 81, 42, 46, 75, 244, 71, 29, 156, 85, 45, 234, 224, 172, 190, 18, 15, 69, 135, 98, 128, 248, 234, 101, 14, 153, 68, 167, 8, 171, 87, 200, 74, 74, 32, 208, 122, 89, 211, 186, 129, 250, 101, 132, 233, 254, 180, 142, 194, 250, 41, 188, 200, 134, 151, 46, 186, 95, 48, 231, 103, 139, 138, 245, 113, 172, 9, 138, 69, 210, 229, 68, 92, 223, 141, 140, 192, 67, 151, 91, 27, 216, 234, 232, 215, 191, 253, 204, 250, 249, 245, 111, 255, 77, 245, 178, 57, 76, 202, 234, 108, 21, 191, 154, 6, 47, 246, 159, 79, 108, 169, 36, 190, 157, 233, 233, 105, 148, 219, 16, 188, 132, 68, 145, 93, 61, 102, 68, 7, 46, 106, 196, 7, 146, 153, 200, 81, 213, 72, 94, 53, 125, 113, 1, 200, 149, 8, 224, 176, 82, 90, 36, 252, 27, 61, 254, 21, 247, 254, 1, 10, 94, 167, 147, 105, 169, 59, 204, 99, 161, 203, 109, 233, 30, 237, 251, 138, 49, 141, 253, 77, 56, 51, 226, 49, 80, 3, 92, 49, 199, 24, 87, 25, 7, 48, 195, 108, 100, 42, 34, 114, 184, 111, 165, 209, 107, 245, 250, 131, 47, 139, 244, 219, 143, 173, 17, 51, 104, 212, 40, 27, 84, 177, 247, 245, 103, 33, 158, 175, 51, 231, 184, 112, 62, 172, 16, 107, 241, 67, 62, 102, 56, 68, 82, 209, 149, 128, 7, 81, 17, 239, 177, 8, 67, 154, 254, 79, 222, 153, 68, 27, 197, 148, 141, 22, 148, 190, 38, 207, 202, 12, 24, 128, 200, 71, 101, 117, 53, 17, 95, 169, 174, 204, 150, 102, 135, 178, 29, 53, 43, 73, 26, 118, 95, 138, 136, 168, 105, 201, 70, 102, 55, 178, 9, 169, 244, 67, 210, 129, 229, 173, 38, 38, 30, 178, 68, 121, 37, 94, 235, 245, 20, 124, 161, 74, 176, 218, 144, 10, 175, 162, 96, 67, 83, 26, 47, 241, 187, 58, 23, 169, 141, 182, 205, 217, 128, 65, 70, 70, 153, 77, 70, 212, 155, 218, 29, 235, 77, 173, 205, 243, 25, 108, 11, 61, 253, 142, 47, 242, 22, 139, 218, 163, 191, 136, 54, 71, 253, 113, 18, 184, 104, 65, 127, 14, 77, 115, 222, 202, 254, 171, 218, 118, 166, 66, 49, 32, 30, 101, 205, 26, 254, 24, 47, 174, 128, 139, 6, 15, 181, 98, 160, 22, 239, 133, 241, 42, 46, 210, 194, 29, 100, 40, 130, 194, 244, 30, 183, 150, 183, 169, 104, 194, 190, 52, 117, 24, 174, 95, 232, 138, 65, 190, 239, 11, 150, 240, 46, 29, 95, 74, 225, 37, 188, 193, 44, 169, 69, 68, 249, 75, 2, 99, 16, 185, 72, 117, 14, 34, 56, 247, 154, 136, 113, 72, 69, 207, 117, 184, 212, 29, 109, 252, 161, 54, 2, 201, 127, 48, 90, 30, 235, 44, 108, 197, 137, 124, 224, 36, 119, 225, 78, 242, 30, 78, 132, 227, 197, 235, 25, 41, 211, 24, 76, 200, 215, 141, 228, 46, 178, 27, 155, 9, 217, 65, 96, 92, 150, 96, 67, 87, 135, 190, 197, 36, 140, 201, 241, 168, 170, 67, 53, 57, 171, 106, 19, 47, 169, 69, 156, 203, 21, 84, 55, 36, 239, 161, 67, 212, 8, 63, 45, 222, 135, 70, 58, 198, 49, 72, 198, 185, 169, 143, 152, 83, 65, 36, 233, 88, 105, 168, 228, 152, 137, 23, 26, 98, 180, 117, 71, 130, 15, 81, 138, 227, 203, 174, 232, 99, 2, 41, 96, 165, 46, 81, 164, 47, 152, 15, 103, 135, 183, 18, 201, 135, 200, 112, 209, 145, 23, 234, 249, 78, 219, 177, 61, 76, 72, 218, 42, 84, 239, 60, 163, 169, 137, 186, 123, 13, 142, 28, 16, 39, 92, 211, 153, 136, 214, 14, 172, 65, 159, 148, 12, 187, 204, 9, 207, 182, 128, 169, 204, 153, 145, 93, 27, 189, 195, 53, 211, 59, 176, 46, 150, 112, 10, 181, 41, 85, 27, 3, 104, 235, 21, 209, 162, 76, 73, 203, 78, 34, 37, 249, 137, 1, 90, 11, 254, 105, 231, 221, 219, 1, 51, 10, 232, 151, 236, 78, 177, 138, 141, 108, 98, 209, 224, 153, 177, 223, 120, 42, 78, 180, 116, 230, 33, 186, 153, 122, 138, 201, 58, 88, 136, 34, 180, 18, 172, 69, 129, 19, 200, 70, 218, 110, 15, 149, 0, 201, 254, 94, 19, 182, 11, 120, 90, 172, 219, 5, 123, 178, 74, 21, 170, 218, 5, 57, 209, 204, 6, 164, 161, 128, 165, 88, 226, 44, 51, 207, 81, 219, 38, 47, 6, 121, 94, 221, 5, 40, 251, 134, 185, 104, 11, 38, 15, 55, 36, 36, 173, 69, 124, 56, 74, 48, 77, 12, 82, 91, 137, 111, 98, 56, 153, 217, 189, 43, 54, 70, 168, 21, 252, 250, 239, 255, 17, 240, 159, 51, 126, 95, 154, 177, 123, 19, 107, 110, 109, 55, 217, 156, 13, 50, 93, 138, 122, 51, 22, 37, 15, 231, 3, 144, 68, 204, 60, 248, 85, 1, 170, 37, 107, 54, 76, 141, 127, 96, 35, 111, 176, 108, 68, 3, 6, 101, 119, 69, 194, 171, 202, 142, 126, 149, 167, 125, 212, 237, 146, 86, 181, 68, 138, 226, 139, 97, 208, 177, 202, 87, 110, 118, 91, 82, 101, 143, 192, 227, 151, 103, 91, 172, 187, 231, 236, 239, 62, 151, 5, 111, 80, 166, 20, 0, 255, 222, 143, 64, 62, 76, 121, 181, 196, 188, 210, 74, 22, 235, 114, 161, 55, 19, 151, 122, 104, 194, 125, 31, 245, 209, 7, 49, 170, 53, 253, 221, 100, 196, 115, 96, 199, 165, 245, 158, 130, 229, 217, 100, 82, 246, 30, 215, 245, 213, 152, 58, 220, 12, 227, 229, 39, 172, 155, 222, 21, 207, 79, 196, 21, 243, 230, 246, 133, 247, 26, 155, 164, 73, 113, 140, 230, 32, 206, 153, 125, 71, 4, 70, 239, 247, 240, 115, 112, 158, 228, 39, 64, 94, 199, 193, 65, 28, 51, 59, 18, 184, 88, 7, 207, 15, 112, 120, 211, 162, 24, 50, 86, 10, 100, 137, 225, 228, 227, 116, 141, 138, 190, 54, 15, 42, 54, 5, 196, 170, 167, 131, 56, 135, 81, 204, 39, 23, 92, 247, 126, 136, 65, 163, 247, 96, 246, 189, 107, 230, 40, 249, 114, 123, 251, 255, 182, 168, 210, 44, 87, 92, 147, 1, 113, 97, 193, 166, 61, 73, 231, 101, 173, 164, 128, 255, 245, 45, 225, 69, 65, 170, 201, 96, 209, 192, 169, 154, 4, 131, 142, 201, 65, 230, 226, 79, 190, 89, 124, 241, 163, 27, 54, 193, 40, 230, 119, 14, 77, 79, 94, 195, 202, 88, 213, 69, 173, 110, 197, 215, 198, 191, 252, 15, 38, 130, 161, 121, 215, 117, 57, 7, 31, 232, 239, 228, 42, 44, 85, 86, 62, 157, 185, 103, 139, 41, 253, 143, 87, 41, 237, 105, 216, 77, 69, 60, 167, 146, 216, 201, 195, 93, 243, 42, 234, 21, 99, 140, 231, 202, 154, 39, 74, 25, 220, 131, 124, 57, 188, 246, 219, 161, 243, 72, 216, 249, 1, 143, 182, 44, 186, 214, 59, 163, 245, 162, 56, 31, 103, 209, 55, 230, 40, 230, 59, 242, 239, 65, 199, 178, 37, 120, 195, 151, 103, 120, 186, 167, 160, 116, 11, 108, 229, 49, 88, 249, 11, 106, 205, 66, 23, 82, 167, 76, 228, 154, 111, 225, 211, 224, 142, 254, 91, 11, 134, 109, 250, 160, 115, 43, 76, 244, 144, 3, 142, 183, 29, 93, 50, 147, 95, 111, 82, 26, 142, 206, 200, 138, 219, 170, 74, 173, 144, 177, 90, 229, 208, 104, 218, 28, 248, 196, 122, 88, 224, 79, 9, 118, 248, 90, 172, 128, 87, 84, 119, 48, 88, 74, 12, 70, 84, 14, 141, 166, 141, 131, 209, 244, 102, 212, 8, 118, 240, 142, 67, 224, 131, 93, 136, 8, 132, 136, 234, 161, 217, 186, 113, 20, 250, 53, 88, 220, 182, 60, 195, 201, 38, 212, 104, 178, 9, 57, 152, 108, 162, 143, 37, 155, 204, 49, 148, 108, 66, 143, 228, 37, 187, 188, 184, 99, 225, 229, 196, 104, 84, 131, 208, 2, 208, 56, 34, 253, 158, 228, 194, 245, 153, 36, 206, 179, 240, 29, 30, 210, 27, 227, 115, 216, 84, 206, 196, 104, 162, 39, 242, 204, 104, 207, 23, 90, 159, 164, 55, 123, 142, 222, 244, 41, 122, 205, 51, 244, 102, 79, 208, 153, 187, 76, 237, 225, 118, 88, 58, 25, 84, 229, 38, 76, 162, 186, 209, 129, 132, 80, 111, 247, 181, 246, 187, 150, 170, 230, 145, 156, 8, 55, 130, 58, 164, 48, 247, 240, 91, 194, 136, 95, 74, 108, 137, 16, 221, 50, 108, 30, 188, 252, 220, 163, 163, 228, 219, 124, 157, 221, 174, 52, 190, 46, 127, 55, 243, 117, 207, 5, 146, 138, 4, 24, 151, 27, 241, 25, 203, 95, 55, 142, 251, 109, 195, 0, 14, 227, 179, 197, 44, 29, 95, 106, 182, 173, 24, 229, 224, 25, 70, 229, 212, 45, 214, 101, 114, 26, 55, 247, 48, 75, 40, 178, 110, 68, 199, 211, 10, 221, 119, 103, 128, 100, 71, 189, 80, 69, 54, 251, 212, 0, 133, 122, 87, 206, 74, 138, 225, 161, 234, 193, 50, 179, 53, 1, 170, 158, 194, 106, 28, 117, 192, 104, 211, 93, 143, 20, 204, 19, 41, 156, 25, 7, 190, 40, 51, 15, 251, 170, 98, 168, 53, 234, 192, 22, 97, 173, 89, 78, 1, 185, 218, 21, 30, 69, 18, 4, 199, 215, 94, 133, 98, 81, 211, 134, 235, 128, 157, 39, 19, 163, 78, 226, 141, 150, 213, 194, 167, 52, 179, 130, 147, 62, 1, 179, 115, 140, 237, 25, 171, 82, 75, 192, 209, 42, 135, 70, 211, 54, 193, 48, 155, 102, 75, 202, 199, 44, 40, 5, 200, 248, 246, 8, 181, 114, 115, 140, 102, 131, 208, 2, 48, 199, 177, 53, 140, 13, 34, 103, 40, 109, 78, 205, 77, 110, 13, 92, 46, 199, 100, 92, 144, 50, 169, 201, 150, 241, 18, 54, 54, 127, 220, 91, 127, 183, 177, 105, 158, 99, 146, 198, 93, 101, 2, 157, 153, 29, 80, 16, 48, 27, 255, 132, 89, 21, 184, 245, 91, 188, 86, 180, 94, 71, 171, 145, 206, 214, 26, 226, 79, 213, 100, 80, 206, 3, 134, 254, 150, 67, 183, 60, 47, 220, 251, 160, 165, 233, 51, 163, 161, 185, 169, 189, 234, 226, 206, 169, 180, 51, 168, 141, 126, 179, 185, 251, 254, 245, 250, 30, 252, 247, 252, 131, 222, 236, 19, 210, 203, 85, 48, 77, 147, 114, 181, 135, 185, 7, 163, 131, 120, 188, 26, 232, 85, 204, 60, 182, 24, 32, 238, 48, 57, 133, 201, 175, 174, 4, 51, 147, 56, 46, 159, 95, 36, 197, 22, 66, 8, 131, 234, 111, 215, 137, 7, 199, 52, 96, 61, 49, 151, 76, 246, 19, 71, 96, 56, 102, 34, 192, 11, 13, 32, 161, 204, 56, 60, 17, 174, 119, 128, 51, 215, 35, 172, 10, 253, 247, 70, 5, 249, 96, 105, 74, 194, 224, 194, 55, 184, 222, 159, 226, 164, 12, 250, 175, 94, 173, 190, 121, 179, 186, 179, 195, 157, 139, 66, 235, 65, 77, 185, 203, 216, 208, 185, 39, 69, 43, 224, 28, 242, 2, 186, 45, 145, 224, 59, 0, 49, 219, 147, 238, 152, 94, 104, 36, 126, 240, 132, 226, 189, 156, 158, 174, 22, 5, 236, 21, 248, 155, 93, 168, 67, 118, 88, 233, 35, 200, 227, 113, 4, 219, 55, 108, 116, 232, 180, 188, 19, 137, 190, 49, 20, 56, 123, 1, 177, 202, 31, 121, 202, 173, 240, 230, 218, 130, 136, 192, 170, 88, 233, 108, 50, 236, 25, 222, 108, 204, 228, 183, 38, 132, 181, 185, 243, 169, 120, 201, 168, 148, 230, 81, 167, 185, 205, 172, 105, 40, 240, 149, 254, 180, 132, 6, 234, 85, 248, 110, 255, 232, 106, 6, 100, 58, 97, 89, 113, 144, 39, 67, 1, 125, 61, 74, 207, 34, 96, 217, 118, 9, 105, 249, 47, 71, 212, 44, 213, 176, 96, 147, 44, 150, 168, 128, 239, 20, 53, 117, 80, 239, 225, 129, 112, 54, 79, 39, 229, 165, 240, 31, 98, 127, 55, 10, 228, 156, 148, 142, 163, 2, 99, 204, 111, 240, 215, 54, 31, 33, 67, 173, 247, 241, 33, 172, 213, 78, 116, 58, 25, 219, 177, 46, 191, 91, 70, 7, 105, 24, 30, 84, 67, 64, 175, 83, 134, 101, 39, 34, 166, 132, 237, 72, 157, 89, 42, 34, 173, 251, 250, 231, 89, 175, 94, 23, 128, 146, 113, 121, 124, 105, 65, 38, 181, 132, 150, 41, 182, 18, 59, 198, 227, 236, 252, 29, 247, 166, 171, 37, 38, 13, 49, 214, 193, 206, 7, 243, 138, 207, 213, 12, 57, 202, 188, 168, 116, 148, 178, 160, 118, 98, 122, 44, 20, 165, 214, 125, 232, 114, 99, 14, 108, 43, 30, 193, 129, 37, 45, 114, 248, 47, 227, 211, 2, 102, 86, 37, 157, 201, 90, 144, 129, 115, 159, 26, 2, 234, 202, 152, 183, 217, 138, 46, 179, 105, 89, 136, 212, 6, 158, 200, 193, 158, 188, 7, 53, 177, 131, 237, 67, 213, 220, 223, 147, 40, 129, 235, 211, 118, 150, 164, 44, 13, 128, 9, 85, 134, 118, 84, 241, 245, 141, 132, 163, 102, 75, 188, 201, 178, 40, 149, 88, 218, 39, 9, 78, 172, 161, 222, 112, 0, 76, 104, 84, 30, 139, 69, 111, 69, 192, 68, 120, 201, 142, 28, 142, 181, 50, 248, 154, 243, 198, 228, 115, 100, 246, 98, 227, 11, 44, 228, 117, 137, 148, 8, 34, 44, 236, 206, 170, 34, 183, 146, 8, 162, 219, 46, 226, 174, 39, 26, 112, 72, 153, 0, 72, 194, 49, 232, 136, 133, 25, 109, 65, 72, 232, 129, 249, 163, 58, 41, 244, 99, 164, 188, 88, 112, 206, 16, 157, 126, 56, 203, 130, 182, 187, 44, 46, 127, 177, 224, 102, 231, 227, 215, 243, 5, 49, 68, 107, 51, 253, 121, 26, 23, 229, 243, 20, 228, 71, 20, 111, 95, 230, 32, 29, 247, 249, 45, 231, 75, 142, 169, 94, 220, 23, 199, 75, 130, 1, 29, 46, 213, 14, 173, 146, 157, 114, 105, 252, 21, 255, 222, 230, 168, 48, 108, 63, 112, 158, 27, 213, 41, 234, 109, 191, 96, 142, 96, 129, 9, 242, 243, 178, 78, 110, 61, 161, 206, 98, 139, 145, 54, 249, 192, 33, 106, 147, 177, 138, 171, 76, 29, 90, 236, 22, 65, 196, 91, 198, 240, 245, 86, 160, 48, 94, 68, 4, 9, 99, 31, 86, 173, 29, 45, 22, 99, 122, 106, 65, 129, 146, 100, 152, 148, 151, 36, 36, 245, 145, 134, 6, 196, 100, 65, 99, 37, 58, 32, 60, 241, 84, 144, 182, 183, 209, 219, 190, 172, 22, 202, 62, 176, 9, 13, 30, 3, 94, 219, 170, 168, 51, 142, 66, 87, 29, 85, 53, 121, 155, 9, 81, 198, 108, 194, 139, 221, 38, 19, 250, 232, 16, 55, 156, 65, 49, 70, 62, 179, 248, 221, 178, 125, 185, 215, 70, 135, 71, 126, 213, 51, 125, 223, 87, 171, 133, 51, 197, 22, 108, 41, 180, 223, 18, 117, 242, 183, 129, 53, 108, 29, 58, 165, 0, 130, 21, 78, 140, 67, 12, 129, 44, 155, 199, 66, 53, 184, 22, 194, 64, 53, 49, 251, 198, 116, 18, 163, 16, 131, 54, 191, 105, 175, 221, 217, 94, 215, 177, 37, 140, 122, 187, 221, 255, 113, 123, 35, 184, 119, 5, 248, 129, 11, 244, 203, 228, 2, 22, 231, 126, 56, 11, 78, 182, 35, 237, 146, 235, 114, 61, 92, 115, 201, 242, 216, 223, 130, 223, 241, 133, 245, 242, 58, 89, 251, 152, 61, 96, 18, 220, 183, 27, 239, 189, 217, 81, 16, 89, 138, 176, 221, 46, 183, 207, 51, 97, 13, 64, 204, 100, 23, 54, 179, 244, 56, 25, 49, 139, 179, 123, 87, 110, 117, 29, 191, 191, 254, 229, 175, 78, 21, 108, 235, 172, 1, 243, 6, 39, 66, 244, 136, 41, 240, 182, 206, 110, 228, 197, 182, 198, 204, 248, 24, 154, 32, 140, 240, 191, 251, 79, 138, 73, 132, 166, 215, 81, 81, 60, 253, 120, 119, 204, 196, 141, 197, 226, 28, 109, 54, 62, 222, 13, 24, 173, 65, 249, 65, 116, 120, 50, 98, 97, 158, 86, 25, 1, 49, 129, 162, 79, 224, 9, 54, 209, 242, 224, 251, 112, 246, 241, 238, 179, 39, 75, 8, 250, 89, 240, 167, 36, 30, 3, 217, 85, 24, 158, 237, 59, 122, 101, 16, 250, 94, 99, 34, 236, 179, 104, 220, 103, 198, 24, 72, 233, 11, 193, 253, 111, 181, 104, 81, 122, 29, 126, 12, 236, 138, 224, 123, 211, 98, 193, 12, 44, 53, 201, 38, 211, 49, 236, 46, 172, 240, 25, 118, 135, 16, 134, 116, 80, 148, 174, 205, 134, 90, 125, 193, 19, 101, 39, 46, 49, 194, 181, 86, 90, 28, 70, 233, 219, 184, 60, 207, 242, 19, 171, 238, 86, 166, 215, 243, 155, 151, 208, 169, 202, 39, 57, 12, 65, 30, 231, 38, 20, 118, 79, 18, 69, 128, 96, 150, 126, 254, 217, 87, 248, 55, 26, 146, 241, 191, 142, 203, 211, 49, 254, 245, 213, 255, 2, 113, 177, 230, 160, 35, 190, 2, 0
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
    if (key.length() == 0) continue;
    String path = cloudDailyPath(key);
    if (path.length() == 0) continue;
    int code = 0;
    String url = buildWebDavUrl(path);
    if (!webdavRequest("HEAD", url, "", nullptr, code, nullptr, CLOUD_TEST_TIMEOUT_MS)) continue;
    if (code >= 200 && code < 300) {
      keys.push_back(key);
    }
  }
  return keys;
}

void handleDailyHistory() {
  if (!enforceAuth()) return;
  if (!cloudConnected()) {
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
  bool cloudOk = cloudConnected();
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
  String sensor = server.hasArg("sensor") ? server.arg("sensor") : "";
  if (sensor.length() == 0 && server.hasArg("metric")) sensor = server.arg("metric");
  if (sensor.length() == 0) sensor = "all";
  String from = server.hasArg("from") ? server.arg("from") : "";
  String to = server.hasArg("to") ? server.arg("to") : "";

  if (sensor == "all") {
    server.send(400, "text/plain", "sensor required");
    return;
  }

  bool cloudOk = cloudConnected();
  if (!cloudOk) {
    String json;
    json.reserve(128);
    json = "{\"cloud\":false,\"hasData\":false,\"message\":\"Cloud not connected\",\"days\":[]}";
    server.send(200, "application/json", json);
#if CLOUD_DIAG
    Serial.println("/api/cloud/daily served");
#endif
    return;
  }

  bool hasData = cloudStatus.lastUploadMs > 0 || cloudStatus.lastUploadedPath.length() > 0;
  if (!hasData) {
    String json;
    json.reserve(128);
    json = "{\"cloud\":true,\"hasData\":false,\"message\":\"No cloud history yet\",\"days\":[]}";
    server.send(200, "application/json", json);
#if CLOUD_DIAG
    Serial.println("/api/cloud/daily served");
#endif
    return;
  }

  std::vector<String> keys;
  if (from.length() > 0 && to.length() > 0) {
    keys = dayKeysBetween(from, to);
  } else {
    keys = recentDayKeys((int)retentionDays());
  }
  String json;
  json.reserve(256);
  json = "{\"cloud\":true,\"hasData\":true,\"sensor\":\"" + sensor + "\",\"from\":\"" + from + "\",\"to\":\"" + to + "\",\"days\":[";
  bool first = true;
  for (const auto &k : keys) {
    DailyPoint p;
    if (!fetchCloudDailyPoint(k, sensor, p)) {
      continue;
    }
    if (!first) json += ",";
    first = false;
    json += "{\"date\":\"" + p.dayKey + "\",\"avg\":" + String((double)p.avg, 2) + ",\"min\":" + String((double)p.min, 2) + ",\"max\":" + String((double)p.max, 2) + "}";
  }
  json += "]}";
  server.send(200, "application/json", json);
#if CLOUD_DIAG
  Serial.println("/api/cloud/daily served");
#endif
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
    if (recordingStartDay.length() == 0) {
      String dayKey = currentDayKey();
      if (dayKey.length() > 0 && dayKey != "unsynced") saveRecordingStartDay(dayKey);
    }
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
  if (action == "grow_end") {
    String path;
    String payload;
    String err;
    if (!buildGrowReport(path, payload, err)) {
      String msg = err.length() > 0 ? err : "report failed";
      server.send(500, "application/json", "{\"ok\":0,\"error\":\"" + msg + "\"}");
      return;
    }
    enqueueCloudJob(path, payload, currentDayKey(), "report", "text/plain", true);
    if (cloudConfig.recording) {
      setRecordingActive(false, "grow end");
      saveCloudConfig("grow end");
      enqueueRecordingEvent("stop", "Grow end");
    }
    clearRecordingStartDay();
    server.send(200, "application/json", "{\"ok\":1,\"path\":\"" + path + "\"}");
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
