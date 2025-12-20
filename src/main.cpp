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
static const char *FIRMWARE_VERSION = "v0.3.4";
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
  return cloudConfig.enabled && cloudConfig.baseUrl.length() > 0;
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
  if (months > 6) months = 6;
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
  if (cloudConfig.retentionMonths > 6) cloudConfig.retentionMonths = 6;
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

String dailyVpdAssessment(float vpdAvg, float vpdMin, float vpdMax) {
  if (isnan(vpdAvg)) return "keine VPD Daten";
  if (!isnan(vpdMin) && !isnan(vpdMax) && (vpdMax - vpdMin) > 0.8f) {
    return "stark schwankend";
  }
  if (vpdAvg > 1.6f) return "eher hoch";
  if (vpdAvg < 0.6f) return "eher niedrig";
  return "im Zielkorridor";
}

String dailyDayConclusion(float tempMin, float tempMax, float humAvg, float vpdAvg, float vpdMin, float vpdMax) {
  if (!isnan(tempMin) && !isnan(tempMax) && (tempMax - tempMin) > 6.0f) {
    return "Temperatur schwankte deutlich";
  }
  if (!isnan(vpdAvg) && vpdAvg > 1.6f) {
    return "VPD zu hoch, Stressrisiko";
  }
  if (!isnan(vpdAvg) && vpdAvg < 0.6f) {
    return "VPD zu niedrig, Schimmelrisiko";
  }
  if (!isnan(vpdMin) && !isnan(vpdMax) && (vpdMax - vpdMin) > 0.8f) {
    return "VPD schwankend, Klima prüfen";
  }
  if (!isnan(humAvg) && humAvg > 75.0f) {
    return "Luftfeuchte hoch, Lüftung prüfen";
  }
  return "Klima heute stabil und unauffällig";
}

String buildDailySummaryText(const String &dayKey) {
  if (dayKey.length() == 0 || dayKey == "unsynced") return "";
  DailyAggregate tempAgg;
  DailyAggregate humAgg;
  DailyAggregate luxAgg;
  DailyAggregate ppfdAgg;
  DailyAggregate co2Agg;
  DailyAggregate vpdAgg;
  bool hasTemp = dailyAggregateFor(dayKey, "temp", tempAgg);
  bool hasHum = dailyAggregateFor(dayKey, "humidity", humAgg);
  bool hasLux = dailyAggregateFor(dayKey, "lux", luxAgg);
  bool hasPpfd = dailyAggregateFor(dayKey, "ppfd", ppfdAgg);
  bool hasCo2 = dailyAggregateFor(dayKey, "co2", co2Agg);
  bool hasVpd = dailyAggregateFor(dayKey, "vpd", vpdAgg);
  if (!hasTemp && !hasHum && !hasVpd && !hasLux && !hasPpfd && !hasCo2) return "";
  float tempAvg = hasTemp && tempAgg.count > 0 ? tempAgg.sum / tempAgg.count : NAN;
  float humAvg = hasHum && humAgg.count > 0 ? humAgg.sum / humAgg.count : NAN;
  float luxAvg = hasLux && luxAgg.count > 0 ? luxAgg.sum / luxAgg.count : NAN;
  float ppfdAvg = hasPpfd && ppfdAgg.count > 0 ? ppfdAgg.sum / ppfdAgg.count : NAN;
  float co2Avg = hasCo2 && co2Agg.count > 0 ? co2Agg.sum / co2Agg.count : NAN;
  float vpdAvg = hasVpd && vpdAgg.count > 0 ? vpdAgg.sum / vpdAgg.count : NAN;
  String text;
  text.reserve(320);
  text += "Datum: " + dayKey + "\n";
  if (hasTemp) {
    text += "Durchschnittstemperatur: " + formatMetricValue(tempAvg, 1) + " °C\n";
    text += "Temp Min / Max: " + formatMetricValue(tempAgg.min, 1) + " / " + formatMetricValue(tempAgg.max, 1) + " °C\n";
  }
  if (hasHum) {
    text += "Durchschnitt Luftfeuchte: " + formatMetricValue(humAvg, 1) + " %\n";
  }
  if (hasLux || hasPpfd) {
    text += "Durchschnitt Licht: " + formatMetricValue(luxAvg, 1) + " Lux / " + formatMetricValue(ppfdAvg, 1) + " PPFD\n";
  }
  if (hasCo2) {
    text += "Durchschnitt CO2: " + formatMetricValue(co2Avg, 0) + " ppm\n";
  }
  if (hasVpd) {
    text += "Durchschnitt VPD: " + formatMetricValue(vpdAvg, 2) + " kPa\n";
    text += "VPD Min / Max: " + formatMetricValue(vpdAgg.min, 2) + " / " + formatMetricValue(vpdAgg.max, 2) + " kPa\n";
    text += "VPD Verlauf: " + dailyVpdAssessment(vpdAvg, vpdAgg.min, vpdAgg.max) + "\n";
  }
  String assessment = dailyDayConclusion(tempAgg.min, tempAgg.max, humAvg, vpdAvg, vpdAgg.min, vpdAgg.max);
  text += "Kurzfazit des Tages: " + assessment + "\n";
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
  String path = cloudRootPath() + "/daily/" + dayKey + "_summary.txt";
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
  31, 139, 8, 0, 129, 44, 71, 105, 2, 255, 236, 189, 219, 114, 27, 73, 146, 40, 248, 222, 95, 145, 197, 170, 110, 0, 85,
  0, 8, 128, 23, 145, 160, 72, 29, 137, 162, 90, 156, 210, 109, 69, 150, 212, 213, 26, 153, 148, 0, 18, 68, 54, 19, 72,
  76, 102, 130, 151, 210, 161, 217, 216, 121, 216, 215, 181, 181, 89, 219, 221, 183, 53, 59, 214, 31, 112, 108, 205, 246,
  60, 245, 211, 169, 63, 233, 47, 89, 119, 143, 75, 198, 53, 1, 82, 170, 234, 106, 219, 157, 158, 18, 145, 113, 241, 240,
  240, 240, 240, 240, 240, 240, 240, 248, 93, 16, 220, 255, 106, 148, 14, 139, 235, 121, 20, 76, 138, 105, 114, 240, 59,
  72, 193, 31, 65, 18, 206, 206, 246, 215, 70, 209, 26, 75, 138, 194, 17, 254, 128, 159, 211, 168, 8, 131, 225, 36, 204,
  242, 168, 216, 95, 251, 225, 244, 73, 107, 103, 45, 88, 87, 51, 103, 225, 52, 218, 95, 187, 136, 163, 203, 121, 154,
  21, 107, 193, 48, 157, 21, 209, 12, 10, 95, 198, 163, 98, 178, 63, 138, 46, 226, 97, 212, 162, 143, 102, 16, 207, 226,
  34, 14, 147, 86, 62, 12, 147, 104, 191, 91, 130, 42, 226, 34, 137, 14, 254, 152, 165, 151, 39, 209, 44, 79, 179, 224,
  162, 211, 222, 104, 111, 222, 95, 103, 25, 172, 80, 94, 92, 139, 223, 65, 208, 207, 210, 180, 8, 62, 65, 123, 73, 154,
  1, 192, 73, 52, 141, 250, 65, 18, 159, 77, 138, 96, 20, 102, 231, 123, 193, 13, 47, 137, 61, 108, 6, 131, 116, 116, 13,
  197, 7, 225, 240, 252, 44, 75, 23, 179, 81, 63, 248, 186, 51, 238, 222, 235, 133, 123, 12, 6, 124, 71, 189, 104, 103,
  220, 217, 11, 166, 241, 172, 53, 137, 16, 84, 63, 232, 118, 58, 191, 47, 97, 113, 40, 99, 232, 100, 107, 28, 78, 227,
  228, 186, 31, 228, 215, 121, 17, 77, 91, 139, 184, 25, 228, 225, 44, 111, 229, 81, 22, 143, 1, 72, 152, 157, 197, 179,
  126, 0, 240, 230, 225, 104, 20, 207, 206, 232, 183, 196, 10, 136, 28, 101, 0, 75, 102, 118, 183, 231, 87, 123, 58, 130,
  221, 110, 119, 167, 119, 15, 18, 211, 171, 86, 62, 9, 71, 233, 37, 192, 8, 122, 243, 171, 0, 202, 6, 217, 217, 32, 172,
  119, 154, 244, 191, 118, 111, 171, 1, 13, 165, 57, 16, 56, 133, 86, 243, 34, 30, 158, 95, 239, 5, 69, 58, 167, 102,
  127, 106, 197, 179, 81, 116, 5, 173, 244, 20, 28, 186, 208, 190, 130, 39, 117, 43, 143, 127, 2, 66, 118, 219, 189, 44,
  154, 42, 16, 179, 40, 9, 139, 248, 34, 42, 107, 183, 89, 23, 90, 48, 104, 0, 102, 20, 231, 243, 36, 188, 238, 143, 147,
  8, 122, 241, 151, 5, 32, 48, 190, 110, 113, 110, 0, 124, 230, 33, 176, 193, 32, 42, 46, 163, 104, 182, 23, 132, 48, 80,
  179, 86, 12, 116, 203, 251, 193, 16, 74, 68, 217, 94, 112, 22, 206, 251, 221, 30, 18, 1, 129, 180, 46, 51, 248, 14,
  240, 95, 165, 77, 0, 56, 179, 90, 83, 161, 169, 192, 118, 16, 150, 172, 154, 68, 35, 168, 73, 156, 200, 155, 225, 99,
  204, 62, 6, 105, 70, 189, 9, 71, 241, 34, 239, 111, 225, 168, 43, 99, 241, 117, 52, 222, 132, 255, 211, 134, 162, 19,
  224, 255, 54, 196, 72, 244, 54, 118, 155, 219, 59, 248, 255, 124, 52, 4, 146, 241, 44, 137, 103, 208, 249, 36, 29, 158,
  171, 93, 73, 210, 197, 168, 165, 97, 213, 81, 177, 234, 104, 216, 179, 210, 48, 136, 241, 48, 44, 210, 108, 101, 26,
  108, 107, 80, 6, 225, 232, 44, 82, 234, 234, 168, 9, 86, 220, 132, 46, 237, 216, 52, 217, 221, 221, 165, 193, 145, 92,
  210, 105, 239, 16, 147, 168, 116, 234, 142, 123, 187, 27, 247, 196, 156, 250, 122, 56, 24, 109, 69, 93, 19, 131, 22,
  136, 5, 125, 50, 126, 221, 237, 12, 118, 119, 186, 178, 158, 152, 154, 70, 189, 203, 48, 155, 25, 21, 199, 91, 187, 81,
  103, 224, 175, 120, 54, 73, 243, 66, 175, 83, 100, 48, 79, 231, 97, 6, 100, 18, 125, 236, 119, 161, 207, 121, 154, 196,
  163, 192, 236, 130, 144, 10, 2, 226, 52, 140, 103, 246, 188, 21, 20, 13, 206, 178, 120, 196, 72, 31, 48, 198, 42, 135,
  48, 204, 70, 166, 8, 42, 103, 56, 97, 17, 56, 208, 208, 7, 129, 3, 53, 165, 134, 38, 32, 248, 248, 85, 10, 136, 114,
  62, 167, 23, 81, 54, 78, 160, 230, 36, 30, 141, 112, 114, 74, 132, 115, 146, 197, 45, 142, 183, 67, 20, 88, 85, 21,
  201, 217, 221, 233, 168, 116, 97, 92, 74, 19, 123, 20, 103, 209, 144, 32, 1, 133, 23, 83, 181, 69, 36, 158, 194, 158,
  54, 49, 49, 161, 5, 76, 14, 217, 69, 212, 98, 245, 115, 236, 205, 60, 10, 139, 122, 184, 40, 210, 214, 56, 46, 154,
  136, 200, 52, 188, 170, 247, 58, 128, 68, 51, 232, 142, 179, 70, 67, 105, 230, 34, 76, 22, 145, 144, 227, 66, 224, 109,
  19, 47, 83, 210, 69, 152, 197, 33, 252, 157, 45, 166, 32, 204, 135, 253, 160, 8, 7, 139, 36, 204, 48, 33, 47, 225, 36,
  225, 32, 74, 84, 116, 249, 60, 98, 34, 181, 69, 178, 119, 71, 159, 50, 65, 167, 189, 75, 237, 8, 24, 241, 108, 190, 0,
  124, 243, 40, 1, 154, 192, 50, 181, 40, 138, 116, 38, 37, 2, 95, 126, 202, 209, 38, 154, 170, 240, 55, 237, 121, 170,
  206, 93, 55, 71, 105, 139, 224, 160, 219, 235, 117, 236, 69, 80, 46, 121, 2, 163, 225, 34, 203, 177, 196, 60, 141, 153,
  120, 17, 45, 204, 210, 89, 164, 3, 69, 161, 2, 212, 58, 67, 132, 96, 146, 213, 187, 189, 206, 40, 58, 107, 6, 95, 247,
  122, 163, 141, 40, 130, 31, 219, 27, 219, 219, 227, 110, 163, 108, 86, 160, 65, 164, 186, 228, 139, 239, 118, 199, 66,
  164, 15, 196, 14, 7, 76, 106, 166, 176, 168, 196, 197, 53, 18, 117, 123, 79, 226, 55, 75, 139, 86, 152, 0, 87, 70, 35,
  101, 200, 245, 85, 42, 96, 236, 72, 140, 69, 212, 114, 46, 72, 254, 101, 8, 161, 29, 4, 223, 34, 11, 37, 180, 170, 170,
  179, 166, 8, 139, 69, 46, 184, 203, 217, 21, 94, 166, 157, 158, 11, 13, 6, 40, 176, 177, 57, 218, 216, 221, 181, 11,
  69, 89, 166, 148, 26, 239, 220, 235, 222, 83, 155, 75, 210, 51, 152, 253, 38, 199, 104, 42, 12, 155, 137, 154, 210,
  178, 136, 91, 211, 116, 150, 210, 178, 220, 12, 78, 158, 60, 135, 143, 214, 235, 232, 12, 185, 188, 25, 60, 143, 102,
  73, 10, 127, 210, 89, 56, 132, 191, 135, 41, 200, 129, 36, 204, 155, 193, 218, 179, 120, 16, 101, 33, 78, 94, 204, 77,
  215, 32, 233, 48, 93, 100, 49, 168, 50, 47, 162, 75, 248, 148, 80, 247, 130, 203, 9, 208, 179, 69, 31, 192, 53, 89,
  212, 50, 215, 114, 208, 43, 11, 85, 176, 40, 66, 73, 224, 78, 19, 184, 172, 51, 12, 103, 23, 97, 110, 118, 214, 83,
  120, 12, 26, 34, 169, 88, 69, 116, 133, 44, 1, 35, 92, 142, 109, 57, 165, 122, 214, 12, 221, 217, 162, 41, 42, 72, 190,
  187, 25, 110, 12, 118, 12, 65, 222, 146, 10, 220, 242, 117, 120, 153, 54, 100, 235, 42, 108, 232, 91, 35, 82, 113, 239,
  170, 178, 108, 15, 238, 245, 118, 58, 85, 42, 75, 183, 115, 175, 217, 237, 110, 54, 187, 61, 169, 180, 56, 80, 104,
  195, 127, 173, 249, 34, 201, 35, 96, 31, 90, 75, 195, 89, 60, 13, 217, 144, 65, 222, 43, 204, 10, 122, 237, 205, 60,
  136, 66, 40, 4, 172, 151, 46, 10, 144, 108, 99, 212, 247, 163, 101, 48, 243, 132, 166, 166, 11, 230, 102, 123, 231,
  110, 48, 227, 81, 18, 121, 96, 222, 91, 6, 241, 63, 157, 71, 215, 227, 12, 182, 53, 121, 89, 233, 83, 208, 249, 61, 46,
  221, 38, 37, 59, 140, 142, 91, 189, 102, 175, 219, 109, 118, 183, 54, 128, 140, 93, 34, 99, 112, 207, 93, 3, 71, 206,
  174, 68, 53, 144, 153, 87, 108, 132, 202, 75, 18, 20, 113, 18, 181, 104, 167, 116, 71, 213, 152, 0, 124, 6, 67, 251,
  248, 153, 47, 84, 131, 20, 36, 247, 212, 80, 69, 97, 247, 8, 75, 107, 11, 155, 118, 45, 46, 14, 109, 131, 212, 54, 46,
  41, 96, 113, 231, 242, 45, 128, 101, 99, 202, 198, 180, 201, 138, 140, 211, 108, 138, 66, 79, 166, 242, 117, 66, 75,
  227, 115, 135, 230, 56, 12, 138, 200, 112, 233, 52, 178, 169, 254, 189, 94, 71, 81, 190, 68, 191, 54, 119, 124, 29,
  235, 79, 16, 26, 74, 32, 129, 87, 159, 253, 68, 237, 229, 199, 122, 11, 120, 161, 177, 167, 161, 130, 235, 192, 198,
  102, 119, 107, 203, 28, 29, 78, 106, 183, 30, 38, 118, 119, 189, 21, 245, 45, 57, 64, 156, 220, 173, 232, 2, 96, 231,
  125, 212, 159, 148, 13, 136, 186, 243, 101, 88, 240, 253, 111, 245, 224, 168, 99, 179, 105, 143, 2, 72, 105, 199, 120,
  149, 169, 46, 138, 139, 149, 190, 187, 167, 144, 82, 161, 36, 78, 136, 74, 101, 116, 107, 123, 101, 101, 84, 18, 71,
  161, 170, 174, 57, 182, 230, 225, 140, 20, 191, 74, 157, 103, 135, 233, 60, 76, 208, 110, 53, 123, 27, 205, 205, 30,
  136, 135, 221, 134, 157, 118, 15, 181, 83, 4, 54, 202, 210, 57, 40, 176, 73, 129, 122, 213, 32, 89, 100, 245, 46, 227,
  17, 208, 35, 6, 231, 113, 209, 170, 44, 100, 109, 101, 88, 59, 155, 59, 205, 238, 246, 70, 179, 187, 179, 137, 66, 190,
  215, 48, 183, 11, 93, 220, 47, 108, 116, 204, 13, 195, 102, 175, 177, 116, 243, 129, 149, 88, 154, 177, 163, 92, 74,
  225, 77, 38, 31, 174, 90, 98, 235, 75, 122, 110, 37, 107, 235, 172, 152, 47, 6, 184, 174, 107, 90, 188, 80, 174, 141,
  205, 39, 97, 36, 56, 161, 189, 225, 158, 171, 109, 168, 148, 132, 243, 156, 52, 75, 133, 3, 119, 137, 115, 20, 94, 218,
  222, 116, 200, 0, 182, 13, 19, 108, 10, 120, 236, 84, 112, 234, 146, 230, 181, 169, 166, 96, 210, 81, 26, 240, 64, 111,
  109, 19, 35, 136, 193, 96, 154, 249, 178, 230, 72, 72, 181, 132, 54, 86, 246, 161, 215, 91, 17, 85, 185, 114, 232, 242,
  190, 179, 172, 186, 67, 60, 26, 24, 235, 168, 73, 246, 8, 7, 192, 223, 11, 92, 180, 227, 89, 30, 17, 101, 4, 79, 114,
  205, 168, 156, 152, 200, 186, 97, 82, 206, 203, 97, 156, 13, 97, 201, 9, 65, 56, 193, 114, 11, 255, 241, 233, 184, 177,
  73, 203, 107, 111, 3, 85, 161, 206, 150, 99, 150, 110, 109, 53, 92, 115, 183, 215, 168, 50, 34, 232, 19, 72, 159, 43,
  246, 2, 155, 23, 89, 84, 12, 39, 142, 21, 86, 100, 56, 37, 182, 152, 36, 93, 149, 5, 55, 96, 248, 46, 226, 60, 30, 196,
  9, 38, 208, 207, 68, 151, 212, 246, 194, 88, 185, 142, 121, 57, 101, 219, 156, 156, 209, 117, 228, 30, 175, 36, 26,
  151, 218, 43, 155, 57, 244, 193, 164, 64, 111, 91, 209, 113, 123, 219, 62, 29, 215, 79, 108, 69, 251, 213, 247, 181,
  114, 91, 235, 148, 83, 171, 232, 56, 170, 246, 164, 73, 65, 83, 195, 118, 90, 95, 186, 59, 13, 141, 242, 186, 250, 161,
  172, 146, 101, 23, 180, 100, 69, 177, 81, 11, 203, 118, 131, 238, 118, 57, 132, 130, 29, 54, 28, 195, 34, 231, 156,
  166, 117, 72, 165, 67, 51, 201, 113, 219, 148, 91, 210, 116, 249, 146, 163, 244, 28, 141, 210, 180, 152, 104, 93, 223,
  104, 184, 208, 8, 135, 40, 223, 181, 185, 239, 147, 146, 146, 163, 242, 139, 179, 114, 59, 180, 163, 110, 135, 118,
  212, 105, 101, 26, 90, 101, 253, 54, 252, 211, 26, 38, 41, 147, 241, 183, 19, 146, 58, 144, 116, 30, 205, 62, 19, 132,
  133, 135, 137, 182, 58, 217, 140, 125, 175, 182, 237, 181, 54, 252, 44, 193, 0, 75, 118, 138, 110, 208, 13, 152, 196,
  112, 138, 17, 217, 244, 40, 186, 104, 205, 96, 243, 44, 237, 14, 210, 202, 106, 175, 181, 138, 49, 74, 83, 240, 191, 6,
  32, 207, 211, 81, 152, 168, 130, 96, 28, 95, 161, 85, 70, 74, 109, 157, 128, 183, 153, 134, 170, 132, 87, 185, 109,
  187, 81, 242, 255, 118, 199, 34, 231, 36, 158, 153, 42, 131, 182, 219, 151, 155, 125, 95, 183, 102, 225, 133, 181, 69,
  146, 26, 99, 105, 48, 98, 166, 14, 5, 136, 110, 206, 71, 40, 134, 169, 143, 143, 11, 23, 42, 59, 82, 177, 90, 197, 236,
  106, 131, 237, 247, 7, 17, 76, 42, 144, 17, 106, 90, 56, 102, 6, 17, 65, 204, 181, 181, 61, 255, 162, 218, 218, 48, 12,
  10, 238, 149, 180, 92, 62, 187, 187, 247, 154, 187, 168, 99, 110, 54, 130, 142, 35, 185, 211, 8, 182, 59, 191, 111,
  120, 180, 24, 58, 19, 172, 179, 1, 172, 92, 162, 180, 93, 131, 178, 113, 51, 88, 154, 177, 212, 52, 190, 106, 193, 170,
  55, 27, 181, 166, 233, 40, 130, 70, 178, 200, 71, 48, 65, 28, 222, 123, 90, 106, 84, 3, 66, 22, 143, 139, 96, 35, 231,
  138, 190, 195, 112, 160, 0, 35, 102, 115, 142, 1, 207, 17, 141, 57, 55, 54, 140, 20, 221, 134, 219, 38, 65, 120, 0, 11,
  103, 233, 212, 179, 179, 220, 24, 213, 91, 192, 64, 77, 212, 6, 155, 64, 117, 5, 94, 176, 69, 54, 6, 79, 45, 172, 180,
  163, 213, 65, 77, 104, 239, 38, 40, 210, 138, 166, 176, 21, 179, 37, 197, 58, 129, 103, 195, 150, 172, 172, 216, 25,
  24, 167, 38, 88, 189, 45, 87, 11, 125, 226, 41, 18, 119, 182, 104, 13, 10, 243, 108, 200, 115, 146, 227, 87, 32, 36,
  192, 203, 120, 28, 183, 164, 37, 247, 22, 54, 21, 93, 5, 214, 45, 3, 115, 110, 75, 42, 185, 138, 165, 116, 151, 155,
  185, 20, 6, 152, 223, 218, 34, 181, 89, 109, 145, 234, 124, 121, 139, 20, 19, 76, 254, 21, 242, 98, 62, 178, 181, 122,
  203, 250, 203, 13, 215, 46, 21, 218, 18, 129, 46, 213, 111, 133, 113, 38, 68, 196, 226, 90, 181, 191, 208, 106, 96,
  235, 208, 171, 101, 91, 18, 151, 64, 90, 201, 4, 177, 196, 190, 166, 44, 18, 123, 254, 221, 175, 121, 144, 67, 152, 39,
  209, 25, 8, 67, 255, 10, 230, 226, 109, 115, 85, 83, 78, 212, 17, 88, 43, 191, 12, 97, 107, 82, 42, 39, 219, 214, 41,
  182, 62, 130, 155, 14, 123, 129, 169, 252, 32, 174, 48, 149, 206, 73, 72, 222, 202, 0, 206, 198, 188, 87, 142, 185, 56,
  17, 182, 184, 88, 106, 170, 104, 1, 223, 198, 29, 29, 78, 150, 141, 173, 134, 206, 77, 227, 193, 96, 220, 219, 52, 112,
  155, 165, 173, 81, 88, 132, 75, 56, 224, 246, 150, 84, 145, 110, 104, 35, 166, 217, 135, 89, 125, 152, 6, 110, 40, 66,
  202, 214, 116, 91, 55, 232, 43, 167, 185, 252, 56, 76, 197, 136, 70, 25, 36, 94, 86, 216, 117, 70, 81, 62, 44, 117, 65,
  129, 148, 205, 121, 102, 61, 227, 56, 151, 159, 230, 58, 143, 112, 205, 19, 92, 113, 128, 139, 246, 191, 38, 63, 191,
  117, 172, 14, 188, 33, 110, 69, 190, 221, 54, 220, 107, 50, 144, 2, 100, 101, 123, 33, 177, 60, 183, 97, 75, 19, 182,
  106, 175, 86, 246, 111, 189, 21, 140, 208, 174, 254, 221, 193, 152, 108, 219, 146, 57, 188, 187, 172, 108, 218, 112,
  111, 185, 236, 108, 158, 86, 156, 167, 89, 21, 174, 46, 188, 118, 72, 100, 206, 87, 215, 182, 203, 13, 95, 120, 182,
  212, 199, 101, 227, 151, 243, 113, 249, 154, 86, 182, 165, 123, 159, 170, 29, 204, 221, 247, 69, 98, 239, 179, 211,
  113, 35, 212, 158, 226, 159, 150, 237, 139, 178, 194, 170, 105, 76, 162, 77, 109, 18, 109, 151, 102, 156, 93, 218, 144,
  74, 219, 238, 86, 175, 99, 248, 168, 112, 165, 163, 103, 153, 73, 54, 85, 191, 169, 165, 179, 78, 151, 5, 95, 51, 190,
  121, 27, 255, 100, 248, 171, 252, 114, 155, 78, 221, 56, 232, 17, 211, 108, 94, 170, 227, 226, 218, 222, 108, 42, 39,
  80, 229, 246, 200, 217, 55, 159, 54, 172, 236, 37, 220, 52, 249, 85, 198, 190, 167, 141, 253, 246, 166, 99, 236, 209,
  78, 180, 209, 51, 199, 126, 171, 225, 218, 14, 238, 110, 58, 54, 140, 140, 118, 138, 76, 221, 118, 28, 246, 109, 219,
  226, 212, 73, 70, 157, 42, 174, 93, 152, 131, 176, 188, 82, 1, 139, 190, 95, 64, 185, 10, 203, 93, 63, 51, 200, 40, 68,
  236, 152, 171, 144, 78, 250, 91, 120, 168, 217, 237, 149, 44, 163, 52, 224, 241, 210, 17, 78, 58, 166, 143, 142, 193,
  36, 124, 137, 209, 124, 233, 12, 4, 200, 196, 229, 86, 145, 74, 147, 72, 166, 234, 115, 30, 31, 61, 247, 254, 141, 239,
  36, 165, 168, 22, 62, 171, 230, 58, 172, 108, 234, 112, 220, 97, 89, 138, 230, 183, 218, 144, 234, 203, 147, 2, 101,
  233, 190, 116, 30, 39, 137, 226, 43, 88, 237, 88, 185, 194, 18, 227, 176, 91, 57, 13, 218, 82, 39, 113, 73, 55, 221, 1,
  167, 5, 154, 88, 113, 253, 107, 42, 178, 62, 109, 85, 85, 86, 195, 171, 56, 111, 9, 255, 58, 239, 22, 199, 242, 204,
  161, 109, 251, 32, 204, 220, 147, 114, 211, 164, 8, 13, 58, 108, 97, 74, 77, 100, 219, 24, 106, 6, 78, 170, 45, 155,
  230, 56, 73, 187, 185, 54, 160, 61, 39, 20, 231, 36, 132, 217, 54, 220, 218, 82, 85, 62, 96, 193, 116, 54, 3, 62, 148,
  199, 204, 150, 25, 209, 144, 25, 165, 124, 24, 133, 249, 36, 26, 5, 30, 180, 44, 17, 227, 92, 196, 12, 151, 227, 47,
  160, 195, 111, 154, 58, 188, 203, 181, 217, 82, 73, 239, 208, 18, 200, 124, 189, 165, 29, 195, 93, 114, 199, 209, 48,
  159, 163, 159, 181, 28, 106, 163, 178, 99, 159, 241, 45, 217, 208, 87, 207, 82, 194, 114, 152, 69, 35, 200, 137, 195,
  36, 119, 113, 196, 29, 119, 62, 29, 223, 206, 199, 223, 250, 47, 225, 39, 84, 209, 90, 145, 158, 157, 209, 222, 206,
  101, 37, 151, 242, 180, 98, 43, 106, 131, 204, 23, 83, 160, 223, 181, 181, 4, 124, 1, 79, 39, 121, 92, 176, 212, 39,
  161, 2, 63, 126, 242, 95, 229, 136, 212, 229, 174, 142, 166, 249, 216, 118, 192, 233, 172, 166, 25, 217, 104, 168, 39,
  87, 94, 28, 189, 222, 9, 78, 219, 215, 93, 27, 179, 7, 204, 88, 102, 163, 44, 131, 13, 227, 32, 4, 177, 153, 105, 14,
  165, 222, 107, 48, 187, 29, 195, 13, 250, 222, 184, 59, 234, 142, 74, 167, 207, 113, 52, 28, 141, 54, 20, 95, 151, 242,
  68, 134, 207, 40, 110, 226, 85, 29, 172, 7, 187, 221, 97, 119, 168, 92, 6, 176, 25, 75, 187, 237, 226, 114, 62, 53,
  216, 87, 235, 218, 34, 209, 47, 236, 8, 39, 20, 58, 85, 15, 186, 250, 92, 210, 106, 74, 181, 83, 237, 180, 235, 30,
  132, 218, 157, 241, 48, 220, 10, 183, 108, 154, 24, 78, 65, 219, 154, 79, 144, 156, 147, 150, 75, 159, 114, 32, 59,
  141, 248, 181, 33, 75, 144, 217, 171, 54, 75, 191, 149, 97, 178, 248, 169, 197, 28, 234, 13, 185, 129, 135, 164, 194,
  72, 217, 49, 60, 32, 1, 165, 85, 253, 137, 74, 75, 69, 214, 74, 103, 137, 71, 121, 226, 13, 169, 118, 14, 195, 109,
  128, 143, 100, 171, 235, 180, 104, 15, 147, 120, 222, 71, 125, 84, 236, 144, 26, 186, 75, 245, 44, 101, 189, 230, 3,
  215, 209, 221, 211, 221, 38, 248, 224, 171, 120, 138, 247, 244, 66, 77, 91, 103, 122, 96, 133, 105, 152, 13, 129, 73,
  110, 231, 144, 172, 76, 59, 110, 60, 198, 234, 182, 177, 102, 133, 59, 77, 183, 215, 169, 111, 113, 44, 192, 111, 21,
  46, 225, 161, 173, 142, 113, 193, 140, 122, 100, 212, 85, 202, 107, 222, 163, 216, 139, 64, 159, 177, 231, 243, 152,
  232, 187, 244, 58, 157, 32, 134, 234, 13, 160, 65, 97, 58, 235, 42, 154, 148, 247, 234, 204, 82, 93, 138, 201, 46, 167,
  247, 146, 156, 13, 170, 252, 85, 196, 111, 183, 235, 246, 52, 81, 199, 180, 163, 154, 17, 184, 27, 168, 183, 191, 156,
  139, 190, 156, 14, 231, 18, 70, 119, 80, 226, 202, 201, 179, 130, 163, 12, 249, 223, 43, 126, 28, 91, 219, 22, 119,
  112, 9, 181, 178, 133, 108, 211, 130, 176, 116, 79, 101, 77, 84, 172, 101, 95, 152, 234, 182, 187, 254, 251, 82, 250,
  117, 169, 187, 122, 96, 217, 215, 23, 137, 4, 25, 147, 82, 30, 84, 248, 125, 155, 123, 157, 207, 107, 137, 90, 105, 45,
  230, 229, 169, 135, 181, 79, 99, 69, 96, 64, 103, 138, 155, 140, 121, 59, 135, 21, 26, 195, 148, 43, 11, 69, 91, 209,
  189, 104, 96, 21, 226, 119, 27, 13, 135, 27, 163, 208, 89, 154, 142, 150, 226, 52, 8, 149, 50, 226, 226, 170, 33, 237,
  139, 52, 77, 138, 120, 238, 176, 147, 58, 85, 183, 138, 77, 227, 110, 111, 85, 215, 227, 173, 134, 197, 12, 154, 191,
  139, 235, 180, 215, 99, 58, 182, 221, 149, 233, 220, 206, 230, 106, 183, 195, 244, 246, 10, 78, 213, 219, 154, 221,
  182, 219, 211, 4, 82, 111, 211, 216, 195, 170, 68, 93, 122, 252, 173, 15, 65, 219, 53, 39, 119, 170, 55, 13, 26, 0,
  144, 187, 233, 236, 76, 14, 121, 60, 155, 192, 44, 44, 28, 19, 66, 122, 19, 76, 163, 81, 28, 6, 245, 121, 22, 141, 163,
  44, 111, 129, 174, 189, 24, 70, 232, 35, 35, 238, 97, 225, 119, 35, 248, 196, 203, 87, 93, 7, 106, 86, 92, 235, 105,
  174, 122, 61, 71, 167, 15, 251, 123, 127, 93, 94, 248, 191, 191, 46, 98, 18, 220, 199, 141, 7, 143, 7, 192, 54, 162,
  34, 32, 192, 253, 81, 124, 1, 58, 83, 152, 231, 251, 107, 229, 205, 244, 181, 3, 217, 5, 44, 80, 126, 97, 253, 174, 26,
  113, 224, 239, 255, 254, 31, 50, 234, 0, 228, 168, 5, 85, 200, 210, 139, 108, 237, 224, 25, 26, 115, 158, 167, 179,
  184, 72, 51, 224, 225, 251, 235, 90, 3, 230, 167, 27, 189, 128, 250, 184, 191, 38, 23, 138, 53, 95, 203, 104, 20, 210,
  50, 49, 34, 194, 60, 156, 137, 252, 36, 26, 173, 5, 241, 136, 21, 124, 6, 31, 7, 64, 64, 200, 119, 85, 17, 197, 78, 97,
  57, 89, 59, 72, 199, 99, 148, 143, 254, 226, 188, 5, 186, 130, 205, 218, 64, 147, 214, 243, 20, 3, 70, 188, 244, 85,
  54, 250, 111, 116, 70, 191, 205, 190, 164, 95, 129, 188, 42, 207, 123, 136, 159, 171, 116, 17, 203, 177, 62, 30, 226,
  207, 219, 244, 144, 234, 62, 162, 239, 3, 96, 141, 85, 250, 199, 247, 88, 88, 27, 253, 67, 31, 71, 23, 107, 2, 48, 93,
  66, 151, 131, 173, 168, 147, 107, 7, 80, 172, 5, 164, 92, 228, 247, 215, 25, 0, 13, 166, 236, 203, 40, 186, 56, 161,
  169, 180, 166, 33, 27, 200, 251, 244, 18, 186, 38, 115, 8, 126, 16, 158, 23, 241, 133, 217, 5, 173, 3, 250, 135, 50,
  82, 98, 175, 166, 78, 36, 38, 175, 198, 105, 198, 178, 127, 130, 134, 78, 72, 221, 117, 247, 240, 207, 81, 92, 96, 153,
  251, 235, 84, 81, 1, 196, 149, 100, 236, 159, 9, 72, 52, 47, 118, 114, 250, 188, 72, 231, 116, 5, 149, 212, 146, 253,
  181, 163, 5, 8, 239, 104, 253, 81, 148, 1, 43, 174, 29, 104, 159, 247, 215, 89, 217, 138, 234, 63, 156, 30, 174, 29,
  192, 63, 43, 20, 229, 160, 159, 165, 179, 81, 90, 182, 196, 62, 87, 168, 254, 144, 212, 163, 112, 253, 69, 116, 249,
  225, 199, 52, 59, 95, 59, 48, 83, 86, 1, 2, 106, 104, 6, 90, 76, 184, 126, 114, 61, 154, 69, 215, 0, 196, 72, 89, 5,
  72, 14, 165, 79, 211, 243, 235, 20, 170, 203, 223, 118, 69, 96, 26, 34, 191, 58, 104, 230, 132, 9, 202, 216, 12, 107,
  114, 44, 249, 212, 193, 177, 15, 102, 241, 112, 82, 4, 249, 245, 108, 56, 201, 144, 78, 6, 31, 170, 0, 229, 62, 156,
  65, 74, 210, 97, 152, 156, 66, 26, 155, 197, 214, 68, 244, 178, 173, 216, 74, 173, 185, 37, 48, 223, 35, 177, 70, 224,
  227, 81, 152, 121, 5, 175, 216, 95, 172, 5, 232, 241, 211, 98, 206, 221, 128, 41, 236, 164, 76, 185, 101, 212, 162,
  126, 232, 69, 236, 66, 52, 39, 214, 14, 78, 1, 92, 80, 255, 31, 255, 237, 176, 97, 9, 22, 87, 45, 26, 70, 144, 127, 82,
  62, 64, 226, 41, 97, 84, 210, 72, 35, 172, 84, 157, 215, 212, 226, 167, 148, 162, 214, 177, 27, 183, 5, 93, 149, 100,
  247, 80, 107, 178, 152, 198, 163, 184, 184, 254, 130, 20, 123, 202, 65, 6, 245, 223, 223, 157, 102, 79, 37, 94, 43,
  211, 77, 84, 249, 213, 104, 55, 76, 123, 95, 144, 108, 135, 47, 255, 254, 95, 254, 11, 168, 126, 243, 233, 221, 169,
  118, 136, 24, 173, 76, 48, 40, 253, 171, 209, 234, 98, 62, 250, 130, 180, 122, 243, 234, 113, 80, 63, 127, 21, 222,
  157, 82, 111, 230, 163, 91, 80, 10, 74, 127, 9, 74, 85, 172, 236, 179, 80, 45, 167, 104, 44, 144, 241, 56, 204, 39,
  131, 52, 204, 70, 114, 245, 21, 110, 213, 160, 69, 136, 60, 91, 75, 49, 160, 48, 117, 58, 119, 192, 96, 57, 209, 108,
  41, 8, 82, 214, 28, 0, 184, 18, 167, 215, 190, 191, 46, 251, 196, 182, 9, 98, 75, 64, 195, 131, 16, 201, 16, 254, 136,
  236, 224, 18, 168, 106, 28, 151, 220, 112, 159, 109, 164, 14, 158, 68, 147, 36, 202, 112, 255, 65, 159, 34, 119, 145,
  148, 224, 158, 197, 160, 207, 193, 216, 44, 164, 58, 163, 246, 97, 152, 68, 97, 118, 132, 229, 114, 232, 246, 112, 146,
  196, 209, 207, 255, 151, 217, 113, 101, 96, 238, 99, 76, 33, 109, 25, 67, 40, 232, 249, 222, 26, 89, 131, 66, 254, 244,
  236, 40, 121, 77, 211, 165, 200, 252, 35, 53, 78, 60, 182, 101, 179, 34, 87, 139, 193, 190, 49, 198, 251, 143, 66, 17,
  71, 175, 23, 245, 42, 190, 226, 37, 106, 76, 173, 100, 113, 181, 86, 189, 45, 162, 93, 41, 80, 133, 59, 84, 235, 121,
  148, 230, 0, 201, 140, 122, 251, 107, 27, 189, 206, 26, 55, 148, 239, 175, 117, 55, 59, 72, 93, 86, 233, 160, 82, 16,
  168, 183, 227, 43, 166, 190, 26, 179, 67, 185, 180, 90, 41, 9, 202, 184, 10, 128, 12, 162, 240, 140, 20, 153, 250, 179,
  197, 21, 151, 8, 218, 124, 46, 183, 186, 92, 121, 89, 92, 61, 78, 11, 185, 73, 89, 101, 34, 59, 218, 199, 13, 111, 37,
  150, 202, 149, 116, 171, 156, 163, 164, 196, 141, 11, 25, 151, 96, 187, 139, 200, 225, 140, 245, 69, 88, 109, 62, 31,
  143, 190, 52, 175, 17, 204, 127, 38, 102, 123, 245, 234, 9, 172, 62, 255, 227, 255, 153, 166, 201, 250, 244, 127, 252,
  223, 235, 249, 42, 60, 135, 189, 252, 205, 50, 29, 27, 86, 31, 215, 57, 208, 224, 119, 252, 65, 134, 206, 163, 243, 34,
  91, 76, 251, 202, 142, 24, 161, 65, 250, 16, 211, 181, 5, 115, 144, 173, 31, 60, 129, 45, 47, 30, 62, 234, 197, 159,
  128, 216, 68, 107, 67, 229, 234, 250, 27, 224, 127, 83, 227, 251, 2, 236, 143, 32, 255, 153, 184, 223, 82, 83, 43, 217,
  30, 122, 247, 155, 229, 250, 161, 84, 150, 127, 139, 172, 102, 109, 99, 191, 0, 175, 17, 204, 127, 38, 102, 251, 97,
  122, 22, 13, 22, 179, 51, 12, 228, 58, 199, 248, 102, 139, 76, 221, 139, 87, 178, 30, 86, 249, 205, 242, 94, 81, 154,
  4, 126, 139, 204, 231, 180, 10, 124, 1, 6, 148, 112, 255, 169, 148, 203, 197, 184, 24, 71, 11, 208, 48, 163, 210, 164,
  81, 201, 122, 162, 155, 191, 89, 246, 155, 232, 214, 149, 223, 34, 11, 194, 78, 109, 252, 197, 183, 53, 8, 243, 159,
  138, 245, 0, 225, 150, 105, 129, 172, 222, 219, 64, 141, 223, 238, 230, 134, 198, 244, 55, 203, 114, 166, 141, 234, 11,
  112, 28, 130, 252, 103, 98, 56, 195, 176, 86, 201, 106, 208, 183, 223, 44, 167, 93, 204, 151, 111, 104, 120, 57, 227,
  244, 78, 223, 223, 172, 202, 168, 6, 34, 226, 22, 189, 68, 230, 20, 160, 30, 50, 230, 177, 145, 225, 220, 164, 22, 229,
  188, 164, 194, 99, 73, 37, 207, 84, 147, 162, 188, 12, 239, 104, 81, 39, 0, 54, 248, 140, 92, 228, 180, 6, 153, 215,
  156, 135, 4, 18, 134, 113, 182, 121, 87, 135, 44, 39, 146, 54, 154, 167, 97, 118, 22, 21, 183, 67, 211, 134, 34, 70,
  156, 227, 174, 122, 99, 96, 52, 224, 170, 46, 123, 216, 105, 57, 151, 97, 187, 47, 210, 199, 32, 27, 52, 236, 249, 117,
  117, 223, 25, 241, 121, 20, 207, 162, 0, 106, 161, 141, 242, 23, 18, 153, 120, 146, 56, 100, 135, 139, 94, 163, 37, 19,
  150, 216, 149, 240, 226, 236, 16, 63, 20, 128, 147, 13, 129, 189, 226, 132, 8, 210, 237, 143, 17, 168, 237, 217, 44,
  248, 249, 255, 12, 234, 189, 205, 9, 136, 148, 201, 134, 251, 212, 15, 109, 162, 150, 228, 61, 16, 86, 223, 103, 139,
  171, 190, 180, 249, 42, 134, 3, 64, 229, 153, 180, 150, 185, 69, 144, 6, 135, 118, 174, 62, 72, 230, 201, 73, 37, 36,
  92, 145, 125, 128, 172, 195, 190, 74, 72, 79, 152, 94, 233, 3, 246, 212, 176, 160, 84, 194, 2, 217, 237, 131, 99, 158,
  118, 84, 30, 75, 172, 200, 14, 238, 161, 84, 86, 33, 201, 212, 150, 103, 181, 22, 133, 165, 210, 61, 83, 231, 10, 147,
  211, 250, 56, 89, 223, 160, 148, 91, 140, 117, 238, 210, 157, 33, 216, 53, 107, 90, 146, 13, 127, 136, 91, 188, 213,
  96, 185, 176, 25, 50, 139, 224, 159, 27, 62, 44, 165, 31, 133, 31, 135, 42, 39, 102, 123, 101, 212, 221, 5, 216, 62,
  242, 84, 238, 142, 93, 30, 6, 142, 106, 165, 254, 175, 108, 110, 86, 172, 75, 75, 43, 240, 218, 138, 197, 201, 202, 66,
  83, 111, 197, 10, 100, 1, 135, 137, 237, 46, 110, 251, 61, 96, 154, 233, 191, 226, 119, 241, 82, 125, 187, 238, 232,
  55, 92, 21, 81, 76, 137, 168, 108, 171, 146, 200, 6, 120, 160, 68, 138, 128, 177, 226, 170, 174, 254, 142, 5, 200, 226,
  230, 215, 225, 236, 44, 250, 229, 152, 153, 192, 87, 243, 178, 134, 129, 244, 133, 43, 93, 236, 151, 177, 46, 44, 9,
  107, 7, 240, 207, 138, 108, 177, 209, 81, 136, 69, 110, 103, 25, 34, 176, 118, 208, 125, 190, 34, 132, 93, 31, 132,
  141, 85, 33, 96, 232, 96, 55, 136, 237, 231, 159, 193, 174, 142, 225, 61, 68, 66, 254, 114, 195, 251, 36, 204, 6, 75,
  134, 87, 195, 192, 57, 188, 119, 158, 138, 210, 141, 143, 38, 194, 139, 20, 212, 145, 104, 77, 223, 101, 4, 81, 150,
  185, 149, 162, 37, 238, 252, 138, 10, 178, 105, 77, 66, 221, 1, 81, 105, 93, 241, 69, 12, 184, 215, 101, 240, 247, 127,
  255, 223, 130, 159, 64, 43, 140, 130, 36, 61, 15, 97, 227, 72, 188, 106, 121, 27, 106, 167, 203, 80, 255, 53, 136, 248,
  235, 229, 254, 133, 218, 109, 19, 21, 117, 90, 211, 8, 136, 227, 56, 222, 47, 215, 140, 157, 169, 28, 70, 223, 38, 115,
  149, 101, 222, 80, 203, 92, 59, 235, 234, 245, 89, 104, 130, 226, 148, 40, 184, 252, 249, 175, 147, 4, 117, 217, 202,
  181, 122, 70, 91, 189, 103, 71, 143, 131, 239, 195, 89, 152, 184, 230, 139, 206, 173, 51, 123, 111, 104, 204, 220, 241,
  34, 73, 62, 228, 242, 80, 234, 77, 154, 36, 226, 203, 51, 113, 245, 250, 116, 191, 107, 237, 224, 45, 254, 89, 169,
  194, 32, 73, 83, 104, 232, 81, 242, 243, 223, 138, 232, 156, 117, 195, 225, 252, 231, 154, 66, 10, 71, 229, 225, 5,
  110, 27, 89, 255, 128, 138, 241, 112, 2, 26, 181, 211, 33, 85, 33, 33, 219, 212, 162, 195, 172, 61, 26, 122, 160, 99,
  22, 123, 103, 237, 224, 213, 56, 9, 103, 63, 69, 51, 152, 124, 163, 24, 198, 169, 14, 235, 123, 99, 9, 221, 101, 43,
  149, 116, 200, 163, 104, 4, 211, 233, 12, 208, 47, 162, 225, 57, 254, 12, 214, 131, 147, 159, 255, 58, 165, 159, 245,
  110, 240, 54, 133, 78, 53, 86, 34, 234, 69, 116, 134, 250, 222, 89, 92, 132, 228, 55, 123, 219, 113, 152, 79, 194, 124,
  181, 225, 195, 171, 88, 31, 120, 221, 147, 249, 207, 127, 45, 162, 1, 65, 184, 203, 32, 134, 243, 121, 114, 237, 29,
  144, 29, 146, 82, 15, 177, 12, 62, 232, 7, 171, 137, 219, 225, 120, 174, 17, 221, 48, 90, 228, 250, 142, 214, 184, 244,
  140, 50, 96, 238, 6, 38, 54, 212, 110, 188, 196, 53, 135, 112, 107, 176, 57, 30, 186, 192, 56, 188, 239, 159, 199, 179,
  184, 245, 71, 80, 138, 38, 36, 62, 11, 96, 54, 16, 161, 81, 241, 19, 232, 88, 193, 246, 36, 152, 103, 41, 204, 107, 24,
  243, 164, 173, 193, 83, 54, 166, 191, 171, 16, 56, 210, 205, 253, 112, 101, 241, 243, 54, 110, 61, 137, 131, 147, 168,
  88, 204, 45, 201, 35, 214, 35, 130, 200, 194, 59, 68, 163, 71, 56, 45, 36, 113, 217, 165, 145, 10, 203, 155, 18, 147,
  210, 86, 122, 76, 151, 121, 186, 104, 33, 9, 174, 133, 172, 96, 239, 12, 149, 61, 164, 39, 78, 220, 190, 244, 165, 79,
  20, 108, 127, 6, 80, 29, 133, 170, 238, 22, 181, 146, 9, 78, 85, 135, 171, 150, 78, 107, 179, 121, 114, 114, 236, 222,
  109, 34, 226, 39, 121, 60, 90, 126, 152, 175, 130, 59, 126, 229, 5, 118, 60, 191, 173, 215, 221, 202, 93, 92, 201, 18,
  165, 119, 27, 202, 135, 73, 137, 235, 82, 83, 160, 12, 124, 82, 14, 235, 35, 252, 90, 181, 35, 138, 28, 97, 17, 23,
  222, 2, 132, 39, 105, 54, 93, 65, 197, 176, 164, 204, 115, 244, 249, 142, 22, 209, 52, 120, 1, 243, 241, 50, 202, 206,
  3, 152, 183, 131, 152, 177, 143, 67, 232, 56, 205, 210, 162, 31, 175, 69, 60, 148, 87, 100, 155, 213, 167, 75, 96, 68,
  75, 249, 21, 166, 207, 206, 96, 52, 222, 217, 91, 101, 190, 96, 135, 219, 237, 246, 234, 51, 102, 110, 119, 154, 84,
  70, 151, 252, 59, 129, 205, 116, 20, 0, 86, 151, 236, 164, 161, 77, 190, 242, 134, 232, 244, 179, 168, 208, 225, 109,
  82, 40, 172, 32, 137, 251, 114, 14, 146, 105, 133, 203, 44, 38, 58, 193, 207, 255, 125, 60, 158, 185, 71, 221, 219, 24,
  87, 109, 93, 240, 143, 50, 224, 171, 2, 185, 41, 199, 238, 123, 224, 186, 73, 107, 211, 208, 166, 246, 147, 48, 73,
  112, 176, 215, 14, 240, 87, 30, 48, 195, 40, 31, 203, 5, 134, 55, 16, 140, 12, 203, 205, 112, 18, 76, 129, 211, 71,
  192, 231, 36, 244, 91, 15, 95, 201, 225, 47, 47, 150, 181, 248, 130, 192, 51, 218, 230, 226, 86, 201, 251, 52, 3, 125,
  7, 54, 250, 173, 28, 211, 84, 78, 241, 197, 122, 246, 240, 42, 10, 92, 78, 242, 19, 69, 172, 67, 7, 51, 245, 48, 86,
  216, 165, 132, 44, 149, 40, 57, 236, 12, 80, 158, 172, 105, 184, 117, 245, 135, 12, 123, 76, 118, 8, 137, 17, 5, 254,
  17, 174, 52, 91, 204, 129, 54, 160, 105, 194, 191, 151, 105, 86, 184, 148, 75, 122, 114, 145, 249, 132, 97, 217, 0, 31,
  68, 102, 191, 161, 6, 172, 252, 160, 178, 14, 163, 73, 154, 140, 34, 0, 247, 246, 217, 195, 23, 129, 128, 38, 31, 44,
  94, 105, 48, 24, 82, 70, 151, 237, 224, 10, 219, 43, 175, 16, 12, 113, 134, 46, 208, 102, 120, 62, 72, 175, 24, 19,
  163, 88, 139, 135, 199, 243, 83, 146, 221, 238, 185, 19, 156, 80, 161, 224, 248, 149, 49, 118, 54, 133, 170, 142, 9,
  49, 110, 7, 87, 85, 180, 166, 95, 219, 4, 40, 9, 29, 207, 13, 178, 30, 191, 10, 234, 76, 197, 13, 147, 134, 65, 86,
  173, 230, 217, 165, 81, 243, 143, 160, 50, 95, 134, 215, 43, 86, 207, 103, 70, 245, 147, 197, 96, 22, 21, 254, 218,
  149, 215, 250, 112, 159, 68, 140, 124, 251, 37, 78, 19, 116, 121, 84, 104, 243, 193, 88, 65, 29, 239, 14, 139, 91, 225,
  227, 49, 140, 228, 91, 104, 46, 7, 201, 148, 23, 81, 146, 128, 84, 242, 52, 89, 174, 38, 43, 171, 241, 59, 78, 53, 126,
  245, 147, 29, 173, 130, 211, 159, 61, 55, 174, 7, 96, 98, 133, 27, 251, 45, 54, 255, 242, 118, 129, 67, 245, 22, 29,
  47, 195, 18, 243, 99, 38, 114, 163, 23, 87, 22, 60, 231, 227, 158, 221, 237, 154, 95, 224, 133, 163, 17, 131, 249, 168,
  152, 185, 39, 227, 119, 1, 191, 118, 12, 11, 209, 79, 139, 241, 207, 127, 59, 91, 73, 212, 221, 138, 212, 67, 237, 26,
  197, 23, 35, 52, 51, 95, 213, 95, 128, 98, 66, 45, 4, 111, 163, 193, 227, 135, 111, 154, 193, 211, 211, 211, 87, 235,
  248, 207, 73, 163, 106, 16, 202, 176, 114, 142, 213, 203, 179, 92, 185, 76, 147, 14, 145, 41, 204, 230, 48, 146, 126,
  89, 73, 237, 31, 205, 232, 17, 89, 143, 164, 100, 93, 124, 6, 194, 20, 141, 7, 116, 123, 54, 142, 136, 183, 220, 203,
  164, 213, 59, 37, 122, 149, 210, 232, 161, 146, 90, 125, 174, 238, 139, 186, 230, 62, 100, 23, 7, 152, 41, 142, 18, 63,
  197, 117, 169, 156, 30, 83, 162, 130, 150, 88, 63, 84, 77, 47, 240, 5, 101, 115, 19, 239, 239, 255, 251, 223, 124, 218,
  94, 245, 9, 182, 183, 235, 60, 252, 151, 155, 144, 39, 60, 211, 77, 24, 221, 16, 107, 213, 98, 246, 216, 35, 24, 228,
  40, 129, 46, 21, 110, 149, 222, 73, 180, 163, 81, 92, 172, 162, 17, 63, 138, 208, 6, 30, 23, 126, 29, 248, 142, 84, 33,
  135, 22, 103, 231, 30, 185, 92, 93, 44, 251, 39, 86, 250, 33, 75, 112, 53, 193, 233, 27, 60, 10, 243, 40, 248, 225,
  245, 51, 15, 143, 27, 235, 170, 172, 174, 175, 174, 147, 162, 152, 247, 215, 215, 145, 28, 235, 89, 52, 77, 139, 168,
  61, 159, 204, 215, 71, 225, 197, 250, 56, 78, 162, 124, 125, 145, 71, 217, 186, 181, 96, 123, 208, 123, 149, 165, 69,
  10, 43, 31, 168, 115, 248, 235, 60, 77, 146, 42, 236, 84, 99, 173, 94, 221, 237, 174, 97, 156, 85, 2, 234, 107, 7, 248,
  111, 80, 135, 245, 114, 54, 2, 129, 216, 240, 29, 216, 120, 33, 228, 12, 68, 94, 85, 209, 173, 67, 87, 237, 83, 244,
  211, 104, 138, 142, 20, 48, 57, 247, 52, 158, 93, 70, 113, 222, 15, 168, 213, 32, 202, 198, 20, 62, 165, 8, 198, 139,
  217, 57, 34, 64, 114, 107, 20, 229, 193, 159, 33, 21, 84, 129, 243, 16, 119, 44, 248, 90, 179, 16, 223, 245, 147, 40,
  25, 183, 208, 226, 0, 251, 222, 120, 26, 160, 194, 155, 158, 55, 3, 160, 115, 70, 69, 143, 78, 94, 5, 243, 236, 231,
  191, 141, 65, 229, 89, 100, 65, 132, 211, 37, 31, 78, 178, 159, 255, 10, 77, 52, 218, 214, 150, 211, 199, 108, 57, 202,
  47, 252, 119, 22, 78, 163, 219, 176, 25, 86, 212, 249, 172, 196, 94, 192, 91, 153, 167, 104, 123, 240, 112, 62, 111, 9,
  165, 62, 88, 15, 78, 211, 115, 175, 112, 119, 97, 244, 106, 249, 190, 65, 109, 97, 85, 220, 96, 227, 139, 83, 24, 35,
  11, 200, 159, 65, 29, 95, 225, 46, 162, 198, 109, 56, 95, 1, 180, 10, 227, 118, 215, 14, 186, 244, 216, 119, 113, 43,
  126, 135, 205, 101, 143, 85, 139, 110, 85, 111, 99, 237, 96, 227, 46, 245, 54, 215, 14, 54, 239, 82, 111, 107, 237, 96,
  235, 46, 245, 182, 215, 14, 182, 87, 168, 87, 57, 157, 239, 164, 190, 152, 65, 17, 151, 171, 51, 175, 162, 44, 143,
  125, 75, 80, 160, 40, 6, 193, 40, 92, 68, 217, 36, 132, 185, 156, 151, 231, 60, 126, 206, 90, 110, 224, 236, 58, 253,
  71, 60, 171, 230, 73, 136, 151, 86, 43, 79, 152, 42, 106, 159, 70, 185, 185, 230, 162, 218, 63, 138, 2, 204, 169, 2,
  230, 247, 180, 51, 219, 120, 146, 106, 174, 127, 250, 202, 110, 29, 120, 176, 210, 129, 178, 46, 223, 114, 149, 95,
  106, 216, 189, 37, 213, 109, 138, 23, 116, 58, 75, 127, 2, 180, 115, 101, 35, 138, 85, 228, 197, 210, 6, 144, 206, 45,
  162, 67, 218, 157, 128, 61, 137, 103, 113, 62, 49, 193, 161, 177, 44, 24, 68, 145, 111, 243, 236, 39, 212, 92, 235, 40,
  108, 106, 159, 231, 103, 119, 221, 230, 122, 45, 252, 142, 86, 61, 182, 68, 190, 56, 20, 147, 167, 190, 101, 27, 154,
  253, 97, 158, 164, 225, 40, 15, 146, 16, 251, 27, 196, 179, 96, 189, 180, 22, 174, 255, 33, 41, 246, 70, 209, 69, 60,
  140, 142, 71, 127, 56, 43, 246, 214, 29, 203, 171, 173, 24, 122, 204, 218, 174, 162, 24, 238, 154, 7, 41, 224, 91, 185,
  136, 237, 133, 100, 136, 2, 22, 77, 204, 220, 42, 33, 125, 101, 44, 164, 138, 83, 137, 101, 109, 190, 94, 204, 48, 200,
  139, 175, 53, 158, 253, 197, 90, 43, 153, 212, 211, 158, 40, 240, 165, 90, 148, 167, 123, 190, 22, 101, 129, 47, 208,
  216, 255, 180, 136, 22, 94, 82, 82, 230, 73, 252, 19, 244, 170, 243, 121, 205, 60, 9, 227, 100, 145, 225, 137, 177,
  187, 37, 145, 191, 180, 161, 165, 226, 78, 99, 103, 239, 204, 189, 37, 250, 207, 66, 216, 197, 46, 104, 218, 249, 122,
  128, 69, 216, 196, 252, 2, 163, 66, 237, 21, 180, 36, 249, 91, 163, 197, 236, 51, 218, 146, 7, 57, 24, 214, 149, 197,
  252, 236, 7, 180, 225, 237, 237, 105, 136, 204, 65, 30, 249, 16, 97, 93, 70, 137, 37, 225, 225, 133, 132, 22, 41, 39,
  125, 122, 154, 99, 239, 151, 70, 146, 38, 158, 15, 65, 202, 124, 29, 133, 121, 58, 251, 199, 97, 72, 100, 164, 176, 31,
  85, 3, 74, 241, 62, 62, 11, 73, 223, 26, 183, 108, 71, 168, 204, 140, 163, 233, 124, 156, 162, 239, 83, 95, 217, 225,
  105, 187, 157, 217, 162, 248, 41, 154, 53, 121, 80, 174, 81, 152, 7, 79, 195, 197, 188, 224, 219, 152, 162, 189, 124,
  85, 188, 165, 77, 114, 137, 83, 183, 207, 204, 8, 74, 107, 30, 212, 153, 67, 71, 176, 109, 185, 245, 107, 246, 244, 49,
  200, 158, 9, 86, 128, 157, 221, 121, 177, 0, 178, 231, 220, 94, 87, 21, 105, 6, 67, 183, 34, 251, 179, 138, 143, 249,
  151, 163, 206, 60, 147, 190, 33, 73, 122, 38, 53, 111, 248, 253, 8, 126, 163, 30, 145, 85, 92, 115, 240, 95, 114, 152,
  131, 78, 54, 139, 50, 242, 48, 241, 220, 204, 88, 78, 167, 87, 12, 72, 240, 135, 224, 100, 49, 199, 216, 222, 24, 185,
  198, 186, 1, 161, 52, 39, 66, 215, 172, 228, 187, 92, 105, 244, 214, 78, 210, 8, 246, 177, 185, 13, 62, 126, 108, 158,
  175, 88, 117, 94, 208, 238, 93, 223, 222, 155, 27, 122, 19, 91, 11, 200, 227, 40, 31, 26, 64, 30, 145, 145, 34, 138,
  241, 98, 183, 14, 204, 170, 109, 27, 178, 126, 120, 253, 204, 119, 70, 100, 87, 7, 6, 74, 141, 250, 152, 228, 5, 112,
  235, 77, 225, 182, 190, 41, 92, 106, 226, 230, 120, 73, 35, 55, 229, 70, 35, 215, 238, 16, 16, 11, 30, 178, 240, 144,
  166, 75, 173, 113, 240, 197, 25, 173, 228, 184, 220, 183, 141, 51, 36, 194, 253, 245, 50, 188, 209, 253, 113, 154, 66,
  175, 72, 229, 71, 127, 248, 44, 77, 18, 0, 197, 2, 175, 6, 245, 232, 106, 30, 101, 160, 254, 205, 10, 32, 90, 240, 247,
  127, 255, 175, 252, 152, 228, 44, 154, 252, 252, 215, 69, 30, 81, 193, 251, 235, 28, 200, 239, 244, 56, 79, 226, 193,
  241, 53, 87, 64, 192, 161, 58, 201, 202, 56, 190, 155, 20, 88, 188, 124, 182, 110, 149, 25, 39, 35, 118, 106, 39, 3,
  218, 148, 83, 44, 59, 128, 213, 33, 69, 74, 197, 127, 109, 34, 151, 204, 36, 10, 86, 219, 149, 176, 113, 86, 108, 221,
  63, 123, 189, 46, 130, 120, 220, 5, 11, 42, 134, 38, 61, 120, 168, 32, 95, 125, 96, 57, 12, 103, 195, 40, 177, 227,
  153, 30, 60, 28, 12, 178, 200, 125, 74, 111, 206, 216, 185, 30, 197, 244, 142, 251, 52, 53, 102, 150, 178, 202, 148,
  238, 222, 242, 161, 67, 207, 189, 161, 242, 173, 183, 53, 163, 130, 237, 231, 199, 41, 160, 215, 196, 39, 205, 148,
  170, 135, 244, 125, 240, 7, 220, 179, 228, 123, 78, 66, 174, 124, 107, 201, 229, 204, 161, 227, 120, 202, 174, 208, 62,
  142, 10, 80, 184, 241, 185, 23, 88, 194, 93, 155, 86, 95, 76, 91, 9, 232, 22, 129, 109, 109, 234, 225, 187, 114, 107,
  124, 56, 17, 145, 147, 97, 58, 143, 78, 49, 209, 199, 64, 80, 3, 131, 37, 203, 17, 23, 209, 203, 48, 113, 25, 243, 97,
  93, 208, 174, 182, 39, 80, 92, 40, 3, 43, 214, 161, 171, 31, 188, 18, 121, 213, 175, 84, 203, 119, 3, 196, 179, 66,
  227, 189, 144, 149, 224, 238, 222, 14, 238, 198, 170, 112, 189, 183, 69, 60, 128, 183, 159, 223, 202, 227, 95, 213, 6,
  110, 117, 85, 169, 250, 218, 211, 154, 207, 127, 135, 241, 212, 63, 244, 98, 138, 3, 133, 187, 223, 76, 209, 22, 40,
  132, 123, 155, 27, 89, 174, 113, 17, 114, 195, 122, 54, 221, 235, 40, 102, 78, 219, 139, 249, 232, 77, 28, 93, 210,
  148, 93, 166, 119, 58, 152, 238, 105, 20, 22, 211, 112, 110, 205, 102, 158, 238, 241, 201, 211, 97, 96, 76, 95, 188,
  254, 194, 34, 4, 95, 136, 187, 149, 171, 248, 122, 233, 215, 216, 37, 50, 154, 110, 195, 158, 8, 225, 239, 104, 244,
  182, 181, 199, 60, 120, 15, 93, 23, 220, 85, 216, 156, 9, 216, 245, 250, 101, 192, 61, 224, 52, 9, 126, 132, 239, 44,
  26, 195, 30, 177, 52, 247, 24, 124, 95, 113, 43, 91, 7, 253, 56, 26, 44, 206, 52, 79, 81, 88, 104, 91, 51, 80, 146, 60,
  151, 155, 60, 47, 114, 210, 60, 166, 199, 164, 184, 178, 191, 212, 19, 172, 124, 167, 241, 115, 174, 59, 90, 118, 117,
  247, 66, 120, 253, 16, 90, 123, 198, 66, 179, 190, 141, 178, 194, 231, 145, 141, 101, 175, 148, 178, 200, 98, 119, 141,
  153, 170, 254, 212, 104, 126, 202, 30, 95, 48, 6, 84, 60, 201, 32, 60, 228, 64, 191, 69, 99, 0, 47, 123, 224, 84, 90,
  212, 71, 105, 157, 186, 171, 170, 180, 172, 50, 251, 151, 105, 46, 106, 131, 203, 20, 152, 37, 254, 86, 186, 255, 146,
  239, 226, 189, 250, 240, 42, 151, 22, 44, 126, 9, 166, 208, 249, 163, 103, 49, 96, 152, 30, 194, 20, 56, 75, 179, 107,
  46, 140, 15, 190, 103, 223, 113, 180, 228, 234, 144, 187, 182, 83, 106, 91, 109, 158, 130, 18, 46, 106, 176, 174, 174,
  212, 152, 90, 205, 17, 59, 221, 191, 204, 42, 36, 210, 104, 227, 15, 4, 200, 204, 87, 76, 84, 141, 195, 69, 82, 188,
  138, 103, 249, 241, 108, 156, 210, 105, 15, 249, 47, 204, 33, 37, 184, 140, 50, 60, 103, 128, 1, 90, 128, 38, 212, 94,
  54, 165, 87, 118, 202, 58, 56, 121, 252, 208, 231, 16, 85, 238, 106, 160, 95, 39, 163, 80, 108, 106, 102, 139, 233, 0,
  117, 223, 105, 60, 219, 95, 235, 172, 225, 131, 42, 251, 107, 27, 187, 107, 40, 156, 105, 175, 106, 185, 114, 174, 120,
  36, 195, 49, 58, 124, 182, 26, 70, 195, 228, 87, 194, 232, 245, 159, 86, 66, 232, 245, 213, 175, 132, 207, 233, 106,
  248, 156, 126, 65, 124, 42, 61, 106, 195, 209, 5, 110, 46, 71, 200, 187, 228, 44, 185, 220, 163, 234, 33, 175, 18, 64,
  157, 224, 48, 157, 141, 227, 51, 143, 251, 171, 238, 245, 57, 34, 19, 145, 217, 232, 91, 122, 181, 192, 189, 0, 191,
  141, 179, 243, 4, 221, 252, 161, 32, 144, 44, 58, 91, 204, 206, 130, 159, 255, 58, 131, 237, 219, 236, 1, 232, 144, 73,
  142, 87, 48, 16, 74, 112, 254, 243, 127, 159, 205, 240, 120, 29, 254, 227, 146, 113, 49, 27, 100, 225, 98, 56, 193,
  183, 216, 166, 120, 17, 109, 102, 94, 68, 187, 181, 48, 216, 88, 51, 122, 104, 135, 169, 120, 20, 23, 176, 219, 65,
  151, 157, 8, 223, 113, 75, 130, 112, 145, 183, 0, 151, 81, 192, 28, 134, 135, 231, 128, 97, 253, 69, 4, 139, 52, 172,
  87, 141, 38, 128, 199, 75, 12, 212, 9, 50, 103, 112, 121, 97, 221, 83, 208, 29, 152, 11, 186, 211, 124, 185, 202, 120,
  137, 182, 130, 191, 224, 78, 236, 206, 123, 143, 106, 75, 164, 130, 29, 16, 239, 81, 168, 220, 171, 227, 182, 138, 63,
  47, 178, 159, 255, 54, 60, 95, 182, 169, 130, 218, 47, 200, 43, 240, 45, 58, 237, 101, 43, 20, 39, 159, 6, 15, 7, 85,
  120, 58, 184, 13, 36, 8, 239, 206, 206, 218, 62, 235, 72, 62, 204, 226, 185, 92, 134, 64, 47, 19, 199, 25, 207, 163,
  60, 15, 207, 162, 60, 216, 15, 102, 209, 37, 222, 98, 169, 55, 246, 236, 98, 44, 118, 58, 20, 26, 165, 195, 5, 26, 231,
  218, 103, 81, 113, 148, 68, 248, 243, 209, 245, 241, 168, 94, 83, 138, 213, 92, 16, 208, 230, 188, 180, 62, 22, 50,
  107, 43, 145, 212, 65, 60, 84, 129, 80, 74, 34, 16, 14, 101, 188, 152, 49, 227, 59, 121, 193, 241, 252, 186, 250, 178,
  85, 60, 14, 234, 95, 169, 221, 252, 207, 255, 57, 248, 74, 226, 211, 128, 138, 197, 34, 155, 237, 201, 242, 50, 171,
  29, 99, 241, 167, 167, 207, 159, 1, 90, 181, 154, 81, 66, 144, 182, 13, 58, 197, 17, 204, 254, 250, 52, 63, 11, 246,
  15, 148, 134, 69, 23, 147, 88, 237, 214, 48, 131, 13, 77, 196, 123, 86, 175, 37, 113, 73, 17, 252, 191, 36, 110, 227,
  201, 210, 33, 211, 173, 161, 34, 128, 85, 243, 75, 236, 194, 249, 28, 186, 124, 56, 137, 147, 81, 61, 137, 21, 32, 55,
  13, 3, 85, 214, 239, 54, 49, 88, 155, 51, 48, 64, 214, 187, 129, 155, 233, 224, 65, 80, 195, 205, 119, 45, 232, 7, 53,
  228, 112, 217, 233, 27, 139, 222, 243, 69, 62, 33, 106, 215, 167, 12, 132, 69, 115, 153, 238, 164, 176, 108, 56, 28,
  141, 36, 136, 178, 12, 146, 14, 84, 235, 54, 62, 58, 227, 200, 214, 71, 219, 143, 165, 194, 52, 245, 105, 88, 12, 39,
  175, 178, 104, 28, 95, 225, 124, 88, 36, 137, 137, 178, 86, 98, 223, 46, 99, 162, 78, 208, 235, 42, 233, 131, 40, 201,
  35, 173, 198, 195, 44, 11, 175, 219, 227, 44, 157, 214, 181, 202, 141, 42, 198, 225, 232, 228, 103, 109, 146, 172, 249,
  219, 184, 152, 168, 216, 53, 26, 6, 38, 35, 88, 189, 138, 8, 107, 104, 220, 164, 178, 194, 205, 170, 196, 195, 166,
  245, 105, 169, 210, 64, 207, 193, 209, 59, 194, 119, 4, 145, 37, 35, 224, 50, 156, 167, 241, 240, 188, 214, 12, 96, 18,
  66, 175, 212, 1, 104, 148, 109, 153, 116, 42, 185, 169, 246, 60, 206, 115, 244, 202, 127, 252, 242, 57, 148, 162, 105,
  210, 15, 140, 201, 111, 96, 124, 25, 207, 70, 233, 165, 3, 23, 34, 18, 226, 66, 111, 29, 54, 116, 50, 179, 217, 73,
  212, 15, 40, 255, 65, 155, 51, 26, 138, 136, 218, 15, 179, 97, 184, 192, 39, 182, 255, 229, 132, 17, 91, 145, 0, 37,
  186, 31, 33, 151, 126, 245, 131, 111, 62, 1, 172, 155, 143, 37, 122, 165, 148, 242, 34, 184, 152, 77, 64, 147, 7, 45,
  43, 139, 254, 194, 142, 84, 170, 177, 205, 232, 164, 188, 68, 152, 125, 239, 57, 251, 84, 231, 133, 255, 240, 7, 94,
  173, 45, 103, 228, 3, 35, 5, 230, 123, 29, 213, 193, 116, 44, 91, 0, 246, 175, 229, 5, 190, 123, 87, 147, 197, 81, 44,
  252, 32, 48, 198, 11, 248, 211, 24, 70, 177, 68, 189, 225, 163, 208, 43, 86, 180, 146, 70, 114, 206, 146, 224, 175,
  199, 163, 134, 213, 123, 216, 193, 249, 23, 8, 168, 176, 103, 8, 125, 152, 188, 10, 30, 78, 198, 250, 230, 83, 60, 82,
  240, 9, 184, 176, 130, 2, 126, 161, 146, 71, 116, 107, 23, 90, 108, 50, 87, 82, 15, 166, 178, 35, 58, 94, 136, 86, 148,
  24, 98, 158, 224, 84, 54, 249, 6, 75, 124, 102, 155, 236, 153, 215, 85, 90, 123, 204, 150, 9, 214, 94, 156, 199, 176,
  19, 104, 10, 11, 219, 9, 46, 36, 184, 38, 82, 216, 143, 218, 157, 48, 49, 23, 35, 222, 6, 176, 154, 214, 136, 119, 29,
  98, 13, 157, 30, 63, 59, 250, 240, 248, 232, 201, 9, 64, 120, 39, 155, 249, 4, 154, 86, 191, 150, 44, 174, 96, 46, 209,
  94, 168, 95, 163, 135, 61, 224, 115, 49, 139, 11, 248, 162, 44, 190, 171, 126, 195, 90, 238, 23, 217, 34, 146, 137, 47,
  217, 147, 219, 193, 77, 211, 128, 138, 175, 12, 148, 96, 241, 9, 7, 9, 85, 121, 201, 97, 37, 232, 93, 27, 250, 48, 237,
  149, 192, 41, 216, 153, 132, 62, 159, 79, 87, 130, 218, 179, 161, 98, 112, 183, 18, 172, 136, 133, 94, 226, 253, 223,
  14, 87, 130, 188, 97, 67, 22, 241, 223, 20, 66, 151, 113, 224, 100, 3, 191, 95, 9, 252, 166, 13, 30, 227, 252, 42, 160,
  69, 16, 227, 91, 99, 190, 101, 131, 190, 152, 43, 195, 248, 230, 85, 57, 138, 231, 175, 194, 149, 96, 110, 151, 48,
  223, 239, 217, 76, 249, 230, 248, 228, 248, 209, 241, 179, 227, 211, 31, 63, 124, 127, 244, 35, 78, 22, 12, 8, 251,
  129, 216, 60, 78, 128, 100, 31, 46, 58, 189, 109, 201, 215, 176, 122, 83, 176, 221, 55, 50, 31, 170, 124, 186, 209, 1,
  99, 1, 106, 28, 242, 36, 231, 183, 167, 225, 188, 94, 224, 90, 1, 10, 235, 168, 225, 64, 229, 232, 199, 163, 15, 207,
  31, 190, 254, 254, 135, 87, 80, 239, 163, 110, 59, 252, 87, 22, 167, 54, 186, 142, 254, 149, 27, 4, 254, 117, 141, 21,
  128, 111, 124, 244, 185, 53, 135, 205, 96, 30, 141, 176, 36, 16, 1, 75, 225, 25, 37, 124, 178, 216, 43, 184, 235, 60,
  79, 72, 23, 21, 53, 136, 168, 238, 2, 90, 32, 224, 127, 93, 203, 179, 86, 58, 75, 174, 255, 21, 45, 126, 162, 108, 107,
  29, 54, 180, 64, 115, 30, 138, 132, 121, 99, 93, 156, 201, 58, 128, 106, 43, 101, 173, 225, 141, 199, 71, 233, 21, 36,
  118, 130, 78, 208, 219, 132, 255, 23, 56, 48, 243, 172, 68, 250, 224, 62, 58, 201, 5, 216, 139, 231, 221, 94, 176, 53,
  108, 109, 5, 157, 214, 110, 112, 15, 255, 203, 55, 131, 123, 193, 46, 254, 215, 162, 255, 90, 155, 240, 31, 252, 253,
  105, 218, 9, 186, 189, 112, 43, 128, 178, 65, 23, 254, 215, 105, 117, 59, 1, 251, 196, 4, 124, 169, 24, 202, 180, 118,
  194, 141, 96, 131, 167, 116, 130, 237, 128, 125, 209, 255, 90, 219, 63, 1, 70, 227, 56, 65, 122, 12, 23, 25, 104, 94,
  44, 32, 218, 191, 174, 173, 163, 249, 240, 226, 204, 238, 28, 217, 113, 71, 119, 237, 222, 20, 91, 239, 238, 192, 255,
  183, 186, 237, 205, 0, 254, 107, 109, 180, 187, 248, 223, 97, 119, 43, 232, 117, 218, 144, 187, 209, 134, 95, 208, 161,
  30, 252, 203, 41, 193, 122, 12, 191, 218, 187, 80, 7, 202, 7, 27, 173, 205, 246, 238, 179, 110, 123, 59, 216, 4, 56,
  240, 253, 211, 20, 186, 14, 159, 80, 235, 33, 252, 199, 187, 216, 37, 26, 149, 223, 240, 107, 8, 229, 59, 65, 251, 94,
  171, 13, 109, 180, 218, 27, 9, 166, 1, 224, 246, 22, 160, 180, 219, 222, 1, 224, 240, 79, 175, 189, 3, 205, 109, 193,
  223, 30, 252, 187, 69, 77, 195, 207, 110, 251, 94, 27, 19, 183, 126, 154, 110, 180, 119, 17, 239, 33, 210, 123, 151,
  15, 81, 135, 176, 235, 66, 47, 54, 160, 129, 13, 128, 177, 213, 222, 74, 32, 1, 97, 111, 13, 161, 54, 252, 133, 246,
  176, 42, 20, 217, 104, 33, 16, 74, 68, 26, 108, 182, 182, 224, 239, 61, 250, 183, 141, 189, 5, 64, 4, 110, 23, 176,
  132, 63, 219, 248, 223, 16, 107, 0, 149, 0, 71, 164, 19, 253, 89, 62, 138, 98, 251, 255, 209, 214, 103, 208, 233, 236,
  84, 155, 219, 218, 254, 20, 167, 62, 186, 223, 140, 180, 25, 31, 4, 69, 118, 173, 237, 9, 68, 153, 127, 57, 121, 249,
  162, 61, 15, 179, 60, 170, 83, 8, 142, 147, 34, 205, 64, 133, 67, 117, 232, 24, 36, 125, 221, 33, 126, 26, 164, 208,
  126, 186, 169, 209, 15, 181, 145, 155, 96, 136, 187, 138, 0, 247, 38, 13, 107, 239, 42, 55, 96, 53, 116, 0, 134, 214,
  139, 52, 160, 150, 73, 20, 5, 165, 48, 3, 137, 137, 0, 92, 155, 141, 10, 169, 38, 164, 20, 170, 27, 76, 180, 162, 214,
  138, 26, 199, 152, 135, 233, 128, 111, 228, 240, 134, 185, 63, 210, 129, 190, 139, 71, 239, 161, 36, 17, 136, 126, 127,
  5, 202, 43, 198, 50, 26, 199, 120, 229, 237, 129, 145, 3, 192, 115, 212, 44, 100, 35, 50, 81, 193, 191, 252, 89, 74,
  90, 177, 97, 3, 192, 136, 80, 137, 54, 166, 180, 17, 111, 66, 86, 33, 195, 203, 1, 170, 198, 237, 243, 232, 58, 175,
  179, 241, 35, 250, 151, 123, 191, 120, 100, 118, 141, 52, 87, 94, 111, 142, 23, 28, 81, 42, 183, 39, 97, 254, 242, 114,
  246, 10, 95, 163, 205, 138, 235, 54, 12, 123, 82, 215, 137, 208, 132, 101, 173, 209, 48, 118, 145, 43, 208, 201, 232,
  184, 58, 116, 55, 21, 27, 235, 57, 187, 144, 84, 193, 215, 38, 255, 106, 204, 154, 251, 153, 181, 201, 56, 156, 237,
  61, 226, 241, 181, 209, 207, 70, 227, 115, 184, 55, 135, 246, 87, 231, 94, 187, 219, 88, 243, 56, 63, 186, 154, 227,
  221, 14, 180, 91, 96, 28, 94, 203, 242, 193, 211, 241, 234, 135, 137, 188, 57, 36, 172, 228, 123, 206, 232, 214, 54,
  196, 83, 218, 26, 56, 27, 211, 197, 124, 132, 118, 166, 107, 172, 190, 8, 147, 250, 160, 152, 65, 55, 57, 226, 22, 198,
  3, 220, 228, 155, 118, 26, 110, 190, 162, 147, 179, 125, 89, 23, 141, 68, 214, 210, 78, 22, 35, 158, 138, 17, 0, 227,
  51, 72, 43, 33, 1, 120, 28, 242, 135, 5, 160, 15, 130, 50, 170, 215, 84, 221, 162, 214, 212, 128, 35, 33, 8, 30, 245,
  80, 221, 84, 122, 192, 16, 134, 66, 155, 171, 44, 78, 186, 139, 85, 210, 38, 30, 133, 218, 171, 224, 109, 185, 7, 253,
  183, 69, 36, 14, 30, 211, 236, 33, 76, 202, 90, 91, 121, 35, 162, 86, 78, 116, 98, 57, 167, 121, 144, 243, 202, 62,
  141, 117, 27, 15, 3, 0, 103, 14, 101, 207, 20, 12, 130, 225, 204, 161, 146, 59, 47, 65, 199, 125, 15, 171, 238, 25, 82,
  180, 77, 170, 7, 25, 20, 217, 165, 255, 58, 108, 66, 18, 24, 84, 54, 46, 95, 73, 142, 81, 43, 154, 188, 69, 128, 52,
  82, 0, 29, 132, 98, 89, 107, 40, 124, 183, 183, 146, 112, 193, 13, 54, 212, 54, 112, 47, 193, 52, 133, 248, 145, 235,
  195, 39, 207, 122, 163, 204, 47, 73, 26, 135, 212, 115, 142, 183, 190, 111, 229, 45, 54, 124, 146, 207, 223, 155, 104,
  150, 47, 178, 136, 58, 116, 29, 229, 255, 16, 62, 162, 245, 191, 102, 113, 211, 146, 113, 115, 173, 38, 237, 120, 150,
  195, 18, 244, 112, 244, 151, 16, 157, 59, 208, 58, 94, 175, 13, 34, 64, 15, 175, 247, 1, 199, 24, 91, 142, 134, 123,
  105, 145, 252, 122, 29, 9, 140, 253, 120, 152, 120, 67, 98, 3, 107, 154, 157, 220, 15, 204, 89, 227, 96, 50, 151, 54,
  38, 10, 153, 35, 37, 210, 61, 236, 97, 31, 65, 208, 61, 133, 167, 120, 141, 195, 177, 52, 140, 178, 240, 82, 203, 83,
  165, 61, 244, 62, 47, 30, 206, 226, 105, 136, 160, 158, 100, 225, 52, 170, 51, 11, 170, 85, 173, 122, 226, 44, 230, 84,
  248, 41, 25, 233, 178, 252, 183, 46, 182, 168, 182, 109, 23, 157, 166, 139, 60, 34, 231, 33, 105, 73, 118, 81, 183, 90,
  158, 184, 168, 177, 167, 27, 144, 152, 143, 47, 115, 10, 19, 38, 171, 26, 37, 90, 7, 85, 5, 158, 26, 168, 229, 255,
  240, 7, 245, 19, 245, 111, 50, 231, 93, 21, 176, 136, 185, 51, 234, 181, 222, 8, 116, 240, 62, 29, 44, 232, 224, 249,
  75, 190, 208, 132, 114, 96, 32, 206, 236, 228, 198, 191, 97, 32, 245, 232, 135, 195, 239, 143, 78, 63, 108, 61, 135,
  122, 91, 193, 183, 193, 118, 7, 254, 233, 118, 58, 29, 103, 177, 46, 149, 235, 86, 20, 60, 57, 254, 211, 135, 167, 31,
  158, 163, 57, 109, 91, 148, 242, 20, 125, 252, 240, 71, 86, 16, 54, 166, 213, 37, 159, 29, 191, 57, 250, 240, 246, 248,
  197, 227, 151, 111, 121, 141, 37, 21, 38, 49, 170, 105, 215, 39, 164, 171, 169, 27, 6, 78, 164, 242, 228, 4, 249, 66,
  45, 253, 110, 138, 146, 254, 83, 48, 141, 71, 253, 224, 221, 123, 88, 232, 83, 140, 233, 135, 191, 194, 179, 179, 173,
  254, 167, 96, 176, 24, 158, 71, 69, 191, 213, 109, 6, 249, 98, 218, 239, 52, 161, 205, 197, 172, 32, 83, 31, 150, 233,
  46, 45, 148, 95, 163, 107, 65, 159, 237, 34, 110, 140, 1, 121, 254, 240, 79, 31, 182, 159, 126, 120, 245, 242, 248,
  197, 41, 118, 117, 167, 179, 23, 172, 175, 99, 184, 218, 255, 180, 53, 5, 117, 240, 59, 128, 61, 30, 71, 153, 85, 169,
  183, 169, 212, 234, 118, 89, 181, 222, 38, 212, 235, 186, 42, 226, 166, 49, 161, 75, 134, 104, 200, 6, 85, 251, 33,
  174, 133, 29, 243, 108, 53, 29, 158, 211, 5, 59, 34, 138, 138, 56, 108, 85, 226, 105, 244, 83, 58, 139, 250, 181, 163,
  5, 110, 43, 214, 31, 69, 89, 18, 227, 241, 67, 52, 79, 135, 147, 215, 209, 184, 143, 92, 202, 63, 15, 195, 57, 204,
  220, 104, 244, 176, 160, 212, 114, 127, 132, 136, 176, 123, 25, 216, 136, 182, 172, 114, 62, 57, 122, 243, 225, 240,
  229, 227, 35, 200, 93, 195, 43, 145, 221, 222, 198, 230, 215, 107, 198, 128, 227, 28, 101, 179, 133, 206, 170, 203, 17,
  175, 146, 88, 234, 115, 89, 138, 200, 26, 18, 83, 168, 16, 223, 13, 13, 249, 132, 76, 50, 52, 38, 172, 111, 154, 26, 3,
  172, 120, 141, 154, 98, 67, 201, 170, 185, 107, 145, 8, 177, 33, 128, 32, 177, 18, 117, 236, 170, 178, 43, 132, 74,
  233, 62, 107, 226, 90, 230, 152, 168, 42, 117, 8, 93, 11, 6, 96, 107, 166, 233, 200, 86, 228, 86, 227, 90, 190, 88,
  165, 33, 90, 38, 59, 112, 165, 76, 129, 168, 2, 128, 97, 89, 38, 88, 40, 58, 179, 42, 240, 43, 93, 124, 141, 133, 130,
  210, 76, 204, 74, 239, 212, 178, 116, 153, 102, 150, 190, 118, 148, 190, 246, 150, 22, 177, 127, 203, 178, 34, 197, 44,
  41, 230, 56, 155, 55, 101, 121, 61, 221, 90, 233, 204, 23, 102, 140, 14, 171, 89, 206, 186, 202, 139, 30, 70, 85, 37,
  199, 89, 83, 113, 201, 55, 106, 42, 57, 158, 201, 229, 170, 106, 101, 153, 117, 141, 103, 84, 202, 154, 70, 134, 187,
  77, 179, 146, 154, 234, 26, 11, 186, 149, 163, 15, 3, 37, 153, 101, 201, 84, 130, 30, 244, 120, 204, 88, 150, 215, 146,
  109, 71, 154, 50, 148, 3, 139, 242, 166, 80, 80, 201, 115, 214, 251, 33, 75, 142, 201, 61, 80, 175, 2, 201, 206, 226,
  34, 240, 150, 53, 82, 106, 166, 187, 33, 216, 60, 56, 91, 202, 109, 199, 34, 25, 137, 201, 85, 1, 211, 157, 21, 100,
  104, 36, 39, 114, 50, 215, 221, 24, 223, 218, 185, 200, 199, 243, 156, 245, 148, 216, 108, 70, 45, 37, 103, 89, 77,
  140, 234, 230, 175, 141, 185, 203, 32, 56, 241, 182, 242, 151, 65, 225, 161, 243, 252, 96, 120, 129, 21, 225, 232, 44,
  92, 81, 200, 205, 208, 163, 184, 96, 126, 97, 26, 43, 67, 170, 179, 56, 11, 215, 99, 87, 96, 233, 206, 42, 242, 186,
  162, 81, 69, 166, 87, 77, 52, 161, 92, 57, 230, 25, 101, 185, 57, 84, 137, 128, 98, 178, 167, 146, 229, 225, 110, 53,
  154, 137, 197, 220, 106, 166, 123, 124, 68, 108, 18, 115, 72, 68, 186, 179, 150, 12, 52, 98, 212, 146, 233, 238, 193,
  224, 65, 67, 204, 177, 224, 201, 206, 58, 101, 152, 14, 163, 86, 153, 225, 173, 135, 218, 165, 163, 22, 38, 187, 101,
  145, 140, 141, 97, 10, 35, 153, 225, 229, 24, 30, 178, 194, 193, 51, 60, 199, 139, 37, 57, 163, 56, 208, 164, 116, 143,
  8, 100, 241, 134, 44, 9, 200, 146, 221, 227, 92, 190, 73, 100, 142, 116, 153, 179, 172, 166, 107, 234, 234, 185, 62, 9,
  156, 93, 219, 115, 144, 146, 221, 100, 177, 216, 241, 153, 135, 17, 141, 5, 180, 76, 43, 75, 179, 13, 9, 174, 195, 207,
  133, 133, 66, 85, 230, 148, 108, 182, 101, 169, 37, 241, 69, 84, 179, 179, 223, 176, 59, 116, 88, 98, 194, 53, 101,
  181, 16, 121, 185, 209, 181, 44, 115, 219, 35, 182, 102, 111, 196, 171, 40, 180, 163, 129, 53, 253, 146, 169, 149, 77,
  216, 177, 158, 77, 216, 111, 125, 31, 101, 10, 199, 151, 176, 135, 201, 98, 123, 99, 133, 101, 69, 240, 120, 12, 220,
  239, 202, 151, 193, 246, 81, 95, 200, 28, 68, 144, 5, 30, 71, 225, 136, 30, 170, 82, 246, 143, 90, 129, 135, 116, 161,
  199, 141, 4, 186, 177, 35, 10, 184, 51, 180, 55, 160, 92, 253, 194, 99, 1, 135, 59, 195, 243, 163, 211, 215, 199, 135,
  31, 224, 207, 67, 204, 45, 15, 68, 23, 87, 125, 32, 152, 234, 170, 35, 157, 75, 152, 223, 206, 48, 158, 2, 42, 186, 3,
  13, 58, 230, 244, 63, 57, 93, 113, 116, 103, 29, 119, 245, 97, 218, 43, 27, 101, 190, 54, 134, 27, 142, 172, 167, 121,
  5, 161, 111, 77, 217, 44, 249, 163, 240, 122, 229, 99, 138, 222, 70, 133, 251, 140, 4, 240, 123, 159, 35, 141, 187, 62,
  250, 199, 184, 27, 87, 125, 101, 220, 117, 97, 11, 36, 171, 50, 159, 23, 205, 25, 70, 86, 82, 188, 126, 110, 92, 38,
  43, 124, 138, 86, 218, 24, 170, 45, 53, 70, 5, 97, 172, 137, 128, 205, 133, 85, 2, 39, 14, 18, 216, 43, 119, 89, 13,
  182, 196, 138, 74, 25, 91, 53, 229, 167, 88, 6, 69, 194, 80, 44, 110, 34, 129, 155, 240, 185, 91, 79, 34, 151, 22, 180,
  244, 224, 215, 43, 186, 175, 207, 126, 243, 229, 74, 124, 226, 106, 130, 191, 255, 13, 151, 62, 252, 49, 230, 203, 153,
  40, 192, 156, 65, 107, 53, 229, 235, 100, 49, 30, 199, 87, 50, 13, 5, 183, 252, 96, 171, 5, 125, 230, 236, 108, 20,
  229, 82, 159, 233, 250, 31, 208, 59, 134, 6, 3, 227, 197, 61, 161, 8, 17, 80, 212, 28, 136, 211, 215, 71, 47, 30, 127,
  56, 124, 249, 226, 201, 241, 31, 181, 153, 68, 204, 9, 244, 98, 238, 167, 253, 96, 167, 137, 215, 124, 30, 71, 73, 17,
  246, 131, 78, 187, 179, 131, 141, 102, 100, 34, 235, 180, 55, 182, 148, 81, 15, 220, 124, 234, 135, 181, 89, 130, 234,
  182, 253, 144, 248, 52, 115, 2, 233, 110, 149, 48, 182, 58, 42, 136, 142, 201, 184, 21, 125, 234, 170, 125, 234, 104,
  152, 108, 148, 7, 187, 46, 2, 62, 63, 122, 254, 242, 245, 143, 14, 65, 69, 27, 208, 103, 225, 117, 186, 40, 114, 71,
  54, 217, 154, 74, 230, 196, 157, 163, 144, 244, 76, 0, 138, 47, 52, 227, 2, 26, 159, 110, 204, 246, 249, 245, 209, 163,
  196, 216, 244, 242, 43, 168, 229, 226, 38, 13, 253, 160, 52, 159, 243, 125, 120, 56, 157, 39, 145, 60, 34, 43, 114,
  135, 31, 190, 49, 237, 152, 13, 172, 17, 120, 50, 188, 147, 82, 63, 16, 227, 254, 194, 72, 18, 244, 21, 102, 247, 200,
  106, 104, 120, 249, 234, 5, 253, 110, 199, 249, 139, 240, 69, 29, 48, 210, 15, 146, 60, 205, 182, 177, 25, 104, 251,
  57, 76, 143, 246, 52, 188, 170, 87, 149, 163, 142, 186, 220, 77, 200, 111, 190, 170, 129, 131, 160, 227, 237, 121, 27,
  123, 109, 28, 197, 223, 24, 130, 8, 200, 141, 94, 42, 23, 97, 66, 140, 215, 196, 11, 115, 116, 62, 193, 240, 142, 103,
  117, 217, 1, 81, 166, 193, 10, 89, 131, 56, 129, 181, 157, 15, 177, 117, 99, 69, 178, 68, 163, 228, 14, 229, 172, 22,
  47, 77, 212, 152, 23, 152, 234, 15, 111, 30, 7, 77, 210, 75, 209, 192, 39, 38, 223, 185, 247, 48, 42, 111, 204, 6, 204,
  126, 81, 220, 3, 146, 148, 192, 176, 179, 40, 131, 173, 4, 164, 134, 179, 225, 36, 205, 254, 36, 126, 252, 8, 114, 217,
  100, 46, 5, 79, 243, 112, 167, 196, 91, 189, 72, 243, 81, 189, 255, 197, 46, 150, 31, 124, 243, 137, 126, 220, 176, 75,
  77, 34, 220, 253, 55, 159, 36, 170, 55, 101, 92, 53, 87, 109, 209, 15, 14, 224, 163, 11, 7, 230, 112, 60, 160, 232,
  217, 100, 16, 66, 27, 48, 253, 197, 179, 209, 175, 123, 189, 209, 70, 20, 213, 252, 53, 135, 118, 157, 168, 23, 237,
  140, 59, 206, 58, 229, 72, 97, 148, 244, 139, 200, 30, 172, 210, 163, 159, 140, 20, 26, 229, 17, 188, 180, 124, 99, 28,
  120, 52, 81, 62, 194, 247, 66, 96, 97, 58, 76, 98, 72, 198, 98, 117, 11, 216, 156, 246, 77, 59, 102, 114, 58, 30, 231,
  17, 182, 210, 221, 54, 179, 80, 144, 226, 93, 62, 224, 133, 249, 219, 166, 8, 122, 128, 95, 79, 131, 27, 156, 10, 178,
  75, 43, 162, 128, 209, 5, 176, 63, 56, 81, 234, 156, 129, 130, 239, 56, 10, 180, 56, 23, 109, 42, 243, 29, 98, 203, 19,
  40, 130, 94, 208, 34, 36, 224, 15, 100, 88, 112, 49, 34, 173, 14, 246, 71, 86, 225, 105, 176, 30, 244, 56, 28, 44, 164,
  194, 101, 207, 40, 139, 114, 38, 96, 115, 136, 57, 234, 31, 129, 29, 225, 215, 205, 252, 170, 130, 145, 24, 58, 80, 20,
  126, 168, 37, 173, 57, 200, 68, 203, 211, 48, 71, 105, 227, 56, 21, 230, 46, 112, 124, 249, 240, 8, 38, 203, 47, 232,
  171, 175, 234, 172, 10, 136, 91, 250, 65, 162, 203, 47, 9, 38, 97, 14, 195, 5, 131, 102, 44, 23, 108, 21, 125, 142,
  203, 218, 118, 7, 254, 239, 51, 48, 35, 113, 192, 74, 227, 13, 59, 134, 21, 138, 92, 33, 25, 76, 167, 11, 6, 127, 150,
  94, 178, 85, 239, 225, 124, 158, 165, 87, 71, 120, 218, 244, 28, 143, 173, 31, 60, 160, 208, 24, 109, 40, 80, 183, 239,
  103, 96, 181, 86, 80, 54, 18, 220, 223, 151, 157, 169, 162, 2, 118, 224, 120, 70, 118, 104, 73, 132, 41, 104, 92, 166,
  71, 38, 30, 24, 225, 165, 2, 163, 131, 83, 218, 52, 226, 122, 183, 209, 25, 213, 176, 163, 101, 202, 174, 149, 130,
  145, 123, 106, 13, 199, 225, 249, 121, 116, 205, 184, 135, 161, 112, 211, 130, 95, 80, 235, 230, 163, 122, 52, 206, 81,
  80, 54, 81, 239, 160, 222, 123, 108, 66, 69, 203, 113, 231, 140, 215, 4, 162, 158, 68, 89, 28, 169, 172, 199, 123, 235,
  190, 22, 70, 148, 197, 202, 237, 60, 157, 70, 245, 57, 46, 107, 115, 100, 177, 121, 251, 130, 220, 102, 104, 207, 10,
  223, 124, 237, 159, 183, 11, 109, 241, 247, 243, 31, 63, 188, 103, 42, 203, 75, 138, 154, 231, 184, 165, 105, 29, 47,
  248, 124, 210, 194, 11, 140, 143, 53, 72, 74, 198, 132, 173, 70, 156, 20, 210, 51, 128, 207, 55, 85, 138, 152, 176, 29,
  71, 138, 44, 156, 159, 114, 150, 8, 9, 78, 255, 76, 72, 111, 243, 91, 56, 62, 79, 44, 92, 123, 1, 59, 137, 41, 44, 130,
  195, 100, 49, 138, 242, 122, 89, 89, 173, 133, 169, 242, 202, 253, 62, 204, 31, 168, 111, 230, 243, 39, 17, 173, 220,
  27, 195, 87, 169, 108, 52, 137, 102, 103, 197, 4, 149, 30, 82, 204, 28, 216, 216, 116, 97, 184, 25, 108, 235, 46, 165,
  118, 240, 93, 231, 253, 158, 187, 134, 240, 155, 176, 235, 187, 92, 46, 85, 34, 30, 190, 124, 246, 242, 245, 135, 87,
  15, 159, 29, 157, 158, 30, 57, 238, 248, 12, 175, 67, 60, 177, 198, 247, 33, 96, 163, 206, 62, 8, 112, 95, 174, 232,
  246, 213, 15, 144, 18, 241, 89, 42, 171, 29, 139, 79, 81, 113, 123, 99, 123, 123, 220, 117, 84, 12, 137, 201, 69, 189,
  135, 252, 75, 84, 27, 111, 237, 70, 157, 129, 163, 90, 52, 141, 178, 48, 25, 201, 138, 71, 242, 91, 84, 237, 118, 6,
  187, 59, 174, 22, 231, 241, 236, 92, 214, 123, 197, 62, 68, 165, 104, 184, 185, 179, 187, 235, 168, 148, 69, 101, 91,
  175, 211, 66, 173, 66, 111, 141, 57, 170, 36, 160, 64, 201, 58, 207, 216, 135, 168, 180, 179, 57, 28, 118, 183, 29,
  149, 242, 243, 107, 89, 231, 132, 126, 139, 42, 236, 9, 77, 71, 149, 139, 56, 5, 225, 42, 107, 189, 161, 79, 21, 195,
  112, 103, 107, 107, 124, 207, 213, 24, 190, 164, 92, 54, 199, 191, 68, 181, 221, 205, 112, 99, 224, 106, 48, 75, 243,
  72, 33, 70, 174, 86, 26, 15, 238, 117, 119, 182, 106, 174, 251, 82, 97, 34, 43, 157, 178, 15, 57, 84, 155, 131, 157,
  80, 165, 134, 113, 249, 135, 115, 236, 235, 163, 39, 226, 222, 15, 19, 72, 31, 72, 111, 204, 237, 91, 63, 148, 142,
  183, 122, 245, 173, 165, 118, 123, 224, 80, 150, 169, 244, 176, 230, 10, 101, 136, 114, 199, 121, 53, 64, 71, 206, 242,
  171, 131, 154, 13, 29, 31, 229, 174, 1, 101, 222, 241, 246, 0, 211, 153, 231, 4, 148, 221, 31, 24, 147, 71, 182, 229,
  120, 29, 248, 200, 97, 11, 8, 211, 19, 189, 138, 70, 110, 223, 115, 157, 26, 150, 219, 121, 137, 73, 163, 177, 103,
  118, 181, 170, 131, 220, 53, 85, 239, 162, 31, 127, 80, 74, 38, 39, 212, 42, 168, 116, 153, 169, 135, 76, 84, 171, 106,
  128, 1, 140, 40, 102, 114, 16, 83, 58, 252, 185, 143, 230, 14, 46, 231, 225, 251, 187, 239, 244, 129, 152, 136, 29,
  117, 60, 93, 36, 245, 141, 46, 168, 248, 13, 80, 145, 177, 14, 202, 100, 140, 52, 250, 176, 168, 199, 48, 180, 106, 51,
  150, 82, 64, 32, 194, 65, 94, 159, 248, 215, 120, 34, 194, 147, 52, 123, 76, 214, 42, 227, 2, 47, 45, 155, 152, 196, 1,
  106, 146, 29, 150, 14, 115, 61, 160, 75, 232, 114, 4, 240, 182, 130, 172, 170, 39, 155, 170, 65, 60, 66, 103, 16, 133,
  168, 216, 232, 239, 245, 246, 4, 185, 204, 94, 234, 72, 1, 36, 3, 173, 27, 135, 191, 229, 161, 217, 107, 190, 153, 118,
  116, 158, 244, 98, 158, 105, 235, 53, 106, 167, 196, 62, 83, 185, 79, 109, 115, 121, 245, 64, 124, 31, 93, 75, 149, 79,
  188, 55, 225, 49, 13, 145, 106, 80, 150, 225, 196, 80, 253, 133, 121, 18, 47, 254, 64, 209, 92, 251, 223, 124, 18, 53,
  73, 247, 101, 169, 31, 131, 62, 150, 17, 57, 55, 31, 151, 50, 13, 211, 13, 56, 17, 171, 240, 86, 21, 104, 127, 71, 117,
  70, 194, 210, 232, 173, 89, 210, 24, 21, 105, 23, 67, 97, 186, 94, 87, 246, 78, 7, 32, 146, 157, 80, 100, 166, 69, 65,
  99, 142, 168, 164, 67, 36, 37, 5, 27, 171, 176, 92, 37, 205, 28, 108, 120, 23, 210, 125, 197, 209, 90, 129, 111, 105,
  115, 114, 23, 198, 53, 151, 56, 231, 49, 3, 71, 47, 183, 143, 138, 106, 143, 158, 118, 239, 109, 117, 106, 198, 89,
  144, 35, 157, 172, 207, 181, 231, 79, 255, 220, 221, 173, 25, 71, 56, 65, 237, 228, 233, 233, 70, 183, 230, 50, 122,
  91, 89, 116, 238, 2, 128, 158, 253, 105, 183, 179, 221, 221, 172, 25, 198, 105, 118, 132, 98, 154, 154, 181, 219, 245,
  188, 55, 146, 250, 177, 119, 110, 34, 237, 75, 185, 89, 210, 213, 32, 139, 98, 188, 141, 71, 94, 230, 17, 131, 44, 153,
  199, 97, 144, 208, 230, 186, 13, 95, 178, 104, 187, 72, 127, 152, 207, 209, 203, 18, 180, 3, 63, 191, 146, 239, 188, 4,
  243, 36, 75, 167, 210, 103, 181, 206, 247, 169, 159, 110, 108, 46, 157, 134, 104, 95, 161, 189, 40, 67, 250, 67, 12,
  99, 111, 232, 31, 220, 38, 155, 246, 0, 190, 40, 13, 95, 31, 88, 13, 66, 53, 156, 99, 138, 101, 164, 66, 123, 147, 82,
  9, 248, 200, 168, 4, 41, 86, 59, 9, 122, 231, 71, 106, 91, 44, 197, 108, 143, 165, 218, 134, 177, 112, 172, 54, 9, 159,
  102, 155, 144, 164, 207, 62, 129, 39, 106, 28, 42, 219, 136, 224, 6, 34, 123, 79, 207, 21, 65, 10, 202, 108, 221, 64,
  94, 246, 196, 2, 204, 99, 5, 40, 37, 12, 216, 202, 149, 127, 173, 144, 222, 2, 239, 108, 195, 192, 154, 223, 231, 231,
  153, 230, 106, 223, 179, 107, 176, 128, 8, 60, 75, 47, 95, 55, 56, 180, 141, 168, 35, 41, 213, 222, 129, 208, 182, 202,
  97, 243, 88, 78, 160, 161, 43, 76, 90, 243, 44, 74, 128, 92, 237, 170, 218, 34, 147, 48, 255, 172, 221, 124, 103, 87,
  49, 154, 165, 242, 68, 16, 45, 252, 200, 205, 234, 243, 136, 191, 106, 92, 15, 197, 249, 252, 187, 247, 246, 60, 26,
  92, 31, 134, 133, 161, 74, 179, 10, 210, 200, 145, 59, 77, 28, 52, 217, 242, 54, 63, 106, 37, 51, 15, 217, 239, 92, 22,
  15, 106, 228, 93, 222, 30, 242, 72, 156, 116, 203, 180, 141, 70, 34, 6, 4, 119, 82, 236, 87, 41, 155, 108, 171, 5, 65,
  105, 19, 211, 122, 248, 93, 45, 225, 102, 121, 189, 196, 141, 3, 62, 31, 34, 31, 227, 235, 133, 252, 188, 111, 150,
  115, 53, 133, 163, 235, 158, 1, 74, 190, 139, 6, 192, 239, 206, 121, 80, 230, 238, 185, 14, 188, 44, 238, 52, 211, 5,
  254, 174, 60, 134, 236, 221, 103, 194, 74, 188, 127, 27, 134, 103, 81, 170, 152, 51, 109, 29, 15, 174, 162, 89, 129,
  198, 76, 98, 244, 38, 218, 195, 114, 107, 237, 144, 97, 127, 172, 144, 99, 137, 55, 154, 27, 183, 40, 38, 73, 122, 201,
  148, 144, 120, 120, 30, 145, 119, 53, 54, 209, 118, 100, 236, 155, 151, 117, 57, 102, 229, 173, 110, 32, 138, 251, 214,
  20, 158, 94, 249, 131, 193, 65, 174, 126, 219, 13, 18, 216, 81, 18, 62, 152, 66, 206, 74, 68, 14, 138, 174, 108, 93,
  231, 195, 196, 54, 221, 112, 109, 80, 69, 250, 137, 250, 128, 76, 119, 88, 41, 47, 105, 87, 235, 71, 9, 3, 112, 232,
  56, 177, 42, 78, 180, 88, 86, 205, 81, 154, 159, 190, 201, 119, 239, 5, 90, 134, 178, 104, 222, 57, 94, 29, 43, 170,
  97, 196, 84, 162, 22, 24, 40, 224, 61, 250, 82, 229, 15, 35, 175, 26, 70, 143, 33, 219, 168, 42, 98, 222, 49, 230, 6,
  87, 147, 73, 204, 171, 147, 34, 208, 1, 119, 73, 246, 246, 203, 112, 82, 23, 19, 145, 44, 166, 26, 197, 213, 64, 245,
  1, 39, 191, 154, 86, 211, 65, 232, 219, 95, 237, 170, 204, 39, 35, 80, 44, 63, 50, 156, 87, 225, 41, 12, 229, 123, 70,
  93, 105, 218, 166, 11, 54, 198, 150, 190, 44, 162, 15, 211, 199, 191, 255, 31, 255, 75, 240, 205, 167, 33, 45, 19, 250,
  17, 136, 168, 96, 28, 193, 186, 33, 147, 18, 33, 154, 223, 87, 25, 172, 193, 128, 16, 97, 216, 221, 232, 204, 174, 206,
  169, 172, 14, 54, 212, 50, 186, 120, 227, 30, 26, 71, 20, 188, 9, 30, 49, 201, 203, 139, 159, 28, 184, 242, 67, 20,
  146, 50, 233, 140, 184, 231, 144, 106, 177, 51, 21, 33, 8, 107, 13, 71, 17, 154, 236, 77, 209, 188, 125, 154, 96, 99,
  106, 241, 58, 59, 96, 241, 93, 207, 133, 201, 164, 150, 134, 218, 158, 187, 150, 78, 169, 173, 220, 184, 224, 13, 29,
  129, 0, 87, 67, 167, 88, 82, 91, 20, 243, 29, 249, 40, 163, 167, 194, 193, 153, 189, 204, 150, 36, 64, 123, 151, 128,
  21, 103, 199, 93, 103, 198, 146, 89, 113, 171, 25, 177, 210, 108, 48, 103, 130, 160, 221, 210, 121, 32, 41, 229, 159,
  5, 246, 216, 51, 178, 124, 255, 226, 229, 219, 23, 31, 78, 143, 159, 31, 253, 249, 229, 139, 35, 10, 200, 102, 222, 44,
  172, 253, 112, 122, 8, 255, 242, 212, 103, 233, 108, 132, 225, 14, 107, 15, 167, 17, 40, 11, 225, 250, 139, 232, 242,
  195, 143, 105, 118, 142, 73, 121, 28, 174, 159, 166, 231, 215, 41, 126, 44, 242, 34, 11, 19, 72, 57, 185, 30, 205, 162,
  235, 218, 123, 205, 126, 174, 157, 51, 49, 53, 110, 79, 141, 100, 105, 221, 102, 18, 247, 118, 29, 135, 94, 183, 59,
  181, 146, 205, 191, 102, 179, 214, 190, 253, 36, 154, 82, 210, 248, 176, 60, 240, 101, 244, 131, 90, 111, 19, 215, 80,
  107, 3, 176, 40, 38, 209, 232, 73, 4, 43, 84, 125, 145, 37, 164, 1, 225, 49, 170, 80, 130, 228, 41, 187, 85, 66, 81,
  79, 37, 180, 113, 18, 158, 49, 7, 37, 37, 214, 14, 87, 240, 63, 17, 225, 32, 147, 152, 167, 12, 164, 3, 83, 77, 36,
  178, 200, 168, 229, 185, 62, 65, 216, 19, 9, 162, 20, 178, 151, 90, 171, 171, 126, 172, 117, 215, 246, 202, 187, 218,
  33, 222, 78, 85, 208, 195, 78, 252, 75, 158, 206, 156, 93, 213, 44, 228, 89, 148, 251, 131, 38, 177, 235, 7, 225, 101,
  24, 23, 94, 2, 174, 20, 80, 70, 137, 35, 249, 240, 213, 49, 76, 78, 128, 112, 195, 109, 252, 24, 69, 18, 42, 136, 96,
  154, 154, 122, 11, 40, 77, 240, 29, 13, 200, 119, 153, 215, 41, 238, 27, 115, 179, 87, 5, 146, 217, 13, 94, 132, 245,
  3, 250, 68, 242, 162, 254, 143, 68, 156, 100, 54, 98, 146, 158, 187, 28, 28, 88, 4, 82, 119, 139, 88, 139, 5, 189, 190,
  249, 230, 19, 191, 144, 249, 49, 96, 63, 201, 114, 92, 171, 233, 50, 79, 9, 53, 108, 196, 183, 101, 40, 226, 53, 121,
  87, 190, 129, 45, 194, 151, 60, 91, 21, 105, 139, 23, 81, 14, 191, 168, 234, 45, 169, 141, 213, 181, 211, 174, 160,
  206, 72, 209, 184, 51, 217, 29, 97, 152, 210, 249, 2, 207, 66, 79, 181, 27, 157, 182, 103, 133, 126, 227, 211, 183,
  198, 242, 133, 149, 130, 58, 168, 229, 185, 104, 34, 123, 135, 184, 82, 222, 22, 69, 20, 58, 234, 149, 124, 171, 173,
  177, 82, 148, 177, 38, 126, 250, 37, 22, 220, 226, 167, 234, 181, 86, 207, 55, 186, 176, 202, 50, 200, 151, 25, 78, 58,
  60, 74, 49, 59, 88, 250, 90, 176, 66, 141, 198, 151, 87, 43, 24, 228, 234, 174, 58, 10, 173, 222, 95, 87, 111, 27, 110,
  62, 113, 180, 68, 250, 174, 163, 108, 195, 197, 81, 30, 254, 171, 8, 70, 134, 179, 236, 121, 142, 75, 150, 229, 71,
  203, 148, 108, 177, 232, 72, 79, 37, 101, 169, 218, 171, 172, 192, 99, 32, 187, 134, 12, 96, 1, 170, 204, 217, 153,
  218, 222, 179, 133, 136, 116, 133, 126, 18, 207, 64, 103, 175, 67, 29, 12, 191, 140, 85, 245, 91, 230, 142, 67, 87, 53,
  251, 198, 21, 38, 217, 116, 153, 251, 100, 88, 121, 37, 101, 89, 232, 7, 82, 71, 202, 68, 17, 231, 65, 243, 236, 50,
  243, 203, 192, 15, 178, 88, 195, 37, 44, 93, 112, 191, 11, 234, 165, 7, 95, 208, 170, 128, 221, 184, 51, 21, 216, 126,
  227, 148, 95, 154, 92, 216, 84, 144, 151, 172, 27, 150, 141, 211, 34, 144, 29, 240, 136, 215, 53, 38, 82, 13, 31, 156,
  162, 120, 26, 32, 171, 103, 198, 86, 187, 172, 100, 59, 1, 211, 91, 141, 45, 244, 25, 208, 167, 178, 195, 135, 111, 89,
  235, 236, 121, 231, 91, 224, 64, 46, 227, 94, 4, 60, 19, 93, 187, 114, 238, 156, 3, 220, 123, 212, 230, 198, 189, 101,
  212, 70, 118, 99, 213, 221, 156, 21, 232, 247, 224, 13, 42, 224, 162, 127, 60, 43, 146, 246, 99, 190, 252, 225, 109,
  184, 176, 168, 75, 117, 181, 9, 122, 236, 36, 93, 100, 253, 90, 175, 53, 138, 207, 98, 116, 51, 154, 198, 179, 69, 17,
  169, 41, 57, 222, 114, 27, 169, 41, 72, 187, 63, 99, 96, 20, 167, 104, 98, 177, 27, 177, 33, 68, 0, 219, 174, 83, 31,
  26, 203, 199, 179, 170, 55, 52, 166, 255, 182, 136, 146, 36, 2, 133, 219, 57, 170, 55, 43, 168, 3, 44, 48, 219, 161,
  196, 187, 62, 15, 175, 249, 101, 90, 219, 80, 202, 243, 100, 223, 140, 225, 117, 202, 101, 179, 142, 57, 200, 166, 158,
  225, 89, 31, 252, 96, 110, 204, 80, 226, 140, 157, 168, 14, 19, 242, 162, 46, 229, 124, 152, 230, 232, 40, 44, 210, 96,
  40, 62, 136, 116, 203, 131, 156, 113, 221, 19, 216, 248, 224, 86, 7, 247, 63, 42, 26, 31, 184, 8, 208, 87, 3, 165, 14,
  112, 171, 64, 198, 205, 175, 54, 131, 219, 27, 108, 151, 152, 220, 23, 112, 171, 10, 42, 114, 120, 223, 233, 23, 205,
  89, 206, 28, 88, 222, 45, 35, 36, 42, 106, 41, 101, 215, 150, 119, 195, 10, 18, 234, 107, 6, 151, 204, 142, 101, 252,
  116, 118, 90, 95, 252, 150, 246, 217, 44, 126, 227, 125, 216, 66, 93, 12, 42, 67, 165, 209, 75, 185, 60, 196, 55, 157,
  48, 240, 32, 244, 43, 157, 40, 240, 99, 3, 138, 216, 134, 44, 141, 39, 12, 35, 174, 50, 40, 207, 239, 213, 12, 207,
  220, 153, 56, 125, 50, 136, 233, 104, 154, 155, 225, 204, 72, 249, 236, 133, 22, 157, 114, 85, 241, 227, 34, 235, 222,
  16, 59, 255, 171, 173, 32, 178, 34, 215, 77, 22, 81, 221, 180, 132, 210, 85, 227, 116, 81, 240, 118, 63, 57, 145, 103,
  209, 252, 49, 142, 86, 119, 167, 227, 91, 130, 108, 84, 42, 95, 12, 240, 188, 89, 163, 130, 148, 245, 231, 105, 140,
  81, 229, 200, 90, 155, 235, 80, 240, 57, 171, 138, 151, 111, 164, 106, 94, 241, 228, 137, 25, 169, 152, 31, 182, 139,
  155, 226, 120, 173, 135, 197, 202, 114, 123, 75, 38, 233, 89, 189, 118, 248, 236, 248, 240, 123, 116, 27, 108, 23, 116,
  125, 188, 25, 80, 124, 162, 233, 28, 86, 174, 17, 189, 138, 80, 23, 89, 13, 189, 59, 26, 57, 223, 213, 196, 115, 233,
  181, 102, 173, 124, 24, 26, 62, 52, 246, 124, 95, 17, 138, 120, 217, 115, 14, 234, 147, 14, 106, 23, 208, 7, 174, 198,
  7, 170, 230, 66, 63, 105, 136, 113, 132, 130, 188, 11, 190, 130, 90, 15, 161, 248, 79, 190, 130, 63, 29, 195, 228, 191,
  90, 250, 252, 205, 13, 143, 210, 188, 183, 194, 184, 158, 71, 215, 163, 244, 114, 230, 27, 217, 168, 77, 190, 84, 56,
  229, 143, 242, 97, 56, 143, 140, 93, 194, 59, 157, 212, 198, 48, 200, 225, 49, 135, 192, 148, 76, 146, 242, 77, 110,
  160, 115, 245, 202, 17, 142, 18, 207, 111, 83, 246, 74, 73, 122, 222, 68, 239, 84, 64, 167, 104, 138, 219, 211, 183,
  123, 182, 195, 41, 3, 165, 255, 50, 206, 236, 175, 183, 7, 247, 122, 59, 234, 117, 56, 204, 70, 129, 119, 136, 34, 4,
  139, 140, 210, 162, 53, 95, 64, 23, 90, 241, 40, 137, 12, 209, 40, 60, 13, 240, 186, 9, 67, 213, 150, 146, 166, 177,
  41, 80, 154, 223, 216, 28, 109, 236, 238, 26, 130, 209, 211, 252, 56, 204, 245, 19, 50, 167, 244, 43, 97, 143, 7, 131,
  113, 111, 115, 53, 216, 121, 146, 94, 214, 42, 5, 155, 209, 219, 175, 156, 221, 45, 27, 231, 94, 251, 42, 72, 173, 233,
  90, 229, 93, 160, 187, 193, 177, 5, 167, 118, 118, 107, 28, 219, 150, 133, 210, 171, 147, 73, 56, 162, 155, 47, 31,
  217, 131, 0, 27, 243, 43, 60, 202, 192, 10, 55, 27, 27, 31, 181, 74, 246, 218, 98, 12, 81, 211, 34, 108, 211, 226, 162,
  134, 189, 194, 82, 151, 26, 246, 202, 87, 230, 217, 97, 97, 205, 128, 138, 175, 20, 205, 217, 158, 91, 204, 193, 129,
  107, 25, 242, 238, 181, 168, 139, 254, 247, 14, 88, 246, 140, 35, 3, 148, 172, 229, 244, 110, 99, 70, 6, 102, 218, 0,
  225, 224, 51, 104, 216, 247, 174, 177, 6, 154, 26, 176, 164, 105, 106, 80, 125, 236, 180, 24, 237, 44, 244, 70, 122,
  222, 103, 10, 114, 65, 158, 113, 48, 235, 164, 248, 208, 50, 4, 227, 74, 153, 162, 229, 242, 52, 46, 111, 197, 21, 10,
  236, 16, 43, 208, 80, 111, 94, 8, 167, 205, 95, 165, 113, 108, 202, 104, 157, 7, 38, 80, 26, 71, 71, 66, 103, 227, 152,
  225, 111, 28, 115, 61, 141, 203, 43, 243, 88, 200, 26, 61, 150, 42, 110, 106, 233, 87, 232, 201, 231, 7, 7, 147, 21,
  18, 193, 99, 154, 218, 17, 1, 139, 245, 160, 246, 128, 187, 39, 186, 123, 193, 51, 43, 122, 194, 75, 84, 146, 18, 155,
  53, 72, 169, 70, 139, 248, 149, 177, 17, 77, 27, 24, 49, 127, 93, 157, 181, 208, 253, 210, 205, 91, 152, 83, 193, 92,
  152, 93, 205, 218, 228, 73, 165, 33, 192, 98, 86, 104, 171, 135, 130, 11, 228, 34, 42, 100, 15, 33, 8, 144, 208, 144,
  155, 204, 166, 86, 109, 41, 225, 116, 253, 125, 53, 34, 106, 117, 180, 190, 32, 38, 234, 82, 214, 116, 189, 144, 97,
  92, 20, 14, 231, 170, 179, 48, 81, 93, 196, 93, 225, 200, 243, 79, 35, 140, 11, 107, 88, 15, 139, 36, 35, 125, 134, 9,
  42, 71, 252, 194, 47, 51, 35, 8, 2, 89, 78, 246, 159, 130, 118, 155, 78, 152, 24, 80, 84, 128, 38, 81, 152, 20, 19, 96,
  73, 6, 32, 61, 47, 47, 78, 115, 164, 148, 155, 212, 229, 210, 204, 154, 189, 241, 239, 34, 89, 71, 143, 243, 167, 12,
  254, 157, 150, 2, 205, 225, 218, 183, 164, 52, 218, 188, 11, 126, 92, 96, 172, 30, 195, 74, 135, 47, 31, 187, 44, 225,
  234, 209, 45, 14, 140, 99, 177, 112, 88, 92, 185, 25, 80, 139, 103, 5, 74, 254, 165, 6, 202, 200, 198, 248, 86, 246,
  41, 177, 13, 21, 49, 186, 31, 216, 176, 101, 21, 71, 179, 45, 219, 98, 143, 96, 14, 92, 56, 168, 70, 126, 168, 232, 40,
  97, 177, 78, 103, 217, 80, 159, 162, 141, 193, 123, 141, 127, 56, 70, 219, 146, 26, 143, 71, 159, 10, 34, 104, 205,
  182, 30, 180, 70, 139, 89, 211, 51, 163, 231, 152, 115, 161, 242, 118, 55, 11, 103, 214, 16, 183, 161, 203, 43, 220,
  158, 43, 219, 90, 214, 197, 18, 149, 2, 74, 52, 12, 117, 139, 251, 178, 211, 45, 227, 251, 65, 175, 60, 95, 13, 70,
  113, 214, 135, 157, 121, 88, 80, 164, 163, 2, 227, 38, 201, 168, 56, 172, 223, 118, 215, 24, 125, 200, 228, 196, 174,
  161, 39, 120, 181, 165, 5, 100, 109, 179, 44, 163, 117, 81, 126, 117, 12, 68, 141, 119, 102, 213, 86, 208, 125, 255, 0,
  40, 240, 224, 193, 82, 44, 25, 172, 96, 191, 26, 88, 251, 194, 172, 55, 142, 51, 189, 90, 199, 81, 104, 20, 143, 199,
  36, 52, 168, 141, 22, 171, 100, 107, 110, 103, 226, 94, 31, 94, 202, 195, 58, 150, 185, 115, 114, 13, 219, 88, 144,
  109, 49, 42, 247, 72, 65, 193, 113, 193, 183, 24, 175, 201, 10, 29, 146, 69, 23, 143, 227, 76, 178, 47, 139, 134, 164,
  178, 47, 35, 165, 190, 189, 27, 81, 13, 51, 135, 189, 165, 123, 6, 179, 82, 109, 183, 193, 75, 83, 15, 81, 207, 121,
  16, 212, 22, 115, 178, 184, 208, 46, 91, 221, 26, 240, 93, 146, 13, 4, 40, 162, 116, 12, 221, 251, 37, 226, 200, 186,
  8, 15, 80, 213, 210, 8, 120, 67, 180, 206, 179, 148, 247, 157, 92, 221, 69, 44, 51, 189, 171, 204, 228, 194, 72, 100,
  203, 50, 241, 28, 2, 54, 136, 190, 210, 13, 167, 243, 111, 94, 188, 202, 232, 10, 136, 42, 171, 105, 132, 27, 123, 206,
  226, 47, 104, 43, 165, 150, 102, 140, 97, 157, 174, 72, 216, 234, 97, 158, 128, 224, 59, 95, 17, 81, 250, 18, 22, 104,
  66, 128, 104, 137, 138, 182, 169, 167, 100, 57, 54, 162, 247, 141, 17, 86, 137, 84, 155, 69, 11, 116, 220, 50, 182,
  205, 52, 184, 106, 57, 214, 62, 103, 136, 34, 189, 68, 91, 9, 50, 69, 120, 25, 94, 215, 150, 153, 124, 133, 238, 145,
  177, 137, 183, 79, 83, 227, 128, 49, 60, 147, 169, 8, 149, 253, 66, 168, 117, 37, 95, 242, 20, 150, 72, 199, 5, 123,
  116, 8, 89, 185, 225, 208, 41, 128, 33, 132, 24, 145, 194, 1, 89, 185, 41, 91, 111, 42, 221, 170, 208, 25, 176, 244,
  232, 97, 150, 165, 151, 48, 106, 150, 35, 228, 72, 97, 229, 242, 46, 230, 223, 255, 231, 255, 213, 152, 95, 35, 157,
  189, 149, 146, 255, 97, 95, 220, 172, 213, 150, 157, 160, 126, 63, 143, 229, 26, 66, 234, 85, 147, 225, 105, 175, 113,
  236, 248, 235, 13, 63, 201, 97, 62, 100, 98, 189, 194, 16, 67, 119, 223, 166, 182, 139, 244, 73, 124, 21, 141, 234, 2,
  28, 70, 98, 175, 253, 253, 223, 255, 195, 242, 214, 95, 182, 206, 122, 214, 79, 115, 162, 146, 179, 96, 67, 190, 231,
  91, 59, 159, 199, 60, 102, 162, 210, 201, 58, 39, 7, 114, 140, 68, 172, 225, 7, 42, 47, 137, 232, 128, 159, 150, 119,
  71, 62, 3, 56, 222, 3, 209, 225, 30, 210, 205, 144, 207, 0, 201, 164, 149, 10, 242, 13, 221, 245, 184, 5, 72, 110, 204,
  64, 126, 57, 74, 158, 91, 86, 5, 182, 53, 229, 129, 237, 56, 141, 73, 143, 170, 53, 220, 251, 198, 178, 168, 160, 154,
  163, 56, 237, 216, 203, 146, 64, 7, 71, 33, 218, 120, 149, 133, 80, 253, 99, 133, 252, 251, 24, 222, 13, 58, 188, 19,
  29, 242, 68, 59, 226, 5, 124, 135, 67, 108, 193, 161, 66, 15, 218, 248, 225, 89, 68, 135, 52, 113, 16, 191, 22, 21,
  102, 85, 90, 174, 69, 181, 114, 149, 161, 3, 88, 214, 154, 34, 95, 137, 209, 153, 84, 109, 248, 154, 58, 75, 211, 145,
  38, 106, 229, 10, 236, 134, 71, 162, 217, 11, 109, 16, 122, 128, 33, 13, 190, 218, 23, 250, 130, 183, 62, 185, 39, 84,
  26, 53, 93, 192, 204, 51, 199, 188, 164, 125, 185, 60, 112, 143, 158, 148, 189, 106, 238, 166, 3, 10, 157, 37, 40, 185,
  215, 34, 206, 14, 218, 69, 13, 192, 195, 58, 51, 68, 209, 47, 144, 43, 215, 129, 61, 11, 142, 238, 34, 192, 170, 33,
  11, 169, 18, 209, 231, 3, 0, 242, 252, 81, 152, 213, 111, 111, 146, 36, 137, 13, 98, 93, 23, 227, 203, 132, 56, 19,
  225, 150, 157, 177, 92, 91, 196, 253, 186, 11, 124, 46, 78, 32, 193, 172, 71, 77, 109, 107, 197, 133, 114, 195, 9, 68,
  185, 128, 167, 3, 146, 134, 31, 3, 88, 41, 140, 221, 0, 217, 189, 58, 29, 22, 218, 218, 12, 48, 36, 118, 221, 16, 216,
  197, 56, 29, 2, 90, 77, 12, 8, 52, 91, 27, 203, 70, 13, 132, 152, 242, 196, 216, 237, 7, 79, 216, 50, 184, 127, 131,
  214, 39, 105, 34, 178, 4, 30, 189, 241, 229, 121, 106, 71, 127, 25, 236, 29, 238, 202, 90, 44, 97, 127, 13, 128, 174,
  189, 55, 237, 239, 88, 140, 189, 114, 105, 157, 216, 10, 236, 96, 226, 185, 78, 107, 121, 120, 130, 121, 92, 133, 12,
  78, 76, 188, 251, 178, 2, 38, 80, 180, 129, 224, 110, 135, 135, 111, 104, 240, 189, 152, 167, 81, 56, 226, 161, 94,
  115, 109, 116, 108, 231, 154, 50, 183, 220, 96, 43, 53, 112, 54, 165, 244, 200, 171, 33, 185, 124, 103, 16, 104, 210,
  114, 229, 53, 49, 163, 4, 124, 227, 58, 195, 249, 205, 190, 86, 169, 199, 22, 52, 76, 95, 78, 142, 255, 204, 23, 47,
  249, 186, 156, 22, 199, 35, 45, 254, 222, 227, 180, 248, 184, 103, 92, 125, 77, 139, 58, 21, 108, 74, 155, 97, 83, 183,
  24, 54, 117, 123, 225, 63, 205, 211, 154, 165, 13, 172, 158, 164, 151, 44, 28, 189, 231, 62, 45, 63, 130, 98, 183, 15,
  244, 199, 124, 47, 203, 243, 12, 97, 14, 244, 175, 15, 104, 212, 195, 21, 2, 75, 246, 45, 171, 54, 139, 135, 47, 160,
  73, 235, 161, 31, 28, 25, 247, 16, 30, 149, 101, 0, 253, 10, 29, 187, 138, 55, 34, 159, 193, 171, 130, 172, 136, 168,
  55, 56, 140, 139, 176, 184, 82, 17, 2, 235, 40, 163, 155, 12, 173, 32, 255, 88, 215, 240, 10, 231, 77, 67, 15, 232, 14,
  95, 14, 3, 55, 99, 158, 180, 236, 221, 82, 30, 23, 241, 141, 124, 50, 11, 228, 74, 166, 221, 229, 19, 254, 181, 12, 82,
  213, 54, 206, 130, 4, 179, 121, 24, 61, 166, 240, 145, 194, 135, 171, 234, 222, 52, 195, 240, 60, 158, 159, 146, 8,
  103, 107, 220, 27, 246, 82, 86, 125, 201, 108, 100, 145, 114, 212, 87, 23, 132, 138, 90, 190, 110, 38, 95, 84, 216, 87,
  222, 84, 32, 19, 142, 138, 40, 249, 200, 148, 128, 26, 166, 183, 49, 34, 197, 138, 214, 87, 122, 177, 89, 44, 172, 20,
  2, 30, 159, 5, 98, 82, 38, 156, 171, 79, 214, 53, 233, 61, 5, 245, 113, 20, 113, 21, 202, 140, 164, 59, 227, 79, 37,
  232, 158, 205, 198, 83, 124, 104, 23, 193, 103, 13, 59, 214, 142, 32, 85, 31, 212, 128, 230, 180, 135, 46, 228, 130,
  172, 191, 187, 1, 197, 244, 183, 54, 100, 108, 7, 253, 53, 12, 236, 2, 118, 202, 92, 7, 161, 81, 157, 136, 144, 224,
  58, 212, 231, 125, 123, 80, 250, 82, 224, 226, 232, 114, 27, 80, 0, 40, 7, 254, 101, 253, 242, 228, 63, 59, 27, 132,
  245, 173, 94, 179, 215, 237, 54, 187, 91, 27, 77, 52, 111, 55, 8, 174, 81, 166, 183, 177, 219, 220, 222, 193, 255, 103,
  69, 124, 55, 15, 216, 37, 158, 194, 246, 160, 45, 91, 103, 191, 168, 145, 116, 60, 166, 223, 70, 144, 5, 219, 29, 124,
  224, 240, 179, 230, 44, 2, 16, 31, 190, 34, 211, 145, 206, 35, 144, 254, 246, 217, 195, 23, 212, 206, 75, 222, 142, 38,
  202, 25, 76, 7, 161, 75, 192, 50, 104, 164, 11, 250, 215, 189, 222, 112, 107, 43, 210, 70, 193, 219, 64, 233, 230, 209,
  25, 119, 239, 245, 194, 218, 42, 115, 3, 141, 168, 63, 241, 87, 240, 234, 236, 33, 197, 38, 62, 116, 202, 68, 154, 29,
  158, 149, 61, 173, 71, 209, 160, 202, 66, 210, 64, 198, 3, 73, 119, 202, 40, 210, 14, 219, 121, 134, 254, 137, 210, 6,
  206, 99, 251, 188, 138, 175, 162, 228, 53, 229, 0, 240, 174, 47, 68, 182, 124, 173, 111, 149, 56, 212, 195, 60, 127,
  171, 70, 147, 239, 54, 217, 111, 26, 132, 58, 69, 135, 38, 132, 27, 174, 154, 79, 151, 212, 100, 29, 180, 171, 94, 138,
  122, 172, 44, 225, 240, 45, 235, 179, 109, 159, 183, 202, 62, 117, 148, 165, 85, 132, 117, 156, 208, 165, 181, 131, 86,
  42, 158, 202, 80, 161, 228, 137, 97, 116, 86, 171, 65, 165, 61, 59, 143, 87, 134, 170, 78, 15, 108, 49, 200, 24, 50,
  241, 52, 11, 103, 57, 218, 132, 128, 30, 29, 248, 31, 253, 171, 118, 170, 44, 60, 12, 19, 140, 23, 9, 29, 105, 154,
  221, 49, 153, 5, 41, 84, 242, 11, 209, 160, 194, 114, 58, 67, 147, 84, 2, 44, 139, 243, 167, 110, 134, 109, 214, 227,
  51, 227, 37, 218, 210, 42, 202, 174, 212, 186, 75, 110, 171, 5, 183, 253, 229, 110, 17, 241, 89, 28, 233, 66, 186, 109,
  139, 213, 30, 28, 178, 59, 201, 236, 110, 143, 195, 107, 140, 206, 54, 50, 156, 133, 249, 123, 145, 236, 124, 140, 2,
  123, 62, 1, 221, 231, 199, 8, 182, 250, 22, 131, 161, 223, 16, 143, 129, 40, 10, 63, 7, 225, 54, 169, 99, 0, 200, 110,
  163, 61, 15, 241, 73, 149, 172, 168, 247, 154, 32, 142, 237, 229, 101, 100, 215, 167, 251, 14, 141, 170, 170, 188, 147,
  160, 90, 95, 83, 84, 107, 252, 71, 139, 255, 103, 133, 36, 11, 175, 233, 18, 52, 6, 197, 11, 175, 115, 104, 234, 58,
  119, 184, 38, 206, 70, 252, 206, 7, 67, 193, 225, 124, 144, 21, 158, 18, 148, 135, 44, 76, 25, 0, 169, 236, 9, 44, 221,
  212, 32, 30, 216, 53, 92, 108, 138, 207, 48, 247, 245, 33, 33, 112, 160, 120, 23, 169, 145, 65, 214, 242, 10, 238, 197,
  187, 161, 143, 232, 101, 225, 122, 18, 227, 33, 194, 48, 156, 55, 3, 242, 111, 181, 68, 45, 79, 181, 28, 46, 113, 51,
  129, 112, 234, 172, 128, 25, 45, 44, 47, 202, 136, 207, 0, 189, 193, 42, 192, 190, 23, 143, 81, 241, 129, 26, 165, 68,
  139, 74, 248, 209, 13, 71, 35, 134, 45, 15, 25, 143, 207, 211, 68, 44, 10, 207, 53, 62, 191, 209, 228, 207, 36, 63,
  207, 197, 185, 7, 16, 161, 201, 154, 53, 135, 47, 60, 67, 123, 189, 40, 79, 83, 165, 124, 186, 250, 1, 189, 124, 19,
  181, 241, 133, 230, 160, 95, 126, 116, 183, 236, 176, 109, 116, 126, 42, 91, 99, 115, 110, 26, 143, 106, 18, 8, 124,
  72, 24, 248, 248, 179, 165, 51, 17, 14, 199, 20, 36, 148, 164, 239, 56, 73, 97, 79, 5, 58, 241, 186, 196, 207, 244, 40,
  60, 59, 107, 179, 44, 106, 175, 213, 109, 4, 106, 82, 9, 210, 208, 50, 100, 75, 247, 213, 242, 32, 46, 202, 156, 150,
  154, 67, 67, 6, 75, 64, 79, 23, 227, 203, 218, 98, 37, 114, 114, 19, 236, 152, 201, 244, 110, 181, 158, 81, 202, 247,
  203, 9, 110, 243, 21, 68, 15, 148, 198, 92, 7, 152, 50, 230, 183, 132, 204, 78, 237, 4, 2, 235, 74, 78, 223, 186, 177,
  225, 230, 255, 79, 1, 8, 126, 165, 143, 223, 42, 108, 117, 209, 231, 77, 234, 193, 76, 148, 210, 223, 237, 171, 234,
  194, 221, 73, 193, 92, 114, 209, 130, 226, 62, 43, 21, 112, 161, 61, 42, 230, 6, 174, 99, 83, 165, 119, 225, 11, 85,
  90, 4, 198, 242, 212, 45, 111, 234, 55, 149, 42, 111, 41, 10, 233, 199, 94, 74, 215, 159, 66, 247, 60, 208, 0, 121,
  190, 3, 139, 34, 215, 110, 90, 21, 246, 29, 42, 216, 240, 156, 146, 97, 61, 87, 174, 65, 161, 95, 34, 126, 247, 93,
  151, 252, 30, 4, 245, 101, 143, 59, 224, 49, 159, 242, 181, 103, 201, 126, 102, 100, 83, 206, 20, 121, 92, 147, 202,
  83, 69, 138, 97, 255, 32, 16, 81, 53, 236, 35, 123, 115, 188, 189, 111, 42, 81, 167, 181, 5, 5, 165, 139, 188, 28, 165,
  140, 214, 3, 22, 242, 162, 175, 21, 81, 2, 239, 85, 138, 84, 106, 166, 89, 10, 198, 38, 19, 109, 77, 253, 81, 249, 198,
  221, 192, 117, 9, 30, 74, 68, 14, 176, 124, 112, 190, 98, 13, 192, 219, 224, 176, 254, 191, 194, 197, 70, 128, 55, 213,
  46, 193, 24, 92, 53, 163, 117, 218, 214, 211, 246, 156, 43, 130, 90, 75, 40, 109, 234, 122, 208, 213, 22, 132, 45, 183,
  48, 39, 158, 245, 64, 42, 123, 15, 128, 36, 101, 151, 202, 119, 167, 203, 218, 231, 203, 64, 169, 86, 172, 34, 242, 42,
  111, 126, 251, 222, 245, 248, 236, 113, 169, 18, 37, 236, 108, 29, 120, 178, 79, 17, 248, 18, 114, 88, 123, 247, 222,
  231, 198, 85, 53, 46, 143, 31, 254, 248, 225, 249, 9, 74, 12, 179, 192, 54, 229, 159, 28, 255, 233, 195, 83, 86, 226,
  217, 241, 155, 163, 15, 111, 143, 95, 60, 126, 249, 22, 18, 236, 247, 134, 240, 69, 61, 228, 207, 211, 138, 230, 234,
  165, 66, 240, 174, 252, 169, 187, 126, 21, 40, 134, 234, 82, 139, 120, 39, 127, 153, 197, 124, 50, 209, 33, 252, 48,
  230, 165, 34, 9, 26, 171, 74, 195, 186, 218, 43, 61, 215, 210, 242, 227, 25, 181, 46, 183, 171, 29, 62, 245, 1, 95, 49,
  14, 13, 143, 34, 85, 57, 253, 144, 66, 114, 246, 77, 213, 192, 121, 124, 155, 76, 231, 252, 30, 16, 186, 132, 1, 48,
  154, 12, 243, 120, 52, 146, 118, 122, 27, 199, 69, 76, 59, 216, 103, 36, 104, 72, 103, 65, 69, 171, 149, 231, 98, 76,
  142, 225, 218, 230, 144, 104, 101, 31, 12, 205, 79, 84, 195, 166, 216, 79, 181, 65, 230, 162, 200, 180, 113, 150, 107,
  239, 32, 176, 72, 149, 75, 41, 40, 178, 79, 217, 76, 147, 243, 152, 52, 251, 156, 205, 231, 127, 132, 42, 112, 231,
  213, 237, 142, 2, 39, 156, 197, 5, 47, 206, 186, 14, 92, 60, 103, 227, 95, 39, 73, 9, 163, 76, 114, 177, 62, 191, 189,
  179, 42, 224, 140, 37, 197, 133, 137, 70, 195, 237, 203, 249, 70, 184, 151, 11, 100, 108, 54, 84, 159, 64, 194, 91,
  137, 243, 186, 185, 251, 82, 32, 149, 120, 42, 169, 230, 99, 73, 126, 133, 67, 171, 100, 52, 227, 144, 151, 70, 216,
  86, 185, 15, 82, 251, 195, 39, 135, 79, 149, 8, 148, 85, 182, 220, 109, 88, 34, 240, 129, 186, 103, 90, 34, 42, 161,
  187, 235, 229, 146, 139, 82, 173, 213, 245, 52, 232, 208, 216, 149, 92, 151, 226, 94, 250, 148, 184, 4, 144, 139, 32,
  36, 206, 60, 20, 113, 232, 66, 26, 14, 93, 139, 38, 234, 242, 225, 32, 74, 197, 66, 163, 146, 165, 91, 73, 151, 110,
  53, 97, 186, 85, 148, 249, 255, 28, 67, 120, 101, 44, 30, 185, 70, 195, 226, 85, 24, 103, 120, 38, 236, 84, 146, 208,
  229, 36, 183, 92, 234, 133, 203, 138, 83, 108, 77, 22, 83, 71, 13, 197, 63, 197, 89, 235, 98, 62, 114, 212, 98, 46, 36,
  206, 10, 9, 61, 247, 37, 159, 22, 37, 68, 57, 77, 155, 132, 131, 252, 64, 208, 252, 195, 130, 146, 46, 10, 227, 45, 59,
  215, 59, 50, 80, 217, 241, 128, 12, 212, 101, 107, 220, 39, 238, 199, 71, 56, 188, 83, 49, 193, 3, 48, 192, 243, 187,
  32, 70, 15, 122, 194, 171, 79, 200, 189, 83, 48, 52, 11, 145, 103, 30, 34, 253, 78, 193, 92, 47, 228, 188, 191, 45,
  151, 86, 68, 76, 147, 210, 44, 20, 181, 234, 255, 60, 71, 191, 32, 35, 5, 90, 43, 229, 248, 50, 43, 44, 70, 52, 120,
  134, 17, 127, 235, 229, 138, 76, 238, 218, 184, 179, 162, 67, 63, 185, 91, 119, 45, 207, 176, 155, 255, 73, 79, 23, 1,
  79, 154, 1, 15, 10, 246, 152, 121, 64, 240, 83, 72, 246, 176, 37, 79, 171, 213, 62, 71, 157, 103, 75, 188, 35, 172,
  145, 238, 244, 173, 170, 141, 69, 46, 238, 27, 156, 230, 150, 131, 185, 75, 79, 199, 243, 242, 165, 226, 87, 206, 152,
  137, 110, 114, 99, 24, 172, 7, 27, 244, 240, 100, 199, 136, 43, 192, 149, 219, 169, 94, 135, 87, 250, 189, 172, 4, 245,
  183, 29, 181, 75, 43, 52, 55, 96, 79, 38, 154, 209, 26, 109, 214, 248, 116, 13, 207, 157, 78, 237, 220, 143, 190, 200,
  184, 110, 204, 68, 111, 108, 108, 184, 154, 147, 123, 122, 178, 45, 250, 209, 53, 43, 90, 157, 112, 161, 89, 118, 34,
  207, 43, 59, 97, 186, 204, 11, 63, 128, 21, 98, 36, 149, 17, 145, 128, 157, 209, 1, 17, 227, 184, 154, 126, 213, 38,
  35, 176, 123, 72, 220, 237, 129, 130, 44, 33, 75, 11, 160, 174, 179, 6, 105, 187, 119, 216, 157, 10, 49, 17, 87, 139,
  252, 132, 173, 202, 64, 77, 166, 86, 175, 204, 60, 231, 204, 128, 244, 91, 52, 245, 201, 77, 30, 148, 22, 51, 124, 231,
  189, 36, 227, 40, 188, 46, 63, 149, 64, 82, 35, 107, 174, 125, 165, 200, 1, 244, 135, 144, 24, 161, 224, 42, 243, 26,
  10, 139, 200, 50, 55, 1, 123, 15, 153, 125, 124, 172, 16, 160, 178, 84, 229, 225, 12, 240, 211, 116, 78, 226, 207, 39,
  203, 28, 119, 25, 125, 116, 171, 69, 179, 214, 225, 195, 90, 5, 213, 174, 163, 16, 184, 17, 84, 100, 140, 9, 124, 27,
  42, 170, 220, 83, 33, 215, 135, 44, 110, 200, 143, 236, 5, 88, 50, 181, 228, 230, 253, 5, 251, 45, 154, 171, 55, 228,
  21, 43, 133, 101, 187, 205, 238, 160, 230, 174, 237, 184, 90, 22, 214, 108, 87, 89, 26, 100, 51, 52, 32, 107, 134, 30,
  217, 179, 243, 8, 108, 67, 57, 109, 135, 148, 126, 135, 30, 22, 239, 227, 245, 193, 121, 56, 235, 107, 119, 29, 112,
  113, 7, 113, 48, 138, 70, 207, 99, 212, 33, 24, 4, 103, 126, 120, 69, 247, 102, 174, 222, 152, 55, 43, 69, 199, 247,
  69, 117, 215, 124, 97, 111, 78, 75, 210, 200, 27, 66, 162, 59, 120, 219, 172, 211, 83, 104, 204, 236, 101, 84, 110,
  158, 94, 214, 187, 208, 139, 150, 122, 227, 163, 171, 205, 7, 173, 15, 12, 31, 122, 199, 217, 81, 70, 233, 7, 123, 5,
  122, 73, 96, 12, 7, 246, 117, 217, 132, 232, 48, 161, 191, 213, 12, 220, 253, 234, 170, 25, 106, 141, 138, 78, 48, 14,
  185, 75, 39, 108, 67, 34, 114, 65, 9, 157, 177, 67, 9, 137, 243, 69, 217, 189, 178, 141, 150, 90, 11, 240, 237, 0, 198,
  149, 135, 151, 73, 56, 163, 216, 117, 241, 240, 60, 175, 15, 139, 171, 199, 89, 120, 137, 243, 152, 155, 41, 112, 213,
  65, 101, 40, 154, 141, 240, 207, 60, 73, 139, 183, 120, 196, 47, 212, 35, 20, 31, 180, 182, 16, 0, 232, 226, 22, 33,
  43, 190, 144, 9, 216, 177, 158, 72, 97, 151, 48, 233, 185, 133, 87, 128, 41, 179, 222, 116, 55, 131, 155, 47, 96, 227,
  4, 162, 24, 14, 30, 132, 54, 123, 80, 58, 211, 181, 31, 254, 170, 160, 166, 173, 57, 141, 94, 232, 231, 38, 196, 165,
  36, 71, 241, 19, 187, 200, 47, 115, 56, 125, 32, 221, 14, 37, 166, 232, 126, 106, 123, 15, 60, 112, 251, 142, 103, 76,
  70, 81, 142, 251, 156, 71, 97, 30, 201, 131, 82, 70, 209, 7, 101, 127, 123, 26, 173, 17, 144, 204, 18, 3, 164, 185,
  188, 200, 193, 4, 45, 101, 167, 99, 219, 80, 120, 163, 170, 208, 19, 67, 219, 212, 90, 85, 176, 115, 26, 50, 79, 230,
  225, 144, 13, 244, 230, 142, 29, 255, 255, 108, 202, 99, 118, 169, 3, 39, 218, 110, 105, 98, 67, 190, 132, 42, 184,
  105, 201, 222, 103, 95, 130, 119, 236, 128, 4, 24, 182, 13, 82, 232, 194, 71, 3, 227, 156, 18, 75, 173, 75, 40, 40, 4,
  226, 70, 195, 175, 125, 17, 95, 63, 33, 143, 45, 92, 176, 112, 255, 226, 218, 113, 148, 140, 102, 79, 49, 117, 94, 41,
  236, 162, 110, 34, 28, 198, 54, 180, 51, 17, 124, 164, 138, 236, 25, 90, 220, 4, 74, 246, 192, 132, 87, 84, 227, 45,
  119, 36, 82, 129, 180, 129, 246, 139, 97, 4, 114, 19, 230, 42, 117, 163, 100, 37, 114, 37, 67, 49, 209, 158, 70, 97,
  190, 200, 40, 238, 101, 61, 105, 112, 191, 43, 16, 59, 174, 153, 137, 12, 240, 34, 138, 70, 145, 38, 151, 117, 28, 190,
  211, 228, 66, 83, 97, 29, 23, 242, 135, 104, 46, 120, 116, 141, 37, 34, 21, 102, 175, 169, 42, 228, 42, 147, 171, 12,
  166, 161, 212, 96, 174, 51, 206, 199, 131, 116, 132, 5, 112, 207, 100, 48, 230, 25, 150, 226, 156, 220, 52, 81, 110,
  52, 26, 142, 83, 154, 104, 238, 114, 81, 27, 70, 113, 82, 151, 163, 202, 247, 212, 235, 2, 63, 199, 149, 189, 213, 166,
  71, 96, 128, 196, 196, 239, 246, 9, 139, 6, 131, 193, 230, 134, 40, 246, 46, 126, 111, 221, 10, 193, 228, 66, 69, 11,
  77, 66, 36, 27, 101, 45, 19, 113, 44, 225, 134, 239, 44, 185, 231, 154, 99, 116, 88, 189, 156, 199, 65, 153, 15, 19,
  88, 23, 5, 143, 39, 38, 119, 39, 141, 253, 131, 59, 178, 118, 121, 234, 199, 70, 156, 193, 110, 234, 243, 170, 111,
  160, 176, 138, 23, 28, 179, 33, 213, 197, 129, 65, 181, 18, 203, 116, 80, 148, 55, 172, 60, 170, 153, 239, 222, 55, 74,
  99, 251, 252, 65, 251, 66, 198, 141, 184, 192, 20, 253, 50, 249, 234, 87, 193, 12, 61, 151, 181, 44, 108, 83, 37, 57,
  24, 30, 236, 44, 145, 244, 24, 168, 206, 53, 24, 250, 133, 246, 21, 17, 50, 130, 233, 49, 14, 127, 210, 105, 60, 91,
  166, 108, 75, 65, 176, 154, 6, 143, 77, 189, 166, 55, 180, 167, 164, 39, 1, 88, 143, 2, 33, 75, 98, 208, 83, 88, 96,
  187, 120, 52, 194, 210, 42, 79, 69, 124, 35, 96, 189, 193, 71, 15, 171, 137, 147, 14, 103, 220, 22, 118, 200, 161, 158,
  43, 195, 48, 22, 120, 100, 216, 161, 51, 19, 51, 154, 142, 138, 143, 163, 7, 157, 54, 30, 184, 35, 84, 214, 113, 180,
  75, 96, 25, 135, 89, 66, 59, 154, 33, 112, 55, 238, 232, 248, 229, 49, 45, 70, 50, 32, 154, 191, 211, 88, 130, 102,
  175, 99, 186, 8, 254, 80, 23, 193, 41, 87, 114, 5, 119, 16, 88, 25, 178, 131, 198, 165, 98, 230, 140, 96, 198, 62, 131,
  109, 251, 33, 198, 97, 84, 29, 158, 153, 50, 171, 159, 188, 77, 163, 34, 172, 186, 46, 161, 123, 67, 63, 166, 247, 209,
  203, 7, 81, 242, 232, 115, 15, 198, 148, 233, 76, 70, 20, 249, 13, 221, 69, 220, 30, 104, 41, 150, 211, 244, 181, 80,
  125, 168, 50, 255, 130, 130, 86, 224, 143, 49, 236, 176, 79, 0, 19, 81, 84, 126, 163, 43, 118, 207, 29, 68, 89, 148,
  21, 62, 62, 15, 108, 219, 167, 181, 212, 252, 36, 42, 201, 112, 210, 203, 222, 147, 96, 21, 185, 175, 176, 240, 18,
  134, 173, 220, 190, 215, 113, 157, 70, 65, 243, 73, 38, 33, 61, 76, 162, 48, 35, 55, 113, 244, 92, 86, 253, 162, 155,
  186, 39, 180, 45, 12, 72, 196, 62, 99, 199, 229, 15, 179, 44, 188, 134, 137, 71, 127, 235, 220, 178, 133, 249, 141,
  242, 130, 15, 125, 151, 167, 35, 74, 34, 76, 172, 119, 159, 2, 116, 158, 80, 18, 229, 171, 217, 48, 156, 242, 165, 186,
  26, 203, 171, 241, 165, 66, 171, 240, 76, 20, 82, 234, 240, 167, 158, 121, 57, 250, 45, 152, 57, 184, 121, 239, 8, 244,
  50, 15, 153, 186, 94, 246, 142, 132, 80, 238, 190, 242, 199, 102, 64, 61, 111, 235, 98, 235, 118, 7, 245, 250, 90, 82,
  25, 86, 200, 136, 227, 213, 148, 130, 0, 17, 209, 132, 140, 192, 129, 16, 23, 248, 217, 135, 32, 76, 150, 242, 110,
  155, 235, 144, 115, 174, 130, 194, 244, 138, 117, 118, 95, 210, 171, 141, 247, 204, 159, 11, 50, 137, 214, 108, 142,
  73, 51, 118, 131, 232, 29, 32, 47, 225, 188, 111, 99, 122, 189, 30, 54, 7, 164, 37, 135, 109, 140, 237, 51, 112, 120,
  147, 200, 117, 154, 1, 82, 214, 231, 246, 133, 99, 135, 195, 31, 158, 170, 180, 106, 53, 156, 129, 136, 78, 203, 70,
  222, 117, 222, 139, 85, 195, 37, 183, 213, 146, 28, 43, 221, 37, 6, 107, 214, 5, 80, 151, 130, 204, 182, 155, 39, 108,
  217, 36, 46, 149, 46, 66, 88, 245, 139, 59, 9, 217, 26, 58, 121, 220, 176, 104, 58, 85, 79, 21, 168, 91, 241, 35, 178,
  9, 136, 103, 64, 149, 36, 68, 185, 94, 190, 97, 80, 194, 46, 125, 9, 31, 40, 169, 125, 253, 172, 169, 188, 11, 54, 13,
  227, 217, 209, 76, 219, 54, 136, 51, 37, 163, 57, 150, 236, 1, 113, 194, 125, 223, 213, 163, 156, 18, 120, 75, 33, 190,
  211, 172, 126, 98, 91, 67, 212, 218, 74, 27, 14, 201, 138, 177, 20, 206, 35, 10, 194, 76, 151, 143, 186, 227, 222, 238,
  198, 189, 154, 93, 16, 102, 106, 82, 22, 219, 221, 12, 55, 6, 59, 174, 98, 41, 123, 0, 239, 155, 79, 98, 25, 186, 153,
  95, 193, 162, 3, 91, 141, 105, 107, 17, 127, 148, 145, 88, 229, 50, 39, 55, 179, 238, 61, 204, 126, 103, 47, 190, 191,
  207, 150, 191, 61, 220, 218, 59, 189, 154, 81, 190, 209, 92, 106, 51, 157, 143, 127, 160, 62, 241, 109, 61, 94, 103,
  213, 97, 31, 102, 71, 168, 81, 197, 22, 199, 133, 237, 85, 244, 247, 96, 110, 108, 109, 244, 71, 109, 83, 125, 253,
  204, 222, 113, 124, 246, 118, 122, 206, 54, 200, 207, 162, 177, 198, 31, 196, 208, 106, 30, 170, 5, 155, 234, 22, 210,
  64, 15, 166, 116, 175, 225, 131, 254, 154, 95, 33, 82, 161, 178, 52, 84, 75, 122, 158, 90, 167, 233, 220, 168, 131, 41,
  70, 13, 97, 163, 134, 220, 71, 105, 81, 164, 83, 95, 47, 120, 46, 84, 223, 232, 105, 166, 43, 169, 204, 124, 27, 116,
  219, 155, 13, 247, 30, 177, 180, 47, 18, 76, 45, 13, 49, 218, 52, 48, 146, 150, 2, 125, 214, 176, 139, 86, 45, 141,
  234, 45, 141, 74, 13, 27, 208, 83, 113, 3, 75, 133, 196, 53, 157, 150, 74, 172, 150, 78, 9, 231, 86, 254, 85, 66, 83,
  185, 210, 86, 219, 119, 25, 149, 250, 234, 52, 231, 22, 166, 126, 41, 6, 150, 218, 115, 185, 234, 1, 159, 127, 146,
  138, 166, 98, 224, 21, 217, 225, 85, 153, 109, 88, 124, 121, 145, 43, 151, 177, 82, 205, 193, 213, 69, 92, 33, 215, 71,
  207, 222, 115, 32, 210, 146, 111, 20, 82, 126, 167, 80, 94, 17, 40, 183, 18, 26, 215, 38, 200, 186, 50, 154, 235, 92,
  251, 38, 51, 160, 182, 249, 226, 51, 120, 16, 157, 197, 179, 87, 33, 222, 197, 218, 43, 167, 117, 122, 17, 157, 166,
  117, 133, 125, 154, 193, 181, 146, 143, 55, 76, 245, 124, 222, 19, 62, 48, 106, 89, 38, 151, 235, 13, 255, 211, 202,
  92, 224, 188, 139, 223, 187, 16, 196, 155, 176, 15, 147, 248, 12, 217, 169, 150, 97, 167, 106, 190, 98, 104, 205, 229,
  151, 162, 209, 95, 93, 15, 236, 174, 47, 0, 76, 98, 49, 149, 85, 159, 37, 59, 138, 33, 76, 25, 182, 86, 208, 195, 126,
  105, 130, 84, 121, 110, 142, 113, 124, 155, 217, 119, 100, 236, 10, 143, 26, 123, 85, 142, 24, 167, 93, 189, 78, 30,
  15, 234, 10, 7, 67, 39, 214, 68, 28, 60, 73, 221, 219, 14, 226, 85, 83, 225, 14, 123, 12, 175, 154, 10, 115, 86, 14,
  155, 18, 216, 223, 219, 99, 50, 21, 199, 163, 171, 198, 47, 221, 113, 141, 129, 36, 50, 9, 103, 164, 209, 213, 114, 86,
  26, 70, 244, 214, 195, 42, 188, 84, 164, 243, 149, 24, 73, 163, 37, 116, 174, 231, 163, 30, 61, 32, 128, 162, 100, 49,
  139, 11, 226, 126, 215, 113, 38, 102, 10, 233, 238, 90, 112, 13, 8, 108, 245, 253, 130, 83, 104, 105, 183, 117, 4, 172,
  101, 135, 22, 154, 166, 46, 155, 122, 149, 39, 17, 215, 204, 61, 76, 143, 16, 80, 174, 6, 86, 50, 46, 8, 198, 185, 165,
  144, 87, 118, 50, 67, 199, 78, 7, 196, 28, 169, 108, 16, 181, 12, 41, 223, 140, 68, 38, 106, 213, 84, 118, 143, 180,
  207, 54, 69, 77, 45, 154, 11, 110, 115, 251, 114, 39, 167, 230, 153, 75, 162, 26, 70, 196, 160, 212, 213, 147, 52, 99,
  142, 254, 252, 196, 230, 11, 76, 42, 190, 152, 0, 228, 55, 60, 238, 22, 143, 243, 175, 9, 193, 122, 157, 69, 48, 230,
  42, 42, 179, 203, 149, 42, 170, 128, 109, 173, 104, 229, 214, 85, 72, 10, 34, 133, 71, 78, 136, 16, 2, 84, 134, 7, 20,
  128, 189, 126, 105, 84, 32, 219, 196, 215, 189, 222, 104, 35, 114, 203, 120, 125, 55, 96, 60, 18, 17, 104, 66, 80, 30,
  38, 81, 99, 101, 138, 104, 80, 75, 233, 45, 19, 191, 90, 172, 119, 126, 27, 56, 26, 237, 91, 207, 86, 177, 214, 184,
  157, 64, 18, 101, 94, 52, 246, 15, 62, 89, 177, 78, 191, 154, 211, 109, 77, 124, 101, 179, 194, 0, 11, 185, 228, 178,
  33, 172, 31, 69, 155, 93, 198, 47, 221, 188, 63, 25, 248, 8, 143, 123, 205, 171, 172, 74, 88, 19, 76, 157, 179, 214,
  37, 95, 125, 235, 20, 213, 154, 154, 162, 179, 18, 98, 220, 42, 57, 105, 93, 225, 163, 111, 85, 46, 210, 201, 129, 215,
  5, 168, 19, 141, 79, 214, 82, 135, 138, 135, 232, 34, 61, 120, 38, 188, 45, 172, 85, 15, 74, 222, 120, 222, 231, 9,
  150, 44, 130, 150, 20, 159, 64, 243, 153, 98, 119, 42, 19, 219, 69, 110, 120, 218, 219, 204, 62, 65, 66, 179, 25, 93,
  215, 106, 186, 81, 10, 47, 76, 101, 74, 228, 68, 5, 90, 148, 31, 135, 176, 229, 123, 183, 217, 220, 124, 223, 88, 62,
  49, 148, 246, 148, 121, 21, 109, 69, 247, 162, 65, 237, 22, 172, 110, 12, 196, 196, 80, 58, 60, 51, 79, 148, 84, 244,
  143, 149, 6, 193, 211, 231, 247, 150, 247, 154, 210, 61, 154, 105, 238, 49, 162, 44, 227, 18, 135, 199, 113, 20, 153,
  152, 139, 200, 186, 13, 193, 116, 28, 117, 24, 28, 220, 4, 119, 8, 178, 165, 36, 47, 11, 132, 217, 144, 232, 56, 185,
  110, 6, 176, 119, 238, 112, 5, 246, 213, 49, 221, 224, 246, 227, 228, 131, 103, 90, 82, 172, 48, 46, 62, 9, 218, 109,
  111, 85, 65, 172, 55, 188, 142, 172, 188, 92, 22, 145, 111, 125, 221, 169, 36, 48, 245, 160, 45, 215, 62, 246, 99, 207,
  204, 47, 87, 48, 254, 203, 113, 158, 131, 47, 94, 160, 120, 104, 10, 157, 163, 234, 140, 51, 10, 17, 45, 50, 158, 2,
  192, 103, 84, 161, 206, 234, 161, 163, 178, 117, 26, 195, 65, 226, 105, 140, 110, 166, 231, 8, 114, 67, 189, 231, 106,
  39, 46, 26, 3, 22, 211, 221, 157, 241, 152, 157, 0, 28, 207, 198, 232, 143, 119, 109, 245, 159, 219, 253, 197, 146,
  194, 45, 254, 198, 50, 203, 147, 77, 59, 58, 175, 131, 239, 63, 155, 239, 175, 221, 110, 25, 114, 5, 218, 83, 131, 137,
  171, 33, 227, 249, 154, 98, 200, 187, 50, 150, 120, 112, 95, 246, 219, 156, 149, 129, 74, 17, 44, 187, 231, 200, 22,
  49, 11, 231, 24, 38, 143, 31, 117, 244, 249, 175, 54, 190, 200, 193, 143, 44, 120, 138, 61, 45, 245, 83, 75, 157, 115,
  111, 60, 107, 3, 31, 91, 108, 223, 207, 90, 147, 112, 6, 123, 212, 167, 40, 13, 142, 113, 51, 18, 82, 114, 61, 186, 16,
  124, 73, 97, 44, 216, 85, 172, 215, 81, 158, 38, 80, 144, 157, 9, 150, 95, 44, 12, 217, 147, 153, 125, 204, 46, 213,
  105, 122, 235, 141, 241, 109, 254, 78, 2, 54, 239, 224, 177, 12, 223, 125, 124, 25, 108, 81, 199, 198, 188, 121, 198,
  227, 44, 178, 151, 56, 136, 29, 121, 2, 234, 33, 190, 24, 140, 226, 18, 39, 29, 98, 238, 7, 209, 69, 155, 63, 218, 205,
  162, 248, 124, 153, 24, 76, 148, 247, 39, 6, 190, 72, 23, 195, 73, 68, 49, 251, 203, 175, 242, 164, 172, 76, 195, 103,
  17, 68, 205, 62, 33, 198, 62, 220, 208, 127, 188, 51, 244, 31, 85, 232, 63, 154, 208, 175, 200, 217, 153, 97, 209, 162,
  238, 3, 180, 177, 69, 151, 107, 89, 236, 71, 81, 12, 118, 112, 250, 248, 92, 209, 43, 39, 52, 48, 170, 106, 7, 195,
  117, 69, 15, 151, 88, 57, 223, 201, 68, 105, 106, 132, 194, 215, 22, 24, 220, 221, 81, 198, 129, 35, 67, 3, 194, 148,
  58, 59, 92, 180, 100, 76, 30, 74, 5, 20, 76, 88, 133, 105, 169, 100, 167, 178, 152, 226, 120, 125, 85, 94, 191, 51, 0,
  176, 208, 125, 58, 136, 145, 136, 64, 232, 4, 162, 20, 196, 176, 150, 185, 242, 4, 131, 89, 126, 18, 143, 64, 123, 73,
  147, 34, 158, 215, 141, 243, 66, 54, 29, 235, 142, 83, 68, 255, 206, 151, 123, 113, 112, 42, 169, 103, 30, 90, 18, 41,
  220, 118, 164, 12, 179, 38, 59, 133, 249, 14, 152, 1, 29, 53, 235, 87, 37, 24, 101, 100, 27, 134, 115, 153, 57, 204,
  13, 82, 36, 186, 184, 161, 211, 221, 46, 68, 32, 78, 90, 21, 233, 98, 194, 146, 245, 209, 144, 16, 188, 252, 255, 207,
  0, 134, 120, 13, 203, 97, 164, 47, 152, 77, 207, 143, 78, 95, 31, 31, 126, 128, 63, 15, 245, 192, 7, 104, 115, 233,
  215, 106, 202, 235, 60, 93, 121, 66, 207, 10, 194, 212, 255, 97, 62, 143, 178, 195, 48, 199, 224, 82, 238, 39, 178,
  212, 160, 190, 228, 222, 194, 134, 6, 61, 105, 72, 49, 144, 41, 114, 141, 20, 37, 248, 201, 190, 26, 240, 161, 175, 45,
  76, 122, 155, 183, 25, 94, 21, 51, 199, 59, 44, 183, 25, 228, 10, 80, 21, 67, 237, 174, 229, 26, 87, 221, 10, 137, 163,
  86, 122, 78, 184, 134, 193, 121, 192, 126, 202, 66, 108, 138, 144, 50, 130, 194, 203, 156, 219, 100, 57, 186, 89, 254,
  241, 155, 79, 50, 65, 158, 12, 18, 66, 154, 103, 78, 3, 175, 228, 80, 50, 114, 16, 41, 57, 181, 155, 143, 109, 192,
  116, 90, 247, 190, 112, 81, 240, 119, 226, 241, 110, 160, 225, 24, 172, 48, 140, 96, 93, 114, 88, 178, 197, 81, 211,
  18, 110, 77, 103, 96, 128, 7, 85, 142, 60, 77, 247, 253, 69, 165, 233, 242, 208, 190, 89, 186, 170, 187, 176, 113, 93,
  30, 178, 149, 134, 116, 6, 156, 52, 139, 200, 97, 135, 194, 106, 48, 157, 3, 45, 89, 179, 226, 8, 131, 172, 206, 138,
  50, 0, 98, 99, 185, 50, 146, 79, 210, 75, 33, 54, 62, 5, 220, 108, 44, 217, 160, 41, 105, 237, 155, 112, 26, 70, 226,
  108, 254, 79, 125, 161, 30, 136, 148, 31, 251, 82, 19, 168, 138, 192, 28, 22, 5, 168, 251, 28, 159, 167, 164, 140, 102,
  165, 19, 211, 103, 169, 160, 138, 95, 154, 87, 165, 132, 41, 200, 27, 69, 218, 70, 44, 206, 255, 23, 213, 137, 237, 59,
  204, 161, 214, 102, 99, 73, 100, 113, 191, 178, 251, 91, 89, 178, 202, 167, 56, 26, 191, 216, 242, 165, 46, 30, 108, 2,
  216, 239, 83, 79, 211, 69, 78, 15, 231, 210, 69, 57, 73, 228, 198, 10, 85, 73, 27, 182, 171, 226, 41, 241, 60, 204,
  243, 248, 34, 98, 155, 117, 253, 88, 181, 18, 17, 26, 103, 116, 90, 83, 198, 123, 101, 84, 240, 81, 22, 79, 85, 57,
  133, 66, 20, 72, 186, 55, 39, 243, 228, 180, 231, 0, 38, 31, 106, 14, 154, 104, 135, 69, 199, 140, 163, 44, 195, 24,
  234, 207, 99, 232, 228, 236, 44, 120, 252, 242, 121, 16, 49, 153, 210, 103, 123, 182, 90, 195, 101, 161, 85, 76, 228,
  44, 16, 251, 219, 44, 196, 53, 70, 236, 244, 14, 203, 28, 88, 25, 204, 36, 216, 100, 164, 57, 72, 148, 122, 141, 164,
  79, 173, 225, 12, 72, 102, 67, 111, 56, 90, 180, 158, 51, 168, 141, 147, 232, 202, 120, 82, 38, 61, 23, 151, 202, 45,
  223, 166, 11, 96, 108, 12, 88, 47, 103, 90, 46, 220, 232, 166, 56, 49, 89, 218, 211, 144, 7, 151, 178, 188, 185, 101,
  253, 54, 191, 116, 146, 215, 217, 179, 230, 124, 62, 160, 67, 148, 44, 34, 124, 237, 180, 217, 94, 150, 198, 48, 90,
  162, 44, 236, 203, 204, 137, 174, 148, 100, 20, 104, 4, 86, 82, 91, 196, 227, 82, 114, 150, 106, 124, 170, 138, 167,
  212, 243, 235, 121, 174, 71, 24, 175, 41, 190, 85, 111, 19, 116, 245, 237, 142, 248, 167, 171, 5, 253, 166, 23, 242,
  88, 108, 163, 119, 239, 173, 200, 240, 89, 241, 154, 185, 239, 169, 193, 101, 205, 244, 93, 79, 58, 15, 52, 171, 29,
  218, 177, 150, 194, 203, 16, 180, 12, 212, 163, 88, 152, 116, 17, 95, 72, 233, 104, 83, 1, 215, 168, 188, 36, 233, 124,
  107, 114, 5, 72, 150, 47, 51, 134, 121, 166, 71, 23, 196, 79, 216, 181, 240, 0, 136, 42, 251, 236, 253, 206, 125, 38,
  69, 127, 101, 149, 199, 4, 67, 199, 67, 192, 245, 184, 240, 226, 16, 48, 239, 91, 81, 80, 234, 238, 31, 255, 24, 101,
  63, 255, 181, 136, 142, 31, 247, 65, 81, 19, 217, 55, 31, 185, 58, 80, 250, 159, 18, 49, 108, 135, 90, 118, 16, 1, 11,
  155, 185, 4, 193, 84, 48, 146, 218, 202, 3, 78, 218, 172, 121, 96, 173, 95, 238, 39, 187, 51, 234, 137, 225, 199, 94,
  10, 59, 242, 17, 107, 18, 162, 234, 200, 8, 103, 246, 79, 156, 24, 205, 64, 158, 31, 247, 3, 169, 153, 170, 207, 142,
  218, 90, 44, 57, 251, 48, 247, 157, 254, 54, 185, 246, 240, 15, 116, 228, 97, 238, 46, 253, 45, 225, 29, 212, 215, 213,
  76, 117, 142, 45, 209, 53, 133, 94, 216, 119, 107, 157, 68, 165, 166, 12, 108, 214, 15, 156, 243, 232, 1, 155, 158,
  223, 110, 116, 130, 190, 123, 70, 137, 18, 187, 174, 18, 52, 183, 100, 17, 248, 194, 248, 148, 204, 23, 85, 93, 13, 73,
  198, 126, 245, 21, 12, 137, 113, 58, 69, 143, 11, 103, 104, 126, 226, 214, 61, 75, 252, 113, 187, 160, 208, 84, 200,
  82, 90, 86, 16, 195, 197, 152, 165, 31, 104, 124, 78, 210, 73, 29, 180, 68, 25, 70, 185, 17, 82, 170, 248, 55, 165,
  174, 249, 14, 66, 181, 136, 108, 36, 125, 49, 83, 49, 143, 184, 240, 89, 116, 70, 241, 143, 153, 42, 195, 190, 204,
  220, 38, 146, 236, 65, 80, 186, 195, 155, 183, 196, 204, 101, 83, 128, 83, 146, 172, 50, 124, 150, 154, 203, 20, 190,
  167, 125, 187, 142, 5, 43, 168, 144, 94, 13, 78, 187, 170, 224, 184, 162, 96, 76, 81, 243, 88, 205, 188, 161, 80, 22,
  151, 215, 20, 148, 36, 235, 174, 2, 131, 177, 196, 197, 150, 23, 98, 14, 182, 181, 110, 79, 245, 171, 173, 185, 96,
  177, 247, 0, 191, 143, 208, 49, 5, 111, 147, 206, 96, 73, 132, 153, 223, 221, 89, 233, 221, 13, 148, 81, 180, 161, 240,
  63, 150, 92, 92, 61, 229, 162, 147, 40, 207, 122, 87, 106, 208, 198, 136, 202, 226, 92, 161, 123, 202, 206, 243, 248,
  110, 71, 85, 238, 62, 58, 149, 59, 38, 166, 185, 193, 27, 125, 253, 228, 235, 63, 31, 171, 52, 62, 97, 32, 55, 90, 116,
  90, 224, 165, 214, 196, 123, 140, 152, 78, 88, 218, 241, 140, 249, 230, 203, 119, 154, 49, 110, 155, 249, 182, 137,
  251, 82, 11, 53, 107, 142, 54, 199, 229, 14, 23, 91, 150, 219, 197, 170, 31, 152, 70, 196, 247, 150, 235, 85, 211, 219,
  171, 84, 85, 74, 194, 244, 14, 250, 193, 116, 153, 106, 48, 81, 217, 207, 181, 135, 195, 0, 71, 171, 174, 194, 198,
  128, 137, 85, 24, 233, 37, 215, 95, 219, 41, 201, 82, 74, 220, 58, 135, 230, 178, 164, 172, 195, 106, 178, 186, 34,
  171, 110, 81, 44, 113, 67, 77, 19, 110, 209, 70, 81, 121, 172, 220, 239, 106, 110, 95, 110, 181, 64, 45, 226, 212, 39,
  212, 222, 186, 212, 130, 149, 172, 79, 218, 163, 90, 85, 186, 129, 82, 176, 84, 15, 196, 21, 17, 53, 87, 185, 91, 209,
  15, 220, 209, 88, 173, 23, 170, 112, 68, 43, 110, 55, 126, 190, 42, 240, 238, 35, 181, 209, 42, 133, 210, 251, 101,
  138, 193, 170, 186, 192, 244, 139, 168, 1, 14, 4, 61, 107, 231, 29, 205, 32, 55, 213, 11, 248, 175, 36, 29, 173, 5,
  173, 212, 75, 248, 74, 227, 177, 223, 169, 43, 60, 179, 73, 53, 185, 169, 75, 83, 223, 88, 146, 48, 150, 10, 11, 134,
  68, 224, 37, 189, 9, 216, 198, 16, 222, 120, 181, 91, 91, 23, 75, 87, 129, 250, 59, 193, 1, 162, 175, 239, 13, 163, 26,
  97, 45, 22, 76, 96, 65, 107, 189, 244, 244, 66, 47, 214, 12, 172, 65, 23, 61, 152, 234, 253, 33, 33, 199, 126, 91, 11,
  127, 105, 200, 145, 191, 202, 231, 194, 14, 151, 147, 214, 42, 216, 148, 54, 59, 217, 166, 242, 98, 152, 145, 70, 166,
  113, 245, 217, 176, 210, 37, 24, 183, 231, 212, 191, 215, 196, 91, 175, 195, 177, 193, 173, 252, 145, 34, 219, 100,
  197, 152, 81, 54, 175, 211, 93, 7, 217, 64, 86, 27, 70, 201, 195, 25, 8, 77, 212, 140, 158, 100, 225, 52, 50, 11, 149,
  12, 104, 33, 148, 69, 255, 182, 136, 242, 194, 168, 111, 91, 81, 113, 218, 189, 204, 160, 155, 146, 77, 200, 158, 243,
  73, 190, 202, 169, 62, 141, 8, 170, 135, 50, 78, 160, 255, 232, 174, 120, 149, 84, 209, 30, 24, 44, 169, 169, 105, 126,
  143, 203, 49, 211, 2, 148, 43, 151, 172, 35, 124, 47, 222, 176, 219, 241, 161, 46, 174, 72, 117, 42, 221, 26, 190, 114,
  48, 75, 213, 253, 205, 95, 95, 149, 88, 201, 30, 84, 222, 212, 161, 3, 11, 44, 95, 158, 159, 80, 192, 60, 216, 30, 125,
  82, 22, 248, 45, 117, 163, 189, 163, 238, 123, 245, 11, 143, 32, 195, 251, 170, 163, 145, 246, 184, 145, 1, 211, 218,
  188, 219, 171, 165, 31, 28, 223, 94, 175, 8, 143, 17, 229, 219, 0, 246, 225, 94, 136, 187, 119, 130, 184, 91, 1, 81,
  108, 223, 111, 9, 18, 247, 249, 58, 204, 96, 85, 16, 210, 56, 224, 8, 213, 226, 178, 14, 241, 211, 5, 221, 62, 196, 18,
  85, 11, 17, 255, 233, 46, 38, 186, 203, 34, 218, 62, 112, 28, 94, 172, 110, 65, 114, 72, 88, 57, 19, 221, 97, 17, 86,
  209, 100, 153, 50, 178, 162, 117, 205, 208, 113, 13, 125, 242, 129, 165, 80, 234, 58, 169, 17, 11, 225, 31, 167, 121,
  150, 236, 162, 76, 243, 242, 94, 156, 91, 95, 215, 138, 138, 84, 135, 10, 191, 229, 214, 113, 149, 218, 34, 241, 55,
  175, 238, 150, 7, 108, 191, 152, 138, 203, 24, 246, 14, 246, 46, 134, 154, 71, 185, 245, 30, 12, 174, 164, 213, 178,
  218, 78, 91, 153, 154, 197, 13, 101, 119, 155, 81, 48, 147, 92, 182, 53, 62, 161, 171, 141, 107, 86, 33, 219, 186, 38,
  22, 216, 193, 226, 236, 81, 122, 165, 60, 83, 138, 36, 124, 140, 201, 230, 99, 165, 172, 245, 139, 231, 70, 232, 109,
  43, 224, 129, 25, 253, 127, 238, 143, 71, 228, 176, 189, 177, 64, 64, 122, 96, 153, 7, 174, 192, 64, 154, 167, 134, 46,
  56, 122, 14, 119, 12, 61, 126, 144, 27, 190, 22, 79, 232, 214, 240, 5, 45, 141, 231, 71, 63, 10, 19, 192, 55, 159, 180,
  86, 111, 130, 117, 72, 210, 98, 93, 220, 4, 127, 255, 247, 255, 74, 20, 0, 69, 61, 158, 241, 79, 64, 24, 62, 195, 43,
  61, 148, 177, 108, 205, 58, 70, 28, 160, 104, 171, 45, 153, 46, 190, 202, 198, 91, 238, 118, 180, 238, 243, 10, 143,
  136, 139, 179, 122, 152, 101, 182, 133, 144, 13, 62, 69, 211, 128, 252, 219, 114, 6, 105, 148, 2, 132, 25, 246, 227,
  69, 248, 194, 242, 27, 150, 101, 197, 37, 124, 140, 214, 177, 127, 16, 126, 55, 104, 82, 132, 102, 3, 214, 178, 243,
  105, 251, 8, 238, 203, 62, 213, 115, 142, 14, 12, 202, 35, 230, 176, 75, 43, 65, 168, 131, 174, 170, 214, 176, 47, 136,
  222, 65, 77, 216, 48, 154, 41, 142, 83, 74, 51, 238, 134, 56, 165, 52, 211, 119, 61, 233, 142, 83, 74, 66, 6, 159, 202,
  89, 140, 78, 212, 231, 211, 105, 151, 90, 166, 14, 229, 75, 184, 122, 58, 123, 58, 13, 35, 253, 153, 110, 233, 142,
  238, 120, 44, 152, 20, 163, 220, 247, 202, 200, 156, 226, 206, 187, 227, 137, 155, 77, 184, 111, 86, 148, 191, 240,
  125, 177, 79, 198, 221, 180, 41, 30, 226, 143, 12, 90, 255, 98, 212, 86, 182, 39, 174, 231, 145, 184, 58, 79, 231, 100,
  117, 103, 51, 15, 2, 58, 34, 3, 208, 206, 0, 237, 34, 0, 141, 243, 21, 79, 231, 61, 4, 245, 144, 122, 28, 21, 195, 201,
  191, 228, 233, 172, 254, 113, 61, 156, 199, 235, 52, 206, 235, 35, 160, 241, 245, 131, 60, 154, 229, 105, 182, 47, 185,
  251, 15, 248, 20, 39, 124, 178, 235, 106, 248, 113, 243, 135, 34, 149, 9, 69, 138, 102, 124, 173, 65, 73, 233, 58, 123,
  168, 103, 121, 208, 53, 195, 191, 87, 188, 131, 55, 127, 215, 121, 223, 216, 115, 150, 3, 201, 252, 114, 240, 23, 44,
  251, 174, 203, 108, 221, 55, 222, 130, 120, 103, 90, 190, 96, 7, 181, 218, 32, 247, 116, 127, 195, 7, 106, 142, 253,
  128, 162, 25, 125, 173, 48, 158, 224, 211, 158, 209, 19, 79, 137, 89, 247, 40, 244, 251, 64, 182, 140, 95, 105, 164,
  184, 54, 253, 128, 13, 142, 50, 74, 52, 26, 251, 186, 36, 250, 130, 227, 194, 99, 181, 16, 181, 253, 151, 97, 32, 151,
  28, 52, 41, 183, 207, 198, 241, 9, 136, 227, 130, 101, 237, 221, 113, 200, 111, 71, 123, 229, 185, 194, 229, 143, 21,
  138, 167, 10, 3, 87, 200, 62, 115, 208, 244, 88, 108, 252, 109, 168, 49, 252, 97, 4, 197, 125, 201, 7, 254, 242, 131,
  125, 185, 199, 25, 167, 223, 37, 77, 42, 31, 120, 112, 190, 92, 197, 6, 214, 136, 90, 43, 113, 52, 187, 128, 198, 89,
  58, 175, 203, 235, 181, 135, 175, 142, 3, 149, 175, 106, 222, 75, 107, 14, 41, 207, 154, 85, 245, 150, 97, 8, 220, 26,
  212, 35, 93, 167, 96, 100, 75, 65, 111, 185, 12, 179, 89, 189, 198, 219, 10, 34, 68, 162, 214, 196, 191, 14, 189, 114,
  158, 69, 227, 24, 85, 191, 47, 37, 158, 233, 61, 121, 222, 93, 69, 224, 5, 99, 248, 55, 26, 209, 107, 239, 38, 57, 68,
  158, 249, 154, 42, 63, 238, 4, 109, 144, 144, 188, 65, 93, 49, 66, 69, 41, 202, 243, 240, 44, 50, 102, 222, 146, 245,
  222, 82, 217, 124, 171, 157, 79, 223, 153, 103, 192, 120, 130, 39, 84, 194, 227, 178, 71, 251, 230, 87, 88, 98, 100,
  123, 166, 81, 228, 21, 205, 11, 20, 230, 13, 119, 78, 211, 71, 80, 41, 201, 150, 30, 40, 248, 142, 172, 207, 196, 152,
  239, 205, 213, 207, 92, 129, 191, 148, 120, 163, 191, 150, 100, 43, 247, 10, 119, 145, 112, 191, 168, 140, 91, 93, 202,
  253, 35, 229, 156, 45, 233, 238, 44, 235, 132, 215, 64, 201, 119, 13, 47, 197, 89, 68, 107, 186, 223, 202, 134, 238,
  29, 251, 99, 132, 185, 179, 41, 181, 152, 227, 187, 27, 135, 210, 38, 84, 255, 164, 152, 138, 36, 130, 204, 117, 73,
  65, 85, 201, 251, 32, 28, 159, 34, 52, 200, 124, 152, 194, 126, 79, 98, 115, 227, 26, 69, 109, 30, 209, 109, 123, 147,
  130, 198, 119, 165, 168, 102, 177, 50, 252, 82, 90, 151, 26, 218, 94, 131, 79, 1, 151, 4, 174, 150, 194, 134, 244, 242,
  8, 187, 106, 89, 102, 220, 7, 117, 8, 49, 197, 59, 184, 172, 200, 198, 235, 225, 69, 148, 1, 72, 45, 240, 223, 151, 59,
  61, 1, 81, 156, 69, 249, 132, 29, 31, 188, 156, 163, 104, 84, 90, 178, 119, 191, 38, 78, 246, 147, 142, 81, 241, 16,
  53, 197, 160, 142, 247, 102, 133, 111, 137, 211, 135, 158, 174, 196, 48, 131, 76, 60, 114, 25, 73, 150, 248, 105, 184,
  221, 51, 116, 193, 132, 187, 117, 231, 198, 138, 169, 6, 150, 187, 62, 134, 217, 137, 18, 195, 184, 97, 201, 5, 92, 22,
  209, 48, 18, 144, 210, 90, 117, 155, 198, 233, 34, 207, 104, 84, 175, 1, 114, 207, 22, 87, 53, 124, 234, 24, 254, 52,
  156, 5, 14, 211, 30, 22, 24, 194, 31, 119, 129, 83, 246, 238, 27, 123, 255, 205, 93, 228, 233, 98, 138, 37, 228, 123,
  111, 238, 82, 111, 232, 97, 55, 122, 223, 173, 130, 1, 32, 155, 44, 110, 117, 248, 97, 158, 136, 225, 107, 101, 222,
  53, 0, 203, 75, 131, 70, 45, 59, 27, 132, 245, 238, 86, 179, 183, 209, 220, 236, 53, 59, 141, 154, 227, 105, 95, 122,
  229, 238, 221, 239, 212, 3, 14, 144, 224, 157, 118, 167, 25, 12, 251, 193, 187, 173, 221, 102, 119, 163, 211, 236, 109,
  110, 191, 15, 110, 154, 118, 177, 109, 86, 108, 99, 179, 217, 221, 189, 215, 220, 221, 116, 149, 234, 182, 187, 172,
  84, 15, 139, 221, 219, 109, 238, 184, 75, 113, 88, 189, 205, 157, 102, 183, 187, 129, 255, 105, 229, 222, 219, 183,
  125, 65, 44, 170, 161, 190, 165, 233, 13, 200, 128, 193, 15, 155, 65, 175, 221, 49, 34, 236, 197, 163, 43, 246, 106,
  225, 28, 157, 220, 103, 163, 99, 152, 191, 87, 44, 128, 43, 131, 135, 47, 63, 104, 33, 36, 232, 129, 40, 168, 5, 25,
  157, 242, 125, 37, 160, 109, 253, 155, 79, 4, 135, 238, 15, 183, 255, 2, 75, 122, 29, 148, 142, 198, 77, 227, 163, 229,
  97, 47, 154, 196, 144, 88, 173, 238, 251, 102, 48, 80, 83, 172, 158, 177, 75, 78, 132, 78, 43, 8, 49, 156, 204, 122,
  80, 31, 80, 8, 238, 208, 17, 243, 117, 74, 218, 104, 8, 72, 160, 38, 81, 191, 104, 198, 101, 160, 72, 22, 244, 240, 2,
  163, 198, 12, 218, 195, 119, 241, 251, 214, 69, 227, 219, 162, 97, 95, 145, 231, 61, 2, 88, 206, 174, 56, 222, 77, 10,
  147, 33, 176, 51, 189, 64, 120, 72, 79, 252, 17, 227, 155, 252, 74, 217, 26, 199, 138, 146, 50, 177, 242, 117, 109, 50,
  4, 3, 163, 117, 59, 59, 193, 183, 172, 79, 209, 21, 244, 178, 123, 175, 221, 187, 7, 41, 4, 159, 8, 196, 90, 250, 46,
  232, 109, 220, 107, 111, 56, 30, 141, 199, 81, 136, 240, 92, 175, 46, 49, 160, 55, 214, 218, 29, 215, 171, 140, 172,
  112, 11, 170, 233, 172, 224, 159, 111, 206, 151, 189, 213, 16, 172, 146, 81, 123, 237, 45, 122, 238, 176, 234, 89, 42,
  129, 35, 6, 207, 40, 169, 108, 8, 132, 47, 76, 162, 12, 163, 135, 212, 187, 24, 20, 8, 41, 176, 30, 80, 124, 12, 58,
  179, 174, 32, 66, 54, 89, 149, 6, 27, 42, 17, 0, 40, 104, 27, 147, 42, 26, 224, 146, 10, 157, 127, 26, 133, 5, 114,
  182, 140, 91, 41, 124, 81, 216, 249, 35, 139, 25, 9, 159, 184, 248, 38, 225, 245, 81, 210, 164, 135, 116, 35, 124, 241,
  196, 246, 40, 224, 80, 152, 211, 168, 231, 102, 220, 170, 209, 196, 217, 219, 230, 101, 20, 132, 75, 87, 44, 175, 73,
  153, 63, 49, 34, 41, 185, 195, 145, 95, 54, 237, 7, 59, 113, 228, 216, 107, 81, 221, 205, 38, 251, 162, 227, 135, 141,
  45, 199, 59, 164, 172, 32, 82, 27, 63, 168, 92, 215, 246, 58, 224, 79, 134, 32, 207, 212, 185, 95, 11, 59, 58, 122, 5,
  11, 109, 242, 26, 93, 75, 40, 226, 123, 195, 17, 69, 247, 154, 189, 4, 130, 1, 8, 38, 248, 71, 190, 253, 97, 107, 32,
  244, 6, 231, 190, 68, 25, 120, 11, 231, 221, 4, 249, 170, 94, 38, 242, 254, 105, 186, 130, 108, 237, 138, 181, 134, 81,
  19, 46, 241, 143, 179, 53, 165, 255, 232, 192, 201, 168, 240, 29, 198, 90, 88, 15, 46, 27, 124, 218, 179, 198, 88, 166,
  211, 130, 194, 102, 190, 42, 219, 136, 136, 43, 132, 20, 210, 22, 109, 127, 113, 26, 229, 171, 102, 112, 221, 164, 62,
  176, 127, 27, 203, 236, 198, 36, 73, 25, 159, 83, 240, 36, 246, 179, 157, 40, 33, 166, 213, 244, 9, 176, 153, 18, 123,
  218, 30, 20, 182, 163, 64, 185, 177, 209, 243, 197, 22, 148, 158, 227, 164, 71, 200, 37, 190, 217, 105, 119, 119, 26,
  43, 68, 123, 171, 125, 29, 110, 141, 55, 198, 195, 218, 178, 136, 111, 183, 9, 230, 102, 132, 100, 229, 253, 216, 179,
  158, 104, 82, 151, 82, 49, 119, 128, 25, 226, 245, 58, 175, 210, 234, 54, 26, 223, 10, 6, 108, 185, 216, 79, 97, 168,
  167, 72, 207, 125, 75, 38, 55, 213, 129, 112, 214, 37, 51, 81, 93, 64, 144, 156, 183, 206, 121, 177, 197, 191, 27, 223,
  94, 250, 98, 181, 49, 113, 92, 40, 83, 100, 221, 49, 109, 26, 223, 78, 108, 195, 94, 12, 11, 44, 168, 44, 206, 232,
  108, 100, 228, 117, 69, 99, 243, 88, 217, 20, 186, 75, 2, 238, 197, 7, 56, 8, 173, 214, 47, 73, 250, 103, 20, 161, 221,
  79, 121, 100, 245, 106, 210, 35, 132, 95, 145, 242, 183, 32, 106, 41, 254, 211, 60, 242, 71, 115, 179, 131, 132, 85, 4,
  198, 51, 35, 30, 139, 71, 11, 202, 21, 145, 153, 157, 202, 42, 118, 240, 195, 101, 177, 99, 41, 100, 32, 202, 89, 73,
  86, 84, 46, 76, 241, 138, 50, 247, 114, 239, 119, 126, 210, 98, 196, 169, 136, 116, 92, 65, 76, 161, 163, 24, 36, 6,
  64, 147, 101, 82, 138, 20, 251, 125, 233, 51, 32, 236, 52, 93, 220, 70, 126, 29, 141, 55, 225, 255, 200, 146, 202, 54,
  69, 32, 204, 182, 113, 99, 180, 1, 210, 108, 123, 53, 97, 230, 8, 191, 246, 185, 194, 76, 141, 28, 199, 150, 132, 138,
  94, 108, 3, 246, 90, 88, 185, 111, 123, 159, 207, 45, 102, 24, 69, 161, 65, 185, 214, 140, 132, 60, 81, 48, 120, 158,
  40, 213, 254, 183, 69, 148, 93, 51, 191, 16, 188, 66, 141, 79, 105, 183, 88, 177, 154, 125, 73, 159, 251, 184, 232,
  162, 130, 165, 182, 227, 217, 44, 202, 158, 158, 62, 127, 134, 231, 215, 134, 101, 232, 62, 133, 229, 129, 237, 80,
  158, 239, 175, 177, 242, 173, 252, 18, 77, 73, 107, 1, 121, 29, 236, 175, 13, 194, 225, 249, 25, 109, 117, 250, 223,
  124, 146, 11, 113, 7, 116, 220, 155, 181, 131, 251, 235, 8, 224, 96, 22, 71, 163, 44, 62, 251, 114, 192, 119, 21, 224,
  241, 40, 10, 147, 47, 5, 186, 219, 222, 82, 64, 79, 210, 225, 68, 131, 252, 177, 90, 89, 176, 213, 232, 116, 30, 205,
  30, 11, 223, 231, 48, 113, 92, 191, 82, 189, 165, 229, 45, 240, 61, 51, 155, 28, 12, 249, 249, 144, 153, 9, 82, 249,
  77, 28, 161, 168, 174, 77, 152, 186, 94, 187, 237, 149, 160, 138, 192, 57, 77, 113, 95, 200, 176, 240, 176, 11, 105,
  204, 157, 30, 187, 118, 26, 23, 9, 58, 97, 127, 100, 221, 13, 103, 121, 60, 156, 20, 125, 17, 109, 37, 97, 47, 32, 91,
  202, 245, 32, 28, 209, 153, 181, 234, 184, 68, 240, 30, 97, 134, 233, 185, 68, 165, 27, 172, 146, 97, 196, 210, 98,
  186, 152, 84, 20, 15, 159, 12, 158, 1, 1, 203, 214, 120, 66, 173, 225, 42, 25, 230, 197, 246, 68, 47, 75, 73, 222, 210,
  189, 77, 187, 56, 89, 225, 140, 215, 247, 88, 163, 13, 129, 78, 155, 216, 148, 222, 253, 9, 71, 163, 122, 13, 195, 125,
  92, 68, 174, 90, 212, 124, 163, 68, 78, 169, 153, 81, 228, 137, 37, 149, 1, 153, 134, 130, 236, 74, 213, 185, 206, 58,
  73, 113, 55, 40, 249, 147, 157, 45, 160, 41, 77, 227, 137, 199, 204, 5, 169, 142, 57, 200, 146, 167, 225, 0, 31, 78,
  226, 181, 155, 60, 56, 130, 219, 23, 78, 245, 38, 183, 146, 44, 23, 39, 129, 208, 3, 238, 236, 132, 203, 139, 233, 50,
  197, 77, 118, 124, 11, 43, 96, 155, 41, 85, 160, 25, 68, 132, 109, 120, 84, 65, 95, 137, 73, 223, 196, 121, 60, 72,
  162, 186, 197, 189, 53, 152, 55, 120, 30, 208, 48, 195, 39, 177, 217, 81, 101, 130, 38, 141, 68, 149, 25, 170, 180, 88,
  169, 97, 58, 208, 51, 111, 49, 84, 122, 76, 58, 253, 37, 125, 71, 139, 116, 62, 71, 182, 102, 21, 51, 243, 112, 175,
  242, 96, 175, 70, 103, 12, 5, 93, 7, 45, 204, 19, 102, 122, 95, 71, 100, 61, 196, 185, 93, 30, 114, 169, 229, 172, 243,
  30, 108, 204, 97, 103, 143, 152, 119, 173, 114, 212, 70, 7, 62, 226, 140, 199, 101, 153, 79, 103, 169, 93, 1, 83, 139,
  116, 22, 15, 161, 18, 90, 191, 41, 113, 102, 236, 62, 140, 135, 217, 220, 135, 99, 20, 24, 145, 99, 37, 182, 138, 54,
  144, 34, 7, 149, 251, 132, 244, 118, 100, 75, 238, 28, 45, 43, 246, 209, 205, 158, 240, 84, 206, 4, 121, 138, 122, 46,
  232, 162, 25, 243, 231, 130, 193, 68, 194, 45, 120, 255, 194, 249, 7, 233, 200, 222, 101, 55, 38, 219, 151, 241, 56,
  254, 80, 186, 127, 81, 150, 243, 29, 139, 28, 95, 227, 50, 2, 215, 195, 80, 46, 174, 250, 58, 17, 33, 229, 3, 150, 110,
  52, 181, 130, 243, 249, 120, 180, 90, 201, 97, 218, 51, 10, 66, 138, 171, 32, 170, 176, 70, 73, 76, 114, 21, 21, 91,
  28, 163, 184, 72, 118, 85, 73, 162, 112, 108, 34, 12, 73, 174, 162, 32, 113, 140, 146, 144, 194, 10, 170, 218, 132, 74,
  85, 25, 11, 70, 28, 127, 113, 169, 123, 224, 220, 237, 177, 163, 32, 132, 108, 223, 214, 214, 74, 197, 35, 205, 237,
  105, 133, 19, 98, 76, 173, 43, 1, 76, 233, 160, 6, 87, 88, 172, 126, 16, 184, 253, 208, 38, 49, 173, 200, 156, 41, 220,
  56, 209, 221, 44, 44, 87, 154, 74, 112, 155, 196, 163, 201, 16, 211, 203, 243, 81, 44, 231, 112, 158, 161, 14, 85, 84,
  83, 230, 143, 163, 182, 156, 153, 202, 65, 234, 135, 8, 4, 229, 71, 60, 202, 175, 196, 5, 39, 191, 58, 57, 225, 211,
  51, 221, 244, 67, 99, 169, 53, 37, 116, 42, 38, 206, 234, 5, 183, 47, 25, 14, 81, 140, 78, 237, 197, 135, 60, 155, 235,
  74, 111, 102, 119, 147, 56, 193, 204, 54, 49, 109, 149, 70, 177, 92, 217, 42, 126, 221, 170, 217, 39, 33, 110, 81, 92,
  141, 127, 24, 83, 214, 170, 56, 240, 226, 58, 42, 60, 81, 98, 180, 233, 198, 136, 159, 91, 77, 66, 16, 101, 9, 15, 13,
  165, 106, 157, 152, 92, 209, 135, 147, 57, 84, 200, 232, 156, 81, 135, 129, 14, 177, 106, 66, 59, 167, 63, 209, 136,
  31, 54, 191, 235, 188, 103, 33, 168, 42, 139, 104, 186, 108, 37, 65, 135, 116, 94, 170, 81, 18, 146, 44, 10, 202, 12,
  156, 163, 130, 92, 240, 45, 201, 212, 89, 50, 112, 5, 59, 118, 213, 26, 98, 230, 228, 229, 99, 133, 229, 202, 65, 194,
  175, 149, 249, 69, 158, 225, 26, 77, 107, 7, 87, 75, 154, 151, 199, 97, 18, 5, 145, 178, 50, 26, 40, 202, 173, 25, 26,
  225, 229, 203, 21, 166, 40, 148, 83, 230, 40, 124, 45, 107, 86, 154, 191, 95, 158, 107, 42, 3, 46, 19, 120, 243, 186,
  12, 217, 47, 146, 87, 193, 195, 180, 133, 203, 206, 93, 208, 65, 56, 107, 238, 129, 4, 41, 145, 220, 88, 66, 27, 40,
  203, 162, 82, 227, 102, 239, 207, 49, 238, 27, 191, 249, 100, 224, 247, 1, 173, 227, 134, 55, 173, 154, 39, 27, 43, 47,
  97, 224, 61, 137, 255, 112, 64, 34, 123, 186, 15, 20, 102, 58, 97, 157, 191, 10, 63, 186, 244, 50, 82, 119, 142, 74,
  199, 12, 236, 13, 211, 129, 106, 222, 226, 124, 109, 165, 6, 121, 138, 254, 56, 39, 91, 80, 4, 236, 134, 43, 108, 60,
  81, 155, 191, 62, 66, 165, 140, 221, 107, 237, 92, 137, 236, 178, 87, 150, 98, 155, 19, 113, 27, 182, 246, 245, 238,
  48, 220, 0, 214, 52, 159, 43, 145, 129, 16, 57, 126, 247, 113, 214, 251, 27, 91, 96, 88, 201, 0, 199, 174, 170, 173,
  241, 214, 110, 212, 25, 44, 109, 235, 160, 186, 173, 159, 255, 54, 88, 165, 173, 157, 123, 221, 123, 93, 119, 91, 126,
  216, 241, 116, 41, 224, 141, 205, 209, 198, 238, 174, 1, 248, 198, 214, 181, 153, 73, 27, 182, 156, 179, 51, 254, 26,
  111, 241, 70, 48, 186, 170, 181, 165, 151, 77, 157, 253, 40, 144, 87, 156, 80, 84, 27, 43, 70, 99, 128, 47, 143, 38,
  215, 143, 249, 181, 176, 252, 73, 150, 78, 229, 214, 198, 222, 175, 48, 237, 252, 20, 160, 193, 22, 117, 196, 247, 108,
  185, 175, 220, 97, 218, 163, 109, 96, 156, 128, 104, 243, 21, 58, 33, 207, 125, 4, 153, 115, 93, 191, 81, 169, 104,
  162, 142, 201, 174, 117, 136, 11, 218, 77, 174, 89, 190, 215, 52, 42, 225, 166, 230, 218, 94, 124, 63, 143, 31, 133,
  153, 141, 145, 211, 11, 172, 202, 15, 236, 203, 222, 163, 103, 27, 112, 28, 85, 168, 103, 99, 231, 222, 159, 27, 39,
  128, 130, 65, 112, 137, 213, 12, 119, 194, 30, 82, 230, 72, 179, 220, 190, 98, 152, 51, 133, 67, 85, 60, 1, 238, 187,
  86, 158, 194, 231, 117, 246, 98, 7, 48, 220, 121, 60, 199, 94, 72, 150, 243, 158, 115, 16, 255, 189, 133, 93, 156, 103,
  139, 92, 237, 32, 183, 186, 255, 182, 220, 203, 11, 247, 104, 219, 133, 219, 229, 102, 104, 86, 171, 118, 52, 244, 236,
  94, 201, 222, 97, 155, 61, 110, 86, 184, 18, 118, 18, 21, 69, 60, 59, 203, 63, 215, 148, 145, 115, 56, 181, 91, 90, 40,
  80, 204, 224, 125, 194, 82, 7, 229, 226, 133, 127, 226, 74, 83, 171, 121, 170, 176, 181, 235, 44, 170, 53, 181, 5, 234,
  44, 114, 215, 146, 203, 55, 85, 226, 171, 94, 211, 44, 200, 38, 22, 243, 147, 45, 117, 92, 205, 125, 182, 17, 232, 5,
  100, 216, 79, 173, 148, 189, 156, 94, 114, 54, 164, 37, 149, 175, 191, 101, 154, 141, 70, 153, 103, 78, 155, 50, 199,
  88, 18, 164, 209, 130, 145, 96, 207, 87, 139, 76, 159, 47, 96, 174, 225, 34, 193, 23, 178, 26, 30, 149, 114, 69, 89,
  24, 60, 0, 72, 202, 44, 128, 192, 149, 254, 139, 16, 229, 152, 196, 115, 49, 26, 241, 156, 69, 23, 247, 12, 222, 217,
  165, 40, 120, 118, 89, 89, 48, 159, 137, 130, 249, 204, 81, 176, 212, 84, 226, 225, 105, 250, 255, 182, 247, 109, 203,
  113, 27, 73, 162, 239, 254, 10, 200, 227, 112, 55, 142, 201, 38, 169, 139, 199, 106, 93, 24, 20, 69, 141, 116, 134,
  146, 120, 72, 218, 62, 49, 146, 131, 4, 217, 96, 19, 135, 77, 0, 3, 160, 121, 145, 162, 35, 230, 105, 63, 96, 247, 196,
  126, 193, 126, 193, 121, 216, 167, 121, 243, 159, 248, 75, 78, 102, 221, 80, 151, 44, 0, 221, 36, 53, 158, 141, 157,
  137, 176, 216, 133, 170, 172, 172, 172, 172, 172, 172, 172, 172, 204, 241, 120, 162, 217, 183, 121, 233, 155, 156, 151,
  187, 4, 214, 91, 133, 6, 12, 224, 191, 248, 248, 140, 237, 132, 188, 107, 246, 13, 187, 119, 210, 184, 137, 125, 70,
  116, 165, 237, 72, 238, 62, 88, 203, 161, 207, 129, 105, 89, 26, 6, 14, 225, 215, 128, 234, 171, 75, 129, 176, 74, 137,
  10, 210, 70, 165, 62, 39, 249, 80, 146, 123, 41, 24, 95, 14, 37, 73, 97, 127, 74, 135, 138, 108, 214, 158, 172, 130,
  178, 242, 221, 209, 218, 115, 232, 199, 48, 212, 10, 247, 11, 71, 74, 214, 73, 8, 29, 68, 157, 35, 186, 248, 44, 239,
  109, 189, 219, 123, 191, 123, 176, 191, 245, 118, 103, 123, 99, 127, 107, 207, 240, 31, 253, 204, 14, 9, 195, 222, 139,
  215, 107, 127, 124, 180, 10, 60, 147, 2, 123, 107, 63, 1, 213, 120, 156, 21, 215, 195, 222, 132, 229, 191, 4, 218, 193,
  209, 25, 106, 36, 247, 143, 209, 72, 63, 138, 134, 247, 215, 224, 223, 227, 201, 240, 254, 125, 221, 251, 83, 2, 222,
  223, 219, 190, 255, 232, 241, 154, 130, 92, 255, 190, 49, 232, 189, 215, 251, 15, 106, 192, 242, 87, 13, 246, 120, 130,
  251, 99, 60, 63, 224, 23, 111, 183, 238, 255, 160, 17, 67, 254, 188, 5, 208, 111, 183, 255, 247, 227, 213, 239, 215,
  30, 42, 224, 90, 129, 70, 16, 126, 146, 156, 23, 246, 235, 191, 172, 61, 174, 1, 191, 94, 230, 63, 53, 172, 153, 25,
  64, 64, 157, 98, 160, 236, 165, 160, 184, 26, 174, 125, 15, 74, 26, 252, 243, 71, 31, 208, 135, 38, 208, 135, 55, 6,
  186, 183, 249, 242, 65, 77, 95, 249, 203, 3, 179, 51, 59, 108, 190, 124, 104, 0, 125, 184, 48, 80, 101, 118, 228, 43,
  104, 243, 253, 238, 214, 129, 88, 70, 111, 94, 238, 177, 228, 45, 151, 1, 104, 1, 253, 15, 220, 42, 87, 243, 131, 152,
  58, 214, 217, 47, 68, 100, 37, 254, 38, 150, 239, 39, 160, 22, 111, 10, 236, 250, 18, 77, 204, 220, 205, 30, 83, 187,
  94, 163, 92, 235, 126, 113, 45, 219, 88, 118, 115, 182, 136, 134, 220, 107, 126, 201, 16, 76, 12, 181, 161, 112, 134,
  55, 130, 5, 50, 115, 180, 64, 249, 43, 203, 90, 206, 134, 64, 198, 178, 167, 98, 253, 215, 104, 125, 144, 67, 249, 133,
  239, 59, 214, 35, 118, 53, 60, 245, 68, 73, 100, 108, 80, 78, 31, 42, 64, 72, 111, 20, 71, 103, 85, 114, 145, 196, 192,
  84, 118, 90, 148, 123, 102, 86, 172, 70, 56, 41, 94, 42, 131, 102, 121, 6, 218, 145, 9, 73, 75, 234, 162, 134, 34, 244,
  67, 105, 187, 53, 46, 150, 68, 122, 19, 235, 50, 146, 181, 31, 228, 160, 18, 199, 60, 87, 34, 47, 64, 43, 140, 139,
  157, 56, 91, 214, 216, 177, 49, 58, 227, 51, 96, 82, 112, 196, 65, 187, 134, 163, 27, 3, 116, 104, 55, 32, 16, 17, 232,
  1, 119, 228, 250, 120, 40, 104, 133, 103, 42, 199, 111, 151, 23, 210, 57, 44, 70, 25, 115, 54, 131, 26, 142, 75, 10,
  215, 171, 150, 161, 134, 125, 231, 123, 15, 202, 252, 89, 208, 212, 162, 96, 80, 113, 15, 135, 173, 115, 160, 202, 45,
  94, 20, 151, 124, 34, 112, 129, 213, 72, 21, 227, 33, 12, 207, 74, 61, 31, 199, 116, 95, 206, 154, 255, 69, 86, 9, 131,
  67, 237, 77, 34, 211, 231, 12, 140, 140, 187, 190, 151, 70, 196, 241, 92, 247, 13, 201, 142, 167, 24, 47, 215, 164,
  236, 198, 4, 84, 187, 63, 240, 171, 122, 161, 192, 4, 3, 142, 254, 50, 14, 190, 87, 199, 38, 196, 159, 120, 148, 108,
  152, 235, 134, 155, 104, 126, 34, 150, 58, 18, 239, 144, 59, 64, 79, 208, 46, 225, 202, 53, 94, 197, 136, 68, 99, 160,
  233, 176, 129, 106, 224, 50, 131, 250, 100, 56, 40, 185, 19, 95, 198, 113, 170, 137, 241, 208, 134, 160, 165, 129, 68,
  28, 104, 207, 186, 81, 60, 154, 230, 241, 159, 101, 12, 15, 94, 117, 128, 155, 17, 114, 155, 248, 137, 187, 17, 190,
  181, 19, 63, 37, 99, 152, 17, 93, 216, 130, 7, 148, 6, 167, 17, 6, 75, 20, 96, 201, 108, 144, 172, 26, 186, 124, 212,
  213, 136, 75, 96, 54, 131, 53, 39, 28, 23, 49, 244, 43, 18, 239, 244, 123, 35, 144, 55, 142, 193, 196, 58, 221, 104,
  156, 225, 212, 116, 22, 151, 92, 8, 222, 229, 22, 120, 22, 152, 108, 39, 11, 214, 197, 122, 195, 179, 19, 59, 51, 16,
  97, 124, 184, 236, 240, 141, 13, 93, 176, 204, 193, 225, 122, 35, 78, 110, 76, 194, 52, 89, 111, 187, 18, 175, 164, 14,
  135, 156, 124, 252, 83, 143, 168, 141, 207, 45, 211, 209, 230, 105, 50, 129, 153, 204, 42, 242, 118, 156, 167, 199,
  234, 62, 80, 214, 194, 14, 65, 164, 56, 15, 25, 81, 227, 75, 100, 211, 89, 208, 119, 25, 51, 60, 108, 65, 151, 117,
  227, 178, 143, 94, 133, 183, 162, 24, 51, 26, 207, 69, 91, 172, 79, 82, 150, 167, 55, 42, 137, 9, 60, 170, 210, 134,
  30, 142, 166, 85, 149, 89, 116, 131, 22, 66, 32, 95, 10, 159, 208, 94, 52, 173, 178, 158, 93, 199, 164, 172, 203, 185,
  47, 149, 186, 130, 59, 48, 6, 1, 168, 127, 218, 176, 220, 128, 164, 160, 167, 29, 159, 193, 158, 204, 237, 77, 50, 173,
  147, 105, 193, 136, 47, 6, 248, 8, 109, 167, 200, 242, 104, 204, 76, 127, 125, 242, 46, 157, 83, 135, 68, 114, 148,
  148, 248, 55, 183, 78, 176, 98, 203, 222, 193, 207, 185, 64, 128, 211, 120, 244, 10, 13, 86, 202, 88, 197, 164, 241,
  10, 244, 112, 146, 140, 123, 104, 80, 4, 149, 232, 52, 27, 13, 123, 59, 239, 247, 246, 161, 224, 148, 153, 159, 203,
  225, 231, 158, 160, 210, 242, 62, 48, 89, 111, 216, 195, 141, 36, 57, 102, 248, 174, 92, 45, 95, 94, 94, 46, 99, 66,
  180, 229, 105, 49, 137, 211, 99, 56, 152, 143, 122, 179, 165, 224, 40, 27, 93, 15, 15, 57, 230, 207, 190, 249, 204,
  255, 152, 125, 155, 140, 158, 41, 30, 77, 70, 179, 67, 251, 69, 117, 203, 177, 124, 230, 178, 147, 206, 168, 48, 21,
  142, 125, 227, 158, 165, 238, 51, 161, 172, 48, 8, 61, 209, 121, 226, 201, 139, 185, 25, 47, 16, 205, 90, 185, 79, 85,
  52, 150, 194, 248, 52, 43, 43, 186, 158, 117, 229, 177, 253, 235, 127, 150, 152, 160, 137, 174, 188, 24, 47, 118, 225,
  70, 249, 130, 11, 89, 166, 56, 239, 31, 242, 41, 10, 154, 101, 210, 101, 82, 156, 65, 255, 167, 193, 68, 96, 189, 126,
  232, 201, 139, 108, 6, 196, 252, 93, 48, 46, 143, 70, 216, 202, 181, 130, 48, 24, 52, 209, 142, 146, 109, 62, 197, 98,
  233, 3, 249, 200, 176, 50, 22, 244, 137, 152, 2, 154, 229, 72, 208, 88, 18, 47, 56, 137, 79, 39, 227, 24, 254, 158, 68,
  227, 56, 101, 215, 174, 8, 21, 168, 142, 0, 185, 164, 38, 194, 96, 4, 30, 138, 219, 177, 9, 90, 214, 159, 27, 14, 194,
  89, 131, 156, 11, 253, 111, 46, 116, 239, 160, 206, 27, 7, 214, 55, 87, 11, 15, 131, 141, 229, 61, 167, 226, 109, 73,
  118, 103, 108, 142, 19, 146, 179, 85, 98, 19, 211, 206, 168, 84, 88, 163, 90, 98, 249, 35, 53, 40, 231, 100, 148, 231,
  198, 235, 12, 49, 113, 55, 189, 205, 48, 214, 25, 177, 251, 243, 161, 73, 235, 176, 248, 101, 61, 110, 209, 95, 33,
  178, 3, 131, 186, 33, 80, 5, 208, 194, 182, 105, 186, 87, 114, 254, 3, 72, 243, 69, 171, 217, 174, 203, 77, 164, 255,
  10, 208, 103, 19, 246, 209, 105, 94, 203, 176, 14, 103, 17, 251, 48, 62, 15, 187, 76, 62, 69, 197, 104, 143, 63, 45,
  93, 123, 226, 124, 217, 23, 116, 179, 188, 127, 235, 10, 27, 163, 139, 72, 250, 173, 114, 43, 191, 125, 34, 28, 103,
  63, 171, 78, 250, 246, 59, 80, 163, 127, 252, 248, 164, 211, 65, 150, 15, 158, 195, 13, 6, 28, 200, 50, 54, 215, 206,
  177, 168, 62, 63, 199, 192, 21, 126, 151, 117, 194, 43, 31, 111, 108, 117, 73, 99, 218, 39, 14, 253, 61, 127, 64, 54,
  101, 127, 61, 251, 248, 53, 134, 26, 136, 243, 217, 199, 175, 127, 57, 116, 220, 234, 177, 135, 80, 244, 212, 238, 199,
  47, 111, 193, 62, 189, 227, 155, 129, 186, 2, 99, 5, 100, 205, 189, 72, 127, 175, 32, 10, 236, 99, 180, 0, 16, 74, 208,
  174, 75, 59, 206, 201, 211, 224, 1, 10, 192, 36, 197, 39, 83, 203, 94, 215, 118, 1, 16, 251, 9, 37, 6, 52, 64, 180,
  169, 116, 0, 233, 218, 21, 240, 153, 148, 224, 35, 49, 25, 84, 164, 21, 88, 64, 182, 159, 30, 159, 48, 105, 154, 225,
  95, 137, 231, 24, 176, 227, 211, 45, 247, 213, 23, 199, 18, 81, 247, 134, 143, 211, 107, 16, 132, 149, 74, 214, 244,
  26, 38, 234, 214, 45, 182, 11, 113, 82, 227, 25, 241, 54, 138, 34, 186, 102, 177, 20, 251, 210, 152, 97, 203, 70, 22,
  100, 162, 194, 165, 80, 159, 216, 205, 40, 231, 10, 160, 90, 58, 80, 68, 219, 60, 178, 188, 105, 27, 206, 152, 43, 128,
  41, 250, 161, 172, 206, 240, 24, 85, 246, 39, 115, 235, 181, 42, 212, 84, 211, 247, 66, 104, 230, 121, 18, 199, 113,
  20, 14, 49, 64, 79, 225, 154, 192, 83, 16, 87, 142, 18, 203, 205, 148, 13, 179, 214, 105, 86, 2, 103, 55, 146, 129,
  109, 45, 162, 243, 156, 129, 128, 71, 109, 105, 203, 39, 62, 247, 236, 249, 9, 109, 146, 26, 64, 51, 125, 218, 173,
  225, 24, 8, 176, 42, 51, 83, 29, 90, 154, 90, 61, 118, 63, 249, 205, 9, 152, 217, 129, 154, 180, 89, 232, 215, 124,
  166, 7, 45, 115, 87, 122, 158, 229, 83, 220, 114, 248, 106, 223, 73, 160, 41, 160, 104, 61, 104, 17, 247, 220, 40, 107,
  70, 17, 232, 240, 56, 136, 114, 20, 57, 78, 12, 102, 213, 227, 137, 172, 122, 60, 105, 174, 186, 123, 37, 106, 22, 87,
  205, 21, 247, 101, 197, 202, 174, 232, 142, 172, 196, 36, 218, 124, 88, 246, 144, 156, 221, 22, 121, 131, 208, 116,
  140, 36, 171, 190, 29, 88, 61, 2, 167, 100, 38, 53, 14, 82, 70, 46, 89, 72, 213, 210, 227, 54, 132, 167, 190, 238, 62,
  223, 229, 138, 243, 13, 66, 173, 194, 4, 116, 195, 187, 90, 134, 8, 187, 235, 58, 100, 117, 111, 105, 33, 26, 141, 36,
  50, 22, 37, 76, 180, 102, 218, 251, 114, 103, 253, 153, 45, 67, 226, 77, 231, 40, 62, 137, 166, 147, 10, 107, 191, 73,
  79, 50, 116, 17, 130, 51, 73, 58, 130, 102, 57, 148, 5, 151, 113, 49, 130, 211, 40, 28, 65, 167, 213, 167, 106, 160,
  211, 236, 131, 90, 197, 106, 141, 202, 21, 40, 22, 216, 47, 245, 92, 141, 230, 13, 155, 166, 5, 48, 19, 182, 47, 55,
  236, 222, 44, 164, 95, 36, 70, 98, 101, 225, 168, 126, 142, 10, 244, 169, 177, 253, 197, 12, 229, 86, 127, 229, 196,
  73, 246, 30, 166, 43, 30, 121, 223, 163, 121, 94, 230, 233, 58, 166, 243, 40, 208, 114, 239, 160, 150, 110, 87, 229,
  135, 223, 59, 184, 53, 76, 89, 235, 126, 167, 76, 70, 204, 199, 145, 200, 167, 164, 169, 45, 234, 108, 109, 65, 99,
  252, 73, 28, 1, 231, 148, 42, 255, 189, 161, 223, 254, 134, 174, 230, 2, 7, 70, 209, 37, 29, 121, 169, 210, 186, 125,
  209, 187, 94, 139, 16, 163, 232, 224, 209, 24, 26, 13, 33, 250, 234, 217, 159, 139, 211, 234, 85, 179, 79, 238, 97,
  246, 183, 69, 86, 11, 187, 69, 124, 230, 130, 242, 45, 149, 246, 233, 225, 16, 159, 241, 167, 124, 108, 110, 76, 226,
  207, 51, 45, 173, 138, 3, 78, 18, 161, 45, 220, 100, 166, 128, 136, 220, 54, 195, 237, 236, 242, 166, 88, 43, 53, 103,
  72, 255, 18, 26, 173, 27, 12, 222, 124, 58, 116, 53, 141, 96, 224, 250, 236, 164, 243, 16, 175, 191, 137, 239, 176,
  109, 38, 210, 62, 81, 92, 164, 125, 14, 93, 96, 237, 216, 118, 219, 67, 248, 198, 213, 128, 58, 141, 181, 31, 97, 151,
  229, 121, 121, 19, 194, 49, 121, 6, 3, 85, 132, 27, 67, 18, 225, 115, 97, 32, 192, 94, 147, 214, 155, 103, 176, 108,
  109, 173, 97, 240, 60, 88, 123, 180, 26, 206, 71, 8, 154, 227, 44, 67, 202, 139, 232, 248, 204, 48, 164, 96, 129, 73,
  17, 81, 24, 202, 234, 173, 211, 101, 40, 12, 42, 210, 222, 218, 146, 110, 9, 91, 14, 214, 66, 123, 170, 218, 45, 64,
  164, 85, 167, 13, 157, 207, 182, 13, 71, 89, 227, 216, 171, 237, 238, 162, 137, 165, 5, 167, 164, 21, 218, 118, 122,
  255, 112, 145, 213, 46, 120, 104, 149, 78, 133, 63, 124, 96, 204, 208, 119, 56, 67, 110, 86, 76, 41, 170, 106, 181,
  209, 146, 86, 198, 7, 91, 96, 25, 31, 67, 27, 204, 156, 83, 169, 110, 221, 122, 63, 203, 11, 53, 0, 116, 4, 19, 51,
  158, 166, 227, 224, 215, 255, 64, 171, 117, 186, 222, 35, 238, 214, 156, 99, 165, 169, 45, 183, 104, 200, 252, 185,
  137, 8, 131, 241, 15, 86, 243, 173, 243, 240, 204, 59, 101, 237, 86, 83, 210, 196, 217, 122, 117, 106, 77, 140, 112,
  81, 200, 70, 215, 194, 251, 232, 199, 221, 237, 189, 56, 42, 142, 79, 119, 162, 34, 58, 55, 78, 231, 88, 107, 0, 180,
  230, 70, 225, 12, 73, 139, 219, 29, 109, 186, 212, 95, 39, 54, 233, 200, 74, 23, 119, 27, 116, 177, 144, 254, 164, 114,
  160, 108, 177, 96, 242, 236, 15, 114, 153, 171, 142, 120, 19, 222, 227, 58, 255, 151, 108, 80, 15, 151, 163, 115, 128,
  192, 145, 157, 120, 183, 36, 97, 164, 138, 193, 253, 124, 205, 90, 28, 1, 22, 188, 224, 144, 63, 16, 253, 137, 69, 165,
  222, 206, 46, 101, 148, 162, 217, 193, 55, 159, 235, 141, 69, 215, 150, 235, 46, 18, 124, 148, 148, 184, 217, 66, 127,
  7, 119, 206, 28, 203, 42, 219, 171, 138, 36, 29, 99, 70, 104, 219, 62, 77, 221, 47, 119, 190, 89, 86, 71, 123, 100,
  118, 249, 238, 71, 222, 29, 247, 196, 53, 243, 89, 150, 2, 246, 1, 119, 32, 61, 77, 210, 79, 211, 113, 124, 242, 235,
  223, 199, 149, 56, 240, 15, 168, 123, 64, 5, 208, 92, 110, 222, 7, 61, 159, 228, 123, 30, 245, 39, 249, 20, 7, 31, 219,
  152, 23, 115, 166, 88, 155, 17, 91, 157, 20, 114, 20, 141, 152, 221, 162, 117, 153, 6, 172, 158, 185, 80, 129, 133,
  171, 3, 44, 238, 121, 42, 186, 92, 165, 127, 45, 153, 69, 179, 175, 145, 6, 10, 194, 117, 190, 108, 216, 62, 216, 91,
  94, 243, 129, 46, 153, 141, 83, 111, 12, 5, 93, 27, 23, 87, 102, 91, 16, 206, 93, 155, 86, 86, 211, 253, 182, 166, 255,
  216, 181, 195, 16, 247, 172, 157, 217, 87, 141, 107, 160, 247, 167, 184, 204, 99, 224, 248, 184, 168, 6, 193, 139, 164,
  130, 5, 240, 46, 158, 178, 83, 67, 48, 154, 2, 151, 192, 10, 56, 45, 44, 230, 159, 143, 245, 231, 98, 252, 76, 15, 96,
  101, 104, 47, 15, 116, 207, 86, 191, 167, 134, 189, 27, 130, 64, 192, 193, 188, 203, 46, 13, 245, 165, 46, 54, 183, 69,
  163, 122, 104, 182, 158, 123, 139, 244, 177, 133, 128, 234, 242, 3, 157, 203, 218, 242, 115, 40, 143, 163, 244, 93, 92,
  93, 102, 197, 89, 105, 199, 198, 146, 51, 172, 222, 20, 162, 237, 114, 10, 211, 11, 179, 90, 125, 2, 57, 118, 22, 15,
  6, 198, 92, 46, 224, 29, 145, 138, 206, 201, 119, 120, 101, 50, 114, 204, 15, 80, 70, 188, 189, 83, 53, 109, 167, 161,
  250, 75, 131, 93, 43, 16, 79, 21, 37, 50, 50, 161, 141, 84, 187, 82, 127, 218, 174, 69, 204, 87, 166, 1, 43, 29, 32,
  142, 84, 13, 199, 128, 197, 171, 50, 247, 212, 116, 80, 192, 223, 179, 209, 139, 243, 240, 208, 110, 171, 141, 185,
  201, 158, 229, 250, 32, 33, 41, 105, 74, 136, 0, 158, 120, 232, 88, 13, 111, 157, 20, 150, 91, 222, 159, 217, 139, 13,
  197, 100, 48, 245, 192, 175, 35, 199, 77, 111, 158, 129, 250, 159, 154, 250, 185, 60, 58, 226, 254, 97, 89, 9, 162,
  183, 215, 193, 131, 133, 226, 229, 249, 124, 87, 20, 217, 23, 240, 90, 209, 110, 211, 178, 124, 55, 22, 175, 63, 95,
  77, 64, 135, 19, 237, 25, 223, 235, 72, 20, 178, 214, 134, 244, 64, 178, 14, 6, 92, 138, 137, 74, 251, 201, 121, 12,
  99, 96, 99, 199, 191, 179, 105, 101, 127, 124, 226, 130, 102, 31, 156, 24, 121, 234, 243, 75, 216, 179, 208, 225, 129,
  69, 70, 55, 250, 21, 72, 135, 230, 36, 169, 129, 177, 92, 188, 152, 52, 134, 215, 162, 143, 97, 70, 139, 157, 136, 63,
  9, 55, 13, 29, 158, 219, 72, 147, 128, 246, 35, 158, 187, 165, 137, 154, 13, 243, 124, 73, 209, 76, 179, 255, 124, 23,
  124, 191, 186, 170, 71, 199, 111, 37, 132, 239, 68, 218, 72, 112, 177, 56, 198, 69, 118, 41, 28, 2, 161, 57, 156, 101,
  172, 173, 64, 70, 109, 158, 176, 196, 58, 228, 142, 166, 148, 113, 99, 216, 212, 229, 132, 65, 114, 53, 250, 111, 191,
  213, 135, 255, 220, 165, 143, 179, 23, 52, 142, 139, 75, 157, 141, 20, 87, 160, 210, 95, 126, 138, 139, 163, 36, 29,
  161, 101, 32, 47, 126, 253, 251, 73, 156, 6, 160, 47, 21, 232, 80, 50, 205, 151, 55, 118, 2, 188, 182, 115, 212, 249,
  14, 43, 139, 246, 36, 213, 101, 147, 155, 224, 205, 61, 97, 157, 112, 69, 224, 180, 170, 242, 225, 202, 138, 61, 39,
  76, 164, 128, 66, 103, 42, 139, 127, 218, 66, 93, 145, 61, 3, 239, 29, 227, 59, 34, 114, 27, 240, 185, 224, 138, 4, 11,
  8, 95, 248, 210, 123, 123, 239, 209, 89, 38, 27, 36, 115, 67, 98, 173, 149, 149, 32, 25, 167, 89, 1, 12, 91, 224, 139,
  67, 15, 209, 156, 37, 134, 115, 46, 214, 37, 50, 227, 18, 154, 68, 87, 105, 207, 12, 252, 222, 247, 153, 217, 21, 96,
  52, 175, 90, 218, 159, 246, 197, 86, 0, 205, 70, 161, 3, 102, 14, 227, 213, 34, 148, 119, 21, 88, 209, 253, 46, 210,
  144, 30, 198, 174, 25, 68, 212, 24, 135, 108, 22, 186, 144, 230, 24, 9, 37, 96, 9, 85, 213, 126, 221, 71, 4, 69, 112,
  76, 72, 85, 231, 160, 13, 50, 134, 157, 140, 204, 32, 90, 174, 139, 63, 84, 204, 134, 161, 189, 114, 197, 129, 165, 64,
  141, 163, 112, 123, 218, 213, 79, 1, 226, 192, 194, 235, 134, 178, 145, 230, 220, 200, 59, 195, 140, 157, 35, 84, 112,
  150, 130, 123, 162, 95, 151, 15, 221, 88, 34, 134, 99, 178, 25, 68, 164, 225, 106, 205, 39, 140, 5, 1, 63, 177, 87,
  159, 110, 68, 146, 39, 115, 138, 38, 234, 0, 43, 130, 62, 220, 209, 83, 3, 137, 245, 51, 80, 154, 216, 183, 31, 119,
  223, 108, 102, 231, 57, 20, 129, 2, 90, 125, 10, 103, 135, 164, 168, 243, 62, 55, 104, 52, 9, 13, 152, 172, 18, 209,
  133, 122, 174, 94, 91, 157, 194, 170, 100, 70, 18, 161, 228, 137, 67, 233, 55, 159, 181, 247, 5, 226, 213, 65, 99, 10,
  64, 145, 25, 145, 135, 28, 150, 99, 196, 73, 250, 100, 182, 105, 204, 216, 88, 125, 178, 210, 52, 106, 48, 221, 60,
  141, 192, 210, 27, 121, 94, 100, 87, 91, 60, 60, 174, 101, 7, 104, 150, 212, 55, 140, 214, 97, 132, 130, 247, 94, 79,
  71, 23, 241, 38, 15, 234, 99, 8, 49, 173, 220, 186, 160, 51, 26, 132, 22, 128, 5, 237, 213, 50, 172, 144, 27, 250, 82,
  24, 120, 236, 231, 158, 244, 249, 120, 254, 85, 19, 216, 203, 38, 184, 137, 201, 231, 80, 160, 13, 235, 70, 252, 229,
  44, 148, 134, 101, 114, 131, 69, 114, 131, 37, 210, 229, 16, 151, 199, 103, 24, 101, 20, 38, 71, 153, 165, 190, 108,
  188, 153, 122, 43, 19, 247, 95, 236, 109, 10, 198, 139, 50, 111, 191, 84, 177, 117, 241, 165, 87, 15, 205, 214, 11,
  114, 172, 140, 87, 101, 135, 108, 196, 174, 61, 60, 187, 32, 171, 91, 175, 134, 173, 222, 104, 195, 94, 73, 132, 167,
  42, 169, 192, 84, 56, 183, 108, 66, 83, 60, 117, 180, 62, 58, 182, 95, 27, 207, 254, 249, 22, 228, 183, 42, 28, 217,
  51, 150, 96, 131, 81, 209, 89, 166, 11, 175, 69, 191, 198, 111, 204, 139, 103, 54, 126, 218, 121, 25, 136, 224, 142,
  65, 116, 86, 77, 163, 9, 168, 104, 184, 218, 158, 24, 243, 81, 191, 230, 200, 240, 184, 105, 109, 114, 218, 129, 134,
  135, 195, 55, 247, 27, 30, 188, 114, 110, 212, 20, 163, 88, 239, 253, 188, 168, 241, 200, 100, 22, 110, 29, 5, 194, 28,
  82, 235, 206, 132, 207, 151, 34, 139, 87, 200, 225, 206, 138, 161, 201, 156, 125, 25, 11, 221, 77, 89, 84, 13, 245,
  118, 11, 10, 183, 238, 17, 241, 106, 243, 178, 109, 88, 110, 148, 102, 57, 144, 164, 110, 128, 191, 58, 136, 63, 60,
  19, 248, 207, 35, 235, 234, 156, 225, 70, 131, 19, 55, 181, 90, 235, 36, 111, 233, 112, 124, 89, 87, 30, 95, 182, 97,
  167, 79, 80, 234, 173, 236, 4, 15, 244, 7, 12, 236, 9, 115, 9, 187, 23, 120, 210, 18, 34, 208, 8, 35, 219, 116, 36, 92,
  76, 76, 99, 231, 119, 40, 162, 145, 93, 232, 131, 6, 126, 9, 103, 223, 34, 123, 208, 21, 240, 11, 84, 224, 172, 128,
  239, 161, 37, 151, 136, 56, 124, 179, 111, 147, 156, 110, 153, 228, 208, 110, 124, 73, 127, 28, 95, 34, 208, 212, 131,
  84, 26, 126, 169, 221, 98, 94, 94, 9, 206, 161, 227, 52, 158, 198, 231, 202, 228, 63, 55, 251, 52, 236, 16, 93, 209,
  121, 5, 139, 15, 214, 98, 25, 236, 64, 87, 104, 2, 108, 197, 128, 95, 172, 91, 91, 133, 107, 124, 239, 73, 128, 210,
  130, 104, 171, 193, 148, 196, 103, 81, 55, 191, 192, 206, 98, 245, 211, 188, 171, 116, 37, 101, 195, 238, 50, 55, 69,
  9, 122, 106, 54, 89, 11, 126, 23, 37, 28, 35, 135, 85, 206, 6, 165, 74, 157, 27, 92, 85, 57, 52, 154, 222, 249, 30,
  101, 19, 220, 79, 238, 159, 97, 197, 148, 113, 2, 224, 227, 201, 4, 168, 18, 91, 90, 177, 208, 224, 185, 219, 155, 180,
  29, 194, 233, 254, 60, 135, 51, 147, 219, 216, 184, 170, 95, 15, 246, 147, 60, 143, 131, 221, 173, 189, 173, 253, 224,
  19, 156, 166, 94, 196, 101, 245, 235, 127, 84, 9, 118, 99, 7, 188, 101, 247, 135, 70, 71, 60, 196, 234, 160, 42, 146,
  115, 144, 41, 70, 46, 52, 158, 2, 133, 1, 198, 75, 169, 57, 6, 188, 113, 52, 142, 143, 138, 140, 71, 238, 144, 86, 101,
  39, 99, 228, 188, 222, 100, 2, 111, 28, 147, 64, 234, 73, 235, 229, 59, 79, 151, 113, 189, 204, 56, 163, 113, 183, 185,
  43, 127, 38, 205, 190, 205, 57, 78, 218, 135, 7, 69, 140, 62, 13, 152, 115, 206, 48, 121, 187, 10, 219, 92, 209, 110,
  91, 34, 221, 54, 216, 29, 125, 86, 92, 175, 111, 2, 162, 191, 157, 221, 60, 156, 244, 36, 179, 67, 73, 215, 169, 41,
  178, 241, 139, 140, 57, 232, 140, 84, 66, 209, 80, 228, 141, 255, 152, 246, 194, 14, 151, 190, 54, 244, 249, 228, 47,
  182, 94, 228, 178, 215, 34, 213, 40, 187, 76, 239, 142, 92, 98, 61, 77, 178, 35, 177, 158, 94, 192, 159, 253, 15, 30,
  154, 253, 178, 36, 67, 143, 98, 104, 205, 171, 106, 37, 159, 68, 73, 218, 35, 149, 15, 224, 118, 0, 9, 203, 83, 120,
  11, 188, 63, 250, 63, 32, 230, 225, 119, 31, 123, 163, 66, 101, 52, 120, 24, 68, 150, 31, 213, 0, 4, 216, 9, 212, 135,
  78, 204, 114, 73, 45, 22, 44, 72, 221, 157, 44, 195, 192, 7, 213, 149, 25, 6, 37, 26, 48, 169, 110, 158, 70, 17, 223,
  34, 190, 200, 206, 52, 124, 161, 147, 47, 206, 4, 98, 253, 30, 71, 169, 123, 224, 18, 133, 214, 194, 173, 171, 134,
  122, 187, 134, 205, 76, 119, 16, 114, 46, 147, 88, 180, 123, 228, 57, 107, 47, 85, 229, 246, 110, 170, 55, 8, 45, 0,
  13, 72, 72, 214, 182, 16, 208, 185, 222, 192, 64, 255, 96, 162, 96, 53, 9, 109, 24, 13, 72, 232, 53, 21, 76, 66, 100,
  193, 30, 83, 65, 195, 27, 175, 195, 92, 192, 161, 214, 226, 4, 176, 211, 79, 162, 172, 38, 162, 76, 164, 171, 133, 82,
  39, 89, 45, 158, 235, 219, 221, 161, 36, 6, 182, 59, 84, 238, 119, 135, 58, 230, 15, 102, 58, 199, 35, 226, 209, 12,
  138, 145, 169, 13, 98, 73, 143, 172, 198, 99, 103, 156, 71, 197, 56, 73, 247, 51, 60, 26, 247, 126, 200, 175, 232, 186,
  70, 46, 222, 167, 152, 52, 56, 29, 63, 255, 230, 115, 206, 223, 0, 62, 93, 17, 37, 79, 143, 138, 149, 231, 60, 199, 45,
  207, 102, 251, 241, 107, 30, 17, 86, 228, 135, 121, 242, 241, 107, 214, 106, 4, 122, 102, 145, 48, 47, 166, 153, 72,
  105, 123, 136, 65, 238, 243, 1, 74, 177, 117, 232, 130, 1, 138, 2, 20, 59, 44, 212, 9, 251, 50, 251, 248, 181, 136,
  105, 2, 101, 7, 71, 147, 40, 61, 3, 128, 219, 73, 122, 246, 116, 37, 2, 8, 67, 234, 242, 136, 77, 144, 238, 210, 132,
  3, 106, 116, 222, 154, 181, 110, 84, 20, 63, 205, 39, 167, 20, 63, 44, 46, 171, 162, 139, 88, 172, 15, 199, 62, 36,
  202, 93, 19, 81, 221, 32, 180, 0, 204, 173, 132, 243, 112, 185, 241, 5, 166, 0, 70, 157, 243, 182, 116, 71, 230, 183,
  108, 174, 198, 55, 182, 53, 137, 108, 136, 172, 232, 52, 197, 101, 208, 165, 177, 198, 145, 14, 140, 151, 240, 173, 11,
  12, 224, 80, 167, 237, 143, 197, 164, 75, 83, 216, 182, 50, 167, 237, 54, 22, 118, 104, 44, 162, 136, 57, 237, 183, 68,
  185, 102, 25, 91, 15, 122, 107, 44, 18, 206, 106, 23, 149, 92, 177, 249, 63, 224, 113, 65, 237, 74, 92, 111, 2, 132,
  222, 221, 16, 192, 105, 192, 195, 107, 219, 161, 135, 185, 88, 125, 238, 137, 125, 206, 222, 145, 128, 192, 147, 97,
  100, 157, 132, 205, 244, 235, 195, 243, 108, 138, 81, 115, 171, 184, 32, 31, 220, 170, 96, 213, 122, 86, 32, 30, 92,
  156, 242, 209, 210, 146, 5, 241, 74, 190, 231, 69, 29, 136, 128, 189, 46, 199, 215, 58, 5, 88, 248, 210, 6, 2, 96, 240,
  80, 223, 248, 155, 3, 139, 18, 143, 47, 227, 230, 0, 142, 44, 254, 56, 2, 123, 201, 131, 59, 184, 9, 142, 172, 40, 236,
  246, 217, 3, 134, 103, 145, 116, 41, 240, 145, 186, 43, 25, 231, 24, 32, 195, 80, 189, 46, 13, 240, 240, 143, 143, 61,
  178, 147, 64, 108, 212, 161, 47, 156, 184, 8, 160, 172, 30, 166, 178, 244, 202, 101, 101, 243, 173, 21, 209, 196, 138,
  130, 238, 204, 157, 17, 3, 217, 158, 188, 86, 130, 26, 47, 101, 107, 132, 106, 30, 242, 53, 242, 50, 183, 235, 53, 72,
  77, 152, 149, 145, 186, 221, 161, 207, 147, 190, 190, 131, 205, 170, 206, 72, 189, 53, 161, 210, 187, 155, 123, 166,
  94, 59, 52, 218, 206, 203, 36, 206, 3, 100, 189, 75, 34, 183, 182, 207, 237, 140, 53, 179, 158, 128, 215, 101, 4, 246,
  226, 209, 119, 253, 119, 3, 230, 54, 26, 86, 223, 34, 49, 188, 161, 110, 56, 169, 234, 181, 84, 242, 76, 205, 168, 255,
  158, 195, 173, 109, 36, 113, 96, 58, 236, 4, 59, 184, 113, 218, 240, 70, 212, 58, 198, 200, 83, 41, 238, 77, 2, 176,
  34, 111, 237, 251, 15, 221, 234, 80, 214, 35, 48, 99, 128, 194, 186, 155, 134, 184, 130, 100, 99, 0, 27, 106, 221, 118,
  106, 238, 201, 255, 78, 204, 61, 67, 201, 153, 125, 107, 240, 198, 80, 20, 153, 229, 175, 133, 121, 0, 186, 184, 5, 14,
  48, 80, 153, 99, 206, 225, 131, 46, 46, 28, 166, 95, 124, 198, 25, 228, 176, 238, 228, 119, 55, 227, 0, 148, 154, 114,
  99, 56, 38, 62, 250, 164, 243, 159, 11, 207, 58, 246, 114, 123, 211, 46, 144, 185, 187, 121, 111, 149, 11, 183, 53,
  235, 243, 11, 137, 185, 230, 252, 193, 234, 136, 154, 115, 40, 38, 231, 156, 87, 15, 205, 214, 11, 207, 57, 246, 114,
  123, 115, 46, 144, 105, 157, 243, 15, 106, 114, 151, 180, 169, 91, 210, 217, 93, 253, 120, 12, 24, 170, 31, 107, 63,
  192, 47, 39, 172, 0, 29, 76, 64, 132, 16, 192, 231, 22, 240, 95, 188, 180, 177, 40, 216, 24, 73, 214, 204, 27, 58, 215,
  132, 62, 166, 39, 244, 177, 103, 66, 31, 155, 19, 250, 248, 102, 19, 250, 248, 86, 39, 244, 241, 29, 76, 232, 131, 91,
  159, 208, 199, 119, 60, 161, 136, 36, 53, 163, 12, 121, 106, 74, 69, 131, 208, 2, 176, 240, 164, 178, 142, 110, 111,
  86, 37, 58, 119, 55, 173, 143, 111, 97, 86, 21, 17, 111, 113, 90, 181, 188, 109, 149, 72, 205, 219, 191, 128, 255, 44,
  225, 115, 78, 188, 218, 254, 60, 11, 157, 89, 80, 57, 124, 131, 11, 149, 202, 23, 125, 210, 241, 169, 86, 143, 165,
  184, 81, 63, 134, 117, 146, 95, 98, 215, 18, 25, 124, 13, 62, 122, 45, 115, 2, 19, 245, 247, 5, 92, 163, 129, 44, 36,
  54, 43, 1, 43, 212, 250, 34, 158, 108, 8, 202, 45, 53, 102, 40, 118, 96, 203, 110, 67, 29, 177, 57, 161, 87, 30, 212,
  239, 33, 241, 7, 60, 127, 118, 232, 155, 195, 175, 190, 162, 169, 105, 47, 76, 135, 160, 38, 113, 228, 178, 172, 127,
  118, 9, 200, 37, 121, 165, 38, 144, 43, 41, 36, 81, 108, 132, 220, 9, 179, 72, 42, 81, 210, 126, 207, 133, 83, 77, 86,
  42, 65, 225, 101, 82, 29, 159, 178, 138, 50, 82, 121, 167, 212, 98, 3, 228, 117, 205, 142, 117, 129, 189, 94, 204, 19,
  122, 29, 1, 104, 250, 156, 232, 221, 156, 121, 94, 39, 20, 117, 219, 5, 146, 38, 249, 206, 163, 36, 117, 228, 30, 191,
  215, 28, 156, 197, 215, 101, 95, 171, 139, 6, 147, 178, 30, 203, 25, 142, 197, 254, 252, 225, 236, 23, 1, 46, 108, 145,
  163, 102, 88, 177, 52, 186, 120, 25, 149, 167, 71, 153, 17, 114, 77, 47, 53, 39, 94, 255, 18, 26, 173, 219, 39, 189,
  158, 203, 30, 82, 108, 121, 84, 119, 97, 241, 35, 192, 149, 89, 225, 116, 156, 156, 20, 108, 2, 35, 81, 30, 106, 237,
  230, 199, 166, 148, 192, 93, 92, 54, 39, 217, 212, 164, 14, 43, 113, 240, 96, 165, 161, 106, 49, 63, 14, 199, 28, 44,
  97, 87, 226, 185, 32, 157, 247, 103, 206, 199, 57, 163, 59, 214, 205, 209, 154, 237, 0, 171, 175, 19, 88, 182, 78, 35,
  83, 73, 11, 47, 123, 119, 113, 79, 230, 10, 55, 238, 157, 26, 252, 46, 226, 239, 25, 187, 246, 109, 129, 161, 179, 214,
  114, 228, 58, 168, 122, 224, 230, 177, 242, 139, 142, 123, 19, 239, 67, 61, 227, 214, 190, 205, 55, 110, 30, 140, 173,
  158, 105, 202, 76, 43, 147, 72, 241, 68, 36, 120, 177, 44, 254, 124, 149, 21, 188, 149, 206, 149, 196, 69, 57, 187,
  201, 149, 116, 213, 81, 85, 116, 221, 124, 191, 253, 126, 247, 96, 103, 99, 123, 107, 127, 127, 235, 195, 234, 47, 238,
  107, 71, 216, 35, 88, 75, 213, 39, 79, 140, 162, 247, 188, 164, 16, 91, 226, 93, 118, 73, 144, 130, 226, 242, 125, 1,
  123, 180, 18, 168, 60, 220, 182, 50, 154, 235, 182, 233, 48, 212, 111, 88, 44, 117, 205, 159, 108, 133, 158, 84, 174,
  83, 120, 102, 213, 249, 184, 192, 180, 10, 181, 251, 38, 243, 106, 128, 240, 79, 172, 139, 237, 45, 204, 172, 222, 247,
  63, 197, 212, 146, 233, 104, 235, 4, 241, 194, 207, 196, 84, 202, 149, 91, 168, 72, 18, 255, 44, 56, 153, 68, 99, 86,
  119, 125, 96, 38, 148, 119, 116, 18, 166, 124, 107, 213, 69, 38, 249, 240, 9, 13, 61, 73, 199, 102, 253, 186, 220, 125,
  149, 113, 154, 93, 110, 106, 72, 213, 8, 194, 201, 230, 94, 148, 219, 245, 241, 162, 214, 244, 165, 133, 217, 60, 119,
  207, 1, 44, 202, 133, 89, 81, 117, 243, 194, 142, 128, 97, 189, 150, 103, 1, 51, 204, 182, 86, 48, 13, 75, 31, 211,
  198, 215, 33, 224, 69, 243, 227, 7, 194, 179, 221, 117, 138, 14, 125, 225, 220, 204, 49, 132, 214, 152, 154, 95, 162,
  83, 147, 132, 48, 25, 41, 205, 129, 177, 162, 70, 104, 230, 188, 194, 242, 36, 193, 11, 175, 15, 189, 110, 99, 20, 143,
  55, 57, 30, 145, 208, 89, 41, 201, 17, 168, 248, 243, 160, 78, 162, 213, 251, 237, 111, 255, 102, 59, 216, 152, 47, 70,
  75, 230, 189, 193, 90, 178, 39, 65, 18, 12, 254, 104, 1, 196, 125, 90, 153, 23, 93, 84, 148, 44, 119, 67, 118, 194, 91,
  99, 120, 38, 126, 92, 75, 167, 231, 71, 113, 129, 39, 220, 250, 195, 208, 214, 144, 61, 47, 24, 220, 193, 118, 29, 79,
  7, 116, 93, 20, 60, 76, 132, 107, 140, 186, 193, 197, 121, 122, 197, 215, 159, 92, 120, 24, 215, 2, 169, 213, 183, 38,
  28, 23, 175, 103, 198, 17, 122, 43, 235, 32, 108, 47, 147, 251, 150, 219, 188, 62, 247, 95, 232, 177, 211, 140, 12, 90,
  97, 158, 119, 89, 209, 207, 132, 52, 99, 59, 136, 108, 16, 214, 109, 45, 236, 140, 249, 88, 71, 159, 249, 35, 88, 250,
  167, 34, 79, 223, 91, 231, 201, 77, 112, 33, 70, 147, 246, 218, 210, 108, 43, 254, 65, 78, 118, 210, 155, 95, 138, 175,
  84, 248, 92, 254, 165, 37, 143, 211, 36, 190, 136, 101, 52, 85, 88, 76, 212, 58, 98, 61, 7, 207, 159, 5, 203, 143, 86,
  225, 231, 67, 24, 146, 42, 249, 30, 75, 30, 232, 37, 127, 196, 146, 251, 122, 201, 15, 171, 242, 173, 85, 136, 255,
  209, 18, 195, 128, 78, 209, 199, 28, 110, 201, 179, 181, 39, 201, 211, 103, 15, 159, 36, 223, 125, 71, 241, 212, 81,
  84, 204, 227, 230, 8, 213, 77, 174, 64, 90, 44, 67, 105, 207, 102, 190, 36, 120, 250, 140, 147, 32, 172, 91, 53, 29,
  229, 57, 112, 238, 23, 121, 26, 39, 227, 83, 17, 43, 238, 251, 224, 187, 32, 249, 31, 15, 102, 249, 213, 161, 25, 15,
  90, 76, 130, 145, 238, 53, 42, 58, 69, 22, 83, 33, 38, 166, 163, 55, 233, 8, 29, 166, 178, 194, 137, 141, 117, 143,
  157, 29, 183, 185, 144, 231, 63, 94, 68, 163, 49, 17, 216, 9, 9, 45, 245, 184, 222, 31, 226, 147, 135, 240, 191, 158,
  249, 89, 102, 123, 238, 101, 39, 39, 204, 54, 243, 132, 232, 139, 31, 130, 100, 70, 121, 99, 182, 84, 251, 104, 106,
  39, 69, 246, 246, 43, 4, 177, 13, 255, 152, 222, 144, 26, 48, 236, 210, 139, 203, 90, 167, 81, 249, 166, 44, 167, 236,
  113, 163, 214, 253, 95, 167, 176, 94, 131, 231, 193, 42, 219, 59, 235, 114, 244, 197, 156, 226, 83, 72, 241, 233, 158,
  142, 51, 48, 79, 197, 188, 56, 109, 54, 83, 157, 56, 174, 193, 114, 56, 121, 145, 29, 33, 63, 247, 236, 68, 202, 114,
  68, 39, 143, 30, 199, 171, 71, 189, 214, 45, 172, 38, 208, 153, 23, 214, 253, 251, 199, 143, 30, 153, 93, 145, 226, 82,
  112, 150, 96, 247, 163, 232, 248, 108, 92, 100, 211, 148, 235, 137, 19, 125, 152, 118, 213, 236, 106, 239, 52, 26, 101,
  104, 125, 62, 92, 13, 240, 255, 15, 242, 171, 224, 155, 207, 172, 217, 236, 193, 131, 67, 171, 41, 99, 89, 75, 166,
  178, 145, 144, 245, 58, 226, 163, 87, 174, 7, 191, 122, 178, 246, 199, 251, 81, 131, 216, 229, 39, 128, 77, 57, 173,
  211, 210, 163, 208, 59, 107, 65, 87, 186, 213, 250, 120, 66, 53, 40, 166, 41, 154, 63, 101, 82, 81, 241, 243, 64, 193,
  1, 249, 139, 33, 24, 79, 128, 193, 209, 33, 147, 104, 57, 212, 186, 178, 154, 123, 186, 4, 77, 180, 24, 241, 163, 1,
  111, 85, 23, 52, 116, 167, 42, 25, 29, 202, 82, 186, 171, 60, 46, 74, 238, 49, 207, 253, 218, 249, 207, 3, 144, 218,
  176, 223, 85, 73, 52, 41, 155, 186, 148, 173, 245, 14, 9, 16, 142, 71, 152, 6, 2, 20, 246, 34, 26, 199, 226, 30, 74,
  87, 19, 197, 23, 118, 124, 18, 73, 30, 152, 103, 169, 82, 23, 141, 10, 67, 15, 80, 178, 83, 242, 104, 55, 240, 31, 234,
  80, 88, 252, 152, 163, 247, 42, 143, 77, 158, 195, 54, 17, 191, 229, 204, 198, 36, 201, 193, 148, 125, 61, 56, 47, 201,
  182, 59, 128, 182, 175, 37, 70, 161, 243, 181, 123, 197, 37, 152, 175, 169, 16, 112, 190, 214, 251, 113, 89, 249, 154,
  86, 240, 205, 106, 103, 202, 70, 62, 92, 104, 251, 142, 233, 22, 131, 164, 124, 149, 164, 9, 156, 152, 13, 90, 132, 48,
  23, 38, 113, 12, 165, 193, 130, 185, 195, 89, 154, 130, 40, 40, 36, 225, 73, 130, 53, 65, 19, 196, 241, 0, 172, 73, 39,
  97, 106, 196, 108, 2, 139, 84, 243, 192, 20, 4, 149, 0, 37, 125, 125, 208, 248, 190, 100, 50, 53, 43, 59, 40, 147, 79,
  49, 121, 2, 210, 62, 15, 249, 172, 189, 1, 173, 201, 254, 132, 71, 174, 85, 80, 255, 215, 86, 233, 249, 83, 27, 159,
  217, 119, 93, 76, 244, 172, 62, 218, 253, 170, 15, 109, 189, 170, 29, 85, 10, 19, 198, 104, 49, 43, 177, 163, 44, 16,
  173, 246, 166, 39, 39, 201, 149, 219, 246, 160, 228, 31, 154, 65, 236, 68, 24, 131, 55, 176, 23, 100, 60, 58, 200, 241,
  139, 169, 24, 200, 6, 94, 104, 187, 113, 84, 178, 183, 192, 53, 60, 60, 172, 196, 7, 5, 255, 224, 130, 227, 45, 220,
  124, 133, 146, 246, 220, 82, 118, 112, 146, 77, 48, 70, 166, 33, 205, 224, 252, 231, 86, 9, 245, 14, 248, 151, 87, 162,
  45, 81, 219, 50, 177, 96, 75, 241, 90, 128, 1, 16, 192, 244, 34, 59, 139, 170, 187, 65, 194, 201, 232, 58, 46, 69, 158,
  221, 30, 209, 195, 46, 223, 202, 244, 30, 244, 34, 127, 15, 114, 95, 108, 239, 65, 238, 93, 70, 31, 70, 97, 67, 47,
  106, 59, 108, 237, 71, 51, 168, 152, 191, 253, 208, 235, 29, 164, 21, 250, 255, 194, 133, 187, 7, 235, 86, 64, 87, 191,
  45, 232, 226, 221, 132, 45, 64, 66, 2, 164, 16, 101, 165, 128, 40, 127, 182, 2, 148, 107, 153, 130, 137, 11, 226, 53,
  38, 159, 15, 140, 159, 126, 18, 24, 92, 185, 30, 28, 242, 61, 160, 4, 209, 136, 103, 229, 32, 73, 81, 135, 164, 171,
  207, 86, 216, 235, 46, 183, 197, 202, 159, 138, 236, 146, 95, 15, 174, 60, 149, 214, 229, 231, 43, 20, 89, 183, 165,
  224, 16, 8, 171, 223, 126, 140, 107, 9, 37, 141, 86, 30, 184, 28, 49, 210, 174, 83, 69, 231, 185, 11, 85, 236, 150,
  174, 92, 96, 187, 9, 33, 125, 96, 98, 136, 200, 68, 231, 57, 29, 67, 142, 125, 122, 41, 114, 170, 199, 151, 44, 184,
  175, 168, 255, 132, 170, 30, 31, 163, 236, 103, 137, 136, 152, 210, 221, 55, 179, 97, 137, 158, 86, 64, 148, 27, 17,
  95, 53, 43, 246, 152, 39, 108, 2, 56, 79, 131, 71, 200, 227, 227, 184, 0, 113, 26, 196, 71, 220, 78, 2, 199, 104, 252,
  58, 43, 161, 106, 102, 133, 89, 183, 200, 232, 6, 107, 87, 195, 97, 201, 85, 142, 163, 73, 44, 223, 12, 177, 0, 238,
  248, 98, 206, 12, 221, 78, 158, 158, 154, 187, 177, 166, 215, 111, 193, 83, 96, 112, 170, 92, 155, 52, 49, 159, 158,
  57, 106, 33, 58, 9, 136, 158, 2, 29, 35, 107, 88, 11, 77, 73, 51, 249, 136, 94, 230, 35, 30, 167, 63, 10, 12, 177, 20,
  235, 130, 230, 181, 184, 35, 182, 102, 223, 82, 100, 53, 249, 206, 170, 239, 135, 188, 164, 25, 244, 46, 181, 81, 235,
  103, 13, 183, 91, 218, 132, 67, 93, 194, 108, 103, 233, 152, 221, 68, 151, 244, 214, 9, 59, 58, 185, 85, 152, 229, 106,
  131, 114, 116, 119, 9, 158, 93, 190, 240, 171, 30, 178, 19, 218, 8, 83, 27, 151, 172, 126, 40, 169, 207, 110, 232, 222,
  101, 21, 8, 90, 75, 220, 89, 95, 197, 217, 124, 196, 195, 166, 227, 145, 223, 198, 18, 56, 242, 100, 18, 95, 137, 157,
  208, 50, 247, 176, 247, 214, 86, 139, 208, 19, 41, 87, 232, 94, 247, 124, 219, 45, 119, 245, 144, 70, 37, 236, 78, 233,
  4, 176, 135, 68, 103, 85, 114, 209, 115, 99, 217, 218, 3, 218, 103, 73, 239, 168, 82, 87, 96, 113, 156, 102, 193, 111,
  127, 251, 191, 193, 167, 56, 1, 225, 56, 201, 206, 64, 108, 5, 247, 31, 158, 30, 118, 94, 39, 117, 136, 104, 227, 167,
  158, 125, 236, 94, 159, 158, 105, 218, 218, 230, 89, 50, 211, 242, 109, 57, 198, 86, 205, 4, 191, 43, 114, 155, 120,
  220, 128, 152, 22, 160, 246, 4, 78, 154, 121, 210, 37, 70, 211, 153, 3, 213, 241, 34, 30, 37, 5, 27, 244, 114, 149, 45,
  99, 252, 240, 178, 71, 44, 9, 223, 192, 122, 123, 113, 113, 1, 138, 80, 13, 38, 168, 178, 224, 245, 254, 254, 206, 222,
  147, 96, 10, 88, 177, 63, 3, 208, 61, 176, 71, 76, 180, 200, 192, 241, 224, 67, 227, 65, 239, 238, 6, 126, 207, 89,
  169, 40, 58, 252, 131, 25, 36, 233, 241, 100, 58, 138, 203, 190, 53, 245, 161, 33, 78, 155, 218, 185, 12, 210, 189,
  173, 159, 142, 189, 48, 156, 103, 66, 230, 36, 41, 25, 5, 181, 201, 125, 145, 129, 92, 46, 152, 187, 135, 243, 26, 23,
  31, 214, 218, 18, 211, 218, 58, 96, 105, 37, 41, 18, 118, 153, 167, 153, 80, 114, 211, 246, 93, 181, 218, 33, 47, 215,
  142, 81, 44, 84, 20, 123, 189, 227, 119, 157, 114, 28, 164, 104, 183, 45, 143, 135, 21, 209, 158, 246, 146, 242, 222,
  63, 120, 183, 38, 41, 225, 172, 98, 97, 66, 148, 210, 185, 54, 36, 190, 7, 206, 40, 146, 17, 145, 52, 66, 109, 206,
  211, 9, 28, 223, 38, 147, 40, 103, 47, 42, 59, 108, 195, 210, 134, 169, 164, 44, 209, 31, 181, 113, 214, 181, 168, 141,
  179, 254, 74, 220, 223, 30, 11, 4, 241, 50, 218, 68, 153, 36, 167, 136, 118, 89, 202, 74, 74, 33, 208, 122, 89, 215,
  186, 129, 250, 85, 132, 233, 74, 181, 142, 194, 230, 33, 188, 200, 70, 215, 46, 185, 95, 176, 152, 4, 22, 23, 235, 120,
  172, 11, 142, 69, 214, 229, 76, 220, 220, 141, 12, 140, 69, 151, 91, 11, 216, 234, 232, 183, 127, 255, 59, 235, 231,
  183, 127, 255, 127, 84, 47, 91, 163, 164, 170, 247, 86, 241, 171, 13, 121, 177, 254, 124, 106, 75, 173, 241, 237, 77,
  207, 207, 163, 194, 134, 224, 101, 36, 138, 237, 154, 41, 35, 58, 112, 73, 35, 62, 144, 194, 68, 98, 213, 160, 121, 53,
  244, 197, 21, 32, 87, 35, 128, 205, 74, 89, 153, 240, 111, 12, 196, 161, 164, 247, 143, 80, 240, 38, 205, 167, 149, 30,
  199, 2, 11, 93, 105, 75, 247, 104, 159, 103, 140, 97, 28, 110, 193, 158, 17, 79, 128, 27, 224, 8, 58, 193, 112, 231,
  136, 192, 12, 147, 4, 170, 64, 229, 225, 161, 149, 6, 116, 214, 229, 118, 8, 47, 252, 233, 187, 33, 219, 98, 102, 240,
  168, 81, 54, 168, 83, 98, 232, 215, 70, 60, 223, 112, 193, 105, 225, 124, 88, 35, 230, 226, 199, 98, 194, 104, 136,
  172, 162, 27, 9, 143, 162, 50, 62, 96, 129, 191, 52, 251, 160, 60, 83, 137, 54, 74, 40, 27, 45, 40, 123, 78, 145, 85,
  25, 8, 0, 145, 38, 206, 234, 42, 23, 95, 169, 174, 204, 150, 102, 135, 178, 29, 53, 42, 201, 26, 118, 95, 138, 137,
  168, 97, 201, 70, 102, 55, 178, 9, 105, 20, 68, 214, 129, 233, 173, 7, 38, 46, 186, 68, 121, 173, 94, 235, 245, 20,
  124, 97, 106, 176, 218, 144, 6, 177, 178, 100, 168, 41, 139, 152, 248, 93, 239, 139, 212, 66, 219, 225, 98, 192, 96,
  35, 163, 204, 102, 35, 234, 206, 237, 158, 117, 231, 214, 229, 122, 13, 150, 133, 158, 21, 203, 23, 16, 143, 5, 211,
  210, 111, 76, 219, 131, 113, 57, 121, 149, 180, 88, 92, 199, 166, 151, 125, 237, 150, 89, 47, 59, 211, 224, 24, 16,
  151, 182, 102, 13, 127, 232, 37, 87, 193, 69, 63, 164, 70, 53, 80, 11, 195, 196, 100, 21, 87, 105, 225, 12, 50, 18,
  177, 154, 122, 79, 58, 235, 219, 86, 40, 41, 250, 58, 209, 78, 102, 213, 102, 86, 104, 11, 96, 200, 242, 113, 8, 171,
  52, 156, 20, 250, 66, 184, 188, 79, 39, 215, 178, 187, 240, 22, 243, 69, 151, 17, 245, 32, 26, 68, 140, 200, 202, 172,
  203, 34, 177, 7, 172, 139, 32, 166, 84, 120, 108, 71, 222, 221, 211, 240, 15, 53, 12, 164, 36, 195, 112, 152, 172, 179,
  176, 147, 76, 243, 129, 147, 114, 138, 71, 193, 240, 200, 52, 196, 23, 15, 122, 164, 118, 100, 136, 51, 95, 55, 82, 78,
  201, 110, 108, 113, 102, 71, 121, 114, 133, 139, 13, 93, 169, 15, 150, 184, 49, 6, 199, 195, 38, 143, 212, 224, 172,
  170, 109, 82, 169, 145, 112, 174, 124, 81, 221, 144, 82, 140, 142, 65, 37, 30, 98, 242, 62, 52, 214, 49, 54, 84, 50,
  144, 85, 115, 72, 172, 26, 34, 201, 199, 202, 214, 37, 113, 38, 238, 130, 8, 108, 155, 54, 23, 31, 161, 212, 222, 33,
  187, 162, 55, 28, 228, 128, 181, 166, 76, 176, 190, 104, 93, 92, 176, 222, 73, 168, 46, 34, 133, 205, 156, 82, 85, 79,
  104, 220, 77, 128, 162, 139, 83, 167, 88, 220, 139, 96, 211, 16, 86, 123, 1, 217, 94, 255, 229, 238, 149, 109, 187, 43,
  250, 85, 176, 6, 125, 82, 199, 156, 103, 76, 184, 75, 6, 204, 56, 207, 188, 104, 187, 88, 48, 110, 152, 191, 133, 117,
  177, 130, 67, 104, 204, 153, 220, 26, 33, 95, 175, 136, 174, 138, 74, 239, 118, 50, 165, 201, 79, 12, 208, 122, 240,
  63, 247, 222, 191, 27, 48, 247, 131, 126, 197, 78, 39, 67, 108, 100, 51, 139, 6, 207, 220, 145, 121, 174, 93, 124, 202,
  192, 99, 240, 51, 67, 23, 211, 154, 176, 16, 149, 113, 165, 162, 139, 2, 39, 82, 149, 124, 156, 49, 82, 170, 40, 251,
  123, 93, 120, 73, 224, 110, 177, 105, 23, 28, 200, 42, 117, 44, 122, 23, 100, 174, 57, 40, 72, 151, 4, 203, 68, 197,
  69, 102, 81, 160, 221, 78, 30, 49, 138, 162, 62, 85, 80, 158, 20, 11, 241, 22, 12, 30, 206, 90, 200, 90, 203, 120, 69,
  149, 96, 30, 40, 228, 182, 10, 111, 223, 112, 48, 179, 111, 62, 51, 28, 161, 86, 240, 219, 191, 252, 107, 192, 127,
  206, 248, 201, 107, 198, 78, 96, 172, 185, 181, 220, 100, 115, 134, 100, 186, 18, 245, 102, 44, 12, 38, 142, 7, 32,
  137, 160, 152, 240, 171, 6, 212, 200, 214, 12, 77, 77, 126, 96, 35, 175, 58, 134, 100, 192, 172, 11, 174, 114, 249,
  185, 126, 40, 51, 228, 121, 93, 117, 15, 168, 161, 150, 41, 85, 124, 49, 92, 71, 134, 124, 230, 102, 119, 165, 159,
  246, 8, 58, 126, 121, 177, 197, 186, 219, 96, 127, 247, 185, 46, 120, 139, 58, 165, 0, 248, 123, 223, 2, 57, 154, 242,
  144, 138, 137, 227, 149, 46, 54, 143, 105, 192, 204, 76, 236, 225, 9, 247, 38, 214, 199, 31, 4, 86, 235, 250, 13, 204,
  152, 39, 185, 143, 43, 235, 102, 6, 203, 179, 60, 175, 122, 79, 154, 250, 242, 237, 212, 158, 56, 125, 126, 198, 186,
  237, 85, 177, 113, 38, 14, 171, 183, 183, 46, 188, 7, 226, 36, 77, 202, 83, 116, 60, 33, 28, 246, 121, 230, 131, 126,
  15, 63, 7, 151, 73, 113, 6, 236, 117, 26, 28, 197, 49, 243, 88, 65, 127, 230, 141, 35, 68, 111, 90, 150, 35, 38, 74,
  129, 45, 49, 95, 68, 156, 174, 83, 225, 21, 23, 33, 197, 150, 128, 88, 247, 116, 20, 23, 128, 197, 98, 122, 193, 77,
  207, 135, 24, 21, 254, 0, 70, 223, 187, 97, 18, 162, 47, 183, 182, 255, 107, 171, 42, 237, 122, 197, 13, 5, 16, 87, 22,
  108, 222, 147, 124, 94, 53, 106, 10, 248, 79, 223, 82, 94, 20, 164, 134, 20, 53, 45, 146, 170, 77, 49, 152, 51, 251,
  207, 66, 242, 201, 55, 138, 47, 190, 117, 195, 34, 24, 199, 252, 204, 161, 89, 220, 27, 68, 25, 171, 186, 172, 213,
  173, 229, 218, 228, 215, 255, 196, 76, 79, 180, 236, 186, 169, 228, 224, 136, 254, 147, 28, 133, 165, 201, 202, 103,
  125, 247, 44, 49, 101, 255, 241, 154, 183, 61, 13, 231, 51, 54, 47, 104, 110, 118, 108, 147, 13, 247, 171, 94, 53, 198,
  184, 248, 108, 184, 236, 148, 209, 123, 200, 59, 200, 27, 223, 66, 58, 215, 141, 115, 95, 5, 210, 62, 74, 55, 186, 177,
  180, 238, 38, 23, 147, 44, 250, 194, 28, 199, 124, 69, 254, 30, 108, 44, 219, 66, 54, 124, 121, 129, 167, 63, 5, 150,
  239, 126, 59, 61, 9, 174, 31, 4, 107, 205, 66, 23, 210, 28, 81, 243, 140, 199, 195, 207, 130, 123, 250, 111, 45, 218,
  189, 25, 100, 130, 251, 123, 226, 147, 65, 144, 120, 59, 209, 53, 115, 46, 246, 102, 157, 226, 228, 140, 172, 192, 204,
  170, 212, 138, 9, 173, 85, 14, 141, 166, 237, 145, 141, 172, 139, 5, 126, 149, 96, 199, 167, 198, 10, 120, 68, 117,
  145, 193, 82, 2, 25, 81, 57, 52, 154, 182, 34, 163, 217, 205, 40, 12, 246, 240, 140, 67, 208, 131, 29, 136, 8, 130,
  136, 234, 161, 217, 186, 21, 11, 253, 24, 44, 78, 91, 30, 116, 178, 156, 194, 38, 203, 73, 100, 178, 92, 199, 37, 203,
  23, 64, 37, 203, 105, 76, 94, 177, 195, 139, 139, 11, 47, 39, 176, 81, 13, 66, 11, 64, 43, 70, 250, 57, 201, 133, 235,
  115, 110, 92, 100, 226, 231, 184, 146, 111, 13, 192, 99, 115, 57, 83, 163, 137, 158, 200, 61, 163, 187, 92, 232, 188,
  147, 222, 238, 62, 122, 219, 187, 232, 13, 247, 208, 219, 221, 65, 103, 238, 52, 117, 135, 59, 199, 212, 201, 168, 73,
  183, 225, 92, 53, 31, 31, 72, 8, 205, 30, 100, 235, 255, 212, 90, 213, 34, 154, 19, 21, 1, 160, 129, 40, 44, 254, 195,
  29, 81, 196, 175, 37, 118, 36, 136, 238, 99, 182, 8, 93, 254, 222, 163, 211, 96, 216, 114, 157, 157, 174, 52, 185, 46,
  127, 183, 203, 117, 207, 1, 146, 10, 245, 25, 87, 47, 227, 11, 150, 160, 114, 18, 247, 187, 198, 249, 28, 197, 23, 203,
  89, 58, 185, 214, 188, 100, 49, 236, 193, 115, 12, 187, 171, 251, 190, 203, 236, 83, 110, 114, 113, 150, 49, 104, 211,
  8, 127, 169, 21, 186, 247, 206, 0, 201, 14, 107, 163, 138, 108, 241, 169, 1, 10, 245, 174, 156, 153, 20, 232, 161, 233,
  193, 114, 216, 53, 1, 170, 158, 194, 26, 143, 38, 96, 180, 19, 176, 71, 11, 230, 153, 82, 46, 140, 13, 95, 148, 153,
  155, 125, 93, 49, 212, 26, 205, 33, 22, 97, 174, 89, 210, 16, 57, 219, 53, 29, 69, 150, 19, 231, 85, 191, 138, 181,
  164, 134, 13, 199, 1, 59, 17, 46, 134, 149, 197, 19, 45, 171, 133, 87, 105, 102, 5, 39, 63, 10, 166, 223, 153, 216, 35,
  86, 165, 150, 130, 163, 85, 14, 141, 166, 93, 162, 221, 182, 141, 150, 212, 143, 89, 212, 25, 208, 241, 109, 12, 181,
  114, 19, 71, 179, 65, 104, 1, 88, 96, 219, 26, 197, 6, 147, 51, 146, 182, 100, 183, 247, 45, 13, 156, 46, 199, 249, 92,
  176, 50, 105, 201, 150, 145, 25, 94, 110, 253, 116, 176, 249, 254, 229, 150, 185, 143, 73, 30, 119, 141, 9, 37, 25, 53,
  10, 72, 16, 176, 215, 2, 9, 243, 42, 112, 235, 119, 184, 173, 232, 60, 143, 86, 35, 93, 172, 181, 4, 152, 107, 72, 145,
  94, 4, 140, 252, 29, 81, 183, 222, 112, 184, 231, 65, 203, 210, 103, 134, 59, 116, 115, 247, 53, 5, 150, 84, 121, 165,
  208, 26, 253, 118, 107, 127, 247, 205, 230, 1, 252, 179, 241, 65, 111, 246, 11, 242, 203, 231, 96, 154, 38, 213, 176,
  135, 201, 69, 49, 170, 203, 48, 208, 171, 152, 137, 170, 49, 2, 228, 113, 114, 14, 131, 31, 174, 5, 51, 147, 57, 174,
  55, 174, 146, 114, 27, 33, 132, 65, 253, 183, 251, 28, 8, 113, 26, 176, 158, 216, 227, 79, 246, 19, 49, 48, 158, 128,
  34, 192, 43, 13, 32, 97, 204, 56, 62, 19, 143, 252, 128, 102, 238, 219, 178, 58, 182, 231, 91, 21, 78, 132, 229, 33,
  10, 131, 43, 31, 114, 189, 191, 196, 73, 21, 244, 95, 191, 30, 190, 125, 59, 220, 219, 227, 207, 148, 66, 235, 66, 77,
  61, 188, 177, 161, 243, 55, 25, 157, 128, 115, 200, 75, 248, 0, 138, 4, 63, 7, 144, 176, 61, 234, 144, 23, 26, 73, 31,
  220, 161, 120, 47, 231, 231, 195, 178, 132, 181, 2, 127, 179, 3, 117, 200, 54, 43, 29, 131, 34, 158, 68, 176, 124, 195,
  246, 104, 69, 230, 59, 71, 162, 111, 140, 245, 207, 110, 64, 172, 242, 199, 158, 114, 43, 127, 129, 54, 33, 34, 114,
  50, 86, 186, 200, 71, 61, 227, 93, 28, 115, 30, 110, 136, 81, 111, 174, 124, 42, 32, 58, 26, 165, 121, 88, 121, 238,
  125, 107, 58, 10, 24, 1, 187, 208, 213, 189, 142, 207, 239, 199, 174, 1, 33, 243, 57, 151, 21, 232, 60, 31, 9, 232,
  155, 81, 122, 17, 129, 200, 182, 75, 200, 55, 4, 18, 163, 118, 173, 134, 69, 147, 101, 193, 130, 5, 124, 167, 168, 173,
  131, 230, 183, 34, 8, 103, 235, 60, 175, 174, 197, 75, 36, 246, 119, 171, 66, 174, 34, 131, 97, 18, 137, 151, 252, 182,
  205, 199, 200, 80, 107, 55, 62, 134, 185, 218, 139, 206, 243, 137, 29, 204, 246, 251, 85, 124, 138, 13, 232, 65, 53, 4,
  244, 38, 101, 84, 118, 66, 222, 74, 216, 142, 214, 153, 165, 34, 149, 130, 175, 127, 158, 214, 238, 77, 9, 36, 153, 84,
  167, 215, 22, 100, 210, 74, 104, 57, 117, 107, 227, 221, 72, 175, 197, 112, 57, 216, 215, 28, 107, 51, 58, 48, 139, 55,
  233, 103, 53, 141, 108, 110, 32, 219, 201, 36, 187, 124, 207, 159, 254, 53, 242, 171, 6, 196, 210, 29, 52, 44, 241,
  133, 151, 62, 73, 44, 14, 166, 32, 24, 139, 94, 171, 245, 22, 186, 242, 157, 247, 190, 29, 143, 97, 11, 148, 62, 62,
  252, 151, 241, 105, 9, 147, 49, 147, 15, 221, 58, 48, 150, 115, 66, 27, 193, 100, 84, 49, 111, 179, 29, 93, 103, 211,
  170, 20, 217, 80, 60, 193, 198, 61, 169, 82, 26, 194, 141, 219, 219, 180, 41, 49, 242, 40, 129, 3, 217, 78, 150, 164,
  44, 115, 136, 9, 85, 70, 131, 85, 41, 57, 140, 28, 197, 102, 75, 60, 27, 179, 192, 182, 88, 218, 39, 89, 88, 76, 153,
  222, 112, 0, 98, 109, 92, 157, 138, 57, 238, 180, 36, 136, 136, 180, 115, 202, 76, 214, 202, 144, 148, 206, 173, 149,
  239, 145, 181, 151, 26, 95, 96, 34, 111, 202, 164, 68, 220, 113, 225, 201, 86, 23, 185, 149, 68, 220, 237, 110, 65,
  186, 61, 1, 196, 67, 202, 169, 64, 50, 142, 193, 71, 44, 50, 113, 7, 70, 194, 215, 161, 63, 169, 189, 71, 223, 152,
  170, 171, 37, 103, 87, 210, 249, 135, 11, 65, 104, 187, 207, 82, 121, 148, 75, 110, 66, 79, 126, 224, 95, 18, 40, 90,
  139, 233, 175, 211, 184, 172, 54, 82, 208, 72, 81, 97, 126, 85, 128, 190, 221, 231, 231, 166, 47, 137, 83, 216, 37,
  148, 101, 130, 193, 40, 174, 213, 10, 173, 243, 35, 115, 253, 254, 53, 255, 222, 101, 243, 49, 188, 73, 112, 156, 47,
  235, 125, 217, 219, 126, 201, 196, 96, 137, 29, 13, 22, 21, 157, 220, 31, 67, 237, 238, 150, 32, 109, 123, 159, 135,
  164, 77, 38, 42, 20, 59, 181, 13, 178, 115, 9, 17, 162, 29, 51, 94, 88, 65, 206, 120, 17, 17, 224, 140, 125, 24, 90,
  43, 90, 76, 198, 244, 220, 130, 2, 37, 201, 40, 169, 174, 73, 72, 234, 35, 13, 13, 152, 201, 130, 198, 74, 116, 64,
  184, 227, 169, 0, 115, 239, 162, 119, 125, 89, 45, 148, 125, 96, 19, 26, 60, 198, 200, 183, 141, 91, 23, 156, 132, 174,
  129, 171, 110, 242, 46, 19, 218, 130, 217, 132, 23, 187, 77, 114, 122, 235, 16, 103, 166, 65, 57, 65, 57, 179, 252,
  253, 170, 109, 46, 208, 176, 195, 45, 191, 238, 153, 182, 32, 168, 217, 194, 145, 98, 11, 54, 21, 218, 111, 73, 58,
  249, 219, 160, 26, 182, 14, 157, 82, 0, 193, 10, 115, 99, 19, 67, 32, 171, 230, 182, 80, 35, 215, 65, 25, 168, 7, 102,
  159, 193, 206, 98, 84, 98, 208, 139, 56, 237, 117, 219, 219, 155, 58, 182, 212, 91, 111, 183, 135, 63, 237, 188, 12,
  190, 249, 12, 244, 129, 35, 249, 171, 228, 10, 38, 231, 126, 56, 11, 206, 118, 34, 237, 216, 236, 74, 61, 156, 115, 41,
  242, 216, 223, 66, 222, 241, 137, 245, 202, 58, 89, 251, 148, 93, 137, 18, 210, 119, 62, 217, 123, 187, 88, 16, 137,
  205, 176, 221, 62, 247, 248, 51, 97, 13, 64, 205, 100, 71, 64, 179, 244, 52, 25, 51, 31, 182, 111, 62, 187, 213, 117,
  250, 254, 246, 183, 127, 115, 170, 96, 91, 103, 14, 216, 75, 117, 35, 188, 144, 18, 14, 56, 4, 222, 214, 89, 141, 188,
  216, 182, 193, 25, 31, 67, 19, 132, 17, 49, 252, 240, 105, 153, 71, 232, 204, 29, 149, 229, 179, 143, 95, 79, 152, 186,
  177, 92, 94, 162, 23, 200, 199, 175, 3, 198, 107, 80, 94, 7, 227, 29, 50, 6, 98, 10, 69, 159, 160, 19, 44, 162, 213,
  193, 15, 225, 236, 227, 215, 207, 159, 174, 32, 232, 231, 193, 95, 146, 120, 2, 108, 87, 83, 120, 118, 232, 88, 170,
  65, 233, 123, 3, 108, 90, 92, 68, 147, 62, 115, 239, 64, 78, 95, 10, 238, 63, 210, 34, 93, 57, 117, 52, 23, 144, 165,
  224, 209, 170, 167, 42, 223, 49, 246, 69, 140, 65, 172, 106, 196, 207, 202, 179, 124, 58, 129, 133, 136, 21, 62, 193,
  66, 18, 122, 147, 14, 138, 50, 244, 217, 80, 235, 47, 184, 249, 236, 197, 21, 198, 207, 215, 74, 203, 227, 40, 125, 23,
  87, 151, 89, 113, 102, 213, 221, 206, 244, 122, 126, 223, 22, 30, 31, 139, 221, 53, 104, 165, 121, 1, 40, 200, 157,
  223, 132, 194, 142, 118, 162, 8, 230, 226, 184, 72, 242, 234, 249, 87, 248, 55, 122, 177, 241, 191, 78, 171, 243, 9,
  254, 245, 213, 255, 7, 252, 88, 38, 142, 21, 204, 2, 0,
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
  time_t start = parseDayKey(from);
  time_t end = parseDayKey(to);
  if (start == 0 || end == 0 || start > end) return keys;
  const time_t daySec = 24 * 60 * 60;
  int totalDays = (int)((end - start) / daySec) + 1;
  if (totalDays > 0) keys.reserve(totalDays);
  for (time_t t = start; t <= end; t += daySec) {
    String key = formatDayKey(t);
    if (key.length() == 0) continue;
    keys.push_back(key);
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
    size_t maxDays = retentionDays();
    if (maxDays > 0 && keys.size() > maxDays) {
      keys.erase(keys.begin(), keys.end() - maxDays);
    }
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
      cloudConfig.retentionMonths = (uint8_t)std::max(1, std::min(6, r));
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
    for (const auto &existing : sensorConfigs) {
      if (existing.id == cfg.id) {
        server.send(409, "text/plain", "duplicate id");
        return;
      }
      if (existing.type.equalsIgnoreCase(cfg.type) && existing.category == cfg.category) {
        server.send(409, "text/plain", "sensor already exists");
        return;
      }
    }
    cfg.enabled = true;
    sensorConfigs.push_back(cfg);
    saveSensorConfigs();
    server.send(200, "text/plain", "added");
    return;
  }
  if (action == "delete") {
    if (!server.hasArg("id")) { server.send(400, "text/plain", "missing id"); return; }
    String id = server.arg("id");
    if (id == "lux" || id == "climate" || id == "leaf" || id == "co2") {
      server.send(400, "text/plain", "core sensor");
      return;
    }
    size_t before = sensorConfigs.size();
    sensorConfigs.erase(std::remove_if(sensorConfigs.begin(), sensorConfigs.end(),
                                       [&](const SensorConfig &cfg) { return cfg.id == id; }),
                        sensorConfigs.end());
    if (sensorConfigs.size() == before) {
      server.send(404, "text/plain", "not found");
      return;
    }
    applyPinsFromConfig();
    saveSensorConfigs();
    rebuildSensorList();
    server.send(200, "text/plain", "deleted");
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
