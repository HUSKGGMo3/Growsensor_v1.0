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
String buildCloudUrl(const String &path);
bool ensureCollection(const String &path, const char *label);

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
  cloudStatus.lastHttpCode = 0;
  cloudStatus.lastUrl = "";
  cloudStatus.lastStateReason = reason;
  cloudStatus.lastStateChangeMs = currentEpochMs();
  refreshStorageMode(reason);
}

void markCloudFailure(const String &msg) {
  lastCloudOkMs = 0;
  cloudStatus.lastError = msg;
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
  if (cloudConfig.retentionMonths < 1) cloudConfig.retentionMonths = 1;
  if (cloudConfig.retentionMonths > 4) cloudConfig.retentionMonths = 4;
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
  return normalizeCloudBaseUrl(cloudConfig.baseUrl);
}

String normalizeCloudBaseUrl(String url) {
  url.trim();
  if (url.length() == 0) return "";
  if (url.startsWith("https://")) {
    url = "http://" + url.substring(strlen("https://"));
  } else if (!url.startsWith("http://")) {
    url = "http://" + url;
  }
  String rootTag = String("/") + CLOUD_ROOT_FOLDER;
  int idx = url.lastIndexOf(rootTag);
  if (idx > 0) {
    int end = idx + rootTag.length();
    if (end == (int)url.length() || url.charAt(end) == '/') {
      url = url.substring(0, idx);
    }
  }
  return ensureTrailingSlash(url);
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

String buildCloudUrl(const String &path) {
  String base = safeBaseUrl();
  if (base.length() == 0) return "";
  String cleanedPath = path;
  while (cleanedPath.startsWith("//")) cleanedPath.remove(0, 1);
  return joinUrl(base, cleanedPath);
}

String cloudShortPath(const String &path) {
  if (path.length() == 0) return "/";
  return path.startsWith("/") ? path : (String("/") + path);
}

void setCloudError(const char *op, const String &label, const String &path, int code, const String &detail = "") {
  cloudStatus.lastHttpCode = code;
  cloudStatus.lastUrl = cloudShortPath(path);
  String msg = String(op) + " " + label + " -> " + String(code);
#if CLOUD_DIAG
  if (detail.length() > 0) msg += String(" ") + detail;
#endif
  markCloudFailure(msg);
}

bool webdavRequest(const String &method, const String &url, const String &body, const char *contentType, int &code, String *resp = nullptr, unsigned long timeoutMs = CLOUD_TEST_TIMEOUT_MS) {
  HTTPClient http;
  http.setTimeout(timeoutMs);
  WiFiClient client;
  if (url.startsWith("https://")) {
    code = -1;
    if (resp) *resp = "HTTPS disabled";
    return false;
  }
  if (!http.begin(client, url)) return false;
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
  if (resp) {
    String out = http.getString();
    if (out.length() > 120) out = out.substring(0, 120);
    *resp = out;
  }
  http.end();
  return code > 0;
}

bool ensureCollection(const String &path, const char *label) {
  String url = buildCloudUrl(path);
  int code = 0;
  String resp;
  bool ok = webdavRequest("MKCOL", url, "", "text/plain", code, &resp);
  if (!ok) {
    setCloudError("MKCOL", label, path, code, resp);
    return false;
  }
  if (code == 201 || code == 200 || code == 204 || code == 405 || code == 409) return true;
  setCloudError("MKCOL", label, path, code, resp);
  return false;
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
  bool ok = webdavRequest("PROPFIND", url, "", "text/plain", code, &resp);
  if (!ok) {
    setCloudError("PROPFIND", "base", "/", code, resp);
    return false;
  }
  if (code == 401 || code == 403) {
    setCloudError("PROPFIND", "auth", "/", code, resp);
    return false;
  }
  if (code == 404) {
    setCloudError("PROPFIND", "base", "/", code, resp);
    return false;
  }
  if (code >= 200 && code < 400) {
    if (!cloudEnsureFolders("")) {
      return false;
    }
    markCloudSuccess("cloud ok");
    return true;
  }
  setCloudError("PROPFIND", "base", "/", code, resp);
  return false;
}

bool testCloudConnection() {
  return pingCloud(true);
}

bool cloudGet(const String &path, String &resp, int &code) {
  String fullPath = path.startsWith("/") ? path : (String("/") + path);
  String url = buildCloudUrl(fullPath);
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
  markCloudSuccess();
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
  String url = buildCloudUrl(fullPath);
  int code = 0;
  String resp;
  const char *ctype = job.contentType.length() > 0 ? job.contentType.c_str() : "application/json";
  bool ok = webdavRequest("PUT", url, job.payload, ctype, code, &resp, CLOUD_TEST_TIMEOUT_MS);
  if (!ok || (code != 200 && code != 201 && code != 204)) {
    setCloudError("PUT", "upload", fullPath, code, resp);
    return false;
  }
  cloudStatus.lastUploadMs = currentEpochMs();
  cloudStatus.lastUploadedPath = fullPath;
  cloudStatus.lastError = "";
  markCloudSuccess(job.kind.length() > 0 ? job.kind : "upload");
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
  bool ok = webdavRequest("PUT", buildCloudUrl(pathOut), body, "text/plain", code, &resp, CLOUD_TEST_TIMEOUT_MS);
  httpCode = code;
  if (ok && (code == 200 || code == 201 || code == 204)) {
    cloudStatus.lastTestMs = currentEpochMs();
    cloudStatus.lastUploadMs = cloudStatus.lastTestMs;
    cloudStatus.lastUploadedPath = pathOut;
    cloudStatus.lastError = "";
    markCloudSuccess("test upload");
    return true;
  }
  setCloudError("PUT", "test", pathOut, code, resp);
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
static const char INDEX_HTML[] PROGMEM = R"HTML(
  <!doctype html>
  <html lang="de">
  <head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1" />
    <title>GrowSensor v0.3.3</title>
    <style>
      :root { color-scheme: light dark; }
      html, body { background: #0f172a; color: #e2e8f0; min-height: 100%; }
      body { font-family: system-ui, sans-serif; margin: 0; padding: 0; }
      header { padding: 16px; background: #111827; box-shadow: 0 2px 6px rgba(0,0,0,0.25); position: sticky; top: 0; z-index: 12; }
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
      .sensor-card { position:relative; overflow:hidden; min-height:180px; display:flex; flex-direction:column; }
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
      .status-dot.dot-pulse-fast { animation: dotPulse 2.4s ease-in-out infinite; }
      .status-dot.dot-pulse-slow { animation: dotPulse 4.8s ease-in-out infinite; }
      .status-dot.dot-pulse-idle { animation: dotPulse 7s ease-in-out infinite; }
      @keyframes dotPulse { 0% { box-shadow:0 0 0 0 rgba(52,211,153,0.15); } 70% { box-shadow:0 0 0 12px rgba(52,211,153,0); } 100% { box-shadow:0 0 0 0 rgba(52,211,153,0); } }
      .tile-title { display:flex; align-items:center; gap:8px; }
      .tile-header { display:flex; align-items:center; justify-content:space-between; gap:8px; margin-bottom:6px; }
      .metric-tile { cursor: pointer; position:relative; transition: max-height 220ms ease, transform 180ms ease, opacity 180ms ease, border-color 120ms ease; overflow:hidden; max-height:720px; padding-bottom:48px; }
      .metric-tile:hover { transform: translateY(-2px); border-color: #334155; }
      .tile-content { position:relative; z-index:2; display:flex; flex-direction:column; gap:8px; pointer-events:auto; height:100%; }
      .tile-body { position:relative; transition:max-height 240ms ease, opacity 200ms ease, transform 200ms ease; max-height:720px; opacity:1; transform:translateY(0); overflow:hidden; min-height:156px; display:flex; flex-direction:column; gap:8px; z-index:2; }
      .value-panel { background: linear-gradient(180deg, rgba(15,23,42,0.9), rgba(15,23,42,0.7)); backdrop-filter: blur(12px); -webkit-backdrop-filter: blur(12px); border:1px solid rgba(148,163,184,0.22); box-shadow: 0 14px 30px rgba(0,0,0,0.42); border-radius: 12px; padding: 10px 12px; display:inline-flex; flex-direction:column; gap:4px; max-width:100%; position:relative; z-index:2; }
      .tile-subtext { font-size:0.9rem; color:#cbd5e1; line-height:1.3; }
      .metric-tile.collapsed { max-height:96px; min-height:64px; padding-bottom:16px; opacity:0.98; transform:translateY(0); }
      .metric-tile.collapsed .tile-body { max-height:0; opacity:0; transform:translateY(-6px); display:none; }
      .metric-tile.collapsed .hover-chart { opacity:0.22; }
      .metric-tile.collapsed .tile-header { margin-bottom:0; }
      .metric-tile.collapsed:hover { transform:none; }
      .hover-chart { position:absolute; inset:0; padding:12px; background:radial-gradient(circle at 20% 20%, rgba(34,211,238,0.05), rgba(15,23,42,0.55)), rgba(15,23,42,0.72); border:1px solid #1f2937; border-radius:12px; display:flex; align-items:stretch; justify-content:stretch; pointer-events:auto; z-index:1; opacity:0.32; visibility:visible; transition:opacity 180ms ease; }
      .metric-tile:hover .hover-chart { opacity:0.6; }
      .tile-eye { position:absolute; left:12px; bottom:12px; width:26px; height:26px; border-radius:50%; border:1px solid #1f2937; background:#0b1220; color:#e2e8f0; display:inline-flex; align-items:center; justify-content:center; gap:2px; padding:0; box-shadow:0 4px 8px rgba(0,0,0,0.18); transition:border-color 140ms ease, background 140ms ease, transform 140ms ease, box-shadow 160ms ease; z-index:3; }
      .tile-eye:hover { border-color:#334155; background:#111827; transform:translateY(-1px); box-shadow:0 6px 14px rgba(0,0,0,0.3); }
      .tile-eye:active { transform:translateY(0); }
      .tile-eye svg { width:18px; height:18px; display:block; }
      .tile-eye .eye-closed { display:none; }
      .metric-tile.collapsed .tile-eye .eye-open { display:none; }
      .metric-tile.collapsed .tile-eye .eye-closed { display:block; }
      .hover-chart canvas { width:100%; height:100%; min-height:100%; display:block; flex:1 1 auto; pointer-events:auto; }
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
      .reconnect-panel { margin-top:10px; padding:10px; border:1px dashed #334155; border-radius:10px; background:rgba(15,23,42,0.55); }
      .cloud-grid { display:grid; grid-template-columns:repeat(auto-fit,minmax(240px,1fr)); gap:10px; }
      .cloud-status { display:grid; grid-template-columns:repeat(auto-fit,minmax(160px,1fr)); gap:8px; margin-top:8px; }
      .cloud-pill { background:#0b1220; border:1px solid #1f2937; border-radius:10px; padding:8px; display:flex; justify-content:space-between; align-items:center; }
      .cloud-credentials { margin-top:10px; border:1px solid #1f2937; border-radius:12px; padding:10px; background:#0b1220; }
      .cloud-credentials-header { display:flex; align-items:center; justify-content:space-between; gap:8px; }
      .cloud-credentials-toggle { width:auto; padding:4px 8px; font-size:0.9rem; }
      .cloud-credentials-summary { display:none; align-items:center; justify-content:space-between; gap:8px; margin-top:6px; font-size:0.9rem; color:#cbd5e1; }
      .cloud-credentials-body { overflow:hidden; max-height:1200px; opacity:1; transition:max-height 200ms ease, opacity 160ms ease; }
      .cloud-credentials.collapsed .cloud-credentials-body { max-height:0; opacity:0; pointer-events:none; }
      .cloud-credentials.collapsed .cloud-credentials-summary { display:flex; }
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
      .kpi-wrap { display:flex; justify-content:center; width:100%; }
      .kpi-bar { display:grid; grid-template-columns: repeat(auto-fit, minmax(160px,1fr)); gap:8px; margin-top:12px; align-items:stretch; position:sticky; top:0; z-index:11; background:#111827; padding:4px 0; max-width:720px; width:100%; }
      .kpi-item { background:#0b1220; border:1px solid #1f2937; border-radius:10px; padding:10px; display:flex; justify-content:space-between; align-items:center; gap:10px; box-shadow:0 6px 14px rgba(0,0,0,0.25); min-height:56px; }
      .kpi-text { display:flex; flex-direction:column; gap:4px; }
      .kpi-label { font-size:0.9rem; color:#cbd5e1; }
      .kpi-value { font-size:1.1rem; font-variant-numeric:tabular-nums; color:#e2e8f0; display:inline-flex; align-items:center; gap:6px; }
      .kpi-trend { font-size:1.1rem; font-weight:700; display:inline-flex; align-items:center; gap:6px; }
      .trend-up { color:#22c55e; }
      .trend-down { color:#f87171; }
      .trend-flat { color:#e5e7eb; }
      .trend-warn { color:#f59e0b; }
      .trend-good { color:#22c55e; }
      .trend-bad { color:#ef4444; }
      .chart-tooltip { position:fixed; pointer-events:none; background:rgba(15,23,42,0.92); border:1px solid rgba(148,163,184,0.25); color:#e2e8f0; padding:8px 10px; border-radius:10px; box-shadow:0 10px 30px rgba(0,0,0,0.35); font-size:0.9rem; backdrop-filter: blur(6px); -webkit-backdrop-filter: blur(6px); z-index:120; max-width:240px; }
      .chart-tooltip.hidden { display:none; }
      .chart-tooltip .label { font-size:0.8rem; color:#cbd5e1; }
      .chart-tooltip strong { color:inherit; font-weight:700; }
      @media (prefers-reduced-motion: reduce) {
        .status-dot.dot-pulse-fast, .status-dot.dot-pulse-slow, .status-dot.dot-pulse-idle { animation:none; }
      }
    </style>
  </head>
  <body>
    <header>
      <div class="header-row">
        <div>
          <h1>GrowSensor – v0.3.3</h1>
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
      <div class="kpi-wrap">
        <div class="kpi-bar" id="kpiBar">
          <div class="kpi-item" data-metric="temp">
            <div class="kpi-text">
              <div class="kpi-label">Temp (°C)</div>
              <div class="kpi-value"><span id="kpiTemp">–</span><span class="kpi-trend" id="kpiTempTrend">–</span></div>
            </div>
          </div>
          <div class="kpi-item" data-metric="humidity">
            <div class="kpi-text">
              <div class="kpi-label">Humidity (%)</div>
              <div class="kpi-value"><span id="kpiHumidity">–</span><span class="kpi-trend" id="kpiHumidityTrend">–</span></div>
            </div>
          </div>
          <div class="kpi-item" data-metric="co2">
            <div class="kpi-text">
              <div class="kpi-label">CO₂ (ppm)</div>
              <div class="kpi-value"><span id="kpiCo2">–</span><span class="kpi-trend" id="kpiCo2Trend">–</span></div>
            </div>
          </div>
          <div class="kpi-item" data-metric="vpd">
            <div class="kpi-text">
              <div class="kpi-label">VPD (kPa)</div>
              <div class="kpi-value"><span id="kpiVpd">–</span><span class="kpi-trend" id="kpiVpdTrend">–</span></div>
            </div>
          </div>
        </div>
      </div>
      <nav>
        <button id="navDashboard" class="menu-btn">Dashboard</button>
        <button id="navSensors" class="menu-btn">Sensoren</button>
        <button id="navCloud" class="menu-btn">Cloud</button>
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
        <article class="card metric-tile sensor-card" data-metric="lux">
          <div class="hover-chart"><canvas class="hover-canvas" data-metric="lux" width="320" height="140"></canvas></div>
          <div class="tile-content">
            <div class="card-header tile-header">
              <div class="tile-title"><div>Licht (Lux)</div><span class="status-dot" id="luxDot"></span></div>
            </div>
            <div class="tile-body">
              <div class="value-panel">
                <div class="value" id="lux">–</div>
              </div>
            </div>
          </div>
        </article>
        <article class="card metric-tile sensor-card" data-metric="ppfd">
          <div class="hover-chart"><canvas class="hover-canvas" data-metric="ppfd" width="320" height="140"></canvas></div>
          <div class="tile-content">
            <div class="card-header tile-header">
              <div class="tile-title"><div>PPFD (µmol/m²/s)</div><span class="status-dot" id="ppfdDot"></span></div>
            </div>
            <div class="tile-body">
              <div class="value-panel">
                <div class="value" id="ppfd">–</div>
                <div class="tile-subtext">Spektrum: <span id="ppfdSpectrum">–</span><br/>Faktor: <span id="ppfdFactor">–</span></div>
              </div>
            </div>
          </div>
        </article>
        <article class="card metric-tile sensor-card" data-metric="co2">
          <div class="hover-chart"><canvas class="hover-canvas" data-metric="co2" width="320" height="140"></canvas></div>
          <div class="tile-content">
            <div class="card-header tile-header">
              <div class="tile-title"><div>CO₂ (ppm)</div><span class="status-dot" id="co2Dot"></span></div>
            </div>
            <div class="tile-body">
              <div class="value-panel">
                <div class="value" id="co2">–</div>
              </div>
            </div>
          </div>
        </article>
        <article class="card metric-tile sensor-card" data-metric="temp">
          <div class="hover-chart"><canvas class="hover-canvas" data-metric="temp" width="320" height="140"></canvas></div>
          <div class="tile-content">
            <div class="card-header tile-header">
              <div class="tile-title"><div>Umgebungstemperatur (°C)</div><span class="status-dot" id="tempDot"></span></div>
            </div>
            <div class="tile-body">
              <div class="value-panel">
                <div class="value" id="temp">–</div>
              </div>
            </div>
          </div>
        </article>
        <article class="card metric-tile sensor-card" data-metric="humidity">
          <div class="hover-chart"><canvas class="hover-canvas" data-metric="humidity" width="320" height="140"></canvas></div>
          <div class="tile-content">
            <div class="card-header tile-header">
              <div class="tile-title"><div>Luftfeuchte (%)</div><span class="status-dot" id="humidityDot"></span></div>
            </div>
            <div class="tile-body">
              <div class="value-panel">
                <div class="value" id="humidity">–</div>
              </div>
            </div>
          </div>
        </article>
        <article class="card metric-tile sensor-card" data-metric="leaf">
          <div class="hover-chart"><canvas class="hover-canvas" data-metric="leaf" width="320" height="140"></canvas></div>
          <div class="tile-content">
            <div class="card-header tile-header">
              <div class="tile-title"><div>Leaf-Temp (°C)</div><span class="status-dot" id="leafDot"></span></div>
            </div>
            <div class="tile-body">
              <div class="value-panel">
                <div class="value" id="leaf">–</div>
              </div>
            </div>
          </div>
        </article>
        <article class="card metric-tile sensor-card" data-metric="vpd">
          <div class="hover-chart"><canvas class="hover-canvas" data-metric="vpd" width="320" height="140"></canvas></div>
          <div class="tile-content">
            <div class="card-header tile-header">
              <div class="tile-title"><div>VPD (kPa)</div><span class="status-dot" id="vpdDot"></span></div>
            </div>
            <div class="tile-body">
              <div class="value-panel">
                <div class="value" id="vpd">–</div>
                <div id="vpdStatus" class="tile-subtext"></div>
              </div>
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
            </div>
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
          <h3 style="margin:0;">Verlauf</h3>
          <label for="chartMetricSelect" style="display:flex; align-items:center; gap:6px; font-size:0.9rem;">
            Metrik
            <select id="chartMetricSelect" style="width:auto; min-width:150px;">
              <option value="temp">Temperatur</option>
              <option value="humidity">Luftfeuchte</option>
              <option value="vpd">VPD</option>
              <option value="co2">CO₂</option>
              <option value="lux">Lux</option>
            </select>
          </label>
        </div>
        <div class="row" style="justify-content:space-between; align-items:center; gap:8px; flex-wrap:wrap; margin-bottom:6px;">
          <div id="mainChartLegend" class="chart-legend"></div>
          <label for="chartRangeSelect" style="display:flex; align-items:center; gap:6px; font-size:0.9rem;">
            Range
            <select id="chartRangeSelect" class="color-select">
              <option value="24h">24h</option>
              <option value="30d" class="cloud-range">1M</option>
              <option value="90d" class="cloud-range">3M</option>
              <option value="120d" class="cloud-range">4M</option>
            </select>
          </label>
          <label for="chartColorSelect" style="display:flex; align-items:center; gap:6px; font-size:0.9rem;">
            Farbe
            <select id="chartColorSelect" class="color-select"></select>
          </label>
        </div>
        <div id="cloudChartNotice" class="status err" style="display:none; align-items:center; gap:10px; margin-top:4px;">
          <span id="cloudChartNoticeText">Cloud offline — zeige lokale 24h</span>
          <button id="cloudRetry" class="ghost" style="width:auto; padding:6px 10px; margin:0;">Retry</button>
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
            <button id="toggleWifiForm" class="ghost" style="width:auto; margin-top:8px;">Mit neuem Netzwerk verbinden</button>
          </div>
          <div id="wifiReconnectPanel" class="hidden reconnect-panel">
            <div class="wifi-status">
              <span class="led pulse" style="background:#38bdf8;"></span>
              <strong>Verbinde...</strong>
            </div>
            <p id="wifiReconnectText" class="hover-hint">Suche growsensor.local</p>
            <div class="row" style="gap:6px;">
              <button id="reconnectOpen" class="ghost" style="width:auto;">growsensor.local öffnen</button>
              <button id="reconnectRetry" style="width:auto;">Erneut versuchen</button>
            </div>
            <p class="hover-hint" id="wifiReconnectFallback">Falls keine Verbindung: verbinde dich mit dem Setup-AP <strong>GrowSensor-Setup</strong>.</p>
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
            <input id="ip" placeholder="IP (optional)" />
            <input id="gw" placeholder="Gateway (optional)" />
            <input id="sn" placeholder="Subnet (optional)" />
          </div>
          <button id="saveWifi">Mit neuem Netzwerk verbinden</button>
          <button id="resetWifi" style="margin-top:8px;background:#ef4444;color:#fff;">Werkseinstellungen</button>
          <p id="wifiStatus" class="status" style="margin-top:8px;"></p>
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

      <div id="view-cloud" class="view">
        <section class="card">
          <h3 style="margin-top:0">Cloud (Nextcloud WebDAV, HTTP)</h3>
          <div class="cloud-grid">
            <div>
              <label style="display:flex;align-items:center;gap:8px;"><input type="checkbox" id="cloudEnabled" style="width:auto;"> Cloud Logging aktivieren</label>
              <div class="cloud-credentials" id="cloudCredentials">
                <div class="cloud-credentials-header">
                  <strong>Login-Daten</strong>
                  <button id="cloudCredentialsToggle" class="ghost cloud-credentials-toggle" style="width:auto;">▼</button>
                </div>
                <div class="cloud-credentials-summary" id="cloudCredentialsSummary">
                  <span id="cloudCredentialsSummaryText">Eingeloggt</span>
                  <button id="cloudEdit" class="ghost" style="width:auto;">Bearbeiten</button>
                </div>
                <div class="cloud-credentials-body" id="cloudCredentialsBody">
                  <label for="cloudUrl">WebDAV Base URL</label>
                  <input id="cloudUrl" placeholder="http://host/remote.php/dav/files/user/" />
                  <label for="cloudUser">Username</label>
                  <input id="cloudUser" placeholder="Nextcloud Username" />
                  <label for="cloudPass">App-Passwort / Token</label>
                  <input id="cloudPass" type="password" placeholder="App-Passwort" />
                  <label for="cloudRetention">Retention (Monate)</label>
                  <select id="cloudRetention">
                    <option value="1">1 Monat</option>
                    <option value="2">2 Monate</option>
                    <option value="3">3 Monate</option>
                    <option value="4">4 Monate</option>
                  </select>
                  <label style="display:flex;align-items:center;gap:8px;margin-top:10px;"><input type="checkbox" id="cloudPersist" style="width:auto;"> Login-Daten dauerhaft speichern</label>
                  <div class="row" style="margin-top:10px;">
                    <button id="cloudSave">Speichern</button>
                    <button id="cloudTest" class="ghost">Sende Test</button>
                  </div>
                  <button id="cloudForget" class="ghost" style="margin-top:6px;">Forget credentials</button>
                </div>
              </div>
              <div class="row" style="margin-top:10px;">
                <button id="cloudStart">Start Recording</button>
                <button id="cloudStop" class="ghost">Stop Recording</button>
              </div>
              <p id="cloudStatusMsg" class="status" style="margin-top:8px;"></p>
            </div>
            <div>
              <p class="hover-hint" id="cloudPathHint" style="margin:4px 0;">Uploads landen in /GrowSensor/&lt;deviceId&gt;/</p>
              <div class="cloud-status">
                <div class="cloud-pill"><span>Cloud enabled</span><strong id="cloudEnabledState">–</strong></div>
                <div class="cloud-pill"><span>Runtime</span><strong id="cloudRuntimeState">–</strong></div>
                <div class="cloud-pill"><span>Recording</span><strong id="cloudRecordingState">–</strong></div>
                <div class="cloud-pill"><span>Connected</span><strong id="cloudConnected">–</strong></div>
                <div class="cloud-pill"><span>Queue</span><strong id="cloudQueueSize">0</strong></div>
                <div class="cloud-pill"><span>Failures</span><strong id="cloudFailures">0</strong></div>
              </div>
              <div class="cloud-status" style="margin-top:8px;">
                <div class="cloud-pill"><span>Last upload</span><strong id="cloudLastUpload">–</strong></div>
                <div class="cloud-pill"><span>Last test</span><strong id="cloudLastTest">–</strong></div>
                <div class="cloud-pill" style="grid-column: span 2;"><span>Last path</span><strong id="cloudUploadPath" style="text-align:right;">–</strong></div>
                <div class="cloud-pill" style="grid-column: span 2;"><span>State</span><strong id="cloudStateReason" style="text-align:right;">–</strong></div>
                <div class="cloud-pill" style="grid-column: span 2;"><span>Last error</span><strong id="cloudLastError" style="text-align:right;">–</strong></div>
              </div>
              <p class="hover-hint" style="margin-top:8px;">Empfohlen: Nextcloud App-Passwort nutzen, nicht das Hauptpasswort.</p>
            </div>
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
    <footer>Growcontroller v0.3.3 (experimental) • Sensorgehäuse v0.3</footer>

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
          <button id="tabLast30d" class="cloud-range" style="display:none;">1M</button>
          <button id="tabLast90d" class="cloud-range" style="display:none;">3M</button>
          <button id="tabLast120d" class="cloud-range" style="display:none;">4M</button>
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
    <div id="chartTooltip" class="chart-tooltip hidden" role="tooltip"></div>

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

      function refreshHover(metric) {
        drawHover(metric);
        requestAnimationFrame(() => drawHover(metric));
      }

      function setupHoverHandlers() {
        document.querySelectorAll('.metric-tile').forEach(tile => {
          const metric = tile.dataset.metric;
          if (!metric) return;
          tile.addEventListener('mouseenter', () => refreshHover(metric));
        });
      }

      setupHoverHandlers();

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
      const chartRangeSelect = getEl('chartRangeSelect');
      const chartColorSelect = getEl('chartColorSelect');
      const detailColorSelect = getEl('detailColorSelect');
      const mainChartLegend = getEl('mainChartLegend');
      const detailLegend = getEl('detailLegend');
      const timeBadge = getEl('timeBadge');
      const localTimeText = getEl('localTimeText');
      const cloudEnabledToggle = getEl('cloudEnabled');
      const cloudUrlInput = getEl('cloudUrl');
      const cloudUserInput = getEl('cloudUser');
      const cloudPassInput = getEl('cloudPass');
      const cloudRetentionSelect = getEl('cloudRetention');
      const cloudPersistToggle = getEl('cloudPersist');
      const cloudCredentials = getEl('cloudCredentials');
      const cloudCredentialsBody = getEl('cloudCredentialsBody');
      const cloudCredentialsToggle = getEl('cloudCredentialsToggle');
      const cloudCredentialsSummary = getEl('cloudCredentialsSummary');
      const cloudCredentialsSummaryText = getEl('cloudCredentialsSummaryText');
      const cloudEditBtn = getEl('cloudEdit');
      const cloudForgetBtn = getEl('cloudForget');
      const cloudStatusMsg = getEl('cloudStatusMsg');
      const cloudEnabledState = getEl('cloudEnabledState');
      const cloudRuntimeState = getEl('cloudRuntimeState');
      const cloudRecordingState = getEl('cloudRecordingState');
      const cloudConnected = getEl('cloudConnected');
      const cloudQueueSize = getEl('cloudQueueSize');
      const cloudFailures = getEl('cloudFailures');
      const cloudLastUpload = getEl('cloudLastUpload');
      const cloudLastTest = getEl('cloudLastTest');
      const cloudUploadPath = getEl('cloudUploadPath');
      const cloudStateReason = getEl('cloudStateReason');
      const cloudLastError = getEl('cloudLastError');
      const cloudPathHint = getEl('cloudPathHint');
      const cloudChartNotice = getEl('cloudChartNotice');
      const cloudChartNoticeText = getEl('cloudChartNoticeText');
      const cloudRetryBtn = getEl('cloudRetry');
      let detailMetric = null;
      let detailMode = 'live';
      let detailVpdView = 'heatmap';
      let clickDebug = false;
      let lastVpdTargets = { low: null, high: null };
      let cloudCredentialsOverride = false;
      let wifiFormOpen = false;
      let reconnectTimer = null;
      let reconnectDeadline = 0;
      let reconnectActive = false;
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
      const metricDataState = {};
      metrics.forEach(m => metricDataState[m] = { ever:false, last:0 });
      const cloudState = { enabled:false, runtime:false, recording:false, connected:false, persist:true, lastUpload:0, lastPing:0, lastFailure:0, lastTest:0, queue:0, failures:0, lastError:'', lastPath:'', lastReason:'', storageMode:'local_only', deviceFolder:'' };
      const TREND_CONFIG = {
        temp: { window: 8, minDelta: 0.08, strong: 0.35, decimals: 1 },
        humidity: { window: 8, minDelta: 0.4, strong: 1.5, decimals: 1 },
        co2: { window: 8, minDelta: 15, strong: 50, decimals: 0 },
        vpd: { window: 8, minDelta: 0.01, strong: 0.05, decimals: 3 }
      };
      const TREND_MEMORY = {};
      const chartLayouts = {};
      const hoverState = { main: null, detail: null, tiles: {} };
      const tooltipEl = getEl('chartTooltip');
      function markMetricSample(metric, ts) {
        if (!metricDataState[metric]) metricDataState[metric] = { ever:false, last:0 };
        if (typeof ts === 'number' && !Number.isNaN(ts)) {
          metricDataState[metric].last = Math.max(metricDataState[metric].last, ts);
        }
        if (metricDataState[metric].last > 0) metricDataState[metric].ever = true;
      }
      const clamp = (val, min, max) => Math.min(Math.max(val, min), max);
      function hideTooltip() {
        if (tooltipEl) tooltipEl.classList.add('hidden');
      }
      function showTooltip({ label, valueText, timeText, color, containerRect, anchorX, anchorY }) {
        if (!tooltipEl) return;
        tooltipEl.innerHTML = `<div class="label">${label}</div><strong>${valueText}</strong><div class="label">${timeText}</div>`;
        tooltipEl.style.borderColor = color || '#22d3ee';
        tooltipEl.style.color = color || '#e2e8f0';
        tooltipEl.classList.remove('hidden');
        const rect = containerRect || document.body.getBoundingClientRect();
        const pad = 8;
        const offset = 16;
        const { width: tipW, height: tipH } = tooltipEl.getBoundingClientRect();
        const left = clamp(anchorX + offset, rect.left + pad, rect.right - tipW - pad);
        const top = clamp(anchorY - tipH / 2, rect.top + pad, rect.bottom - tipH - pad);
        tooltipEl.style.left = `${left}px`;
        tooltipEl.style.top = `${top}px`;
      }
      function metricHasData(metric) {
        const state = metricDataState[metric];
        return !!(state && state.ever);
      }
      function hasRecentSample(metric, windowMs = 60000) {
        const state = metricDataState[metric];
        if (!state || !state.last) return false;
        const now = getApproxEpochMs() ?? Date.now();
        return now - state.last <= windowMs;
      }
      function hasDataInRange(metric, mode) {
        let data = [];
        if (mode === '30d' || mode === '90d' || mode === '120d') {
          const key = `${metric}-${mode}`;
          data = detailCache[key] || [];
        } else {
          data = getSeriesData(metric, mode);
        }
        return data.some(p => p && p.v !== null && typeof p.t === 'number');
      }
      function refreshMetricOptions() {
        if (!chartMetricSelect) return;
        const available = metrics.filter(metricHasData);
        chartMetricSelect.querySelectorAll('option').forEach(opt => {
          if (!opt.value) return;
          const show = available.includes(opt.value);
          opt.disabled = !show;
          opt.hidden = !show;
        });
        if (available.length > 0 && !available.includes(chartMetricSelect.value)) {
          chartMetricSelect.value = available[0];
          chartMetric = chartMetricSelect.value;
        }
      }
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
      let chartRange = chartRangeSelect && chartRangeSelect.value ? chartRangeSelect.value : '24h';

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
        let animClass = 'dot-pulse-idle';
        if (enabled && present) {
          if (ok) {
            color = '#34d399';
            animClass = 'dot-pulse-fast';
          } else {
            color = '#fbbf24';
            animClass = 'dot-pulse-slow';
          }
        } else if (enabled && !present) {
          color = '#ef4444';
          animClass = '';
        } else {
          color = '#ef4444';
          animClass = '';
        }
        el.style.background = color;
        el.style.boxShadow = `0 0 0 3px ${color}33`;
        el.classList.remove('dot-pulse-fast', 'dot-pulse-slow', 'dot-pulse-idle');
        if (animClass) el.classList.add(animClass);
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

      function vpdDistance(val) {
        if (val === null || Number.isNaN(val)) return null;
        if (lastVpdTargets.low === null || lastVpdTargets.high === null) return null;
        if (val < lastVpdTargets.low) return lastVpdTargets.low - val;
        if (val > lastVpdTargets.high) return val - lastVpdTargets.high;
        return 0;
      }

      function metricTrend(metric) {
        const cfg = TREND_CONFIG[metric] || { window: 6, minDelta: 0.1, strong: 0.25, decimals: 1 };
        const data = getSeriesData(metric, 'live').filter(p => p && typeof p.t === 'number' && typeof p.v === 'number' && !Number.isNaN(p.v));
        if (data.length < 2) return { dir:'flat', latest: null, delta: 0 };
        const windowed = data.slice(-cfg.window);
        if (windowed.length < 2) return { dir:'flat', latest: windowed[windowed.length - 1]?.v ?? null, delta: 0 };
        const latest = windowed[windowed.length - 1].v;
        const first = windowed[0].v;
        const diff = latest - first;
        const mag = Math.abs(diff);
        const hysteresis = cfg.minDelta * 0.4;
        const prevDir = TREND_MEMORY[metric] || 'flat';
        let dir = 'flat';
        if (mag > cfg.minDelta) dir = diff > 0 ? 'up' : 'down';
        else if (mag > cfg.minDelta - hysteresis && (prevDir === 'up' || prevDir === 'down')) dir = prevDir;
        TREND_MEMORY[metric] = dir;
        let targetDir = null;
        if (metric === 'vpd') {
          const distPrev = vpdDistance(first);
          const distNow = vpdDistance(latest);
          if (distPrev !== null && distNow !== null) {
            const delta = distPrev - distNow;
            if (Math.abs(delta) < cfg.minDelta) targetDir = 'neutral';
            else targetDir = delta > 0 ? 'toward' : 'away';
          }
        }
        const strength = mag >= cfg.strong ? 'strong' : (mag >= cfg.minDelta ? 'soft' : 'flat');
        return { dir, latest, delta: diff, strength, targetDir };
      }

      function trendArrow(dir) {
        if (dir === 'up') return '↑';
        if (dir === 'down') return '↓';
        return '';
      }

      function renderKpi(metric, value, trend) {
        const formatValue = (val, decimals = 1) => (typeof val === 'number' && !Number.isNaN(val)) ? val.toFixed(decimals) : '–';
        const cfg = TREND_CONFIG[metric] || { decimals: 1 };
        if (metric === 'temp') setText('kpiTemp', formatValue(value, cfg.decimals));
        if (metric === 'humidity') setText('kpiHumidity', formatValue(value, cfg.decimals));
        if (metric === 'co2') setText('kpiCo2', formatValue(value, cfg.decimals));
        if (metric === 'vpd') setText('kpiVpd', formatValue(value, cfg.decimals));
        const trendElMap = {
          temp: getEl('kpiTempTrend'),
          humidity: getEl('kpiHumidityTrend'),
          co2: getEl('kpiCo2Trend'),
          vpd: getEl('kpiVpdTrend')
        };
        const trendEl = trendElMap[metric];
        if (!trendEl) return;
        const dir = trend?.dir || 'flat';
        let cls = 'kpi-trend trend-flat';
        if (metric === 'vpd') {
          if (trend?.targetDir === 'toward') cls = 'kpi-trend trend-good';
          else if (trend?.targetDir === 'away') cls = 'kpi-trend trend-bad';
          else if (dir !== 'flat') cls = 'kpi-trend trend-warn';
        } else {
          if (dir !== 'flat') {
            cls = trend?.strength === 'strong' ? 'kpi-trend trend-good' : 'kpi-trend trend-warn';
          }
        }
        trendEl.className = cls;
        const arrow = trendArrow(dir);
        trendEl.textContent = arrow || '–';
      }

      function updateKpiBar(telemetry = lastTelemetryPayload) {
        const val = (v) => (typeof v === 'number' && !Number.isNaN(v)) ? v : null;
        renderKpi('temp', val(telemetry.temp), metricTrend('temp'));
        renderKpi('humidity', val(telemetry.humidity), metricTrend('humidity'));
        renderKpi('co2', val(telemetry.co2), metricTrend('co2'));
        renderKpi('vpd', val(telemetry.vpd), metricTrend('vpd'));
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

      function setVpdTargets(low, high, opts = {}) {
        const next = {
          low: (typeof low === 'number' && !Number.isNaN(low)) ? low : null,
          high: (typeof high === 'number' && !Number.isNaN(high)) ? high : null
        };
        const changed = next.low !== lastVpdTargets.low || next.high !== lastVpdTargets.high;
        lastVpdTargets = next;
        if (changed && opts.silent !== true) refreshVpdHeatmaps(true);
        return changed;
      }

      function refreshVpdHeatmaps(forceDetail = false, opts = {}) {
        if (!opts.skipTile) renderVpdTile(lastTelemetryPayload);
        if (detailMetric === 'vpd' && detailVpdView === 'heatmap' && (forceDetail || detailMetric)) {
          renderDetail();
        }
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
        if (mode === '30d' || mode === '90d' || mode === '120d') return mode;
        return 'live';
      }

      function formatDayKey(date) {
        const y = date.getFullYear();
        const m = String(date.getMonth() + 1).padStart(2, '0');
        const d = String(date.getDate()).padStart(2, '0');
        return `${y}-${m}-${d}`;
      }

      function dayRangeForDays(days) {
        const end = new Date();
        const start = new Date();
        start.setDate(end.getDate() - (days - 1));
        return { from: formatDayKey(start), to: formatDayKey(end) };
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
        if (entry !== null) markMetricSample(metric, nowTs);
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
        const latestValid = sanitized.filter(p => p && p.v !== null).pop();
        if (latestValid && typeof latestValid.t === 'number') markMetricSample(metric, latestValid.t);
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
        const desiredBase = targetTicks ? Math.max(2, targetTicks) : Math.max(minTicks, Math.round(plotWidth / 80));
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
        const allowed = Math.max(2, Math.min(maxTicks, Math.max(minTicks, Math.min(desired, maxCountBySpace))));
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
        const layout = {
          domainStart,
          domainEnd,
          paddingLeft,
          paddingRight,
          paddingTop,
          paddingBottom,
          plotWidth,
          plotHeight,
          yRange: range,
          series: prepared,
          mode: normalized
        };

        const xForTs = (ts) => paddingLeft + ((ts - domainStart) / timeSpan) * plotWidth;
        const yForVal = (val) => plotBottom - ((val - range.min) / range.span) * plotHeight;

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
        if (opts.hover && typeof opts.hover.ts === 'number') {
          const hx = xForTs(opts.hover.ts);
          ctxDraw.save();
          ctxDraw.setLineDash([4,4]);
          ctxDraw.strokeStyle = opts.hover.color || '#e5e7eb';
          ctxDraw.beginPath();
          ctxDraw.moveTo(hx, paddingTop);
          ctxDraw.lineTo(hx, plotBottom);
          ctxDraw.stroke();
          ctxDraw.setLineDash([]);
          if (opts.hover.point && typeof opts.hover.point.v === 'number') {
            const hy = yForVal(opts.hover.point.v);
            ctxDraw.fillStyle = opts.hover.color || '#22d3ee';
            ctxDraw.beginPath();
            ctxDraw.arc(hx, hy, 4, 0, Math.PI * 2);
            ctxDraw.fill();
            ctxDraw.strokeStyle = '#0f172a';
            ctxDraw.lineWidth = 1.5;
            ctxDraw.stroke();
          }
          ctxDraw.restore();
        }
        layout.xForTs = xForTs;
        layout.yForVal = yForVal;
        return { ok: true, layout };
      }

      function nearestPointForLayout(layout, ts) {
        if (!layout || !Array.isArray(layout.series)) return null;
        let best = null;
        let bestDist = Infinity;
        layout.series.forEach(series => {
          (series.points || []).forEach(pt => {
            if (!pt || pt.v === null || Number.isNaN(pt.v)) return;
            const dist = Math.abs(pt.t - ts);
            if (dist < bestDist) {
              bestDist = dist;
              best = { ...pt, seriesId: series.id, color: series.color || '#22d3ee' };
            }
          });
        });
        return best;
      }

      function handleHoverInteraction(ev, layoutKey, metricResolver, modeResolver, renderFn) {
        const layout = chartLayouts[layoutKey];
        if (!layout) return;
        const metric = metricResolver();
        if (!metric || layout.metric !== metric) return;
        const canvas = ev.currentTarget;
        const rect = canvas.getBoundingClientRect();
        const clientX = ev.touches && ev.touches.length ? ev.touches[0].clientX : ev.clientX;
        const clientY = ev.touches && ev.touches.length ? ev.touches[0].clientY : ev.clientY;
        const x = clientX - rect.left;
        const y = clientY - rect.top;
        if (x < layout.paddingLeft || x > layout.paddingLeft + layout.plotWidth || y < layout.paddingTop || y > layout.paddingTop + layout.plotHeight) {
          if (layoutKey === 'main') hoverState.main = null;
          else if (layoutKey === 'detail') hoverState.detail = null;
          else hoverState.tiles[metric] = null;
          hideTooltip();
          renderFn();
          return;
        }
        const span = layout.domainEnd - layout.domainStart;
        const ts = layout.domainStart + clamp((x - layout.paddingLeft) / Math.max(1, layout.plotWidth), 0, 1) * span;
        const nearest = nearestPointForLayout(layout, ts);
        if (!nearest) {
          if (layoutKey === 'main') hoverState.main = null;
          else if (layoutKey === 'detail') hoverState.detail = null;
          else hoverState.tiles[metric] = null;
          hideTooltip();
          renderFn();
          return;
        }
        const meta = layout.meta || METRIC_META[metric] || { unit:'', decimals:1, label: metric.toUpperCase() };
        const statePayload = { ts: nearest.t, point: nearest, color: nearest.color, metric, mode: modeResolver() };
        if (layoutKey === 'main') hoverState.main = statePayload;
        else if (layoutKey === 'detail') hoverState.detail = statePayload;
        else hoverState.tiles[metric] = statePayload;
        renderFn();
        const label = meta.label || metric.toUpperCase();
        const valueText = (typeof nearest.v === 'number' && !Number.isNaN(nearest.v)) ? `${nearest.v.toFixed(meta.decimals ?? 1)} ${meta.unit || ''}`.trim() : '–';
        const timeText = formatTimeLabel(nearest.t, layout.mode, layout.domainStart, layout.domainEnd, historyStore[metric]?.synced ?? clockState.synced, clockState.timezone, layout.mode === '24h', dayStamp(layout.domainStart, clockState.timezone));
        const containerRect = (canvas.parentElement || canvas).getBoundingClientRect();
        showTooltip({ label, valueText, timeText, color: nearest.color, containerRect, anchorX: clientX, anchorY: clientY });
      }

      function attachTooltipHandlers(canvas, layoutKey, metricResolver, modeResolver, renderFn) {
        if (!canvas) return;
        const moveHandler = (ev) => handleHoverInteraction(ev, layoutKey, metricResolver, modeResolver, renderFn);
        const leaveHandler = () => {
          const metric = metricResolver();
          if (layoutKey === 'main') hoverState.main = null;
          else if (layoutKey === 'detail') hoverState.detail = null;
          else if (metric) hoverState.tiles[metric] = null;
          hideTooltip();
          renderFn();
        };
        canvas.addEventListener('mousemove', moveHandler);
        canvas.addEventListener('touchmove', moveHandler, { passive: true });
        canvas.addEventListener('mouseleave', leaveHandler);
        canvas.addEventListener('touchend', leaveHandler);
      }

      async function drawChart() {
        if (!chartCanvas || !ctx) { pushError('Missing DOM element: chart'); return; }
        const colorSelectWrapper = chartColorSelect ? chartColorSelect.closest('label') : null;
        if (colorSelectWrapper) colorSelectWrapper.style.display = 'flex';
        let ok = false;
        const available = metrics.filter(m => metricHasData(m));
        if (!available.includes(chartMetric) && available.length) {
          chartMetric = available[0];
          if (chartMetricSelect) chartMetricSelect.value = chartMetric;
        }
        const meta = METRIC_META[chartMetric] || { unit:'', decimals:1 };
        const dayMs = 24 * 60 * 60 * 1000;
        let data = [];
        if (chartRange === '30d' || chartRange === '90d' || chartRange === '120d') {
          data = await loadDetailHistory(chartMetric, chartRange);
        } else {
          data = getSeriesData(chartMetric, chartRange);
        }
        const deviceId = deviceIdForMetric(chartMetric);
        const color = colorForMetricDevice(chartMetric, deviceId);
        const series = [{ id: deviceId, label: `GeräteID: ${deviceId}`, color, points: data }];
        const hover = (hoverState.main && hoverState.main.metric === chartMetric) ? hoverState.main : null;
        const res = drawLineChart(chartCanvas, ctx, data, chartRange, meta, { series, unitLabel: meta.unit, decimals: meta.decimals ?? 1, minXTicks:6, maxXTicks:10, yTicks:5, synced: historyStore[chartMetric]?.synced ?? clockState.synced, timezone: clockState.timezone, hover, windowMs: chartRange === '30d' ? dayMs*30 : chartRange === '90d' ? dayMs*90 : chartRange === '120d' ? dayMs*120 : DAY_MS });
        ok = !!res;
        if (ok && res.layout) {
          chartLayouts.main = { ...res.layout, meta, metric: chartMetric, unit: meta.unit, label: meta.label || chartMetric.toUpperCase() };
        } else {
          delete chartLayouts.main;
        }
        if (mainChartLegend) renderLegend(mainChartLegend, ok ? series : []);
        if (chartColorSelect) renderColorSelect(chartColorSelect, color);
        if (!ok) {
          delete chartLayouts.main;
          hoverState.main = null;
          hideTooltip();
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
        if (!metricHasData(metric) || !hasDataInRange(metric, '6h')) {
          resizeCanvas(canvas, ctxHover);
          ctxHover.clearRect(0,0,canvas.width, canvas.height);
          return;
        }
        const data = getSeriesData(metric, '6h');
        const meta = METRIC_META[metric] || { unit:'', decimals:1 };
        const deviceId = deviceIdForMetric(metric);
        const color = colorForMetricDevice(metric, deviceId);
        const hover = hoverState.tiles[metric] || null;
        const res = drawLineChart(canvas, ctxHover, data, '6h', meta, {
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
          anchorEndTs: getApproxEpochMs() ?? null,
          hover
        });
        const ok = !!res;
        if (ok && res.layout) {
          chartLayouts[`hover-${metric}`] = { ...res.layout, meta, metric, unit: meta.unit, label: meta.label || metric.toUpperCase() };
        } else {
          delete chartLayouts[`hover-${metric}`];
          hoverState.tiles[metric] = null;
          hideTooltip();
        }
        if (!ok) {
          resizeCanvas(canvas, ctxHover);
          ctxHover.clearRect(0,0,canvas.width, canvas.height);
        }
      }

      if (chartCanvas) attachTooltipHandlers(chartCanvas, 'main', () => chartMetric, () => '24h', drawChart);
      Object.entries(hoverCanvases).forEach(([metric, ctxHover]) => {
        if (ctxHover && ctxHover.canvas) attachTooltipHandlers(ctxHover.canvas, `hover-${metric}`, () => metric, () => '6h', () => drawHover(metric));
      });
      if (detailChartCanvas) attachTooltipHandlers(detailChartCanvas, 'detail', () => detailMetric, () => detailMode, renderDetail);

      let hoverResizeRaf = null;
      window.addEventListener('resize', () => {
        if (hoverResizeRaf) cancelAnimationFrame(hoverResizeRaf);
        hoverResizeRaf = requestAnimationFrame(() => {
          tileOrder.forEach(m => { if (tileIsExpanded(m)) drawHover(m); });
          hoverResizeRaf = null;
        });
      });

      function drawDetailChart(metric, mode, points, meta) {
        if (!detailCtx || !metric || !detailChartCanvas) return false;
        const deviceId = deviceIdForMetric(metric);
        const color = colorForMetricDevice(metric, deviceId);
        const dayMs = 24 * 60 * 60 * 1000;
        const tickBounds = mode === 'live' ? { minXTicks:5, maxXTicks:8, windowMs: LIVE_WINDOW_MS } :
          (mode === '6h' ? { minXTicks:6, maxXTicks:10, windowMs: SIX_H_MS } :
          (mode === '30d' ? { minXTicks:6, maxXTicks:10, windowMs: dayMs * 30 } :
          (mode === '90d' ? { minXTicks:6, maxXTicks:10, windowMs: dayMs * 90 } :
          (mode === '120d' ? { minXTicks:6, maxXTicks:10, windowMs: dayMs * 120 } :
            { minXTicks:6, maxXTicks:10, windowMs: DAY_MS }))));
        const hover = (hoverState.detail && hoverState.detail.metric === metric && hoverState.detail.mode === mode) ? hoverState.detail : null;
        const res = drawLineChart(detailChartCanvas, detailCtx, points, mode, meta, {
          series:[{ id: deviceId, label:`GeräteID: ${deviceId}`, color, points }],
          unitLabel: meta?.unit,
          decimals: meta?.decimals ?? 1,
          synced: historyStore[metric]?.synced ?? clockState.synced,
          timezone: clockState.timezone,
          minXTicks: tickBounds.minXTicks,
          maxXTicks: tickBounds.maxXTicks,
          yTicks:5,
          windowMs: tickBounds.windowMs,
          anchorEndTs: getApproxEpochMs() ?? null,
          hover
        });
        const ok = !!res;
        if (ok && res.layout) {
          chartLayouts.detail = { ...res.layout, meta, metric, unit: meta.unit, label: meta.label || metric.toUpperCase(), mode };
        } else {
          delete chartLayouts.detail;
          hoverState.detail = null;
          hideTooltip();
        }
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
        if ((normalized === '30d' || normalized === '90d' || normalized === '120d') && !(cloudState.enabled && cloudState.connected && cloudState.recording)) {
          detailCache[key] = [];
          return detailCache[key];
        }
        try {
          let mapped = [];
          if (normalized === '30d' || normalized === '90d' || normalized === '120d') {
            const days = normalized === '30d' ? 30 : (normalized === '90d' ? 90 : 120);
            const range = dayRangeForDays(days);
            const data = await fetchJson(`/api/cloud/daily?sensor=${metric}&from=${range.from}&to=${range.to}`);
            mapped = (data.points || []).map(p => {
              const ts = parseMs(p[0]);
              const valObj = p[1] || {};
              const val = typeof valObj.avg === 'number' ? valObj.avg : null;
              return { t: ts !== null ? ts : Date.now(), v: val };
            });
          } else {
            const data = await fetchJson(`/api/history?metric=${metric}&range=${normalized}`);
            mapped = (data.points || []).map(p => {
              const val = (p[1] === null || Number.isNaN(p[1])) ? null : parseFloat(p[1]);
              const ts = parseMs(p[0]);
              return { t: ts !== null ? ts : Date.now(), v: (typeof val === 'number' && !Number.isNaN(val)) ? val : null };
            });
            const syncedFlag = flag(data.time_synced);
            if (normalized === 'live' || normalized === '6h' || normalized === '24h') mergeHistory(metric, mapped, normalized, syncedFlag);
            clearErrors('API /api/history');
          }
          detailCache[key] = mapped;
        } catch (err) {
          console.warn('history error', err);
          const prefix = (normalized === '30d' || normalized === '90d' || normalized === '120d') ? 'API /api/cloud/daily failed' : 'API /api/history failed';
          pushError(`${prefix}: ${err.message}`);
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
        refreshMetricOptions();
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
        if (!ctxDraw || !canvas) return;
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
        hoverState.detail = null;
        hideTooltip();
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
          const lastMap = {
            lux: parseMs(data.lux_last),
            ppfd: parseMs(data.lux_last),
            co2: parseMs(data.co2_last),
            temp: parseMs(data.temp_last),
            humidity: parseMs(data.humidity_last),
            leaf: parseMs(data.leaf_last),
            vpd: parseMs(data.vpd_last)
          };
          metrics.forEach(metric => {
            const val = data[metric];
            const valid = typeof val === 'number' && !Number.isNaN(val) && (metric !== 'co2' || val > 0);
            const hint = lastMap[metric];
            if (hint !== null) markMetricSample(metric, hint);
            if (valid) markMetricSample(metric, tsForSample);
            if (flag(data[`${metric}_ever`])) markMetricSample(metric, hint ?? tsForSample ?? Date.now());
          });
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
          const targetsChanged = setVpdTargets(data.vpd_low, data.vpd_high, { silent: true });
          applyDeviceIdsFromTelemetry(data);
          updateTileHeaderStates(data);
          metrics.forEach(m => recordMetric(m, data[m], tsForSample, synced));
          updateKpiBar(data);
          drawChart();
          updateAverages();
          tileOrder.forEach(m => { if (tileIsExpanded(m)) drawHover(m); });
          renderVpdTile(data);
          renderDetail();
          if (targetsChanged && detailMetric === 'vpd' && detailVpdView === 'heatmap') {
            requestAnimationFrame(() => refreshVpdHeatmaps(true, { skipTile: true }));
          }
          applyWifiState(data);
          refreshMetricOptions();
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
        await authedFetch('/api/restart', { method:'POST' });
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

      function stopReconnectFlow(message = '') {
        reconnectActive = false;
        if (reconnectTimer) clearTimeout(reconnectTimer);
        reconnectTimer = null;
        reconnectDeadline = 0;
        if (message) setText('wifiReconnectText', message);
        setDisplay('wifiReconnectPanel', false);
      }

      function startReconnectFlow() {
        if (reconnectTimer) clearTimeout(reconnectTimer);
        reconnectTimer = null;
        reconnectActive = true;
        reconnectDeadline = Date.now() + 60000;
        setDisplay('wifiReconnectPanel', true, 'block');
        setText('wifiReconnectText', 'Suche growsensor.local ...');
        const poll = async () => {
          if (!reconnectActive) return;
          if (reconnectDeadline && Date.now() > reconnectDeadline) {
            setText('wifiReconnectText', 'Keine Antwort. Bitte Verbindung prüfen oder Setup-AP nutzen.');
            reconnectActive = false;
            return;
          }
          try {
            const res = await fetch('http://growsensor.local/api/ping', { method:'GET', mode:'cors' });
            if (res.ok) {
              window.location = 'http://growsensor.local/';
              return;
            }
          } catch (err) {
            // ignore, retry
          }
          reconnectTimer = setTimeout(poll, 1500);
        };
        poll();
      }

      const reconnectOpenBtn = getEl('reconnectOpen');
      if (reconnectOpenBtn) reconnectOpenBtn.addEventListener('click', () => {
        window.location = 'http://growsensor.local/';
      });
      const reconnectRetryBtn = getEl('reconnectRetry');
      if (reconnectRetryBtn) reconnectRetryBtn.addEventListener('click', () => {
        startReconnectFlow();
      });

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
        const ssid = getEl('ssid')?.value || '';
        const pass = getEl('pass')?.value || '';
        const staticIp = getEl('staticIpToggle')?.checked || false;
        const ip = getEl('ip')?.value || '';
        const gw = getEl('gw')?.value || '';
        const sn = getEl('sn')?.value || '';
        if (wifiStatus) { wifiStatus.textContent = 'Verbinde...'; wifiStatus.className = 'status'; }
        startReconnectFlow();
        try {
          const res = await authedFetch('/api/wifi', { method: 'POST', headers: {'Content-Type':'application/x-www-form-urlencoded'}, body: `ssid=${encodeURIComponent(ssid)}&pass=${encodeURIComponent(pass)}&static=${staticIp ? 1 : 0}&ip=${encodeURIComponent(ip)}&gw=${encodeURIComponent(gw)}&sn=${encodeURIComponent(sn)}` });
          const text = await res.text().catch(() => '');
          if (res.ok) {
            if (wifiStatus) { wifiStatus.textContent = 'Verbinde mit neuem Netzwerk...'; wifiStatus.className = 'status'; }
          } else {
            if (wifiStatus) { wifiStatus.textContent = 'Falsches Passwort'; wifiStatus.className = 'status err'; }
            stopReconnectFlow('Passwort prüfen');
            pushError(`API /api/wifi failed: ${res.status} ${text}`);
          }
        } catch (err) {
          pushError(`API /api/wifi failed: ${err.message}`);
          if (wifiStatus) { wifiStatus.textContent = 'Speichern fehlgeschlagen'; wifiStatus.className = 'status err'; }
          stopReconnectFlow('Verbindung fehlgeschlagen');
        }
      });

      const resetWifiBtn = getEl('resetWifi');
      if (resetWifiBtn) resetWifiBtn.addEventListener('click', async () => {
        const wifiStatus = getEl('wifiStatus');
        if (wifiStatus) wifiStatus.textContent = 'Werkseinstellungen...';
        const confirmation = prompt('Werkseinstellungen durchführen? Tippe RESET zum Bestätigen.', '');
        if ((confirmation || '').trim().toUpperCase() !== 'RESET') { if (wifiStatus) wifiStatus.textContent = 'Abgebrochen'; return; }
        const body = new URLSearchParams();
        body.set('confirm', 'RESET');
        await authedFetch('/api/factory-reset', { method: 'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body: body.toString() });
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
        hoverState.detail = null;
        hideTooltip();
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
        hoverState.detail = null;
        hideTooltip();
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
        hoverState.detail = null;
        hideTooltip();
        tabLast24hBtn.classList.add('active');
        const tabLiveEl = getEl('tabLive');
        const tabLast6h = getEl('tabLast6h');
        if (tabLiveEl) tabLiveEl.classList.remove('active');
        if (tabLast6h) tabLast6h.classList.remove('active');
        renderDetail();
      });
      const tabLast30dBtn = getEl('tabLast30d');
      if (tabLast30dBtn) tabLast30dBtn.addEventListener('click', () => {
        detailMode = '30d';
        hoverState.detail = null;
        hideTooltip();
        tabLast30dBtn.classList.add('active');
        ['tabLive','tabLast6h','tabLast24h','tabLast90d','tabLast120d'].forEach(id => { const el = getEl(id); if (el && el !== tabLast30dBtn) el.classList.remove('active'); });
        renderDetail();
      });
      const tabLast90dBtn = getEl('tabLast90d');
      if (tabLast90dBtn) tabLast90dBtn.addEventListener('click', () => {
        detailMode = '90d';
        hoverState.detail = null;
        hideTooltip();
        tabLast90dBtn.classList.add('active');
        ['tabLive','tabLast6h','tabLast24h','tabLast30d','tabLast120d'].forEach(id => { const el = getEl(id); if (el && el !== tabLast90dBtn) el.classList.remove('active'); });
        renderDetail();
      });
      const tabLast120dBtn = getEl('tabLast120d');
      if (tabLast120dBtn) tabLast120dBtn.addEventListener('click', () => {
        detailMode = '120d';
        hoverState.detail = null;
        hideTooltip();
        tabLast120dBtn.classList.add('active');
        ['tabLive','tabLast6h','tabLast24h','tabLast30d','tabLast90d'].forEach(id => { const el = getEl(id); if (el && el !== tabLast120dBtn) el.classList.remove('active'); });
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
        hoverState.main = null;
        Object.keys(hoverState.tiles).forEach(k => hoverState.tiles[k] = null);
        hideTooltip();
      }
      const navDashboard = getEl('navDashboard');
      if (navDashboard) navDashboard.addEventListener('click', () => switchView('view-dashboard'));
      const navSensors = getEl('navSensors');
      if (navSensors) navSensors.addEventListener('click', () => switchView('view-sensors'));
      const navCloud = getEl('navCloud');
      if (navCloud) navCloud.addEventListener('click', () => switchView('view-cloud'));
      if (chartMetricSelect) {
        chartMetricSelect.addEventListener('change', () => {
          chartMetric = chartMetricSelect.value || 'temp';
          hoverState.main = null;
          hideTooltip();
          drawChart();
        });
      }
      if (chartRangeSelect) {
        chartRangeSelect.addEventListener('change', () => {
          chartRange = chartRangeSelect.value || '24h';
          hoverState.main = null;
          hideTooltip();
          drawChart();
        });
      }
      if (chartColorSelect) {
        chartColorSelect.addEventListener('change', () => {
          if (!chartMetric) return;
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
        const connecting = flag(data?.connecting);
        const showConnected = connected && !ap;
        const form = getEl('wifiForm');
        const block = getEl('wifiConnectedBlock');
        const reconnectPanel = getEl('wifiReconnectPanel');
        if (connecting) {
          if (!reconnectActive) startReconnectFlow();
        } else {
          stopReconnectFlow();
        }
        if (reconnectPanel) reconnectPanel.classList.toggle('hidden', !connecting);
        if (block) {
          block.classList.toggle('hidden', !showConnected || connecting);
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
          const showForm = wifiFormOpen || (!showConnected && !connecting);
          form.classList.toggle('hidden', !showForm);
        }
        if (connecting) {
          const wifiStatus = getEl('wifiStatus');
          if (wifiStatus) { wifiStatus.textContent = 'Verbinde...'; wifiStatus.className = 'status'; }
        }
        const toggleBtn = getEl('toggleWifiForm');
        if (toggleBtn) toggleBtn.textContent = wifiFormOpen ? 'Abbrechen' : 'Mit neuem Netzwerk verbinden';
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

      function renderCloudStatus(data = {}) {
        cloudState.enabled = flag(data.enabled);
        cloudState.runtime = data.runtime_enabled === undefined ? cloudState.runtime : flag(data.runtime_enabled);
        cloudState.recording = data.recording === undefined ? cloudState.recording : flag(data.recording);
        cloudState.persist = data.persist_credentials === undefined ? cloudState.persist : flag(data.persist_credentials, true);
        cloudState.storageMode = typeof data.storage_mode === 'string' ? data.storage_mode : cloudState.storageMode;
        cloudState.connected = flag(data.connected);
        const lastUploadVal = parseMs(data.last_upload_ms);
        const lastPingVal = parseMs(data.last_ping_ms);
        const lastFailureVal = parseMs(data.last_failure_ms);
        const lastTestVal = parseMs(data.last_test_ms);
        cloudState.lastUpload = Number.isFinite(lastUploadVal) ? lastUploadVal : 0;
        cloudState.lastPing = Number.isFinite(lastPingVal) ? lastPingVal : 0;
        cloudState.lastFailure = Number.isFinite(lastFailureVal) ? lastFailureVal : 0;
        cloudState.lastTest = Number.isFinite(lastTestVal) ? lastTestVal : 0;
        cloudState.queue = typeof data.queue_size === 'number' ? data.queue_size : parseInt(data.queue_size || '0', 10);
        cloudState.failures = typeof data.failures === 'number' ? data.failures : parseInt(data.failures || '0', 10);
        cloudState.lastError = data.last_error || '';
        cloudState.lastPath = data.last_uploaded_path || cloudState.lastPath;
        cloudState.lastReason = data.last_state_reason || cloudState.lastReason;
        if (typeof data.device_folder === 'string' && data.device_folder) cloudState.deviceFolder = data.device_folder;
        if (cloudEnabledState) cloudEnabledState.textContent = cloudState.enabled ? 'yes' : 'no';
        if (cloudRuntimeState) cloudRuntimeState.textContent = cloudState.runtime ? 'yes' : 'no';
        if (cloudRecordingState) cloudRecordingState.textContent = cloudState.recording ? 'yes' : 'no';
        if (cloudConnected) cloudConnected.textContent = cloudState.connected ? 'yes' : 'no';
        if (cloudQueueSize) cloudQueueSize.textContent = String(cloudState.queue);
        if (cloudFailures) cloudFailures.textContent = String(cloudState.failures);
        if (cloudPathHint) cloudPathHint.textContent = cloudState.deviceFolder ? `Uploads landen in ${cloudState.deviceFolder}/` : 'Uploads landen in /GrowSensor/<deviceId>/';
        if (cloudLastError) cloudLastError.textContent = cloudState.lastError || '–';
        if (cloudLastUpload) {
          const stamp = cloudState.lastUpload || cloudState.lastTest || cloudState.lastPing;
          if (stamp) {
            const stampDate = new Date(stamp);
            const secs = Math.round((Date.now() - stamp) / 1000);
            const age = secs < 5 ? 'gerade eben' : `${secs}s ago`;
            cloudLastUpload.textContent = `${stampDate.toLocaleString()} (${age})`;
          } else {
            cloudLastUpload.textContent = '–';
          }
        }
        if (cloudLastTest) {
          if (cloudState.lastTest) {
            const secs = Math.round((Date.now() - cloudState.lastTest) / 1000);
            cloudLastTest.textContent = secs < 5 ? 'gerade eben' : `${secs}s ago`;
          } else {
            cloudLastTest.textContent = '–';
          }
        }
        if (cloudUploadPath) cloudUploadPath.textContent = cloudState.lastPath || '–';
        if (cloudStateReason) cloudStateReason.textContent = cloudState.lastReason || cloudState.storageMode || '–';
        const showLongRanges = cloudState.enabled && cloudState.connected && cloudState.recording;
        const longRangeBlocked = cloudState.enabled && (!cloudState.connected || !cloudState.recording);
        if (cloudChartNotice) {
          cloudChartNotice.style.display = longRangeBlocked ? 'flex' : 'none';
          if (longRangeBlocked) {
            const reason = !cloudState.connected ? 'Cloud offline' : 'Recording inaktiv';
            if (cloudChartNoticeText) cloudChartNoticeText.textContent = `${reason} — zeige lokale 24h`;
          }
        }
        if (cloudRetryBtn) cloudRetryBtn.disabled = !(cloudState.enabled && !cloudState.connected);
        if (cloudStatusMsg && longRangeBlocked) {
          const reason = !cloudState.connected ? 'Cloud offline' : 'Recording inaktiv';
          cloudStatusMsg.textContent = `${reason} — zeige lokale 24h`;
          cloudStatusMsg.className = 'status err';
        } else if (cloudStatusMsg && !longRangeBlocked && (cloudStatusMsg.textContent.includes('Cloud offline') || cloudStatusMsg.textContent.includes('Recording inaktiv'))) {
          cloudStatusMsg.textContent = '';
          cloudStatusMsg.className = 'status';
        }
        document.querySelectorAll('.cloud-range').forEach(btn => btn.style.display = showLongRanges ? 'inline-block' : 'none');
        if (!showLongRanges && chartRange !== '24h') {
          chartRange = '24h';
          if (chartRangeSelect) chartRangeSelect.value = '24h';
          drawChart();
        }
        if (!cloudState.connected || !cloudState.enabled || !cloudState.persist) cloudCredentialsOverride = false;
        const shouldCollapse = cloudState.connected && cloudState.persist && !cloudCredentialsOverride;
        if (cloudCredentials) {
          cloudCredentials.classList.toggle('collapsed', shouldCollapse);
        }
        const isCollapsed = cloudCredentials?.classList.contains('collapsed');
        if (cloudCredentialsBody) cloudCredentialsBody.style.display = isCollapsed ? 'none' : 'block';
        if (cloudCredentialsToggle) cloudCredentialsToggle.textContent = isCollapsed ? '▼' : '▲';
        if (cloudEditBtn) cloudEditBtn.style.display = isCollapsed ? 'inline-flex' : 'none';
        const showSummary = isCollapsed && cloudState.persist && cloudState.connected;
        if (cloudCredentialsSummary) cloudCredentialsSummary.style.display = showSummary ? 'flex' : 'none';
        if (cloudCredentialsSummaryText) {
          const user = data.username || cloudUserInput?.value || 'User';
          cloudCredentialsSummaryText.textContent = showSummary ? `Eingeloggt als ${user} (gespeichert)` : '';
        }
      }

      function renderCloudForm(data = {}) {
        if (cloudEnabledToggle) cloudEnabledToggle.checked = data.enabled === true || data.enabled === 1;
        if (cloudUrlInput && typeof data.base_url === 'string') cloudUrlInput.value = data.base_url;
        if (cloudUserInput && typeof data.username === 'string') cloudUserInput.value = data.username;
        if (cloudRetentionSelect && data.retention) cloudRetentionSelect.value = String(data.retention);
        if (cloudPassInput) cloudPassInput.value = '';
        if (cloudPersistToggle) cloudPersistToggle.checked = data.persist_credentials !== undefined ? flag(data.persist_credentials, true) : true;
      }

      async function fetchCloudStatus() {
        try {
          const res = await fetchJson('/api/cloud');
          renderCloudForm(res);
          renderCloudStatus(res);
        } catch (err) {
          if (cloudStatusMsg) { cloudStatusMsg.textContent = err.message || 'Cloud read failed'; cloudStatusMsg.className = 'status err'; }
        }
      }

      async function saveCloudConfig(enabledOnly = false) {
        const body = new URLSearchParams();
        body.set('action', 'save');
        const urlVal = cloudUrlInput?.value?.trim() || '';
        if (cloudUrlInput && !enabledOnly) body.set('base_url', urlVal);
        if (cloudUserInput && !enabledOnly) body.set('username', cloudUserInput.value || '');
        if (cloudPassInput && !enabledOnly && cloudPassInput.value) body.set('password', cloudPassInput.value);
        if (cloudPersistToggle && !enabledOnly) body.set('persist_credentials', cloudPersistToggle.checked ? '1' : '0');
        const enabledVal = cloudEnabledToggle?.checked ? '1' : '0';
        body.set('enabled', enabledVal);
        body.set('recording', cloudState.recording ? '1' : '0');
        if (cloudRetentionSelect && !enabledOnly) body.set('retention', cloudRetentionSelect.value || '1');
        try {
          await authedFetch('/api/cloud', { method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body });
          if (cloudStatusMsg) { cloudStatusMsg.textContent = 'Gespeichert'; cloudStatusMsg.className = 'status ok'; }
        } catch (err) {
          if (cloudStatusMsg) { cloudStatusMsg.textContent = 'Speichern fehlgeschlagen'; cloudStatusMsg.className = 'status err'; }
        }
        fetchCloudStatus();
      }

      async function testCloud() {
        if (cloudStatusMsg) { cloudStatusMsg.textContent = 'Sende Test...'; cloudStatusMsg.className = 'status'; }
        try {
          const res = await authedFetch('/api/cloud/test', { method:'POST' });
          const text = await res.text();
          let data = {};
          try { data = text ? JSON.parse(text) : {}; } catch (err) { data = {}; }
          const ok = res.ok && (data.ok === 1 || data.ok === true);
          const code = data.code ?? data.httpCode ?? data.http_code ?? res.status;
          const path = data.path || '';
          const errMsg = data.err || data.error || '';
          if (cloudStatusMsg) { cloudStatusMsg.textContent = ok ? `Test-Datei gesendet (${code}${path ? ` → ${path}` : ''})` : `Test fehlgeschlagen (${code || 'n/a'}${errMsg ? `: ${errMsg}` : ''})`; cloudStatusMsg.className = ok ? 'status ok' : 'status err'; }
          if (ok) renderCloudStatus({ connected:true, last_test_ms: Date.now(), last_uploaded_path: path });
        } catch (err) {
          if (cloudStatusMsg) { cloudStatusMsg.textContent = 'Test fehlgeschlagen'; cloudStatusMsg.className = 'status err'; }
        }
        fetchCloudStatus();
      }

      async function cloudAction(action) {
        const body = new URLSearchParams();
        body.set('action', action);
        try {
          await authedFetch('/api/cloud', { method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body });
          if (action === 'start' && cloudEnabledToggle) cloudEnabledToggle.checked = true;
          if (cloudStatusMsg) {
            cloudStatusMsg.textContent = action === 'start' ? 'Recording gestartet' : 'Recording gestoppt';
            cloudStatusMsg.className = 'status ok';
          }
          fetchCloudStatus();
        } catch (err) {
          if (cloudStatusMsg) { cloudStatusMsg.textContent = 'Aktion fehlgeschlagen'; cloudStatusMsg.className = 'status err'; }
        }
      }

      async function forgetCloudCredentials() {
        if (!confirm('Cloud-Credentials wirklich löschen?')) return;
        const body = new URLSearchParams();
        body.set('action', 'forget');
        try {
          await authedFetch('/api/cloud', { method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body });
          if (cloudUrlInput) cloudUrlInput.value = '';
          if (cloudUserInput) cloudUserInput.value = '';
          if (cloudPassInput) cloudPassInput.value = '';
          if (cloudPersistToggle) cloudPersistToggle.checked = false;
          cloudCredentialsOverride = true;
          if (cloudCredentials) cloudCredentials.classList.remove('collapsed');
          if (cloudCredentialsBody) cloudCredentialsBody.style.display = 'block';
          if (cloudEditBtn) cloudEditBtn.style.display = 'none';
          if (cloudCredentialsToggle) cloudCredentialsToggle.textContent = '▲';
          if (cloudStatusMsg) { cloudStatusMsg.textContent = 'Credentials gelöscht'; cloudStatusMsg.className = 'status ok'; }
        } catch (err) {
          if (cloudStatusMsg) { cloudStatusMsg.textContent = 'Löschen fehlgeschlagen'; cloudStatusMsg.className = 'status err'; }
        }
        fetchCloudStatus();
      }

      const toggleWifiFormBtn = getEl('toggleWifiForm');
      if (toggleWifiFormBtn) toggleWifiFormBtn.addEventListener('click', () => {
        wifiFormOpen = !wifiFormOpen;
        applyWifiState(lastTelemetryPayload);
      });

      const cloudSaveBtn = getEl('cloudSave');
      if (cloudSaveBtn) cloudSaveBtn.addEventListener('click', () => saveCloudConfig(false));
      const cloudTestBtn = getEl('cloudTest');
      if (cloudTestBtn) cloudTestBtn.addEventListener('click', () => testCloud());
      const cloudStartBtn = getEl('cloudStart');
      if (cloudStartBtn) cloudStartBtn.addEventListener('click', () => cloudAction('start'));
      const cloudStopBtn = getEl('cloudStop');
      if (cloudStopBtn) cloudStopBtn.addEventListener('click', () => cloudAction('stop'));
      if (cloudRetryBtn) cloudRetryBtn.addEventListener('click', () => testCloud());
      if (cloudEnabledToggle) cloudEnabledToggle.addEventListener('change', () => saveCloudConfig(true));
      if (cloudEditBtn) cloudEditBtn.addEventListener('click', () => {
        cloudCredentialsOverride = true;
        if (cloudCredentials) cloudCredentials.classList.remove('collapsed');
        if (cloudCredentialsBody) cloudCredentialsBody.style.display = 'block';
        if (cloudEditBtn) cloudEditBtn.style.display = 'none';
        if (cloudCredentialsToggle) cloudCredentialsToggle.textContent = '▲';
      });
      if (cloudCredentialsToggle) cloudCredentialsToggle.addEventListener('click', () => {
        const collapsed = cloudCredentials?.classList.contains('collapsed');
        cloudCredentialsOverride = true;
        if (collapsed) {
          cloudCredentials?.classList.remove('collapsed');
          if (cloudCredentialsBody) cloudCredentialsBody.style.display = 'block';
          if (cloudEditBtn) cloudEditBtn.style.display = 'none';
          cloudCredentialsToggle.textContent = '▲';
        } else {
          cloudCredentials?.classList.add('collapsed');
          if (cloudCredentialsBody) cloudCredentialsBody.style.display = 'none';
          if (cloudEditBtn) cloudEditBtn.style.display = 'inline-flex';
          cloudCredentialsToggle.textContent = '▼';
        }
      });
      if (cloudForgetBtn) cloudForgetBtn.addEventListener('click', () => forgetCloudCredentials());

      function setDevVisible() {
        document.querySelectorAll('.dev-only').forEach(el => el.disabled = !devMode);
        const partnerCard = getEl('partnerCard');
        const devStatus = getEl('devStatus');
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
        const hasLiveData = detailMode === 'live' ? hasRecentSample(detailMetric, 60000) : hasDataInRange(detailMetric, detailMode);
        const online = detailMode === 'live' ? metricIsHealthy(detailMetric, lastTelemetryPayload) : true;
        if (!metricHasData(detailMetric) || !hasLiveData || !online) {
          if (detailLegend) renderLegend(detailLegend, []);
          if (chartEmpty) chartEmpty.style.display = 'flex';
          delete chartLayouts.detail;
          hoverState.detail = null;
          hideTooltip();
          return;
        }
        if (showHeatmap) {
          delete chartLayouts.detail;
          hoverState.detail = null;
          hideTooltip();
          if (detailLegend) renderLegend(detailLegend, []);
          if (detailColorSelect) renderColorSelect(detailColorSelect, colorForMetricDevice(detailMetric, deviceIdForMetric(detailMetric)));
          const points = collectPaired(detailMode);
          drawVpdHeatmap(vpdHeatmapCtx, vpdHeatmapCanvas, detailMode, lastVpdTargets, getEl('chartModalCard'), points);
          requestAnimationFrame(() => drawVpdHeatmap(vpdHeatmapCtx, vpdHeatmapCanvas, detailMode, lastVpdTargets, getEl('chartModalCard'), points));
        } else {
          const historyPoints = await loadDetailHistory(detailMetric, detailMode);
          const ok = drawDetailChart(detailMetric, detailMode, historyPoints, meta);
          if (chartEmpty) chartEmpty.style.display = ok ? 'none' : 'flex';
        }
      }

      function renderVpdTile(data = lastTelemetryPayload || {}) {
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
        requestAnimationFrame(() => drawVpdHeatmap(vpdTileCtx, vpdTileCanvas, 'live', lastVpdTargets, getEl('vpdTileChart'), points));
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
      fetchCloudStatus();
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
