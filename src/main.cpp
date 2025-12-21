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
static constexpr unsigned long DEBUG_LOG_INTERVAL_MS = 5UL * 60UL * 1000UL;
static constexpr unsigned long DEBUG_LOG_MAX_CACHE_MS = 5UL * 60UL * 1000UL;
static constexpr size_t DEBUG_LOG_MAX_FILE_BYTES = 64000;
static const char *FIRMWARE_VERSION = "v0.3.5";
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
  unsigned long createdAtMs = 0;
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
uint32_t wifiReconnectAttempts = 0;

unsigned long lastDebugLogMs = 0;
unsigned long lastLoopDurationMs = 0;
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
String phaseId = "seedling";
float ppfdScale = 1.0f;

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

struct PhasePreset {
  const char *id;
  const char *label;
  LightChannel channel;
  const char *vpdStage;
  float ppfdScale;
};

const PhasePreset PHASE_PRESETS[] = {
    {"seedling", "Steckling/Sämling", LightChannel::FullSpectrum, "seedling", 1.0f},
    {"veg", "Vegitativ", LightChannel::FullSpectrum, "veg", 1.0f},
    {"bloom", "Blütephase", LightChannel::Bloom, "bloom", 1.0f},
    {"late_bloom", "Späteblüte", LightChannel::Bloom, "late_bloom", 1.0f},
};
const size_t PHASE_PRESET_COUNT = sizeof(PHASE_PRESETS) / sizeof(PHASE_PRESETS[0]);

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

const PhasePreset *phasePresetById(const String &id) {
  for (size_t i = 0; i < PHASE_PRESET_COUNT; i++) {
    if (id == PHASE_PRESETS[i].id) return &PHASE_PRESETS[i];
  }
  return nullptr;
}

String phaseFromState() {
  for (size_t i = 0; i < PHASE_PRESET_COUNT; i++) {
    const PhasePreset &preset = PHASE_PRESETS[i];
    if (preset.channel == channel && preset.vpdStage == vpdStageId &&
        fabs(preset.ppfdScale - ppfdScale) < 0.0001f) {
      return preset.id;
    }
  }
  return "custom";
}

void applyPhasePreset(const PhasePreset &preset, bool persist) {
  phaseId = preset.id;
  channel = preset.channel;
  vpdStageId = preset.vpdStage;
  ppfdScale = preset.ppfdScale;
  if (!persist) return;
  prefsSystem.begin("system", false);
  prefsSystem.putString("phase", phaseId);
  prefsSystem.putString("channel", lightChannelName());
  prefsSystem.putString("vpd_stage", vpdStageId);
  prefsSystem.putFloat("ppfd_scale", ppfdScale);
  prefsSystem.end();
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
  String logsPath = deviceRoot + "/logs";
  String logChunksPath = logsPath + "/chunks";
  bool ok = true;
  if (!ensureCollection(baseRoot, "root")) ok = false;
  if (!ensureCollection(deviceRoot, "device")) ok = false;
  if (!ensureCollection(dailyPath, "daily")) ok = false;
  if (!ensureCollection(metaPath, "meta")) ok = false;
  if (!ensureCollection(reportsPath, "reports")) ok = false;
  if (!ensureCollection(logsPath, "logs")) ok = false;
  if (!ensureCollection(logChunksPath, "logs/chunks")) ok = false;
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
      job.createdAtMs = millis();
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
  job.createdAtMs = millis();
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
  return luxToPPFD(1.0f, channel) * ppfdScale;
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

bool isValidLightChannelName(const String &value) {
  return value == "full_spectrum" || value == "white" || value == "bloom";
}

bool isValidVpdStageId(const String &id) {
  for (const auto &p : VPD_PROFILES) {
    if (id == p.id) return true;
  }
  return false;
}

bool parseJsonBody(JsonDocument &doc, String &errorOut) {
  if (!server.hasArg("plain")) {
    errorOut = "missing_body";
    return false;
  }
  String body = server.arg("plain");
  if (body.length() == 0) {
    errorOut = "empty_body";
    return false;
  }
  DeserializationError err = deserializeJson(doc, body);
  if (err) {
    errorOut = "invalid_json";
    return false;
  }
  return true;
}

void sendJsonAck(bool applied) {
  String payload = String("{\"ok\":true,\"applied\":") + (applied ? "true" : "false") + "}";
  server.send(200, "application/json", payload);
}

void sendJsonError(int code, const String &message) {
  String payload = String("{\"ok\":false,\"error\":\"") + message + "\"}";
  server.send(code, "application/json", payload);
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

String debugIsoOrPlaceholder(uint64_t epochMs) {
  if (!isTimeSynced() || epochMs == 0) return "unsynced";
  String iso = isoTimestamp(epochMs);
  return iso.length() > 0 ? iso : "unsynced";
}

String debugLastSampleLabel(const String &metricId, uint64_t &epochOut) {
  MetricSampleInfo *info = sampleInfoById(metricId);
  epochOut = info ? info->lastEpochMs : 0ULL;
  return debugIsoOrPlaceholder(epochOut);
}

void appendDebugSensorLine(String &out, const String &label, bool present, bool enabled, bool healthy,
                           float value, int decimals, const char *unit, const String &metricId) {
  uint64_t lastEpoch = 0;
  String lastIso = debugLastSampleLabel(metricId, lastEpoch);
  out += "- " + label + ": present=" + String(present ? 1 : 0) + " enabled=" + String(enabled ? 1 : 0)
      + " healthy=" + String(healthy ? 1 : 0) + " last=" + formatMetricValue(value, decimals) + " " + unit
      + " @ " + lastIso + " (epoch_ms=" + String((unsigned long long)lastEpoch) + ")\n";
}

String buildDebugLogBlock() {
  String block;
  block.reserve(1200);
  uint64_t epochMs = currentEpochMs();
  String localStamp = debugIsoOrPlaceholder(epochMs);
  block += "=== Debug Snapshot ===\n";
  block += "Local Time: " + localStamp + " (" + timezoneName + ")\n";
  block += "Epoch ms: " + String((unsigned long long)epochMs) + "\n";
  block += "Uptime: " + String(millis() / 1000UL) + "s\n";

  bool wifiConnected = WiFi.status() == WL_CONNECTED;
  String ssid = wifiConnected ? WiFi.SSID() : (apMode ? String(AP_SSID) : savedSsid);
  int rssi = wifiConnected ? WiFi.RSSI() : 0;
  block += "WiFi: " + String(wifiConnected ? "connected" : "offline");
  block += " ssid=" + ssid + " rssi=" + String(rssi) + "dBm";
  if (wifiConnectInProgress || wifiReconnectAttempts > 0) {
    block += " reconnect_attempts=" + String(wifiReconnectAttempts);
  }
  if (wifiConnectInProgress) block += " connecting=1";
  if (apMode) block += " ap_mode=1";
  block += "\n";

  String lastUploadIso = debugIsoOrPlaceholder(cloudStatus.lastUploadMs);
  block += "Cloud: " + String(cloudStatus.connected ? "connected" : "offline");
  block += " enabled=" + String(cloudConfig.enabled ? 1 : 0);
  block += " recording=" + String(cloudConfig.recording ? 1 : 0);
  block += " queue=" + String(cloudQueue.size());
  block += " last_upload=" + lastUploadIso + " (" + String((unsigned long long)cloudStatus.lastUploadMs) + ")";
  if (cloudStatus.lastUploadedPath.length() > 0) block += " path=" + cloudStatus.lastUploadedPath;
  if (cloudStatus.lastError.length() > 0) {
    block += " error=\"" + cloudStatus.lastError + "\"";
    if (cloudStatus.lastErrorSuffix.length() > 0) block += " suffix=" + cloudStatus.lastErrorSuffix;
  }
  block += "\n";

  block += "Sensors:\n";
  appendDebugSensorLine(block, "Lux", lightHealth.present, enableLight, lightHealth.healthy, latest.lux, 1, "Lux", "lux");
  appendDebugSensorLine(block, "PPFD", lightHealth.present, enableLight, lightHealth.healthy, latest.ppfd, 1, "µmol/m²/s", "lux");
  appendDebugSensorLine(block, "CO2", co2Health.present, enableCo2, co2Health.healthy,
                        latest.co2ppm > 0 ? static_cast<float>(latest.co2ppm) : NAN, 0, "ppm", "co2");
  appendDebugSensorLine(block, "Temp", climateHealth.present, enableClimate, climateHealth.healthy, latest.ambientTempC, 1, "°C", "temp");
  appendDebugSensorLine(block, "Humidity", climateHealth.present, enableClimate, climateHealth.healthy, latest.humidity, 1, "%", "humidity");
  appendDebugSensorLine(block, "Leaf", leafHealth.present, enableLeaf, leafHealth.healthy, latest.leafTempC, 1, "°C", "leaf");
  bool vpdOk = !isnan(latest.vpd);
  appendDebugSensorLine(block, "VPD", climateHealth.present, enableClimate, vpdOk, latest.vpd, 3, "kPa", "vpd");

  bool backoffActive = nextCloudAttemptAfterMs != 0 && millis() < nextCloudAttemptAfterMs;
  block += "Loop: last_ms=" + String(lastLoopDurationMs) + " free_heap=" + String(ESP.getFreeHeap()) + "\n";
  if (backoffActive) {
    block += "Queue Backoff: active (next_attempt_in=" + String((nextCloudAttemptAfterMs - millis()) / 1000UL) + "s)\n";
  } else {
    block += "Queue Backoff: inactive\n";
  }
  block += "\n";
  return block;
}

void enqueueDebugLogBlock() {
  if (!cloudConnected()) return;
  unsigned long now = millis();
  if (lastDebugLogMs != 0 && now - lastDebugLogMs < DEBUG_LOG_INTERVAL_MS) return;
  lastDebugLogMs = now;
  String dayKey = currentDayKey();
  if (dayKey.length() == 0) dayKey = "unsynced";
  String path = cloudRootPath() + "/logs/debug_" + dayKey + ".txt";
  String block = buildDebugLogBlock();
  enqueueCloudJob(path, block, dayKey, "debug", "text/plain", false);
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

bool uploadDebugLogJob(const CloudJob &job) {
  String base = safeBaseUrl();
  if (base.length() == 0) return false;
  String fullPath = job.path;
  if (!fullPath.startsWith("/")) fullPath = "/" + fullPath;
  if (!cloudEnsureFolders(job.dayKey)) {
    return false;
  }
  String existing;
  int code = 0;
  bool fetched = cloudGet(fullPath, existing, code);
  if (fetched) {
    if (code == 404) {
      existing = "";
    } else if (code < 200 || code >= 300) {
      markCloudFailure("Debug log GET code: " + String(code));
      return false;
    }
  } else {
    return false;
  }

  if (existing.length() > DEBUG_LOG_MAX_FILE_BYTES) {
    existing = existing.substring(existing.length() - DEBUG_LOG_MAX_FILE_BYTES);
  }
  String payload = existing;
  if (payload.length() > 0 && !payload.endsWith("\n")) payload += "\n";
  payload += job.payload;

  String url = buildWebDavUrl(fullPath);
  String location;
  String chain;
  String resp;
  bool httpsRedirected = false;
  int putCode = 0;
  bool ok = webdavRequestFollowRedirects("PUT", url, payload, "text/plain", putCode, location, resp, &chain, &httpsRedirected, CLOUD_TEST_TIMEOUT_MS);
  setCloudRequestNote("PUT", "debug", fullPath, putCode, chain, httpsRedirected);
  if (!ok) {
    if (chain.length() > 0) {
      markCloudFailure(cloudStatus.lastError);
      cloudStatus.lastErrorSuffix = httpsRedirected ? "redirected-to-https" : "";
    } else {
      setCloudError("PUT", "debug", fullPath, putCode, resp, httpsRedirected ? "redirected-to-https" : "");
    }
    return false;
  }
  cloudStatus.lastUploadMs = currentEpochMs();
  cloudStatus.lastUploadedPath = fullPath;
  cloudStatus.lastError = "";
  markCloudSuccess(cloudRequestStateText("PUT", "debug", putCode));
  if (chain.length() > 0) {
    cloudStatus.lastError = String("PUT debug -> ") + chain;
  }
  cloudStatus.queueSize = cloudQueue.size();
  return true;
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
  if (job.kind == "debug") {
    return uploadDebugLogJob(job);
  }
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
  if (job.kind == "debug" && job.createdAtMs != 0 && now - job.createdAtMs > DEBUG_LOG_MAX_CACHE_MS) {
    cloudQueue.erase(cloudQueue.begin());
    cloudStatus.queueSize = cloudQueue.size();
    return;
  }
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
  wifiReconnectAttempts++;
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
  ppfdScale = prefsSystem.getFloat("ppfd_scale", 1.0f);
  phaseId = prefsSystem.getString("phase", "");
  prefsSystem.end();
  if (phaseId.length() > 0 && phaseId != "custom") {
    const PhasePreset *preset = phasePresetById(phaseId);
    if (preset) {
      applyPhasePreset(*preset, false);
    } else {
      phaseId = phaseFromState();
    }
  } else {
    phaseId = phaseFromState();
  }
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
    latest.ppfd = luxToPPFD(latest.lux, channel) * ppfdScale;
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
  31, 139, 8, 0, 164, 50, 71, 105, 2, 255, 236, 189, 219, 114, 27, 73,
  146, 40, 248, 222, 95, 145, 197, 170, 110, 0, 85, 0, 8, 128, 23, 145,
  160, 72, 29, 137, 162, 90, 156, 162, 36, 174, 200, 146, 186, 90, 45, 147,
  146, 64, 128, 200, 102, 2, 137, 201, 76, 240, 82, 58, 52, 27, 59, 15,
  251, 186, 182, 54, 107, 187, 251, 182, 102, 199, 250, 3, 142, 173, 217, 158,
  167, 121, 58, 245, 39, 253, 37, 235, 238, 113, 201, 184, 101, 2, 164, 84,
  213, 213, 182, 59, 61, 37, 34, 227, 226, 225, 17, 225, 225, 225, 238, 225,
  225, 241, 187, 32, 120, 248, 213, 48, 25, 228, 55, 51, 22, 140, 243, 73,
  188, 247, 59, 72, 193, 31, 65, 28, 78, 207, 119, 87, 134, 108, 133, 39,
  177, 112, 136, 63, 224, 231, 132, 229, 97, 48, 24, 135, 105, 198, 242, 221,
  149, 31, 78, 159, 181, 182, 86, 130, 85, 61, 115, 26, 78, 216, 238, 202,
  101, 196, 174, 102, 73, 154, 175, 4, 131, 100, 154, 179, 41, 20, 190, 138,
  134, 249, 120, 119, 200, 46, 163, 1, 107, 209, 71, 51, 136, 166, 81, 30,
  133, 113, 43, 27, 132, 49, 219, 237, 22, 160, 242, 40, 143, 217, 222, 31,
  211, 228, 234, 132, 77, 179, 36, 13, 46, 59, 237, 181, 246, 198, 195, 85,
  158, 193, 11, 101, 249, 141, 252, 29, 4, 253, 52, 73, 242, 224, 19, 180,
  23, 39, 41, 0, 28, 179, 9, 235, 7, 113, 116, 62, 206, 131, 97, 152,
  94, 236, 4, 183, 162, 36, 246, 176, 25, 156, 37, 195, 27, 40, 126, 22,
  14, 46, 206, 211, 100, 62, 29, 246, 131, 175, 59, 163, 238, 131, 94, 184,
  195, 97, 192, 55, 235, 177, 173, 81, 103, 39, 152, 68, 211, 214, 152, 33,
  168, 126, 208, 237, 116, 126, 95, 192, 18, 80, 70, 208, 201, 214, 40, 156,
  68, 241, 77, 63, 200, 110, 178, 156, 77, 90, 243, 168, 25, 100, 225, 52,
  107, 101, 44, 141, 70, 0, 36, 76, 207, 163, 105, 63, 0, 120, 179, 112,
  56, 140, 166, 231, 244, 91, 97, 5, 131, 204, 82, 128, 165, 50, 187, 155,
  179, 235, 29, 19, 193, 110, 183, 187, 213, 123, 0, 137, 201, 117, 43, 27,
  135, 195, 228, 10, 96, 4, 189, 217, 117, 0, 101, 131, 244, 252, 44, 172,
  119, 154, 244, 191, 118, 111, 163, 1, 13, 37, 25, 12, 112, 2, 173, 102,
  121, 52, 184, 184, 217, 9, 242, 100, 70, 205, 254, 212, 138, 166, 67, 118,
  13, 173, 244, 52, 28, 186, 208, 190, 134, 39, 117, 43, 139, 126, 130, 129,
  236, 182, 123, 41, 155, 104, 16, 83, 22, 135, 121, 116, 201, 138, 218, 109,
  222, 133, 22, 76, 26, 128, 25, 70, 217, 44, 14, 111, 250, 163, 152, 65,
  47, 254, 58, 7, 4, 70, 55, 45, 65, 13, 128, 207, 44, 4, 50, 56,
  99, 249, 21, 99, 211, 157, 32, 132, 137, 154, 182, 34, 24, 183, 172, 31,
  12, 160, 4, 75, 119, 130, 243, 112, 214, 239, 246, 112, 16, 16, 72, 235,
  42, 133, 239, 0, 255, 213, 218, 4, 128, 83, 167, 53, 29, 154, 14, 108,
  11, 97, 169, 170, 49, 27, 66, 77, 162, 68, 209, 140, 152, 99, 254, 113,
  150, 164, 212, 155, 112, 24, 205, 179, 254, 6, 206, 186, 54, 23, 95, 179,
  209, 58, 252, 159, 49, 21, 157, 0, 255, 183, 38, 103, 162, 183, 182, 221,
  220, 220, 194, 255, 23, 179, 33, 145, 140, 166, 113, 52, 133, 206, 199, 201,
  224, 66, 239, 74, 156, 204, 135, 45, 3, 171, 142, 142, 85, 199, 192, 158,
  151, 134, 73, 140, 6, 97, 158, 164, 75, 143, 193, 166, 1, 229, 44, 28,
  158, 51, 173, 174, 137, 154, 36, 197, 117, 232, 210, 150, 59, 38, 219, 219,
  219, 52, 57, 138, 74, 58, 237, 45, 34, 18, 125, 156, 186, 163, 222, 246,
  218, 3, 185, 166, 190, 30, 156, 13, 55, 88, 215, 198, 160, 5, 108, 193,
  92, 140, 95, 119, 59, 103, 219, 91, 93, 85, 79, 46, 77, 171, 222, 85,
  152, 78, 173, 138, 163, 141, 109, 214, 57, 43, 175, 120, 62, 78, 178, 220,
  172, 147, 167, 176, 78, 103, 97, 10, 195, 36, 251, 216, 239, 66, 159, 179,
  36, 142, 134, 129, 221, 5, 201, 21, 36, 196, 73, 24, 77, 221, 117, 43,
  71, 52, 56, 79, 163, 33, 31, 250, 128, 19, 86, 49, 133, 97, 58, 180,
  89, 80, 177, 194, 9, 139, 192, 131, 134, 57, 9, 2, 168, 205, 53, 12,
  6, 33, 230, 175, 146, 65, 20, 235, 57, 185, 100, 233, 40, 134, 154, 227,
  104, 56, 196, 197, 169, 16, 206, 136, 23, 183, 4, 222, 30, 86, 224, 84,
  213, 56, 103, 119, 171, 163, 143, 11, 167, 82, 90, 216, 195, 40, 101, 3,
  130, 4, 35, 60, 159, 232, 45, 226, 224, 105, 228, 233, 14, 38, 38, 180,
  128, 200, 33, 59, 103, 45, 94, 63, 195, 222, 204, 88, 152, 215, 195, 121,
  158, 180, 70, 81, 222, 68, 68, 38, 225, 117, 189, 215, 1, 36, 154, 65,
  119, 148, 54, 26, 90, 51, 151, 97, 60, 103, 146, 143, 75, 134, 183, 73,
  180, 76, 73, 151, 97, 26, 133, 240, 119, 58, 159, 0, 51, 31, 244, 131,
  60, 60, 155, 199, 97, 138, 9, 89, 1, 39, 14, 207, 88, 172, 163, 43,
  214, 17, 103, 169, 45, 226, 189, 91, 230, 146, 9, 58, 237, 109, 106, 71,
  194, 136, 166, 179, 57, 224, 155, 177, 24, 198, 4, 182, 169, 121, 158, 39,
  83, 197, 17, 196, 246, 83, 204, 54, 141, 169, 14, 127, 221, 93, 167, 250,
  218, 245, 83, 148, 177, 9, 158, 117, 123, 189, 142, 187, 9, 170, 45, 79,
  98, 52, 152, 167, 25, 150, 152, 37, 17, 103, 47, 178, 133, 105, 50, 101,
  38, 80, 100, 42, 48, 90, 231, 136, 16, 44, 178, 122, 183, 215, 25, 178,
  243, 102, 240, 117, 175, 55, 92, 99, 12, 126, 108, 174, 109, 110, 142, 186,
  141, 162, 89, 137, 6, 13, 213, 149, 216, 124, 55, 59, 14, 34, 125, 24,
  236, 240, 140, 115, 205, 4, 54, 149, 40, 191, 193, 65, 221, 220, 81, 248,
  77, 147, 188, 21, 198, 64, 149, 108, 168, 77, 185, 185, 75, 5, 156, 28,
  137, 176, 104, 180, 188, 27, 82, 249, 54, 132, 208, 246, 130, 111, 145, 132,
  98, 218, 85, 245, 85, 147, 135, 249, 60, 147, 212, 229, 237, 138, 40, 211,
  78, 46, 164, 4, 3, 35, 176, 182, 62, 92, 219, 222, 118, 11, 177, 52,
  213, 74, 141, 182, 30, 116, 31, 232, 205, 197, 201, 57, 172, 126, 155, 98,
  12, 17, 134, 175, 68, 67, 104, 153, 71, 173, 73, 50, 77, 104, 91, 110,
  6, 39, 207, 94, 192, 71, 235, 53, 59, 71, 42, 111, 6, 47, 216, 52,
  78, 224, 79, 50, 13, 7, 240, 119, 63, 1, 62, 16, 135, 89, 51, 88,
  57, 138, 206, 88, 26, 226, 226, 197, 220, 100, 5, 146, 246, 147, 121, 26,
  129, 40, 243, 146, 93, 193, 167, 130, 186, 19, 92, 141, 97, 60, 91, 244,
  1, 84, 147, 178, 150, 189, 151, 131, 92, 153, 235, 140, 69, 99, 74, 18,
  119, 90, 192, 69, 157, 65, 56, 189, 12, 51, 187, 179, 37, 133, 71, 32,
  33, 146, 136, 149, 179, 107, 36, 9, 152, 225, 98, 110, 139, 37, 213, 115,
  86, 232, 214, 6, 45, 81, 57, 228, 219, 235, 225, 218, 217, 150, 197, 200,
  91, 74, 128, 91, 188, 15, 47, 146, 134, 92, 89, 133, 79, 125, 107, 72,
  34, 238, 125, 69, 150, 205, 179, 7, 189, 173, 78, 149, 200, 210, 237, 60,
  104, 118, 187, 235, 205, 110, 79, 9, 45, 30, 20, 218, 240, 95, 107, 54,
  143, 51, 6, 228, 67, 123, 105, 56, 141, 38, 33, 159, 50, 200, 59, 198,
  172, 160, 215, 94, 207, 2, 22, 66, 33, 32, 189, 100, 158, 3, 103, 27,
  161, 188, 207, 22, 193, 204, 98, 90, 154, 62, 152, 235, 237, 173, 251, 193,
  140, 134, 49, 43, 129, 249, 96, 17, 196, 255, 116, 193, 110, 70, 41, 168,
  53, 89, 81, 233, 83, 208, 249, 61, 110, 221, 246, 72, 118, 248, 56, 110,
  244, 154, 189, 110, 183, 217, 221, 88, 131, 97, 236, 210, 48, 6, 15, 252,
  53, 112, 230, 220, 74, 84, 3, 137, 121, 201, 70, 168, 188, 26, 130, 60,
  138, 89, 139, 52, 165, 123, 138, 198, 4, 224, 51, 8, 186, 140, 158, 197,
  70, 117, 150, 0, 231, 158, 88, 162, 40, 104, 143, 176, 181, 182, 176, 105,
  223, 230, 226, 145, 54, 72, 108, 19, 156, 2, 54, 119, 193, 223, 2, 216,
  54, 38, 124, 78, 155, 188, 200, 40, 73, 39, 200, 244, 84, 170, 216, 39,
  140, 52, 177, 118, 104, 141, 195, 164, 200, 12, 159, 76, 163, 154, 234, 63,
  232, 117, 52, 225, 75, 246, 107, 125, 171, 172, 99, 253, 49, 66, 67, 14,
  36, 241, 234, 243, 159, 40, 189, 252, 88, 111, 1, 45, 52, 118, 12, 84,
  112, 31, 88, 91, 239, 110, 108, 216, 179, 35, 134, 218, 47, 135, 73, 237,
  174, 183, 164, 188, 165, 38, 72, 12, 119, 139, 93, 2, 236, 172, 143, 242,
  147, 166, 128, 232, 154, 47, 199, 66, 232, 191, 213, 147, 163, 207, 205, 186,
  59, 11, 192, 165, 61, 243, 85, 164, 250, 70, 92, 238, 244, 221, 29, 109,
  40, 181, 145, 196, 5, 81, 41, 140, 110, 108, 46, 45, 140, 170, 193, 209,
  70, 213, 148, 28, 91, 179, 112, 74, 130, 95, 165, 204, 179, 197, 101, 30,
  206, 104, 55, 154, 189, 181, 230, 122, 15, 216, 195, 118, 195, 77, 123, 128,
  210, 41, 2, 27, 166, 201, 12, 4, 216, 56, 71, 185, 234, 44, 158, 167,
  245, 46, 167, 17, 144, 35, 206, 46, 162, 188, 85, 89, 200, 81, 101, 120,
  59, 235, 91, 205, 238, 230, 90, 179, 187, 181, 142, 76, 190, 215, 176, 213,
  133, 46, 234, 11, 107, 29, 91, 97, 88, 239, 53, 22, 42, 31, 88, 137,
  167, 89, 26, 229, 194, 17, 94, 231, 252, 225, 186, 37, 85, 95, 146, 115,
  43, 73, 219, 36, 197, 108, 126, 134, 251, 186, 33, 197, 75, 225, 218, 82,
  62, 9, 35, 73, 9, 237, 53, 255, 90, 109, 67, 165, 56, 156, 101, 36,
  89, 106, 20, 184, 77, 148, 163, 209, 210, 230, 186, 135, 7, 112, 53, 76,
  146, 41, 224, 177, 85, 65, 169, 11, 154, 55, 150, 154, 134, 73, 71, 107,
  160, 4, 122, 107, 147, 8, 65, 78, 6, 151, 204, 23, 53, 71, 76, 170,
  37, 165, 177, 162, 15, 189, 222, 146, 168, 170, 157, 195, 228, 247, 157, 69,
  213, 61, 236, 209, 194, 216, 68, 77, 145, 71, 120, 6, 244, 61, 199, 77,
  59, 154, 102, 140, 70, 70, 210, 164, 144, 140, 138, 133, 137, 164, 27, 198,
  197, 186, 28, 68, 233, 0, 182, 156, 16, 152, 19, 108, 183, 240, 159, 88,
  142, 107, 235, 180, 189, 246, 214, 80, 20, 234, 108, 120, 86, 233, 198, 70,
  195, 183, 118, 123, 141, 42, 35, 130, 185, 128, 204, 181, 226, 110, 176, 89,
  158, 178, 124, 48, 246, 236, 176, 50, 195, 203, 177, 229, 34, 233, 234, 36,
  184, 6, 211, 119, 25, 101, 209, 89, 20, 99, 2, 253, 140, 77, 78, 237,
  110, 140, 149, 251, 88, 41, 165, 108, 218, 139, 147, 221, 48, 255, 124, 197,
  108, 84, 72, 175, 124, 229, 208, 7, 231, 2, 189, 77, 77, 198, 237, 109,
  150, 201, 184, 229, 131, 173, 73, 191, 166, 94, 171, 212, 90, 47, 159, 90,
  70, 198, 209, 165, 39, 131, 11, 218, 18, 182, 215, 250, 210, 221, 106, 24,
  35, 111, 138, 31, 218, 46, 89, 116, 193, 72, 214, 4, 27, 189, 176, 106,
  55, 232, 110, 22, 83, 40, 201, 97, 205, 51, 45, 106, 205, 25, 82, 135,
  18, 58, 12, 147, 156, 176, 77, 249, 57, 77, 87, 108, 57, 90, 207, 209,
  40, 77, 155, 137, 209, 245, 181, 134, 15, 141, 112, 128, 252, 221, 88, 251,
  101, 92, 82, 81, 84, 118, 121, 94, 168, 67, 91, 186, 58, 180, 165, 47,
  43, 219, 208, 170, 234, 183, 225, 159, 214, 32, 78, 56, 143, 191, 27, 147,
  52, 129, 36, 51, 54, 253, 76, 16, 14, 30, 54, 218, 250, 98, 179, 244,
  94, 67, 237, 117, 20, 126, 158, 96, 129, 37, 59, 69, 55, 232, 6, 156,
  99, 120, 217, 136, 106, 122, 200, 46, 91, 83, 80, 158, 149, 221, 65, 89,
  89, 221, 189, 86, 51, 70, 25, 2, 254, 215, 0, 228, 69, 50, 12, 99,
  157, 17, 140, 162, 107, 180, 202, 40, 174, 109, 14, 224, 93, 150, 161, 206,
  225, 117, 106, 219, 108, 20, 244, 191, 217, 113, 134, 115, 28, 77, 109, 145,
  193, 208, 246, 149, 178, 95, 214, 173, 105, 120, 233, 168, 72, 74, 98, 44,
  12, 70, 220, 212, 161, 1, 49, 205, 249, 8, 197, 50, 245, 137, 121, 17,
  76, 101, 75, 9, 86, 203, 152, 93, 93, 176, 253, 254, 25, 131, 69, 5,
  60, 66, 79, 11, 71, 220, 32, 34, 7, 115, 101, 101, 167, 124, 83, 109,
  173, 89, 6, 5, 255, 78, 90, 108, 159, 221, 237, 7, 205, 109, 148, 49,
  215, 27, 65, 199, 147, 220, 105, 4, 155, 157, 223, 55, 74, 164, 24, 58,
  19, 172, 243, 9, 172, 220, 162, 12, 173, 65, 83, 220, 44, 146, 230, 36,
  53, 137, 174, 91, 176, 235, 77, 135, 173, 73, 50, 100, 208, 72, 202, 202,
  6, 76, 14, 142, 232, 61, 109, 53, 186, 1, 33, 141, 70, 121, 176, 150,
  9, 65, 223, 99, 56, 208, 128, 17, 177, 121, 231, 64, 228, 200, 198, 188,
  138, 13, 31, 138, 110, 195, 111, 147, 32, 60, 128, 132, 211, 100, 82, 162,
  89, 174, 13, 235, 45, 32, 160, 38, 74, 131, 77, 24, 117, 13, 94, 176,
  65, 54, 134, 146, 90, 88, 105, 203, 168, 131, 146, 208, 206, 109, 144, 39,
  21, 77, 97, 43, 118, 75, 154, 117, 2, 207, 134, 29, 94, 89, 161, 25,
  88, 167, 38, 88, 189, 173, 118, 11, 115, 225, 105, 28, 119, 58, 111, 157,
  229, 246, 217, 80, 201, 73, 78, 185, 0, 161, 0, 94, 69, 163, 168, 165,
  44, 185, 119, 176, 169, 152, 34, 176, 105, 25, 152, 9, 91, 82, 65, 85,
  60, 165, 187, 216, 204, 165, 17, 192, 236, 206, 22, 169, 245, 106, 139, 84,
  231, 203, 91, 164, 56, 99, 42, 223, 33, 47, 103, 67, 87, 170, 119, 172,
  191, 194, 112, 237, 19, 161, 29, 22, 232, 19, 253, 150, 152, 103, 66, 68,
  110, 174, 85, 250, 133, 81, 3, 91, 135, 94, 45, 82, 73, 124, 12, 105,
  41, 19, 196, 2, 251, 154, 182, 73, 236, 148, 107, 191, 246, 65, 14, 97,
  30, 179, 115, 96, 134, 229, 59, 152, 143, 182, 237, 93, 77, 59, 81, 71,
  96, 173, 236, 42, 4, 213, 164, 16, 78, 54, 157, 83, 108, 115, 6, 215,
  61, 246, 2, 91, 248, 65, 92, 97, 41, 93, 16, 147, 188, 147, 1, 156,
  207, 121, 175, 152, 115, 121, 34, 236, 80, 177, 146, 84, 209, 2, 190, 137,
  26, 29, 46, 150, 181, 141, 134, 73, 77, 163, 179, 179, 81, 111, 221, 194,
  109, 154, 180, 134, 97, 30, 46, 160, 128, 187, 91, 82, 101, 186, 37, 141,
  216, 102, 31, 110, 245, 225, 18, 184, 37, 8, 105, 170, 233, 166, 105, 208,
  215, 78, 115, 197, 113, 152, 142, 17, 205, 50, 112, 188, 52, 119, 235, 12,
  89, 54, 40, 100, 65, 137, 148, 75, 121, 118, 61, 235, 56, 87, 156, 230,
  122, 143, 112, 237, 19, 92, 121, 128, 139, 246, 191, 166, 56, 191, 245, 236,
  14, 162, 33, 97, 69, 190, 155, 26, 94, 106, 50, 80, 12, 100, 105, 123,
  33, 145, 188, 176, 97, 43, 19, 182, 110, 175, 214, 244, 183, 222, 18, 70,
  104, 95, 255, 238, 97, 76, 118, 109, 201, 2, 222, 125, 118, 54, 99, 186,
  55, 124, 118, 182, 146, 86, 188, 167, 89, 21, 174, 46, 162, 118, 72, 195,
  156, 45, 47, 109, 23, 10, 95, 120, 190, 208, 199, 101, 237, 151, 243, 113,
  249, 154, 118, 182, 133, 186, 79, 149, 6, 115, 127, 189, 72, 234, 62, 91,
  29, 63, 66, 237, 9, 254, 105, 185, 190, 40, 75, 236, 154, 214, 34, 90,
  55, 22, 209, 102, 97, 198, 217, 38, 133, 84, 217, 118, 55, 122, 29, 203,
  71, 69, 8, 29, 61, 199, 76, 178, 174, 251, 77, 45, 92, 117, 38, 47,
  248, 154, 211, 205, 219, 232, 39, 203, 95, 229, 151, 83, 58, 77, 227, 96,
  9, 155, 230, 235, 82, 159, 23, 159, 122, 179, 174, 157, 64, 21, 234, 145,
  183, 111, 101, 210, 176, 166, 75, 248, 199, 228, 87, 153, 251, 158, 49, 247,
  155, 235, 158, 185, 71, 59, 209, 90, 207, 158, 251, 141, 134, 79, 29, 220,
  94, 247, 40, 140, 124, 236, 52, 158, 186, 233, 57, 236, 219, 116, 217, 169,
  119, 24, 205, 81, 241, 105, 97, 158, 129, 21, 149, 114, 216, 244, 203, 25,
  148, 175, 176, 210, 250, 185, 65, 70, 27, 196, 142, 189, 11, 153, 67, 127,
  7, 15, 53, 183, 189, 130, 100, 180, 6, 74, 188, 116, 164, 147, 142, 237,
  163, 99, 17, 137, 216, 98, 12, 95, 58, 11, 1, 50, 113, 249, 69, 164,
  194, 36, 146, 234, 242, 92, 137, 143, 158, 95, 127, 19, 154, 164, 98, 213,
  210, 103, 213, 222, 135, 53, 165, 14, 231, 29, 182, 37, 54, 187, 147, 66,
  106, 110, 79, 26, 148, 133, 122, 233, 44, 138, 99, 205, 87, 176, 218, 177,
  114, 137, 45, 198, 99, 183, 242, 26, 180, 149, 76, 226, 227, 110, 166, 3,
  78, 11, 36, 177, 252, 230, 215, 20, 100, 203, 164, 85, 93, 88, 13, 175,
  163, 172, 37, 253, 235, 74, 85, 28, 199, 51, 135, 212, 246, 179, 48, 245,
  47, 202, 117, 123, 68, 104, 210, 65, 133, 41, 36, 145, 77, 107, 170, 57,
  56, 37, 182, 172, 219, 243, 164, 236, 230, 198, 132, 246, 188, 80, 188, 139,
  16, 86, 219, 96, 99, 67, 23, 249, 128, 4, 147, 233, 20, 232, 80, 29,
  51, 59, 102, 68, 139, 103, 20, 252, 97, 24, 102, 99, 54, 12, 74, 208,
  114, 88, 140, 119, 19, 179, 92, 142, 191, 128, 12, 191, 110, 203, 240, 62,
  215, 102, 71, 36, 189, 71, 75, 192, 243, 205, 150, 182, 44, 119, 201, 45,
  79, 195, 98, 141, 126, 214, 118, 104, 204, 202, 150, 123, 198, 183, 64, 161,
  175, 94, 165, 132, 229, 32, 101, 67, 200, 137, 194, 56, 243, 81, 196, 61,
  53, 159, 78, 153, 230, 83, 222, 250, 47, 225, 39, 84, 209, 90, 158, 156,
  159, 147, 110, 231, 179, 146, 43, 126, 90, 161, 138, 186, 32, 179, 249, 4,
  198, 239, 198, 217, 2, 190, 128, 167, 147, 58, 46, 88, 232, 147, 80, 129,
  159, 56, 249, 175, 114, 68, 234, 10, 87, 71, 219, 124, 236, 58, 224, 116,
  150, 147, 140, 92, 52, 244, 147, 171, 82, 28, 75, 189, 19, 188, 182, 175,
  251, 54, 230, 78, 152, 181, 205, 178, 52, 5, 133, 241, 44, 4, 182, 153,
  26, 14, 165, 165, 215, 96, 182, 59, 150, 27, 244, 131, 81, 119, 216, 29,
  22, 78, 159, 35, 54, 24, 14, 215, 52, 95, 151, 226, 68, 70, 172, 40,
  97, 226, 213, 29, 172, 207, 182, 187, 131, 238, 64, 187, 12, 224, 18, 150,
  113, 219, 197, 231, 124, 106, 145, 175, 209, 181, 121, 108, 94, 216, 145, 78,
  40, 116, 170, 30, 116, 205, 181, 100, 212, 84, 98, 167, 222, 105, 223, 61,
  8, 189, 59, 163, 65, 184, 17, 110, 184, 99, 98, 57, 5, 109, 26, 62,
  65, 106, 77, 58, 46, 125, 218, 129, 236, 132, 137, 107, 67, 14, 35, 115,
  119, 109, 158, 126, 39, 195, 100, 254, 83, 139, 59, 212, 91, 124, 3, 15,
  73, 165, 145, 178, 99, 121, 64, 2, 74, 203, 250, 19, 21, 150, 138, 180,
  149, 76, 227, 18, 225, 73, 52, 164, 219, 57, 44, 183, 1, 49, 147, 173,
  174, 215, 162, 61, 136, 163, 89, 31, 229, 81, 169, 33, 53, 76, 151, 234,
  105, 194, 123, 45, 38, 174, 99, 186, 167, 251, 77, 240, 193, 87, 209, 4,
  239, 233, 133, 134, 180, 206, 229, 192, 10, 211, 48, 159, 2, 123, 184, 189,
  83, 178, 244, 216, 9, 227, 49, 86, 119, 141, 53, 75, 220, 105, 186, 187,
  76, 125, 135, 99, 1, 113, 171, 112, 1, 13, 109, 116, 172, 11, 102, 212,
  35, 171, 174, 86, 222, 240, 30, 197, 94, 4, 230, 138, 189, 152, 69, 52,
  190, 11, 175, 211, 201, 193, 208, 189, 1, 12, 40, 92, 102, 93, 70, 146,
  42, 189, 58, 179, 80, 150, 226, 188, 203, 235, 189, 164, 86, 131, 206, 127,
  53, 246, 219, 237, 250, 61, 77, 244, 57, 237, 232, 102, 4, 225, 6, 90,
  218, 95, 65, 69, 95, 78, 134, 243, 49, 163, 123, 8, 113, 197, 226, 89,
  194, 81, 134, 252, 239, 53, 63, 142, 141, 77, 135, 58, 16, 58, 157, 63,
  100, 225, 200, 82, 36, 156, 227, 246, 110, 207, 54, 7, 123, 14, 234, 119,
  130, 86, 11, 1, 159, 35, 231, 113, 242, 215, 12, 125, 192, 64, 192, 189,
  137, 199, 47, 65, 174, 111, 52, 187, 27, 91, 205, 110, 23, 49, 88, 247,
  98, 96, 148, 33, 139, 159, 141, 131, 81, 162, 2, 137, 179, 112, 232, 195,
  65, 187, 136, 89, 134, 130, 86, 196, 143, 129, 86, 160, 2, 1, 247, 68,
  23, 178, 151, 191, 187, 128, 176, 196, 166, 179, 180, 209, 115, 221, 33, 138,
  133, 106, 178, 195, 123, 177, 150, 123, 7, 174, 219, 238, 150, 95, 129, 51,
  111, 192, 221, 215, 169, 206, 189, 145, 74, 67, 144, 242, 141, 167, 4, 21,
  113, 133, 234, 65, 231, 243, 90, 162, 86, 90, 243, 89, 113, 144, 229, 168,
  222, 188, 8, 172, 209, 169, 230, 249, 100, 95, 184, 226, 133, 70, 192, 69,
  139, 66, 108, 131, 61, 96, 103, 78, 33, 177, 72, 44, 31, 42, 171, 208,
  121, 146, 12, 23, 226, 196, 41, 93, 182, 38, 238, 34, 91, 27, 120, 158,
  36, 113, 30, 205, 60, 166, 111, 175, 52, 94, 97, 7, 216, 238, 45, 235,
  77, 190, 209, 112, 136, 193, 112, 97, 242, 29, 224, 151, 156, 6, 184, 30,
  232, 180, 240, 92, 170, 246, 251, 192, 111, 46, 225, 39, 191, 105, 152, 226,
  187, 61, 99, 143, 233, 173, 91, 102, 9, 125, 80, 23, 122, 52, 152, 83,
  208, 246, 173, 201, 173, 106, 61, 208, 0, 0, 91, 105, 50, 61, 87, 83,
  30, 77, 199, 176, 10, 115, 207, 130, 240, 56, 136, 40, 22, 84, 238, 35,
  2, 171, 187, 94, 176, 188, 166, 119, 106, 27, 203, 93, 99, 42, 200, 102,
  25, 151, 17, 171, 180, 194, 126, 194, 134, 81, 24, 212, 103, 41, 27, 177,
  52, 107, 129, 242, 55, 31, 48, 116, 218, 146, 23, 3, 241, 187, 17, 124,
  18, 229, 171, 238, 167, 53, 43, 238, 153, 53, 43, 238, 139, 53, 23, 242,
  118, 115, 202, 249, 223, 135, 171, 42, 44, 197, 195, 85, 25, 57, 227, 33,
  170, 199, 34, 106, 5, 55, 151, 200, 176, 21, 15, 135, 209, 37, 72, 246,
  97, 150, 237, 174, 20, 241, 19, 86, 246, 84, 191, 176, 64, 241, 133, 245,
  187, 122, 92, 140, 191, 255, 219, 191, 171, 216, 24, 144, 163, 23, 212, 33,
  43, 95, 199, 149, 189, 35, 52, 57, 190, 72, 96, 231, 73, 82, 88, 150,
  15, 87, 141, 6, 236, 79, 63, 122, 1, 245, 113, 119, 69, 137, 51, 43,
  101, 45, 163, 233, 210, 200, 196, 184, 29, 179, 112, 42, 243, 99, 54, 92,
  9, 162, 33, 47, 120, 4, 31, 123, 48, 128, 144, 239, 171, 34, 139, 157,
  194, 14, 185, 178, 151, 140, 70, 200, 242, 203, 139, 139, 22, 40, 80, 0,
  111, 3, 13, 175, 47, 18, 12, 107, 242, 170, 172, 178, 213, 127, 171, 51,
  102, 204, 133, 5, 253, 10, 84, 64, 7, 209, 67, 252, 92, 166, 139, 88,
  142, 247, 113, 31, 127, 222, 165, 135, 84, 247, 9, 125, 239, 1, 105, 44,
  211, 63, 97, 9, 192, 218, 232, 197, 252, 148, 93, 174, 72, 192, 20, 42,
  65, 77, 182, 166, 244, 172, 236, 65, 177, 22, 12, 229, 60, 123, 184, 202,
  1, 24, 48, 85, 95, 134, 236, 242, 132, 214, 215, 138, 129, 108, 160, 162,
  62, 40, 232, 6, 27, 37, 248, 65, 120, 145, 71, 151, 118, 23, 140, 14,
  152, 31, 218, 76, 73, 139, 130, 190, 144, 56, 11, 30, 37, 41, 207, 254,
  9, 26, 58, 33, 165, 204, 223, 195, 63, 179, 40, 199, 50, 15, 87, 169,
  162, 6, 72, 168, 114, 216, 63, 27, 144, 108, 94, 218, 27, 204, 117, 145,
  204, 232, 162, 52, 73, 90, 187, 43, 7, 115, 216, 143, 216, 234, 19, 150,
  2, 41, 174, 236, 25, 159, 15, 87, 121, 217, 138, 234, 63, 156, 238, 175,
  236, 193, 63, 75, 20, 21, 160, 143, 146, 233, 48, 41, 90, 226, 159, 75,
  84, 127, 76, 18, 95, 184, 250, 146, 93, 125, 248, 49, 73, 47, 86, 246,
  236, 148, 101, 128, 128, 178, 148, 130, 96, 22, 174, 158, 220, 12, 167, 236,
  6, 128, 88, 41, 203, 0, 201, 160, 244, 105, 114, 113, 147, 64, 117, 245,
  219, 173, 8, 68, 67, 195, 175, 79, 154, 189, 96, 130, 34, 130, 200, 138,
  154, 75, 177, 116, 112, 238, 131, 105, 52, 24, 231, 65, 118, 51, 29, 140,
  83, 28, 39, 139, 14, 117, 128, 202, 90, 196, 33, 197, 201, 32, 140, 79,
  33, 141, 175, 98, 103, 33, 150, 146, 173, 84, 248, 87, 252, 28, 88, 104,
  242, 188, 17, 248, 120, 18, 166, 165, 140, 87, 110, 93, 43, 1, 250, 165,
  181, 248, 21, 4, 192, 20, 244, 125, 155, 111, 89, 181, 168, 31, 102, 17,
  183, 16, 173, 137, 149, 189, 83, 0, 23, 212, 255, 199, 127, 219, 111, 56,
  140, 197, 87, 139, 166, 17, 248, 159, 226, 15, 144, 120, 74, 24, 21, 99,
  100, 12, 172, 210, 6, 86, 244, 226, 167, 148, 162, 215, 113, 27, 119, 25,
  93, 21, 103, 47, 25, 173, 241, 124, 18, 13, 163, 252, 230, 11, 142, 216,
  115, 1, 50, 168, 255, 254, 254, 99, 246, 92, 225, 181, 244, 184, 201, 42,
  191, 218, 216, 13, 146, 222, 23, 28, 182, 253, 87, 127, 255, 47, 255, 5,
  228, 193, 217, 228, 254, 163, 182, 143, 24, 45, 61, 96, 80, 250, 87, 27,
  43, 16, 47, 191, 224, 88, 189, 57, 126, 26, 212, 47, 142, 195, 251, 143,
  212, 155, 217, 240, 14, 35, 5, 165, 191, 196, 72, 85, 236, 236, 211, 80,
  47, 167, 73, 44, 144, 241, 52, 204, 198, 103, 73, 152, 14, 213, 238, 43,
  157, 255, 65, 138, 144, 121, 174, 148, 98, 65, 225, 226, 116, 230, 129, 193,
  115, 216, 116, 33, 8, 18, 214, 60, 0, 132, 16, 103, 214, 126, 184, 170,
  250, 196, 213, 4, 169, 18, 208, 244, 32, 68, 58, 174, 121, 66, 167, 53,
  10, 168, 126, 132, 163, 168, 225, 33, 215, 13, 247, 158, 177, 113, 204, 82,
  212, 63, 232, 83, 230, 206, 227, 2, 220, 81, 4, 242, 28, 204, 205, 92,
  137, 51, 122, 31, 6, 49, 11, 211, 3, 44, 151, 65, 183, 7, 227, 56,
  98, 63, 255, 95, 118, 199, 181, 137, 121, 136, 145, 175, 140, 109, 12, 161,
  224, 253, 140, 214, 208, 153, 20, 186, 245, 193, 29, 30, 86, 12, 89, 138,
  44, 90, 74, 226, 68, 231, 2, 190, 42, 50, 189, 24, 168, 194, 17, 222,
  210, 149, 130, 56, 250, 102, 233, 1, 35, 52, 95, 102, 107, 105, 197, 243,
  235, 149, 106, 181, 136, 20, 109, 24, 21, 225, 246, 111, 230, 81, 154, 7,
  36, 55, 61, 239, 174, 172, 245, 58, 43, 226, 56, 103, 119, 165, 187, 222,
  193, 209, 229, 149, 246, 42, 25, 129, 30, 195, 161, 98, 233, 235, 145, 101,
  180, 171, 213, 149, 156, 160, 136, 254, 1, 200, 32, 10, 71, 36, 200, 212,
  143, 230, 215, 130, 35, 24, 235, 185, 208, 127, 133, 240, 50, 191, 126, 154,
  228, 74, 73, 89, 102, 33, 123, 218, 71, 133, 183, 18, 75, 45, 112, 130,
  83, 206, 83, 82, 225, 38, 152, 140, 143, 177, 221, 135, 229, 8, 194, 250,
  34, 164, 54, 155, 141, 134, 95, 154, 214, 8, 230, 63, 19, 177, 29, 31,
  63, 131, 221, 231, 127, 252, 63, 147, 36, 94, 157, 252, 143, 255, 123, 53,
  91, 134, 230, 176, 151, 191, 89, 162, 227, 211, 90, 70, 117, 30, 52, 68,
  36, 10, 224, 161, 51, 118, 145, 167, 243, 73, 95, 211, 136, 17, 26, 164,
  15, 48, 221, 216, 48, 207, 210, 213, 189, 103, 160, 242, 226, 17, 185, 89,
  252, 25, 176, 77, 180, 54, 84, 238, 174, 191, 1, 250, 183, 37, 190, 47,
  64, 254, 8, 242, 159, 137, 250, 29, 49, 181, 146, 236, 161, 119, 191, 89,
  170, 31, 40, 97, 249, 183, 72, 106, 142, 26, 251, 5, 104, 141, 96, 254,
  51, 17, 219, 15, 147, 115, 118, 54, 159, 158, 99, 184, 225, 25, 70, 225,
  155, 167, 186, 46, 94, 73, 122, 88, 229, 55, 75, 123, 121, 97, 18, 248,
  45, 18, 159, 215, 42, 240, 5, 8, 80, 193, 253, 167, 18, 46, 231, 163,
  124, 196, 230, 32, 97, 178, 194, 164, 81, 73, 122, 178, 155, 191, 89, 242,
  27, 155, 214, 149, 223, 34, 9, 130, 166, 54, 250, 226, 106, 13, 194, 252,
  167, 34, 61, 64, 184, 101, 91, 32, 171, 117, 27, 168, 241, 219, 85, 110,
  104, 78, 127, 179, 36, 103, 219, 168, 190, 0, 197, 33, 200, 127, 38, 130,
  179, 12, 107, 149, 164, 6, 125, 251, 205, 82, 218, 229, 108, 177, 66, 35,
  202, 89, 167, 119, 166, 126, 179, 44, 161, 90, 136, 200, 88, 15, 10, 153,
  83, 128, 186, 207, 137, 199, 69, 70, 80, 147, 94, 84, 208, 146, 14, 143,
  39, 21, 52, 83, 61, 20, 69, 200, 6, 79, 139, 230, 0, 96, 131, 71,
  228, 200, 105, 52, 200, 125, 59, 75, 134, 64, 193, 176, 206, 54, 239, 235,
  54, 232, 69, 210, 69, 243, 52, 76, 207, 89, 126, 55, 52, 93, 40, 114,
  198, 5, 238, 186, 131, 9, 198, 172, 174, 234, 114, 9, 57, 45, 166, 50,
  108, 247, 101, 242, 20, 120, 131, 129, 189, 8, 170, 80, 118, 70, 124, 193,
  162, 41, 11, 160, 22, 218, 40, 127, 33, 150, 137, 39, 137, 3, 126, 184,
  88, 106, 180, 228, 204, 18, 187, 18, 94, 158, 239, 227, 135, 6, 112, 188,
  38, 177, 215, 92, 101, 129, 187, 253, 145, 129, 216, 158, 78, 131, 159, 255,
  207, 160, 222, 91, 31, 3, 75, 25, 175, 249, 79, 253, 208, 38, 234, 112,
  222, 61, 105, 245, 61, 154, 95, 247, 149, 205, 87, 51, 28, 0, 42, 71,
  202, 90, 230, 103, 65, 6, 28, 210, 92, 203, 32, 217, 39, 39, 149, 144,
  112, 71, 46, 3, 228, 28, 246, 85, 66, 122, 198, 229, 202, 50, 96, 207,
  45, 11, 74, 37, 44, 224, 221, 101, 112, 236, 211, 142, 202, 99, 137, 37,
  201, 193, 63, 149, 218, 46, 164, 136, 218, 241, 255, 55, 98, 5, 85, 58,
  17, 155, 84, 97, 83, 90, 31, 23, 235, 27, 228, 114, 243, 145, 73, 93,
  166, 51, 4, 15, 6, 64, 91, 178, 229, 15, 113, 135, 23, 69, 28, 175,
  60, 139, 103, 17, 252, 11, 203, 135, 165, 240, 163, 40, 199, 161, 202, 213,
  222, 221, 25, 77, 119, 1, 174, 71, 158, 42, 237, 216, 231, 97, 224, 169,
  86, 200, 255, 154, 114, 179, 100, 93, 218, 90, 129, 214, 150, 44, 78, 86,
  22, 90, 122, 75, 86, 32, 11, 56, 44, 108, 127, 113, 215, 239, 1, 211,
  108, 255, 149, 114, 23, 47, 221, 183, 235, 158, 222, 237, 85, 113, 239, 180,
  184, 223, 174, 40, 137, 100, 128, 7, 74, 36, 8, 88, 59, 174, 126, 33,
  197, 179, 1, 57, 212, 252, 58, 156, 158, 179, 95, 142, 152, 9, 124, 53,
  45, 27, 24, 40, 95, 184, 226, 34, 200, 34, 210, 133, 45, 97, 101, 15,
  254, 89, 146, 44, 214, 58, 218, 96, 145, 219, 89, 138, 8, 172, 236, 117,
  95, 44, 9, 97, 187, 12, 194, 218, 178, 16, 48, 192, 181, 31, 196, 230,
  139, 207, 32, 87, 207, 244, 238, 227, 64, 254, 114, 211, 251, 44, 76, 207,
  22, 76, 175, 129, 129, 119, 122, 239, 189, 20, 149, 27, 31, 45, 132, 151,
  9, 136, 35, 108, 197, 212, 50, 2, 150, 166, 126, 161, 104, 193, 165, 19,
  77, 4, 89, 119, 22, 161, 233, 128, 168, 181, 174, 249, 34, 6, 194, 235,
  50, 248, 251, 191, 253, 111, 193, 79, 32, 21, 178, 32, 78, 46, 66, 80,
  28, 137, 86, 29, 111, 67, 227, 116, 25, 234, 191, 6, 22, 127, 179, 216,
  191, 208, 184, 19, 165, 163, 78, 123, 26, 1, 241, 28, 199, 151, 243, 53,
  75, 51, 85, 211, 88, 166, 100, 46, 179, 205, 91, 98, 153, 79, 179, 174,
  222, 159, 165, 36, 40, 79, 137, 130, 171, 159, 255, 54, 142, 81, 150, 173,
  220, 171, 167, 164, 234, 29, 29, 60, 13, 190, 15, 167, 97, 236, 91, 47,
  38, 181, 78, 93, 221, 208, 90, 185, 163, 121, 28, 127, 200, 212, 161, 212,
  155, 36, 142, 229, 87, 201, 194, 53, 235, 211, 45, 196, 149, 189, 183, 248,
  103, 169, 10, 103, 113, 146, 64, 67, 79, 226, 159, 255, 35, 103, 23, 188,
  27, 30, 231, 63, 223, 18, 210, 40, 42, 11, 47, 81, 109, 228, 253, 131,
  81, 140, 6, 99, 144, 168, 189, 14, 169, 218, 16, 114, 165, 22, 29, 102,
  221, 217, 48, 195, 113, 243, 8, 81, 43, 123, 199, 163, 56, 156, 254, 196,
  166, 176, 248, 134, 17, 204, 83, 29, 246, 247, 198, 130, 113, 87, 173, 84,
  142, 67, 198, 216, 16, 150, 211, 57, 160, 159, 179, 193, 5, 254, 12, 86,
  131, 147, 159, 255, 54, 161, 159, 245, 110, 240, 54, 129, 78, 53, 150, 26,
  212, 75, 118, 142, 242, 222, 121, 148, 135, 228, 55, 123, 215, 121, 152, 141,
  195, 108, 185, 233, 195, 11, 131, 31, 68, 221, 147, 217, 207, 127, 203, 217,
  25, 65, 184, 207, 36, 134, 179, 89, 124, 83, 58, 33, 91, 196, 165, 30,
  99, 25, 124, 118, 18, 118, 19, 191, 195, 241, 204, 24, 116, 203, 104, 145,
  153, 26, 173, 117, 53, 31, 121, 192, 204, 15, 76, 42, 212, 126, 188, 228,
  205, 141, 112, 227, 108, 125, 52, 240, 129, 241, 120, 223, 191, 136, 166, 81,
  235, 143, 32, 20, 141, 137, 125, 230, 64, 108, 192, 66, 89, 254, 19, 200,
  88, 193, 230, 56, 152, 165, 9, 172, 107, 152, 243, 184, 109, 192, 211, 20,
  211, 223, 85, 48, 28, 229, 230, 190, 191, 52, 251, 121, 27, 181, 158, 69,
  193, 9, 203, 231, 51, 135, 243, 200, 253, 136, 32, 242, 32, 36, 108, 248,
  4, 151, 133, 26, 92, 126, 15, 166, 194, 242, 166, 69, 78, 117, 133, 30,
  219, 101, 158, 238, 88, 168, 1, 55, 2, 171, 240, 215, 176, 138, 30, 210,
  149, 22, 191, 47, 125, 225, 19, 5, 234, 207, 25, 84, 71, 166, 106, 186,
  69, 45, 101, 130, 211, 197, 225, 170, 173, 211, 81, 54, 79, 78, 14, 253,
  218, 38, 34, 126, 146, 69, 195, 197, 135, 249, 58, 184, 195, 227, 82, 96,
  135, 179, 187, 122, 221, 45, 221, 197, 165, 44, 81, 102, 183, 161, 124, 24,
  23, 184, 46, 52, 5, 170, 240, 60, 197, 180, 62, 193, 175, 101, 59, 162,
  241, 17, 30, 23, 228, 45, 64, 120, 150, 164, 147, 37, 68, 12, 135, 203,
  188, 64, 159, 111, 54, 103, 147, 224, 37, 172, 199, 43, 150, 94, 4, 176,
  110, 207, 34, 78, 62, 30, 166, 227, 53, 75, 203, 126, 188, 150, 81, 123,
  142, 201, 54, 107, 46, 151, 192, 138, 233, 243, 43, 44, 159, 173, 179, 225,
  104, 107, 103, 153, 245, 130, 29, 110, 183, 219, 203, 175, 152, 153, 219, 105,
  18, 25, 125, 252, 239, 4, 148, 105, 22, 0, 86, 87, 252, 164, 161, 77,
  190, 242, 22, 235, 44, 39, 81, 41, 195, 187, 67, 161, 145, 130, 26, 220,
  87, 51, 224, 76, 75, 92, 102, 177, 209, 9, 126, 254, 239, 163, 209, 212,
  63, 235, 165, 141, 9, 209, 214, 7, 255, 32, 5, 186, 202, 145, 154, 50,
  236, 126, 9, 92, 255, 208, 186, 99, 232, 142, 246, 179, 48, 142, 113, 178,
  87, 246, 240, 87, 22, 112, 195, 168, 152, 203, 57, 6, 225, 144, 132, 12,
  219, 205, 96, 28, 76, 128, 210, 135, 64, 231, 196, 244, 91, 143, 143, 213,
  244, 23, 23, 203, 90, 98, 67, 16, 25, 109, 123, 115, 171, 164, 125, 90,
  129, 101, 7, 54, 230, 173, 28, 219, 84, 78, 81, 240, 122, 238, 244, 106,
  2, 92, 70, 252, 19, 89, 172, 71, 6, 179, 229, 48, 94, 216, 39, 132,
  44, 228, 40, 25, 104, 6, 200, 79, 86, 12, 220, 186, 230, 115, 155, 61,
  206, 59, 36, 199, 96, 65, 249, 12, 87, 154, 45, 102, 48, 54, 32, 105,
  194, 191, 87, 73, 154, 251, 132, 75, 122, 24, 148, 251, 132, 97, 217, 0,
  159, 237, 230, 191, 161, 6, 236, 252, 32, 178, 14, 216, 56, 137, 135, 12,
  192, 189, 61, 122, 252, 50, 144, 208, 212, 179, 218, 75, 77, 6, 71, 202,
  234, 178, 27, 2, 100, 115, 233, 29, 130, 35, 206, 209, 133, 177, 25, 92,
  156, 37, 215, 156, 136, 145, 173, 69, 131, 195, 217, 41, 241, 110, 255, 218,
  9, 78, 168, 80, 112, 120, 108, 205, 157, 59, 66, 85, 199, 132, 24, 93,
  70, 136, 42, 70, 211, 175, 221, 1, 40, 6, 58, 154, 89, 195, 122, 120,
  28, 212, 185, 136, 27, 198, 13, 107, 88, 141, 154, 231, 87, 86, 205, 63,
  130, 200, 124, 21, 222, 44, 89, 61, 155, 90, 213, 79, 230, 103, 83, 150,
  151, 215, 174, 188, 214, 135, 122, 18, 17, 242, 221, 183, 56, 131, 209, 101,
  44, 55, 214, 131, 181, 131, 122, 94, 199, 150, 23, 221, 71, 35, 152, 201,
  183, 208, 92, 6, 156, 41, 203, 89, 28, 3, 87, 42, 105, 178, 216, 77,
  150, 22, 227, 183, 188, 98, 252, 242, 39, 59, 70, 5, 175, 63, 123, 102,
  93, 15, 192, 196, 10, 55, 246, 59, 40, 255, 234, 118, 129, 71, 244, 150,
  29, 47, 130, 103, 139, 99, 38, 114, 163, 151, 87, 22, 74, 206, 199, 75,
  180, 219, 149, 114, 134, 23, 14, 135, 28, 230, 147, 124, 234, 95, 140, 223,
  5, 226, 218, 49, 108, 68, 63, 205, 71, 63, 255, 199, 249, 82, 172, 238,
  78, 67, 61, 48, 174, 81, 124, 177, 129, 230, 230, 171, 250, 75, 16, 76,
  168, 133, 224, 45, 59, 123, 250, 248, 77, 51, 120, 126, 122, 122, 188, 138,
  255, 156, 52, 170, 38, 161, 8, 126, 232, 217, 189, 74, 182, 43, 159, 105,
  210, 195, 50, 165, 217, 28, 102, 178, 156, 87, 82, 251, 7, 83, 122, 234,
  184, 132, 83, 242, 46, 30, 1, 51, 69, 227, 1, 221, 158, 141, 24, 209,
  150, 127, 155, 116, 122, 167, 197, 88, 211, 26, 221, 215, 82, 171, 207, 213,
  203, 98, 3, 250, 15, 217, 229, 1, 102, 130, 179, 36, 78, 113, 125, 34,
  103, 137, 41, 81, 67, 75, 238, 31, 186, 164, 23, 148, 133, 14, 244, 15,
  222, 223, 255, 247, 255, 40, 147, 246, 170, 79, 176, 75, 187, 46, 130, 212,
  249, 7, 242, 68, 100, 250, 7, 198, 52, 196, 58, 181, 184, 61, 246, 0,
  38, 153, 197, 208, 165, 220, 47, 210, 123, 7, 237, 96, 24, 229, 203, 72,
  196, 79, 24, 218, 192, 163, 188, 92, 6, 190, 231, 168, 144, 67, 139, 183,
  115, 79, 124, 174, 46, 142, 253, 19, 43, 253, 144, 198, 184, 155, 224, 242,
  13, 158, 132, 25, 11, 126, 120, 125, 84, 66, 227, 214, 190, 170, 170, 155,
  187, 235, 56, 207, 103, 253, 213, 85, 28, 142, 213, 148, 77, 146, 156, 181,
  103, 227, 217, 234, 48, 188, 92, 29, 69, 49, 203, 86, 231, 25, 75, 87,
  157, 13, 187, 4, 189, 227, 52, 201, 19, 216, 249, 64, 156, 195, 95, 23,
  73, 28, 87, 97, 167, 27, 107, 205, 234, 126, 119, 13, 235, 172, 18, 80,
  95, 217, 195, 127, 131, 58, 236, 151, 211, 33, 48, 196, 70, 217, 129, 77,
  41, 132, 140, 131, 200, 170, 42, 250, 101, 232, 42, 61, 197, 60, 141, 166,
  24, 94, 1, 231, 115, 207, 163, 233, 21, 139, 178, 126, 64, 173, 6, 44,
  29, 81, 68, 152, 60, 24, 205, 167, 23, 136, 0, 241, 173, 33, 203, 130,
  63, 67, 42, 136, 2, 23, 33, 106, 44, 248, 166, 184, 100, 223, 245, 19,
  22, 143, 90, 104, 113, 0, 189, 55, 154, 4, 40, 240, 38, 23, 205, 0,
  198, 57, 165, 162, 7, 39, 199, 193, 44, 253, 249, 63, 70, 32, 242, 204,
  211, 128, 225, 114, 201, 6, 227, 244, 231, 191, 65, 19, 141, 182, 163, 114,
  150, 17, 91, 134, 252, 11, 255, 157, 134, 19, 118, 23, 50, 195, 138, 38,
  157, 21, 216, 75, 120, 75, 211, 20, 169, 7, 143, 103, 179, 150, 20, 234,
  131, 213, 224, 52, 185, 40, 101, 238, 62, 140, 142, 23, 235, 13, 122, 11,
  203, 226, 6, 138, 47, 46, 97, 140, 44, 160, 126, 6, 117, 124, 43, 62,
  103, 141, 187, 80, 190, 6, 104, 25, 194, 237, 174, 236, 117, 233, 73, 250,
  252, 78, 244, 14, 202, 101, 143, 87, 99, 119, 170, 183, 182, 178, 183, 118,
  159, 122, 235, 43, 123, 235, 247, 169, 183, 177, 178, 183, 113, 159, 122, 155,
  43, 123, 155, 75, 212, 171, 92, 206, 247, 18, 95, 236, 208, 157, 139, 197,
  153, 99, 150, 102, 81, 217, 22, 20, 104, 130, 65, 48, 12, 231, 44, 29,
  135, 176, 150, 179, 226, 156, 167, 156, 178, 22, 27, 56, 187, 94, 255, 145,
  146, 93, 243, 36, 196, 75, 171, 149, 39, 76, 21, 181, 79, 89, 102, 239,
  185, 40, 246, 15, 89, 128, 57, 85, 192, 202, 61, 237, 236, 54, 158, 37,
  134, 235, 159, 185, 179, 59, 7, 30, 188, 116, 160, 237, 203, 119, 220, 229,
  23, 26, 118, 239, 56, 234, 238, 136, 231, 116, 58, 75, 127, 2, 180, 115,
  165, 67, 138, 85, 84, 138, 165, 11, 32, 153, 57, 131, 14, 105, 247, 2,
  246, 44, 154, 70, 217, 216, 6, 135, 198, 178, 224, 140, 177, 50, 229, 185,
  124, 160, 102, 70, 71, 65, 169, 125, 145, 157, 223, 87, 205, 45, 181, 240,
  123, 90, 45, 177, 37, 138, 205, 33, 31, 63, 47, 219, 182, 161, 217, 31,
  102, 113, 18, 14, 179, 32, 14, 177, 191, 65, 52, 13, 86, 11, 107, 225,
  234, 31, 226, 124, 103, 200, 46, 163, 1, 59, 28, 254, 225, 60, 223, 89,
  245, 108, 175, 174, 96, 88, 98, 214, 246, 21, 197, 160, 236, 34, 72, 129,
  80, 229, 24, 215, 133, 84, 136, 2, 30, 32, 205, 86, 149, 112, 124, 85,
  44, 164, 138, 83, 137, 69, 109, 190, 158, 79, 49, 200, 75, 89, 107, 34,
  251, 139, 181, 86, 16, 105, 73, 123, 178, 192, 151, 106, 81, 157, 238, 149,
  181, 168, 10, 124, 129, 198, 254, 167, 57, 155, 151, 14, 37, 101, 158, 68,
  63, 65, 175, 58, 159, 215, 204, 179, 48, 138, 231, 41, 158, 24, 251, 91,
  146, 249, 11, 27, 90, 200, 238, 12, 114, 46, 93, 185, 119, 68, 255, 40,
  4, 45, 118, 78, 203, 174, 172, 7, 88, 132, 47, 204, 47, 48, 43, 212,
  94, 78, 91, 82, 121, 107, 180, 153, 125, 70, 91, 234, 32, 7, 131, 15,
  243, 48, 166, 253, 128, 20, 222, 222, 142, 129, 200, 12, 248, 81, 25, 34,
  188, 203, 200, 177, 20, 60, 188, 144, 208, 34, 225, 164, 79, 15, 200, 236,
  252, 210, 72, 210, 194, 43, 67, 144, 50, 95, 179, 48, 75, 166, 255, 56,
  12, 105, 24, 41, 236, 71, 213, 132, 82, 188, 143, 207, 66, 178, 108, 143,
  91, 164, 17, 106, 43, 227, 96, 50, 27, 37, 232, 251, 212, 215, 52, 60,
  67, 219, 153, 206, 243, 159, 216, 180, 41, 130, 114, 13, 195, 44, 120, 30,
  206, 103, 185, 80, 99, 242, 246, 226, 93, 241, 142, 54, 201, 5, 78, 221,
  101, 102, 70, 16, 90, 179, 160, 206, 29, 58, 130, 77, 199, 173, 223, 176,
  167, 143, 128, 247, 140, 177, 2, 104, 118, 23, 249, 28, 134, 61, 19, 246,
  186, 170, 72, 51, 24, 141, 22, 201, 159, 87, 124, 42, 190, 60, 117, 102,
  169, 242, 13, 137, 147, 115, 37, 121, 195, 239, 39, 240, 27, 229, 136, 180,
  226, 154, 67, 249, 37, 135, 25, 200, 100, 83, 150, 146, 135, 73, 201, 205,
  140, 197, 227, 116, 204, 129, 4, 127, 8, 78, 230, 51, 140, 64, 143, 145,
  107, 156, 27, 16, 90, 115, 50, 116, 205, 82, 190, 203, 149, 70, 111, 227,
  36, 141, 96, 31, 218, 106, 240, 225, 83, 251, 124, 197, 169, 243, 146, 180,
  119, 83, 189, 183, 21, 122, 27, 91, 7, 200, 83, 150, 13, 44, 32, 79,
  200, 72, 193, 34, 188, 216, 109, 2, 115, 106, 187, 134, 172, 31, 94, 31,
  149, 157, 17, 185, 213, 129, 128, 18, 171, 62, 38, 149, 2, 184, 179, 82,
  184, 105, 42, 133, 11, 77, 220, 2, 47, 101, 228, 166, 92, 54, 244, 105,
  135, 128, 88, 240, 152, 135, 135, 180, 93, 106, 173, 131, 47, 65, 104, 5,
  197, 101, 101, 106, 156, 197, 17, 30, 174, 22, 225, 141, 30, 142, 146, 4,
  122, 69, 34, 63, 250, 195, 167, 73, 28, 3, 40, 30, 120, 149, 12, 111,
  103, 81, 140, 65, 221, 190, 11, 14, 167, 25, 61, 180, 242, 195, 108, 136,
  246, 143, 224, 239, 255, 246, 95, 197, 161, 201, 57, 27, 255, 252, 183, 121,
  198, 168, 218, 195, 85, 1, 242, 119, 102, 212, 39, 16, 162, 233, 93, 198,
  21, 95, 120, 192, 129, 190, 228, 138, 64, 197, 235, 20, 12, 191, 120, 106,
  113, 153, 245, 167, 226, 119, 26, 231, 4, 198, 2, 212, 236, 60, 128, 213,
  62, 197, 77, 197, 127, 221, 33, 47, 72, 75, 22, 172, 182, 50, 97, 227,
  188, 216, 106, 249, 90, 46, 117, 24, 196, 195, 47, 24, 90, 12, 84, 186,
  247, 88, 67, 190, 250, 248, 114, 16, 78, 7, 44, 118, 163, 155, 238, 61,
  62, 59, 75, 153, 255, 204, 222, 94, 191, 51, 51, 166, 233, 61, 181, 54,
  61, 130, 150, 182, 231, 20, 206, 223, 234, 113, 206, 146, 91, 68, 197, 251,
  132, 43, 86, 5, 215, 235, 79, 140, 128, 89, 19, 159, 225, 211, 170, 238,
  211, 247, 222, 31, 80, 131, 201, 118, 188, 3, 185, 244, 29, 38, 159, 107,
  135, 137, 227, 41, 191, 80, 251, 148, 229, 32, 126, 227, 19, 69, 176, 161,
  251, 84, 216, 178, 8, 183, 10, 208, 29, 194, 220, 186, 163, 135, 111, 33,
  174, 136, 233, 68, 68, 78, 6, 201, 140, 157, 98, 98, 25, 1, 65, 13,
  12, 157, 172, 102, 92, 198, 50, 195, 196, 69, 196, 135, 117, 65, 214, 218,
  28, 67, 113, 41, 26, 44, 89, 135, 46, 130, 136, 74, 228, 99, 191, 84,
  173, 178, 251, 32, 37, 251, 53, 222, 18, 89, 10, 238, 246, 221, 224, 174,
  45, 11, 183, 244, 238, 72, 9, 224, 205, 23, 119, 242, 255, 215, 101, 131,
  59, 93, 92, 170, 190, 4, 181, 82, 230, 205, 195, 105, 234, 31, 122, 77,
  197, 131, 194, 253, 239, 169, 24, 27, 20, 194, 189, 203, 253, 44, 223, 188,
  72, 190, 33, 223, 37, 72, 89, 140, 78, 235, 172, 60, 144, 184, 189, 108,
  47, 103, 195, 55, 17, 187, 162, 37, 187, 72, 10, 245, 16, 221, 115, 22,
  230, 147, 112, 230, 172, 102, 145, 94, 226, 161, 103, 194, 192, 8, 191, 120,
  25, 134, 199, 11, 190, 148, 55, 45, 151, 241, 252, 50, 47, 181, 43, 100,
  12, 73, 135, 63, 107, 35, 222, 126, 233, 109, 26, 15, 208, 136, 30, 250,
  174, 187, 235, 176, 5, 17, 240, 203, 246, 139, 128, 151, 128, 51, 56, 248,
  1, 190, 13, 106, 77, 59, 227, 105, 254, 57, 248, 190, 226, 142, 182, 9,
  250, 41, 59, 155, 159, 27, 126, 163, 176, 209, 182, 166, 32, 36, 149, 92,
  117, 42, 121, 69, 150, 214, 49, 61, 128, 38, 68, 255, 133, 126, 97, 197,
  219, 162, 159, 115, 249, 209, 177, 178, 251, 55, 194, 155, 199, 208, 218, 17,
  15, 212, 250, 150, 165, 121, 153, 127, 54, 150, 189, 214, 202, 34, 137, 221,
  55, 130, 170, 254, 211, 24, 243, 83, 254, 186, 132, 53, 161, 242, 205, 9,
  233, 47, 7, 210, 46, 154, 6, 68, 217, 61, 175, 208, 162, 63, 164, 236,
  149, 93, 117, 161, 101, 153, 213, 191, 72, 114, 209, 27, 92, 36, 192, 44,
  240, 190, 50, 189, 153, 202, 174, 225, 235, 143, 5, 11, 110, 193, 163, 153,
  96, 10, 157, 70, 150, 108, 6, 28, 211, 125, 88, 2, 231, 73, 122, 35,
  152, 241, 222, 247, 252, 59, 98, 11, 46, 18, 249, 107, 123, 185, 182, 211,
  230, 41, 8, 225, 178, 6, 239, 234, 82, 141, 233, 213, 60, 145, 212, 203,
  183, 89, 109, 136, 140, 177, 41, 15, 11, 200, 141, 89, 156, 85, 141, 194,
  121, 156, 31, 71, 211, 236, 112, 58, 74, 232, 236, 135, 188, 25, 102, 144,
  18, 92, 177, 20, 79, 29, 96, 130, 230, 32, 9, 181, 23, 45, 233, 165,
  93, 180, 246, 78, 158, 62, 46, 115, 143, 42, 180, 26, 232, 215, 201, 48,
  148, 74, 205, 116, 62, 57, 67, 217, 119, 18, 77, 119, 87, 58, 43, 248,
  98, 204, 238, 202, 218, 246, 10, 50, 103, 210, 92, 29, 199, 206, 37, 15,
  104, 4, 70, 251, 71, 203, 97, 52, 136, 127, 37, 140, 94, 255, 105, 41,
  132, 94, 95, 255, 74, 248, 156, 46, 135, 207, 233, 23, 196, 167, 210, 191,
  54, 28, 94, 162, 114, 57, 68, 218, 37, 215, 201, 197, 254, 85, 143, 69,
  149, 0, 234, 4, 251, 201, 116, 20, 157, 151, 56, 195, 154, 62, 160, 67,
  50, 24, 217, 141, 190, 165, 55, 12, 252, 27, 240, 219, 40, 189, 136, 209,
  233, 31, 10, 194, 144, 177, 243, 249, 244, 60, 248, 249, 111, 83, 80, 223,
  166, 143, 64, 134, 140, 51, 188, 144, 129, 80, 130, 139, 159, 255, 251, 116,
  138, 135, 237, 240, 159, 224, 140, 243, 233, 89, 26, 206, 7, 99, 124, 63,
  112, 130, 215, 210, 166, 246, 181, 180, 59, 51, 131, 181, 21, 171, 135, 110,
  208, 138, 39, 81, 14, 218, 14, 58, 240, 48, 124, 123, 48, 14, 194, 121,
  214, 2, 92, 134, 1, 119, 31, 30, 92, 0, 134, 245, 151, 12, 54, 105,
  216, 175, 26, 77, 0, 143, 87, 26, 168, 19, 100, 206, 16, 252, 194, 185,
  181, 96, 186, 51, 231, 116, 195, 249, 106, 153, 249, 146, 109, 5, 127, 69,
  77, 236, 222, 186, 71, 181, 93, 82, 195, 14, 6, 239, 73, 168, 221, 178,
  19, 182, 138, 63, 207, 211, 159, 255, 99, 112, 177, 72, 169, 130, 218, 47,
  201, 71, 240, 45, 186, 240, 165, 75, 20, 39, 15, 135, 18, 10, 170, 240,
  123, 240, 27, 72, 16, 222, 189, 93, 183, 203, 172, 35, 217, 32, 141, 102,
  106, 27, 2, 185, 76, 30, 110, 188, 96, 89, 22, 158, 179, 44, 216, 13,
  166, 236, 10, 239, 180, 212, 27, 59, 110, 49, 30, 73, 29, 10, 13, 147,
  193, 124, 2, 50, 91, 251, 156, 229, 7, 49, 195, 159, 79, 110, 14, 135,
  245, 154, 86, 172, 230, 131, 128, 22, 232, 133, 245, 177, 144, 93, 91, 139,
  171, 14, 236, 161, 10, 132, 86, 18, 129, 8, 40, 163, 249, 148, 155, 226,
  201, 39, 78, 228, 215, 245, 199, 175, 162, 81, 80, 255, 74, 239, 230, 127,
  254, 207, 193, 87, 10, 159, 6, 84, 204, 231, 233, 116, 71, 149, 87, 89,
  237, 8, 139, 63, 63, 125, 113, 4, 104, 213, 106, 86, 9, 57, 180, 109,
  144, 41, 14, 96, 245, 215, 39, 217, 121, 176, 187, 167, 53, 44, 187, 24,
  71, 122, 183, 6, 41, 40, 52, 76, 244, 172, 94, 139, 163, 98, 68, 240,
  255, 226, 168, 141, 231, 76, 251, 92, 182, 134, 138, 0, 86, 207, 47, 176,
  11, 103, 51, 232, 242, 254, 56, 138, 135, 245, 56, 210, 128, 220, 54, 44,
  84, 121, 191, 219, 68, 96, 109, 65, 192, 0, 217, 236, 6, 42, 211, 193,
  163, 160, 134, 202, 119, 45, 232, 7, 53, 164, 112, 213, 233, 91, 103, 188,
  103, 243, 108, 76, 163, 93, 159, 112, 16, 206, 152, 171, 116, 239, 8, 171,
  134, 195, 225, 80, 129, 40, 202, 224, 208, 129, 104, 221, 198, 39, 104, 60,
  217, 230, 108, 151, 99, 169, 17, 77, 125, 18, 230, 131, 241, 113, 202, 70,
  209, 53, 174, 135, 121, 28, 219, 40, 27, 37, 118, 221, 50, 54, 234, 4,
  189, 174, 15, 125, 192, 232, 29, 52, 173, 198, 227, 52, 13, 111, 218, 163,
  52, 153, 212, 141, 202, 141, 42, 194, 17, 232, 100, 231, 109, 226, 172, 217,
  219, 40, 31, 235, 216, 53, 26, 22, 38, 67, 216, 189, 114, 134, 53, 12,
  106, 210, 73, 225, 118, 217, 193, 195, 166, 205, 101, 169, 143, 129, 153, 131,
  179, 119, 128, 15, 37, 34, 73, 50, 160, 50, 92, 167, 209, 224, 162, 214,
  12, 96, 17, 66, 175, 244, 9, 104, 20, 109, 217, 227, 84, 80, 83, 237,
  69, 148, 101, 232, 163, 255, 244, 213, 11, 40, 69, 203, 164, 31, 88, 139,
  223, 194, 248, 42, 154, 14, 147, 43, 15, 46, 52, 72, 136, 11, 61, 230,
  216, 48, 135, 153, 175, 78, 26, 253, 128, 242, 31, 181, 5, 161, 33, 139,
  168, 253, 48, 29, 132, 115, 60, 173, 248, 151, 19, 62, 216, 26, 7, 40,
  208, 253, 8, 185, 244, 171, 31, 124, 243, 9, 96, 221, 126, 44, 208, 43,
  184, 84, 41, 130, 243, 233, 24, 36, 121, 144, 178, 82, 246, 87, 126, 192,
  82, 141, 109, 74, 231, 230, 5, 194, 252, 123, 199, 219, 167, 186, 40, 252,
  135, 63, 136, 106, 109, 181, 34, 31, 89, 41, 176, 222, 235, 40, 14, 38,
  35, 213, 2, 144, 127, 45, 203, 241, 21, 188, 154, 42, 142, 108, 225, 7,
  137, 49, 94, 199, 159, 68, 48, 139, 5, 234, 141, 178, 17, 58, 230, 69,
  43, 199, 72, 173, 89, 98, 252, 245, 104, 216, 112, 122, 15, 26, 92, 249,
  6, 1, 21, 118, 44, 166, 15, 139, 87, 195, 195, 75, 88, 223, 124, 138,
  134, 26, 62, 129, 96, 86, 80, 160, 156, 169, 100, 140, 238, 240, 66, 139,
  77, 238, 88, 90, 130, 169, 234, 136, 137, 23, 162, 197, 98, 139, 205, 19,
  156, 202, 38, 223, 96, 137, 207, 108, 147, 191, 99, 187, 76, 107, 79, 249,
  54, 193, 219, 139, 178, 232, 12, 223, 124, 20, 123, 199, 9, 110, 36, 184,
  39, 82, 16, 144, 218, 189, 48, 177, 55, 35, 209, 6, 144, 154, 209, 72,
  233, 62, 196, 27, 58, 61, 60, 58, 248, 240, 244, 224, 217, 9, 64, 120,
  167, 154, 249, 4, 146, 86, 191, 22, 207, 175, 97, 45, 145, 46, 212, 175,
  209, 51, 31, 240, 57, 159, 70, 57, 124, 81, 150, 208, 170, 223, 240, 150,
  251, 121, 58, 103, 42, 241, 21, 127, 38, 62, 184, 109, 90, 80, 241, 205,
  129, 2, 44, 62, 232, 160, 160, 106, 239, 58, 44, 5, 189, 235, 66, 31,
  36, 189, 2, 56, 133, 62, 83, 208, 103, 179, 201, 82, 80, 123, 46, 84,
  12, 245, 86, 128, 149, 145, 209, 11, 188, 255, 219, 254, 82, 144, 215, 92,
  200, 50, 26, 156, 54, 208, 69, 84, 56, 213, 192, 239, 151, 2, 191, 238,
  130, 199, 168, 191, 26, 104, 25, 210, 248, 206, 152, 111, 184, 160, 47, 103,
  218, 52, 190, 57, 46, 102, 241, 226, 56, 92, 10, 230, 102, 1, 243, 253,
  142, 75, 148, 111, 14, 79, 14, 159, 28, 30, 29, 158, 254, 248, 225, 251,
  131, 31, 113, 177, 96, 120, 216, 15, 68, 230, 116, 28, 254, 225, 178, 211,
  219, 84, 116, 13, 187, 55, 133, 222, 125, 163, 242, 161, 202, 167, 91, 19,
  48, 22, 160, 198, 33, 79, 81, 126, 123, 18, 206, 234, 57, 238, 21, 32,
  176, 14, 27, 30, 84, 14, 126, 60, 248, 240, 226, 241, 235, 239, 127, 56,
  134, 122, 31, 77, 219, 225, 95, 120, 212, 90, 118, 195, 254, 34, 12, 2,
  127, 89, 225, 5, 224, 27, 95, 181, 110, 205, 64, 25, 204, 216, 16, 75,
  194, 32, 96, 41, 60, 163, 132, 79, 30, 137, 5, 181, 206, 139, 152, 100,
  81, 89, 131, 6, 213, 95, 192, 8, 11, 252, 151, 149, 44, 109, 37, 211,
  248, 230, 47, 104, 241, 147, 101, 91, 171, 160, 208, 194, 152, 139, 192, 36,
  220, 55, 235, 242, 92, 213, 1, 84, 91, 9, 111, 13, 239, 63, 62, 73,
  174, 33, 17, 159, 204, 237, 173, 195, 255, 75, 28, 184, 121, 86, 33, 189,
  247, 16, 93, 230, 2, 236, 197, 139, 110, 47, 216, 24, 180, 54, 130, 78,
  107, 59, 120, 128, 255, 101, 235, 193, 131, 96, 27, 255, 107, 209, 127, 173,
  117, 248, 15, 254, 254, 52, 193, 167, 123, 195, 141, 96, 3, 223, 240, 133,
  255, 117, 90, 221, 78, 192, 63, 49, 1, 159, 98, 134, 50, 173, 173, 112,
  45, 88, 19, 41, 157, 96, 51, 224, 95, 244, 191, 214, 230, 79, 128, 209,
  40, 138, 113, 60, 6, 243, 20, 36, 47, 30, 30, 237, 47, 43, 171, 104,
  62, 188, 60, 119, 59, 71, 118, 220, 225, 125, 187, 55, 193, 214, 187, 91,
  240, 255, 173, 110, 123, 61, 128, 255, 90, 107, 237, 46, 254, 183, 223, 221,
  8, 122, 157, 54, 228, 162, 95, 70, 15, 58, 212, 131, 127, 197, 72, 240,
  30, 195, 175, 246, 54, 212, 129, 242, 193, 90, 107, 189, 189, 125, 212, 109,
  111, 6, 235, 0, 7, 190, 127, 154, 64, 215, 225, 19, 106, 61, 134, 255,
  68, 23, 187, 52, 70, 197, 55, 252, 26, 64, 249, 78, 208, 126, 208, 106,
  67, 27, 173, 246, 90, 140, 105, 0, 184, 189, 1, 40, 109, 183, 183, 0,
  56, 252, 211, 107, 111, 65, 115, 27, 240, 183, 7, 255, 110, 80, 211, 240,
  179, 219, 126, 208, 198, 196, 141, 159, 38, 107, 237, 109, 196, 123, 128, 227,
  189, 45, 166, 168, 67, 216, 117, 161, 23, 107, 208, 192, 26, 192, 216, 104,
  111, 196, 144, 128, 176, 55, 6, 80, 27, 254, 66, 123, 88, 21, 138, 172,
  181, 16, 8, 37, 226, 24, 172, 183, 54, 224, 239, 3, 250, 183, 141, 189,
  5, 64, 4, 110, 27, 176, 132, 63, 155, 248, 223, 0, 107, 192, 40, 1,
  142, 56, 78, 244, 103, 241, 44, 74, 245, 255, 163, 43, 207, 160, 11, 218,
  169, 177, 182, 13, 253, 20, 151, 62, 58, 227, 12, 141, 21, 31, 4, 121,
  122, 99, 232, 4, 178, 204, 191, 156, 188, 122, 217, 158, 133, 105, 198, 234,
  20, 144, 227, 36, 79, 82, 16, 225, 80, 28, 58, 4, 78, 95, 247, 176,
  159, 6, 9, 180, 159, 110, 107, 244, 67, 111, 228, 54, 24, 160, 86, 17,
  160, 110, 210, 112, 116, 87, 165, 128, 213, 208, 29, 24, 90, 207, 147, 128,
  90, 38, 86, 20, 20, 204, 12, 56, 38, 2, 240, 41, 27, 21, 92, 77,
  114, 41, 20, 55, 56, 107, 69, 169, 21, 37, 142, 145, 8, 218, 1, 223,
  72, 225, 13, 91, 63, 50, 129, 190, 139, 134, 239, 161, 36, 13, 16, 253,
  254, 10, 132, 87, 140, 108, 52, 138, 240, 2, 220, 35, 43, 7, 128, 103,
  40, 89, 168, 70, 84, 162, 134, 127, 241, 179, 224, 180, 82, 97, 3, 192,
  136, 80, 129, 54, 166, 180, 17, 111, 66, 86, 27, 134, 87, 103, 40, 26,
  183, 47, 216, 77, 86, 231, 243, 71, 227, 95, 232, 126, 209, 208, 238, 26,
  73, 174, 162, 222, 12, 175, 59, 34, 87, 110, 143, 195, 236, 213, 213, 244,
  24, 223, 166, 77, 243, 155, 54, 76, 123, 92, 55, 7, 161, 9, 219, 90,
  163, 97, 105, 145, 75, 140, 147, 213, 113, 125, 234, 110, 43, 20, 235, 25,
  191, 158, 84, 65, 215, 54, 253, 26, 196, 154, 149, 19, 107, 147, 83, 56,
  215, 61, 162, 209, 141, 213, 207, 70, 227, 115, 168, 55, 131, 246, 151, 167,
  94, 183, 219, 88, 243, 48, 59, 184, 158, 225, 77, 15, 180, 91, 96, 84,
  94, 199, 242, 33, 210, 241, 34, 136, 141, 188, 61, 37, 188, 228, 123, 65,
  232, 142, 26, 82, 82, 218, 153, 56, 23, 211, 57, 249, 216, 29, 220, 96,
  245, 121, 24, 215, 207, 242, 41, 116, 83, 32, 238, 96, 124, 134, 74, 190,
  109, 167, 17, 230, 43, 58, 57, 219, 85, 117, 209, 72, 228, 108, 237, 100,
  49, 18, 169, 24, 15, 48, 58, 135, 180, 2, 18, 128, 199, 41, 127, 156,
  3, 250, 192, 40, 89, 189, 166, 203, 22, 181, 166, 1, 28, 7, 130, 224,
  81, 15, 117, 165, 178, 4, 12, 97, 40, 165, 185, 202, 226, 36, 187, 56,
  37, 221, 193, 163, 192, 123, 21, 180, 173, 116, 208, 127, 157, 51, 121, 240,
  152, 164, 143, 97, 81, 214, 218, 218, 139, 17, 181, 98, 161, 19, 201, 121,
  205, 131, 130, 86, 118, 105, 174, 219, 120, 24, 0, 56, 11, 40, 59, 54,
  99, 144, 4, 103, 79, 149, 210, 188, 228, 56, 238, 150, 144, 234, 142, 197,
  69, 219, 36, 122, 144, 65, 145, 135, 0, 168, 131, 18, 18, 195, 164, 242,
  121, 249, 74, 81, 140, 94, 209, 166, 45, 2, 100, 12, 5, 140, 131, 20,
  44, 107, 13, 141, 238, 118, 150, 98, 46, 168, 96, 67, 109, 11, 247, 2,
  76, 83, 178, 31, 181, 63, 124, 42, 217, 111, 180, 245, 165, 134, 198, 195,
  245, 188, 243, 109, 234, 173, 162, 197, 70, 25, 231, 211, 76, 31, 41, 195,
  45, 255, 57, 250, 246, 147, 123, 73, 86, 39, 36, 203, 251, 203, 166, 217,
  60, 101, 212, 229, 27, 150, 253, 67, 40, 141, 36, 132, 154, 67, 111, 11,
  102, 214, 183, 223, 180, 163, 105, 6, 155, 212, 227, 225, 95, 67, 116, 255,
  64, 251, 121, 189, 118, 198, 0, 61, 188, 14, 8, 52, 101, 41, 37, 13,
  255, 230, 163, 40, 250, 134, 73, 140, 203, 241, 176, 241, 134, 196, 6, 214,
  180, 59, 185, 27, 216, 235, 202, 67, 134, 62, 121, 77, 22, 178, 103, 74,
  166, 151, 16, 144, 123, 72, 65, 247, 26, 136, 52, 60, 155, 199, 48, 13,
  175, 140, 60, 125, 63, 128, 222, 103, 249, 227, 105, 52, 9, 17, 212, 179,
  52, 156, 176, 58, 183, 177, 58, 213, 170, 151, 214, 124, 70, 133, 159, 147,
  25, 47, 205, 126, 235, 140, 141, 106, 187, 150, 211, 73, 50, 207, 24, 185,
  23, 41, 91, 179, 111, 116, 151, 228, 56, 158, 53, 11, 29, 28, 32, 233,
  17, 171, 112, 109, 92, 211, 228, 10, 50, 209, 121, 171, 13, 63, 109, 110,
  241, 21, 175, 253, 135, 63, 4, 117, 44, 216, 130, 29, 39, 203, 9, 250,
  49, 111, 233, 49, 112, 146, 135, 193, 243, 87, 111, 14, 94, 127, 56, 126,
  125, 112, 244, 234, 241, 211, 15, 135, 47, 79, 15, 94, 191, 121, 124, 228,
  142, 130, 91, 27, 79, 48, 146, 171, 29, 131, 237, 145, 109, 160, 56, 86,
  160, 89, 33, 108, 236, 173, 0, 214, 173, 70, 51, 141, 29, 223, 208, 248,
  8, 101, 199, 180, 190, 113, 7, 105, 238, 81, 39, 237, 125, 53, 74, 116,
  78, 249, 114, 60, 114, 209, 203, 195, 200, 104, 159, 168, 188, 144, 45, 244,
  58, 7, 9, 192, 159, 81, 175, 245, 134, 160, 192, 244, 233, 84, 198, 4,
  47, 30, 69, 134, 38, 180, 211, 22, 121, 224, 169, 70, 166, 97, 33, 245,
  228, 135, 253, 239, 15, 78, 63, 108, 188, 128, 122, 27, 193, 183, 193, 102,
  7, 254, 233, 118, 58, 29, 111, 177, 46, 149, 235, 86, 20, 60, 57, 252,
  211, 135, 231, 31, 94, 160, 45, 114, 83, 150, 42, 41, 250, 244, 241, 143,
  188, 32, 104, 245, 213, 37, 143, 14, 223, 28, 124, 120, 123, 248, 242, 233,
  171, 183, 162, 198, 130, 10, 227, 8, 101, 220, 155, 19, 18, 116, 117, 109,
  75, 12, 146, 73, 31, 122, 233, 119, 19, 220, 38, 63, 5, 147, 104, 216,
  15, 222, 189, 7, 41, 41, 193, 240, 136, 248, 43, 60, 63, 223, 232, 127,
  10, 206, 230, 131, 11, 150, 247, 91, 221, 102, 144, 205, 39, 253, 78, 19,
  218, 156, 79, 115, 178, 147, 98, 153, 238, 194, 66, 217, 13, 250, 101, 244,
  185, 10, 118, 107, 77, 200, 139, 199, 127, 250, 176, 249, 252, 195, 241, 43,
  88, 8, 216, 213, 173, 206, 78, 176, 186, 138, 145, 127, 255, 211, 198, 4,
  100, 233, 239, 0, 246, 104, 196, 82, 167, 82, 111, 93, 171, 213, 237, 242,
  106, 189, 117, 168, 215, 245, 85, 68, 141, 59, 166, 251, 154, 120, 10, 0,
  122, 10, 45, 167, 206, 142, 149, 125, 172, 158, 151, 21, 199, 133, 122, 1,
  60, 123, 5, 253, 4, 203, 188, 102, 217, 32, 140, 21, 171, 176, 193, 56,
  235, 214, 154, 48, 63, 15, 160, 238, 219, 115, 59, 64, 123, 59, 221, 159,
  164, 137, 210, 7, 19, 116, 207, 104, 194, 126, 74, 166, 172, 95, 59, 152,
  163, 158, 184, 250, 132, 165, 113, 132, 231, 73, 108, 150, 12, 198, 175, 217,
  168, 143, 157, 16, 159, 251, 225, 12, 88, 12, 3, 140, 40, 181, 80, 120,
  17, 109, 126, 209, 198, 233, 145, 160, 221, 131, 55, 31, 246, 95, 61, 61,
  128, 220, 21, 188, 241, 218, 237, 173, 173, 127, 189, 98, 17, 33, 241, 81,
  90, 193, 228, 124, 80, 80, 97, 213, 6, 163, 191, 134, 166, 237, 48, 3,
  34, 84, 29, 226, 187, 129, 181, 157, 32, 225, 14, 44, 38, 82, 198, 58,
  44, 162, 211, 220, 128, 109, 86, 166, 101, 213, 252, 181, 136, 173, 185, 16,
  128, 185, 57, 137, 38, 118, 85, 217, 21, 140, 174, 240, 135, 182, 113, 45,
  114, 108, 84, 181, 58, 132, 174, 3, 3, 176, 181, 211, 76, 100, 43, 114,
  171, 113, 45, 30, 36, 51, 16, 45, 146, 61, 184, 82, 166, 68, 84, 3,
  192, 177, 44, 18, 28, 20, 189, 89, 21, 248, 21, 62, 219, 214, 230, 69,
  105, 54, 102, 133, 187, 113, 81, 186, 72, 179, 75, 223, 120, 74, 223, 148,
  150, 150, 161, 157, 139, 178, 50, 197, 46, 41, 215, 56, 95, 55, 69, 121,
  51, 221, 217, 125, 237, 7, 132, 172, 14, 235, 89, 222, 186, 218, 131, 45,
  86, 85, 45, 199, 91, 83, 187, 99, 97, 213, 212, 114, 74, 22, 151, 175,
  170, 147, 101, 215, 181, 94, 201, 41, 106, 90, 25, 254, 54, 237, 74, 122,
  170, 111, 46, 232, 154, 149, 57, 13, 148, 100, 151, 37, 219, 23, 94, 137,
  192, 115, 227, 162, 188, 145, 236, 122, 70, 21, 145, 58, 120, 16, 63, 109,
  4, 181, 60, 111, 189, 31, 210, 248, 144, 252, 61, 205, 42, 144, 236, 45,
  46, 227, 170, 57, 51, 165, 103, 250, 27, 2, 93, 207, 219, 82, 230, 122,
  138, 169, 64, 91, 190, 10, 152, 238, 173, 160, 34, 95, 121, 145, 83, 185,
  254, 198, 132, 174, 238, 27, 62, 145, 231, 173, 167, 133, 222, 179, 106, 105,
  57, 139, 106, 98, 208, 190, 242, 218, 152, 187, 8, 130, 23, 111, 39, 127,
  17, 20, 17, 25, 177, 28, 140, 40, 176, 36, 28, 147, 132, 43, 10, 249,
  9, 122, 24, 229, 220, 209, 207, 32, 101, 72, 245, 22, 231, 209, 152, 220,
  10, 60, 221, 91, 69, 221, 63, 181, 170, 168, 244, 170, 133, 38, 133, 43,
  207, 58, 163, 44, 63, 133, 106, 1, 110, 108, 242, 212, 178, 74, 168, 91,
  15, 86, 227, 16, 183, 158, 233, 159, 31, 25, 122, 198, 158, 18, 153, 238,
  173, 165, 226, 200, 88, 181, 84, 186, 127, 50, 68, 76, 24, 123, 46, 68,
  178, 183, 78, 17, 133, 197, 170, 85, 100, 148, 214, 67, 233, 210, 83, 11,
  147, 253, 188, 72, 133, 62, 177, 153, 145, 202, 40, 165, 24, 17, 145, 196,
  67, 51, 34, 167, 20, 75, 242, 46, 242, 160, 73, 233, 37, 44, 144, 135,
  147, 114, 56, 32, 79, 246, 207, 115, 241, 228, 148, 61, 211, 69, 206, 162,
  154, 190, 165, 107, 230, 150, 113, 224, 244, 198, 93, 131, 148, 236, 31, 22,
  135, 28, 143, 74, 8, 209, 218, 64, 139, 180, 162, 52, 87, 72, 112, 31,
  126, 33, 13, 74, 182, 50, 38, 178, 185, 202, 82, 139, 163, 75, 86, 115,
  179, 223, 240, 75, 145, 88, 98, 44, 36, 101, 189, 16, 185, 45, 210, 61,
  187, 50, 69, 238, 141, 124, 244, 134, 52, 26, 216, 211, 175, 184, 88, 217,
  4, 45, 250, 124, 204, 127, 155, 122, 148, 205, 28, 95, 129, 14, 147, 70,
  67, 175, 170, 40, 223, 6, 192, 119, 25, 124, 249, 234, 45, 5, 148, 23,
  124, 26, 169, 42, 240, 148, 133, 67, 122, 135, 204, 210, 105, 85, 129, 199,
  116, 67, 203, 143, 4, 222, 75, 64, 20, 152, 87, 87, 21, 226, 23, 158,
  243, 120, 252, 83, 94, 28, 156, 190, 62, 220, 255, 0, 127, 30, 99, 110,
  97, 189, 154, 95, 247, 97, 192, 116, 223, 43, 229, 45, 196, 29, 177, 6,
  209, 4, 80, 49, 61, 162, 208, 211, 170, 255, 201, 235, 91, 101, 122, 95,
  249, 171, 15, 146, 94, 209, 40, 119, 158, 178, 252, 170, 84, 61, 195, 205,
  11, 157, 165, 138, 102, 201, 193, 72, 212, 43, 222, 202, 44, 109, 84, 250,
  67, 41, 0, 191, 47, 243, 140, 242, 215, 71, 135, 39, 127, 227, 186, 243,
  147, 191, 46, 168, 64, 170, 42, 119, 98, 50, 188, 155, 84, 37, 205, 141,
  235, 214, 103, 70, 195, 151, 134, 149, 141, 161, 218, 122, 100, 85, 144, 6,
  36, 6, 100, 46, 173, 18, 184, 112, 112, 128, 75, 249, 46, 175, 193, 183,
  88, 89, 41, 229, 187, 166, 250, 148, 219, 160, 76, 24, 200, 205, 77, 38,
  136, 51, 25, 225, 167, 21, 171, 173, 5, 173, 79, 100, 202, 161, 0, 12,
  252, 183, 216, 174, 228, 39, 238, 38, 248, 251, 95, 113, 235, 195, 31, 35,
  177, 157, 201, 2, 220, 187, 183, 86, 211, 190, 78, 230, 163, 81, 116, 173,
  210, 144, 113, 171, 15, 190, 91, 208, 103, 198, 15, 187, 145, 47, 245, 185,
  172, 255, 1, 221, 157, 104, 50, 48, 28, 224, 51, 10, 249, 1, 69, 237,
  137, 56, 125, 125, 240, 242, 233, 135, 253, 87, 47, 159, 29, 254, 209, 88,
  73, 68, 156, 48, 94, 220, 159, 184, 31, 108, 53, 241, 222, 214, 83, 22,
  231, 97, 63, 232, 180, 59, 91, 216, 104, 74, 102, 187, 78, 123, 109, 67,
  155, 245, 192, 79, 167, 229, 176, 214, 11, 80, 221, 118, 57, 36, 177, 204,
  188, 64, 186, 27, 5, 140, 141, 142, 14, 162, 99, 19, 110, 69, 159, 186,
  122, 159, 58, 6, 38, 107, 197, 73, 189, 111, 0, 95, 28, 188, 120, 245,
  250, 71, 15, 163, 34, 5, 244, 40, 188, 73, 230, 121, 230, 201, 38, 91,
  83, 65, 156, 168, 57, 74, 78, 207, 25, 160, 252, 66, 211, 50, 160, 241,
  233, 214, 110, 95, 220, 7, 62, 136, 45, 165, 87, 220, 41, 46, 54, 55,
  117, 0, 1, 66, 243, 133, 208, 195, 195, 201, 44, 102, 234, 204, 51, 207,
  60, 23, 43, 172, 101, 199, 109, 96, 141, 160, 36, 163, 116, 81, 154, 103,
  22, 194, 1, 28, 135, 4, 157, 191, 249, 197, 192, 26, 26, 94, 190, 122,
  73, 191, 219, 81, 246, 50, 124, 89, 7, 140, 204, 115, 191, 146, 102, 219,
  216, 12, 180, 253, 2, 150, 71, 123, 18, 94, 215, 171, 202, 81, 71, 125,
  254, 67, 116, 17, 162, 170, 129, 189, 160, 83, 218, 243, 54, 246, 218, 242,
  173, 184, 181, 24, 17, 12, 55, 186, 29, 93, 134, 49, 17, 94, 19, 111,
  64, 210, 113, 18, 199, 59, 154, 214, 85, 7, 100, 153, 6, 47, 228, 76,
  226, 24, 246, 118, 49, 197, 206, 21, 36, 69, 18, 141, 130, 58, 180, 195,
  119, 188, 5, 83, 227, 110, 125, 250, 5, 7, 251, 244, 110, 156, 92, 201,
  6, 62, 113, 254, 46, 220, 193, 81, 120, 227, 54, 96, 254, 139, 2, 89,
  16, 167, 4, 130, 157, 178, 20, 84, 9, 72, 13, 167, 131, 113, 146, 254,
  73, 254, 248, 17, 248, 178, 77, 92, 26, 158, 246, 41, 84, 129, 183, 126,
  51, 234, 163, 126, 161, 143, 71, 10, 216, 251, 230, 19, 253, 184, 229, 183,
  212, 228, 107, 6, 223, 124, 82, 168, 222, 22, 97, 243, 124, 181, 101, 63,
  4, 128, 143, 62, 28, 184, 7, 249, 25, 5, 71, 39, 131, 16, 218, 128,
  233, 47, 30, 101, 127, 221, 235, 13, 215, 24, 171, 149, 215, 28, 184, 117,
  88, 143, 109, 141, 58, 222, 58, 197, 76, 97, 16, 252, 75, 230, 78, 86,
  113, 69, 131, 140, 20, 198, 200, 35, 120, 101, 249, 198, 48, 255, 104, 162,
  124, 130, 207, 193, 192, 198, 180, 31, 71, 144, 140, 197, 234, 14, 176, 25,
  233, 77, 91, 118, 114, 50, 26, 101, 12, 91, 233, 110, 218, 89, 200, 72,
  241, 114, 38, 208, 194, 236, 109, 83, 70, 177, 192, 175, 231, 193, 45, 46,
  5, 213, 165, 37, 81, 192, 112, 17, 216, 31, 92, 40, 117, 65, 64, 193,
  119, 2, 5, 218, 156, 243, 54, 149, 249, 14, 177, 21, 9, 20, 32, 49,
  104, 17, 18, 240, 7, 50, 28, 184, 24, 112, 216, 4, 251, 35, 175, 240,
  60, 88, 13, 122, 2, 14, 22, 210, 225, 242, 87, 178, 101, 57, 27, 176,
  61, 197, 2, 245, 143, 64, 142, 240, 235, 118, 118, 93, 65, 72, 28, 29,
  40, 10, 63, 244, 146, 206, 26, 228, 172, 229, 121, 152, 33, 183, 241, 28,
  226, 11, 159, 70, 177, 125, 148, 48, 38, 199, 209, 235, 171, 175, 234, 188,
  10, 176, 91, 250, 65, 172, 171, 156, 19, 140, 195, 12, 166, 11, 38, 205,
  218, 46, 248, 46, 250, 2, 183, 181, 205, 14, 252, 223, 103, 96, 70, 236,
  128, 151, 198, 43, 147, 28, 43, 100, 185, 146, 51, 216, 94, 52, 250, 193,
  56, 16, 215, 227, 217, 44, 77, 174, 15, 240, 180, 233, 5, 122, 25, 60,
  122, 228, 61, 46, 23, 176, 248, 49, 121, 209, 72, 240, 112, 87, 117, 166,
  106, 20, 176, 3, 135, 83, 178, 67, 171, 65, 152, 128, 196, 101, 187, 216,
  226, 129, 17, 222, 18, 177, 58, 56, 33, 165, 17, 247, 187, 181, 206, 176,
  134, 29, 45, 82, 182, 157, 20, 12, 197, 84, 107, 120, 124, 29, 46, 216,
  13, 167, 30, 142, 194, 109, 11, 126, 65, 173, 219, 143, 186, 39, 131, 64,
  65, 83, 162, 222, 65, 189, 247, 216, 132, 142, 150, 231, 18, 161, 168, 9,
  131, 122, 194, 210, 136, 233, 164, 39, 122, 235, 191, 231, 71, 35, 139, 149,
  219, 89, 50, 97, 245, 25, 110, 107, 51, 36, 177, 89, 251, 146, 252, 160,
  72, 103, 133, 111, 177, 247, 207, 218, 185, 177, 249, 151, 211, 159, 240, 181,
  224, 34, 203, 43, 10, 138, 232, 185, 118, 235, 28, 47, 148, 57, 25, 134,
  151, 24, 240, 236, 44, 46, 8, 19, 84, 141, 40, 206, 149, 35, 135, 88,
  111, 58, 23, 177, 97, 123, 142, 20, 121, 180, 70, 237, 44, 17, 18, 188,
  14, 183, 144, 222, 22, 215, 170, 202, 92, 235, 112, 239, 5, 236, 20, 166,
  176, 9, 14, 226, 249, 144, 101, 245, 162, 178, 94, 11, 83, 85, 12, 133,
  93, 88, 63, 99, 221, 95, 131, 231, 139, 23, 47, 157, 220, 91, 203, 157,
  164, 104, 52, 102, 211, 243, 124, 140, 66, 15, 9, 102, 30, 108, 220, 113,
  225, 184, 89, 100, 235, 47, 165, 119, 240, 93, 231, 253, 142, 191, 134, 244,
  229, 112, 235, 251, 124, 104, 245, 65, 220, 127, 117, 244, 234, 245, 135, 227,
  199, 71, 7, 167, 167, 7, 158, 75, 91, 131, 155, 16, 79, 172, 241, 249,
  15, 80, 212, 249, 7, 1, 238, 171, 29, 221, 189, 203, 3, 92, 34, 58,
  79, 84, 181, 67, 249, 41, 43, 110, 174, 109, 110, 142, 186, 158, 138, 33,
  17, 185, 172, 247, 88, 124, 201, 106, 163, 141, 109, 214, 57, 243, 84, 99,
  19, 150, 134, 241, 80, 85, 60, 80, 223, 178, 106, 183, 115, 182, 189, 229,
  107, 113, 22, 77, 47, 84, 189, 99, 254, 33, 43, 177, 193, 250, 214, 246,
  182, 167, 82, 202, 138, 182, 94, 39, 185, 94, 133, 158, 146, 243, 84, 137,
  65, 128, 82, 117, 142, 248, 135, 172, 180, 181, 62, 24, 116, 55, 61, 149,
  178, 139, 27, 85, 231, 132, 126, 203, 42, 252, 133, 84, 79, 149, 203, 40,
  1, 230, 170, 106, 189, 161, 79, 29, 195, 112, 107, 99, 99, 244, 192, 215,
  24, 62, 148, 93, 52, 39, 190, 100, 181, 237, 245, 112, 237, 204, 215, 96,
  154, 100, 76, 27, 140, 76, 175, 52, 58, 123, 208, 221, 218, 168, 249, 46,
  192, 133, 177, 170, 116, 202, 63, 212, 84, 173, 159, 109, 133, 250, 104, 88,
  183, 185, 4, 197, 190, 62, 120, 38, 47, 114, 113, 134, 244, 129, 228, 198,
  204, 189, 198, 69, 233, 120, 77, 219, 84, 45, 141, 235, 32, 251, 170, 76,
  165, 203, 188, 16, 40, 67, 228, 59, 222, 187, 30, 38, 114, 142, 27, 36,
  212, 108, 152, 248, 104, 151, 71, 40, 243, 158, 215, 65, 184, 204, 60, 35,
  160, 252, 66, 200, 136, 92, 236, 29, 79, 250, 160, 108, 56, 92, 6, 97,
  95, 45, 168, 26, 35, 255, 101, 2, 115, 52, 156, 123, 4, 5, 38, 13,
  116, 121, 51, 187, 90, 213, 65, 225, 107, 108, 118, 177, 28, 127, 16, 74,
  198, 39, 212, 42, 136, 116, 169, 45, 135, 140, 117, 171, 106, 128, 17, 169,
  40, 36, 118, 16, 81, 58, 252, 121, 136, 230, 14, 193, 231, 225, 251, 187,
  239, 204, 137, 24, 75, 141, 58, 154, 204, 227, 250, 90, 23, 68, 252, 6,
  136, 200, 88, 7, 121, 50, 134, 142, 125, 156, 215, 35, 152, 90, 189, 25,
  71, 40, 32, 16, 225, 89, 86, 31, 151, 239, 241, 52, 8, 207, 146, 244,
  41, 89, 171, 172, 27, 217, 180, 109, 98, 146, 0, 104, 112, 118, 216, 58,
  236, 253, 128, 162, 10, 168, 25, 192, 235, 39, 170, 170, 153, 108, 139, 6,
  209, 16, 157, 65, 180, 65, 197, 70, 127, 111, 182, 39, 135, 203, 238, 165,
  137, 20, 64, 178, 208, 186, 245, 184, 199, 238, 219, 189, 22, 202, 180, 167,
  243, 36, 23, 139, 76, 87, 174, 209, 59, 37, 245, 76, 205, 75, 220, 165,
  242, 234, 137, 248, 158, 221, 40, 145, 79, 62, 39, 82, 98, 26, 34, 209,
  160, 40, 35, 6, 67, 119, 239, 22, 73, 162, 248, 35, 77, 114, 237, 127,
  243, 73, 214, 36, 217, 151, 167, 126, 12, 250, 88, 70, 230, 220, 126, 92,
  72, 52, 92, 54, 16, 131, 88, 133, 183, 46, 64, 151, 119, 212, 36, 36,
  44, 141, 30, 164, 197, 24, 163, 32, 237, 35, 40, 76, 55, 235, 170, 222,
  153, 0, 100, 178, 23, 138, 202, 116, 70, 208, 90, 35, 250, 208, 33, 146,
  106, 4, 27, 203, 144, 92, 229, 152, 121, 200, 240, 62, 67, 247, 149, 64,
  107, 9, 186, 37, 229, 228, 62, 132, 107, 111, 113, 222, 99, 6, 129, 94,
  230, 30, 21, 213, 158, 60, 239, 62, 216, 232, 212, 172, 179, 32, 79, 58,
  89, 159, 107, 47, 158, 255, 185, 187, 93, 179, 142, 112, 130, 218, 201, 243,
  211, 181, 110, 205, 103, 244, 118, 178, 232, 220, 5, 0, 29, 253, 105, 187,
  179, 217, 93, 175, 89, 198, 105, 126, 132, 98, 155, 154, 141, 112, 9, 162,
  55, 106, 244, 163, 210, 181, 137, 99, 95, 240, 205, 98, 92, 173, 97, 209,
  140, 183, 209, 176, 148, 120, 228, 36, 43, 226, 241, 24, 36, 140, 181, 238,
  194, 87, 36, 218, 206, 147, 31, 102, 51, 244, 178, 4, 233, 160, 156, 94,
  233, 170, 131, 2, 243, 44, 77, 38, 202, 143, 182, 46, 244, 212, 79, 183,
  46, 149, 78, 66, 180, 175, 144, 46, 202, 145, 254, 16, 193, 220, 91, 242,
  135, 176, 201, 38, 61, 128, 47, 75, 195, 215, 7, 94, 131, 80, 13, 103,
  152, 226, 24, 169, 208, 222, 164, 85, 2, 58, 178, 42, 65, 138, 211, 78,
  140, 151, 41, 152, 222, 22, 79, 177, 219, 227, 169, 174, 97, 44, 28, 233,
  77, 194, 167, 221, 38, 36, 153, 171, 79, 226, 137, 18, 135, 78, 54, 50,
  90, 133, 204, 222, 49, 115, 101, 212, 137, 34, 219, 52, 144, 23, 61, 113,
  0, 139, 224, 15, 90, 9, 11, 182, 22, 195, 193, 40, 100, 182, 32, 58,
  219, 176, 176, 22, 1, 26, 68, 166, 189, 219, 247, 220, 26, 60, 194, 133,
  200, 50, 203, 215, 45, 10, 109, 35, 234, 56, 148, 122, 239, 240, 70, 133,
  93, 14, 155, 199, 114, 18, 13, 83, 96, 50, 154, 231, 97, 31, 212, 110,
  87, 213, 22, 153, 132, 197, 103, 237, 246, 59, 183, 138, 213, 44, 149, 167,
  1, 49, 226, 201, 220, 46, 191, 142, 196, 163, 213, 245, 80, 158, 207, 191,
  123, 239, 174, 163, 179, 155, 253, 48, 183, 68, 105, 94, 65, 25, 57, 50,
  175, 137, 131, 22, 91, 214, 22, 71, 173, 100, 230, 17, 87, 91, 92, 139,
  7, 53, 242, 46, 107, 15, 68, 104, 85, 186, 54, 220, 70, 35, 17, 7,
  130, 154, 20, 255, 85, 240, 38, 215, 106, 65, 80, 218, 68, 180, 37, 244,
  174, 151, 240, 147, 188, 89, 226, 214, 3, 95, 76, 81, 25, 225, 155, 133,
  202, 105, 223, 46, 231, 107, 10, 103, 215, 191, 2, 180, 124, 223, 24, 0,
  189, 123, 215, 65, 145, 187, 227, 59, 240, 114, 168, 211, 78, 151, 248, 251,
  242, 56, 178, 247, 95, 9, 75, 209, 254, 93, 8, 158, 135, 29, 227, 206,
  180, 117, 60, 184, 98, 211, 28, 141, 153, 68, 232, 77, 180, 135, 101, 206,
  222, 161, 226, 56, 57, 49, 228, 226, 210, 240, 124, 194, 162, 24, 199, 201,
  21, 23, 66, 162, 193, 5, 35, 239, 106, 108, 162, 237, 201, 216, 181, 111,
  95, 11, 204, 138, 107, 250, 48, 40, 254, 75, 110, 120, 122, 85, 30, 221,
  15, 114, 205, 203, 137, 144, 192, 143, 146, 240, 61, 28, 114, 86, 162, 225,
  160, 112, 217, 206, 237, 75, 76, 108, 211, 149, 229, 6, 85, 164, 159, 40,
  15, 168, 116, 143, 149, 242, 138, 180, 218, 114, 148, 48, 162, 138, 137, 19,
  175, 226, 69, 139, 103, 213, 60, 165, 197, 233, 91, 56, 184, 56, 79, 241,
  16, 73, 162, 101, 9, 139, 246, 37, 242, 229, 177, 162, 26, 86, 144, 44,
  106, 129, 131, 2, 218, 163, 47, 157, 255, 240, 225, 213, 227, 34, 114, 100,
  27, 85, 69, 236, 75, 227, 194, 224, 106, 19, 137, 125, 211, 85, 70, 174,
  16, 46, 201, 165, 253, 178, 156, 212, 229, 66, 36, 139, 169, 49, 226, 250,
  203, 3, 129, 24, 126, 61, 173, 102, 130, 48, 213, 95, 227, 170, 204, 39,
  43, 242, 175, 56, 50, 156, 85, 225, 41, 13, 229, 59, 86, 93, 101, 218,
  166, 11, 54, 150, 74, 95, 20, 49, 167, 233, 227, 223, 255, 143, 255, 37,
  248, 230, 211, 128, 182, 9, 243, 8, 68, 86, 176, 142, 96, 253, 144, 73,
  136, 144, 205, 239, 234, 4, 214, 224, 64, 104, 96, 248, 101, 247, 212, 173,
  46, 70, 89, 159, 108, 168, 101, 117, 241, 214, 63, 53, 158, 176, 134, 99,
  60, 98, 82, 119, 77, 63, 121, 112, 21, 135, 40, 196, 101, 146, 41, 81,
  207, 62, 213, 226, 103, 42, 146, 17, 214, 26, 158, 34, 180, 216, 155, 178,
  121, 247, 52, 193, 197, 212, 161, 117, 126, 192, 82, 118, 155, 26, 22, 147,
  94, 26, 106, 151, 92, 141, 245, 114, 109, 237, 198, 133, 104, 232, 0, 24,
  184, 30, 11, 199, 225, 218, 178, 88, 217, 145, 143, 54, 123, 58, 28, 92,
  217, 139, 108, 73, 18, 116, 233, 22, 176, 228, 234, 184, 239, 202, 88, 176,
  42, 238, 180, 34, 150, 90, 13, 246, 74, 144, 99, 183, 112, 29, 168, 145,
  42, 95, 5, 238, 220, 243, 97, 249, 254, 229, 171, 183, 47, 63, 156, 30,
  190, 56, 248, 243, 171, 151, 7, 20, 97, 207, 190, 89, 88, 251, 225, 116,
  31, 254, 21, 169, 71, 201, 116, 136, 241, 43, 107, 143, 39, 12, 132, 133,
  112, 245, 37, 187, 250, 240, 99, 146, 94, 96, 82, 22, 133, 171, 167, 201,
  197, 77, 130, 31, 243, 44, 79, 195, 24, 82, 78, 110, 134, 83, 118, 83,
  123, 111, 216, 207, 141, 115, 38, 46, 198, 237, 232, 161, 73, 157, 219, 76,
  242, 46, 177, 231, 208, 235, 110, 167, 86, 170, 249, 215, 124, 213, 186, 183,
  159, 100, 83, 90, 154, 152, 150, 71, 101, 25, 253, 160, 214, 91, 199, 61,
  212, 81, 0, 230, 249, 152, 13, 159, 49, 216, 161, 234, 243, 52, 38, 9,
  8, 143, 81, 165, 16, 164, 78, 217, 157, 18, 154, 120, 170, 160, 141, 226,
  240, 156, 59, 40, 105, 193, 147, 228, 221, 117, 26, 56, 200, 36, 226, 41,
  34, 35, 193, 82, 147, 137, 60, 212, 109, 113, 174, 79, 16, 118, 100, 130,
  44, 133, 228, 165, 215, 234, 234, 31, 43, 221, 149, 157, 226, 254, 120, 136,
  183, 83, 53, 244, 176, 19, 255, 146, 37, 83, 111, 87, 13, 11, 121, 202,
  178, 242, 40, 88, 252, 250, 65, 120, 21, 70, 121, 233, 0, 46, 21, 33,
  72, 11, 12, 250, 248, 248, 16, 22, 39, 64, 184, 21, 54, 126, 12, 11,
  10, 21, 100, 116, 84, 67, 188, 5, 148, 198, 248, 48, 10, 228, 251, 204,
  235, 20, 200, 143, 187, 217, 235, 12, 201, 238, 134, 40, 194, 251, 1, 125,
  34, 126, 81, 255, 71, 34, 78, 60, 27, 49, 73, 46, 124, 14, 14, 60,
  164, 172, 191, 69, 172, 197, 163, 152, 223, 126, 243, 73, 92, 200, 252, 24,
  240, 159, 100, 57, 174, 213, 76, 158, 167, 197, 142, 182, 2, 22, 115, 20,
  241, 234, 190, 47, 223, 194, 22, 225, 43, 154, 173, 10, 157, 38, 138, 104,
  135, 95, 84, 245, 142, 163, 141, 213, 141, 211, 174, 160, 206, 135, 162, 113,
  239, 97, 247, 4, 162, 72, 102, 115, 60, 11, 61, 53, 110, 116, 186, 158,
  21, 230, 141, 207, 178, 61, 86, 108, 172, 20, 131, 67, 47, 47, 88, 19,
  217, 59, 228, 149, 242, 182, 44, 162, 7, 149, 48, 42, 149, 237, 182, 214,
  78, 81, 132, 6, 249, 233, 151, 216, 112, 243, 159, 170, 247, 90, 51, 223,
  234, 194, 50, 219, 160, 216, 102, 196, 208, 225, 81, 138, 221, 193, 194, 215,
  130, 23, 106, 52, 190, 188, 88, 193, 33, 87, 119, 213, 83, 104, 249, 254,
  250, 122, 219, 240, 211, 137, 167, 37, 30, 86, 196, 45, 219, 240, 81, 84,
  9, 253, 85, 132, 99, 193, 85, 246, 34, 195, 45, 203, 241, 163, 229, 66,
  182, 220, 116, 148, 167, 146, 182, 85, 237, 84, 86, 16, 65, 173, 125, 83,
  6, 176, 0, 85, 238, 236, 76, 109, 239, 184, 76, 68, 185, 66, 63, 139,
  166, 32, 179, 215, 161, 14, 198, 211, 198, 170, 230, 45, 115, 207, 161, 171,
  158, 125, 235, 139, 123, 109, 187, 204, 125, 178, 172, 188, 106, 100, 121, 232,
  7, 18, 71, 138, 68, 25, 231, 193, 240, 236, 178, 243, 139, 192, 15, 170,
  88, 195, 199, 44, 125, 112, 191, 11, 234, 133, 7, 95, 208, 170, 128, 221,
  184, 247, 40, 112, 125, 227, 84, 92, 154, 156, 187, 163, 160, 46, 89, 55,
  28, 27, 167, 51, 64, 110, 124, 42, 81, 215, 90, 72, 53, 124, 65, 140,
  226, 105, 0, 175, 158, 90, 170, 118, 81, 201, 117, 2, 166, 199, 55, 91,
  232, 51, 96, 46, 101, 143, 15, 223, 162, 214, 249, 235, 221, 119, 192, 129,
  92, 198, 75, 17, 40, 89, 232, 198, 149, 115, 239, 26, 16, 222, 163, 46,
  53, 238, 44, 26, 109, 36, 55, 94, 221, 79, 89, 129, 121, 15, 222, 26,
  5, 220, 244, 15, 167, 121, 220, 126, 42, 182, 63, 188, 13, 23, 230, 117,
  37, 174, 54, 65, 142, 29, 39, 243, 180, 95, 235, 181, 134, 209, 121, 132,
  110, 70, 147, 104, 58, 207, 153, 158, 146, 225, 45, 183, 161, 158, 130, 99,
  247, 103, 12, 140, 226, 101, 77, 60, 24, 39, 54, 132, 8, 96, 219, 117,
  234, 67, 99, 241, 124, 86, 245, 134, 230, 244, 95, 231, 44, 142, 25, 8,
  220, 222, 89, 189, 93, 66, 28, 224, 145, 246, 246, 21, 222, 245, 89, 120,
  35, 46, 211, 186, 134, 82, 145, 167, 250, 102, 77, 175, 151, 47, 219, 117,
  236, 73, 182, 229, 140, 146, 253, 161, 28, 204, 173, 29, 27, 158, 147, 19,
  213, 225, 76, 94, 214, 165, 156, 15, 147, 12, 29, 133, 101, 26, 76, 197,
  7, 153, 238, 120, 144, 115, 170, 123, 6, 138, 15, 170, 58, 168, 255, 232,
  104, 124, 16, 44, 192, 220, 13, 180, 58, 64, 173, 18, 25, 63, 189, 186,
  4, 238, 42, 216, 62, 54, 185, 43, 225, 86, 21, 212, 248, 176, 63, 140,
  152, 32, 57, 123, 98, 69, 183, 172, 24, 183, 40, 165, 20, 93, 91, 220,
  13, 39, 234, 107, 89, 51, 184, 101, 118, 28, 227, 167, 183, 211, 230, 230,
  183, 176, 207, 118, 241, 219, 210, 151, 74, 244, 205, 160, 50, 178, 29, 61,
  125, 44, 98, 182, 211, 9, 131, 120, 85, 96, 169, 19, 5, 113, 108, 64,
  1, 246, 144, 164, 241, 132, 97, 40, 68, 6, 237, 61, 197, 154, 229, 153,
  59, 149, 167, 79, 214, 96, 122, 154, 22, 102, 56, 251, 233, 3, 254, 228,
  142, 57, 114, 85, 225, 254, 152, 115, 111, 136, 159, 255, 213, 150, 96, 89,
  204, 119, 147, 69, 86, 183, 45, 161, 116, 213, 56, 153, 231, 162, 221, 79,
  94, 228, 249, 243, 12, 24, 219, 171, 187, 213, 41, 219, 130, 92, 84, 42,
  159, 128, 40, 121, 132, 72, 7, 169, 234, 207, 146, 8, 131, 0, 146, 181,
  54, 51, 161, 224, 251, 100, 21, 79, 25, 41, 209, 188, 226, 13, 27, 59,
  244, 180, 56, 108, 151, 55, 197, 241, 90, 15, 143, 149, 229, 247, 150, 140,
  147, 243, 122, 109, 255, 232, 112, 255, 123, 116, 27, 108, 231, 116, 125, 188,
  25, 80, 124, 162, 201, 12, 118, 174, 33, 61, 115, 81, 151, 89, 13, 179,
  59, 198, 112, 190, 171, 241, 166, 208, 123, 181, 86, 188, 244, 13, 31, 6,
  121, 190, 175, 136, 45, 189, 232, 125, 14, 253, 141, 14, 189, 11, 232, 3,
  87, 19, 19, 85, 243, 161, 31, 55, 228, 60, 66, 65, 209, 133, 178, 130,
  70, 15, 161, 248, 79, 101, 5, 127, 58, 132, 197, 127, 189, 240, 61, 163,
  91, 17, 118, 123, 103, 137, 121, 189, 96, 55, 195, 228, 106, 90, 54, 179,
  172, 77, 190, 84, 184, 228, 15, 178, 65, 56, 99, 150, 150, 240, 206, 28,
  106, 107, 26, 212, 244, 216, 83, 96, 115, 38, 53, 242, 77, 97, 160, 243,
  245, 202, 19, 61, 20, 207, 111, 19, 254, 236, 76, 114, 209, 68, 239, 84,
  64, 39, 111, 202, 219, 211, 119, 123, 135, 197, 203, 3, 149, 255, 50, 174,
  236, 175, 55, 207, 30, 244, 182, 244, 235, 112, 152, 141, 12, 111, 31, 89,
  8, 22, 25, 38, 121, 107, 54, 135, 46, 180, 162, 97, 204, 44, 214, 40,
  61, 13, 240, 186, 9, 71, 213, 229, 146, 182, 177, 41, 208, 154, 95, 91,
  31, 174, 109, 111, 91, 140, 177, 164, 249, 81, 152, 153, 39, 100, 94, 238,
  87, 192, 30, 157, 157, 141, 122, 235, 203, 193, 206, 226, 228, 170, 86, 201,
  216, 172, 222, 126, 229, 237, 110, 209, 184, 240, 218, 215, 65, 26, 77, 215,
  42, 239, 2, 221, 15, 142, 203, 56, 141, 179, 91, 235, 216, 182, 40, 148,
  92, 159, 140, 195, 33, 221, 124, 249, 200, 95, 120, 88, 155, 93, 227, 81,
  6, 86, 184, 93, 91, 251, 104, 84, 114, 247, 22, 107, 138, 154, 206, 192,
  54, 29, 42, 106, 184, 59, 44, 117, 169, 225, 238, 124, 69, 158, 27, 197,
  215, 14, 242, 120, 172, 73, 206, 238, 218, 226, 14, 14, 66, 202, 80, 119,
  175, 101, 93, 244, 191, 247, 192, 114, 87, 28, 25, 160, 84, 45, 175, 119,
  27, 55, 50, 112, 211, 6, 48, 135, 50, 131, 134, 123, 239, 26, 107, 160,
  169, 1, 75, 218, 166, 6, 221, 199, 206, 8, 186, 207, 67, 111, 36, 23,
  125, 46, 32, 231, 228, 25, 7, 171, 78, 177, 15, 35, 67, 18, 174, 226,
  41, 70, 174, 72, 19, 252, 86, 94, 161, 192, 14, 241, 2, 13, 253, 230,
  133, 116, 218, 252, 85, 26, 199, 166, 172, 214, 69, 96, 2, 173, 113, 116,
  36, 244, 54, 142, 25, 229, 141, 99, 110, 73, 227, 234, 202, 60, 22, 114,
  102, 143, 167, 202, 155, 90, 230, 21, 122, 242, 249, 193, 201, 228, 133, 100,
  240, 152, 166, 113, 68, 192, 99, 61, 232, 61, 16, 238, 137, 254, 94, 136,
  204, 138, 158, 136, 18, 149, 67, 137, 205, 90, 67, 169, 71, 139, 248, 149,
  177, 145, 77, 91, 24, 113, 127, 93, 147, 180, 208, 253, 210, 79, 91, 152,
  83, 65, 92, 152, 93, 77, 218, 228, 73, 101, 32, 192, 99, 86, 24, 187,
  135, 134, 11, 228, 34, 42, 100, 15, 33, 8, 144, 208, 80, 74, 102, 211,
  168, 182, 112, 224, 76, 249, 125, 185, 65, 52, 234, 24, 125, 65, 76, 244,
  173, 172, 233, 123, 242, 196, 186, 40, 28, 206, 116, 103, 97, 26, 117, 25,
  119, 69, 32, 47, 62, 173, 48, 46, 188, 97, 51, 44, 146, 138, 244, 25,
  198, 40, 28, 137, 11, 191, 220, 140, 32, 7, 200, 113, 178, 255, 20, 180,
  219, 116, 194, 196, 129, 162, 0, 52, 102, 97, 156, 143, 129, 36, 57, 128,
  228, 162, 184, 56, 45, 144, 210, 110, 82, 23, 91, 51, 111, 246, 182, 92,
  139, 132, 241, 225, 155, 0, 109, 43, 117, 231, 169, 61, 113, 144, 42, 206,
  225, 169, 111, 48, 38, 54, 159, 166, 235, 150, 158, 107, 23, 100, 129, 51,
  66, 73, 129, 124, 125, 101, 128, 178, 178, 49, 180, 148, 123, 64, 235, 186,
  187, 17, 152, 34, 100, 133, 219, 72, 211, 7, 217, 49, 169, 240, 230, 138,
  144, 29, 247, 133, 195, 213, 25, 220, 231, 16, 177, 239, 8, 110, 3, 239,
  245, 91, 66, 168, 109, 77, 23, 106, 16, 14, 132, 0, 241, 144, 236, 15,
  165, 253, 30, 98, 156, 24, 137, 49, 222, 103, 226, 147, 211, 18, 213, 93,
  155, 81, 56, 98, 79, 66, 146, 121, 68, 3, 223, 6, 157, 118, 183, 99,
  151, 67, 139, 170, 91, 174, 183, 97, 223, 37, 193, 214, 31, 238, 42, 176,
  5, 166, 64, 72, 45, 76, 173, 149, 212, 144, 13, 152, 53, 200, 144, 235,
  208, 63, 101, 157, 133, 195, 218, 162, 231, 94, 96, 90, 190, 159, 69, 220,
  82, 40, 77, 115, 182, 93, 203, 247, 148, 166, 253, 178, 194, 197, 44, 34,
  151, 196, 119, 244, 54, 57, 95, 252, 187, 127, 89, 1, 52, 254, 178, 242,
  190, 182, 140, 38, 161, 226, 249, 190, 17, 104, 104, 114, 142, 88, 60, 149,
  146, 14, 173, 160, 71, 129, 116, 161, 240, 73, 59, 3, 10, 206, 105, 47,
  88, 209, 98, 99, 129, 92, 170, 166, 167, 169, 13, 124, 179, 24, 105, 241,
  147, 196, 82, 187, 195, 208, 112, 195, 54, 102, 152, 114, 41, 148, 40, 3,
  80, 110, 235, 229, 195, 124, 152, 61, 231, 172, 237, 94, 82, 168, 113, 215,
  163, 76, 154, 109, 180, 5, 247, 172, 100, 131, 79, 161, 51, 225, 116, 192,
  124, 135, 112, 186, 215, 136, 143, 255, 21, 220, 207, 156, 184, 47, 197, 255,
  92, 168, 136, 209, 195, 192, 133, 173, 170, 120, 154, 109, 185, 135, 133, 8,
  102, 207, 203, 225, 116, 87, 152, 150, 175, 132, 179, 106, 59, 59, 11, 166,
  250, 20, 205, 155, 165, 17, 68, 6, 35, 52, 107, 235, 161, 192, 204, 93,
  88, 198, 203, 218, 52, 227, 101, 25, 225, 178, 122, 118, 224, 46, 135, 125,
  86, 5, 150, 224, 145, 20, 27, 50, 16, 67, 17, 61, 162, 36, 90, 132,
  145, 117, 185, 96, 141, 67, 137, 134, 181, 174, 196, 53, 26, 10, 112, 240,
  48, 232, 21, 174, 29, 193, 48, 74, 251, 53, 16, 121, 114, 10, 178, 150,
  99, 200, 54, 21, 144, 139, 247, 219, 237, 26, 31, 31, 178, 118, 243, 8,
  24, 49, 222, 170, 107, 193, 176, 182, 121, 150, 213, 186, 44, 191, 60, 6,
  178, 198, 59, 187, 106, 43, 232, 190, 127, 4, 35, 240, 232, 209, 66, 44,
  57, 172, 96, 183, 26, 88, 251, 210, 174, 55, 138, 82, 179, 90, 199, 83,
  104, 24, 141, 70, 196, 52, 168, 141, 22, 175, 228, 42, 141, 231, 250, 254,
  137, 117, 92, 177, 224, 38, 3, 10, 96, 89, 132, 28, 23, 71, 80, 82,
  28, 237, 138, 235, 78, 212, 162, 148, 93, 62, 141, 82, 69, 190, 60, 16,
  155, 78, 190, 124, 40, 77, 203, 210, 144, 106, 216, 57, 252, 93, 246, 115,
  88, 149, 122, 187, 13, 81, 154, 122, 136, 42, 214, 163, 160, 54, 159, 145,
  177, 151, 12, 124, 58, 247, 23, 6, 26, 23, 8, 140, 136, 214, 49, 188,
  89, 164, 16, 71, 210, 69, 120, 128, 170, 145, 70, 192, 27, 178, 117, 145,
  165, 189, 21, 232, 235, 46, 98, 153, 154, 93, 229, 34, 5, 31, 34, 151,
  151, 201, 135, 115, 176, 65, 188, 166, 209, 240, 222, 59, 200, 242, 227, 148,
  110, 159, 233, 188, 154, 102, 184, 177, 227, 45, 254, 146, 68, 68, 189, 52,
  39, 12, 231, 96, 87, 193, 214, 253, 8, 36, 132, 178, 163, 93, 83, 28,
  83, 32, 90, 178, 162, 107, 101, 46, 72, 142, 207, 232, 67, 107, 134, 245,
  65, 170, 77, 217, 28, 125, 70, 45, 139, 29, 77, 174, 94, 142, 183, 47,
  8, 34, 79, 174, 208, 76, 139, 68, 17, 94, 133, 55, 181, 69, 167, 77,
  82, 237, 73, 249, 194, 219, 165, 165, 177, 199, 9, 158, 243, 84, 132, 202,
  127, 33, 212, 186, 150, 175, 104, 10, 75, 36, 163, 156, 63, 96, 135, 164,
  220, 240, 168, 51, 64, 16, 146, 141, 40, 230, 128, 164, 220, 84, 173, 55,
  181, 110, 85, 168, 43, 88, 122, 248, 56, 77, 147, 43, 152, 53, 199, 7,
  123, 168, 145, 114, 33, 103, 254, 253, 127, 254, 95, 109, 161, 212, 36, 111,
  173, 228, 191, 123, 132, 209, 218, 34, 231, 13, 16, 65, 213, 30, 66, 2,
  92, 147, 227, 233, 238, 113, 252, 228, 93, 137, 136, 228, 190, 42, 247, 43,
  140, 110, 118, 127, 11, 89, 59, 79, 158, 69, 215, 108, 88, 151, 224, 240,
  17, 136, 218, 223, 255, 237, 223, 29, 13, 98, 209, 62, 91, 178, 127, 218,
  11, 149, 252, 148, 27, 234, 109, 248, 26, 136, 208, 34, 92, 171, 214, 201,
  186, 24, 14, 164, 24, 133, 88, 163, 28, 168, 186, 159, 102, 2, 126, 94,
  92, 91, 251, 12, 224, 120, 5, 205, 132, 187, 79, 151, 210, 62, 3, 36,
  231, 86, 58, 200, 55, 116, 205, 236, 14, 32, 133, 254, 136, 244, 114, 16,
  191, 112, 12, 154, 220, 42, 38, 98, 106, 138, 49, 38, 57, 170, 214, 240,
  155, 172, 138, 162, 114, 212, 60, 197, 201, 88, 88, 148, 132, 113, 240, 20,
  34, 155, 79, 81, 8, 197, 63, 94, 168, 220, 132, 34, 186, 65, 126, 3,
  178, 67, 37, 129, 214, 68, 129, 50, 77, 138, 111, 56, 84, 232, 81, 27,
  63, 74, 54, 81, 174, 20, 33, 126, 45, 42, 204, 171, 180, 124, 155, 106,
  229, 46, 67, 190, 31, 188, 53, 141, 191, 18, 161, 115, 174, 218, 40, 107,
  234, 60, 73, 134, 6, 171, 85, 59, 176, 31, 30, 177, 230, 82, 104, 186,
  214, 107, 0, 195, 49, 248, 106, 87, 202, 11, 165, 245, 45, 133, 218, 115,
  158, 226, 3, 102, 187, 59, 100, 197, 216, 23, 219, 131, 112, 38, 164, 205,
  224, 81, 201, 56, 32, 211, 89, 128, 146, 127, 47, 18, 228, 96, 220, 17,
  3, 60, 28, 119, 5, 100, 253, 18, 185, 98, 31, 216, 113, 224, 152, 222,
  73, 188, 26, 146, 144, 206, 17, 203, 172, 10, 192, 207, 159, 132, 105, 253,
  238, 167, 33, 196, 177, 129, 173, 155, 108, 124, 17, 19, 231, 44, 220, 81,
  250, 139, 189, 69, 94, 237, 189, 196, 167, 71, 37, 18, 220, 112, 221, 52,
  84, 43, 193, 148, 27, 94, 32, 218, 221, 95, 19, 144, 178, 57, 91, 192,
  10, 102, 236, 7, 200, 175, 244, 154, 176, 208, 204, 111, 129, 33, 182, 235,
  135, 192, 239, 228, 154, 16, 208, 96, 107, 65, 160, 213, 170, 65, 240, 218,
  126, 44, 16, 141, 69, 147, 12, 60, 79, 123, 187, 242, 238, 115, 45, 173,
  174, 194, 19, 203, 24, 2, 101, 204, 118, 248, 35, 61, 30, 89, 110, 135,
  210, 158, 156, 52, 76, 81, 43, 0, 212, 53, 68, 97, 49, 254, 192, 178,
  227, 91, 34, 177, 131, 117, 234, 243, 43, 17, 129, 84, 102, 209, 157, 141,
  98, 126, 76, 160, 104, 3, 193, 221, 13, 143, 178, 169, 193, 151, 173, 158,
  179, 112, 40, 130, 82, 103, 198, 236, 184, 110, 128, 69, 110, 161, 143, 107,
  53, 112, 241, 37, 244, 190, 184, 197, 232, 202, 78, 75, 209, 248, 238, 203,
  107, 98, 70, 1, 248, 214, 119, 218, 252, 155, 125, 40, 217, 140, 130, 106,
  89, 202, 188, 20, 255, 153, 143, 45, 139, 109, 60, 201, 15, 135, 70, 164,
  208, 167, 73, 254, 113, 199, 186, 164, 159, 228, 117, 42, 216, 84, 167, 27,
  77, 243, 108, 163, 105, 158, 108, 252, 211, 188, 234, 92, 152, 204, 234, 116,
  148, 128, 118, 179, 146, 155, 255, 40, 204, 76, 97, 211, 58, 162, 253, 77,
  110, 30, 210, 90, 88, 190, 125, 160, 205, 15, 55, 16, 44, 105, 111, 33,
  18, 230, 115, 126, 202, 33, 129, 42, 27, 99, 57, 84, 50, 1, 34, 88,
  42, 219, 247, 40, 237, 18, 87, 93, 103, 86, 109, 217, 137, 88, 112, 79,
  101, 91, 78, 70, 162, 64, 83, 229, 163, 9, 225, 157, 252, 104, 74, 0,
  239, 203, 125, 108, 167, 252, 118, 153, 124, 163, 68, 194, 19, 207, 148, 72,
  164, 220, 184, 59, 116, 19, 121, 72, 190, 216, 215, 57, 153, 72, 17, 113,
  143, 229, 20, 36, 7, 42, 50, 150, 157, 171, 180, 135, 58, 143, 167, 96,
  93, 203, 164, 46, 154, 134, 241, 161, 187, 209, 25, 144, 217, 148, 223, 80,
  224, 15, 124, 139, 120, 179, 111, 212, 83, 132, 214, 163, 218, 6, 152, 134,
  103, 75, 244, 173, 233, 71, 180, 53, 186, 49, 189, 56, 148, 42, 53, 215,
  65, 134, 222, 251, 125, 74, 145, 125, 165, 123, 109, 85, 72, 11, 222, 201,
  139, 104, 118, 74, 123, 22, 151, 1, 222, 240, 71, 12, 235, 11, 216, 15,
  63, 70, 210, 31, 196, 145, 34, 124, 241, 240, 164, 122, 236, 102, 87, 123,
  238, 134, 76, 92, 58, 162, 228, 190, 88, 0, 106, 216, 23, 65, 16, 41,
  94, 180, 190, 212, 9, 134, 148, 36, 232, 117, 14, 124, 177, 141, 179, 213,
  112, 166, 191, 38, 218, 164, 167, 110, 244, 119, 171, 74, 94, 88, 78, 166,
  226, 21, 27, 243, 210, 137, 245, 114, 43, 218, 141, 240, 21, 92, 231, 4,
  15, 56, 168, 246, 250, 2, 52, 103, 188, 65, 164, 36, 16, 243, 73, 36,
  40, 102, 62, 131, 164, 194, 238, 152, 15, 21, 97, 23, 176, 83, 246, 198,
  15, 141, 154, 131, 8, 9, 62, 127, 43, 209, 183, 71, 133, 155, 27, 74,
  3, 62, 143, 46, 13, 128, 230, 139, 85, 212, 47, 156, 178, 210, 243, 179,
  176, 190, 209, 107, 246, 186, 221, 102, 119, 99, 173, 137, 230, 255, 6, 193,
  181, 202, 244, 214, 182, 155, 155, 91, 248, 255, 188, 72, 217, 165, 48, 126,
  191, 50, 119, 47, 55, 20, 173, 243, 95, 212, 72, 50, 26, 209, 111, 43,
  254, 141, 123, 83, 231, 204, 115, 5, 70, 144, 8, 64, 124, 124, 76, 166,
  53, 147, 70, 32, 253, 237, 209, 227, 151, 212, 206, 43, 209, 142, 177, 119,
  113, 152, 158, 129, 46, 0, 171, 120, 190, 62, 232, 95, 247, 122, 131, 141,
  13, 102, 204, 66, 105, 3, 133, 7, 94, 103, 212, 125, 208, 11, 107, 203,
  172, 13, 52, 50, 255, 36, 30, 40, 173, 243, 55, 110, 155, 248, 46, 246,
  169, 56, 241, 182, 35, 103, 243, 87, 79, 41, 80, 95, 81, 72, 25, 16,
  69, 140, 255, 78, 17, 224, 223, 115, 182, 144, 162, 235, 184, 58, 35, 16,
  97, 215, 142, 163, 107, 22, 191, 166, 28, 0, 222, 45, 123, 189, 64, 61,
  164, 186, 204, 19, 1, 131, 44, 123, 171, 123, 13, 116, 155, 252, 55, 77,
  66, 157, 2, 247, 19, 194, 13, 95, 205, 231, 11, 106, 242, 14, 186, 85,
  149, 191, 3, 47, 75, 56, 124, 203, 251, 236, 158, 95, 56, 101, 159, 123,
  202, 210, 14, 194, 59, 78, 232, 210, 246, 67, 155, 157, 72, 229, 168, 80,
  178, 181, 105, 27, 213, 160, 210, 142, 155, 39, 42, 67, 85, 239, 198, 45,
  39, 25, 163, 217, 158, 166, 225, 52, 67, 155, 25, 140, 71, 7, 254, 71,
  255, 234, 157, 42, 10, 227, 203, 211, 117, 234, 72, 211, 238, 142, 77, 44,
  56, 66, 5, 189, 208, 24, 84, 88, 150, 167, 104, 178, 139, 129, 100, 113,
  253, 212, 237, 136, 250, 102, 232, 124, 140, 111, 80, 88, 141, 121, 180, 3,
  127, 201, 77, 189, 224, 102, 121, 185, 59, 4, 227, 151, 71, 222, 144, 238,
  218, 170, 141, 183, 224, 220, 78, 114, 187, 228, 211, 240, 6, 3, 103, 14,
  173, 123, 28, 226, 41, 95, 126, 126, 72, 49, 151, 159, 129, 8, 247, 35,
  11, 83, 119, 5, 160, 75, 167, 8, 79, 43, 11, 191, 0, 230, 54, 174,
  99, 108, 222, 110, 163, 61, 11, 209, 75, 33, 205, 235, 189, 38, 176, 99,
  119, 123, 25, 186, 245, 233, 42, 90, 163, 170, 170, 232, 36, 232, 18, 55,
  244, 224, 0, 254, 99, 132, 102, 117, 162, 69, 134, 55, 20, 159, 2, 227,
  149, 134, 55, 25, 52, 117, 147, 121, 92, 67, 166, 67, 113, 29, 143, 163,
  224, 241, 11, 75, 243, 146, 18, 148, 135, 36, 76, 25, 0, 169, 232, 9,
  108, 221, 212, 32, 30, 104, 54, 124, 100, 58, 74, 147, 73, 223, 156, 18,
  2, 7, 154, 70, 158, 88, 25, 116, 154, 80, 65, 189, 120, 109, 255, 9,
  61, 68, 95, 143, 35, 60, 100, 25, 132, 179, 102, 64, 87, 15, 28, 86,
  43, 82, 29, 95, 120, 212, 158, 16, 78, 157, 23, 176, 3, 57, 102, 121,
  17, 140, 31, 160, 55, 120, 5, 80, 244, 241, 152, 25, 223, 14, 211, 74,
  180, 168, 68, 57, 186, 225, 112, 200, 177, 21, 175, 121, 224, 203, 97, 140,
  7, 72, 187, 193, 151, 145, 154, 193, 25, 101, 191, 200, 228, 185, 16, 12,
  66, 147, 55, 107, 79, 95, 120, 142, 231, 25, 178, 60, 45, 149, 39, 63,
  236, 127, 127, 112, 250, 97, 227, 5, 108, 117, 4, 186, 13, 133, 54, 130,
  126, 241, 209, 221, 112, 35, 106, 210, 249, 178, 106, 141, 175, 185, 73, 52,
  172, 41, 32, 240, 161, 96, 196, 201, 244, 220, 145, 153, 8, 135, 67, 138,
  223, 76, 220, 119, 20, 39, 160, 68, 130, 76, 188, 170, 240, 179, 157, 189,
  207, 207, 219, 60, 139, 218, 107, 117, 27, 129, 158, 84, 128, 180, 164, 12,
  213, 210, 67, 189, 60, 176, 139, 34, 167, 165, 231, 208, 148, 193, 22, 208,
  51, 217, 248, 162, 182, 120, 137, 140, 60, 184, 59, 118, 242, 0, 246, 150,
  220, 204, 40, 248, 251, 213, 24, 237, 26, 26, 162, 123, 90, 99, 190, 3,
  94, 245, 28, 131, 130, 204, 79, 53, 37, 2, 171, 90, 78, 223, 185, 76,
  231, 167, 255, 79, 1, 48, 126, 173, 143, 223, 106, 100, 117, 217, 23, 77,
  154, 113, 166, 180, 210, 223, 237, 234, 226, 194, 253, 135, 130, 223, 150, 64,
  147, 145, 255, 44, 89, 194, 133, 246, 168, 152, 31, 184, 137, 77, 149, 220,
  133, 143, 7, 26, 193, 113, 139, 83, 73, 232, 183, 113, 137, 180, 242, 2,
  185, 228, 126, 64, 238, 184, 139, 71, 248, 235, 6, 99, 208, 151, 191, 157,
  3, 121, 101, 7, 58, 121, 102, 92, 130, 205, 221, 235, 173, 160, 240, 156,
  210, 193, 67, 166, 221, 80, 69, 151, 113, 252, 238, 251, 238, 95, 63, 10,
  234, 139, 222, 221, 193, 99, 80, 237, 107, 199, 225, 253, 220, 170, 248, 37,
  189, 245, 124, 243, 93, 250, 220, 29, 117, 218, 216, 80, 144, 187, 168, 123,
  171, 218, 108, 61, 226, 209, 136, 250, 70, 17, 45, 38, 106, 37, 75, 165,
  102, 154, 5, 99, 108, 114, 214, 6, 210, 231, 227, 63, 125, 216, 124, 254,
  225, 248, 213, 225, 203, 211, 147, 198, 253, 192, 117, 9, 30, 114, 68, 1,
  176, 183, 238, 64, 244, 108, 89, 140, 228, 236, 99, 220, 108, 36, 120, 91,
  236, 146, 132, 33, 68, 51, 218, 167, 93, 57, 109, 199, 187, 35, 232, 181,
  164, 208, 166, 239, 7, 93, 99, 67, 216, 240, 51, 115, 162, 217, 18, 72,
  69, 239, 1, 144, 26, 217, 133, 252, 221, 235, 210, 247, 249, 60, 80, 137,
  21, 203, 176, 188, 202, 160, 28, 101, 79, 46, 125, 246, 188, 84, 177, 18,
  238, 123, 0, 52, 217, 167, 224, 168, 49, 57, 244, 189, 123, 95, 230, 230,
  86, 53, 47, 79, 31, 255, 248, 225, 197, 9, 114, 12, 187, 192, 38, 229,
  159, 28, 254, 233, 195, 115, 94, 226, 232, 240, 205, 193, 135, 183, 135, 47,
  159, 190, 122, 11, 9, 238, 83, 112, 248, 216, 41, 210, 231, 105, 69, 115,
  245, 66, 32, 120, 87, 252, 52, 93, 227, 114, 100, 67, 117, 37, 69, 188,
  83, 191, 236, 98, 101, 60, 209, 195, 252, 48, 28, 177, 198, 9, 26, 203,
  114, 195, 186, 222, 43, 51, 215, 145, 242, 163, 41, 181, 174, 212, 213, 142,
  88, 250, 128, 175, 156, 135, 70, 137, 32, 85, 185, 252, 112, 132, 212, 234,
  155, 232, 49, 77, 133, 154, 76, 126, 16, 37, 32, 76, 14, 3, 96, 12,
  30, 86, 226, 241, 73, 210, 233, 93, 28, 59, 49, 109, 111, 151, 15, 65,
  67, 57, 83, 106, 82, 173, 58, 8, 228, 124, 12, 247, 54, 15, 71, 43,
  250, 96, 73, 126, 178, 26, 54, 197, 127, 234, 13, 114, 23, 78, 46, 141,
  243, 92, 87, 131, 192, 34, 85, 46, 183, 32, 200, 62, 231, 43, 77, 173,
  99, 146, 236, 51, 190, 158, 255, 17, 162, 192, 189, 119, 183, 123, 50, 156,
  112, 26, 229, 162, 56, 239, 58, 80, 241, 140, 207, 127, 157, 56, 37, 204,
  50, 241, 197, 250, 236, 238, 206, 188, 128, 51, 150, 148, 119, 217, 26, 13,
  191, 175, 235, 27, 121, 243, 71, 34, 227, 146, 161, 254, 58, 29, 94, 24,
  159, 213, 109, 237, 75, 131, 84, 224, 169, 165, 218, 239, 216, 149, 11, 28,
  70, 37, 171, 25, 15, 191, 180, 34, 106, 43, 61, 72, 239, 143, 88, 28,
  101, 162, 68, 160, 237, 178, 133, 182, 225, 176, 192, 71, 186, 206, 180, 128,
  85, 66, 119, 87, 139, 45, 23, 185, 90, 171, 91, 210, 160, 71, 98, 215,
  114, 125, 130, 123, 225, 115, 227, 99, 64, 190, 1, 33, 118, 86, 50, 34,
  30, 89, 200, 192, 161, 235, 140, 137, 190, 125, 120, 6, 165, 98, 163, 209,
  135, 165, 91, 57, 46, 221, 234, 129, 233, 86, 141, 204, 255, 231, 8, 162,
  202, 198, 141, 246, 72, 46, 48, 213, 51, 250, 131, 49, 14, 240, 236, 216,
  115, 75, 69, 148, 224, 239, 95, 54, 10, 86, 228, 81, 134, 113, 203, 191,
  63, 75, 250, 86, 224, 224, 81, 148, 117, 49, 145, 51, 63, 195, 145, 161,
  81, 117, 8, 120, 54, 143, 226, 225, 241, 108, 52, 148, 219, 138, 219, 83,
  239, 37, 55, 89, 12, 250, 45, 240, 210, 47, 185, 57, 92, 115, 126, 125,
  226, 217, 104, 124, 15, 154, 224, 53, 104, 111, 89, 204, 176, 118, 36, 5,
  22, 143, 24, 84, 69, 23, 9, 149, 37, 168, 218, 156, 99, 9, 5, 51,
  213, 76, 251, 42, 11, 150, 80, 82, 27, 115, 43, 171, 147, 189, 10, 79,
  180, 249, 242, 64, 114, 134, 253, 122, 62, 161, 35, 16, 162, 81, 235, 4,
  196, 168, 218, 189, 91, 221, 97, 26, 94, 237, 99, 100, 143, 186, 227, 166,
  229, 158, 134, 206, 82, 134, 231, 179, 207, 241, 205, 120, 170, 228, 28, 76,
  251, 159, 74, 45, 39, 44, 244, 212, 96, 131, 252, 56, 140, 82, 116, 37,
  241, 170, 26, 232, 216, 150, 57, 23, 119, 164, 99, 156, 119, 243, 31, 207,
  39, 158, 26, 154, 23, 156, 183, 214, 229, 108, 232, 169, 197, 29, 213, 188,
  21, 98, 122, 207, 84, 93, 68, 37, 68, 5, 103, 106, 18, 14, 234, 3,
  65, 139, 15, 7, 74, 50, 207, 173, 199, 122, 125, 15, 229, 65, 101, 207,
  11, 121, 80, 151, 75, 138, 159, 132, 183, 48, 225, 240, 78, 199, 4, 143,
  145, 1, 207, 239, 130, 8, 239, 233, 16, 94, 125, 66, 238, 157, 134, 161,
  93, 136, 252, 127, 17, 233, 119, 26, 230, 102, 33, 111, 128, 26, 197, 100,
  16, 49, 67, 214, 225, 111, 109, 232, 206, 33, 51, 244, 62, 180, 82, 160,
  181, 66, 26, 90, 116, 150, 129, 33, 155, 142, 240, 73, 131, 122, 33, 215,
  210, 165, 16, 180, 79, 208, 209, 185, 178, 121, 249, 132, 220, 102, 128, 65,
  91, 125, 177, 226, 154, 129, 136, 122, 250, 148, 59, 78, 137, 179, 124, 254,
  114, 183, 72, 171, 213, 62, 71, 41, 230, 130, 178, 39, 110, 163, 231, 166,
  175, 80, 190, 242, 76, 222, 106, 58, 205, 156, 107, 44, 62, 109, 23, 29,
  87, 22, 10, 49, 106, 197, 140, 77, 195, 53, 199, 96, 53, 88, 163, 151,
  181, 59, 86, 224, 36, 161, 34, 78, 204, 58, 162, 210, 239, 85, 37, 168,
  191, 233, 169, 93, 156, 229, 136, 99, 160, 241, 216, 56, 250, 193, 147, 31,
  124, 155, 79, 228, 78, 38, 110, 238, 199, 178, 208, 255, 126, 204, 100, 111,
  92, 108, 132, 178, 144, 149, 244, 100, 83, 246, 163, 107, 87, 116, 58, 225,
  67, 179, 232, 68, 150, 85, 118, 194, 246, 106, 146, 222, 52, 75, 4, 129,
  44, 66, 62, 2, 57, 163, 155, 51, 6, 170, 183, 111, 111, 216, 132, 192,
  111, 59, 10, 255, 35, 138, 34, 137, 36, 45, 129, 250, 78, 236, 212, 9,
  152, 199, 122, 155, 203, 133, 184, 92, 104, 75, 108, 85, 69, 162, 180, 117,
  99, 109, 229, 121, 87, 6, 164, 223, 161, 169, 79, 254, 225, 65, 110, 49,
  205, 199, 250, 48, 14, 195, 155, 226, 83, 139, 148, 57, 116, 214, 218, 87,
  26, 31, 64, 175, 34, 133, 17, 50, 174, 34, 175, 161, 145, 136, 42, 115,
  139, 161, 194, 229, 104, 249, 9, 64, 212, 82, 165, 42, 143, 56, 129, 158,
  38, 51, 98, 127, 101, 188, 204, 35, 139, 150, 141, 91, 141, 77, 91, 251,
  143, 107, 21, 163, 118, 195, 66, 160, 70, 144, 67, 241, 209, 131, 187, 140,
  162, 78, 61, 149, 226, 0, 5, 70, 251, 145, 63, 113, 79, 6, 203, 204,
  190, 37, 229, 62, 182, 119, 253, 134, 124, 239, 21, 179, 108, 183, 121, 144,
  141, 204, 103, 212, 210, 203, 194, 158, 237, 43, 235, 21, 100, 121, 51, 36,
  200, 186, 121, 4, 182, 161, 249, 172, 64, 74, 31, 120, 54, 84, 234, 163,
  0, 54, 11, 167, 125, 227, 70, 21, 110, 238, 192, 14, 134, 108, 248, 34,
  66, 25, 130, 67, 240, 230, 135, 215, 116, 59, 239, 250, 141, 125, 127, 91,
  118, 124, 87, 86, 247, 173, 151, 25, 249, 53, 171, 161, 81, 247, 16, 101,
  119, 240, 78, 107, 167, 167, 141, 49, 183, 58, 83, 185, 89, 114, 85, 239,
  66, 47, 90, 250, 189, 178, 174, 177, 30, 140, 62, 112, 124, 90, 152, 232,
  41, 163, 245, 3, 68, 8, 163, 140, 55, 242, 151, 7, 251, 186, 106, 66,
  118, 152, 208, 223, 104, 6, 254, 126, 117, 245, 12, 189, 70, 69, 39, 56,
  133, 220, 167, 19, 174, 57, 30, 169, 160, 128, 206, 201, 161, 128, 36, 232,
  162, 232, 94, 209, 70, 75, 175, 5, 248, 118, 0, 227, 74, 23, 128, 56,
  156, 82, 112, 222, 104, 112, 145, 213, 7, 249, 245, 83, 16, 235, 113, 29,
  11, 99, 31, 238, 58, 40, 12, 129, 92, 143, 127, 102, 113, 146, 191, 69,
  71, 25, 41, 30, 33, 251, 160, 189, 133, 0, 64, 23, 55, 8, 89, 249,
  133, 68, 192, 15, 199, 101, 10, 191, 234, 77, 239, 73, 29, 3, 166, 220,
  6, 218, 93, 15, 110, 191, 192, 73, 1, 12, 138, 229, 38, 69, 104, 195,
  160, 136, 126, 56, 85, 76, 105, 205, 107, 58, 70, 111, 81, 201, 46, 213,
  112, 228, 63, 241, 72, 69, 42, 71, 140, 15, 164, 187, 177, 82, 53, 217,
  79, 111, 239, 81, 9, 220, 190, 55, 128, 75, 134, 122, 206, 147, 48, 99,
  202, 221, 128, 143, 232, 163, 162, 191, 61, 99, 172, 17, 144, 202, 146, 19,
  100, 56, 142, 169, 201, 4, 41, 101, 171, 227, 90, 34, 69, 163, 58, 211,
  147, 83, 219, 52, 90, 213, 176, 243, 30, 7, 156, 204, 194, 1, 159, 232,
  245, 45, 247, 129, 163, 243, 137, 8, 74, 170, 79, 156, 108, 187, 101, 176,
  13, 245, 212, 187, 164, 166, 5, 186, 207, 174, 2, 239, 209, 128, 36, 24,
  174, 6, 105, 227, 34, 102, 3, 3, 185, 19, 73, 173, 42, 40, 200, 4,
  162, 70, 163, 92, 250, 34, 186, 126, 70, 126, 143, 184, 97, 161, 254, 226,
  211, 56, 10, 66, 115, 151, 152, 190, 174, 52, 114, 209, 149, 8, 143, 201,
  26, 173, 181, 4, 31, 71, 69, 245, 12, 141, 69, 18, 37, 119, 98, 194,
  107, 170, 241, 86, 184, 227, 233, 64, 218, 48, 246, 243, 1, 3, 190, 9,
  107, 149, 186, 81, 144, 18, 57, 100, 34, 155, 104, 79, 88, 152, 205, 83,
  10, 236, 93, 143, 27, 194, 123, 17, 216, 142, 111, 101, 34, 1, 188, 100,
  108, 200, 12, 190, 108, 226, 240, 157, 193, 23, 154, 26, 233, 248, 144, 223,
  71, 163, 196, 147, 27, 44, 193, 116, 152, 189, 166, 46, 144, 235, 68, 174,
  19, 152, 129, 82, 131, 59, 160, 121, 95, 71, 52, 17, 150, 192, 75, 22,
  131, 181, 206, 176, 148, 160, 228, 166, 141, 114, 163, 209, 240, 156, 117, 178,
  153, 207, 209, 115, 192, 162, 184, 174, 102, 85, 232, 212, 171, 18, 63, 207,
  197, 224, 229, 150, 71, 96, 129, 196, 196, 239, 118, 9, 139, 6, 135, 193,
  215, 134, 44, 246, 46, 122, 239, 92, 38, 195, 228, 92, 71, 11, 13, 171,
  196, 27, 85, 45, 27, 113, 44, 225, 135, 239, 45, 185, 227, 91, 99, 228,
  242, 177, 152, 198, 65, 152, 15, 99, 216, 23, 37, 141, 199, 54, 117, 199,
  141, 221, 189, 123, 146, 118, 97, 20, 229, 51, 206, 97, 55, 205, 117, 213,
  183, 80, 88, 198, 151, 84, 216, 253, 228, 177, 91, 181, 16, 203, 101, 80,
  178, 252, 82, 121, 199, 78, 60, 123, 212, 190, 84, 209, 105, 46, 49, 197,
  12, 89, 177, 252, 133, 83, 75, 206, 229, 45, 75, 219, 84, 49, 28, 28,
  15, 126, 34, 79, 114, 12, 84, 23, 18, 12, 253, 66, 251, 138, 12, 76,
  195, 229, 24, 143, 87, 246, 36, 154, 46, 18, 182, 21, 35, 88, 78, 130,
  199, 166, 94, 135, 87, 92, 10, 227, 130, 96, 137, 0, 161, 74, 98, 84,
  119, 216, 96, 187, 120, 192, 200, 211, 42, 207, 22, 203, 102, 192, 121, 100,
  152, 94, 142, 149, 198, 121, 111, 116, 40, 110, 151, 55, 204, 238, 143, 218,
  57, 30, 188, 119, 232, 228, 209, 14, 23, 168, 227, 227, 233, 65, 167, 141,
  110, 43, 8, 149, 119, 28, 237, 18, 88, 166, 218, 198, 223, 231, 224, 110,
  253, 207, 255, 20, 206, 14, 24, 47, 133, 198, 252, 157, 65, 18, 180, 122,
  61, 203, 69, 210, 135, 190, 9, 78, 132, 144, 43, 169, 131, 192, 170, 192,
  64, 52, 47, 21, 43, 7, 77, 209, 71, 160, 182, 115, 115, 180, 118, 109,
  128, 11, 179, 230, 249, 245, 132, 229, 97, 213, 165, 35, 243, 78, 1, 66,
  208, 95, 124, 203, 216, 231, 30, 47, 107, 203, 153, 140, 40, 234, 27, 186,
  139, 184, 61, 50, 82, 156, 171, 7, 55, 82, 244, 161, 202, 226, 11, 10,
  58, 225, 133, 70, 160, 97, 159, 0, 38, 178, 168, 250, 198, 11, 13, 61,
  255, 43, 17, 178, 172, 244, 148, 123, 228, 218, 62, 157, 173, 230, 39, 89,
  73, 189, 151, 177, 232, 193, 44, 94, 81, 120, 220, 75, 95, 123, 80, 229,
  118, 75, 175, 127, 208, 44, 24, 158, 253, 196, 164, 7, 49, 11, 83, 186,
  108, 129, 254, 255, 250, 237, 130, 166, 121, 159, 192, 101, 6, 196, 98, 143,
  184, 211, 201, 227, 52, 13, 111, 96, 225, 209, 223, 186, 176, 108, 97, 126,
  163, 184, 105, 71, 223, 197, 25, 163, 150, 8, 11, 235, 221, 167, 0, 93,
  144, 180, 196, 195, 161, 120, 108, 26, 166, 83, 61, 197, 91, 227, 121, 53,
  177, 85, 24, 21, 142, 100, 33, 173, 78, 147, 7, 156, 22, 229, 232, 183,
  36, 230, 224, 246, 189, 39, 156, 212, 44, 228, 226, 122, 209, 59, 98, 66,
  153, 255, 184, 144, 175, 128, 122, 214, 54, 217, 214, 221, 220, 93, 204, 189,
  164, 50, 120, 153, 21, 168, 180, 169, 24, 1, 34, 98, 48, 25, 137, 3,
  33, 46, 241, 115, 15, 65, 56, 47, 21, 221, 182, 247, 33, 239, 90, 5,
  129, 233, 152, 119, 118, 87, 141, 87, 27, 163, 89, 188, 144, 195, 36, 91,
  115, 41, 38, 73, 249, 61, 188, 119, 128, 188, 130, 243, 190, 141, 233, 245,
  122, 216, 60, 35, 41, 57, 108, 99, 4, 177, 51, 143, 79, 150, 218, 167,
  57, 32, 109, 127, 110, 95, 122, 52, 28, 241, 178, 102, 165, 85, 171, 225,
  13, 119, 118, 90, 52, 242, 174, 243, 94, 238, 26, 62, 190, 173, 151, 20,
  88, 153, 142, 101, 88, 179, 46, 129, 250, 4, 100, 174, 110, 158, 240, 109,
  147, 168, 84, 57, 218, 97, 213, 47, 238, 106, 231, 74, 232, 228, 183, 198,
  99, 118, 85, 189, 197, 164, 171, 226, 7, 100, 19, 144, 239, 156, 107, 73,
  136, 114, 189, 120, 164, 169, 128, 93, 120, 228, 62, 210, 82, 251, 230, 89,
  83, 113, 163, 114, 18, 70, 211, 131, 233, 208, 142, 16, 139, 202, 157, 213,
  28, 79, 46, 1, 113, 34, 110, 144, 232, 71, 57, 5, 240, 150, 54, 248,
  94, 179, 250, 137, 107, 13, 209, 107, 107, 109, 120, 56, 43, 70, 108, 185,
  96, 244, 202, 4, 93, 225, 235, 142, 122, 219, 107, 15, 106, 110, 65, 88,
  169, 113, 81, 108, 123, 61, 92, 59, 219, 242, 21, 75, 248, 11, 191, 223,
  124, 146, 219, 208, 237, 236, 26, 54, 29, 80, 53, 38, 173, 121, 244, 81,
  133, 154, 87, 219, 156, 82, 102, 253, 58, 204, 110, 103, 39, 122, 184, 203,
  183, 191, 29, 84, 237, 189, 119, 3, 144, 191, 209, 90, 106, 115, 153, 79,
  124, 160, 60, 241, 109, 61, 90, 229, 213, 65, 15, 115, 227, 96, 233, 108,
  75, 224, 194, 117, 21, 243, 193, 187, 91, 87, 26, 253, 209, 80, 170, 111,
  142, 92, 141, 227, 179, 213, 233, 25, 87, 144, 143, 216, 200, 160, 15, 34,
  104, 61, 15, 197, 130, 117, 93, 133, 180, 208, 131, 37, 221, 107, 148, 65,
  127, 45, 46, 226, 233, 80, 121, 26, 138, 37, 189, 146, 90, 167, 201, 204,
  170, 131, 41, 86, 13, 105, 163, 134, 220, 39, 73, 158, 39, 147, 178, 94,
  136, 92, 168, 190, 214, 51, 76, 87, 74, 152, 249, 54, 232, 182, 215, 27,
  126, 29, 177, 176, 47, 18, 76, 35, 13, 49, 90, 183, 48, 82, 150, 2,
  115, 213, 240, 235, 138, 45, 99, 212, 91, 198, 40, 53, 92, 64, 207, 229,
  61, 70, 29, 146, 144, 116, 90, 250, 96, 181, 204, 145, 240, 170, 242, 199,
  49, 45, 229, 74, 91, 109, 223, 103, 84, 234, 235, 203, 92, 88, 152, 250,
  5, 27, 88, 104, 207, 21, 162, 7, 124, 254, 73, 9, 154, 154, 129, 87,
  102, 135, 215, 69, 182, 101, 241, 21, 69, 174, 125, 198, 74, 61, 7, 119,
  151, 160, 239, 179, 14, 187, 58, 7, 34, 173, 232, 70, 27, 202, 239, 180,
  145, 215, 24, 202, 157, 152, 198, 141, 13, 178, 174, 205, 230, 170, 144, 190,
  201, 12, 104, 40, 95, 98, 5, 159, 177, 243, 104, 122, 28, 226, 141, 198,
  157, 98, 89, 39, 151, 236, 52, 169, 107, 228, 211, 12, 110, 180, 124, 188,
  167, 109, 230, 139, 158, 136, 137, 209, 203, 114, 190, 92, 247, 28, 68, 199,
  226, 104, 83, 48, 156, 119, 209, 123, 31, 130, 120, 159, 252, 113, 28, 157,
  35, 57, 213, 82, 236, 84, 173, 172, 24, 90, 115, 69, 104, 1, 188, 245,
  97, 190, 92, 99, 110, 0, 156, 99, 113, 145, 213, 92, 37, 91, 154, 33,
  76, 155, 182, 86, 208, 195, 126, 25, 140, 84, 123, 79, 151, 83, 124, 155,
  219, 119, 84, 200, 155, 18, 49, 246, 186, 152, 49, 49, 118, 245, 58, 121,
  60, 232, 59, 28, 76, 157, 220, 19, 113, 242, 212, 232, 222, 117, 18, 175,
  155, 26, 117, 184, 115, 120, 221, 212, 136, 179, 114, 218, 180, 151, 139, 74,
  123, 76, 166, 226, 104, 120, 221, 248, 165, 59, 110, 16, 144, 66, 38, 22,
  132, 52, 188, 94, 76, 74, 3, 70, 143, 89, 45, 67, 75, 121, 50, 91,
  138, 144, 140, 177, 132, 206, 245, 202, 70, 143, 94, 72, 66, 86, 50, 159,
  70, 57, 81, 191, 239, 56, 19, 51, 37, 119, 247, 109, 184, 22, 4, 190,
  251, 126, 193, 37, 180, 176, 219, 38, 2, 206, 182, 67, 27, 77, 211, 228,
  77, 189, 202, 147, 136, 27, 238, 30, 102, 198, 217, 40, 118, 3, 39, 25,
  55, 4, 235, 220, 82, 242, 43, 55, 153, 163, 227, 166, 3, 98, 158, 84,
  62, 137, 70, 134, 226, 111, 86, 34, 103, 181, 122, 42, 191, 141, 221, 231,
  74, 81, 211, 8, 2, 133, 106, 110, 95, 105, 114, 122, 158, 189, 37, 234,
  209, 39, 173, 145, 186, 126, 150, 164, 252, 186, 140, 56, 177, 249, 2, 139,
  74, 108, 38, 0, 249, 141, 136, 238, 39, 30, 50, 50, 152, 96, 189, 206,
  227, 164, 11, 17, 149, 219, 229, 10, 17, 85, 194, 118, 118, 180, 66, 117,
  149, 156, 130, 134, 162, 132, 79, 200, 64, 28, 84, 70, 132, 229, 0, 93,
  191, 48, 42, 144, 109, 226, 235, 94, 111, 184, 198, 252, 60, 222, 212, 6,
  172, 87, 176, 2, 131, 9, 170, 195, 36, 106, 172, 72, 145, 13, 26, 41,
  189, 69, 236, 215, 120, 204, 70, 220, 169, 103, 195, 93, 231, 93, 78, 222,
  154, 176, 19, 168, 65, 153, 229, 141, 221, 189, 79, 78, 68, 229, 175, 102,
  116, 231, 25, 159, 17, 175, 48, 192, 66, 46, 127, 96, 67, 88, 63, 242,
  54, 15, 105, 81, 92, 150, 248, 100, 225, 35, 157, 130, 13, 175, 178, 42,
  102, 77, 48, 77, 202, 90, 85, 116, 245, 173, 151, 85, 27, 98, 138, 73,
  74, 136, 113, 171, 160, 164, 85, 141, 142, 190, 213, 169, 200, 28, 14, 188,
  116, 67, 157, 104, 124, 114, 182, 58, 20, 60, 100, 23, 233, 69, 87, 233,
  109, 225, 236, 122, 80, 242, 182, 228, 1, 194, 96, 193, 38, 232, 112, 241,
  49, 250, 8, 107, 118, 167, 34, 177, 157, 103, 214, 125, 21, 151, 216, 199,
  56, 208, 124, 69, 215, 141, 154, 126, 148, 194, 75, 91, 152, 146, 57, 44,
  71, 139, 242, 211, 16, 84, 190, 119, 235, 205, 245, 247, 141, 197, 11, 67,
  107, 79, 91, 87, 108, 131, 61, 96, 103, 181, 59, 144, 186, 53, 17, 99,
  75, 232, 40, 89, 121, 178, 164, 38, 127, 44, 53, 9, 37, 125, 126, 239,
  120, 175, 105, 221, 163, 149, 230, 159, 35, 202, 178, 238, 29, 148, 56, 142,
  34, 17, 11, 22, 89, 119, 33, 216, 142, 163, 30, 131, 131, 127, 192, 61,
  140, 108, 225, 144, 23, 5, 194, 116, 64, 227, 56, 190, 105, 6, 160, 59,
  119, 132, 0, 123, 124, 72, 113, 16, 202, 113, 42, 131, 103, 91, 82, 156,
  96, 72, 101, 28, 180, 219, 222, 168, 130, 88, 111, 148, 58, 178, 138, 114,
  41, 163, 27, 42, 117, 175, 144, 192, 197, 131, 182, 218, 251, 248, 143, 29,
  59, 191, 216, 193, 196, 47, 207, 121, 14, 62, 233, 133, 236, 161, 41, 101,
  142, 170, 51, 78, 22, 34, 90, 100, 60, 5, 128, 71, 84, 161, 206, 235,
  161, 163, 178, 115, 26, 35, 64, 226, 105, 140, 105, 166, 23, 8, 10, 67,
  125, 201, 5, 105, 220, 52, 206, 248, 203, 17, 254, 140, 167, 252, 4, 224,
  112, 58, 66, 127, 188, 27, 167, 255, 194, 238, 47, 183, 20, 97, 241, 183,
  182, 89, 145, 108, 219, 209, 69, 29, 216, 109, 156, 7, 102, 239, 182, 13,
  249, 226, 115, 234, 79, 22, 232, 15, 83, 136, 61, 197, 226, 119, 197, 139,
  5, 193, 67, 213, 111, 123, 85, 6, 250, 136, 96, 217, 29, 79, 182, 12,
  117, 58, 195, 232, 154, 226, 168, 163, 47, 126, 181, 241, 118, 139, 56, 178,
  16, 41, 238, 178, 52, 79, 45, 77, 202, 189, 45, 217, 27, 196, 220, 98,
  251, 229, 164, 53, 14, 167, 160, 163, 210, 13, 147, 67, 84, 70, 66, 74,
  174, 179, 75, 73, 151, 20, 12, 134, 95, 104, 124, 205, 178, 36, 134, 130,
  252, 76, 176, 248, 226, 215, 87, 158, 77, 221, 99, 118, 37, 78, 211, 99,
  182, 156, 110, 179, 119, 10, 176, 125, 147, 149, 103, 148, 93, 86, 82, 49,
  90, 77, 108, 236, 251, 155, 34, 60, 43, 127, 239, 135, 200, 81, 36, 160,
  28, 82, 22, 186, 85, 94, 133, 166, 67, 204, 221, 128, 93, 182, 7, 243,
  20, 250, 149, 243, 88, 88, 95, 38, 146, 25, 229, 253, 137, 131, 207, 147,
  249, 96, 204, 232, 101, 144, 226, 171, 56, 41, 43, 210, 240, 241, 21, 89,
  179, 79, 136, 241, 15, 63, 244, 31, 239, 13, 253, 71, 29, 250, 143, 54,
  244, 107, 114, 118, 230, 88, 180, 168, 251, 0, 109, 228, 140, 203, 141, 42,
  246, 163, 44, 6, 26, 156, 57, 63, 215, 244, 150, 18, 77, 140, 46, 218,
  193, 116, 93, 211, 243, 72, 78, 206, 119, 42, 81, 153, 26, 161, 240, 141,
  3, 6, 181, 59, 202, 216, 243, 100, 24, 64, 184, 80, 231, 6, 165, 87,
  132, 41, 2, 18, 129, 128, 9, 187, 48, 109, 149, 252, 84, 22, 83, 60,
  207, 203, 171, 75, 172, 22, 0, 30, 0, 211, 4, 49, 148, 113, 60, 189,
  64, 180, 130, 24, 13, 55, 211, 30, 122, 177, 203, 143, 163, 33, 72, 47,
  73, 156, 71, 179, 186, 117, 94, 200, 151, 99, 221, 115, 138, 88, 174, 249,
  10, 47, 14, 49, 74, 250, 153, 135, 145, 68, 2, 183, 27, 111, 198, 174,
  201, 79, 97, 190, 3, 98, 64, 71, 205, 250, 117, 1, 70, 155, 217, 134,
  229, 92, 102, 79, 115, 131, 4, 137, 46, 42, 116, 166, 219, 133, 140, 68,
  75, 187, 34, 93, 76, 88, 176, 63, 90, 28, 66, 148, 255, 255, 9, 192,
  98, 175, 97, 49, 141, 244, 5, 171, 233, 197, 193, 233, 235, 195, 253, 15,
  240, 231, 177, 25, 62, 4, 109, 46, 253, 90, 77, 123, 3, 172, 171, 78,
  232, 121, 65, 88, 250, 63, 204, 102, 44, 221, 15, 51, 12, 209, 230, 127,
  3, 84, 143, 5, 78, 238, 45, 124, 106, 208, 147, 134, 4, 3, 149, 162,
  246, 72, 89, 66, 156, 236, 235, 97, 83, 250, 198, 198, 100, 182, 121, 151,
  233, 213, 49, 243, 188, 246, 116, 151, 73, 174, 0, 85, 49, 213, 254, 90,
  190, 121, 53, 173, 144, 56, 107, 133, 231, 132, 111, 26, 188, 7, 236, 167,
  60, 80, 173, 12, 204, 36, 71, 120, 145, 115, 155, 42, 71, 151, 161, 63,
  126, 243, 73, 37, 168, 147, 65, 66, 200, 240, 204, 105, 224, 149, 28, 74,
  70, 10, 34, 33, 167, 118, 251, 177, 13, 152, 78, 234, 165, 239, 232, 160,
  126, 47, 144, 180, 29, 131, 53, 130, 145, 164, 75, 14, 75, 46, 59, 106,
  58, 204, 173, 233, 13, 175, 241, 168, 202, 145, 167, 233, 191, 191, 168, 53,
  93, 28, 218, 55, 11, 87, 117, 31, 54, 190, 203, 67, 174, 208, 144, 76,
  129, 146, 166, 140, 28, 118, 40, 56, 13, 151, 57, 208, 146, 53, 205, 15,
  48, 84, 241, 52, 47, 194, 136, 54, 22, 11, 35, 217, 56, 185, 146, 108,
  227, 83, 32, 204, 198, 138, 12, 154, 106, 172, 203, 22, 156, 129, 145, 60,
  155, 255, 83, 95, 138, 7, 50, 229, 199, 190, 146, 4, 170, 110, 186, 135,
  121, 14, 226, 190, 192, 231, 57, 9, 163, 105, 225, 196, 244, 89, 34, 168,
  230, 151, 86, 42, 82, 194, 18, 20, 141, 226, 216, 50, 254, 154, 200, 23,
  149, 137, 221, 59, 204, 161, 209, 102, 99, 193, 131, 4, 229, 194, 238, 111,
  101, 203, 42, 30, 252, 105, 252, 98, 219, 151, 190, 121, 240, 5, 0, 98,
  196, 193, 37, 144, 23, 250, 107, 49, 160, 198, 122, 109, 146, 204, 51, 122,
  129, 149, 46, 202, 169, 65, 110, 44, 81, 149, 164, 97, 183, 42, 158, 18,
  207, 194, 44, 139, 46, 25, 87, 214, 205, 99, 213, 74, 68, 104, 158, 209,
  105, 77, 155, 239, 165, 81, 193, 167, 159, 74, 170, 170, 37, 20, 34, 67,
  50, 189, 57, 69, 96, 1, 103, 13, 96, 242, 190, 225, 160, 137, 118, 88,
  116, 204, 56, 72, 83, 124, 122, 225, 69, 4, 157, 156, 158, 7, 79, 95,
  189, 8, 24, 231, 41, 125, 174, 179, 213, 26, 62, 11, 173, 102, 34, 231,
  239, 55, 188, 77, 67, 220, 99, 164, 166, 183, 95, 228, 192, 206, 96, 39,
  129, 146, 145, 100, 192, 81, 234, 53, 226, 62, 181, 134, 55, 172, 159, 11,
  189, 225, 105, 209, 121, 5, 165, 54, 138, 217, 181, 245, 112, 85, 114, 33,
  47, 149, 59, 190, 77, 151, 64, 216, 248, 206, 133, 90, 105, 153, 116, 163,
  155, 224, 194, 228, 105, 207, 67, 17, 162, 205, 241, 230, 86, 245, 219, 226,
  210, 73, 86, 167, 222, 138, 160, 247, 228, 16, 165, 138, 72, 95, 59, 99,
  181, 23, 165, 49, 24, 157, 44, 11, 122, 153, 189, 208, 181, 146, 124, 4,
  26, 129, 147, 212, 150, 81, 237, 180, 156, 133, 18, 159, 46, 226, 105, 245,
  202, 229, 60, 223, 83, 175, 55, 20, 37, 174, 183, 14, 178, 250, 102, 71,
  254, 211, 53, 66, 231, 211, 59, 156, 60, 66, 216, 187, 247, 206, 219, 10,
  105, 254, 154, 187, 239, 233, 33, 154, 237, 244, 237, 146, 116, 17, 174, 217,
  56, 180, 227, 45, 133, 87, 97, 132, 47, 152, 135, 67, 30, 94, 67, 134,
  83, 209, 58, 218, 212, 192, 53, 42, 47, 73, 122, 95, 180, 93, 2, 146,
  227, 203, 140, 193, 210, 233, 173, 22, 249, 19, 180, 22, 17, 70, 84, 39,
  159, 157, 223, 249, 207, 164, 232, 175, 170, 242, 148, 96, 152, 120, 72, 184,
  37, 46, 188, 56, 5, 220, 251, 86, 22, 84, 178, 251, 199, 63, 178, 244,
  231, 191, 229, 236, 240, 105, 31, 4, 53, 153, 125, 251, 81, 136, 3, 133,
  255, 41, 13, 134, 235, 80, 203, 15, 34, 96, 99, 179, 183, 32, 88, 10,
  86, 82, 91, 123, 38, 206, 88, 53, 143, 156, 253, 203, 255, 74, 119, 74,
  61, 177, 252, 216, 11, 102, 71, 62, 98, 77, 66, 84, 159, 25, 233, 204,
  254, 41, 144, 81, 132, 212, 249, 113, 63, 80, 146, 169, 254, 184, 177, 43,
  197, 146, 179, 15, 119, 223, 233, 111, 146, 107, 143, 248, 64, 71, 30, 238,
  238, 210, 223, 144, 222, 65, 125, 83, 204, 212, 215, 216, 2, 89, 83, 202,
  133, 125, 191, 212, 73, 163, 212, 84, 225, 1, 251, 129, 119, 29, 61, 226,
  203, 243, 219, 181, 78, 208, 247, 175, 40, 89, 98, 219, 87, 130, 214, 150,
  42, 2, 95, 24, 229, 149, 251, 162, 234, 187, 33, 241, 216, 175, 190, 130,
  41, 177, 78, 167, 46, 112, 230, 83, 52, 63, 9, 235, 158, 195, 254, 132,
  93, 80, 74, 42, 100, 41, 45, 42, 200, 233, 226, 196, 210, 15, 12, 58,
  39, 238, 164, 79, 90, 172, 77, 163, 82, 132, 180, 42, 229, 74, 169, 111,
  189, 3, 83, 205, 153, 139, 100, 89, 228, 97, 204, 35, 42, 60, 98, 231,
  20, 69, 156, 139, 50, 252, 203, 206, 109, 226, 144, 61, 10, 10, 119, 248,
  247, 13, 15, 83, 212, 182, 77, 9, 78, 75, 114, 202, 136, 85, 106, 111,
  83, 201, 69, 227, 142, 29, 11, 150, 16, 33, 75, 37, 56, 227, 170, 130,
  231, 138, 130, 181, 68, 237, 99, 53, 251, 134, 66, 81, 92, 93, 83, 208,
  146, 156, 187, 10, 28, 198, 2, 23, 91, 81, 136, 59, 216, 214, 186, 61,
  221, 175, 182, 230, 131, 197, 95, 29, 253, 158, 161, 99, 10, 222, 38, 157,
  194, 150, 8, 43, 191, 187, 181, 212, 235, 53, 200, 163, 72, 161, 40, 127,
  146, 61, 191, 126, 46, 88, 39, 141, 60, 239, 93, 33, 65, 91, 51, 170,
  138, 11, 129, 238, 57, 63, 207, 19, 218, 142, 46, 220, 125, 244, 10, 119,
  156, 77, 11, 131, 55, 250, 250, 169, 71, 195, 62, 86, 73, 124, 210, 64,
  110, 181, 232, 181, 192, 43, 169, 73, 244, 24, 49, 29, 243, 180, 195, 41,
  247, 205, 87, 175, 193, 99, 244, 67, 251, 133, 32, 255, 165, 22, 106, 214,
  158, 109, 129, 203, 61, 46, 182, 44, 182, 139, 85, 63, 99, 143, 136, 239,
  44, 150, 171, 38, 119, 23, 169, 170, 132, 132, 201, 61, 228, 131, 201, 34,
  209, 96, 172, 147, 159, 79, 135, 195, 0, 71, 203, 238, 194, 214, 132, 201,
  93, 24, 199, 75, 237, 191, 174, 83, 146, 35, 148, 248, 101, 14, 195, 101,
  73, 219, 135, 245, 100, 125, 71, 214, 221, 162, 120, 226, 154, 158, 38, 221,
  162, 173, 162, 234, 88, 185, 223, 53, 220, 190, 252, 98, 129, 94, 196, 43,
  79, 232, 189, 245, 137, 5, 75, 89, 159, 140, 183, 248, 170, 100, 3, 173,
  96, 33, 30, 200, 43, 34, 122, 174, 118, 183, 162, 31, 248, 99, 26, 147,
  123, 177, 189, 51, 84, 220, 110, 252, 124, 81, 224, 221, 71, 106, 163, 85,
  48, 165, 247, 139, 4, 131, 101, 101, 129, 201, 23, 17, 3, 60, 8, 150,
  236, 157, 247, 52, 131, 220, 86, 111, 224, 191, 18, 119, 116, 54, 180, 66,
  46, 17, 59, 77, 137, 253, 78, 223, 225, 185, 77, 170, 41, 76, 93, 134,
  248, 198, 147, 164, 177, 84, 90, 48, 20, 2, 175, 232, 41, 209, 54, 6,
  194, 199, 171, 221, 198, 190, 88, 184, 10, 212, 223, 73, 10, 144, 125, 125,
  111, 25, 213, 8, 107, 185, 97, 2, 9, 58, 251, 101, 73, 47, 204, 98,
  205, 192, 153, 116, 217, 131, 137, 217, 31, 98, 114, 252, 183, 179, 241, 23,
  134, 28, 245, 171, 120, 116, 111, 127, 241, 208, 58, 5, 155, 202, 102, 167,
  218, 212, 222, 221, 179, 210, 200, 52, 174, 135, 155, 44, 92, 130, 81, 61,
  167, 254, 189, 38, 218, 122, 29, 142, 44, 106, 21, 79, 125, 185, 38, 43,
  78, 140, 170, 121, 115, 220, 77, 144, 13, 36, 181, 1, 139, 31, 79, 129,
  105, 162, 100, 244, 44, 13, 39, 204, 46, 84, 16, 160, 131, 80, 202, 254,
  117, 206, 178, 220, 170, 239, 90, 81, 23, 135, 206, 12, 170, 123, 107, 4,
  104, 45, 70, 201, 144, 232, 158, 22, 115, 97, 132, 239, 215, 46, 79, 67,
  9, 199, 30, 39, 166, 48, 191, 38, 145, 168, 112, 87, 248, 202, 67, 4,
  85, 247, 50, 127, 125, 17, 97, 41, 59, 79, 113, 3, 135, 14, 34, 176,
  124, 113, 46, 66, 129, 240, 64, 237, 249, 164, 109, 220, 27, 186, 2, 189,
  165, 235, 179, 230, 69, 70, 224, 205, 125, 221, 129, 200, 120, 250, 203, 130,
  233, 40, 229, 238, 46, 88, 14, 78, 168, 205, 75, 194, 227, 131, 242, 109,
  0, 250, 117, 41, 196, 237, 123, 65, 220, 174, 128, 40, 213, 242, 59, 130,
  68, 253, 221, 132, 25, 44, 11, 66, 41, 253, 158, 16, 44, 62, 171, 143,
  56, 53, 48, 237, 62, 60, 81, 183, 252, 136, 159, 254, 98, 178, 187, 60,
  82, 237, 35, 207, 161, 196, 242, 150, 33, 15, 231, 84, 43, 209, 31, 238,
  96, 25, 9, 149, 11, 25, 75, 90, 205, 44, 217, 213, 146, 19, 31, 57,
  130, 162, 41, 107, 90, 49, 14, 254, 113, 18, 101, 65, 46, 218, 50, 47,
  238, 187, 249, 229, 112, 163, 168, 76, 245, 136, 230, 27, 126, 217, 85, 171,
  45, 19, 127, 243, 98, 108, 113, 112, 246, 139, 137, 174, 156, 96, 239, 97,
  199, 226, 168, 149, 8, 173, 165, 7, 126, 75, 73, 171, 188, 182, 215, 6,
  166, 103, 9, 3, 216, 253, 86, 20, 172, 36, 159, 205, 76, 44, 232, 106,
  163, 153, 83, 200, 181, 154, 201, 13, 246, 108, 126, 254, 36, 185, 214, 30,
  241, 197, 33, 124, 138, 201, 246, 83, 190, 188, 245, 203, 23, 86, 72, 109,
  39, 144, 129, 253, 54, 198, 172, 60, 206, 144, 199, 166, 198, 3, 252, 152,
  1, 99, 30, 249, 2, 254, 24, 30, 24, 38, 227, 232, 121, 220, 44, 204,
  184, 64, 126, 248, 70, 156, 160, 59, 195, 151, 99, 105, 61, 206, 251, 81,
  170, 246, 223, 124, 50, 90, 189, 13, 86, 33, 201, 136, 97, 113, 27, 252,
  253, 223, 254, 43, 141, 0, 8, 224, 209, 84, 124, 2, 194, 240, 25, 94,
  155, 33, 138, 85, 107, 206, 241, 224, 25, 178, 182, 218, 130, 229, 82, 86,
  121, 154, 24, 143, 16, 187, 81, 184, 47, 42, 60, 29, 46, 207, 235, 97,
  154, 186, 150, 63, 62, 249, 20, 37, 3, 242, 239, 74, 25, 36, 81, 74,
  16, 118, 56, 143, 151, 225, 75, 199, 31, 88, 149, 149, 151, 235, 49, 10,
  199, 238, 94, 248, 221, 89, 147, 34, 47, 91, 176, 22, 157, 59, 187, 71,
  107, 95, 246, 33, 171, 11, 116, 76, 64, 231, 34, 161, 115, 129, 246, 85,
  128, 208, 39, 93, 23, 173, 65, 111, 98, 239, 160, 38, 40, 130, 118, 138,
  231, 244, 209, 142, 167, 33, 79, 31, 237, 244, 237, 146, 116, 207, 233, 35,
  33, 131, 15, 73, 205, 135, 156, 163, 178, 41, 30, 235, 82, 132, 12, 45,
  117, 160, 222, 137, 54, 211, 249, 195, 130, 24, 193, 207, 118, 55, 247, 116,
  167, 196, 50, 73, 177, 199, 203, 222, 224, 153, 81, 60, 121, 127, 156, 112,
  187, 9, 255, 141, 137, 226, 23, 190, 190, 247, 201, 186, 115, 54, 193, 195,
  249, 161, 53, 214, 191, 216, 104, 107, 234, 137, 239, 241, 48, 33, 206, 211,
  249, 87, 221, 219, 204, 163, 128, 142, 190, 0, 180, 55, 240, 186, 12, 44,
  227, 125, 227, 214, 123, 191, 64, 63, 124, 30, 177, 124, 48, 254, 151, 44,
  153, 214, 63, 174, 134, 179, 104, 149, 230, 121, 117, 8, 99, 124, 243, 40,
  99, 211, 44, 73, 119, 21, 117, 255, 1, 31, 170, 133, 79, 126, 13, 13,
  63, 110, 255, 144, 39, 42, 33, 79, 208, 60, 111, 52, 168, 70, 186, 206,
  159, 177, 90, 28, 76, 205, 242, 219, 149, 175, 68, 206, 222, 117, 222, 55,
  118, 188, 229, 128, 51, 191, 58, 251, 43, 150, 125, 215, 229, 54, 236, 219,
  210, 130, 120, 23, 90, 189, 239, 8, 181, 218, 192, 247, 76, 63, 194, 71,
  122, 142, 251, 106, 138, 29, 85, 45, 183, 30, 168, 52, 30, 153, 148, 15,
  237, 57, 247, 35, 204, 123, 62, 46, 143, 95, 106, 166, 132, 52, 253, 136,
  79, 142, 54, 75, 52, 27, 187, 38, 39, 250, 130, 243, 34, 98, 176, 208,
  104, 151, 95, 114, 129, 92, 114, 188, 164, 220, 62, 159, 199, 103, 192, 142,
  115, 158, 181, 115, 207, 41, 191, 219, 216, 107, 143, 121, 46, 126, 202, 83,
  62, 228, 25, 248, 66, 241, 217, 147, 102, 198, 88, 19, 47, 167, 141, 224,
  15, 31, 80, 212, 75, 62, 136, 23, 29, 220, 75, 59, 222, 248, 251, 62,
  110, 82, 249, 112, 131, 247, 93, 55, 62, 177, 86, 52, 90, 133, 163, 221,
  5, 52, 186, 210, 57, 92, 86, 175, 61, 62, 62, 12, 116, 186, 170, 149,
  94, 70, 243, 112, 121, 222, 172, 46, 183, 12, 66, 160, 214, 160, 206, 76,
  153, 130, 15, 91, 2, 114, 203, 85, 152, 78, 235, 53, 209, 86, 192, 16,
  137, 90, 19, 255, 122, 228, 202, 89, 202, 70, 17, 138, 126, 95, 138, 61,
  63, 10, 138, 238, 106, 12, 47, 24, 193, 191, 12, 106, 246, 3, 103, 56,
  100, 158, 253, 214, 176, 56, 198, 4, 105, 144, 144, 188, 69, 89, 145, 161,
  160, 196, 178, 44, 60, 103, 214, 202, 91, 176, 223, 59, 34, 91, 217, 110,
  87, 38, 239, 204, 82, 32, 60, 73, 19, 250, 192, 227, 182, 71, 122, 243,
  49, 150, 24, 186, 30, 103, 20, 81, 197, 240, 238, 132, 117, 35, 156, 206,
  204, 25, 212, 74, 242, 173, 7, 10, 190, 35, 171, 50, 17, 230, 123, 123,
  247, 179, 119, 224, 47, 197, 222, 232, 175, 195, 217, 10, 93, 225, 62, 28,
  238, 23, 229, 113, 203, 115, 185, 127, 36, 159, 115, 57, 221, 189, 121, 157,
  244, 6, 40, 232, 174, 81, 58, 226, 60, 82, 53, 221, 91, 229, 83, 247,
  142, 255, 177, 194, 215, 185, 35, 53, 159, 225, 123, 26, 251, 202, 38, 84,
  255, 164, 153, 138, 20, 130, 220, 37, 73, 67, 85, 203, 251, 32, 29, 154,
  24, 26, 100, 62, 76, 64, 223, 83, 216, 220, 250, 102, 209, 88, 71, 116,
  139, 222, 30, 65, 235, 187, 146, 85, 243, 24, 24, 229, 92, 218, 228, 26,
  134, 174, 33, 150, 128, 143, 3, 87, 115, 97, 139, 123, 149, 48, 187, 106,
  94, 102, 221, 243, 244, 48, 49, 239, 115, 98, 124, 190, 30, 95, 178, 20,
  64, 102, 191, 236, 131, 98, 118, 91, 238, 67, 166, 44, 127, 140, 18, 96,
  80, 199, 123, 174, 210, 23, 196, 235, 243, 78, 87, 88, 184, 161, 37, 26,
  250, 140, 31, 11, 252, 42, 252, 238, 20, 38, 195, 65, 45, 220, 171, 48,
  241, 45, 223, 113, 175, 199, 176, 56, 44, 182, 140, 22, 206, 122, 199, 237,
  14, 13, 30, 1, 9, 163, 85, 183, 95, 188, 46, 237, 124, 140, 234, 53,
  64, 238, 104, 126, 93, 195, 7, 190, 225, 79, 195, 91, 96, 63, 233, 97,
  129, 1, 252, 241, 23, 56, 229, 239, 180, 241, 247, 218, 252, 69, 158, 207,
  39, 88, 66, 189, 207, 230, 47, 245, 134, 30, 98, 163, 247, 216, 42, 8,
  0, 178, 201, 146, 86, 135, 31, 246, 73, 23, 190, 46, 86, 202, 219, 177,
  188, 50, 84, 212, 210, 243, 179, 176, 222, 221, 104, 246, 214, 154, 235, 189,
  102, 167, 81, 243, 60, 104, 77, 175, 210, 189, 251, 157, 126, 112, 1, 156,
  185, 211, 198, 167, 246, 250, 193, 187, 141, 237, 102, 119, 173, 211, 236, 173,
  111, 190, 15, 110, 155, 110, 177, 77, 94, 108, 109, 189, 217, 221, 126, 208,
  220, 94, 247, 149, 234, 182, 187, 188, 84, 15, 139, 61, 216, 110, 110, 249,
  75, 9, 88, 189, 245, 173, 102, 183, 187, 134, 255, 25, 229, 222, 187, 183,
  115, 129, 221, 233, 161, 185, 149, 73, 13, 134, 1, 131, 21, 54, 131, 94,
  187, 99, 69, 196, 139, 134, 215, 252, 173, 206, 25, 58, 165, 79, 135, 135,
  211, 33, 187, 230, 1, 87, 57, 60, 124, 169, 193, 8, 249, 64, 15, 58,
  65, 45, 253, 205, 199, 224, 35, 140, 109, 253, 155, 79, 4, 135, 238, 251,
  182, 255, 10, 91, 117, 29, 132, 137, 198, 109, 227, 163, 227, 17, 47, 155,
  196, 16, 86, 173, 238, 251, 102, 112, 166, 167, 56, 61, 227, 151, 146, 8,
  157, 86, 16, 98, 248, 151, 213, 160, 126, 70, 33, 179, 67, 79, 140, 214,
  9, 73, 153, 33, 32, 129, 18, 66, 253, 178, 25, 21, 129, 29, 121, 144,
  194, 75, 140, 242, 114, 214, 30, 188, 139, 222, 183, 46, 27, 223, 230, 13,
  247, 74, 187, 232, 17, 192, 242, 118, 197, 243, 206, 81, 24, 15, 128, 156,
  233, 197, 192, 125, 122, 146, 143, 8, 223, 166, 87, 202, 54, 40, 86, 150,
  84, 137, 37, 33, 19, 188, 79, 24, 17, 56, 255, 11, 70, 10, 131, 202,
  55, 234, 201, 96, 12, 132, 219, 237, 108, 5, 223, 242, 49, 98, 215, 48,
  106, 221, 7, 237, 222, 3, 72, 17, 13, 172, 74, 204, 191, 11, 122, 107,
  15, 218, 107, 238, 241, 28, 195, 89, 101, 120, 254, 167, 90, 230, 111, 172,
  181, 59, 190, 87, 25, 121, 225, 22, 84, 51, 123, 88, 190, 126, 77, 236,
  69, 162, 30, 130, 85, 17, 126, 175, 189, 65, 207, 29, 86, 61, 75, 37,
  113, 196, 224, 25, 197, 172, 121, 24, 140, 59, 97, 58, 203, 249, 66, 115,
  85, 209, 209, 47, 54, 77, 94, 156, 48, 148, 54, 32, 4, 224, 141, 245,
  236, 61, 239, 196, 216, 39, 245, 46, 134, 52, 194, 17, 88, 13, 40, 186,
  7, 157, 204, 87, 76, 97, 58, 94, 118, 6, 215, 244, 41, 4, 160, 32,
  83, 141, 171, 102, 16, 229, 18, 152, 186, 231, 44, 204, 113, 157, 171, 168,
  155, 210, 147, 134, 159, 178, 242, 136, 151, 240, 137, 242, 72, 28, 222, 28,
  196, 77, 122, 76, 155, 225, 123, 45, 174, 223, 132, 128, 194, 93, 94, 75,
  238, 245, 45, 27, 11, 157, 70, 237, 170, 136, 225, 112, 229, 139, 68, 54,
  46, 242, 199, 86, 28, 40, 127, 48, 245, 171, 166, 251, 220, 40, 206, 57,
  127, 235, 170, 187, 222, 228, 95, 116, 200, 178, 182, 225, 121, 69, 149, 23,
  196, 209, 198, 15, 42, 215, 117, 125, 43, 196, 131, 39, 72, 109, 117, 225,
  149, 195, 15, 200, 142, 65, 236, 136, 95, 163, 99, 12, 197, 171, 111, 120,
  98, 0, 223, 240, 119, 76, 48, 124, 194, 24, 255, 168, 151, 75, 92, 121,
  140, 94, 16, 221, 85, 40, 3, 109, 33, 215, 24, 35, 93, 213, 139, 68,
  209, 63, 67, 114, 82, 173, 93, 243, 214, 48, 230, 195, 21, 254, 241, 182,
  166, 245, 31, 221, 79, 249, 40, 124, 135, 145, 34, 86, 131, 171, 134, 96,
  90, 188, 49, 158, 233, 181, 19, 113, 190, 165, 115, 122, 26, 196, 37, 2,
  34, 25, 34, 76, 121, 113, 154, 229, 235, 102, 112, 211, 164, 62, 240, 127,
  27, 139, 172, 227, 196, 166, 56, 157, 83, 232, 39, 254, 179, 29, 107, 1,
  178, 245, 244, 49, 144, 153, 22, 57, 219, 157, 20, 174, 55, 33, 199, 89,
  235, 149, 69, 70, 84, 126, 239, 36, 85, 41, 129, 167, 217, 105, 119, 183,
  26, 75, 196, 170, 171, 125, 29, 110, 140, 214, 70, 131, 218, 162, 120, 117,
  119, 9, 69, 103, 5, 148, 21, 253, 216, 113, 30, 152, 210, 5, 11, 185,
  118, 128, 24, 162, 213, 186, 168, 210, 234, 54, 26, 223, 74, 2, 108, 249,
  200, 79, 35, 168, 231, 56, 158, 187, 206, 142, 210, 212, 39, 194, 99, 63,
  84, 53, 213, 76, 224, 181, 231, 104, 106, 43, 162, 69, 116, 148, 186, 170,
  163, 168, 116, 85, 208, 109, 75, 124, 55, 190, 189, 42, 139, 74, 199, 89,
  119, 174, 45, 167, 85, 207, 18, 107, 124, 59, 118, 81, 141, 0, 71, 216,
  28, 188, 113, 232, 200, 236, 237, 139, 59, 87, 98, 119, 212, 230, 72, 13,
  246, 78, 180, 135, 19, 214, 106, 253, 146, 211, 116, 68, 177, 232, 203, 103,
  9, 151, 133, 127, 154, 168, 230, 221, 102, 9, 171, 252, 138, 147, 116, 135,
  241, 47, 118, 149, 36, 99, 229, 33, 238, 220, 200, 105, 21, 209, 2, 237,
  48, 208, 242, 37, 135, 98, 163, 229, 54, 59, 205, 74, 224, 68, 132, 92,
  20, 80, 151, 226, 40, 34, 251, 86, 195, 138, 210, 142, 205, 181, 145, 149,
  95, 237, 252, 174, 124, 104, 49, 12, 23, 35, 69, 66, 14, 166, 20, 154,
  172, 33, 6, 64, 227, 69, 204, 143, 180, 167, 93, 229, 112, 33, 141, 92,
  93, 212, 213, 191, 102, 163, 117, 248, 63, 50, 67, 115, 205, 19, 120, 228,
  38, 106, 159, 107, 192, 36, 55, 151, 227, 145, 158, 152, 116, 159, 203, 35,
  245, 112, 122, 124, 167, 169, 232, 197, 38, 96, 111, 196, 218, 251, 182, 247,
  249, 212, 98, 199, 150, 148, 130, 153, 111, 43, 138, 201, 141, 7, 35, 10,
  202, 82, 237, 127, 157, 179, 244, 134, 59, 213, 224, 189, 114, 124, 95, 188,
  197, 139, 213, 220, 200, 5, 194, 65, 200, 228, 42, 60, 181, 29, 77, 167,
  44, 125, 126, 250, 226, 8, 15, 255, 45, 179, 218, 67, 138, 85, 4, 58,
  103, 150, 237, 174, 240, 242, 173, 236, 10, 237, 112, 43, 1, 185, 108, 236,
  174, 156, 133, 131, 139, 115, 210, 39, 251, 223, 124, 82, 251, 123, 7, 132,
  238, 219, 149, 189, 135, 171, 8, 96, 111, 26, 177, 97, 26, 157, 127, 57,
  224, 219, 26, 240, 104, 200, 194, 248, 75, 129, 238, 182, 55, 52, 208, 227,
  100, 48, 54, 32, 127, 172, 150, 65, 92, 233, 60, 153, 177, 233, 83, 233,
  16, 30, 198, 158, 59, 105, 186, 11, 185, 186, 26, 191, 99, 103, 147, 119,
  166, 56, 92, 179, 51, 129, 129, 191, 137, 24, 114, 245, 218, 152, 107, 1,
  181, 187, 222, 147, 170, 136, 38, 212, 148, 151, 168, 44, 51, 26, 191, 165,
  199, 239, 24, 96, 215, 78, 163, 60, 70, 207, 244, 143, 188, 187, 225, 52,
  139, 6, 227, 188, 47, 67, 208, 196, 252, 89, 104, 71, 102, 63, 11, 135,
  116, 224, 175, 123, 125, 17, 188, 39, 152, 97, 187, 125, 81, 233, 6, 175,
  100, 89, 10, 141, 64, 55, 246, 40, 202, 215, 96, 206, 142, 96, 0, 139,
  214, 68, 66, 173, 225, 43, 25, 102, 249, 230, 216, 44, 75, 73, 165, 165,
  123, 235, 110, 113, 50, 117, 90, 79, 18, 242, 70, 27, 18, 157, 54, 145,
  41, 61, 134, 20, 14, 135, 245, 26, 198, 64, 185, 100, 190, 90, 212, 124,
  163, 64, 78, 171, 153, 82, 56, 142, 5, 149, 1, 153, 134, 134, 236, 82,
  213, 133, 40, 60, 78, 80, 201, 84, 244, 201, 15, 102, 208, 94, 105, 208,
  196, 83, 238, 191, 85, 199, 28, 36, 201, 211, 240, 12, 95, 147, 18, 181,
  155, 34, 98, 132, 223, 145, 80, 119, 197, 119, 146, 28, 255, 48, 137, 208,
  35, 225, 41, 134, 219, 139, 237, 111, 38, 236, 162, 66, 51, 150, 176, 237,
  148, 42, 208, 28, 34, 194, 182, 220, 209, 160, 175, 68, 164, 111, 162, 44,
  58, 139, 89, 221, 161, 222, 26, 172, 27, 199, 222, 95, 220, 5, 169, 178,
  243, 147, 68, 162, 243, 12, 157, 91, 44, 213, 48, 157, 134, 218, 87, 59,
  42, 221, 77, 189, 206, 166, 101, 231, 178, 116, 184, 73, 6, 125, 29, 51,
  251, 100, 180, 242, 84, 180, 70, 7, 52, 57, 221, 145, 205, 237, 227, 121,
  122, 116, 72, 102, 61, 198, 181, 93, 156, 16, 234, 229, 156, 195, 50, 108,
  204, 115, 152, 193, 184, 107, 178, 118, 78, 73, 167, 101, 242, 128, 204, 119,
  252, 145, 76, 19, 183, 2, 166, 230, 201, 52, 26, 64, 37, 60, 98, 160,
  196, 169, 165, 212, 88, 175, 213, 249, 79, 22, 41, 90, 164, 192, 74, 106,
  160, 46, 144, 60, 3, 233, 252, 132, 68, 124, 36, 75, 225, 89, 174, 42,
  246, 241, 142, 2, 225, 169, 29, 168, 138, 20, 253, 80, 213, 55, 102, 220,
  25, 14, 38, 19, 7, 110, 46, 250, 23, 206, 62, 168, 91, 0, 93, 126,
  141, 180, 125, 21, 141, 162, 15, 133, 239, 28, 101, 121, 31, 247, 200, 240,
  137, 50, 43, 154, 63, 76, 229, 252, 186, 111, 14, 34, 164, 124, 192, 210,
  141, 166, 81, 112, 54, 27, 13, 151, 43, 57, 72, 122, 86, 65, 72, 241,
  21, 68, 17, 214, 42, 137, 73, 190, 162, 82, 27, 178, 138, 203, 100, 95,
  149, 152, 133, 35, 27, 97, 72, 242, 21, 5, 142, 99, 149, 132, 20, 94,
  80, 151, 38, 244, 81, 85, 1, 114, 132, 98, 32, 185, 238, 158, 87, 49,
  228, 231, 109, 8, 217, 189, 194, 110, 148, 138, 134, 134, 207, 216, 18, 199,
  235, 152, 90, 215, 162, 186, 210, 105, 24, 89, 127, 161, 250, 94, 224, 119,
  226, 27, 71, 180, 35, 11, 162, 240, 227, 68, 58, 37, 150, 43, 44, 48,
  168, 38, 137, 16, 59, 68, 244, 234, 112, 25, 203, 121, 84, 82, 234, 80,
  69, 53, 109, 253, 120, 106, 171, 149, 169, 157, 66, 127, 96, 192, 40, 63,
  162, 31, 68, 37, 46, 184, 248, 245, 197, 9, 159, 37, 203, 205, 60, 113,
  87, 82, 83, 76, 71, 143, 210, 209, 65, 82, 251, 130, 233, 144, 197, 200,
  229, 65, 126, 252, 191, 237, 125, 219, 114, 220, 70, 146, 232, 187, 191, 2,
  242, 56, 220, 196, 49, 217, 34, 117, 177, 45, 82, 18, 67, 162, 168, 145,
  206, 80, 18, 87, 164, 229, 141, 145, 20, 36, 200, 6, 155, 88, 54, 27,
  88, 0, 205, 139, 20, 29, 49, 79, 251, 1, 59, 39, 246, 11, 246, 11,
  246, 97, 159, 252, 230, 63, 241, 151, 108, 102, 221, 47, 89, 0, 186, 73,
  202, 158, 141, 51, 19, 97, 177, 11, 85, 89, 89, 89, 89, 89, 89, 89,
  89, 153, 234, 2, 116, 69, 185, 130, 211, 93, 226, 2, 115, 251, 196, 178,
  46, 157, 98, 61, 221, 43, 254, 154, 169, 219, 231, 9, 30, 81, 168, 206,
  247, 142, 216, 167, 174, 56, 136, 234, 54, 42, 162, 80, 97, 116, 143, 198,
  72, 92, 14, 30, 39, 32, 202, 70, 34, 94, 150, 169, 117, 98, 113, 195,
  24, 118, 10, 104, 80, 178, 203, 92, 27, 6, 122, 19, 155, 5, 253, 138,
  253, 147, 14, 196, 141, 254, 251, 229, 143, 60, 46, 87, 99, 21, 75, 151,
  109, 36, 232, 33, 187, 148, 182, 40, 9, 69, 30, 5, 213, 7, 92, 163,
  146, 92, 240, 91, 145, 105, 185, 101, 226, 106, 126, 183, 109, 117, 196, 173,
  212, 237, 115, 133, 245, 244, 36, 225, 175, 206, 252, 162, 46, 202, 157, 174,
  173, 219, 193, 150, 238, 141, 27, 191, 117, 187, 117, 103, 52, 80, 148, 123,
  43, 52, 197, 151, 171, 29, 150, 40, 212, 51, 214, 40, 252, 106, 235, 86,
  89, 213, 223, 156, 88, 42, 3, 110, 19, 248, 28, 93, 231, 49, 144, 197,
  93, 240, 112, 77, 236, 106, 112, 103, 204, 219, 128, 119, 183, 174, 64, 42,
  36, 239, 182, 208, 6, 234, 242, 80, 221, 120, 216, 251, 107, 134, 231, 198,
  111, 62, 59, 248, 237, 141, 132, 173, 208, 112, 69, 54, 191, 169, 206, 244,
  11, 22, 124, 100, 242, 119, 2, 210, 177, 52, 14, 83, 160, 240, 35, 9,
  235, 100, 59, 217, 167, 244, 50, 166, 238, 108, 106, 239, 23, 28, 13, 215,
  129, 122, 193, 234, 98, 111, 101, 29, 138, 18, 59, 99, 41, 223, 80, 36,
  236, 152, 138, 165, 207, 168, 45, 82, 178, 176, 90, 206, 233, 181, 119, 98,
  132, 187, 89, 211, 181, 248, 225, 68, 62, 37, 238, 253, 233, 193, 97, 114,
  23, 88, 211, 205, 225, 162, 162, 67, 10, 252, 30, 226, 170, 15, 119, 54,
  193, 88, 155, 17, 206, 93, 83, 95, 71, 247, 31, 164, 203, 7, 173, 125,
  61, 110, 238, 235, 215, 95, 14, 186, 244, 245, 227, 15, 43, 63, 172, 208,
  125, 133, 97, 103, 167, 173, 128, 239, 222, 27, 220, 125, 240, 192, 1, 60,
  245, 117, 109, 110, 253, 134, 35, 231, 120, 40, 82, 20, 215, 239, 36, 163,
  155, 90, 91, 126, 190, 104, 179, 31, 139, 110, 150, 141, 88, 168, 31, 47,
  112, 101, 132, 233, 88, 71, 151, 207, 196, 155, 186, 234, 121, 153, 159, 170,
  163, 141, 127, 94, 225, 218, 249, 46, 64, 131, 35, 234, 64, 156, 217, 170,
  80, 189, 141, 252, 14, 59, 6, 102, 35, 16, 109, 161, 74, 59, 236, 217,
  3, 130, 172, 132, 174, 79, 187, 123, 129, 114, 243, 78, 62, 156, 35, 116,
  120, 45, 148, 80, 45, 71, 161, 164, 117, 16, 153, 22, 145, 80, 232, 131,
  205, 152, 234, 65, 181, 99, 234, 189, 110, 134, 186, 189, 209, 12, 119, 52,
  170, 21, 215, 245, 141, 102, 35, 12, 90, 144, 154, 77, 217, 230, 69, 181,
  213, 202, 127, 99, 123, 181, 3, 81, 48, 248, 105, 192, 24, 45, 30, 5,
  76, 42, 225, 222, 65, 53, 100, 103, 3, 95, 226, 107, 17, 39, 90, 117,
  62, 40, 224, 25, 129, 191, 105, 146, 209, 9, 22, 173, 233, 125, 127, 250,
  209, 210, 140, 165, 175, 102, 76, 176, 197, 24, 86, 219, 115, 161, 161, 221,
  168, 242, 70, 188, 77, 97, 79, 11, 116, 255, 244, 61, 173, 169, 214, 241,
  101, 139, 103, 142, 109, 165, 115, 90, 254, 42, 42, 5, 137, 1, 118, 201,
  105, 16, 163, 52, 235, 47, 47, 47, 175, 216, 7, 6, 23, 172, 65, 26,
  255, 100, 81, 164, 44, 38, 52, 214, 127, 155, 86, 135, 201, 40, 21, 145,
  241, 16, 71, 223, 117, 181, 76, 15, 38, 217, 104, 128, 213, 165, 87, 173,
  134, 238, 185, 207, 18, 192, 61, 247, 247, 176, 59, 171, 148, 10, 127, 41,
  178, 167, 73, 233, 203, 12, 210, 201, 181, 201, 205, 149, 116, 116, 37, 98,
  12, 163, 44, 5, 25, 228, 247, 72, 91, 197, 156, 235, 124, 41, 150, 81,
  177, 181, 204, 229, 210, 10, 169, 191, 40, 99, 248, 35, 195, 28, 238, 18,
  189, 41, 180, 137, 112, 203, 213, 46, 53, 220, 113, 151, 137, 249, 147, 172,
  192, 81, 40, 65, 31, 188, 93, 100, 82, 255, 231, 236, 40, 11, 24, 166,
  154, 125, 127, 187, 63, 57, 81, 22, 52, 249, 162, 195, 127, 117, 66, 121,
  70, 187, 205, 154, 125, 163, 3, 54, 35, 198, 116, 190, 177, 113, 218, 225,
  21, 235, 78, 90, 215, 192, 196, 213, 85, 13, 136, 149, 128, 211, 155, 209,
  46, 136, 155, 59, 74, 67, 125, 242, 19, 155, 186, 248, 137, 250, 93, 175,
  23, 104, 194, 53, 198, 97, 218, 91, 180, 212, 194, 97, 74, 183, 82, 74,
  51, 107, 36, 116, 205, 69, 183, 34, 99, 118, 225, 218, 175, 79, 150, 150,
  199, 127, 28, 217, 21, 84, 4, 98, 171, 150, 47, 197, 207, 5, 27, 50,
  69, 86, 104, 189, 186, 204, 71, 67, 127, 115, 151, 141, 254, 226, 40, 98,
  202, 84, 200, 73, 176, 22, 106, 197, 46, 28, 94, 195, 90, 67, 213, 76,
  168, 143, 61, 244, 101, 16, 155, 187, 52, 51, 2, 144, 156, 219, 221, 129,
  43, 195, 111, 183, 244, 156, 100, 133, 156, 141, 172, 224, 137, 14, 2, 147,
  55, 60, 151, 21, 135, 231, 141, 21, 171, 177, 172, 88, 141, 137, 138, 250,
  124, 144, 29, 238, 230, 195, 225, 200, 184, 85, 226, 165, 47, 11, 94, 238,
  19, 216, 108, 21, 91, 48, 128, 255, 210, 195, 19, 182, 145, 241, 174, 217,
  55, 236, 222, 147, 240, 66, 187, 19, 93, 25, 122, 160, 175, 125, 106, 57,
  244, 57, 178, 237, 185, 171, 145, 71, 248, 21, 160, 250, 242, 98, 36, 108,
  193, 162, 130, 180, 12, 171, 207, 89, 177, 42, 201, 189, 24, 13, 207, 87,
  37, 73, 65, 155, 24, 175, 42, 178, 57, 154, 176, 138, 15, 205, 117, 82,
  87, 19, 37, 223, 239, 81, 43, 60, 44, 28, 41, 89, 39, 33, 116, 16,
  117, 158, 232, 226, 179, 188, 179, 249, 122, 231, 205, 219, 189, 221, 205, 87,
  219, 91, 79, 118, 55, 119, 44, 215, 248, 207, 236, 104, 190, 218, 123, 250,
  98, 229, 135, 251, 203, 192, 51, 99, 96, 111, 227, 39, 160, 154, 14, 97,
  79, 95, 237, 141, 88, 42, 94, 160, 29, 232, 60, 80, 35, 187, 115, 136,
  87, 99, 131, 100, 245, 206, 10, 252, 123, 56, 90, 189, 115, 199, 116, 108,
  151, 128, 119, 119, 182, 238, 220, 127, 176, 162, 32, 235, 223, 87, 6, 189,
  243, 98, 247, 174, 6, 44, 127, 105, 176, 66, 11, 158, 29, 240, 211, 87,
  155, 119, 126, 52, 136, 33, 127, 94, 3, 232, 87, 91, 255, 252, 96, 249,
  251, 149, 123, 10, 184, 81, 96, 16, 132, 219, 111, 102, 133, 253, 226, 175,
  43, 15, 52, 224, 23, 75, 252, 167, 129, 53, 51, 190, 9, 168, 19, 140,
  217, 191, 24, 149, 23, 171, 43, 223, 131, 74, 13, 255, 252, 16, 2, 122,
  207, 6, 122, 239, 202, 64, 119, 54, 158, 221, 213, 244, 149, 191, 2, 48,
  59, 179, 195, 198, 179, 123, 22, 208, 123, 115, 3, 85, 198, 126, 190, 130,
  54, 222, 188, 221, 220, 19, 203, 232, 229, 179, 29, 166, 63, 159, 71, 160,
  5, 44, 188, 231, 182, 112, 205, 15, 98, 234, 88, 103, 31, 137, 96, 112,
  252, 25, 63, 223, 79, 224, 16, 179, 33, 176, 91, 144, 104, 130, 2, 196,
  227, 63, 248, 239, 163, 248, 49, 232, 233, 165, 108, 227, 156, 116, 217, 34,
  90, 229, 15, 130, 22, 45, 193, 196, 80, 91, 21, 239, 124, 172, 184, 165,
  236, 216, 39, 80, 254, 202, 57, 196, 178, 33, 144, 105, 53, 168, 180, 35,
  26, 173, 247, 114, 40, 31, 249, 190, 227, 184, 174, 171, 225, 169, 87, 149,
  34, 121, 140, 114, 181, 82, 49, 141, 122, 131, 52, 57, 169, 179, 179, 44,
  5, 166, 114, 51, 52, 221, 178, 19, 244, 53, 194, 25, 163, 43, 7, 104,
  150, 39, 160, 29, 217, 144, 140, 252, 82, 106, 40, 66, 63, 148, 55, 38,
  214, 117, 174, 200, 180, 228, 184, 0, 176, 246, 125, 56, 73, 84, 41, 79,
  219, 202, 11, 240, 36, 236, 99, 39, 44, 58, 26, 59, 54, 70, 111, 124,
  22, 76, 10, 142, 48, 111, 105, 56, 166, 9, 206, 132, 118, 5, 2, 17,
  177, 105, 112, 71, 214, 70, 25, 65, 43, 12, 93, 234, 57, 225, 243, 66,
  58, 157, 206, 32, 103, 222, 160, 80, 195, 115, 4, 227, 122, 213, 18, 212,
  112, 61, 45, 110, 65, 89, 56, 33, 163, 90, 20, 12, 42, 238, 225, 176,
  117, 246, 85, 185, 195, 139, 226, 106, 93, 196, 90, 113, 26, 169, 98, 60,
  132, 225, 89, 169, 23, 226, 152, 238, 203, 217, 240, 122, 202, 107, 97, 230,
  211, 62, 92, 50, 147, 87, 223, 74, 254, 29, 122, 68, 73, 24, 197, 76,
  143, 172, 252, 112, 130, 161, 187, 109, 202, 62, 25, 129, 106, 247, 39, 238,
  32, 35, 20, 152, 168, 207, 209, 95, 194, 193, 247, 116, 152, 84, 252, 137,
  71, 201, 134, 185, 110, 240, 255, 224, 39, 98, 169, 35, 241, 14, 249, 107,
  134, 17, 90, 3, 125, 185, 198, 171, 88, 193, 179, 44, 52, 61, 54, 80,
  13, 124, 102, 80, 159, 44, 183, 64, 127, 226, 171, 52, 29, 27, 98, 60,
  118, 33, 24, 25, 105, 17, 7, 218, 159, 117, 144, 14, 38, 69, 250, 23,
  25, 118, 136, 87, 237, 227, 102, 132, 220, 38, 126, 226, 110, 132, 207, 131,
  197, 79, 201, 24, 118, 16, 42, 182, 224, 1, 165, 254, 113, 130, 113, 91,
  5, 88, 50, 49, 45, 171, 134, 142, 86, 186, 26, 225, 122, 193, 102, 80,
  115, 194, 97, 153, 66, 191, 34, 7, 216, 66, 111, 0, 242, 198, 106, 197,
  184, 223, 62, 221, 24, 156, 225, 213, 244, 22, 151, 92, 8, 193, 229, 22,
  5, 22, 152, 108, 39, 11, 214, 197, 122, 195, 179, 19, 59, 51, 16, 145,
  199, 184, 236, 8, 141, 13, 29, 31, 237, 193, 225, 122, 35, 78, 110, 76,
  194, 52, 221, 153, 116, 37, 94, 69, 29, 14, 57, 249, 248, 167, 30, 81,
  27, 95, 136, 143, 7, 27, 199, 217, 8, 102, 50, 175, 73, 159, 20, 158,
  169, 175, 251, 64, 89, 11, 55, 106, 154, 226, 60, 100, 68, 131, 47, 145,
  77, 167, 209, 130, 207, 152, 241, 126, 11, 186, 172, 27, 159, 125, 204, 42,
  188, 21, 197, 152, 201, 112, 38, 218, 98, 125, 146, 178, 60, 211, 90, 69,
  76, 224, 65, 61, 110, 232, 225, 96, 82, 215, 185, 67, 55, 104, 33, 4,
  242, 185, 240, 196, 238, 37, 147, 58, 239, 185, 117, 108, 202, 250, 156, 251,
  76, 169, 43, 184, 3, 99, 220, 18, 253, 211, 133, 229, 199, 70, 6, 61,
  237, 240, 4, 246, 100, 110, 111, 146, 25, 230, 108, 11, 70, 122, 214, 199,
  247, 181, 219, 101, 94, 36, 67, 102, 250, 91, 32, 61, 88, 56, 117, 72,
  36, 7, 89, 133, 127, 115, 235, 4, 43, 118, 236, 29, 252, 156, 11, 4,
  56, 78, 7, 207, 209, 96, 165, 140, 85, 76, 26, 223, 134, 30, 142, 178,
  97, 15, 13, 138, 160, 18, 29, 231, 131, 213, 222, 246, 155, 157, 93, 40,
  56, 102, 151, 62, 213, 234, 231, 158, 160, 210, 210, 46, 48, 89, 111, 181,
  135, 27, 73, 118, 200, 240, 189, 125, 177, 116, 126, 126, 190, 132, 185, 25,
  151, 38, 229, 40, 29, 31, 194, 193, 124, 208, 155, 46, 70, 7, 249, 224,
  114, 117, 159, 99, 254, 232, 155, 207, 252, 143, 233, 183, 217, 224, 145, 226,
  209, 108, 48, 221, 119, 131, 64, 180, 28, 203, 167, 62, 59, 153, 140, 10,
  83, 225, 217, 55, 110, 57, 234, 62, 19, 202, 10, 131, 56, 16, 80, 44,
  29, 61, 157, 153, 241, 34, 209, 172, 149, 251, 84, 69, 107, 41, 12, 143,
  243, 170, 166, 235, 57, 23, 141, 91, 191, 254, 119, 133, 185, 226, 232, 202,
  243, 241, 98, 23, 110, 148, 207, 49, 145, 101, 202, 211, 133, 125, 62, 69,
  81, 179, 76, 58, 207, 202, 19, 232, 255, 56, 26, 9, 172, 215, 247, 3,
  41, 218, 237, 24, 190, 127, 8, 198, 229, 1, 84, 91, 185, 86, 16, 6,
  227, 188, 186, 1, 251, 237, 119, 149, 44, 147, 41, 31, 25, 86, 198, 130,
  5, 34, 12, 138, 97, 57, 18, 52, 150, 196, 139, 142, 210, 227, 209, 48,
  133, 191, 71, 201, 48, 29, 51, 103, 7, 132, 10, 84, 71, 128, 92, 82,
  19, 145, 123, 162, 0, 197, 221, 112, 42, 45, 235, 207, 143, 96, 227, 173,
  65, 206, 133, 225, 151, 78, 166, 79, 94, 231, 141, 3, 235, 219, 171, 133,
  71, 228, 199, 242, 158, 87, 241, 186, 36, 187, 55, 54, 207, 245, 207, 219,
  42, 177, 137, 109, 103, 84, 42, 172, 85, 45, 115, 188, 0, 27, 148, 115,
  50, 48, 125, 227, 117, 134, 152, 184, 171, 222, 102, 88, 235, 140, 216, 253,
  249, 208, 164, 117, 88, 252, 114, 158, 148, 153, 79, 138, 217, 129, 65, 221,
  16, 168, 2, 104, 225, 218, 52, 253, 43, 185, 240, 1, 164, 217, 189, 193,
  110, 215, 229, 118, 49, 124, 5, 24, 178, 9, 135, 232, 52, 171, 101, 216,
  132, 51, 143, 125, 24, 223, 111, 158, 103, 159, 146, 114, 176, 195, 223, 137,
  171, 107, 99, 253, 101, 87, 208, 205, 241, 185, 215, 21, 158, 12, 206, 18,
  233, 45, 206, 173, 252, 238, 137, 112, 152, 255, 172, 58, 89, 112, 31, 117,
  91, 253, 227, 199, 181, 78, 7, 89, 62, 120, 14, 55, 234, 115, 32, 75,
  216, 220, 56, 199, 162, 250, 252, 24, 99, 242, 132, 31, 138, 16, 111, 97,
  240, 198, 214, 148, 52, 182, 125, 98, 63, 220, 243, 123, 100, 83, 246, 215,
  163, 15, 95, 99, 20, 149, 180, 152, 126, 248, 250, 227, 190, 247, 152, 5,
  123, 136, 69, 79, 237, 175, 103, 228, 45, 216, 167, 215, 124, 51, 80, 87,
  96, 172, 128, 172, 185, 147, 152, 175, 132, 68, 129, 123, 140, 22, 0, 98,
  9, 218, 127, 72, 130, 115, 242, 48, 186, 139, 2, 48, 27, 227, 67, 197,
  165, 224, 131, 18, 1, 16, 251, 137, 37, 6, 52, 64, 180, 169, 116, 0,
  233, 219, 21, 240, 113, 162, 224, 35, 49, 25, 84, 16, 41, 88, 64, 174,
  119, 44, 159, 48, 105, 154, 225, 95, 137, 71, 80, 176, 227, 211, 45, 119,
  213, 23, 207, 18, 161, 123, 195, 72, 19, 26, 4, 97, 165, 146, 53, 131,
  134, 9, 221, 186, 197, 118, 33, 78, 106, 60, 57, 231, 147, 178, 76, 46,
  89, 248, 215, 5, 105, 204, 112, 101, 35, 139, 159, 83, 227, 82, 208, 39,
  118, 59, 49, 131, 2, 168, 150, 14, 20, 209, 54, 143, 188, 104, 218, 134,
  115, 230, 10, 96, 139, 126, 40, 211, 201, 102, 147, 218, 253, 100, 111, 189,
  78, 5, 77, 53, 115, 47, 132, 102, 129, 135, 168, 28, 71, 225, 134, 6,
  244, 20, 174, 9, 60, 27, 122, 237, 41, 177, 220, 76, 217, 48, 107, 157,
  102, 37, 242, 118, 35, 25, 139, 219, 33, 58, 79, 95, 10, 120, 104, 75,
  91, 49, 10, 61, 138, 152, 157, 208, 54, 169, 1, 52, 211, 167, 253, 26,
  158, 129, 0, 171, 50, 51, 213, 190, 163, 169, 233, 177, 135, 201, 111, 79,
  192, 212, 141, 45, 103, 204, 194, 130, 230, 51, 51, 206, 162, 191, 210, 139,
  188, 152, 224, 150, 195, 87, 251, 118, 6, 77, 1, 69, 231, 25, 153, 184,
  231, 70, 89, 51, 72, 64, 135, 199, 65, 84, 131, 196, 115, 98, 176, 171,
  30, 142, 100, 213, 195, 81, 115, 213, 183, 23, 162, 102, 121, 209, 92, 113,
  87, 86, 172, 221, 138, 254, 200, 64, 219, 46, 107, 62, 44, 119, 72, 222,
  110, 139, 188, 65, 104, 58, 86, 190, 231, 208, 14, 172, 162, 52, 80, 50,
  147, 26, 7, 41, 35, 23, 29, 164, 180, 244, 184, 14, 225, 105, 174, 187,
  207, 55, 185, 226, 66, 131, 80, 171, 48, 3, 221, 240, 166, 150, 33, 194,
  238, 186, 14, 89, 221, 107, 90, 136, 86, 35, 137, 140, 67, 9, 27, 173,
  169, 17, 213, 193, 91, 127, 118, 203, 152, 120, 73, 61, 72, 143, 146, 201,
  168, 198, 218, 47, 199, 71, 57, 186, 8, 193, 153, 100, 60, 128, 102, 5,
  148, 69, 231, 105, 57, 128, 211, 40, 28, 65, 39, 245, 167, 186, 111, 210,
  236, 189, 90, 197, 106, 141, 202, 21, 40, 22, 216, 71, 61, 87, 131, 89,
  35, 66, 26, 177, 25, 133, 237, 203, 143, 20, 58, 141, 233, 119, 192, 137,
  88, 89, 56, 170, 159, 147, 18, 125, 106, 92, 127, 49, 75, 185, 53, 223,
  22, 114, 146, 189, 129, 233, 74, 7, 193, 87, 160, 129, 247, 176, 166, 142,
  233, 61, 197, 117, 220, 59, 168, 165, 219, 85, 249, 225, 247, 14, 126, 13,
  91, 214, 250, 223, 41, 147, 17, 243, 113, 36, 82, 187, 25, 106, 139, 58,
  91, 59, 208, 24, 127, 18, 71, 192, 25, 165, 202, 255, 223, 208, 175, 127,
  67, 87, 115, 129, 3, 163, 232, 50, 30, 4, 169, 210, 186, 125, 209, 187,
  94, 139, 16, 163, 232, 16, 208, 24, 26, 13, 33, 230, 234, 217, 157, 137,
  211, 244, 170, 217, 37, 247, 48, 247, 219, 60, 171, 133, 221, 34, 62, 242,
  65, 133, 150, 74, 251, 244, 112, 136, 143, 248, 3, 90, 54, 55, 54, 241,
  103, 153, 150, 86, 197, 1, 39, 137, 208, 22, 174, 50, 83, 64, 68, 110,
  155, 225, 118, 118, 121, 83, 108, 148, 218, 51, 100, 126, 137, 173, 214, 13,
  6, 111, 62, 29, 166, 154, 70, 48, 176, 62, 59, 153, 60, 196, 235, 111,
  96, 244, 3, 151, 137, 140, 79, 20, 23, 25, 159, 99, 31, 88, 59, 182,
  221, 246, 16, 190, 113, 53, 160, 78, 99, 29, 70, 216, 103, 121, 94, 222,
  132, 112, 74, 158, 193, 64, 21, 225, 198, 144, 76, 248, 92, 88, 8, 176,
  55, 220, 122, 243, 140, 150, 156, 173, 21, 31, 61, 172, 220, 95, 142, 103,
  35, 4, 205, 113, 142, 33, 229, 105, 114, 120, 98, 25, 82, 176, 192, 166,
  136, 40, 140, 101, 245, 214, 233, 178, 20, 6, 21, 54, 115, 101, 209, 180,
  132, 45, 69, 43, 177, 59, 85, 237, 22, 32, 210, 170, 211, 134, 206, 103,
  215, 134, 163, 172, 113, 44, 86, 66, 119, 209, 20, 173, 7, 164, 21, 218,
  118, 122, 191, 187, 200, 106, 23, 60, 180, 74, 167, 98, 153, 222, 181, 102,
  232, 59, 156, 33, 63, 65, 175, 20, 85, 90, 109, 116, 164, 149, 245, 193,
  21, 88, 214, 199, 216, 5, 51, 227, 84, 170, 91, 183, 222, 207, 242, 66,
  13, 0, 29, 192, 196, 12, 39, 227, 97, 244, 235, 127, 162, 213, 122, 188,
  222, 35, 238, 214, 188, 99, 165, 173, 45, 183, 104, 200, 252, 185, 137, 8,
  62, 243, 59, 171, 249, 206, 121, 120, 26, 156, 178, 118, 171, 41, 105, 226,
  108, 189, 58, 117, 38, 70, 184, 40, 228, 131, 75, 225, 125, 244, 211, 219,
  173, 157, 52, 41, 15, 143, 183, 147, 50, 57, 181, 78, 231, 88, 171, 15,
  180, 230, 70, 225, 28, 73, 139, 219, 29, 109, 186, 52, 223, 4, 55, 233,
  200, 74, 23, 247, 27, 116, 177, 144, 190, 83, 105, 155, 54, 89, 254, 11,
  246, 7, 185, 204, 85, 71, 188, 9, 239, 113, 157, 255, 75, 54, 208, 195,
  229, 232, 236, 33, 112, 100, 39, 222, 45, 73, 24, 169, 98, 112, 63, 95,
  187, 22, 71, 128, 133, 12, 217, 231, 207, 178, 223, 177, 128, 251, 91, 249,
  185, 140, 13, 54, 221, 251, 230, 179, 222, 88, 76, 109, 89, 119, 145, 225,
  163, 164, 204, 79, 112, 252, 7, 184, 115, 230, 88, 214, 249, 78, 93, 102,
  227, 33, 38, 167, 119, 237, 211, 212, 253, 114, 231, 155, 101, 117, 180, 71,
  102, 151, 239, 126, 228, 221, 113, 79, 92, 51, 159, 228, 99, 192, 62, 226,
  14, 164, 199, 217, 248, 211, 100, 152, 30, 253, 250, 203, 176, 22, 7, 254,
  62, 117, 15, 168, 0, 218, 203, 45, 248, 160, 231, 147, 124, 207, 163, 254,
  36, 159, 226, 224, 99, 27, 251, 98, 206, 22, 107, 83, 98, 171, 147, 66,
  142, 162, 17, 179, 91, 180, 46, 211, 136, 213, 179, 23, 42, 176, 112, 189,
  135, 197, 189, 64, 69, 159, 171, 204, 175, 21, 179, 104, 46, 24, 164, 129,
  130, 120, 157, 47, 27, 182, 15, 246, 150, 86, 66, 160, 43, 102, 227, 52,
  27, 67, 65, 215, 198, 229, 133, 221, 22, 132, 115, 215, 166, 181, 211, 116,
  183, 173, 233, 239, 187, 118, 24, 226, 129, 181, 51, 253, 170, 113, 13, 244,
  254, 156, 86, 69, 10, 28, 159, 150, 117, 63, 122, 154, 213, 176, 0, 94,
  167, 19, 118, 106, 136, 6, 19, 224, 18, 88, 1, 199, 165, 195, 252, 179,
  177, 254, 76, 140, 159, 155, 97, 227, 44, 237, 229, 174, 233, 217, 26, 246,
  212, 112, 119, 67, 16, 8, 56, 152, 215, 249, 185, 165, 190, 232, 98, 123,
  91, 180, 170, 199, 118, 235, 153, 183, 200, 16, 91, 8, 168, 62, 63, 56,
  187, 58, 237, 231, 80, 29, 38, 227, 215, 105, 125, 158, 151, 39, 149, 27,
  145, 78, 206, 176, 122, 83, 136, 182, 203, 9, 76, 47, 204, 106, 253, 9,
  228, 216, 73, 218, 239, 91, 115, 57, 135, 119, 196, 88, 116, 78, 190, 195,
  171, 178, 129, 103, 126, 128, 50, 226, 237, 157, 170, 233, 58, 13, 233, 47,
  13, 118, 173, 72, 60, 85, 148, 200, 200, 28, 92, 82, 237, 26, 135, 51,
  13, 206, 99, 190, 178, 13, 88, 227, 62, 226, 72, 213, 240, 12, 88, 188,
  42, 115, 79, 29, 247, 75, 248, 123, 58, 120, 122, 26, 239, 187, 109, 141,
  49, 55, 217, 179, 124, 31, 36, 36, 37, 77, 9, 17, 54, 23, 15, 29,
  203, 241, 181, 147, 194, 113, 203, 251, 11, 123, 177, 161, 152, 12, 166, 30,
  248, 117, 224, 185, 233, 205, 50, 208, 240, 83, 211, 48, 151, 39, 7, 220,
  63, 44, 175, 64, 244, 246, 58, 120, 176, 80, 188, 60, 155, 239, 138, 34,
  251, 28, 94, 43, 198, 109, 90, 94, 188, 77, 197, 235, 207, 231, 35, 208,
  225, 68, 123, 198, 247, 38, 18, 165, 172, 245, 68, 122, 32, 57, 7, 3,
  46, 197, 68, 165, 221, 236, 52, 133, 49, 176, 177, 227, 223, 249, 164, 118,
  63, 174, 249, 160, 217, 7, 47, 50, 165, 250, 252, 12, 246, 44, 116, 120,
  96, 105, 14, 172, 126, 5, 210, 177, 61, 73, 106, 96, 44, 125, 56, 6,
  193, 224, 181, 232, 99, 152, 213, 98, 59, 225, 79, 194, 109, 67, 71, 224,
  54, 210, 38, 160, 251, 136, 231, 102, 105, 162, 102, 195, 62, 95, 82, 52,
  51, 236, 63, 223, 69, 223, 47, 47, 155, 169, 46, 90, 9, 17, 58, 145,
  54, 18, 92, 44, 142, 97, 153, 159, 11, 135, 64, 104, 14, 103, 25, 103,
  43, 144, 177, 210, 71, 44, 103, 24, 185, 163, 41, 101, 220, 26, 54, 117,
  57, 97, 145, 92, 141, 254, 219, 111, 205, 225, 63, 246, 233, 227, 237, 5,
  141, 227, 226, 82, 231, 201, 24, 87, 160, 210, 95, 222, 165, 229, 65, 54,
  30, 160, 101, 160, 40, 127, 253, 229, 40, 29, 71, 160, 47, 149, 232, 80,
  50, 41, 150, 158, 108, 71, 120, 109, 231, 169, 243, 29, 86, 22, 237, 73,
  106, 202, 38, 63, 39, 165, 127, 194, 58, 226, 138, 192, 113, 93, 23, 171,
  183, 111, 187, 115, 194, 68, 10, 40, 116, 182, 178, 248, 231, 77, 212, 21,
  217, 51, 240, 222, 33, 190, 35, 34, 183, 129, 144, 11, 174, 200, 150, 130,
  240, 133, 47, 125, 176, 247, 30, 157, 24, 183, 65, 50, 55, 228, 2, 188,
  125, 59, 202, 134, 227, 188, 4, 134, 45, 241, 197, 97, 128, 104, 222, 18,
  195, 57, 23, 235, 18, 153, 113, 17, 77, 162, 203, 180, 103, 6, 126, 95,
  8, 153, 217, 21, 96, 52, 175, 58, 218, 159, 241, 197, 85, 0, 237, 70,
  177, 7, 102, 6, 227, 213, 60, 148, 247, 21, 88, 209, 253, 91, 164, 33,
  61, 140, 183, 118, 232, 94, 107, 28, 178, 89, 236, 67, 154, 97, 36, 148,
  128, 37, 84, 85, 247, 117, 31, 17, 20, 193, 51, 33, 213, 157, 131, 54,
  200, 16, 67, 50, 50, 131, 104, 185, 46, 254, 80, 49, 27, 86, 221, 149,
  43, 14, 44, 37, 106, 28, 165, 223, 211, 91, 243, 20, 32, 14, 44, 188,
  110, 44, 27, 25, 206, 141, 188, 51, 76, 50, 60, 64, 5, 103, 49, 186,
  37, 250, 245, 249, 208, 143, 37, 98, 57, 38, 219, 65, 68, 26, 174, 214,
  66, 194, 88, 16, 240, 19, 123, 245, 233, 71, 36, 89, 155, 81, 52, 81,
  7, 88, 17, 244, 225, 134, 158, 26, 72, 172, 31, 129, 210, 196, 190, 253,
  244, 246, 229, 70, 126, 90, 64, 17, 40, 160, 245, 167, 120, 186, 79, 138,
  186, 224, 115, 131, 70, 147, 80, 159, 201, 42, 17, 93, 168, 231, 235, 181,
  245, 49, 172, 74, 102, 36, 17, 74, 158, 56, 148, 126, 243, 217, 120, 95,
  32, 94, 29, 52, 102, 45, 21, 201, 92, 121, 160, 111, 57, 70, 156, 164,
  79, 118, 155, 198, 36, 179, 245, 39, 39, 179, 172, 1, 211, 79, 45, 11,
  44, 253, 164, 40, 202, 252, 98, 147, 7, 165, 118, 236, 0, 205, 146, 250,
  138, 209, 58, 172, 4, 12, 193, 235, 233, 228, 44, 221, 224, 65, 125, 44,
  33, 102, 148, 59, 23, 116, 86, 131, 216, 1, 48, 167, 189, 90, 134, 21,
  242, 3, 206, 10, 3, 143, 251, 220, 147, 62, 31, 207, 190, 106, 34, 119,
  217, 68, 87, 49, 249, 236, 11, 180, 97, 221, 136, 191, 188, 133, 210, 176,
  76, 174, 176, 72, 174, 176, 68, 204, 5, 66, 198, 78, 115, 83, 30, 27,
  250, 18, 143, 113, 191, 214, 225, 24, 88, 164, 39, 24, 29, 24, 166, 87,
  25, 182, 190, 108, 196, 26, 189, 25, 138, 27, 52, 246, 186, 5, 35, 78,
  217, 247, 103, 170, 216, 185, 58, 51, 171, 199, 118, 235, 57, 121, 94, 70,
  188, 114, 67, 173, 98, 215, 1, 174, 159, 115, 177, 56, 239, 142, 157, 222,
  104, 211, 96, 69, 4, 184, 170, 168, 208, 86, 56, 183, 108, 66, 199, 120,
  110, 105, 125, 182, 236, 190, 87, 158, 254, 227, 45, 233, 111, 85, 64, 179,
  71, 44, 49, 14, 163, 162, 183, 208, 231, 94, 205, 225, 51, 131, 53, 47,
  129, 217, 120, 183, 253, 44, 18, 65, 89, 163, 228, 164, 158, 36, 35, 80,
  242, 112, 181, 173, 89, 243, 161, 223, 131, 228, 120, 96, 117, 182, 201, 198,
  37, 30, 133, 34, 0, 58, 251, 26, 15, 77, 59, 243, 0, 20, 59, 57,
  239, 10, 131, 3, 224, 17, 208, 156, 17, 116, 20, 27, 51, 72, 199, 27,
  19, 81, 95, 138, 44, 65, 81, 136, 59, 56, 134, 64, 243, 246, 127, 44,
  244, 55, 127, 81, 53, 54, 219, 205, 41, 2, 187, 71, 222, 211, 102, 108,
  215, 128, 221, 40, 243, 10, 32, 137, 110, 128, 191, 58, 8, 73, 60, 123,
  132, 207, 61, 235, 234, 60, 227, 71, 157, 19, 55, 194, 70, 235, 172, 104,
  233, 112, 120, 174, 43, 15, 207, 219, 176, 51, 39, 104, 28, 172, 236, 5,
  41, 12, 7, 38, 236, 9, 179, 12, 187, 127, 88, 107, 9, 69, 104, 5,
  137, 110, 58, 122, 206, 39, 204, 177, 243, 27, 20, 228, 200, 46, 244, 129,
  6, 191, 196, 211, 111, 145, 61, 232, 10, 248, 5, 42, 112, 86, 192, 119,
  215, 146, 75, 68, 188, 191, 233, 183, 89, 65, 183, 204, 10, 104, 55, 60,
  167, 63, 14, 207, 17, 232, 56, 128, 212, 56, 254, 82, 123, 202, 172, 188,
  18, 157, 102, 24, 251, 120, 146, 158, 170, 171, 133, 153, 217, 167, 97, 135,
  232, 138, 206, 115, 88, 124, 176, 22, 171, 104, 27, 186, 66, 83, 99, 43,
  6, 252, 2, 223, 217, 42, 124, 35, 127, 79, 2, 148, 150, 74, 87, 221,
  166, 36, 62, 139, 238, 249, 5, 118, 22, 167, 159, 230, 93, 165, 43, 41,
  27, 118, 151, 153, 41, 74, 208, 211, 176, 253, 58, 240, 187, 168, 234, 24,
  161, 172, 246, 54, 40, 85, 234, 221, 20, 171, 202, 177, 213, 244, 198, 247,
  40, 151, 224, 97, 114, 255, 12, 43, 166, 74, 51, 0, 159, 142, 70, 64,
  149, 212, 209, 157, 133, 158, 207, 221, 235, 164, 141, 178, 40, 65, 54, 192,
  201, 202, 111, 108, 185, 4, 172, 71, 187, 89, 81, 164, 209, 219, 205, 157,
  205, 221, 232, 19, 156, 185, 158, 166, 85, 253, 235, 127, 214, 25, 118, 227,
  6, 214, 101, 247, 148, 86, 71, 60, 148, 107, 191, 46, 179, 83, 144, 41,
  86, 166, 67, 158, 224, 136, 1, 198, 203, 175, 25, 6, 252, 228, 96, 152,
  30, 148, 57, 143, 16, 34, 173, 215, 94, 62, 216, 89, 189, 214, 4, 222,
  56, 38, 129, 212, 90, 235, 37, 63, 143, 167, 126, 185, 196, 56, 163, 113,
  183, 185, 41, 191, 41, 195, 142, 206, 57, 78, 218, 161, 251, 60, 80, 56,
  102, 148, 180, 76, 235, 190, 194, 54, 83, 84, 221, 150, 136, 186, 13, 246,
  205, 144, 181, 56, 232, 3, 129, 232, 111, 229, 87, 15, 91, 61, 202, 221,
  144, 213, 58, 241, 76, 62, 124, 154, 51, 71, 160, 129, 74, 23, 28, 247,
  255, 37, 207, 160, 237, 135, 113, 47, 238, 112, 185, 236, 66, 159, 77, 254,
  98, 235, 121, 46, 149, 29, 82, 13, 242, 243, 241, 205, 145, 75, 172, 167,
  81, 126, 32, 214, 211, 83, 248, 115, 225, 125, 128, 102, 31, 23, 101, 136,
  83, 12, 225, 121, 81, 223, 46, 70, 73, 54, 238, 145, 202, 7, 112, 59,
  128, 132, 229, 41, 188, 18, 222, 28, 252, 11, 136, 121, 248, 189, 128, 189,
  81, 33, 57, 26, 60, 25, 18, 199, 95, 171, 15, 2, 236, 8, 234, 67,
  39, 118, 185, 164, 22, 11, 74, 164, 238, 104, 150, 96, 224, 253, 250, 194,
  14, 183, 194, 114, 83, 28, 158, 216, 103, 86, 196, 183, 76, 207, 242, 19,
  3, 95, 232, 228, 139, 51, 129, 88, 191, 135, 201, 216, 63, 112, 137, 66,
  103, 225, 234, 170, 177, 217, 174, 97, 51, 51, 29, 145, 188, 75, 43, 118,
  112, 71, 158, 115, 246, 82, 85, 238, 238, 166, 102, 131, 216, 1, 208, 128,
  132, 100, 109, 7, 1, 147, 235, 45, 12, 204, 15, 54, 10, 78, 147, 216,
  133, 209, 128, 132, 89, 83, 193, 36, 68, 22, 236, 49, 53, 52, 188, 242,
  58, 44, 4, 28, 106, 45, 142, 0, 59, 243, 36, 202, 106, 34, 202, 68,
  50, 106, 40, 245, 82, 81, 227, 185, 190, 221, 237, 74, 98, 224, 186, 93,
  21, 97, 183, 171, 67, 254, 48, 167, 115, 220, 35, 30, 53, 161, 28, 216,
  218, 32, 150, 244, 200, 106, 60, 70, 199, 105, 82, 14, 179, 241, 110, 142,
  71, 227, 222, 143, 197, 5, 93, 215, 202, 180, 253, 16, 83, 130, 143, 135,
  143, 191, 249, 92, 240, 183, 134, 15, 111, 139, 146, 135, 7, 229, 237, 199,
  60, 131, 53, 207, 85, 253, 225, 107, 30, 121, 86, 100, 127, 90, 251, 240,
  53, 107, 53, 0, 61, 179, 204, 152, 183, 212, 84, 36, 172, 222, 199, 96,
  250, 69, 31, 165, 216, 58, 116, 193, 0, 37, 17, 138, 29, 22, 82, 133,
  125, 153, 126, 248, 90, 196, 78, 129, 178, 189, 131, 81, 50, 62, 1, 128,
  91, 217, 248, 228, 225, 237, 4, 32, 172, 82, 151, 84, 108, 130, 76, 215,
  41, 28, 80, 163, 147, 216, 180, 117, 163, 162, 248, 105, 54, 57, 165, 248,
  97, 126, 89, 149, 156, 165, 98, 125, 120, 246, 33, 81, 238, 155, 136, 116,
  131, 216, 1, 48, 179, 18, 206, 195, 242, 166, 103, 152, 224, 27, 117, 206,
  235, 210, 29, 153, 127, 180, 189, 26, 95, 186, 214, 36, 178, 33, 178, 162,
  215, 20, 151, 65, 151, 198, 6, 71, 122, 48, 158, 193, 183, 46, 48, 128,
  67, 189, 182, 63, 149, 163, 46, 77, 97, 219, 202, 189, 182, 91, 88, 216,
  161, 177, 136, 86, 230, 181, 223, 20, 229, 134, 101, 108, 61, 234, 173, 176,
  136, 59, 203, 93, 84, 114, 197, 230, 191, 195, 35, 6, 237, 178, 172, 55,
  1, 66, 239, 110, 8, 20, 213, 231, 97, 188, 221, 16, 199, 92, 172, 62,
  14, 196, 88, 103, 239, 85, 64, 224, 201, 112, 181, 94, 58, 118, 250, 149,
  227, 105, 62, 193, 232, 188, 117, 90, 146, 15, 123, 85, 80, 236, 151, 213,
  230, 5, 72, 59, 160, 130, 8, 50, 78, 134, 55, 196, 104, 99, 44, 43,
  145, 172, 20, 122, 198, 212, 129, 8, 216, 235, 82, 122, 105, 82, 128, 133,
  73, 109, 32, 0, 6, 41, 13, 141, 191, 57, 128, 41, 241, 200, 51, 109,
  14, 20, 201, 226, 156, 35, 176, 103, 60, 136, 132, 159, 72, 201, 137, 246,
  238, 158, 61, 96, 120, 14, 73, 23, 163, 16, 169, 187, 146, 113, 134, 1,
  50, 12, 213, 43, 214, 8, 15, 255, 248, 168, 36, 63, 138, 196, 70, 29,
  135, 194, 150, 139, 64, 205, 234, 1, 44, 75, 158, 94, 213, 46, 223, 58,
  145, 83, 156, 104, 235, 222, 220, 89, 177, 150, 221, 201, 107, 37, 168, 245,
  34, 87, 35, 164, 121, 40, 212, 40, 200, 220, 190, 119, 34, 53, 97, 222,
  37, 85, 155, 227, 96, 14, 219, 185, 153, 104, 222, 95, 39, 161, 35, 186,
  206, 55, 191, 105, 95, 212, 170, 44, 244, 230, 158, 105, 214, 142, 173, 182,
  179, 50, 137, 247, 208, 217, 236, 50, 98, 180, 54, 7, 20, 124, 159, 193,
  154, 57, 79, 205, 117, 25, 129, 189, 120, 92, 174, 255, 110, 192, 220, 69,
  195, 233, 187, 78, 14, 182, 48, 64, 165, 169, 110, 136, 50, 187, 99, 93,
  49, 54, 26, 205, 224, 62, 55, 144, 56, 48, 29, 118, 132, 29, 232, 185,
  101, 225, 60, 185, 95, 15, 175, 231, 185, 25, 31, 103, 131, 116, 55, 207,
  71, 117, 86, 44, 120, 49, 248, 72, 212, 58, 198, 226, 195, 22, 73, 85,
  127, 127, 108, 19, 128, 21, 5, 107, 223, 185, 231, 87, 135, 178, 30, 129,
  25, 3, 20, 235, 110, 26, 226, 23, 146, 141, 1, 108, 108, 116, 219, 169,
  57, 29, 196, 114, 74, 204, 61, 67, 201, 155, 125, 103, 240, 214, 80, 20,
  153, 229, 175, 185, 121, 0, 186, 184, 6, 14, 176, 80, 153, 97, 206, 225,
  131, 41, 46, 60, 166, 159, 127, 198, 25, 228, 88, 119, 242, 135, 155, 113,
  0, 74, 77, 185, 53, 28, 27, 31, 115, 210, 249, 207, 185, 103, 29, 123,
  185, 190, 105, 23, 200, 220, 220, 188, 183, 202, 133, 235, 154, 245, 217, 133,
  196, 76, 115, 126, 119, 121, 64, 205, 57, 20, 147, 115, 206, 171, 199, 118,
  235, 185, 231, 28, 123, 185, 190, 57, 23, 200, 180, 206, 249, 123, 53, 185,
  139, 198, 212, 45, 154, 236, 174, 126, 60, 0, 12, 213, 143, 149, 31, 225,
  151, 23, 190, 128, 14, 90, 32, 66, 21, 224, 179, 14, 248, 47, 94, 218,
  56, 20, 108, 140, 88, 187, 102, 29, 144, 102, 154, 208, 7, 244, 132, 62,
  8, 76, 232, 3, 123, 66, 31, 92, 109, 66, 31, 92, 235, 132, 62, 184,
  129, 9, 189, 123, 237, 19, 250, 224, 134, 39, 20, 145, 164, 102, 148, 33,
  79, 77, 169, 104, 16, 59, 0, 230, 158, 84, 214, 209, 245, 205, 170, 68,
  231, 230, 166, 245, 193, 53, 204, 170, 34, 226, 53, 78, 171, 145, 31, 174,
  22, 41, 128, 23, 206, 224, 63, 139, 248, 108, 148, 229, 52, 159, 198, 222,
  44, 168, 92, 193, 209, 153, 74, 25, 140, 190, 239, 248, 36, 172, 199, 82,
  233, 168, 31, 171, 58, 153, 48, 177, 107, 9, 63, 65, 139, 143, 94, 200,
  220, 195, 68, 253, 93, 1, 215, 106, 32, 11, 137, 205, 74, 192, 138, 141,
  190, 136, 167, 33, 130, 114, 139, 141, 153, 144, 61, 216, 178, 219, 216, 68,
  108, 70, 232, 117, 0, 245, 91, 72, 252, 126, 5, 135, 85, 158, 130, 141,
  156, 195, 175, 190, 162, 169, 233, 46, 76, 143, 160, 54, 113, 228, 178, 212,
  63, 187, 4, 254, 146, 188, 162, 9, 228, 75, 10, 73, 20, 23, 33, 127,
  194, 28, 146, 74, 148, 140, 223, 51, 225, 164, 201, 74, 37, 66, 60, 207,
  234, 195, 99, 86, 81, 70, 68, 239, 148, 194, 172, 143, 188, 110, 216, 177,
  206, 176, 215, 179, 89, 66, 188, 35, 0, 67, 159, 19, 189, 219, 51, 207,
  235, 196, 162, 110, 187, 64, 50, 36, 223, 105, 146, 141, 61, 185, 199, 239,
  53, 251, 39, 233, 101, 181, 96, 212, 69, 131, 73, 165, 199, 114, 130, 99,
  113, 63, 191, 63, 249, 40, 192, 197, 45, 114, 212, 14, 95, 54, 78, 206,
  158, 37, 213, 241, 65, 110, 133, 118, 51, 75, 237, 137, 55, 191, 196, 86,
  235, 246, 73, 215, 115, 217, 67, 138, 45, 13, 116, 23, 14, 63, 2, 92,
  153, 125, 206, 196, 201, 75, 245, 38, 48, 18, 229, 177, 209, 110, 118, 108,
  42, 9, 220, 199, 101, 99, 148, 79, 108, 234, 176, 18, 15, 15, 86, 26,
  171, 22, 179, 227, 112, 200, 193, 18, 118, 37, 158, 115, 210, 123, 231, 230,
  125, 156, 49, 138, 164, 110, 142, 214, 108, 15, 152, 190, 78, 96, 89, 65,
  173, 140, 40, 45, 188, 28, 220, 197, 3, 25, 50, 252, 248, 122, 106, 240,
  111, 17, 255, 192, 216, 141, 111, 115, 12, 157, 181, 150, 35, 55, 65, 233,
  129, 219, 199, 202, 47, 58, 238, 13, 188, 15, 13, 140, 219, 248, 54, 219,
  184, 121, 208, 55, 61, 211, 148, 153, 86, 38, 171, 226, 9, 79, 240, 98,
  89, 252, 249, 60, 47, 121, 43, 147, 43, 137, 139, 114, 118, 147, 43, 233,
  106, 162, 170, 232, 186, 241, 102, 235, 205, 219, 189, 237, 39, 91, 155, 187,
  187, 155, 239, 151, 63, 250, 175, 42, 97, 143, 96, 45, 85, 159, 60, 1,
  139, 217, 243, 162, 66, 108, 145, 119, 217, 37, 17, 75, 193, 61, 180, 216,
  165, 9, 251, 76, 60, 124, 8, 37, 107, 161, 39, 139, 235, 10, 129, 217,
  242, 62, 206, 49, 93, 66, 157, 190, 202, 124, 89, 32, 194, 19, 230, 99,
  123, 13, 51, 102, 246, 253, 187, 78, 25, 153, 166, 86, 39, 142, 23, 126,
  33, 182, 18, 173, 220, 56, 69, 242, 248, 71, 209, 209, 40, 25, 178, 186,
  235, 125, 59, 209, 188, 167, 67, 48, 101, 217, 168, 46, 50, 204, 199, 107,
  52, 244, 108, 60, 180, 235, 235, 114, 255, 21, 197, 113, 126, 190, 97, 32,
  165, 17, 132, 147, 200, 173, 164, 112, 235, 227, 197, 170, 237, 251, 10, 179,
  116, 234, 235, 237, 44, 250, 133, 93, 81, 117, 243, 212, 141, 140, 225, 188,
  162, 103, 129, 52, 236, 182, 78, 144, 13, 71, 127, 50, 198, 215, 33, 16,
  70, 243, 99, 5, 194, 19, 221, 119, 98, 142, 67, 97, 222, 236, 49, 196,
  206, 152, 154, 95, 168, 83, 147, 132, 48, 25, 41, 237, 129, 177, 162, 70,
  104, 246, 188, 194, 178, 35, 193, 11, 47, 13, 179, 110, 99, 116, 143, 151,
  5, 30, 105, 208, 185, 40, 43, 16, 168, 248, 115, 79, 39, 215, 234, 253,
  246, 183, 191, 187, 14, 49, 246, 59, 208, 138, 121, 91, 176, 150, 236, 9,
  143, 4, 131, 63, 90, 0, 113, 31, 84, 230, 245, 150, 148, 21, 203, 233,
  144, 31, 241, 214, 24, 182, 137, 31, 175, 198, 147, 211, 131, 180, 196, 19,
  169, 254, 176, 234, 106, 180, 129, 23, 7, 254, 96, 187, 142, 167, 3, 186,
  62, 10, 1, 38, 194, 53, 70, 221, 184, 226, 60, 61, 231, 235, 79, 46,
  60, 140, 119, 129, 212, 90, 112, 38, 28, 23, 111, 96, 198, 17, 122, 43,
  235, 32, 236, 32, 147, 135, 150, 219, 172, 62, 242, 95, 232, 113, 210, 148,
  12, 102, 97, 159, 79, 89, 209, 207, 132, 52, 99, 135, 84, 217, 32, 214,
  109, 29, 236, 172, 249, 88, 71, 31, 247, 3, 88, 250, 199, 34, 127, 223,
  43, 239, 137, 76, 116, 38, 70, 51, 238, 181, 165, 223, 86, 252, 131, 156,
  236, 165, 61, 63, 23, 95, 169, 176, 186, 252, 75, 75, 126, 167, 81, 122,
  150, 202, 40, 171, 176, 152, 168, 117, 196, 122, 142, 30, 63, 138, 150, 238,
  47, 195, 207, 123, 48, 36, 85, 242, 61, 150, 220, 53, 75, 126, 192, 146,
  59, 102, 201, 143, 203, 242, 109, 84, 140, 255, 49, 18, 198, 128, 174, 176,
  128, 185, 221, 178, 71, 43, 107, 217, 195, 71, 247, 214, 178, 239, 190, 163,
  120, 234, 32, 41, 103, 113, 75, 132, 234, 54, 87, 32, 45, 150, 160, 180,
  231, 50, 95, 22, 61, 124, 196, 73, 16, 235, 86, 77, 71, 111, 14, 156,
  251, 49, 30, 167, 217, 240, 88, 196, 144, 251, 62, 250, 46, 202, 254, 207,
  221, 105, 113, 177, 111, 199, 137, 22, 147, 96, 165, 129, 77, 202, 78, 17,
  199, 84, 232, 137, 201, 224, 229, 120, 128, 14, 78, 121, 233, 197, 204, 186,
  197, 206, 122, 91, 92, 200, 243, 31, 79, 147, 193, 144, 8, 248, 132, 132,
  150, 250, 89, 239, 79, 233, 209, 61, 248, 95, 207, 254, 44, 179, 64, 247,
  242, 163, 35, 102, 75, 89, 35, 250, 226, 135, 22, 153, 105, 222, 154, 45,
  213, 62, 153, 184, 201, 146, 131, 253, 10, 65, 236, 194, 63, 164, 55, 164,
  6, 12, 187, 244, 226, 179, 214, 113, 82, 189, 172, 170, 9, 123, 140, 104,
  116, 255, 175, 19, 88, 175, 209, 227, 104, 153, 237, 157, 186, 28, 125, 39,
  39, 248, 116, 81, 124, 186, 101, 226, 12, 204, 83, 51, 175, 75, 151, 205,
  84, 39, 158, 43, 175, 28, 78, 81, 230, 7, 200, 207, 61, 55, 193, 178,
  28, 209, 209, 253, 7, 233, 242, 65, 175, 117, 11, 211, 4, 58, 9, 194,
  186, 115, 231, 240, 254, 125, 187, 43, 82, 92, 10, 206, 18, 236, 126, 144,
  28, 158, 12, 203, 124, 50, 230, 122, 226, 200, 28, 166, 91, 53, 191, 216,
  57, 78, 6, 57, 90, 139, 247, 151, 35, 252, 255, 221, 226, 34, 250, 230,
  51, 107, 54, 189, 123, 119, 223, 105, 202, 88, 214, 145, 169, 108, 36, 100,
  189, 142, 248, 152, 149, 245, 224, 151, 143, 86, 126, 184, 147, 52, 136, 93,
  126, 2, 216, 144, 211, 58, 169, 2, 10, 189, 183, 22, 76, 165, 91, 173,
  143, 53, 170, 65, 57, 25, 163, 185, 82, 38, 27, 21, 63, 247, 20, 28,
  144, 191, 24, 154, 241, 8, 24, 28, 29, 40, 137, 150, 171, 70, 87, 78,
  243, 64, 151, 160, 137, 150, 3, 126, 52, 224, 173, 116, 65, 67, 119, 170,
  146, 213, 161, 44, 165, 187, 42, 210, 178, 226, 30, 238, 220, 15, 157, 255,
  220, 3, 169, 13, 251, 93, 157, 37, 163, 170, 169, 75, 217, 218, 236, 144,
  0, 225, 121, 112, 25, 32, 64, 97, 47, 147, 97, 42, 238, 141, 76, 53,
  81, 124, 97, 199, 39, 145, 252, 129, 121, 130, 42, 117, 209, 170, 176, 26,
  0, 74, 118, 74, 30, 237, 250, 225, 67, 29, 10, 139, 159, 10, 60, 143,
  242, 152, 229, 5, 108, 19, 233, 43, 206, 108, 76, 146, 236, 77, 216, 215,
  189, 211, 138, 108, 187, 13, 104, 135, 90, 98, 116, 186, 80, 187, 231, 92,
  130, 133, 154, 10, 1, 23, 106, 189, 155, 86, 117, 168, 105, 13, 223, 156,
  118, 182, 108, 228, 195, 133, 182, 175, 153, 110, 209, 207, 170, 231, 217, 56,
  131, 19, 179, 69, 139, 24, 230, 194, 38, 142, 165, 52, 56, 48, 183, 57,
  75, 83, 16, 5, 133, 36, 60, 73, 176, 38, 104, 130, 56, 1, 128, 154,
  116, 18, 166, 65, 204, 38, 176, 72, 181, 0, 76, 65, 80, 9, 80, 210,
  55, 4, 141, 239, 75, 54, 83, 179, 178, 189, 42, 251, 148, 146, 39, 32,
  227, 243, 42, 159, 181, 151, 160, 53, 185, 159, 240, 200, 181, 12, 234, 255,
  202, 50, 61, 127, 106, 227, 179, 251, 214, 197, 68, 207, 234, 163, 219, 175,
  250, 208, 214, 171, 218, 81, 165, 48, 97, 140, 150, 178, 18, 55, 42, 2,
  209, 106, 103, 114, 116, 148, 93, 248, 109, 247, 42, 254, 161, 25, 196, 118,
  130, 177, 121, 35, 119, 65, 166, 131, 189, 2, 191, 216, 138, 129, 108, 16,
  132, 246, 54, 77, 42, 246, 118, 87, 195, 195, 195, 74, 186, 87, 242, 15,
  62, 56, 222, 194, 207, 99, 40, 105, 207, 45, 96, 123, 71, 249, 8, 99,
  103, 90, 210, 12, 206, 127, 126, 149, 216, 236, 128, 127, 121, 46, 218, 18,
  181, 29, 19, 11, 182, 20, 222, 253, 12, 128, 0, 102, 22, 185, 217, 85,
  253, 13, 18, 78, 70, 151, 105, 37, 242, 239, 246, 136, 30, 222, 242, 173,
  204, 236, 193, 44, 10, 247, 32, 247, 197, 246, 30, 228, 222, 101, 245, 97,
  21, 54, 244, 162, 182, 195, 214, 126, 12, 131, 138, 253, 59, 12, 93, 239,
  32, 173, 208, 255, 9, 23, 238, 14, 172, 91, 1, 93, 253, 118, 160, 139,
  119, 14, 174, 0, 137, 9, 144, 66, 148, 85, 2, 162, 252, 217, 10, 80,
  174, 101, 10, 38, 46, 136, 23, 152, 148, 62, 178, 126, 134, 73, 96, 113,
  229, 122, 180, 207, 247, 128, 10, 68, 35, 158, 149, 163, 108, 140, 58, 36,
  93, 125, 122, 155, 189, 198, 242, 91, 220, 254, 115, 153, 159, 243, 235, 188,
  219, 15, 165, 213, 248, 241, 109, 138, 172, 91, 82, 112, 8, 132, 213, 239,
  48, 198, 90, 66, 73, 163, 85, 0, 46, 71, 140, 180, 235, 212, 201, 105,
  225, 67, 21, 187, 165, 47, 23, 216, 110, 66, 72, 31, 152, 24, 34, 146,
  208, 105, 65, 199, 150, 99, 159, 158, 137, 92, 235, 233, 57, 11, 250, 43,
  234, 175, 81, 213, 211, 67, 148, 253, 44, 65, 17, 83, 186, 23, 236, 44,
  89, 162, 167, 219, 32, 202, 173, 72, 176, 134, 21, 123, 200, 19, 57, 1,
  156, 135, 209, 125, 228, 241, 97, 90, 130, 56, 141, 210, 3, 110, 39, 129,
  99, 52, 126, 157, 86, 80, 53, 119, 194, 175, 59, 100, 244, 131, 184, 171,
  225, 176, 164, 43, 24, 174, 78, 190, 241, 97, 129, 221, 241, 133, 155, 29,
  210, 157, 60, 61, 53, 119, 227, 76, 111, 216, 130, 167, 192, 224, 84, 249,
  54, 105, 98, 62, 3, 115, 212, 66, 116, 18, 16, 61, 5, 38, 70, 206,
  176, 230, 154, 146, 102, 242, 17, 189, 204, 70, 60, 78, 127, 20, 24, 98,
  41, 234, 130, 230, 181, 184, 45, 182, 230, 208, 82, 100, 53, 249, 206, 106,
  238, 135, 188, 164, 25, 244, 91, 106, 163, 54, 207, 26, 126, 183, 180, 9,
  135, 186, 132, 217, 202, 199, 67, 118, 115, 92, 209, 91, 39, 236, 232, 228,
  86, 97, 151, 171, 13, 202, 211, 221, 37, 120, 118, 249, 194, 175, 122, 200,
  78, 104, 35, 140, 54, 46, 57, 253, 80, 82, 159, 93, 173, 189, 206, 107,
  16, 180, 142, 184, 115, 190, 138, 179, 249, 128, 135, 83, 199, 35, 191, 139,
  37, 112, 228, 209, 40, 189, 16, 59, 161, 99, 238, 97, 239, 163, 157, 22,
  113, 32, 130, 174, 208, 189, 110, 133, 182, 91, 238, 154, 33, 141, 74, 216,
  157, 210, 9, 96, 15, 73, 78, 234, 236, 172, 231, 199, 184, 117, 7, 180,
  203, 146, 225, 81, 165, 190, 192, 226, 56, 77, 163, 223, 254, 246, 255, 162,
  79, 105, 6, 194, 113, 148, 159, 96, 148, 205, 59, 247, 142, 247, 59, 175,
  19, 29, 58, 218, 250, 105, 102, 37, 187, 181, 64, 207, 52, 109, 109, 11,
  44, 153, 73, 245, 170, 26, 98, 171, 102, 130, 223, 20, 185, 109, 60, 174,
  64, 76, 7, 80, 123, 98, 39, 195, 60, 233, 19, 163, 233, 204, 129, 234,
  120, 153, 14, 178, 146, 13, 122, 169, 206, 151, 48, 174, 120, 213, 35, 150,
  68, 104, 96, 189, 157, 180, 60, 3, 69, 72, 131, 137, 234, 60, 122, 177,
  187, 187, 189, 179, 22, 77, 0, 43, 246, 103, 4, 186, 7, 246, 136, 9,
  24, 25, 56, 30, 44, 104, 216, 239, 221, 220, 192, 111, 121, 43, 21, 69,
  71, 120, 48, 253, 108, 124, 56, 154, 12, 210, 106, 193, 153, 250, 216, 18,
  167, 77, 237, 124, 6, 233, 222, 54, 76, 199, 94, 28, 207, 50, 33, 51,
  146, 148, 140, 109, 218, 228, 110, 200, 64, 46, 149, 204, 141, 195, 123, 61,
  139, 15, 97, 93, 137, 233, 108, 29, 176, 180, 178, 49, 18, 118, 137, 167,
  159, 80, 114, 211, 245, 53, 117, 218, 33, 47, 107, 71, 38, 22, 218, 137,
  189, 182, 9, 187, 58, 121, 14, 77, 180, 155, 85, 192, 35, 138, 104, 79,
  123, 53, 5, 239, 31, 130, 91, 147, 148, 112, 78, 177, 48, 33, 74, 233,
  172, 13, 137, 111, 128, 51, 202, 108, 64, 36, 147, 80, 155, 243, 100, 4,
  199, 183, 209, 40, 41, 216, 11, 200, 14, 219, 176, 180, 97, 42, 41, 75,
  244, 71, 109, 156, 186, 22, 181, 113, 234, 175, 196, 253, 237, 161, 64, 16,
  47, 163, 109, 148, 73, 114, 138, 232, 148, 149, 172, 164, 20, 2, 163, 151,
  117, 163, 27, 168, 95, 39, 152, 198, 212, 232, 40, 110, 30, 194, 211, 124,
  112, 233, 147, 251, 41, 139, 33, 224, 112, 177, 137, 199, 186, 224, 88, 100,
  93, 206, 196, 205, 221, 200, 64, 86, 116, 185, 179, 128, 157, 142, 126, 251,
  143, 95, 88, 63, 191, 253, 199, 127, 81, 189, 108, 14, 178, 90, 239, 173,
  226, 87, 27, 242, 98, 253, 133, 212, 22, 173, 241, 237, 76, 78, 79, 147,
  210, 133, 16, 100, 36, 138, 237, 154, 41, 35, 58, 240, 73, 35, 62, 144,
  194, 68, 98, 213, 160, 121, 53, 244, 197, 21, 32, 95, 35, 128, 205, 74,
  89, 153, 240, 111, 12, 156, 161, 164, 247, 79, 80, 240, 114, 92, 76, 106,
  51, 238, 4, 22, 250, 210, 150, 238, 209, 61, 207, 88, 195, 216, 223, 132,
  61, 35, 29, 1, 55, 192, 17, 116, 132, 97, 208, 17, 129, 41, 38, 15,
  84, 225, 199, 227, 125, 39, 61, 232, 180, 203, 237, 16, 94, 248, 211, 119,
  67, 174, 197, 204, 226, 81, 171, 172, 175, 83, 101, 152, 215, 70, 60, 15,
  113, 201, 105, 225, 125, 88, 33, 230, 226, 167, 114, 196, 104, 136, 172, 98,
  26, 9, 15, 146, 42, 221, 99, 129, 186, 12, 251, 160, 60, 83, 137, 54,
  74, 40, 91, 45, 40, 123, 78, 153, 215, 57, 8, 0, 145, 62, 206, 233,
  170, 16, 95, 169, 174, 236, 150, 118, 135, 178, 29, 53, 42, 201, 26, 110,
  95, 138, 137, 168, 97, 201, 70, 118, 55, 178, 9, 105, 20, 68, 214, 129,
  233, 213, 3, 19, 23, 93, 162, 92, 171, 215, 102, 61, 5, 95, 152, 26,
  156, 54, 164, 65, 172, 170, 24, 106, 202, 34, 38, 126, 235, 125, 145, 90,
  104, 219, 92, 12, 88, 108, 100, 149, 185, 108, 68, 221, 185, 221, 114, 238,
  220, 186, 92, 175, 193, 178, 48, 67, 254, 135, 2, 216, 177, 224, 87, 230,
  141, 105, 123, 240, 44, 47, 223, 146, 17, 59, 235, 208, 246, 138, 215, 110,
  153, 122, 217, 217, 6, 199, 136, 184, 180, 181, 107, 132, 67, 37, 249, 10,
  46, 250, 33, 53, 170, 129, 70, 216, 36, 38, 171, 184, 74, 11, 103, 144,
  129, 136, 173, 212, 91, 235, 172, 111, 59, 161, 159, 232, 235, 68, 55, 201,
  85, 155, 89, 161, 45, 224, 32, 203, 211, 33, 172, 210, 112, 82, 88, 16,
  194, 229, 205, 120, 116, 41, 187, 139, 175, 49, 143, 116, 149, 80, 15, 152,
  65, 196, 136, 108, 205, 166, 44, 18, 123, 192, 186, 8, 58, 74, 133, 179,
  246, 228, 221, 45, 3, 255, 216, 192, 64, 74, 50, 12, 95, 201, 58, 139,
  59, 201, 180, 16, 56, 41, 167, 120, 212, 138, 128, 76, 67, 124, 241, 160,
  71, 106, 71, 150, 56, 11, 117, 35, 229, 148, 236, 198, 21, 103, 110, 84,
  38, 95, 184, 184, 208, 149, 250, 224, 136, 27, 107, 112, 60, 204, 241, 64,
  13, 206, 169, 218, 38, 149, 26, 9, 231, 203, 23, 213, 13, 41, 197, 232,
  152, 81, 226, 225, 36, 239, 195, 96, 29, 107, 67, 37, 3, 79, 53, 135,
  176, 210, 16, 73, 62, 86, 182, 46, 137, 51, 113, 23, 68, 96, 219, 180,
  185, 132, 8, 165, 246, 14, 217, 21, 189, 225, 32, 7, 172, 52, 101, 136,
  13, 69, 215, 226, 130, 245, 70, 66, 107, 17, 169, 109, 102, 148, 170, 102,
  162, 227, 110, 2, 20, 93, 156, 58, 197, 206, 158, 7, 155, 134, 48, 216,
  115, 200, 118, 253, 151, 191, 87, 182, 237, 174, 232, 87, 193, 26, 44, 144,
  58, 230, 44, 99, 194, 93, 50, 98, 198, 121, 230, 69, 219, 197, 130, 113,
  197, 172, 44, 172, 139, 219, 56, 132, 198, 92, 202, 173, 17, 237, 205, 138,
  232, 170, 168, 244, 110, 47, 131, 154, 252, 196, 0, 173, 71, 255, 119, 231,
  205, 235, 62, 115, 63, 88, 168, 217, 233, 100, 21, 27, 185, 204, 98, 192,
  179, 119, 100, 158, 131, 23, 159, 50, 240, 152, 249, 204, 208, 197, 180, 38,
  44, 68, 101, 92, 169, 232, 162, 192, 123, 82, 34, 31, 103, 12, 148, 42,
  202, 254, 94, 23, 94, 18, 184, 91, 108, 184, 5, 123, 178, 138, 142, 29,
  239, 131, 44, 12, 7, 5, 233, 146, 224, 152, 168, 184, 200, 44, 75, 180,
  219, 201, 35, 70, 89, 234, 83, 5, 229, 73, 49, 23, 111, 193, 224, 225,
  172, 133, 172, 181, 132, 87, 84, 25, 102, 119, 66, 110, 171, 241, 246, 13,
  7, 51, 253, 230, 51, 195, 17, 106, 69, 191, 253, 219, 191, 71, 252, 231,
  148, 159, 188, 166, 236, 4, 198, 154, 59, 203, 77, 54, 103, 72, 142, 111,
  39, 189, 41, 11, 91, 137, 227, 1, 72, 34, 136, 37, 252, 210, 128, 26,
  217, 154, 161, 105, 200, 15, 108, 20, 84, 199, 144, 12, 152, 37, 193, 87,
  46, 63, 235, 135, 50, 171, 60, 223, 171, 233, 1, 181, 106, 100, 80, 21,
  95, 44, 215, 145, 85, 62, 115, 211, 155, 210, 79, 123, 4, 29, 191, 188,
  216, 98, 221, 61, 97, 127, 47, 112, 93, 240, 26, 117, 74, 1, 240, 143,
  190, 5, 114, 52, 229, 33, 21, 19, 202, 43, 93, 108, 22, 211, 128, 155,
  118, 141, 228, 9, 255, 38, 54, 196, 31, 4, 86, 235, 230, 13, 12, 112,
  13, 150, 166, 181, 115, 51, 131, 229, 121, 81, 212, 189, 181, 166, 190, 66,
  59, 117, 32, 174, 94, 152, 177, 174, 123, 85, 60, 57, 17, 135, 213, 235,
  91, 23, 193, 3, 113, 54, 206, 170, 99, 116, 60, 33, 28, 246, 121, 166,
  130, 133, 30, 126, 142, 206, 179, 242, 4, 216, 235, 56, 58, 72, 83, 230,
  177, 130, 254, 204, 79, 14, 16, 189, 73, 85, 13, 152, 40, 5, 182, 196,
  252, 14, 233, 120, 157, 10, 135, 56, 15, 41, 54, 5, 68, 221, 211, 65,
  90, 2, 22, 243, 233, 5, 87, 61, 31, 98, 20, 247, 61, 24, 125, 239,
  138, 73, 131, 190, 220, 218, 254, 223, 173, 170, 180, 235, 21, 87, 20, 64,
  92, 89, 112, 121, 79, 242, 121, 221, 168, 41, 224, 63, 11, 142, 242, 162,
  32, 53, 164, 148, 105, 145, 84, 109, 138, 193, 140, 217, 122, 230, 146, 79,
  161, 81, 124, 241, 173, 27, 22, 193, 48, 229, 103, 14, 195, 226, 222, 32,
  202, 88, 213, 37, 163, 174, 150, 107, 163, 95, 255, 27, 51, 51, 209, 178,
  235, 170, 146, 131, 35, 250, 15, 114, 20, 150, 38, 171, 144, 245, 61, 176,
  196, 148, 253, 39, 104, 222, 14, 52, 156, 205, 216, 60, 167, 185, 217, 179,
  77, 54, 220, 175, 6, 213, 24, 235, 226, 179, 225, 178, 83, 70, 219, 33,
  239, 32, 175, 124, 11, 233, 93, 55, 206, 124, 21, 72, 251, 40, 93, 233,
  198, 210, 185, 155, 156, 79, 178, 152, 11, 115, 152, 242, 21, 249, 71, 176,
  177, 108, 9, 217, 240, 229, 5, 158, 249, 20, 88, 190, 251, 237, 244, 36,
  88, 63, 8, 54, 154, 197, 62, 164, 25, 162, 220, 89, 143, 135, 31, 69,
  183, 204, 223, 70, 116, 122, 59, 200, 4, 247, 247, 196, 39, 131, 32, 241,
  182, 147, 75, 230, 92, 28, 204, 18, 197, 201, 153, 56, 129, 148, 85, 169,
  19, 195, 217, 168, 28, 91, 77, 219, 35, 17, 57, 23, 11, 252, 42, 193,
  141, 39, 141, 21, 240, 136, 234, 35, 131, 165, 4, 50, 162, 114, 108, 53,
  109, 69, 198, 176, 155, 81, 24, 236, 224, 25, 135, 160, 7, 59, 16, 17,
  4, 17, 213, 99, 187, 117, 43, 22, 230, 49, 88, 156, 182, 2, 232, 228,
  5, 133, 77, 94, 144, 200, 228, 133, 137, 75, 94, 204, 129, 74, 94, 208,
  152, 60, 103, 135, 23, 31, 23, 94, 78, 96, 163, 26, 196, 14, 128, 86,
  140, 204, 115, 146, 15, 55, 228, 220, 56, 207, 196, 207, 112, 37, 223, 26,
  88, 199, 229, 114, 166, 70, 19, 61, 145, 123, 70, 119, 185, 208, 121, 39,
  189, 222, 125, 244, 186, 119, 209, 43, 238, 161, 215, 187, 131, 78, 253, 105,
  234, 14, 119, 134, 169, 147, 209, 144, 174, 195, 185, 106, 54, 62, 144, 16,
  154, 61, 200, 214, 255, 161, 181, 170, 121, 52, 39, 42, 2, 64, 3, 81,
  88, 252, 135, 27, 162, 72, 88, 75, 236, 72, 16, 211, 199, 108, 30, 186,
  252, 210, 163, 211, 86, 184, 114, 157, 157, 174, 12, 185, 46, 127, 183, 203,
  245, 192, 1, 146, 10, 205, 153, 214, 207, 210, 51, 150, 80, 114, 148, 46,
  116, 141, 203, 57, 72, 207, 150, 242, 241, 232, 210, 240, 146, 197, 176, 7,
  143, 49, 76, 174, 233, 251, 46, 179, 69, 249, 201, 192, 89, 134, 159, 13,
  43, 92, 165, 81, 232, 223, 59, 3, 36, 55, 172, 141, 42, 114, 197, 167,
  1, 40, 54, 187, 242, 102, 82, 160, 135, 166, 7, 199, 97, 215, 6, 168,
  122, 138, 53, 30, 77, 192, 104, 39, 224, 128, 22, 204, 51, 155, 156, 89,
  27, 190, 40, 179, 55, 123, 93, 49, 54, 26, 205, 32, 22, 97, 174, 89,
  146, 15, 57, 219, 154, 142, 34, 43, 137, 247, 170, 95, 197, 90, 82, 195,
  134, 227, 128, 155, 184, 22, 195, 192, 226, 137, 150, 213, 194, 171, 52, 187,
  130, 151, 207, 4, 211, 229, 140, 220, 17, 171, 82, 71, 193, 49, 42, 199,
  86, 211, 46, 209, 105, 219, 70, 75, 234, 199, 44, 234, 12, 232, 248, 46,
  134, 70, 185, 141, 163, 221, 32, 118, 0, 204, 177, 109, 13, 82, 139, 201,
  25, 73, 91, 178, 209, 135, 150, 6, 78, 151, 231, 124, 46, 88, 153, 180,
  100, 203, 200, 12, 207, 54, 223, 237, 109, 188, 121, 182, 105, 239, 99, 146,
  199, 125, 99, 66, 69, 70, 141, 2, 18, 68, 236, 181, 64, 198, 188, 10,
  252, 250, 29, 110, 43, 58, 207, 163, 211, 200, 20, 107, 45, 1, 230, 26,
  82, 154, 151, 17, 35, 127, 71, 212, 157, 55, 28, 254, 121, 208, 177, 244,
  217, 225, 14, 253, 92, 123, 77, 1, 35, 85, 30, 40, 180, 70, 191, 218,
  220, 125, 251, 114, 99, 15, 254, 121, 242, 222, 108, 246, 17, 249, 229, 115,
  52, 25, 103, 245, 106, 15, 147, 129, 98, 84, 151, 213, 200, 172, 98, 39,
  150, 198, 200, 142, 135, 217, 41, 12, 126, 117, 37, 154, 218, 204, 113, 249,
  228, 34, 171, 182, 16, 66, 28, 233, 191, 253, 231, 64, 136, 83, 159, 245,
  196, 30, 127, 178, 159, 136, 129, 245, 4, 20, 1, 94, 24, 0, 9, 99,
  198, 225, 137, 120, 228, 7, 52, 243, 223, 150, 233, 152, 157, 175, 84, 56,
  17, 150, 55, 40, 142, 46, 66, 200, 245, 254, 154, 102, 117, 180, 240, 226,
  197, 234, 171, 87, 171, 59, 59, 252, 153, 82, 236, 92, 168, 169, 135, 55,
  46, 116, 254, 38, 163, 19, 112, 14, 121, 17, 31, 64, 145, 224, 103, 0,
  18, 183, 71, 29, 10, 66, 35, 233, 131, 59, 20, 239, 229, 244, 116, 181,
  170, 96, 173, 192, 223, 236, 64, 29, 179, 205, 202, 196, 160, 76, 71, 9,
  44, 223, 184, 61, 90, 145, 253, 206, 145, 232, 27, 99, 243, 179, 27, 16,
  167, 252, 65, 160, 220, 201, 55, 96, 76, 136, 136, 116, 140, 149, 206, 138,
  65, 207, 122, 23, 199, 156, 135, 27, 98, 202, 219, 43, 159, 10, 96, 142,
  70, 105, 30, 6, 158, 123, 223, 218, 142, 2, 86, 192, 46, 116, 117, 215,
  241, 244, 195, 216, 53, 32, 100, 63, 231, 114, 2, 147, 23, 3, 1, 125,
  35, 25, 159, 37, 32, 178, 221, 18, 242, 13, 129, 196, 168, 93, 171, 97,
  81, 98, 89, 112, 95, 1, 223, 43, 106, 235, 160, 249, 173, 8, 194, 217,
  60, 45, 234, 75, 241, 18, 137, 253, 221, 170, 144, 171, 200, 96, 152, 244,
  225, 25, 191, 109, 11, 49, 50, 212, 122, 155, 30, 194, 92, 237, 36, 167,
  197, 200, 13, 82, 251, 253, 50, 62, 197, 6, 244, 160, 26, 2, 122, 57,
  102, 84, 246, 66, 217, 74, 216, 158, 214, 153, 143, 69, 234, 131, 80, 255,
  60, 13, 221, 203, 10, 72, 50, 170, 143, 47, 29, 200, 164, 149, 208, 113,
  234, 54, 198, 251, 100, 124, 41, 134, 203, 193, 190, 224, 88, 219, 81, 127,
  89, 188, 201, 48, 171, 25, 100, 243, 3, 217, 142, 70, 249, 249, 27, 254,
  244, 175, 145, 95, 13, 32, 142, 238, 96, 96, 137, 47, 188, 204, 73, 98,
  113, 48, 5, 193, 88, 244, 90, 163, 183, 216, 151, 239, 188, 247, 173, 116,
  8, 91, 160, 244, 241, 225, 191, 172, 79, 139, 152, 60, 153, 124, 232, 214,
  129, 177, 188, 19, 218, 0, 38, 163, 78, 121, 155, 173, 228, 50, 159, 212,
  149, 200, 94, 18, 8, 14, 30, 72, 109, 210, 16, 30, 220, 221, 166, 109,
  137, 81, 36, 25, 28, 200, 182, 243, 108, 204, 50, 125, 216, 80, 101, 52,
  88, 149, 66, 195, 202, 41, 108, 183, 196, 179, 49, 11, 108, 139, 165, 11,
  36, 11, 139, 41, 51, 27, 246, 65, 172, 13, 235, 99, 49, 199, 157, 150,
  4, 17, 145, 118, 70, 153, 201, 90, 89, 146, 210, 187, 181, 10, 61, 178,
  14, 82, 227, 11, 76, 228, 85, 153, 148, 136, 39, 46, 60, 217, 116, 145,
  95, 73, 196, 211, 238, 22, 124, 59, 16, 24, 60, 166, 156, 10, 36, 227,
  88, 124, 196, 34, 19, 119, 96, 36, 124, 29, 250, 78, 237, 61, 230, 198,
  84, 95, 44, 122, 187, 146, 201, 63, 92, 8, 66, 219, 93, 150, 122, 163,
  90, 244, 19, 112, 242, 3, 255, 162, 64, 209, 89, 76, 255, 58, 73, 171,
  250, 201, 24, 52, 82, 84, 152, 159, 151, 160, 111, 47, 240, 115, 211, 151,
  196, 41, 238, 18, 202, 50, 195, 96, 20, 151, 106, 133, 234, 124, 198, 92,
  191, 127, 193, 191, 119, 217, 124, 44, 111, 18, 28, 231, 51, 189, 47, 7,
  219, 47, 218, 24, 44, 178, 163, 193, 188, 162, 147, 251, 99, 168, 221, 221,
  17, 164, 109, 239, 243, 144, 180, 217, 72, 133, 98, 167, 182, 65, 118, 46,
  33, 66, 180, 99, 134, 10, 39, 200, 25, 47, 34, 2, 156, 177, 15, 171,
  206, 138, 22, 147, 49, 57, 117, 160, 64, 73, 54, 200, 234, 75, 18, 146,
  250, 72, 67, 3, 102, 114, 160, 177, 18, 19, 16, 238, 120, 42, 192, 220,
  235, 228, 245, 130, 172, 22, 203, 62, 176, 9, 13, 30, 83, 191, 184, 198,
  173, 51, 78, 66, 223, 192, 165, 155, 188, 206, 133, 182, 96, 55, 225, 197,
  126, 147, 130, 222, 58, 196, 153, 169, 95, 141, 80, 206, 44, 125, 191, 236,
  154, 11, 12, 236, 112, 203, 215, 61, 211, 22, 4, 53, 91, 56, 82, 108,
  193, 166, 194, 248, 45, 73, 39, 127, 91, 84, 195, 214, 177, 87, 10, 32,
  88, 97, 97, 109, 98, 8, 100, 217, 222, 22, 52, 114, 29, 148, 1, 61,
  48, 247, 12, 118, 146, 162, 18, 131, 94, 196, 227, 94, 183, 189, 189, 169,
  99, 71, 189, 13, 118, 187, 255, 110, 251, 89, 244, 205, 103, 160, 15, 28,
  201, 159, 103, 23, 48, 57, 119, 226, 105, 116, 178, 157, 24, 199, 102, 95,
  234, 225, 156, 75, 145, 199, 254, 22, 242, 142, 79, 108, 80, 214, 201, 218,
  199, 236, 74, 148, 144, 190, 179, 201, 222, 235, 197, 130, 72, 68, 134, 237,
  118, 185, 199, 223, 130, 13, 172, 15, 122, 38, 123, 94, 201, 56, 10, 15,
  131, 246, 231, 227, 108, 120, 172, 190, 199, 198, 116, 174, 163, 177, 194, 135,
  101, 82, 255, 183, 191, 253, 221, 171, 130, 240, 188, 25, 98, 239, 216, 173,
  224, 67, 74, 116, 224, 0, 121, 91, 111, 173, 242, 98, 215, 66, 103, 125,
  140, 109, 16, 86, 60, 241, 253, 135, 85, 145, 160, 171, 119, 82, 85, 143,
  62, 124, 61, 98, 202, 200, 82, 117, 142, 62, 34, 31, 190, 142, 24, 39,
  66, 185, 14, 213, 187, 202, 216, 139, 169, 27, 20, 17, 97, 137, 45, 247,
  127, 140, 167, 31, 190, 126, 252, 240, 54, 130, 126, 28, 253, 53, 75, 71,
  192, 148, 154, 254, 211, 125, 207, 142, 13, 42, 225, 75, 96, 226, 242, 44,
  25, 45, 48, 231, 15, 92, 7, 139, 209, 157, 251, 70, 28, 44, 175, 142,
  225, 32, 178, 24, 221, 95, 14, 84, 229, 251, 201, 174, 136, 64, 136, 85,
  173, 232, 90, 69, 94, 76, 70, 176, 76, 177, 194, 39, 88, 102, 66, 171,
  50, 65, 81, 102, 64, 23, 170, 254, 130, 91, 211, 78, 90, 99, 116, 125,
  163, 180, 58, 76, 198, 175, 211, 250, 60, 47, 79, 156, 186, 91, 185, 89,
  47, 236, 249, 194, 163, 103, 177, 155, 8, 163, 180, 40, 1, 5, 169, 23,
  216, 80, 216, 193, 79, 20, 193, 92, 28, 150, 89, 81, 63, 254, 10, 255,
  70, 31, 55, 254, 215, 113, 125, 58, 194, 191, 190, 250, 31, 247, 87, 222,
  163, 60, 222, 2, 0
};
const size_t INDEX_HTML_GZ_LEN = sizeof(INDEX_HTML_GZ);


void handleRoot() {
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Content-Encoding", "gzip");
  server.send_P(200, "text/html", (PGM_P)INDEX_HTML_GZ, INDEX_HTML_GZ_LEN);
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

void handleFactors() {
  if (!enforceAuth())
    return;
  if (server.method() == HTTP_GET) {
    const VpdProfile &profile = vpdProfileById(vpdStageId);
    String json = "{";
    json += "\"channel\":\"" + lightChannelName() + "\",";
    json += "\"ppfd_scale\":" + String(ppfdScale, 4) + ",";
    json += "\"ppfd_factor\":" + String(currentPpfdFactor(), 6) + ",";
    json += "\"vpd_stage\":\"" + vpdStageId + "\",";
    json += "\"vpd_target_low\":" + String(profile.targetLow, 2) + ",";
    json += "\"vpd_target_high\":" + String(profile.targetHigh, 2) + ",";
    json += "\"phase\":\"" + phaseId + "\"";
    json += "}";
    server.send(200, "application/json", json);
    return;
  }

  StaticJsonDocument<384> doc;
  String error;
  if (!parseJsonBody(doc, error)) {
    sendJsonError(400, error);
    return;
  }

  bool hasChannel = doc.containsKey("channel");
  bool hasVpdStage = doc.containsKey("vpd_stage");
  bool hasScale = doc.containsKey("ppfd_scale");
  if (!hasChannel && !hasVpdStage && !hasScale) {
    sendJsonError(400, "missing_fields");
    return;
  }

  if (hasChannel) {
    String nextChannel = doc["channel"].as<String>();
    if (!isValidLightChannelName(nextChannel)) {
      sendJsonError(400, "invalid_channel");
      return;
    }
    channel = lightChannelFromString(nextChannel);
  }
  if (hasVpdStage) {
    String nextStage = doc["vpd_stage"].as<String>();
    if (!isValidVpdStageId(nextStage)) {
      sendJsonError(400, "invalid_vpd_stage");
      return;
    }
    vpdStageId = nextStage;
  }
  if (hasScale) {
    float nextScale = doc["ppfd_scale"].as<float>();
    if (!isfinite(nextScale) || nextScale <= 0.1f || nextScale > 5.0f) {
      sendJsonError(400, "invalid_ppfd_scale");
      return;
    }
    ppfdScale = nextScale;
  }

  phaseId = phaseFromState();
  prefsSystem.begin("system", false);
  prefsSystem.putString("channel", lightChannelName());
  prefsSystem.putString("vpd_stage", vpdStageId);
  prefsSystem.putFloat("ppfd_scale", ppfdScale);
  prefsSystem.putString("phase", phaseId);
  prefsSystem.end();
  sendJsonAck(true);
}

void handlePhase() {
  if (!enforceAuth())
    return;
  if (server.method() == HTTP_GET) {
    const VpdProfile &profile = vpdProfileById(vpdStageId);
    String json = "{";
    json += "\"phase\":\"" + phaseId + "\",";
    json += "\"channel\":\"" + lightChannelName() + "\",";
    json += "\"ppfd_scale\":" + String(ppfdScale, 4) + ",";
    json += "\"ppfd_factor\":" + String(currentPpfdFactor(), 6) + ",";
    json += "\"vpd_stage\":\"" + vpdStageId + "\",";
    json += "\"vpd_target_low\":" + String(profile.targetLow, 2) + ",";
    json += "\"vpd_target_high\":" + String(profile.targetHigh, 2) + ",";
    json += "\"available\":[";
    for (size_t i = 0; i < PHASE_PRESET_COUNT; i++) {
      if (i > 0) json += ",";
      json += "{\"id\":\"" + String(PHASE_PRESETS[i].id) + "\",\"label\":\"" + String(PHASE_PRESETS[i].label) + "\"}";
    }
    json += "]}";
    server.send(200, "application/json", json);
    return;
  }

  StaticJsonDocument<256> doc;
  String error;
  if (!parseJsonBody(doc, error)) {
    sendJsonError(400, error);
    return;
  }

  String nextPhase;
  if (doc.containsKey("phase")) {
    nextPhase = doc["phase"].as<String>();
  } else if (doc.containsKey("id")) {
    nextPhase = doc["id"].as<String>();
  }

  if (nextPhase.length() == 0) {
    sendJsonError(400, "missing_phase");
    return;
  }

  const PhasePreset *preset = phasePresetById(nextPhase);
  if (!preset) {
    sendJsonError(400, "invalid_phase");
    return;
  }

  applyPhasePreset(*preset, true);
  sendJsonAck(true);
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

  bool hasChannel = false;
  bool hasVpdStage = false;
  bool hasTimezone = false;
  String channelValue;
  String vpdStageValue;
  String timezoneValue;
  bool parsedJson = server.hasArg("plain") && server.arg("plain").length() > 0;
  if (parsedJson) {
    StaticJsonDocument<384> doc;
    String error;
    if (!parseJsonBody(doc, error)) {
      sendJsonError(400, error);
      return;
    }
    if (doc.containsKey("channel")) {
      hasChannel = true;
      channelValue = doc["channel"].as<String>();
    }
    if (doc.containsKey("vpd_stage")) {
      hasVpdStage = true;
      vpdStageValue = doc["vpd_stage"].as<String>();
    }
    if (doc.containsKey("timezone")) {
      hasTimezone = true;
      timezoneValue = doc["timezone"].as<String>();
    }
  } else {
    hasChannel = server.hasArg("channel");
    hasVpdStage = server.hasArg("vpd_stage");
    hasTimezone = server.hasArg("timezone");
    if (hasChannel) channelValue = server.arg("channel");
    if (hasVpdStage) vpdStageValue = server.arg("vpd_stage");
    if (hasTimezone) timezoneValue = server.arg("timezone");
  }

  if (!hasChannel && !hasVpdStage && !hasTimezone) {
    sendJsonError(400, "missing_fields");
    return;
  }
  if (hasChannel) {
    if (!isValidLightChannelName(channelValue)) {
      sendJsonError(400, "invalid_channel");
      return;
    }
    channel = lightChannelFromString(channelValue);
  }
  if (hasVpdStage) {
    if (!isValidVpdStageId(vpdStageValue)) {
      sendJsonError(400, "invalid_vpd_stage");
      return;
    }
    vpdStageId = vpdStageValue;
  }
  if (hasTimezone) {
    timezoneName = timezoneValue;
    applyTimezoneEnv();
    timeSynced = false; // force refresh
    lastTimeSyncAttempt = 0;
    startNtpSync();
  }
  phaseId = phaseFromState();
  prefsSystem.begin("system", false);
  if (hasChannel) prefsSystem.putString("channel", lightChannelName());
  if (hasVpdStage) prefsSystem.putString("vpd_stage", vpdStageId);
  if (hasTimezone) prefsSystem.putString("timezone", timezoneName);
  if (hasChannel || hasVpdStage) prefsSystem.putString("phase", phaseId);
  prefsSystem.end();
  sendJsonAck(true);
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
  server.on("/api/factors", HTTP_GET, handleFactors);
  server.on("/api/factors", HTTP_POST, handleFactors);
  server.on("/api/phase", HTTP_GET, handlePhase);
  server.on("/api/phase", HTTP_POST, handlePhase);
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
  static uint32_t lastLoopMicros = 0;
  uint32_t loopNow = micros();
  if (lastLoopMicros != 0) {
    lastLoopDurationMs = (loopNow - lastLoopMicros) / 1000UL;
  }
  lastLoopMicros = loopNow;
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
  enqueueDebugLogBlock();
  processCloudQueue();

  if (millis() - lastSensorMillis >= SENSOR_INTERVAL) {
    lastSensorMillis = millis();
    readSensors();
  }
}
