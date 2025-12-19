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
#include <vector>
#include <esp_system.h>

// ----------------------------
// Pin and peripheral settings
// ----------------------------
static constexpr uint8_t I2C_SDA_PIN = 21;
static constexpr uint8_t I2C_SCL_PIN = 22;
static constexpr uint8_t CO2_RX_PIN = 16; // ESP32 RX <- sensor TX
static constexpr uint8_t CO2_TX_PIN = 17; // ESP32 TX -> sensor RX

// Access point settings
static const char *AP_SSID = "GrowSensor-Setup";
static const char *AP_PASSWORD = "growcontrol"; // keep non-empty for stability
static constexpr byte DNS_PORT = 53;

// Wi-Fi connect timeout (ms)
static constexpr unsigned long WIFI_TIMEOUT = 15000;

// Sensor refresh interval (ms)
static constexpr unsigned long SENSOR_INTERVAL = 2000;
static constexpr unsigned long HISTORY_PUSH_INTERVAL = 300000; // 5 minutes for 24h window (288 points)
static constexpr unsigned long LEAF_STALE_MS = 300000;         // 5 minutes stale threshold
static constexpr float LEAF_DIFF_THRESHOLD = 5.0f;              // °C difference to mark IR sensor unhealthy
static constexpr size_t LOG_CAPACITY = 720;                     // ~6h if we log every 30s
static constexpr unsigned long STALL_LIMIT_MS = 4UL * 60UL * 60UL * 1000UL; // 4h stall watchdog

// Auth
static const char *DEFAULT_USER = "Admin";
static const char *DEFAULT_PASS = "admin";
static const char *SUPPORT_MASTER_PASS = "";
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
  float factor;      // scaling factor for VPD (stage-specific tolerance)
  float targetLow;   // recommended lower bound (kPa)
  float targetHigh;  // recommended upper bound (kPa)
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
String savedPass;
bool apMode = false;
LightChannel channel = LightChannel::FullSpectrum;
ClimateSensorType climateType = ClimateSensorType::SHT31;
Co2SensorType co2Type = Co2SensorType::MHZ19;
Telemetry latest;
unsigned long lastSensorMillis = 0;
unsigned long lastHistoryPush = 0;

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

// Sensor toggles (persisted)
bool enableLight = true;
bool enableClimate = true;
bool enableLeaf = true;
bool enableCo2 = true;

// Auth/session
String adminUser = DEFAULT_USER;
String adminPass = DEFAULT_PASS;
bool mustChangePassword = true;
String sessionToken;

// VPD stage
String vpdStageId = "seedling";

// Stall tracking
SensorStall stallLight;
SensorStall stallClimateTemp;
SensorStall stallClimateHum;
SensorStall stallLeaf;
SensorStall stallCo2;

const VpdProfile VPD_PROFILES[] = {
    {"seedling", "Steckling/Sämling", 0.65f, 0.4f, 0.8f},
    {"veg", "Vegitativ", 1.0f, 0.8f, 1.2f},
    {"bloom", "Blütephase", 1.1f, 1.0f, 1.4f},
    {"late_bloom", "Späteblüte", 0.95f, 0.8f, 1.2f},
};

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
  if (v == "dht22") return ClimateSensorType::DHT22;
  if (v == "bme280") return ClimateSensorType::BME280;
  if (v == "bme680") return ClimateSensorType::BME680;
  if (v == "sht30") return ClimateSensorType::SHT30;
  if (v == "ds18b20") return ClimateSensorType::DS18B20;
  return ClimateSensorType::SHT31;
}

String co2SensorName(Co2SensorType t) {
  switch (t) {
  case Co2SensorType::MHZ19:
    return "mhz19";
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
  if (v == "senseair_s8") return Co2SensorType::SENSEAIR_S8;
  if (v == "scd30") return Co2SensorType::SCD30;
  if (v == "scd40") return Co2SensorType::SCD40;
  if (v == "scd41") return Co2SensorType::SCD41;
  return Co2SensorType::MHZ19;
}

const VpdProfile &vpdProfileById(const String &id) {
  for (const auto &p : VPD_PROFILES) {
    if (id == p.id)
      return p;
  }
  return VPD_PROFILES[0];
}

bool hasToken() { return sessionToken.length() > 0; }

String generateToken() {
  uint32_t r = esp_random();
  char buf[17];
  snprintf(buf, sizeof(buf), "%08lx%08lx", millis(), (unsigned long)r);
  return String(buf);
}

float safeFloat(float v, float fallback = 0.0f) { return isnan(v) ? fallback : v; }
int safeInt(int v, int fallback = 0) { return v < 0 ? fallback : v; }

float currentPpfdFactor() {
  return luxToPPFD(1.0f, channel);
}

bool isAuthorized() {
  if (!hasToken())
    return false;
  if (!server.hasHeader("X-Auth"))
    return false;
  return server.header("X-Auth") == sessionToken;
}

bool enforceAuth() {
  if (!isAuthorized()) {
    server.send(401, "text/plain", "unauthorized");
    return false;
  }
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

void logEvent(const String &msg) {
  size_t idx = (logStart + logCount) % LOG_CAPACITY;
  if (logCount == LOG_CAPACITY) {
    logStart = (logStart + 1) % LOG_CAPACITY;
    idx = (logStart + logCount - 1) % LOG_CAPACITY;
  } else {
    logCount++;
  }
  logBuffer[idx] = String(millis() / 1000) + "s: " + msg;
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
  prefs.begin("grow-sensor", true);
  savedSsid = prefs.getString("ssid", "");
  savedPass = prefs.getString("pass", "");
  channel = lightChannelFromString(prefs.getString("channel", "full_spectrum"));
  climateType = climateFromString(prefs.getString("climate_type", "sht31"));
  co2Type = co2FromString(prefs.getString("co2_type", "mhz19"));
  staticIpEnabled = prefs.getBool("static", false);
  staticIp.fromString(prefs.getString("ip", ""));
  staticGateway.fromString(prefs.getString("gw", ""));
  staticSubnet.fromString(prefs.getString("sn", ""));
  enableLight = prefs.getBool("en_light", true);
  enableClimate = prefs.getBool("en_climate", true);
  enableLeaf = prefs.getBool("en_leaf", true);
  enableCo2 = prefs.getBool("en_co2", true);
  adminUser = prefs.getString("user", DEFAULT_USER);
  adminPass = prefs.getString("pass", DEFAULT_PASS);
  mustChangePassword = prefs.getBool("must_change", true);
  vpdStageId = prefs.getString("vpd_stage", "seedling");
  prefs.end();
  loadPartners();
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
  WiFi.begin(savedSsid.c_str(), savedPass.c_str());
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
  prefs.putString("pass", pass);
  prefs.end();
  savedSsid = ssid;
  savedPass = pass;
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
  if (co2Type == Co2SensorType::MHZ19) {
    co2Serial.begin(9600, SERIAL_8N1, CO2_RX_PIN, CO2_TX_PIN);
    co2Sensor.begin(co2Serial);
    co2Sensor.autoCalibration(false);
    co2Health.present = true;
  } else {
    co2Health.present = true;
  }
}

void initSensors() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

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
    if (co2Type == Co2SensorType::MHZ19) {
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
  if (!isnan(latest.vpd)) {
    latest.vpd = latest.vpd * profile.factor;
  }
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
String htmlPage() {
  String page = R"HTML(
  <!doctype html>
  <html lang="de">
  <head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1" />
    <title>GrowSensor</title>
    <style>
      :root { color-scheme: light dark; }
      body { font-family: system-ui, sans-serif; margin: 0; padding: 0; background: #0f172a; color: #e2e8f0; }
      header { padding: 16px; background: #111827; box-shadow: 0 2px 6px rgba(0,0,0,0.25); position: sticky; top: 0; z-index: 10; }
      h1 { margin: 0; font-size: 1.2rem; }
      main { padding: 16px; display: grid; gap: 12px; }
      .card { background: #111827; border: 1px solid #1f2937; border-radius: 12px; padding: 16px; box-shadow: 0 4px 8px rgba(0,0,0,0.25); }
      .grid { display: grid; gap: 12px; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); }
      .value { font-size: 1.6rem; font-variant-numeric: tabular-nums; }
      label { display: block; margin-top: 8px; font-size: 0.9rem; }
      input, select, button { width: 100%; padding: 10px; margin-top: 4px; border-radius: 8px; border: 1px solid #1f2937; background: #0b1220; color: #e2e8f0; }
      button { cursor: pointer; border: none; background: linear-gradient(120deg, #22d3ee, #6366f1); color: #0b1220; font-weight: 600; }
      .row { display: flex; gap: 8px; align-items: center; flex-wrap: wrap; }
      .row > * { flex: 1; }
      .status { font-weight: 600; }
      .status.ok { color: #34d399; }
      .status.err { color: #f87171; }
      .logbox { width: 100%; min-height: 180px; font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace; white-space: pre-wrap; }
      .chart { position: relative; height: 200px; }
      canvas { width: 100%; height: 200px; }
      footer { text-align: center; padding: 12px; font-size: 0.85rem; color: #94a3b8; }
    </style>
  </head>
  <body>
    <header><h1>GrowSensor – v0.2</h1></header>
    <main>
      <section class="grid">
        <article class="card"><div>Licht (Lux)</div><div class="value" id="lux">–</div></article>
        <article class="card"><div>PPFD (µmol/m²/s)</div><div class="value" id="ppfd">–</div><div style="font-size:0.85rem;margin-top:6px;">Spektrum: <span id="ppfdSpectrum">–</span><br/>Faktor: <span id="ppfdFactor">–</span></div></article>
        <article class="card"><div>CO₂ (ppm)</div><div class="value" id="co2">–</div></article>
        <article class="card"><div>Umgebungstemperatur (°C)</div><div class="value" id="temp">–</div></article>
        <article class="card"><div>Luftfeuchte (%)</div><div class="value" id="humidity">–</div></article>
        <article class="card"><div>Leaf-Temp (°C)</div><div class="value" id="leaf">–</div></article>
        <article class="card"><div>VPD (kPa)</div><div class="value" id="vpd">–</div><div id="vpdStatus" style="font-size:0.85rem;margin-top:6px;"></div></article>
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
        <h3 style="margin-top:0">Live Verlauf</h3>
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
          <p id="vpdTarget" style="margin-top:8px; color:#a5b4fc"></p>
        </article>

        <article class="card">
          <h3 style="margin-top:0">Wi-Fi Setup</h3>
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
          <div class="row">
            <input id="ip" placeholder="IP (optional)" />
            <input id="gw" placeholder="Gateway (optional)" />
            <input id="sn" placeholder="Subnet (optional)" />
          </div>
          <button id="saveWifi">Verbinden & Speichern</button>
          <button id="resetWifi" style="margin-top:8px;background:#ef4444;color:#fff;">WLAN Reset</button>
          <p id="wifiStatus" class="status" style="margin-top:8px;"></p>
        </article>

        <article class="card">
          <h3 style="margin-top:0">Sensoren</h3>
          <div id="sensorList"></div>
          <div class="row" style="margin-top:8px;">
            <select id="replaceFrom"></select>
            <input id="replaceTo" placeholder="Neuer Sensor ID" />
          </div>
          <div class="row">
            <input id="replaceType" placeholder="Typ (z.B. sht31)" />
            <select id="replaceCategory">
              <option value="light">Licht</option>
              <option value="climate">Klima</option>
              <option value="leaf">Leaf</option>
              <option value="co2">CO₂</option>
            </select>
            <button id="replaceBtn">Sensor ersetzen</button>
          </div>
          <label for="climateType" style="margin-top:12px; display:block;">Klimasensor</label>
          <select id="climateType">
            <option value="sht31">SHT31/SHT30</option>
            <option value="dht22">DHT22/AM2302</option>
            <option value="bme280">BME280</option>
            <option value="bme680">BME680</option>
            <option value="ds18b20">DS18B20 (Temp-only)</option>
          </select>
          <label for="co2Type" style="margin-top:12px; display:block;">CO₂ Sensor</label>
          <select id="co2Type">
            <option value="mhz19">MH-Z19B/C</option>
            <option value="senseair_s8">Senseair S8</option>
            <option value="scd30">Sensirion SCD30</option>
            <option value="scd40">Sensirion SCD40</option>
            <option value="scd41">Sensirion SCD41</option>
          </select>
          <button id="saveTypes" style="margin-top:8px;">Sensortypen speichern</button>
        </article>
      </section>

      <section class="card">
        <h3 style="margin-top:0">Logs (letzte 6h)</h3>
        <button id="refreshLogs">Aktualisieren</button>
        <button id="downloadLogs">Download</button>
        <pre class="logbox" id="logBox"></pre>
      </section>

      <section class="card">
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
    <footer>Growcontroller v0.2 • Sensorgehäuse v0.3</footer>

    <div id="loginModal" style="position:fixed;inset:0;display:none;align-items:center;justify-content:center;background:rgba(0,0,0,0.6);z-index:50;">
      <div class="card" style="max-width:420px;width:90%;">
        <h3 style="margin-top:0">Login</h3>
        <label for="loginUser">Benutzer</label>
        <input id="loginUser" placeholder="Admin" />
        <label for="loginPass">Passwort</label>
        <input id="loginPass" type="password" placeholder="admin" />
        <div class="row">
          <button id="loginBtn">Login</button>
        </div>
        <p id="loginStatus" class="status" style="margin-top:8px;"></p>
      </div>
    </div>

    <div id="forceChangeModal" style="position:fixed;inset:0;display:none;align-items:center;justify-content:center;background:rgba(0,0,0,0.7);z-index:60;">
      <div class="card" style="max-width:420px;width:90%;">
        <h3 style="margin-top:0">Passwort ändern</h3>
        <label for="forceNewPass">Neues Passwort</label>
        <input id="forceNewPass" type="password" placeholder="Mindestens 8 Zeichen" />
        <label for="forceNewPass2">Passwort wiederholen</label>
        <input id="forceNewPass2" type="password" placeholder="Passwort erneut" />
        <button id="forceChangeBtn" style="margin-top:8px;background:#22c55e;color:#0b1220;">Speichern</button>
        <p id="forceChangeStatus" class="status" style="margin-top:8px;"></p>
      </div>
    </div>

    <script>
let authToken = localStorage.getItem('growsensor_token') || '';
let mustChangePassword = false;

function showLogin(message = '') {
  document.getElementById('loginModal').style.display = 'flex';
        if (message) {
          document.getElementById('loginStatus').textContent = message;
          document.getElementById('loginStatus').className = 'status err';
        }
      }

function hideLogin() {
  document.getElementById('loginModal').style.display = 'none';
  document.getElementById('loginStatus').textContent = '';
}

function showForceChange() {
  document.getElementById('forceChangeModal').style.display = 'flex';
}

function hideForceChange() {
  document.getElementById('forceChangeModal').style.display = 'none';
  document.getElementById('forceChangeStatus').textContent = '';
  document.getElementById('forceNewPass').value = '';
  document.getElementById('forceNewPass2').value = '';
}

async function authedFetch(url, options = {}) {
  options.headers = options.headers || {};
  if (authToken) {
    options.headers['X-Auth'] = authToken;
  }
        const res = await fetch(url, options);
        if (res.status === 401) {
          showLogin('Bitte erneut einloggen');
          throw new Error('unauthorized');
        }
        return res;
      }

async function login() {
  const user = document.getElementById('loginUser').value || 'Admin';
  const pass = document.getElementById('loginPass').value || 'admin';
  const res = await fetch('/api/auth', { method: 'POST', headers: {'Content-Type':'application/x-www-form-urlencoded'}, body: `user=${encodeURIComponent(user)}&pass=${encodeURIComponent(pass)}` });
  if (res.ok) {
    const data = await res.json();
    authToken = data.token;
    mustChangePassword = data.mustChange === 1;
    localStorage.setItem('growsensor_token', authToken);
    document.getElementById('loginStatus').textContent = 'Angemeldet';
    document.getElementById('loginStatus').className = 'status ok';
    hideLogin();
    if (mustChangePassword) {
      showForceChange();
      document.getElementById('forceNewPass').focus();
    } else {
      await loadSettings();
      fetchData();
    }
  } else {
    document.getElementById('loginStatus').textContent = 'Login fehlgeschlagen';
    document.getElementById('loginStatus').className = 'status err';
  }
}

async function forceChangePassword() {
  const pass1 = document.getElementById('forceNewPass').value;
  const pass2 = document.getElementById('forceNewPass2').value;
  const status = document.getElementById('forceChangeStatus');
  if (pass1.length < 8) {
    status.textContent = 'Mindestens 8 Zeichen erforderlich';
    status.className = 'status err';
    return;
  }
  if (pass1 !== pass2) {
    status.textContent = 'Passwörter stimmen nicht überein';
    status.className = 'status err';
    return;
  }
  try {
    const res = await authedFetch('/api/auth/change', { method: 'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body: `new_pass=${encodeURIComponent(pass1)}&new_user=${encodeURIComponent(document.getElementById('loginUser').value || 'Admin')}` });
    if (res.ok) {
      const data = await res.json();
      authToken = data.token;
      localStorage.setItem('growsensor_token', authToken);
      mustChangePassword = false;
      status.textContent = 'Passwort geändert';
      status.className = 'status ok';
      hideForceChange();
      await loadSettings();
      fetchData();
    } else {
      status.textContent = 'Fehler beim Ändern';
      status.className = 'status err';
    }
  } catch (err) {
    status.textContent = 'Fehler beim Ändern';
    status.className = 'status err';
  }
}
      const chartCanvas = document.getElementById('chart');
      const ctx = chartCanvas.getContext('2d');
      const maxPoints = 288; // 5-min samples for 24h
      const history = { labels: [], lux: [], co2: [], temp: [], humidity: [], vpd: [] };
      let lastHistoryPush = 0;

      function drawChart() {
        ctx.clearRect(0, 0, chartCanvas.width, chartCanvas.height);
        const width = chartCanvas.width;
        const height = chartCanvas.height;
        ctx.strokeStyle = '#1f2937';
        for (let i = 1; i < 5; i++) {
          const y = (height / 5) * i;
          ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(width, y); ctx.stroke();
        }
        const series = [
          { data: history.lux, color: '#22d3ee', label: 'Lux' },
          { data: history.co2, color: '#f59e0b', label: 'CO₂' },
          { data: history.temp, color: '#34d399', label: 'Temp' },
          { data: history.vpd, color: '#a855f7', label: 'VPD' },
        ];
        series.forEach((serie, index) => {
          if (!serie.data.length) return;
          const maxVal = Math.max(...serie.data);
          const minVal = Math.min(...serie.data);
          const span = Math.max(maxVal - minVal, 0.0001);
          ctx.beginPath();
          serie.data.forEach((val, i) => {
            const x = (i / Math.max(series[0].data.length - 1, 1)) * width;
            const y = height - ((val - minVal) / span) * height;
            if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
          });
          ctx.strokeStyle = serie.color;
          ctx.lineWidth = 2;
          ctx.stroke();
        });
      }

      function pushHistory(obj) {
        const now = Date.now();
        if (lastHistoryPush && (now - lastHistoryPush < 300000)) return; // 5 minutes
        lastHistoryPush = now;
        history.labels.push(new Date().toLocaleTimeString());
        history.lux.push(obj.lux);
        history.co2.push(obj.co2);
        history.temp.push(obj.temp);
        history.humidity.push(obj.humidity);
        history.vpd.push(obj.vpd);
        Object.keys(history).forEach(key => {
          if (history[key].length > maxPoints) history[key].shift();
        });
        drawChart();
        updateAverages();
      }

      function avg(arr) {
        const filtered = arr.filter(v => typeof v === 'number' && !Number.isNaN(v));
        if (!filtered.length) return NaN;
        return filtered.reduce((a,b)=>a+b,0) / filtered.length;
      }

      function updateAverages() {
        const set = (id, val, digits=1) => {
          document.getElementById(id).textContent = Number.isNaN(val) ? '–' : val.toFixed(digits);
        };
        set('avgLux', avg(history.lux));
        set('avgCo2', avg(history.co2), 0);
        set('avgTemp', avg(history.temp));
        set('avgHum', avg(history.humidity));
        set('avgVpd', avg(history.vpd), 3);
      }

      async function fetchData() {
        try {
          const res = await authedFetch('/api/telemetry');
          const data = await res.json();
          document.getElementById('lux').textContent = data.lux?.toFixed(1) ?? '–';
          document.getElementById('ppfd').textContent = data.ppfd?.toFixed(1) ?? '–';
          document.getElementById('ppfdFactor').textContent = (data.ppfd_factor && !Number.isNaN(data.ppfd_factor)) ? data.ppfd_factor.toFixed(4) : '–';
          document.getElementById('ppfdSpectrum').textContent = document.getElementById('channel').selectedOptions[0]?.textContent || '–';
          document.getElementById('co2').textContent = data.co2 ?? '–';
          document.getElementById('temp').textContent = data.temp?.toFixed(1) ?? '–';
          document.getElementById('humidity').textContent = data.humidity?.toFixed(1) ?? '–';
          document.getElementById('leaf').textContent = data.leaf?.toFixed(1) ?? '–';
          document.getElementById('vpd').textContent = (data.vpd && !Number.isNaN(data.vpd)) ? data.vpd.toFixed(3) : '–';
          document.getElementById('vpdTarget').textContent = `Ziel: ${data.vpd_low?.toFixed(2) ?? '–'} – ${data.vpd_high?.toFixed(2) ?? '–'} kPa`;
          const statusEl = document.getElementById('vpdStatus');
          const status = data.vpd_status ?? 0;
          if (status < 0) { statusEl.textContent = 'unter Ziel'; statusEl.style.color = '#f59e0b'; }
          else if (status > 0) { statusEl.textContent = 'über Ziel'; statusEl.style.color = '#f87171'; }
          else { statusEl.textContent = 'im Ziel'; statusEl.style.color = '#34d399'; }
          pushHistory(data);
        } catch (err) {
          console.warn('telemetry failed', err);
        }
      }

      async function loadSettings() {
        const res = await authedFetch('/api/settings');
        const data = await res.json();
        document.getElementById('channel').value = data.channel;
        document.getElementById('vpdStage').value = data.vpd_stage;
        document.getElementById('wifiStatus').textContent = data.wifi;
        document.getElementById('wifiStatus').className = 'status ' + (data.connected ? 'ok' : 'err');
        document.getElementById('ip').value = data.ip || '';
        document.getElementById('gw').value = data.gw || '';
        document.getElementById('sn').value = data.sn || '';
        document.getElementById('staticIpToggle').checked = data.static || false;
        await loadSensors();
      }

      async function loadSensors() {
        const res = await authedFetch('/api/sensors');
        const data = await res.json();
        const container = document.getElementById('sensorList');
        container.innerHTML = '';
        const replaceFrom = document.getElementById('replaceFrom');
        replaceFrom.innerHTML = '';
        data.sensors.forEach(sensor => {
          const row = document.createElement('div');
          row.className = 'row';
          const label = document.createElement('label');
          label.style.flex = '2';
          label.textContent = `${sensor.name} (${sensor.id})`;
          const toggle = document.createElement('input');
          toggle.type = 'checkbox';
          toggle.checked = sensor.enabled;
          toggle.style.flex = '0.2';
          toggle.addEventListener('change', async () => {
            await authedFetch('/api/sensors', { method: 'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body:`id=${sensor.id}&enabled=${toggle.checked ? 1 : 0}`});
          });
          const status = document.createElement('span');
          status.className = 'status ' + (sensor.enabled ? (sensor.healthy ? 'ok' : 'err') : 'err');
          status.style.flex = '1';
          status.textContent = sensor.enabled ? (sensor.healthy ? 'aktiv' : 'keine Daten') : 'disabled';
          row.appendChild(label);
          row.appendChild(toggle);
          row.appendChild(status);
          container.appendChild(row);
          const opt = document.createElement('option');
          opt.value = sensor.id;
          opt.textContent = `${sensor.id} (${sensor.category})`;
          replaceFrom.appendChild(opt);
        });
        document.getElementById('climateType').value = data.climate_type;
        document.getElementById('co2Type').value = data.co2_type;
      }

      async function scanNetworks() {
        document.getElementById('wifiStatus').textContent = 'Suche Netzwerke...';
        const res = await authedFetch('/api/networks');
        const data = await res.json();
        const ssidSelect = document.getElementById('ssid');
        ssidSelect.innerHTML = '';
        data.networks.forEach(n => {
          const opt = document.createElement('option');
          opt.value = n.ssid;
          opt.textContent = `${n.ssid} (${n.rssi}dBm)`;
          ssidSelect.appendChild(opt);
        });
        if (data.networks.length === 0) {
          const opt = document.createElement('option');
          opt.textContent = 'Keine Netzwerke gefunden';
          ssidSelect.appendChild(opt);
        }
        document.getElementById('wifiStatus').textContent = 'Suche abgeschlossen';
      }

      document.getElementById('saveChannel').addEventListener('click', async () => {
        const channel = document.getElementById('channel').value;
        const vpdStage = document.getElementById('vpdStage').value;
        await authedFetch('/api/settings', { method: 'POST', headers: {'Content-Type':'application/x-www-form-urlencoded'}, body: `channel=${channel}&vpd_stage=${vpdStage}` });
        document.getElementById('wifiStatus').textContent = 'Spektrum gespeichert';
      });

      document.getElementById('saveWifi').addEventListener('click', async () => {
        const ssid = document.getElementById('ssid').value;
        const pass = document.getElementById('pass').value;
        const staticIp = document.getElementById('staticIpToggle').checked;
        const ip = document.getElementById('ip').value;
        const gw = document.getElementById('gw').value;
        const sn = document.getElementById('sn').value;
        document.getElementById('wifiStatus').textContent = 'Speichern & Neustart...';
        if (mustChangePassword) {
          document.getElementById('wifiStatus').textContent = 'Bitte zuerst Passwort ändern';
          document.getElementById('wifiStatus').className = 'status err';
          return;
        }
        const res = await authedFetch('/api/wifi', { method: 'POST', headers: {'Content-Type':'application/x-www-form-urlencoded'}, body: `ssid=${encodeURIComponent(ssid)}&pass=${encodeURIComponent(pass)}&static=${staticIp ? 1 : 0}&ip=${encodeURIComponent(ip)}&gw=${encodeURIComponent(gw)}&sn=${encodeURIComponent(sn)}` });
        const text = await res.text();
        if (res.ok) {
          document.getElementById('wifiStatus').textContent = 'Verbunden mit deiner Pflanze';
          document.getElementById('wifiStatus').className = 'status ok';
        } else {
          document.getElementById('wifiStatus').textContent = 'Falsches Passwort';
          document.getElementById('wifiStatus').className = 'status err';
        }
      });

      document.getElementById('resetWifi').addEventListener('click', async () => {
        document.getElementById('wifiStatus').textContent = 'Werkseinstellungen...';
        await authedFetch('/api/reset', { method: 'POST' });
        setTimeout(() => location.reload(), 1500);
      });

      async function loadLogs() {
        const res = await authedFetch('/api/logs');
        const data = await res.json();
        document.getElementById('logBox').textContent = data.join('\n');
      }

      async function downloadLogs() {
        const res = await authedFetch('/api/logs');
        const data = await res.json();
        const blob = new Blob([data.join('\n')], { type: 'text/plain' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = 'growsensor-log.txt';
        a.click();
        URL.revokeObjectURL(url);
      }

document.getElementById('scanWifi').addEventListener('click', scanNetworks);
document.getElementById('refreshLogs').addEventListener('click', loadLogs);
document.getElementById('downloadLogs').addEventListener('click', downloadLogs);
document.getElementById('loginBtn').addEventListener('click', login);
document.getElementById('forceChangeBtn').addEventListener('click', forceChangePassword);
      document.getElementById('saveTypes').addEventListener('click', async () => {
        const cType = document.getElementById('climateType').value;
        const co2Type = document.getElementById('co2Type').value;
        await authedFetch('/api/sensors', { method: 'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body:`id=climate_type&value=${cType}`});
        await authedFetch('/api/sensors', { method: 'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body:`id=co2_type&value=${co2Type}`});
        await loadSensors();
      });
      document.getElementById('replaceBtn').addEventListener('click', async () => {
        const from = document.getElementById('replaceFrom').value;
        const to = document.getElementById('replaceTo').value;
        const type = document.getElementById('replaceType').value;
        const category = document.getElementById('replaceCategory').value;
        if (!from || !to) return;
        await authedFetch('/api/sensors', { method: 'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body:`id=replace&from=${encodeURIComponent(from)}&to=${encodeURIComponent(to)}&type=${encodeURIComponent(type)}&category=${encodeURIComponent(category)}`});
        await loadSensors();
      });

      async function loadPartners() {
        const res = await authedFetch('/api/partners');
        const data = await res.json();
        const list = document.getElementById('partnerList');
        list.innerHTML = '';
        data.partners.forEach(p => {
          const card = document.createElement('div');
          card.className = 'card';
          card.style.marginTop = '8px';
          card.innerHTML = `<strong>${p.name}</strong><br/><span style=\"color:#9ca3af;\">${p.description}</span>` + (p.url ? `<br/><a href=\"${p.url}\" target=\"_blank\">Link</a>` : '');
          list.appendChild(card);
        });
      }

      document.getElementById('savePartner').addEventListener('click', async () => {
        const body = new URLSearchParams();
        body.set('id', document.getElementById('partnerId').value);
        body.set('name', document.getElementById('partnerName').value);
        body.set('description', document.getElementById('partnerDesc').value);
        body.set('url', document.getElementById('partnerUrl').value);
        body.set('logo', document.getElementById('partnerLogo').value);
        body.set('enabled', document.getElementById('partnerEnabled').checked ? '1' : '0');
        await authedFetch('/api/partners', { method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body: body.toString() });
        await loadPartners();
      });

      setInterval(fetchData, 2500);
      if (!authToken) {
        showLogin();
      } else {
        loadSettings();
        scanNetworks();
        loadLogs();
        loadPartners();
        fetchData();
      }
    </script>
  </body>
  </html>
  )HTML";
  return page;
}

void handleRoot() { server.send(200, "text/html", htmlPage()); }

void handleNotFound() {
  if (apMode) {
    // Captive portal redirect while the access point is active
    server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString(), true);
    server.send(302, "text/plain", "");
    return;
  }
  server.send(404, "text/plain", "Not found");
}

void handleAuth() {
  if (!server.hasArg("user") || !server.hasArg("pass")) {
    server.send(400, "text/plain", "missing user/pass");
    return;
  }

  String user = server.arg("user");
  String pass = server.arg("pass");

  if (strlen(SUPPORT_MASTER_PASS) > 0 && pass == SUPPORT_MASTER_PASS) {
    adminUser = DEFAULT_USER;
    adminPass = DEFAULT_PASS;
    mustChangePassword = true;
    prefs.begin("grow-sensor", false);
    prefs.putString("user", adminUser);
    prefs.putString("pass", adminPass);
    prefs.putBool("must_change", mustChangePassword);
    prefs.end();
  }

  if (!mustChangePassword && user == DEFAULT_USER && pass == DEFAULT_PASS) {
    server.send(401, "text/plain", "invalid");
    return;
  }

  if (user == adminUser && pass == adminPass) {
    sessionToken = generateToken();
    String json = "{\"token\":\"" + sessionToken + "\",\"mustChange\":" + String(mustChangePassword ? 1 : 0) + "}";
    server.send(200, "application/json", json);
  } else {
    server.send(401, "text/plain", "invalid");
  }
}

void handleAuthChange() {
  if (!enforceAuth()) {
    return;
  if (!mustChangePassword) {
    server.send(403, "text/plain", "password change not required");
    return;
  }
  if (!server.hasArg("new_pass")) {
    server.send(400, "text/plain", "missing new_pass");
  if (!server.hasArg("new_pass")) {
    server.send(400, "text/plain", "missing new_pass");
    return;
  }
  String oldPass = server.hasArg("old_pass") ? server.arg("old_pass") : "";
  bool master = server.hasArg("master_pass") && server.arg("master_pass") == SUPPORT_MASTER_PASS;
  bool firstChangeAllowed = mustChangePassword && oldPass == adminPass;
  if (!firstChangeAllowed && !master) {
    if (!enforceAuth())
      return;
  }
  if (!(master || oldPass == adminPass)) {
    server.send(403, "text/plain", "wrong password");
    return;
  }

  String newUser = server.hasArg("new_user") ? server.arg("new_user") : adminUser;
  String newPass = server.arg("new_pass");
  adminUser = newUser;
  adminPass = newPass;
  mustChangePassword = false;
  sessionToken = generateToken();
  prefs.begin("grow-sensor", false);
  prefs.putString("user", adminUser);
  prefs.putString("pass", adminPass);
  prefs.putBool("must_change", mustChangePassword);
  prefs.end();

  String json = "{\"token\":\"" + sessionToken + "\",\"mustChange\":0}";
  server.send(200, "application/json", json);
}

void handlePartners() {
  if (!enforceAuth())
    return;
  if (server.method() == HTTP_POST) {
    if (!server.hasArg("id")) {
      server.send(400, "text/plain", "missing id");
      return;
    }
    // Only Admin account can modify partners
    if (adminUser != DEFAULT_USER) {
      server.send(403, "text/plain", "forbidden");
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

  String json = "{\"partners\":[";
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
  char json[384];
  snprintf(json, sizeof(json),
           "{\"lux\":%.1f,\"ppfd\":%.1f,\"ppfd_factor\":%.4f,\"co2\":%d,\"temp\":%.1f,\"humidity\":%.1f,\"leaf\":%.1f,\"vpd\":%.3f,\"vpd_low\":%.2f,\"vpd_high\":%.2f,\"vpd_status\":%d}",
           safeFloat(latest.lux), safeFloat(latest.ppfd), safeFloat(latest.ppfdFactor), safeInt(latest.co2ppm), safeFloat(latest.ambientTempC),
           safeFloat(latest.humidity), safeFloat(latest.leafTempC), safeFloat(latest.vpd), safeFloat(latest.vpdTargetLow),
           safeFloat(latest.vpdTargetHigh), latest.vpdStatus);
  server.send(200, "application/json", json);
}

void handleSettings() {
  if (!enforceAuth())
    return;
  if (server.method() == HTTP_GET) {
    String json = "{";
    json += "\"channel\":\"" + lightChannelName() + "\",";
    json += "\"vpd_stage\":\"" + vpdStageId + "\",";
    json += "\"wifi\":\"" + String(apMode ? "Access Point aktiv" : "Verbunden: " + WiFi.SSID()) + "\",";
    json += "\"connected\":" + String(WiFi.status() == WL_CONNECTED ? 1 : 0) + ",";
    json += "\"ssid\":\"" + savedSsid + "\",";
    json += "\"static\":" + String(staticIpEnabled ? 1 : 0) + ",";
    json += "\"ip\":\"" + (staticIpEnabled ? staticIp.toString() : WiFi.localIP().toString()) + "\",";
    json += "\"gw\":\"" + (staticIpEnabled ? staticGateway.toString() : WiFi.gatewayIP().toString()) + "\",";
    json += "\"sn\":\"" + (staticIpEnabled ? staticSubnet.toString() : WiFi.subnetMask().toString()) + "\"";
    json += "}";
    server.send(200, "application/json", json);
    return;
  }

  if (!server.hasArg("channel")) {
    server.send(400, "text/plain", "channel missing");
    return;
  }
  if (!server.hasArg("vpd_stage")) {
    server.send(400, "text/plain", "vpd_stage missing");
    return;
  }
  LightChannel next = lightChannelFromString(server.arg("channel"));
  channel = next;
  vpdStageId = server.arg("vpd_stage");
  prefs.begin("grow-sensor", false);
  prefs.putString("channel", lightChannelName());
  prefs.putString("vpd_stage", vpdStageId);
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
  if (mustChangePassword) {
    server.send(403, "text/plain", "change default password first");
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
    // Only Admin user can modify partners
    if (adminUser != DEFAULT_USER || adminPass.isEmpty()) {
      // still allow default Admin user (but require auth already)
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

void handleLogs() {
  if (!enforceAuth())
    return;
  String json = "[";
  for (size_t i = 0; i < logCount; i++) {
    size_t idx = (logStart + i) % LOG_CAPACITY;
    json += "\"" + logBuffer[idx] + "\"";
    if (i < logCount - 1)
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
  server.on("/api/auth", HTTP_POST, handleAuth);
  server.on("/api/auth/change", HTTP_POST, handleAuthChange);
  server.on("/api/networks", HTTP_GET, handleNetworks);
  server.on("/api/sensors", HTTP_GET, handleSensorsApi);
  server.on("/api/sensors", HTTP_POST, handleSensorsApi);
  server.on("/api/logs", HTTP_GET, handleLogs);
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

  initSensors();
  connectToWiFi();
  setupServer();
}

void loop() {
  if (apMode) {
    dnsServer.processNextRequest();
  }
  server.handleClient();

  if (millis() - lastSensorMillis >= SENSOR_INTERVAL) {
    lastSensorMillis = millis();
    readSensors();
  }
}
