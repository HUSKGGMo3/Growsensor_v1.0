diff --git a/src/main.cpp b/src/main.cpp
new file mode 100644
index 0000000000000000000000000000000000000000..8da767e67cad75b13d8045aa2067a269fe6b3012
--- /dev/null
+++ b/src/main.cpp
@@ -0,0 +1,487 @@
+#include <Arduino.h>
+#include <WiFi.h>
+#include <WebServer.h>
+#include <DNSServer.h>
+#include <Preferences.h>
+#include <Wire.h>
+#include <BH1750.h>
+#include <Adafruit_SHT31.h>
+#include <Adafruit_MLX90614.h>
+#include <MHZ19.h>
+
+// ----------------------------
+// Pin and peripheral settings
+// ----------------------------
+static constexpr uint8_t I2C_SDA_PIN = 21;
+static constexpr uint8_t I2C_SCL_PIN = 22;
+static constexpr uint8_t CO2_RX_PIN = 16; // ESP32 RX <- sensor TX
+static constexpr uint8_t CO2_TX_PIN = 17; // ESP32 TX -> sensor RX
+
+// Access point settings
+static const char *AP_SSID = "GrowSensor-Setup";
+static const char *AP_PASSWORD = "growcontrol"; // keep non-empty for stability
+static constexpr byte DNS_PORT = 53;
+
+// Wi-Fi connect timeout (ms)
+static constexpr unsigned long WIFI_TIMEOUT = 15000;
+
+// Sensor refresh interval (ms)
+static constexpr unsigned long SENSOR_INTERVAL = 2000;
+
+// Lux to PPFD conversion factors (approximate for common horticulture spectra)
+enum class LightChannel {
+  FullSpectrum,
+  White,
+  Bloom
+};
+
+float luxToPPFD(float lux, LightChannel channel) {
+  // Factors derived from typical LED spectral profiles; adjust in UI if needed.
+  switch (channel) {
+  case LightChannel::FullSpectrum:
+    return lux * 0.014f; // full-spectrum horticulture LEDs
+  case LightChannel::White:
+    return lux * 0.0157f; // cool/neutral white channels
+  case LightChannel::Bloom:
+    return lux * 0.0175f; // red-heavy bloom channels
+  default:
+    return lux * 0.015f;
+  }
+}
+
+struct Telemetry {
+  float lux = NAN;
+  float ppfd = NAN;
+  float ambientTempC = NAN;
+  float humidity = NAN;
+  float leafTempC = NAN;
+  int co2ppm = -1;
+};
+
+// Globals
+BH1750 lightMeter;
+Adafruit_SHT31 sht31 = Adafruit_SHT31();
+Adafruit_MLX90614 mlx = Adafruit_MLX90614();
+HardwareSerial co2Serial(2);
+MHZ19 co2Sensor;
+Preferences prefs;
+DNSServer dnsServer;
+WebServer server(80);
+
+String savedSsid;
+String savedPass;
+bool apMode = false;
+LightChannel channel = LightChannel::FullSpectrum;
+Telemetry latest;
+unsigned long lastSensorMillis = 0;
+
+// ----------------------------
+// Helpers
+// ----------------------------
+String lightChannelName() {
+  switch (channel) {
+  case LightChannel::FullSpectrum:
+    return "full_spectrum";
+  case LightChannel::White:
+    return "white";
+  case LightChannel::Bloom:
+    return "bloom";
+  default:
+    return "full_spectrum";
+  }
+}
+
+LightChannel lightChannelFromString(const String &value) {
+  if (value == "white")
+    return LightChannel::White;
+  if (value == "bloom")
+    return LightChannel::Bloom;
+  return LightChannel::FullSpectrum;
+}
+
+// ----------------------------
+// Wi-Fi handling
+// ----------------------------
+void startAccessPoint() {
+  apMode = true;
+  WiFi.mode(WIFI_AP);
+  WiFi.softAP(AP_SSID, AP_PASSWORD);
+  delay(100); // give the AP time to start
+
+  IPAddress apIP = WiFi.softAPIP();
+  dnsServer.start(DNS_PORT, "*", apIP);
+  Serial.printf("[AP] Started %s (%s)\n", AP_SSID, apIP.toString().c_str());
+}
+
+bool connectToWiFi() {
+  prefs.begin("grow-sensor", true);
+  savedSsid = prefs.getString("ssid", "");
+  savedPass = prefs.getString("pass", "");
+  channel = lightChannelFromString(prefs.getString("channel", "full_spectrum"));
+  prefs.end();
+
+  if (savedSsid.isEmpty()) {
+    Serial.println("[WiFi] No stored credentials, starting AP");
+    startAccessPoint();
+    return false;
+  }
+
+  WiFi.mode(WIFI_STA);
+  WiFi.begin(savedSsid.c_str(), savedPass.c_str());
+  Serial.printf("[WiFi] Connecting to %s ...\n", savedSsid.c_str());
+
+  unsigned long start = millis();
+  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_TIMEOUT) {
+    delay(250);
+    Serial.print('.');
+  }
+  Serial.println();
+
+  if (WiFi.status() == WL_CONNECTED) {
+    apMode = false;
+    Serial.printf("[WiFi] Connected, IP: %s\n", WiFi.localIP().toString().c_str());
+    return true;
+  }
+
+  Serial.println("[WiFi] Connection failed, starting AP");
+  startAccessPoint();
+  return false;
+}
+
+void saveWifiCredentials(const String &ssid, const String &pass) {
+  prefs.begin("grow-sensor", false);
+  prefs.putString("ssid", ssid);
+  prefs.putString("pass", pass);
+  prefs.end();
+  savedSsid = ssid;
+  savedPass = pass;
+}
+
+void clearPreferences() {
+  prefs.begin("grow-sensor", false);
+  prefs.clear();
+  prefs.end();
+}
+
+// ----------------------------
+// Sensor handling
+// ----------------------------
+void initSensors() {
+  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
+
+  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire)) {
+    Serial.println("[Sensor] BH1750 initialized");
+  } else {
+    Serial.println("[Sensor] BH1750 not detected");
+  }
+
+  if (sht31.begin(0x44)) {
+    Serial.println("[Sensor] SHT31 initialized");
+  } else {
+    Serial.println("[Sensor] SHT31 not detected");
+  }
+
+  if (mlx.begin()) {
+    Serial.println("[Sensor] MLX90614 initialized");
+  } else {
+    Serial.println("[Sensor] MLX90614 not detected");
+  }
+
+  co2Serial.begin(9600, SERIAL_8N1, CO2_RX_PIN, CO2_TX_PIN);
+  co2Sensor.begin(co2Serial);
+  co2Sensor.autoCalibration(false);
+}
+
+void readSensors() {
+  if (lightMeter.measurementReady()) {
+    latest.lux = lightMeter.readLightLevel();
+    latest.ppfd = luxToPPFD(latest.lux, channel);
+  }
+
+  float temp = sht31.readTemperature();
+  float humidity = sht31.readHumidity();
+  if (!isnan(temp) && !isnan(humidity)) {
+    latest.ambientTempC = temp;
+    latest.humidity = humidity;
+  }
+
+  double objTemp = mlx.readObjectTempC();
+  if (!isnan(objTemp)) {
+    latest.leafTempC = objTemp;
+  }
+
+  int ppm = co2Sensor.getCO2();
+  if (ppm > 0 && ppm < 5000) {
+    latest.co2ppm = ppm;
+  }
+}
+
+// ----------------------------
+// Web server and API
+// ----------------------------
+String htmlPage() {
+  String page = R"HTML(
+  <!doctype html>
+  <html lang="de">
+  <head>
+    <meta charset="UTF-8" />
+    <meta name="viewport" content="width=device-width, initial-scale=1" />
+    <title>GrowSensor</title>
+    <style>
+      :root { color-scheme: light dark; }
+      body { font-family: system-ui, sans-serif; margin: 0; padding: 0; background: #0f172a; color: #e2e8f0; }
+      header { padding: 16px; background: #111827; box-shadow: 0 2px 6px rgba(0,0,0,0.25); }
+      h1 { margin: 0; font-size: 1.2rem; }
+      main { padding: 16px; display: grid; gap: 12px; }
+      .card { background: #111827; border: 1px solid #1f2937; border-radius: 12px; padding: 16px; box-shadow: 0 4px 8px rgba(0,0,0,0.25); }
+      .grid { display: grid; gap: 12px; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); }
+      .value { font-size: 1.6rem; font-variant-numeric: tabular-nums; }
+      label { display: block; margin-top: 8px; font-size: 0.9rem; }
+      input, select, button { width: 100%; padding: 10px; margin-top: 4px; border-radius: 8px; border: 1px solid #1f2937; background: #0b1220; color: #e2e8f0; }
+      button { cursor: pointer; border: none; background: linear-gradient(120deg, #22d3ee, #6366f1); color: #0b1220; font-weight: 600; }
+      .chart { position: relative; height: 200px; }
+      canvas { width: 100%; height: 200px; }
+      footer { text-align: center; padding: 12px; font-size: 0.85rem; color: #94a3b8; }
+    </style>
+  </head>
+  <body>
+    <header><h1>GrowSensor – v0.1</h1></header>
+    <main>
+      <section class="grid">
+        <article class="card"><div>Licht (Lux)</div><div class="value" id="lux">–</div></article>
+        <article class="card"><div>PPFD (µmol/m²/s)</div><div class="value" id="ppfd">–</div></article>
+        <article class="card"><div>CO₂ (ppm)</div><div class="value" id="co2">–</div></article>
+        <article class="card"><div>Temperatur (°C)</div><div class="value" id="temp">–</div></article>
+        <article class="card"><div>Luftfeuchte (%)</div><div class="value" id="humidity">–</div></article>
+        <article class="card"><div>Blatt-Temp (°C)</div><div class="value" id="leaf">–</div></article>
+      </section>
+
+      <section class="card">
+        <h3 style="margin-top:0">Live Verlauf</h3>
+        <div class="chart"><canvas id="chart"></canvas></div>
+      </section>
+
+      <section class="grid">
+        <article class="card">
+          <h3 style="margin-top:0">Spektrum wählen</h3>
+          <label for="channel">LED Kanal</label>
+          <select id="channel">
+            <option value="full_spectrum">Vollspectrum</option>
+            <option value="white">White</option>
+            <option value="bloom">Blütekanal</option>
+          </select>
+          <button id="saveChannel">Speichern</button>
+        </article>
+
+        <article class="card">
+          <h3 style="margin-top:0">Wi-Fi Setup</h3>
+          <label for="ssid">SSID</label>
+          <input id="ssid" placeholder="WLAN Name" />
+          <label for="pass">Passwort</label>
+          <input id="pass" type="password" placeholder="WLAN Passwort" />
+          <button id="saveWifi">Verbinden & Speichern</button>
+          <button id="resetWifi" style="margin-top:8px;background:#ef4444;color:#fff;">WLAN Reset</button>
+          <p id="wifiStatus" style="margin-top:8px;color:#a5b4fc"></p>
+        </article>
+      </section>
+    </main>
+    <footer>Growcontroller v0.1 • Sensorgehäuse v0.3</footer>
+
+    <script>
+      const chartCanvas = document.getElementById('chart');
+      const ctx = chartCanvas.getContext('2d');
+      const maxPoints = 48;
+      const history = { labels: [], lux: [], co2: [], temp: [], humidity: [] };
+
+      function drawChart() {
+        ctx.clearRect(0, 0, chartCanvas.width, chartCanvas.height);
+        const width = chartCanvas.width;
+        const height = chartCanvas.height;
+        ctx.strokeStyle = '#1f2937';
+        for (let i = 1; i < 5; i++) {
+          const y = (height / 5) * i;
+          ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(width, y); ctx.stroke();
+        }
+        const series = [
+          { data: history.lux, color: '#22d3ee', label: 'Lux' },
+          { data: history.co2, color: '#f59e0b', label: 'CO₂' },
+          { data: history.temp, color: '#34d399', label: 'Temp' },
+        ];
+        series.forEach((serie, index) => {
+          if (!serie.data.length) return;
+          const maxVal = Math.max(...serie.data);
+          const minVal = Math.min(...serie.data);
+          const span = Math.max(maxVal - minVal, 0.0001);
+          ctx.beginPath();
+          serie.data.forEach((val, i) => {
+            const x = (i / Math.max(series[0].data.length - 1, 1)) * width;
+            const y = height - ((val - minVal) / span) * height;
+            if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
+          });
+          ctx.strokeStyle = serie.color;
+          ctx.lineWidth = 2;
+          ctx.stroke();
+        });
+      }
+
+      function pushHistory(obj) {
+        history.labels.push(new Date().toLocaleTimeString());
+        history.lux.push(obj.lux);
+        history.co2.push(obj.co2);
+        history.temp.push(obj.temp);
+        history.humidity.push(obj.humidity);
+        Object.keys(history).forEach(key => {
+          if (history[key].length > maxPoints) history[key].shift();
+        });
+        drawChart();
+      }
+
+      async function fetchData() {
+        try {
+          const res = await fetch('/api/telemetry');
+          const data = await res.json();
+          document.getElementById('lux').textContent = data.lux?.toFixed(1) ?? '–';
+          document.getElementById('ppfd').textContent = data.ppfd?.toFixed(1) ?? '–';
+          document.getElementById('co2').textContent = data.co2 ?? '–';
+          document.getElementById('temp').textContent = data.temp?.toFixed(1) ?? '–';
+          document.getElementById('humidity').textContent = data.humidity?.toFixed(1) ?? '–';
+          document.getElementById('leaf').textContent = data.leaf?.toFixed(1) ?? '–';
+          pushHistory(data);
+        } catch (err) {
+          console.warn('telemetry failed', err);
+        }
+      }
+
+      async function loadSettings() {
+        const res = await fetch('/api/settings');
+        const data = await res.json();
+        document.getElementById('channel').value = data.channel;
+        document.getElementById('wifiStatus').textContent = data.wifi;
+        document.getElementById('ssid').value = data.ssid || '';
+      }
+
+      document.getElementById('saveChannel').addEventListener('click', async () => {
+        const channel = document.getElementById('channel').value;
+        await fetch('/api/settings', { method: 'POST', headers: {'Content-Type':'application/x-www-form-urlencoded'}, body: `channel=${channel}` });
+        document.getElementById('wifiStatus').textContent = 'Spektrum gespeichert';
+      });
+
+      document.getElementById('saveWifi').addEventListener('click', async () => {
+        const ssid = document.getElementById('ssid').value;
+        const pass = document.getElementById('pass').value;
+        document.getElementById('wifiStatus').textContent = 'Speichern & Neustart...';
+        await fetch('/api/wifi', { method: 'POST', headers: {'Content-Type':'application/x-www-form-urlencoded'}, body: `ssid=${encodeURIComponent(ssid)}&pass=${encodeURIComponent(pass)}` });
+        setTimeout(() => location.reload(), 2000);
+      });
+
+      document.getElementById('resetWifi').addEventListener('click', async () => {
+        document.getElementById('wifiStatus').textContent = 'Werkseinstellungen...';
+        await fetch('/api/reset', { method: 'POST' });
+        setTimeout(() => location.reload(), 1500);
+      });
+
+      setInterval(fetchData, 2500);
+      loadSettings();
+      fetchData();
+    </script>
+  </body>
+  </html>
+  )HTML";
+  return page;
+}
+
+void handleRoot() { server.send(200, "text/html", htmlPage()); }
+
+void handleNotFound() {
+  if (apMode) {
+    // Captive portal redirect while the access point is active
+    server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString(), true);
+    server.send(302, "text/plain", "");
+    return;
+  }
+  server.send(404, "text/plain", "Not found");
+}
+
+void handleTelemetry() {
+  char json[256];
+  snprintf(json, sizeof(json),
+           "{\"lux\":%.1f,\"ppfd\":%.1f,\"co2\":%d,\"temp\":%.1f,\"humidity\":%.1f,\"leaf\":%.1f}",
+           latest.lux, latest.ppfd, latest.co2ppm, latest.ambientTempC, latest.humidity,
+           latest.leafTempC);
+  server.send(200, "application/json", json);
+}
+
+void handleSettings() {
+  if (server.method() == HTTP_GET) {
+    String json = "{";
+    json += "\"channel\":\"" + lightChannelName() + "\",";
+    json += "\"wifi\":\"" + String(apMode ? "Access Point aktiv" : "Verbunden: " + WiFi.SSID()) + "\",";
+    json += "\"ssid\":\"" + savedSsid + "\"";
+    json += "}";
+    server.send(200, "application/json", json);
+    return;
+  }
+
+  if (!server.hasArg("channel")) {
+    server.send(400, "text/plain", "channel missing");
+    return;
+  }
+  LightChannel next = lightChannelFromString(server.arg("channel"));
+  channel = next;
+  prefs.begin("grow-sensor", false);
+  prefs.putString("channel", lightChannelName());
+  prefs.end();
+  server.send(200, "text/plain", "saved");
+}
+
+void handleWifiSave() {
+  if (!server.hasArg("ssid") || !server.hasArg("pass")) {
+    server.send(400, "text/plain", "missing ssid/pass");
+    return;
+  }
+  saveWifiCredentials(server.arg("ssid"), server.arg("pass"));
+  server.send(200, "text/plain", "ok, rebooting");
+  delay(500);
+  ESP.restart();
+}
+
+void handleReset() {
+  clearPreferences();
+  server.send(200, "text/plain", "cleared");
+  delay(200);
+  ESP.restart();
+}
+
+void setupServer() {
+  server.on("/", HTTP_GET, handleRoot);
+  server.on("/api/telemetry", HTTP_GET, handleTelemetry);
+  server.on("/api/settings", HTTP_GET, handleSettings);
+  server.on("/api/settings", HTTP_POST, handleSettings);
+  server.on("/api/wifi", HTTP_POST, handleWifiSave);
+  server.on("/api/reset", HTTP_POST, handleReset);
+  server.onNotFound(handleNotFound);
+  server.begin();
+}
+
+// ----------------------------
+// Arduino entry points
+// ----------------------------
+void setup() {
+  Serial.begin(115200);
+  delay(200);
+  Serial.println("\nGrowSensor booting...");
+
+  initSensors();
+  connectToWiFi();
+  setupServer();
+}
+
+void loop() {
+  dnsServer.processNextRequest();
+  server.handleClient();
+
+  if (millis() - lastSensorMillis >= SENSOR_INTERVAL) {
+    lastSensorMillis = millis();
+    readSensors();
+  }
+}
