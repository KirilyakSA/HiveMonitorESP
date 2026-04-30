#include "web/WebPortal.h"
#include <ArduinoJson.h>
#include <Update.h>

namespace {
String jsonNumber(float value) {
    if (isnan(value)) return "null";
    return String(value, 2);
}
}

WebPortal::WebPortal(ConfigManager& configManager)
    : server_(80), configManager_(configManager) {
}

void WebPortal::begin(
    const Telemetry* latestTelemetry,
    TareHandler onTare,
    CalibrateHandler onCalibrate,
    ClearBufferHandler onClearBuffer
) {
    latestTelemetry_ = latestTelemetry;
    onTare_ = onTare;
    onCalibrate_ = onCalibrate;
    onClearBuffer_ = onClearBuffer;

    server_.on("/", HTTP_GET, [this]() { handleIndex(); });
    server_.on("/api/config", HTTP_GET, [this]() { handleConfigGet(); });
    server_.on("/api/config", HTTP_POST, [this]() { handleConfigPost(); });
    server_.on("/api/status", HTTP_GET, [this]() { handleStatus(); });
    server_.on("/api/tare", HTTP_POST, [this]() { handleTare(); });
    server_.on("/api/calibrate", HTTP_POST, [this]() { handleCalibrate(); });
    server_.on("/api/buffer/clear", HTTP_POST, [this]() { handleClearBuffer(); });
    server_.on("/api/config/reset", HTTP_POST, [this]() { handleResetConfig(); });
    server_.on("/api/restart", HTTP_POST, [this]() { handleRestart(); });
    server_.on("/update", HTTP_GET, [this]() { handleUpdateForm(); });
    server_.on(
        "/update",
        HTTP_POST,
        [this]() {
            bool ok = !Update.hasError();
            server_.send(200, "text/plain; charset=utf-8", ok ? "Обновление загружено. Перезагрузка..." : "Ошибка обновления");
            delay(800);
            if (ok) platformRestart();
        },
        [this]() { handleUpdateUpload(); }
    );
    server_.begin();
}

void WebPortal::loop() {
    server_.handleClient();
}

bool WebPortal::authenticated() {
    const AppConfig& cfg = configManager_.data();
    if (cfg.adminPassword.length() == 0) return true;
    return server_.authenticate("admin", cfg.adminPassword.c_str());
}

void WebPortal::sendUnauthorized() {
    server_.requestAuthentication();
}

void WebPortal::handleIndex() {
    if (!authenticated()) return sendUnauthorized();

    const char* html = R"HTML(
<!doctype html>
<html lang="ru">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>HiveMonitor</title>
  <style>
    body{font-family:Arial,sans-serif;margin:0;background:#f5f7f2;color:#18231b}
    main{max-width:980px;margin:0 auto;padding:20px}
    h1{font-size:28px;margin:0 0 16px}
    section{background:#fff;border:1px solid #d9dfd3;border-radius:8px;padding:16px;margin:12px 0}
    label{display:block;font-size:13px;margin:10px 0 4px;color:#40503e}
    input,select{box-sizing:border-box;width:100%;padding:9px;border:1px solid #b9c3b2;border-radius:6px}
    .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:10px 14px}
    button{padding:10px 14px;border:0;border-radius:6px;background:#236b3a;color:white;cursor:pointer;margin:8px 8px 0 0}
    button.secondary{background:#52665a}
    pre{white-space:pre-wrap;background:#eef2ea;padding:12px;border-radius:6px;overflow:auto}
  </style>
</head>
<body>
<main>
  <h1>HiveMonitor</h1>
  <section><h2>Текущие показания</h2><pre id="status">Загрузка...</pre></section>
  <section>
    <h2>Настройки</h2>
    <div class="grid">
      <div><label>Device ID</label><input id="deviceId"></div>
      <div><label>Device Token</label><input id="deviceToken"></div>
      <div><label>Пароль администратора</label><input id="adminPassword" type="password"></div>
      <div><label>Wi-Fi SSID</label><input id="wifiSsid"></div>
      <div><label>Wi-Fi пароль</label><input id="wifiPassword" type="password"></div>
      <div><label>MQTT host</label><input id="mqttHost"></div>
      <div><label>MQTT port</label><input id="mqttPort" type="number"></div>
      <div><label>MQTT TLS</label><select id="mqttTls"><option value="false">без TLS</option><option value="true">с TLS</option></select></div>
      <div><label>HX711 DOUT</label><input id="hx711DoutPin" type="number"></div>
      <div><label>HX711 SCK</label><input id="hx711SckPin" type="number"></div>
      <div><label>DHT тип</label><select id="thSensorType"><option>DHT22</option><option>DHT11</option><option>DHT21</option></select></div>
      <div><label>DHT пин</label><input id="thSensorPin" type="number"></div>
      <div><label>Hall пин</label><input id="hallPin" type="number"></div>
      <div><label>Период, сек</label><input id="measurementIntervalSeconds" type="number"></div>
      <div><label>Весовой порог, кг</label><input id="weightThresholdKg" type="number" step="0.01"></div>
      <div><label>Значимое изменение, кг</label><input id="significantWeightChangeKg" type="number" step="0.01"></div>
      <div><label>Коэффициент калибровки</label><input id="calibrationFactor" type="number" step="0.0001"></div>
    </div>
    <button onclick="saveConfig()">Сохранить</button>
    <button class="secondary" onclick="tare()">Тарировать</button>
    <button class="secondary" onclick="calibrate()">Калибровать</button>
    <button class="secondary" onclick="clearBuffer()">Очистить буфер</button>
    <button class="secondary" onclick="location.href='/update'">Обновить прошивку</button>
  </section>
</main>
<script>
let config = {};
async function load() {
  config = await (await fetch('/api/config')).json();
  status.textContent = JSON.stringify(await (await fetch('/api/status')).json(), null, 2);
  deviceId.value = config.deviceId || '';
  deviceToken.value = config.deviceToken || '';
  adminPassword.value = config.adminPassword || '';
  wifiSsid.value = config.wifi?.ssid || '';
  wifiPassword.value = config.wifi?.password || '';
  mqttHost.value = config.mqtt?.host || '';
  mqttPort.value = config.mqtt?.port || 1883;
  mqttTls.value = String(!!config.mqtt?.tls);
  hx711DoutPin.value = config.weight?.doutPin ?? '';
  hx711SckPin.value = config.weight?.sckPin ?? '';
  thSensorType.value = config.environment?.combinedType || 'DHT22';
  thSensorPin.value = config.environment?.combinedPin ?? '';
  hallPin.value = config.hall?.pin ?? '';
  measurementIntervalSeconds.value = config.measurementIntervalSeconds || 1800;
  weightThresholdKg.value = config.weight?.thresholdKg ?? 5;
  significantWeightChangeKg.value = config.weight?.significantChangeKg ?? 20;
  calibrationFactor.value = config.weight?.calibrationFactor ?? 1;
}
async function saveConfig() {
  const next = {
    deviceId: deviceId.value, deviceToken: deviceToken.value, adminPassword: adminPassword.value,
    wifi:{ssid:wifiSsid.value,password:wifiPassword.value,apFallbackEnabled:true},
    mqtt:{host:mqttHost.value,port:+mqttPort.value,tls:mqttTls.value === 'true'},
    weight:{enabled:true,doutPin:+hx711DoutPin.value,sckPin:+hx711SckPin.value,calibrationFactor:+calibrationFactor.value,thresholdKg:+weightThresholdKg.value,significantChangeKg:+significantWeightChangeKg.value},
    environment:{temperatureEnabled:true,humidityEnabled:true,combinedType:thSensorType.value,combinedPin:+thSensorPin.value},
    hall:{enabled:true,pin:+hallPin.value},
    measurementIntervalSeconds:+measurementIntervalSeconds.value
  };
  await fetch('/api/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(next)});
  await load();
}
async function tare(){ await fetch('/api/tare',{method:'POST'}); await load(); }
async function calibrate(){ const kg = prompt('Укажите известный вес, кг'); if(kg) await fetch('/api/calibrate?kg='+encodeURIComponent(kg),{method:'POST'}); await load(); }
async function clearBuffer(){ await fetch('/api/buffer/clear',{method:'POST'}); await load(); }
load(); setInterval(load, 10000);
</script>
</body>
</html>
)HTML";
    server_.send(200, "text/html; charset=utf-8", html);
}

void WebPortal::handleConfigGet() {
    if (!authenticated()) return sendUnauthorized();
    server_.send(200, "application/json", configManager_.toJson(true));
}

void WebPortal::handleConfigPost() {
    if (!authenticated()) return sendUnauthorized();
    String error;
    if (!configManager_.updateFromJson(server_.arg("plain"), error)) {
        server_.send(400, "text/plain", error);
        return;
    }
    server_.send(200, "application/json", "{\"ok\":true}");
}

void WebPortal::handleStatus() {
    if (!authenticated()) return sendUnauthorized();

    JsonDocument doc;
    doc["firmwareVersion"] = HIVE_FW_VERSION;
    doc["ip"] = WiFi.localIP().toString();
    doc["wifiConnected"] = WiFi.status() == WL_CONNECTED;
    doc["rssi"] = WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0;
    doc["freeHeap"] = ESP.getFreeHeap();
    if (latestTelemetry_) {
        doc["telemetry"]["deviceId"] = latestTelemetry_->deviceId;
        doc["telemetry"]["timestamp"] = latestTelemetry_->timestamp;
        doc["telemetry"]["weight"] = serialized(jsonNumber(latestTelemetry_->weight));
        doc["telemetry"]["weightChange"] = serialized(jsonNumber(latestTelemetry_->weightChange));
        doc["telemetry"]["temperature"] = serialized(jsonNumber(latestTelemetry_->temperature));
        doc["telemetry"]["humidity"] = serialized(jsonNumber(latestTelemetry_->humidity));
        doc["telemetry"]["hiveOpened"] = latestTelemetry_->hiveOpened;
        doc["telemetry"]["batteryPercent"] = latestTelemetry_->batteryPercent;
        doc["telemetry"]["batteryVoltage"] = serialized(jsonNumber(latestTelemetry_->batteryVoltage));
    }
    String out;
    serializeJsonPretty(doc, out);
    server_.send(200, "application/json", out);
}

void WebPortal::handleTare() {
    if (!authenticated()) return sendUnauthorized();
    bool ok = onTare_ && onTare_();
    server_.send(ok ? 200 : 500, "application/json", ok ? "{\"ok\":true}" : "{\"ok\":false}");
}

void WebPortal::handleCalibrate() {
    if (!authenticated()) return sendUnauthorized();
    float kg = server_.arg("kg").toFloat();
    bool ok = kg > 0.0f && onCalibrate_ && onCalibrate_(kg);
    server_.send(ok ? 200 : 400, "application/json", ok ? "{\"ok\":true}" : "{\"ok\":false}");
}

void WebPortal::handleClearBuffer() {
    if (!authenticated()) return sendUnauthorized();
    bool ok = onClearBuffer_ && onClearBuffer_();
    server_.send(ok ? 200 : 500, "application/json", ok ? "{\"ok\":true}" : "{\"ok\":false}");
}

void WebPortal::handleResetConfig() {
    if (!authenticated()) return sendUnauthorized();
    bool ok = configManager_.resetDefaults();
    server_.send(ok ? 200 : 500, "application/json", ok ? "{\"ok\":true}" : "{\"ok\":false}");
}

void WebPortal::handleRestart() {
    if (!authenticated()) return sendUnauthorized();
    server_.send(200, "application/json", "{\"ok\":true}");
    delay(500);
    platformRestart();
}

void WebPortal::handleUpdateForm() {
    if (!authenticated()) return sendUnauthorized();
    String html = "<!doctype html><html lang='ru'><meta charset='utf-8'><title>Обновление</title>"
        "<body><h1>Обновление прошивки</h1><form method='POST' action='/update' enctype='multipart/form-data'>"
        "<input type='file' name='firmware'><button>Загрузить</button></form><p><a href='/'>Назад</a></p></body></html>";
    server_.send(200, "text/html; charset=utf-8", html);
}

void WebPortal::handleUpdateUpload() {
    if (!authenticated()) return;
    HTTPUpload& upload = server_.upload();
    if (upload.status == UPLOAD_FILE_START) {
        Update.begin(UPDATE_SIZE_UNKNOWN);
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        Update.write(upload.buf, upload.currentSize);
    } else if (upload.status == UPLOAD_FILE_END) {
        Update.end(true);
    }
}

