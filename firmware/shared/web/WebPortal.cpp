#include "web/WebPortal.h"
#include <ArduinoJson.h>
#if defined(HIVE_PLATFORM_ESP8266)
  #include <Updater.h>
#else
  #include <Update.h>
#endif

namespace {
String jsonNumber(float value) {
    if (isnan(value)) return "null";
    return String(value, 2);
}

bool beginFirmwareUpdate() {
#if defined(HIVE_PLATFORM_ESP8266)
    uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
    return Update.begin(maxSketchSpace);
#else
    return Update.begin(UPDATE_SIZE_UNKNOWN);
#endif
}
}

WebPortal::WebPortal(ConfigManager& configManager)
    : server_(80), configManager_(configManager) {
}

void WebPortal::begin(
    const Telemetry* latestTelemetry,
    TareHandler onTare,
    CalibrateHandler onCalibrate,
    ClearBufferHandler onClearBuffer,
    BufferPendingHandler onBufferPending,
    BufferSizeHandler onBufferSize
) {
    latestTelemetry_ = latestTelemetry;
    onTare_ = onTare;
    onCalibrate_ = onCalibrate;
    onClearBuffer_ = onClearBuffer;
    onBufferPending_ = onBufferPending;
    onBufferSize_ = onBufferSize;

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
            if (updateOk_) {
                server_.send(200, "text/plain; charset=utf-8", "Обновление загружено. Перезагрузка...");
                delay(800);
                platformRestart();
            } else {
                String message = updateError_.length() > 0 ? updateError_ : "Ошибка обновления";
                server_.send(500, "text/plain; charset=utf-8", message);
            }
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
      <div><label>Fallback AP</label><select id="apFallbackEnabled"><option value="true">включен</option><option value="false">выключен</option></select></div>
      <div><label>AP пароль</label><input id="apPassword" type="password"></div>
      <div><label>MQTT host</label><input id="mqttHost"></div>
      <div><label>MQTT port</label><input id="mqttPort" type="number"></div>
      <div><label>MQTT TLS</label><select id="mqttTls"><option value="false">без TLS</option><option value="true">с TLS</option></select></div>
      <div><label>MQTT user</label><input id="mqttUser"></div>
      <div><label>MQTT пароль</label><input id="mqttPassword" type="password"></div>
      <div><label>Весы</label><select id="weightEnabled"><option value="true">включены</option><option value="false">выключены</option></select></div>
      <div><label>HX711 DOUT</label><input id="hx711DoutPin" type="number"></div>
      <div><label>HX711 SCK</label><input id="hx711SckPin" type="number"></div>
      <div><label>Смещение тары</label><input id="tareOffset" type="number"></div>
      <div><label>Температура</label><select id="temperatureEnabled"><option value="true">включена</option><option value="false">выключена</option></select></div>
      <div><label>Влажность</label><select id="humidityEnabled"><option value="true">включена</option><option value="false">выключена</option></select></div>
      <div><label>DHT тип</label><select id="thSensorType"><option>DHT22</option><option>DHT11</option><option>DHT21</option></select></div>
      <div><label>DHT пин</label><input id="thSensorPin" type="number"></div>
      <div><label>Темп. тип</label><select id="temperatureSensorType"><option>DHT22</option><option>DHT11</option><option>DHT21</option></select></div>
      <div><label>Темп. пин</label><input id="temperatureSensorPin" type="number"></div>
      <div><label>Влажн. тип</label><select id="humiditySensorType"><option>DHT22</option><option>DHT11</option><option>DHT21</option></select></div>
      <div><label>Влажн. пин</label><input id="humiditySensorPin" type="number"></div>
      <div><label>Hall датчик</label><select id="hallEnabled"><option value="true">включен</option><option value="false">выключен</option></select></div>
      <div><label>Hall пин</label><input id="hallPin" type="number"></div>
      <div><label>Открыто при HIGH</label><select id="hallOpenLevelHigh"><option value="false">нет</option><option value="true">да</option></select></div>
      <div><label>Hall wake</label><select id="hallWakeMode"><option value="HALL_NO_WAKE">без wake</option><option value="HALL_WAKE_RST">RST wake</option></select></div>
      <div><label>Батарея</label><select id="batteryEnabled"><option value="true">включена</option><option value="false">выключена</option></select></div>
      <div><label>ADC пин</label><input id="batteryAdcPin" type="number"></div>
      <div><label>Мин. напряжение</label><input id="batteryMinVoltage" type="number" step="0.01"></div>
      <div><label>Макс. напряжение</label><input id="batteryMaxVoltage" type="number" step="0.01"></div>
      <div><label>Делитель</label><input id="batteryDividerRatio" type="number" step="0.01"></div>
      <div><label>ADC reference</label><input id="adcReferenceVoltage" type="number" step="0.01"></div>
      <div><label>ADC max</label><input id="adcMaxValue" type="number"></div>
      <div><label>Источник времени</label><select id="timeSource"><option>NTP</option><option>RTC</option><option>AUTO</option></select></div>
      <div><label>NTP server</label><input id="ntpServer"></div>
      <div><label>Timezone, мин</label><input id="timezoneOffsetMinutes" type="number"></div>
      <div><label>RTC sync, мин</label><input id="rtcNtpSyncIntervalMinutes" type="number"></div>
      <div><label>Период, сек</label><input id="measurementIntervalSeconds" type="number"></div>
      <div><label>Deep sleep</label><select id="deepSleepEnabled"><option value="false">выключен</option><option value="true">включен</option></select></div>
      <div><label>Весовой порог, кг</label><input id="weightThresholdKg" type="number" step="0.01"></div>
      <div><label>Значимое изменение, кг</label><input id="significantWeightChangeKg" type="number" step="0.01"></div>
      <div><label>Коэффициент калибровки</label><input id="calibrationFactor" type="number" step="0.0001"></div>
    </div>
    <button onclick="saveConfig()">Сохранить</button>
    <button class="secondary" onclick="tare()">Тарировать</button>
    <button class="secondary" onclick="calibrate()">Калибровать</button>
    <button class="secondary" onclick="clearBuffer()">Очистить буфер</button>
    <button class="secondary" onclick="resetConfig()">Сбросить настройки</button>
    <button class="secondary" onclick="restartDevice()">Перезагрузить</button>
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
  apFallbackEnabled.value = String(config.wifi?.apFallbackEnabled ?? true);
  apPassword.value = config.wifi?.apPassword || '';
  mqttHost.value = config.mqtt?.host || '';
  mqttPort.value = config.mqtt?.port || 1883;
  mqttTls.value = String(!!config.mqtt?.tls);
  mqttUser.value = config.mqtt?.user || '';
  mqttPassword.value = config.mqtt?.password || '';
  weightEnabled.value = String(config.weight?.enabled ?? true);
  hx711DoutPin.value = config.weight?.doutPin ?? '';
  hx711SckPin.value = config.weight?.sckPin ?? '';
  tareOffset.value = config.weight?.tareOffset ?? 0;
  temperatureEnabled.value = String(config.environment?.temperatureEnabled ?? true);
  humidityEnabled.value = String(config.environment?.humidityEnabled ?? true);
  thSensorType.value = config.environment?.combinedType || 'DHT22';
  thSensorPin.value = config.environment?.combinedPin ?? '';
  temperatureSensorType.value = config.environment?.temperatureType || 'DHT22';
  temperatureSensorPin.value = config.environment?.temperaturePin ?? '';
  humiditySensorType.value = config.environment?.humidityType || 'DHT22';
  humiditySensorPin.value = config.environment?.humidityPin ?? '';
  hallEnabled.value = String(config.hall?.enabled ?? true);
  hallPin.value = config.hall?.pin ?? '';
  hallOpenLevelHigh.value = String(config.hall?.openLevelHigh ?? false);
  hallWakeMode.value = config.hall?.wakeMode || 'HALL_NO_WAKE';
  batteryEnabled.value = String(config.battery?.enabled ?? true);
  batteryAdcPin.value = config.battery?.adcPin ?? '';
  batteryMinVoltage.value = config.battery?.minVoltage ?? 3.2;
  batteryMaxVoltage.value = config.battery?.maxVoltage ?? 4.2;
  batteryDividerRatio.value = config.battery?.dividerRatio ?? 1;
  adcReferenceVoltage.value = config.battery?.adcReferenceVoltage ?? 3.3;
  adcMaxValue.value = config.battery?.adcMaxValue ?? 1023;
  timeSource.value = config.time?.source || 'NTP';
  ntpServer.value = config.time?.ntpServer || 'pool.ntp.org';
  timezoneOffsetMinutes.value = config.time?.timezoneOffsetMinutes ?? 0;
  rtcNtpSyncIntervalMinutes.value = config.time?.rtcNtpSyncIntervalMinutes ?? 1440;
  measurementIntervalSeconds.value = config.measurementIntervalSeconds || 1800;
  deepSleepEnabled.value = String(!!config.deepSleepEnabled);
  weightThresholdKg.value = config.weight?.thresholdKg ?? 5;
  significantWeightChangeKg.value = config.weight?.significantChangeKg ?? 20;
  calibrationFactor.value = config.weight?.calibrationFactor ?? 1;
}
async function saveConfig() {
  const next = {
    deviceId: deviceId.value, deviceToken: deviceToken.value, adminPassword: adminPassword.value,
    wifi:{ssid:wifiSsid.value,password:wifiPassword.value,apFallbackEnabled:apFallbackEnabled.value === 'true',apPassword:apPassword.value},
    mqtt:{host:mqttHost.value,port:+mqttPort.value,tls:mqttTls.value === 'true',user:mqttUser.value,password:mqttPassword.value},
    weight:{enabled:weightEnabled.value === 'true',doutPin:+hx711DoutPin.value,sckPin:+hx711SckPin.value,calibrationFactor:+calibrationFactor.value,tareOffset:+tareOffset.value,thresholdKg:+weightThresholdKg.value,significantChangeKg:+significantWeightChangeKg.value},
    environment:{temperatureEnabled:temperatureEnabled.value === 'true',humidityEnabled:humidityEnabled.value === 'true',combinedType:thSensorType.value,combinedPin:+thSensorPin.value,temperatureType:temperatureSensorType.value,temperaturePin:+temperatureSensorPin.value,humidityType:humiditySensorType.value,humidityPin:+humiditySensorPin.value},
    hall:{enabled:hallEnabled.value === 'true',pin:+hallPin.value,openLevelHigh:hallOpenLevelHigh.value === 'true',wakeMode:hallWakeMode.value},
    battery:{enabled:batteryEnabled.value === 'true',adcPin:+batteryAdcPin.value,minVoltage:+batteryMinVoltage.value,maxVoltage:+batteryMaxVoltage.value,dividerRatio:+batteryDividerRatio.value,adcReferenceVoltage:+adcReferenceVoltage.value,adcMaxValue:+adcMaxValue.value},
    time:{source:timeSource.value,ntpServer:ntpServer.value,timezoneOffsetMinutes:+timezoneOffsetMinutes.value,rtcNtpSyncIntervalMinutes:+rtcNtpSyncIntervalMinutes.value},
    measurementIntervalSeconds:+measurementIntervalSeconds.value,
    deepSleepEnabled:deepSleepEnabled.value === 'true'
  };
  await fetch('/api/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(next)});
  await load();
}
async function tare(){ await fetch('/api/tare',{method:'POST'}); await load(); }
async function calibrate(){ const kg = prompt('Укажите известный вес, кг'); if(kg) await fetch('/api/calibrate?kg='+encodeURIComponent(kg),{method:'POST'}); await load(); }
async function clearBuffer(){ await fetch('/api/buffer/clear',{method:'POST'}); await load(); }
async function resetConfig(){ if(confirm('Сбросить настройки?')) await fetch('/api/config/reset',{method:'POST'}); await load(); }
async function restartDevice(){ await fetch('/api/restart',{method:'POST'}); }
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
    if (onBufferPending_) doc["buffer"]["pendingCount"] = onBufferPending_();
    if (onBufferSize_) doc["buffer"]["sizeBytes"] = onBufferSize_();
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
        updateInProgress_ = true;
        updateOk_ = false;
        updateBytes_ = 0;
        updateError_ = "";
        if (!beginFirmwareUpdate()) {
            failUpdate("Не удалось начать обновление");
        }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (!updateInProgress_ || updateError_.length() > 0) return;
        size_t written = Update.write(upload.buf, upload.currentSize);
        updateBytes_ += written;
        if (written != upload.currentSize) {
            failUpdate("Ошибка записи прошивки");
        }
    } else if (upload.status == UPLOAD_FILE_END) {
        if (!updateInProgress_ || updateError_.length() > 0) return;
        if (updateBytes_ == 0) {
            failUpdate("Файл прошивки пустой");
            return;
        }
        if (!Update.end(true)) {
            failUpdate("Ошибка завершения обновления");
            return;
        }
        updateInProgress_ = false;
        updateOk_ = !Update.hasError();
        if (!updateOk_) {
            updateError_ = "Ошибка проверки прошивки";
        }
    } else if (upload.status == UPLOAD_FILE_ABORTED) {
        failUpdate("Загрузка прошивки прервана");
    }
}

void WebPortal::failUpdate(const String& message) {
    updateInProgress_ = false;
    updateOk_ = false;
    updateError_ = message;
}
