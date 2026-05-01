#include "domain/HiveMonitorApp.h"
#include <ArduinoJson.h>
#include "platform/Platform.h"

namespace {
String jsonFloat(float value) {
    if (isnan(value)) return "null";
    return String(value, 2);
}
}

HiveMonitorApp::HiveMonitorApp() : webPortal_(configManager_) {
}

void HiveMonitorApp::setup() {
    Serial.begin(115200);
    delay(100);
    Serial.println();
    Serial.println("HiveMonitor firmware " HIVE_FW_VERSION);

    setupFileSystem();
    if (handleFactoryResetButton()) {
        return;
    }
    if (!configManager_.begin()) {
        Serial.println("Config load failed, using defaults");
        configManager_.resetDefaults();
    }

    setupWifi();
    rememberWifiConfig();
    timeService_.begin(configManager_.data());
    telemetryBuffer_.begin();
    loadCell_.begin(configManager_.data());
    environmentSensor_.begin(configManager_.data());
    batteryMonitor_.begin(configManager_.data());
    hallSensor_.begin(configManager_.data());
    mqttService_.begin(configManager_.data());
    mqttService_.setMessageHandler([this](const String& topic, const String& payload) {
        handleMqttMessage(topic, payload);
    });
    appliedConfigVersion_ = configManager_.data().configVersion;

    webPortal_.begin(
        &latestTelemetry_,
        [this]() { return handleTare(); },
        [this](float kg) { return handleCalibrate(kg); },
        [this]() { return telemetryBuffer_.clear(); },
        [this]() { return telemetryBuffer_.pendingCount(); },
        [this]() { return telemetryBuffer_.sizeBytes(); },
        [this]() { return mqttConnected(); },
        [this]() { return lastPublishOk(); },
        [this]() { return lastPublishMessage(); }
    );

    measureAndSend();
    enterDeepSleepIfEnabled();
}

void HiveMonitorApp::loop() {
    webPortal_.loop();
    if (configManager_.data().configVersion != appliedConfigVersion_) {
        applyConfigChanges();
    }
    timeService_.loop(configManager_.data());
    mqttService_.loop(configManager_.data());

    if (mqttService_.connected()) {
        telemetryBuffer_.flushTo([this](const String& line) {
            return mqttService_.publishTelemetry(configManager_.data(), line);
        });
    }

    uint32_t intervalMs = configManager_.data().measurementIntervalSeconds * 1000UL;
    if (intervalMs == 0) intervalMs = 1800000UL;
    if (millis() - lastMeasureMs_ >= intervalMs) {
        measureAndSend();
        enterDeepSleepIfEnabled();
    }
}

void HiveMonitorApp::setupFileSystem() {
    if (!HIVE_FS.begin()) {
#if defined(HIVE_PLATFORM_ESP8266)
        HIVE_FS.format();
        HIVE_FS.begin();
#else
        HIVE_FS.begin(true);
#endif
    }
}

bool HiveMonitorApp::handleFactoryResetButton() {
    int pin = platformDefaultFactoryResetPin();
    pinMode(pin, INPUT_PULLUP);
    delay(20);
    if (digitalRead(pin) != LOW) {
        return false;
    }

    Serial.println("Factory reset button detected; keep holding for 5 seconds...");
    uint32_t started = millis();
    while (millis() - started < 5000) {
        if (digitalRead(pin) != LOW) {
            Serial.println("Factory reset cancelled");
            return false;
        }
        delay(100);
        yield();
    }

    Serial.println("Factory reset confirmed; resetting config");
    configManager_.resetDefaults();
    delay(500);
    platformRestart();
    return true;
}

void HiveMonitorApp::setupWifi() {
    const AppConfig& cfg = configManager_.data();
    accessPointActive_ = false;
    WiFi.mode(WIFI_STA);
    if (cfg.wifiSsid.length() > 0) {
        WiFi.begin(cfg.wifiSsid.c_str(), cfg.wifiPassword.c_str());
        uint32_t started = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - started < 15000) {
            delay(250);
            Serial.print(".");
        }
        Serial.println();
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("Wi-Fi connected: ");
        Serial.println(WiFi.localIP());
    } else if (cfg.apFallbackEnabled) {
        startAccessPoint();
    }
}

void HiveMonitorApp::startAccessPoint() {
    const AppConfig& cfg = configManager_.data();
    String ssid = "HiveMonitor-" + platformChipId();
    WiFi.mode(WIFI_AP_STA);
    bool ok = WiFi.softAP(ssid.c_str(), cfg.apPassword.c_str());
    accessPointActive_ = ok;
    Serial.print("AP ");
    Serial.print(ssid);
    Serial.println(ok ? " started" : " failed");
}

void HiveMonitorApp::applyConfigChanges() {
    const AppConfig& cfg = configManager_.data();
    if (wifiConfigChanged()) {
        setupWifi();
        rememberWifiConfig();
    }
    timeService_.begin(cfg);
    loadCell_.updateConfig(cfg);
    environmentSensor_.updateConfig(cfg);
    batteryMonitor_.begin(cfg);
    hallSensor_.begin(cfg);
    mqttService_.begin(cfg);
    appliedConfigVersion_ = cfg.configVersion;
}

void HiveMonitorApp::rememberWifiConfig() {
    const AppConfig& cfg = configManager_.data();
    appliedWifiSsid_ = cfg.wifiSsid;
    appliedWifiPassword_ = cfg.wifiPassword;
    appliedApFallbackEnabled_ = cfg.apFallbackEnabled;
    appliedApPassword_ = cfg.apPassword;
}

bool HiveMonitorApp::wifiConfigChanged() const {
    const AppConfig& cfg = configManager_.data();
    return appliedWifiSsid_ != cfg.wifiSsid ||
        appliedWifiPassword_ != cfg.wifiPassword ||
        appliedApFallbackEnabled_ != cfg.apFallbackEnabled ||
        appliedApPassword_ != cfg.apPassword;
}

void HiveMonitorApp::measureAndSend() {
    const AppConfig& cfg = configManager_.data();
    latestTelemetry_ = Telemetry{};
    latestTelemetry_.deviceId = cfg.deviceId;
    latestTelemetry_.timestamp = timeService_.isoTimestamp();
    latestTelemetry_.rssi = WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0;

    bool error = false;

    if (cfg.weightEnabled) {
        latestTelemetry_.weight = loadCell_.readKg(5);
        latestTelemetry_.weightError = isnan(latestTelemetry_.weight);
        if (latestTelemetry_.weightError) error = true;
        if (!isnan(lastWeightKg_) && !isnan(latestTelemetry_.weight)) {
            latestTelemetry_.weightChange = latestTelemetry_.weight - lastWeightKg_;
        } else {
            latestTelemetry_.weightChange = 0.0f;
        }
    }

    if (cfg.temperatureEnabled) {
        latestTelemetry_.temperature = environmentSensor_.readTemperature();
        latestTelemetry_.temperatureError = isnan(latestTelemetry_.temperature);
        if (latestTelemetry_.temperatureError) error = true;
    }
    if (cfg.humidityEnabled) {
        latestTelemetry_.humidity = environmentSensor_.readHumidity();
        latestTelemetry_.humidityError = isnan(latestTelemetry_.humidity);
        if (latestTelemetry_.humidityError) error = true;
    }
    if (cfg.hallEnabled) {
        latestTelemetry_.hiveOpened = hallSensor_.isOpen(cfg);
    }
    if (cfg.batteryEnabled) {
        BatteryState battery = batteryMonitor_.read(cfg);
        latestTelemetry_.batteryVoltage = battery.voltage;
        latestTelemetry_.batteryPercent = battery.percent;
        latestTelemetry_.batteryError = isnan(battery.voltage) || battery.percent < 0;
        if (latestTelemetry_.batteryError) error = true;
    }

    latestTelemetry_.errorFlag = error;
    lastMeasureMs_ = millis();
    if (!isnan(latestTelemetry_.weight)) {
        lastWeightKg_ = latestTelemetry_.weight;
    }

    String payload = telemetryToJson(latestTelemetry_);
    publishOrBuffer(payload);

    if (latestTelemetry_.hiveOpened || fabs(latestTelemetry_.weightChange) >= cfg.significantWeightChangeKg || error) {
        mqttService_.publishEvent(cfg, payload);
    }
}

void HiveMonitorApp::enterDeepSleepIfEnabled() {
    const AppConfig& cfg = configManager_.data();
    if (!cfg.deepSleepEnabled || accessPointActive_) return;

    uint32_t sleepSeconds = cfg.measurementIntervalSeconds;
    if (sleepSeconds == 0) sleepSeconds = 1800;

    waitForTelemetryDelivery(10000);
    Serial.print("Deep sleep for ");
    Serial.print(sleepSeconds);
    Serial.println(" seconds");
    Serial.flush();
    platformDeepSleepSeconds(sleepSeconds);
}

bool HiveMonitorApp::waitForTelemetryDelivery(uint32_t timeoutMs) {
    const AppConfig& cfg = configManager_.data();
    if (WiFi.status() != WL_CONNECTED || cfg.mqttHost.length() == 0) {
        return false;
    }

    uint32_t started = millis();
    do {
        mqttService_.loop(cfg);
        if (mqttService_.connected()) {
            bool allSent = telemetryBuffer_.flushTo([this](const String& line) {
                return mqttService_.publishTelemetry(configManager_.data(), line);
            });
            if (allSent && telemetryBuffer_.pendingCount() == 0) {
                return true;
            }
        }
        delay(100);
        yield();
    } while (millis() - started < timeoutMs);

    return false;
}

void HiveMonitorApp::handleMqttMessage(const String& topic, const String& payload) {
    const AppConfig& cfg = configManager_.data();
    String commandsTopic = "hives/" + cfg.deviceId + "/commands";
    String configTopic = "hives/" + cfg.deviceId + "/config";

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, payload);
    String command;
    JsonVariantConst data;

    if (!err) {
        command = doc["command"] | doc["cmd"] | "";
        data = doc["data"].isNull() ? doc.as<JsonVariantConst>() : doc["data"].as<JsonVariantConst>();
    } else {
        command = payload;
        command.trim();
        data = JsonVariantConst();
    }

    if (topic == configTopic && command.length() == 0) {
        command = "configUpdate";
        data = doc.as<JsonVariantConst>();
    }

    if (topic != commandsTopic && topic != configTopic) {
        return;
    }
    if (command.length() == 0) {
        publishCommandStatus("unknown", false, "Missing command");
        return;
    }

    String message;
    bool ok = handleMqttCommand(command, data, message);
    publishCommandStatus(command, ok, message);
    if (ok && command == "restart") {
        delay(500);
        platformRestart();
    }
}

bool HiveMonitorApp::handleMqttCommand(const String& command, JsonVariantConst data, String& message) {
    if (command == "__payloadTooLarge") {
        message = "MQTT payload is too large";
        return false;
    }
    if (command == "measure") {
        measureAndSend();
        message = "Measurement published or buffered";
        return true;
    }
    if (command == "restart") {
        message = "Restarting";
        return true;
    }
    if (command == "tare") {
        bool ok = handleTare();
        message = ok ? "Tare saved" : "Tare failed";
        return ok;
    }
    if (command == "clearBuffer") {
        bool ok = telemetryBuffer_.clear();
        message = ok ? "Buffer cleared" : "Buffer clear failed";
        return ok;
    }
    if (command == "configUpdate") {
        return handleMqttConfigUpdate(data, message);
    }

    message = "Unsupported command";
    return false;
}

bool HiveMonitorApp::handleMqttConfigUpdate(JsonVariantConst data, String& message) {
    if (data.isNull()) {
        message = "Missing config data";
        return false;
    }

    String body;
    serializeJson(data, body);
    String error;
    if (!configManager_.updateFromJson(body, error)) {
        message = error.length() > 0 ? error : "Config update failed";
        return false;
    }
    applyConfigChanges();
    message = "Config updated";
    return true;
}

void HiveMonitorApp::publishCommandStatus(const String& command, bool ok, const String& message) {
    JsonDocument doc;
    doc["schemaVersion"] = 1;
    doc["firmwareVersion"] = HIVE_FW_VERSION;
    doc["deviceId"] = configManager_.data().deviceId;
    doc["timestamp"] = timeService_.isoTimestamp();
    doc["type"] = "commandStatus";
    doc["command"] = command;
    doc["ok"] = ok;
    doc["message"] = message;

    String payload;
    serializeJson(doc, payload);
    mqttService_.publishStatus(configManager_.data(), payload);
}

String HiveMonitorApp::telemetryToJson(const Telemetry& telemetry) const {
    const AppConfig& cfg = configManager_.data();
    JsonDocument doc;
    doc["schemaVersion"] = 1;
    doc["firmwareVersion"] = HIVE_FW_VERSION;
    doc["configVersion"] = cfg.configVersion;
    doc["deviceId"] = telemetry.deviceId;
    doc["timestamp"] = telemetry.timestamp;
    doc["uptimeSeconds"] = millis() / 1000UL;
    doc["measurementIntervalSeconds"] = cfg.measurementIntervalSeconds;
    doc["weight"] = serialized(jsonFloat(telemetry.weight));
    doc["weightChange"] = serialized(jsonFloat(telemetry.weightChange));
    doc["temperature"] = serialized(jsonFloat(telemetry.temperature));
    doc["humidity"] = serialized(jsonFloat(telemetry.humidity));
    doc["hiveOpened"] = telemetry.hiveOpened;
    doc["errorFlag"] = telemetry.errorFlag;
    doc["weightError"] = telemetry.weightError;
    doc["temperatureError"] = telemetry.temperatureError;
    doc["humidityError"] = telemetry.humidityError;
    doc["batteryError"] = telemetry.batteryError;
    doc["batteryPercent"] = telemetry.batteryPercent;
    doc["batteryVoltage"] = serialized(jsonFloat(telemetry.batteryVoltage));
    doc["rssi"] = telemetry.rssi;
    doc["freeHeap"] = ESP.getFreeHeap();

    String out;
    serializeJson(doc, out);
    return out;
}

void HiveMonitorApp::publishOrBuffer(const String& payload) {
    if (WiFi.status() == WL_CONNECTED && mqttService_.publishTelemetry(configManager_.data(), payload)) {
        lastPublishOk_ = true;
        lastPublishMessage_ = "Telemetry published";
        return;
    }
    lastPublishOk_ = false;
    lastPublishMessage_ = telemetryBuffer_.append(payload) ? "Telemetry buffered" : "Telemetry publish and buffer failed";
}

bool HiveMonitorApp::mqttConnected() {
    return mqttService_.connected();
}

bool HiveMonitorApp::lastPublishOk() const {
    return lastPublishOk_;
}

String HiveMonitorApp::lastPublishMessage() const {
    return lastPublishMessage_;
}

bool HiveMonitorApp::handleTare() {
    AppConfig& cfg = configManager_.data();
    cfg.tareOffset = loadCell_.tare(10);
    cfg.configVersion++;
    return configManager_.save();
}

bool HiveMonitorApp::handleCalibrate(float knownWeightKg) {
    AppConfig& cfg = configManager_.data();
    cfg.calibrationFactor = loadCell_.calibrate(knownWeightKg, 10);
    cfg.configVersion++;
    return configManager_.save();
}
