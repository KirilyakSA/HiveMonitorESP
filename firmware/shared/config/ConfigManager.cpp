#include "config/ConfigManager.h"
#include "platform/Platform.h"
#include <time.h>

namespace {
const char* CONFIG_PATH = "/config.json";
const size_t MAX_CONFIG_JSON_BYTES = 4096;

const char* toString(TimeSource value) {
    switch (value) {
        case TimeSource::Rtc: return "RTC";
        case TimeSource::Auto: return "AUTO";
        case TimeSource::Ntp:
        default: return "NTP";
    }
}

TimeSource parseTimeSource(const String& value) {
    if (value == "RTC") return TimeSource::Rtc;
    if (value == "AUTO") return TimeSource::Auto;
    return TimeSource::Ntp;
}

const char* toString(HallWakeMode value) {
    return value == HallWakeMode::WakeByReset ? "HALL_WAKE_RST" : "HALL_NO_WAKE";
}

HallWakeMode parseHallWakeMode(const String& value) {
    return value == "HALL_WAKE_RST" ? HallWakeMode::WakeByReset : HallWakeMode::NoWake;
}
}

bool ConfigManager::begin() {
    applyDefaults();
    if (!HIVE_FS.exists(CONFIG_PATH)) {
        return save();
    }

    File file = HIVE_FS.open(CONFIG_PATH, "r");
    if (!file) {
        return false;
    }

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, file);
    file.close();
    if (err) {
        return false;
    }
    fromDocument(doc);
    String error;
    if (!validate(error)) {
        return false;
    }
    return true;
}

AppConfig& ConfigManager::data() {
    return config_;
}

const AppConfig& ConfigManager::data() const {
    return config_;
}

bool ConfigManager::save() {
    JsonDocument doc;
    toDocument(doc, true);
    File file = HIVE_FS.open(CONFIG_PATH, "w");
    if (!file) {
        return false;
    }
    serializeJsonPretty(doc, file);
    file.close();
    return true;
}

bool ConfigManager::resetDefaults() {
    applyDefaults();
    return save();
}

String ConfigManager::toJson(bool includeSecrets) const {
    JsonDocument doc;
    toDocument(doc, includeSecrets);
    String out;
    serializeJsonPretty(doc, out);
    return out;
}

bool ConfigManager::updateFromJson(const String& body, String& error) {
    if (body.length() > MAX_CONFIG_JSON_BYTES) {
        error = "Config JSON is too large";
        return false;
    }

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, body);
    if (err) {
        error = err.c_str();
        return false;
    }
    AppConfig previous = config_;
    fromDocument(doc);
    if (!validate(error)) {
        config_ = previous;
        return false;
    }
    config_.configVersion++;
    config_.updatedAt = (uint32_t)time(nullptr);
    return save();
}

void ConfigManager::applyDefaults() {
    config_ = AppConfig{};
    config_.deviceId = platformDefaultDeviceId();
    config_.adminPassword = "admin";
}

void ConfigManager::fromDocument(JsonDocument& doc) {
    config_.deviceId = doc["deviceId"] | config_.deviceId;
    config_.deviceToken = doc["deviceToken"] | config_.deviceToken;
    config_.adminPassword = doc["adminPassword"] | config_.adminPassword;

    config_.wifiSsid = doc["wifi"]["ssid"] | config_.wifiSsid;
    config_.wifiPassword = doc["wifi"]["password"] | config_.wifiPassword;
    config_.apFallbackEnabled = true;
    config_.apPassword = doc["wifi"]["apPassword"] | config_.apPassword;

    config_.mqttHost = doc["mqtt"]["host"] | config_.mqttHost;
    config_.mqttPort = doc["mqtt"]["port"] | config_.mqttPort;
    config_.mqttTls = doc["mqtt"]["tls"] | config_.mqttTls;
    config_.mqttUser = doc["mqtt"]["user"] | config_.mqttUser;
    config_.mqttPassword = doc["mqtt"]["password"] | config_.mqttPassword;

    config_.weightEnabled = doc["weight"]["enabled"] | config_.weightEnabled;
    config_.hx711DoutPin = doc["weight"]["doutPin"] | config_.hx711DoutPin;
    config_.hx711SckPin = doc["weight"]["sckPin"] | config_.hx711SckPin;
    config_.calibrationFactor = doc["weight"]["calibrationFactor"] | config_.calibrationFactor;
    config_.tareOffset = doc["weight"]["tareOffset"] | config_.tareOffset;
    config_.weightThresholdKg = doc["weight"]["thresholdKg"] | config_.weightThresholdKg;
    config_.significantWeightChangeKg = doc["weight"]["significantChangeKg"] | config_.significantWeightChangeKg;

    config_.temperatureEnabled = doc["environment"]["temperatureEnabled"] | config_.temperatureEnabled;
    config_.humidityEnabled = doc["environment"]["humidityEnabled"] | config_.humidityEnabled;
    config_.thSensorType = doc["environment"]["combinedType"] | config_.thSensorType;
    config_.thSensorPin = doc["environment"]["combinedPin"] | config_.thSensorPin;
    config_.temperatureSensorType = doc["environment"]["temperatureType"] | config_.temperatureSensorType;
    config_.temperatureSensorPin = doc["environment"]["temperaturePin"] | config_.temperatureSensorPin;
    config_.humiditySensorType = doc["environment"]["humidityType"] | config_.humiditySensorType;
    config_.humiditySensorPin = doc["environment"]["humidityPin"] | config_.humiditySensorPin;

    config_.hallEnabled = doc["hall"]["enabled"] | config_.hallEnabled;
    config_.hallPin = doc["hall"]["pin"] | config_.hallPin;
    config_.hallOpenLevelHigh = doc["hall"]["openLevelHigh"] | config_.hallOpenLevelHigh;
    config_.hallWakeMode = parseHallWakeMode(doc["hall"]["wakeMode"] | toString(config_.hallWakeMode));

    config_.batteryEnabled = doc["battery"]["enabled"] | config_.batteryEnabled;
    config_.batteryAdcPin = doc["battery"]["adcPin"] | config_.batteryAdcPin;
    config_.batteryMinVoltage = doc["battery"]["minVoltage"] | config_.batteryMinVoltage;
    config_.batteryMaxVoltage = doc["battery"]["maxVoltage"] | config_.batteryMaxVoltage;
    config_.batteryDividerRatio = doc["battery"]["dividerRatio"] | config_.batteryDividerRatio;
    config_.adcReferenceVoltage = doc["battery"]["adcReferenceVoltage"] | config_.adcReferenceVoltage;
    config_.adcMaxValue = doc["battery"]["adcMaxValue"] | config_.adcMaxValue;

    config_.timeSource = parseTimeSource(doc["time"]["source"] | toString(config_.timeSource));
    config_.ntpServer = doc["time"]["ntpServer"] | config_.ntpServer;
    config_.timezoneOffsetMinutes = doc["time"]["timezoneOffsetMinutes"] | config_.timezoneOffsetMinutes;
    config_.rtcNtpSyncIntervalMinutes = doc["time"]["rtcNtpSyncIntervalMinutes"] | config_.rtcNtpSyncIntervalMinutes;

    config_.measurementIntervalSeconds = doc["measurementIntervalSeconds"] | config_.measurementIntervalSeconds;
    config_.deepSleepEnabled = doc["deepSleepEnabled"] | config_.deepSleepEnabled;
    config_.configVersion = doc["configVersion"] | config_.configVersion;
    config_.updatedAt = doc["updatedAt"] | config_.updatedAt;
}

void ConfigManager::toDocument(JsonDocument& doc, bool includeSecrets) const {
    doc["deviceId"] = config_.deviceId;
    if (includeSecrets) {
        doc["deviceToken"] = config_.deviceToken;
        doc["adminPassword"] = config_.adminPassword;
    }
    doc["configVersion"] = config_.configVersion;
    doc["updatedAt"] = config_.updatedAt;
    doc["measurementIntervalSeconds"] = config_.measurementIntervalSeconds;
    doc["deepSleepEnabled"] = config_.deepSleepEnabled;

    doc["wifi"]["ssid"] = config_.wifiSsid;
    if (includeSecrets) doc["wifi"]["password"] = config_.wifiPassword;
    doc["wifi"]["apFallbackEnabled"] = config_.apFallbackEnabled;
    if (includeSecrets) doc["wifi"]["apPassword"] = config_.apPassword;

    doc["mqtt"]["host"] = config_.mqttHost;
    doc["mqtt"]["port"] = config_.mqttPort;
    doc["mqtt"]["tls"] = config_.mqttTls;
    doc["mqtt"]["user"] = config_.mqttUser;
    if (includeSecrets) doc["mqtt"]["password"] = config_.mqttPassword;

    doc["weight"]["enabled"] = config_.weightEnabled;
    doc["weight"]["doutPin"] = config_.hx711DoutPin;
    doc["weight"]["sckPin"] = config_.hx711SckPin;
    doc["weight"]["calibrationFactor"] = config_.calibrationFactor;
    doc["weight"]["tareOffset"] = config_.tareOffset;
    doc["weight"]["thresholdKg"] = config_.weightThresholdKg;
    doc["weight"]["significantChangeKg"] = config_.significantWeightChangeKg;

    doc["environment"]["temperatureEnabled"] = config_.temperatureEnabled;
    doc["environment"]["humidityEnabled"] = config_.humidityEnabled;
    doc["environment"]["combinedType"] = config_.thSensorType;
    doc["environment"]["combinedPin"] = config_.thSensorPin;
    doc["environment"]["temperatureType"] = config_.temperatureSensorType;
    doc["environment"]["temperaturePin"] = config_.temperatureSensorPin;
    doc["environment"]["humidityType"] = config_.humiditySensorType;
    doc["environment"]["humidityPin"] = config_.humiditySensorPin;

    doc["hall"]["enabled"] = config_.hallEnabled;
    doc["hall"]["pin"] = config_.hallPin;
    doc["hall"]["openLevelHigh"] = config_.hallOpenLevelHigh;
    doc["hall"]["wakeMode"] = toString(config_.hallWakeMode);

    doc["battery"]["enabled"] = config_.batteryEnabled;
    doc["battery"]["adcPin"] = config_.batteryAdcPin;
    doc["battery"]["minVoltage"] = config_.batteryMinVoltage;
    doc["battery"]["maxVoltage"] = config_.batteryMaxVoltage;
    doc["battery"]["dividerRatio"] = config_.batteryDividerRatio;
    doc["battery"]["adcReferenceVoltage"] = config_.adcReferenceVoltage;
    doc["battery"]["adcMaxValue"] = config_.adcMaxValue;

    doc["time"]["source"] = toString(config_.timeSource);
    doc["time"]["ntpServer"] = config_.ntpServer;
    doc["time"]["timezoneOffsetMinutes"] = config_.timezoneOffsetMinutes;
    doc["time"]["rtcNtpSyncIntervalMinutes"] = config_.rtcNtpSyncIntervalMinutes;
}

bool ConfigManager::validate(String& error) const {
    if (config_.deviceId.length() == 0) {
        error = "deviceId is required";
        return false;
    }
    if (config_.apPassword.length() > 0 && config_.apPassword.length() < 8) {
        error = "AP password must be empty or at least 8 characters";
        return false;
    }
    if (config_.mqttPort == 0) {
        error = "MQTT port must be greater than 0";
        return false;
    }
    if (config_.measurementIntervalSeconds > 0 && config_.measurementIntervalSeconds < 10) {
        error = "Measurement interval must be 0 or at least 10 seconds";
        return false;
    }
    if (config_.calibrationFactor == 0.0f) {
        error = "Calibration factor must not be 0";
        return false;
    }
    if (config_.weightThresholdKg < 0.0f || config_.significantWeightChangeKg < 0.0f) {
        error = "Weight thresholds must be non-negative";
        return false;
    }
    if (config_.batteryMaxVoltage <= config_.batteryMinVoltage) {
        error = "Battery max voltage must be greater than min voltage";
        return false;
    }
    if (config_.batteryDividerRatio <= 0.0f || config_.adcReferenceVoltage <= 0.0f || config_.adcMaxValue <= 0) {
        error = "Battery ADC settings must be positive";
        return false;
    }
    return true;
}
