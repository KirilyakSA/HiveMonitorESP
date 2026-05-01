#include "communication/MqttService.h"

MqttService::MqttService() : mqtt_(plainClient_) {
}

void MqttService::begin(const AppConfig& config) {
    useTls_ = config.mqttTls;
    if (useTls_) {
        secureClient_.setInsecure();
        mqtt_.setClient(secureClient_);
    } else {
        mqtt_.setClient(plainClient_);
    }
    mqtt_.setServer(config.mqttHost.c_str(), config.mqttPort);
    mqtt_.setBufferSize(1024);
    mqtt_.setCallback([this](char* topic, uint8_t* payload, unsigned int length) {
        handleMessage(topic, payload, length);
    });
}

void MqttService::setMessageHandler(MessageHandler handler) {
    messageHandler_ = handler;
}

void MqttService::loop(const AppConfig& config) {
    if (WiFi.status() != WL_CONNECTED) return;
    if (!mqtt_.connected()) {
        ensureConnected(config);
    }
    mqtt_.loop();
}

bool MqttService::connected() {
    return mqtt_.connected();
}

bool MqttService::publishTelemetry(const AppConfig& config, const String& payload) {
    if (!ensureConnected(config)) return false;
    return mqtt_.publish(topic(config, "telemetry").c_str(), payload.c_str(), false);
}

bool MqttService::publishEvent(const AppConfig& config, const String& payload) {
    if (!ensureConnected(config)) return false;
    return mqtt_.publish(topic(config, "events").c_str(), payload.c_str(), false);
}

bool MqttService::publishStatus(const AppConfig& config, const String& payload) {
    if (!ensureConnected(config)) return false;
    return mqtt_.publish(topic(config, "status").c_str(), payload.c_str(), true);
}

void MqttService::handleMessage(char* topic, uint8_t* payload, unsigned int length) {
    if (!messageHandler_) return;

    String topicString(topic);
    String payloadString;
    payloadString.reserve(length);
    for (unsigned int i = 0; i < length; i++) {
        payloadString += (char)payload[i];
    }
    messageHandler_(topicString, payloadString);
}

bool MqttService::ensureConnected(const AppConfig& config) {
    if (mqtt_.connected()) return true;
    if (config.mqttHost.length() == 0 || config.deviceId.length() == 0) return false;
    if (millis() - lastConnectAttemptMs_ < 5000) return false;
    lastConnectAttemptMs_ = millis();

    if (useTls_ != config.mqttTls) {
        begin(config);
    }

    String clientId = config.deviceId + "-" + platformChipId();
    bool ok;
    if (config.deviceToken.length() > 0) {
        const char* user = config.mqttUser.length() > 0 ? config.mqttUser.c_str() : config.deviceId.c_str();
        const char* pass = config.mqttPassword.length() > 0 ? config.mqttPassword.c_str() : config.deviceToken.c_str();
        ok = mqtt_.connect(clientId.c_str(), user, pass);
    } else {
        ok = mqtt_.connect(clientId.c_str());
    }

    if (ok) {
        mqtt_.subscribe(topic(config, "commands").c_str());
        mqtt_.subscribe(topic(config, "config").c_str());
    }
    return ok;
}

String MqttService::topic(const AppConfig& config, const char* suffix) const {
    String value = "hives/";
    value += config.deviceId;
    value += "/";
    value += suffix;
    return value;
}
