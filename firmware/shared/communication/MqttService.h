#pragma once

#include <Arduino.h>
#include <PubSubClient.h>
#include <functional>
#include "config/AppConfig.h"
#include "platform/Platform.h"

class MqttService {
public:
    using MessageHandler = std::function<void(const String&, const String&)>;

    MqttService();

    void begin(const AppConfig& config);
    void setMessageHandler(MessageHandler handler);
    void loop(const AppConfig& config);
    bool connected();
    bool publishTelemetry(const AppConfig& config, const String& payload);
    bool publishEvent(const AppConfig& config, const String& payload);
    bool publishStatus(const AppConfig& config, const String& payload);

private:
    HivePlainClient plainClient_;
    HiveSecureClient secureClient_;
    PubSubClient mqtt_;
    MessageHandler messageHandler_;
    bool useTls_ = false;
    uint32_t lastConnectAttemptMs_ = 0;

    void handleMessage(char* topic, uint8_t* payload, unsigned int length);
    bool ensureConnected(const AppConfig& config);
    String topic(const AppConfig& config, const char* suffix) const;
};
