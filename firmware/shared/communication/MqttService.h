#pragma once

#include <Arduino.h>
#include <PubSubClient.h>
#include "config/AppConfig.h"
#include "platform/Platform.h"

class MqttService {
public:
    MqttService();

    void begin(const AppConfig& config);
    void loop(const AppConfig& config);
    bool connected() const;
    bool publishTelemetry(const AppConfig& config, const String& payload);
    bool publishEvent(const AppConfig& config, const String& payload);
    bool publishStatus(const AppConfig& config, const String& payload);

private:
    HivePlainClient plainClient_;
    HiveSecureClient secureClient_;
    PubSubClient mqtt_;
    bool useTls_ = false;
    uint32_t lastConnectAttemptMs_ = 0;

    bool ensureConnected(const AppConfig& config);
    String topic(const AppConfig& config, const char* suffix) const;
};

