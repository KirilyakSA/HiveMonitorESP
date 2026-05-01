#pragma once

#include <Arduino.h>
#include "communication/MqttService.h"
#include "config/ConfigManager.h"
#include "domain/Telemetry.h"
#include "sensors/BatteryMonitor.h"
#include "sensors/EnvironmentSensor.h"
#include "sensors/HallSensor.h"
#include "sensors/LoadCell.h"
#include "storage/TelemetryBuffer.h"
#include "time/TimeService.h"
#include "web/WebPortal.h"

class HiveMonitorApp {
public:
    HiveMonitorApp();

    void setup();
    void loop();

private:
    ConfigManager configManager_;
    TimeService timeService_;
    MqttService mqttService_;
    TelemetryBuffer telemetryBuffer_;
    LoadCell loadCell_;
    EnvironmentSensor environmentSensor_;
    BatteryMonitor batteryMonitor_;
    HallSensor hallSensor_;
    WebPortal webPortal_;
    Telemetry latestTelemetry_;

    uint32_t lastMeasureMs_ = 0;
    uint32_t appliedConfigVersion_ = 0;
    float lastWeightKg_ = NAN;
    bool accessPointActive_ = false;
    String appliedWifiSsid_;
    String appliedWifiPassword_;
    bool appliedApFallbackEnabled_ = false;
    String appliedApPassword_;
    bool lastPublishOk_ = false;
    String lastPublishMessage_ = "No telemetry published yet";

    void setupFileSystem();
    bool handleFactoryResetButton();
    void setupWifi();
    void startAccessPoint();
    void applyConfigChanges();
    void rememberWifiConfig();
    bool wifiConfigChanged() const;
    void measureAndSend();
    void enterDeepSleepIfEnabled();
    bool waitForTelemetryDelivery(uint32_t timeoutMs);
    void handleMqttMessage(const String& topic, const String& payload);
    bool handleMqttCommand(const String& command, JsonVariantConst data, String& message);
    bool handleMqttConfigUpdate(JsonVariantConst data, String& message);
    void publishCommandStatus(const String& command, bool ok, const String& message);
    String telemetryToJson(const Telemetry& telemetry) const;
    void publishOrBuffer(const String& payload);
    bool mqttConnected();
    bool lastPublishOk() const;
    String lastPublishMessage() const;
    bool handleTare();
    bool handleCalibrate(float knownWeightKg);
};
