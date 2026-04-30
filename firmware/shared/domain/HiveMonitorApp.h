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
    float lastWeightKg_ = NAN;

    void setupFileSystem();
    void setupWifi();
    void startAccessPoint();
    void measureAndSend();
    String telemetryToJson(const Telemetry& telemetry) const;
    void publishOrBuffer(const String& payload);
    bool handleTare();
    bool handleCalibrate(float knownWeightKg);
};

