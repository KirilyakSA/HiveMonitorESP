#pragma once

#include <Arduino.h>

struct Telemetry {
    String deviceId;
    String timestamp;
    float weight = NAN;
    float weightChange = NAN;
    float temperature = NAN;
    float humidity = NAN;
    bool hiveOpened = false;
    bool errorFlag = false;
    int batteryPercent = -1;
    float batteryVoltage = NAN;
    int rssi = 0;
};

