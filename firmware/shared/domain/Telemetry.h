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
    bool weightError = false;
    bool temperatureError = false;
    bool humidityError = false;
    bool batteryError = false;
    int batteryPercent = -1;
    float batteryVoltage = NAN;
    int rssi = 0;
};
