#pragma once

#include <Arduino.h>
#include "platform/Platform.h"

enum class TimeSource : uint8_t {
    Ntp,
    Rtc,
    Auto
};

enum class HallWakeMode : uint8_t {
    WakeByReset,
    NoWake
};

struct AppConfig {
    String deviceId;
    String deviceToken;
    String adminPassword;

    String wifiSsid;
    String wifiPassword;
    bool apFallbackEnabled = true;
    String apPassword = "hivemonitor";

    String mqttHost = "192.168.1.10";
    uint16_t mqttPort = 1883;
    bool mqttTls = false;
    String mqttUser;
    String mqttPassword;

    bool weightEnabled = true;
    int hx711DoutPin = platformDefaultHx711DoutPin();
    int hx711SckPin = platformDefaultHx711SckPin();
    float calibrationFactor = 1.0f;
    long tareOffset = 0;

    bool temperatureEnabled = true;
    bool humidityEnabled = true;
    String thSensorType = "DHT22";
    int thSensorPin = platformDefaultEnvironmentPin();
    String temperatureSensorType = "DHT22";
    int temperatureSensorPin = platformDefaultEnvironmentPin();
    String humiditySensorType = "DHT22";
    int humiditySensorPin = platformDefaultEnvironmentPin();

    bool hallEnabled = true;
    int hallPin = platformDefaultHallPin();
    bool hallOpenLevelHigh = false;
    HallWakeMode hallWakeMode = HallWakeMode::NoWake;

    bool batteryEnabled = true;
    int batteryAdcPin = platformDefaultAdcPin();
    float batteryMinVoltage = 3.2f;
    float batteryMaxVoltage = 4.2f;
    float batteryDividerRatio = 1.0f;
    float adcReferenceVoltage = 3.3f;
    int adcMaxValue = 1023;

    TimeSource timeSource = TimeSource::Ntp;
    String ntpServer = "pool.ntp.org";
    int timezoneOffsetMinutes = 0;
    uint32_t rtcNtpSyncIntervalMinutes = 1440;

    uint32_t measurementIntervalSeconds = 1800;
    bool deepSleepEnabled = false;

    float weightThresholdKg = 5.0f;
    float significantWeightChangeKg = 20.0f;

    uint32_t configVersion = 1;
    uint32_t updatedAt = 0;
};
