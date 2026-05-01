#pragma once

#include <Arduino.h>
#include <DHT.h>
#include "config/AppConfig.h"

class EnvironmentSensor {
public:
    ~EnvironmentSensor();

    void begin(const AppConfig& config);
    void updateConfig(const AppConfig& config);
    float readTemperature();
    float readHumidity();

private:
    DHT* temperatureDht_ = nullptr;
    DHT* humidityDht_ = nullptr;
    int temperaturePin_ = -1;
    int humidityPin_ = -1;
    uint8_t temperatureType_ = DHT22;
    uint8_t humidityType_ = DHT22;
    bool sharedDht_ = false;

    void recreateDhts(int temperaturePin, uint8_t temperatureType, int humidityPin, uint8_t humidityType);
};
