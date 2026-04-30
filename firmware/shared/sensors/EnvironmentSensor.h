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
    DHT* dht_ = nullptr;
    int pin_ = -1;
    uint8_t type_ = DHT22;

    void recreateDht(int pin, uint8_t type);
};

