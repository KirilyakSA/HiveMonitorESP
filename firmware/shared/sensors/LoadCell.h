#pragma once

#include <Arduino.h>
#include <GyverHX711.h>
#include "config/AppConfig.h"

class LoadCell {
public:
    ~LoadCell();

    void begin(const AppConfig& config);
    void updateConfig(const AppConfig& config);
    bool available();
    float readKg(uint8_t samples = 5);
    long readRawAverage(uint8_t samples = 5, uint32_t timeoutMs = 1200);
    long tare(uint8_t samples = 10);
    float calibrate(float knownWeightKg, uint8_t samples = 10);
    long tareOffset() const;

private:
    GyverHX711* sensor_ = nullptr;
    int doutPin_ = -1;
    int sckPin_ = -1;
    float calibrationFactor_ = 1.0f;
    long tareOffset_ = 0;

    void recreateSensor(int doutPin, int sckPin);
};

