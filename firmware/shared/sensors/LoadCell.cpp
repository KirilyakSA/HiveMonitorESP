#include "sensors/LoadCell.h"
#include <limits.h>

LoadCell::~LoadCell() {
    delete sensor_;
}

void LoadCell::begin(const AppConfig& config) {
    updateConfig(config);
}

void LoadCell::updateConfig(const AppConfig& config) {
    calibrationFactor_ = config.calibrationFactor == 0.0f ? 1.0f : config.calibrationFactor;
    tareOffset_ = config.tareOffset;
    if (!sensor_ || config.hx711DoutPin != doutPin_ || config.hx711SckPin != sckPin_) {
        recreateSensor(config.hx711DoutPin, config.hx711SckPin);
    }
    if (sensor_) {
        sensor_->setOffset(tareOffset_);
    }
}

bool LoadCell::available() {
    return sensor_ && sensor_->available();
}

float LoadCell::readKg(uint8_t samples) {
    if (!sensor_) return NAN;
    long raw = readRawAverage(samples);
    if (raw == LONG_MIN) return NAN;
    return (raw - tareOffset_) / calibrationFactor_;
}

long LoadCell::readRawAverage(uint8_t samples, uint32_t timeoutMs) {
    if (!sensor_ || samples == 0) return LONG_MIN;

    long sum = 0;
    uint8_t count = 0;
    uint32_t started = millis();

    while (count < samples && millis() - started < timeoutMs) {
        if (sensor_->available()) {
            sum += sensor_->read();
            count++;
        }
        delay(2);
        yield();
    }

    if (count == 0) return LONG_MIN;
    return sum / count;
}

long LoadCell::tare(uint8_t samples) {
    long raw = readRawAverage(samples, 2000);
    if (raw != LONG_MIN) {
        tareOffset_ = raw;
        if (sensor_) sensor_->setOffset(tareOffset_);
    }
    return tareOffset_;
}

float LoadCell::calibrate(float knownWeightKg, uint8_t samples) {
    if (knownWeightKg == 0.0f) return calibrationFactor_;
    long raw = readRawAverage(samples, 2500);
    if (raw != LONG_MIN) {
        calibrationFactor_ = (raw - tareOffset_) / knownWeightKg;
        if (calibrationFactor_ == 0.0f) calibrationFactor_ = 1.0f;
    }
    return calibrationFactor_;
}

long LoadCell::tareOffset() const {
    return tareOffset_;
}

void LoadCell::recreateSensor(int doutPin, int sckPin) {
    delete sensor_;
    doutPin_ = doutPin;
    sckPin_ = sckPin;
    sensor_ = new GyverHX711(doutPin_, sckPin_, HX_GAIN64_A);
    delay(250);
    sensor_->setOffset(tareOffset_);
}
