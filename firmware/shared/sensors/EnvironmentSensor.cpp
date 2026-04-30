#include "sensors/EnvironmentSensor.h"

namespace {
uint8_t parseDhtType(const String& type) {
    if (type == "DHT11") return DHT11;
    if (type == "DHT21") return DHT21;
    return DHT22;
}
}

EnvironmentSensor::~EnvironmentSensor() {
    delete dht_;
}

void EnvironmentSensor::begin(const AppConfig& config) {
    updateConfig(config);
}

void EnvironmentSensor::updateConfig(const AppConfig& config) {
    uint8_t nextType = parseDhtType(config.thSensorType);
    if (!dht_ || config.thSensorPin != pin_ || nextType != type_) {
        recreateDht(config.thSensorPin, nextType);
    }
}

float EnvironmentSensor::readTemperature() {
    return dht_ ? dht_->readTemperature() : NAN;
}

float EnvironmentSensor::readHumidity() {
    return dht_ ? dht_->readHumidity() : NAN;
}

void EnvironmentSensor::recreateDht(int pin, uint8_t type) {
    delete dht_;
    pin_ = pin;
    type_ = type;
    dht_ = new DHT(pin_, type_);
    dht_->begin();
}

