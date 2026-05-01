#include "sensors/EnvironmentSensor.h"

namespace {
uint8_t parseDhtType(const String& type) {
    if (type == "DHT11") return DHT11;
    if (type == "DHT21") return DHT21;
    return DHT22;
}
}

EnvironmentSensor::~EnvironmentSensor() {
    delete temperatureDht_;
    if (!sharedDht_) {
        delete humidityDht_;
    }
}

void EnvironmentSensor::begin(const AppConfig& config) {
    updateConfig(config);
}

void EnvironmentSensor::updateConfig(const AppConfig& config) {
    int nextTemperaturePin = config.temperatureSensorPin;
    int nextHumidityPin = config.humiditySensorPin;
    uint8_t nextTemperatureType = parseDhtType(config.temperatureSensorType);
    uint8_t nextHumidityType = parseDhtType(config.humiditySensorType);

    if (nextTemperaturePin < 0) {
        nextTemperaturePin = config.thSensorPin;
        nextTemperatureType = parseDhtType(config.thSensorType);
    }
    if (nextHumidityPin < 0) {
        nextHumidityPin = config.thSensorPin;
        nextHumidityType = parseDhtType(config.thSensorType);
    }

    if (!temperatureDht_ ||
        nextTemperaturePin != temperaturePin_ ||
        nextHumidityPin != humidityPin_ ||
        nextTemperatureType != temperatureType_ ||
        nextHumidityType != humidityType_) {
        recreateDhts(nextTemperaturePin, nextTemperatureType, nextHumidityPin, nextHumidityType);
    }
}

float EnvironmentSensor::readTemperature() {
    return temperatureDht_ ? temperatureDht_->readTemperature() : NAN;
}

float EnvironmentSensor::readHumidity() {
    return humidityDht_ ? humidityDht_->readHumidity() : NAN;
}

void EnvironmentSensor::recreateDhts(int temperaturePin, uint8_t temperatureType, int humidityPin, uint8_t humidityType) {
    delete temperatureDht_;
    if (!sharedDht_) {
        delete humidityDht_;
    }

    temperaturePin_ = temperaturePin;
    humidityPin_ = humidityPin;
    temperatureType_ = temperatureType;
    humidityType_ = humidityType;
    sharedDht_ = temperaturePin_ == humidityPin_ && temperatureType_ == humidityType_;

    temperatureDht_ = new DHT(temperaturePin_, temperatureType_);
    temperatureDht_->begin();
    if (sharedDht_) {
        humidityDht_ = temperatureDht_;
    } else {
        humidityDht_ = new DHT(humidityPin_, humidityType_);
        humidityDht_->begin();
    }
}
