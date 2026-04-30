#include "sensors/HallSensor.h"

void HallSensor::begin(const AppConfig& config) {
    pinMode(config.hallPin, INPUT_PULLUP);
}

bool HallSensor::isOpen(const AppConfig& config) const {
    int level = digitalRead(config.hallPin);
    return config.hallOpenLevelHigh ? level == HIGH : level == LOW;
}

