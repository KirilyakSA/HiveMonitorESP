#include "sensors/BatteryMonitor.h"

void BatteryMonitor::begin(const AppConfig& config) {
    pinMode(config.batteryAdcPin, INPUT);
}

BatteryState BatteryMonitor::read(const AppConfig& config) {
    BatteryState state;
    int raw = analogRead(config.batteryAdcPin);
    state.voltage = ((float)raw / (float)config.adcMaxValue) * config.adcReferenceVoltage * config.batteryDividerRatio;

    float range = config.batteryMaxVoltage - config.batteryMinVoltage;
    if (range <= 0.0f) {
        state.percent = 0;
        return state;
    }

    float normalized = (state.voltage - config.batteryMinVoltage) / range;
    normalized = constrain(normalized, 0.0f, 1.0f);
    state.percent = (int)roundf(normalized * 100.0f);
    return state;
}

