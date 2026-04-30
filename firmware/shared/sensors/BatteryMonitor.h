#pragma once

#include <Arduino.h>
#include "config/AppConfig.h"

struct BatteryState {
    float voltage = NAN;
    int percent = -1;
};

class BatteryMonitor {
public:
    void begin(const AppConfig& config);
    BatteryState read(const AppConfig& config);
};

