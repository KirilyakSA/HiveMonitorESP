#pragma once

#include <Arduino.h>
#include "config/AppConfig.h"

class HallSensor {
public:
    void begin(const AppConfig& config);
    bool isOpen(const AppConfig& config) const;
};

