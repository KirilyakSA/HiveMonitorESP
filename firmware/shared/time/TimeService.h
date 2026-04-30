#pragma once

#include <Arduino.h>
#include <WiFiUdp.h>
#include <time.h>
#include "config/AppConfig.h"

class TimeService {
public:
    void begin(const AppConfig& config);
    void loop(const AppConfig& config);
    String isoTimestamp() const;
    bool hasValidTime() const;

private:
    uint32_t lastSyncMs_ = 0;
    bool configured_ = false;
};

