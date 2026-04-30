#include "time/TimeService.h"

void TimeService::begin(const AppConfig& config) {
    configTime(config.timezoneOffsetMinutes * 60, 0, config.ntpServer.c_str());
    configured_ = true;
    lastSyncMs_ = millis();
}

void TimeService::loop(const AppConfig& config) {
    uint32_t intervalMs = config.timeSource == TimeSource::Rtc
        ? config.rtcNtpSyncIntervalMinutes * 60000UL
        : 3600000UL;

    if (!configured_ || millis() - lastSyncMs_ > intervalMs) {
        begin(config);
    }
}

String TimeService::isoTimestamp() const {
    time_t nowSec = time(nullptr);
    if (nowSec < 1700000000) {
        return String(millis() / 1000);
    }

    struct tm timeinfo;
    gmtime_r(&nowSec, &timeinfo);
    char buffer[25];
    strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
    return String(buffer);
}

bool TimeService::hasValidTime() const {
    return time(nullptr) >= 1700000000;
}

