#pragma once

#include <Arduino.h>
#include <functional>

class TelemetryBuffer {
public:
    bool begin();
    bool append(const String& jsonLine);
    bool clear();
    size_t sizeBytes() const;
    uint16_t pendingCount() const;
    bool flushTo(std::function<bool(const String&)> sender, uint16_t maxLines = 20);

private:
    const char* path_ = "/telemetry-buffer.ndjson";
    const char* tempPath_ = "/telemetry-buffer.tmp";
    const size_t maxBytes_ = 180 * 1024;

    bool replaceWithTemp();
    void trimIfNeeded();
};
