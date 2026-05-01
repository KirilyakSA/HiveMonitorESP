#include "storage/TelemetryBuffer.h"
#include "platform/Platform.h"

bool TelemetryBuffer::begin() {
    if (!HIVE_FS.exists(path_)) {
        File file = HIVE_FS.open(path_, "w");
        if (!file) return false;
        file.close();
    }
    return true;
}

bool TelemetryBuffer::append(const String& jsonLine) {
    trimIfNeeded();
    File file = HIVE_FS.open(path_, "a");
    if (!file) return false;
    file.println(jsonLine);
    file.close();
    trimIfNeeded();
    return true;
}

bool TelemetryBuffer::clear() {
    if (HIVE_FS.exists(path_)) HIVE_FS.remove(path_);
    return begin();
}

size_t TelemetryBuffer::sizeBytes() const {
    File file = HIVE_FS.open(path_, "r");
    if (!file) return 0;
    size_t size = file.size();
    file.close();
    return size;
}

uint16_t TelemetryBuffer::pendingCount() const {
    File file = HIVE_FS.open(path_, "r");
    if (!file) return 0;
    uint16_t count = 0;
    while (file.available()) {
        if (file.readStringUntil('\n').length() > 0) count++;
        yield();
    }
    file.close();
    return count;
}

bool TelemetryBuffer::flushTo(std::function<bool(const String&)> sender, uint16_t maxLines) {
    File file = HIVE_FS.open(path_, "r");
    if (!file) return false;

    HIVE_FS.remove(tempPath_);
    File out = HIVE_FS.open(tempPath_, "w");
    if (!out) {
        file.close();
        return false;
    }

    uint16_t sent = 0;
    bool allSent = true;

    while (file.available()) {
        String line = file.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) continue;

        if (sent < maxLines && sender(line)) {
            sent++;
        } else {
            allSent = false;
            out.println(line);
        }
        yield();
    }
    file.close();
    out.close();

    return replaceWithTemp() && allSent;
}

bool TelemetryBuffer::replaceWithTemp() {
    HIVE_FS.remove(path_);
    return HIVE_FS.rename(tempPath_, path_);
}

void TelemetryBuffer::trimIfNeeded() {
    size_t currentSize = sizeBytes();
    if (currentSize <= maxBytes_) return;

    File file = HIVE_FS.open(path_, "r");
    if (!file) return;

    HIVE_FS.remove(tempPath_);
    File out = HIVE_FS.open(tempPath_, "w");
    if (!out) {
        file.close();
        return;
    }

    const size_t targetSize = maxBytes_ * 3 / 4;
    size_t bytesToDrop = currentSize > targetSize ? currentSize - targetSize : 0;
    size_t dropped = 0;

    while (file.available()) {
        String line = file.readStringUntil('\n');
        size_t lineBytes = line.length() + 1;
        if (dropped < bytesToDrop) {
            dropped += lineBytes;
            continue;
        }
        line.trim();
        if (line.length() > 0) out.println(line);
        yield();
    }
    file.close();
    out.close();

    replaceWithTemp();
}
