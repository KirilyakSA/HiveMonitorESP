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

    String kept;
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
            kept += line;
            kept += '\n';
        }
        yield();
    }
    file.close();

    File out = HIVE_FS.open(path_, "w");
    if (!out) return false;
    out.print(kept);
    out.close();
    return allSent;
}

void TelemetryBuffer::trimIfNeeded() {
    if (sizeBytes() <= maxBytes_) return;

    File file = HIVE_FS.open(path_, "r");
    if (!file) return;

    String kept;
    bool dropping = true;
    while (file.available()) {
        String line = file.readStringUntil('\n');
        if (dropping) {
            if (line.indexOf("\"timestamp\"") >= 0) {
                dropping = false;
            }
            continue;
        }
        kept += line;
        kept += '\n';
        yield();
    }
    file.close();

    File out = HIVE_FS.open(path_, "w");
    if (!out) return;
    out.print(kept);
    out.close();
}

