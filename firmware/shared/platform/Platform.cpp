#include "platform/Platform.h"

String platformChipId() {
#if defined(HIVE_PLATFORM_ESP8266)
    return String(ESP.getChipId(), HEX);
#else
    uint64_t mac = ESP.getEfuseMac();
    char id[17];
    snprintf(id, sizeof(id), "%04X%08X", (uint16_t)(mac >> 32), (uint32_t)mac);
    return String(id);
#endif
}

String platformDefaultDeviceId() {
    String id = "hive-";
    id += platformChipId();
    id.toLowerCase();
    return id;
}

int platformDefaultAdcPin() {
#if defined(HIVE_PLATFORM_ESP8266)
    return A0;
#else
    return 34;
#endif
}

int platformDefaultHx711DoutPin() {
#if defined(HIVE_PLATFORM_ESP8266)
    return D2;
#else
    return 18;
#endif
}

int platformDefaultHx711SckPin() {
#if defined(HIVE_PLATFORM_ESP8266)
    return D3;
#else
    return 19;
#endif
}

int platformDefaultEnvironmentPin() {
#if defined(HIVE_PLATFORM_ESP8266)
    return D4;
#else
    return 4;
#endif
}

int platformDefaultHallPin() {
#if defined(HIVE_PLATFORM_ESP8266)
    return D5;
#else
    return 5;
#endif
}

int platformDefaultFactoryResetPin() {
#if defined(HIVE_PLATFORM_ESP8266)
    return D6;
#else
    return 12;
#endif
}

void platformRestart() {
    ESP.restart();
}

void platformDeepSleepSeconds(uint32_t seconds) {
#if defined(HIVE_PLATFORM_ESP8266)
    ESP.deepSleep((uint64_t)seconds * 1000000ULL, WAKE_RF_DEFAULT);
#else
    esp_sleep_enable_timer_wakeup((uint64_t)seconds * 1000000ULL);
    esp_deep_sleep_start();
#endif
}

bool platformHttpFirmwareUpdate(const String& artifactUrl, String& message) {
    if (artifactUrl.length() == 0) {
        message = "Firmware artifact URL is empty";
        return false;
    }
    bool secure = artifactUrl.startsWith("https://");
    bool plain = artifactUrl.startsWith("http://");
    if (!secure && !plain) {
        message = "Firmware artifact URL must start with http:// or https://";
        return false;
    }

#if defined(HIVE_PLATFORM_ESP8266)
    ESPhttpUpdate.rebootOnUpdate(false);
    t_httpUpdate_return result;
    if (secure) {
        HiveSecureClient client;
        client.setInsecure();
        result = ESPhttpUpdate.update(client, artifactUrl);
    } else {
        HivePlainClient client;
        result = ESPhttpUpdate.update(client, artifactUrl);
    }

    if (result == HTTP_UPDATE_OK) {
        message = "Firmware update downloaded; restarting";
        return true;
    }
    if (result == HTTP_UPDATE_NO_UPDATES) {
        message = "No firmware update available";
        return false;
    }
    message = "Firmware update failed: " + ESPhttpUpdate.getLastErrorString();
    return false;
#else
    httpUpdate.rebootOnUpdate(false);
    t_httpUpdate_return result;
    if (secure) {
        HiveSecureClient client;
        client.setInsecure();
        result = httpUpdate.update(client, artifactUrl);
    } else {
        HivePlainClient client;
        result = httpUpdate.update(client, artifactUrl);
    }

    if (result == HTTP_UPDATE_OK) {
        message = "Firmware update downloaded; restarting";
        return true;
    }
    if (result == HTTP_UPDATE_NO_UPDATES) {
        message = "No firmware update available";
        return false;
    }
    message = "Firmware update failed: " + httpUpdate.getLastErrorString();
    return false;
#endif
}
