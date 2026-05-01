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
