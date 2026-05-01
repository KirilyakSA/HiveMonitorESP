#pragma once

#include <ArduinoJson.h>
#include "config/AppConfig.h"

class ConfigManager {
public:
    bool begin();
    AppConfig& data();
    const AppConfig& data() const;
    bool save();
    bool resetDefaults();
    String toJson(bool includeSecrets) const;
    bool updateFromJson(const String& body, String& error);

private:
    AppConfig config_;
    void applyDefaults();
    void fromDocument(JsonDocument& doc);
    void toDocument(JsonDocument& doc, bool includeSecrets) const;
    bool validate(String& error) const;
};
