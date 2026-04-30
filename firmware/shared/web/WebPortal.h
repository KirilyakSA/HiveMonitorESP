#pragma once

#include <Arduino.h>
#include <functional>
#include "config/ConfigManager.h"
#include "domain/Telemetry.h"
#include "platform/Platform.h"

class WebPortal {
public:
    using TareHandler = std::function<bool()>;
    using CalibrateHandler = std::function<bool(float)>;
    using ClearBufferHandler = std::function<bool()>;

    WebPortal(ConfigManager& configManager);

    void begin(
        const Telemetry* latestTelemetry,
        TareHandler onTare,
        CalibrateHandler onCalibrate,
        ClearBufferHandler onClearBuffer
    );
    void loop();

private:
    HiveWebServer server_;
    ConfigManager& configManager_;
    const Telemetry* latestTelemetry_ = nullptr;
    TareHandler onTare_;
    CalibrateHandler onCalibrate_;
    ClearBufferHandler onClearBuffer_;

    bool authenticated();
    void sendUnauthorized();
    void handleIndex();
    void handleConfigGet();
    void handleConfigPost();
    void handleStatus();
    void handleTare();
    void handleCalibrate();
    void handleClearBuffer();
    void handleResetConfig();
    void handleRestart();
    void handleUpdateForm();
    void handleUpdateUpload();
};
