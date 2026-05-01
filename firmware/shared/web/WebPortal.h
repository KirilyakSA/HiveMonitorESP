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
    using BufferPendingHandler = std::function<uint16_t()>;
    using BufferSizeHandler = std::function<size_t()>;
    using BoolStatusHandler = std::function<bool()>;
    using StringStatusHandler = std::function<String()>;

    WebPortal(ConfigManager& configManager);

    void begin(
        const Telemetry* latestTelemetry,
        TareHandler onTare,
        CalibrateHandler onCalibrate,
        ClearBufferHandler onClearBuffer,
        BufferPendingHandler onBufferPending,
        BufferSizeHandler onBufferSize,
        BoolStatusHandler onMqttConnected,
        BoolStatusHandler onLastPublishOk,
        StringStatusHandler onLastPublishMessage
    );
    void loop();

private:
    HiveWebServer server_;
    ConfigManager& configManager_;
    const Telemetry* latestTelemetry_ = nullptr;
    TareHandler onTare_;
    CalibrateHandler onCalibrate_;
    ClearBufferHandler onClearBuffer_;
    BufferPendingHandler onBufferPending_;
    BufferSizeHandler onBufferSize_;
    BoolStatusHandler onMqttConnected_;
    BoolStatusHandler onLastPublishOk_;
    StringStatusHandler onLastPublishMessage_;
    bool updateInProgress_ = false;
    bool updateOk_ = false;
    size_t updateBytes_ = 0;
    String updateError_;

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
    void failUpdate(const String& message);
};
