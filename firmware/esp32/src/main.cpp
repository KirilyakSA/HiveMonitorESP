#include <Arduino.h>
#include "domain/HiveMonitorApp.h"

HiveMonitorApp app;

void setup() {
    app.setup();
}

void loop() {
    app.loop();
}

