#pragma once

#include <Arduino.h>

#if defined(HIVE_PLATFORM_ESP8266)
  #include <ESP8266WiFi.h>
  #include <ESP8266WebServer.h>
  #include <ESP8266HTTPUpdateServer.h>
  #include <LittleFS.h>
  #include <WiFiClientSecureBearSSL.h>
  using HiveWebServer = ESP8266WebServer;
  using HivePlainClient = WiFiClient;
  using HiveSecureClient = BearSSL::WiFiClientSecure;
  #define HIVE_FS LittleFS
#elif defined(HIVE_PLATFORM_ESP32)
  #include <WiFi.h>
  #include <WebServer.h>
  #include <Update.h>
  #include <LittleFS.h>
  #include <WiFiClientSecure.h>
  using HiveWebServer = WebServer;
  using HivePlainClient = WiFiClient;
  using HiveSecureClient = WiFiClientSecure;
  #define HIVE_FS LittleFS
#else
  #error "Unsupported HiveMonitor platform"
#endif

#ifndef HIVE_FW_VERSION
#define HIVE_FW_VERSION "dev"
#endif

String platformChipId();
String platformDefaultDeviceId();
int platformDefaultAdcPin();
int platformDefaultHx711DoutPin();
int platformDefaultHx711SckPin();
int platformDefaultEnvironmentPin();
int platformDefaultHallPin();
int platformDefaultFactoryResetPin();
void platformRestart();
void platformDeepSleepSeconds(uint32_t seconds);
