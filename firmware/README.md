# Firmware

Код прошивок ESP8266/ESP32 для HiveMonitor.

## Сборка

Основной инструмент сборки - PlatformIO.

```powershell
cd firmware
pio run -e esp8266
pio run -e esp32
```

## Структура

```text
firmware/
  platformio.ini
  esp8266/src/main.cpp
  esp32/src/main.cpp
  shared/
    communication/   MQTT
    config/          JSON-конфигурация устройства
    domain/          основной цикл приложения и телеметрия
    platform/        отличия ESP8266/ESP32
    sensors/         HX711, DHT, Hall, батарея
    storage/         LittleFS-буфер телеметрии
    time/            NTP/RTC-ready слой времени
    web/             локальный web-интерфейс и web-обновление
```

## Реализовано в текущем каркасе

- две сборки PlatformIO: `esp8266` и `esp32`;
- общий код прошивки с платформенными отличиями в `shared/platform`;
- локальный web-интерфейс с Basic Auth;
- настройка Wi-Fi, MQTT, TLS on/off, пинов, периода измерений, порогов и калибровки;
- локальное web-обновление прошивки через `/update`;
- HX711 через `GyverHX711`;
- DHT11/DHT21/DHT22;
- датчик открытия улья;
- измерение батареи через ADC;
- JSON-телеметрия в MQTT-топик `hives/{deviceId}/telemetry`;
- события в `hives/{deviceId}/events`;
- буферизация телеметрии в LittleFS при отсутствии связи.

Детальная документация: [firmware/docs](docs/README.md).
