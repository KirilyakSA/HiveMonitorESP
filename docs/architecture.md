# Архитектура HiveMonitor

## Общая модель

HiveMonitor - система мониторинга пасек и ульев. Главный пользовательский объект - пасека и улей, а IoT-устройство является заменяемым источником данных.

```text
Organization / аккаунт
  -> пользователи и роли
  -> пасеки
      -> ульи
          -> устройства
          -> телеметрия
          -> события
          -> задачи
          -> теги
      -> метеостанция / погодный provider
      -> уведомления
      -> схема расположения ульев
```

## Компоненты

```text
ESP8266/ESP32 firmware
  -> MQTT broker
  -> mqtt-ingestion-service
  -> PostgreSQL
  -> api-service
  -> Web UI / Mobile apps

NATS + JetStream
  -> async events between backend services
```

## Firmware

Прошивка отвечает за:

- измерение веса;
- измерение температуры и влажности, если датчики включены;
- фиксацию открытия улья датчиком Холла;
- измерение батареи;
- локальную настройку через web-интерфейс;
- локальный буфер телеметрии в LittleFS;
- публикацию телеметрии и событий в MQTT;
- прием MQTT-команд `reboot`, `firmware_update`, `config_update`;
- локальную firmware tare/калибровку весов через web-интерфейс устройства;
- backend-тару улья/магазинов как параметры улья, которые вычитаются из сырого веса в API/UI;
- локальное web-обновление прошивки.

Текущая firmware публикует legacy topics:

```text
hives/{deviceId}/telemetry
hives/{deviceId}/events
hives/{deviceId}/status
```

И подписывается на:

```text
hives/{deviceId}/commands
hives/{deviceId}/config
```

## Backend

Backend реализуется на Go и PostgreSQL.

Текущие процессы:

- `api-service` - REST API для web/mobile;
- `mqtt-ingestion-service` - прием MQTT-телеметрии.

Текущая backend-реализация поддерживает legacy и apiary-aware topics:

```text
hives/+/telemetry
apiaries/+/devices/+/telemetry
hives/+/events
apiaries/+/devices/+/events
hives/+/status
apiaries/+/devices/+/status
```

`hives/+/...` нужен для совместимости с legacy firmware mode.

`apiaries/+/devices/+/...` - целевой topic namespace для provisioning по пасеке: backend получает `apiary_id` из topic и может автоматически показать устройство в списке непривязанных устройств пасеки.

## MQTT и provisioning

Последнее продуктовое требование: MQTT credentials выдаются на уровне пасеки. После factory reset устройство поднимает собственную AP, пользователь вводит Wi-Fi и MQTT credentials пасеки через локальный web-интерфейс устройства.

Текущее состояние:

- firmware умеет хранить `mqttUser` и `mqttPassword`, что совместимо с credentials пасеки;
- firmware имеет настройку `apiaryId`;
- если `apiaryId` задан, firmware публикует `apiaries/{apiary_id}/devices/{device_id}/telemetry|events|status`;
- если `apiaryId` пустой, firmware остается в legacy mode `hives/{deviceId}/telemetry|events|status`;
- backend принимает legacy topics только для dev/MVP с `DEFAULT_APIARY_ID` или для уже зарегистрированных устройств;
- production path должен использовать apiary-aware topics.

Поле `deviceToken` в firmware остается legacy/fallback секретом локальной конфигурации. В актуальной backend-архитектуре MVP оно не является основным способом авторизации устройства.

## Телеметрия

Firmware публикует плоский JSON:

```json
{
  "schemaVersion": 1,
  "firmwareVersion": "1.0.0",
  "configVersion": 1,
  "deviceId": "device-1",
  "timestamp": "2026-05-04T00:00:00Z",
  "measurementIntervalSeconds": 1800,
  "weight": 42.5,
  "weightChange": 0.3,
  "temperature": 34.2,
  "humidity": 61.5,
  "hiveOpened": false,
  "batteryPercent": 87,
  "batteryVoltage": 3.91,
  "rssi": -55
}
```

Backend нормализует данные в `sensor_readings`:

```text
metric_type
value
unit
measured_at
device_id
apiary_id
hive_id
```

Backend также сохраняет raw payload в `raw_payloads`.

## Состояние связи

Для спящих устройств обычный `offline` не является аварией. Актуальная логика предупреждений должна строиться на пропущенных плановых передачах:

```text
expected_next_telemetry_at
missed_telemetry_count
telemetry_interval_minutes
```

Порог по умолчанию: 5 пропущенных передач.

## Целевые backend-сервисы

Текущий MVP:

- `api-service`;
- `mqtt-ingestion-service`.

Следующие сервисы/модули по ТЗ:

- `command-service` для MQTT-команд устройствам;
- `notification-service` для push/Telegram/in-app;
- `scheduler-service` для напоминаний, повторяющихся задач и проверки пропущенной телеметрии;
- `weather-service` для метеостанций и внешних погодных API;
- `ota-service` для firmware rollout;
- `ai-service` для premium аналитики.

## Безопасность

- Пользователи авторизуются через backend.
- MVP backend уже поддерживает email/password и JWT access token.
- Google/Apple OAuth, SMS и 2FA переносятся на следующие инкременты.
- MQTT credentials должны выдаваться на уровне пасеки.
- В локальной dev-конфигурации Mosquitto разрешает anonymous access; production-конфигурация должна включить ACL, TLS и secrets.
- Секреты не должны храниться в репозитории.

## Документация по деталям

- Backend: [../backend/docs/README.md](../backend/docs/README.md)
- Backend architecture: [../backend/docs/architecture.md](../backend/docs/architecture.md)
- Firmware: [../firmware/docs/README.md](../firmware/docs/README.md)
- MVP status: [implementation-status.md](implementation-status.md)
