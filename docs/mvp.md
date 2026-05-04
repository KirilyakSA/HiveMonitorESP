# MVP HiveMonitor

## Цель MVP

Первый MVP должен доказать вертикаль данных:

```text
пользователь
  -> организация
  -> пасека
  -> улей
  -> устройство отправляет MQTT-телеметрию
  -> backend принимает данные
  -> устройство привязывается к улью
  -> API возвращает latest/history telemetry
```

## Реализовано сейчас

### Firmware

- ESP8266/ESP32 PlatformIO-проект.
- Локальный web-интерфейс устройства.
- Настройка Wi-Fi, MQTT, TLS, пинов, датчиков, периода измерений.
- HX711, DHT11/DHT21/DHT22, датчик Холла, батарея.
- JSON-телеметрия в `hives/{deviceId}/telemetry`.
- События в `hives/{deviceId}/events`.
- MQTT-команды `measure`, `restart`, `tare`, `clearBuffer`, `configUpdate`.
- Буферизация телеметрии в LittleFS.
- Локальное web-обновление прошивки.
- Factory reset кнопкой.
- Deep sleep с попыткой доставить текущую и буферизованную телеметрию.

### Backend

- Go + chi + pgx.
- PostgreSQL миграции через goose.
- Docker Compose для PostgreSQL, NATS, Mosquitto, API и MQTT ingestion.
- Email/password регистрация и вход.
- JWT access token.
- Организации.
- Пасеки.
- Ульи.
- Непривязанные устройства.
- Привязка устройства к улью.
- Импорт или удаление старой непривязанной телеметрии при привязке.
- MQTT ingestion для:
  - `hives/+/telemetry`;
  - `apiaries/+/devices/+/telemetry`.
- Сохранение raw payload.
- Динамические `sensor_readings`.
- Latest/history telemetry endpoints.

## Проверенный сценарий

Проверялось:

```text
go test ./...
docker compose config
goose up
docker compose up --build api-service mqtt-ingestion-service
GET /healthz
POST /auth/register
POST /organizations/
POST /apiaries/
POST /apiaries/{id}/hives
MQTT publish telemetry
GET /apiaries/{id}/devices/unassigned
POST /apiaries/{id}/devices/{deviceUUID}/assign
GET /hives/{id}/telemetry/latest
GET /hives/{id}/telemetry/history
```

Результат: telemetry была принята, устройство создано как `unassigned`, привязано к улью, latest/history вернули readings.

## Известные ограничения MVP

- Firmware пока публикует только legacy topic `hives/{deviceId}/telemetry`.
- Для автоматического помещения устройства в конкретную пасеку нужен целевой topic `apiaries/{apiary_id}/devices/{device_id}/telemetry` или backend `DEFAULT_APIARY_ID`.
- Backend пока не отправляет MQTT-команды устройствам, хотя firmware их принимает.
- Backend пока не обрабатывает `hives/{deviceId}/events` и `hives/{deviceId}/status`.
- Alerts, системные теги, события от alerts и уведомления еще не реализованы.
- Проверка пропущенных передач еще не реализована.
- Tasks/reminders еще не реализованы.
- OTA через backend еще не реализована.
- Weather provider и метеостанции еще не реализованы.
- Billing/tariffs пока описаны в ТЗ, но не реализованы.

## Следующий MVP-инкремент

Рекомендуемый порядок:

1. Добавить `apiaryId` или configurable topic prefix в firmware.
2. Добавить backend обработку `events` и `status`.
3. Добавить `device_commands` API и MQTT publish команд.
4. Добавить проверку пропущенных передач.
5. Добавить базовые `alert_rules`, `alerts`, системные теги и события от alerts.
6. Добавить notification records без реальной доставки.
7. Добавить задачи, события и напоминания.
