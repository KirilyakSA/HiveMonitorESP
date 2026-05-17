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
  -> web показывает рабочую панель пасеки
```

## Реализовано сейчас

### Firmware

- ESP8266/ESP32 PlatformIO-проект.
- Локальный web-интерфейс устройства.
- Настройка Wi-Fi, MQTT, TLS, пинов, датчиков, периода измерений.
- HX711, DHT11/DHT21/DHT22, датчик Холла, батарея.
- JSON-телеметрия в legacy или apiary-aware MQTT topics.
- События и статусы в legacy или apiary-aware MQTT topics.
- MQTT-команды backend: `reboot`, `firmware_update`, `config_update`.
- Firmware tare и калибровка весов выполняются локально в web-интерфейсе устройства.
- Backend-тара улья/магазинов является параметром улья, сохраняется в `hive_scale_profiles`, журналируется в `hive_tare_events` и вычитается из сырого веса в API/UI.
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
  - `apiaries/+/devices/+/telemetry`;
  - `hives/+/events`;
  - `apiaries/+/devices/+/events`;
  - `hives/+/status`;
  - `apiaries/+/devices/+/status`.
- Сохранение raw payload.
- Динамические `sensor_readings`.
- Журнал device events для пасеки и улья.
- Latest/history telemetry endpoints.

### Web

- React + TypeScript + Vite SPA.
- Email/password вход и регистрация.
- Организации.
- Пасеки.
- Ульи.
- Непривязанные устройства.
- Привязка устройства к улью.
- Latest telemetry карточки.
- History telemetry график за 24 часа.
- Журналы событий пасеки и улья.

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
MQTT publish device event/status
GET /apiaries/{id}/devices/unassigned
POST /apiaries/{id}/devices/{deviceUUID}/assign
GET /hives/{id}/telemetry/latest
GET /hives/{id}/telemetry/history
GET /apiaries/{id}/events
GET /hives/{id}/events
Web: login/register, create organization/apiary/hive, assign device, view telemetry/events
```

Результат: telemetry была принята, устройство создано как `unassigned`, привязано к улью, latest/history вернули readings, event/status messages сохраняются.

## Известные ограничения MVP

- Legacy mode `hives/{deviceId}/...` остается только для совместимости и dev/MVP.
- Для production provisioning нужно задавать `apiaryId` в локальной конфигурации firmware и использовать topics `apiaries/{apiary_id}/devices/{device_id}/...`.
- Backend отправляет MQTT-команды устройствам через `device_commands`; firmware ack/result flow еще не реализован.
- Web отправляет базовые команды обслуживания и мастер backend-тары. Firmware публикует ack/result для `capture_weight`; ручной fallback raw-веса в dev UI остается как страховка для старых прошивок.
- Alerts, системные теги, события от alerts и уведомления еще не реализованы.
- Проверка пропущенных передач еще не реализована.
- Tasks/reminders еще не реализованы.
- OTA через backend еще не реализована.
- Weather provider и метеостанции еще не реализованы.
- Billing/tariffs пока описаны в ТЗ, но не реализованы.

## Следующий MVP-инкремент

Рекомендуемый порядок:

1. Добавить `device_commands` API и MQTT publish команд.
2. Добавить проверку пропущенных передач.
3. Добавить базовые `alert_rules`, `alerts`, системные теги и события от alerts.
4. Добавить notification records без реальной доставки.
5. Добавить задачи, события и напоминания.
