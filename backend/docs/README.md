# Backend

## Назначение

Backend принимает телеметрию IoT-устройств, хранит данные, управляет пользователями, организациями, пасеками, ульями, устройствами и предоставляет API для web/mobile клиентов.

Подробное описание архитектуры backend, сервисов, файлов и данных: [architecture.md](architecture.md).

Сводка по соответствию реализации и ТЗ: [../../docs/implementation-status.md](../../docs/implementation-status.md).

## Стек

- Go;
- chi;
- PostgreSQL;
- pgx;
- goose;
- NATS + JetStream;
- Mosquitto MQTT broker для MVP/dev;
- Docker/Docker Compose.

## Реализация MVP

Первый backend-инкремент находится в:

```text
cmd/api-service
cmd/mqtt-ingestion-service
internal/
migrations/
```

Запуск инфраструктуры описан в `backend/README.md` и `deploy/docker-compose.yml`.

## Реализовано сейчас

- Email/password регистрация и вход.
- JWT access token.
- Organizations.
- Apiaries.
- Hives.
- Список непривязанных устройств.
- Привязка устройства к улью.
- Выбор действия со старой непривязанной телеметрией при привязке:
  - `attach_to_hive`;
  - `delete`;
  - `keep`.
- Прием MQTT telemetry.
- Сохранение raw MQTT payload.
- Нормализация readings в `sensor_readings`.
- Latest/history telemetry endpoints.
- Публикация NATS event `telemetry.received`.

## MQTT

Backend MVP подписывается на:

```text
hives/+/telemetry
apiaries/+/devices/+/telemetry
```

`hives/+/telemetry` - совместимость с текущей firmware.

`apiaries/+/devices/+/telemetry` - целевой topic для provisioning по пасеке.

Текущая firmware публикует только:

```text
hives/{deviceId}/telemetry
```

Поэтому для автоматического попадания legacy-устройства в конкретную пасеку нужен `DEFAULT_APIARY_ID` или следующий firmware-инкремент с `apiaryId`/topic prefix.

## Авторизация устройств

Актуальное целевое требование: MQTT credentials выдаются на уровне пасеки.

Текущее состояние:

- firmware поддерживает `mqttUser` и `mqttPassword`;
- dev Mosquitto в `deploy/docker-compose.yml` пока разрешает anonymous access;
- production-конфигурация должна добавить users/passwords, ACL и TLS;
- поле `deviceToken` в firmware остается legacy/fallback секретом, но backend MVP не проверяет `deviceToken`.

## API MVP

```text
POST /auth/register
POST /auth/login
GET  /me

GET  /organizations/
POST /organizations/

GET  /apiaries/
POST /apiaries/
GET  /apiaries/{apiaryID}/hives
POST /apiaries/{apiaryID}/hives
GET  /apiaries/{apiaryID}/devices/unassigned
POST /apiaries/{apiaryID}/devices/{deviceUUID}/assign

GET  /hives/{hiveID}/telemetry/latest
GET  /hives/{hiveID}/telemetry/history
```

## Не реализовано, но требуется по ТЗ

- Полная permission matrix для ролей организации и пасеки.
- Email invitations.
- Обработка `hives/{deviceId}/events`.
- Обработка `hives/{deviceId}/status`.
- MQTT command API/service.
- Alerts, системные теги и события от alerts.
- Проверка пропущенных плановых передач.
- Уведомления push/Telegram/in-app.
- Tasks, recurring tasks, reminders.
- Weather providers и метеостанции.
- Архивация телеметрии.
- OTA rollout.
- Tariffs/billing.
- AI premium functions.

## MVP-уведомления

В актуальной модели для спящих устройств не используем простой `offline` как аварийный сигнал. Правильные MVP alerts:

- устройство пропустило заданное количество плановых передач;
- низкий заряд батареи;
- резкое падение веса;
- температура вне нормы;
- влажность вне нормы;
- открытие улья, если включен датчик Холла.
