# Backend

## Назначение

Backend принимает телеметрию IoT-устройств, хранит данные, управляет пользователями, организациями, пасеками, ульями, устройствами и предоставляет API для web/mobile клиентов.

Подробное описание архитектуры backend, сервисов, файлов и данных: [architecture.md](architecture.md).

Границы текущих и будущих backend-сервисов: [service-boundaries.md](service-boundaries.md).

Сводка по соответствию реализации и ТЗ: [../../docs/implementation-status.md](../../docs/implementation-status.md).

Текущая backend-очередь работ / Current backend backlog: [backlog.md](backlog.md).

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
cmd/worker-service
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
- Выбор поведения, если в улье уже есть активное устройство:
  - добавить новое устройство как дополнительное;
  - заменить существующую активную привязку через `replace_existing`.
- Прием MQTT telemetry.
- Прием MQTT device events/status.
- Сохранение raw MQTT payload.
- Нормализация readings в `sensor_readings`.
- Latest/history telemetry endpoints.
- Apiary/hive event journal endpoints.
- Публикация NATS event `telemetry.received`.
- Стартовый адаптируемый календарь пасечных работ.
- Seed базы советов, задач, медоносов и периодов цветения.
- API советов dashboard и календарных задач пасеки.
- Worker service для обновления статусов календарных задач и счетчика пропущенной телеметрии.

## MQTT

Backend MVP подписывается на:

```text
hives/+/telemetry
apiaries/+/devices/+/telemetry
hives/+/events
apiaries/+/devices/+/events
hives/+/status
apiaries/+/devices/+/status
```

`hives/+/...` - совместимость с legacy firmware topics.

`apiaries/+/devices/+/...` - целевой topic namespace для provisioning по пасеке.

Firmware публикует apiary-aware topics, если в локальной конфигурации задан `apiaryId`. Если `apiaryId` пустой, устройство остается в legacy mode:

```text
hives/{deviceId}/telemetry
hives/{deviceId}/events
hives/{deviceId}/status
```

Для автоматического попадания legacy-устройства в конкретную пасеку все еще нужен `DEFAULT_APIARY_ID` или заранее зарегистрированное/привязанное устройство. Production path должен использовать apiary-aware topics.

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
DELETE /apiaries/{apiaryID}
GET  /apiaries/{apiaryID}/hives
POST /apiaries/{apiaryID}/hives
GET  /apiaries/{apiaryID}/devices/unassigned
GET  /apiaries/{apiaryID}/devices/{deviceUUID}
DELETE /apiaries/{apiaryID}/devices/{deviceUUID}
POST /apiaries/{apiaryID}/devices/{deviceUUID}/assign
GET  /apiaries/{apiaryID}/devices/{deviceUUID}/commands
POST /apiaries/{apiaryID}/devices/{deviceUUID}/commands
GET  /apiaries/{apiaryID}/events
GET  /apiaries/{apiaryID}/advice
PATCH /apiaries/{apiaryID}/advice/{adviceCode}
GET  /apiaries/{apiaryID}/calendar/tasks
POST /apiaries/{apiaryID}/calendar/tasks
PATCH /apiaries/{apiaryID}/calendar/tasks/{taskID}

GET  /hives/{hiveID}/telemetry/latest
GET  /hives/{hiveID}/telemetry/history
GET  /hives/{hiveID}/events
DELETE /hives/{hiveID}
```

Device command MVP / MVP команд устройств:

- RU: API создает запись в `device_commands`, публикует MQTT command message и возвращает текущий статус команды.
  EN: the API creates a `device_commands` record, publishes an MQTT command message and returns the current command status.
- RU: поддержанные backend-команды устройства: `reboot`, `firmware_update`, `config_update`, `hold_config_session`, `capture_weight`, `finish_config_session`. `restart` временно оставлен как legacy alias.
  EN: supported backend device commands: `reboot`, `firmware_update`, `config_update`, `hold_config_session`, `capture_weight`, `finish_config_session`. `restart` is temporarily kept as a legacy alias.
- RU: отдельной команды `measure` в MVP нет: sleeping-устройство при физическом пробуждении само делает замеры, отправляет телеметрию и снова засыпает.
  EN: there is no separate `measure` command in the MVP: when a sleeping device is physically woken up, it measures, sends telemetry and sleeps again.
- RU: `reboot`, `firmware_update` и `config_update` ставятся в очередь и выполняются при следующем пробуждении; UI предупреждает пользователя, что при необходимости устройство можно разбудить вручную.
  EN: `reboot`, `firmware_update` and `config_update` are queued and run on the next wake-up; the UI warns the user that the device can be woken manually if needed.
- RU: firmware tare и калибровка весов выполняются локально в web-интерфейсе устройства: сначала firmware tare сбрасывает собственные весы в ноль, затем calibration flow просит калибровочные веса 1 кг, 100 г и 10 г.
  EN: firmware tare and scale calibration are performed locally in the device web UI: firmware tare first zeros the scale itself, then the calibration flow asks for 1 kg, 100 g and 10 g calibration weights.
- RU: тара улья и тара магазинов не отправляются на устройство. Это backend-параметры улья: backend хранит вес тары и вычитает его из сырого веса устройства для отображения полезного веса в web/mobile.
  EN: hive tare and super tare are not sent to the device. They are backend hive parameters: the backend stores tare weight and subtracts it from raw device weight to show useful/net weight in web/mobile.
- RU: для backend-тары устройство все равно участвует в процессе: UI удерживает устройство командой `hold_config_session`, просит подготовить улей/магазин, отправляет `capture_weight` для одноразового сырого замера и закрывает сеанс через `finish_config_session`; сохранение тары происходит только в backend.
  EN: for backend tare, the device still participates in the process: the UI keeps it awake with `hold_config_session`, asks the user to prepare the hive/super, sends `capture_weight` for a one-off raw measurement and closes the session with `finish_config_session`; tare storage happens only in the backend.
- RU: публикация идет в `apiaries/{apiaryId}/devices/{deviceId}/commands` и legacy `hives/{deviceId}/commands`.
  EN: publishing goes to `apiaries/{apiaryId}/devices/{deviceId}/commands` and legacy `hives/{deviceId}/commands`.
- RU: статусы `acknowledged`, `expired` и сохранение результата firmware заложены в модель и будут включены после firmware ack flow.
  EN: `acknowledged`, `expired` statuses and firmware result storage are modeled and will be enabled after the firmware ack flow.

## Календарь работ и советы

Backend хранит шаблонную базу знаний, а не жесткий календарь:

```text
calendar_template + apiary_calendar_settings + date_shift_days + trigger_type + user overrides
```

В MVP есть дефолтный seed `backend/seeds/beekeeping_calendar_seed.sql` для `ua_forest_steppe`. Он добавляет периоды пасечного года, шаблоны советов, шаблоны работ, медоносы и периоды цветения. Для каждой пасеки настройки создаются автоматически при первом запросе советов или задач.

Сервис советов уже возвращает календарные, bloom и базовые telemetry-подсказки. Weather-trigger правила заложены в данных, но полноценный weather provider остается отдельной задачей.

## Не реализовано, но требуется по ТЗ

- Полная permission matrix для ролей организации и пасеки.
- Email invitations.
- MQTT command API/service.
- Alerts, системные теги и события от alerts.
- Проверка пропущенных плановых передач.
- Уведомления push/Telegram/in-app.
- Weather providers и метеостанции.
- Полная recurring task engine с RRULE.
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
