# Service boundaries / Границы сервисов

RU: Документ фиксирует, какие части HiveMonitor остаются в core API, а какие должны развиваться как отдельные сервисы. Это помогает не дробить систему слишком рано, но держать понятные границы для роста.

EN: This document defines which parts of HiveMonitor remain in the core API and which should evolve into separate services. It avoids premature fragmentation while keeping clear growth boundaries.

## Сейчас реализовано / Implemented Now

```text
api-service
  REST API, auth, organizations, apiaries, hives, devices, telemetry reads, events, calendar/advice API

mqtt-ingestion-service
  MQTT telemetry/events/status ingestion, raw payloads, device metadata, sensor readings

worker-service
  Periodic backend jobs: due/overdue task status refresh and missed telemetry counter refresh

PostgreSQL
  System of record

NATS
  Internal event bus for async workflows

Mosquitto
  MQTT broker for devices
```

## Что остается в api-service / What Stays in api-service

RU: В `api-service` остаются синхронные CRUD/API операции и core domain.

EN: `api-service` keeps synchronous CRUD/API operations and the core domain.

- RU: users, organizations, roles, apiaries, hives, devices and assignments.
  EN: users, organizations, roles, apiaries, hives, devices and assignments.
- RU: чтение latest/history telemetry.
  EN: latest/history telemetry reads.
- RU: журнал событий, ручные события, комментарии, осмотры.
  EN: event journal, manual events, comments and inspections.
- RU: CRUD календаря, задач, советов и настроек пасеки.
  EN: calendar, tasks, advice and apiary settings CRUD.
- RU: схема расположения ульев.
  EN: apiary hive layout.
- RU: публичный паспорт партии меда на уровне данных.
  EN: public honey batch passport at the data/API level.

## worker-service

RU: `worker-service` - первый общий фоновой сервис. Он не обслуживает HTTP и выполняет периодические jobs.

EN: `worker-service` is the first shared background service. It does not serve HTTP and runs periodic jobs.

Текущий MVP / Current MVP:

- RU: обновляет `apiary_tasks.status` в `due` / `overdue`.
  EN: updates `apiary_tasks.status` to `due` / `overdue`.
- RU: пересчитывает `devices.missed_telemetry_count` по `expected_next_telemetry_at`.
  EN: recalculates `devices.missed_telemetry_count` using `expected_next_telemetry_at`.
- RU: публикует `worker.tick.completed` в NATS.
  EN: publishes `worker.tick.completed` to NATS.

Следующие jobs / Next jobs:

- RU: генерация recurring/seasonal tasks.
  EN: recurring/seasonal task generation.
- RU: проверка anti-theft окон обслуживания.
  EN: anti-theft maintenance window checks.
- RU: запуск alert-engine rules.
  EN: alert-engine rule execution.
- RU: подготовка daily summaries.
  EN: daily summary preparation.

## Сервисы, которые стоит выделять следующими / Next Services to Extract

### alert-engine-service

- RU: alert rules, системные теги, “проблемные ульи сегодня”, anti-theft MVP, антироевые сигналы.
  EN: alert rules, system tags, problem hives today, anti-theft MVP and anti-swarming signals.
- RU: читает telemetry/events/tasks/inspections, пишет alerts/events/tags.
  EN: reads telemetry/events/tasks/inspections, writes alerts/events/tags.

### notification-service

- RU: Telegram, push, in-app, email later.
  EN: Telegram, push, in-app and later email.
- RU: слушает notification records/events и доставляет сообщения.
  EN: consumes notification records/events and delivers messages.

### command-service

- RU: публикует MQTT commands/config, отслеживает ack/result/timeout.
  EN: publishes MQTT commands/config and tracks ack/result/timeout.
- RU: API создает command record, сервис доставляет его в MQTT.
  EN: API creates a command record, the service delivers it to MQTT.

### weather-service

- RU: external weather providers, own weather station devices, weather readings and aggregates.
  EN: external weather providers, owned weather station devices, weather readings and aggregates.

### analytics-service

- RU: weather analytics, honey flow forecast, apiary comparisons, anti-swarming recommendations.
  EN: weather analytics, honey flow forecast, apiary comparisons and anti-swarming recommendations.

### geo-service

- RU: foraging radius, honey plant zones, pesticide risk layers, migratory stands.
  EN: foraging radius, honey plant zones, pesticide risk layers and migratory stands.

### media-service

- RU: uploads, frame/event/inspection photos, thumbnails, object storage.
  EN: uploads, frame/event/inspection photos, thumbnails and object storage.

### ai-service

- RU: audio analysis, frame image analysis, beekeeper AI assistant, model/provider isolation.
  EN: audio analysis, frame image analysis, beekeeper AI assistant and model/provider isolation.

### ota-service

- RU: firmware versions, rollout waves, device eligibility, rollback.
  EN: firmware versions, rollout waves, device eligibility and rollback.

### billing-service

- RU: tariffs, limits, premium feature gates and payment provider integration later.
  EN: tariffs, limits, premium feature gates and later payment provider integration.

## Правило выделения / Extraction Rule

RU: Новый отдельный сервис нужен, если функция:

EN: A new service is justified when the feature:

- RU: работает по расписанию или долго считает;
  EN: runs on a schedule or performs long-running computations;
- RU: слушает события и реагирует асинхронно;
  EN: consumes events and reacts asynchronously;
- RU: ходит во внешние API;
  EN: calls external APIs;
- RU: отправляет уведомления или MQTT-команды;
  EN: sends notifications or MQTT commands;
- RU: обрабатывает файлы, аудио, изображения или AI jobs;
  EN: processes files, audio, images or AI jobs;
- RU: имеет отдельное масштабирование, лимиты или premium billing.
  EN: needs independent scaling, limits or premium billing.
