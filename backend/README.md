# Backend

Go backend HiveMonitor.

## Стек MVP

- Go + chi;
- PostgreSQL + pgx;
- миграции goose;
- NATS + JetStream;
- MQTT broker Mosquitto;
- Docker Compose для локального запуска и развертывания на VPS.

## Сервисы

```text
cmd/api-service              REST API для web/mobile
cmd/mqtt-ingestion-service   MQTT consumer телеметрии
cmd/worker-service           Фоновые jobs: статусы задач и пропущенная телеметрия
```

## Локальный запуск

Запустить инфраструктуру из корня репозитория:

```bash
docker compose -f deploy/docker-compose.yml up -d postgres nats mosquitto
```

Применить миграции:

```bash
cd backend
go run github.com/pressly/goose/v3/cmd/goose -dir migrations postgres "$DATABASE_URL" up
```

Запустить API:

```bash
go run ./cmd/api-service
```

Запустить MQTT ingestion:

```bash
go run ./cmd/mqtt-ingestion-service
```

Запустить фоновые jobs:

```bash
go run ./cmd/worker-service
```

## Тестовые данные

Для локальной разработки можно заполнить базу демо-организацией, пасеками, ульями, устройствами, телеметрией и событиями:

```bash
docker exec -i deploy-postgres-1 psql -U hivemonitor -d hivemonitor < backend/seeds/dev_seed.sql
```

Тестовый пользователь:

```text
email: demo@hivemonitor.local
password: password123
```

Seed идемпотентный: его можно запускать повторно, он обновляет фиксированный демо-набор данных.

Стартовый календарь пасечных работ, советы и медоносы заполняются отдельным seed:

```bash
docker exec -i deploy-postgres-1 psql -U hivemonitor -d hivemonitor < backend/seeds/beekeeping_calendar_seed.sql
```

Он создает дефолтный шаблон `ua_forest_steppe`, периоды пасечного года, шаблоны советов, шаблоны работ, медоносы и периоды цветения. Seed также идемпотентный.

## Реализованный MVP-срез

- регистрация и вход по email/password;
- JWT access token;
- организации;
- пасеки;
- ульи;
- список непривязанных устройств;
- привязка устройства к улью с режимом импорта старой телеметрии;
- выбор замены существующей активной привязки улья или добавления устройства как дополнительного;
- MQTT ingestion для текущего firmware topic `hives/{deviceId}/telemetry`;
- MQTT ingestion для целевого topic `apiaries/{apiary_id}/devices/{device_id}/telemetry`;
- MQTT ingestion для device events/status в legacy и apiary-aware topics;
- хранение raw payload;
- динамические sensor readings;
- журнал device events на уровне пасеки и улья;
- endpoints последней и исторической телеметрии.
- адаптируемый календарь пасечных работ;
- API советов и сезонных рекомендаций;
- API задач пасеки с выполнением, откладыванием и скрытием.
- фоновый `worker-service`, который обновляет `due` / `overdue` статусы задач и счетчик пропущенных передач телеметрии.

## API календаря и советов

```http
GET /apiaries/{apiaryID}/advice?date=2026-05-07
GET /apiaries/{apiaryID}/calendar/tasks?from=2026-05-01&to=2026-05-31
POST /apiaries/{apiaryID}/calendar/tasks
PATCH /apiaries/{apiaryID}/calendar/tasks/{taskID}
PATCH /apiaries/{apiaryID}/advice/{adviceCode}
```

Советы формулируются как подсказки: “проверьте”, “запланируйте”, “обратите внимание”. Seed не содержит дозировок препаратов и не дает безусловных ветеринарных инструкций.

Подробная документация: [backend/docs](docs/README.md).

Границы текущих и будущих backend-сервисов: [backend/docs/service-boundaries.md](docs/service-boundaries.md).
