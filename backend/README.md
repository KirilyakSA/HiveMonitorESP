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

## Реализованный MVP-срез

- регистрация и вход по email/password;
- JWT access token;
- организации;
- пасеки;
- ульи;
- список непривязанных устройств;
- привязка устройства к улью с режимом импорта старой телеметрии;
- MQTT ingestion для текущего firmware topic `hives/{deviceId}/telemetry`;
- MQTT ingestion для целевого topic `apiaries/{apiary_id}/devices/{device_id}/telemetry`;
- MQTT ingestion для device events/status в legacy и apiary-aware topics;
- хранение raw payload;
- динамические sensor readings;
- журнал device events на уровне пасеки и улья;
- endpoints последней и исторической телеметрии.

Подробная документация: [backend/docs](docs/README.md).
