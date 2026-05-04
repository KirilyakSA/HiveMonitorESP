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
go run github.com/pressly/goose/v3/cmd/goose@latest -dir migrations postgres "$DATABASE_URL" up
```

Запустить API:

```bash
go run ./cmd/api-service
```

Запустить MQTT ingestion:

```bash
go run ./cmd/mqtt-ingestion-service
```

## Реализованный MVP-срез

- регистрация и вход по email/password;
- JWT access token;
- организации;
- пасеки;
- ульи;
- список непривязанных устройств;
- привязка устройства к улью с режимом импорта старой телеметрии;
- MQTT ingestion для текущего firmware topic `hives/{deviceId}/telemetry`;
- хранение raw payload;
- динамические sensor readings;
- endpoints последней и исторической телеметрии.

Подробная документация: [backend/docs](docs/README.md).
