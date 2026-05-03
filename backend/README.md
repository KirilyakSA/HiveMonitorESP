# Backend

Go backend HiveMonitor.

## MVP stack

- Go + chi;
- PostgreSQL + pgx;
- goose migrations;
- NATS + JetStream;
- Mosquitto MQTT broker;
- Docker Compose for local/VPS deployment.

## Services

```text
cmd/api-service              REST API for web/mobile
cmd/mqtt-ingestion-service   MQTT telemetry consumer
```

## Local run

Start infrastructure from repository root:

```bash
docker compose -f deploy/docker-compose.yml up -d postgres nats mosquitto
```

Run migrations:

```bash
cd backend
go run github.com/pressly/goose/v3/cmd/goose@latest -dir migrations postgres "$DATABASE_URL" up
```

Run API:

```bash
go run ./cmd/api-service
```

Run MQTT ingestion:

```bash
go run ./cmd/mqtt-ingestion-service
```

## Implemented MVP slice

- email/password registration and login;
- JWT access token;
- organizations;
- apiaries;
- hives;
- unassigned device list;
- device assignment to hive with old telemetry import mode;
- MQTT ingestion for current firmware topic `hives/{deviceId}/telemetry`;
- raw payload storage;
- dynamic sensor readings;
- latest/history telemetry endpoints.

Detailed documentation: [backend/docs](docs/README.md).
