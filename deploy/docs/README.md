# Deploy

## Назначение

Подпроект `deploy/` содержит инфраструктуру запуска HiveMonitor.

## Подход

Развертывание проектируется по подходу Docker-first.

Текущий сценарий:

- локальный запуск;
- VPS или выделенный сервер через Docker Compose.

Будущий сценарий:

- перенос сервисов в Kubernetes по мере роста нагрузки.

## Реализованный Docker Compose

Файл:

```text
deploy/docker-compose.yml
```

Сервисы:

- `postgres` - PostgreSQL 16;
- `nats` - NATS с JetStream;
- `mosquitto` - MQTT broker;
- `api-service` - REST API backend;
- `mqtt-ingestion-service` - MQTT consumer телеметрии, device events и device status.

Порты:

```text
5432  PostgreSQL
4222  NATS client
8222  NATS monitoring
1883  Mosquitto MQTT
8080  API
```

## Mosquitto

Dev-конфигурация:

```text
deploy/mosquitto/mosquitto.conf
```

В MVP/dev разрешен anonymous access. Для production нужно заменить на:

- users/passwords;
- ACL по пасекам и устройствам;
- TLS;
- отдельное хранение секретов.

## NATS

NATS запускается с JetStream:

```text
-js -sd /data
```

Сейчас backend публикует событие:

```text
telemetry.received
device.event.received
device.status.received
```

Будущие сервисы будут использовать NATS для notifications, reminders, alerts, weather polling, OTA и AI jobs.

## PostgreSQL

Для MVP используется один volume:

```text
postgres-data
```

Миграции применяются из `backend/migrations` через goose.

## Подробная инструкция

Полный runbook по установке зависимостей, настройке `.env`, генерации секретов, запуску, проверке сервисов, firmware-настройкам и VPS-сценарию находится здесь:

- [Развертывание и эксплуатация](../../docs/deployment-guide.md)

## Безопасность production

- TLS для публичного web/API.
- TLS для MQTT.
- Закрыть anonymous MQTT.
- Не хранить secrets в репозитории.
- Пароли и токены передавать через env vars или secret storage.
- Добавить backup PostgreSQL.
- Добавить reverse proxy, например Caddy, Nginx или Traefik.

## Не реализовано

- production profile;
- reverse proxy;
- backup jobs;
- secret storage;
- object storage;
- отдельные profiles для dev/prod;
- Kubernetes manifests.
