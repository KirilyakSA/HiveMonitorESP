# Deploy

Docker/Docker Compose файлы и инфраструктурные настройки HiveMonitor.

Текущий compose поднимает:

- PostgreSQL;
- NATS + JetStream;
- Mosquitto;
- `api-service`;
- `mqtt-ingestion-service`.

Детальная документация: [deploy/docs](docs/README.md).
