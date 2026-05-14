# Deploy

Docker/Docker Compose файлы и инфраструктурные настройки HiveMonitor.

Текущий compose поднимает:

- PostgreSQL;
- NATS + JetStream;
- Mosquitto;
- `api-service`;
- `mqtt-ingestion-service`;
- `worker-service`.

Детальная документация:

- [Deploy docs](docs/README.md)
- [Развертывание и эксплуатация](../docs/deployment-guide.md)
