# Web

Web-интерфейс HiveMonitor на React + TypeScript + Vite.

## Что реализовано

- email/password вход и регистрация;
- рабочая панель пасеки;
- организации и пасеки;
- создание ульев;
- просмотр непривязанных устройств;
- привязка устройства к улью;
- latest telemetry;
- history telemetry за 24 часа;
- журналы событий пасеки и улья.

## Запуск

Backend должен быть запущен на `http://localhost:8080`.

```bash
cd web
npm install
npm run dev
```

Dev server по умолчанию:

```text
http://localhost:5173
```

Для другого backend host:

```bash
VITE_API_BASE_URL=http://backend-host:8080 npm run dev
```

## Документация

- [Web docs](docs/README.md)
- [ТЗ frontend](docs/technical-spec.md)
- [Архитектура frontend](docs/architecture.md)
- [MVP frontend](docs/mvp.md)
