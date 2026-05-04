# Web frontend HiveMonitor

## Назначение

Web frontend - рабочий интерфейс пасечника и администратора пасеки для управления организациями, пасеками, ульями, устройствами, телеметрией и журналами событий.

Текущий web MVP ориентирован на уже реализованный backend:

- email/password auth;
- организации;
- пасеки;
- ульи;
- непривязанные устройства;
- привязка устройства к улью;
- latest/history telemetry;
- журналы событий пасеки и улья.

## Документы

- [Техническое задание frontend](technical-spec.md)
- [Архитектура frontend](architecture.md)
- [MVP frontend](mvp.md)

## Реализация

Текущий MVP находится в `web/` и реализован как SPA на React + TypeScript + Vite.

Основные файлы:

```text
web/
  package.json
  vite.config.ts
  index.html
  src/
    api.ts
    App.tsx
    main.tsx
    styles.css
```

## Запуск

Backend должен быть доступен на `http://localhost:8080`. В dev-режиме Vite проксирует backend routes:

```text
/auth
/me
/organizations
/apiaries
/hives
/healthz
```

Команды:

```bash
cd web
npm install
npm run dev
```

Если backend находится не на `localhost:8080`, можно задать:

```bash
VITE_API_BASE_URL=http://backend-host:8080 npm run dev
```

## Текущие ограничения

- Нет refresh tokens и управления сессиями, потому что backend пока выдает только access token.
- Нет command API для устройств, поэтому web пока не отправляет `measure`, `tare`, `restart`.
- Нет alert rules и уведомлений, поэтому статус связи по пропущенным передачам будет полноценно отображаться после backend-инкремента #19.
- Нет ролей/permission matrix в UI, потому что backend пока проверяет только membership на уровне организации/пасеки.
- Нет полноценной i18n-системы; MVP интерфейс написан на русском.
