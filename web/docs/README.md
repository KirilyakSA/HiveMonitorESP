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
- журналы событий пасеки и улья;
- советы dashboard и календарь работ пасеки.

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
    components/
      DashboardParts.tsx
      MapProvider.tsx
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

## E2E-тесты интерфейса

Для web MVP добавлен первый слой Playwright-тестов. Они используют реальные backend API и demo-пользователя, поэтому перед запуском нужен поднятый backend и seed demo-данных.

```bash
cd web
npx playwright install chromium
npm run test:e2e
```

Переменные окружения:

```text
HIVEMONITOR_E2E_EMAIL     email demo-пользователя, по умолчанию demo@hivemonitor.local
HIVEMONITOR_E2E_PASSWORD  пароль demo-пользователя, по умолчанию password123
PLAYWRIGHT_BASE_URL       адрес web UI, по умолчанию http://127.0.0.1:5173
HIVEMONITOR_E2E_DB_CONTAINER  docker-контейнер Postgres, по умолчанию deploy-postgres-1
HIVEMONITOR_E2E_DB_USER       пользователь БД, по умолчанию hivemonitor
HIVEMONITOR_E2E_DB_NAME       база данных, по умолчанию hivemonitor
```

Большинство тестов не изменяют данные. Отдельные full-cycle тесты создают fixture-устройства напрямую в Postgres, затем через UI проверяют создание пасеки, создание улья, привязку устройства, замену устройства, удаление устройства, удаление улья и удаление пасеки. Cleanup fixture остается как страховка в `beforeEach/afterEach`, а пользовательские delete-сценарии проходят через публичные backend API и web UI.

## Текущие ограничения

- Нет refresh tokens и управления сессиями, потому что backend пока выдает только access token.
- Command API MVP уже есть: web может отправлять deferred-команды обслуживания (`reboot`, `firmware_update`, `config_update`) и команды сессии для backend-тары (`hold_config_session`, `capture_weight`, `finish_config_session`). Полное сохранение `scale_profile` и история тары остаются следующим backend-инкрементом.
- Нет alert rules и уведомлений, поэтому статус связи по пропущенным передачам будет полноценно отображаться после backend-инкремента #19.
- Погодная карточка уже читает `GET /apiaries/{apiaryID}/weather/history`; если ряд отсутствует, используется UI fallback до подключения внешнего provider или метеостанции.
- Нет ролей/permission matrix в UI, потому что backend пока проверяет только membership на уровне организации/пасеки.
- Нет полноценной i18n-системы; MVP интерфейс написан на русском.
- Удаление устройства, улья и пасеки реализовано как MVP hard-delete flow. После MVP нужно добавить soft-delete/archive policy, audit trail и более подробный выбор сохранения истории.
