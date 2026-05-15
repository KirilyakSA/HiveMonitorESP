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

## Автотесты интерфейса

E2E-тесты написаны на Playwright и проверяют базовые пользовательские сценарии web MVP:

- переключение вход/регистрация и ошибку входа;
- вход демо-пользователя;
- загрузка dashboard;
- выход из аккаунта;
- открытие панели улья, вкладки и периоды графиков;
- вкладки событий;
- переключение периодов сравнительного графика;
- открытие формы создания улья без сохранения;
- раскрытие советов и переключение режима всех советов;
- экран пасек и форму создания пасеки;
- выбор пасеки и переход к dashboard;
- модальное окно привязки устройства без изменения данных;
- полный цикл: seed непривязанного устройства, создание улья, привязка устройства, проверка телеметрии и cleanup тестовых данных;
- замену устройства на улье;
- удаление устройства, улья и пасеки через web UI.

Перед запуском backend должен быть доступен на `http://localhost:8080`, а база должна быть засеяна demo-данными.
Для полного цикла привязки нужен доступ к Docker-контейнеру Postgres, потому что тест сам создает и удаляет fixture-данные.

```bash
cd web
npm install
npx playwright install chromium
npm run test:e2e
```

Демо-учетку можно переопределить переменными окружения:

```bash
HIVEMONITOR_E2E_EMAIL=demo@hivemonitor.local \
HIVEMONITOR_E2E_PASSWORD=password123 \
npm run test:e2e
```

Настройки Postgres fixture для полного цикла:

```text
HIVEMONITOR_E2E_DB_CONTAINER  имя docker-контейнера Postgres, по умолчанию deploy-postgres-1
HIVEMONITOR_E2E_DB_USER       пользователь БД, по умолчанию hivemonitor
HIVEMONITOR_E2E_DB_NAME       база данных, по умолчанию hivemonitor
```

## Документация

- [Web docs](docs/README.md)
- [ТЗ frontend](docs/technical-spec.md)
- [Архитектура frontend](docs/architecture.md)
- [MVP frontend](docs/mvp.md)
