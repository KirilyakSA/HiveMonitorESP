# Архитектура web frontend

## 1. Общий подход

Frontend реализован как SPA:

```text
React + TypeScript + Vite
```

MVP сознательно простой:

- без React Router;
- без внешнего state manager;
- без UI-kit зависимости;
- один API client;
- рабочие экраны переключаются через локальное состояние sidebar.

Это позволяет быстро проверить backend/firmware вертикаль и не закреплять преждевременную архитектуру до появления ролей, command-service, alerts и tasks.

## 2. Структура файлов

```text
web/
  package.json
  tsconfig.json
  vite.config.ts
  index.html
  src/
    api.ts                        typed REST client
    App.tsx                       dashboard data container and orchestration
    components/
      DashboardParts.tsx          current UI components, forms, charts and UI helpers
      MapProvider.tsx             map provider abstraction, OpenStreetMap/Leaflet MVP implementation
    main.tsx                      React entrypoint
    styles.css                    application styles
```

## 3. API layer

`src/api.ts` содержит:

- TypeScript-типы backend models;
- `ApiClient`;
- `ApiError`;
- методы для auth, organizations, apiaries, hives, devices, telemetry, events.

UI не вызывает `fetch` напрямую.

## 4. State management

MVP state хранится в `App.tsx`:

- `token`;
- `user`;
- `organizations`;
- `apiaries`;
- `hives`;
- `devices`;
- `apiaryEvents`;
- выбранные `organization/apiary/hive`;
- активный экран sidebar: `apiaries` или `apiary-dashboard`;
- выбранная пасека на экране списка пасек;
- сводки пасек по текущей организации;
- latest/history/events выбранного улья.

После появления более сложных workflows можно выделить:

- auth provider;
- apiary workspace provider;
- server-state library или lightweight store.

## 5. Routing

MVP не использует router. Состояние навигации задается выбранными entity IDs.

Будущий routing:

```text
/login
/organizations/:organizationId
/apiaries/:apiaryId
/apiaries/:apiaryId/hives/:hiveId
/apiaries/:apiaryId/tasks
/apiaries/:apiaryId/events
/apiaries/:apiaryId/settings
```

## 6. Backend connection

Локальный dev:

```text
Vite dev server -> proxy -> backend localhost:8080
```

Production:

```text
Browser -> HTTPS reverse proxy -> api-service
```

`VITE_API_BASE_URL` позволяет указать внешний backend host, если frontend и backend развернуты отдельно.

## 7. UI composition

Текущие крупные блоки:

- `AuthView`;
- `ApiariesScreen`;
- `ApiaryMap`;
- sidebar organizations/apiaries;
- apiary summary;
- hives panel;
- unassigned devices panel;
- hive dashboard;
- telemetry chart;
- apiary/hive event journals.

Крупные компоненты вынесены из `App.tsx` в `src/components/DashboardParts.tsx`. Это промежуточный шаг: `App.tsx` остается контейнером данных, а `DashboardParts.tsx` пока держит UI, формы, графики и форматтеры вместе.

Карты подключены через `src/components/MapProvider.tsx`. MVP provider - OpenStreetMap через Leaflet. Экраны используют provider abstraction, чтобы в будущем можно было заменить OpenStreetMap на другой map provider без переписывания экранов.

Следующий шаг рефакторинга - разнести этот файл по feature-модулям:

```text
src/components/
src/features/auth/
src/features/apiaries/
src/features/hives/
src/features/devices/
src/features/telemetry/
src/features/events/
```

## 8. Ошибки и загрузка

MVP показывает ошибки backend как текстовый notice.

Целевое поведение:

- field-level errors для форм;
- global toast для фоновых действий;
- empty states для отсутствующих данных;
- retry для сетевых ошибок;
- skeleton/loading states для больших таблиц.

## 9. Security notes

- Access token хранится в `localStorage` только для MVP.
- MQTT credentials и device secrets не должны отображаться в web frontend.
- После появления refresh tokens лучше перейти на httpOnly cookie или другой согласованный auth flow.
- Frontend не должен принимать решения о правах доступа без backend.

## 10. Расширение

Следующие архитектурные изменения, которые стоит сделать перед ростом UI:

- добавить router;
- добавить i18n;
- вынести server state;
- добавить дизайн-токены;
- добавить тесты компонентов;
- добавить e2e smoke через Playwright;
- добавить API schema/codegen, если backend начнет публиковать OpenAPI.
