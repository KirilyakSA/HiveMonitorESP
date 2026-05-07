# Backend backlog / Беклог backend

RU: Этот документ описывает backend-очередь после текущего MVP. GitHub Issues остаются источником задач для выполнения, документ нужен как техническая карта зависимостей.

EN: This document describes the backend backlog after the current MVP. GitHub Issues remain the execution source of truth; this file is a technical dependency map.

## P0

### 1. Device command API / API команд устройств - [#18](https://github.com/KirilyakSA/HiveMonitorESP/issues/18)

RU: Цель - дать web/mobile возможность отправлять команды устройству через backend.

EN: Goal - allow web/mobile clients to send device commands through the backend.

Нужно сделать / Required work:

- RU: таблицу `device_commands` или эквивалентный command journal.
  EN: add a `device_commands` table or an equivalent command journal.
- RU: HTTP API для создания команды к устройству.
  EN: add an HTTP API to create a command for a device.
- RU: публикацию MQTT command topic: `apiaries/{apiaryId}/devices/{deviceId}/commands`; legacy fallback `hives/{deviceId}/commands`.
  EN: publish MQTT command topics: `apiaries/{apiaryId}/devices/{deviceId}/commands`; legacy fallback `hives/{deviceId}/commands`.
- RU: статусы команды: created, published, acknowledged, failed, expired.
  EN: command statuses: created, published, acknowledged, failed, expired.
- RU: сохранение результата/ack от firmware, когда firmware начнет его публиковать.
  EN: store firmware result/ack once firmware starts publishing it.

### 2. Alerts по пропущенным передачам / Alerts for Missed Transmissions - [#19](https://github.com/KirilyakSA/HiveMonitorESP/issues/19)

RU: Цель - для sleeping devices не считать простой `offline` аварией, а реагировать на пропуск плановых передач.

EN: Goal - for sleeping devices, avoid treating plain `offline` as an alarm and instead react to missed scheduled transmissions.

Нужно сделать / Required work:

- RU: scheduler/job, который сравнивает `expected_next_telemetry_at` с текущим временем.
  EN: scheduler/job that compares `expected_next_telemetry_at` with current time.
- RU: настраиваемый порог пропусков, MVP default: 5 передач.
  EN: configurable missed-transmission threshold, MVP default: 5 transmissions.
- RU: обновление `missed_telemetry_count`.
  EN: update `missed_telemetry_count`.
- RU: создание device/hive/apiary event.
  EN: create device/hive/apiary events.
- RU: API-поле для web, чтобы подсвечивать улей как `attention` или `no_connection`.
  EN: expose an API field for web to highlight a hive as `attention` or `no_connection`.

### 3. Production MQTT auth/ACL / Production MQTT Auth and ACL - [#21](https://github.com/KirilyakSA/HiveMonitorESP/issues/21)

RU: Цель - закрыть anonymous MQTT в production profile.

EN: Goal - disable anonymous MQTT access in the production profile.

Backend-часть / Backend part:

- RU: модель credentials на уровне пасеки.
  EN: credentials model at the apiary level.
- RU: хранение только безопасного hash/secret material.
  EN: store only safe hash/secret material.
- RU: endpoint для просмотра/ротации MQTT credentials админом пасеки.
  EN: endpoint for apiary admins to view/rotate MQTT credentials.
- RU: ACL-правила на topics конкретной пасеки.
  EN: ACL rules for topics of a specific apiary.

RU: Deploy-часть описана в deploy backlog.

EN: The deploy part is described in the deploy backlog.

## P1

### 4. Журнал осмотров семьи / Hive Inspection Medical Record - [#29](https://github.com/KirilyakSA/HiveMonitorESP/issues/29)

RU: Цель - развить журнал событий в полноценную историю улья/пчелосемьи, где осмотры становятся “медицинской картой семьи”, а не просто набором технических событий.

EN: Goal - evolve the event journal into a full hive/colony history where inspections become a colony medical record rather than only technical events.

Нужно сделать / Required work:

- RU: таблицы осмотров с привязкой к организации, пасеке и улью.
  EN: inspection tables linked to organization, apiary and hive.
- RU: дата осмотра, сила семьи, расплод, матка, корм, признаки роения, болезни/клещ.
  EN: inspection date, colony strength, brood, queen, feed stores, swarming signs, disease/varroa signs.
- RU: выполненные работы, комментарии, результат осмотра и следующий рекомендуемый осмотр.
  EN: completed works, comments, inspection result and next recommended inspection.
- RU: вложения/фото рамок с учетом будущего file storage.
  EN: attachments/frame photos with future file storage in mind.
- RU: связь с календарем задач и журналом событий.
  EN: linkage with calendar tasks and event journals.
- RU: CRUD API и endpoint истории семьи для web/mobile.
  EN: CRUD API and colony history endpoint for web/mobile.

### 5. Alert rules и системные теги / Alert Rules and System Tags

RU: Цель - превратить сырую телеметрию в понятные признаки внимания.

EN: Goal - transform raw telemetry into actionable attention signals.

Нужно сделать / Required work:

- RU: таблицы alert rules на уровне пасеки и override на уровне улья.
  EN: alert rule tables at apiary level with hive-level overrides.
- RU: базовые правила: низкая батарея, резкое падение веса, температура/влажность вне порога, открытие улья.
  EN: basic rules: low battery, sudden weight drop, temperature/humidity outside thresholds, hive opening.
- RU: системные теги, которые backend может ставить/снимать автоматически.
  EN: system tags that backend can set/remove automatically.
- RU: журнал решений alert engine.
  EN: alert engine decision journal.

### 6. Tasks, recurring tasks, reminders / Задачи, повторяющиеся задачи и напоминания

RU: Цель - планировщик работ по пасеке и ульям.

EN: Goal - work planner for apiaries and hives.

Нужно сделать / Required work:

- RU: tasks на уровне пасеки и улья.
  EN: tasks at apiary and hive level.
- RU: recurring rules.
  EN: recurring rules.
- RU: статусы выполнения.
  EN: execution statuses.
- RU: запрос подтверждения выполнения у пользователя.
  EN: request completion confirmation from the user.
- RU: комментарии и результат работы.
  EN: comments and work result.
- RU: reminder events для будущих push/Telegram/in-app каналов.
  EN: reminder events for future push/Telegram/in-app channels.

### 7. Weather providers, weather stations and telemetry context / Погодные провайдеры, метеостанции и контекст телеметрии - [#30](https://github.com/KirilyakSA/HiveMonitorESP/issues/30)

RU: Цель - погодный контекст для анализа ульев, где погода используется не как “красивая погода”, а как аналитический слой для интерпретации веса, температуры улья и поведения семей.

EN: Goal - weather context for hive analysis, where weather is not a decorative widget but an analytical layer for interpreting weight, hive temperature and colony behavior.

Нужно сделать / Required work:

- RU: абстракцию `weather_provider`.
  EN: `weather_provider` abstraction.
- RU: default provider через внешний API по координатам пасеки.
  EN: default provider through an external API using apiary coordinates.
- RU: выбор provider из списка.
  EN: selectable provider from a list.
- RU: модель weather readings.
  EN: weather readings model.
- RU: поддержка собственной метеостанции как IoT-устройства пасеки.
  EN: support an owned weather station as an apiary IoT device.
- RU: архивирование min/max/avg weather metrics.
  EN: archive min/max/avg weather metrics.
- RU: API погодного ряда рядом с telemetry history.
  EN: weather time-series API alongside telemetry history.
- RU: правила интерпретации: “вес не растет, но был дождь” и “погода хорошая, соседние ульи растут, этот отстает”.
  EN: interpretation rules: “weight is not growing, but there was rain” and “weather is good, neighboring hives gain weight, this one lags”.
- RU: поддержку аналитики по слоям: вес улья, температура улья, внешняя температура, дождь, ветер, влажность, давление.
  EN: layered analytics support: hive weight, hive temperature, outside temperature, rain, wind, humidity and pressure.

### 8. Roles and permissions / Роли и права доступа

RU: Цель - подготовить SaaS-модель с организациями, пасеками и командами.

EN: Goal - prepare a SaaS model with organizations, apiaries and teams.

Нужно сделать / Required work:

- RU: роли организации: owner/admin/member.
  EN: organization roles: owner/admin/member.
- RU: роли пасеки: owner/admin/worker/viewer.
  EN: apiary roles: owner/admin/worker/viewer.
- RU: invitation flow.
  EN: invitation flow.
- RU: permission matrix для API handlers.
  EN: permission matrix for API handlers.
- RU: тесты на доступ к apiary/hive/device/event ресурсам.
  EN: access tests for apiary/hive/device/event resources.

### 9. Карта радиуса лёта и медоносов / Foraging Radius and Honey Plant Map - [#31](https://github.com/KirilyakSA/HiveMonitorESP/issues/31)

RU: Цель - добавить geo-слой пасеки: радиус лёта, медоносы, поля, сады, лесополосы, риски обработок и прогноз медосбора.

EN: Goal - add an apiary geo layer: foraging radius, honey plants, fields, orchards, forest belts, treatment risks and honey flow forecast.

Нужно сделать / Required work:

- RU: geo-модели зон вокруг пасеки: точка, полигон, радиус, тип зоны, заметки.
  EN: geo models for zones around an apiary: point, polygon, radius, zone type and notes.
- RU: радиусы 1/2/3/5 км от координат пасеки.
  EN: 1/2/3/5 km radiuses from apiary coordinates.
- RU: зоны медоносов с периодами цветения и связью с `honey_plants`.
  EN: honey plant zones with bloom periods and relation to `honey_plants`.
- RU: слои полей, садов, лесополос и потенциальных обработок пестицидами.
  EN: layers for fields, orchards, forest belts and potential pesticide treatments.
- RU: пользовательские заметки по точкам и зонам.
  EN: user notes for points and zones.
- RU: API карты пасеки, радиусов и слоев.
  EN: API for apiary map, radiuses and layers.
- RU: основу прогноза медосбора по медоносам, погоде и динамике веса.
  EN: foundation for honey flow forecast based on honey plants, weather and weight dynamics.

### 10. Режим кочевой пасеки / Migratory Apiary Mode - [#32](https://github.com/KirilyakSA/HiveMonitorESP/issues/32)

RU: Цель - поддержать профессиональных пасечников, которые перевозят партии ульев между стоянками под конкретные медоносы и культуры.

EN: Goal - support professional beekeepers who move hive batches between stands for specific honey plants and crops.

Нужно сделать / Required work:

- RU: модель партии ульев и состава партии.
  EN: hive batch model and batch membership.
- RU: GPS-точки стоянок и карточка стоянки.
  EN: GPS stand locations and stand cards.
- RU: история переездов: откуда, куда, когда, какие ульи/партии.
  EN: relocation history: from, to, when, which hives/batches.
- RU: акт размещения пасеки и контакт владельца поля.
  EN: apiary placement act/document and field owner contact.
- RU: культура/медонос и ожидаемый период цветения.
  EN: crop/honey plant and expected bloom period.
- RU: отчет по результату медосбора: период, динамика веса, погода, фактический результат.
  EN: honey flow result report: period, weight dynamics, weather and actual result.

### 11. Паспорт пасеки/партии меда и QR / Apiary or Honey Batch Passport and QR - [#33](https://github.com/KirilyakSA/HiveMonitorESP/issues/33)

RU: Цель - отчеты для продажи меда и доверия покупателей: публичная история партии меда без раскрытия точных координат и приватных данных.

EN: Goal - honey sales and buyer trust reports: a public honey batch story without exposing exact coordinates or private data.

Нужно сделать / Required work:

- RU: модель партии меда / harvest batch.
  EN: honey batch / harvest batch model.
- RU: связь партии с пасекой, стоянкой, периодом медосбора и ульями/партиями ульев.
  EN: relation between batch, apiary, stand, harvest period and hives/hive batches.
- RU: динамика набора веса и погодные условия в отчете.
  EN: weight gain dynamics and weather conditions in the report.
- RU: учет обработок пчел: препарат/метод, дата, безопасная публичная формулировка.
  EN: bee treatment records: product/method, date and safe public wording.
- RU: публичный QR token/slug с возможностью отключить страницу.
  EN: public QR token/slug with ability to disable the page.
- RU: API публичного паспорта без точных координат.
  EN: public passport API without exact coordinates.
- RU: будущий PDF/HTML export.
  EN: future PDF/HTML export.

## P2

- RU: Telegram/push/in-app notifications.
  EN: Telegram, push and in-app notifications.
- RU: Tariffs, limits and premium feature gates.
  EN: Tariffs, limits and premium feature gates.
- RU: Архивирование телеметрии после 1 года.
  EN: Telemetry archiving after 1 year.
- RU: OTA rollout service.
  EN: OTA rollout service.
- RU: AI/recommendation pipeline.
  EN: AI/recommendation pipeline.
