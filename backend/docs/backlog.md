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

### 8. Scale calibration and hive/super tare workflow / Калибровка весов и тарирование улья/магазина - [#45](https://github.com/KirilyakSA/HiveMonitorESP/issues/45)

RU: Цель - сделать весовые данные интерпретируемыми: отделить калибровку датчика от тарирования пустого улья, семьи с рамками и магазинов.

EN: Goal - make weight data interpretable by separating sensor calibration from tare operations for the empty hive, colony with frames and honey supers.

Нужно сделать / Required work:

- RU: модели/таблицы для истории калибровок, истории тарирования и активного `scale_profile` улья/устройства.
  EN: models/tables for calibration history, tare history and active hive/device `scale_profile`.
- RU: backend command/API для `calibrate_scale`, `tare_empty_hive`, `tare_super`.
  EN: backend command/API for `calibrate_scale`, `tare_empty_hive`, `tare_super`.
- RU: после калибровки поддержать тарирование пустого улья без рамок, но с крышкой; сохранить этот вес как нулевое/стартовое значение улья.
  EN: after calibration, support taring an empty hive without frames but with cover; store this as the hive zero/baseline value.
- RU: при установке магазина поддержать второе тарирование: улей с семьей и рамками + пустой магазин без рамок; сохранить вес семьи с рамками и базовый вес пустого магазина.
  EN: when adding a super, support a second tare: hive with colony and frames + empty super without frames; store colony-with-frames weight and empty-super baseline.
- RU: хранить пользователя, источник операции (`local_device`, `backend_command`), время, результат, параметры и комментарий.
  EN: store user, operation source (`local_device`, `backend_command`), time, result, parameters and comment.
- RU: связать результат firmware ack/status с command journal.
  EN: link firmware ack/status result with the command journal.
- RU: web/mobile UI для мастера калибровки/тарирования и просмотра истории.
  EN: web/mobile UI for calibration/tare wizard and history view.
- RU: использовать активный профиль веса в аналитике: общий вес, прирост меда, вес семьи, вклад магазинов.
  EN: use active weight profile in analytics: gross weight, honey gain, colony weight and super contribution.

### 9. Roles and permissions / Роли и права доступа

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

### 10. Карта радиуса лёта и медоносов / Foraging Radius and Honey Plant Map - [#31](https://github.com/KirilyakSA/HiveMonitorESP/issues/31)

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

### 11. Режим кочевой пасеки / Migratory Apiary Mode - [#32](https://github.com/KirilyakSA/HiveMonitorESP/issues/32)

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

### 12. Паспорт пасеки/партии меда и QR / Apiary or Honey Batch Passport and QR - [#33](https://github.com/KirilyakSA/HiveMonitorESP/issues/33)

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

### 13. Карточка проблемных ульев сегодня / Problem Hives Today Card - [#34](https://github.com/KirilyakSA/HiveMonitorESP/issues/34)

RU: Цель - дать пасечнику быстрый ежедневный список ульев, которые сегодня требуют внимания, с понятной причиной.

EN: Goal - give the beekeeper a fast daily list of hives requiring attention today, with a clear reason.

Нужно сделать / Required work:

- RU: агрегировать причины из alert rules, missed telemetry, задач, советов, телеметрии и событий.
  EN: aggregate reasons from alert rules, missed telemetry, tasks, advice, telemetry and events.
- RU: endpoint `GET /apiaries/{apiaryID}/problem-hives?date=...`.
  EN: endpoint `GET /apiaries/{apiaryID}/problem-hives?date=...`.
- RU: возвращать улей, severity, причину, объяснение, последнее измерение/событие и related entities.
  EN: return hive, severity, reason, explanation, latest measurement/event and related entities.
- RU: поддержать действие “скрыть на сегодня”.
  EN: support “hide for today”.

### 14. Календарь работ по сезонам / Seasonal Work Calendar - [#35](https://github.com/KirilyakSA/HiveMonitorESP/issues/35)

RU: Цель - развить текущий календарь работ в полноценную сезонную картину пасеки: текущие, будущие и просроченные работы по периодам пасечного года.

EN: Goal - evolve the current work calendar into a full seasonal apiary view: current, upcoming and overdue work by beekeeping periods.

Нужно сделать / Required work:

- RU: сезонные периоды и настраиваемые диапазоны.
  EN: seasonal periods and configurable ranges.
- RU: фильтры по пасеке, улью, категории, статусу и периоду.
  EN: filters by apiary, hive, category, status and period.
- RU: генерация задач из шаблонов без дублей.
  EN: template-based task generation without duplicates.
- RU: ручное добавление задач.
  EN: manual task creation.
- RU: связи task -> advice/inspection/event.
  EN: task -> advice/inspection/event links.
- RU: выполнение, откладывание, скрытие, комментарий и результат работы.
  EN: complete, snooze, dismiss, comment and work result.

### 15. Ручные события, комментарии и фото / Manual Events, Comments and Photos - [#36](https://github.com/KirilyakSA/HiveMonitorESP/issues/36)

RU: Цель - дать пасечнику возможность вручную фиксировать наблюдения, комментарии, результаты работ и фото в истории пасеки/улья.

EN: Goal - allow beekeepers to manually record observations, comments, work results and photos in apiary/hive history.

Нужно сделать / Required work:

- RU: ручные события на уровне пасеки и улья.
  EN: manual events at apiary and hive level.
- RU: комментарии к событиям.
  EN: event comments.
- RU: результат выполненной работы.
  EN: completed work result.
- RU: фото к событию, осмотру и рамкам.
  EN: photos for events, inspections and frames.
- RU: связь события с задачей, советом или осмотром.
  EN: link event with task, advice or inspection.
- RU: file storage abstraction, permissions и audit автора.
  EN: file storage abstraction, permissions and author audit.

### 16. Схема расположения ульев / Apiary Hive Layout Map - [#37](https://github.com/KirilyakSA/HiveMonitorESP/issues/37)

RU: Цель - хранить и показывать абстрактную схему расположения ульев на пасеке без GPS-координат каждого улья.

EN: Goal - store and display an abstract apiary hive layout without GPS coordinates for each hive.

Нужно сделать / Required work:

- RU: layout position для улья или таблицу `apiary_hive_layout`.
  EN: hive layout position fields or an `apiary_hive_layout` table.
- RU: x/y, row/column, order, layout mode и заметки по месту.
  EN: x/y, row/column, order, layout mode and position notes.
- RU: режимы: ряд, колонка, сетка, периметр, произвольно.
  EN: modes: row, column, grid, perimeter, freeform.
- RU: API чтения/сохранения схемы.
  EN: API for reading/saving layout.
- RU: статусы, алармы, теги и задачи как визуальные слои на схеме.
  EN: statuses, alerts, tags and tasks as visual layers on the layout.

### 17. Weather analytics / Погодная аналитика - [#38](https://github.com/KirilyakSA/HiveMonitorESP/issues/38)

RU: Цель - превратить погодный контекст в аналитический слой, который объясняет телеметрию ульев.

EN: Goal - turn weather context into an analytics layer that explains hive telemetry.

Нужно сделать / Required work:

- RU: корреляция веса с температурой, дождем, ветром, влажностью и давлением.
  EN: correlate weight with temperature, rain, wind, humidity and pressure.
- RU: сравнение динамики ульев между собой с учетом одинаковой погоды.
  EN: compare hive dynamics under the same weather conditions.
- RU: объяснения: “вес не растет из-за плохой погоды” или “улей отстает при хорошей погоде”.
  EN: explanations: “weight does not grow because of bad weather” or “hive lags under favorable weather”.
- RU: погодные слои на графиках и агрегаты для отчетов.
  EN: weather layers on charts and aggregates for reports.

### 18. Прогноз медосбора / Honey Flow Forecast - [#39](https://github.com/KirilyakSA/HiveMonitorESP/issues/39)

RU: Цель - прогнозировать потенциал медосбора по медоносам, цветению, погоде и динамике веса ульев.

EN: Goal - forecast honey flow potential using honey plants, bloom periods, weather and hive weight dynamics.

Нужно сделать / Required work:

- RU: учитывать медоносы и периоды цветения вокруг пасеки.
  EN: account for honey plants and bloom periods around the apiary.
- RU: учитывать weather analytics: дождь, ветер, температура, влажность.
  EN: use weather analytics: rain, wind, temperature and humidity.
- RU: учитывать текущий весовой прирост и сравнение ульев внутри пасеки.
  EN: use current weight gain and comparison between hives inside the apiary.
- RU: показывать прогноз ожидаемого взятка, изменения потенциала и завершения активного периода.
  EN: show forecast for expected flow, potential changes and active period ending.

### 19. Антироевые подсказки / Anti-Swarming Recommendations - [#40](https://github.com/KirilyakSA/HiveMonitorESP/issues/40)

RU: Цель - помогать пасечнику замечать риск роения и планировать безопасные действия-подсказки без жесткого автопилота.

EN: Goal - help the beekeeper notice swarming risk and plan safe recommendation-style actions without rigid autopilot.

Нужно сделать / Required work:

- RU: сигналы из сезона, роста семьи, отсутствия места, веса, осмотров, погоды и медоносов.
  EN: signals from season, colony growth, lack of space, weight, inspections, weather and honey plants.
- RU: риск роения по улью.
  EN: swarming risk per hive.
- RU: подсказки: осмотр, расширение, проверка маточников, создание отводка.
  EN: recommendations: inspection, expansion, queen-cell check, split/nuc creation.
- RU: связь подсказки с календарной задачей.
  EN: link recommendation with a calendar task.

### 20. Защита от кражи, GPS и наклон / Theft, GPS and Tilt Monitoring - [#41](https://github.com/KirilyakSA/HiveMonitorESP/issues/41)

RU: Цель - защитить удаленные пасеки от кражи и вандализма. Первый MVP должен использовать уже доступные сигналы: вес, датчик открытия, RSSI, батарея и missed telemetry. GPS/tilt/удар датчики остаются следующим расширением.

EN: Goal - protect remote apiaries from theft and vandalism. The first MVP should use already available signals: weight, opening sensor, RSSI, battery and missed telemetry. GPS/tilt/shock sensors remain the next extension.

Нужно сделать / Required work:

- RU: резкое падение веса ночью.
  EN: sudden weight drop at night.
- RU: открытие улья вне разрешенного времени.
  EN: hive opening outside allowed time.
- RU: пропажа связи после события открытия.
  EN: connection loss after an opening event.
- RU: тревога в Telegram, в будущем push/in-app.
  EN: Telegram alarm, later push/in-app.
- RU: режим охраны и разрешенные временные окна обслуживания.
  EN: armed mode and allowed maintenance time windows.
- RU: поддержать GPS/tilt датчики как отдельные устройства или модули.
  EN: support GPS/tilt sensors as separate devices or modules.
- RU: события наклона, перемещения, удара и выхода из geo-fence.
  EN: tilt, movement, shock and geofence-exit events.
- RU: тревоги на уровне улья/пасеки.
  EN: alerts at hive/apiary level.
- RU: карта последней позиции и история перемещений.
  EN: last position map and movement history.
- RU: настройки чувствительности и режим охраны.
  EN: sensitivity settings and armed mode.

### 21. Анализ звука ульев / Hive Audio Analysis - [#42](https://github.com/KirilyakSA/HiveMonitorESP/issues/42)

RU: Цель - добавить audio analysis как будущую premium/AI-функцию.

EN: Goal - add audio analysis as a future premium/AI feature.

Нужно сделать / Required work:

- RU: поддержать аудио-датчик/устройство.
  EN: support audio sensor/device.
- RU: хранить аудио-сэмплы или извлеченные признаки.
  EN: store audio samples or extracted features.
- RU: анализировать шум, частоты и паттерны.
  EN: analyze noise, frequencies and patterns.
- RU: искать признаки роевого состояния, беспокойства, проблем с маткой и аномалий.
  EN: detect possible swarming, stress, queen issues and anomalies.
- RU: показывать объяснимые подсказки и уверенность модели без жестких диагнозов.
  EN: show explainable recommendations and model confidence without hard diagnoses.

### 22. Анализ изображений рамок / Frame Image Analysis - [#43](https://github.com/KirilyakSA/HiveMonitorESP/issues/43)

RU: Цель - добавить image analysis рамок как AI-функцию, связанную с журналом осмотров.

EN: Goal - add frame image analysis as an AI feature linked to the inspection journal.

Нужно сделать / Required work:

- RU: фото рамок в осмотре.
  EN: frame photos in inspections.
- RU: классификация расплода, кормов, запечатки, маточников и видимых проблем.
  EN: classify brood, feed stores, capping, queen cells and visible issues.
- RU: подсказки по качеству фото.
  EN: photo quality hints.
- RU: объяснимые результаты с уверенностью.
  EN: explainable results with confidence.
- RU: связь с журналом осмотров и рекомендациями.
  EN: link to inspection journal and recommendations.

### 23. AI-помощник пасечника / Beekeeper AI Assistant - [#44](https://github.com/KirilyakSA/HiveMonitorESP/issues/44)

RU: Цель - premium AI-помощник, который объясняет данные и помогает планировать работы.

EN: Goal - premium AI assistant that explains data and helps plan work.

Нужно сделать / Required work:

- RU: отвечать на вопросы по конкретной пасеке/улью.
  EN: answer questions about a specific apiary/hive.
- RU: объяснять графики, события, советы, осмотры и погоду.
  EN: explain charts, events, advice, inspections and weather.
- RU: предлагать план работ без жесткого автопилота.
  EN: suggest work plans without rigid autopilot.
- RU: ссылаться на факты: телеметрия, журнал осмотров, задачи, погода.
  EN: cite facts: telemetry, inspection journal, tasks and weather.
- RU: соблюдать guardrails: без опасных дозировок препаратов и диагнозов без осмотра.
  EN: follow guardrails: no dangerous treatment dosages and no hard diagnoses without inspection.

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
