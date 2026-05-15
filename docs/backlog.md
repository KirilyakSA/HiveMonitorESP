# Backlog HiveMonitor / Беклог HiveMonitor

RU: Этот документ фиксирует ближайшую очередь работ по проекту. Канонический список исполняемых задач остается в GitHub Issues, а здесь хранится удобная карта приоритетов по подпроектам.

EN: This document captures the nearest project backlog. GitHub Issues remain the canonical execution tracker, while this file provides a readable priority map across subprojects.

## P0 - текущая очередь / Current Queue

1. Web: [#25](https://github.com/KirilyakSA/HiveMonitorESP/issues/25)
   - RU: показывать состояние пропущенных передач вместо простого offline. Зависит от backend #19.
   - EN: show missed telemetry state instead of a simple offline state. Depends on backend #19.
2. Web: [#28](https://github.com/KirilyakSA/HiveMonitorESP/issues/28)
   - RU: сделать нормальный UX привязки устройства к улью с выбором улья и режима исторических данных.
   - EN: build proper device-to-hive assignment UX with hive selection and historical data mode.
3. Backend: [#18](https://github.com/KirilyakSA/HiveMonitorESP/issues/18)
   - RU: command API для текущей firmware: `reboot`, `config_update`, `firmware_update` как deferred commands. Firmware tare/calibration выполняются локально на устройстве.
   - EN: command API for the current firmware: `reboot`, `config_update`, `firmware_update` as deferred commands. Firmware tare/calibration are performed locally on the device.
4. Backend: [#19](https://github.com/KirilyakSA/HiveMonitorESP/issues/19)
   - RU: alerts по пропущенным плановым передачам.
   - EN: alerts for missed scheduled telemetry transmissions.
5. Deploy: [#21](https://github.com/KirilyakSA/HiveMonitorESP/issues/21)
   - RU: production MQTT auth/ACL вместо anonymous access.
   - EN: production MQTT auth/ACL instead of anonymous access.
6. Deploy: [#23](https://github.com/KirilyakSA/HiveMonitorESP/issues/23)
   - RU: backup/restore PostgreSQL.
   - EN: PostgreSQL backup/restore procedure.
7. Backend/Web: [#46](https://github.com/KirilyakSA/HiveMonitorESP/issues/46)
   - RU: усилить безопасное удаление: soft-delete/archive policy, audit trail и подробный выбор сохранения истории.
   - EN: harden safe deletion: soft-delete/archive policy, audit trail and detailed history retention choices.

## Реализовано и закрыто / Implemented and Closed

- Firmware: [#15](https://github.com/KirilyakSA/HiveMonitorESP/issues/15)
  - RU: apiary-aware MQTT topics.
  - EN: apiary-aware MQTT topics.
- Backend: [#16](https://github.com/KirilyakSA/HiveMonitorESP/issues/16)
  - RU: ingest device events.
  - EN: ingest device events.
- Backend: [#17](https://github.com/KirilyakSA/HiveMonitorESP/issues/17)
  - RU: ingest device status.
  - EN: ingest device status.
- Backend: [#20](https://github.com/KirilyakSA/HiveMonitorESP/issues/20)
  - RU: production provisioning path без жесткой привязки к `DEFAULT_APIARY_ID`.
  - EN: production provisioning path without hard dependency on `DEFAULT_APIARY_ID`.
- Web: [#24](https://github.com/KirilyakSA/HiveMonitorESP/issues/24)
  - RU: привязка unassigned device к улью.
  - EN: assign an unassigned device to a hive.
- Web: [#27](https://github.com/KirilyakSA/HiveMonitorESP/issues/27)
  - RU: периоды 1/10/30 дней для history telemetry и графиков.
  - EN: 1/10/30 day periods for telemetry history and charts.

## Следующий слой после P0 / Next Layer After P0

- Product/Backend/Web/Mobile: [#29](https://github.com/KirilyakSA/HiveMonitorESP/issues/29)
  - RU: журнал осмотров улья как “медицинская карта семьи”: структурированные осмотры, матка, расплод, корм, признаки роения, болезни/клещ, выполненные работы, фото рамок и следующий рекомендуемый осмотр.
  - EN: hive inspection journal as a colony medical record: structured inspections, queen, brood, feed, swarming signs, disease/varroa, completed works, frame photos and next recommended inspection.
- Analytics/Backend/Web: [#30](https://github.com/KirilyakSA/HiveMonitorESP/issues/30)
  - RU: погодный контекст как аналитический слой: вес улья + температура улья + внешняя температура + дождь + ветер + влажность + давление + сравнение с соседними ульями.
  - EN: weather context as an analytics layer: hive weight + hive temperature + outside temperature + rain + wind + humidity + pressure + comparison with neighboring hives.
- Geo/Backend/Web: [#31](https://github.com/KirilyakSA/HiveMonitorESP/issues/31)
  - RU: карта радиуса лёта и медоносов: радиусы 1/2/3/5 км, зоны медоносов, поля, сады, лесополосы, риски обработок, заметки и прогноз медосбора.
  - EN: foraging radius and honey plant map: 1/2/3/5 km radiuses, honey plant zones, fields, orchards, forest belts, treatment risks, notes and honey flow forecast.
- Professional/Backend/Web/Mobile: [#32](https://github.com/KirilyakSA/HiveMonitorESP/issues/32)
  - RU: режим кочевой пасеки: партии ульев, GPS-точки стоянок, история переездов, акт размещения, контакт владельца поля, культура/медонос и отчет медосбора.
  - EN: migratory apiary mode: hive batches, GPS stand locations, relocation history, placement act, field owner contact, crop/honey plant and honey flow result report.
- Reports/Backend/Web: [#33](https://github.com/KirilyakSA/HiveMonitorESP/issues/33)
  - RU: паспорт пасеки/партии меда и QR-код для покупателей: история происхождения, период медосбора, динамика веса, погода, обработки без раскрытия точных координат.
  - EN: apiary/honey batch passport and QR code for buyers: origin story, harvest period, weight dynamics, weather and treatments without exposing exact coordinates.
- Dashboard/Backend/Web: [#34](https://github.com/KirilyakSA/HiveMonitorESP/issues/34)
  - RU: карточка “Проблемные ульи сегодня”: агрегировать причины внимания из alerts, задач, советов, телеметрии и событий.
  - EN: “Problem hives today” card: aggregate attention reasons from alerts, tasks, advice, telemetry and events.
- Calendar/Backend/Web: [#35](https://github.com/KirilyakSA/HiveMonitorESP/issues/35)
  - RU: календарь работ по сезонам: сезонные периоды, фильтры, задачи из шаблонов, ручные задачи, связи с советами/осмотрами/событиями.
  - EN: seasonal work calendar: seasonal periods, filters, template tasks, manual tasks, links with advice/inspections/events.
- Events/Backend/Web/Mobile: [#36](https://github.com/KirilyakSA/HiveMonitorESP/issues/36)
  - RU: ручные события, комментарии и фото к событию/осмотру с timeline истории улья.
  - EN: manual events, comments and photos for events/inspections with hive history timeline.
- Web/Backend: [#37](https://github.com/KirilyakSA/HiveMonitorESP/issues/37)
  - RU: схема расположения ульев на пасеке: абстрактные координаты, ряд/колонка/сетка/периметр/freeform и drag-and-drop.
  - EN: apiary hive layout map: abstract coordinates, row/column/grid/perimeter/freeform and drag-and-drop.
- Backend/Web: [#46](https://github.com/KirilyakSA/HiveMonitorESP/issues/46)
  - RU: production hardening для удаления устройства, улья и пасеки: soft-delete/archive, audit trail, restore flow и подробные режимы сохранения истории.
  - EN: production hardening for deleting device, hive and apiary: soft-delete/archive, audit trail, restore flow and detailed history retention modes.
- Analytics/Backend/Web: [#38](https://github.com/KirilyakSA/HiveMonitorESP/issues/38)
  - RU: weather analytics: корреляция веса с дождем, ветром, температурой, влажностью и давлением, объяснения отклонений.
  - EN: weather analytics: correlate weight with rain, wind, temperature, humidity and pressure, explain anomalies.
- Analytics/Backend/Web: [#39](https://github.com/KirilyakSA/HiveMonitorESP/issues/39)
  - RU: прогноз медосбора на основе медоносов, цветения, погоды и динамики веса ульев.
  - EN: honey flow forecast based on honey plants, bloom periods, weather and hive weight dynamics.
- Recommendations/Backend/Web/Mobile: [#40](https://github.com/KirilyakSA/HiveMonitorESP/issues/40)
  - RU: антироевые подсказки: риск роения, сигналы из осмотров/веса/сезона и создание связанных задач.
  - EN: anti-swarming recommendations: swarming risk, signals from inspections/weight/season and linked tasks.
- Security/Firmware/Backend/Web/Mobile: [#41](https://github.com/KirilyakSA/HiveMonitorESP/issues/41)
  - RU: защита от кражи и вандализма: MVP на текущих сигналах веса/открытия/RSSI/батареи плюс будущие GPS/наклон/geo-fence и тревоги Telegram.
  - EN: theft and vandalism protection: MVP using current weight/opening/RSSI/battery signals plus future GPS/tilt/geofence and Telegram alarms.
- Firmware/Backend/Web: [#45](https://github.com/KirilyakSA/HiveMonitorESP/issues/45)
  - RU: workflow калибровки весов и тарирования улья/магазина: локально и через backend, сохранение базового веса пустого улья, веса семьи с рамками, пустого магазина и истории операций.
  - EN: scale calibration and hive/super tare workflow: local and backend-driven, store empty-hive baseline, colony-with-frames weight, empty-super baseline and operation history.
- Firmware/Backend/Web:
  - RU: четко разделить firmware tare/calibration и backend hive/super tare: firmware tare/калибровка делаются в web UI устройства, backend-тара улья/магазинов хранится в базе и вычитается из сырого веса.
  - EN: clearly separate firmware tare/calibration and backend hive/super tare: firmware tare/calibration are done in the device web UI, backend hive/super tare is stored in DB and subtracted from raw weight.
- Backend/Web:
  - RU: `scale_profile` и история тарирования: тара пустого улья сбрасывает отображаемый вес в ноль; тары магазинов 2/3/4... добавляются/снимаются слоями с возвратом к предыдущей активной таре.
  - EN: `scale_profile` and tare history: empty-hive tare resets displayed weight to zero; super tares 2/3/4... are added/removed in layers with fallback to the previous active tare.
- AI/Backend/Web/Mobile: [#42](https://github.com/KirilyakSA/HiveMonitorESP/issues/42)
  - RU: анализ звука ульев: аудио-признаки, паттерны, аномалии и объяснимые подсказки.
  - EN: hive audio analysis: audio features, patterns, anomalies and explainable recommendations.
- AI/Backend/Web/Mobile: [#43](https://github.com/KirilyakSA/HiveMonitorESP/issues/43)
  - RU: анализ изображений рамок: расплод, корм, запечатка, маточники, видимые проблемы и качество фото.
  - EN: frame image analysis: brood, feed stores, capping, queen cells, visible issues and photo quality.
- AI/Backend/Web/Mobile: [#44](https://github.com/KirilyakSA/HiveMonitorESP/issues/44)
  - RU: AI-помощник пасечника: объясняет данные, отвечает по пасеке/улью, предлагает план работ с guardrails.
  - EN: beekeeper AI assistant: explains data, answers by apiary/hive, suggests work plans with guardrails.
- Web:
  - RU: вынести mock weather/tips/scheduled events в отдельный data/module слой.
  - EN: move mock weather, tips and scheduled events into a dedicated data/module layer.
- Web:
  - RU: подключить placeholder-assets во все empty states.
  - EN: connect placeholder assets to all empty states.
- Backend:
  - RU: базовые alert rules и системные теги.
  - EN: basic alert rules and system tags.
- Backend:
  - RU: tasks/recurring tasks/reminders.
  - EN: tasks, recurring tasks and reminders.
- Backend:
  - RU: weather providers и поддержка метеостанции как IoT-устройства.
  - EN: weather providers and weather station support as an IoT device.
- Backend:
  - RU: roles/permissions на уровне организации и пасеки.
  - EN: organization-level and apiary-level roles and permissions.
- Backend:
  - RU: Telegram/push/in-app notifications.
  - EN: Telegram, push and in-app notifications.
- Deploy:
  - RU: TLS/reverse proxy для API, web и MQTT.
  - EN: TLS/reverse proxy for API, web and MQTT.

## Отложено за MVP / Deferred Beyond MVP

- RU: Billing/tariffs/limits.
  EN: Billing, tariffs and limits.
- RU: AI premium функции.
  EN: AI premium features.
- RU: Архивирование телеметрии после 1 года.
  EN: Telemetry archiving after 1 year.
- RU: OTA rollout UI и backend rollout service.
  EN: OTA rollout UI and backend rollout service.
- Mobile: [#26](https://github.com/KirilyakSA/HiveMonitorESP/issues/26)
  - RU: MVP экраны пасеки и улья.
  - EN: MVP apiary and hive screens.
