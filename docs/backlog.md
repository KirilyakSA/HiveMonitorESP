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
   - RU: command API для текущей firmware.
   - EN: implement the command API for the current firmware.
4. Backend: [#19](https://github.com/KirilyakSA/HiveMonitorESP/issues/19)
   - RU: alerts по пропущенным плановым передачам.
   - EN: alerts for missed scheduled telemetry transmissions.
5. Deploy: [#21](https://github.com/KirilyakSA/HiveMonitorESP/issues/21)
   - RU: production MQTT auth/ACL вместо anonymous access.
   - EN: production MQTT auth/ACL instead of anonymous access.
6. Deploy: [#23](https://github.com/KirilyakSA/HiveMonitorESP/issues/23)
   - RU: backup/restore PostgreSQL.
   - EN: PostgreSQL backup/restore procedure.

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
