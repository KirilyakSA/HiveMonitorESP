# Статус реализации и соответствие ТЗ

Документ фиксирует актуальное состояние проекта после ревизии backend, firmware и документации.

## Краткий итог

Критических конфликтов, которые полностью ломают текущую MVP-вертикаль, не найдено. Текущая связка работает так:

```text
firmware -> hives/{deviceId}/... или apiaries/{apiaryId}/devices/{deviceId}/... -> mqtt-ingestion-service
```

Главное несоответствие по provisioning закрыто на уровне firmware/backend контракта: firmware умеет публиковать apiary-aware topics через настройку `apiaryId`, а backend принимает telemetry/events/status в этом namespace. Legacy topics остаются compatibility mode.

## P0 backlog в GitHub Issues / P0 Backlog in GitHub Issues

RU: По итогам ревизии несоответствия вынесены в GitHub Issues с высшим приоритетом.

EN: After the project review, mismatches were moved to GitHub Issues with the highest priority.

- Firmware: [#15](https://github.com/KirilyakSA/HiveMonitorESP/issues/15)
  - RU: привести MQTT topics к модели пасек. Реализовано и закрыто.
  - EN: align MQTT topics with the apiary model. Implemented and closed.
- Backend: [#16](https://github.com/KirilyakSA/HiveMonitorESP/issues/16)
  - RU: ingest device events. Реализовано и закрыто.
  - EN: ingest device events. Implemented and closed.
- Backend: [#17](https://github.com/KirilyakSA/HiveMonitorESP/issues/17)
  - RU: ingest device status. Реализовано и закрыто.
  - EN: ingest device status. Implemented and closed.
- Backend: [#18](https://github.com/KirilyakSA/HiveMonitorESP/issues/18)
  - RU: command API для текущей firmware.
  - EN: command API for the current firmware.
- Backend: [#19](https://github.com/KirilyakSA/HiveMonitorESP/issues/19)
  - RU: alerts по пропущенным передачам вместо простого offline.
  - EN: alerts for missed transmissions instead of a simple offline state.
- Backend: [#20](https://github.com/KirilyakSA/HiveMonitorESP/issues/20)
  - RU: production provisioning без `DEFAULT_APIARY_ID`. Реализовано и закрыто.
  - EN: production provisioning without `DEFAULT_APIARY_ID`. Implemented and closed.
- Deploy: [#21](https://github.com/KirilyakSA/HiveMonitorESP/issues/21)
  - RU: закрыть anonymous MQTT в production profile.
  - EN: disable anonymous MQTT in the production profile.
- Deploy: [#22](https://github.com/KirilyakSA/HiveMonitorESP/issues/22)
  - RU: TLS/reverse proxy план для VPS.
  - EN: TLS/reverse proxy plan for VPS.
- Deploy: [#23](https://github.com/KirilyakSA/HiveMonitorESP/issues/23)
  - RU: backup/restore PostgreSQL.
  - EN: PostgreSQL backup/restore.
- Web: [#24](https://github.com/KirilyakSA/HiveMonitorESP/issues/24)
  - RU: привязка unassigned device к улью. Реализовано и закрыто.
  - EN: assign an unassigned device to a hive. Implemented and closed.
- Web: [#25](https://github.com/KirilyakSA/HiveMonitorESP/issues/25)
  - RU: статус по пропущенным передачам.
  - EN: missed-transmission status.
- Web: [#28](https://github.com/KirilyakSA/HiveMonitorESP/issues/28)
  - RU: нормальный UX привязки устройства к улью с выбором улья, режима исторических данных и поведения при уже привязанном устройстве. Реализовано и закрыто.
  - EN: proper device-to-hive assignment UX with hive selection, historical data mode and existing-device handling. Implemented and closed.
- Web: [#27](https://github.com/KirilyakSA/HiveMonitorESP/issues/27)
  - RU: поддержать периоды 1/10/30 дней для истории телеметрии и сравнительного графика. Реализовано и закрыто.
  - EN: support 1/10/30 day periods for telemetry history and the comparison chart. Implemented and closed.
- Mobile: [#26](https://github.com/KirilyakSA/HiveMonitorESP/issues/26)
  - RU: MVP экраны пасеки и улья.
  - EN: MVP apiary and hive screens.

Существующая firmware-задача [#2](https://github.com/KirilyakSA/HiveMonitorESP/issues/2) дополнена комментарием про apiary-aware command topics.

## Совместимость backend и firmware

### Совместимо

- Firmware публикует `hives/{deviceId}/telemetry`.
- Backend подписан на `hives/+/telemetry`.
- Firmware публикует apiary-aware topics `apiaries/{apiaryId}/devices/{deviceId}/telemetry|events|status`, если задан `apiaryId`.
- Backend подписан на apiary-aware telemetry/events/status topics.
- Firmware payload содержит поля, которые backend парсит:
  - `deviceId`;
  - `timestamp`;
  - `firmwareVersion`;
  - `configVersion`;
  - `measurementIntervalSeconds`;
  - `weight`;
  - `weightChange`;
  - `temperature`;
  - `humidity`;
  - `hiveOpened`;
  - `batteryPercent`;
  - `batteryVoltage`;
  - `rssi`;
  - `freeHeap`.
- Backend сохраняет raw payload и нормализованные readings.
- Backend сохраняет device events и обновляет `last_status_at` по status messages.
- Firmware поддерживает `mqttUser` и `mqttPassword`, что совместимо с идеей credentials на уровне пасеки.

### Частично совместимо

- Firmware принимает MQTT-команды, но backend MVP пока не имеет command API/service.

### Устаревшее в старой документации

- `deviceId + deviceToken` больше не является основным backend-MVP способом авторизации устройства.
- `deviceToken` остается в firmware как legacy/fallback поле локальной конфигурации.
- Для спящих устройств термин `offline` не должен использоваться как прямой alert. Правильная логика - пропущенные плановые передачи.
- Push-уведомления и mobile MVP пока не реализованы.
- OTA через backend является целевым требованием, но не реализовано.

## Требования, которые уже отражены в ТЗ, но не реализованы

- Организационные тарифы и лимиты.
- Участники организации и пасеки с полноценной permission matrix.
- Email invitations.
- Tasks и recurring tasks.
- Журнал событий пасеки и улья.
- Вложения к событиям.
- Reminders.
- Alert rules.
- Alerts создают события, системные теги и уведомления.
- Push и Telegram delivery.
- Weather providers и метеостанция пасеки.
- Архив телеметрии после 1 года.
- OTA rollout.
- AI premium функции.

## Рекомендации

### Firmware

1. Поддерживать `apiaryId` как production-настройку устройства.
2. Оставить legacy `hives/{deviceId}/...` как compatibility mode на переходный период.
3. После реализации backend command-service проверить полный command loop на apiary-aware topics.

### Backend

1. Добавить `device_commands` API и MQTT publish в firmware command topics.
2. Добавить scheduler для `missed_telemetry_count`.
3. Добавить базовые alert/tag таблицы.
4. Добавить production MQTT auth/ACL вместо anonymous Mosquitto.

### Документация

Документация должна различать:

- реализовано сейчас;
- совместимость с текущей firmware;
- целевой контракт по ТЗ;
- будущие инкременты.
