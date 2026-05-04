# Статус реализации и соответствие ТЗ

Документ фиксирует актуальное состояние проекта после ревизии backend, firmware и документации.

## Краткий итог

Критических конфликтов, которые полностью ломают текущую MVP-вертикаль, не найдено. Текущая связка работает так:

```text
firmware -> hives/{deviceId}/... или apiaries/{apiaryId}/devices/{deviceId}/... -> mqtt-ingestion-service
```

Главное несоответствие по provisioning закрыто на уровне firmware/backend контракта: firmware умеет публиковать apiary-aware topics через настройку `apiaryId`, а backend принимает telemetry/events/status в этом namespace. Legacy topics остаются compatibility mode.

## P0 backlog в GitHub Issues

По итогам ревизии несоответствия вынесены в GitHub Issues с высшим приоритетом:

- Firmware: [#15](https://github.com/KirilyakSA/HiveMonitorESP/issues/15) - привести MQTT topics к модели пасек. Реализовано в коде, issue можно закрывать после review.
- Backend: [#16](https://github.com/KirilyakSA/HiveMonitorESP/issues/16) - ingest device events. Реализовано в коде, issue можно закрывать после review.
- Backend: [#17](https://github.com/KirilyakSA/HiveMonitorESP/issues/17) - ingest device status. Реализовано в коде, issue можно закрывать после review.
- Backend: [#18](https://github.com/KirilyakSA/HiveMonitorESP/issues/18) - command API для текущей firmware.
- Backend: [#19](https://github.com/KirilyakSA/HiveMonitorESP/issues/19) - alerts по пропущенным передачам вместо простого offline.
- Backend: [#20](https://github.com/KirilyakSA/HiveMonitorESP/issues/20) - production provisioning без `DEFAULT_APIARY_ID`. Реализовано в коде, issue можно закрывать после review.
- Deploy: [#21](https://github.com/KirilyakSA/HiveMonitorESP/issues/21) - закрыть anonymous MQTT в production profile.
- Deploy: [#22](https://github.com/KirilyakSA/HiveMonitorESP/issues/22) - TLS/reverse proxy план для VPS.
- Deploy: [#23](https://github.com/KirilyakSA/HiveMonitorESP/issues/23) - backup/restore PostgreSQL.
- Web: [#24](https://github.com/KirilyakSA/HiveMonitorESP/issues/24) - привязка unassigned device к улью.
- Web: [#25](https://github.com/KirilyakSA/HiveMonitorESP/issues/25) - статус по пропущенным передачам.
- Mobile: [#26](https://github.com/KirilyakSA/HiveMonitorESP/issues/26) - MVP экраны пасеки и улья.

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
