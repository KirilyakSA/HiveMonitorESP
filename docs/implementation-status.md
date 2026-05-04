# Статус реализации и соответствие ТЗ

Документ фиксирует актуальное состояние проекта после ревизии backend, firmware и документации.

## Краткий итог

Критических конфликтов, которые полностью ломают текущую MVP-вертикаль, не найдено. Текущая связка работает так:

```text
firmware -> hives/{deviceId}/telemetry -> mqtt-ingestion-service -> sensor_readings
```

Главное несоответствие последним требованиям: целевая модель provisioning по пасеке требует topic с `apiary_id`, а firmware пока публикует только legacy topic без `apiary_id`.

## P0 backlog в GitHub Issues

По итогам ревизии несоответствия вынесены в GitHub Issues с высшим приоритетом:

- Firmware: [#15](https://github.com/KirilyakSA/HiveMonitorESP/issues/15) - привести MQTT topics к модели пасек.
- Backend: [#16](https://github.com/KirilyakSA/HiveMonitorESP/issues/16) - ingest device events.
- Backend: [#17](https://github.com/KirilyakSA/HiveMonitorESP/issues/17) - ingest device status.
- Backend: [#18](https://github.com/KirilyakSA/HiveMonitorESP/issues/18) - command API для текущей firmware.
- Backend: [#19](https://github.com/KirilyakSA/HiveMonitorESP/issues/19) - alerts по пропущенным передачам вместо простого offline.
- Backend: [#20](https://github.com/KirilyakSA/HiveMonitorESP/issues/20) - production provisioning без `DEFAULT_APIARY_ID`.
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
- Firmware поддерживает `mqttUser` и `mqttPassword`, что совместимо с идеей credentials на уровне пасеки.

### Частично совместимо

- Backend поддерживает целевой topic `apiaries/{apiary_id}/devices/{device_id}/telemetry`, но firmware его еще не публикует.
- Backend умеет создать unassigned device, но для legacy topic без `apiary_id` устройство не попадет в конкретную пасеку без `DEFAULT_APIARY_ID`.
- Firmware публикует `hives/{deviceId}/events` и `hives/{deviceId}/status`, но backend MVP пока их не обрабатывает.
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

1. Добавить в конфигурацию устройства `apiaryId` или `mqttTopicPrefix`.
2. Перевести публикацию телеметрии на целевой topic:

```text
apiaries/{apiary_id}/devices/{device_id}/telemetry
```

3. Оставить legacy `hives/{deviceId}/telemetry` как compatibility mode на переходный период.
4. Добавить публикацию результатов команд в целевой topic, когда backend command-service будет реализован.

### Backend

1. Добавить ingestion для `events` и `status`.
2. Добавить `device_commands` API и MQTT publish в `hives/{deviceId}/commands` для совместимости с текущей firmware.
3. Добавить scheduler для `missed_telemetry_count`.
4. Добавить базовые alert/event/tag таблицы.
5. Добавить production MQTT auth/ACL вместо anonymous Mosquitto.

### Документация

Документация должна различать:

- реализовано сейчас;
- совместимость с текущей firmware;
- целевой контракт по ТЗ;
- будущие инкременты.
