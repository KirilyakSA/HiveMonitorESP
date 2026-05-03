# Техническое задание: backend HiveMonitor

## 1. Назначение

Backend HiveMonitor - сервисная система для управления пасеками, ульями, IoT-устройствами, телеметрией, задачами, событиями, уведомлениями, погодными данными, OTA-обновлениями и будущими AI-функциями.

Главный объект продукта для пользователя - не устройство, а пасека и улей. Устройство является источником данных и может быть заменено, перенесено или отвязано без потери истории улья.

## 2. Ключевая модель

```text
Organization
  -> Users / members
  -> Apiaries
      -> Hives
          -> Devices
          -> Sensors
          -> Telemetry
          -> Events
          -> Tags
      -> Apiary devices
      -> Weather station
      -> Tasks
      -> Events
      -> Reminders
      -> Notification settings
```

Один пользователь может состоять в нескольких организациях. Одна организация может иметь несколько пасек. Одна пасека может иметь несколько участников. Один улей может иметь несколько устройств разных типов.

## 3. Технологический стек

- Язык backend: Go.
- База данных: PostgreSQL.
- API: REST для web и mobile clients.
- MQTT broker для MVP: Mosquitto.
- MQTT broker должен быть заменяемым: Mosquitto, EMQX, HiveMQ, cloud provider, custom.
- Message bus: NATS + JetStream.
- Object storage: S3-compatible storage, для dev/self-hosted можно MinIO.
- Развертывание MVP: VPS + Docker Compose.
- Развертывание при росте нагрузки: Kubernetes.

Сервисы должны быть stateless, где это возможно. Конфигурация через environment variables. Нужны health/readiness endpoints, структурированные логи, миграции БД отдельным процессом и отсутствие зависимости от локального диска контейнера.

## 4. Сервисная архитектура

Базовые сервисы:

- `api-service` - REST API, авторизация, пользователи, организации, пасеки, ульи, задачи, события, настройки, выдача телеметрии.
- `mqtt-ingestion-service` - прием MQTT-телеметрии, валидация, запись в БД, обновление статусов, запуск правил.
- `command-service` - отправка MQTT-команд устройствам, обработка результатов, таймауты.
- `notification-service` - push, Telegram, in-app notifications, история доставок.
- `scheduler-service` - напоминания, повторяющиеся задачи, проверка пропущенной телеметрии, архивация.
- `weather-service` - внешние погодные API, данные метеостанций, погодный архив.
- `ota-service` - версии прошивок, rollout, статусы обновлений.
- `file-service` - загрузка и выдача вложений.

Будущие сервисы:

- `ai-service`;
- `billing-service`;
- `analytics-service`.

Примеры NATS subjects:

```text
telemetry.received
device.command.created
device.command.result
alert.created
notification.send
reminder.due
task.occurrence.created
weather.poll.requested
ota.rollout.started
ai.analysis.requested
```

## 5. Организации, тарифы и биллинг

Тариф применяется к организации, а не к отдельной пасеке или пользователю.

Для MVP достаточно тарифов и лимитов без интеграции платежного провайдера. Тариф назначается вручную администратором платформы, backend применяет лимиты.

Лимиты тарифа:

- максимальное количество пасек;
- максимальное количество ульев;
- максимальное количество устройств;
- максимальное количество пользователей;
- срок хранения raw-телеметрии;
- доступ к архиву;
- доступ к внешнему погодному API;
- доступ к AI-функциям;
- доступ к push/Telegram-уведомлениям;
- минимальный разрешенный интервал телеметрии.

Сущности:

```text
organizations
organization_members
tariff_plans
subscriptions
usage_counters
```

Платежи, счета, автопродление и платежные провайдеры - v2.

## 6. Роли и права доступа

Роли существуют на двух уровнях.

Роли организации:

```text
organization_owner
organization_admin
organization_member
organization_viewer
```

`organization_owner` имеет полный доступ ко всем пасекам организации, управляет биллингом, тарифом, участниками и может удалить организацию.

`organization_admin` имеет доступ ко всем пасекам организации и управляет пасеками, ульями и устройствами, но без удаления организации и управления биллингом, если это не разрешено отдельно.

Роли пасеки:

```text
apiary_owner
apiary_admin
apiary_worker
apiary_viewer
```

Роль на пасеке управляет доступом к конкретной пасеке: просмотр, ульи, устройства, задачи, события, напоминания, уведомления и схема ульев.

Нужна матрица permissions:

```text
organization.manage_billing
organization.manage_members
apiary.create
apiary.update
apiary.delete
hive.create
hive.update
device.assign
device.command.measure_now
device.command.update_config
device.command.restart
device.command.ota
task.manage
event.create
notification.manage
```

## 7. Приглашения пользователей

Участники добавляются через email-приглашение.

Сценарий:

1. Владелец или админ вводит email.
2. Выбирает роль.
3. Backend создает приглашение и отправляет email.
4. Пользователь регистрируется или входит в существующий аккаунт.
5. После принятия приглашения получает доступ.

Сущность:

```text
apiary_invitations
```

Поля:

```text
id
apiary_id
email
role
token
status      // pending | accepted | expired | cancelled
expires_at
invited_by
accepted_by
created_at
accepted_at
```

## 8. Авторизация

MVP:

- email + password;
- Google OAuth;
- Apple OAuth;
- подтверждение email;
- восстановление пароля;
- access/refresh tokens;
- управление сессиями для web и mobile.

V2:

- телефон;
- SMS-подтверждение;
- SMS как 2FA;
- TOTP;
- абстракция `sms_provider`.

Сущности:

```text
users
auth_identities
user_sessions
email_verifications
password_resets
two_factor_settings
```

Mobile app должен поддерживаться с первого этапа, поэтому backend хранит пользовательские устройства и push-токены.

## 9. Пасеки

Пасека принадлежит организации и имеет участников.

Поля пасеки:

```text
id
organization_id
name
description
country
region
locality
address
location_description
latitude
longitude
timezone
status
created_at
updated_at
```

Backend хранит координаты пасек для карты. Координаты отдельных ульев не нужны.

## 10. Схема расположения ульев

Для пасеки нужна схема расположения ульев без GPS-привязки. Это абстрактные координаты для UI.

Режимы:

```text
manual
rows
grid
perimeter
custom
```

Сущности:

```text
apiary_layout_settings
hive_layout_positions
```

Поля позиции улья:

```text
id
apiary_id
hive_id
x
y
rotation
row_number
column_number
display_order
updated_at
```

UI должен позволять перетаскивать ульи, размещать рядами, сеткой, по периметру и показывать состояние улья: норма, предупреждение, критично, требует внимания.

## 11. Ульи

Поля улья:

```text
id
apiary_id
name
number
type
frame_count
super_count
bee_breed
settled_at
queen_year
queen_breed
queen_status
status
notes
created_at
updated_at
```

`super_count` - количество магазинов/надстроек для медовых рамок.

## 12. Теги ульев

Система должна поддерживать пользовательские и системные теги.

Пользовательские теги:

- создаются пользователем;
- принадлежат пасеке;
- используются для фильтрации, группировки и назначения задач.

Системные теги:

- назначаются backend, rules engine или AI;
- показывают состояние улья или необходимость внимания;
- могут иметь severity;
- могут сниматься автоматически, когда проблема исчезла.

Примеры системных тегов:

```text
requires_attention
critical
missed_telemetry
low_battery
weight_anomaly
temperature_anomaly
humidity_anomaly
possible_swarming
possible_theft
needs_inspection
```

Сущности:

```text
hive_tags
hive_tag_assignments
```

Поля тега:

```text
id
apiary_id nullable
code
name
type        // system | custom
color
severity    // info | warning | critical
description
created_by nullable
created_at
```

Поля назначения:

```text
id
hive_id
tag_id
source_type     // manual | rule | ai | device | system
source_id nullable
assigned_by nullable
assigned_at
expires_at nullable
is_active
```

Задачи можно назначать по пользовательским и системным тегам.

## 13. Устройства и датчики

Устройство имеет уникальный `deviceId`, который генерируется при прошивке. Backend не генерирует `deviceId`, а принимает его от устройства.

Одному улью может соответствовать несколько устройств. Устройства могут быть привязаны к улью или к пасеке.

Типы устройств:

```text
hive_monitor
scale
temperature_sensor
humidity_sensor
microphone
weather_station
security_sensor
camera
gateway
other
```

Привязки устройств:

```text
device_assignments
```

Поля:

```text
id
device_id
scope_type      // apiary | hive
scope_id
role            // primary_monitor | secondary_sensor | weather_station | camera | security
assigned_at
unassigned_at
assigned_by
```

История телеметрии должна сохранять связь с тем ульем или пасекой, к которому устройство было привязано на момент получения данных.

Перенос устройства внутри одной пасеки не требует сброса настроек. Перенос между пасеками требует сброса настроек устройства и повторной первичной настройки.

## 14. Первичная настройка и непривязанные устройства

После factory reset устройство становится как новое:

- поднимает собственную Wi-Fi AP;
- пользователь подключается к устройству;
- через локальный web/captive portal вводит Wi-Fi и MQTT-настройки пасеки;
- устройство подключается к Wi-Fi и MQTT broker;
- отправляет первое сообщение.

Backend по первому сообщению:

1. Определяет пасеку по MQTT credentials/topic.
2. Проверяет `deviceId`.
3. Если устройство неизвестно, создает его со статусом `unassigned`.
4. Показывает в списке непривязанных устройств пасеки.
5. Админ пасеки привязывает устройство к улью или пасеке.

Статусы устройства:

```text
new
unassigned
assigned
disabled
removed
```

Если устройство до привязки уже отправило данные, при привязке нужно спросить пользователя, что сделать со старыми данными:

- привязать к выбранному улью;
- удалить;
- оставить как диагностические данные устройства.

Для MVP достаточно вариантов `attach_to_hive` и `delete`.

## 15. MQTT

Для MVP broker - Mosquitto. Конфигурация broker должна быть вынесена:

```text
MQTT_BROKER_URL
MQTT_USERNAME
MQTT_PASSWORD
MQTT_TLS_ENABLED
MQTT_CLIENT_ID
MQTT_PROVIDER
```

MQTT credentials выдаются на уровне пасеки: общий логин/пароль для всех устройств одной пасеки.

Сущность:

```text
apiary_mqtt_credentials
```

Поля:

```text
id
apiary_id
username
password_secret_ref
is_active
created_at
rotated_at
```

Пароль нельзя отдавать в API после создания. Нужна ротация credentials.

Текущая firmware-документация использует topics:

```text
hives/{deviceId}/telemetry
hives/{deviceId}/events
hives/{deviceId}/status
hives/{deviceId}/commands
hives/{deviceId}/config
```

Backend MVP должен поддержать этот формат для совместимости.

Целевой формат для multi-apiary авторизации:

```text
apiaries/{apiary_id}/devices/{device_id}/telemetry
apiaries/{apiary_id}/devices/{device_id}/events
apiaries/{apiary_id}/devices/{device_id}/status
apiaries/{apiary_id}/devices/{device_id}/command-results
apiaries/{apiary_id}/devices/{device_id}/commands
```

Переход между форматами должен быть описан отдельно перед релизом firmware/backend.

## 16. Телеметрия и readings

Устройство может иметь разный набор датчиков. Базовая комплектация - датчик веса. Дополнительно могут быть температура, влажность, батарея, звук и другие датчики.

Backend должен поддерживать динамический набор сенсоров.

Сущности:

```text
device_sensors
sensor_readings
raw_payloads
```

Поля `device_sensors`:

```text
id
device_id
sensor_type     // weight | temperature | humidity | battery | sound | custom
sensor_name
unit
is_enabled
calibration_data
created_at
updated_at
```

Поля `sensor_readings`:

```text
id
device_id
sensor_id nullable
scope_type      // hive | apiary | device
scope_id nullable
metric_type     // weight | temperature | humidity | battery_percent | rssi | custom
value
unit
measured_at
received_at
raw_payload_id
```

Рекомендуемый гибкий MQTT payload:

```json
{
  "schemaVersion": 1,
  "deviceId": "abc-123",
  "timestamp": "2026-05-03T12:00:00Z",
  "firmwareVersion": "1.0.0",
  "configVersion": 12,
  "measurementIntervalSeconds": 1800,
  "readings": [
    {"type": "weight", "value": 42.8, "unit": "kg"},
    {"type": "temperature", "value": 34.2, "unit": "celsius"},
    {"type": "humidity", "value": 61.5, "unit": "percent"}
  ],
  "battery": {"percent": 78, "voltage": 3.91},
  "rssi": -67,
  "freeHeap": 42128,
  "errors": []
}
```

Для совместимости с текущей firmware backend должен также принять плоские поля из документации firmware: `weight`, `weightChange`, `temperature`, `humidity`, `hiveOpened`, `errorFlag`, `batteryPercent`, `batteryVoltage`, `rssi`, `freeHeap`.

Период измерений по умолчанию - 30 минут. Backend хранит желаемый интервал устройства и проверяет лимит тарифа.

## 17. Пропущенная телеметрия

Так как устройство после передачи уходит в sleep, обычный `offline` не является аварией.

Backend должен отслеживать пропущенные плановые передачи.

Поля:

```text
expected_next_telemetry_at
missed_telemetry_count
last_telemetry_at
telemetry_interval_minutes
```

Если устройство пропустило заданное количество таймеров, по умолчанию 5, создается alert, системный тег и уведомление.

## 18. Правила предупреждений

Alerts должны быть универсальными, по `metric_type`, а не жесткими колонками.

Сущность:

```text
alert_rules
```

Поля:

```text
id
scope_type      // system | organization | apiary | hive | device
scope_id nullable
metric_type     // weight | temperature | humidity | battery_percent | no_recent_telemetry | custom
operator        // lt | lte | gt | gte | eq | neq | delta_gt | delta_lt | missing_count
threshold_value
threshold_unit
period_minutes nullable
severity        // info | warning | critical
is_enabled
notification_enabled
created_at
updated_at
```

Наследование:

```text
system default rules
  -> organization rules
      -> apiary rules
          -> hive/device rules
```

В MVP нужны простые пороги с наследованием. Профили порогов с временными диапазонами и активацией после задач/событий - v2.

Критические уведомления MVP:

- устройство пропустило заданное количество плановых передач;
- низкий заряд батареи;
- резкое падение веса;
- температура вне нормы;
- влажность вне нормы.

## 19. Alerts, события и системные теги

Каждый alert должен:

1. Создать запись в `alerts`.
2. Назначить или обновить системный тег улья, если alert относится к улью.
3. Создать событие в журнале улья или пасеки.
4. Отправить уведомление, если правило включает уведомления и настройки получателей это разрешают.

Событие от alert должно иметь:

```text
source_type = alert
source_id = alert_id
```

Пример:

```text
Alert: резкое падение веса
System tag: weight_anomaly
Event: "Обнаружено резкое падение веса"
Notification: push/Telegram ответственным пользователям
```

Если alert восстановлен, система может создать событие восстановления и снять системный тег.

## 20. Журнал событий

Нужны два уровня журнала:

- журнал пасеки;
- журнал улья.

Можно использовать одну таблицу:

```text
events
```

Поля:

```text
id
apiary_id
hive_id nullable
scope_type      // apiary | hive
type
title
description
result
comment
event_at
created_by
source_type nullable   // manual | task | alert | ai | device
source_id nullable
created_at
updated_at
```

События пасеки: общий осмотр, обработка всей пасеки, переезд, оборудование, территория, сезонные заметки.

События улья: осмотр, подкормка, лечение, замена матки, роение, сбор меда, установка/снятие устройства, заметка.

## 21. Вложения к событиям

Вложения хранятся отдельной сущностью:

```text
event_attachments
```

Поля:

```text
id
event_id
file_type     // image | audio | document | other
file_url
file_name
mime_type
size_bytes
uploaded_by
created_at
```

MVP поддерживает фото и файлы. Аудио-заметки можно поддержать как обычные вложения, но AI-анализ звука улья остается отдельной будущей функцией.

## 22. Задачи и планировщик

На каждой пасеке должен быть планировщик работ.

Задача может относиться:

- ко всей пасеке;
- к одному улью;
- к нескольким ульям;
- к группе ульев по тегу.

Система поддерживает разовые и повторяющиеся задачи.

Сущности:

```text
tasks
task_targets
task_recurrence_rules
task_occurrences
```

Статусы:

```text
planned
in_progress
done
cancelled
overdue
skipped
```

Типы работ:

```text
inspection
feeding
treatment
queen_check
honey_harvest
frame_replacement
winter_preparation
entrance_position_change
other
```

При закрытии задачи система должна спросить пользователя, нужно ли сохранить результат в журнал. Пользователь заполняет результат, комментарий, дату выполнения, вложения. Если выбрано сохранение, backend создает событие пасеки или улья.

Поля результата:

```text
completion_result
completion_comment
completed_by
completed_at
create_event_on_completion
```

## 23. Напоминания

Напоминания могут быть созданы:

- от задачи;
- от события;
- вручную;
- будущим AI-модулем.

Сущность:

```text
reminders
```

Поля:

```text
id
user_id nullable
apiary_id
hive_id nullable
task_id nullable
event_id nullable
message
remind_at
status       // scheduled | sent | cancelled | failed
channel      // push | telegram | in_app
created_at
sent_at
```

## 24. Уведомления

MVP-каналы:

- push;
- Telegram;
- in-app журнал уведомлений.

Email используется для приглашений и восстановления пароля, но не основной канал напоминаний.

Настройки уведомлений задаются на уровне пасеки и улья. Настройки улья переопределяют настройки пасеки.

Сущность:

```text
notification_settings
```

Поля:

```text
id
user_id
scope_type      // apiary | hive
scope_id
push_enabled
telegram_enabled
task_reminders_enabled
event_reminders_enabled
telemetry_alerts_enabled
remind_before_minutes
quiet_hours_from
quiet_hours_to
```

Получатели уведомлений:

- конкретный пользователь;
- ответственный за задачу;
- все участники пасеки;
- роли пасеки, например `apiary_owner` и `apiary_admin`;
- выбранный список пользователей.

Сущности:

```text
notifications
notification_deliveries
notification_recipients
user_push_tokens
telegram_links
```

## 25. Погода и метеостанции

Пасека может получать погодные данные двумя способами:

1. Собственная метеостанция как IoT-устройство.
2. Внешний погодный API по координатам пасеки.

Если есть своя метеостанция, она привязывается к пасеке. Если ее нет, backend подтягивает погоду из внешнего API. В режиме `auto` backend использует метеостанцию, если данные свежие, иначе fallback на внешний API.

Нужна абстракция `weather_provider`, провайдер по умолчанию и возможность выбрать другого провайдера из списка.

Примеры провайдеров:

```text
openweather
meteostat
weatherapi
custom
```

Сущности:

```text
weather_providers
apiary_weather_settings
weather_stations
weather_telemetry
weather_archive
```

Погодные показатели:

- внешняя температура;
- влажность воздуха;
- скорость ветра;
- направление ветра, опционально;
- атмосферное давление;
- дождь/осадки;
- интенсивность осадков, если доступно.

Поля `weather_telemetry`:

```text
id
apiary_id
source_type      // station | external_api
source_id
temperature
humidity
wind_speed
wind_direction
pressure
rainfall
rain_intensity
measured_at
received_at
```

## 26. Архив данных

Подробная raw-телеметрия хранится 1 год по умолчанию.

После 1 года данные переносятся в архив, если тариф разрешает архив. Архив хранит агрегированные данные, а не сырые readings.

Архив улья:

- вес: min, max, avg за день;
- температура улья: min, max, avg за день;
- влажность улья: min, max, avg за день;
- события за день;
- alerts/предупреждения за день.

Не хранить в архиве улья:

- батарею;
- RSSI;
- количество измерений;
- сырые диагностические данные.

Погодный архив:

- внешняя температура: min, max, avg;
- влажность воздуха: min, max, avg;
- скорость ветра: min, max, avg;
- давление: min, max, avg;
- осадки: total;
- максимальная интенсивность осадков.

Архивация выполняется периодической задачей, результат логируется.

## 27. Команды устройствам

Backend должен отправлять команды устройствам через MQTT.

Минимальные команды:

```text
update_config
measure_now
restart
ping
```

Дополнительные команды:

```text
firmware_update
calibrate_scale
tare_scale
clear_buffer
set_sleep_mode
request_diagnostics
```

Текущая firmware поддерживает команды `measure`, `restart`, `tare`, `clearBuffer`, `configUpdate` и публикацию ответа в `hives/{deviceId}/status`. Backend должен учитывать это для MVP.

Сущность:

```text
device_commands
```

Поля:

```text
id
device_id
type
payload
status       // pending | sent | success | failed | timeout | cancelled
created_by
created_at
sent_at
acknowledged_at
result_payload
error_message
```

Права на команды зависят от роли:

- owner: все команды для устройств своей пасеки;
- admin: update_config, measure_now, ping, request_diagnostics, tare_scale, calibrate_scale;
- worker: measure_now, ping;
- viewer: нет прав;
- platform_admin: firmware_update, сервисные команды, диагностика.

Опасные команды требуют подтверждения.

## 28. OTA

OTA-обновление устройств нужно в первой версии.

Система должна поддерживать:

- загрузку firmware-файла;
- хранение версий;
- ручное обновление одного устройства;
- ручное обновление выбранных устройств;
- ручное обновление всей пасеки до последней версии;
- автоматическое обновление до последней разрешенной стабильной версии;
- включение/отключение автообновления на уровне пасеки;
- включение/отключение автообновления на уровне устройства;
- каналы `stable`, `beta`, `dev`;
- окно обновлений;
- историю обновлений.

Сущности:

```text
firmware_versions
firmware_rollouts
device_firmware_updates
```

Поля `firmware_versions`:

```text
id
version
platform       // esp8266 | esp32
channel        // stable | beta | dev
file_url
file_checksum
file_size
release_notes
is_active
is_latest
min_supported_version
force_update
uploaded_by
created_at
```

Статусы обновления:

```text
pending
sent
downloading
installing
success
failed
cancelled
```

Права:

- `platform_admin` загружает firmware и запускает массовые обновления;
- `owner/admin` пасеки может запускать обновление своих устройств, если версия разрешена;
- `worker/viewer` не имеют доступа к OTA.

## 29. AI / Premium функции

AI-функции являются premium add-ons и могут быть реализованы после MVP, но архитектура должна их учитывать.

Функции:

- анализ аномалий веса;
- анализ температуры и влажности;
- прогноз медосбора;
- рекомендации по работам;
- предупреждения о проблемах;
- анализ звука улья;
- распознавание проблем по описанию;
- распознавание проблем по фото;
- чат-помощник пасечника;
- автоматическое создание задач и напоминаний.

Сущности:

```text
ai_insights
ai_recommendations
ai_alerts
audio_samples
photo_attachments
generated_tasks
feature_flags
enabled_addons
```

Для MVP можно начать с rule-based анализа, а AI подключить позже.

## 30. REST API MVP

Минимальные группы endpoints:

```text
/auth/*
/organizations/*
/organizations/{id}/members
/apiaries
/apiaries/{id}
/apiaries/{id}/members
/apiaries/{id}/invitations
/apiaries/{id}/hives
/apiaries/{id}/tasks
/apiaries/{id}/events
/apiaries/{id}/reminders
/apiaries/{id}/devices
/apiaries/{id}/weather
/hives/{id}
/hives/{id}/telemetry/latest
/hives/{id}/telemetry/history
/hives/{id}/events
/hives/{id}/tags
/devices/{id}
/devices/{id}/commands
/devices/{id}/assignments
/events/{id}/attachments
/notifications
/notification-settings
/firmware-versions
/firmware-rollouts
```

Все endpoints должны проверять права доступа и не возвращать данные чужих организаций/пасек.

## 31. Нефункциональные требования

- Все timestamps хранить в UTC.
- Timezone пасеки использовать для календаря, задач, напоминаний, локального отображения и погодных архивов.
- API должен возвращать понятные ошибки.
- При ошибке одного MQTT-сообщения обработка других не должна останавливаться.
- MQTT consumer должен автоматически переподключаться к broker.
- Background jobs должны быть идемпотентными там, где возможно.
- Все команды устройствам, alerts, уведомления и OTA-действия логируются.
- Файлы хранятся в object storage, не на локальном диске сервиса.
- Для больших таблиц telemetry нужны индексы и/или партиционирование по времени и scope.

Рекомендуемые индексы:

```text
sensor_readings(device_id, measured_at)
sensor_readings(scope_type, scope_id, measured_at)
events(apiary_id, event_at)
events(hive_id, event_at)
weather_telemetry(apiary_id, measured_at)
device_commands(device_id, created_at)
```

## 32. MVP

В MVP должно быть:

- organizations;
- пользователи и приглашения;
- email/password, Google OAuth, Apple OAuth;
- роли организации и пасеки;
- пасеки с локацией и координатами;
- карта пасек;
- ульи;
- схема расположения ульев;
- пользовательские и системные теги;
- устройства и непривязанные устройства;
- привязка устройств к ульям/пасекам;
- MQTT ingestion;
- совместимость с текущими firmware topics;
- динамические sensor readings;
- последняя и историческая телеметрия;
- простые alert rules с наследованием;
- alerts создают события, системные теги и уведомления;
- задачи разовые и повторяющиеся;
- события пасеки и улья;
- вложения к событиям;
- напоминания;
- push, Telegram, in-app notifications;
- weather provider abstraction и один provider по умолчанию;
- метеостанция как устройство пасеки;
- команды устройствам через MQTT;
- OTA;
- тарифы и лимиты без платежного провайдера;
- хранение raw-телеметрии 1 год и задел под архив.

## 33. V2 и позже

- Реальные платежи и платежные провайдеры.
- SMS, телефон, 2FA.
- Профили порогов с временными диапазонами.
- Активация профиля порогов после задачи или события.
- AI analytics.
- Анализ звука улья.
- Распознавание проблем по фото.
- Чат-помощник.
- Автоматическое создание задач и напоминаний AI-модулем.
- Поиск новых устройств в mobile app и передача Wi-Fi/MQTT настроек устройству.
- Расширенные отчеты и экспорт.
