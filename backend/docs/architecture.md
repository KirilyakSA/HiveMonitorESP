# Архитектура backend HiveMonitor

## 1. Назначение backend

Backend HiveMonitor обслуживает web и mobile клиенты, принимает MQTT-телеметрию от IoT-устройств, хранит данные в PostgreSQL, управляет организациями, пасеками, ульями, устройствами и связями между ними.

Главная продуктовая модель:

```text
Organization
  -> Users / members
  -> Apiaries
      -> Hives
          -> Devices
          -> Sensor readings
      -> Unassigned devices
      -> Future: tasks, events, reminders, weather, OTA, AI
```

Для пользователя главным объектом является пасека и улей. Устройство является заменяемым источником данных. История измерений должна оставаться привязанной к тому улью, к которому устройство было привязано на момент приема данных.

## 2. Текущий MVP-срез

В первом backend-инкременте реализована вертикаль:

```text
User registers
  -> creates organization
  -> creates apiary
  -> creates hive
  -> device publishes MQTT telemetry
  -> backend creates unassigned device
  -> user assigns device to hive
  -> old readings can be attached to hive
  -> API returns latest/history telemetry
```

Реализовано:

- email/password регистрация и вход;
- JWT access token;
- организации;
- пасеки;
- ульи;
- список непривязанных устройств;
- привязка устройства к улью;
- выбор действия со старыми данными устройства при привязке;
- MQTT ingestion;
- хранение raw MQTT payload;
- хранение динамических sensor readings;
- latest/history telemetry endpoints;
- Docker Compose инфраструктура: PostgreSQL, NATS, Mosquitto, API, MQTT ingestion.

Пока не реализовано, но заложено в ТЗ:

- события и журнал работ;
- задачи и повторяющиеся задачи;
- уведомления;
- alert rules;
- weather service;
- OTA service;
- billing;
- AI.

## 3. Стек

- Go;
- chi для HTTP routing;
- pgx/pgxpool для PostgreSQL;
- goose для миграций;
- PostgreSQL как основная БД;
- NATS + JetStream как message bus;
- Mosquitto как MQTT broker MVP;
- Docker Compose для локального/VPS запуска.

## 4. Сервисная архитектура

Текущий backend уже разделен на два запускаемых процесса.

```text
api-service
  -> HTTP REST API
  -> PostgreSQL
  -> NATS, optional

mqtt-ingestion-service
  -> MQTT broker
  -> PostgreSQL
  -> NATS, optional

PostgreSQL
  -> system of record

NATS
  -> event bus for future async workflows

Mosquitto
  -> MQTT broker for devices
```

### 4.1 api-service

Путь:

```text
backend/cmd/api-service/main.go
```

Ответственность:

- загрузить конфигурацию;
- подключиться к PostgreSQL;
- подключиться к NATS, если доступен;
- создать repository layer;
- создать HTTP server;
- обслуживать REST API.

Текущие группы endpoints:

```text
GET  /healthz
POST /auth/register
POST /auth/login
GET  /me

GET  /organizations/
POST /organizations/

GET  /apiaries/
POST /apiaries/
GET  /apiaries/{apiaryID}/hives
POST /apiaries/{apiaryID}/hives
GET  /apiaries/{apiaryID}/devices/unassigned
POST /apiaries/{apiaryID}/devices/{deviceUUID}/assign

GET  /hives/{hiveID}/telemetry/latest
GET  /hives/{hiveID}/telemetry/history
```

### 4.2 mqtt-ingestion-service

Путь:

```text
backend/cmd/mqtt-ingestion-service/main.go
```

Ответственность:

- загрузить конфигурацию;
- подключиться к PostgreSQL;
- подключиться к NATS, если доступен;
- подключиться к MQTT broker;
- подписаться на telemetry topics;
- распарсить MQTT payload;
- создать или обновить устройство;
- сохранить raw payload;
- сохранить sensor readings;
- опубликовать событие `telemetry.received` в NATS.

Текущие MQTT topics:

```text
hives/+/telemetry
apiaries/+/devices/+/telemetry
```

Первый topic нужен для совместимости с текущей firmware.

Второй topic предпочтителен для backend MVP, потому что содержит `apiary_id` и позволяет автоматически положить новое устройство в список непривязанных устройств конкретной пасеки.

### 4.3 Совместимость с текущей firmware

Текущая firmware публикует:

```text
hives/{deviceId}/telemetry
hives/{deviceId}/events
hives/{deviceId}/status
```

И подписывается на:

```text
hives/{deviceId}/commands
hives/{deviceId}/config
```

Backend MVP сейчас обрабатывает только telemetry. `events`, `status`, `commands` и `config` являются следующими backend-инкрементами.

Для legacy telemetry topic `hives/{deviceId}/telemetry` в topic нет `apiary_id`. Поэтому есть два рабочих сценария:

- задать `DEFAULT_APIARY_ID` для dev/MVP;
- принимать устройство без пасеки как диагностическое до будущего firmware-инкремента.

Целевой сценарий - firmware публикует `apiaries/{apiary_id}/devices/{device_id}/telemetry`. Для этого в firmware нужно добавить `apiaryId` или настраиваемый topic prefix.

## 5. Структура файлов backend

```text
backend/
  .dockerignore
  .env.example
  Dockerfile
  README.md
  go.mod
  go.sum
  tools.go

  cmd/
    api-service/
      main.go
    mqtt-ingestion-service/
      main.go

  internal/
    api/
      server.go
    auth/
      auth.go
    config/
      config.go
    database/
      database.go
    domain/
      models.go
    events/
      bus.go
    ingestion/
      service.go
    repository/
      repository.go

  migrations/
    00001_init.sql

  docs/
    README.md
    architecture.md
```

## 6. Слои приложения

### 6.1 cmd layer

`cmd/*/main.go` содержит только composition root:

- конфигурация;
- logger;
- signal handling;
- соединения с внешними сервисами;
- сборка зависимостей;
- запуск процесса.

Бизнес-логика не должна жить в `cmd`.

### 6.2 api layer

Путь:

```text
internal/api/server.go
```

Основные структуры и функции:

```text
Server
NewServer
Routes
authMiddleware
register
login
me
createOrganization
listOrganizations
createApiary
listApiaries
createHive
listHives
listUnassignedDevices
assignDevice
latestTelemetry
telemetryHistory
```

Ответственность:

- HTTP routing;
- JSON decode/encode;
- проверка bearer token;
- базовая валидация request body;
- вызовы repository;
- преобразование ошибок в HTTP status codes.

API слой не должен напрямую писать SQL и не должен знать детали MQTT.

### 6.3 auth layer

Путь:

```text
internal/auth/auth.go
```

Основные структуры и функции:

```text
Claims
HashPassword
CheckPassword
CreateToken
ParseToken
```

Ответственность:

- bcrypt hash/check password;
- выпуск JWT;
- парсинг JWT;
- структура claims.

Текущий MVP использует только access token. Refresh tokens, session management, OAuth Google/Apple и 2FA относятся к следующим инкрементам.

### 6.4 config layer

Путь:

```text
internal/config/config.go
```

Основная структура:

```text
Config
```

Поля:

```text
HTTPAddr
DatabaseURL
JWTSecret
AccessTokenTTL
NATSURL
MQTTBrokerURL
MQTTUsername
MQTTPassword
MQTTClientID
MQTTTelemetryTopic
DefaultAPIaryID
DefaultIntervalMinute
```

Конфигурация читается из environment variables. Для локальной разработки поддержан `.env` через `godotenv`.

### 6.5 database layer

Путь:

```text
internal/database/database.go
```

Функция:

```text
Open
```

Ответственность:

- создать `pgxpool.Pool`;
- настроить базовые параметры pool;
- проверить соединение через `Ping`;
- вернуть готовый pool.

### 6.6 domain layer

Путь:

```text
internal/domain/models.go
```

Основные модели:

```text
User
Organization
Apiary
Hive
Device
DeviceAssignment
SensorReading
```

Это простые структуры данных для обмена между API/repository и JSON response. В текущем MVP они не содержат методов бизнес-логики.

### 6.7 events layer

Путь:

```text
internal/events/bus.go
```

Основная структура:

```text
Bus
```

Функции:

```text
Connect
Close
PublishJSON
```

Ответственность:

- подключение к NATS;
- публикация JSON events.

В MVP NATS является optional dependency: если NATS недоступен, API может стартовать с warning. Для production это поведение можно ужесточить.

### 6.8 ingestion layer

Путь:

```text
internal/ingestion/service.go
```

Основные структуры:

```text
Service
telemetryPayload
readingPayload
```

Основные функции:

```text
NewService
Run
handleMessage
parseTelemetry
deviceIDFromTopic
apiaryIDFromTopic
parseTimestamp
normalizeMetric
firstNonEmpty
```

Ответственность:

- MQTT connection lifecycle;
- подписка на telemetry topics;
- парсинг payload текущей firmware и нового гибкого формата;
- нормализация readings;
- передача готового `IngestTelemetryInput` в repository;
- публикация `telemetry.received` в NATS.

Поддерживаемые форматы telemetry payload:

Плоский формат текущей firmware:

```json
{
  "schemaVersion": 1,
  "deviceId": "device-1",
  "timestamp": "2026-05-03T10:56:00Z",
  "firmwareVersion": "1.0.0",
  "measurementIntervalSeconds": 1800,
  "weight": 42.5,
  "temperature": 34.2,
  "humidity": 61.5,
  "batteryPercent": 87,
  "rssi": -55
}
```

Гибкий формат будущих устройств:

```json
{
  "schemaVersion": 1,
  "deviceId": "device-1",
  "timestamp": "2026-05-03T10:56:00Z",
  "readings": [
    {"type": "weight", "value": 42.5, "unit": "kg"},
    {"type": "temperature", "value": 34.2, "unit": "celsius"}
  ]
}
```

### 6.9 repository layer

Путь:

```text
internal/repository/repository.go
```

Основная структура:

```text
Repository
```

Основные функции:

```text
CreateUser
GetUserByEmail
GetUserByID
CreateOrganization
ListOrganizations
UserCanAccessOrganization
CreateApiary
ListApiaries
UserCanAccessApiary
CreateHive
ListHives
ListUnassignedDevices
AssignDeviceToHive
IngestTelemetry
LatestTelemetryForHive
TelemetryHistoryForHive
```

Ответственность:

- SQL queries;
- транзакции;
- проверка доступа к организации/пасеке;
- создание/обновление устройств при telemetry ingestion;
- сохранение raw payload и readings;
- выборка latest/history telemetry.

Repository слой сейчас совмещает persistence и часть application logic. Для роста проекта его можно разделить на:

```text
internal/repository
internal/service
internal/permissions
```

## 7. Архитектура данных

Миграции находятся в:

```text
backend/migrations/00001_init.sql
```

Используется PostgreSQL extension:

```sql
pgcrypto
```

Она нужна для `gen_random_uuid()`.

### 7.1 users

Пользователи системы.

Основные поля:

```text
id
email
name
password_hash
created_at
updated_at
```

Ограничения:

- `email` уникальный;
- password хранится только как bcrypt hash.

### 7.2 organizations

Организация или аккаунт владельца. Тарифы и биллинг в будущем будут привязаны к организации.

Поля:

```text
id
name
created_at
updated_at
```

### 7.3 organization_members

Связь пользователей и организаций.

Поля:

```text
id
organization_id
user_id
role
status
invited_by
created_at
```

Текущая роль при создании организации:

```text
organization_owner
```

Ограничения:

- уникальная пара `organization_id + user_id`;
- cascade delete при удалении организации или пользователя.

### 7.4 apiaries

Пасеки внутри организации.

Поля:

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

Координаты пасеки нужны для карты и будущего weather provider.

### 7.5 hives

Ульи внутри пасеки.

Поля:

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

`super_count` - количество магазинов/надстроек.

### 7.6 devices

Физические устройства.

Поля:

```text
id
apiary_id
device_id
device_type
status
firmware_version
config_version
last_telemetry_at
expected_next_telemetry_at
missed_telemetry_count
telemetry_interval_minutes
created_at
updated_at
```

Разница между `id` и `device_id`:

- `id` - внутренний UUID backend;
- `device_id` - уникальный идентификатор, который приходит от firmware.

Статусы MVP:

```text
unassigned
assigned
disabled
removed
```

В MVP устройство появляется автоматически при первом MQTT-сообщении.

### 7.7 device_assignments

История привязок устройства к улью или пасеке.

Поля:

```text
id
device_id
apiary_id
hive_id
scope_type
scope_id
role
assigned_at
unassigned_at
```

Активная привязка:

```text
unassigned_at is null
```

При новой привязке старые активные привязки устройства закрываются.

### 7.8 raw_payloads

Raw MQTT payload для диагностики.

Поля:

```text
id
device_id
apiary_id
topic
payload
received_at
```

`payload` хранится как `jsonb`.

### 7.9 sensor_readings

Нормализованные readings из MQTT payload.

Поля:

```text
id
device_id
apiary_id
hive_id
metric_type
value
unit
measured_at
received_at
raw_payload_id
```

`metric_type` является динамическим. Это позволяет поддерживать разные наборы датчиков без миграции схемы под каждый новый sensor.

Примеры `metric_type`:

```text
weight
weight_change
temperature
humidity
battery_percent
battery_voltage
rssi
free_heap
hive_opened
```

## 8. Связи данных

```text
users
  1..n organization_members

organizations
  1..n organization_members
  1..n apiaries

apiaries
  1..n hives
  1..n devices
  1..n device_assignments
  1..n sensor_readings

hives
  1..n device_assignments
  1..n sensor_readings

devices
  1..n device_assignments
  1..n raw_payloads
  1..n sensor_readings

raw_payloads
  1..n sensor_readings
```

## 9. Поток данных MQTT ingestion

```text
Device publishes MQTT
  -> Mosquitto receives message
  -> mqtt-ingestion-service receives message
  -> parseTelemetry extracts:
       device_id
       apiary_id from topic, if present
       measured_at
       firmware/config versions
       readings
  -> Repository.IngestTelemetry
       upsert devices
       find active assignment
       insert raw_payloads
       insert sensor_readings
  -> Publish NATS event telemetry.received
```

Если устройство еще не привязано:

- `devices.status = unassigned`;
- readings сохраняются с `hive_id = null`;
- устройство доступно через `GET /apiaries/{apiaryID}/devices/unassigned`.

Если устройство привязано:

- readings сохраняются с текущим `hive_id`;
- latest/history endpoints начинают отдавать данные по улью.

## 10. Привязка устройства к улью

Endpoint:

```text
POST /apiaries/{apiaryID}/devices/{deviceUUID}/assign
```

Body:

```json
{
  "hive_id": "uuid",
  "import_mode": "attach_to_hive"
}
```

`import_mode`:

```text
attach_to_hive
delete
keep
```

Поведение:

- `attach_to_hive` - старые readings устройства без `hive_id` привязываются к выбранному улью;
- `delete` - старые readings устройства без `hive_id` удаляются;
- `keep` или пустое значение - старые readings остаются диагностическими без привязки к улью.

После привязки:

- создается запись `device_assignments`;
- старые активные assignment записи устройства закрываются;
- `devices.status` становится `assigned`.

## 11. latest/history telemetry

Latest endpoint:

```text
GET /hives/{hiveID}/telemetry/latest
```

Возвращает последнее значение по каждому `metric_type`.

History endpoint:

```text
GET /hives/{hiveID}/telemetry/history?from=2026-05-03T00:00:00Z&to=2026-05-04T00:00:00Z&metric=weight&limit=1000
```

Параметры:

```text
from   RFC3339, optional, default now - 24h
to     RFC3339, optional, default now
metric optional
limit  optional, default 1000, max 5000
```

Оба endpoint проверяют доступ пользователя к пасеке улья.

## 12. Авторизация и доступы

В MVP доступ проверяется через membership в organization.

Проверки:

- пользователь видит только организации, где он есть в `organization_members`;
- пользователь видит пасеки только в доступных организациях;
- доступ к ульям проверяется через пасеку;
- доступ к telemetry проверяется через улей -> пасеку -> organization membership.

Текущая модель ролей уже хранит `role`, но в MVP нет детальной permission matrix. Следующий шаг - вынести проверку прав в отдельный слой и различать:

```text
organization_owner
organization_admin
organization_member
organization_viewer
apiary_owner
apiary_admin
apiary_worker
apiary_viewer
```

## 13. Индексы

Текущие индексы:

```text
organization_members_user_idx
apiaries_organization_idx
hives_apiary_idx
devices_apiary_idx
device_assignments_device_active_idx
device_assignments_hive_idx
sensor_readings_device_measured_idx
sensor_readings_hive_measured_idx
sensor_readings_scope_metric_idx
```

Индексы telemetry ориентированы на:

- поиск readings по устройству и времени;
- поиск readings по улью и времени;
- latest readings по `hive_id + metric_type`.

При росте объема данных нужно добавить партиционирование `sensor_readings` по времени.

## 14. Docker Compose

Файл:

```text
deploy/docker-compose.yml
```

Сервисы:

```text
postgres
nats
mosquitto
api-service
mqtt-ingestion-service
```

PostgreSQL:

- database: `hivemonitor`;
- user: `hivemonitor`;
- local port: `5432`.

NATS:

- JetStream включен через `-js`;
- client port: `4222`;
- monitoring port: `8222`.

Mosquitto:

- local port: `1883`;
- config: `deploy/mosquitto/mosquitto.conf`;
- MVP config разрешает anonymous access только для локальной разработки.

API:

- local port: `8080`;
- health: `GET /healthz`.

## 15. Проверенный сценарий

Текущий MVP был проверен таким сценарием:

```text
go test ./...
docker compose config
goose up
docker compose up --build api-service mqtt-ingestion-service
GET /healthz
POST /auth/register
POST /organizations/
POST /apiaries/
POST /apiaries/{id}/hives
MQTT publish telemetry
GET /apiaries/{id}/devices/unassigned
POST /apiaries/{id}/devices/{deviceUUID}/assign
GET /hives/{id}/telemetry/latest
GET /hives/{id}/telemetry/history
```

Фактический результат:

- MQTT telemetry была принята;
- создано `unassigned` устройство;
- устройство привязано к улью;
- latest/history вернули 5 readings.

## 16. Ближайшие архитектурные шаги

Рекомендуемый порядок следующих инкрементов:

1. Добавить `apiaryId` или topic prefix в firmware.
2. Добавить backend ingestion для `hives/{deviceId}/events` и `hives/{deviceId}/status`.
3. Добавить API для device commands и MQTT publish в firmware topics.
4. Добавить refresh tokens и user sessions.
5. Вынести permission checks в отдельный слой.
6. Добавить базовые `events`, `hive_tags`, `alerts`.
7. Сделать alert rule: пропущенные передачи, низкая батарея, резкое изменение веса.
8. Добавить notification records без реальной доставки.
9. Добавить tasks/reminders.
10. Добавить weather provider abstraction.
11. Добавить OTA модели и endpoints.
12. Добавить партиционирование telemetry перед production-нагрузкой.
