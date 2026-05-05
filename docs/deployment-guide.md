# Руководство по развертыванию HiveMonitor

Документ описывает установку и проверку всей системы HiveMonitor: backend, PostgreSQL, NATS, Mosquitto, web-интерфейс и подключение IoT-устройств.

## Состав системы

Текущий MVP разворачивается через Docker Compose и состоит из:

| Компонент | Назначение | Где лежит |
| --- | --- | --- |
| `api-service` | REST API для web/mobile, авторизация, организации, пасеки, ульи, устройства, телеметрия | `backend/cmd/api-service` |
| `mqtt-ingestion-service` | Прием MQTT telemetry/events/status от устройств и запись в PostgreSQL | `backend/cmd/mqtt-ingestion-service` |
| PostgreSQL | Основная база данных | Docker volume `deploy_postgres-data` |
| NATS + JetStream | Внутренняя шина событий для дальнейших сервисов уведомлений, AI, OTA, reminders | Docker volume `deploy_nats-data` |
| Mosquitto | MQTT broker для IoT-устройств | `deploy/mosquitto/mosquitto.conf` |
| Web | React + TypeScript + Vite интерфейс | `web/` |
| Firmware | ESP8266/ESP32 прошивка устройства | `firmware/` |

## Требования

Для локальной разработки:

- Linux, macOS или Windows с WSL2.
- Git.
- Docker Engine.
- Docker Compose plugin.
- Node.js 22+ или новее для web-разработки.
- npm 10+ или новее.
- Go 1.23+ для локального запуска backend без Docker.
- PlatformIO для сборки firmware.

Для VPS:

- Ubuntu/Debian VPS или другой Linux-сервер.
- Открытые порты по необходимости:
  - `80/443` для production web/API через reverse proxy;
  - `1883` для MQTT без TLS или `8883` для MQTT over TLS;
  - `22` для SSH.
- Docker Engine и Docker Compose plugin.
- Доменное имя желательно для production TLS.

Порты dev-стека:

| Порт | Сервис |
| --- | --- |
| `5173` | Vite dev server web |
| `8080` | Backend API |
| `5432` | PostgreSQL |
| `1883` | Mosquitto MQTT |
| `4222` | NATS client |
| `8222` | NATS monitoring |

## Быстрый локальный запуск

Из корня репозитория:

```bash
docker compose -f deploy/docker-compose.yml up -d --build
```

Проверить контейнеры:

```bash
docker compose -f deploy/docker-compose.yml ps
```

Проверить API:

```bash
curl http://localhost:8080/healthz
```

Ожидаемый ответ:

```json
{"status":"ok"}
```

Запустить web:

```bash
cd web
npm install
npm run dev
```

Открыть:

```text
http://localhost:5173
```

Проверить proxy web -> backend:

```bash
curl http://localhost:5173/healthz
```

Ожидаемый ответ:

```json
{"status":"ok"}
```

## Настройки и секреты

Основной файл настроек для Docker Compose:

```text
deploy/.env
```

В репозитории хранится только шаблон:

```text
deploy/.env.example
```

Создать локальный `.env`:

```bash
cp deploy/.env.example deploy/.env
```

Файл `deploy/.env` добавлен в `.gitignore`, его нельзя коммитить.

### Обязательные переменные

| Переменная | Для чего нужна | Где используется |
| --- | --- | --- |
| `POSTGRES_DB` | Имя базы данных | PostgreSQL |
| `POSTGRES_USER` | Пользователь базы | PostgreSQL |
| `POSTGRES_PASSWORD` | Пароль пользователя базы | PostgreSQL |
| `DATABASE_URL` | Строка подключения backend к PostgreSQL | `api-service`, `mqtt-ingestion-service` |
| `JWT_SECRET` | Секрет подписи access token | `api-service` |
| `NATS_URL` | Адрес NATS | backend services |
| `MQTT_BROKER_URL` | Адрес MQTT broker | `mqtt-ingestion-service` |

### Дополнительные переменные

| Переменная | Значение по умолчанию | Назначение |
| --- | --- | --- |
| `HTTP_ADDR` | `:8080` | Адрес/порт API внутри контейнера |
| `ACCESS_TOKEN_TTL_MINUTES` | `1440` | Время жизни JWT access token |
| `MQTT_USERNAME` | пусто | MQTT login, если Mosquitto требует авторизацию |
| `MQTT_PASSWORD` | пусто | MQTT password |
| `MQTT_CLIENT_ID` | `hivemonitor-mqtt-ingestion` | MQTT client id ingestion-сервиса |
| `MQTT_TELEMETRY_TOPIC` | список telemetry/events/status topics | MQTT topics для подписки backend |
| `DEFAULT_APIARY_ID` | пусто | Fallback apiary id для legacy-сценариев |
| `DEFAULT_TELEMETRY_INTERVAL_MINUTES` | `30` | Ожидаемый интервал передачи данных |

## Генерация секретов

### JWT_SECRET

Для dev можно оставить default, но для VPS/production обязательно заменить.

Сгенерировать:

```bash
openssl rand -base64 64
```

Вставить в `deploy/.env`:

```text
JWT_SECRET=...
```

После изменения перезапустить backend:

```bash
docker compose -f deploy/docker-compose.yml up -d api-service
```

Старые access token станут недействительными.

### POSTGRES_PASSWORD

Сгенерировать:

```bash
openssl rand -base64 32
```

Вставить одинаково в `POSTGRES_PASSWORD` и внутрь `DATABASE_URL`:

```text
POSTGRES_PASSWORD=...
DATABASE_URL=postgres://hivemonitor:...@postgres:5432/hivemonitor?sslmode=disable
```

Важно: если volume PostgreSQL уже создан со старым паролем, простая замена `POSTGRES_PASSWORD` не поменяет пароль внутри существующей базы. Для dev можно пересоздать volume, для production нужно менять пароль SQL-командой.

Dev-сброс базы:

```bash
docker compose -f deploy/docker-compose.yml down -v
docker compose -f deploy/docker-compose.yml up -d --build
```

Команда `down -v` удаляет данные PostgreSQL, NATS и Mosquitto volumes.

### MQTT credentials

В текущем dev-MVP Mosquitto разрешает anonymous access через:

```text
deploy/mosquitto/mosquitto.conf
```

Для production нужно включить password file и ACL. Пример генерации пользователя:

```bash
docker run --rm eclipse-mosquitto:2 mosquitto_passwd -b -c /tmp/passwd apiary-demo strong-password
```

Практический production-вариант:

1. Создать файл паролей на сервере, например `deploy/mosquitto/passwd`.
2. Добавить volume в `deploy/docker-compose.yml`.
3. Изменить `deploy/mosquitto/mosquitto.conf`:

```text
allow_anonymous false
password_file /mosquitto/config/passwd
```

4. В `deploy/.env` указать:

```text
MQTT_USERNAME=apiary-demo
MQTT_PASSWORD=strong-password
```

5. Такие же `mqttUser` и `mqttPassword` указать в локальной настройке firmware устройства.

ACL по пасекам пока не реализован в compose-конфигурации. Целевой production-контракт: отдельные credentials на уровне пасеки и доступ только к topics этой пасеки.

## Ключи внешних сервисов

Для текущего MVP внешние ключи не обязательны.

Пока не нужны:

- погодный API token;
- Telegram bot token;
- push notification keys;
- платежный provider token;
- AI/OpenAI token;
- SMS provider token.

Эти интеграции заложены в ТЗ как будущие этапы. Когда они появятся, настройки должны храниться в `deploy/.env` или в production secret storage, а не в репозитории.

Рекомендуемые будущие переменные:

```text
WEATHER_PROVIDER=openweather
WEATHER_API_KEY=
TELEGRAM_BOT_TOKEN=
PUSH_PROVIDER=
PUSH_API_KEY=
AI_PROVIDER=
AI_API_KEY=
```

## Миграции базы данных

В Docker-сборке backend содержит миграции в контейнере:

```text
/app/migrations
```

Если сервисы запускаются локально без Docker, миграции можно применить из `backend/`:

```bash
cd backend
go run github.com/pressly/goose/v3/cmd/goose -dir migrations postgres "$DATABASE_URL" up
```

Проверить статус:

```bash
go run github.com/pressly/goose/v3/cmd/goose -dir migrations postgres "$DATABASE_URL" status
```

В Docker Compose сценарии миграции выполняются backend-приложением при старте, если это включено в текущей реализации сервиса. Если после изменения схемы появились ошибки API, сначала проверьте логи `api-service`.

## Проверка backend

Health:

```bash
curl http://localhost:8080/healthz
```

Регистрация пользователя:

```bash
curl -s -X POST http://localhost:8080/auth/register \
  -H 'Content-Type: application/json' \
  -d '{"email":"owner@example.com","password":"password123","name":"Owner"}'
```

Логин:

```bash
curl -s -X POST http://localhost:8080/auth/login \
  -H 'Content-Type: application/json' \
  -d '{"email":"owner@example.com","password":"password123"}'
```

В ответе будет access token. Его нужно передавать в защищенные API:

```text
Authorization: Bearer <access_token>
```

## Тестовые данные

Для проверки frontend и API можно засеять локальную базу демо-данными:

```bash
docker exec -i deploy-postgres-1 psql -U hivemonitor -d hivemonitor < backend/seeds/dev_seed.sql
```

После seed можно войти во frontend:

```text
http://localhost:5173
```

Тестовый пользователь:

```text
email: demo@hivemonitor.local
password: password123
```

Seed создает:

- организацию `Демо хозяйство HiveMonitor`;
- две пасеки;
- несколько ульев;
- назначенные устройства;
- одно непривязанное устройство;
- телеметрию за последние 24 часа;
- события пасеки и ульев.

Файл seed:

```text
backend/seeds/dev_seed.sql
```

Seed идемпотентный: его можно запускать повторно, он обновляет фиксированный демо-набор.

## Проверка MQTT ingestion

Посмотреть topics, на которые подписался ingestion:

```bash
docker compose -f deploy/docker-compose.yml logs --tail 50 mqtt-ingestion-service
```

Опубликовать тестовую telemetry в целевой apiary-aware topic:

```bash
docker exec -i deploy-mosquitto-1 mosquitto_pub \
  -h localhost \
  -t apiaries/demo-apiary/devices/demo-device/telemetry \
  -m '{"schemaVersion":1,"deviceId":"demo-device","apiaryId":"demo-apiary","timestamp":"2026-05-04T00:00:00Z","weight":42.5,"temperature":28.1,"humidity":61.2,"batteryPercent":87}'
```

Проверить логи:

```bash
docker compose -f deploy/docker-compose.yml logs --tail 50 mqtt-ingestion-service
```

Legacy topic для совместимости:

```text
hives/{deviceId}/telemetry
```

Целевые production topics:

```text
apiaries/{apiaryId}/devices/{deviceId}/telemetry
apiaries/{apiaryId}/devices/{deviceId}/events
apiaries/{apiaryId}/devices/{deviceId}/status
```

## Настройка web

Для разработки:

```bash
cd web
npm install
npm run dev
```

Vite proxy настроен в:

```text
web/vite.config.ts
```

Сейчас proxy направляет эти пути на `http://localhost:8080`:

```text
/auth
/me
/organizations
/apiaries
/hives
/healthz
```

Для production web нужно будет собрать static files:

```bash
cd web
npm run build
```

Результат:

```text
web/dist/
```

`web/dist/` не коммитится. Его должен отдавать reverse proxy или отдельный static hosting.

## Настройка firmware

Устройство настраивается через локальный web-интерфейс firmware.

Минимальные поля:

| Поле firmware | Что указать |
| --- | --- |
| `wifiSsid` | Wi-Fi сеть пасеки или тестовой сети |
| `wifiPassword` | Пароль Wi-Fi |
| `mqttHost` | IP/host сервера с Mosquitto |
| `mqttPort` | `1883` для dev |
| `mqttTls` | `false` для dev |
| `mqttUser` | MQTT login, если включена авторизация |
| `mqttPassword` | MQTT password, если включена авторизация |
| `deviceId` | Уникальный id устройства |
| `apiaryId` | Id пасеки для целевых topics |
| `measurementIntervalSeconds` | `1800` по умолчанию |

Если `apiaryId` пустой, firmware публикует legacy topics:

```text
hives/{deviceId}/telemetry
hives/{deviceId}/events
hives/{deviceId}/status
```

Если `apiaryId` задан, firmware публикует целевые topics:

```text
apiaries/{apiaryId}/devices/{deviceId}/telemetry
apiaries/{apiaryId}/devices/{deviceId}/events
apiaries/{apiaryId}/devices/{deviceId}/status
```

Для dev на той же машине `mqttHost` должен быть IP компьютера в локальной сети, а не `localhost`, потому что для ESP `localhost` означает само устройство.

## Настройка устройства в backend

Текущий MVP поддерживает сценарий:

1. Устройство публикует telemetry/status/events в MQTT.
2. Backend видит неизвестный `deviceId` как непривязанное устройство.
3. Пользователь создает организацию, пасеку и улей в web.
4. Пользователь выбирает непривязанное устройство и привязывает его к улью.
5. При привязке выбирается режим старых данных:
   - привязать старые данные к улью;
   - оставить как есть;
   - удалить старые данные.

Перенос устройства между пасеками по ТЗ требует сброса настроек устройства и повторной начальной настройки.

## VPS-развертывание

Базовый порядок:

1. Установить Docker Engine и Docker Compose plugin.
2. Склонировать репозиторий на сервер.
3. Создать `deploy/.env` из `deploy/.env.example`.
4. Заменить `POSTGRES_PASSWORD`, `DATABASE_URL`, `JWT_SECRET`.
5. Поднять стек:

```bash
docker compose -f deploy/docker-compose.yml up -d --build
```

6. Проверить:

```bash
docker compose -f deploy/docker-compose.yml ps
curl http://localhost:8080/healthz
```

7. Настроить reverse proxy для HTTPS.
8. Настроить MQTT credentials/TLS перед подключением реальных пасек.
9. Настроить backup PostgreSQL.

Production-рекомендации:

- не публиковать PostgreSQL наружу;
- не публиковать NATS наружу без авторизации;
- закрыть Mosquitto anonymous access;
- включить TLS для web/API и MQTT;
- хранить `.env` только на сервере;
- регулярно делать backup volume PostgreSQL;
- добавить мониторинг диска, CPU, RAM и логов контейнеров.

## Reverse proxy

В MVP reverse proxy еще не добавлен в репозиторий. Для production подойдет Caddy, Nginx или Traefik.

Целевая схема:

```text
https://app.example.com      -> web static files
https://api.example.com      -> api-service:8080
mqtts://mqtt.example.com     -> mosquitto:8883
```

Для локального dev reverse proxy не нужен.

## Backup

Dev backup PostgreSQL:

```bash
docker exec deploy-postgres-1 pg_dump -U hivemonitor hivemonitor > hivemonitor-backup.sql
```

Restore в dev:

```bash
docker exec -i deploy-postgres-1 psql -U hivemonitor hivemonitor < hivemonitor-backup.sql
```

Для production нужно хранить backup вне сервера или минимум вне Docker volume: object storage, отдельный backup-сервер или managed backup.

## Обновление системы

Получить изменения:

```bash
git pull
```

Пересобрать и перезапустить backend-стек:

```bash
docker compose -f deploy/docker-compose.yml up -d --build
```

Проверить:

```bash
docker compose -f deploy/docker-compose.yml ps
curl http://localhost:8080/healthz
```

Пересобрать web:

```bash
cd web
npm install
npm run build
```

## Диагностика

Статус контейнеров:

```bash
docker compose -f deploy/docker-compose.yml ps
```

Логи всех сервисов:

```bash
docker compose -f deploy/docker-compose.yml logs --tail 100
```

Логи API:

```bash
docker compose -f deploy/docker-compose.yml logs --tail 100 api-service
```

Логи MQTT ingestion:

```bash
docker compose -f deploy/docker-compose.yml logs --tail 100 mqtt-ingestion-service
```

Проверить Postgres:

```bash
docker exec deploy-postgres-1 pg_isready -U hivemonitor -d hivemonitor
```

Проверить NATS monitoring:

```bash
curl http://localhost:8222/varz
```

Проверить frontend:

```bash
curl -I http://localhost:5173/
```

Проверить frontend proxy:

```bash
curl http://localhost:5173/healthz
```

## Частые проблемы

### `permission denied while trying to connect to the docker API`

Пользователь не имеет доступа к Docker socket.

Проверить группы:

```bash
id
```

Добавить пользователя в группу Docker:

```bash
sudo usermod -aG docker $USER
```

После этого нужно выйти из сессии и войти снова или перезагрузить систему.

### `localhost:5173/healthz` возвращает `500`

Web работает, но backend API недоступен для Vite proxy.

Проверить:

```bash
curl http://localhost:8080/healthz
docker compose -f deploy/docker-compose.yml ps
```

Если backend остановлен:

```bash
docker compose -f deploy/docker-compose.yml up -d
```

### Устройство публикует данные, но они не видны в улье

Проверить:

- совпадает ли `deviceId`;
- задан ли `apiaryId`;
- правильный ли MQTT host для ESP;
- видит ли backend topic;
- привязано ли устройство к улью в web;
- нет ли ошибок в логах `mqtt-ingestion-service`.

Команда:

```bash
docker compose -f deploy/docker-compose.yml logs --tail 100 mqtt-ingestion-service
```

### После изменения `POSTGRES_PASSWORD` база не стартует

Если volume уже существовал, пароль в `POSTGRES_PASSWORD` не меняет пароль внутри базы. Для dev проще удалить volumes:

```bash
docker compose -f deploy/docker-compose.yml down -v
docker compose -f deploy/docker-compose.yml up -d --build
```

Для production нельзя удалять volume без backup.

### Порт уже занят

Найти процесс:

```bash
sudo lsof -i :8080
```

Либо изменить published port в `deploy/docker-compose.yml`.

## Где лежат настройки

| Настройка | Файл/место |
| --- | --- |
| Docker Compose services | `deploy/docker-compose.yml` |
| Docker env template | `deploy/.env.example` |
| Реальные Docker secrets/env | `deploy/.env` |
| Mosquitto config | `deploy/mosquitto/mosquitto.conf` |
| Backend env template | `backend/.env.example` |
| Backend config loader | `backend/internal/config/config.go` |
| Backend migrations | `backend/migrations/` |
| Web dev proxy | `web/vite.config.ts` |
| Web package/scripts | `web/package.json` |
| Firmware config model | `firmware/shared/config/AppConfig.h` |
| Firmware docs | `firmware/docs/README.md` |

## Что еще нужно до production

- Production profile для Docker Compose.
- Reverse proxy с TLS.
- MQTT authentication, ACL и TLS.
- Backup job для PostgreSQL.
- Secret storage или минимум защищенный `.env` на сервере.
- Healthcheck для `api-service` и `mqtt-ingestion-service` в compose.
- Логирование с ротацией.
- Мониторинг сервера.
- CI/CD pipeline для сборки и деплоя.
- Отдельная стратегия OTA firmware.
