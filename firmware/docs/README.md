# Firmware

Смежный документ: [ТЗ backend HiveMonitor](backend-requirements.md).

## Назначение

Подпроект `firmware/` содержит прошивки для IoT-устройств HiveMonitor на ESP8266 и ESP32.

Прошивка отвечает за измерения, локальное хранение, настройку устройства, отправку телеметрии и локальное обслуживание.

## Платформы

Поддерживаются две альтернативные прошивки:

- ESP8266;
- ESP32.

Конкретные модели плат пока не фиксируются. Используются совместимые платы ESP8266/ESP32.

## Сборка

Основной инструмент сборки - PlatformIO.

Framework для MVP - Arduino framework внутри PlatformIO.

Окружения сборки:

```text
esp8266
esp32
```

## Предварительная структура

```text
firmware/
  platformio.ini
  shared/
    sensors/
    communication/
    storage/
    time/
    config/
    web/
    update/
    domain/
  esp8266/
    src/
    platform/
  esp32/
    src/
    platform/
```

## Датчики и модули

- Весовой модуль HX711, библиотека `GyverHX711`.
- Датчик температуры/влажности: DHT22 по умолчанию.
- Возможность выбрать другой тип датчика температуры/влажности.
- Возможность использовать отдельные датчики температуры и влажности.
- Датчик Холла для открытия улья.
- RTC-модуль как опциональный источник времени.
- Измерение батареи/питания: процент и напряжение.
- Bluetooth, если выбранный контроллер поддерживает его.
- LittleFS для буфера измерений.

## Пины по умолчанию

| Назначение | Пин ESP8266 |
| --- | --- |
| HX711 DOUT | D2 |
| HX711 SCK | D3 |
| Датчик температуры/влажности | D4 |
| Датчик Холла | D5 |
| Factory reset кнопка | D6 |

Пины датчиков должны быть настраиваемыми через конфигурацию устройства, где это технически возможно.

Для MVP изменение пинов выполняется через локальный web-интерфейс устройства. Удаленное изменение пинов переносится после MVP.

## Измерения

Период планового измерения по умолчанию - 30 минут. Период является настройкой устройства.

Телеметрия включает:

- `schemaVersion`;
- `firmwareVersion`;
- `configVersion`;
- `deviceId`;
- `apiaryId`, если задан в конфигурации;
- `timestamp`;
- `uptimeSeconds`;
- `measurementIntervalSeconds`;
- `weight`;
- `weightChange`;
- `temperature`;
- `humidity`;
- `hiveOpened`;
- `errorFlag`;
- `weightError`;
- `temperatureError`;
- `humidityError`;
- `batteryError`;
- `batteryPercent`;
- `batteryVoltage`;
- `rssi`;
- `freeHeap`.

## Вес и калибровка

Пороги являются настройками устройства.

Значения по умолчанию:

- `WEIGHT_THRESHOLD = 5.0 кг`;
- `SIGNIFICANT_WEIGHT_CHANGE = 20.0 кг`.

Для MVP калибровка весов выполняется через локальный web-интерфейс устройства.

Калибровка через backend переносится после MVP.

Следующий целевой workflow вынесен в backlog [#45](https://github.com/KirilyakSA/HiveMonitorESP/issues/45):

- RU: отдельная калибровка весов локально и через backend command API.
  EN: separate scale calibration locally and through backend command API.
- RU: тарирование пустого улья без рамок, но с крышкой, с сохранением базового веса улья.
  EN: tare an empty hive without frames but with cover and store the hive baseline weight.
- RU: второе тарирование при установке пустого магазина: сохранить вес семьи с рамками и базовый вес магазина.
  EN: second tare when adding an empty super: store colony-with-frames weight and super baseline weight.
- RU: firmware должна возвращать ack/result операции для backend-журнала.
  EN: firmware must return operation ack/result for the backend journal.

## MQTT topics и совместимость с backend

Firmware публикует legacy topics, если `apiaryId` не задан:

```text
hives/{deviceId}/telemetry
hives/{deviceId}/events
hives/{deviceId}/status
```

Если в конфигурации задан `apiaryId`, firmware публикует apiary-aware topics:

```text
apiaries/{apiaryId}/devices/{deviceId}/telemetry
apiaries/{apiaryId}/devices/{deviceId}/events
apiaries/{apiaryId}/devices/{deviceId}/status
```

Текущий backend MVP принимает legacy и apiary-aware topics для telemetry, events и status. Apiary-aware topics являются целевым production-контрактом для автоматического provisioning по пасеке. Legacy topics остаются compatibility mode для уже прошитых устройств и локального MVP/dev сценария.

`mqttUser` и `mqttPassword` в firmware совместимы с последним требованием "общие MQTT credentials на уровне пасеки" и используются независимо от `deviceToken`. Поле `deviceToken` остается legacy/fallback секретом локальной конфигурации, но не является основным backend-MVP способом авторизации устройства.

## MQTT-команды

Устройство подписывается на:

- `hives/{deviceId}/commands`;
- `hives/{deviceId}/config`.

Если задан `apiaryId`, устройство также подписывается на:

- `apiaries/{apiaryId}/devices/{deviceId}/commands`;
- `apiaries/{apiaryId}/devices/{deviceId}/config`.

Поддерживаемые команды:

- `reboot` / `restart` - перезагрузить устройство;
- `config_update` / `configUpdate` - применить JSON-конфигурацию;
- `hold_config_session` - удержать устройство активным и не уходить в deep sleep;
- `capture_weight` - выполнить одноразовый raw-замер веса для backend-мастера тары;
- `finish_config_session` - завершить сессию настройки и снова разрешить deep sleep;
- `firmware_update` - backend уже может передать metadata релиза (`version`, `artifact_url`, `checksum_sha256`), но OTA flashing в прошивке еще не реализован и сейчас возвращает ошибку "not implemented";
- legacy/dev команды `measure`, `tare`, `clearBuffer` пока сохранены для локальной совместимости.

Команды можно отправлять строкой (`measure`) или JSON:

```json
{"id":"command_uuid","command":"capture_weight","payload":{"purpose":"hive_tare"}}
```

Для обновления конфигурации:

```json
{"command":"configUpdate","data":{"measurementIntervalSeconds":900}}
```

Конфигурация хранит отдельное поле `schemaVersion` для версии формата настроек. `configVersion` остается счетчиком изменений настроек и увеличивается при сохранении через web или MQTT. Старые конфиги без `schemaVersion` автоматически помечаются текущей схемой, а конфиги с более новой схемой не загружаются старой прошивкой.

Ответ публикуется в текущий status topic: `hives/{deviceId}/status` в legacy mode или `apiaries/{apiaryId}/devices/{deviceId}/status` при заданном `apiaryId`.

Для команд из backend ответ содержит `commandId`, `ok`, `message` и при наличии `result`.
Для `capture_weight` результат содержит:

```json
{
  "type": "commandStatus",
  "commandId": "command_uuid",
  "command": "capture_weight",
  "ok": true,
  "result": {
    "weight_kg": 38.75,
    "raw_weight_kg": 38.75,
    "session_active": true
  }
}
```

## Датчик Холла и сон

Для ESP8266 поддерживаются режимы:

- `HALL_WAKE_RST` - датчик подключен через внешнюю схему к `RST`;
- `HALL_NO_WAKE` - пробуждения от открытия нет, состояние проверяется в плановом цикле.

Для ESP32 нужно предусмотреть wake-up по GPIO средствами deep sleep.

Перед уходом в deep sleep прошивка пытается доставить текущую и буферизованную телеметрию в MQTT в пределах ограниченного таймаута, затем коротко слушает входящие команды. Если пришла `hold_config_session`, deep sleep откладывается до `finish_config_session` или истечения hold timeout. После успешного `finish_config_session` прошивка публикует `commandStatus`, коротко дает MQTT-клиенту доставить ответ и снова разрешает deep sleep. Если связь недоступна, данные остаются в локальном буфере до следующего пробуждения.

## Время

Источник времени настраивается:

- `NTP`;
- `RTC`;
- `AUTO`.

Если RTC включен, устройство использует RTC как основной источник времени и периодически синхронизирует его по NTP значительно реже, чем в режиме без RTC.

## Локальный web-интерфейс устройства

В MVP локальный web-интерфейс:

- доступен постоянно в рабочей Wi-Fi сети;
- доступен через временную точку доступа для первичной настройки и восстановления;
- защищен паролем администратора устройства;
- не имеет разделения на роли;
- авторизованный пользователь имеет доступ ко всем функциям;
- одноязычный на русском языке;
- минимальный, аккуратный и функциональный.

Функции MVP:

- настройка Wi-Fi;
- настройка MQTT;
- настройка `deviceId`;
- настройка `apiaryId` для apiary-aware MQTT topics;
- ввод `deviceToken` как legacy/fallback секрета;
- выбор модулей;
- настройка пинов;
- настройка источника времени;
- настройка периода измерений;
- калибровка весов;
- просмотр текущих показаний;
- локальное обновление прошивки;
- очистка локального буфера;
- сброс настроек.

Fallback-точка доступа восстановления всегда включена при ошибке подключения к Wi-Fi. Отключение этой возможности намеренно не поддерживается, чтобы устройство нельзя было случайно оставить без локального способа настройки.

Web-интерфейс показывает ошибки сохранения настроек и включает в `/api/status` состояние MQTT и последний результат публикации телеметрии.

`/api/config` не возвращает сохраненные секреты: `deviceToken`, пароль администратора, Wi-Fi пароль, пароль fallback AP и MQTT пароль. В web-форме пустое поле секрета означает "оставить без изменений"; чтобы заменить секрет, нужно ввести новое значение.

Для защиты памяти ESP8266 входящий JSON конфигурации через web ограничен 4096 байтами, а входящие MQTT-команды ограничены 1024 байтами.

## Физический сброс настроек

Для восстановления устройства без доступа к web-интерфейсу предусмотрена физическая кнопка factory reset.

Подключение по умолчанию для ESP8266: кнопка между `D6` и `GND`, используется внутренняя подтяжка `INPUT_PULLUP`.

Чтобы сбросить настройки:

1. Обесточить или перезагрузить устройство.
2. Зажать кнопку factory reset.
3. Держать кнопку около 5 секунд после старта.
4. Устройство сбросит конфигурацию к заводским настройкам и перезагрузится.

## Локальное хранение

Для ESP8266:

- EEPROM только для маленьких настроек или служебных флагов;
- буфер измерений в LittleFS.

Для ESP32:

- NVS для маленьких настроек;
- буфер измерений в LittleFS.

При заполнении буфера удаляются самые старые данные по дням.

## Обновление прошивки

Обязательно для MVP:

- локальная загрузка файла прошивки через web-интерфейс;
- USB/serial как аварийный способ восстановления.

Локальный web-update проверяет старт, запись и завершение загрузки. При ошибке загрузки устройство возвращает ошибку и не перезагружается.

После MVP:

- публикация в topic с `apiary_id` или настраиваемым topic prefix;
- OTA по локальной сети;
- OTA через backend или сервер обновлений.
