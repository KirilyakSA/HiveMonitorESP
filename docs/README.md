# HiveMonitor

HiveMonitor - комплексная система мониторинга ульев и пасеки.

Система состоит из IoT-устройств на ESP8266/ESP32, backend, базы данных, MQTT broker, web-интерфейса, мобильных приложений Android/iOS и инфраструктуры развертывания.

## Цели

- Собирать телеметрию с одного или нескольких ульев.
- Хранить историю измерений.
- Показывать текущее состояние ульев.
- Строить графики веса, температуры, влажности и питания.
- Фиксировать открытие улья.
- Выявлять резкие изменения веса и ошибки датчиков.
- Отправлять уведомления пользователю.
- Поддерживать настройку IoT-устройства через локальный web-интерфейс.
- Поддерживать web и mobile интерфейсы на русском, украинском и английском языках.

## Подпроекты

| Подпроект | Назначение | Документация |
| --- | --- | --- |
| `firmware/` | Прошивки ESP8266/ESP32 и локальный web-интерфейс устройства | [Firmware](../firmware/docs/README.md) |
| `backend/` | Go backend, API, обработка телеметрии и уведомлений | [Backend](../backend/docs/README.md) |
| `web/` | Web-интерфейс системы на React + TypeScript + Vite | [Web](../web/docs/README.md) |
| `mobile/` | Мобильные приложения Android/iOS на React Native + TypeScript | [Mobile](../mobile/docs/README.md) |
| `deploy/` | Docker-first развертывание backend, PostgreSQL и MQTT broker | [Deploy](../deploy/docs/README.md) |

## Верхнеуровневые документы

- [Архитектура](architecture.md)
- [Структура проекта](project-structure.md)
- [MVP](mvp.md)

## Исходные заметки

Исходные OneNote/PDF заметки сохранены в [references](references/).
