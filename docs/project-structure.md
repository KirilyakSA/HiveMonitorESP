# Структура проекта

## Каталоги репозитория

```text
backend/              Go backend и API
deploy/               Docker Compose, инфраструктура, окружения
docs/                 Документация проекта
firmware/             Прошивки ESP8266/ESP32
mobile/               React Native приложение
web/                  React + TypeScript + Vite web-интерфейс
sketch_apr25a/        Старый Arduino-скетч, исходный прототип
```

## Документация

```text
docs/
  README.md                    Верхнеуровневое описание системы
  architecture.md              Архитектура
  project-structure.md         Структура репозитория
  mvp.md                       Состав MVP
  references/                  Исходные PDF/OneNote заметки
  subprojects/
    firmware/README.md         Детальное ТЗ прошивки
    backend/README.md          Детальное ТЗ backend
    web/README.md              Детальное ТЗ web-интерфейса
    mobile/README.md           Детальное ТЗ мобильных приложений
    deploy/README.md           Детальное ТЗ развертывания
```

## Принцип разделения

В корне `docs/` находятся только документы уровня системы: назначение, архитектура, структура и MVP.

Детальные требования к реализации находятся в `docs/subprojects/<subproject>/README.md`.

Исходные заметки и материалы, которые использовались при подготовке ТЗ, находятся в `docs/references/`.
