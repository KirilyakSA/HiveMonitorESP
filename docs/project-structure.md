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
  implementation-status.md     Статус реализации и соответствие ТЗ
  references/                  Исходные PDF/OneNote заметки
```

## Принцип разделения

В корне `docs/` находятся только документы уровня системы: назначение, архитектура, структура и MVP.

Детальные требования к реализации находятся рядом с каждым подпроектом:

```text
firmware/docs/README.md
backend/docs/README.md
web/docs/README.md
mobile/docs/README.md
deploy/docs/README.md
```

Исходные заметки и материалы, которые использовались при подготовке ТЗ, находятся в `docs/references/`.
