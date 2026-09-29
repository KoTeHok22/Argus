# Argus

Инструмент обработки пространственных данных и анализа последовательностей измерений.

## Быстрый старт

```bash
docker compose -f docker/docker-compose.yml up --build ui
```

Панель открывается на `http://localhost:8080`.

Прогон записи:

```bash
docker compose -f docker/docker-compose.yml run --rm argus detect /data/<recording>
```

Без compose:

```bash
./scripts/fetch_third_party.sh
docker build -f docker/Dockerfile -t argus .
docker run --rm --ipc=host --shm-size=256m \
     -v "$PWD/data/recordings":/data \
     argus detect /data/<recording>
```

`--ipc=host` и `--shm-size=256m` нужны для крупных сообщений.

Панель без Docker: `./scripts/run_ui.sh`, в Windows — `$env:PYTHONPATH="argus_web"; python -m argus_web`.

## Режимы контейнера

| Команда | Назначение |
|---|---|
| `detect <recording> [rate] [params]` | Прогон записи с выводом результата |
| `demo <recording>` | Прогон с визуализацией RViz2 |
| `eval <recording>` | Отчёт по прогону |
| `ui` | Панель в браузере |
| `shell` | Оболочка внутри образа |
| `help` | Справка (по умолчанию) |

## Что делает система

1. Читает `PointCloud2` по фактической раскладке `fields[]`.
2. Отбрасывает мусор и помечает лучи без возврата.
3. Вычитает пол и рельсы порогом по Z. Метод по умолчанию — `z_threshold`; Patchwork++ доступен как опция.
4. Ищет геометрические отклонения и подтверждает их во времени.
5. Формирует результат и диагностические события.

## Проверки

```bash
scripts/ci/all.sh      # lint + build + test
```

## Структура

| Путь | Назначение |
|---|---|
| `argus_core` | Алгоритмы, C++ тесты, CLI `offline_detector` |
| `argus_msgs` | Сообщения ROS 2 |
| `argus_node_*` | Ноды обработки |
| `argus_launch` | Launch-файлы и конфиги |
| `argus_eval` | Метрики и оценка |
| `argus_web` | Панель в браузере |
| `docker` | Образы и точка входа |
| `scripts` | Сборка, CI, синтетика |
| `third_party` | `argus.repos` |

## Документы

| Документ | О чём |
|---|---|
| [`docs/EXPERIMENTS.md`](docs/EXPERIMENTS.md) | Что пробовали, что отвергли и почему |

## Требования

Ubuntu 22.04, ROS 2 Humble, Docker. GPU не нужен.
