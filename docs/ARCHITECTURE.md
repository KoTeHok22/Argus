# Архитектура Argus

Статус: десять пакетов ROS 2 Humble. Граф нод запускает `argus.launch.py`. Панель — `argus_web`. Диагностический world-space runner в ROS не включён.

## Компоненты

| Пакет | Роль |
|---|---|
| `argus_core` | Библиотека без rclcpp: фильтр, range image, земля, одометрия, карта, детекторы, контролируемая зона, кластеризация, трекинг, fusion |
| `argus_msgs` | `Obstacle`, `ObstacleArray`, `Diagnostics`, `GaugeState`, `CleanCloud`, `AnomalySet` |
| `argus_node_preprocess` | Подписка на `PointCloud2`, sanity-фильтр, range image, диагностика |
| `argus_node_odometry` | 4DoF point-to-plane ICP, топик `/argus/pose` |
| `argus_node_tunnel_model` | Модель свободного пространства + профиль сечения |
| `argus_node_detector` | Три детектора аномалий |
| `argus_node_fusion` | Голосование, контролируемая зона, кластеризация, трекинг, тревоги |
| `argus_launch` | Launch-файлы, RViz2-конфиг, параметры |
| `argus_eval` | Оценка: метрики, синтетические препятствия, отчёты |
| `argus_web` | Панель в браузере: загрузка записи, эфир облака, отчёт и snapshot настроек контролируемой зоны для ROS-прогона |

## Принципы

1. **`argus_core` — без ROS.** Вся геометрия тестируется юнит-тестами
   без ROS; ноды — тонкие обёртки.
2. **Динамическая раскладка облака.** `fields[]` и `point_step` читаются
   из сообщения; жёсткие смещения запрещены.
3. **Range image вместо KD-tree.** Развёртка `ring = i mod 128` даёт
   O(1)-соседей и дешёвую связную сегментацию.
4. **Состояние пути.** `STATUS_CLEAR` публикуется только когда система
   полностью здорова; любая деградация — `STATUS_DEGRADED`.
