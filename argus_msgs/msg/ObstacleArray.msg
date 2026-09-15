# Итог одного кадра обработки: сводка по всем обнаруженным препятствиям.
# Это главный выходной топик системы (/argus/obstacles).

std_msgs/Header header

# Состояние пути
uint8 status
uint8 STATUS_CLEAR    = 0   # путь свободен
uint8 STATUS_WARNING  = 1   # объект в габарите, но далеко
uint8 STATUS_BLOCKED  = 2   # объект в габарите, критично близко
uint8 STATUS_DEGRADED = 3   # данные отсутствуют или деградировали; CLEAR гарантировать нельзя

# Сводка
float32 nearest_range_m     # -1 если препятствий нет
uint32  obstacles_detected
Obstacle[] obstacles

# Производительность кадра
float32 processing_ms
float32 fps

# Готовность модели тоннеля (прогрета ли карта свободного пространства)
bool model_ready
