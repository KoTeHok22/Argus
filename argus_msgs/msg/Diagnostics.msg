# Диагностика конвейера. Публикуется каждый кадр (/argus/diagnostics).
# Назначение: отладка, замеры производительности, проверка совпадения
# наблюдений с форензикой данных (см. PLAN.md, раздел 2).

std_msgs/Header header

# --- Входные данные: счётчики точек ---
uint32 n_points_raw          # точек пришло в кадре
uint32 n_points_valid        # прошло sanity-фильтр
uint32 n_points_rejected     # отброшено всего
uint32 reject_zero           # отброшено ровно (0,0,0) — нет возврата
uint32 reject_nonfinite      # отброшено NaN/Inf
uint32 reject_out_of_range   # отброшено по дальности |p|

# --- Входные данные: структура развёртки ---
uint32 rings_detected        # число колец (ожидается 128)
uint32 az_steps_detected     # азимутальных шагов (307200/128=2400 или 921600/128=7200)
uint8  echo_multiplier       # 1 обычный режим, 3 тройное эхо (doubleT_obstacle)
uint32 point_step            # байт на точку (наблюдается 26)
bool   layout_ok             # раскладка полей распознана
string layout_error          # описание проблемы, если layout_ok == false

# --- Модель тоннеля ---
float32 profile_median_deviation_m  # медианное отклонение от профиля (<0.3 м на пустом)
uint32  model_voxel_count           # занятых вокселей в карте
bool    model_ready                 # карта прогрета

# --- Производительность (миллисекунды по шагам) ---
float32 t_filter_ms
float32 t_range_image_ms
float32 t_odometry_ms
float32 t_model_ms
float32 t_detect_ms
float32 t_cluster_ms
float32 t_track_ms
float32 t_total_ms
float32 fps

# --- Счётчики за всё время работы ---
uint64 frames_processed
uint64 alerts_total
uint64 false_alerts_suppressed
