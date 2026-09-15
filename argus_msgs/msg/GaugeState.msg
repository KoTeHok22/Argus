# Состояние габарита движения поезда. Публикуется (/argus/gauge).
# Используется визуализацией (RViz2) и оценщиком тревог.

std_msgs/Header header

# Параметры габарита (см. config/gauge_default.yaml)
float32 half_width      # половина ширины, м
float32 height          # высота от головки рельса, м
float32 nose_offset     # смещение от лидара до носа поезда, м
float32 max_range       # предел проверки, м

# Двумерный профиль габарита (вид спереди) для отрисовки
geometry_msgs/Polygon profile_2d

# Динамика поезда
float32 train_speed_mps
float32 braking_distance_m
