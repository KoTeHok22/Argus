// Copyright 2026 Argus Team
// Licensed under the Apache License, Version 2.0

#include "argus_core/tracking.hpp"

#include <cmath>
#include <limits>

namespace argus {

namespace {
constexpr int kStateDim = 6;  // [x, y, z, vx, vy, vz]
}

ObstacleTracker::ObstacleTracker(const TrackingParams& p) : p_(p) {}

float compute_ttc(const Track& t, float train_speed_mps) {
  // Поезд движется вперёд по X; объект впереди имеет x > 0.
  // Скорость сближения = скорость поезда минус скорость объекта по X.
  const float closing = train_speed_mps - t.state(3);
  if (closing <= 0.0f) {
    return -1.0f;  // не сближаемся
  }
  const float x = t.state(0);
  if (x <= 0.0f) {
    return -1.0f;
  }
  return x / closing;
}

std::vector<Track> ObstacleTracker::update(
    const std::vector<Cluster>& clusters, float dt, float train_speed_mps) {
  // --- Ассоциация: жадный nearest-neighbor (достаточно для каркаса). ---
  std::vector<bool> matched(clusters.size(), false);
  for (auto& tr : tracks_) {
    float best = p_.max_association_distance;
    int best_j = -1;
    for (size_t j = 0; j < clusters.size(); ++j) {
      if (matched[j]) continue;
      const Eigen::Vector3f c = clusters[j].centroid;
      const float d = (c.head<3>() - tr.state.head<3>()).norm();
      if (d < best) {
        best = d;
        best_j = static_cast<int>(j);
      }
    }
    if (best_j >= 0) {
      matched[best_j] = true;
      const Cluster& c = clusters[best_j];

      // Предсказание (константная скорость) + обновление позиции.
      tr.state.head<3>() += tr.state.tail<3>() * dt;
      tr.state(3) = (c.centroid.x() - tr.state(0)) / std::max(dt, 1e-3f);
      tr.state(0) = c.centroid.x();
      tr.state(1) = c.centroid.y();
      tr.state(2) = c.centroid.z();

      tr.hits++;
      tr.misses = 0;
      tr.confirmed = tr.hits >= p_.min_hits_to_confirm;
      tr.nearest_range = c.nearest_range;
      tr.volume = c.volume;
      tr.votes_free_space = c.votes_free_space;
      tr.votes_no_return = c.votes_no_return;
      tr.votes_geometry = c.votes_geometry;
      tr.ttc = compute_ttc(tr, train_speed_mps);
    } else {
      tr.misses++;
      // Прогноз на пропущенный кадр.
      tr.state.head<3>() += tr.state.tail<3>() * dt;
    }
  }

  // --- Сброс потерянных треков. ---
  tracks_.erase(
      std::remove_if(tracks_.begin(), tracks_.end(),
                     [&](const Track& t) { return t.misses > p_.max_misses_to_keep; }),
      tracks_.end());

  // --- Новые треки. ---
  for (size_t j = 0; j < clusters.size(); ++j) {
    if (matched[j]) continue;
    Track t;
    t.id = next_id_++;
    t.state.setZero();
    t.state.head<3>() = clusters[j].centroid;
    t.covariance = Eigen::Matrix<float, 6, 6>::Identity() * p_.measurement_noise;
    t.hits = 1;
    t.misses = 0;
    t.confirmed = false;  // подтверждение только после min_hits_to_confirm
    t.nearest_range = clusters[j].nearest_range;
    t.volume = clusters[j].volume;
    t.votes_free_space = clusters[j].votes_free_space;
    t.votes_no_return = clusters[j].votes_no_return;
    t.votes_geometry = clusters[j].votes_geometry;
    tracks_.push_back(t);
  }

  // --- Подтверждённые треки на выход. ---
  std::vector<Track> confirmed;
  for (const auto& t : tracks_) {
    if (t.confirmed) {
      confirmed.push_back(t);
    }
  }
  return confirmed;
}

}  // namespace argus
