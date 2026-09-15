// Copyright 2026 Argus Team
// Licensed under the Apache License, Version 2.0

#include "argus_core/clustering.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace argus {

float neighbor_distance_at(float base, float d, float ref, bool adaptive) {
  if (!adaptive) {
    return base;
  }
  return base * std::max(1.0f, d / std::max(0.001f, ref));
}

std::vector<Cluster> cluster_anomalies(const CleanCloud& cloud,
                                       const RangeImage& ri,
                                       const AnomalySet& merged,
                                       const ClusteringParams& p) {
  std::vector<Cluster> out;
  if (merged.indices.empty() || !ri.valid()) {
    return out;
  }

  // Пометим аномальные точки (индексы в cloud -> позиция в развёртке).
  // Индекс в cloud = az * rings + ring при полной развёртке; но фильтр
  // удалил часть точек, поэтому сопоставляем через ring/az, если возможно.
  // Для каркаса: считаем, что аномалии заданы позициями (az, ring) через
  // индекс в cloud ДО фильтрации — уточняется в Ф4.7.2 вместе с детекторами.
  // Здесь реализована связность по range image от переданных индексов,
  // трактуемых как позиции в развёртке (az * height + ring).

  std::vector<uint8_t> visited(ri.range.size(), 0);
  std::vector<size_t> stack;

  const auto ring_of = [&](size_t idx) { return idx % ri.height; };
  const auto az_of = [&](size_t idx) { return idx / ri.height; };

  for (uint32_t seed : merged.indices) {
    const size_t s = seed;
    if (s >= ri.range.size() || visited[s]) {
      continue;
    }

    Cluster c;
    std::vector<uint32_t> members;
    stack.clear();
    stack.push_back(s);
    visited[s] = 1;

    while (!stack.empty()) {
      const size_t cur = stack.back();
      stack.pop_back();
      members.push_back(static_cast<uint32_t>(cur));

      std::array<std::pair<uint32_t, uint32_t>, 8> nb{};
      const uint32_t nn =
          neighbors8(ri, az_of(cur), ring_of(cur), nb);
      const float range_here = ri.range[cur];

      for (uint32_t k = 0; k < nn; ++k) {
        const size_t nidx = ri.idx(nb[k].first, nb[k].second);
        if (visited[nidx]) continue;

        const bool anomalous =
            std::binary_search(merged.indices.begin(), merged.indices.end(),
                               static_cast<uint32_t>(nidx));
        if (!anomalous) continue;

        // Связность по фактической дистанции (адаптивный порог).
        const float r2 = ri.range[nidx];
        if (!std::isfinite(r2) || !std::isfinite(range_here)) continue;
        const float thr =
            neighbor_distance_at(p.neighbor_distance, range_here,
                                 p.reference_range, p.adaptive_scaling);
        if (std::abs(r2 - range_here) > thr) continue;

        visited[nidx] = 1;
        stack.push_back(nidx);
      }
    }

    if (members.size() < p.min_cluster_size ||
        members.size() > p.max_cluster_size) {
      continue;
    }

    // Метрики кластера в декартовых координатах.
    Eigen::Vector3f sum = Eigen::Vector3f::Zero();
    c.min_corner.setConstant(std::numeric_limits<float>::max());
    c.max_corner.setConstant(-std::numeric_limits<float>::max());
    c.nearest_range = std::numeric_limits<float>::max();

    for (uint32_t m : members) {
      const size_t ci = m;  // позиция в развёртке == индекс в cloud при
                            // полной развёртке (уточняется в Ф4.7.2)
      if (ci >= cloud.size()) continue;
      const Eigen::Vector3f pt(cloud.x[ci], cloud.y[ci], cloud.z[ci]);
      sum += pt;
      c.min_corner = c.min_corner.cwiseMin(pt);
      c.max_corner = c.max_corner.cwiseMax(pt);
      c.nearest_range =
          std::min(c.nearest_range, pt.norm());
      c.point_count++;
    }

    if (c.point_count == 0) continue;
    c.centroid = sum / static_cast<float>(c.point_count);

    const Eigen::Vector3f ext = c.max_corner - c.min_corner;
    if (ext.minCoeff() < p.min_extent_m || ext.maxCoeff() > p.max_extent_m) {
      continue;
    }
    c.volume = ext.x() * ext.y() * ext.z();
    out.push_back(std::move(c));
  }

  return out;
}

}  // namespace argus
