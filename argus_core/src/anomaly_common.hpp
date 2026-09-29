
#pragma once

#include <algorithm>
#include <cstdint>
#include <limits>
#include <vector>

#include "argus_core/types.hpp"

namespace argus {
namespace anomaly_detail {

constexpr uint32_t kNoCloudIndex = std::numeric_limits<uint32_t>::max();

inline float median_of_sorted(const std::vector<float>& sorted) {
    if (sorted.empty()) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    return sorted[sorted.size() / 2];
}

inline void sorted_insert(std::vector<float>& v, float x) {
    v.insert(std::upper_bound(v.begin(), v.end(), x), x);
}

inline void sorted_remove_one(std::vector<float>& v, float x) {
    const auto it = std::lower_bound(v.begin(), v.end(), x);
    if (it != v.end() && *it == x) {
        v.erase(it);
    }
}

inline uint32_t cyclic_az_diff(uint32_t a, uint32_t b, uint32_t width) {
    const uint32_t d = (a + width - b) % width;
    return std::min(d, width - d);
}

inline std::vector<uint32_t> build_cell_to_cloud(const CleanCloud& cloud, size_t n_cells) {
    std::vector<uint32_t> map(n_cells, kNoCloudIndex);
    const bool has_raw = !cloud.raw_idx.empty();
    for (size_t i = 0; i < cloud.size(); ++i) {
        const uint32_t src = has_raw ? cloud.raw_idx[i] : static_cast<uint32_t>(i);
        if (src < n_cells) {
            map[src] = static_cast<uint32_t>(i);
        }
    }
    return map;
}

} // namespace anomaly_detail
} // namespace argus
