
#include "argus_core/range_image.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace argus {

uint32_t detect_rings(const CleanCloud& cloud, uint32_t max_rings) {
    if (cloud.size() < 16) {
        return cloud.rings > 0 ? cloud.rings : 1;
    }

    const uint16_t first = cloud.ring[0];
    for (uint32_t period = 2; period <= max_rings && period < cloud.size(); ++period) {
        if (cloud.ring[period] == first) {
            const uint32_t check =
                std::min<uint32_t>(4 * period, static_cast<uint32_t>(cloud.size()));
            bool ok = true;
            for (uint32_t i = 0; i < check; ++i) {
                if (cloud.ring[i] != cloud.ring[i % period]) {
                    ok = false;
                    break;
                }
            }
            if (ok) {
                return period;
            }
        }
    }

    if (cloud.rings > 0) {
        return cloud.rings;
    }
    uint16_t max_ring = 0;
    for (uint16_t r : cloud.ring) {
        max_ring = std::max(max_ring, r);
    }
    return static_cast<uint32_t>(max_ring) + 1;
}

RangeImage build_range_image(const CleanCloud& cloud, const RangeImageParams& params) {
    RangeImage ri;

    uint32_t rings = 0;
    if (cloud.rings > 1) {
        rings = cloud.rings;
    } else if (params.prefer_ring_field && !cloud.ring.empty()) {
        rings = detect_rings(cloud, params.max_rings);
    }
    if (rings <= 1) {
        rings = params.rings_fallback;
    }

    const bool has_raw = !cloud.raw_idx.empty();
    uint32_t n_eff = static_cast<uint32_t>(cloud.size());
    if (has_raw) {
        n_eff = cloud.n_raw;
        for (uint32_t r : cloud.raw_idx) {
            n_eff = std::max(n_eff, r + 1);
        }
    }

    ri.height = rings;
    ri.width = std::max<uint32_t>(1, static_cast<uint32_t>((n_eff + rings - 1) / rings));

    const size_t total = static_cast<size_t>(ri.width) * ri.height;
    ri.range.assign(total, std::numeric_limits<float>::quiet_NaN());
    ri.intensity.assign(total, 0.0f);
    ri.no_return_mask.assign(total, 0);

    for (size_t i = 0; i < cloud.size(); ++i) {
        const uint32_t src = has_raw ? cloud.raw_idx[i] : static_cast<uint32_t>(i);
        const uint32_t ring = src % ri.height;
        const uint32_t az = src / ri.height;
        if (az >= ri.width) {
            continue;
        }
        const size_t idx = ri.idx(az, ring);

        const float x = cloud.x[i], y = cloud.y[i], z = cloud.z[i];
        ri.range[idx] = std::sqrt(x * x + y * y + z * z);
        ri.intensity[idx] = cloud.intensity[i];
    }

    for (uint32_t src : cloud.no_return_raw) {
        if (src >= n_eff) {
            continue;
        }
        const uint32_t ring = src % ri.height;
        const uint32_t az = src / ri.height;
        ri.no_return_mask[ri.idx(az, ring)] = 1;
    }

    return ri;
}

float sample(const RangeImage& ri, int az, int ring) {
    if (az < 0 || ring < 0) return std::numeric_limits<float>::quiet_NaN();
    const auto u = static_cast<uint32_t>(az);
    const auto v = static_cast<uint32_t>(ring);
    if (u >= ri.width || v >= ri.height) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    return ri.range[ri.idx(u, v)];
}

uint32_t neighbors8(const RangeImage& ri, uint32_t az, uint32_t ring,
                    std::array<std::pair<uint32_t, uint32_t>, 8>& out) {
    uint32_t n = 0;
    const uint32_t w = ri.width;
    const uint32_t az_prev = (az + w - 1) % w;
    const uint32_t az_next = (az + 1) % w;

    for (int dr = -1; dr <= 1; ++dr) {
        const int r = static_cast<int>(ring) + dr;
        if (r < 0 || r >= static_cast<int>(ri.height)) {
            continue;
        }
        const uint32_t rv = static_cast<uint32_t>(r);
        if (dr == 0) {
            out[n++] = {az_prev, rv};
            out[n++] = {az_next, rv};
        } else {
            out[n++] = {az_prev, rv};
            out[n++] = {az, rv};
            out[n++] = {az_next, rv};
        }
    }
    return n;
}

RangeImage azimuth_median_filter(const RangeImage& ri, uint32_t half_window) {
    RangeImage out = ri;
    if (half_window == 0 || ri.width == 0) {
        return out;
    }

    std::vector<float> window;
    for (uint32_t az = 0; az < ri.width; ++az) {
        for (uint32_t ring = 0; ring < ri.height; ++ring) {
            window.clear();
            for (uint32_t d = 0; d <= half_window; ++d) {
                const uint32_t a1 = (az + d) % ri.width;
                const uint32_t a2 = (az + ri.width - d) % ri.width;
                for (uint32_t a : {a1, a2}) {
                    const float v = ri.range[ri.idx(a, ring)];
                    if (std::isfinite(v)) {
                        window.push_back(v);
                    }
                }
            }
            if (window.empty()) {
                out.range[out.idx(az, ring)] = std::numeric_limits<float>::quiet_NaN();
                continue;
            }
            std::sort(window.begin(), window.end());
            out.range[out.idx(az, ring)] = window[window.size() / 2];
        }
    }
    return out;
}

} // namespace argus
