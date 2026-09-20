
#include <algorithm>
#include <cmath>
#include <vector>

#include "anomaly_common.hpp"
#include "argus_core/anomaly.hpp"

namespace argus {

using anomaly_detail::median_of_sorted;

NoReturnDetector::NoReturnDetector(const NoReturnParams& p) : p_(p) {}

AnomalySet NoReturnDetector::detect(const CleanCloud&, const RangeImage& ri) {
    AnomalySet out;
    out.source = name();
    if (!ri.valid() || p_.azimuth_window == 0) {
        return out;
    }

    const uint32_t w = ri.width;
    const uint32_t h = ri.height;
    const int aw = static_cast<int>(p_.azimuth_window);
    const uint32_t max_run = std::max<uint32_t>(
        p_.min_missing_run, static_cast<uint32_t>(p_.max_missing_run_deg * w / 360.0f) + 1);

    for (uint32_t r = 0; r < h; ++r) {
        uint32_t anchor = w;
        for (uint32_t az = 0; az < w; ++az) {
            if (ri.no_return_mask[ri.idx(az, r)] == 0) {
                anchor = az;
                break;
            }
        }
        if (anchor == w) {
            continue;
        }

        uint32_t run_start = 0;
        bool in_run = false;
        for (uint32_t step = 0; step <= w; ++step) {
            const uint32_t az = (anchor + step) % w;
            const bool missing = ri.no_return_mask[ri.idx(az, r)] != 0 && step < w;
            if (missing && !in_run) {
                run_start = az;
                in_run = true;
            } else if (!missing && in_run) {
                in_run = false;
                const uint32_t run_len = (az + w - run_start) % w;
                if (run_len < p_.min_missing_run || run_len > max_run) {
                    continue;
                }
                std::vector<float> base;
                base.reserve(2 * static_cast<size_t>(aw));
                for (int d = 1; d <= aw; ++d) {
                    const uint32_t a1 = (run_start + w - static_cast<uint32_t>(d)) % w;
                    const uint32_t a2 = (az + w - 1 + static_cast<uint32_t>(d)) % w;
                    const float v1 = ri.range[ri.idx(a1, r)];
                    const float v2 = ri.range[ri.idx(a2, r)];
                    if (std::isfinite(v1)) base.push_back(v1);
                    if (std::isfinite(v2)) base.push_back(v2);
                }
                const size_t total = 2 * static_cast<size_t>(aw);
                const float frac = base.size() / static_cast<float>(total);
                std::sort(base.begin(), base.end());
                const float base_med = median_of_sorted(base);
                if (frac >= p_.min_baseline_hits && base_med >= p_.min_range) {
                    for (uint32_t k = 0; k < run_len; ++k) {
                        const uint32_t a = (run_start + k) % w;
                        out.cells.push_back(static_cast<uint32_t>(ri.idx(a, r)));
                        out.cells_score.push_back(std::min(1.0f, frac));
                    }
                }
            }
        }
    }
    return out;
}

} // namespace argus
