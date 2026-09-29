
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "anomaly_common.hpp"
#include "argus_core/anomaly.hpp"
#include "argus_core/range_image.hpp"

namespace argus {

using anomaly_detail::build_cell_to_cloud;
using anomaly_detail::cyclic_az_diff;
using anomaly_detail::kNoCloudIndex;
using anomaly_detail::median_of_sorted;
using anomaly_detail::sorted_insert;
using anomaly_detail::sorted_remove_one;

GeometryResidualDetector::GeometryResidualDetector(const GeometryResidualParams& p) : p_(p) {}

AnomalySet GeometryResidualDetector::detect(const CleanCloud& cloud, const RangeImage& ri) {
    AnomalySet out;
    out.source = name();
    if (!ri.valid() || ri.width < 3 || p_.median_half_window == 0) {
        tracks_.clear();
        return out;
    }

    const uint32_t w = ri.width;
    const uint32_t h = ri.height;
    const size_t n_cells = static_cast<size_t>(w) * h;
    const auto build_baseline = [&](uint32_t configured_half) {
        const uint32_t half = std::min(configured_half, (w - 1) / 2);
        std::vector<float> values(n_cells, std::numeric_limits<float>::quiet_NaN());
        std::vector<float> win;
        win.reserve(2 * static_cast<size_t>(half) + 1);
        for (uint32_t r = 0; r < h; ++r) {
            win.clear();
            for (uint32_t d = 0; d <= 2 * half; ++d) {
                const float value = ri.range[ri.idx(d % w, r)];
                if (std::isfinite(value)) {
                    win.push_back(value);
                }
            }
            std::sort(win.begin(), win.end());
            for (uint32_t az = half; az < w + half; ++az) {
                const uint32_t c = az % w;
                values[ri.idx(c, r)] = median_of_sorted(win);
                const float out_v = ri.range[ri.idx((c + w - half) % w, r)];
                const float in_v = ri.range[ri.idx((c + half + 1) % w, r)];
                if (std::isfinite(out_v)) sorted_remove_one(win, out_v);
                if (std::isfinite(in_v)) sorted_insert(win, in_v);
            }
        }
        return values;
    };
    const auto baseline = build_baseline(p_.median_half_window);
    const bool has_override = p_.median_half_window_override != p_.median_half_window;
    const auto override_baseline =
        has_override ? build_baseline(p_.median_half_window_override) : baseline;
    std::vector<float> selected_baseline(n_cells);
    for (size_t i = 0; i < n_cells; ++i) {
        const float range = ri.range[i];
        const bool use_override = has_override && std::isfinite(range) &&
                                  range >= p_.median_window_override_min_range_m &&
                                  range <= p_.median_window_override_max_range_m;
        selected_baseline[i] = use_override ? override_baseline[i] : baseline[i];
    }

    std::vector<uint8_t> hit(n_cells, 0);
    std::vector<float> resid(n_cells, 0.0f);
    for (size_t i = 0; i < n_cells; ++i) {
        const float v = ri.range[i];
        const float b = selected_baseline[i];
        if (!std::isfinite(v) || !std::isfinite(b) || v < p_.min_range) {
            continue;
        }
        const float d = b - v;
        const float residual_threshold =
            v >= p_.far_range_m ? p_.residual_threshold_far_m : p_.residual_threshold_m;
        if (d > residual_threshold) {
            hit[i] = 1;
            resid[i] = d;
        }
    }

    std::vector<uint8_t> visited(n_cells, 0);
    std::vector<size_t> stack;
    struct Candidate {
        std::vector<uint32_t> cells;
        float az_center = 0.0f;
        float nearest = 0.0f;
    };
    std::vector<Candidate> candidates;

    for (size_t s = 0; s < n_cells; ++s) {
        if (!hit[s] || visited[s]) {
            continue;
        }

        Candidate cand;
        float nearest = std::numeric_limits<float>::max();
        uint32_t r0 = h, r1 = 0;
        uint32_t az_anchor = w;
        stack.clear();
        stack.push_back(s);
        visited[s] = 1;

        while (!stack.empty()) {
            const size_t cur = stack.back();
            stack.pop_back();
            const uint32_t az = static_cast<uint32_t>(cur) / h;
            const uint32_t ring = static_cast<uint32_t>(cur) % h;
            cand.cells.push_back(static_cast<uint32_t>(cur));
            const float v = ri.range[cur];
            if (v < nearest) nearest = v;
            r0 = std::min(r0, ring);
            r1 = std::max(r1, ring);
            az_anchor = std::min(az_anchor, az);

            std::array<std::pair<uint32_t, uint32_t>, 8> nb{};
            const uint32_t nn = neighbors8(ri, az, ring, nb);
            for (uint32_t k = 0; k < nn; ++k) {
                const size_t nidx = ri.idx(nb[k].first, nb[k].second);
                if (hit[nidx] && !visited[nidx]) {
                    visited[nidx] = 1;
                    stack.push_back(nidx);
                }
            }
        }

        const size_t cells = cand.cells.size();
        const bool far_candidate = nearest >= p_.far_range_m;
        const uint32_t min_cells = far_candidate ? p_.min_cells_far : p_.min_cells;
        const float min_fill = far_candidate ? p_.min_fill_far : p_.min_fill;
        if (cells < min_cells) {
            continue;
        }

        uint32_t span = 0;
        for (uint32_t cidx : cand.cells) {
            const uint32_t az = cidx / h;
            span = std::max(span, (az + w - az_anchor) % w);
        }
        const float fill =
            static_cast<float>(cells) / static_cast<float>((span + 1) * (r1 - r0 + 1));
        if (fill < min_fill) {
            continue;
        }
        if (nearest < p_.min_range || nearest > p_.max_target_range_m) {
            continue;
        }

        double wsum = 0.0, asum = 0.0;
        for (uint32_t cidx : cand.cells) {
            const float v = ri.range[cidx];
            if (v <= nearest + p_.range_tolerance_m) {
                const double wt = 1.0 / (static_cast<double>(v) * v);
                wsum += wt;
                asum += wt * (cidx / h);
            }
        }
        cand.az_center = wsum > 0.0 ? static_cast<float>(asum / wsum) : 0.0f;
        cand.nearest = nearest;
        candidates.push_back(std::move(cand));
    }

    if (p_.min_stable_frames <= 1) {
        tracks_.clear();
    } else {
        std::vector<Track> next_tracks;
        for (const Candidate& cand : candidates) {
            const uint32_t az_c = static_cast<uint32_t>(cand.az_center) % w;
            int best = -1;
            float best_rng_diff = std::numeric_limits<float>::max();
            for (size_t t = 0; t < tracks_.size(); ++t) {
                const Track& tr = tracks_[t];
                const float rng_diff = std::fabs(tr.nearest - cand.nearest);
                if (rng_diff > p_.range_tolerance_m) {
                    continue;
                }
                if (cyclic_az_diff(az_c, static_cast<uint32_t>(tr.az_center) % w, w) >
                    p_.az_tolerance) {
                    continue;
                }
                if (rng_diff < best_rng_diff) {
                    best_rng_diff = rng_diff;
                    best = static_cast<int>(t);
                }
            }
            if (best >= 0) {
                Track tr = tracks_[best];
                tr.az_center = cand.az_center;
                tr.nearest = cand.nearest;
                tr.streak++;
                next_tracks.push_back(tr);
                tracks_.erase(tracks_.begin() + best);
            } else {
                Track tr;
                tr.az_center = cand.az_center;
                tr.nearest = cand.nearest;
                tr.streak = 1;
                next_tracks.push_back(tr);
            }
        }
        const bool enough =
            std::any_of(next_tracks.begin(), next_tracks.end(),
                        [this](const Track& t) { return t.streak >= p_.min_stable_frames; });
        if (!enough) {
            tracks_ = std::move(next_tracks);
            return out;
        }
        std::vector<Candidate> stable;
        for (const Track& t : next_tracks) {
            if (t.streak < p_.min_stable_frames) {
                continue;
            }
            for (const Candidate& cand : candidates) {
                const uint32_t az_c = static_cast<uint32_t>(cand.az_center) % w;
                if (cyclic_az_diff(az_c, static_cast<uint32_t>(t.az_center) % w, w) <=
                        p_.az_tolerance &&
                    std::fabs(t.nearest - cand.nearest) <= p_.range_tolerance_m) {
                    stable.push_back(cand);
                    break;
                }
            }
        }
        candidates = std::move(stable);
        tracks_ = std::move(next_tracks);
    }

    const std::vector<uint32_t> cell_to_cloud = build_cell_to_cloud(cloud, n_cells);
    for (const Candidate& cand : candidates) {
        for (uint32_t cidx : cand.cells) {
            const uint32_t ci = cell_to_cloud[cidx];
            if (ci == kNoCloudIndex) {
                continue;
            }
            out.indices.push_back(ci);
            out.score.push_back(std::min(1.0f, resid[cidx] / (2.0f * p_.residual_threshold_m)));
        }
    }
    return out;
}

} // namespace argus
