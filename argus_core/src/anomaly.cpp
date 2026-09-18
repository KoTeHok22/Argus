
#include "argus_core/anomaly.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "argus_core/tunnel_model.hpp"

namespace argus {
namespace {

constexpr uint32_t kNoCloudIndex = std::numeric_limits<uint32_t>::max();

float median_of_sorted(const std::vector<float>& sorted) {
    if (sorted.empty()) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    return sorted[sorted.size() / 2];
}

void sorted_insert(std::vector<float>& v, float x) {
    v.insert(std::upper_bound(v.begin(), v.end(), x), x);
}

void sorted_remove_one(std::vector<float>& v, float x) {
    const auto it = std::lower_bound(v.begin(), v.end(), x);
    if (it != v.end() && *it == x) {
        v.erase(it);
    }
}

uint32_t cyclic_az_diff(uint32_t a, uint32_t b, uint32_t width) {
    const uint32_t d = (a + width - b) % width;
    return std::min(d, width - d);
}

std::vector<uint32_t> build_cell_to_cloud(const CleanCloud& cloud, size_t n_cells) {
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

} // namespace

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
    const uint32_t half = std::min(p_.median_half_window, (w - 1) / 2);

    std::vector<float> baseline(n_cells, std::numeric_limits<float>::quiet_NaN());
    std::vector<float> win;
    win.reserve(2 * static_cast<size_t>(half) + 1);
    for (uint32_t r = 0; r < h; ++r) {
        win.clear();
        for (uint32_t d = 0; d <= 2 * half; ++d) {
            const float v = ri.range[ri.idx(d % w, r)];
            if (std::isfinite(v)) {
                win.push_back(v);
            }
        }
        std::sort(win.begin(), win.end());
        for (uint32_t az = half; az < w + half; ++az) {
            const uint32_t c = az % w;
            baseline[ri.idx(c, r)] = median_of_sorted(win);
            const float out_v = ri.range[ri.idx((c + w - half) % w, r)];
            const float in_v = ri.range[ri.idx((c + half + 1) % w, r)];
            if (std::isfinite(out_v)) sorted_remove_one(win, out_v);
            if (std::isfinite(in_v)) sorted_insert(win, in_v);
        }
    }

    std::vector<uint8_t> hit(n_cells, 0);
    std::vector<float> resid(n_cells, 0.0f);
    for (size_t i = 0; i < n_cells; ++i) {
        const float v = ri.range[i];
        const float b = baseline[i];
        if (!std::isfinite(v) || !std::isfinite(b) || v < p_.min_range) {
            continue;
        }
        const float d = b - v;
        if (d > p_.residual_threshold_m) {
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
        if (cells < p_.min_cells) {
            continue;
        }

        uint32_t span = 0;
        for (uint32_t cidx : cand.cells) {
            const uint32_t az = cidx / h;
            span = std::max(span, (az + w - az_anchor) % w);
        }
        const float fill =
            static_cast<float>(cells) / static_cast<float>((span + 1) * (r1 - r0 + 1));
        if (fill < p_.min_fill) {
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

FreeSpaceDetector::FreeSpaceDetector(TunnelModel* model, const FreeSpaceParams& p)
    : model_(model), p_(p) {}

AnomalySet FreeSpaceDetector::detect(const CleanCloud& cloud, const RangeImage& ri) {
    return detect(cloud, ri, Eigen::Isometry3d::Identity());
}

AnomalySet FreeSpaceDetector::detect(const CleanCloud& cloud, const RangeImage& ri,
                                     const Eigen::Isometry3d& pose) {
    AnomalySet out;
    out.source = name();
    if (!p_.enabled || model_ == nullptr || !model_->ready() || !ri.valid()) {
        return out;
    }

    const Eigen::Vector3f origin = (pose.inverse() * Eigen::Vector3d::Zero()).cast<float>();

    for (size_t i = 0; i < cloud.size(); ++i) {
        if (!cloud.ground_mask.empty() && i < cloud.ground_mask.size() &&
            cloud.ground_mask[i] != 0) {
            continue;
        }
        const Eigen::Vector3f lidar(cloud.x[i], cloud.y[i], cloud.z[i]);
        const float range = (lidar - origin).norm();
        if (!std::isfinite(range) || range < p_.min_range || range > p_.max_range) {
            continue;
        }
        float confidence = 0.0f;
        if (!model_->contradicts_free_space(lidar, confidence, pose, p_.min_confidence,
                                            p_.min_free_observations)) {
            continue;
        }
        out.indices.push_back(static_cast<uint32_t>(i));
        out.score.push_back(confidence);
    }
    return out;
}

} // namespace argus
