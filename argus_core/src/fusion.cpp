
#include "argus_core/fusion.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace argus {

FusionPipeline::FusionPipeline(ClearanceGauge gauge, const FusionParams& params)
    : gauge_(std::move(gauge)), p_(params), tracker_(params.tracking) {}

FusionResult FusionPipeline::update(const CleanCloud& cloud, const RangeImage& ri,
                                    const AnomalySet& geometry, const AnomalySet& no_return,
                                    float dt, float train_speed_mps) {
    FusionResult out;
    out.n_anom_points = static_cast<uint32_t>(geometry.indices.size());
    out.n_anom_cells = static_cast<uint32_t>(no_return.cells.size());

    AnomalySet merged;
    merged.indices.reserve(geometry.indices.size());
    for (uint32_t idx : geometry.indices) {
        if (idx >= cloud.size()) {
            continue;
        }
        if (!cloud.ground_mask.empty() && idx < cloud.ground_mask.size() &&
            cloud.ground_mask[idx] != 0) {
            continue;
        }
        if (p_.ground_filter) {
            const float z_rel = cloud.z[idx] + gauge_.params().sensor_height;
            const float lat =
                std::fabs(cloud.x[idx] * gauge_.left_x() + cloud.y[idx] * gauge_.left_y());
            if (z_rel < p_.ground_clearance_m && lat < p_.rail_zone_m) {
                continue;
            }
        }
        merged.indices.push_back(idx);
    }
    const std::vector<Cluster> all = cluster_anomalies(cloud, ri, merged, p_.clustering);

    std::vector<float> cloud_score(cloud.size(), 0.0f);
    for (size_t i = 0; i < geometry.indices.size() && i < geometry.score.size(); ++i) {
        if (geometry.indices[i] < cloud.size()) {
            cloud_score[geometry.indices[i]] = geometry.score[i];
        }
    }

    std::vector<uint32_t> nr_cells = no_return.cells;
    std::sort(nr_cells.begin(), nr_cells.end());

    out.n_clusters_raw = static_cast<uint32_t>(all.size());
    for (Cluster c : all) {
        c.votes_geometry = 1;
        float fwd_min = -1.0f;
        bool in_gauge = false;
        double score_sum = 0.0;
        uint32_t score_n = 0;
        for (uint32_t idx : c.indices) {
            const float x = cloud.x[idx], y = cloud.y[idx], z = cloud.z[idx];
            score_sum += cloud_score[idx];
            ++score_n;
            if (!gauge_.contains(x, y, z)) {
                continue;
            }
            in_gauge = true;
            const float fwd = gauge_.forward_distance(x, y, z);
            if (fwd_min < 0.0f || fwd < fwd_min) {
                fwd_min = fwd;
            }
            const uint32_t src = cloud.raw_idx.empty() ? idx : cloud.raw_idx[idx];
            if (std::binary_search(nr_cells.begin(), nr_cells.end(), src)) {
                c.votes_no_return = 1;
            }
        }
        c.score = score_n > 0 ? static_cast<float>(score_sum / score_n) : 0.0f;
        if (!in_gauge) {
            ++out.n_filtered_out;
            continue;
        }
        c.forward_distance = fwd_min;
        out.clusters.push_back(std::move(c));
    }

    out.tracks = tracker_.update(out.clusters, dt, train_speed_mps);

    for (const Track& t : out.tracks) {
        out.nearest_range_m = out.nearest_range_m < 0.0f
                                  ? t.nearest_range
                                  : std::min(out.nearest_range_m, t.nearest_range);
        const float ttc = compute_ttc(t, train_speed_mps);
        if (out.ttc_s < 0.0f || (ttc >= 0.0f && ttc < out.ttc_s)) {
            out.ttc_s = ttc;
        }
    }
    for (const Cluster& c : out.clusters) {
        if (c.forward_distance < 0.0f) {
            continue;
        }
        out.nearest_forward_m = out.nearest_forward_m < 0.0f
                                    ? c.forward_distance
                                    : std::min(out.nearest_forward_m, c.forward_distance);
    }
    if (p_.require_confirmed_track) {
        for (const Track& t : out.tracks) {
            const uint32_t votes = static_cast<uint32_t>(t.votes_free_space) +
                                   static_cast<uint32_t>(t.votes_no_return) +
                                   static_cast<uint32_t>(t.votes_geometry);
            if (t.confirmed && votes >= p_.min_votes_for_alert) {
                out.alert = true;
                break;
            }
        }
    } else {
        for (const Cluster& c : out.clusters) {
            const uint32_t votes = static_cast<uint32_t>(c.votes_free_space) +
                                   static_cast<uint32_t>(c.votes_no_return) +
                                   static_cast<uint32_t>(c.votes_geometry);
            if (votes >= p_.min_votes_for_alert) {
                out.alert = true;
                break;
            }
        }
    }
    if (!out.alert) {
        out.nearest_forward_m = -1.0f;
        out.nearest_range_m = -1.0f;
        out.ttc_s = -1.0f;
    }
    return out;
}

} // namespace argus
