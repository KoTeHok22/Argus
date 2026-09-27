
#include "argus_core/fusion.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <utility>

namespace argus {

FusionParams default_fusion_params() {
    return FusionParams{};
}

FusionPipeline::FusionPipeline(ClearanceGauge gauge, const FusionParams& params)
    : gauge_(std::move(gauge)), p_(params), tracker_(params.tracking), model_(params.model) {}

FusionResult FusionPipeline::update(const CleanCloud& cloud, const RangeImage& ri,
                                    const AnomalySet& geometry, const AnomalySet& no_return,
                                    float dt, float train_speed_mps) {
    return update(cloud, ri, geometry, no_return, AnomalySet{}, dt, train_speed_mps,
                  Eigen::Isometry3d::Identity(), false, false);
}

FusionResult FusionPipeline::update(const CleanCloud& cloud, const RangeImage& ri,
                                    const AnomalySet& geometry, const AnomalySet& no_return,
                                    float dt, float train_speed_mps,
                                    const Eigen::Isometry3d& pose) {
    return update(cloud, ri, geometry, no_return, AnomalySet{}, dt, train_speed_mps, pose, false);
}

FusionResult FusionPipeline::update(const CleanCloud& cloud, const RangeImage& ri,
                                    const AnomalySet& geometry, const AnomalySet& no_return,
                                    const AnomalySet& free_space, float dt, float train_speed_mps,
                                    const Eigen::Isometry3d& pose) {
    return update(cloud, ri, geometry, no_return, free_space, dt, train_speed_mps, pose, true,
                  true);
}

FusionResult FusionPipeline::update(const CleanCloud& cloud, const RangeImage& ri,
                                    const AnomalySet& geometry, const AnomalySet& no_return,
                                    const AnomalySet& free_space, float dt, float train_speed_mps,
                                    const Eigen::Isometry3d& pose, bool caller_drives_model) {
    return update(cloud, ri, geometry, no_return, free_space, dt, train_speed_mps, pose,
                  caller_drives_model, true);
}

FusionResult FusionPipeline::update(const CleanCloud& cloud, const RangeImage& ri,
                                    const AnomalySet& geometry, const AnomalySet& no_return,
                                    const AnomalySet& free_space, float dt, float train_speed_mps,
                                    const Eigen::Isometry3d& pose, bool caller_drives_model,
                                    bool pose_valid) {
    return update(cloud, ri, geometry, no_return, AnomalySet{}, free_space, dt, train_speed_mps,
                  pose, caller_drives_model, pose_valid);
}

FusionResult FusionPipeline::update(const CleanCloud& cloud, const RangeImage& ri,
                                    const AnomalySet& geometry, const AnomalySet& no_return,
                                    const AnomalySet& temporal, const AnomalySet& free_space,
                                    float dt, float train_speed_mps, const Eigen::Isometry3d& pose,
                                    bool caller_drives_model, bool pose_valid) {
    FusionResult out;
    out.n_anom_points = static_cast<uint32_t>(geometry.indices.size());
    out.n_anom_cells = static_cast<uint32_t>(no_return.cells.size());
    out.pose_used = pose_valid;

    AnomalySet merged;
    merged.indices.reserve(geometry.indices.size() + temporal.indices.size());
    const auto append_source = [&](const AnomalySet& source) {
        for (uint32_t idx : source.indices) {
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
    };
    append_source(geometry);
    if (p_.use_temporal_candidates) {
        append_source(temporal);
    }

    const bool prior_ready =
        pose_valid && model_.ready() && model_.frames_integrated() >= p_.free_space_warmup_frames;
    if (p_.use_free_space && prior_ready && !free_space.indices.empty()) {
        std::vector<uint8_t> already(cloud.size(), 0);
        for (uint32_t idx : merged.indices) {
            already[idx] = 1;
        }
        for (uint32_t idx : free_space.indices) {
            if (idx >= cloud.size() || already[idx] != 0) {
                continue;
            }
            merged.indices.push_back(idx);
        }
    }

    const std::vector<Cluster> all = cluster_anomalies(cloud, ri, merged, p_.clustering);
    std::unordered_set<uint32_t> temporal_indices;
    if (p_.use_temporal_candidates) {
        for (uint32_t idx : temporal.indices) {
            temporal_indices.insert(idx);
        }
    }

    std::vector<float> cloud_score(cloud.size(), 0.0f);
    for (size_t i = 0; i < geometry.indices.size() && i < geometry.score.size(); ++i) {
        if (geometry.indices[i] < cloud.size()) {
            cloud_score[geometry.indices[i]] = geometry.score[i];
        }
    }
    for (size_t i = 0; i < temporal.indices.size() && i < temporal.score.size(); ++i) {
        if (temporal.indices[i] < cloud.size()) {
            cloud_score[temporal.indices[i]] =
                std::max(cloud_score[temporal.indices[i]], temporal.score[i]);
        }
    }

    std::vector<uint32_t> nr_cells = no_return.cells;
    std::sort(nr_cells.begin(), nr_cells.end());

    std::vector<uint8_t> free_cell(cloud.size(), 0);
    if (p_.use_free_space && prior_ready) {
        for (uint32_t idx : free_space.indices) {
            if (idx < free_cell.size()) {
                free_cell[idx] = 1;
            }
        }
    }

    out.n_clusters_raw = static_cast<uint32_t>(all.size());
    for (Cluster c : all) {
        const bool has_geometry =
            std::any_of(c.indices.begin(), c.indices.end(), [&](uint32_t idx) {
                return idx < cloud.size() &&
                       std::find(geometry.indices.begin(), geometry.indices.end(), idx) !=
                           geometry.indices.end();
            });
        const bool has_temporal =
            std::any_of(c.indices.begin(), c.indices.end(),
                        [&](uint32_t idx) { return temporal_indices.count(idx) != 0; });
        c.votes_geometry = has_geometry ? 1 : 0;
        c.votes_temporal = has_temporal ? 1 : 0;
        const float max_extent = (c.max_corner - c.min_corner).maxCoeff();
        if (has_temporal && !has_geometry &&
            (c.point_count < p_.temporal_min_cluster_size ||
             max_extent < p_.temporal_min_extent_m || max_extent > p_.temporal_max_extent_m)) {
            ++out.n_filtered_out;
            continue;
        }
        uint32_t free_space_points = 0;
        float fwd_min = -1.0f;
        bool in_gauge = false;
        double score_sum = 0.0;
        uint32_t score_n = 0;
        for (uint32_t idx : c.indices) {
            const float x = cloud.x[idx], y = cloud.y[idx], z = cloud.z[idx];
            score_sum += cloud_score[idx];
            ++score_n;
            if (free_cell[idx] != 0) {
                ++free_space_points;
            }
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
        const uint32_t free_space_cells = free_space_points;
        c.score = score_n > 0 ? static_cast<float>(score_sum / score_n) : 0.0f;
        out.n_free_space_points += free_space_points;
        if (free_space_points >= p_.free_space_min_points &&
            free_space_cells >= p_.free_space_min_cells) {
            c.votes_free_space = 1;
        }
        if (!in_gauge) {
            ++out.n_filtered_out;
            continue;
        }
        c.forward_distance = fwd_min;
        out.clusters.push_back(std::move(c));
    }

    out.tracks = tracker_.update(out.clusters, dt, train_speed_mps);
    for (const Cluster& cluster : out.clusters) {
        const uint32_t votes = static_cast<uint32_t>(cluster.votes_free_space) +
                               static_cast<uint32_t>(cluster.votes_no_return) +
                               static_cast<uint32_t>(cluster.votes_geometry) +
                               static_cast<uint32_t>(cluster.votes_temporal);
        if (votes < p_.min_votes_for_alert) {
            continue;
        }
        const auto confirmed =
            std::any_of(out.tracks.begin(), out.tracks.end(), [&](const Track& track) {
                return track.confirmed && (track.state.head<3>() - cluster.centroid).norm() <
                                              p_.tracking.max_association_distance;
            });
        if (!confirmed) out.candidates.push_back(cluster);
    }

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
                                   static_cast<uint32_t>(t.votes_geometry) +
                                   static_cast<uint32_t>(t.votes_temporal);
            const bool stale_far_track = t.nearest_range >= p_.strict_track_freshness_range_m &&
                                         t.consecutive_misses > p_.max_confirmed_track_misses;
            if (t.confirmed && !stale_far_track && votes >= p_.min_votes_for_alert) {
                out.alert = true;
                break;
            }
        }
    } else {
        for (const Cluster& c : out.clusters) {
            const uint32_t votes = static_cast<uint32_t>(c.votes_free_space) +
                                   static_cast<uint32_t>(c.votes_no_return) +
                                   static_cast<uint32_t>(c.votes_geometry) +
                                   static_cast<uint32_t>(c.votes_temporal);
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

    if (pose_valid && p_.update_model_with_clean_frames &&
        (p_.use_free_space || caller_drives_model)) {
        const bool clean_frame = geometry.indices.empty();
        model_.integrate(cloud, pose, clean_frame);
        out.model_updated = true;
    }
    out.model_ready = model_.ready();
    if (pose_valid && p_.compute_free_space_violation_rate) {
        const FreeSpaceCheck fs = model_.check(cloud, pose, p_.free_space_min_confidence);
        out.free_space_violation_rate = fs.violation_rate();
    }
    return out;
}

} // namespace argus
