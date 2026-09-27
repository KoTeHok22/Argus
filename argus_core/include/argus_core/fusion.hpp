
#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Geometry>

#include "argus_core/clustering.hpp"
#include "argus_core/gauge.hpp"
#include "argus_core/range_image.hpp"
#include "argus_core/tracking.hpp"
#include "argus_core/tunnel_model.hpp"
#include "argus_core/types.hpp"

namespace argus {

struct FusionParams {
    bool require_confirmed_track = true;
    float critical_range_m = 50.0f;
    float warning_range_m = 150.0f;
    float critical_ttc_s = 3.0f;
    uint32_t min_votes_for_alert = 1;
    bool ground_filter = true;
    float ground_clearance_m = 0.45f;
    float rail_zone_m = 2.0f;
    bool use_free_space = false;
    float free_space_min_confidence = 0.85f;
    float free_space_min_range = 3.0f;
    float free_space_max_range = 90.0f;
    uint32_t free_space_min_points = 30;
    uint32_t free_space_min_cells = 30;
    uint32_t free_space_min_observations = 10;
    uint32_t free_space_warmup_frames = 60;
    uint32_t max_confirmed_track_misses = 4;
    float strict_track_freshness_range_m = 90.0f;
    bool update_model_with_clean_frames = true;
    bool compute_free_space_violation_rate = false;
    TunnelModelParams model;
    ClusteringParams clustering;
    TrackingParams tracking;
};

FusionParams default_fusion_params();

struct FusionResult {
    std::vector<Cluster> clusters;
    std::vector<Cluster> candidates;
    std::vector<Track> tracks;
    uint32_t n_anom_points = 0;
    uint32_t n_anom_cells = 0;
    uint32_t n_clusters_raw = 0;
    uint32_t n_filtered_out = 0;
    uint32_t n_free_space_points = 0;
    float free_space_violation_rate = 0.0f;
    bool model_ready = false;
    bool model_updated = false;
    bool pose_used = false;
    bool alert = false;
    float nearest_forward_m = -1.0f;
    float nearest_range_m = -1.0f;
    float ttc_s = -1.0f;
};

class FusionPipeline {
public:
    FusionPipeline(ClearanceGauge gauge, const FusionParams& params);

    FusionResult update(const CleanCloud& cloud, const RangeImage& ri, const AnomalySet& geometry,
                        const AnomalySet& no_return, float dt, float train_speed_mps);

    FusionResult update(const CleanCloud& cloud, const RangeImage& ri, const AnomalySet& geometry,
                        const AnomalySet& no_return, float dt, float train_speed_mps,
                        const Eigen::Isometry3d& pose);
    FusionResult update(const CleanCloud& cloud, const RangeImage& ri, const AnomalySet& geometry,
                        const AnomalySet& no_return, const AnomalySet& free_space, float dt,
                        float train_speed_mps, const Eigen::Isometry3d& pose);

    FusionResult update(const CleanCloud& cloud, const RangeImage& ri, const AnomalySet& geometry,
                        const AnomalySet& no_return, const AnomalySet& free_space, float dt,
                        float train_speed_mps, const Eigen::Isometry3d& pose,
                        bool caller_drives_model);

    FusionResult update(const CleanCloud& cloud, const RangeImage& ri, const AnomalySet& geometry,
                        const AnomalySet& no_return, const AnomalySet& free_space, float dt,
                        float train_speed_mps, const Eigen::Isometry3d& pose,
                        bool caller_drives_model, bool pose_valid);

    const ClearanceGauge& gauge() const { return gauge_; }

    const TunnelModel& model() const { return model_; }

private:
    ClearanceGauge gauge_;
    FusionParams p_;
    ObstacleTracker tracker_;
    TunnelModel model_;
};

} // namespace argus
