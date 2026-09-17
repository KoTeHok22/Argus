
#pragma once

#include <cstdint>
#include <vector>

#include "argus_core/clustering.hpp"
#include "argus_core/gauge.hpp"
#include "argus_core/range_image.hpp"
#include "argus_core/tracking.hpp"
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
    ClusteringParams clustering;
    TrackingParams tracking;
};

struct FusionResult {
    std::vector<Cluster> clusters;
    std::vector<Track> tracks;
    uint32_t n_anom_points = 0;
    uint32_t n_anom_cells = 0;
    uint32_t n_clusters_raw = 0;
    uint32_t n_filtered_out = 0;
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

    const ClearanceGauge& gauge() const { return gauge_; }

private:
    ClearanceGauge gauge_;
    FusionParams p_;
    ObstacleTracker tracker_;
};

} // namespace argus
