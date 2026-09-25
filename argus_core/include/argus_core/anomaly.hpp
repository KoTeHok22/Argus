
#pragma once

#include <memory>
#include <string>

#include <Eigen/Geometry>

#include "argus_core/range_image.hpp"
#include "argus_core/types.hpp"

namespace argus {

class IAnomalyDetector {
public:
    virtual ~IAnomalyDetector() = default;

    virtual AnomalySet detect(const CleanCloud& cloud, const RangeImage& ri) = 0;

    virtual void update(const CleanCloud& cloud, const RangeImage& ri) {
        (void)cloud;
        (void)ri;
    }

    virtual std::string name() const = 0;
};

struct NoReturnParams {
    uint32_t min_missing_run = 8;
    float max_missing_run_deg = 15.0f;
    uint32_t azimuth_window = 5;
    float min_baseline_hits = 0.70f;
    float min_range = 3.0f;
};

class NoReturnDetector : public IAnomalyDetector {
public:
    explicit NoReturnDetector(const NoReturnParams& p);
    AnomalySet detect(const CleanCloud& cloud, const RangeImage& ri) override;
    std::string name() const override { return "no_return"; }

private:
    NoReturnParams p_;
};

struct GeometryResidualParams {
    uint32_t median_half_window = 100;
    float median_window_override_min_range_m = 18.5f;
    float median_window_override_max_range_m = 40.0f;
    uint32_t median_half_window_override = 200;
    float residual_threshold_m = 1.0f;
    float residual_threshold_far_m = 1.0f;
    uint32_t min_cells = 50;
    float min_fill = 0.3f;
    float min_range = 4.0f;
    float max_target_range_m = 45.0f;
    float far_range_m = 60.0f;
    uint32_t min_cells_far = 8;
    float min_fill_far = 0.12f;
    uint32_t min_stable_frames = 2;
    uint32_t az_tolerance = 20;
    float range_tolerance_m = 3.0f;
};

class GeometryResidualDetector : public IAnomalyDetector {
public:
    explicit GeometryResidualDetector(const GeometryResidualParams& p);
    AnomalySet detect(const CleanCloud& cloud, const RangeImage& ri) override;
    std::string name() const override { return "geometry"; }

private:
    struct Track {
        float az_center = 0.0f;
        float nearest = 0.0f;
        uint32_t streak = 0;
    };

    GeometryResidualParams p_;
    std::vector<Track> tracks_;
};

struct FreeSpaceParams {
    bool enabled = true;
    float min_confidence = 0.70f;
    float min_range = 1.0f;
    float max_range = 150.0f;
    uint32_t min_free_observations = 5;
    bool require_multiple_observations = true;
};

class TunnelModel;

class FreeSpaceDetector : public IAnomalyDetector {
public:
    FreeSpaceDetector(TunnelModel* model, const FreeSpaceParams& p);
    AnomalySet detect(const CleanCloud& cloud, const RangeImage& ri) override;
    AnomalySet detect(const CleanCloud& cloud, const RangeImage& ri, const Eigen::Isometry3d& pose);
    std::string name() const override { return "free_space"; }

private:
    TunnelModel* model_;
    FreeSpaceParams p_;
};

} // namespace argus
