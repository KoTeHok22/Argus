#include "argus_core/ground.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

#include <patchwork/patchworkpp.h>
#include <Eigen/Dense>

namespace argus {

namespace {

patchwork::Params make_pp_params(const GroundParams& p) {
    patchwork::Params out;
    out.verbose = false;
    out.enable_RNR = p.enable_RNR;
    out.enable_RVPF = true;
    out.enable_TGR = p.enable_TGR;
    out.sensor_height = static_cast<double>(p.sensor_height);
    out.min_range = static_cast<double>(p.min_range);
    out.max_range = static_cast<double>(p.max_range);
    if (p.num_zones > 0) {
        out.num_zones = p.num_zones;
    }
    if (p.num_sectors > 0) {
        out.num_sectors_each_zone.assign(static_cast<size_t>(std::max(p.num_zones, 4)),
                                         p.num_sectors);
    }
    return out;
}

ClearanceGaugeParams make_gauge_params(const GroundParams& p) {
    ClearanceGaugeParams gp;
    gp.sensor_height = p.sensor_height;
    gp.forward_axis = p.forward_axis;
    return gp;
}

} // namespace

struct GroundSegmenter::Impl {
    GroundParams p;
    ClearanceGauge gauge;
    std::unique_ptr<patchwork::PatchWorkpp> pp;

    Impl(const GroundParams& params, const ClearanceGaugeParams& gp) : p(params), gauge(gp) {
        if (params.enabled && params.method == GroundMethod::PatchworkPp) {
            pp = std::make_unique<patchwork::PatchWorkpp>(make_pp_params(params));
        }
    }
};

GroundSegmenter::GroundSegmenter(const GroundParams& p)
    : impl_(std::make_unique<Impl>(p, make_gauge_params(p))) {}

GroundSegmenter::~GroundSegmenter() = default;
GroundSegmenter::GroundSegmenter(GroundSegmenter&&) noexcept = default;
GroundSegmenter& GroundSegmenter::operator=(GroundSegmenter&&) noexcept = default;

void GroundSegmenter::apply_z_threshold(CleanCloud& cloud, GroundStats& s) {
    const GroundParams& p = impl_->p;
    for (size_t i = 0; i < cloud.size(); ++i) {
        if (cloud.ground_mask[i] != 0) {
            continue;
        }
        const float z_rel = cloud.z[i] + p.sensor_height;
        const float lat =
            std::fabs(cloud.x[i] * impl_->gauge.left_x() + cloud.y[i] * impl_->gauge.left_y());
        if (lat >= p.rail_zone_m) {
            continue;
        }
        if (z_rel >= p.rail_max_height || z_rel <= -0.2f) {
            continue;
        }
        cloud.ground_mask[i] = 1;
        ++s.n_rail_fallback;
    }
}

void GroundSegmenter::apply(CleanCloud& cloud, GroundStats* stats) {
    GroundStats local;
    GroundStats& s = stats != nullptr ? *stats : local;
    s = GroundStats{};
    cloud.ground_mask.assign(cloud.size(), 0);
    if (!impl_->p.enabled || cloud.empty()) {
        s.n_nonground = static_cast<uint32_t>(cloud.size());
        return;
    }

    const GroundParams& p = impl_->p;
    if (p.method == GroundMethod::ZThreshold || impl_->pp == nullptr) {
        apply_z_threshold(cloud, s);
    } else {
        Eigen::MatrixXf mat(static_cast<int>(cloud.size()), 4);
        for (size_t i = 0; i < cloud.size(); ++i) {
            mat(static_cast<int>(i), 0) = cloud.x[i];
            mat(static_cast<int>(i), 1) = cloud.y[i];
            mat(static_cast<int>(i), 2) = cloud.z[i];
            mat(static_cast<int>(i), 3) = cloud.intensity.empty() ? 0.0f : cloud.intensity[i];
        }
        impl_->pp->estimateGround(mat);
        const Eigen::VectorXi gidx = impl_->pp->getGroundIndices();
        for (int k = 0; k < gidx.size(); ++k) {
            const int i = gidx(k);
            if (i < 0 || static_cast<size_t>(i) >= cloud.size()) {
                continue;
            }
            const float z_rel = cloud.z[static_cast<size_t>(i)] + p.sensor_height;
            const float lat = std::fabs(cloud.x[static_cast<size_t>(i)] * impl_->gauge.left_x() +
                                        cloud.y[static_cast<size_t>(i)] * impl_->gauge.left_y());
            if (lat > p.rail_zone_m || z_rel > p.max_ground_z_rel) {
                ++s.n_wall_guarded;
                continue;
            }
            cloud.ground_mask[static_cast<size_t>(i)] = 1;
        }
        apply_z_threshold(cloud, s);
    }

    s.n_ground = 0;
    s.n_nonground = 0;
    for (uint8_t m : cloud.ground_mask) {
        if (m != 0) {
            ++s.n_ground;
        } else {
            ++s.n_nonground;
        }
    }
}

} // namespace argus
