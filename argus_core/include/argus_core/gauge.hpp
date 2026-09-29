
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "argus_core/types.hpp"

namespace argus {

enum class ForwardAxis : uint8_t { PosX, NegX, PosY, NegY };

bool parse_forward_axis(const std::string& name, ForwardAxis& out);

struct ClearanceGaugeParams {
    float half_width = 1.45f;
    float height = 3.60f;
    float base_offset = 0.20f;
    float chamfer = 0.20f;
    float nose_offset = 0.00f;
    float max_range = 300.0f;
    float safety_margin = 0.10f;
    float sensor_height = 0.0f;
    ForwardAxis forward_axis = ForwardAxis::PosX;
};

class ClearanceGauge {
public:
    explicit ClearanceGauge(const ClearanceGaugeParams& p);

    bool contains(float x, float y, float z) const;
    bool contains_at_path(float forward, float lateral, float z) const;

    float forward_distance(float x, float y, float z) const;

    float distance_to(float x, float y, float z) const;

    void to_mesh(float max_range, std::vector<Eigen::Vector3f>& vertices,
                 std::vector<uint32_t>& indices) const;

    const ClearanceGaugeParams& params() const { return p_; }

    float forward_x() const { return fx_; }
    float forward_y() const { return fy_; }
    float left_x() const { return lx_; }
    float left_y() const { return ly_; }

private:
    ClearanceGaugeParams p_;
    float chamfer_start_z_ = 0.0f;
    float chamfer_height_ = 0.0f;
    float fx_ = 1.0f, fy_ = 0.0f;
    float lx_ = 0.0f, ly_ = 1.0f;

    void to_gauge(float x, float y, float& gx, float& gy) const;
    void from_gauge(float gx, float gy, float& x, float& y) const;
};

} // namespace argus
