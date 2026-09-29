
#include "argus_core/gauge.hpp"

#include <algorithm>
#include <cmath>

namespace argus {

bool parse_forward_axis(const std::string& name, ForwardAxis& out) {
    std::string s;
    for (char c : name) {
        s += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    if (s == "x" || s == "+x") {
        out = ForwardAxis::PosX;
        return true;
    }
    if (s == "-x") {
        out = ForwardAxis::NegX;
        return true;
    }
    if (s == "y" || s == "+y") {
        out = ForwardAxis::PosY;
        return true;
    }
    if (s == "-y") {
        out = ForwardAxis::NegY;
        return true;
    }
    return false;
}

void ClearanceGauge::to_gauge(float x, float y, float& gx, float& gy) const {
    gx = x * fx_ + y * fy_;
    gy = x * lx_ + y * ly_;
}

void ClearanceGauge::from_gauge(float gx, float gy, float& x, float& y) const {
    x = gx * fx_ + gy * lx_;
    y = gx * fy_ + gy * ly_;
}

bool ClearanceGauge::contains(float x, float y, float z) const {
    float gx = 0.0f, gy = 0.0f;
    to_gauge(x, y, gx, gy);
    return contains_at_path(gx, gy, z);
}

bool ClearanceGauge::contains_at_path(float gx, float gy, float z) const {
    const float x_min = p_.nose_offset - p_.safety_margin;
    const float x_max = p_.max_range;
    if (gx < x_min || gx > x_max) {
        return false;
    }

    const float z_shift = -p_.sensor_height;
    const float hw = p_.half_width;
    const float z_min = z_shift + p_.base_offset - p_.safety_margin;
    const float z_max = z_shift + p_.height + p_.safety_margin;
    if (z < z_min || z > z_max) {
        return false;
    }
    if (std::abs(gy) > hw) {
        return false;
    }

    if (z > chamfer_start_z_ + z_shift && chamfer_height_ > 0.0f) {
        const float t = (z - chamfer_start_z_ - z_shift) / chamfer_height_;
        const float hw_at_z = hw - p_.chamfer * t;
        if (std::abs(gy) > hw_at_z) {
            return false;
        }
    }
    return true;
}

float ClearanceGauge::forward_distance(float x, float y, float z) const {
    if (!contains(x, y, z)) {
        return -1.0f;
    }
    float gx = 0.0f, gy = 0.0f;
    to_gauge(x, y, gx, gy);
    return gx;
}

float ClearanceGauge::distance_to(float x, float y, float z) const {
    if (contains(x, y, z)) {
        return 0.0f;
    }

    float gx = 0.0f, gy = 0.0f;
    to_gauge(x, y, gx, gy);
    const float z_shift = -p_.sensor_height;
    const float hw = p_.half_width + p_.safety_margin;
    const float z_min = z_shift + p_.base_offset - p_.safety_margin;
    const float z_max = z_shift + p_.height + p_.safety_margin;

    const float dx = std::max(0.0f, p_.nose_offset - gx);
    const float dy = std::max(0.0f, std::abs(gy) - hw);
    const float dz = std::max(0.0f, std::max(z_min - z, z - z_max));
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void ClearanceGauge::to_mesh(float max_range, std::vector<Eigen::Vector3f>& vertices,
                             std::vector<uint32_t>& indices) const {
    vertices.clear();
    indices.clear();

    const float hw = p_.half_width;
    const float z_shift = -p_.sensor_height;
    const float z0 = z_shift + p_.base_offset;
    const float z1 = z_shift + p_.height - p_.chamfer;
    const float z2 = z_shift + p_.height;
    const float hw_top = p_.half_width - p_.chamfer;

    for (int side = 0; side < 2; ++side) {
        const float gx = (side == 0) ? p_.nose_offset : max_range;
        float x = 0.0f, y = 0.0f;
        const float gy[6] = {-hw, hw, hw, hw_top, -hw_top, -hw};
        const float gz[6] = {z0, z0, z1, z2, z2, z1};
        for (int k = 0; k < 6; ++k) {
            from_gauge(gx, gy[k], x, y);
            vertices.emplace_back(x, y, gz[k]);
        }
    }
}

ClearanceGauge::ClearanceGauge(const ClearanceGaugeParams& p) : p_(p) {
    chamfer_start_z_ = p_.height - p_.chamfer;
    chamfer_height_ = std::max(0.001f, p_.chamfer);

    switch (p_.forward_axis) {
        case ForwardAxis::NegX:
            fx_ = -1.0f;
            fy_ = 0.0f;
            lx_ = 0.0f;
            ly_ = -1.0f;
            break;
        case ForwardAxis::PosY:
            fx_ = 0.0f;
            fy_ = 1.0f;
            lx_ = -1.0f;
            ly_ = 0.0f;
            break;
        case ForwardAxis::NegY:
            fx_ = 0.0f;
            fy_ = -1.0f;
            lx_ = 1.0f;
            ly_ = 0.0f;
            break;
        case ForwardAxis::PosX:
        default:
            fx_ = 1.0f;
            fy_ = 0.0f;
            lx_ = 0.0f;
            ly_ = 1.0f;
            break;
    }
}

} // namespace argus
