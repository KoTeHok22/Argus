
#include "argus_core/cloud_filter.hpp"

#include <cmath>
#include <cstring>

#include <sensor_msgs/msg/point_field.hpp>

namespace argus {

using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

namespace {

const PointField* find_field(const PointCloud2& msg, const std::string& name) {
    for (const auto& f : msg.fields) {
        if (f.name == name) {
            return &f;
        }
    }
    return nullptr;
}

const PointField* find_float32(const PointCloud2& msg, const std::string& name) {
    const PointField* f = find_field(msg, name);
    if (f == nullptr || f->count < 1) {
        return nullptr;
    }
    if (f->datatype == PointField::FLOAT32 || f->datatype == PointField::INT32 ||
        f->datatype == PointField::UINT32) {
        return f;
    }
    return nullptr;
}

} // namespace

std::optional<CloudLayout> parse_layout(const PointCloud2& msg) {
    CloudLayout out;
    out.point_step = msg.point_step;

    if (msg.point_step == 0) {
        out.error = "point_step == 0";
        return std::nullopt;
    }

    const PointField* x = find_float32(msg, "x");
    const PointField* y = find_float32(msg, "y");
    const PointField* z = find_float32(msg, "z");
    if (x == nullptr || y == nullptr || z == nullptr) {
        out.error = "missing x/y/z float32 fields";
        return std::nullopt;
    }
    out.offset_x = x->offset;
    out.offset_y = y->offset;
    out.offset_z = z->offset;

    if (const PointField* i = find_float32(msg, "intensity")) {
        out.has_intensity = true;
        out.offset_intensity = i->offset;
    }

    if (const PointField* r = find_field(msg, "ring");
        r != nullptr && r->datatype == PointField::UINT16 && r->count >= 1) {
        out.has_ring = true;
        out.offset_ring = r->offset;
    }

    if (const PointField* t = find_field(msg, "timestamp");
        t != nullptr && t->datatype == PointField::FLOAT64 && t->count >= 1) {
        out.has_timestamp = true;
        out.offset_timestamp = t->offset;
    }

    const uint64_t declared = static_cast<uint64_t>(msg.width) * msg.height * msg.point_step;
    if (declared > msg.data.size()) {
        out.error = "data buffer smaller than width*height*point_step";
        return std::nullopt;
    }
    out.n_points = static_cast<uint32_t>(msg.width) * msg.height;

    return out;
}

CleanCloud filter_and_index(const PointCloud2& msg, const CloudLayout& layout,
                            const FilterParams& params, FilterStats* stats) {
    FilterStats local;
    FilterStats& s = (stats != nullptr) ? *stats : local;
    s = FilterStats{};

    CleanCloud out;
    out.stamp_s = static_cast<double>(msg.header.stamp.sec) +
                  static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
    out.frame_id = msg.header.frame_id;

    constexpr uint32_t kRings = 128;
    out.rings = kRings;
    out.az_steps = (static_cast<uint32_t>(layout.n_points) + kRings - 1) / kRings;
    out.n_raw = static_cast<uint32_t>(layout.n_points);

    const size_t n = layout.n_points;
    out.x.reserve(n);
    out.y.reserve(n);
    out.z.reserve(n);
    out.intensity.reserve(n);
    out.ring.reserve(n);
    out.azimuth_idx.reserve(n);
    out.raw_idx.reserve(n);

    const uint8_t* base = msg.data.data();
    const uint32_t ps = layout.point_step;

    for (size_t i = 0; i < n; ++i) {
        ++s.n_raw;
        const uint8_t* p = base + i * ps;
        const auto raw = static_cast<uint32_t>(i);

        float x = 0.0f, y = 0.0f, z = 0.0f;
        std::memcpy(&x, p + layout.offset_x, sizeof(float));
        std::memcpy(&y, p + layout.offset_y, sizeof(float));
        std::memcpy(&z, p + layout.offset_z, sizeof(float));

        if (params.drop_nonfinite &&
            (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))) {
            ++s.reject_nonfinite;
            ++s.n_rejected;
            continue;
        }

        if (params.drop_zero_xyz && x == 0.0f && y == 0.0f && z == 0.0f) {
            ++s.reject_zero;
            ++s.n_rejected;
            out.no_return_raw.push_back(raw);
            continue;
        }

        if (std::abs(x) > params.max_abs_coord || std::abs(y) > params.max_abs_coord ||
            std::abs(z) > params.max_abs_coord) {
            ++s.reject_out_of_range;
            ++s.n_rejected;
            continue;
        }

        const float d2 = x * x + y * y + z * z;
        const float min2 = params.min_range * params.min_range;
        const float max2 = params.max_range * params.max_range;
        if (d2 < min2 || d2 > max2) {
            ++s.reject_out_of_range;
            ++s.n_rejected;
            continue;
        }

        out.x.push_back(x);
        out.y.push_back(y);
        out.z.push_back(z);
        out.raw_idx.push_back(raw);

        float intensity = 0.0f;
        if (layout.has_intensity) {
            std::memcpy(&intensity, p + layout.offset_intensity, sizeof(float));
        }
        out.intensity.push_back(intensity);

        uint16_t ring = static_cast<uint16_t>(raw % kRings);
        if (layout.has_ring) {
            std::memcpy(&ring, p + layout.offset_ring, sizeof(uint16_t));
        }
        out.ring.push_back(ring);
        out.azimuth_idx.push_back(raw / kRings);
    }

    s.n_valid = static_cast<uint32_t>(out.size());
    return out;
}

} // namespace argus
