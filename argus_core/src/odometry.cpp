#include "argus_core/odometry.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Eigenvalues>

namespace argus {

namespace {

struct VoxelKey {
    int x = 0;
    int y = 0;
    int z = 0;
    bool operator==(const VoxelKey& o) const { return x == o.x && y == o.y && z == o.z; }
};

struct VoxelKeyHash {
    size_t operator()(const VoxelKey& k) const {
        return static_cast<size_t>(k.x) * 73856093u ^ static_cast<size_t>(k.y) * 19349663u ^
               static_cast<size_t>(k.z) * 83492791u;
    }
};

using VoxelMap = std::unordered_map<VoxelKey, Eigen::Vector3f, VoxelKeyHash>;

VoxelKey key_of(const Eigen::Vector3f& p, float inv) {
    VoxelKey k;
    k.x = static_cast<int>(std::floor(p.x() * inv));
    k.y = static_cast<int>(std::floor(p.y() * inv));
    k.z = static_cast<int>(std::floor(p.z() * inv));
    return k;
}

std::vector<Eigen::Vector3f> downsample(const CleanCloud& cloud, const OdometryParams& p) {
    std::vector<Eigen::Vector3f> out;
    if (cloud.empty()) {
        return out;
    }
    const float vs = std::max(p.voxel_size, 0.05f);
    const float inv = 1.0f / vs;
    const float min2 = p.min_range * p.min_range;
    const float max2 = p.max_range * p.max_range;
    VoxelMap acc;
    acc.reserve(cloud.size() / 4 + 1);
    for (size_t i = 0; i < cloud.size(); ++i) {
        if (!p.keep_ground && !cloud.ground_mask.empty() && i < cloud.ground_mask.size() &&
            cloud.ground_mask[i] != 0) {
            continue;
        }
        const Eigen::Vector3f pt(cloud.x[i], cloud.y[i], cloud.z[i]);
        const float r2 = pt.squaredNorm();
        if (r2 < min2 || r2 > max2) {
            continue;
        }
        acc[key_of(pt, inv)] = pt;
    }
    out.reserve(acc.size());
    for (const auto& kv : acc) {
        out.push_back(kv.second);
    }
    return out;
}

VoxelMap index_cloud(const std::vector<Eigen::Vector3f>& cloud, float inv) {
    VoxelMap m;
    m.reserve(cloud.size());
    for (const Eigen::Vector3f& p : cloud) {
        m[key_of(p, inv)] = p;
    }
    return m;
}

bool nearest(const Eigen::Vector3f& q, const VoxelMap& map, float inv, int radius, float max_d2,
             Eigen::Vector3f& out) {
    const VoxelKey c = key_of(q, inv);
    float best = max_d2;
    bool found = false;
    for (int dx = -radius; dx <= radius; ++dx) {
        for (int dy = -radius; dy <= radius; ++dy) {
            for (int dz = -radius; dz <= radius; ++dz) {
                VoxelKey k{c.x + dx, c.y + dy, c.z + dz};
                const auto it = map.find(k);
                if (it == map.end()) {
                    continue;
                }
                const float d2 = (it->second - q).squaredNorm();
                if (d2 < best) {
                    best = d2;
                    out = it->second;
                    found = true;
                }
            }
        }
    }
    return found;
}

Eigen::Vector3f normal_at(const Eigen::Vector3f& p, const VoxelMap& map, float inv) {
    const VoxelKey c = key_of(p, inv);
    Eigen::Vector3f acc = Eigen::Vector3f::Zero();
    int n = 0;
    Eigen::Vector3f pts[27];
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                VoxelKey k{c.x + dx, c.y + dy, c.z + dz};
                const auto it = map.find(k);
                if (it == map.end()) {
                    continue;
                }
                pts[n++] = it->second;
                acc += it->second;
            }
        }
    }
    if (n < 5) {
        return Eigen::Vector3f::Zero();
    }
    const Eigen::Vector3f mean = acc / static_cast<float>(n);
    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (int i = 0; i < n; ++i) {
        const Eigen::Vector3f d = pts[i] - mean;
        cov += d * d.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(cov);
    Eigen::Vector3f nrm = eig.eigenvectors().col(0);
    if (nrm.dot(p) > 0.0f) {
        nrm = -nrm;
    }
    return nrm;
}

Eigen::Matrix3f rot_yaw_pitch(const Eigen::Vector3f& left, const Eigen::Vector3f& up, float yaw,
                              float pitch) {
    const Eigen::AngleAxisf Ry(yaw, up);
    const Eigen::AngleAxisf Rp(pitch, left);
    return (Rp * Ry).toRotationMatrix();
}

} // namespace

TunnelOdometry::TunnelOdometry(const OdometryParams& p) : p_(p), gauge_(ClearanceGaugeParams{}) {
    ClearanceGaugeParams gp;
    gp.forward_axis = p.forward_axis;
    gauge_ = ClearanceGauge(gp);
}

OdometryResult TunnelOdometry::update(const CleanCloud& cloud, double stamp_s) {
    OdometryResult out;
    out.stamp_s = stamp_s;
    out.pose = pose_;
    out.frames = last_.frames + 1;
    if (!p_.enabled) {
        out.valid = true;
        last_ = out;
        return out;
    }

    std::vector<Eigen::Vector3f> cur = downsample(cloud, p_);
    out.n_downsampled = static_cast<uint32_t>(cur.size());
    if (!have_prev_ || prev_.empty() || cur.size() < 20) {
        prev_ = std::move(cur);
        prev_stamp_s_ = stamp_s;
        have_prev_ = true;
        out.valid = true;
        last_ = out;
        return out;
    }

    const float dt = stamp_s > prev_stamp_s_ ? static_cast<float>(stamp_s - prev_stamp_s_) : 0.1f;
    const Eigen::Vector3f forward(gauge_.forward_x(), gauge_.forward_y(), 0.0f);
    const Eigen::Vector3f left(gauge_.left_x(), gauge_.left_y(), 0.0f);
    const Eigen::Vector3f up(0.0f, 0.0f, 1.0f);
    const float vs = std::max(p_.voxel_size, 0.05f);
    const float inv = 1.0f / vs;
    const VoxelMap prev_map = index_cloud(prev_, inv);
    std::unordered_map<VoxelKey, Eigen::Vector3f, VoxelKeyHash> prev_normals;
    prev_normals.reserve(prev_map.size());
    for (const auto& kv : prev_map) {
        prev_normals.emplace(kv.first, normal_at(kv.second, prev_map, inv));
    }
    const float max_d2 = p_.max_correspondence_m * p_.max_correspondence_m;
    const int radius =
        std::max(1, std::min(3, static_cast<int>(std::ceil(p_.max_correspondence_m / vs))));

    float ds = 0.0f;
    float dyaw = 0.0f;
    float dpitch = 0.0f;
    float dz = 0.0f;
    float forward_support = 0.0f;

    for (uint32_t it = 0; it < p_.max_iterations; ++it) {
        const Eigen::Matrix3f R = rot_yaw_pitch(left, up, dyaw, dpitch);
        Eigen::Vector3f t = ds * forward;
        if (!p_.lock_vertical) {
            t += dz * up;
        }

        Eigen::Matrix4f AtA = Eigen::Matrix4f::Zero();
        Eigen::Vector4f Atb = Eigen::Vector4f::Zero();
        double err = 0.0;
        uint32_t n_corr = 0;
        double abs_fwd = 0.0;
        for (const Eigen::Vector3f& src : cur) {
            const Eigen::Vector3f q = R * src + t;
            Eigen::Vector3f tgt;
            if (!nearest(q, prev_map, inv, radius, max_d2, tgt)) {
                continue;
            }
            const auto n_it = prev_normals.find(key_of(tgt, inv));
            const Eigen::Vector3f nrm =
                n_it == prev_normals.end() ? Eigen::Vector3f::Zero() : n_it->second;
            if (nrm.squaredNorm() < 0.25f) {
                continue;
            }
            const Eigen::Vector3f src_r = R * src;
            Eigen::Vector4f J = Eigen::Vector4f::Zero();
            J(0) = nrm.dot(forward);
            J(1) = nrm.dot(up.cross(src_r));
            J(2) = nrm.dot(left.cross(src_r));
            J(3) = p_.lock_vertical ? 0.0f : nrm.dot(up);
            const float residual = nrm.dot(tgt - q);
            AtA += J * J.transpose();
            Atb += J * residual;
            ++n_corr;
            err += static_cast<double>(residual * residual);
            abs_fwd += static_cast<double>(std::fabs(J(0)));
        }
        out.n_correspondences = n_corr;
        if (n_corr < 20) {
            out.valid = false;
            last_ = out;
            return out;
        }
        out.fitness = static_cast<float>(err / static_cast<double>(n_corr));
        forward_support = static_cast<float>(abs_fwd / static_cast<double>(n_corr));
        AtA += 1e-3f * Eigen::Matrix4f::Identity();
        const Eigen::Vector4f x = AtA.ldlt().solve(Atb);
        if (forward_support >= 0.15f) {
            ds += x(0);
        }
        dyaw += x(1);
        dpitch += x(2);
        if (!p_.lock_vertical) {
            dz += x(3);
            const float z_lim = p_.voxel_size;
            dz = std::clamp(dz, -z_lim, z_lim);
        }
        if (std::fabs(x(0)) + std::fabs(x(1)) + std::fabs(x(2)) + std::fabs(x(3)) < 1e-4f) {
            break;
        }
    }

    const float trans = std::hypot(ds, p_.lock_vertical ? 0.0f : dz);
    const bool rest = trans < p_.rest_translation_m || forward_support < 0.15f;
    const float apply_ds = rest ? 0.0f : ds;
    const float apply_dyaw = rest ? 0.0f : dyaw;
    const float apply_dpitch = rest ? 0.0f : dpitch;
    const float apply_dz = rest ? 0.0f : dz;

    const float speed = std::fabs(apply_ds) / std::max(dt, 1e-3f);
    const float accel = (speed - prev_speed_mps_) / std::max(dt, 1e-3f);
    out.speed_mps = speed;
    out.accel_mps2 = accel;
    out.valid = speed <= p_.max_speed_mps && out.n_correspondences >= 20;

    if (out.valid && !rest) {
        Eigen::Isometry3d delta = Eigen::Isometry3d::Identity();
        const Eigen::Vector3d f(static_cast<double>(forward.x()), static_cast<double>(forward.y()),
                                0.0);
        const Eigen::Vector3d lft(static_cast<double>(left.x()), static_cast<double>(left.y()),
                                  0.0);
        const Eigen::Vector3d u(0.0, 0.0, 1.0);
        delta.rotate(Eigen::AngleAxisd(static_cast<double>(apply_dyaw), u));
        delta.rotate(Eigen::AngleAxisd(static_cast<double>(apply_dpitch), lft));
        Eigen::Vector3d tr = static_cast<double>(apply_ds) * f;
        if (!p_.lock_vertical) {
            tr += static_cast<double>(apply_dz) * u;
        }
        if (p_.lock_lateral) {
            const double lat =
                tr.x() * static_cast<double>(left.x()) + tr.y() * static_cast<double>(left.y());
            tr -= lat * lft;
        }
        delta.pretranslate(tr);
        pose_ = pose_ * delta;
        prev_speed_mps_ = speed;
    } else if (out.valid) {
        prev_speed_mps_ = 0.0f;
    }
    out.pose = pose_;
    prev_ = std::move(cur);
    prev_stamp_s_ = stamp_s;
    last_ = out;
    return out;
}

} // namespace argus
