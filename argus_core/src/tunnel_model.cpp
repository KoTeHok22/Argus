
#include "argus_core/tunnel_model.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <utility>

namespace argus {

namespace {

constexpr uint32_t kSaveVersion = 1;

float clampf(float v, float lo, float hi) {
    return std::max(lo, std::min(hi, v));
}

} // namespace

TunnelModel::TunnelModel(const TunnelModelParams& p) : p_(p) {
    const float vs = std::max(p_.voxel_size, 0.02f);
    p_.voxel_size = vs;
    inv_voxel_ = 1.0f / vs;
}

TunnelModel::VoxelKey TunnelModel::key_at(const Eigen::Vector3f& point) const {
    VoxelKey k;
    k.x = static_cast<int32_t>(std::floor(point.x() * inv_voxel_));
    k.y = static_cast<int32_t>(std::floor(point.y() * inv_voxel_));
    k.z = static_cast<int32_t>(std::floor(point.z() * inv_voxel_));
    return k;
}

TunnelModel::VoxelKey TunnelModel::key_at(const Eigen::Vector3f& point,
                                          const Eigen::Isometry3d& pose) const {
    return key_at(to_world(point, pose));
}

Eigen::Vector3f TunnelModel::center_of(const VoxelKey& k) const {
    const float half = 0.5f * p_.voxel_size;
    return Eigen::Vector3f(static_cast<float>(k.x) * p_.voxel_size + half,
                           static_cast<float>(k.y) * p_.voxel_size + half,
                           static_cast<float>(k.z) * p_.voxel_size + half);
}

Eigen::Vector3f TunnelModel::to_lidar(const Eigen::Vector3f& world,
                                      const Eigen::Isometry3d& pose) const {
    return (pose.inverse() * world.cast<double>()).cast<float>();
}

Eigen::Vector3f TunnelModel::to_world(const Eigen::Vector3f& lidar,
                                      const Eigen::Isometry3d& pose) const {
    return (pose * lidar.cast<double>()).cast<float>();
}

void TunnelModel::carve_free_space(const Eigen::Vector3f& start, const Eigen::Vector3f& end,
                                   const VoxelKey& stop_key) {
    const Eigen::Vector3f delta = end - start;
    if (delta.squaredNorm() <= 0.0f) {
        return;
    }
    const VoxelKey first = key_at(start);
    const VoxelKey last = key_at(end);
    const int32_t dx = last.x - first.x;
    const int32_t dy = last.y - first.y;
    const int32_t dz = last.z - first.z;
    const int32_t steps = std::max({std::abs(dx), std::abs(dy), std::abs(dz), 1});
    const float inv = 1.0f / static_cast<float>(steps);
    VoxelKey prev;
    for (int32_t s = 1; s <= steps; ++s) {
        const float t = static_cast<float>(s) * inv;
        const VoxelKey k = key_at(start + delta * t);
        if (s > 1 && k == prev) {
            continue;
        }
        prev = k;
        if (k == stop_key) {
            break;
        }
        if (s == 1 && k == first) {
            continue;
        }
        Voxel& v = voxels_[k];
        v.free_observations++;
        v.free_score = 1.0f - 1.0f / static_cast<float>(v.free_observations);
        v.occupancy_log_odds = clampf(v.occupancy_log_odds + p_.occupancy_miss_log_odds,
                                      p_.occupancy_min_log_odds, p_.occupancy_max_log_odds);
        v.last_frame = static_cast<uint32_t>(frames_);
        ++cells_carved_;
    }
}

bool TunnelModel::carve_ray_selected(const CleanCloud& cloud, size_t i) const {
    const uint32_t stride_az = std::max<uint32_t>(1, p_.carve_stride_az);
    const uint32_t stride_ring = std::max<uint32_t>(1, p_.carve_stride_ring);
    if (stride_az == 1 && stride_ring == 1) {
        return true;
    }
    uint32_t az = 0;
    uint32_t ring = 0;
    if (!cloud.azimuth_idx.empty() && !cloud.ring.empty() && i < cloud.azimuth_idx.size() &&
        i < cloud.ring.size()) {
        az = cloud.azimuth_idx[i];
        ring = static_cast<uint32_t>(cloud.ring[i]);
    } else if (cloud.rings > 1 && !cloud.raw_idx.empty() && i < cloud.raw_idx.size()) {
        az = cloud.raw_idx[i] / cloud.rings;
        ring = cloud.raw_idx[i] % cloud.rings;
    } else {
        return true;
    }
    const uint32_t frame = static_cast<uint32_t>(frames_);
    return az % stride_az == frame % stride_az && ring % stride_ring == frame % stride_ring;
}

void TunnelModel::integrate(const CleanCloud& cloud, const Eigen::Isometry3d& pose) {
    integrate(cloud, pose, true);
}

void TunnelModel::integrate(const CleanCloud& cloud, const Eigen::Isometry3d& pose,
                            bool register_hits) {
    ++frames_;
    if (!p_.enabled || cloud.empty()) {
        return;
    }

    const float margin = std::max(0.0f, p_.free_space_margin);
    const float max_range = p_.max_range;
    const float carve_range =
        p_.carve_max_range > 0.0f ? std::min(p_.max_range, p_.carve_max_range) : p_.max_range;
    const Eigen::Vector3f origin = to_lidar(Eigen::Vector3f::Zero(), pose);
    voxels_.reserve(voxels_.size() + cloud.size() / 4 + 1024);

    for (size_t i = 0; i < cloud.size(); ++i) {
        if (!cloud.ground_mask.empty() && i < cloud.ground_mask.size() &&
            cloud.ground_mask[i] != 0) {
            continue;
        }
        const Eigen::Vector3f lidar(cloud.x[i], cloud.y[i], cloud.z[i]);
        const float range = lidar.norm();
        if (!std::isfinite(range) || range < p_.voxel_size || range > max_range) {
            continue;
        }
        if (!carve_ray_selected(cloud, i)) {
            continue;
        }

        const Eigen::Vector3f hit = to_world(lidar, pose);
        const VoxelKey hit_key = key_at(hit);
        if (register_hits) {
            Voxel& occupied = voxels_[hit_key];
            occupied.occupied_observations++;
            occupied.occupancy_log_odds =
                clampf(occupied.occupancy_log_odds + p_.occupancy_hit_log_odds,
                       p_.occupancy_min_log_odds, p_.occupancy_max_log_odds);
            occupied.last_frame = static_cast<uint32_t>(frames_);
        }

        const float stop =
            std::max(0.0f, std::min(range, carve_range) - margin - 0.5f * p_.voxel_size);
        if (stop <= 0.0f) {
            continue;
        }
        if (p_.carve_half_width_m > 0.0f && std::fabs(lidar.x()) > p_.carve_half_width_m) {
            continue;
        }
        const Eigen::Vector3f dir = lidar / range;
        carve_free_space(origin, to_world(dir * stop, pose), hit_key);
    }

    if (p_.enable_decay && p_.decay_after_frames > 0 && frames_ % p_.decay_after_frames == 0) {
        for (auto it = voxels_.begin(); it != voxels_.end();) {
            if (frames_ - it->second.last_frame > p_.decay_after_frames) {
                it = voxels_.erase(it);
            } else {
                ++it;
            }
        }
    }
}

bool TunnelModel::violates_free_space(const Eigen::Vector3f& p, float& out_confidence) const {
    return violates_free_space(p, out_confidence, Eigen::Isometry3d::Identity());
}

bool TunnelModel::violates_free_space(const Eigen::Vector3f& p, float& out_confidence,
                                      const Eigen::Isometry3d& pose) const {
    out_confidence = 0.0f;
    const auto it = voxels_.find(key_at(p, pose));
    if (it == voxels_.end()) {
        return false;
    }
    const Voxel& v = it->second;
    if (v.free_observations < p_.min_observations) {
        return false;
    }
    if (v.occupied_observations >= p_.free_hits_to_clear) {
        return false;
    }
    out_confidence = v.free_score;
    return true;
}

bool TunnelModel::contradicts_free_space(const Eigen::Vector3f& p, float& out_confidence,
                                         const Eigen::Isometry3d& pose, float min_confidence,
                                         uint32_t min_free_observations) const {
    out_confidence = 0.0f;
    const auto it = voxels_.find(key_at(p, pose));
    if (it == voxels_.end()) {
        return false;
    }
    const Voxel& v = it->second;
    if (v.free_observations < std::max(p_.min_observations, min_free_observations)) {
        return false;
    }
    if (v.free_score < min_confidence) {
        return false;
    }
    if (v.occupied_observations >= p_.free_hits_to_clear) {
        return false;
    }
    out_confidence = v.free_score;
    return true;
}

FreeSpaceCheck TunnelModel::check(const CleanCloud& cloud, const Eigen::Isometry3d& pose,
                                  float min_confidence) const {
    FreeSpaceCheck out;
    for (size_t i = 0; i < cloud.size(); ++i) {
        if (!cloud.ground_mask.empty() && i < cloud.ground_mask.size() &&
            cloud.ground_mask[i] != 0) {
            continue;
        }
        const Eigen::Vector3f lidar(cloud.x[i], cloud.y[i], cloud.z[i]);
        const float range = lidar.norm();
        if (!std::isfinite(range) || range < p_.voxel_size || range > p_.max_range) {
            continue;
        }
        const auto it = voxels_.find(key_at(lidar, pose));
        if (it == voxels_.end() || it->second.free_observations == 0) {
            ++out.unknown;
            continue;
        }
        ++out.observed;
        if (it->second.occupied_observations < p_.free_hits_to_clear &&
            it->second.free_score >= min_confidence) {
            ++out.empty;
        }
    }
    return out;
}

TunnelModelStats TunnelModel::stats() const {
    TunnelModelStats s;
    s.n_voxels = static_cast<uint32_t>(voxels_.size());
    for (const auto& kv : voxels_) {
        const Voxel& v = kv.second;
        if (v.occupied_observations >= p_.free_hits_to_clear) {
            ++s.n_occupied;
            continue;
        }
        if (v.free_observations >= p_.min_observations) {
            ++s.n_free;
        }
    }
    s.free_ratio =
        s.n_voxels == 0 ? 0.0f : static_cast<float>(s.n_free) / static_cast<float>(s.n_voxels);
    s.frames_integrated = frames_;
    s.ready = ready();
    return s;
}

bool TunnelModel::ready() const {
    if (p_.min_observations == 0) {
        return !voxels_.empty();
    }
    return frames_ >= p_.min_observations && !voxels_.empty();
}

void TunnelModel::export_occupancy(std::vector<Eigen::Vector3f>& pts, std::vector<float>& conf,
                                   bool free_only) const {
    pts.clear();
    conf.clear();
    pts.reserve(voxels_.size());
    conf.reserve(voxels_.size());
    for (const auto& kv : voxels_) {
        const Voxel& v = kv.second;
        const bool occupied = v.occupied_observations >= p_.free_hits_to_clear;
        if (free_only && occupied) {
            continue;
        }
        const bool solid = occupied || v.occupancy_log_odds > 0.0f;
        if (!solid && v.free_observations < p_.min_observations) {
            continue;
        }
        pts.push_back(center_of(kv.first));
        conf.push_back(solid ? 1.0f : v.free_score);
    }
}

void TunnelModel::clear() {
    voxels_.clear();
    frames_ = 0;
    cells_carved_ = 0;
}

bool TunnelModel::save(const std::string& path) const {
    FILE* f = std::fopen(path.c_str(), "wb");
    if (f == nullptr) {
        return false;
    }
    bool ok = true;
    const char magic[8] = {'A', 'R', 'G', 'T', 'M', '0', '0', '1'};
    ok = ok && std::fwrite(magic, 1, 8, f) == 8;
    ok = ok && std::fwrite(&kSaveVersion, sizeof(kSaveVersion), 1, f) == 1;
    ok = ok && std::fwrite(&p_.voxel_size, sizeof(p_.voxel_size), 1, f) == 1;
    const uint64_t frames = frames_;
    ok = ok && std::fwrite(&frames, sizeof(frames), 1, f) == 1;
    const uint64_t n = voxels_.size();
    ok = ok && std::fwrite(&n, sizeof(n), 1, f) == 1;
    for (const auto& kv : voxels_) {
        ok = ok && std::fwrite(&kv.first, sizeof(VoxelKey), 1, f) == 1;
        ok = ok && std::fwrite(&kv.second, sizeof(Voxel), 1, f) == 1;
        if (!ok) {
            break;
        }
    }
    std::fclose(f);
    return ok;
}

bool TunnelModel::load(const std::string& path) {
    FILE* f = std::fopen(path.c_str(), "rb");
    if (f == nullptr) {
        return false;
    }
    char magic[8] = {0};
    uint32_t version = 0;
    float voxel_size = 0.0f;
    uint64_t frames = 0;
    uint64_t n = 0;
    bool ok = std::fread(magic, 1, 8, f) == 8 && std::memcmp(magic, "ARGTM001", 8) == 0 &&
              std::fread(&version, sizeof(version), 1, f) == 1 && version == kSaveVersion &&
              std::fread(&voxel_size, sizeof(voxel_size), 1, f) == 1 &&
              std::fread(&frames, sizeof(frames), 1, f) == 1 &&
              std::fread(&n, sizeof(n), 1, f) == 1;
    if (!ok || voxel_size <= 0.0f) {
        std::fclose(f);
        return false;
    }
    VoxelMap loaded;
    loaded.reserve(static_cast<size_t>(n));
    for (uint64_t i = 0; i < n; ++i) {
        VoxelKey k;
        Voxel v;
        if (std::fread(&k, sizeof(VoxelKey), 1, f) != 1 ||
            std::fread(&v, sizeof(Voxel), 1, f) != 1) {
            std::fclose(f);
            return false;
        }
        loaded[k] = v;
    }
    std::fclose(f);
    voxels_ = std::move(loaded);
    p_.voxel_size = voxel_size;
    inv_voxel_ = 1.0f / voxel_size;
    frames_ = frames;
    return true;
}

} // namespace argus
