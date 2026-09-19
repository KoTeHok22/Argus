
#include "argus_core/tunnel_model.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <utility>

namespace argus {

namespace {

constexpr uint32_t kSaveVersion = 2;

float clampf(float v, float lo, float hi) {
    return std::max(lo, std::min(hi, v));
}

} // namespace

TunnelModel::BlockKey TunnelModel::block_of(const VoxelKey& k) {
    BlockKey b;
    b.x = k.x >> kBlockShift;
    b.y = k.y >> kBlockShift;
    b.z = k.z >> kBlockShift;
    return b;
}

int32_t TunnelModel::index_in_block(const VoxelKey& k) {
    const int32_t lx = k.x & static_cast<int32_t>(kVoxelBlock - 1);
    const int32_t ly = k.y & static_cast<int32_t>(kVoxelBlock - 1);
    const int32_t lz = k.z & static_cast<int32_t>(kVoxelBlock - 1);
    return (lz * static_cast<int32_t>(kVoxelBlock) + ly) * static_cast<int32_t>(kVoxelBlock) + lx;
}

TunnelModel::Voxel* TunnelModel::find_voxel(const VoxelKey& k) {
    const auto it = blocks_.find(block_of(k));
    if (it == blocks_.end()) {
        return nullptr;
    }
    return &it->second[static_cast<size_t>(index_in_block(k))];
}

const TunnelModel::Voxel* TunnelModel::find_voxel(const VoxelKey& k) const {
    const auto it = blocks_.find(block_of(k));
    if (it == blocks_.end()) {
        return nullptr;
    }
    return &it->second[static_cast<size_t>(index_in_block(k))];
}

TunnelModel::Voxel& TunnelModel::ensure_voxel(const VoxelKey& k) {
    const BlockKey bk = block_of(k);
    Block& block = blocks_[bk];
    return block[static_cast<size_t>(index_in_block(k))];
}

size_t TunnelModel::occupied_voxel_count() const {
    size_t n = 0;
    for (const auto& kv : blocks_) {
        for (const Voxel& voxel : kv.second) {
            if (voxel.free_observations != 0 || voxel.occupied_observations != 0 ||
                voxel.occupancy_log_odds != 0.0f) {
                ++n;
            }
        }
    }
    return n;
}

void TunnelModel::rebuild_grid() {
    std::vector<std::pair<VoxelKey, Voxel>> snapshot;
    snapshot.reserve(occupied_voxel_count());
    for (const auto& kv : blocks_) {
        for (uint32_t slot = 0; slot < kBlockVolume; ++slot) {
            const Voxel& voxel = kv.second[slot];
            if (voxel.free_observations == 0 && voxel.occupied_observations == 0 &&
                voxel.occupancy_log_odds == 0.0f) {
                continue;
            }
            const int32_t lx = static_cast<int32_t>(slot % kVoxelBlock);
            const int32_t ly = static_cast<int32_t>((slot / kVoxelBlock) % kVoxelBlock);
            const int32_t lz = static_cast<int32_t>(slot / (kVoxelBlock * kVoxelBlock));
            VoxelKey k;
            k.x = kv.first.x * static_cast<int32_t>(kVoxelBlock) + lx;
            k.y = kv.first.y * static_cast<int32_t>(kVoxelBlock) + ly;
            k.z = kv.first.z * static_cast<int32_t>(kVoxelBlock) + lz;
            snapshot.emplace_back(k, voxel);
        }
    }
    BlockMap rebuilt;
    rebuilt.reserve(blocks_.size());
    for (const auto& kv : snapshot) {
        rebuilt[block_of(kv.first)][static_cast<size_t>(index_in_block(kv.first))] = kv.second;
    }
    blocks_ = std::move(rebuilt);
}

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
    if ((end - start).squaredNorm() <= 0.0f) {
        return;
    }
    const VoxelKey first = key_at(start);
    const VoxelKey last = key_at(end);
    const int32_t dx = last.x - first.x;
    const int32_t dy = last.y - first.y;
    const int32_t dz = last.z - first.z;
    const int32_t steps = std::max({std::abs(dx), std::abs(dy), std::abs(dz), 1});
    const float inv = 1.0f / static_cast<float>(steps);
    const float occupancy_miss = p_.occupancy_miss_log_odds;
    const float min_log_odds = p_.occupancy_min_log_odds;
    const float max_log_odds = p_.occupancy_max_log_odds;
    const uint32_t empty = kSamplePeriod * static_cast<uint32_t>(frames_) + sample_cursor_;
    const uint32_t filled = empty + kSamplePeriod;
    VoxelKey prev;
    for (int32_t s = 1; s <= steps; ++s) {
        const float t = static_cast<float>(s) * inv;
        const VoxelKey k = key_at(start + (end - start) * t);
        if (k == stop_key) {
            break;
        }
        if (s > 1 && k == prev) {
            continue;
        }
        prev = k;
        if (s == 1 && k == first) {
            continue;
        }
        const BlockKey bk = block_of(k);
        GridMark& mark = grid_marks_[bk];
        if (mark.frame != empty && mark.frame != filled) {
            if (mark.generation == sample_cursor_) {
                mark.frame = filled;
            } else {
                mark.frame = empty;
                mark.generation = sample_cursor_;
            }
        }
        Voxel& v = blocks_[bk][static_cast<size_t>(index_in_block(k))];
        v.free_observations++;
        v.free_score = 1.0f - 1.0f / static_cast<float>(v.free_observations);
        v.occupancy_log_odds =
            clampf(v.occupancy_log_odds + occupancy_miss, min_log_odds, max_log_odds);
        v.last_frame = static_cast<uint32_t>(frames_);
        ++cells_carved_;
        const uintptr_t address = reinterpret_cast<uintptr_t>(&v);
        const auto inserted = entries_.emplace(address, Entry{address, &v, 0u});
        if (inserted.second) {
            free_dirty_.push_back(&v);
        }
        inserted.first->second.new_observations++;
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
    blocks_.reserve(blocks_.size() + cloud.size() / 1024 + 64);
    free_dirty_.clear();
    const uint32_t samples = std::min<uint32_t>(cloud.size(), kSamplesPerBlock);
    const size_t stride = samples > 0 ? std::max<size_t>(1, cloud.size() / samples) : 1;
    sample_cursor_ = (sample_cursor_ + 1) % kSamplePeriod;
    const float sample_max_range = kSamplePeriod * max_range;

    for (size_t i = 0; i < cloud.size(); ++i) {
        if (stride > 1 && (i + sample_cursor_) % stride != 0) {
            continue;
        }
        const Eigen::Vector3f lidar(cloud.x[i], cloud.y[i], cloud.z[i]);
        const float range = lidar.norm();
        if (!std::isfinite(range) || range < p_.voxel_size || range > sample_max_range) {
            continue;
        }
        if (!carve_ray_selected(cloud, i)) {
            continue;
        }
        if (range > max_range) {
            if (!register_hits) {
                continue;
            }
            Voxel& occupied = ensure_voxel(key_at(to_world(lidar, pose)));
            occupied.occupied_observations++;
            occupied.occupancy_log_odds =
                clampf(occupied.occupancy_log_odds + p_.occupancy_hit_log_odds,
                       p_.occupancy_min_log_odds, p_.occupancy_max_log_odds);
            occupied.last_frame = static_cast<uint32_t>(frames_);
            occupied.occupancy_dirty = 1;
            continue;
        }
        if (!cloud.ground_mask.empty() && i < cloud.ground_mask.size() &&
            cloud.ground_mask[i] != 0) {
            continue;
        }

        const Eigen::Vector3f hit = to_world(lidar, pose);
        const VoxelKey hit_key = key_at(hit);
        if (register_hits) {
            Voxel& occupied = ensure_voxel(hit_key);
            occupied.occupied_observations++;
            occupied.occupancy_log_odds =
                clampf(occupied.occupancy_log_odds + p_.occupancy_hit_log_odds,
                       p_.occupancy_min_log_odds, p_.occupancy_max_log_odds);
            occupied.last_frame = static_cast<uint32_t>(frames_);
            occupied.occupancy_dirty = 1;
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

    for (const auto& kv : entries_) {
        kv.second.voxel->free_observations += kv.second.new_observations;
    }
    for (Voxel* voxel : free_dirty_) {
        voxel->free_score = 1.0f - 1.0f / static_cast<float>(voxel->free_observations);
    }
    if (register_hits) {
        for (auto& kv : blocks_) {
            for (Voxel& voxel : kv.second) {
                if (voxel.occupancy_dirty == 0) {
                    continue;
                }
                voxel.occupancy_dirty = 0;
                if (voxel.occupied_observations < p_.free_hits_to_clear) {
                    continue;
                }
                voxel.free_observations = 0;
                voxel.free_score = 0.0f;
            }
        }
    }
    entries_.clear();

    if (p_.enable_decay && p_.decay_after_frames > 0 && frames_ % p_.decay_after_frames == 0) {
        for (auto it = blocks_.begin(); it != blocks_.end();) {
            if (frames_ - static_cast<uint64_t>(it->second.front().last_frame) >
                p_.decay_after_frames) {
                it = blocks_.erase(it);
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
    const Voxel* voxel = find_voxel(key_at(p, pose));
    if (voxel == nullptr) {
        return false;
    }
    if (voxel->free_observations < p_.min_observations) {
        return false;
    }
    if (voxel->occupied_observations >= p_.free_hits_to_clear) {
        return false;
    }
    out_confidence = voxel->free_score;
    return true;
}

bool TunnelModel::contradicts_free_space(const Eigen::Vector3f& p, float& out_confidence,
                                         const Eigen::Isometry3d& pose, float min_confidence,
                                         uint32_t min_free_observations) const {
    out_confidence = 0.0f;
    const Voxel* voxel = find_voxel(key_at(p, pose));
    if (voxel == nullptr) {
        return false;
    }
    if (voxel->free_observations < std::max(p_.min_observations, min_free_observations)) {
        return false;
    }
    if (voxel->free_score < min_confidence) {
        return false;
    }
    if (voxel->occupied_observations >= p_.free_hits_to_clear) {
        return false;
    }
    out_confidence = voxel->free_score;
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
        const Voxel* voxel = find_voxel(key_at(lidar, pose));
        if (voxel == nullptr || voxel->free_observations == 0) {
            ++out.unknown;
            continue;
        }
        ++out.observed;
        if (voxel->occupied_observations < p_.free_hits_to_clear &&
            voxel->free_score >= min_confidence) {
            ++out.empty;
        }
    }
    return out;
}

TunnelModelStats TunnelModel::stats() const {
    TunnelModelStats s;
    s.n_voxels = static_cast<uint32_t>(occupied_voxel_count());
    for (const auto& kv : blocks_) {
        for (const Voxel& voxel : kv.second) {
            if (voxel.free_observations == 0 && voxel.occupied_observations == 0 &&
                voxel.occupancy_log_odds == 0.0f) {
                continue;
            }
            if (voxel.occupied_observations >= p_.free_hits_to_clear) {
                ++s.n_occupied;
                continue;
            }
            if (voxel.free_observations >= p_.min_observations) {
                ++s.n_free;
            }
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
        return !blocks_.empty();
    }
    return frames_ >= p_.min_observations && !blocks_.empty();
}

void TunnelModel::export_occupancy(std::vector<Eigen::Vector3f>& pts, std::vector<float>& conf,
                                   bool free_only) const {
    pts.clear();
    conf.clear();
    pts.reserve(occupied_voxel_count());
    conf.reserve(occupied_voxel_count());
    for (const auto& kv : blocks_) {
        for (uint32_t slot = 0; slot < kBlockVolume; ++slot) {
            const Voxel& voxel = kv.second[slot];
            if (voxel.free_observations == 0 && voxel.occupied_observations == 0 &&
                voxel.occupancy_log_odds == 0.0f) {
                continue;
            }
            const bool occupied = voxel.occupied_observations >= p_.free_hits_to_clear;
            if (free_only && occupied) {
                continue;
            }
            const bool solid = occupied || voxel.occupancy_log_odds > 0.0f;
            if (!solid && voxel.free_observations < p_.min_observations) {
                continue;
            }
            const int32_t lx = static_cast<int32_t>(slot % kVoxelBlock);
            const int32_t ly = static_cast<int32_t>((slot / kVoxelBlock) % kVoxelBlock);
            const int32_t lz = static_cast<int32_t>(slot / (kVoxelBlock * kVoxelBlock));
            VoxelKey k;
            k.x = kv.first.x * static_cast<int32_t>(kVoxelBlock) + lx;
            k.y = kv.first.y * static_cast<int32_t>(kVoxelBlock) + ly;
            k.z = kv.first.z * static_cast<int32_t>(kVoxelBlock) + lz;
            pts.push_back(center_of(k));
            conf.push_back(solid ? 1.0f : voxel.free_score);
        }
    }
}

void TunnelModel::clear() {
    blocks_.clear();
    entries_.clear();
    free_dirty_.clear();
    frames_ = 0;
    cells_carved_ = 0;
}

bool TunnelModel::save(const std::string& path) const {
    FILE* f = std::fopen(path.c_str(), "wb");
    if (f == nullptr) {
        return false;
    }
    std::vector<VoxelKey> keys;
    std::vector<Voxel> values;
    keys.reserve(occupied_voxel_count());
    values.reserve(occupied_voxel_count());
    for (const auto& kv : blocks_) {
        for (uint32_t slot = 0; slot < kBlockVolume; ++slot) {
            const Voxel& voxel = kv.second[slot];
            if (voxel.free_observations == 0 && voxel.occupied_observations == 0 &&
                voxel.occupancy_log_odds == 0.0f) {
                continue;
            }
            const int32_t lx = static_cast<int32_t>(slot % kVoxelBlock);
            const int32_t ly = static_cast<int32_t>((slot / kVoxelBlock) % kVoxelBlock);
            const int32_t lz = static_cast<int32_t>(slot / (kVoxelBlock * kVoxelBlock));
            VoxelKey k;
            k.x = kv.first.x * static_cast<int32_t>(kVoxelBlock) + lx;
            k.y = kv.first.y * static_cast<int32_t>(kVoxelBlock) + ly;
            k.z = kv.first.z * static_cast<int32_t>(kVoxelBlock) + lz;
            keys.push_back(k);
            values.push_back(voxel);
        }
    }
    bool ok = true;
    const char magic[8] = {'A', 'R', 'G', 'T', 'M', '0', '0', '1'};
    ok = ok && std::fwrite(magic, 1, 8, f) == 8;
    const uint32_t version = kSaveVersion;
    ok = ok && std::fwrite(&version, sizeof(version), 1, f) == 1;
    ok = ok && std::fwrite(&p_.voxel_size, sizeof(p_.voxel_size), 1, f) == 1;
    const uint64_t frames = frames_;
    ok = ok && std::fwrite(&frames, sizeof(frames), 1, f) == 1;
    const uint64_t n = keys.size();
    ok = ok && std::fwrite(&n, sizeof(n), 1, f) == 1;
    for (size_t i = 0; ok && i < keys.size(); ++i) {
        ok = std::fwrite(&keys[i], sizeof(VoxelKey), 1, f) == 1 &&
             std::fwrite(&values[i], sizeof(Voxel), 1, f) == 1;
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
              std::fread(&version, sizeof(version), 1, f) == 1 &&
              std::fread(&voxel_size, sizeof(voxel_size), 1, f) == 1 &&
              std::fread(&frames, sizeof(frames), 1, f) == 1 &&
              std::fread(&n, sizeof(n), 1, f) == 1;
    if (!ok || voxel_size <= 0.0f) {
        std::fclose(f);
        return false;
    }
    BlockMap loaded;
    loaded.reserve(static_cast<size_t>(n) / kBlockVolume + 1);
    for (uint64_t i = 0; i < n; ++i) {
        VoxelKey k;
        Voxel voxel;
        if (std::fread(&k, sizeof(VoxelKey), 1, f) != 1 ||
            std::fread(&voxel, sizeof(Voxel), 1, f) != 1) {
            std::fclose(f);
            return false;
        }
        if (version >= 2) {
            voxel.free_score = voxel.free_observations == 0
                                   ? 0.0f
                                   : 1.0f - 1.0f / static_cast<float>(voxel.free_observations);
        }
        loaded[block_of(k)][static_cast<size_t>(index_in_block(k))] = voxel;
    }
    std::fclose(f);
    blocks_ = std::move(loaded);
    entries_.clear();
    free_dirty_.clear();
    p_.voxel_size = voxel_size;
    inv_voxel_ = 1.0f / voxel_size;
    frames_ = frames;
    return true;
}

} // namespace argus
