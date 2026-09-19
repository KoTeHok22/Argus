
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>

#include "argus_core/types.hpp"

namespace argus {

struct TunnelModelParams {
    bool enabled = true;
    float voxel_size = 0.20f;
    float free_space_margin = 0.15f;
    uint32_t min_observations = 3;
    uint32_t free_hits_to_clear = 2;
    float max_range = 150.0f;
    float carve_max_range = 90.0f;
    float carve_half_width_m = 0.0f;
    uint32_t carve_stride_az = 1;
    uint32_t carve_stride_ring = 1;
    float forget_ray_fraction = 0.02f;
    bool enable_decay = false;
    uint32_t decay_after_frames = 3000;
    float occupancy_hit_log_odds = 0.85f;
    float occupancy_miss_log_odds = -0.40f;
    float occupancy_min_log_odds = -4.0f;
    float occupancy_max_log_odds = 3.5f;
};

struct TunnelModelStats {
    uint32_t n_voxels = 0;
    uint32_t n_free = 0;
    uint32_t n_occupied = 0;
    float free_ratio = 0.0f;
    uint64_t frames_integrated = 0;
    bool ready = false;
};

struct FreeSpaceCheck {
    uint32_t empty = 0;
    uint32_t observed = 0;
    uint32_t unknown = 0;

    float violation_rate() const {
        return observed == 0 ? 0.0f : static_cast<float>(empty) / static_cast<float>(observed);
    }
};

class TunnelModel {
public:
    explicit TunnelModel(const TunnelModelParams& p);

    void integrate(const CleanCloud& cloud, const Eigen::Isometry3d& pose);

    void integrate(const CleanCloud& cloud, const Eigen::Isometry3d& pose, bool register_hits);

    bool violates_free_space(const Eigen::Vector3f& p, float& out_confidence) const;

    bool violates_free_space(const Eigen::Vector3f& p, float& out_confidence,
                             const Eigen::Isometry3d& pose) const;

    bool contradicts_free_space(const Eigen::Vector3f& p, float& out_confidence,
                                const Eigen::Isometry3d& pose, float min_confidence,
                                uint32_t min_free_observations) const;

    uint64_t frames_integrated() const { return frames_; }

    FreeSpaceCheck check(const CleanCloud& cloud, const Eigen::Isometry3d& pose,
                         float min_confidence) const;

    TunnelModelStats stats() const;

    size_t voxel_count() const { return occupied_voxel_count(); }

    bool ready() const;

    void export_occupancy(std::vector<Eigen::Vector3f>& pts, std::vector<float>& conf,
                          bool free_only) const;

    void clear();

    bool save(const std::string& path) const;

    bool load(const std::string& path);

    const TunnelModelParams& params() const { return p_; }

private:
    struct Voxel {
        float free_score = 0.0f;
        float occupancy_log_odds = 0.0f;
        uint32_t free_observations = 0;
        uint32_t occupied_observations = 0;
        uint32_t last_frame = 0;
        uint8_t occupancy_dirty = 0;
        uint8_t reserved[3] = {};
    };

    struct VoxelKey {
        int32_t x = 0;
        int32_t y = 0;
        int32_t z = 0;
        bool operator==(const VoxelKey& o) const { return x == o.x && y == o.y && z == o.z; }
    };

    static constexpr uint32_t kVoxelBlock = 8;
    static constexpr int32_t kBlockShift = 3;
    static constexpr uint32_t kSamplePeriod = 8;
    static constexpr uint32_t kSamplesPerBlock = 4;
    static constexpr uint32_t kBlockVolume = kVoxelBlock * kVoxelBlock * kVoxelBlock;

    struct BlockKey {
        int32_t x = 0;
        int32_t y = 0;
        int32_t z = 0;
        bool operator==(const BlockKey& o) const { return x == o.x && y == o.y && z == o.z; }
    };

    struct BlockKeyHash {
        size_t operator()(const BlockKey& k) const {
            return static_cast<size_t>(k.x) * 73856093u ^ static_cast<size_t>(k.y) * 19349663u ^
                   static_cast<size_t>(k.z) * 83492791u;
        }
    };

    using Block = std::array<Voxel, kBlockVolume>;
    using BlockMap = std::unordered_map<BlockKey, Block, BlockKeyHash>;

    struct Entry {
        uintptr_t key = 0;
        Voxel* voxel = nullptr;
        uint32_t new_observations = 0;
    };

    struct GridMark {
        uint32_t frame = 0;
        uint32_t generation = 0;
    };

    static_assert(sizeof(Voxel) == 24, "Voxel layout is serialized by TunnelModel::save");

    static BlockKey block_of(const VoxelKey& k);

    static int32_t index_in_block(const VoxelKey& k);

    VoxelKey key_at(const Eigen::Vector3f& point) const;

    VoxelKey key_at(const Eigen::Vector3f& point, const Eigen::Isometry3d& pose) const;

    Eigen::Vector3f center_of(const VoxelKey& k) const;

    Eigen::Vector3f to_lidar(const Eigen::Vector3f& world, const Eigen::Isometry3d& pose) const;

    Eigen::Vector3f to_world(const Eigen::Vector3f& lidar, const Eigen::Isometry3d& pose) const;

    Voxel* find_voxel(const VoxelKey& k);

    const Voxel* find_voxel(const VoxelKey& k) const;

    Voxel& ensure_voxel(const VoxelKey& k);

    size_t occupied_voxel_count() const;

    void rebuild_grid();

    void carve_free_space(const Eigen::Vector3f& start, const Eigen::Vector3f& end,
                          const VoxelKey& stop_key);

    bool carve_ray_selected(const CleanCloud& cloud, size_t i) const;

    TunnelModelParams p_;
    float inv_voxel_ = 5.0f;
    uint64_t frames_ = 0;
    uint64_t cells_carved_ = 0;
    BlockMap blocks_;
    std::unordered_map<uintptr_t, Entry> entries_;
    std::vector<Voxel*> free_dirty_;
    std::unordered_map<BlockKey, GridMark, BlockKeyHash> grid_marks_;
    uint32_t sample_cursor_ = 0;
};

} // namespace argus
