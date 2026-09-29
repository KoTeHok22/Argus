
#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

namespace argus {

enum class FrameChannel : uint32_t {
    cloud = 1u << 0,
    geometry = 1u << 1,
    no_return = 1u << 2,
    temporal = 1u << 3,
};

struct FrameSyncParams {
    uint64_t tolerance_ns = 20000000;
    size_t max_pending = 16;
    bool require_temporal = false;
};

struct FrameSyncStats {
    uint64_t arrivals = 0;
    uint64_t snapped = 0;
    uint64_t expired = 0;
    uint64_t expired_missing_cloud = 0;
    uint64_t expired_missing_geometry = 0;
    uint64_t expired_missing_no_return = 0;
    uint64_t expired_missing_temporal = 0;
};

class FrameSynchronizer {
public:
    explicit FrameSynchronizer(const FrameSyncParams& p);

    uint64_t add(FrameChannel ch, uint64_t stamp_ns);

    bool complete(uint64_t key) const;

    void release(uint64_t key);

    std::vector<uint64_t> take_expired();

    const FrameSyncStats& stats() const { return stats_; }

    size_t pending() const { return slots_.size(); }

private:
    uint32_t required_mask() const;
    void expire_oldest();

    FrameSyncParams p_;
    std::map<uint64_t, uint32_t> slots_;
    FrameSyncStats stats_;
    std::vector<uint64_t> expired_keys_;
};

} // namespace argus
