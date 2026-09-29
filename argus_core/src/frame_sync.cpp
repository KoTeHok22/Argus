
#include "argus_core/frame_sync.hpp"

#include <cstdlib>

namespace argus {

FrameSynchronizer::FrameSynchronizer(const FrameSyncParams& p) : p_(p) {}

uint32_t FrameSynchronizer::required_mask() const {
    uint32_t mask = static_cast<uint32_t>(FrameChannel::cloud) |
                    static_cast<uint32_t>(FrameChannel::geometry) |
                    static_cast<uint32_t>(FrameChannel::no_return);
    if (p_.require_temporal) {
        mask |= static_cast<uint32_t>(FrameChannel::temporal);
    }
    return mask;
}

uint64_t FrameSynchronizer::add(FrameChannel ch, uint64_t stamp_ns) {
    ++stats_.arrivals;
    uint64_t key = stamp_ns;
    if (!slots_.empty()) {
        auto it = slots_.lower_bound(stamp_ns);
        uint64_t best_key = 0;
        uint64_t best_dist = UINT64_MAX;
        if (it != slots_.end()) {
            best_key = it->first;
            best_dist = it->first - stamp_ns;
        }
        if (it != slots_.begin()) {
            auto prev = std::prev(it);
            const uint64_t dist = stamp_ns - prev->first;
            if (dist < best_dist) {
                best_key = prev->first;
                best_dist = dist;
            }
        }
        if (best_dist <= p_.tolerance_ns) {
            key = best_key;
            if (key != stamp_ns) {
                ++stats_.snapped;
            }
        }
    }
    slots_[key] |= static_cast<uint32_t>(ch);
    while (slots_.size() > p_.max_pending) {
        expire_oldest();
    }
    return key;
}

bool FrameSynchronizer::complete(uint64_t key) const {
    const auto it = slots_.find(key);
    if (it == slots_.end()) {
        return false;
    }
    return (it->second & required_mask()) == required_mask();
}

void FrameSynchronizer::release(uint64_t key) {
    slots_.erase(key);
}

std::vector<uint64_t> FrameSynchronizer::take_expired() {
    std::vector<uint64_t> out;
    out.swap(expired_keys_);
    return out;
}

void FrameSynchronizer::expire_oldest() {
    if (slots_.empty()) {
        return;
    }
    const auto it = slots_.begin();
    const uint32_t mask = it->second;
    ++stats_.expired;
    if ((mask & static_cast<uint32_t>(FrameChannel::cloud)) == 0) {
        ++stats_.expired_missing_cloud;
    }
    if ((mask & static_cast<uint32_t>(FrameChannel::geometry)) == 0) {
        ++stats_.expired_missing_geometry;
    }
    if ((mask & static_cast<uint32_t>(FrameChannel::no_return)) == 0) {
        ++stats_.expired_missing_no_return;
    }
    if (p_.require_temporal && (mask & static_cast<uint32_t>(FrameChannel::temporal)) == 0) {
        ++stats_.expired_missing_temporal;
    }
    expired_keys_.push_back(it->first);
    slots_.erase(it);
}

} // namespace argus
