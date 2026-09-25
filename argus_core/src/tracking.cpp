
#include "argus_core/tracking.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace argus {

namespace {
constexpr int kStateDim = 6;
}

ObstacleTracker::ObstacleTracker(const TrackingParams& p) : p_(p) {}

uint32_t ObstacleTracker::required_hits_at(float nearest_range_m) const {
    const float r = std::max(0.0f, nearest_range_m);
    if (r <= p_.confirm_near_range_m) {
        return p_.min_hits_to_confirm;
    }
    if (r >= p_.confirm_far_range_m) {
        return p_.min_hits_to_confirm_far;
    }
    const float near = static_cast<float>(p_.min_hits_to_confirm);
    const float far = static_cast<float>(p_.min_hits_to_confirm_far);
    const float span = std::max(0.001f, p_.confirm_far_range_m - p_.confirm_near_range_m);
    const float t = (r - p_.confirm_near_range_m) / span;
    return static_cast<uint32_t>(std::lround(near + (far - near) * std::clamp(t, 0.0f, 1.0f)));
}

float compute_ttc(const Track& t, float train_speed_mps) {
    const float closing = train_speed_mps - t.state(3);
    if (closing <= 0.0f) {
        return -1.0f;
    }
    const float x = t.state(0);
    if (x <= 0.0f) {
        return -1.0f;
    }
    return x / closing;
}

std::vector<Track> ObstacleTracker::update(const std::vector<Cluster>& clusters, float dt,
                                           float train_speed_mps) {
    std::vector<bool> matched(clusters.size(), false);
    for (auto& tr : tracks_) {
        float best = p_.max_association_distance;
        int best_j = -1;
        for (size_t j = 0; j < clusters.size(); ++j) {
            if (matched[j]) continue;
            const Eigen::Vector3f c = clusters[j].centroid;
            const float d = (c.head<3>() - tr.state.head<3>()).norm();
            if (d < best) {
                best = d;
                best_j = static_cast<int>(j);
            }
        }
        if (best_j >= 0) {
            matched[best_j] = true;
            const Cluster& c = clusters[best_j];

            tr.state.head<3>() += tr.state.tail<3>() * dt;
            tr.state(3) = (c.centroid.x() - tr.state(0)) / std::max(dt, 1e-3f);
            tr.state(0) = c.centroid.x();
            tr.state(1) = c.centroid.y();
            tr.state(2) = c.centroid.z();

            tr.hits++;
            tr.misses = 0;
            tr.consecutive_misses = 0;
            tr.nearest_range = c.nearest_range;
            tr.confirmed = tr.confirmed || tr.hits >= required_hits_at(tr.nearest_range);
            tr.volume = c.volume;
            tr.extent = c.max_corner - c.min_corner;
            tr.point_count = c.point_count;
            tr.score = c.score;
            tr.votes_free_space = c.votes_free_space;
            tr.votes_no_return = c.votes_no_return;
            tr.votes_geometry = c.votes_geometry;
            tr.ttc = compute_ttc(tr, train_speed_mps);
        } else {
            tr.misses++;
            tr.consecutive_misses++;
            tr.state.head<3>() += tr.state.tail<3>() * dt;
        }
    }

    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                 [&](const Track& t) { return t.misses > p_.max_misses_to_keep; }),
                  tracks_.end());

    for (size_t j = 0; j < clusters.size(); ++j) {
        if (matched[j]) continue;
        Track t;
        t.id = next_id_++;
        t.state.setZero();
        t.state.head<3>() = clusters[j].centroid;
        t.covariance = Eigen::Matrix<float, 6, 6>::Identity() * p_.measurement_noise;
        t.hits = 1;
        t.misses = 0;
        t.consecutive_misses = 0;
        t.nearest_range = clusters[j].nearest_range;
        t.confirmed = t.hits >= required_hits_at(t.nearest_range);
        t.volume = clusters[j].volume;
        t.extent = clusters[j].max_corner - clusters[j].min_corner;
        t.point_count = clusters[j].point_count;
        t.score = clusters[j].score;
        t.votes_free_space = clusters[j].votes_free_space;
        t.votes_no_return = clusters[j].votes_no_return;
        t.votes_geometry = clusters[j].votes_geometry;
        tracks_.push_back(t);
    }

    std::vector<Track> confirmed;
    for (const auto& t : tracks_) {
        if (t.confirmed) {
            confirmed.push_back(t);
        }
    }
    return confirmed;
}

} // namespace argus
