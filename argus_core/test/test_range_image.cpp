
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "argus_core/range_image.hpp"

namespace {

argus::CleanCloud make_cloud(size_t n_points, uint32_t rings) {
    argus::CleanCloud c;
    c.rings = rings;
    for (size_t i = 0; i < n_points; ++i) {
        const float d = 10.0f + static_cast<float>(i % 7);
        c.x.push_back(d);
        c.y.push_back(0.0f);
        c.z.push_back(0.0f);
        c.intensity.push_back(1.0f);
        c.ring.push_back(static_cast<uint16_t>(i % rings));
        c.azimuth_idx.push_back(static_cast<uint32_t>(i / rings));
    }
    return c;
}

} // namespace

TEST(RangeImage, Shape) {
    const uint32_t rings = 4;
    auto cloud = make_cloud(4 * 10, rings);
    argus::RangeImageParams p;
    p.rings_fallback = rings;
    p.az_steps_fallback = 10;

    const argus::RangeImage ri = argus::build_range_image(cloud, p);
    EXPECT_EQ(ri.height, rings);
    EXPECT_EQ(ri.width, 10u);
    EXPECT_EQ(ri.range.size(), static_cast<size_t>(ri.width) * ri.height);
    EXPECT_TRUE(ri.valid());
}

TEST(RangeImage, OrderingRingEqualsIModRings) {
    const uint32_t rings = 8;
    auto cloud = make_cloud(rings * 5, rings);
    argus::RangeImageParams p;
    p.rings_fallback = rings;

    const argus::RangeImage ri = argus::build_range_image(cloud, p);
    for (uint32_t az = 0; az < ri.width; ++az) {
        for (uint32_t ring = 0; ring < ri.height; ++ring) {
            const size_t i = ri.idx(az, ring);
            const size_t src = static_cast<size_t>(az) * rings + ring;
            const float expected = cloud.x[src];
            EXPECT_NEAR(ri.range[i], expected, 1e-4f);
        }
    }
}

TEST(RangeImage, DetectRingsFindsPeriod) {
    const uint32_t rings = 16;
    auto cloud = make_cloud(rings * 4, rings);
    EXPECT_EQ(argus::detect_rings(cloud, 256), rings);
}

TEST(RangeImage, SparseValidPointsKeepPositionalRings) {
    argus::CleanCloud c;
    c.rings = 128;
    c.n_raw = 921600;
    for (uint32_t raw = 0; raw < c.n_raw; raw += 3) {
        c.x.push_back(10.0f);
        c.y.push_back(0.0f);
        c.z.push_back(0.0f);
        c.intensity.push_back(1.0f);
        c.ring.push_back(static_cast<uint16_t>(raw % 128));
        c.raw_idx.push_back(raw);
    }
    argus::RangeImageParams p;
    p.rings_fallback = 128;
    const argus::RangeImage ri = argus::build_range_image(c, p);
    EXPECT_EQ(ri.height, 128u);
    EXPECT_EQ(ri.width, 7200u);
}

TEST(RangeImage, Neighbors8WrapAroundAzimuth) {
    argus::RangeImage ri;
    ri.width = 2400;
    ri.height = 4;
    ri.range.assign(ri.width * ri.height, 1.0f);
    ri.intensity.assign(ri.width * ri.height, 0.0f);
    ri.no_return_mask.assign(ri.width * ri.height, 0);

    std::array<std::pair<uint32_t, uint32_t>, 8> nb{};
    const uint32_t n = argus::neighbors8(ri, 0, 1, nb);
    EXPECT_EQ(n, 8u);
    bool has_wrap = false;
    for (uint32_t k = 0; k < n; ++k) {
        if (nb[k].first == ri.width - 1) has_wrap = true;
    }
    EXPECT_TRUE(has_wrap);

    const uint32_t n0 = argus::neighbors8(ri, 0, 0, nb);
    EXPECT_EQ(n0, 5u);
    const uint32_t nlast = argus::neighbors8(ri, 0, ri.height - 1, nb);
    EXPECT_EQ(nlast, 5u);
}

TEST(RangeImage, MedianFilterKeepsFlatProfile) {
    argus::RangeImage ri;
    ri.width = 8;
    ri.height = 2;
    ri.range.assign(ri.width * ri.height, 25.5f);
    ri.intensity.assign(ri.width * ri.height, 0.0f);
    ri.no_return_mask.assign(ri.width * ri.height, 0);

    const argus::RangeImage med = argus::azimuth_median_filter(ri, 3);
    for (size_t i = 0; i < med.range.size(); ++i) {
        EXPECT_NEAR(med.range[i], 25.5f, 1e-5f);
    }
}

TEST(RangeImage, RawIdxMappingAndNoReturnMask) {
    argus::CleanCloud c;
    c.n_raw = 8;
    for (size_t k = 0; k < 2; ++k) {
        c.x.push_back(10.0f + static_cast<float>(k));
        c.y.push_back(0.0f);
        c.z.push_back(0.0f);
        c.intensity.push_back(2.0f);
    }
    c.raw_idx = {0, 5};
    c.no_return_raw = {1};

    argus::RangeImageParams p;
    p.rings_fallback = 4;
    const argus::RangeImage ri = argus::build_range_image(c, p);

    ASSERT_TRUE(ri.valid());
    EXPECT_EQ(ri.width, 2u);
    EXPECT_EQ(ri.height, 4u);

    EXPECT_FLOAT_EQ(ri.range[ri.idx(0, 0)], 10.0f);
    EXPECT_FLOAT_EQ(ri.range[ri.idx(1, 1)], 11.0f);
    EXPECT_TRUE(std::isnan(ri.range[ri.idx(0, 1)]));
    EXPECT_EQ(ri.no_return_mask[ri.idx(0, 1)], 1u);
    EXPECT_EQ(ri.no_return_mask[ri.idx(1, 0)], 0u);
}
