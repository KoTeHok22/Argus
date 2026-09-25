#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "cli_options.hpp"

namespace {

argus::cli::ParseResult parse(std::vector<std::string> args) {
    std::vector<char*> argv;
    argv.reserve(args.size());
    for (std::string& a : args) {
        argv.push_back(a.data());
    }
    return argus::cli::parse_cli(static_cast<int>(argv.size()), argv.data());
}

TEST(CliOptions, RequiresPath) {
    const auto r = parse({"offline_detector"});
    EXPECT_EQ(r.status, argus::cli::ParseStatus::Help);
}

TEST(CliOptions, PositionalPathIsTaken) {
    const auto r = parse({"offline_detector", "frames.bin", "--fusion"});
    ASSERT_EQ(r.status, argus::cli::ParseStatus::Ok);
    EXPECT_EQ(r.options.path, "frames.bin");
    EXPECT_TRUE(r.options.fusion_mode);
}

TEST(CliOptions, NumericOptionsParsed) {
    const auto r = parse({"offline_detector", "f.bin", "--frames", "200", "--gauge-height", "3.0",
                          "--min-cluster-size", "9"});
    ASSERT_EQ(r.status, argus::cli::ParseStatus::Ok);
    EXPECT_EQ(r.options.limit, 200u);
    EXPECT_FLOAT_EQ(r.options.gauge_height, 3.0f);
    EXPECT_EQ(r.options.min_cluster_size_near, 9u);
}

TEST(CliOptions, MinimumClampApplied) {
    const auto r =
        parse({"offline_detector", "f.bin", "--min-hits", "0", "--odom-iterations", "0"});
    ASSERT_EQ(r.status, argus::cli::ParseStatus::Ok);
    EXPECT_EQ(r.options.min_hits, 1u);
    EXPECT_EQ(r.options.odom_iterations, 1u);
}

TEST(CliOptions, OdomHeadingImpliesOdometry) {
    const auto r = parse({"offline_detector", "f.bin", "--odom-heading"});
    ASSERT_EQ(r.status, argus::cli::ParseStatus::Ok);
    EXPECT_TRUE(r.options.odometry_mode);
    EXPECT_TRUE(r.options.odom_heading);
}

TEST(CliOptions, TrackCorridorSetsWorldFlags) {
    const auto r = parse(
        {"offline_detector", "f.bin", "--track-corridor", "--track-centerline", "/tmp/c.csv"});
    ASSERT_EQ(r.status, argus::cli::ParseStatus::Ok);
    EXPECT_TRUE(r.options.world_residual);
    EXPECT_TRUE(r.options.world_component_mode);
    EXPECT_TRUE(r.options.track_corridor_mode);
    EXPECT_EQ(r.options.track_centerline_path, "/tmp/c.csv");
}

TEST(CliOptions, VerifyFlags) {
    const auto r = parse({"offline_detector", "f.bin", "--verify", "--verify-min-score", "0.5"});
    ASSERT_EQ(r.status, argus::cli::ParseStatus::Ok);
    EXPECT_TRUE(r.options.verify_clusters);
    EXPECT_FLOAT_EQ(r.options.verify_min_score, 0.5f);
}

TEST(CliOptions, UnknownOptionIsError) {
    const auto r = parse({"offline_detector", "f.bin", "--nope"});
    EXPECT_EQ(r.status, argus::cli::ParseStatus::Error);
}

TEST(CliOptions, MissingCenterlineArgumentIsError) {
    const auto r = parse({"offline_detector", "f.bin", "--track-centerline"});
    EXPECT_EQ(r.status, argus::cli::ParseStatus::Error);
}

} // namespace
