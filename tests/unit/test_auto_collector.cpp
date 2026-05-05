#include <gtest/gtest.h>
#include "auto_collector.hpp"

using namespace qd::calibrate;

// Helper: create a chessboard corner vector for a given board size at a given
// image position. Corners are laid out row-major (OpenCV convention) with
// simple grid spacing.
static std::vector<cv::Point2f> make_corners(
    const cv::Size& board, double x0, double y0, double spacing
) {
    std::vector<cv::Point2f> corners;
    for (int r = 0; r < board.height; ++r) {
        for (int c = 0; c < board.width; ++c) {
            corners.emplace_back(
                static_cast<float>(x0 + c * spacing),
                static_cast<float>(y0 + r * spacing)
            );
        }
    }
    return corners;
}

class AutoCollectorTest : public ::testing::Test {
protected:
    cv::Size board_size_{8, 8};    // 8x8 inner corners
    cv::Size image_size_{1280, 720};

    AutoCollector::Params make_params(double x, double y, double size, double skew) {
        AutoCollector::Params p{};
        p.x = x;
        p.y = y;
        p.size = size;
        p.skew = skew;
        return p;
    }
};

// ---- compute_params ----

TEST_F(AutoCollectorTest, ComputeParamsSuccess) {
    AutoCollector ac(board_size_);
    auto corners = make_corners(board_size_, 200., 100., 40.);
    AutoCollector::Params out{};
    ASSERT_TRUE(ac.compute_params(corners, image_size_, out));
    // Params should be in [0, 1]
    EXPECT_GE(out.x, 0.0);
    EXPECT_LE(out.x, 1.0);
    EXPECT_GE(out.y, 0.0);
    EXPECT_LE(out.y, 1.0);
    EXPECT_GT(out.size, 0.0);
    EXPECT_LE(out.size, 1.0);
    EXPECT_GE(out.skew, 0.0);
    EXPECT_LE(out.skew, 1.0);
}

TEST_F(AutoCollectorTest, ComputeParamsWrongCornerCount) {
    AutoCollector ac(board_size_);
    std::vector<cv::Point2f> too_few = { { 10.f, 10.f }, { 20.f, 10.f } };
    AutoCollector::Params out{};
    EXPECT_FALSE(ac.compute_params(too_few, image_size_, out));
}

TEST_F(AutoCollectorTest, ComputeParamsZeroImageSize) {
    AutoCollector ac(board_size_);
    auto corners = make_corners(board_size_, 200., 100., 40.);
    AutoCollector::Params out{};
    EXPECT_FALSE(ac.compute_params(corners, { 0, 0 }, out));
}

// ---- is_good_sample: first sample always accepted ----

TEST_F(AutoCollectorTest, FirstSampleAlwaysGood) {
    AutoCollector ac(board_size_);
    auto corners = make_corners(board_size_, 200., 100., 40.);
    auto params = make_params(0.3, 0.4, 0.2, 0.1);
    EXPECT_TRUE(ac.is_good_sample(params, corners, {}));
}

// ---- is_good_sample: duplicate rejection ----

TEST_F(AutoCollectorTest, RejectsNearDuplicate) {
    AutoCollector ac(board_size_);
    auto corners = make_corners(board_size_, 200., 100., 40.);
    auto p1 = make_params(0.3, 0.4, 0.2, 0.1);
    ac.add_sample(p1);

    // nearly identical
    auto p2 = make_params(0.31, 0.40, 0.20, 0.10);
    EXPECT_FALSE(ac.is_good_sample(p2, corners, {}));
}

TEST_F(AutoCollectorTest, AcceptsFarEnoughSample) {
    AutoCollector ac(board_size_);
    auto corners = make_corners(board_size_, 200., 100., 40.);
    auto p1 = make_params(0.3, 0.4, 0.2, 0.1);
    ac.add_sample(p1);

    // L1 distance > default 0.2
    auto p2 = make_params(0.6, 0.7, 0.5, 0.3);
    EXPECT_TRUE(ac.is_good_sample(p2, corners, {}));
}

// ---- interval_ready ----

TEST_F(AutoCollectorTest, IntervalLimiting) {
    AutoCollector::Config cfg;
    cfg.min_interval_ms = 1000;  // 1 second
    AutoCollector ac(board_size_, cfg);
    auto corners = make_corners(board_size_, 200., 100., 40.);
    auto p1 = make_params(0.3, 0.4, 0.2, 0.1);

    // first sample ok
    EXPECT_TRUE(ac.is_good_sample(p1, corners, {}));
    ac.add_sample(p1);

    auto p2 = make_params(0.6, 0.7, 0.5, 0.3);
    // immediately after - should be blocked by interval
    EXPECT_FALSE(ac.is_good_sample(p2, corners, {}));
}

// ---- speed limiting ----

TEST_F(AutoCollectorTest, SpeedCheckBlocksFastMotion) {
    AutoCollector::Config cfg;
    cfg.max_chessboard_speed = 5.0;  // max 5 px/frame
    AutoCollector ac(board_size_, cfg);
    auto corners1 = make_corners(board_size_, 200., 100., 40.);
    auto corners2 = make_corners(board_size_, 300., 100., 40.);  // moved X by 100 px (ave. ~100/64 ≈ 1.5px, too fast)

    auto p1 = make_params(0.3, 0.4, 0.2, 0.1);
    ac.add_sample(p1);

    // corners2 are far from corners1, the average motion should be > 5.0
    auto p2 = make_params(0.6, 0.7, 0.5, 0.3);
    EXPECT_FALSE(ac.is_good_sample(p2, corners2, corners1));
}

TEST_F(AutoCollectorTest, SpeedCheckAllowsSlowMotion) {
    AutoCollector::Config cfg;
    cfg.max_chessboard_speed = 50.0;
    AutoCollector ac(board_size_, cfg);
    auto corners1 = make_corners(board_size_, 200., 100., 40.);
    auto corners2 = make_corners(board_size_, 202., 100., 40.);  // moved X by 2 px

    auto p1 = make_params(0.3, 0.4, 0.2, 0.1);
    ac.add_sample(p1);

    auto p2 = make_params(0.6, 0.7, 0.5, 0.3);
    EXPECT_TRUE(ac.is_good_sample(p2, corners2, corners1));
}

// ---- compute_progress / good_enough ----

TEST_F(AutoCollectorTest, ProgressEmpty) {
    AutoCollector ac(board_size_);
    auto progress = ac.compute_progress();
    EXPECT_EQ(progress.sample_count, 0);
    EXPECT_FALSE(progress.good_enough);
}

TEST_F(AutoCollectorTest, ProgressNotGoodEnough) {
    AutoCollector ac(board_size_);
    ac.add_sample(make_params(0.3, 0.4, 0.05, 0.0));
    ac.add_sample(make_params(0.5, 0.5, 0.10, 0.1));

    auto progress = ac.compute_progress();
    EXPECT_EQ(progress.sample_count, 2);
    EXPECT_FALSE(progress.good_enough);  // 2 < goodenough_samples (40)
}

TEST_F(AutoCollectorTest, GoodEnoughBySampleCount) {
    AutoCollector::Config cfg;
    cfg.goodenough_samples = 3;
    AutoCollector ac(board_size_, cfg);

    ac.add_sample(make_params(0.3, 0.4, 0.05, 0.1));
    ac.add_sample(make_params(0.5, 0.4, 0.05, 0.1));
    ac.add_sample(make_params(0.7, 0.4, 0.05, 0.1));

    auto progress = ac.compute_progress();
    EXPECT_EQ(progress.sample_count, 3);
    // 3 >= goodenough_samples (3), so good_enough even if coverage is low
    EXPECT_TRUE(progress.good_enough);
}

TEST_F(AutoCollectorTest, GoodEnoughByFullCoverage) {
    AutoCollector::Config cfg;
    cfg.goodenough_samples = 40;
    cfg.param_ranges = { 0.3, 0.3, 0.1, 0.2 };
    AutoCollector ac(board_size_, cfg);

    // cover full X/Y range (Size and Skew use min=0.0)
    ac.add_sample(make_params(0.0, 0.0, 0.0, 0.0));
    ac.add_sample(make_params(0.3, 0.3, 0.1, 0.2));

    auto progress = ac.compute_progress();
    EXPECT_EQ(progress.sample_count, 2);
    // all 4 dims should reach 1.0
    for (int i = 0; i < 4; ++i) {
        EXPECT_GE(progress.progress[i], 1.0) << "dim " << i;
    }
    EXPECT_TRUE(progress.good_enough);
}

// ---- reset ----

TEST_F(AutoCollectorTest, ResetClearsHistory) {
    AutoCollector ac(board_size_);
    ac.add_sample(make_params(0.3, 0.4, 0.2, 0.1));
    EXPECT_EQ(ac.sample_count(), 1);

    ac.reset();
    EXPECT_EQ(ac.sample_count(), 0);

    auto progress = ac.compute_progress();
    EXPECT_EQ(progress.sample_count, 0);
    EXPECT_FALSE(progress.good_enough);
}

// ---- set_enabled ----

TEST_F(AutoCollectorTest, SetEnabledResetsTimer) {
    AutoCollector::Config cfg;
    cfg.min_interval_ms = 2000;
    AutoCollector ac(board_size_, cfg);

    auto corners = make_corners(board_size_, 200., 100., 40.);
    auto p1 = make_params(0.3, 0.4, 0.2, 0.1);
    EXPECT_TRUE(ac.is_good_sample(p1, corners, {}));
    ac.add_sample(p1);

    auto p2 = make_params(0.6, 0.7, 0.5, 0.3);
    // immediately blocked by interval
    EXPECT_FALSE(ac.is_good_sample(p2, corners, {}));

    // disable then re-enable — this resets the timer
    ac.set_enabled(false);
    ac.set_enabled(true);
    EXPECT_TRUE(ac.is_good_sample(p2, corners, {}));
}
