#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <pathforge/geometry/circle.hpp>
#include <pathforge/geometry/shapes.hpp>
#include <pathforge/geometry/track.hpp>
#include <pathforge/geometry/track_data.hpp>
#include <pathforge/geometry/width_profile.hpp>

using namespace pathforge::geometry;

TEST(WidthProfile, OpenPathLinearInterpolationAndHold) {
  const Path line = Path::create({{0, 0}, {10, 0}, {20, 0}, {30, 0}}, Closure::Open);
  // Knots at s = 0 and 30: linear 2 -> 5, vertices at s = 0, 10, 20, 30. Values are exact rationals;
  // rounding of (s - s0) / (s1 - s0) gives errors of a few ulp, so 1e-12 is ample.
  const auto w = sampleWidthProfile(line, {{0.0, 2.0}, {30.0, 5.0}});
  ASSERT_EQ(w.size(), 4u);
  const double expect[] = {2.0, 3.0, 4.0, 5.0};
  for (int i = 0; i < 4; ++i) EXPECT_NEAR(w[static_cast<std::size_t>(i)], expect[i], 1e-12);
  // Outside the knot range the nearest knot is held.
  const auto h = sampleWidthProfile(line, {{10.0, 3.0}, {20.0, 5.0}});
  EXPECT_NEAR(h[0], 3.0, 1e-12);
  EXPECT_NEAR(h[1], 3.0, 1e-12);
  EXPECT_NEAR(h[2], 5.0, 1e-12);
  EXPECT_NEAR(h[3], 5.0, 1e-12);
}

TEST(WidthProfile, SingleKnotGivesConstantWidth) {
  const Path c = makeCircle(10.0, 12);
  for (double w : sampleWidthProfile(c, {{0.0, 4.0}})) EXPECT_EQ(w, 4.0);
}

// Closed profile with knots (0, 4) and (L/2, 8) on a regular 8-gon: vertices sit at multiples of L/8.
// Interpolating periodically gives 4 + 4*(s / (L/2)) on the first half and 8 - 4*((s - L/2)/(L/2)) on the
// second half, i.e. 4, 5, 6, 7, 8, 7, 6, 5 - and the last vertex (5) continues to vertex 0 (4)
// through the seam with the same slope.
TEST(WidthProfile, ClosedProfileIsPeriodicAndContinuousAcrossTheSeam) {
  const Path c = makeCircle(10.0, 8);
  const double L = c.length();
  const auto w = sampleWidthProfile(c, {{0.0, 4.0}, {L / 2.0, 8.0}});
  const double expect[] = {4, 5, 6, 7, 8, 7, 6, 5};
  for (int i = 0; i < 8; ++i) EXPECT_NEAR(w[static_cast<std::size_t>(i)], expect[i], 1e-9);
  EXPECT_NEAR(w.back() - 4.0, 1.0, 1e-9);  // last -> first step equals the other steps (|dw| = 1)
}

TEST(WidthProfile, ClosedProfileWrapsBetweenLastAndFirstKnot) {
  // Knots at 25 % and 75 % of the length: vertex 0 (s = 0) lies in the wrap interval [75 %, 125 %].
  const Path c = makeCircle(10.0, 8);
  const double L = c.length();
  const auto w = sampleWidthProfile(c, {{0.25 * L, 2.0}, {0.75 * L, 6.0}});
  // The wrap interval runs from s = 0.75 L (6) to s = 1.25 L (2): vertex 7 (0.875 L) -> 5, vertex 0 (L) -> 4,
  // vertex 1 (1.125 L) -> 3.
  EXPECT_NEAR(w[0], 4.0, 1e-9);
  EXPECT_NEAR(w[1], 3.0, 1e-9);
  EXPECT_NEAR(w[2], 2.0, 1e-9);
  EXPECT_NEAR(w[6], 6.0, 1e-9);
  EXPECT_NEAR(w[7], 5.0, 1e-9);
}

TEST(WidthProfile, RejectsInvalidKnots) {
  const Path closed = makeCircle(10.0, 12);
  const Path open = Path::create({{0, 0}, {10, 0}}, Closure::Open);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(sampleWidthProfile(open, {}), GeometryError);
  EXPECT_THROW(sampleWidthProfile(open, {{5.0, 1.0}, {5.0, 2.0}}), GeometryError);  // not strictly increasing
  EXPECT_THROW(sampleWidthProfile(open, {{6.0, 1.0}, {5.0, 2.0}}), GeometryError);
  EXPECT_THROW(sampleWidthProfile(open, {{0.0, -1.0}}), GeometryError);
  EXPECT_THROW(sampleWidthProfile(open, {{nan, 1.0}}), GeometryError);
  EXPECT_THROW(sampleWidthProfile(open, {{0.0, nan}}), GeometryError);
  EXPECT_THROW(sampleWidthProfile(open, {{11.0, 1.0}}), GeometryError);  // beyond the path
  EXPECT_NO_THROW(sampleWidthProfile(open, {{10.0, 1.0}}));              // end of an open path is allowed
  EXPECT_THROW(sampleWidthProfile(closed, {{closed.length(), 1.0}}), GeometryError);  // s == L is the seam
}

TEST(VariableWidthTrack, PerVertexWidthsOffsetEachVertexIndividually) {
  const double r = 100.0;
  const std::size_t n = 360;
  const Path c = makeCircle(r, n);
  std::vector<double> left(n), right(n);
  for (std::size_t i = 0; i < n; ++i) {
    left[i] = 3.0 + 0.01 * static_cast<double>(i);  // 3 .. 6.59
    right[i] = 5.0 + 0.5 * std::sin(0.1 * static_cast<double>(i));
  }
  const Track t = Track::create(c, left, right);
  EXPECT_FALSE(t.hasConstantWidths());
  // CCW circle: left boundary radius R - w_left, right boundary radius R + w_right (vertex normals of a
  // regular polygon are exactly radial; rounding only).
  for (std::size_t i = 0; i < n; ++i) {
    EXPECT_NEAR(norm(t.leftBoundary()[i]), r - left[i], 1e-9) << i;
    EXPECT_NEAR(norm(t.rightBoundary()[i]), r + right[i], 1e-9) << i;
  }
  // min/max of the per-vertex sums (not the sum of the minima).
  double lo = left[0] + right[0], hi = lo;
  for (std::size_t i = 0; i < n; ++i) {
    lo = std::min(lo, left[i] + right[i]);
    hi = std::max(hi, left[i] + right[i]);
  }
  EXPECT_DOUBLE_EQ(t.minTotalWidth(), lo);
  EXPECT_DOUBLE_EQ(t.maxTotalWidth(), hi);
  EXPECT_GT(t.maxTotalWidth(), t.minTotalWidth());
  EXPECT_NEAR(t.leftWidth(), 3.0 + 0.01 * 179.5, 1e-9);  // mean of 3 + 0.01 i
}

TEST(VariableWidthTrack, ConstantVectorsAreEquivalentToConstantFactory) {
  const Path c = makeStadium(300.0, 120.0, 256);
  const Track a = Track::create(c, 5.0, 4.0);
  const Track b = Track::create(c, std::vector<double>(256, 5.0), std::vector<double>(256, 4.0));
  EXPECT_TRUE(a.hasConstantWidths());
  EXPECT_TRUE(b.hasConstantWidths());
  EXPECT_EQ(a.leftBoundary(), b.leftBoundary());
  EXPECT_EQ(a.rightBoundary(), b.rightBoundary());
  EXPECT_EQ(b.leftWidth(), 5.0);  // exactly the constant, not a rounded mean
  EXPECT_EQ(b.rightWidth(), 4.0);
}

TEST(VariableWidthTrack, RejectsSizeMismatchAndInvalidEntries) {
  const Path c = makeCircle(10.0, 8);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(Track::create(c, std::vector<double>(7, 1.0), std::vector<double>(8, 1.0)), GeometryError);
  EXPECT_THROW(Track::create(c, std::vector<double>(8, 1.0), std::vector<double>(9, 1.0)), GeometryError);
  std::vector<double> bad(8, 1.0);
  bad[3] = -0.1;
  EXPECT_THROW(Track::create(c, bad, std::vector<double>(8, 1.0)), GeometryError);
  bad[3] = nan;
  EXPECT_THROW(Track::create(c, std::vector<double>(8, 1.0), bad), GeometryError);
  bad[3] = std::numeric_limits<double>::infinity();
  EXPECT_THROW(Track::create(c, bad, bad), GeometryError);
}

TEST(VariableWidthTrack, ProfileFeedsTrackAndBoundariesRegenerate) {
  const Path c = makeCircle(100.0, 360);
  const double L = c.length();
  const auto w = sampleWidthProfile(c, {{0.0, 4.0}, {L / 2.0, 8.0}});
  const Track t = Track::create(c, w, w);
  // Closed-loop continuity: neighbouring offset points across the seam are as close as elsewhere.
  // The width changes by < 0.03 m per vertex, so the step across the seam must match the steps on both
  // sides of it to within a few percent (a width discontinuity at the seam would show up here).
  const auto& lb = t.leftBoundary();
  const double seamStep = distance(lb.back(), lb.front());
  const double before = distance(lb[lb.size() - 2], lb.back());
  const double after = distance(lb[0], lb[1]);
  EXPECT_NEAR(seamStep / before, 1.0, 0.01);
  EXPECT_NEAR(seamStep / after, 1.0, 0.01);
  // Regeneration: a new width profile gives new boundaries, the centerline is unchanged.
  const Track wider = Track::create(c, 2.0 * 4.0, 8.0);
  EXPECT_NE(wider.leftBoundary(), Track::create(c, 4.0, 4.0).leftBoundary());
  EXPECT_EQ(wider.centerline().points(), c.points());
  // First half has smaller width than the second: boundary radius differences.
  EXPECT_GT(norm(t.leftBoundary()[0]), norm(t.leftBoundary()[180]));  // 96 vs 92
}

TEST(TrackData, RoundTripsThroughTrack) {
  const Track t = Track::create(makeCircle(50.0, 64), 3.0, 4.0);
  const TrackData d = toTrackData(t, "circle");
  EXPECT_EQ(d.name, "circle");
  EXPECT_EQ(d.closure, Closure::Closed);
  EXPECT_EQ(d.centerline, t.centerline().points());
  const Track again = buildTrack(d);
  EXPECT_EQ(again.leftBoundary(), t.leftBoundary());
  EXPECT_EQ(again.rightBoundary(), t.rightBoundary());
  TrackData bad = d;
  bad.rightWidths.pop_back();
  EXPECT_THROW(buildTrack(bad), GeometryError);
}
