#include <pathforge/geometry/path.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

using namespace pathforge::geometry;

// Axis-aligned unit-friendly geometry: lengths are sums of small integers, exact in
// binary floating point; 1e-12 only guards against platform libm differences in hypot.
constexpr double kTol = 1e-12;

TEST(Path, OpenPolylineLengthAndArcLength) {
  const Path p = Path::create({{0, 0}, {3, 4}, {3, 10}}, Closure::Open);
  EXPECT_FALSE(p.isClosed());
  EXPECT_EQ(p.segmentCount(), 2u);
  EXPECT_NEAR(p.length(), 11.0, kTol);
  ASSERT_EQ(p.arcLengths().size(), 3u);
  EXPECT_DOUBLE_EQ(p.arcLengths()[0], 0.0);
  EXPECT_NEAR(p.arcLengths()[1], 5.0, kTol);
  EXPECT_NEAR(p.arcLengths()[2], 11.0, kTol);
}

TEST(Path, ClosedPathIncludesClosingSegment) {
  const Path sq = Path::create({{0, 0}, {2, 0}, {2, 2}, {0, 2}}, Closure::Closed);
  EXPECT_TRUE(sq.isClosed());
  EXPECT_EQ(sq.segmentCount(), 4u);
  EXPECT_NEAR(sq.length(), 8.0, kTol);
  // arcLengths excludes the closing segment.
  EXPECT_NEAR(sq.arcLengths().back(), 6.0, kTol);
}

TEST(Path, ClosedLengthExceedsOpenLengthByClosingSegment) {
  const std::vector<Vec2> pts{{0, 0}, {4, 0}, {4, 3}};
  const double open = Path::create(pts, Closure::Open).length();
  const double closed = Path::create(pts, Closure::Closed).length();
  EXPECT_NEAR(closed - open, 5.0, kTol);  // hypotenuse of the 3-4-5 triangle
}

TEST(Path, RejectsEmptyAndTooFewPoints) {
  EXPECT_THROW(Path::create({}, Closure::Open), GeometryError);
  EXPECT_THROW(Path::create({}, Closure::Closed), GeometryError);
  EXPECT_THROW(Path::create({{0, 0}}, Closure::Open), GeometryError);
  EXPECT_THROW(Path::create({{0, 0}, {1, 0}}, Closure::Closed), GeometryError);
  EXPECT_NO_THROW(Path::create({{0, 0}, {1, 0}}, Closure::Open));
}

TEST(Path, RejectsNonFinite) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_THROW(Path::create({{0, 0}, {nan, 1}, {2, 2}}, Closure::Open), GeometryError);
  EXPECT_THROW(Path::create({{0, 0}, {1, inf}, {2, 2}}, Closure::Open), GeometryError);
  const auto v = validate({{0, 0}, {1, 1}, {inf, 0}}, Closure::Open);
  EXPECT_EQ(v.issue, PathIssue::NonFinitePoint);
  EXPECT_EQ(v.index, 2u);
}

TEST(Path, RejectsDuplicateAndZeroLengthSegments) {
  EXPECT_THROW(Path::create({{0, 0}, {0, 0}, {1, 1}}, Closure::Open), GeometryError);
  // Closed path whose last point repeats the first -> degenerate closing segment.
  const auto v = validate({{0, 0}, {1, 0}, {1, 1}, {0, 0}}, Closure::Closed);
  EXPECT_EQ(v.issue, PathIssue::DegenerateSegment);
  EXPECT_EQ(v.index, 3u);
  // The same points are fine as an open path (a closed-looking loop).
  EXPECT_TRUE(validate({{0, 0}, {1, 0}, {1, 1}, {0, 0}}, Closure::Open).ok());
}

TEST(Path, SegmentToleranceIsRespected) {
  const std::vector<Vec2> pts{{0, 0}, {1e-6, 0}, {1, 0}};
  EXPECT_TRUE(validate(pts, Closure::Open, 1e-9).ok());
  EXPECT_EQ(validate(pts, Closure::Open, 1e-3).issue, PathIssue::DegenerateSegment);
  EXPECT_THROW(Path::create(pts, Closure::Open, -1.0), GeometryError);
}

TEST(Path, CurvatureSignAndStraightLine) {
  // Straight line: zero curvature.
  const auto straight = Path::create({{0, 0}, {1, 0}, {2, 0}, {3, 0}}, Closure::Open).curvature();
  for (double k : straight) EXPECT_NEAR(k, 0.0, kTol);

  // Left turn (CCW) positive, right turn negative. Points (0,0),(1,0),(1,1) lie on a
  // circle of radius sqrt(2)/2 -> |kappa| = sqrt(2).
  const auto left = Path::create({{0, 0}, {1, 0}, {1, 1}}, Closure::Open).curvature();
  const auto right = Path::create({{0, 0}, {1, 0}, {1, -1}}, Closure::Open).curvature();
  EXPECT_NEAR(left[1], std::sqrt(2.0), 1e-12);
  EXPECT_NEAR(right[1], -std::sqrt(2.0), 1e-12);
  // Open-path endpoints copy the nearest interior value.
  EXPECT_DOUBLE_EQ(left[0], left[1]);
  EXPECT_DOUBLE_EQ(left[2], left[1]);
}

TEST(Path, StatsOfSquare) {
  const auto st = computeStats(Path::create({{0, 0}, {2, 0}, {2, 2}, {0, 2}}, Closure::Closed));
  EXPECT_EQ(st.pointCount, 4u);
  EXPECT_NEAR(st.length, 8.0, kTol);
  EXPECT_NEAR(st.minSegment, 2.0, kTol);
  EXPECT_NEAR(st.maxSegment, 2.0, kTol);
  // Every vertex of a square lies on the circumscribed circle R = sqrt(2) -> kappa = 1/sqrt(2).
  EXPECT_NEAR(st.minCurvature, 1.0 / std::sqrt(2.0), 1e-12);
  EXPECT_NEAR(st.maxCurvature, 1.0 / std::sqrt(2.0), 1e-12);
}
