#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <pathforge/geometry/circle.hpp>
#include <pathforge/geometry/resample.hpp>

using namespace pathforge::geometry;

namespace {
constexpr double kPi = std::numbers::pi;

struct Spread {
  double min, max;
};
Spread chordRange(const Path& p) {
  const auto& q = p.points();
  Spread s{1e300, 0.0};
  for (std::size_t i = 0; i < p.segmentCount(); ++i) {
    const double d = distance(q[i], q[(i + 1) % q.size()]);
    s.min = std::min(s.min, d);
    s.max = std::max(s.max, d);
  }
  return s;
}
}  // namespace

// Input: dense circle polygon (N = 4096). For spacing h on a circle of radius R the chord is
// 2R sin(h / 2R) = h (1 - theta^2/24 + ...) with theta = h/R, so chord/arc is short by
// (h/R)^2 / 24 = 1.04e-4 at h = 5, R = 100; the dense input adds ~1e-7. Tolerance 2e-4 relative.
TEST(Resample, ClosedCircleHasUniformSpacing) {
  const double r = 100.0;
  const Path dense = makeCircle(r, 4096);
  const ResampleResult res = resampleByArcLength(dense, 5.0);
  const Spread s = chordRange(res.path);
  EXPECT_NEAR(s.min / res.spacing, 1.0, 2e-4);
  EXPECT_NEAR(s.max / res.spacing, 1.0, 2e-4);
  EXPECT_LE(s.max / s.min, 1.0 + 1e-6);  // all chords agree with each other to the rounding of the input
  // Total length consistency: resampled polygon is inscribed in the input, so slightly shorter.
  EXPECT_LE(res.path.length(), dense.length());
  EXPECT_NEAR(res.path.length() / dense.length(), 1.0, 2e-4);
}

TEST(Resample, SpacingAdjustmentIsBoundedByRounding) {
  // h = L / round(L / requested) => |h / requested - 1| <= 0.5 / N (N intervals).
  const Path dense = makeCircle(100.0, 4096);
  for (double req : {3.0, 7.3, 11.0, 49.9}) {
    const ResampleResult res = resampleByArcLength(dense, req);
    const double n = static_cast<double>(res.path.size());
    EXPECT_LE(std::abs(res.spacing / req - 1.0), 0.5 / n + 1e-12) << "req=" << req;
  }
}

TEST(Resample, ArcLengthIsStrictlyMonotonic) {
  const Path res = resampleByArcLength(makeCircle(30.0, 777), 1.7).path;
  const auto& s = res.arcLengths();
  EXPECT_DOUBLE_EQ(s.front(), 0.0);
  for (std::size_t i = 1; i < s.size(); ++i) EXPECT_GT(s[i], s[i - 1]);
}

TEST(Resample, ClosedSquareSeamAndPointsAreExact) {
  const Path sq = Path::create({{0, 0}, {4, 0}, {4, 4}, {0, 4}}, Closure::Closed);
  const Path r = resampleByArcLength(sq, 2.0).path;
  ASSERT_EQ(r.size(), 8u);  // perimeter 16 / 2; seam not duplicated
  EXPECT_TRUE(r.isClosed());
  const std::vector<Vec2> expect{{0, 0}, {2, 0}, {4, 0}, {4, 2}, {4, 4}, {2, 4}, {0, 4}, {0, 2}};
  for (std::size_t i = 0; i < 8; ++i) {
    EXPECT_NEAR(r.points()[i].x, expect[i].x, 1e-12) << i;
    EXPECT_NEAR(r.points()[i].y, expect[i].y, 1e-12) << i;
  }
  EXPECT_NEAR(r.length(), 16.0, 1e-12);  // closing segment (0,2)->(0,0) included
}

TEST(Resample, ClosedAlwaysKeepsAtLeastThreeSamples) {
  const Path sq = Path::create({{0, 0}, {4, 0}, {4, 4}, {0, 4}}, Closure::Closed);
  const ResampleResult r = resampleByArcLength(sq, 1000.0);
  EXPECT_EQ(r.path.size(), 3u);
  EXPECT_NEAR(r.spacing, 16.0 / 3.0, 1e-12);
}

TEST(Resample, OpenKeepsEndpointsAndCorners) {
  const Path l = Path::create({{0, 0}, {10, 0}, {10, 10}}, Closure::Open);
  const Path r = resampleByArcLength(l, 5.0).path;
  ASSERT_EQ(r.size(), 5u);
  EXPECT_FALSE(r.isClosed());
  EXPECT_EQ(r.points().front(), l.points().front());
  EXPECT_EQ(r.points().back(), l.points().back());
  const std::vector<Vec2> expect{{0, 0}, {5, 0}, {10, 0}, {10, 5}, {10, 10}};
  for (std::size_t i = 0; i < 5; ++i) {
    EXPECT_NEAR(r.points()[i].x, expect[i].x, 1e-12);
    EXPECT_NEAR(r.points()[i].y, expect[i].y, 1e-12);
  }
}

TEST(Resample, OpenSpacingRoundsToWholeIntervals) {
  const Path line = Path::create({{0, 0}, {10, 0}}, Closure::Open);
  const ResampleResult r = resampleByArcLength(line, 3.0);  // 10/3 = 3.33 -> 3 intervals
  EXPECT_EQ(r.path.size(), 4u);
  EXPECT_NEAR(r.spacing, 10.0 / 3.0, 1e-12);
  EXPECT_NEAR(r.path.points().back().x, 10.0, 0.0);
  EXPECT_EQ(resampleByArcLength(line, 1000.0).path.size(), 2u);  // endpoints only
}

// Resampling a dense circle with N intervals gives the inscribed N-gon, whose relative length
// error is 1 - sin(x)/x ~ x^2/6 with x = pi/N: second-order, ratio 4 per doubling of N. The
// requested spacing L/N makes the interval count exactly N. The dense-input error (~1e-7) is
// negligible against 1e-4 at N = 128, hence the narrow ratio window [3.9, 4.1] is justified.
TEST(Resample, LengthConvergesQuadraticallyWithResolution) {
  const double r = 100.0;
  const Path dense = makeCircle(r, 20000);
  const double exact = 2.0 * kPi * r;
  double prev = 0.0;
  for (int n : {16, 32, 64, 128}) {
    const Path p = resampleByArcLength(dense, dense.length() / n).path;
    ASSERT_EQ(p.size(), static_cast<std::size_t>(n));
    const double e = (exact - p.length()) / exact;
    const double x = kPi / n;
    // e / (x^2/6) = 1 - x^2/20 + x^4/840 - ... The input is itself a 20000-gon whose vertices the
    // output samples interpolate between (chords), which perturbs e by ~1e-8 absolute, i.e. up to
    // ~1e-4 relative at n = 128 (observed 7.7e-5). Tolerance 2e-4.
    EXPECT_NEAR(e / (x * x / 6.0), 1.0 - x * x / 20.0, 2e-4) << "n=" << n;
    if (prev > 0.0) {
      EXPECT_NEAR(prev / e, 4.0, 0.1) << "n=" << n;
    }
    prev = e;
  }
}

TEST(Resample, RejectsInvalidSpacing) {
  const Path p = makeCircle(10.0, 64);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(resampleByArcLength(p, 0.0), GeometryError);
  EXPECT_THROW(resampleByArcLength(p, -1.0), GeometryError);
  EXPECT_THROW(resampleByArcLength(p, nan), GeometryError);
  EXPECT_THROW(resampleByArcLength(p, std::numeric_limits<double>::infinity()), GeometryError);
  EXPECT_THROW(resampleByArcLength(p, 1e-12), GeometryError);  // > 1e8 samples
}
