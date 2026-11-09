#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <pathforge/geometry/frames.hpp>
#include <pathforge/geometry/spline.hpp>

using namespace pathforge::geometry;

namespace {
constexpr double kPi = std::numbers::pi;

std::vector<Vec2> circleControl(double r, const std::vector<double>& angles) {
  std::vector<Vec2> c;
  for (double a : angles) c.push_back({r * std::cos(a), r * std::sin(a)});
  return c;
}

std::vector<double> uniformAngles(int m) {
  std::vector<double> a;
  for (int i = 0; i < m; ++i) a.push_back(2.0 * kPi * i / m);
  return a;
}

double maxRadialError(const Path& p, double r) {
  double e = 0.0;
  for (const Vec2& q : p.points()) e = std::max(e, std::abs(norm(q) - r));
  return e;
}
}  // namespace

TEST(Spline, InterpolatesControlPointsExactly) {
  const std::vector<Vec2> c{{0, 0}, {10, 5}, {18, -3}, {30, 2}, {41, 9}};
  for (Closure closure : {Closure::Open, Closure::Closed}) {
    SplineOptions o;
    o.samplesPerSegment = 7;
    const Path p = catmullRomSpline(c, closure, o);
    // Control points are emitted directly (u = 0), so equality is exact, not approximate.
    for (std::size_t i = 0; i < c.size(); ++i) EXPECT_EQ(p.points()[i * 7], c[i]) << "i=" << i;
  }
}

TEST(Spline, OutputSizesAndNoDuplicatedSeam) {
  const std::vector<Vec2> c{{0, 0}, {10, 0}, {10, 10}, {0, 10}};
  SplineOptions o;
  o.samplesPerSegment = 5;
  const Path open = catmullRomSpline(c, Closure::Open, o);
  const Path closed = catmullRomSpline(c, Closure::Closed, o);
  EXPECT_EQ(open.size(), 3u * 5u + 1u);
  EXPECT_EQ(closed.size(), 4u * 5u);
  EXPECT_TRUE(closed.isClosed());
  EXPECT_FALSE(open.isClosed());
  EXPECT_EQ(open.points().back(), c.back());
  EXPECT_NE(closed.points().back(), closed.points().front());  // seam not repeated
}

TEST(Spline, TwoControlPointsGiveStraightLine) {
  const Path p = catmullRomSpline({{0, 0}, {6, 8}}, Closure::Open, {10, 0.5});
  EXPECT_EQ(p.size(), 11u);
  EXPECT_NEAR(p.length(), 10.0, 1e-12);
  for (const Vec2& q : p.points()) EXPECT_NEAR(cross({6, 8}, q), 0.0, 1e-12);  // on the line
}

// Uniform Catmull-Rom reproduces quadratics, so position error is at least third order in the
// control spacing h (we observe fourth order on circles: ratios ~15-16). Third order means the
// error shrinks by >= 2^3 = 8 per halving of h; we require > 7 to leave room for higher-order terms.
TEST(Spline, CircleConvergesWithControlDensity) {
  const double r = 100.0;
  double prev = 0.0;
  for (int m : {8, 16, 32, 64}) {
    SplineOptions o;
    o.samplesPerSegment = 64;
    const Path p = catmullRomSpline(circleControl(r, uniformAngles(m)), Closure::Closed, o);
    const double e = maxRadialError(p, r);
    if (prev > 0.0) EXPECT_GT(prev / e, 7.0) << "m=" << m;
    prev = e;
    // Length is underestimated (polyline of an inscribed-like curve) and converges too.
    EXPECT_LT(p.length(), 2.0 * kPi * r * (1.0 + 1e-6));
  }
  // Absolute accuracy for 32 control points: observed 3.5e-3 m (3.5e-5 R); bound 1e-4 R.
  SplineOptions o;
  o.samplesPerSegment = 64;
  EXPECT_LT(maxRadialError(catmullRomSpline(circleControl(r, uniformAngles(32)), Closure::Closed, o), r),
            1e-4 * r);
}

TEST(Spline, ClosedSeamIsSmooth) {
  // Control points are uniform on a circle, so the spline is invariant under rotation by one
  // control interval (32 samples): frames at index i and i + 32 must agree, INCLUDING across the
  // wrap-around seam. Differences are rounding only; 1e-9 is generous.
  const double r = 50.0;
  const std::size_t s = 32;
  SplineOptions o;
  o.samplesPerSegment = s;
  const Path p = catmullRomSpline(circleControl(r, uniformAngles(32)), Closure::Closed, o);
  const PathFrames f = computeFrames(p);
  const std::size_t n = p.size();
  for (std::size_t i = 0; i < n; ++i) {
    const std::size_t j = (i + s) % n;  // wraps over the seam for the last interval
    // Compare in the rotated frame: curvature is rotation invariant.
    EXPECT_NEAR(f.curvature[i], f.curvature[j], 1e-9) << "i=" << i;
  }
  // Tangent is C1 across the seam and close to the circle's: direction error << 1e-3 (spline radial
  // error ~3.5e-5 R, samples 6e-3 rad apart).
  for (std::size_t i : {std::size_t{0}, std::size_t{1}, n - 1, n - 2}) {
    const Vec2 radial = p.points()[i] * (1.0 / norm(p.points()[i]));
    EXPECT_NEAR(f.tangent[i].x, -radial.y, 1e-3) << "i=" << i;
    EXPECT_NEAR(f.tangent[i].y, radial.x, 1e-3) << "i=" << i;
  }
  // Discrete curvature of dense spline samples is noisy at O(spline error / spacing^2): observed
  // <= 2.7% of 1/R at every sample; bound 5%.
  for (std::size_t i = 0; i < n; ++i) EXPECT_NEAR(f.curvature[i] * r, 1.0, 5e-2) << "i=" << i;
}

TEST(Spline, NonUniformControlSpacing) {
  const double r = 100.0;
  // Gaps between 0.2 and 0.9 rad (ratio 4.5:1).
  const std::vector<double> a{0.0, 0.2, 0.6, 1.4, 1.8, 2.3, 3.1, 4.0, 4.4, 5.3, 5.8};
  const auto c = circleControl(r, a);
  SplineOptions o;
  o.samplesPerSegment = 32;
  const Path p = catmullRomSpline(c, Closure::Closed, o);
  for (std::size_t i = 0; i < c.size(); ++i) EXPECT_EQ(p.points()[i * 32], c[i]);
  // Centripetal CR stays near the circle despite uneven spacing: empirical bound 5% of R
  // (observed ~1%; no analytic bound is claimed for arbitrary spacing).
  EXPECT_LT(maxRadialError(p, r), 0.05 * r);
  // alpha is actually used: uniform parameterisation gives a different curve through the same points.
  o.alpha = 0.0;
  const Path u = catmullRomSpline(c, Closure::Closed, o);
  EXPECT_NE(u.points()[5], p.points()[5]);
  for (std::size_t i = 0; i < c.size(); ++i) EXPECT_EQ(u.points()[i * 32], c[i]);
}

// Very uneven spacing (two 100 m gaps around a 5 m gap): the centripetal spline must not reverse
// direction anywhere (no cusp or loop). Consecutive chord directions stay within a few degrees:
// observed minimum cosine of the turning angle 0.9991; require > 0.99.
// Note: overshoot beyond the control polygon does occur (observed 2.7 m in y); it is a documented
// limitation, not something the centripetal scheme removes.
TEST(Spline, CentripetalHasNoCuspOnVeryUnevenSpacing) {
  const std::vector<Vec2> c{{0, 0}, {100, 0}, {101, 5}, {201, 5}};
  SplineOptions o;
  o.samplesPerSegment = 200;
  const Path p = catmullRomSpline(c, Closure::Open, o);
  const auto& q = p.points();
  double minCos = 2.0;
  for (std::size_t i = 1; i + 1 < q.size(); ++i) {
    const Vec2 a = q[i] - q[i - 1], b = q[i + 1] - q[i];
    minCos = std::min(minCos, dot(a, b) / (norm(a) * norm(b)));
  }
  EXPECT_GT(minCos, 0.99);
  double ylo = 0.0;
  for (const Vec2& v : q) ylo = std::min(ylo, v.y);
  EXPECT_LT(ylo, 0.0);  // documents the overshoot below the control polygon
}

TEST(Spline, IsDeterministic) {
  const std::vector<Vec2> c{{0, 0}, {10, 5}, {18, -3}, {30, 2}};
  EXPECT_EQ(catmullRomSpline(c, Closure::Closed).points(), catmullRomSpline(c, Closure::Closed).points());
}

TEST(Spline, RejectsInvalidInput) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_THROW(catmullRomSpline({}, Closure::Open), GeometryError);
  EXPECT_THROW(catmullRomSpline({{0, 0}}, Closure::Open), GeometryError);
  EXPECT_THROW(catmullRomSpline({{0, 0}, {1, 0}}, Closure::Closed), GeometryError);
  EXPECT_THROW(catmullRomSpline({{0, 0}, {nan, 0}, {1, 1}}, Closure::Open), GeometryError);
  EXPECT_THROW(catmullRomSpline({{0, 0}, {inf, 0}, {1, 1}}, Closure::Open), GeometryError);
  EXPECT_THROW(catmullRomSpline({{0, 0}, {0, 0}, {1, 1}}, Closure::Open), GeometryError);  // duplicate
  EXPECT_THROW(catmullRomSpline({{0, 0}, {1, 0}, {1, 1}, {0, 0}}, Closure::Closed), GeometryError);  // seam
  const std::vector<Vec2> ok{{0, 0}, {1, 0}, {1, 1}};
  EXPECT_THROW(catmullRomSpline(ok, Closure::Open, {0, 0.5}), GeometryError);
  EXPECT_THROW(catmullRomSpline(ok, Closure::Open, {100001, 0.5}), GeometryError);
  EXPECT_THROW(catmullRomSpline(ok, Closure::Open, {4, -0.1}), GeometryError);
  EXPECT_THROW(catmullRomSpline(ok, Closure::Open, {4, 1.1}), GeometryError);
  EXPECT_THROW(catmullRomSpline(ok, Closure::Open, {4, nan}), GeometryError);
}
