#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <pathforge/geometry/cubic_spline.hpp>

using namespace pathforge::geometry;

namespace {
constexpr double kPi = std::numbers::pi;

// Deterministic pseudo-random control points (LCG), unevenly spaced on purpose.
std::vector<Vec2> scatter(std::size_t n, double span, unsigned seed) {
  std::vector<Vec2> p;
  unsigned s = seed;
  auto next = [&] {
    s = s * 1664525u + 1013904223u;
    return static_cast<double>(s >> 8) / static_cast<double>(1u << 24);  // [0, 1)
  };
  double x = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    x += span * (0.2 + 1.6 * next());  // spacing ratios up to 8:1
    p.push_back({x, span * (next() - 0.5) * 3.0});
  }
  return p;
}

std::vector<Vec2> circlePoints(double r, int m) {
  std::vector<Vec2> c;
  for (int i = 0; i < m; ++i) c.push_back({r * std::cos(2 * kPi * i / m), r * std::sin(2 * kPi * i / m)});
  return c;
}

// Dense Gaussian elimination with partial pivoting: an independent reference for the spline's linear system.
std::vector<double> denseSolve(std::vector<std::vector<double>> a, std::vector<double> b) {
  const std::size_t n = b.size();
  for (std::size_t c = 0; c < n; ++c) {
    std::size_t piv = c;
    for (std::size_t r = c + 1; r < n; ++r) {
      if (std::abs(a[r][c]) > std::abs(a[piv][c])) piv = r;
    }
    std::swap(a[c], a[piv]);
    std::swap(b[c], b[piv]);
    for (std::size_t r = c + 1; r < n; ++r) {
      const double f = a[r][c] / a[c][c];
      for (std::size_t k = c; k < n; ++k) a[r][k] -= f * a[c][k];
      b[r] -= f * b[c];
    }
  }
  std::vector<double> x(n);
  for (std::size_t r = n; r-- > 0;) {
    double s = b[r];
    for (std::size_t k = r + 1; k < n; ++k) s -= a[r][k] * x[k];
    x[r] = s / a[r][r];
  }
  return x;
}

std::vector<Vec2> referenceSecondDerivatives(const std::vector<Vec2>& p, Closure closure, double alpha) {
  const std::size_t n = p.size();
  const bool closed = closure == Closure::Closed;
  const std::size_t segs = closed ? n : n - 1;
  std::vector<double> h(segs);
  for (std::size_t i = 0; i < segs; ++i) h[i] = std::pow(distance(p[i], p[(i + 1) % n]), alpha);
  auto delta = [&](std::size_t i, int coord) {
    const Vec2 d = p[(i + 1) % n] - p[i];
    return (coord == 0 ? d.x : d.y) / h[i];
  };
  std::vector<Vec2> m(n, Vec2{0, 0});
  for (int coord = 0; coord < 2; ++coord) {
    std::vector<std::vector<double>> a(n, std::vector<double>(n, 0.0));
    std::vector<double> rhs(n, 0.0);
    if (closed) {
      for (std::size_t i = 0; i < n; ++i) {
        const std::size_t prev = (i + n - 1) % n, next = (i + 1) % n;
        a[i][prev] += h[prev];
        a[i][i] += 2.0 * (h[prev] + h[i]);
        a[i][next] += h[i];
        rhs[i] = 6.0 * (delta(i, coord) - delta(prev, coord));
      }
    } else {
      a[0][0] = 1.0;  // natural ends
      a[n - 1][n - 1] = 1.0;
      for (std::size_t i = 1; i + 1 < n; ++i) {
        a[i][i - 1] = h[i - 1];
        a[i][i] = 2.0 * (h[i - 1] + h[i]);
        a[i][i + 1] = h[i];
        rhs[i] = 6.0 * (delta(i, coord) - delta(i - 1, coord));
      }
    }
    const auto x = denseSolve(a, rhs);
    for (std::size_t i = 0; i < n; ++i) (coord == 0 ? m[i].x : m[i].y) = x[i];
  }
  return m;
}

void expectNearRel(Vec2 a, Vec2 b, double rel, const char* what) {
  const double scale = std::max({1.0, norm(a), norm(b)});
  EXPECT_NEAR(a.x, b.x, rel * scale) << what;
  EXPECT_NEAR(a.y, b.y, rel * scale) << what;
}
}  // namespace

TEST(CubicSpline, InterpolatesControlPointsExactly) {
  for (Closure closure : {Closure::Open, Closure::Closed}) {
    for (double alpha : {0.0, 0.5, 1.0}) {
      const auto pts = scatter(9, 10.0, 7);
      const CubicSpline s = CubicSpline::create(pts, closure, alpha);
      for (std::size_t i = 0; i < s.segmentCount(); ++i)
        EXPECT_EQ(s.evaluate(i, 0.0).position, pts[i]);  // exact
      if (closure == Closure::Open) {
        EXPECT_EQ(s.evaluate(s.segmentCount() - 1, 1.0).position, pts.back());
      } else {
        EXPECT_EQ(s.evaluate(s.segmentCount() - 1, 1.0).position, pts.front());  // closing segment ends at P0
      }
      // Evaluation at the global knot parameter agrees (to rounding of the segment lookup).
      for (std::size_t i = 0; i < pts.size(); ++i) {
        expectNearRel(s.evaluateAt(s.knot(i)).position, pts[i], 1e-12, "evaluateAt at knot");
      }
    }
  }
}

// The second derivatives must solve the textbook linear system. The reference solves it with DENSE
// elimination and partial pivoting (a different algorithm), so the tridiagonal / Sherman-Morrison code is
// checked independently.
TEST(CubicSpline, SecondDerivativesMatchDenseSolveOfTheNormalEquations) {
  for (Closure closure : {Closure::Open, Closure::Closed}) {
    for (double alpha : {0.0, 0.5, 1.0}) {
      for (std::size_t n : {3u, 4u, 7u, 25u}) {
        const auto pts = scatter(n, 3.0, static_cast<unsigned>(n * 13));
        const CubicSpline s = CubicSpline::create(pts, closure, alpha);
        const auto ref = referenceSecondDerivatives(pts, closure, alpha);
        double scale = 0.0;
        for (const Vec2& m : ref) scale = std::max(scale, norm(m));
        for (std::size_t i = 0; i < n; ++i) {
          EXPECT_NEAR(s.knotSecondDerivatives()[i].x, ref[i].x, 1e-10 * std::max(1.0, scale))
              << n << " " << i;
          EXPECT_NEAR(s.knotSecondDerivatives()[i].y, ref[i].y, 1e-10 * std::max(1.0, scale))
              << n << " " << i;
        }
      }
    }
  }
}

// C2 continuity: position, first AND second derivative agree on both sides of every interior knot (open) and
// of every knot including the seam (closed). Differences are rounding only (a few ulp of values of order
// 1-100).
TEST(CubicSpline, IsC2AtEveryInteriorKnotAndAcrossTheClosedSeam) {
  for (Closure closure : {Closure::Open, Closure::Closed}) {
    const auto pts = scatter(12, 5.0, 99);
    const CubicSpline s = CubicSpline::create(pts, closure, 0.5);
    const std::size_t segs = s.segmentCount();
    const std::size_t pairs = closure == Closure::Closed ? segs : segs - 1;
    for (std::size_t i = 0; i < pairs; ++i) {
      const auto left = s.evaluate(i, 1.0);
      const auto right = s.evaluate((i + 1) % segs, 0.0);
      expectNearRel(left.position, right.position, 1e-12, "position");
      expectNearRel(left.d1, right.d1, 1e-11, "first derivative");
      expectNearRel(left.d2, right.d2, 1e-10, "second derivative");
    }
  }
}

TEST(CubicSpline, NaturalEndsHaveZeroSecondDerivative) {
  const auto pts = scatter(8, 4.0, 5);
  const CubicSpline s = CubicSpline::create(pts, Closure::Open, 0.5);
  EXPECT_EQ(s.evaluate(0, 0.0).d2, (Vec2{0.0, 0.0}));  // exactly zero: M_0 is set, not solved for
  EXPECT_EQ(s.evaluate(s.segmentCount() - 1, 1.0).d2, (Vec2{0.0, 0.0}));
  EXPECT_TRUE(curvatureFromDerivatives(s.evaluate(0, 0.0).d1, s.evaluate(0, 0.0).d2, s.speedScale()).valid);
  EXPECT_EQ(s.curvature(0, 0.0).value, 0.0);  // natural condition = zero curvature at the end points
}

// The parametric interpolation is linear in the data, so collinear control points give points that stay
// exactly on the line and curvature that vanishes (d2 is parallel to d1), for every parameterisation.
TEST(CubicSpline, CollinearControlPointsGiveAStraightCurveWithZeroCurvature) {
  const std::vector<Vec2> line{{0, 0}, {3, 1.5}, {3.5, 1.75}, {10, 5}, {17, 8.5}, {18, 9}};  // y = x / 2
  for (double alpha : {0.0, 0.5, 1.0}) {
    const CubicSpline s = CubicSpline::create(line, Closure::Open, alpha);
    for (std::size_t seg = 0; seg < s.segmentCount(); ++seg) {
      for (double u : {0.0, 0.1, 0.5, 0.9, 1.0}) {
        const auto e = s.evaluate(seg, u);
        EXPECT_NEAR(e.position.y, 0.5 * e.position.x, 1e-12);
        EXPECT_NEAR(cross(e.d1, e.d2), 0.0, 1e-9 * (1.0 + norm(e.d1) * norm(e.d2)));
        const auto k = s.curvature(seg, u);
        ASSERT_TRUE(k.valid);
        EXPECT_NEAR(k.value, 0.0, 1e-11);
      }
    }
  }
}

TEST(CubicSpline, TwoPointsGiveTheStraightSegment) {
  const CubicSpline s = CubicSpline::create({{1, 2}, {5, 14}}, Closure::Open, 0.5);
  for (double u : {0.0, 0.25, 0.5, 1.0}) {
    const auto e = s.evaluate(0, u);
    EXPECT_NEAR(e.position.x, 1.0 + 4.0 * u, 1e-12);
    EXPECT_NEAR(e.position.y, 2.0 + 12.0 * u, 1e-12);
    EXPECT_EQ(e.d2, (Vec2{0.0, 0.0}));
  }
}

TEST(CubicSpline, ThreePointClosedCurveWorks) {
  const CubicSpline s = CubicSpline::create(circlePoints(10.0, 3), Closure::Closed, 1.0);
  EXPECT_EQ(s.segmentCount(), 3u);
  for (std::size_t i = 0; i < 3; ++i) {
    const auto a = s.evaluate(i, 1.0), b = s.evaluate((i + 1) % 3, 0.0);
    expectNearRel(a.d1, b.d1, 1e-11, "d1");
    expectNearRel(a.d2, b.d2, 1e-10, "d2");
  }
}

TEST(CubicSpline, SamplingMatchesCatmullRomConventions) {
  const auto pts = scatter(6, 5.0, 3);
  SplineOptions o;
  o.samplesPerSegment = 7;
  const Path open = cubicSpline(pts, Closure::Open, o);
  const Path closed = cubicSpline(pts, Closure::Closed, o);
  EXPECT_EQ(open.size(), 5u * 7u + 1u);
  EXPECT_EQ(closed.size(), 6u * 7u);  // no duplicated seam point
  EXPECT_NE(closed.points().front(), closed.points().back());
  for (std::size_t i = 0; i < pts.size(); ++i)
    EXPECT_EQ(closed.points()[i * 7], pts[i]);  // controls emitted exactly
  EXPECT_EQ(open.points().back(), pts.back());
  const auto k = cubicSplineCurvature(pts, Closure::Closed, o);
  EXPECT_EQ(k.size(), closed.size());
  EXPECT_EQ(cubicSplineCurvature(pts, Closure::Open, o).size(), open.size());
}

TEST(CubicSpline, IsDeterministicAndPositionsAgreeWithEvaluateAt) {
  const auto pts = scatter(10, 5.0, 21);
  EXPECT_EQ(cubicSpline(pts, Closure::Closed).points(), cubicSpline(pts, Closure::Closed).points());
  const CubicSpline s = CubicSpline::create(pts, Closure::Closed, 0.5);
  const double total = s.parameterLength();
  // Periodic evaluation: t and t + total (and t - total) give the same point.
  for (double f : {0.13, 0.5, 0.987}) {
    expectNearRel(s.evaluateAt(f * total).position, s.evaluateAt(f * total + total).position, 1e-11,
                  "periodic +");
    expectNearRel(s.evaluateAt(f * total).position, s.evaluateAt(f * total - total).position, 1e-11,
                  "periodic -");
  }
  // Open curves clamp instead of wrapping.
  const CubicSpline o = CubicSpline::create(pts, Closure::Open, 0.5);
  EXPECT_EQ(o.evaluateAt(-5.0).position, pts.front());
  EXPECT_EQ(o.evaluateAt(o.parameterLength() + 5.0).position, pts.back());
}

TEST(CubicSpline, RejectsInvalidInput) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_THROW(CubicSpline::create({}, Closure::Open), GeometryError);
  EXPECT_THROW(CubicSpline::create({{0, 0}}, Closure::Open), GeometryError);
  EXPECT_THROW(CubicSpline::create({{0, 0}, {1, 0}}, Closure::Closed), GeometryError);
  EXPECT_THROW(CubicSpline::create({{0, 0}, {nan, 0}, {1, 1}}, Closure::Open), GeometryError);
  EXPECT_THROW(CubicSpline::create({{0, 0}, {inf, 0}, {1, 1}}, Closure::Open), GeometryError);
  EXPECT_THROW(CubicSpline::create({{0, 0}, {0, 0}, {1, 1}}, Closure::Open), GeometryError);  // duplicate
  EXPECT_THROW(CubicSpline::create({{0, 0}, {1, 0}, {1, 1}, {0, 0}}, Closure::Closed),
               GeometryError);  // seam
  const std::vector<Vec2> ok{{0, 0}, {1, 0}, {1, 1}};
  EXPECT_THROW(CubicSpline::create(ok, Closure::Open, -0.1), GeometryError);
  EXPECT_THROW(CubicSpline::create(ok, Closure::Open, 1.1), GeometryError);
  EXPECT_THROW(CubicSpline::create(ok, Closure::Open, nan), GeometryError);
  EXPECT_THROW(cubicSpline(ok, Closure::Open, {0, 0.5}), GeometryError);
  EXPECT_THROW(cubicSpline(ok, Closure::Open, {100001, 0.5}), GeometryError);
  EXPECT_THROW(cubicSplineCurvature(ok, Closure::Open, {0, 0.5}), GeometryError);
  EXPECT_THROW(CubicSpline::create(ok, Closure::Open).evaluateAt(nan), GeometryError);
}

// Curvature must never be a plausible-looking number when the derivatives do not support one.
TEST(CurvatureFromDerivatives, DegenerateInputsAreReportedInvalidNotZero) {
  const auto bad = [](CurvatureValue v) { return !v.valid && std::isnan(v.value); };
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(bad(curvatureFromDerivatives({0, 0}, {1, 1}, 1.0)));     // stationary point
  EXPECT_TRUE(bad(curvatureFromDerivatives({1e-9, 0}, {1, 1}, 1.0)));  // speed below 1e-6 of the scale
  EXPECT_TRUE(bad(curvatureFromDerivatives({nan, 0}, {1, 1}, 1.0)));
  EXPECT_TRUE(bad(curvatureFromDerivatives({1, 0}, {0, nan}, 1.0)));
  EXPECT_TRUE(bad(curvatureFromDerivatives({1, 0}, {0, 1}, nan)));
  const auto ok = curvatureFromDerivatives({2, 0}, {0, 3}, 2.0);  // x' = 2, y'' = 3 -> 6 / 8
  ASSERT_TRUE(ok.valid);
  EXPECT_DOUBLE_EQ(ok.value, 0.75);
  EXPECT_LT(curvatureFromDerivatives({0, 2}, {3, 0}, 2.0).value, 0.0);  // x'' > 0 while going up: right turn
  // The relative test is independent of units: scaling d1 and the scale together does not change the verdict.
  EXPECT_EQ(curvatureFromDerivatives({1e-3, 0}, {0, 1}, 1e3).valid,
            curvatureFromDerivatives({1e-9, 0}, {0, 1}, 1.0).valid);
}

// Spacing ratios far from 1 make the interpolant overshoot more, but the linear systems stay well
// conditioned: the defining equations are still satisfied to rounding for extreme ratios (1e6 : 1), which an
// ill-conditioned or unstable elimination would not manage.
TEST(CubicSpline, ConditioningIsIndependentOfTheSpacingRatio) {
  std::vector<Vec2> pts;
  double x = 0.0;
  for (int i = 0; i < 14; ++i) {
    x += (i % 2 == 0) ? 1e-3 : 1e3;  // spacing alternates between 1 mm and 1 km
    pts.push_back({x, (i % 3) * 7.0});
  }
  for (Closure closure : {Closure::Open}) {
    const CubicSpline s = CubicSpline::create(pts, closure, 1.0);
    const auto ref = referenceSecondDerivatives(pts, closure, 1.0);
    double scale = 0.0;
    for (const Vec2& m : ref) scale = std::max(scale, norm(m));
    for (std::size_t i = 0; i < pts.size(); ++i) {
      EXPECT_NEAR(s.knotSecondDerivatives()[i].y, ref[i].y, 1e-8 * std::max(1.0, scale)) << i;
    }
    // C1 continuity (equivalent to the equations) at all interior knots, relative to the derivative scale.
    for (std::size_t i = 0; i + 1 < s.segmentCount(); ++i) {
      const auto a = s.evaluate(i, 1.0), b = s.evaluate(i + 1, 0.0);
      EXPECT_NEAR(a.d1.y, b.d1.y, 1e-9 * std::max(1.0, s.speedScale()));
    }
  }
}
