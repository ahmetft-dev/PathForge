// Numerical verification of curvature: discrete Menger curvature, analytic Catmull-Rom curvature and analytic
// C2 cubic spline curvature against analytic reference geometry. Tolerances come from derived convergence
// orders and from measurements made with benchmarks/curvature_study (docs/curvature-study.md); each test says
// which.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <pathforge/geometry/circle.hpp>
#include <pathforge/geometry/cubic_spline.hpp>
#include <pathforge/geometry/frames.hpp>
#include <pathforge/geometry/shapes.hpp>
#include <pathforge/geometry/spline.hpp>

using namespace pathforge::geometry;

namespace {
constexpr double kPi = std::numbers::pi;

std::vector<Vec2> circlePoints(double r, int m) {
  std::vector<Vec2> c;
  for (int i = 0; i < m; ++i) c.push_back({r * std::cos(2 * kPi * i / m), r * std::sin(2 * kPi * i / m)});
  return c;
}

std::vector<Vec2> ellipsePoints(double a, double b, int m) {
  std::vector<Vec2> c;
  for (int i = 0; i < m; ++i) c.push_back({a * std::cos(2 * kPi * i / m), b * std::sin(2 * kPi * i / m)});
  return c;
}

constexpr double kSineL = 300.0, kSineA = 30.0;

// Control points on y = -A sin(2 pi x / L), K intervals over one period (ends set exactly to 0).
std::vector<Vec2> sinePoints(int k) {
  std::vector<Vec2> c;
  for (int i = 0; i <= k; ++i)
    c.push_back({kSineL * i / k, (i == 0 || i == k) ? 0.0 : -kSineA * std::sin(2 * kPi * i / k)});
  return c;
}

// Exact curvature of the graph y(x) = -A sin(w x): kappa = y'' / (1 + y'^2)^(3/2).
double sineCurvature(double x) {
  const double w = 2 * kPi / kSineL;
  const double yp = -kSineA * w * std::cos(w * x), ypp = kSineA * w * w * std::sin(w * x);
  return ypp / std::pow(1.0 + yp * yp, 1.5);
}

struct ErrorStats {
  double max = 0.0, rms = 0.0;
};

ErrorStats sineError(const std::vector<double>& k, const Path& p) {
  ErrorStats e;
  for (std::size_t i = 0; i < k.size(); ++i) {
    const double d = std::abs(k[i] - sineCurvature(p.points()[i].x));
    e.max = std::max(e.max, d);
    e.rms += d * d;
  }
  e.rms = std::sqrt(e.rms / static_cast<double>(k.size()));
  return e;
}

double maxCircleError(const std::vector<double>& k, double r) {
  double m = 0.0;
  for (double v : k) m = std::max(m, std::abs(v * r - 1.0));
  return m;
}
}  // namespace

// ------------------------------------------------------------------------------------------------ straight
// line

TEST(CurvatureAccuracy, StraightLineIsFlatWithStableFrames) {
  // Unevenly spaced collinear points on y = 0.5 x + 3. All three curvature definitions must vanish and the
  // frames must not oscillate: tangent = line direction, normal perpendicular, at every sample.
  const std::vector<Vec2> line{{0, 3}, {3, 4.5}, {3.5, 4.75}, {10, 8}, {17, 11.5}, {18, 12}};
  SplineOptions o;
  o.samplesPerSegment = 16;
  const Vec2 dir = Vec2{2, 1} * (1.0 / std::sqrt(5.0));
  for (const Path& p : {catmullRomSpline(line, Closure::Open, o), cubicSpline(line, Closure::Open, o)}) {
    for (double k : p.curvature()) EXPECT_NEAR(k, 0.0, 1e-9);  // Menger on (nearly) collinear samples
    const PathFrames f = computeFrames(p);
    for (std::size_t i = 0; i < p.size(); ++i) {
      EXPECT_NEAR(f.tangent[i].x, dir.x, 1e-9);
      EXPECT_NEAR(f.tangent[i].y, dir.y, 1e-9);
      EXPECT_NEAR(dot(f.tangent[i], f.normal[i]), 0.0, 1e-12);
    }
  }
  for (double k : cubicSplineCurvature(line, Closure::Open, o)) EXPECT_NEAR(k, 0.0, 1e-11);
  for (double k : catmullRomCurvature(line, Closure::Open, o)) EXPECT_NEAR(k, 0.0, 1e-11);
}

// ---------------------------------------------------------------------------------------------------- circle

// Interpolating a circle with a periodic cubic spline is NOT exact. The curvature error is O(h^2) with h = 2
// pi / M: measured max|kappa R - 1| * M^2 = 3.66 (M = 8), 3.38 (16), 3.31 (32), 3.29 (64), so the bound 3.8 /
// M^2 holds for M >= 8 and the successive ratios must be ~4. Catmull-Rom has the same order with a ~9x larger
// constant (30 / M^2).
TEST(CurvatureAccuracy, CircleErrorOfC2SplineConvergesQuadratically) {
  double prev = 0.0;
  for (int m : {8, 16, 32, 64}) {
    const double e =
        maxCircleError(cubicSplineCurvature(circlePoints(100.0, m), Closure::Closed, {64, 0.5}), 100.0);
    EXPECT_LT(e, 3.8 / (m * m)) << "m=" << m;
    EXPECT_GT(e, 2.5 / (m * m)) << "m="
                                << m;  // and it is not accidentally exact: the error is real and measurable
    if (prev > 0.0) {
      EXPECT_GT(prev / e, 3.9) << m;
      EXPECT_LT(prev / e, 4.5) << m;
    }
    prev = e;
  }
}

TEST(CurvatureAccuracy, CircleErrorIsScaleInvariantAndC2BeatsCatmullRomHere) {
  // kappa * R is dimensionless: the same control-point count gives the same error for R = 0.5 m ... 1 km
  // (rounding only).
  const double ref =
      maxCircleError(cubicSplineCurvature(circlePoints(1.0, 16), Closure::Closed, {64, 0.5}), 1.0);
  for (double r : {0.5, 10.0, 1000.0}) {
    EXPECT_NEAR(maxCircleError(cubicSplineCurvature(circlePoints(r, 16), Closure::Closed, {64, 0.5}), r), ref,
                1e-9);
  }
  // Measured for M = 16: Catmull-Rom 0.119, C2 0.0132 (9x). Case-specific: it says nothing about other shapes
  // (hairpin below).
  const double cr =
      maxCircleError(catmullRomCurvature(circlePoints(100.0, 16), Closure::Closed, {64, 0.5}), 100.0);
  EXPECT_GT(cr / ref, 5.0);
  // Uniform spacing: the knot exponent alpha changes nothing (all chords are equal).
  const auto a = cubicSplineCurvature(circlePoints(100.0, 16), Closure::Closed, {8, 0.0});
  const auto b = cubicSplineCurvature(circlePoints(100.0, 16), Closure::Closed, {8, 1.0});
  for (std::size_t i = 0; i < a.size(); ++i) EXPECT_NEAR(a[i], b[i], 1e-12);
}

TEST(CurvatureAccuracy, CirclePositionErrorOfC2IsSmallerThanCatmullRomAtTheSameControls) {
  // Radial error max | |p| - R | / R for M = 16: Catmull-Rom 5.5e-4, C2 6.4e-5 (measured). Order O(h^4) for
  // both.
  const auto c = circlePoints(100.0, 16);
  auto err = [](const Path& p) {
    double m = 0;
    for (const Vec2& q : p.points()) m = std::max(m, std::abs(norm(q) - 100.0) / 100.0);
    return m;
  };
  const double e2 = err(cubicSpline(c, Closure::Closed, {64, 0.5}));
  EXPECT_LT(e2, 8e-5);
  EXPECT_GT(e2, 4e-5);
  EXPECT_LT(e2, err(catmullRomSpline(c, Closure::Closed, {64, 0.5})) / 5.0);
}

TEST(CurvatureAccuracy, MengerCurvatureOfASampledCircleIsExact) {
  // For comparison: three samples on a circle give exactly 1/R, independent of any spline.
  for (double v : makeCircle(100.0, 360).curvature()) EXPECT_NEAR(v * 100.0, 1.0, 1e-9);
}

// ----------------------------------------------------------------------------------------------------
// S-curve

// y = -A sin(2 pi x / L) sampled at K+1 equally spaced x. It has zero curvature at both ends, which is
// exactly the natural boundary condition, so the open C2 spline suffers no end-condition error here (not true
// for general paths). Measured max curvature error * K^2 / (A (2 pi / L)^2): ~6 for every K, i.e. O(h^2) with
// a small constant.
TEST(CurvatureAccuracy, SCurveC2ErrorMatchesAnalyticSineAndConvergesQuadratically) {
  const double peak = kSineA * std::pow(2 * kPi / kSineL, 2);  // 1.316e-2
  double prevMax = 0.0, prevRms = 0.0;
  for (int k : {8, 16, 32}) {
    const auto c = sinePoints(k);
    const ErrorStats e = sineError(cubicSplineCurvature(c, Closure::Open, {64, 0.5}),
                                   cubicSpline(c, Closure::Open, {64, 0.5}));
    EXPECT_LT(e.max, 0.1 / (k * k)) << k;    // measured 0.073 / K^2 .. 0.081 / K^2
    EXPECT_LT(e.rms, 0.025 / (k * k)) << k;  // measured 0.021 / K^2 .. 0.017 / K^2
    EXPECT_GT(e.max, 0.03 * peak / (k * k) *
                         100);  // ... and a real, non-zero error (> 0.04 / K^2 for A, L used here)
    if (prevMax > 0.0) {
      EXPECT_GT(prevMax / e.max, 3.3) << k;  // observed 3.6, 4.1
      EXPECT_LT(prevMax / e.max, 4.6) << k;
      EXPECT_GT(prevRms / e.rms, 3.3) << k;
    }
    prevMax = e.max;
    prevRms = e.rms;
  }
}

TEST(CurvatureAccuracy, SCurveCatmullRomIsMuchLessAccurateButNotWrongInKind) {
  const auto c = sinePoints(16);
  const ErrorStats cr = sineError(catmullRomCurvature(c, Closure::Open, {64, 0.5}),
                                  catmullRomSpline(c, Closure::Open, {64, 0.5}));
  const ErrorStats c2 =
      sineError(cubicSplineCurvature(c, Closure::Open, {64, 0.5}), cubicSpline(c, Closure::Open, {64, 0.5}));
  EXPECT_NEAR(cr.max, 3.57e-3, 3e-4);  // measured: 27 % of the peak curvature
  EXPECT_NEAR(c2.max, 3.18e-4, 3e-5);  // measured: 2.4 % of the peak curvature
  EXPECT_GT(cr.max / c2.max, 8.0);
}

TEST(CurvatureAccuracy, SCurveHasASingleSignChangeAtTheMiddleForC2AtEveryResolution) {
  for (int k : {8, 12, 16, 24, 32}) {
    const auto c = sinePoints(k);
    const auto kap = cubicSplineCurvature(c, Closure::Open, {32, 0.5});
    double peak = 0;
    for (double v : kap) peak = std::max(peak, std::abs(v));
    EXPECT_EQ(countSignChanges(kap, 0.25 * peak), 1u)
        << k;                     // Catmull-Rom needs K >= 16 for this (5 changes at K = 8, 12)
    EXPECT_GT(kap[1 * 32], 0.0);  // left turn first (positive curvature) ...
    EXPECT_LT(kap[static_cast<std::size_t>(k - 1) * 32], 0.0);  // ... right turn last
  }
  // The control points are point-symmetric about (L/2, 0) and the natural spline is odd under that symmetry,
  // so the curvature vanishes exactly at the inflection (a knot for even K): measured 6.5e-18.
  const auto kap16 = cubicSplineCurvature(sinePoints(16), Closure::Open, {64, 0.5});
  EXPECT_NEAR(kap16[8 * 64], 0.0, 1e-12);
  for (std::size_t i = 0; i < kap16.size(); ++i)
    EXPECT_NEAR(kap16[i], -kap16[kap16.size() - 1 - i], 1e-12) << i;
}

// Menger curvature of the sampled C2 curve vs the analytic curvature of the same curve. The C2 curvature has
// a jump in its DERIVATIVE at the knots, so the discrete estimate converges like O(h) at the samples next to
// a knot and like O(h^2) elsewhere. Measured per halving of the sample spacing: ratio 3.25, 3.65, 3.83, 3.92
// away from knots and exactly ~2.0 next to knots (these are properties of the discrete estimator, not of the
// spline).
TEST(CurvatureAccuracy, MengerOfSampledC2CurveConvergesQuadraticallyAwayFromKnotsAndLinearlyAtThem) {
  const auto c = sinePoints(16);
  double prevAway = 0, prevNear = 0;
  for (std::size_t s : {16u, 32u, 64u, 128u}) {
    const auto analytic = cubicSplineCurvature(c, Closure::Open, {s, 0.5});
    const auto menger = cubicSpline(c, Closure::Open, {s, 0.5}).curvature();
    double away = 0, nearKnot = 0;
    for (std::size_t i = 1; i + 1 < analytic.size(); ++i) {
      const std::size_t ph = i % s;
      const bool atKnot = ph == 0 || ph == 1 || ph + 1 == s;
      (atKnot ? nearKnot : away) = std::max(atKnot ? nearKnot : away, std::abs(analytic[i] - menger[i]));
    }
    if (prevAway > 0) {
      EXPECT_GT(prevAway / away, 3.1) << s;
      EXPECT_LT(prevAway / away, 4.4) << s;
      EXPECT_GT(prevNear / nearKnot, 1.8) << s;
      EXPECT_LT(prevNear / nearKnot, 2.3) << s;
    }
    prevAway = away;
    prevNear = nearKnot;
  }
}

// --------------------------------------------------------------------------------------------- ellipse and
// seam

// The ellipse is an analytically smooth closed reference for the periodic spline: kappa(t) = a b / (a^2 sin^2
// t + b^2 cos^2 t)^(3/2). Error relative to the peak curvature: 13 % (M = 12), 5.5 % (20), 1.3 % (40), 0.32 %
// (80): ratio ~4.2.
TEST(CurvatureAccuracy, EllipseCurvatureConvergesAndIsContinuousAcrossTheSeam) {
  const double a = 150.0, b = 80.0;
  double prev = 0.0;
  for (int m : {20, 40, 80}) {
    const auto c = ellipsePoints(a, b, m);
    const auto k = cubicSplineCurvature(c, Closure::Closed, {32, 0.5});
    const auto p = cubicSpline(c, Closure::Closed, {32, 0.5});
    double err = 0, kmax = 0;
    for (std::size_t i = 0; i < k.size(); ++i) {
      const Vec2 q = p.points()[i];
      const double t = std::atan2(q.y / b, q.x / a);
      const double kt =
          a * b / std::pow(a * a * std::sin(t) * std::sin(t) + b * b * std::cos(t) * std::cos(t), 1.5);
      kmax = std::max(kmax, kt);
      err = std::max(err, std::abs(k[i] - kt));
    }
    err /= kmax;
    EXPECT_LT(err, m == 20 ? 0.06 : (m == 40 ? 0.015 : 0.004)) << m;
    if (prev > 0.0 && m >= 40) {
      EXPECT_GT(prev / err, 3.5) << m;
    }
    prev = err;
  }
}

TEST(CurvatureAccuracy, C2CurvatureHasNoJumpAtKnotsOrAtTheSeamWhileCatmullRomHasOne) {
  const auto c = ellipsePoints(150.0, 80.0, 20);
  const CubicSpline s = CubicSpline::create(c, Closure::Closed, 0.5);
  const std::size_t segs = s.segmentCount();
  for (std::size_t i = 0; i < segs; ++i) {  // two-sided limits at every knot including the seam knot 0
    const auto before = s.curvature((i + segs - 1) % segs, 1.0), after = s.curvature(i, 0.0);
    ASSERT_TRUE(before.valid && after.valid);
    EXPECT_NEAR(before.value, after.value, 1e-12) << i;  // measured 8e-17
  }
  // Catmull-Rom: the curvature genuinely jumps. With 4096 samples per segment the sample-to-sample slope term
  // is ~1e-6, so the step across a knot is the jump: measured 6.1e-3 (a third of the peak curvature of this
  // ellipse) vs 2.4e-6.
  const std::size_t n = 4096;
  const auto cr = catmullRomCurvature(c, Closure::Closed, {n, 0.5});
  const auto c2 = cubicSplineCurvature(c, Closure::Closed, {n, 0.5});
  double jumpCr = 0, jumpC2 = 0;
  for (std::size_t i = 0; i < 20; ++i) {
    const std::size_t at = i * n, before = (at + cr.size() - 1) % cr.size();
    jumpCr = std::max(jumpCr, std::abs(cr[at] - cr[before]));
    jumpC2 = std::max(jumpC2, std::abs(c2[at] - c2[before]));
  }
  EXPECT_GT(jumpCr, 4e-3);
  EXPECT_LT(jumpC2, 1e-5);
}

// ------------------------------------------------------------------------------------------------- stadium

// A mathematically exact stadium has a curvature DISCONTINUITY (0 on straights, 1/r on arcs). A C2 curve
// cannot keep it: it must round the step, and does so with a small undershoot and overshoot (a Gibbs-like
// ripple). This test documents the behaviour; it does not demand that C2 reproduces the exact stadium.
// Measured for r = 60 m, 64 exact points: C2 range [-0.18, 1.06] / r, Catmull-Rom range [-0.63, 1.35] / r.
TEST(CurvatureAccuracy, ExactStadiumCurvatureStepIsRoundedWithBoundedRippleByC2) {
  const Path stadium = makeStadium(300.0, 120.0, 64);
  const double r = 60.0;
  const auto menger = stadium.curvature();
  EXPECT_NEAR(*std::max_element(menger.begin(), menger.end()) * r, 1.0,
              1e-9);  // the exact stadium: step from 0 to 1/r
  EXPECT_NEAR(*std::min_element(menger.begin(), menger.end()), 0.0, 1e-12);
  const auto c2 = cubicSplineCurvature(stadium.points(), Closure::Closed, {32, 0.5});
  const auto cr = catmullRomCurvature(stadium.points(), Closure::Closed, {32, 0.5});
  const double c2max = *std::max_element(c2.begin(), c2.end()) * r,
               c2min = *std::min_element(c2.begin(), c2.end()) * r;
  const double crmax = *std::max_element(cr.begin(), cr.end()) * r,
               crmin = *std::min_element(cr.begin(), cr.end()) * r;
  EXPECT_LT(c2max, 1.12);
  EXPECT_GT(c2max, 1.0);
  EXPECT_GT(c2min, -0.25);  // a small turn in the wrong direction near the straight/arc junctions
  EXPECT_LT(c2min, -0.05);
  EXPECT_GT(crmax, c2max);  // for THIS shape the C2 ripple is smaller than the Catmull-Rom one
  EXPECT_LT(crmin, c2min);
}

// ------------------------------------------------------------------------------------------- hairpin /
// overshoot

namespace {
double distanceToPolyline(const std::vector<Vec2>& poly, Vec2 q) {
  double best = 1e300;
  for (std::size_t i = 0; i + 1 < poly.size(); ++i) {
    const Vec2 a = poly[i], ab = poly[i + 1] - a;
    const double t = std::clamp(dot(q - a, ab) / dot(ab, ab), 0.0, 1.0);
    best = std::min(best, distance(q, a + t * ab));
  }
  return best;
}
double overshoot(const std::vector<Vec2>& control, const Path& p) {
  double m = 0;
  for (const Vec2& q : p.points()) m = std::max(m, distanceToPolyline(control, q));
  return m;
}
double maxAbs(const std::vector<double>& v) {
  double m = 0;
  for (double x : v) m = std::max(m, std::abs(x));
  return m;
}
}  // namespace

// A 180-degree turn (100 m straight, tight U-turn of ~10 m, 100 m straight back). Smooth curvature does NOT
// mean the curve stays closer to the intended path: here the C2 spline wanders further from the control
// polygon than Catmull-Rom while its peak curvature is lower. Measured (alpha = 0.5): overshoot C2 2.16 m vs
// CR 1.66 m; max|kappa| 0.71 vs 1.13 (radius 1.4 m vs 0.9 m). With chordal knots (alpha = 1) both overshoot
// much more (7.7 m vs 6.0 m), which is why 0.5 is the default.
TEST(CurvatureAccuracy, HairpinOvershootAndPeakCurvatureArePropertiesOfTheMethodNotOfSmoothness) {
  const std::vector<Vec2> hairpin{{0, 0}, {100, 0}, {110, 5}, {100, 10}, {0, 10}};
  const Path c2 = cubicSpline(hairpin, Closure::Open, {64, 0.5});
  const Path cr = catmullRomSpline(hairpin, Closure::Open, {64, 0.5});
  const double o2 = overshoot(hairpin, c2), ocr = overshoot(hairpin, cr);
  EXPECT_NEAR(o2, 2.16, 0.25);
  EXPECT_NEAR(ocr, 1.66, 0.25);
  EXPECT_GT(o2, ocr);  // documented: the smoother curve overshoots MORE here
  const double k2 = maxAbs(cubicSplineCurvature(hairpin, Closure::Open, {64, 0.5}));
  const double kcr = maxAbs(catmullRomCurvature(hairpin, Closure::Open, {64, 0.5}));
  EXPECT_NEAR(k2, 0.714, 0.07);
  EXPECT_NEAR(kcr, 1.13, 0.12);
  EXPECT_LT(k2, kcr);
  // Chordal knots overshoot far more than centripetal ones for both methods.
  EXPECT_GT(overshoot(hairpin, cubicSpline(hairpin, Closure::Open, {64, 1.0})), 2.5 * o2);
  EXPECT_GT(overshoot(hairpin, catmullRomSpline(hairpin, Closure::Open, {64, 1.0})), 2.5 * ocr);
  // The curve still passes through every control point.
  for (std::size_t i = 0; i < hairpin.size(); ++i) {
    EXPECT_EQ(c2.points()[i * 64], hairpin[i]);
  }
}

TEST(CurvatureAccuracy, NearlyCoincidentControlPointsEitherGiveFiniteCurvatureOrAreFlaggedNeverSilentlyZero) {
  // Two control points 1 mm apart between 100 m neighbours: the speed |r'| collapses between them, so the
  // curvature there is huge or undefined. Every value is either finite or NaN-and-invalid; the check below
  // ties the two representations.
  const std::vector<Vec2> pts{{0, 0}, {100, 0}, {100.001, 0.0005}, {200, 30}, {300, 0}};
  const CubicSpline s = CubicSpline::create(pts, Closure::Open, 0.5);
  std::size_t finite = 0, invalid = 0;
  for (std::size_t seg = 0; seg < s.segmentCount(); ++seg) {
    for (int i = 0; i <= 50; ++i) {
      const auto k = s.curvature(seg, i / 50.0);
      if (k.valid) {
        EXPECT_TRUE(std::isfinite(k.value));
        ++finite;
      } else {
        EXPECT_TRUE(std::isnan(k.value));
        ++invalid;
      }
    }
  }
  EXPECT_GT(finite, 100u);
  EXPECT_EQ(finite + invalid, 4u * 51u);
  // Points closer than the 1 nm duplicate threshold are rejected outright.
  EXPECT_THROW(CubicSpline::create({{0, 0}, {100, 0}, {100.0 + 1e-10, 0.0}, {200, 0}}, Closure::Open),
               GeometryError);
}
