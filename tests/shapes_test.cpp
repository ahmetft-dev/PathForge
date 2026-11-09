#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <pathforge/geometry/circle.hpp>
#include <pathforge/geometry/frames.hpp>
#include <pathforge/geometry/shapes.hpp>
#include <pathforge/geometry/track.hpp>

using namespace pathforge::geometry;

namespace {
constexpr double kPi = std::numbers::pi;

double signedArea2(const Path& p) {
  double a = 0.0;
  const auto& q = p.points();
  for (std::size_t i = 0; i < q.size(); ++i) a += cross(q[i], q[(i + 1) % q.size()]);
  return a;
}
}  // namespace

TEST(RoundedRectangle, RejectsInvalidParameters) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(makeRoundedRectangle(0.0, 50.0, 10.0, 64), GeometryError);
  EXPECT_THROW(makeRoundedRectangle(100.0, -5.0, 10.0, 64), GeometryError);
  EXPECT_THROW(makeRoundedRectangle(100.0, 50.0, 0.0, 64), GeometryError);
  EXPECT_THROW(makeRoundedRectangle(100.0, 50.0, 26.0, 64), GeometryError);  // r > W / 2
  EXPECT_THROW(makeRoundedRectangle(nan, 50.0, 10.0, 64), GeometryError);
  EXPECT_THROW(makeRoundedRectangle(100.0, 50.0, 10.0, 7), GeometryError);
  EXPECT_NO_THROW(makeRoundedRectangle(100.0, 50.0, 25.0, 64));  // r == W/2 allowed
  EXPECT_THROW(makeStadium(100.0, 120.0, 64), GeometryError);    // length < width
}

TEST(RoundedRectangle, PerimeterFormula) {
  // 2(L-2r) + 2(W-2r) + 2 pi r, e.g. 100 x 60, r = 10: 160 + 80 + 62.83...
  EXPECT_NEAR(roundedRectanglePerimeter(100.0, 60.0, 10.0), 2 * 80.0 + 2 * 40.0 + 2 * kPi * 10.0, 1e-12);
  // r = W/2 stadium of L = 300, W = 120: 2*180 + 2 pi 60.
  EXPECT_NEAR(roundedRectanglePerimeter(300.0, 120.0, 60.0), 360.0 + 120.0 * kPi, 1e-12);
}

// Chords across a corner of radius r and spacing h are shorter than the arc by a factor
// (1 - theta^2/24), theta = h/r; chords that straddle a straight/arc junction are shorter by less.
// Hence 0 <= relative shortfall <= theta^2/24 (no equality claimed; zero error would be false).
TEST(Stadium, PerimeterMatchesAnalyticalWithinDiscretisationBound) {
  const double len = 300.0, wid = 120.0, r = 60.0;
  for (std::size_t n : {128u, 256u, 512u, 2048u}) {
    const Path p = makeStadium(len, wid, n);
    const double ref = roundedRectanglePerimeter(len, wid, r);
    const double h = ref / static_cast<double>(n);
    const double shortfall = (ref - p.length()) / ref;
    EXPECT_GE(shortfall, 0.0) << n;
    EXPECT_LE(shortfall, (h / r) * (h / r) / 24.0 * 1.0001) << n;
  }
}

TEST(Stadium, VerticesLieExactlyOnTheAnalyticalCurve) {
  const double len = 300.0, wid = 120.0, r = 60.0, a = len / 2 - r;
  const Path p = makeStadium(len, wid, 512);
  for (const Vec2& q : p.points()) {
    const double ax = std::abs(q.x);
    double dist;  // distance from the stadium's medial segment [-a, a] on the x axis
    dist = ax <= a ? std::abs(q.y) : std::hypot(ax - a, q.y);
    EXPECT_NEAR(dist, r, 1e-9);  // every centerline point is r from the medial segment
  }
}

TEST(Stadium, StartPointOrientationAndClosure) {
  const Path p = makeStadium(300.0, 120.0, 512);
  EXPECT_TRUE(p.isClosed());
  EXPECT_EQ(p.size(), 512u);
  EXPECT_NEAR(p.points()[0].x, 0.0, 1e-12);
  EXPECT_NEAR(p.points()[0].y, -60.0, 1e-12);
  EXPECT_GT(signedArea2(p), 0.0);  // CCW
  // Seam is not duplicated and the closing segment has the same length as the others: all vertices
  // are equally spaced in arc length, chords differ only through curvature (relative < 1e-4 here).
  EXPECT_NE(p.points().front(), p.points().back());
  const double h = distance(p.points().back(), p.points().front());
  const PathStats st = computeStats(p);
  EXPECT_NEAR(h / st.maxSegment, 1.0, 1e-4);
  EXPECT_NEAR(st.minSegment / st.maxSegment, 1.0, 1e-4);
  // Enclosed area ~ L*W - (4 - pi) r^2 (analytic), polygon slightly smaller.
  const double areaRef = 300.0 * 120.0 - (4.0 - kPi) * 60.0 * 60.0;
  EXPECT_NEAR(signedArea2(p) / 2.0 / areaRef, 1.0, 1e-4);
}

// Straight vertices: all three points collinear => cross product 0 up to rounding (~1e-15 * h^2).
// Corner vertices with both neighbours on the arc lie on a circle of radius r => Menger exactly
// 1/r up to rounding (~eps / turning angle). 1e-9 is far above both.
TEST(Stadium, StraightCurvatureIsZeroAndCornerCurvatureIsReciprocalRadius) {
  const double len = 300.0, wid = 120.0, r = 60.0, a = len / 2 - r;
  const Path p = makeStadium(len, wid, 512);
  const auto k = p.curvature();
  const double h = p.length() / 512.0;
  std::size_t straight = 0, corner = 0;
  for (std::size_t i = 0; i < p.size(); ++i) {
    const Vec2 q = p.points()[i];
    if (std::abs(std::abs(q.y) - r) < 1e-9 && std::abs(q.x) < a - 1.5 * h) {
      ++straight;
      EXPECT_NEAR(k[i], 0.0, 1e-9) << i;
    }
    // Corner region: x - a > 1.5 h means the vertex is more than one chord from either junction
    // (junctions are where x = a), so both neighbours also lie on the arc.
    if (std::abs(q.x) - a > 1.5 * h) {
      ++corner;
      EXPECT_NEAR(k[i] * r, 1.0, 1e-9) << i;
    }
  }
  EXPECT_GT(straight, 100u);
  EXPECT_GT(corner, 100u);
}

TEST(Stadium, LengthEqualWidthIsACircle) {
  const Path s = makeStadium(80.0, 80.0, 360);
  for (const Vec2& q : s.points()) EXPECT_NEAR(norm(q), 40.0, 1e-9);
  const PathStats st = computeStats(s);
  EXPECT_NEAR(st.minCurvature * 40.0, 1.0, 1e-9);
  EXPECT_NEAR(st.maxCurvature * 40.0, 1.0, 1e-9);
}

TEST(RoundedRectangle, MaximumCurvatureIsReciprocalCornerRadius) {
  const Path p = makeRoundedRectangle(200.0, 100.0, 15.0, 1000);
  const PathStats st = computeStats(p);
  EXPECT_LE(st.maxCurvature * 15.0, 1.0 + 1e-9);   // junction vertices mix straight and arc: <= 1/r
  EXPECT_NEAR(st.maxCurvature * 15.0, 1.0, 1e-9);  // interior corner vertices reach exactly 1/r
  EXPECT_GE(st.minCurvature, -1e-9);               // never turns right
}

TEST(SCurve, RejectsInvalidParameters) {
  EXPECT_THROW(makeSCurve(0.0, 10.0, 100), GeometryError);
  EXPECT_THROW(makeSCurve(300.0, -1.0, 100), GeometryError);
  EXPECT_THROW(makeSCurve(std::numeric_limits<double>::infinity(), 10.0, 100), GeometryError);
  EXPECT_THROW(makeSCurve(300.0, 30.0, 7), GeometryError);
}

TEST(SCurve, IsOpenWithExactEndpointsAndRequestedSampleCount) {
  const Path s = makeSCurve(300.0, 30.0, 256);
  EXPECT_FALSE(s.isClosed());
  EXPECT_EQ(s.size(), 256u);
  EXPECT_EQ(s.points().front(), (Vec2{0.0, 0.0}));
  EXPECT_EQ(s.points().back(), (Vec2{300.0, 0.0}));
  const auto& a = s.arcLengths();
  for (std::size_t i = 1; i < a.size(); ++i) EXPECT_GT(a[i], a[i - 1]);
  EXPECT_EQ(makeSCurve(300.0, 30.0, 256).points(), s.points());  // deterministic
}

// Reference length: arc length of y = -A sin(2 pi x / L), integrated by the midpoint rule with
// 200000 panels (error ~1e-12 relative). The spline interpolates the sine at 17 points only, so
// it differs: observed relative length difference 4.9e-5, bound set at 5e-4 (10x margin).
TEST(SCurve, LengthMatchesSineArcLength) {
  const double L = 300.0, A = 30.0;
  const int n = 200000;
  const double dx = L / n;
  double ref = 0.0;
  for (int i = 0; i < n; ++i) {
    const double d = A * 2.0 * kPi / L * std::cos(2.0 * kPi * (i + 0.5) * dx / L);
    ref += std::sqrt(1.0 + d * d) * dx;
  }
  EXPECT_NEAR(makeSCurve(L, A, 1024).length() / ref, 1.0, 5e-4);
}

TEST(SCurve, SamplesAreEquallySpacedInArcLength) {
  // Chord shortfall <= (kappa_max * h)^2 / 24: kappa_max ~ 1.5e-2, h ~ 1.3 m => 1.5e-5; bound 1e-4.
  const Path s = makeSCurve(300.0, 30.0, 256);
  const auto& q = s.points();
  double lo = 1e300, hi = 0.0;
  for (std::size_t i = 0; i + 1 < q.size(); ++i) {
    const double d = distance(q[i], q[i + 1]);
    lo = std::min(lo, d);
    hi = std::max(hi, d);
  }
  EXPECT_LT(hi / lo - 1.0, 1e-4);
}

// Catmull-Rom curvature is only piecewise smooth (jumps at control points, up to ~27% of the peak away from
// the sine's true curvature, see docs/curvature-study.md) - this is the spline, not the estimator. We
// therefore require the sign pattern only where |kappa| exceeds 25% of the peak: exactly one change,
// left-turning before, right-turning after, located near the middle.
TEST(SCurve, CurvatureChangesSignOnceFromLeftToRight) {
  const double L = 300.0;
  const Path s = makeSCurve(L, 30.0, 512);
  const auto k = s.curvature();
  double peak = 0.0;
  for (double v : k) peak = std::max(peak, std::abs(v));
  const double thr = 0.25 * peak;
  EXPECT_EQ(countSignChanges(k, thr), 1u);

  double firstStrong = 0.0, lastStrong = 0.0, changeX = -1.0;
  int last = 0;
  for (std::size_t i = 0; i < k.size(); ++i) {
    if (std::abs(k[i]) <= thr) continue;
    const int sgn = k[i] > 0 ? 1 : -1;
    if (last == 0) firstStrong = sgn;
    if (last != 0 && sgn != last) changeX = s.points()[i].x;
    lastStrong = sgn;
    last = sgn;
  }
  EXPECT_EQ(firstStrong, 1.0);  // left turn first (positive curvature)
  EXPECT_EQ(lastStrong, -1.0);  // then right turn
  EXPECT_NEAR(changeX, L / 2.0, L / 10.0);
  // Peak curvature is near the analytic sine value A (2pi/L)^2 = 1.316e-2: spline observed 1.45e-2
  // (+10%); require within 25%.
  EXPECT_NEAR(peak / (30.0 * std::pow(2 * kPi / 300.0, 2)), 1.0, 0.25);
}

// The control polygon and the resampling are point-symmetric about (L/2, 0), so p[i] + p[n-1-i] = (L, 0)
// up to rounding of the spline evaluation (observed < 1e-12; bound 1e-9), and curvature is odd.
TEST(SCurve, IsPointSymmetricAndCurvatureIsAntisymmetric) {
  const Path s = makeSCurve(300.0, 30.0, 301);
  const auto& q = s.points();
  const auto k = s.curvature();
  const std::size_t n = q.size();
  for (std::size_t i = 0; i < n; ++i) {
    EXPECT_NEAR(q[i].x + q[n - 1 - i].x, 300.0, 1e-9) << i;
    EXPECT_NEAR(q[i].y + q[n - 1 - i].y, 0.0, 1e-9) << i;
    EXPECT_NEAR(k[i], -k[n - 1 - i], 1e-9) << i;
  }
}

TEST(SCurve, ReversalNegatesCurvatureSigns) {
  const Path s = makeSCurve(300.0, 30.0, 256);
  const auto k = s.curvature();
  const auto kr = reversed(s).curvature();
  for (std::size_t i = 0; i < k.size(); ++i) EXPECT_NEAR(kr[i], -k[k.size() - 1 - i], 1e-12);
}

// Consistency across sampling resolution: length converges (polyline error O(h^2)) and the peak
// curvature estimate stays within a few percent (spline itself is fixed by its 17 control points).
TEST(SCurve, GeometryIsConsistentAcrossResolutions) {
  const Path coarse = makeSCurve(300.0, 30.0, 128);
  const Path fine = makeSCurve(300.0, 30.0, 2048);
  EXPECT_NEAR(coarse.length() / fine.length(), 1.0, 1e-4);
  EXPECT_NEAR(computeStats(coarse).maxAbsCurvature / computeStats(fine).maxAbsCurvature, 1.0, 0.1);
  // Regression for the resampling caveat: finer output must NOT make the peak curvature diverge
  // (it did when the dense spline polyline was fixed at 1024 vertices). Observed 2048 vs 4096: 1e-3.
  const Path finer = makeSCurve(300.0, 30.0, 4096);
  EXPECT_NEAR(computeStats(fine).maxAbsCurvature / computeStats(finer).maxAbsCurvature, 1.0, 5e-3);
}

TEST(SCurve, BoundariesHaveOppositeInnerSides) {
  const Track t = Track::create(makeSCurve(300.0, 30.0, 256), 5.0, 5.0);
  EXPECT_FALSE(t.diagnostics().anyLocalIssue());
  // Peak curvature 1.5e-2 => radius 69 m; width 5 m => margin w*kappa ~ 0.07, both sides equal by symmetry.
  EXPECT_NEAR(t.diagnostics().maxLeftWidthCurvature, t.diagnostics().maxRightWidthCurvature, 5e-3);
  EXPECT_NEAR(t.diagnostics().maxLeftWidthCurvature, 5.0 * 1.45e-2, 2e-2);
}
