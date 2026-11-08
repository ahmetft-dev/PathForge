#include <pathforge/geometry/circle.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>

using namespace pathforge::geometry;

namespace {
constexpr double kPi = std::numbers::pi;
}

TEST(Circle, RejectsInvalidParameters) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_THROW(makeCircle(0.0, 10), GeometryError);
  EXPECT_THROW(makeCircle(-5.0, 10), GeometryError);
  EXPECT_THROW(makeCircle(nan, 10), GeometryError);
  EXPECT_THROW(makeCircle(inf, 10), GeometryError);
  EXPECT_THROW(makeCircle(10.0, 0), GeometryError);
  EXPECT_THROW(makeCircle(10.0, 2), GeometryError);
  EXPECT_NO_THROW(makeCircle(10.0, 3));
}

TEST(Circle, IsDeterministicClosedCcwAndOnTheCircle) {
  const Path a = makeCircle(50.0, 64);
  const Path b = makeCircle(50.0, 64);
  EXPECT_EQ(a.points(), b.points());
  EXPECT_TRUE(a.isClosed());
  EXPECT_EQ(a.size(), 64u);
  EXPECT_EQ(a.points()[0], (Vec2{50.0, 0.0}));
  // Vertices lie on the circle to within rounding of cos/sin (~1 ulp * R).
  for (const Vec2& p : a.points()) EXPECT_NEAR(norm(p), 50.0, 1e-12);
  // CCW orientation => positive signed area (shoelace).
  double area2 = 0.0;
  const auto& pts = a.points();
  for (std::size_t i = 0; i < pts.size(); ++i) area2 += cross(pts[i], pts[(i + 1) % pts.size()]);
  EXPECT_GT(area2, 0.0);
}

// The sampled path length equals the inscribed N-gon perimeter up to accumulated
// rounding (N additions, each ~1 ulp of the running sum): relative 1e-12 is ample for N <= 1e4.
TEST(Circle, LengthMatchesInscribedPolygonPerimeter) {
  for (std::size_t n : {3u, 8u, 64u, 1000u, 10000u}) {
    const double r = 37.5;
    const Path c = makeCircle(r, n);
    EXPECT_NEAR(c.length() / inscribedPolygonPerimeter(r, n), 1.0, 1e-12) << "n=" << n;
  }
}

// Discretisation error vs the continuous circumference 2*pi*R is NOT zero. For the inscribed
// N-gon: L = 2NR sin(pi/N) => relative error = 1 - sin(x)/x with x = pi/N ~= x^2/6 - x^4/120 + x^6/5040.
// We check (a) it is positive (polygon is shorter), (b) it matches the series prediction, and
// (c) it decays as 1/N^2 (error ratio ~4 when N doubles).
TEST(Circle, CircumferenceConvergesQuadratically) {
  const double r = 100.0;
  auto relErr = [&](std::size_t n) {
    const double exact = circleCircumference(r);
    return (exact - makeCircle(r, n).length()) / exact;
  };
  double prev = 0.0;
  for (std::size_t n : {16u, 32u, 64u, 128u, 256u, 512u}) {
    const double x = kPi / static_cast<double>(n);
    const double x2 = x * x;
    const double predicted = x2 / 6.0 - x2 * x2 / 120.0 + x2 * x2 * x2 / 5040.0;
    const double e = relErr(n);
    EXPECT_GT(e, 0.0) << "n=" << n;
    // Series truncated after x^6; the next term is x^8/362880, i.e. ~x^6/60480 relative to e
    // (~1e-9 at n=16), far below the 1e-6 tolerance.
    // Measurement noise ~1e-15 absolute, negligible against e >= ~3e-6 at n=512.
    EXPECT_NEAR(e / predicted, 1.0, 1e-6) << "n=" << n;
    if (prev > 0.0) EXPECT_NEAR(prev / e, 4.0, 0.05) << "n=" << n;  // O(h^2) => factor ~4
    prev = e;
  }
  // Concrete sanity value: N=1000 gives relative error ~1.645e-6.
  EXPECT_LT(relErr(1000), 1.7e-6);
  EXPECT_GT(relErr(1000), 1.6e-6);
}

// Menger curvature is exact for points on a circle, so only floating-point rounding of the
// generated coordinates remains. Rounding error of cross() relative to its magnitude grows like
// eps / turning angle (= eps * N / 2pi), i.e. ~1e-13 for N = 1000; 1e-9 is a safe bound.
TEST(Circle, SampledCurvatureEqualsReciprocalRadius) {
  for (double r : {0.5, 1.0, 25.0, 1000.0}) {
    for (std::size_t n : {8u, 64u, 1000u}) {
      const auto k = makeCircle(r, n).curvature();
      ASSERT_EQ(k.size(), n);
      for (double v : k) EXPECT_NEAR(v * r, 1.0, 1e-9) << "r=" << r << " n=" << n;  // CCW => +1/R
    }
  }
}

TEST(Circle, StatsConsistentWithAnalyticalValues) {
  const double r = 200.0;
  const std::size_t n = 720;
  const PathStats st = computeStats(makeCircle(r, n));
  EXPECT_EQ(st.pointCount, n);
  EXPECT_NEAR(st.minCurvature * r, 1.0, 1e-9);
  EXPECT_NEAR(st.maxCurvature * r, 1.0, 1e-9);
  EXPECT_NEAR(st.meanAbsCurvature * r, 1.0, 1e-9);
  // Regular polygon: every chord is 2 R sin(pi/N).
  const double chord = 2.0 * r * std::sin(kPi / static_cast<double>(n));
  EXPECT_NEAR(st.minSegment, chord, 1e-9);
  EXPECT_NEAR(st.maxSegment, chord, 1e-9);
}

TEST(Circle, ClosedHandlingWrapsAround) {
  // Curvature at vertex 0 and vertex N-1 must use wrap-around neighbours and so match the rest.
  const auto k = makeCircle(10.0, 12).curvature();
  EXPECT_NEAR(k.front(), k[5], 1e-12);
  EXPECT_NEAR(k.back(), k[5], 1e-12);
}

TEST(Circle, OpenArcEndpointsDoNotWrap) {
  // A half circle as an OPEN path: interior curvature is 1/R, and the endpoints use one-sided
  // copies (documented behaviour) rather than wrapping to the opposite end.
  const double r = 10.0;
  std::vector<Vec2> pts;
  for (int i = 0; i <= 20; ++i) {
    const double a = kPi * i / 20.0;
    pts.push_back({r * std::cos(a), r * std::sin(a)});
  }
  const auto k = Path::create(pts, Closure::Open).curvature();
  for (double v : k) EXPECT_NEAR(v * r, 1.0, 1e-9);
}
