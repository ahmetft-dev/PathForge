#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <pathforge/geometry/circle.hpp>
#include <pathforge/geometry/shapes.hpp>
#include <pathforge/geometry/track.hpp>

using namespace pathforge::geometry;

namespace {
constexpr double kPi = std::numbers::pi;

double polylineLength(const std::vector<Vec2>& p, bool closed) {
  double s = 0.0;
  for (std::size_t i = 0; i + 1 < p.size(); ++i) s += distance(p[i], p[i + 1]);
  if (closed) s += distance(p.back(), p.front());
  return s;
}
}  // namespace

TEST(Track, RejectsInvalidWidths) {
  const Path c = makeCircle(50.0, 64);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_THROW(Track::create(c, -1.0, 5.0), GeometryError);
  EXPECT_THROW(Track::create(c, 5.0, -0.001), GeometryError);
  EXPECT_THROW(Track::create(c, nan, 5.0), GeometryError);
  EXPECT_THROW(Track::create(c, 5.0, inf), GeometryError);
  EXPECT_NO_THROW(Track::create(c, 0.0, 0.0));
}

// Axis-aligned straight path: offsets are exact (T = +x, N = +y), rounding only.
TEST(Track, StraightPathLeftIsPlusYRightIsMinusY) {
  const Track t = Track::create(Path::create({{0, 0}, {10, 0}, {25, 0}, {40, 0}}, Closure::Open), 3.0, 2.0);
  EXPECT_DOUBLE_EQ(t.totalWidth(), 5.0);
  ASSERT_EQ(t.leftBoundary().size(), 4u);
  ASSERT_EQ(t.rightBoundary().size(), 4u);
  const double xs[] = {0, 10, 25, 40};
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_NEAR(t.leftBoundary()[i].x, xs[i], 1e-12);
    EXPECT_NEAR(t.leftBoundary()[i].y, 3.0, 1e-12);
    EXPECT_NEAR(t.rightBoundary()[i].x, xs[i], 1e-12);
    EXPECT_NEAR(t.rightBoundary()[i].y, -2.0, 1e-12);
  }
  EXPECT_FALSE(t.diagnostics().anyLocalIssue());
}

TEST(Track, LeftAndRightAreRelativeToTravelDirection) {
  const Track fwd = Track::create(Path::create({{0, 0}, {10, 0}, {20, 0}}, Closure::Open), 1.0, 1.0);
  const Track bwd = Track::create(Path::create({{20, 0}, {10, 0}, {0, 0}}, Closure::Open), 1.0, 1.0);
  EXPECT_GT(fwd.leftBoundary()[1].y, 0.0);  // heading +x: left is +y
  EXPECT_LT(bwd.leftBoundary()[1].y, 0.0);  // heading -x: left is -y
}

// Vertex normals of a regular polygon are exactly radial, so boundary vertices lie exactly on
// circles of radius R -/+ w up to rounding (1e-9 is generous for R ~ 100).
TEST(Track, CcwCircleHasInnerLeftAndOuterRightBoundary) {
  const double r = 100.0;
  const Track t = Track::create(makeCircle(r, 360), 8.0, 5.0);
  for (const Vec2& p : t.leftBoundary()) EXPECT_NEAR(norm(p), 92.0, 1e-9);
  for (const Vec2& p : t.rightBoundary()) EXPECT_NEAR(norm(p), 105.0, 1e-9);
  // Boundary lengths ~ 2 pi R' (inscribed polygon, relative shortfall (pi/N)^2/6 = 1.3e-5).
  EXPECT_NEAR(polylineLength(t.leftBoundary(), true) / (2 * kPi * 92.0), 1.0, 2e-5);
  EXPECT_NEAR(polylineLength(t.rightBoundary(), true) / (2 * kPi * 105.0), 1.0, 2e-5);
  EXPECT_FALSE(t.diagnostics().anyLocalIssue());
  EXPECT_NEAR(t.diagnostics().maxLeftWidthCurvature, 0.08, 1e-9);  // w * kappa = 8/100
  EXPECT_DOUBLE_EQ(t.diagnostics().maxRightWidthCurvature, 0.0);   // right is outside everywhere
}

TEST(Track, ClockwiseCircleSwapsInnerAndOuter) {
  const Track t = Track::create(reversed(makeCircle(100.0, 360)), 8.0, 5.0);
  for (const Vec2& p : t.leftBoundary()) EXPECT_NEAR(norm(p), 108.0, 1e-9);
  for (const Vec2& p : t.rightBoundary()) EXPECT_NEAR(norm(p), 95.0, 1e-9);
  EXPECT_NEAR(t.diagnostics().maxRightWidthCurvature, 0.05, 1e-9);
}

TEST(Track, ZeroWidthBoundariesEqualCenterline) {
  const Path c = makeCircle(30.0, 40);
  const Track t = Track::create(c, 0.0, 0.0);
  for (std::size_t i = 0; i < c.size(); ++i) {
    EXPECT_EQ(t.leftBoundary()[i], c.points()[i]);
    EXPECT_EQ(t.rightBoundary()[i], c.points()[i]);
  }
}

TEST(Track, DetectsInnerOffsetBeyondRadiusOfCurvature) {
  // Left width 15 m on a CCW circle of radius 10 m: the inner offset passes through the centre and
  // out the other side, so every boundary segment reverses direction.
  const Track t = Track::create(makeCircle(10.0, 90), 15.0, 1.0);
  const TrackDiagnostics& d = t.diagnostics();
  EXPECT_TRUE(d.anyLocalIssue());
  EXPECT_EQ(d.leftFlippedSegments, 90u);
  EXPECT_EQ(d.leftCurvatureViolations, 90u);
  EXPECT_NEAR(d.maxLeftWidthCurvature, 1.5, 1e-9);
  EXPECT_EQ(d.rightFlippedSegments, 0u);
  EXPECT_EQ(d.rightCurvatureViolations, 0u);
}

TEST(Track, DetectsLocalFoldOnTightCornerOfOpenPath) {
  // Left corner turned in two 45-degree steps: Menger radius at both corner vertices is
  // 1 / 0.1407 = 7.1 m. A 5 m left offset is inside that radius (no fold, w*kappa = 0.70) but an
  // 8 m offset exceeds it (w*kappa = 1.13).
  const Path corner = Path::create({{0, 0}, {9, 0}, {10, 1}, {10, 10}}, Closure::Open);
  EXPECT_FALSE(Track::create(corner, 5.0, 1.0).diagnostics().anyLocalIssue());
  const Track t = Track::create(corner, 8.0, 1.0);
  EXPECT_TRUE(t.diagnostics().anyLocalIssue());
  EXPECT_GE(t.diagnostics().leftCurvatureViolations, 1u);
}

TEST(Track, BoundariesOfStadiumFollowStraightsExactly) {
  const double len = 300.0, wid = 120.0, w = 10.0;
  const Track t = Track::create(makeStadium(len, wid, 512), w, w);
  // Vertex 0 is (0, -W/2) heading +x: left = +y (inside), right = -y (outside).
  EXPECT_NEAR(t.leftBoundary()[0].x, 0.0, 1e-9);
  EXPECT_NEAR(t.leftBoundary()[0].y, -wid / 2 + w, 1e-9);
  EXPECT_NEAR(t.rightBoundary()[0].y, -wid / 2 - w, 1e-9);
  EXPECT_FALSE(t.diagnostics().anyLocalIssue());
  // Corner radius 60, width 10 => w * kappa max = 10/60.
  EXPECT_NEAR(t.diagnostics().maxLeftWidthCurvature, w / (wid / 2), 1e-9);
}

TEST(Track, NoClaimOfGlobalValidity) {
  // A centerline that doubles back close to itself: local diagnostics are clean even though the two
  // boundaries cross each other. This documents the limitation of the local checks.
  const Track t = Track::create(
      Path::create({{0, 0}, {50, 0}, {100, 0}, {100, 1.0}, {50, 1.0}, {0, 1.0}}, Closure::Open), 0.0, 0.0);
  EXPECT_FALSE(t.diagnostics().anyLocalIssue());
}
