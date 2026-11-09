#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <pathforge/geometry/circle.hpp>
#include <pathforge/geometry/frames.hpp>

using namespace pathforge::geometry;

namespace {
constexpr double kPi = std::numbers::pi;
}

// Frame orthonormality only involves normalisation and a 90 degree rotation: errors are a few ulp.
TEST(Frames, TangentAndNormalAreOrthonormalAndRightHanded) {
  const PathFrames f = computeFrames(makeCircle(25.0, 100));
  for (std::size_t i = 0; i < f.tangent.size(); ++i) {
    EXPECT_NEAR(norm(f.tangent[i]), 1.0, 1e-12);
    EXPECT_NEAR(norm(f.normal[i]), 1.0, 1e-12);
    EXPECT_NEAR(dot(f.tangent[i], f.normal[i]), 0.0, 1e-12);
    EXPECT_NEAR(cross(f.tangent[i], f.normal[i]), 1.0, 1e-12);  // N = T rotated +90 deg
    EXPECT_DOUBLE_EQ(f.normal[i].x, -f.tangent[i].y);
    EXPECT_DOUBLE_EQ(f.normal[i].y, f.tangent[i].x);
  }
}

// For vertices of a regular polygon on a circle the tangent is exact (see frames.hpp), so only
// rounding remains (~1e-15); 1e-12 is a safe bound.
TEST(Frames, CcwCircleTangentIsPerpendicularToRadiusAndNormalPointsInward) {
  const double r = 40.0;
  const Path c = makeCircle(r, 64);
  const PathFrames f = computeFrames(c);
  for (std::size_t i = 0; i < c.size(); ++i) {
    const Vec2 radial = c.points()[i] * (1.0 / r);
    EXPECT_NEAR(f.tangent[i].x, -radial.y, 1e-12);
    EXPECT_NEAR(f.tangent[i].y, radial.x, 1e-12);
    EXPECT_NEAR(f.normal[i].x, -radial.x, 1e-12);  // left of a CCW path = toward the centre
    EXPECT_NEAR(f.normal[i].y, -radial.y, 1e-12);
    EXPECT_NEAR(f.curvature[i] * r, 1.0, 1e-9);
  }
}

TEST(Frames, ClockwiseCircleHasNegativeCurvatureAndOutwardLeftNormal) {
  const double r = 40.0;
  const Path cw = reversed(makeCircle(r, 64));
  const PathFrames f = computeFrames(cw);
  for (std::size_t i = 0; i < cw.size(); ++i) {
    const Vec2 radial = cw.points()[i] * (1.0 / r);
    EXPECT_NEAR(f.curvature[i] * r, -1.0, 1e-9);
    EXPECT_NEAR(dot(f.normal[i], radial), 1.0, 1e-12);  // outward
  }
}

// Menger curvature flips sign exactly under reversal (cross changes sign, chord lengths are
// unchanged), so only rounding remains: 1e-12 absolute on values of order 1e-2.
TEST(Frames, CurvatureSignFlipsUnderReversalAndIndexMapsToMirror) {
  const Path open = Path::create({{0, 0}, {3, 1}, {6, 0}, {9, -2}, {12, -2.5}, {15, -1}}, Closure::Open);
  const auto k = open.curvature();
  const auto kr = reversed(open).curvature();
  ASSERT_EQ(k.size(), kr.size());
  for (std::size_t i = 0; i < k.size(); ++i) EXPECT_NEAR(kr[i], -k[k.size() - 1 - i], 1e-12) << i;
  // Reversing twice restores the original.
  EXPECT_EQ(reversed(reversed(open)).points(), open.points());
}

TEST(Frames, StraightLineHasZeroCurvatureAndConstantFrame) {
  const Path line = Path::create({{0, 0}, {1, 1}, {3, 3}, {7, 7}, {8, 8}}, Closure::Open);
  const PathFrames f = computeFrames(line);
  const double s = std::sqrt(0.5);
  for (std::size_t i = 0; i < line.size(); ++i) {
    EXPECT_NEAR(f.curvature[i], 0.0, 1e-12);  // collinear: cross product is exactly 0 up to rounding
    EXPECT_NEAR(f.tangent[i].x, s, 1e-12);
    EXPECT_NEAR(f.tangent[i].y, s, 1e-12);
    EXPECT_NEAR(f.normal[i].x, -s, 1e-12);
    EXPECT_NEAR(f.normal[i].y, s, 1e-12);
  }
}

TEST(Frames, NearlyStraightPathStaysFiniteAndTiny) {
  // 1 micrometre lateral wiggle over 100 m spacing: curvature ~ 2e-6/1e4 = 2e-10 1/m, not NaN.
  const Path p = Path::create({{0, 0}, {100, 1e-6}, {200, 0}, {300, -1e-6}, {400, 0}}, Closure::Open);
  for (double k : computeFrames(p).curvature) {
    EXPECT_TRUE(std::isfinite(k));
    EXPECT_LT(std::abs(k), 1e-9);
  }
}

// The weighted-bisector tangent is exact for any three points on a circle (proof in frames.hpp),
// including highly non-uniform spacing. Tolerance: rounding only.
TEST(Frames, TangentIsExactOnCircleWithNonUniformSpacing) {
  const double r = 10.0;
  const double ang[] = {0.0, 0.1, 0.7, 1.9, 2.2, 3.5, 4.0, 5.9};
  std::vector<Vec2> pts;
  for (double a : ang) pts.push_back({r * std::cos(a), r * std::sin(a)});
  const Path c = Path::create(pts, Closure::Closed);
  const PathFrames f = computeFrames(c);
  for (std::size_t i = 0; i < pts.size(); ++i) {
    EXPECT_NEAR(f.tangent[i].x, -std::sin(ang[i]), 1e-12) << i;
    EXPECT_NEAR(f.tangent[i].y, std::cos(ang[i]), 1e-12) << i;
    EXPECT_NEAR(f.curvature[i] * r, 1.0, 1e-12) << i;  // Menger is exact on circles as well
  }
}

TEST(Frames, OpenPathEndpointsUseOneSidedChords) {
  const double r = 10.0;
  std::vector<Vec2> pts;
  const int n = 20;
  for (int i = 0; i <= n; ++i) pts.push_back({r * std::cos(kPi * i / n), r * std::sin(kPi * i / n)});
  const Path arc = Path::create(pts, Closure::Open);
  const PathFrames f = computeFrames(arc);
  // Endpoint tangent equals the first/last chord direction exactly ...
  const Vec2 d0 = pts[1] - pts[0];
  EXPECT_NEAR(f.tangent.front().x, d0.x / norm(d0), 1e-12);
  EXPECT_NEAR(f.tangent.front().y, d0.y / norm(d0), 1e-12);
  // ... which is first-order accurate: off the true tangent (0,1) by half the step angle.
  const double step = kPi / n;
  const double angleErr = std::acos(std::clamp(f.tangent.front().y, -1.0, 1.0));
  EXPECT_NEAR(angleErr, step / 2.0, 1e-9);
  // Interior vertices remain exact.
  EXPECT_NEAR(f.tangent[5].x, -std::sin(kPi * 5 / n), 1e-12);
  // Endpoint curvature copies the nearest interior value (documented).
  EXPECT_DOUBLE_EQ(f.curvature.front(), f.curvature[1]);
  EXPECT_DOUBLE_EQ(f.curvature.back(), f.curvature[n - 1]);
}

TEST(Frames, CuspHasDefinedUnitTangent) {
  // Exact reversal with equal chords: tangent undefined; fallback must still be a finite unit vector.
  const Path p = Path::create({{0, 0}, {1, 0}, {0, 0}}, Closure::Open);
  const PathFrames f = computeFrames(p);
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(norm(f.tangent[i]), 1.0, 1e-12);
    EXPECT_TRUE(std::isfinite(f.curvature[i]));
  }
  EXPECT_EQ(f.curvature[1], 0.0);  // a and c coincide: circumscribed circle undefined -> 0
}

TEST(Frames, ArcLengthMatchesPath) {
  const Path p = makeCircle(5.0, 50);
  EXPECT_EQ(computeFrames(p).arcLength, p.arcLengths());
}

TEST(Frames, CountSignChanges) {
  EXPECT_EQ(countSignChanges({}, 0.0), 0u);
  EXPECT_EQ(countSignChanges({1, 2, 3}, 0.0), 0u);
  EXPECT_EQ(countSignChanges({1, -1, 1, -1}, 0.0), 3u);
  EXPECT_EQ(countSignChanges({1, 0.01, 1}, 0.1), 0u);          // small values ignored
  EXPECT_EQ(countSignChanges({1, 0.01, -0.01, -1}, 0.1), 1u);  // change seen through ignored run
  EXPECT_EQ(countSignChanges({0, 0, 0}, 0.0), 0u);
}

TEST(Frames, StatsReportCurvatureMagnitudeRange) {
  const PathStats st = computeStats(Path::create({{0, 0}, {1, 0}, {2, 0}, {3, 1}, {3, 2}}, Closure::Open));
  EXPECT_GE(st.maxAbsCurvature, st.minAbsCurvature);
  EXPECT_GE(st.maxAbsCurvature, std::max(std::abs(st.minCurvature), std::abs(st.maxCurvature)) - 1e-15);
  const PathStats circ = computeStats(makeCircle(20.0, 90));
  EXPECT_NEAR(circ.minAbsCurvature * 20.0, 1.0, 1e-9);
  EXPECT_NEAR(circ.maxAbsCurvature * 20.0, 1.0, 1e-9);
}
