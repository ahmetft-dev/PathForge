#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <pathforge/editor/derived_geometry.hpp>
#include <pathforge/geometry/validation.hpp>

using namespace pathforge::editor;
using pathforge::geometry::TrackIssue;
using pathforge::geometry::ValidationStatus;

namespace {
constexpr double kPi = std::numbers::pi;

TrackDocument circleDoc(double r, int m, double w, Interpolation interp, double spacing) {
  TrackDocument d;
  d.setClosure(Closure::Closed);
  GeometrySettings s;
  s.interpolation = interp;
  s.resampleSpacing = spacing;
  d.setSettings(s);
  for (int i = 0; i < m; ++i)
    d.appendPoint({r * std::cos(2 * kPi * i / m), r * std::sin(2 * kPi * i / m)}, w, w);
  return d;
}

bool hasIssue(const pathforge::geometry::ValidationReport& r, TrackIssue t) {
  for (const auto& i : r.issues) {
    if (i.type == t) return true;
  }
  return false;
}
}  // namespace

TEST(DerivedGeometry, TooFewPointsKeepsRawControlDataForDiagnostics) {
  TrackDocument d;
  EXPECT_EQ(generateGeometry(d).status, DerivedStatus::TooFewPoints);
  d.appendPoint({1, 2}, 3, 4);
  DerivedGeometry g = generateGeometry(d);
  EXPECT_EQ(g.status, DerivedStatus::TooFewPoints);
  EXPECT_FALSE(g.track.has_value());
  ASSERT_EQ(g.data.centerline.size(), 1u);
  EXPECT_EQ(g.data.centerline[0], (Vec2{1, 2}));
  EXPECT_EQ(validateTrackData(g.data).status(), ValidationStatus::Invalid);
  EXPECT_TRUE(hasIssue(validateTrackData(g.data), TrackIssue::TooFewPoints));
  d.appendPoint({10, 2}, 3, 4);
  EXPECT_EQ(generateGeometry(d).status, DerivedStatus::Ok);  // two points make an open track
  d.setClosure(Closure::Closed);
  EXPECT_EQ(generateGeometry(d).status, DerivedStatus::TooFewPoints);  // but not a closed one
}

TEST(DerivedGeometry, CoincidentControlPointsFailWithExplanation) {
  TrackDocument d;
  d.appendPoint({0, 0}, 5, 5);
  d.appendPoint({10, 0}, 5, 5);
  d.appendPoint({10, 0}, 5, 5);
  const DerivedGeometry g = generateGeometry(d);
  EXPECT_EQ(g.status, DerivedStatus::Failed);
  EXPECT_NE(g.message.find("coincide"), std::string::npos);
  EXPECT_TRUE(hasIssue(validateTrackData(g.data), TrackIssue::DegenerateSegment));
}

// Polyline interpolation without resampling must reproduce the control points and widths EXACTLY: this is the
// representation used for imported sampled tracks, whose geometry must not change. At a knot the
// interpolation formula reduces to w0 + (w1 - w0) * 0, so equality is exact, not approximate.
TEST(DerivedGeometry, PolylineWithoutResamplingIsExact) {
  TrackDocument d = circleDoc(50.0, 40, 4.0, Interpolation::Polyline, 0.0);
  d.setPointWidths(d.points()[7].id, 6.5, 2.25);
  const DerivedGeometry g = generateGeometry(d);
  ASSERT_EQ(g.status, DerivedStatus::Ok);
  ASSERT_EQ(g.data.centerline.size(), d.size());
  for (std::size_t i = 0; i < d.size(); ++i) {
    EXPECT_EQ(g.data.centerline[i], d.points()[i].position);
    EXPECT_EQ(g.data.leftWidths[i], d.points()[i].leftWidth);
    EXPECT_EQ(g.data.rightWidths[i], d.points()[i].rightWidth);
  }
  EXPECT_EQ(g.denseSamplesPerSegment, 1u);
}

// 16 control points on a circle of radius 100: the centripetal spline deviates radially by 5.5e-2 m at most
// (docs/curvature-study.md); resampling onto that curve adds no deviation beyond chord sagitta h^2/(8R) =
// 1.25e-3 m for h = 1 m. Bound: 0.06 m.
TEST(DerivedGeometry, ClosedSplineIsResampledUniformlyWithoutSeamDuplicate) {
  const double r = 100.0;
  const DerivedGeometry g = generateGeometry(circleDoc(r, 16, 5.0, Interpolation::CatmullRom, 1.0));
  ASSERT_EQ(g.status, DerivedStatus::Ok);
  ASSERT_TRUE(g.track.has_value());
  const auto& c = g.data.centerline;
  for (const Vec2& p : c) EXPECT_NEAR(norm(p), r, 0.06);
  EXPECT_NE(c.front(), c.back());  // seam not duplicated
  double lo = 1e300, hi = 0;
  for (std::size_t k = 0; k < c.size(); ++k) {
    const double h = distance(c[k], c[(k + 1) % c.size()]);  // includes the closing segment
    lo = std::min(lo, h);
    hi = std::max(hi, h);
  }
  // Chords on a curve of radius ~100 m with ~1 m arc spacing are shorter than the arc by (h/R)^2/24 ~ 4e-6.
  EXPECT_NEAR(lo / hi, 1.0, 1e-4);
  EXPECT_NEAR(hi, 1.0, 0.01);  // spacing adjusted to fit the length exactly: |h - 1| <= 0.5 / N
  // Sample 0 is the start of the dense polyline, i.e. control point 0 itself.
  EXPECT_EQ(c.front(), (Vec2{r, 0.0}));
}

// Constant widths are reproduced exactly (every knot interval has w1 - w0 = 0); variable widths are a linear
// interpolation in arc length, so each derived width lies between the widths of the two neighbouring control
// points, and at s = 0 (control point 0) it equals that control point's width exactly.
TEST(DerivedGeometry, WidthsFollowControlPointsInArcLength) {
  TrackDocument d = circleDoc(100.0, 12, 5.0, Interpolation::CatmullRom, 2.0);
  DerivedGeometry g = generateGeometry(d);
  for (double w : g.data.leftWidths) EXPECT_EQ(w, 5.0);
  d.setPointWidths(d.points()[0].id, 3.0, 9.0);
  d.setPointWidths(d.points()[6].id, 8.0, 4.0);
  g = generateGeometry(d);
  ASSERT_EQ(g.status, DerivedStatus::Ok);
  EXPECT_EQ(g.data.leftWidths[0], 3.0);
  EXPECT_EQ(g.data.rightWidths[0], 9.0);
  for (std::size_t k = 0; k < g.data.centerline.size(); ++k) {
    EXPECT_GE(g.data.leftWidths[k], 3.0 - 1e-12);
    EXPECT_LE(g.data.leftWidths[k], 8.0 + 1e-12);
  }
  // Boundaries are regenerated from the per-sample widths.
  EXPECT_NEAR(norm(g.track->leftBoundary()[0]), norm(g.data.centerline[0]) - 3.0, 1e-6);
  // Closed profile: continuous through the seam (last sample close to the first in width).
  EXPECT_NEAR(g.data.leftWidths.back(), g.data.leftWidths.front(), 0.1);
}

TEST(DerivedGeometry, OpenSplineKeepsExactEndPoints) {
  TrackDocument d;
  d.appendPoint({0, 0}, 4, 4);
  d.appendPoint({60, 25}, 4, 4);
  d.appendPoint({120, -10}, 4, 4);
  d.appendPoint({200, 5}, 4, 4);
  const DerivedGeometry g = generateGeometry(d);
  ASSERT_EQ(g.status, DerivedStatus::Ok);
  EXPECT_EQ(g.data.centerline.front(), (Vec2{0, 0}));
  EXPECT_EQ(g.data.centerline.back(), (Vec2{200, 5}));
  EXPECT_EQ(g.sampleArcLength.back(), g.parameterLength);
  EXPECT_EQ(g.controlArcLength.back(), g.parameterLength);
  EXPECT_EQ(g.controlArcLength.front(), 0.0);
}

TEST(DerivedGeometry, IsDeterministicAndRegeneratesAfterEdits) {
  TrackDocument d = circleDoc(80.0, 10, 5.0, Interpolation::CatmullRom, 1.5);
  const DerivedGeometry a = generateGeometry(d);
  const DerivedGeometry b = generateGeometry(d);
  EXPECT_EQ(a.data.centerline, b.data.centerline);
  EXPECT_EQ(a.data.leftWidths, b.data.leftWidths);

  const PointId moved = d.points()[3].id;
  const Vec2 original = d.find(moved)->position;
  d.movePoint(moved, original + Vec2{0.0, 15.0});
  const DerivedGeometry c = generateGeometry(d);
  EXPECT_NE(c.data.centerline, a.data.centerline);
  d.movePoint(moved, original);  // moving back restores identical derived geometry
  EXPECT_EQ(generateGeometry(d).data.centerline, a.data.centerline);
}

TEST(DerivedGeometry, ValidationOfDerivedDataDetectsSelfIntersection) {
  TrackDocument d;
  d.setClosure(Closure::Closed);
  GeometrySettings s;
  s.interpolation = Interpolation::Polyline;
  d.setSettings(s);
  for (Vec2 p : {Vec2{0, 0}, Vec2{200, 200}, Vec2{200, 0}, Vec2{0, 200}}) d.appendPoint(p, 1, 1);  // bowtie
  const DerivedGeometry g = generateGeometry(d);
  ASSERT_EQ(g.status, DerivedStatus::Ok);
  EXPECT_TRUE(hasIssue(validateTrackData(g.data), TrackIssue::CenterlineSelfIntersection));
}

TEST(DerivedGeometry, ImpossibleSampleCountsFailInsteadOfExhaustingMemory) {
  TrackDocument d;
  d.appendPoint({-9e6, 0}, 5, 5);
  d.appendPoint({9e6, 0}, 5, 5);
  GeometrySettings s;
  s.interpolation = Interpolation::Polyline;
  s.resampleSpacing = 0.001;  // 18,000 km at 1 mm
  d.setSettings(s);
  const DerivedGeometry g = generateGeometry(d);
  EXPECT_EQ(g.status, DerivedStatus::Failed);
  EXPECT_NE(g.message.find("spacing"), std::string::npos);
}

TEST(DerivedGeometry, InsertionMapsCurvePositionToControlSegment) {
  TrackDocument d;
  GeometrySettings s;
  s.interpolation = Interpolation::Polyline;
  s.resampleSpacing = 10.0;
  d.setSettings(s);
  d.appendPoint({0, 0}, 5, 5);
  d.appendPoint({100, 0}, 5, 5);
  d.appendPoint({200, 0}, 5, 5);
  DerivedGeometry g = generateGeometry(d);
  auto t = findInsertion(d, g, {150, 3});
  ASSERT_TRUE(t.has_value());
  EXPECT_EQ(t->index, 2u);
  EXPECT_NEAR(t->position.x, 150.0, 1e-9);
  EXPECT_NEAR(t->position.y, 0.0, 1e-9);
  EXPECT_NEAR(t->distance, 3.0, 1e-9);
  EXPECT_EQ(findInsertion(d, g, {20, -1})->index, 1u);
  EXPECT_EQ(findInsertion(d, g, {300, 0})->index, 2u);  // beyond the open end: last segment

  // Closed square: a click next to the closing segment (last -> first) inserts after the last point.
  TrackDocument sq;
  sq.setClosure(Closure::Closed);
  sq.setSettings(s);
  for (Vec2 p : {Vec2{0, 0}, Vec2{100, 0}, Vec2{100, 100}, Vec2{0, 100}}) sq.appendPoint(p, 5, 5);
  g = generateGeometry(sq);
  EXPECT_EQ(findInsertion(sq, g, {-2, 50})->index, 4u);
  EXPECT_EQ(findInsertion(sq, g, {50, 98})->index, 3u);

  // Spline: the mapping goes through the arc-length parameter, not the control polygon.
  TrackDocument sp = circleDoc(100.0, 8, 5.0, Interpolation::CatmullRom, 1.0);
  g = generateGeometry(sp);
  const double mid = 2 * kPi * 2.5 / 8;  // halfway between control points 2 and 3
  EXPECT_EQ(findInsertion(sp, g, {100 * std::cos(mid), 100 * std::sin(mid)})->index, 3u);

  TrackDocument one;
  one.appendPoint({0, 0}, 1, 1);
  EXPECT_FALSE(findInsertion(one, generateGeometry(one), {1, 1}).has_value());
}
