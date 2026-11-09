#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <pathforge/geometry/circle.hpp>
#include <pathforge/geometry/shapes.hpp>
#include <pathforge/geometry/validation.hpp>

using namespace pathforge::geometry;

namespace {
constexpr double kPi = std::numbers::pi;

TrackData dataOf(std::vector<Vec2> pts, Closure closure, double left, double right) {
  TrackData d;
  d.name = "t";
  d.closure = closure;
  d.centerline = std::move(pts);
  d.leftWidths.assign(d.centerline.size(), left);
  d.rightWidths.assign(d.centerline.size(), right);
  return d;
}

std::size_t count(const ValidationReport& r, TrackIssue type) {
  std::size_t n = 0;
  for (const auto& i : r.issues) n += i.type == type;
  return n;
}

const ValidationIssue* first(const ValidationReport& r, TrackIssue type) {
  for (const auto& i : r.issues) {
    if (i.type == type) return &i;
  }
  return nullptr;
}

// Three horizontal legs joined by 180-degree turns (open "serpentine"): leg 1 heads +x, leg 2 -x, leg 3 +x.
std::vector<Vec2> serpentine(double spacing, double legLength) {
  std::vector<Vec2> p;
  const int legPts = 8, arcPts = 12;
  for (int i = 0; i < legPts; ++i) p.push_back({legLength * i / legPts, 0.0});  // leg 1, excludes end
  const double r = spacing / 2.0;
  for (int i = 0; i <= arcPts; ++i) {  // left U-turn (CCW) from (L, 0) around (L, r) to (L, spacing)
    const double a = -kPi / 2.0 + kPi * i / arcPts;
    p.push_back({legLength + r * std::cos(a), r + r * std::sin(a)});
  }
  for (int i = 1; i < legPts; ++i) p.push_back({legLength - legLength * i / legPts, spacing});  // leg 2
  for (int i = 0; i <= arcPts; ++i) {  // right U-turn (CW) around (0, spacing + r)
    const double a = -kPi / 2.0 - kPi * i / arcPts;
    p.push_back({r * std::cos(a), spacing + r + r * std::sin(a)});
  }
  for (int i = 1; i <= legPts; ++i) p.push_back({legLength * i / legPts, 2.0 * spacing});  // leg 3
  return p;
}
}  // namespace

TEST(Validation, WellFormedPresetsAreValid) {
  const Track circle = Track::create(makeCircle(100.0, 360), 5.0, 5.0);
  const ValidationReport rc = validateTrack(circle);
  EXPECT_EQ(rc.status(), ValidationStatus::Valid);
  EXPECT_EQ(rc.errorCount, 0u);
  EXPECT_TRUE(rc.checked.all());
  const ValidationReport ro = validateTrack(Track::create(makeStadium(300.0, 120.0, 512), 5.0, 5.0));
  EXPECT_EQ(ro.status(), ValidationStatus::Valid);
  const ValidationReport rs = validateTrack(Track::create(makeSCurve(300.0, 30.0, 256), 5.0, 5.0));  // open
  EXPECT_EQ(rs.status(), ValidationStatus::Valid);
  EXPECT_EQ(rs.warningCount, 0u);
}

// For the CCW circle the boundaries are regular polygons of radius R -/+ w. The distance from a left
// vertex (radius 95) to the right polygon (vertices at 105): along the radial direction it is exactly w_total
// = 10; the nearest point may lie on an adjacent edge, whose line is rotated by pi/N from the tangent, which
// shortens the distance to at least 10 cos(pi/N). So  10 cos(pi/N) <= clearance <= 10  (plus rounding).
TEST(Validation, CircleClearanceMatchesGeometry) {
  const std::size_t n = 360;
  const ValidationReport r = validateTrack(Track::create(makeCircle(100.0, n), 5.0, 5.0));
  ASSERT_TRUE(r.clearance.computed);
  EXPECT_NEAR(r.clearance.minConfiguredTotalWidth, 10.0, 1e-12);
  EXPECT_NEAR(r.clearance.minCorrespondingDistance, 10.0, 1e-9);
  EXPECT_LE(r.clearance.minBoundaryDistance, 10.0 + 1e-9);
  EXPECT_GE(r.clearance.minBoundaryDistance, 10.0 * std::cos(kPi / static_cast<double>(n)) - 1e-9);
  ASSERT_EQ(r.clearance.vertexClearance.size(), n);
}

TEST(Validation, ValidityIsInvariantToCoordinateMagnitudeAndOffset) {
  // The same circle (radius 100 units, half-width 5 units) expressed in meters from 1e-3 to 1e3 scale and
  // shifted 1e6 m away from the origin. Thresholds are scaled with the geometry so only the predicate's
  // behaviour is tested.
  for (double scale : {1e-3, 1.0, 1e3}) {
    for (double shift : {0.0, 1e6}) {
      TrackData d = toTrackData(Track::create(makeCircle(100.0 * scale, 360), 5.0 * scale, 5.0 * scale), "c");
      for (Vec2& p : d.centerline) p = p + Vec2{shift, -shift};
      ValidationOptions o;
      o.narrowWidthWarning = 1e-6;
      o.collapseClearance = 1e-9;
      const ValidationReport r = validateTrackData(d, o);
      EXPECT_EQ(r.status(), ValidationStatus::Valid) << "scale=" << scale << " shift=" << shift;
      EXPECT_EQ(r.errorCount, 0u) << "scale=" << scale << " shift=" << shift;
    }
  }
}

TEST(Validation, CenterlineSelfIntersectionIsReportedWithSegmentsAndLocation) {
  const TrackData d =
      dataOf({{0, 0}, {200, 200}, {200, 0}, {0, 200}}, Closure::Closed, 0.01, 0.01);  // bowtie
  const ValidationReport r = validateTrackData(d);
  EXPECT_EQ(r.status(), ValidationStatus::Invalid);
  const ValidationIssue* i = first(r, TrackIssue::CenterlineSelfIntersection);
  ASSERT_NE(i, nullptr);
  EXPECT_EQ(i->severity, Severity::Error);
  ASSERT_EQ(i->segments.size(), 2u);
  EXPECT_EQ(i->segments[0].polyline, PolylineId::Centerline);
  EXPECT_EQ(i->segments[0].index, 0u);
  EXPECT_EQ(i->segments[1].index, 2u);
  ASSERT_TRUE(i->location.has_value());
  EXPECT_NEAR(i->location->x, 100.0, 1e-9);
  EXPECT_NEAR(i->location->y, 100.0, 1e-9);
  EXPECT_FALSE(i->message.empty());
}

// Two straight legs meeting in a sharp 60-degree left corner, vertices every 2 m. The left (inner) offset of
// 3 m is larger than the vertex spacing times tan(30 deg) ~ 1.15 m, so the offset vertices of both legs
// overshoot the miter point and the left boundary crosses itself, while the centerline is simple and the
// right boundary (0.5 m, outer side) is fine. Checked against the engine, not an analytic miter formula: the
// exact crossing is a discretisation effect of per-vertex offsetting.
TEST(Validation, ValidCenterlineButSelfIntersectingBoundary) {
  std::vector<Vec2> p;
  for (int i = 0; i <= 10; ++i) p.push_back({2.0 * i, 0.0});
  const double c = std::cos(2.0 * kPi / 3.0), s = std::sin(2.0 * kPi / 3.0);
  for (int i = 1; i <= 10; ++i) p.push_back({20.0 + 2.0 * i * c, 2.0 * i * s});
  const ValidationReport r = validateTrackData(dataOf(p, Closure::Open, 3.0, 0.5));
  EXPECT_EQ(r.status(), ValidationStatus::Invalid);
  EXPECT_EQ(count(r, TrackIssue::CenterlineSelfIntersection), 0u);  // the centerline itself is simple
  EXPECT_EQ(count(r, TrackIssue::RightBoundarySelfIntersection), 0u);
  EXPECT_EQ(count(r, TrackIssue::BoundaryCrossing), 0u);
  const ValidationIssue* i = first(r, TrackIssue::LeftBoundarySelfIntersection);
  ASSERT_NE(i, nullptr);
  ASSERT_EQ(i->segments.size(), 2u);
  EXPECT_EQ(i->segments[0].polyline, PolylineId::LeftBoundary);
  EXPECT_EQ(i->segments[1].polyline, PolylineId::LeftBoundary);
  ASSERT_TRUE(i->location.has_value());
  // With a 1 m offset the same corner is fine (no fold, just a narrow-track warning).
  EXPECT_EQ(validateTrackData(dataOf(p, Closure::Open, 1.0, 0.5)).errorCount, 0u);
}

TEST(Validation, LeftAndRightBoundariesCrossWhenStrandsOfOppositeSidesOverlap) {
  // Legs 1 and 3 both head +x and are 20 m apart; with 12 m half-widths leg 1's left boundary (y = 12) passes
  // beyond leg 3's right boundary (y = 8).
  const TrackData d = dataOf(serpentine(10.0, 80.0), Closure::Open, 12.0, 12.0);
  const ValidationReport r = validateTrackData(d);
  EXPECT_EQ(r.status(), ValidationStatus::Invalid);
  const ValidationIssue* i = first(r, TrackIssue::BoundaryCrossing);
  ASSERT_NE(i, nullptr);
  ASSERT_EQ(i->segments.size(), 2u);
  EXPECT_EQ(i->segments[0].polyline, PolylineId::LeftBoundary);
  EXPECT_EQ(i->segments[1].polyline, PolylineId::RightBoundary);
  EXPECT_DOUBLE_EQ(r.clearance.minBoundaryDistance, 0.0);  // polylines meet
}

TEST(Validation, ConfiguredWidthIsNotTheActualClearance) {
  // Stadium with straights 6 m apart (width 6, corner radius 3) and 4 m half-widths. Every vertex has 8 m of
  // configured width and the same-vertex left/right distance is 8 m, but the left boundary of the bottom
  // strand (y = +1) faces the right boundary of the top strand (y = +7): the actual gap between the boundary
  // polylines is 6 m. The 4 m offset also exceeds the 3 m corner radius, so the track is invalid (reversed
  // offset segments).
  const TrackData d = toTrackData(Track::create(makeStadium(300.0, 6.0, 512), 4.0, 4.0), "tight");
  const ValidationReport r = validateTrackData(d);
  ASSERT_TRUE(r.clearance.computed);
  EXPECT_NEAR(r.clearance.minConfiguredTotalWidth, 8.0, 1e-12);
  EXPECT_NEAR(r.clearance.minCorrespondingDistance, 8.0, 1e-9);  // same-vertex distance: looks fine
  EXPECT_LT(r.clearance.minBoundaryDistance, 6.0 + 1e-6);        // real gap between the polylines
  EXPECT_LT(r.clearance.minBoundaryDistance, r.clearance.minCorrespondingDistance - 1.0);
  EXPECT_EQ(r.status(), ValidationStatus::Invalid);
  EXPECT_GT(count(r, TrackIssue::LocalBoundaryFold), 0u);
}

TEST(Validation, LocalFoldWhenOffsetExceedsRadiusOfCurvature) {
  const ValidationReport r = validateTrack(Track::create(makeCircle(10.0, 90), 15.0, 1.0));
  EXPECT_EQ(r.status(), ValidationStatus::Invalid);
  EXPECT_EQ(count(r, TrackIssue::LocalBoundaryFold),
            r.issues.size() - count(r, TrackIssue::BoundaryCollapse) - count(r, TrackIssue::NarrowSection));
  const ValidationIssue* f = first(r, TrackIssue::LocalBoundaryFold);
  ASSERT_NE(f, nullptr);
  EXPECT_EQ(f->severity, Severity::Error);
  EXPECT_EQ(f->segments[0].polyline, PolylineId::LeftBoundary);
}

TEST(Validation, ZeroNegativeAndMismatchedWidths) {
  TrackData zero = toTrackData(Track::create(makeCircle(50.0, 64), 5.0, 5.0), "z");
  zero.leftWidths[10] = 0.0;
  const ValidationReport rz = validateTrackData(zero);
  EXPECT_EQ(rz.status(), ValidationStatus::Invalid);
  EXPECT_EQ(count(rz, TrackIssue::InvalidWidth), 1u);
  EXPECT_FALSE(rz.checked.collapse);  // boundary-vs-centerline group could not run meaningfully

  TrackData neg = zero;
  neg.leftWidths[10] = -1.0;
  const ValidationReport rn = validateTrackData(neg);
  EXPECT_EQ(rn.status(), ValidationStatus::Invalid);
  EXPECT_EQ(count(rn, TrackIssue::InvalidWidth), 1u);
  EXPECT_FALSE(rn.checked.structure);  // geometric checks skipped
  EXPECT_FALSE(rn.checked.all());

  TrackData mis = zero;
  mis.rightWidths.pop_back();
  EXPECT_GT(count(validateTrackData(mis), TrackIssue::WidthCountMismatch), 0u);

  TrackData nan = zero;
  nan.leftWidths[3] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(validateTrackData(nan).status(), ValidationStatus::Invalid);
}

TEST(Validation, StructuralProblemsSkipGeometricChecks) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const ValidationReport rn = validateTrackData(dataOf({{0, 0}, {nan, 1}, {2, 2}}, Closure::Open, 1, 1));
  EXPECT_EQ(count(rn, TrackIssue::NonFiniteCoordinate), 1u);
  EXPECT_FALSE(rn.checked.structure);
  EXPECT_FALSE(rn.checked.centerline);
  EXPECT_EQ(rn.status(), ValidationStatus::Invalid);

  EXPECT_EQ(count(validateTrackData(dataOf({}, Closure::Closed, 1, 1)), TrackIssue::TooFewPoints), 1u);
  EXPECT_EQ(
      count(validateTrackData(dataOf({{0, 0}, {1, 0}}, Closure::Closed, 1, 1)), TrackIssue::TooFewPoints),
      1u);
  EXPECT_EQ(count(validateTrackData(dataOf({{0, 0}}, Closure::Open, 1, 1)), TrackIssue::TooFewPoints), 1u);

  const ValidationReport dup =
      validateTrackData(dataOf({{0, 0}, {5, 0}, {5, 0}, {9, 3}}, Closure::Open, 1, 1));
  const ValidationIssue* d = first(dup, TrackIssue::DegenerateSegment);
  ASSERT_NE(d, nullptr);
  EXPECT_EQ(d->segments[0].index, 1u);

  // Closed path whose last point repeats the first: degenerate closing segment (invalid closed-loop
  // geometry).
  const ValidationReport seam =
      validateTrackData(dataOf({{0, 0}, {10, 0}, {10, 10}, {0, 0}}, Closure::Closed, 1, 1));
  const ValidationIssue* s = first(seam, TrackIssue::DegenerateSegment);
  ASSERT_NE(s, nullptr);
  EXPECT_EQ(s->segments[0].index, 3u);
}

TEST(Validation, NarrowSectionsWarnAndCollapsedSectionsFail) {
  const Path c = makeCircle(100.0, 360);
  const ValidationReport narrow = validateTrack(Track::create(c, 0.75, 0.75));  // 1.5 m total < 2 m default
  EXPECT_EQ(narrow.status(), ValidationStatus::Valid);  // a warning does not invalidate
  EXPECT_GT(narrow.warningCount, 0u);
  EXPECT_EQ(count(narrow, TrackIssue::NarrowSection), 1u);  // one issue for the whole run of vertices

  const ValidationReport collapsed = validateTrack(Track::create(c, 0.0025, 0.0025));  // 5 mm total < 1 cm
  EXPECT_EQ(collapsed.status(), ValidationStatus::Invalid);
  EXPECT_GT(count(collapsed, TrackIssue::BoundaryCollapse), 0u);

  ValidationOptions o;
  o.narrowWidthWarning = 0.5;  // threshold is configurable
  EXPECT_EQ(validateTrack(Track::create(c, 0.75, 0.75), o).warningCount, 0u);
}

TEST(Validation, IssueListIsCappedButCountsAreComplete) {
  ValidationOptions o;
  o.maxIssuesPerType = 3;
  const ValidationReport r =
      validateTrack(Track::create(makeCircle(10.0, 90), 15.0, 1.0), o);  // 90+ fold issues
  EXPECT_LE(count(r, TrackIssue::LocalBoundaryFold), 3u);
  EXPECT_GT(r.errorCount, r.issues.size());
  EXPECT_GT(r.suppressed, 0u);
}

TEST(Validation, StatusDistinguishesValidInvalidAndIncomplete) {
  ValidationReport r;
  r.checked.structure = true;
  EXPECT_EQ(r.status(), ValidationStatus::Incomplete);  // nothing found, but not everything was checked
  r.checked = CheckCoverage{true, true, true, true, true, true, true};
  EXPECT_EQ(r.status(), ValidationStatus::Valid);
  r.errorCount = 1;
  EXPECT_EQ(r.status(), ValidationStatus::Invalid);
  EXPECT_STREQ(toString(ValidationStatus::Incomplete), "incomplete");
  EXPECT_STREQ(toString(TrackIssue::BoundaryCrossing), "BoundaryCrossing");
  EXPECT_STREQ(toString(Severity::Warning), "warning");
}

TEST(Validation, VariableWidthTrackIsValidatedPerVertex) {
  const Path c = makeCircle(100.0, 360);
  std::vector<double> w(360, 5.0);
  const ValidationReport ok = validateTrack(Track::create(c, w, w));
  EXPECT_EQ(ok.status(), ValidationStatus::Valid);
  w[100] = 0.0;  // a single collapsed sample
  const ValidationReport bad = validateTrack(Track::create(c, w, std::vector<double>(360, 5.0)));
  EXPECT_EQ(bad.status(), ValidationStatus::Invalid);
  const ValidationIssue* i = first(bad, TrackIssue::InvalidWidth);
  ASSERT_NE(i, nullptr);
  EXPECT_NE(i->message.find("100"), std::string::npos);
}
