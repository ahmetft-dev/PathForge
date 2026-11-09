#include <gtest/gtest.h>

#include <cmath>
#include <pathforge/geometry/predicates.hpp>

using namespace pathforge::geometry;

namespace {
Vec2 xf(Vec2 p, double scale, double shift) { return {p.x * scale + shift, p.y * scale + shift}; }
}  // namespace

TEST(Orientation, BasicSigns) {
  EXPECT_EQ(orientation({0, 0}, {1, 0}, {0, 1}), 1);    // left turn (CCW)
  EXPECT_EQ(orientation({0, 0}, {1, 0}, {0, -1}), -1);  // right turn
  EXPECT_EQ(orientation({0, 0}, {1, 1}, {2, 2}), 0);    // collinear
  EXPECT_EQ(orientation({0, 0}, {1, 0}, {0.5, 0}), 0);
}

// Scaling by a power of two and translating by a power of two multiple are exact in binary floating
// point for these small-integer inputs, so the geometry is identical at every magnitude; the
// relative error bound must give identical answers from 1e-9 m to 1e9 m.
TEST(Orientation, ResultIsIndependentOfCoordinateMagnitude) {
  const Vec2 a{0, 0}, b{4, 1}, left{1, 3}, right{1, -3}, col{8, 2};
  for (double scale : {std::ldexp(1.0, -30), 1.0, std::ldexp(1.0, 30)}) {
    for (double shift : {0.0, std::ldexp(1.0, 20) * scale}) {
      EXPECT_EQ(orientation(xf(a, scale, shift), xf(b, scale, shift), xf(left, scale, shift)), 1);
      EXPECT_EQ(orientation(xf(a, scale, shift), xf(b, scale, shift), xf(right, scale, shift)), -1);
      EXPECT_EQ(orientation(xf(a, scale, shift), xf(b, scale, shift), xf(col, scale, shift)), 0);
    }
  }
}

// a=(0,0), b=(1e6,1e6), c=(5e5, 5e5+delta): det = 1e6*delta, error bound = 3.33e-16 * 5e11 = 1.7e-4,
// so offsets above 1.7e-10 m (< ulp(5e5) * 3) are resolved and 0 is reported only at the rounding floor.
TEST(Orientation, ResolvesTinyOffsetsRelativeToMagnitude) {
  const Vec2 a{0, 0}, b{1e6, 1e6};
  EXPECT_EQ(orientation(a, b, {5e5, 5e5 + 1e-3}), 1);
  EXPECT_EQ(orientation(a, b, {5e5, 5e5 - 1e-3}), -1);
  EXPECT_EQ(orientation(a, b, {5e5, 5e5}), 0);
  // At unit scale the same relative geometry (1e-9 relative offset) is also resolved.
  EXPECT_EQ(orientation({0, 0}, {1, 1}, {0.5, 0.5 + 1e-9}), 1);
}

TEST(Segments, ProperCrossingHasCorrectPoint) {
  const SegmentContact c = intersectSegments({0, 0}, {4, 4}, {0, 4}, {4, 0});
  EXPECT_EQ(c.relation, SegmentRelation::ProperCrossing);
  EXPECT_NEAR(c.point.x, 2.0, 1e-12);
  EXPECT_NEAR(c.point.y, 2.0, 1e-12);
}

TEST(Segments, ParallelAndSeparatedAreDisjoint) {
  EXPECT_EQ(intersectSegments({0, 0}, {10, 0}, {0, 1}, {10, 1}).relation, SegmentRelation::Disjoint);
  // Collinear but separated.
  EXPECT_EQ(intersectSegments({0, 0}, {1, 0}, {2, 0}, {3, 0}).relation, SegmentRelation::Disjoint);
  // Pointing at each other but not reaching.
  EXPECT_EQ(intersectSegments({0, 0}, {1, 1}, {3, 0}, {2, 1}).relation, SegmentRelation::Disjoint);
}

TEST(Segments, CollinearOverlapAndEndpointContact) {
  const SegmentContact o = intersectSegments({0, 0}, {4, 0}, {2, 0}, {6, 0});
  EXPECT_EQ(o.relation, SegmentRelation::CollinearOverlap);
  EXPECT_NEAR(o.point.x, 3.0, 1e-12);  // midpoint of the overlap [2, 4]
  EXPECT_EQ(intersectSegments({0, 0}, {2, 0}, {2, 0}, {5, 0}).relation,
            SegmentRelation::Touching);  // end to end
  EXPECT_EQ(intersectSegments({0, 0}, {2, 2}, {2, 2}, {4, 0}).relation,
            SegmentRelation::Touching);  // shared point
  // T junction: end of one lies in the interior of the other.
  const SegmentContact t = intersectSegments({0, 0}, {4, 0}, {2, 0}, {2, 3});
  EXPECT_EQ(t.relation, SegmentRelation::Touching);
  EXPECT_NEAR(t.point.x, 2.0, 1e-12);
  EXPECT_NEAR(t.point.y, 0.0, 1e-12);
}

// A shallow crossing: two 2 km segments that cross at an angle of 2e-9 rad (nearly parallel) must still be a
// proper crossing, while the same pair lifted 1e-6 m apart must be disjoint.
TEST(Segments, NearlyParallelCrossingVersusMiss) {
  const double L = 1000.0;
  EXPECT_EQ(intersectSegments({-L, 0}, {L, 0}, {-L, -1e-6}, {L, 1e-6}).relation,
            SegmentRelation::ProperCrossing);
  EXPECT_EQ(intersectSegments({-L, 0}, {L, 0}, {-L, 1e-6}, {L, 3e-6}).relation, SegmentRelation::Disjoint);
}

TEST(Segments, ClassificationIsInvariantToScaleAndTranslation) {
  struct Case {
    Vec2 p1, p2, q1, q2;
    SegmentRelation expect;
  };
  const Case cases[] = {
      {{0, 0}, {4, 4}, {0, 4}, {4, 0}, SegmentRelation::ProperCrossing},
      {{0, 0}, {4, 0}, {2, 0}, {2, 3}, SegmentRelation::Touching},
      {{0, 0}, {4, 0}, {2, 0}, {6, 0}, SegmentRelation::CollinearOverlap},
      {{0, 0}, {4, 0}, {0, 1}, {4, 1}, SegmentRelation::Disjoint},
      {{0, 0}, {1, 0}, {2, 0}, {3, 0}, SegmentRelation::Disjoint},
  };
  for (double scale : {std::ldexp(1.0, -30), 1.0, std::ldexp(1.0, 30)}) {
    for (double shift : {0.0, std::ldexp(1.0, 20) * scale}) {
      for (const Case& c : cases) {
        EXPECT_EQ(intersectSegments(xf(c.p1, scale, shift), xf(c.p2, scale, shift), xf(c.q1, scale, shift),
                                    xf(c.q2, scale, shift))
                      .relation,
                  c.expect)
            << "scale=" << scale << " shift=" << shift;
      }
    }
  }
}

TEST(PointSegmentDistance, InteriorAndEndpointCases) {
  EXPECT_NEAR(pointSegmentDistance({2, 3}, {0, 0}, {4, 0}), 3.0, 1e-12);   // perpendicular foot inside
  EXPECT_NEAR(pointSegmentDistance({-3, 4}, {0, 0}, {4, 0}), 5.0, 1e-12);  // nearest is endpoint a
  EXPECT_NEAR(pointSegmentDistance({7, 4}, {0, 0}, {4, 0}), 5.0, 1e-12);   // nearest is endpoint b
}

TEST(SelfIntersections, SimplePolylinesHaveNone) {
  EXPECT_TRUE(findSelfIntersections({{0, 0}, {4, 0}, {4, 4}, {0, 4}}, Closure::Closed).empty());
  EXPECT_TRUE(findSelfIntersections({{0, 0}, {4, 0}, {4, 4}, {0, 4}}, Closure::Open).empty());
  EXPECT_TRUE(findSelfIntersections({{0, 0}, {4, 0}, {2, 3}}, Closure::Closed)
                  .empty());  // triangle: all pairs adjacent
}

TEST(SelfIntersections, SharedEndpointsAndSeamAreNotIntersections) {
  // Closed square: segment 3 and segment 0 meet only at the shared vertex 0 (seam adjacency).
  EXPECT_TRUE(findSelfIntersections({{0, 0}, {4, 0}, {4, 4}, {0, 4}}, Closure::Closed).empty());
  // Open loop whose last point sits exactly on the first one is allowed to *touch* only if adjacent:
  // here segment 3 ends on the start point of segment 0 but they are non-adjacent (open) => contact.
  const auto c = findSelfIntersections({{0, 0}, {4, 0}, {4, 4}, {0, 4}, {0, 0}}, Closure::Open);
  ASSERT_EQ(c.size(), 1u);
  EXPECT_EQ(c[0].relation, SegmentRelation::Touching);
}

TEST(SelfIntersections, BowtieCrossesOnceInTheMiddle) {
  const auto c = findSelfIntersections({{0, 0}, {2, 2}, {2, 0}, {0, 2}}, Closure::Closed);
  ASSERT_EQ(c.size(), 1u);
  EXPECT_EQ(c[0].first, 0u);
  EXPECT_EQ(c[0].second, 2u);
  EXPECT_EQ(c[0].relation, SegmentRelation::ProperCrossing);
  EXPECT_NEAR(c[0].point.x, 1.0, 1e-12);
  EXPECT_NEAR(c[0].point.y, 1.0, 1e-12);
}

TEST(SelfIntersections, NonAdjacentTouchAndCollinearOverlap) {
  // T: the last segment ends in the interior of the first.
  const auto t = findSelfIntersections({{0, 0}, {4, 0}, {4, 2}, {2, 2}, {2, 0}}, Closure::Open);
  ASSERT_EQ(t.size(), 1u);
  EXPECT_EQ(t[0].relation, SegmentRelation::Touching);
  // Overlap: the path returns along part of its own first segment.
  const auto o = findSelfIntersections({{0, 0}, {4, 0}, {4, 1}, {3, 1}, {3, 0}, {1, 0}}, Closure::Open);
  bool overlap = false;
  for (const auto& k : o) overlap = overlap || k.relation == SegmentRelation::CollinearOverlap;
  EXPECT_TRUE(overlap);
}

TEST(SelfIntersections, AdjacentFoldBackIsReportedButStraightContinuationIsNot) {
  // Straight continuation (a, b, c collinear, same direction): fine.
  EXPECT_TRUE(findSelfIntersections({{0, 0}, {1, 0}, {3, 0}}, Closure::Open).empty());
  // Spike: path goes out and comes straight back.
  const auto s = findSelfIntersections({{0, 0}, {2, 0}, {1, 0}}, Closure::Open);
  ASSERT_EQ(s.size(), 1u);
  EXPECT_EQ(s[0].first, 0u);
  EXPECT_EQ(s[0].second, 1u);
  EXPECT_EQ(s[0].relation, SegmentRelation::CollinearOverlap);
}

TEST(SelfIntersections, SeamFoldBackIsDetectedOnClosedPaths) {
  // Vertex 0 = (1,0): segment 3 arrives heading -x from (2,0), segment 0 leaves heading +x.
  const auto c = findSelfIntersections({{1, 0}, {3, 0}, {3, 2}, {2, 0}}, Closure::Closed);
  bool seam = false;
  for (const auto& k : c)
    seam = seam || (k.first == 3 && k.second == 0 && k.relation == SegmentRelation::CollinearOverlap);
  EXPECT_TRUE(seam);
}

TEST(SelfIntersections, MaxResultsStopsEarly) {
  // A zig-zag where many segments cross one long segment.
  std::vector<Vec2> p{{0, 0}, {20, 0}};
  for (int i = 1; i <= 8; ++i) {
    p.push_back({2.0 * i, 3.0});
    p.push_back({2.0 * i + 1.0, -3.0});
  }
  EXPECT_GT(findSelfIntersections(p, Closure::Open).size(), 3u);
  EXPECT_EQ(findSelfIntersections(p, Closure::Open, 3).size(), 3u);
}

TEST(Contacts, CrossingAndSeparatePolylines) {
  const std::vector<Vec2> a{{0, 0}, {4, 0}, {4, 4}, {0, 4}};
  const std::vector<Vec2> shifted{{2, 2}, {6, 2}, {6, 6}, {2, 6}};  // overlaps a's top-right corner region
  const auto c = findContacts(a, Closure::Closed, shifted, Closure::Closed);
  EXPECT_EQ(c.size(), 2u);
  const std::vector<Vec2> far{{10, 10}, {11, 10}, {11, 11}};
  EXPECT_TRUE(findContacts(a, Closure::Closed, far, Closure::Closed).empty());
  // Contacts between different polylines include endpoint touching (no adjacency exemption).
  EXPECT_EQ(findContacts({{0, 0}, {4, 0}}, Closure::Open, {{4, 0}, {4, 4}}, Closure::Open).size(), 1u);
}
