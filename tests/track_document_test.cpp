#include <gtest/gtest.h>

#include <limits>
#include <pathforge/editor/track_document.hpp>

using namespace pathforge::editor;

namespace {
TrackDocument threePoints() {
  TrackDocument d;
  d.appendPoint({0, 0}, 5, 5);
  d.appendPoint({50, 0}, 5, 6);
  d.appendPoint({100, 20}, 4, 5);
  return d;
}
}  // namespace

TEST(TrackDocument, StartsEmptyOpenAndUntitled) {
  const TrackDocument d;
  EXPECT_TRUE(d.empty());
  EXPECT_EQ(d.closure(), Closure::Open);
  EXPECT_EQ(d.name(), "Untitled");
  EXPECT_EQ(d.revision(), 0u);
  EXPECT_EQ(d.settings().interpolation, Interpolation::CatmullRom);
}

TEST(TrackDocument, AppendInsertMoveRemoveKeepOrderAndIds) {
  TrackDocument d = threePoints();
  const PointId a = d.points()[0].id, b = d.points()[1].id, c = d.points()[2].id;
  EXPECT_NE(a, 0u);
  EXPECT_NE(a, b);
  EXPECT_NE(b, c);

  const PointId m = d.insertPoint(1, {25, 5}, 5, 5);  // between a and b
  ASSERT_EQ(d.size(), 4u);
  EXPECT_EQ(d.points()[1].id, m);
  EXPECT_EQ(*d.indexOf(b), 2u);  // b moved one index to the right but kept its id
  EXPECT_EQ(d.find(b)->position, (Vec2{50, 0}));

  d.movePoint(b, {55, -3});
  EXPECT_EQ(d.find(b)->position, (Vec2{55, -3}));

  const ControlPoint removed = d.removePoint(m);
  EXPECT_EQ(removed.id, m);
  EXPECT_EQ(removed.position, (Vec2{25, 5}));
  ASSERT_EQ(d.size(), 3u);
  EXPECT_EQ(*d.indexOf(b), 1u);
  EXPECT_FALSE(d.indexOf(m).has_value());
  EXPECT_EQ(d.find(m), nullptr);
}

TEST(TrackDocument, IdsAreNeverReusedAfterDeletion) {
  TrackDocument d = threePoints();
  const PointId last = d.points().back().id;
  d.removePoint(last);
  const PointId fresh = d.appendPoint({1, 1}, 1, 1);
  EXPECT_GT(fresh, last);
}

TEST(TrackDocument, RestoreReinsertsWithOriginalIdAtOriginalIndex) {
  TrackDocument d = threePoints();
  const ControlPoint mid = d.points()[1];
  const ControlPoint removed = d.removePoint(mid.id);
  d.restorePoint(1, removed);
  EXPECT_EQ(d.points()[1], mid);
  // The id is live again, so it cannot be restored twice; unknown future ids and 0 are rejected too.
  EXPECT_THROW(d.restorePoint(0, mid), EditError);
  EXPECT_THROW(d.restorePoint(0, ControlPoint{0, {1, 1}, 1, 1}), EditError);
  EXPECT_THROW(d.restorePoint(0, ControlPoint{999, {1, 1}, 1, 1}), EditError);
}

TEST(TrackDocument, WidthsTravelWithTheirPointOnInsertAndDelete) {
  TrackDocument d = threePoints();
  const PointId b = d.points()[1].id;
  d.setPointWidths(b, 7.5, 2.5);
  d.insertPoint(0, {-50, 0}, 3, 3);
  d.removePoint(d.points()[1].id);  // remove the original first point
  EXPECT_EQ(d.find(b)->leftWidth, 7.5);
  EXPECT_EQ(d.find(b)->rightWidth, 2.5);
}

TEST(TrackDocument, RevisionCountsOnlyRealChanges) {
  TrackDocument d = threePoints();
  const auto r0 = d.revision();
  EXPECT_EQ(r0, 3u);
  const PointId a = d.points()[0].id;
  d.movePoint(a, {0, 0});  // same position: no change
  d.setPointWidths(a, 5, 5);
  d.setClosure(Closure::Open);
  d.setName("Untitled");
  d.setSettings(d.settings());
  EXPECT_EQ(d.revision(), r0);
  d.movePoint(a, {1, 0});
  d.setClosure(Closure::Closed);
  d.setName("Circuit");
  EXPECT_EQ(d.revision(), r0 + 3);
}

TEST(TrackDocument, RejectsInvalidInputAndStaysUnchanged) {
  TrackDocument d = threePoints();
  const TrackDocument before = d;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const PointId a = d.points()[0].id;
  EXPECT_THROW(d.appendPoint({nan, 0}, 1, 1), EditError);
  EXPECT_THROW(d.appendPoint({0, inf}, 1, 1), EditError);
  EXPECT_THROW(d.appendPoint({2e7, 0}, 1, 1), EditError);
  EXPECT_THROW(d.appendPoint({0, 0}, -1, 1), EditError);
  EXPECT_THROW(d.appendPoint({0, 0}, 1, nan), EditError);
  EXPECT_THROW(d.appendPoint({0, 0}, 1, 2e4), EditError);
  EXPECT_THROW(d.insertPoint(9, {0, 0}, 1, 1), EditError);
  EXPECT_THROW(d.movePoint(a, {inf, 0}), EditError);
  EXPECT_THROW(d.movePoint(12345, {0, 0}), EditError);
  EXPECT_THROW(d.setPointWidths(a, 1, -0.5), EditError);
  EXPECT_THROW(d.removePoint(12345), EditError);
  GeometrySettings s;
  s.alpha = 1.5;
  EXPECT_THROW(d.setSettings(s), EditError);
  s = {};
  s.resampleSpacing = -1;
  EXPECT_THROW(d.setSettings(s), EditError);
  s.resampleSpacing = 1e-6;
  EXPECT_THROW(d.setSettings(s), EditError);
  s.resampleSpacing = nan;
  EXPECT_THROW(d.setSettings(s), EditError);
  EXPECT_EQ(d.points(), before.points());
  EXPECT_EQ(d.revision(), before.revision());
  EXPECT_EQ(d.settings(), before.settings());
  // Zero widths are representable (invalid-but-repairable documents); validation reports them.
  EXPECT_NO_THROW(d.setPointWidths(a, 0.0, 5.0));
  s = {};
  s.resampleSpacing = 0.0;  // resampling off
  EXPECT_NO_THROW(d.setSettings(s));
}

TEST(TrackDocument, CopiesAreIndependentSnapshots) {
  TrackDocument d = threePoints();
  const TrackDocument snapshot = d;
  d.movePoint(d.points()[0].id, {9, 9});
  EXPECT_EQ(snapshot.points()[0].position, (Vec2{0, 0}));
  EXPECT_LT(snapshot.revision(), d.revision());
}
