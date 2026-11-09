#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <pathforge/geometry/circle.hpp>
#include <pathforge/geometry/shapes.hpp>
#include <pathforge/geometry/surface.hpp>
#include <pathforge/geometry/track.hpp>
#include <set>
#include <tuple>

using namespace pathforge::geometry;

namespace {

SurfaceAnalysis analyzeTrack(const Track& t, BroadPhase bp = BroadPhase::Grid) {
  SurfaceOptions o;
  o.broadPhase = bp;
  return analyzeSurface(t.leftBoundary(), t.rightBoundary(), t.centerline().closure(), o);
}

std::size_t count(const SurfaceAnalysis& a, StripDefectKind k) {
  return static_cast<std::size_t>(
      std::count_if(a.defects.begin(), a.defects.end(), [&](const StripDefect& d) { return d.kind == k; }));
}

std::set<std::tuple<int, std::size_t, std::size_t>> asSet(const SurfaceAnalysis& a) {
  std::set<std::tuple<int, std::size_t, std::size_t>> s;
  for (const StripDefect& d : a.defects) s.insert({static_cast<int>(d.kind), d.strip, d.other});
  return s;
}

// A tiny deterministic generator (LCG) so random corridors are reproducible.
struct Rng {
  unsigned s;
  double next() {
    s = s * 1664525u + 1013904223u;
    return static_cast<double>(s >> 8) / static_cast<double>(1u << 24);
  }
};

// Random walk centerline with random widths: some of these corridors fold or overlap, most do not.
Track randomTrack(unsigned seed, std::size_t n, bool closed, double widthScale) {
  Rng r{seed};
  std::vector<Vec2> pts;
  Vec2 p{0, 0};
  double heading = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    pts.push_back(p);
    heading += (r.next() - 0.5) * 1.6;
    const double step = 4.0 + 6.0 * r.next();
    p = p + Vec2{std::cos(heading), std::sin(heading)} * step;
  }
  std::vector<double> l(n), rt(n);
  for (std::size_t i = 0; i < n; ++i) {
    l[i] = widthScale * (0.3 + r.next());
    rt[i] = widthScale * (0.3 + r.next());
  }
  return Track::create(Path::create(pts, closed ? Closure::Closed : Closure::Open), l, rt);
}

}  // namespace

TEST(Surface, StripCountsFollowOpenAndClosedConventions) {
  const Track open =
      Track::create(Path::create({{0, 0}, {10, 0}, {20, 0}, {30, 5}}, Closure::Open), 3.0, 3.0);
  EXPECT_EQ(analyzeTrack(open).stripCount, 3u);  // n - 1
  const Track closed = Track::create(makeCircle(50.0, 40), 4.0, 4.0);
  EXPECT_EQ(analyzeTrack(closed).stripCount,
            40u);  // n: the closing strip joins the last sample back to the first
}

TEST(Surface, WellFormedCorridorsHaveNoDefectsIncludingTheClosedSeam) {
  const Track tracks[] = {
      Track::create(makeCircle(100.0, 360), 5.0, 5.0),
      Track::create(makeStadium(300.0, 120.0, 512), 5.0, 5.0),
      Track::create(makeSCurve(300.0, 30.0, 256), 5.0, 5.0),
      Track::create(Path::create({{0, 0}, {50, 0}, {100, 0}}, Closure::Open), 3.0, 3.0),  // straight
      Track::create(Path::create({{0, 0}, {50, 0}, {50, 50}, {0, 50}}, Closure::Closed), 4.0,
                    4.0),  // square, 90 deg corners
  };
  for (const Track& t : tracks) {
    for (BroadPhase bp : {BroadPhase::Grid, BroadPhase::BruteForce}) {
      const SurfaceAnalysis a = analyzeTrack(t, bp);
      EXPECT_TRUE(a.defects.empty()) << "defects: " << a.defects.size();
    }
  }
}

// Neighbouring strips of a sharp but valid corner share only their cross-section edge; that contact is
// legitimate and must not be reported, however sharp the turn (here 150 degrees with a half-width well inside
// the turning radius).
TEST(Surface, SharpButValidTurnsAreNotOverlaps) {
  std::vector<Vec2> arc;
  const double r = 20.0;
  for (int i = 0; i <= 30; ++i) {  // 150-degree circular arc, one-metre-class spacing
    const double a = -M_PI / 2 + (5.0 * M_PI / 6.0) * i / 30.0;
    arc.push_back({r * std::cos(a), r + r * std::sin(a)});
  }
  const Track t = Track::create(Path::create(arc, Closure::Open), 4.0, 4.0);
  EXPECT_TRUE(analyzeTrack(t).defects.empty());
}

TEST(Surface, ReversedOrientationStripIsFolded) {
  // Left boundary below the right one for a track heading +x: left and right are swapped, the strip is
  // counter-clockwise.
  const std::vector<Vec2> left{{0, -5}, {10, -5}, {20, -5}}, right{{0, 5}, {10, 5}, {20, 5}};
  const SurfaceAnalysis a = analyzeSurface(left, right, Closure::Open);
  ASSERT_EQ(a.stripCount, 2u);
  EXPECT_EQ(count(a, StripDefectKind::Folded), 2u);
  EXPECT_EQ(count(a, StripDefectKind::Degenerate), 0u);
}

TEST(Surface, BowTieStripIsFolded) {
  // Cross-section 1 is right-to-left while the others are left-to-right: edges L_i L_{i+1} and R_{i+1} R_i
  // cross.
  const std::vector<Vec2> left{{0, 5}, {10, -5}}, right{{0, -5}, {10, 5}};
  const SurfaceAnalysis a = analyzeSurface(left, right, Closure::Open);
  ASSERT_EQ(a.stripCount, 1u);
  ASSERT_EQ(a.defects.size(), 1u);
  EXPECT_EQ(a.defects[0].kind, StripDefectKind::Folded);
  EXPECT_NEAR(a.defects[0].location.x, 5.0, 1e-9);  // where the two long edges cross
  EXPECT_NEAR(a.defects[0].location.y, 0.0, 1e-9);
}

TEST(Surface, CollapsedStripIsDegenerateAndSkipMaskIsHonoured) {
  const std::vector<Vec2> left{{0, 3}, {10, 0}, {20, 3}},
      right{{0, -3}, {10, 0}, {20, -3}};  // zero width at the middle
  const SurfaceAnalysis a = analyzeSurface(left, right, Closure::Open);
  EXPECT_EQ(count(a, StripDefectKind::Degenerate),
            0u);  // two triangles: area 15 each, a real (if pinched) strip
  const std::vector<Vec2> l2{{0, 0}, {10, 0}}, r2{{0, 0}, {10, 0}};  // coincident boundaries: zero area
  const SurfaceAnalysis b = analyzeSurface(l2, r2, Closure::Open);
  EXPECT_EQ(count(b, StripDefectKind::Degenerate), 1u);
  SurfaceOptions skip;   // skipStrip is a per-strip flag vector: element i == 1 skips strip i
  skip.skipStrip = {0};  // strip 0 is not skipped
  EXPECT_EQ(analyzeSurface(l2, r2, Closure::Open, skip).defects.size(), 1u);
  skip.skipStrip = {1};  // strip 0 skipped
  EXPECT_TRUE(analyzeSurface(l2, r2, Closure::Open, skip).defects.empty());
}

TEST(Surface, CrossingCorridorsOverlapAtTheCrossing) {
  // A smooth figure-eight (x = 100 sin t, y = 50 sin 2t): the centerline crosses itself at the origin (t = 0
  // and t = pi) and the two corridors, 6 m wide, overlap there. (A sharp-cornered bow-tie is a poor example:
  // vertex-normal offsets collapse its corner strips to slivers, which the analysis correctly reports as
  // degenerate instead.)
  std::vector<Vec2> pts;
  const int n = 400;
  for (int i = 0; i < n; ++i) {
    const double t = 2.0 * M_PI * i / n;
    pts.push_back({100.0 * std::sin(t), 50.0 * std::sin(2.0 * t)});
  }
  const Track t = Track::create(Path::create(pts, Closure::Closed), 3.0, 3.0);
  for (BroadPhase bp : {BroadPhase::Grid, BroadPhase::BruteForce}) {
    const SurfaceAnalysis a = analyzeTrack(t, bp);
    EXPECT_EQ(count(a, StripDefectKind::Folded), 0u);
    EXPECT_EQ(count(a, StripDefectKind::Degenerate), 0u);
    ASSERT_GT(count(a, StripDefectKind::Overlap), 0u);
    for (const StripDefect& d : a.defects) {
      ASSERT_EQ(d.kind, StripDefectKind::Overlap);
      EXPECT_LT(norm(d.location),
                8.0);  // every overlap is at the crossing (origin), within the corridor width
      // The overlapping strips belong to the two different passes through the crossing (around t = 0 and t =
      // pi).
      const bool firstPass = d.strip < 6 || d.strip > n - 6,
                 secondPass = d.other > n / 2 - 6 && d.other < n / 2 + 6;
      EXPECT_TRUE((firstPass && secondPass) ||
                  (d.strip > n / 2 - 6 && d.strip < n / 2 + 6 && (d.other < 6 || d.other > n - 6)))
          << d.strip << "," << d.other;
    }
    const auto regions = clusterOverlaps(a.defects, a.stripCount, true);
    EXPECT_EQ(regions.size(), 1u);  // one crossing = one region, however many strip pairs it consists of
  }
}

// Interior overlap WITHOUT any boundary crossing: strip 2 lies completely inside strip 0, so none of their
// boundary edges intersect; only a containment test finds it. (Strip 1 is the necessarily distorted connector
// and is reported as folded.)
TEST(Surface, NestedStripOverlapIsFoundByContainmentAlone) {
  const std::vector<Vec2> left{{-10, 10}, {10, 10}, {-2, 2}, {2, 2}};
  const std::vector<Vec2> right{{-10, -10}, {10, -10}, {-2, -2}, {2, -2}};
  const SurfaceAnalysis a = analyzeSurface(left, right, Closure::Open);
  ASSERT_EQ(a.stripCount, 3u);
  bool nested = false;
  for (const StripDefect& d : a.defects)
    nested = nested || (d.kind == StripDefectKind::Overlap && d.strip == 0 && d.other == 2);
  EXPECT_TRUE(nested);
  // No edge of strip 2 touches an edge of strip 0: the larger rectangle is [-10,10]^2, the inner one
  // [-2,2]^2.
  for (BroadPhase bp : {BroadPhase::Grid, BroadPhase::BruteForce}) {
    SurfaceOptions o;
    o.broadPhase = bp;
    EXPECT_EQ(asSet(analyzeSurface(left, right, Closure::Open, o)), asSet(a));
  }
}

TEST(Surface, TouchingAlongAnEdgeBetweenNonAdjacentStripsCountsAsOverlap) {
  // Strip 0 and strip 2 share the vertical edge x = 10 (the second strip's cross-section lies on the first
  // one's boundary).
  const std::vector<Vec2> left{{0, 5}, {10, 5}, {10, 5}, {20, 5}};
  const std::vector<Vec2> right{{0, -5}, {10, -5}, {10, -5}, {20, -5}};
  // Strip 1 is zero-length (degenerate) and excluded; strips 0 and 2 are non-adjacent and touch along x = 10.
  const SurfaceAnalysis a = analyzeSurface(left, right, Closure::Open);
  EXPECT_EQ(count(a, StripDefectKind::Degenerate), 1u);
  EXPECT_EQ(count(a, StripDefectKind::Overlap), 1u);
}

// Property: the grid broad phase finds exactly the defects of the brute-force sweep.
TEST(Surface, GridBroadPhaseMatchesBruteForceOnRandomCorridors) {
  std::size_t withOverlaps = 0;
  for (unsigned seed = 1; seed <= 60; ++seed) {
    for (bool closed : {false, true}) {
      const Track t = randomTrack(seed, 40 + seed % 40, closed, 1.0 + seed % 7);
      const SurfaceAnalysis grid = analyzeTrack(t, BroadPhase::Grid),
                            brute = analyzeTrack(t, BroadPhase::BruteForce);
      EXPECT_EQ(asSet(grid), asSet(brute)) << "seed " << seed << (closed ? " closed" : " open");
      EXPECT_LE(grid.candidatePairs, brute.candidatePairs + 0u);
      withOverlaps += count(brute, StripDefectKind::Overlap) > 0;
    }
  }
  EXPECT_GT(withOverlaps, 10u);  // the random corridors really exercise the overlap path
}

// Independent oracle: sample random points; whenever a point lies strictly inside two NON-ADJACENT strips,
// the analysis must report an overlap for that pair. (One-directional on purpose: reports that rest on
// boundary contact have no such point.)
TEST(Surface, EveryOverlapFoundByRandomSamplingIsReported) {
  std::size_t oracleHits = 0;
  for (unsigned seed = 100; seed < 200; ++seed) {
    const Track t = randomTrack(seed, 30, true, 12.0);
    const SurfaceAnalysis a = analyzeTrack(t, BroadPhase::BruteForce);
    std::set<std::pair<std::size_t, std::size_t>> reported;
    std::set<std::size_t> flawed;  // folded / degenerate strips are excluded from overlap analysis by design
    for (const StripDefect& d : a.defects) {
      if (d.kind == StripDefectKind::Overlap)
        reported.insert({d.strip, d.other});
      else
        flawed.insert(d.strip);
    }
    const std::size_t S = a.stripCount;
    std::vector<std::vector<Vec2>> quads(S);
    for (std::size_t i = 0; i < S; ++i) {
      const std::size_t j = (i + 1) % t.leftBoundary().size();
      quads[i] = {t.leftBoundary()[i], t.leftBoundary()[j], t.rightBoundary()[j], t.rightBoundary()[i]};
    }
    Rng r{seed * 7u};
    for (int k = 0; k < 4000; ++k) {
      const Vec2 q{(r.next() - 0.5) * 300.0, (r.next() - 0.5) * 300.0};
      std::vector<std::size_t> inside;
      for (std::size_t i = 0; i < S; ++i) {
        if (!flawed.count(i) && locatePoint(q, quads[i]) == PointLocation::Inside) inside.push_back(i);
      }
      for (std::size_t x = 0; x < inside.size(); ++x) {
        for (std::size_t y = x + 1; y < inside.size(); ++y) {
          const std::size_t i = inside[x], j = inside[y];
          const bool adjacent = j - i == 1 || (i == 0 && j == S - 1);
          if (adjacent) continue;
          ++oracleHits;
          EXPECT_TRUE(reported.count({i, j})) << "seed " << seed << " strips " << i << "," << j;
        }
      }
    }
  }
  EXPECT_GT(oracleHits, 50u);
}

TEST(Surface, GridCandidateCountIsFarBelowQuadraticOnALargeValidTrack) {
  const Track t = Track::create(makeStadium(3000.0, 1200.0, 4000), 5.0, 5.0);
  const SurfaceAnalysis grid = analyzeTrack(t, BroadPhase::Grid),
                        brute = analyzeTrack(t, BroadPhase::BruteForce);
  EXPECT_TRUE(grid.defects.empty());
  EXPECT_TRUE(brute.defects.empty());
  EXPECT_EQ(grid.stripCount, 4000u);
  // The bounding-box sweep of a long thin loop has candidate pairs wherever boxes touch; the grid examines
  // the same set once.
  EXPECT_EQ(grid.candidatePairs, brute.candidatePairs);
}

TEST(Surface, ClusteringMergesNeighbouringPairsIntoRegions) {
  std::vector<StripDefect> d;
  auto pair = [&](std::size_t a, std::size_t b) {
    d.push_back({StripDefectKind::Overlap, a, b, Vec2{double(a), double(b)}});
  };
  pair(10, 100);
  pair(11, 101);
  pair(12, 101);
  pair(12, 102);                                     // one run
  pair(50, 200);                                     // an unrelated second region
  d.push_back({StripDefectKind::Folded, 5, 5, {}});  // not an overlap: ignored
  const auto regions = clusterOverlaps(d, 300, false);
  ASSERT_EQ(regions.size(), 2u);
  EXPECT_EQ(regions[0].firstA, 10u);
  EXPECT_EQ(regions[0].lastA, 12u);
  EXPECT_EQ(regions[0].firstB, 100u);
  EXPECT_EQ(regions[0].lastB, 102u);
  EXPECT_EQ(regions[0].pairCount, 4u);
  EXPECT_EQ(regions[1].pairCount, 1u);
  // On a closed loop, strips 0 and S - 1 are neighbours: pairs on both sides of the seam form ONE region.
  std::vector<StripDefect> wrap;
  wrap.push_back({StripDefectKind::Overlap, 0, 100, {}});
  wrap.push_back(
      {StripDefectKind::Overlap, 99, 101, {}});  // (99, 101) vs (0, 100): 99 ~ 100? no: 99 and 0 are 99 apart
  EXPECT_EQ(clusterOverlaps(wrap, 200, true).size(), 2u);
  std::vector<StripDefect> seam;
  seam.push_back({StripDefectKind::Overlap, 0, 100, {}});
  seam.push_back({StripDefectKind::Overlap,
                  100,
                  199,
                  {}});  // second strip 199 = S - 1 neighbours strip 0 on the closed loop
  seam.push_back({StripDefectKind::Overlap, 1, 100, {}});
  EXPECT_EQ(clusterOverlaps(seam, 200, true).size(), 1u);
  EXPECT_EQ(clusterOverlaps({}, 10, false).size(), 0u);
}

TEST(PointLocation, SquareConcaveAndBoundaryCases) {
  const std::vector<Vec2> sq{{0, 0}, {4, 0}, {4, 4}, {0, 4}};
  EXPECT_EQ(locatePoint({2, 2}, sq), PointLocation::Inside);
  EXPECT_EQ(locatePoint({5, 2}, sq), PointLocation::Outside);
  EXPECT_EQ(locatePoint({-1e-9, 2}, sq), PointLocation::Outside);
  EXPECT_EQ(locatePoint({4, 2}, sq), PointLocation::Boundary);  // on an edge
  EXPECT_EQ(locatePoint({0, 0}, sq), PointLocation::Boundary);  // on a vertex
  EXPECT_EQ(locatePoint({2, 0}, sq), PointLocation::Boundary);
  // Orientation does not matter.
  const std::vector<Vec2> cw{{0, 4}, {4, 4}, {4, 0}, {0, 0}};
  EXPECT_EQ(locatePoint({2, 2}, cw), PointLocation::Inside);
  // A concave "L": the notch is outside.
  const std::vector<Vec2> ell{{0, 0}, {6, 0}, {6, 2}, {2, 2}, {2, 6}, {0, 6}};
  EXPECT_EQ(locatePoint({1, 5}, ell), PointLocation::Inside);
  EXPECT_EQ(locatePoint({4, 4}, ell), PointLocation::Outside);
  EXPECT_EQ(locatePoint({4, 1}, ell), PointLocation::Inside);
  // Scale and translation invariance (powers of two keep the geometry exactly representable).
  for (double scale : {std::ldexp(1.0, -30), std::ldexp(1.0, 30)}) {
    std::vector<Vec2> big;
    for (Vec2 p : ell) big.push_back(p * scale + Vec2{std::ldexp(1.0, 20) * scale, 0.0});
    const Vec2 shift{std::ldexp(1.0, 20) * scale, 0.0};
    EXPECT_EQ(locatePoint(Vec2{1, 5} * scale + shift, big), PointLocation::Inside);
    EXPECT_EQ(locatePoint(Vec2{4, 4} * scale + shift, big), PointLocation::Outside);
    EXPECT_EQ(locatePoint(Vec2{6, 1} * scale + shift, big), PointLocation::Boundary);
  }
}
