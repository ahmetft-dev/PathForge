#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <pathforge/geometry/predicates.hpp>
#include <pathforge/geometry/surface.hpp>

namespace pathforge::geometry {
namespace {

using Quad = std::array<Vec2, 4>;  // L_i, L_{i+1}, R_{i+1}, R_i

struct Box {
  double minx, maxx, miny, maxy;
};

Box boxOf(const Quad& q) {
  Box b{q[0].x, q[0].x, q[0].y, q[0].y};
  for (const Vec2& p : q) {
    b.minx = std::min(b.minx, p.x);
    b.maxx = std::max(b.maxx, p.x);
    b.miny = std::min(b.miny, p.y);
    b.maxy = std::max(b.maxy, p.y);
  }
  return b;
}

bool overlaps(const Box& a, const Box& b) {
  return a.minx <= b.maxx && b.minx <= a.maxx && a.miny <= b.maxy && b.miny <= a.maxy;
}

double signedArea(const Quad& q) {
  // Relative to the first vertex: absolute coordinates far from the origin would cancel catastrophically.
  double a = 0.0;
  for (std::size_t i = 1; i + 1 < 4; ++i) a += cross(q[i] - q[0], q[i + 1] - q[0]);
  return 0.5 * a;
}

bool onSegment(Vec2 a, Vec2 b, Vec2 c) {  // c is known collinear with a-b
  return std::min(a.x, b.x) <= c.x && c.x <= std::max(a.x, b.x) && std::min(a.y, b.y) <= c.y &&
         c.y <= std::max(a.y, b.y);
}

bool nonZero(Vec2 a, Vec2 b) { return a.x != b.x || a.y != b.y; }

struct Strips {
  std::size_t count;
  bool closed;
  const std::vector<Vec2>& left;
  const std::vector<Vec2>& right;
  Quad quad(std::size_t i) const {
    const std::size_t n = left.size(), j = (i + 1) % n;
    return {left[i], left[j], right[j], right[i]};
  }
  bool adjacent(std::size_t i, std::size_t j) const {
    if (i == j) return false;
    const std::size_t d = i > j ? i - j : j - i;
    return d == 1 || (closed && d == count - 1);
  }
};

// Result of an overlap test between two strips.
struct Hit {
  bool found = false;
  Vec2 where{};
};

// Non-adjacent strips: any common point.
Hit overlapGeneral(const Quad& a, const Quad& b) {
  for (std::size_t i = 0; i < 4; ++i) {
    const Vec2 a0 = a[i], a1 = a[(i + 1) % 4];
    if (!nonZero(a0, a1)) continue;
    for (std::size_t j = 0; j < 4; ++j) {
      const Vec2 b0 = b[j], b1 = b[(j + 1) % 4];
      if (!nonZero(b0, b1)) continue;
      const SegmentContact c = intersectSegments(a0, a1, b0, b1);
      if (c.relation != SegmentRelation::Disjoint) return {true, c.point};
    }
  }
  const std::vector<Vec2> pa(a.begin(), a.end()), pb(b.begin(), b.end());
  for (const Vec2& v : b) {
    if (locatePoint(v, pa) != PointLocation::Outside) return {true, v};
  }
  for (const Vec2& v : a) {
    if (locatePoint(v, pb) != PointLocation::Outside) return {true, v};
  }
  return {};
}

// Adjacent strips a = Q_i and b = Q_{i+1}: they share the edge (a[1], a[2]) == (b[0], b[3]) reversed.
Hit overlapAdjacent(const Quad& a, const Quad& b) {
  // Edges of a except the shared one (index 1): 0, 2, 3. Edges of b except the shared one (index 3): 0, 1, 2.
  for (std::size_t i : {0u, 2u, 3u}) {
    const Vec2 a0 = a[i], a1 = a[(i + 1) % 4];
    if (!nonZero(a0, a1)) continue;
    for (std::size_t j : {0u, 1u, 2u}) {
      const Vec2 b0 = b[j], b1 = b[(j + 1) % 4];
      if (!nonZero(b0, b1)) continue;
      const SegmentContact c = intersectSegments(a0, a1, b0, b1);
      // Touching at the shared vertices (or the straight continuation of a boundary) is legitimate; a proper
      // crossing or a collinear fold-back is not.
      if (c.relation == SegmentRelation::ProperCrossing || c.relation == SegmentRelation::CollinearOverlap) {
        return {true, c.point};
      }
    }
  }
  const std::vector<Vec2> pa(a.begin(), a.end()), pb(b.begin(), b.end());
  for (std::size_t k :
       {1u, 2u}) {  // b's vertices that are not on the shared edge: b[1] = L_{i+2}, b[2] = R_{i+2}
    if (locatePoint(b[k], pa) == PointLocation::Inside) return {true, b[k]};
  }
  for (std::size_t k : {0u, 3u}) {  // a's vertices not on the shared edge: a[0] = L_i, a[3] = R_i
    if (locatePoint(a[k], pb) == PointLocation::Inside) return {true, a[k]};
  }
  return {};
}

}  // namespace

PointLocation locatePoint(Vec2 q, const std::vector<Vec2>& poly) {
  const std::size_t n = poly.size();
  int winding = 0;
  for (std::size_t i = 0; i < n; ++i) {
    const Vec2 a = poly[i], b = poly[(i + 1) % n];
    if (orientation(a, b, q) == 0 && onSegment(a, b, q)) return PointLocation::Boundary;
    if (a.y <= q.y) {
      if (b.y > q.y && orientation(a, b, q) > 0) ++winding;  // upward edge with q on its left
    } else if (b.y <= q.y && orientation(a, b, q) < 0) {
      --winding;  // downward edge with q on its right
    }
  }
  return winding != 0 ? PointLocation::Inside : PointLocation::Outside;
}

SurfaceAnalysis analyzeSurface(const std::vector<Vec2>& left, const std::vector<Vec2>& right, Closure closure,
                               const SurfaceOptions& options) {
  SurfaceAnalysis out;
  const std::size_t n = left.size();
  if (n < 2 || right.size() != n) return out;
  const bool closed = closure == Closure::Closed;
  const Strips strips{closed ? n : n - 1, closed, left, right};
  const std::size_t S = strips.count;
  out.stripCount = S;
  auto skipped = [&](std::size_t i) { return i < options.skipStrip.size() && options.skipStrip[i]; };

  // ---- per-strip checks
  // -------------------------------------------------------------------------------------------
  std::vector<char> usable(S, 0);  // takes part in overlap tests
  std::vector<Box> boxes(S);
  std::vector<Quad> quads(S);
  for (std::size_t i = 0; i < S; ++i) {
    quads[i] = strips.quad(i);
    boxes[i] = boxOf(quads[i]);
    if (skipped(i)) continue;
    const Quad& q = quads[i];
    const Vec2 centre = (q[0] + q[1] + q[2] + q[3]) * 0.25;
    const double edgeLen = 0.5 * (distance(q[0], q[1]) + distance(q[3], q[2]));
    const double width = 0.5 * (distance(q[0], q[3]) + distance(q[1], q[2]));
    const double area = signedArea(q);
    const double reference = edgeLen * width;
    if (reference == 0.0) {  // zero width at both cross-sections or zero length: nothing left of the strip
      out.defects.push_back({StripDefectKind::Degenerate, i, i, centre});
      continue;
    }
    // Fold checks come BEFORE the area test: a bow-tie has two lobes of opposite orientation whose areas can
    // cancel to ~0, which would otherwise be mistaken for a collapsed strip.
    bool folded = false;
    Vec2 where = centre;
    const std::array<std::pair<std::size_t, std::size_t>, 2> opposite{{{0, 2}, {1, 3}}};
    for (const auto& [e, f] : opposite) {  // opposite edges crossing: long edges, or the two cross-sections
      const Vec2 a0 = q[e], a1 = q[(e + 1) % 4], b0 = q[f], b1 = q[(f + 1) % 4];
      if (!nonZero(a0, a1) || !nonZero(b0, b1)) continue;
      const SegmentContact c = intersectSegments(a0, a1, b0, b1);
      if (c.relation != SegmentRelation::Disjoint) {
        folded = true;
        where = c.point;
      }
    }
    if (!folded && area > 0.0) folded = true;  // reversed orientation: left and right swapped
    if (folded) {
      out.defects.push_back({StripDefectKind::Folded, i, i, where});
      continue;
    }
    if (!(std::abs(area) > options.degenerateAreaRatio * reference)) {
      out.defects.push_back({StripDefectKind::Degenerate, i, i, centre});
      continue;
    }
    usable[i] = 1;
  }

  auto record = [&](std::size_t i, std::size_t j, const Hit& h) {
    if (h.found) out.defects.push_back({StripDefectKind::Overlap, std::min(i, j), std::max(i, j), h.where});
  };
  auto testGeneral = [&](std::size_t i, std::size_t j) { record(i, j, overlapGeneral(quads[i], quads[j])); };

  // ---- adjacent pairs (always, O(S))
  // ---------------------------------------------------------------------------------
  for (std::size_t i = 0; i < S; ++i) {
    std::size_t j = i + 1;
    if (j == S) {
      if (!closed) break;
      j = 0;
    }
    if (j == i || !usable[i] || !usable[j]) continue;
    if (S == 2) continue;  // two strips of a degenerate closed loop would be adjacent on both sides
    record(i, j,
           overlapAdjacent(quads[i], quads[j]));  // j follows i in travel order (also for the seam pair)
  }

  // ---- non-adjacent pairs
  // --------------------------------------------------------------------------------------------
  if (options.broadPhase == BroadPhase::BruteForce) {
    for (std::size_t i = 0; i < S; ++i) {
      if (!usable[i]) continue;
      for (std::size_t j = i + 1; j < S; ++j) {
        if (!usable[j] || strips.adjacent(i, j) || !overlaps(boxes[i], boxes[j])) continue;
        ++out.candidatePairs;
        testGeneral(i, j);
      }
    }
    return out;
  }

  // Grid broad phase: strips are registered in every cell their bounding box covers; a pair is examined only
  // in the cell that contains the lower-left corner of the intersection of their boxes, so each candidate
  // pair is processed exactly once.
  Box bounds{std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
             std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
  double meanExtent = 0.0;
  std::size_t usableCount = 0;
  for (std::size_t i = 0; i < S; ++i) {
    if (!usable[i]) continue;
    bounds.minx = std::min(bounds.minx, boxes[i].minx);
    bounds.maxx = std::max(bounds.maxx, boxes[i].maxx);
    bounds.miny = std::min(bounds.miny, boxes[i].miny);
    bounds.maxy = std::max(bounds.maxy, boxes[i].maxy);
    meanExtent += std::max(boxes[i].maxx - boxes[i].minx, boxes[i].maxy - boxes[i].miny);
    ++usableCount;
  }
  if (usableCount < 2) return out;
  meanExtent /= static_cast<double>(usableCount);
  const double width = bounds.maxx - bounds.minx, height = bounds.maxy - bounds.miny;
  double cell = std::max(2.0 * meanExtent, std::max(width, height) / 1024.0);
  if (!(cell > 0.0)) cell = 1.0;
  const std::size_t gx = static_cast<std::size_t>(width / cell) + 1,
                    gy = static_cast<std::size_t>(height / cell) + 1;

  auto cellX = [&](double x) {
    return static_cast<std::size_t>(std::min((x - bounds.minx) / cell, static_cast<double>(gx - 1)));
  };
  auto cellY = [&](double y) {
    return static_cast<std::size_t>(std::min((y - bounds.miny) / cell, static_cast<double>(gy - 1)));
  };

  constexpr std::size_t kLargeSpan =
      64;  // strips covering more cells than this are tested directly against all strips
  std::vector<std::size_t> large;
  std::vector<std::size_t> counts(gx * gy + 1, 0);
  struct Span {
    std::size_t x0, x1, y0, y1;
  };
  std::vector<Span> spans(S);
  for (std::size_t i = 0; i < S; ++i) {
    if (!usable[i]) continue;
    spans[i] = {cellX(boxes[i].minx), cellX(boxes[i].maxx), cellY(boxes[i].miny), cellY(boxes[i].maxy)};
    const std::size_t cells = (spans[i].x1 - spans[i].x0 + 1) * (spans[i].y1 - spans[i].y0 + 1);
    if (cells > kLargeSpan) {
      large.push_back(i);
      continue;
    }
    for (std::size_t y = spans[i].y0; y <= spans[i].y1; ++y) {
      for (std::size_t x = spans[i].x0; x <= spans[i].x1; ++x) ++counts[y * gx + x + 1];
    }
  }
  for (std::size_t c = 1; c < counts.size(); ++c) counts[c] += counts[c - 1];
  std::vector<std::uint32_t> items(counts.back());
  {
    std::vector<std::size_t> fill(counts.begin(), counts.end() - 1);
    for (std::size_t i = 0; i < S; ++i) {
      if (!usable[i] || std::find(large.begin(), large.end(), i) != large.end()) continue;
      for (std::size_t y = spans[i].y0; y <= spans[i].y1; ++y) {
        for (std::size_t x = spans[i].x0; x <= spans[i].x1; ++x)
          items[fill[y * gx + x]++] = static_cast<std::uint32_t>(i);
      }
    }
  }
  for (std::size_t y = 0; y < gy; ++y) {
    for (std::size_t x = 0; x < gx; ++x) {
      const std::size_t begin = counts[y * gx + x], end = counts[y * gx + x + 1];
      for (std::size_t a = begin; a < end; ++a) {
        for (std::size_t b = a + 1; b < end; ++b) {
          const std::size_t i = items[a], j = items[b];
          if (strips.adjacent(i, j) || !overlaps(boxes[i], boxes[j])) continue;
          // Process only in the cell of the lower-left corner of the boxes' intersection.
          if (cellX(std::max(boxes[i].minx, boxes[j].minx)) != x ||
              cellY(std::max(boxes[i].miny, boxes[j].miny)) != y)
            continue;
          ++out.candidatePairs;
          testGeneral(std::min(i, j), std::max(i, j));
        }
      }
    }
  }
  std::vector<char> isLarge(S, 0);
  for (std::size_t i : large) isLarge[i] = 1;
  for (std::size_t i : large) {
    for (std::size_t j = 0; j < S; ++j) {
      if (j == i || !usable[j] || strips.adjacent(i, j) || (isLarge[j] && j < i) ||
          !overlaps(boxes[i], boxes[j]))
        continue;
      ++out.candidatePairs;
      testGeneral(std::min(i, j), std::max(i, j));
    }
  }
  return out;
}

std::vector<OverlapRegion> clusterOverlaps(const std::vector<StripDefect>& defects, std::size_t stripCount,
                                           bool closed) {
  std::vector<std::pair<std::size_t, std::size_t>> pairs;
  std::vector<Vec2> where;
  for (const StripDefect& d : defects) {
    if (d.kind != StripDefectKind::Overlap) continue;
    pairs.emplace_back(d.strip, d.other);
    where.push_back(d.location);
  }
  const std::size_t P = pairs.size();
  std::vector<std::size_t> parent(P);
  for (std::size_t i = 0; i < P; ++i) parent[i] = i;
  auto find = [&](std::size_t x) {
    while (parent[x] != x) x = parent[x] = parent[parent[x]];
    return x;
  };
  auto near = [&](std::size_t a, std::size_t b) {
    const std::size_t d = a > b ? a - b : b - a;
    return d <= 1 || (closed && stripCount >= 3 && d + 1 >= stripCount);
  };
  // Connect pairs {a, b} and {a', b'} when a ~ a' and b ~ b' (or a ~ b' and b ~ a') with "~" meaning at most
  // one strip apart (cyclically on closed tracks). Each pair is registered under both of its strips so that
  // only the three neighbouring buckets of each strip are compared instead of all pairs. The unordered match
  // is what links pairs across the seam.
  std::map<std::size_t, std::vector<std::size_t>> byStrip;
  for (std::size_t p = 0; p < P; ++p) {
    byStrip[pairs[p].first].push_back(p);
    byStrip[pairs[p].second].push_back(p);
  }
  const std::size_t S = std::max<std::size_t>(stripCount, 1);
  for (std::size_t p = 0; p < P; ++p) {
    for (std::size_t strip : {pairs[p].first, pairs[p].second}) {
      for (std::size_t delta : {std::size_t{0}, std::size_t{1}, S - 1}) {
        const auto it = byStrip.find((strip + delta) % S);
        if (it == byStrip.end()) continue;
        for (std::size_t q : it->second) {
          if (q == p) continue;
          const bool straight =
              near(pairs[p].first, pairs[q].first) && near(pairs[p].second, pairs[q].second);
          const bool crossed = near(pairs[p].first, pairs[q].second) && near(pairs[p].second, pairs[q].first);
          if (straight || crossed) parent[find(p)] = find(q);
        }
      }
    }
  }
  std::map<std::size_t, OverlapRegion> regions;
  for (std::size_t p = 0; p < P; ++p) {
    const std::size_t root = find(p);
    auto it = regions.find(root);
    if (it == regions.end()) {
      OverlapRegion r{pairs[p].first, pairs[p].first, pairs[p].second, pairs[p].second, 0, where[p], {}};
      it = regions.emplace(root, std::move(r)).first;
    }
    OverlapRegion& r = it->second;
    r.firstA = std::min(r.firstA, pairs[p].first);
    r.lastA = std::max(r.lastA, pairs[p].first);
    r.firstB = std::min(r.firstB, pairs[p].second);
    r.lastB = std::max(r.lastB, pairs[p].second);
    ++r.pairCount;
    r.strips.push_back(pairs[p].first);
    r.strips.push_back(pairs[p].second);
  }
  std::vector<OverlapRegion> out;
  for (auto& [root, r] : regions) {
    std::sort(r.strips.begin(), r.strips.end());
    r.strips.erase(std::unique(r.strips.begin(), r.strips.end()), r.strips.end());
    out.push_back(std::move(r));
  }
  std::sort(out.begin(), out.end(),
            [](const OverlapRegion& a, const OverlapRegion& b) { return a.firstA < b.firstA; });
  return out;
}

}  // namespace pathforge::geometry
