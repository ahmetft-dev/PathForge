#include <algorithm>
#include <cmath>
#include <pathforge/geometry/predicates.hpp>

namespace pathforge::geometry {
namespace {

// (3 + 16 eps) * eps with eps = 2^-53 (Shewchuk, ccwerrboundA).
constexpr double kCcwErrBoundA = 3.3306690738754716e-16;

bool onSegmentBox(Vec2 a, Vec2 b, Vec2 c) {
  return std::min(a.x, b.x) <= c.x && c.x <= std::max(a.x, b.x) && std::min(a.y, b.y) <= c.y &&
         c.y <= std::max(a.y, b.y);
}

struct Box {
  double minx, maxx, miny, maxy;
};

Box boxOf(Vec2 a, Vec2 b) {
  return {std::min(a.x, b.x), std::max(a.x, b.x), std::min(a.y, b.y), std::max(a.y, b.y)};
}

bool overlaps(const Box& a, const Box& b) {
  return a.minx <= b.maxx && b.minx <= a.maxx && a.miny <= b.maxy && b.miny <= a.maxy;
}

std::vector<Box> boxes(const std::vector<Vec2>& p, Closure closure) {
  const std::size_t n = p.size();
  const std::size_t segs = closure == Closure::Closed ? n : n - 1;
  std::vector<Box> out;
  out.reserve(segs);
  for (std::size_t i = 0; i < segs; ++i) out.push_back(boxOf(p[i], p[(i + 1) % n]));
  return out;
}

}  // namespace

int orientation(Vec2 a, Vec2 b, Vec2 c) {
  const double detleft = (a.x - c.x) * (b.y - c.y);
  const double detright = (a.y - c.y) * (b.x - c.x);
  const double det = detleft - detright;
  double detsum;
  if (detleft > 0.0) {
    if (detright <= 0.0)
      return det > 0.0 ? 1 : (det < 0.0 ? -1 : 0);  // opposite-sign terms: det exact in sign
    detsum = detleft + detright;
  } else if (detleft < 0.0) {
    if (detright >= 0.0) return det > 0.0 ? 1 : (det < 0.0 ? -1 : 0);
    detsum = -detleft - detright;
  } else {
    return det > 0.0 ? 1 : (det < 0.0 ? -1 : 0);  // detleft == 0: det == -detright exactly
  }
  const double errbound = kCcwErrBoundA * detsum;
  if (det > errbound) return 1;
  if (-det > errbound) return -1;
  return 0;
}

SegmentContact intersectSegments(Vec2 p1, Vec2 p2, Vec2 q1, Vec2 q2) {
  const int o1 = orientation(p1, p2, q1);
  const int o2 = orientation(p1, p2, q2);
  const int o3 = orientation(q1, q2, p1);
  const int o4 = orientation(q1, q2, p2);

  if (o1 * o2 < 0 && o3 * o4 < 0) {
    const Vec2 r = p2 - p1;
    const Vec2 s = q2 - q1;
    const double den = cross(r, s);
    const double t = den != 0.0 ? cross(q1 - p1, s) / den : 0.5;
    return {SegmentRelation::ProperCrossing, p1 + std::clamp(t, 0.0, 1.0) * r};
  }

  if (o1 == 0 && o2 == 0 && o3 == 0 && o4 == 0) {
    // Collinear (within reliability): compare projections on the dominant axis.
    const bool useX =
        std::abs(p2.x - p1.x) + std::abs(q2.x - q1.x) >= std::abs(p2.y - p1.y) + std::abs(q2.y - q1.y);
    auto key = [&](Vec2 v) { return useX ? v.x : v.y; };
    const double plo = std::min(key(p1), key(p2)), phi = std::max(key(p1), key(p2));
    const double qlo = std::min(key(q1), key(q2)), qhi = std::max(key(q1), key(q2));
    const double lo = std::max(plo, qlo), hi = std::min(phi, qhi);
    if (lo > hi) return {};
    // Overlap midpoint: pick the point on p's line at the middle projection value.
    const Vec2 d = p2 - p1;
    const double mid = 0.5 * (lo + hi);
    const double denom = useX ? d.x : d.y;
    const double t = denom != 0.0 ? (mid - key(p1)) / denom : 0.0;
    const Vec2 pt = p1 + std::clamp(t, 0.0, 1.0) * d;
    return {lo < hi ? SegmentRelation::CollinearOverlap : SegmentRelation::Touching, pt};
  }

  if (o1 == 0 && onSegmentBox(p1, p2, q1)) return {SegmentRelation::Touching, q1};
  if (o2 == 0 && onSegmentBox(p1, p2, q2)) return {SegmentRelation::Touching, q2};
  if (o3 == 0 && onSegmentBox(q1, q2, p1)) return {SegmentRelation::Touching, p1};
  if (o4 == 0 && onSegmentBox(q1, q2, p2)) return {SegmentRelation::Touching, p2};
  return {};
}

double pointSegmentDistance(Vec2 p, Vec2 a, Vec2 b) {
  const Vec2 d = b - a;
  const double len2 = dot(d, d);
  const double t = len2 > 0.0 ? std::clamp(dot(p - a, d) / len2, 0.0, 1.0) : 0.0;
  return distance(p, a + t * d);
}

std::vector<SegmentPairContact> findSelfIntersections(const std::vector<Vec2>& p, Closure closure,
                                                      std::size_t maxResults) {
  std::vector<SegmentPairContact> out;
  const std::size_t n = p.size();
  if (n < 2) return out;
  const bool closed = closure == Closure::Closed;
  const std::size_t segs = closed ? n : n - 1;
  const std::vector<Box> bx = boxes(p, closure);
  auto full = [&] { return maxResults != 0 && out.size() >= maxResults; };

  // Adjacent pairs (i, i+1) share vertex (i+1) % n: only an exact fold-back is a contact.
  for (std::size_t i = 0; i < segs; ++i) {
    std::size_t j = i + 1;
    if (j == segs) {
      if (!closed) break;
      j = 0;  // seam: last segment is adjacent to the first
    }
    if (j == i) continue;
    const Vec2 a = p[i], b = p[(i + 1) % n], c = p[(i + 2) % n];
    if (orientation(a, b, c) == 0 && dot(b - a, c - b) < 0.0) {
      out.push_back({i, j, SegmentRelation::CollinearOverlap, b});
      if (full()) return out;
    }
  }

  for (std::size_t i = 0; i < segs; ++i) {
    for (std::size_t j = i + 2; j < segs; ++j) {
      if (closed && i == 0 && j == segs - 1) continue;  // seam neighbours
      if (!overlaps(bx[i], bx[j])) continue;
      const SegmentContact c = intersectSegments(p[i], p[(i + 1) % n], p[j], p[(j + 1) % n]);
      if (c.relation != SegmentRelation::Disjoint) {
        out.push_back({i, j, c.relation, c.point});
        if (full()) return out;
      }
    }
  }
  return out;
}

std::vector<SegmentPairContact> findContacts(const std::vector<Vec2>& a, Closure closureA,
                                             const std::vector<Vec2>& b, Closure closureB,
                                             std::size_t maxResults) {
  std::vector<SegmentPairContact> out;
  if (a.size() < 2 || b.size() < 2) return out;
  const std::size_t segA = closureA == Closure::Closed ? a.size() : a.size() - 1;
  const std::size_t segB = closureB == Closure::Closed ? b.size() : b.size() - 1;
  const std::vector<Box> ba = boxes(a, closureA), bb = boxes(b, closureB);
  for (std::size_t i = 0; i < segA; ++i) {
    for (std::size_t j = 0; j < segB; ++j) {
      if (!overlaps(ba[i], bb[j])) continue;
      const SegmentContact c = intersectSegments(a[i], a[(i + 1) % a.size()], b[j], b[(j + 1) % b.size()]);
      if (c.relation != SegmentRelation::Disjoint) {
        out.push_back({i, j, c.relation, c.point});
        if (maxResults != 0 && out.size() >= maxResults) return out;
      }
    }
  }
  return out;
}

}  // namespace pathforge::geometry
