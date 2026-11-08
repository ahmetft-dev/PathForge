#include <algorithm>
#include <cmath>
#include <numeric>
#include <pathforge/geometry/path.hpp>

namespace pathforge::geometry {

std::string PathValidation::message() const {
  switch (issue) {
    case PathIssue::None:
      return "ok";
    case PathIssue::InvalidTolerance:
      return "minSegmentLength must be finite and >= 0";
    case PathIssue::TooFewPoints:
      return "too few points for this path type";
    case PathIssue::NonFinitePoint:
      return "non-finite coordinate at point " + std::to_string(index);
    case PathIssue::DegenerateSegment:
      return "zero-length (duplicate) segment starting at point " + std::to_string(index);
  }
  return "unknown";
}

PathValidation validate(const std::vector<Vec2>& points, Closure closure, double minSegmentLength) {
  if (!std::isfinite(minSegmentLength) || minSegmentLength < 0.0) {
    return {PathIssue::InvalidTolerance, 0};
  }
  const std::size_t minPoints = closure == Closure::Closed ? 3 : 2;
  if (points.size() < minPoints) return {PathIssue::TooFewPoints, points.size()};

  for (std::size_t i = 0; i < points.size(); ++i) {
    if (!isFinite(points[i])) return {PathIssue::NonFinitePoint, i};
  }
  const std::size_t segs = closure == Closure::Closed ? points.size() : points.size() - 1;
  for (std::size_t i = 0; i < segs; ++i) {
    const Vec2 a = points[i];
    const Vec2 b = points[(i + 1) % points.size()];
    if (distance(a, b) < minSegmentLength) return {PathIssue::DegenerateSegment, i};
  }
  return {};
}

Path Path::create(std::vector<Vec2> points, Closure closure, double minSegmentLength) {
  const PathValidation v = validate(points, closure, minSegmentLength);
  if (!v.ok()) throw GeometryError(v.message());
  return Path(std::move(points), closure);
}

Path::Path(std::vector<Vec2> points, Closure closure) : points_(std::move(points)), closure_(closure) {
  arc_.resize(points_.size());
  double s = 0.0;
  arc_[0] = 0.0;
  for (std::size_t i = 1; i < points_.size(); ++i) {
    s += distance(points_[i - 1], points_[i]);
    arc_[i] = s;
  }
  length_ = s;
  if (closure_ == Closure::Closed) length_ += distance(points_.back(), points_.front());
}

std::vector<double> Path::curvature() const {
  const std::size_t n = points_.size();
  std::vector<double> k(n, 0.0);
  if (n < 3) return k;

  auto menger = [](Vec2 a, Vec2 b, Vec2 c) {
    const double den = distance(a, b) * distance(b, c) * distance(a, c);
    // Validated paths have non-zero consecutive segments, but a and c may coincide
    // for pathological closed paths (fold-back); treat that as undefined -> 0.
    if (den <= 0.0) return 0.0;
    return 2.0 * cross(b - a, c - b) / den;
  };

  if (closure_ == Closure::Closed) {
    for (std::size_t i = 0; i < n; ++i) {
      k[i] = menger(points_[(i + n - 1) % n], points_[i], points_[(i + 1) % n]);
    }
  } else {
    for (std::size_t i = 1; i + 1 < n; ++i) k[i] = menger(points_[i - 1], points_[i], points_[i + 1]);
    k.front() = k[1];
    k.back() = k[n - 2];
  }
  return k;
}

PathStats computeStats(const Path& path) {
  PathStats st;
  st.pointCount = path.size();
  st.length = path.length();

  const auto& p = path.points();
  const std::size_t segs = path.segmentCount();
  st.minSegment = st.maxSegment = distance(p[0], p[1 % p.size()]);
  for (std::size_t i = 1; i < segs; ++i) {
    const double d = distance(p[i], p[(i + 1) % p.size()]);
    st.minSegment = std::min(st.minSegment, d);
    st.maxSegment = std::max(st.maxSegment, d);
  }

  const std::vector<double> k = path.curvature();
  const auto [mn, mx] = std::minmax_element(k.begin(), k.end());
  st.minCurvature = *mn;
  st.maxCurvature = *mx;
  double sumAbs = 0.0;
  for (double v : k) sumAbs += std::abs(v);
  st.meanAbsCurvature = sumAbs / static_cast<double>(k.size());
  return st;
}

}  // namespace pathforge::geometry
