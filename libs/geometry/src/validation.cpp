#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <pathforge/geometry/validation.hpp>
#include <sstream>

namespace pathforge::geometry {
namespace {

class Collector {
 public:
  Collector(ValidationReport& r, const ValidationOptions& o) : report_(r), options_(o) {}

  void add(TrackIssue type, Severity severity, std::vector<SegmentRef> segments, std::optional<Vec2> loc,
           std::string message) {
    if (severity == Severity::Error) ++report_.errorCount;
    if (severity == Severity::Warning) ++report_.warningCount;
    std::size_t& listed = perType_[type];
    if (listed >= options_.maxIssuesPerType) {
      ++report_.suppressed;
      return;
    }
    ++listed;
    report_.issues.push_back({type, severity, std::move(segments), loc, std::move(message)});
  }

 private:
  ValidationReport& report_;
  const ValidationOptions& options_;
  std::map<TrackIssue, std::size_t> perType_;
};

std::string at(Vec2 p) {
  std::ostringstream o;
  o.precision(6);
  o << "(" << p.x << ", " << p.y << ")";
  return o.str();
}

const char* relationName(SegmentRelation r) {
  switch (r) {
    case SegmentRelation::ProperCrossing:
      return "cross";
    case SegmentRelation::Touching:
      return "touch";
    case SegmentRelation::CollinearOverlap:
      return "overlap collinearly";
    case SegmentRelation::Disjoint:
      break;
  }
  return "meet";
}

void reportSelf(Collector& c, const std::vector<Vec2>& pts, Closure closure, PolylineId id, TrackIssue type,
                const char* what) {
  for (const SegmentPairContact& k : findSelfIntersections(pts, closure)) {
    std::ostringstream m;
    m << what << " segments " << k.first << " and " << k.second << " " << relationName(k.relation) << " near "
      << at(k.point);
    c.add(type, Severity::Error, {{id, k.first}, {id, k.second}}, k.point, m.str());
  }
}

void reportCross(Collector& c, const std::vector<Vec2>& a, PolylineId ida, const std::vector<Vec2>& b,
                 PolylineId idb, Closure closure, TrackIssue type, const char* what) {
  for (const SegmentPairContact& k : findContacts(a, closure, b, closure)) {
    std::ostringstream m;
    m << what << ": segment " << k.first << " and segment " << k.second << " " << relationName(k.relation)
      << " near " << at(k.point);
    c.add(type, Severity::Error, {{ida, k.first}, {idb, k.second}}, k.point, m.str());
  }
}

// Distance from p to the nearest segment of polyline `poly`, with early rejection by x/y distance
// against the current best (pruning is exact: a segment whose bounding box is farther than `best`
// in x or y cannot be nearer).
double nearestDistance(Vec2 p, const std::vector<Vec2>& poly, bool closed, double best) {
  const std::size_t n = poly.size();
  const std::size_t segs = closed ? n : n - 1;
  for (std::size_t i = 0; i < segs; ++i) {
    const Vec2 a = poly[i], b = poly[(i + 1) % n];
    if (p.x < std::min(a.x, b.x) - best || p.x > std::max(a.x, b.x) + best) continue;
    if (p.y < std::min(a.y, b.y) - best || p.y > std::max(a.y, b.y) + best) continue;
    best = std::min(best, pointSegmentDistance(p, a, b));
  }
  return best;
}

void geometricChecks(const Track& t, const ValidationOptions& opt, Collector& c, ValidationReport& r) {
  const Path& path = t.centerline();
  const Closure closure = path.closure();
  const bool closed = path.isClosed();
  const auto& cl = path.points();
  const auto& L = t.leftBoundary();
  const auto& R = t.rightBoundary();
  const std::size_t n = cl.size();

  // Widths (zero is representable but not a usable track).
  for (std::size_t i = 0; i < n; ++i) {
    if (t.leftWidths()[i] == 0.0 || t.rightWidths()[i] == 0.0) {
      std::ostringstream m;
      m << "zero " << (t.leftWidths()[i] == 0.0 ? "left" : "right") << " width at point " << i;
      c.add(TrackIssue::InvalidWidth, Severity::Error, {}, cl[i], m.str());
    }
  }
  bool leftZero = false, rightZero = false;
  for (std::size_t i = 0; i < n; ++i) {
    leftZero = leftZero || t.leftWidths()[i] == 0.0;
    rightZero = rightZero || t.rightWidths()[i] == 0.0;
  }

  reportSelf(c, cl, closure, PolylineId::Centerline, TrackIssue::CenterlineSelfIntersection, "centerline");
  r.checked.centerline = true;
  reportSelf(c, L, closure, PolylineId::LeftBoundary, TrackIssue::LeftBoundarySelfIntersection,
             "left boundary");
  reportSelf(c, R, closure, PolylineId::RightBoundary, TrackIssue::RightBoundarySelfIntersection,
             "right boundary");
  r.checked.boundaries = true;

  const std::size_t crossingsBefore = r.errorCount;
  reportCross(c, L, PolylineId::LeftBoundary, R, PolylineId::RightBoundary, closure,
              TrackIssue::BoundaryCrossing, "left/right boundaries");
  const bool crossed = r.errorCount > crossingsBefore;
  r.checked.crossings = true;

  // Boundary vs centerline. A side with zero width coincides with the centerline by definition (already
  // an error); skipping it avoids a flood of redundant overlap reports, and the group stays unchecked.
  if (!leftZero) {
    reportCross(c, L, PolylineId::LeftBoundary, cl, PolylineId::Centerline, closure,
                TrackIssue::BoundaryCollapse, "left boundary reaches the centerline");
  }
  if (!rightZero) {
    reportCross(c, R, PolylineId::RightBoundary, cl, PolylineId::Centerline, closure,
                TrackIssue::BoundaryCollapse, "right boundary reaches the centerline");
  }
  r.checked.collapse = !leftZero && !rightZero;

  // Local folds.
  const std::size_t segs = path.segmentCount();
  for (std::size_t i = 0; i < segs; ++i) {
    const std::size_t j = (i + 1) % n;
    const Vec2 cs = cl[j] - cl[i];
    if (dot(L[j] - L[i], cs) <= 0.0) {
      std::ostringstream m;
      m << "left boundary segment " << i << " is reversed relative to the centerline near " << at(L[i]);
      c.add(TrackIssue::LocalBoundaryFold, Severity::Error, {{PolylineId::LeftBoundary, i}}, L[i], m.str());
    }
    if (dot(R[j] - R[i], cs) <= 0.0) {
      std::ostringstream m;
      m << "right boundary segment " << i << " is reversed relative to the centerline near " << at(R[i]);
      c.add(TrackIssue::LocalBoundaryFold, Severity::Error, {{PolylineId::RightBoundary, i}}, R[i], m.str());
    }
  }
  for (std::size_t i = 0; i < n; ++i) {
    const double k = t.frames().curvature[i];
    if (t.leftWidths()[i] * k >= 1.0) {
      std::ostringstream m;
      m << "left offset " << t.leftWidths()[i] << " m >= local radius of curvature " << 1.0 / k
        << " m at point " << i;
      c.add(TrackIssue::LocalBoundaryFold, Severity::Warning, {}, cl[i], m.str());
    }
    if (t.rightWidths()[i] * -k >= 1.0) {
      std::ostringstream m;
      m << "right offset " << t.rightWidths()[i] << " m >= local radius of curvature " << -1.0 / k
        << " m at point " << i;
      c.add(TrackIssue::LocalBoundaryFold, Severity::Warning, {}, cl[i], m.str());
    }
  }
  r.checked.localFolds = true;

  // Clearance.
  ClearanceMetrics& cm = r.clearance;
  cm.vertexClearance.assign(n, 0.0);
  cm.minConfiguredTotalWidth = t.minTotalWidth();
  cm.minCorrespondingDistance = std::numeric_limits<double>::infinity();
  cm.minBoundaryDistance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < n; ++i) {
    const double corr = distance(L[i], R[i]);
    cm.minCorrespondingDistance = std::min(cm.minCorrespondingDistance, corr);
    const double d1 = nearestDistance(L[i], R, closed, corr);
    const double d2 = nearestDistance(R[i], L, closed, corr);
    const double v = std::min(d1, d2);
    cm.vertexClearance[i] = v;
    if (v < cm.minBoundaryDistance) {
      cm.minBoundaryDistance = v;
      cm.minBoundaryDistanceLocation = cl[i];
    }
  }
  if (crossed) cm.minBoundaryDistance = 0.0;  // polylines meet
  cm.computed = true;
  r.checked.clearance = true;

  // Collapsed / narrow runs of vertices (one issue per maximal run).
  auto runs = [&](double threshold, TrackIssue type, Severity sev, const char* what) {
    std::size_t i = 0;
    while (i < n) {
      if (cm.vertexClearance[i] >= threshold) {
        ++i;
        continue;
      }
      std::size_t j = i, worst = i;
      while (j < n && cm.vertexClearance[j] < threshold) {
        if (cm.vertexClearance[j] < cm.vertexClearance[worst]) worst = j;
        ++j;
      }
      std::ostringstream m;
      m << what << " at points " << i << ".." << j - 1 << ": minimum actual clearance "
        << cm.vertexClearance[worst] << " m (configured width there "
        << t.leftWidths()[worst] + t.rightWidths()[worst] << " m)";
      c.add(type, sev, {}, cl[worst], m.str());
      i = j;
    }
  };
  runs(opt.collapseClearance, TrackIssue::BoundaryCollapse, Severity::Error, "collapsed track section");
  // Narrow (but not collapsed) sections: run detection over [collapse, narrow).
  {
    std::size_t i = 0;
    while (i < n) {
      const double v = cm.vertexClearance[i];
      if (!(v >= opt.collapseClearance && v < opt.narrowWidthWarning)) {
        ++i;
        continue;
      }
      std::size_t j = i, worst = i;
      while (j < n && cm.vertexClearance[j] >= opt.collapseClearance &&
             cm.vertexClearance[j] < opt.narrowWidthWarning) {
        if (cm.vertexClearance[j] < cm.vertexClearance[worst]) worst = j;
        ++j;
      }
      std::ostringstream m;
      m << "narrow section at points " << i << ".." << j - 1 << ": minimum actual clearance "
        << cm.vertexClearance[worst] << " m";
      c.add(TrackIssue::NarrowSection, Severity::Warning, {}, cl[worst], m.str());
      i = j;
    }
  }
}

}  // namespace

ValidationReport validateTrackData(const TrackData& d, const ValidationOptions& options) {
  ValidationReport r;
  Collector c(r, options);
  const std::size_t n = d.centerline.size();
  const std::size_t minPoints = d.closure == Closure::Closed ? 3 : 2;
  bool blocking = false;

  if (n < minPoints) {
    c.add(TrackIssue::TooFewPoints, Severity::Error, {}, std::nullopt,
          "centerline has " + std::to_string(n) + " points; at least " + std::to_string(minPoints) +
              " required");
    blocking = true;
  }
  if (d.leftWidths.size() != n || d.rightWidths.size() != n) {
    c.add(TrackIssue::WidthCountMismatch, Severity::Error, {}, std::nullopt,
          "width arrays have " + std::to_string(d.leftWidths.size()) + " (left) and " +
              std::to_string(d.rightWidths.size()) + " (right) entries for " + std::to_string(n) + " points");
    blocking = true;
  }
  for (std::size_t i = 0; i < n; ++i) {
    if (!isFinite(d.centerline[i])) {
      c.add(TrackIssue::NonFiniteCoordinate, Severity::Error, {}, std::nullopt,
            "non-finite coordinate at point " + std::to_string(i));
      blocking = true;
    }
  }
  auto checkWidths = [&](const std::vector<double>& w, const char* side) {
    for (std::size_t i = 0; i < w.size(); ++i) {
      if (!std::isfinite(w[i]) || w[i] < 0.0) {
        c.add(TrackIssue::InvalidWidth, Severity::Error, {}, std::nullopt,
              std::string(side) + " width at point " + std::to_string(i) + " is negative or non-finite");
        blocking = true;
      }
    }
  };
  checkWidths(d.leftWidths, "left");
  checkWidths(d.rightWidths, "right");
  if (!blocking) {
    const std::size_t segs = d.closure == Closure::Closed ? n : n - 1;
    for (std::size_t i = 0; i < segs; ++i) {
      if (distance(d.centerline[i], d.centerline[(i + 1) % n]) < kDefaultMinSegmentLength) {
        c.add(TrackIssue::DegenerateSegment, Severity::Error, {{PolylineId::Centerline, i}}, d.centerline[i],
              "zero-length (duplicate point) segment " + std::to_string(i));
        blocking = true;
      }
    }
  }
  if (blocking) return r;  // geometric checks need a buildable track; coverage stays false

  r.checked.structure = true;
  const Track track = buildTrack(d);
  geometricChecks(track, options, c, r);
  return r;
}

ValidationReport validateTrack(const Track& track, const ValidationOptions& options) {
  ValidationReport r;
  Collector c(r, options);
  r.checked.structure = true;
  geometricChecks(track, options, c, r);
  return r;
}

const char* toString(TrackIssue issue) {
  switch (issue) {
    case TrackIssue::TooFewPoints:
      return "TooFewPoints";
    case TrackIssue::WidthCountMismatch:
      return "WidthCountMismatch";
    case TrackIssue::InvalidWidth:
      return "InvalidWidth";
    case TrackIssue::NonFiniteCoordinate:
      return "NonFiniteCoordinate";
    case TrackIssue::DegenerateSegment:
      return "DegenerateSegment";
    case TrackIssue::CenterlineSelfIntersection:
      return "CenterlineSelfIntersection";
    case TrackIssue::LeftBoundarySelfIntersection:
      return "LeftBoundarySelfIntersection";
    case TrackIssue::RightBoundarySelfIntersection:
      return "RightBoundarySelfIntersection";
    case TrackIssue::BoundaryCrossing:
      return "BoundaryCrossing";
    case TrackIssue::BoundaryCollapse:
      return "BoundaryCollapse";
    case TrackIssue::LocalBoundaryFold:
      return "LocalBoundaryFold";
    case TrackIssue::NarrowSection:
      return "NarrowSection";
  }
  return "Unknown";
}

const char* toString(Severity s) {
  switch (s) {
    case Severity::Info:
      return "info";
    case Severity::Warning:
      return "warning";
    case Severity::Error:
      return "error";
  }
  return "unknown";
}

const char* toString(ValidationStatus s) {
  switch (s) {
    case ValidationStatus::Valid:
      return "valid";
    case ValidationStatus::Invalid:
      return "invalid";
    case ValidationStatus::Incomplete:
      return "incomplete";
  }
  return "unknown";
}

}  // namespace pathforge::geometry
