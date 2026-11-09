#include <algorithm>
#include <cmath>
#include <limits>
#include <pathforge/editor/derived_geometry.hpp>
#include <pathforge/geometry/predicates.hpp>
#include <pathforge/geometry/resample.hpp>
#include <pathforge/geometry/spline.hpp>
#include <pathforge/geometry/width_profile.hpp>

namespace pathforge::editor {
namespace {

using geometry::GeometryError;
using geometry::Path;

geometry::TrackData rawData(const TrackDocument& d) {
  geometry::TrackData out;
  out.name = d.name();
  out.closure = d.closure();
  for (const ControlPoint& p : d.points()) {
    out.centerline.push_back(p.position);
    out.leftWidths.push_back(p.leftWidth);
    out.rightWidths.push_back(p.rightWidth);
  }
  return out;
}

DerivedGeometry fail(const TrackDocument& d, DerivedStatus status, std::string message) {
  DerivedGeometry g;
  g.status = status;
  g.message = std::move(message);
  g.data = rawData(d);
  return g;
}

}  // namespace

DerivedGeometry generateGeometry(const TrackDocument& d) {
  const auto& pts = d.points();
  const std::size_t n = pts.size();
  const bool closed = d.closure() == Closure::Closed;
  const std::size_t minPoints = closed ? 3 : 2;
  if (n < minPoints) {
    return fail(d, DerivedStatus::TooFewPoints,
                "a" + std::string(closed ? " closed" : "n open") + " track needs at least " +
                    std::to_string(minPoints) + " control points (has " + std::to_string(n) + ")");
  }
  const std::size_t segs = closed ? n : n - 1;
  double maxChord = 0.0;
  for (std::size_t i = 0; i < segs; ++i) {
    const double c = geometry::distance(pts[i].position, pts[(i + 1) % n].position);
    if (c < geometry::kDefaultMinSegmentLength) {
      return fail(
          d, DerivedStatus::Failed,
          "control points " + std::to_string(i) + " and " + std::to_string((i + 1) % n) + " coincide");
    }
    maxChord = std::max(maxChord, c);
  }

  const GeometrySettings& s = d.settings();
  std::vector<Vec2> control;
  control.reserve(n);
  for (const ControlPoint& p : pts) control.push_back(p.position);

  DerivedGeometry g;
  try {
    // 1. Interpolation -> dense polyline.
    std::size_t per = 1;
    std::optional<Path> dense;
    if (s.interpolation == Interpolation::Polyline) {
      dense = Path::create(control, d.closure());
    } else {
      per = 16;
      if (s.resampleSpacing > 0.0) {
        const double want = std::ceil(4.0 * maxChord / s.resampleSpacing);
        per = static_cast<std::size_t>(std::clamp(want, 8.0, 1024.0));
      }
      if (static_cast<double>(segs) * static_cast<double>(per) > static_cast<double>(kMaxDenseSamples)) {
        return fail(d, DerivedStatus::Failed,
                    "too many interpolation samples; increase the resample spacing");
      }
      dense = geometry::catmullRomSpline(control, d.closure(), {per, s.alpha});
    }
    const double length = dense->length();
    g.controlArcLength.resize(n);
    for (std::size_t i = 0; i < n; ++i) g.controlArcLength[i] = dense->arcLengths()[i * per];

    // 2. Optional equal arc-length resampling.
    std::optional<Path> centerline;
    if (s.resampleSpacing > 0.0) {
      if (length / s.resampleSpacing > static_cast<double>(kMaxDerivedSamples)) {
        return fail(d, DerivedStatus::Failed, "resample spacing too small for this track length");
      }
      const geometry::ResampleResult r = geometry::resampleByArcLength(*dense, s.resampleSpacing);
      centerline = r.path;
      const std::size_t count = centerline->size();
      g.sampleArcLength.resize(count);
      for (std::size_t k = 0; k < count; ++k) g.sampleArcLength[k] = static_cast<double>(k) * r.spacing;
      if (!closed) g.sampleArcLength.back() = length;  // the last sample is the exact end point
    } else {
      centerline = *dense;
      g.sampleArcLength = dense->arcLengths();
    }

    // 3. Widths interpolated in arc length between control points.
    std::vector<geometry::WidthKnot> left, right;
    left.reserve(n);
    right.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
      left.push_back({g.controlArcLength[i], pts[i].leftWidth});
      right.push_back({g.controlArcLength[i], pts[i].rightWidth});
    }
    g.data.name = d.name();
    g.data.closure = d.closure();
    g.data.centerline = centerline->points();
    g.data.leftWidths = geometry::evaluateWidthProfile(left, length, d.closure(), g.sampleArcLength);
    g.data.rightWidths = geometry::evaluateWidthProfile(right, length, d.closure(), g.sampleArcLength);
    g.parameterLength = length;
    g.denseSamplesPerSegment = per;

    // 4. Track (frames, boundaries, local diagnostics).
    g.track = geometry::Track::create(*centerline, g.data.leftWidths, g.data.rightWidths);
    g.status = DerivedStatus::Ok;
  } catch (const GeometryError& e) {
    return fail(d, DerivedStatus::Failed, std::string("geometry generation failed: ") + e.what());
  }
  return g;
}

std::optional<InsertionTarget> findInsertion(const TrackDocument& d, const DerivedGeometry& g, Vec2 p) {
  if (g.status != DerivedStatus::Ok || d.size() < 2) return std::nullopt;
  const auto& c = g.data.centerline;
  const std::size_t m = c.size();
  const bool closed = g.data.closure == Closure::Closed;
  const std::size_t segs = closed ? m : m - 1;

  double best = std::numeric_limits<double>::infinity(), bestS = 0.0, bestLeft = 0.0, bestRight = 0.0;
  Vec2 bestPoint{};
  for (std::size_t k = 0; k < segs; ++k) {
    const Vec2 a = c[k], b = c[(k + 1) % m];
    const Vec2 ab = b - a;
    const double len2 = geometry::dot(ab, ab);
    const double t = len2 > 0.0 ? std::clamp(geometry::dot(p - a, ab) / len2, 0.0, 1.0) : 0.0;
    const Vec2 q = a + t * ab;
    const double dist = geometry::distance(p, q);
    if (dist < best) {
      best = dist;
      bestPoint = q;
      const double s0 = g.sampleArcLength[k];
      const double s1 = k + 1 < m ? g.sampleArcLength[k + 1] : g.parameterLength;
      bestS = s0 + t * (s1 - s0);
      const std::size_t k1 = (k + 1) % m;
      bestLeft = g.data.leftWidths[k] + t * (g.data.leftWidths[k1] - g.data.leftWidths[k]);
      bestRight = g.data.rightWidths[k] + t * (g.data.rightWidths[k1] - g.data.rightWidths[k]);
    }
  }
  // Control segment j: the last control point at or before bestS.
  const auto& ca = g.controlArcLength;
  std::size_t j = static_cast<std::size_t>(std::upper_bound(ca.begin(), ca.end(), bestS) - ca.begin());
  j = j == 0 ? 0 : j - 1;
  if (!closed && j >= d.size() - 1) j = d.size() - 2;  // the open end point belongs to the last segment
  return InsertionTarget{j + 1, bestPoint, best, bestLeft, bestRight};
}

}  // namespace pathforge::editor
