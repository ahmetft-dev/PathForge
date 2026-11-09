#pragma once

#include <cstddef>
#include <pathforge/geometry/vec2.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace pathforge::geometry {

/// Open: segments connect consecutive points only.
/// Closed: an additional segment connects the last point back to the first.
///         The first point must NOT be repeated at the end.
enum class Closure { Open, Closed };

/// Default minimum segment length [m] below which two consecutive points are
/// considered duplicates. 1e-9 m is far below any physically meaningful track
/// resolution yet well above double-precision noise for coordinates up to ~1e5 m.
inline constexpr double kDefaultMinSegmentLength = 1e-9;

enum class PathIssue {
  None,
  InvalidTolerance,   ///< minSegmentLength is NaN, infinite or negative.
  TooFewPoints,       ///< Open: < 2 points. Closed: < 3 points.
  NonFinitePoint,     ///< NaN or infinity in a coordinate.
  DegenerateSegment,  ///< Two consecutive points (incl. closing pair) closer than the tolerance.
};

struct PathValidation {
  PathIssue issue = PathIssue::None;
  std::size_t index = 0;  ///< Offending point (or first point of the offending segment).
  bool ok() const { return issue == PathIssue::None; }
  std::string message() const;
};

/// Non-throwing validation of a point sequence.
PathValidation validate(const std::vector<Vec2>& points, Closure closure,
                        double minSegmentLength = kDefaultMinSegmentLength);

/// Thrown by Path::create / generators for invalid input.
class GeometryError : public std::invalid_argument {
 public:
  using std::invalid_argument::invalid_argument;
};

/// Immutable, validated polyline. Construction guarantees: all coordinates finite,
/// enough points for the closure type, no segment shorter than the tolerance.
class Path {
 public:
  /// @throws GeometryError if validate() fails.
  static Path create(std::vector<Vec2> points, Closure closure,
                     double minSegmentLength = kDefaultMinSegmentLength);

  const std::vector<Vec2>& points() const { return points_; }
  Closure closure() const { return closure_; }
  bool isClosed() const { return closure_ == Closure::Closed; }
  std::size_t size() const { return points_.size(); }

  /// Number of segments: n-1 (open) or n (closed).
  std::size_t segmentCount() const { return isClosed() ? points_.size() : points_.size() - 1; }

  /// Cumulative arc length [m] at each point; arcLengths()[0] == 0.
  /// Same size as points(). For closed paths the closing segment is not
  /// included here but is included in length().
  const std::vector<double>& arcLengths() const { return arc_; }

  /// Total length [m] of the polyline (includes closing segment if closed).
  double length() const { return length_; }

  /// Discrete signed curvature [1/m] at every point (positive = left turn / CCW).
  ///
  /// Uses the Menger curvature of each point and its two neighbours:
  ///   kappa = 2 * cross(b-a, c-b) / (|b-a| |c-b| |c-a|),
  /// i.e. the reciprocal radius of the circle through the three points. It is
  /// exact when the samples lie on a circle and a discrete approximation
  /// otherwise (error depends on sampling density and smoothness).
  /// Closed: neighbours wrap around. Open: endpoints have no two neighbours,
  /// so they take the value of the nearest interior point; requires >= 3 points
  /// (returns zeros for a 2-point open path, which is a straight line).
  std::vector<double> curvature() const;

 private:
  Path(std::vector<Vec2> points, Closure closure);

  std::vector<Vec2> points_;
  Closure closure_;
  std::vector<double> arc_;
  double length_ = 0.0;
};

/// Summary used by the CLI and the Studio diagnostics panel.
struct PathStats {
  std::size_t pointCount = 0;
  double length = 0.0;            ///< [m]
  double minSegment = 0.0;        ///< [m]
  double maxSegment = 0.0;        ///< [m]
  double minCurvature = 0.0;      ///< [1/m], signed
  double maxCurvature = 0.0;      ///< [1/m], signed
  double meanAbsCurvature = 0.0;  ///< [1/m]
  double minAbsCurvature = 0.0;   ///< [1/m]
  double maxAbsCurvature = 0.0;   ///< [1/m]
};

PathStats computeStats(const Path& path);

/// Same points in reverse order (travel direction flipped, closure kept). Curvature changes sign.
Path reversed(const Path& path);

}  // namespace pathforge::geometry
