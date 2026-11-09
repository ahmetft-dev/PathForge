#pragma once

#include <cstddef>
#include <optional>
#include <pathforge/geometry/predicates.hpp>
#include <pathforge/geometry/track.hpp>
#include <pathforge/geometry/track_data.hpp>
#include <string>
#include <vector>

namespace pathforge::geometry {

enum class Severity { Info, Warning, Error };

enum class TrackIssue {
  TooFewPoints,                ///< open < 2 points, closed < 3
  WidthCountMismatch,          ///< width arrays do not match the centerline point count
  InvalidWidth,                ///< negative, non-finite or zero width
  NonFiniteCoordinate,         ///< NaN / infinity in a centerline coordinate
  DegenerateSegment,           ///< zero-length segment (duplicate point, incl. closing pair)
  CenterlineSelfIntersection,  ///< centerline crosses/touches itself, or folds straight back
  LeftBoundarySelfIntersection,
  RightBoundarySelfIntersection,
  BoundaryCrossing,   ///< left and right boundaries meet
  BoundaryCollapse,   ///< a boundary meets the centerline, or clearance is ~0
  LocalBoundaryFold,  ///< offset segment reversed, or offset >= local radius of curvature
  NarrowSection,      ///< actual clearance below the (generic) narrow-width threshold
};

enum class PolylineId { Centerline, LeftBoundary, RightBoundary };

/// A segment of one of the track's polylines (segment i joins vertex i and i+1 (mod n)).
struct SegmentRef {
  PolylineId polyline;
  std::size_t index;
};

struct ValidationIssue {
  TrackIssue type;
  Severity severity;
  std::vector<SegmentRef> segments;  ///< affected segments (may be empty)
  std::optional<Vec2> location;      ///< approximate location in world coordinates [m]
  std::string message;               ///< human-readable
};

/// Which groups of checks actually ran. A check group that could not run (for example geometric
/// checks on data whose structure is invalid) stays false: such a track is Incomplete or Invalid,
/// never Valid.
struct CheckCoverage {
  bool structure = false;   ///< counts, finiteness, widths, degenerate segments
  bool centerline = false;  ///< centerline self-intersections
  bool boundaries = false;  ///< left/right boundary self-intersections
  bool crossings = false;   ///< left-vs-right boundary contacts
  bool collapse = false;    ///< boundary-vs-centerline contacts
  bool localFolds = false;  ///< reversed offset segments / radius-of-curvature margin
  bool clearance = false;   ///< clearance metrics + narrow sections
  bool all() const {
    return structure && centerline && boundaries && crossings && collapse && localFolds && clearance;
  }
};

/// Clearance quantities. These are three DIFFERENT things:
///  - configured: the widths requested (left_i + right_i), a property of the input data;
///  - corresponding distance: |left_i - right_i|, distance between the offset points of the SAME
///    vertex. Equals the configured width on a valid track but is NOT a clearance: when the
///    boundaries fold or two track parts come close it can be large while the real gap is tiny;
///  - boundary distance: the actual minimum Euclidean distance between the left and the right
///    boundary POLYLINES (vertex-to-segment distances in both directions, which for polylines that do
///    not cross is exactly the polyline-to-polyline minimum). It is a property of the sampled
///    polylines, not of the underlying continuous curves, and it is 0 when they meet.
/// None of these values may be used to certify validity; validity comes from the intersection
/// checks. No vehicle dimensions are involved.
struct ClearanceMetrics {
  bool computed = false;
  double minConfiguredTotalWidth = 0.0;
  double minCorrespondingDistance = 0.0;
  double minBoundaryDistance = 0.0;
  Vec2 minBoundaryDistanceLocation{};  ///< the vertex at which minBoundaryDistance occurs
  /// Per vertex: min(distance(left_i, right polyline), distance(right_i, left polyline)) [m].
  std::vector<double> vertexClearance;
};

enum class ValidationStatus {
  Valid,       ///< all check groups ran and found no error (warnings possible)
  Invalid,     ///< at least one error
  Incomplete,  ///< no error found but some check group did not run
};

struct ValidationOptions {
  /// Warn where the actual clearance is below this [m]. Generic geometry threshold, NOT a vehicle
  /// requirement.
  double narrowWidthWarning = 2.0;
  /// Error (BoundaryCollapse) where the actual clearance is below this [m].
  double collapseClearance = 0.01;
  /// Report at most this many issues per issue type; the rest are counted in `suppressed`.
  std::size_t maxIssuesPerType = 25;
};

struct ValidationReport {
  std::vector<ValidationIssue> issues;
  std::size_t errorCount = 0;  ///< all errors found (including ones not listed due to the cap)
  std::size_t warningCount = 0;
  std::size_t suppressed = 0;  ///< issues found but not listed because of maxIssuesPerType
  CheckCoverage checked;
  ClearanceMetrics clearance;

  ValidationStatus status() const {
    if (errorCount > 0) return ValidationStatus::Invalid;
    return checked.all() ? ValidationStatus::Valid : ValidationStatus::Incomplete;
  }
};

/// Validates raw track data: first the structure; if the structure allows building a Track, all
/// geometric checks run on it; otherwise they are skipped and recorded as not checked.
///
/// "Valid" means: no centerline/boundary self-intersection, no boundary crossing, no
/// boundary-centerline contact, no reversed offset segment and no collapsed clearance WERE FOUND IN
/// THE SAMPLED POLYLINES. It does not mean the continuous track is valid (features smaller than
/// the sampling can be missed) and it says nothing about physical drivability.
///
/// Intersection predicates are scale-relative (see orientation()); touching and collinear overlap
/// count as intersections for non-adjacent segments. Complexity: O(N^2) in the number of points N
/// (about 7 pair sweeps with bounding-box rejection plus O(N^2) worst-case clearance search with
/// distance pruning). See docs/validation.md for measured timings.
ValidationReport validateTrackData(const TrackData& data, const ValidationOptions& options = {});

/// Same geometric checks for an already built Track (the structure is valid by construction, except
/// that zero widths are still reported).
ValidationReport validateTrack(const Track& track, const ValidationOptions& options = {});

const char* toString(TrackIssue issue);
const char* toString(Severity severity);
const char* toString(ValidationStatus status);

}  // namespace pathforge::geometry
