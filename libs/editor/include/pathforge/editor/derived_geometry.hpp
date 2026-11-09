#pragma once

#include <cstddef>
#include <optional>
#include <pathforge/editor/track_document.hpp>
#include <pathforge/geometry/track.hpp>
#include <pathforge/geometry/track_data.hpp>
#include <pathforge/geometry/validation.hpp>
#include <string>
#include <vector>

namespace pathforge::editor {

enum class DerivedStatus {
  Ok,            ///< centerline generated and a Track (with boundaries) built
  TooFewPoints,  ///< open < 2 / closed < 3 control points: nothing to interpolate yet
  Failed,        ///< generation impossible (e.g. coincident consecutive control points, sample limit)
};

/// Health of the interpolating curve itself (not of the track corridor), measured analytically at the control
/// points and at three interior fractions of every control segment. Only the C2 spline is checked: its first
/// and second derivatives are available in closed form. Catmull-Rom and polyline interpolation leave it
/// empty (`checked` false); the cusp case of Catmull-Rom already fails generation (coincident samples).
struct SplineHealth {
  bool checked = false;
  std::size_t degenerateSamples = 0;    ///< evaluation points with (almost) zero velocity
  std::size_t nonFiniteSamples = 0;     ///< evaluation points with a non-finite derivative or curvature
  std::optional<Vec2> firstDegenerate;  ///< location of the first of each kind
  std::optional<Vec2> firstNonFinite;
  std::size_t firstDegenerateSegment = 0;  ///< control segment of the first of each kind
  std::size_t firstNonFiniteSegment = 0;
};

/// Everything computed FROM a TrackDocument. Never edited directly; regenerate after every document change.
///
/// Pipeline: control points -> interpolation (Catmull-Rom spline with `denseSamplesPerSegment` samples per
/// control segment, or the control polygon itself) -> optional equal arc-length resampling -> per-sample
/// widths -> Track.
///
/// Parameterisation used for widths and point insertion: arc length `s` along the DENSE interpolated polyline
/// (before resampling), total `parameterLength`. Control point i sits at `controlArcLength[i]`; derived
/// sample k at `sampleArcLength[k]` (exactly k * spacing when resampled, because resampling measures arc
/// length along that same polyline). Widths are the control widths interpolated linearly in s (periodically
/// for closed tracks).
///
/// When status != Ok, `data` holds the raw control polygon with the control widths, so the validation engine
/// can still explain what is wrong (too few points, duplicates, ...); `track` is empty.
struct DerivedGeometry {
  DerivedStatus status = DerivedStatus::TooFewPoints;
  std::string message;  ///< reason when status != Ok
  geometry::TrackData data;
  std::optional<geometry::Track> track;
  std::vector<double> controlArcLength;
  std::vector<double> sampleArcLength;
  double parameterLength = 0.0;
  std::size_t denseSamplesPerSegment = 0;  ///< 1 for polyline interpolation
  SplineHealth splineHealth;
};

/// Adds the spline-health findings to a validation report as InvalidSplineDerivative and
/// CurvatureEvaluationFailure errors (one issue per kind, with the number of affected evaluation points). The
/// two issue types can only come from the curve, so the track-level validation (which sees only samples) does
/// not produce them itself. Call this on every report obtained for `derived`, synchronous or asynchronous.
void mergeSplineHealth(geometry::ValidationReport& report, const SplineHealth& health);

/// Hard limits on generated geometry, so that a pathological document (e.g. a 1 mm spacing on a 10 km track)
/// produces a clear failure instead of exhausting memory or stalling the UI.
inline constexpr std::size_t kMaxDenseSamples = 4'000'000;
inline constexpr std::size_t kMaxDerivedSamples = 1'000'000;

/// Deterministic: the same document always produces bit-identical results.
///
/// Catmull-Rom density: with resampling on, every control segment gets
/// clamp(ceil(4 * longest control chord / spacing), 8, 1024) dense samples, so the dense polyline is at least
/// ~4x finer than the output spacing (resampling finer than its input makes curvature meaningless, see
/// resample.hpp); with resampling off, 16 per segment. Interpolation keeps the C1-only limitation of
/// Catmull-Rom (docs/curvature-study.md).
DerivedGeometry generateGeometry(const TrackDocument& document);

/// Where a point clicked near the track should be inserted.
struct InsertionTarget {
  std::size_t index;  ///< insert before this control-point index (== size(): after the last point)
  Vec2 position;      ///< the nearest point ON the derived centerline
  double distance;    ///< distance from the query point to `position` [m]
  double leftWidth;   ///< half-widths of the derived track at `position` (linear between samples), so that a
  double rightWidth;  ///< point inserted there with these widths leaves the boundaries unchanged
};

/// Finds the derived-centerline point nearest to `p` and maps it, through the arc-length parameter, to the
/// control segment it belongs to. For closed tracks the segment from the last control point back to the first
/// maps to index == size(). Requires status Ok (returns nullopt otherwise). O(number of derived samples).
std::optional<InsertionTarget> findInsertion(const TrackDocument& document, const DerivedGeometry& derived,
                                             Vec2 p);

}  // namespace pathforge::editor
