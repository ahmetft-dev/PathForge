#pragma once

#include <cstddef>
#include <pathforge/geometry/path.hpp>
#include <vector>

namespace pathforge::geometry {

struct SplineOptions {
  /// Output samples per control-point interval (>= 1, <= 100000). Sample k of interval i is the
  /// spline at local parameter k / samplesPerSegment, so the control points themselves are
  /// always output exactly and spacing between samples is uniform in the *knot* parameter, not in
  /// arc length (use resampleByArcLength() for that).
  std::size_t samplesPerSegment = 16;

  /// Knot exponent in [0, 1]: knot spacing = |P(i+1) - P(i)|^alpha.
  /// 0.5 = centripetal (default), 0 = uniform, 1 = chordal.
  double alpha = 0.5;
};

/// Catmull-Rom spline interpolating `control` (evaluated with the Barry-Goldman pyramid scheme).
///
/// Why centripetal (alpha = 0.5): for non-uniformly spaced control points the uniform variant can
/// form cusps and self-loops inside a segment; Yuksel et al. (2011) prove the centripetal variant
/// produces neither within a curve segment. This is NOT a statement about overshoot: depending on
/// the data, centripetal can overshoot more or less than uniform (see tests).
///
/// Closed: control points wrap around (C1 continuous at the seam); the output does not repeat
///   the first point; size = n * samplesPerSegment.
/// Open: the two missing end neighbours are reflected phantom points (2*P0 - P1, 2*Pn-1 - Pn-2);
///   the output includes the last control point; size = (n - 1) * samplesPerSegment + 1.
///   Two control points give a straight line.
///
/// Properties and limitations:
///  - Interpolating and C1 continuous; curvature is generally NOT continuous (C2 is not provided).
///  - Overshoot between control points is possible, as is self-intersection of the whole curve and
///    of offset boundaries. Nothing here guarantees a drivable or valid track.
///  - The output is a polyline of the spline samples; polyline length underestimates the true
///    spline length by O(h^2).
///
/// @throws GeometryError for invalid control points (see validate(): non-finite, too few,
///   consecutive duplicates incl. closing pair), alpha outside [0, 1], or invalid samplesPerSegment.
Path catmullRomSpline(const std::vector<Vec2>& control, Closure closure, SplineOptions options = {});

}  // namespace pathforge::geometry
