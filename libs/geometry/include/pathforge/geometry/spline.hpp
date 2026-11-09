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

/// ANALYTIC signed curvature of the same spline, kappa = (x' y'' - y' x'') / (x'^2 + y'^2)^(3/2), with the
/// derivatives taken exactly (product rule through the Barry-Goldman pyramid) with respect to the knot
/// parameter. Returned at exactly the parametric samples that catmullRomSpline() emits for the same arguments
/// (same size and order), positive = left turn.
///
/// This is a DIFFERENT quantity from Path::curvature() (Menger curvature of the sampled polyline); the two
/// must not be mixed. Differences that matter:
///  - It is the curvature of the spline curve itself, free of discretisation error of the sampling.
///  - Catmull-Rom is only C1: curvature is DISCONTINUOUS at control points. At a control point the value of
///  the
///    segment that STARTS there (u = 0) is returned; for an open spline the last control point returns the
///    left-sided limit of the last segment (u = 1).
///  - Within a segment the curve is a cubic, so curvature varies smoothly; across a control point it jumps.
///    This jump (not the polyline discretisation) is the origin of the "noise" seen in Menger curvature of
///    sampled splines. See docs/curvature-study.md.
/// Where the first derivative vanishes (a cusp) the curvature is undefined and 0 is returned.
/// @throws GeometryError for the same invalid inputs as catmullRomSpline().
std::vector<double> catmullRomCurvature(const std::vector<Vec2>& control, Closure closure,
                                        SplineOptions options = {});

}  // namespace pathforge::geometry
