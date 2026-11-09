#pragma once

#include <pathforge/geometry/path.hpp>

namespace pathforge::geometry {

struct ResampleResult {
  Path path;
  /// Actual spacing [m] measured ALONG THE INPUT POLYLINE between consecutive output samples
  /// (= path.length() / intervals). It differs from the requested spacing only through rounding of
  /// the interval count: relative difference <= 0.5 * requested / length (0 when length / requested
  /// is an integer), except when the minimum sample count applies (closed paths: 3).
  double spacing;
};

/// Places samples at equal arc-length intervals along the input polyline, using linear
/// interpolation inside each input segment.
///
/// Definition of length: the polyline length (sum of segment lengths; for closed paths including
/// the closing segment). The interval count is round(length / spacing) so the spacing is adjusted
/// slightly to fit exactly:
///  - Closed: N = max(3, round(L / spacing)) samples, N intervals; sample 0 is the input's first
///    point; the seam is NOT duplicated (the closing interval runs last -> first).
///  - Open: N = max(1, round(L / spacing)) intervals, N + 1 samples; first and last samples equal
///    the input endpoints.
///
/// Accuracy: output samples lie exactly on the input polyline, so the result is only as good as
/// the input. The straight-line (chord) distance between consecutive samples is <= the arc spacing
/// and equals it inside straight input segments; across a corner of turning angle phi it is
/// shorter by O(spacing * phi^2). If the input is a dense sampling of a smooth curve (e.g. a
/// spline), the polyline length underestimates the true curve length by O(h^2); this function does
/// not compute exact spline arc length.
///
/// Curvature caveat: resampling does not add information. If the output spacing is smaller than
/// the input's vertex spacing, the output samples subdivide straight input chords and the discrete
/// (Menger) curvature of the output then reflects the input's corner turning angles divided by the
/// smaller spacing, i.e. it grows without bound as the spacing shrinks. Keep the output spacing
/// >= the input vertex spacing (ideally several times larger) when curvature is needed.
///
/// @throws GeometryError if spacing is not finite and > 0, or the sample count would exceed 1e8.
ResampleResult resampleByArcLength(const Path& path, double spacing);

}  // namespace pathforge::geometry
