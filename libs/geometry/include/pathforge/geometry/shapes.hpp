#pragma once

#include <cstddef>
#include <pathforge/geometry/path.hpp>

namespace pathforge::geometry {

/// Perimeter of the rounded rectangle centerline, exact: 2(L - 2r) + 2(W - 2r) + 2*pi*r.
double roundedRectanglePerimeter(double length, double width, double cornerRadius);

/// Closed CCW rounded rectangle centred at the origin: `length` is the extent along x, `width`
/// the extent along y, corners are quarter circles of radius `cornerRadius` (the centerline
/// itself, not an inner/outer edge). Straight sections have zero curvature and corners have
/// curvature exactly 1/cornerRadius. Vertices lie EXACTLY on the analytical curve at equal arc
/// length spacing P / pointCount; vertex 0 is the midpoint of the bottom straight (0, -width/2),
/// heading +x. Corner-radius limits: 0 < r <= min(length, width) / 2.
///
/// The sampled polygon is slightly shorter than the analytical perimeter (chords across corners);
/// the difference is O(spacing^2 / r) discretisation error.
///
/// @throws GeometryError for non-finite or non-positive dimensions, r outside the limit, or
///   pointCount < 8.
Path makeRoundedRectangle(double length, double width, double cornerRadius, std::size_t pointCount);

/// Stadium ("oval") circuit: rounded rectangle with semicircular ends, r = width / 2. This is NOT
/// an ellipse: it has constant-curvature turns and exactly straight sections (requires
/// length >= width; length == width gives a circle).
Path makeStadium(double length, double width, std::size_t pointCount);

/// Open S-shaped test path, travelling +x over `length`, through 17 deterministic control points on
/// y = -amplitude * sin(2 pi x / length), interpolated by a centripetal Catmull-Rom spline and
/// resampled to `pointCount` points at equal arc-length spacing (endpoints included).
///
/// The first half turns LEFT (positive curvature), the second half turns RIGHT (negative); the
/// curvature sign changes once, near x = length / 2. Starts at (0, 0) and ends at (length, 0).
/// The spline approximates the sine (it is not the sine itself); its length and curvature are
/// numerical results. Caveat: a Catmull-Rom spline is only C1, its curvature is discontinuous at the
/// control points, and with 17 control points it deviates from the sine's true curvature by up to ~27%
/// of the peak value (measured, docs/curvature-study.md); small-magnitude sign flips occur near the
/// ends and the inflection. This is a property of the spline, not of the discrete estimator. Count sign
/// changes with a threshold relative to the peak curvature (see countSignChanges()).
///
Path makeSCurve(double length, double amplitude, std::size_t pointCount);

}  // namespace pathforge::geometry
