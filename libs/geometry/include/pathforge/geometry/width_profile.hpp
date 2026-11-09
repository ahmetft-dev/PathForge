#pragma once

#include <pathforge/geometry/path.hpp>
#include <vector>

namespace pathforge::geometry {

/// A width control value at an explicit ARC-LENGTH position along the path.
struct WidthKnot {
  double s;      ///< arc length [m] from vertex 0 along the polyline (see Path::arcLengths())
  double width;  ///< half-width [m], finite and >= 0
};

/// Evaluates a piecewise-LINEAR width profile (in arc length) at every vertex of `path` and returns
/// one width per vertex, ready for Track::create(path, left, right). Deterministic; no smoothing.
///
///  - knots must be non-empty with strictly increasing, finite s.
///  - Open path: 0 <= s <= path.length(); outside [first.s, last.s] the nearest knot's width is held.
///  - Closed path: 0 <= s < path.length(); the profile is PERIODIC: between the last knot and the
///    first knot (+ length) the width is interpolated linearly through the seam, so width is
///    continuous across the wrap-around. A single knot gives a constant width.
///
/// Interpolation is linear in arc length of the *centerline polyline*; vertex i is evaluated at
/// path.arcLengths()[i].
///
/// @throws GeometryError for empty/unsorted/non-finite knots, negative widths, or s out of range.
std::vector<double> sampleWidthProfile(const Path& path, const std::vector<WidthKnot>& knots);

/// Evaluates the same piecewise-linear profile at arbitrary arc-length positions on a path of total length
/// `length` with the given closure (same knot rules and interpolation as sampleWidthProfile(), which is
/// implemented with this function). `positions` must be finite, non-decreasing and within [0, length]
/// ([0, length) for closed paths, where length itself is the seam). Runs in O(positions + knots).
/// @throws GeometryError for invalid knots or positions.
std::vector<double> evaluateWidthProfile(const std::vector<WidthKnot>& knots, double length, Closure closure,
                                         const std::vector<double>& positions);

}  // namespace pathforge::geometry
