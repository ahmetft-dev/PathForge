#pragma once

#include <cstddef>
#include <pathforge/geometry/path.hpp>
#include <vector>

namespace pathforge::geometry {

/// Per-vertex Frenet-like quantities of a polyline.
///
/// Conventions (right-handed frame, x right, y up, meters, radians, CCW positive):
///  - tangent T: unit vector along the direction of travel (increasing index).
///  - left normal N = (-T.y, T.x), i.e. T rotated +90 degrees. For a CCW path N points toward the
///    inside of the turn; for a CW path it points outward.
///  - curvature: signed, positive for left (CCW) turns; see Path::curvature() for the definition.
///    It is a DISCRETE (three-point, Menger) estimate, not a spline-derivative curvature.
///  - arcLength: cumulative polyline length [m] at each vertex (starts at 0).
struct PathFrames {
  std::vector<Vec2> tangent;
  std::vector<Vec2> normal;
  std::vector<double> curvature;
  std::vector<double> arcLength;
};

/// Tangent estimate at an interior (or any closed-path) vertex with neighbours a, b, c:
///   T ~ normalize( |c-b| * (b-a)/|b-a|  +  |b-a| * (c-b)/|c-b| ).
/// This length-weighted bisector of the unit chord directions is EXACTLY the tangent of the circle
/// through a, b, c (for any spacing, not only symmetric: the perpendicular components
/// -|c-b| sin(t1/2) + |b-a| sin(t2/2) cancel because chord = 2R sin(arc angle / 2)), so it is exact
/// on sampled circles and second-order accurate on smooth curves. If the two chords exactly
/// reverse (a cusp; tangent undefined) the outgoing chord direction is used.
/// Closed paths use periodic neighbours (vertex 0 and n-1 wrap around). Open paths use the
/// one-sided chord direction at the first and last vertex (first-order accurate there).
PathFrames computeFrames(const Path& path);

/// Number of sign changes in `curvature`, ignoring entries with |k| <= threshold (so nearly
/// straight sections neither start nor end a sign run). Closed paths: no wrap-around comparison.
std::size_t countSignChanges(const std::vector<double>& curvature, double threshold);

}  // namespace pathforge::geometry
