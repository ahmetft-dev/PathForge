#pragma once

#include <pathforge/geometry/path.hpp>

#include <cstddef>

namespace pathforge::geometry {

/// Closed polygon with `pointCount` vertices exactly on a circle of radius `radius` [m]
/// centred at the origin, counter-clockwise, first vertex at angle 0.
/// Deterministic: vertex i is (R cos(2 pi i / N), R sin(2 pi i / N)).
///
/// @throws GeometryError if radius is not finite and > 0, or pointCount < 3.
Path makeCircle(double radius, std::size_t pointCount);

/// Analytical circumference of the continuous circle, 2 pi R.
double circleCircumference(double radius);

/// Perimeter of the regular N-gon inscribed in the circle: 2 N R sin(pi / N).
/// This is what the discrete path length equals (up to rounding) and is the
/// correct reference for the *sampled* geometry; it differs from 2 pi R by
/// about pi^2 R / (3 N^2) (relative error ~ pi^2 / (6 N^2)).
double inscribedPolygonPerimeter(double radius, std::size_t pointCount);

}  // namespace pathforge::geometry
