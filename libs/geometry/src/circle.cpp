#include <pathforge/geometry/circle.hpp>

#include <cmath>
#include <numbers>
#include <vector>

namespace pathforge::geometry {

Path makeCircle(double radius, std::size_t pointCount) {
  if (!std::isfinite(radius) || radius <= 0.0) {
    throw GeometryError("circle radius must be finite and > 0");
  }
  if (pointCount < 3) {
    throw GeometryError("circle needs at least 3 points");
  }
  std::vector<Vec2> pts;
  pts.reserve(pointCount);
  const double n = static_cast<double>(pointCount);
  for (std::size_t i = 0; i < pointCount; ++i) {
    const double a = 2.0 * std::numbers::pi * static_cast<double>(i) / n;
    pts.push_back({radius * std::cos(a), radius * std::sin(a)});
  }
  return Path::create(std::move(pts), Closure::Closed);
}

double circleCircumference(double radius) { return 2.0 * std::numbers::pi * radius; }

double inscribedPolygonPerimeter(double radius, std::size_t pointCount) {
  const double n = static_cast<double>(pointCount);
  return 2.0 * n * radius * std::sin(std::numbers::pi / n);
}

}  // namespace pathforge::geometry
