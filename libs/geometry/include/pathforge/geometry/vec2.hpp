#pragma once

#include <cmath>

namespace pathforge::geometry {

/// 2D point / vector in a right-handed Cartesian frame (x right, y up).
/// Units: meters. Angles elsewhere in the library are radians, CCW positive.
struct Vec2 {
  double x = 0.0;
  double y = 0.0;

  friend constexpr Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
  friend constexpr Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
  friend constexpr Vec2 operator*(Vec2 a, double s) { return {a.x * s, a.y * s}; }
  friend constexpr Vec2 operator*(double s, Vec2 a) { return a * s; }
  friend constexpr bool operator==(Vec2 a, Vec2 b) = default;
};

constexpr double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }

/// z-component of the 3D cross product; positive when b is CCW from a.
constexpr double cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }

inline double norm(Vec2 a) { return std::hypot(a.x, a.y); }
inline double distance(Vec2 a, Vec2 b) { return norm(b - a); }
inline bool isFinite(Vec2 a) { return std::isfinite(a.x) && std::isfinite(a.y); }

}  // namespace pathforge::geometry
