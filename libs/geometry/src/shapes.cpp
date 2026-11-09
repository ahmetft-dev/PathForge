#include <algorithm>
#include <cmath>
#include <numbers>
#include <pathforge/geometry/resample.hpp>
#include <pathforge/geometry/shapes.hpp>
#include <pathforge/geometry/spline.hpp>
#include <string>
#include <vector>

namespace pathforge::geometry {
namespace {

constexpr double kPi = std::numbers::pi;

struct Piece {
  double len;
  bool arc;
  Vec2 origin;    // straight: start point; arc: centre
  Vec2 dir;       // straight: unit direction (unused for arcs)
  double angle0;  // arc: start angle (CCW)
};

void requirePositiveFinite(double v, const char* what) {
  if (!std::isfinite(v) || v <= 0.0) throw GeometryError(std::string(what) + " must be finite and > 0");
}

}  // namespace

double roundedRectanglePerimeter(double length, double width, double r) {
  return 2.0 * (length - 2.0 * r) + 2.0 * (width - 2.0 * r) + 2.0 * kPi * r;
}

Path makeRoundedRectangle(double length, double width, double r, std::size_t pointCount) {
  requirePositiveFinite(length, "length");
  requirePositiveFinite(width, "width");
  requirePositiveFinite(r, "corner radius");
  if (2.0 * r > std::min(length, width) * (1.0 + 1e-12)) {
    throw GeometryError("corner radius must be <= min(length, width) / 2");
  }
  if (pointCount < 8) throw GeometryError("rounded rectangle needs at least 8 points");

  const double a = std::max(0.0, length / 2.0 - r);  // half straight length along x
  const double b = std::max(0.0, width / 2.0 - r);   // half straight length along y
  const double qa = kPi * r / 2.0;                   // quarter-arc length
  // Arc pieces: the angle advances by s / r from angle0 around `centre`.
  auto arc = [&](Vec2 centre, double a0) { return Piece{qa, true, centre, {0.0, 0.0}, a0}; };
  const std::vector<Piece> pieces = {
      {a, false, {0.0, -width / 2.0}, {1.0, 0.0}, 0.0},       arc({a, -b}, -kPi / 2.0),
      {2.0 * b, false, {length / 2.0, -b}, {0.0, 1.0}, 0.0},  arc({a, b}, 0.0),
      {2.0 * a, false, {a, width / 2.0}, {-1.0, 0.0}, 0.0},   arc({-a, b}, kPi / 2.0),
      {2.0 * b, false, {-length / 2.0, b}, {0.0, -1.0}, 0.0}, arc({-a, -b}, kPi),
      {a, false, {-a, -width / 2.0}, {1.0, 0.0}, 0.0},
  };

  const double perimeter = roundedRectanglePerimeter(length, width, r);
  std::vector<Vec2> pts;
  pts.reserve(pointCount);
  for (std::size_t k = 0; k < pointCount; ++k) {
    double s = perimeter * static_cast<double>(k) / static_cast<double>(pointCount);
    std::size_t i = 0;
    while (i + 1 < pieces.size() && s > pieces[i].len) s -= pieces[i++].len;
    const Piece& p = pieces[i];
    if (p.arc) {
      const double ang = p.angle0 + s / r;
      pts.push_back(p.origin + Vec2{r * std::cos(ang), r * std::sin(ang)});
    } else {
      pts.push_back(p.origin + s * p.dir);
    }
  }
  return Path::create(std::move(pts), Closure::Closed);
}

Path makeStadium(double length, double width, std::size_t pointCount) {
  requirePositiveFinite(length, "length");
  requirePositiveFinite(width, "width");
  if (length < width) throw GeometryError("stadium length must be >= width");
  return makeRoundedRectangle(length, width, width / 2.0, pointCount);
}

Path makeSCurve(double length, double amplitude, std::size_t pointCount) {
  requirePositiveFinite(length, "length");
  requirePositiveFinite(amplitude, "amplitude");
  if (pointCount < 8) throw GeometryError("S-curve needs at least 8 points");

  constexpr int kSegments = 16;
  std::vector<Vec2> control;
  control.reserve(kSegments + 1);
  for (int i = 0; i <= kSegments; ++i) {
    const double x = length * i / kSegments;
    // Endpoints are set exactly: sin(2 pi) is not exactly zero in floating point.
    const double y = (i == 0 || i == kSegments) ? 0.0 : -amplitude * std::sin(2.0 * kPi * i / kSegments);
    control.push_back({x, y});
  }
  SplineOptions opt;
  // The dense polyline must be much finer than the final sampling: curvature of resampled points
  // is only meaningful if the output spacing is >= the input vertex spacing (see resample.hpp).
  // 16 intervals * max(64, pointCount) dense vertices is >= 16x oversampling.
  opt.samplesPerSegment = std::max<std::size_t>(64, pointCount);
  const Path dense = catmullRomSpline(control, Closure::Open, opt);
  const double spacing = dense.length() / static_cast<double>(pointCount - 1);
  return resampleByArcLength(dense, spacing).path;
}

}  // namespace pathforge::geometry
