#include <cmath>
#include <pathforge/geometry/spline.hpp>

namespace pathforge::geometry {
namespace {

constexpr std::size_t kMaxSamplesPerSegment = 100000;

// Barry-Goldman pyramidal evaluation of the Catmull-Rom segment between p1 and p2 at local
// parameter u in [0, 1]. Knots: t0 = 0, t(i+1) = t(i) + |p(i+1) - p(i)|^alpha. All knot
// intervals are > 0 because consecutive control points are validated to be distinct.
Vec2 evalSegment(Vec2 p0, Vec2 p1, Vec2 p2, Vec2 p3, double alpha, double u) {
  const double t0 = 0.0;
  const double t1 = t0 + std::pow(distance(p0, p1), alpha);
  const double t2 = t1 + std::pow(distance(p1, p2), alpha);
  const double t3 = t2 + std::pow(distance(p2, p3), alpha);
  const double t = t1 + u * (t2 - t1);

  const Vec2 a1 = ((t1 - t) * p0 + (t - t0) * p1) * (1.0 / (t1 - t0));
  const Vec2 a2 = ((t2 - t) * p1 + (t - t1) * p2) * (1.0 / (t2 - t1));
  const Vec2 a3 = ((t3 - t) * p2 + (t - t2) * p3) * (1.0 / (t3 - t2));
  const Vec2 b1 = ((t2 - t) * a1 + (t - t0) * a2) * (1.0 / (t2 - t0));
  const Vec2 b2 = ((t3 - t) * a2 + (t - t1) * a3) * (1.0 / (t3 - t1));
  return ((t2 - t) * b1 + (t - t1) * b2) * (1.0 / (t2 - t1));
}

}  // namespace

Path catmullRomSpline(const std::vector<Vec2>& control, Closure closure, SplineOptions options) {
  if (!(options.alpha >= 0.0 && options.alpha <= 1.0)) {  // also rejects NaN
    throw GeometryError("spline alpha must be in [0, 1]");
  }
  if (options.samplesPerSegment < 1 || options.samplesPerSegment > kMaxSamplesPerSegment) {
    throw GeometryError("samplesPerSegment must be in [1, 100000]");
  }
  const PathValidation v = validate(control, closure);
  if (!v.ok()) throw GeometryError("invalid control points: " + v.message());

  const std::size_t n = control.size();
  const std::size_t s = options.samplesPerSegment;
  const bool closed = closure == Closure::Closed;
  const std::size_t segments = closed ? n : n - 1;

  // Neighbour lookup incl. wrap-around (closed) or reflection (open).
  auto at = [&](std::ptrdiff_t i) -> Vec2 {
    const auto cnt = static_cast<std::ptrdiff_t>(n);
    if (closed) return control[static_cast<std::size_t>(((i % cnt) + cnt) % cnt)];
    if (i < 0) return 2.0 * control[0] - control[1];
    if (i >= cnt) return 2.0 * control[n - 1] - control[n - 2];
    return control[static_cast<std::size_t>(i)];
  };

  std::vector<Vec2> out;
  out.reserve(segments * s + 1);
  for (std::size_t seg = 0; seg < segments; ++seg) {
    const auto i = static_cast<std::ptrdiff_t>(seg);
    out.push_back(control[seg]);  // u = 0: exact control point, no rounding
    for (std::size_t k = 1; k < s; ++k) {
      const double u = static_cast<double>(k) / static_cast<double>(s);
      out.push_back(evalSegment(at(i - 1), at(i), at(i + 1), at(i + 2), options.alpha, u));
    }
  }
  if (!closed) out.push_back(control[n - 1]);
  return Path::create(std::move(out), closure);
}

}  // namespace pathforge::geometry
