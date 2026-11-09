#include <cmath>
#include <pathforge/geometry/spline.hpp>

namespace pathforge::geometry {
namespace {

constexpr std::size_t kMaxSamplesPerSegment = 100000;

// Position with first and second derivative with respect to the knot parameter t.
struct Jet {
  Vec2 v;
  Vec2 d1;
  Vec2 d2;
};

// Linear blend R(t) = ((tb - t) A(t) + (t - ta) B(t)) / (tb - ta) of two jets; the weights are linear in t
// so the product rule gives R' = ((tb - t) A' + (t - ta) B' + (B - A)) / D and
// R'' = ((tb - t) A'' + (t - ta) B'' + 2 (B' - A')) / D. The value part uses exactly the same operations
// as a plain Barry-Goldman evaluation.
Jet blend(const Jet& a, const Jet& b, double ta, double tb, double t) {
  const double inv = 1.0 / (tb - ta);
  const double wa = tb - t, wb = t - ta;
  return {(wa * a.v + wb * b.v) * inv, (wa * a.d1 + wb * b.d1 + (b.v - a.v)) * inv,
          (wa * a.d2 + wb * b.d2 + 2.0 * (b.d1 - a.d1)) * inv};
}

// Barry-Goldman pyramidal evaluation of the Catmull-Rom segment between p1 and p2 at local
// parameter u in [0, 1]. Knots: t0 = 0, t(i+1) = t(i) + |p(i+1) - p(i)|^alpha. All knot
// intervals are > 0 because consecutive control points are validated to be distinct.
Jet evalSegmentJet(Vec2 p0, Vec2 p1, Vec2 p2, Vec2 p3, double alpha, double u) {
  const double t0 = 0.0;
  const double t1 = t0 + std::pow(distance(p0, p1), alpha);
  const double t2 = t1 + std::pow(distance(p1, p2), alpha);
  const double t3 = t2 + std::pow(distance(p2, p3), alpha);
  const double t = t1 + u * (t2 - t1);

  const Jet a1 = blend({p0, {}, {}}, {p1, {}, {}}, t0, t1, t);
  const Jet a2 = blend({p1, {}, {}}, {p2, {}, {}}, t1, t2, t);
  const Jet a3 = blend({p2, {}, {}}, {p3, {}, {}}, t2, t3, t);
  const Jet b1 = blend(a1, a2, t0, t2, t);
  const Jet b2 = blend(a2, a3, t1, t3, t);
  return blend(b1, b2, t1, t2, t);
}

Vec2 evalSegment(Vec2 p0, Vec2 p1, Vec2 p2, Vec2 p3, double alpha, double u) {
  return evalSegmentJet(p0, p1, p2, p3, alpha, u).v;
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

std::vector<double> catmullRomCurvature(const std::vector<Vec2>& control, Closure closure,
                                        SplineOptions options) {
  if (!(options.alpha >= 0.0 && options.alpha <= 1.0)) throw GeometryError("spline alpha must be in [0, 1]");
  if (options.samplesPerSegment < 1 || options.samplesPerSegment > kMaxSamplesPerSegment) {
    throw GeometryError("samplesPerSegment must be in [1, 100000]");
  }
  const PathValidation v = validate(control, closure);
  if (!v.ok()) throw GeometryError("invalid control points: " + v.message());

  const std::size_t n = control.size();
  const std::size_t s = options.samplesPerSegment;
  const bool closed = closure == Closure::Closed;
  const std::size_t segments = closed ? n : n - 1;
  auto at = [&](std::ptrdiff_t i) -> Vec2 {
    const auto cnt = static_cast<std::ptrdiff_t>(n);
    if (closed) return control[static_cast<std::size_t>(((i % cnt) + cnt) % cnt)];
    if (i < 0) return 2.0 * control[0] - control[1];
    if (i >= cnt) return 2.0 * control[n - 1] - control[n - 2];
    return control[static_cast<std::size_t>(i)];
  };
  auto kappa = [](const Jet& j) {
    const double sp2 = dot(j.d1, j.d1);
    if (sp2 <= 0.0) return 0.0;  // stationary point: curvature undefined
    return cross(j.d1, j.d2) / (sp2 * std::sqrt(sp2));
  };

  std::vector<double> out;
  out.reserve(segments * s + 1);
  for (std::size_t seg = 0; seg < segments; ++seg) {
    const auto i = static_cast<std::ptrdiff_t>(seg);
    for (std::size_t k = 0; k < s; ++k) {
      const double u = static_cast<double>(k) / static_cast<double>(s);
      out.push_back(kappa(evalSegmentJet(at(i - 1), at(i), at(i + 1), at(i + 2), options.alpha, u)));
    }
  }
  if (!closed) {  // last control point: left-sided limit from the final segment (u = 1)
    const auto i = static_cast<std::ptrdiff_t>(segments - 1);
    out.push_back(kappa(evalSegmentJet(at(i - 1), at(i), at(i + 1), at(i + 2), options.alpha, 1.0)));
  }
  return out;
}

}  // namespace pathforge::geometry
