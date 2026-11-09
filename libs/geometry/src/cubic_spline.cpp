#include <cmath>
#include <limits>
#include <pathforge/geometry/cubic_spline.hpp>

namespace pathforge::geometry {
namespace {

constexpr std::size_t kMaxSamplesPerSegment = 100000;

// Thomas algorithm for a tridiagonal system: a (sub), b (diag), c (super), a[0] and c[n-1] unused. Stable
// without pivoting because the callers' matrices are strictly diagonally dominant.
std::vector<double> solveTridiagonal(const std::vector<double>& a, const std::vector<double>& b,
                                     const std::vector<double>& c, std::vector<double> d) {
  const std::size_t n = d.size();
  std::vector<double> cp(n), x(n);
  cp[0] = c[0] / b[0];
  d[0] /= b[0];
  for (std::size_t i = 1; i < n; ++i) {
    const double m = b[i] - a[i] * cp[i - 1];
    cp[i] = c[i] / m;
    d[i] = (d[i] - a[i] * d[i - 1]) / m;
  }
  x[n - 1] = d[n - 1];
  for (std::size_t i = n - 1; i-- > 0;) x[i] = d[i] - cp[i] * x[i + 1];
  return x;
}

// Cyclic tridiagonal: as above plus corner entries A[0][n-1] = beta and A[n-1][0] = alpha, solved with the
// Sherman-Morrison formula (Press et al., Numerical Recipes, "cyclic"). n >= 3.
std::vector<double> solveCyclic(const std::vector<double>& a, const std::vector<double>& b,
                                const std::vector<double>& c, double alpha, double beta,
                                const std::vector<double>& r) {
  const std::size_t n = r.size();
  const double gamma = -b[0];
  std::vector<double> bb = b;
  bb[0] = b[0] - gamma;
  bb[n - 1] = b[n - 1] - alpha * beta / gamma;
  const std::vector<double> x = solveTridiagonal(a, bb, c, r);
  std::vector<double> u(n, 0.0);
  u[0] = gamma;
  u[n - 1] = alpha;
  const std::vector<double> z = solveTridiagonal(a, bb, c, u);
  const double fact = (x[0] + beta * x[n - 1] / gamma) / (1.0 + z[0] + beta * z[n - 1] / gamma);
  std::vector<double> out(n);
  for (std::size_t i = 0; i < n; ++i) out[i] = x[i] - fact * z[i];
  return out;
}

}  // namespace

CurvatureValue curvatureFromDerivatives(Vec2 d1, Vec2 d2, double speedScale) {
  constexpr double nan = std::numeric_limits<double>::quiet_NaN();
  if (!isFinite(d1) || !isFinite(d2) || !std::isfinite(speedScale)) return {nan, false};
  const double speed = norm(d1);
  if (!(speed > kMinRelativeSpeed * speedScale) || speed == 0.0) return {nan, false};
  const double k = cross(d1, d2) / (speed * speed * speed);
  if (!std::isfinite(k)) return {nan, false};
  return {k, true};
}

CubicSpline CubicSpline::create(const std::vector<Vec2>& control, Closure closure, double alpha) {
  if (!(alpha >= 0.0 && alpha <= 1.0)) throw GeometryError("spline alpha must be in [0, 1]");
  const PathValidation v = validate(control, closure);
  if (!v.ok()) throw GeometryError("invalid control points: " + v.message());

  CubicSpline s;
  s.closure_ = closure;
  s.points_ = control;
  const std::size_t n = control.size();
  const bool closed = closure == Closure::Closed;
  const std::size_t segs = closed ? n : n - 1;

  std::vector<double> h(segs);
  s.knots_.assign(segs + 1, 0.0);
  for (std::size_t i = 0; i < segs; ++i) {
    h[i] = std::pow(distance(control[i], control[(i + 1) % n]), alpha);
    s.knots_[i + 1] = s.knots_[i] + h[i];
  }
  std::vector<double> delta_x(segs), delta_y(segs);  // divided differences (P_{i+1} - P_i) / h_i
  for (std::size_t i = 0; i < segs; ++i) {
    const Vec2 d = control[(i + 1) % n] - control[i];
    delta_x[i] = d.x / h[i];
    delta_y[i] = d.y / h[i];
  }

  s.m_.assign(n, Vec2{0.0, 0.0});
  if (!closed) {
    if (n >= 3) {  // natural: M_0 = M_{n-1} = 0, interior unknowns M_1 .. M_{n-2}
      const std::size_t m = n - 2;
      std::vector<double> a(m), b(m), c(m), rx(m), ry(m);
      for (std::size_t k = 0; k < m; ++k) {
        const std::size_t i = k + 1;
        a[k] = h[i - 1];
        b[k] = 2.0 * (h[i - 1] + h[i]);
        c[k] = h[i];
        rx[k] = 6.0 * (delta_x[i] - delta_x[i - 1]);
        ry[k] = 6.0 * (delta_y[i] - delta_y[i - 1]);
      }
      const auto mx = solveTridiagonal(a, b, c, rx);
      const auto my = solveTridiagonal(a, b, c, ry);
      for (std::size_t k = 0; k < m; ++k) s.m_[k + 1] = {mx[k], my[k]};
    }  // n == 2: a straight line, all second derivatives zero
  } else {
    std::vector<double> a(n), b(n), c(n), rx(n), ry(n);
    for (std::size_t i = 0; i < n; ++i) {
      const std::size_t prev = (i + n - 1) % n;  // segment ending at knot i
      a[i] = h[prev];
      b[i] = 2.0 * (h[prev] + h[i]);
      c[i] = h[i];
      rx[i] = 6.0 * (delta_x[i] - delta_x[prev]);
      ry[i] = 6.0 * (delta_y[i] - delta_y[prev]);
    }
    // Corner entries: row 0 couples to M_{n-1} with h_{n-1}; row n-1 couples to M_0 with h_{n-1}.
    const auto mx = solveCyclic(a, b, c, c[n - 1], a[0], rx);
    const auto my = solveCyclic(a, b, c, c[n - 1], a[0], ry);
    for (std::size_t i = 0; i < n; ++i) s.m_[i] = {mx[i], my[i]};
  }

  for (std::size_t i = 0; i <= segs; ++i) {
    const Sample k = s.evaluate(i == segs ? segs - 1 : i, i == segs ? 1.0 : 0.0);
    s.speedScale_ = std::max(s.speedScale_, norm(k.d1));
  }
  return s;
}

CubicSpline::Sample CubicSpline::evaluate(std::size_t segment, double u) const {
  const std::size_t n = points_.size();
  const std::size_t i = segment, j = (segment + 1) % n;
  const double h = knots_[i + 1] - knots_[i];
  const double a = 1.0 - u, b = u;
  const Vec2 y0 = points_[i], y1 = points_[j], m0 = m_[i], m1 = m_[j];
  Sample s;
  // Position: a*y0 + b*y1 + ((a^3 - a) M0 + (b^3 - b) M1) h^2 / 6. At u = 0 and u = 1 the cubic terms are
  // exactly 0.
  const double ca = (a * a * a - a) * h * h / 6.0, cb = (b * b * b - b) * h * h / 6.0;
  s.position = a * y0 + b * y1 + ca * m0 + cb * m1;
  if (u == 0.0) s.position = y0;
  if (u == 1.0) s.position = y1;
  // First derivative: (y1 - y0)/h - (3 a^2 - 1) h M0 / 6 + (3 b^2 - 1) h M1 / 6.
  s.d1 = (y1 - y0) * (1.0 / h) - ((3.0 * a * a - 1.0) * h / 6.0) * m0 + ((3.0 * b * b - 1.0) * h / 6.0) * m1;
  s.d2 = a * m0 + b * m1;
  return s;
}

CubicSpline::Sample CubicSpline::evaluateAt(double t) const {
  const std::size_t segs = segmentCount();
  const double total = parameterLength();
  if (!std::isfinite(t)) throw GeometryError("spline parameter must be finite");
  if (closure_ == Closure::Closed) {
    t = std::fmod(t, total);
    if (t < 0.0) t += total;
  } else {
    t = std::min(std::max(t, 0.0), total);
  }
  // Largest segment i with knots_[i] <= t.
  std::size_t lo = 0, hi = segs;
  while (hi - lo > 1) {
    const std::size_t mid = (lo + hi) / 2;
    if (knots_[mid] <= t) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  const double h = knots_[lo + 1] - knots_[lo];
  return evaluate(lo, std::min(1.0, std::max(0.0, (t - knots_[lo]) / h)));
}

CurvatureValue CubicSpline::curvature(std::size_t segment, double u) const {
  const Sample s = evaluate(segment, u);
  return curvatureFromDerivatives(s.d1, s.d2, speedScale_);
}

namespace {
void checkOptions(const SplineOptions& o) {
  if (o.samplesPerSegment < 1 || o.samplesPerSegment > kMaxSamplesPerSegment) {
    throw GeometryError("samplesPerSegment must be in [1, 100000]");
  }
}
}  // namespace

Path cubicSpline(const std::vector<Vec2>& control, Closure closure, SplineOptions options) {
  checkOptions(options);
  const CubicSpline spline = CubicSpline::create(control, closure, options.alpha);
  const std::size_t s = options.samplesPerSegment;
  std::vector<Vec2> out;
  out.reserve(spline.segmentCount() * s + 1);
  for (std::size_t seg = 0; seg < spline.segmentCount(); ++seg) {
    out.push_back(control[seg]);  // u = 0: the control point itself
    for (std::size_t k = 1; k < s; ++k) {
      out.push_back(spline.evaluate(seg, static_cast<double>(k) / static_cast<double>(s)).position);
    }
  }
  if (closure == Closure::Open) out.push_back(control.back());
  return Path::create(std::move(out), closure);
}

std::vector<double> cubicSplineCurvature(const std::vector<Vec2>& control, Closure closure,
                                         SplineOptions options) {
  checkOptions(options);
  const CubicSpline spline = CubicSpline::create(control, closure, options.alpha);
  const std::size_t s = options.samplesPerSegment;
  std::vector<double> out;
  out.reserve(spline.segmentCount() * s + 1);
  for (std::size_t seg = 0; seg < spline.segmentCount(); ++seg) {
    for (std::size_t k = 0; k < s; ++k) {
      out.push_back(spline.curvature(seg, static_cast<double>(k) / static_cast<double>(s)).value);
    }
  }
  if (closure == Closure::Open) out.push_back(spline.curvature(spline.segmentCount() - 1, 1.0).value);
  return out;
}

}  // namespace pathforge::geometry
