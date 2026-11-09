#include <cmath>
#include <pathforge/geometry/width_profile.hpp>

namespace pathforge::geometry {
namespace {

void validateKnots(const std::vector<WidthKnot>& knots, double length, bool closed) {
  if (knots.empty()) throw GeometryError("width profile needs at least one knot");
  for (std::size_t i = 0; i < knots.size(); ++i) {
    const WidthKnot& k = knots[i];
    if (!std::isfinite(k.s) || !std::isfinite(k.width)) throw GeometryError("width knots must be finite");
    if (k.width < 0.0) throw GeometryError("width knots must be >= 0");
    if (k.s < 0.0 || k.s > length || (closed && k.s >= length)) {
      throw GeometryError("width knot arc length out of range for this path");
    }
    if (i > 0 && !(k.s > knots[i - 1].s)) throw GeometryError("width knot positions must strictly increase");
  }
}

}  // namespace

std::vector<double> evaluateWidthProfile(const std::vector<WidthKnot>& knots, double length, Closure closure,
                                         const std::vector<double>& positions) {
  const bool closed = closure == Closure::Closed;
  if (!std::isfinite(length) || length <= 0.0) throw GeometryError("width profile path length must be > 0");
  validateKnots(knots, length, closed);

  std::vector<double> out(positions.size());
  const std::size_t m = knots.size();
  std::size_t k = 0;  // current knot interval; only moves forward because positions are non-decreasing
  double previous = 0.0;
  for (std::size_t v = 0; v < positions.size(); ++v) {
    const double s = positions[v];
    if (!std::isfinite(s) || s < 0.0 || s > length || (closed && s >= length) || (v > 0 && s < previous)) {
      throw GeometryError("width profile positions must be finite, non-decreasing and on the path");
    }
    previous = s;
    if (m == 1) {
      out[v] = knots[0].width;
    } else if (s <= knots.front().s && !closed) {
      out[v] = knots.front().width;
    } else if (s >= knots.back().s && !closed) {
      out[v] = knots.back().width;
    } else if (closed && (s < knots.front().s || s >= knots.back().s)) {
      // Wrap interval through the seam: from the last knot to the first knot + length.
      const double s0 = knots.back().s, w0 = knots.back().width;
      const double s1 = knots.front().s + length, w1 = knots.front().width;
      const double sv = s < knots.front().s ? s + length : s;
      out[v] = w0 + (w1 - w0) * (sv - s0) / (s1 - s0);
    } else {
      while (k + 2 < m && s >= knots[k + 1].s) ++k;
      const double s0 = knots[k].s, w0 = knots[k].width;
      const double s1 = knots[k + 1].s, w1 = knots[k + 1].width;
      out[v] = w0 + (w1 - w0) * (s - s0) / (s1 - s0);
    }
  }
  return out;
}

std::vector<double> sampleWidthProfile(const Path& path, const std::vector<WidthKnot>& knots) {
  return evaluateWidthProfile(knots, path.length(), path.closure(), path.arcLengths());
}

}  // namespace pathforge::geometry
