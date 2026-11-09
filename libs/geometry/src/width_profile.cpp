#include <cmath>
#include <pathforge/geometry/width_profile.hpp>

namespace pathforge::geometry {

std::vector<double> sampleWidthProfile(const Path& path, const std::vector<WidthKnot>& knots) {
  if (knots.empty()) throw GeometryError("width profile needs at least one knot");
  const double length = path.length();
  const bool closed = path.isClosed();
  for (std::size_t i = 0; i < knots.size(); ++i) {
    const WidthKnot& k = knots[i];
    if (!std::isfinite(k.s) || !std::isfinite(k.width)) throw GeometryError("width knots must be finite");
    if (k.width < 0.0) throw GeometryError("width knots must be >= 0");
    if (k.s < 0.0 || k.s > length || (closed && k.s >= length)) {
      throw GeometryError("width knot arc length out of range for this path");
    }
    if (i > 0 && !(k.s > knots[i - 1].s)) throw GeometryError("width knot positions must strictly increase");
  }

  const auto& arc = path.arcLengths();
  std::vector<double> out(arc.size());
  const std::size_t m = knots.size();
  for (std::size_t v = 0; v < arc.size(); ++v) {
    const double s = arc[v];
    if (m == 1) {
      out[v] = knots[0].width;
    } else if (s <= knots.front().s && !closed) {
      out[v] = knots.front().width;
    } else if (s >= knots.back().s && !closed) {
      out[v] = knots.back().width;
    } else {
      // Locate the knot interval [k0, k1] containing s; closed paths add a wrap interval.
      double s0, w0, s1, w1;
      if (closed && (s < knots.front().s || s >= knots.back().s)) {
        s0 = knots.back().s;
        w0 = knots.back().width;
        s1 = knots.front().s + length;
        w1 = knots.front().width;
        const double sv = s < knots.front().s ? s + length : s;
        out[v] = w0 + (w1 - w0) * (sv - s0) / (s1 - s0);
        continue;
      }
      std::size_t k = 0;
      while (k + 2 < m && s >= knots[k + 1].s) ++k;
      s0 = knots[k].s;
      w0 = knots[k].width;
      s1 = knots[k + 1].s;
      w1 = knots[k + 1].width;
      out[v] = w0 + (w1 - w0) * (s - s0) / (s1 - s0);
    }
  }
  return out;
}

}  // namespace pathforge::geometry
