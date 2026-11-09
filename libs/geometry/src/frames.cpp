#include <cmath>
#include <pathforge/geometry/frames.hpp>

namespace pathforge::geometry {

PathFrames computeFrames(const Path& path) {
  const auto& p = path.points();
  const std::size_t n = p.size();
  const bool closed = path.isClosed();

  PathFrames f;
  f.tangent.resize(n);
  f.normal.resize(n);
  f.arcLength = path.arcLengths();
  f.curvature = path.curvature();

  auto unit = [](Vec2 d) { return d * (1.0 / norm(d)); };  // segments are validated non-zero
  for (std::size_t i = 0; i < n; ++i) {
    Vec2 t;
    if (!closed && i == 0) {
      t = unit(p[1] - p[0]);
    } else if (!closed && i == n - 1) {
      t = unit(p[n - 1] - p[n - 2]);
    } else {
      const Vec2 dPrev = p[i] - p[(i + n - 1) % n];
      const Vec2 dNext = p[(i + 1) % n] - p[i];
      const double lp = norm(dPrev);
      const double ln = norm(dNext);
      const Vec2 sum = dPrev * (ln / lp) + dNext * (lp / ln);
      const double m = norm(sum);
      t = m > 1e-12 * (lp + ln) ? sum * (1.0 / m) : dNext * (1.0 / ln);
    }
    f.tangent[i] = t;
    f.normal[i] = {-t.y, t.x};
  }
  return f;
}

std::size_t countSignChanges(const std::vector<double>& curvature, double threshold) {
  std::size_t changes = 0;
  int last = 0;
  for (double k : curvature) {
    if (std::abs(k) <= threshold) continue;
    const int s = k > 0.0 ? 1 : -1;
    if (last != 0 && s != last) ++changes;
    last = s;
  }
  return changes;
}

}  // namespace pathforge::geometry
