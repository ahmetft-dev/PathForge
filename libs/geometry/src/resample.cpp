#include <algorithm>
#include <cmath>
#include <pathforge/geometry/resample.hpp>
#include <vector>

namespace pathforge::geometry {

ResampleResult resampleByArcLength(const Path& path, double spacing) {
  if (!std::isfinite(spacing) || spacing <= 0.0) {
    throw GeometryError("resample spacing must be finite and > 0");
  }
  const double length = path.length();
  const double ratio = std::round(length / spacing);
  if (ratio > 1e8) throw GeometryError("resample spacing too small: sample count would exceed 1e8");

  const bool closed = path.isClosed();
  const std::size_t intervals = closed ? std::max<std::size_t>(3, static_cast<std::size_t>(ratio))
                                       : std::max<std::size_t>(1, static_cast<std::size_t>(ratio));
  const std::size_t count = closed ? intervals : intervals + 1;
  const double h = length / static_cast<double>(intervals);

  const auto& p = path.points();
  const std::size_t segs = path.segmentCount();
  std::vector<double> cum(segs + 1, 0.0);  // cumulative length at segment starts; cum[segs] == L
  for (std::size_t j = 0; j < segs; ++j) cum[j + 1] = cum[j] + distance(p[j], p[(j + 1) % p.size()]);

  std::vector<Vec2> out;
  out.reserve(count);
  std::size_t j = 0;
  for (std::size_t k = 0; k < count; ++k) {
    if (!closed && k == count - 1) {  // exact endpoint, avoids rounding at the end
      out.push_back(p.back());
      break;
    }
    const double s = static_cast<double>(k) * h;
    while (j + 1 < segs && cum[j + 1] < s) ++j;
    const Vec2 a = p[j];
    const Vec2 b = p[(j + 1) % p.size()];
    const double t = std::clamp((s - cum[j]) / (cum[j + 1] - cum[j]), 0.0, 1.0);
    out.push_back(a + t * (b - a));
  }
  return {Path::create(std::move(out), path.closure()), h};
}

}  // namespace pathforge::geometry
