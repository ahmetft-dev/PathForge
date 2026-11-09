#include <algorithm>
#include <cmath>
#include <pathforge/geometry/track.hpp>

namespace pathforge::geometry {

Track Track::create(Path centerline, double leftWidth, double rightWidth) {
  for (double w : {leftWidth, rightWidth}) {
    if (!std::isfinite(w) || w < 0.0) throw GeometryError("track widths must be finite and >= 0");
  }
  return Track(std::move(centerline), leftWidth, rightWidth);
}

Track::Track(Path centerline, double leftWidth, double rightWidth)
    : centerline_(std::move(centerline)),
      frames_(computeFrames(centerline_)),
      leftWidth_(leftWidth),
      rightWidth_(rightWidth) {
  const auto& c = centerline_.points();
  const std::size_t n = c.size();
  leftWidths_.assign(n, leftWidth);
  rightWidths_.assign(n, rightWidth);
  left_.resize(n);
  right_.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    left_[i] = c[i] + leftWidths_[i] * frames_.normal[i];
    right_[i] = c[i] - rightWidths_[i] * frames_.normal[i];
  }

  // Local diagnostics.
  const std::size_t segs = centerline_.segmentCount();
  for (std::size_t i = 0; i < segs; ++i) {
    const std::size_t j = (i + 1) % n;
    const Vec2 cs = c[j] - c[i];
    if (dot(left_[j] - left_[i], cs) <= 0.0) ++diagnostics_.leftFlippedSegments;
    if (dot(right_[j] - right_[i], cs) <= 0.0) ++diagnostics_.rightFlippedSegments;
  }
  for (std::size_t i = 0; i < n; ++i) {
    const double k = frames_.curvature[i];
    const double l = leftWidths_[i] * k;    // inner side of a left turn is the left boundary
    const double r = rightWidths_[i] * -k;  // inner side of a right turn is the right boundary
    diagnostics_.maxLeftWidthCurvature = std::max(diagnostics_.maxLeftWidthCurvature, l);
    diagnostics_.maxRightWidthCurvature = std::max(diagnostics_.maxRightWidthCurvature, r);
    if (l >= 1.0) ++diagnostics_.leftCurvatureViolations;
    if (r >= 1.0) ++diagnostics_.rightCurvatureViolations;
  }
}

}  // namespace pathforge::geometry
