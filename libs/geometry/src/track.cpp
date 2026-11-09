#include <algorithm>
#include <cmath>
#include <numeric>
#include <pathforge/geometry/track.hpp>

namespace pathforge::geometry {

Track Track::create(Path centerline, double leftWidth, double rightWidth) {
  for (double w : {leftWidth, rightWidth}) {
    if (!std::isfinite(w) || w < 0.0) throw GeometryError("track widths must be finite and >= 0");
  }
  const std::size_t n = centerline.size();
  return Track(std::move(centerline), std::vector<double>(n, leftWidth), std::vector<double>(n, rightWidth));
}

Track Track::create(Path centerline, std::vector<double> leftWidths, std::vector<double> rightWidths) {
  if (leftWidths.size() != centerline.size() || rightWidths.size() != centerline.size()) {
    throw GeometryError("width arrays must have one entry per centerline point");
  }
  for (const auto* v : {&leftWidths, &rightWidths}) {
    for (double w : *v) {
      if (!std::isfinite(w) || w < 0.0) throw GeometryError("track widths must be finite and >= 0");
    }
  }
  return Track(std::move(centerline), std::move(leftWidths), std::move(rightWidths));
}

Track::Track(Path centerline, std::vector<double> leftWidths, std::vector<double> rightWidths)
    : centerline_(std::move(centerline)),
      frames_(computeFrames(centerline_)),
      leftWidths_(std::move(leftWidths)),
      rightWidths_(std::move(rightWidths)) {
  const auto& c = centerline_.points();
  const std::size_t n = c.size();

  auto allEqual = [](const std::vector<double>& v) {
    return std::all_of(v.begin(), v.end(), [&](double x) { return x == v.front(); });
  };
  constant_ = allEqual(leftWidths_) && allEqual(rightWidths_);
  auto summary = [&](const std::vector<double>& v) {
    return constant_ ? v.front() : std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(n);
  };
  leftWidth_ = summary(leftWidths_);
  rightWidth_ = summary(rightWidths_);
  minTotal_ = maxTotal_ = leftWidths_[0] + rightWidths_[0];
  for (std::size_t i = 1; i < n; ++i) {
    minTotal_ = std::min(minTotal_, leftWidths_[i] + rightWidths_[i]);
    maxTotal_ = std::max(maxTotal_, leftWidths_[i] + rightWidths_[i]);
  }

  left_.resize(n);
  right_.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    left_[i] = c[i] + leftWidths_[i] * frames_.normal[i];
    right_[i] = c[i] - rightWidths_[i] * frames_.normal[i];
  }

  // Local diagnostics (counts only; validateTrack() reports indices and locations).
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
