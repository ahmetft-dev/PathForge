#include <algorithm>
#include <cmath>
#include <pathforge/editor/track_document.hpp>

namespace pathforge::editor {

void TrackDocument::checkPosition(Vec2 p) {
  if (!geometry::isFinite(p)) throw EditError("control point coordinates must be finite");
  if (std::abs(p.x) > kMaxCoordinate || std::abs(p.y) > kMaxCoordinate) {
    throw EditError("control point coordinates must be within +/-1e7 m");
  }
}

void TrackDocument::checkWidth(double w) {
  if (!std::isfinite(w) || w < 0.0 || w > kMaxWidth)
    throw EditError("widths must be finite and in [0, 1e4] m");
}

void TrackDocument::checkSettings(const GeometrySettings& s) {
  if (!(s.alpha >= 0.0 && s.alpha <= 1.0)) throw EditError("Catmull-Rom alpha must be in [0, 1]");
  if (!std::isfinite(s.resampleSpacing) || s.resampleSpacing < 0.0 ||
      (s.resampleSpacing > 0.0 && s.resampleSpacing < 1e-3)) {
    throw EditError("resample spacing must be 0 (off) or a finite value >= 0.001 m");
  }
}

void TrackDocument::setName(std::string name) {
  if (name == name_) return;
  name_ = std::move(name);
  touch();
}

void TrackDocument::setClosure(Closure closure) {
  if (closure == closure_) return;
  closure_ = closure;
  touch();
}

void TrackDocument::setSettings(const GeometrySettings& settings) {
  checkSettings(settings);
  if (settings == settings_) return;
  settings_ = settings;
  touch();
}

std::optional<std::size_t> TrackDocument::indexOf(PointId id) const {
  const auto it =
      std::find_if(points_.begin(), points_.end(), [&](const ControlPoint& p) { return p.id == id; });
  if (it == points_.end()) return std::nullopt;
  return static_cast<std::size_t>(it - points_.begin());
}

const ControlPoint* TrackDocument::find(PointId id) const {
  const auto i = indexOf(id);
  return i ? &points_[*i] : nullptr;
}

std::size_t TrackDocument::indexOrThrow(PointId id) const {
  const auto i = indexOf(id);
  if (!i) throw EditError("unknown control point id " + std::to_string(id));
  return *i;
}

PointId TrackDocument::appendPoint(Vec2 position, double leftWidth, double rightWidth) {
  return insertPoint(points_.size(), position, leftWidth, rightWidth);
}

PointId TrackDocument::insertPoint(std::size_t index, Vec2 position, double leftWidth, double rightWidth) {
  if (index > points_.size()) throw EditError("insertion index out of range");
  checkPosition(position);
  checkWidth(leftWidth);
  checkWidth(rightWidth);
  const PointId id = nextId_++;
  points_.insert(points_.begin() + static_cast<std::ptrdiff_t>(index),
                 ControlPoint{id, position, leftWidth, rightWidth});
  touch();
  return id;
}

void TrackDocument::restorePoint(std::size_t index, const ControlPoint& p) {
  if (index > points_.size()) throw EditError("insertion index out of range");
  if (p.id == 0 || p.id >= nextId_ || indexOf(p.id)) throw EditError("cannot restore a point with this id");
  checkPosition(p.position);
  checkWidth(p.leftWidth);
  checkWidth(p.rightWidth);
  points_.insert(points_.begin() + static_cast<std::ptrdiff_t>(index), p);
  touch();
}

void TrackDocument::movePoint(PointId id, Vec2 position) {
  const std::size_t i = indexOrThrow(id);
  checkPosition(position);
  if (points_[i].position == position) return;
  points_[i].position = position;
  touch();
}

void TrackDocument::setPointWidths(PointId id, double leftWidth, double rightWidth) {
  const std::size_t i = indexOrThrow(id);
  checkWidth(leftWidth);
  checkWidth(rightWidth);
  if (points_[i].leftWidth == leftWidth && points_[i].rightWidth == rightWidth) return;
  points_[i].leftWidth = leftWidth;
  points_[i].rightWidth = rightWidth;
  touch();
}

ControlPoint TrackDocument::removePoint(PointId id) {
  const std::size_t i = indexOrThrow(id);
  const ControlPoint removed = points_[i];
  points_.erase(points_.begin() + static_cast<std::ptrdiff_t>(i));
  touch();
  return removed;
}

}  // namespace pathforge::editor
