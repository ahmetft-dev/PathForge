#include "EditorSession.h"

namespace pfe = pathforge::editor;
namespace pg = pathforge::geometry;

EditorSession::EditorSession(QObject* parent) : QObject(parent) {
  regenerate();
  validateNow();
}

void EditorSession::setDefaultWidths(double left, double right) {
  pfe::TrackDocument::checkWidth(left);
  pfe::TrackDocument::checkWidth(right);
  defaultLeft_ = left;
  defaultRight_ = right;
}

template <class Fn>
bool EditorSession::apply(Fn&& fn) {
  try {
    const auto before = doc_.revision();
    fn(doc_);
    if (doc_.revision() == before) return true;  // no-op edit
  } catch (const pfe::EditError& e) {
    emit editRejected(QString::fromUtf8(e.what()));
    return false;
  }
  regenerate();
  if (!drag_) validateNow();
  return true;
}

void EditorSession::regenerate() {
  derived_ = pfe::generateGeometry(doc_);
  if (selection_ && !doc_.find(*selection_)) {
    selection_.reset();
    emit selectionChanged();
  }
  emit geometryChanged();
  if (!isReportCurrent()) emit validationChanged();  // the shown report is now stale
}

void EditorSession::validateNow() {
  report_ = pg::validateTrackData(derived_.data);
  reportRevision_ = doc_.revision();
  hasReport_ = true;
  emit validationChanged();
}

void EditorSession::replaceDocument(pfe::TrackDocument document, const QString& filePath) {
  drag_.reset();
  doc_ = std::move(document);
  filePath_ = filePath;
  selection_.reset();
  hasReport_ = false;
  emit selectionChanged();
  regenerate();
  validateNow();
  emit documentReplaced();
}

std::optional<pfe::PointId> EditorSession::appendPoint(pg::Vec2 p) {
  pfe::PointId id = 0;
  if (!apply([&](pfe::TrackDocument& d) { id = d.appendPoint(p, defaultLeft_, defaultRight_); }))
    return std::nullopt;
  select(id);
  return id;
}

std::optional<pfe::PointId> EditorSession::insertNear(pg::Vec2 p, double maxDistance) {
  const auto target = pfe::findInsertion(doc_, derived_, p);
  if (!target || target->distance > maxDistance) return std::nullopt;
  // New points take the widths of the curve where they are inserted, so the boundary does not jump.
  double left = defaultLeft_, right = defaultRight_;
  if (!doc_.empty()) {
    const auto& n = doc_.points()[target->index > 0 ? target->index - 1 : 0];
    left = n.leftWidth;
    right = n.rightWidth;
  }
  pfe::PointId id = 0;
  if (!apply(
          [&](pfe::TrackDocument& d) { id = d.insertPoint(target->index, target->position, left, right); })) {
    return std::nullopt;
  }
  select(id);
  return id;
}

bool EditorSession::removePoint(pfe::PointId id) {
  return apply([&](pfe::TrackDocument& d) { d.removePoint(id); });
}

bool EditorSession::setPointPosition(pfe::PointId id, pg::Vec2 p) {
  return apply([&](pfe::TrackDocument& d) { d.movePoint(id, p); });
}

bool EditorSession::setPointWidths(pfe::PointId id, double left, double right) {
  return apply([&](pfe::TrackDocument& d) { d.setPointWidths(id, left, right); });
}

bool EditorSession::setClosure(pfe::Closure c) {
  return apply([&](pfe::TrackDocument& d) { d.setClosure(c); });
}

bool EditorSession::setSettings(const pfe::GeometrySettings& s) {
  return apply([&](pfe::TrackDocument& d) { d.setSettings(s); });
}

bool EditorSession::setName(const QString& name) {
  return apply([&](pfe::TrackDocument& d) { d.setName(name.toStdString()); });
}

void EditorSession::select(std::optional<pfe::PointId> id) {
  if (id && !doc_.find(*id)) id.reset();
  if (id == selection_) return;
  selection_ = id;
  emit selectionChanged();
}

bool EditorSession::beginDrag(pfe::PointId id) {
  const pfe::ControlPoint* p = doc_.find(id);
  if (!p) return false;
  drag_ = Drag{id, p->position};
  select(id);
  return true;
}

void EditorSession::dragTo(pg::Vec2 p) {
  if (!drag_) return;
  apply([&](pfe::TrackDocument& d) { d.movePoint(drag_->id, p); });
}

void EditorSession::endDrag() {
  if (!drag_) return;
  drag_.reset();
  if (!isReportCurrent()) validateNow();
}
