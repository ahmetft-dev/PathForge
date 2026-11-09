#include "EditorSession.h"

#include <QUndoCommand>
#include <chrono>

namespace pfe = pathforge::editor;
namespace pg = pathforge::geometry;

// --------------------------------------------------------------------------------------------- undo commands
//
// Commands are thin wrappers around TrackDocument operations (which are Qt-free and independently tested).
// Points are restored under their ORIGINAL ids, so selection and later commands in the history stay valid.

struct UndoAccess {
  template <class Fn>
  static bool mutate(EditorSession* s, Fn&& fn) {
    return s->mutate(std::forward<Fn>(fn));
  }
  static void select(EditorSession* s, std::optional<pfe::PointId> id) { s->select(id); }
};

namespace {

enum CommandId { kWidthsCommand = 1, kNameCommand = 2 };

class AddPointCommand : public QUndoCommand {
 public:
  AddPointCommand(EditorSession* s, std::size_t index, pg::Vec2 position, double left, double right)
      : QUndoCommand("Add point"), s_(s), index_(index), position_(position), left_(left), right_(right) {}
  void redo() override {
    UndoAccess::mutate(s_, [&](pfe::TrackDocument& d) {
      if (id_ == 0) {
        id_ = d.insertPoint(index_, position_, left_, right_);
      } else {
        d.restorePoint(index_, {id_, position_, left_, right_});
      }
    });
    UndoAccess::select(s_, id_);
  }
  void undo() override {
    UndoAccess::mutate(s_, [&](pfe::TrackDocument& d) { d.removePoint(id_); });
  }
  pfe::PointId pointId() const { return id_; }

 private:
  EditorSession* s_;
  std::size_t index_;
  pg::Vec2 position_;
  double left_, right_;
  pfe::PointId id_ = 0;
};

class RemovePointCommand : public QUndoCommand {
 public:
  RemovePointCommand(EditorSession* s, pfe::PointId id) : QUndoCommand("Delete point"), s_(s), id_(id) {}
  void redo() override {
    UndoAccess::mutate(s_, [&](pfe::TrackDocument& d) {
      index_ = *d.indexOf(id_);
      point_ = d.removePoint(id_);
    });
  }
  void undo() override {
    UndoAccess::mutate(s_, [&](pfe::TrackDocument& d) { d.restorePoint(index_, point_); });
    UndoAccess::select(s_, id_);
  }

 private:
  EditorSession* s_;
  pfe::PointId id_;
  std::size_t index_ = 0;
  pfe::ControlPoint point_;
};

// One completed drag. The live movement during the drag was applied directly (no history); this command is
// pushed once at the end and its first redo() only records that state, because the document already holds the
// final position.
class MovePointCommand : public QUndoCommand {
 public:
  MovePointCommand(EditorSession* s, pfe::PointId id, pg::Vec2 from, pg::Vec2 to)
      : QUndoCommand("Move point"), s_(s), id_(id), from_(from), to_(to) {}
  void redo() override {
    if (first_) {
      first_ = false;
      return;
    }
    UndoAccess::mutate(s_, [&](pfe::TrackDocument& d) { d.movePoint(id_, to_); });
    UndoAccess::select(s_, id_);
  }
  void undo() override {
    UndoAccess::mutate(s_, [&](pfe::TrackDocument& d) { d.movePoint(id_, from_); });
    UndoAccess::select(s_, id_);
  }

 private:
  EditorSession* s_;
  pfe::PointId id_;
  pg::Vec2 from_, to_;
  bool first_ = true;
};

class SetWidthsCommand : public QUndoCommand {
 public:
  SetWidthsCommand(EditorSession* s, pfe::PointId id, double l0, double r0, double l1, double r1)
      : QUndoCommand("Change width"), s_(s), id_(id), l0_(l0), r0_(r0), l1_(l1), r1_(r1) {}
  int id() const override { return kWidthsCommand; }
  bool mergeWith(const QUndoCommand* other) override {  // consecutive edits of the same point form one step
    const auto* o = static_cast<const SetWidthsCommand*>(other);
    if (o->id_ != id_) return false;
    l1_ = o->l1_;
    r1_ = o->r1_;
    return true;
  }
  void redo() override {
    UndoAccess::mutate(s_, [&](pfe::TrackDocument& d) { d.setPointWidths(id_, l1_, r1_); });
  }
  void undo() override {
    UndoAccess::mutate(s_, [&](pfe::TrackDocument& d) { d.setPointWidths(id_, l0_, r0_); });
    UndoAccess::select(s_, id_);
  }

 private:
  EditorSession* s_;
  pfe::PointId id_;
  double l0_, r0_, l1_, r1_;
};

// Sets the widths of many points in ONE document mutation (one regeneration, one validation), as one undo
// step.
class SetAllWidthsCommand : public QUndoCommand {
 public:
  struct Old {
    pfe::PointId id;
    double left, right;
  };
  SetAllWidthsCommand(EditorSession* s, std::vector<Old> old, double left, double right)
      : QUndoCommand("Set all widths"), s_(s), old_(std::move(old)), l_(left), r_(right) {}
  void redo() override {
    UndoAccess::mutate(s_, [&](pfe::TrackDocument& d) {
      for (const Old& o : old_) d.setPointWidths(o.id, l_, r_);
    });
  }
  void undo() override {
    UndoAccess::mutate(s_, [&](pfe::TrackDocument& d) {
      for (const Old& o : old_) d.setPointWidths(o.id, o.left, o.right);
    });
  }

 private:
  EditorSession* s_;
  std::vector<Old> old_;
  double l_, r_;
};

class SetClosureCommand : public QUndoCommand {
 public:
  SetClosureCommand(EditorSession* s, pfe::Closure from, pfe::Closure to)
      : QUndoCommand(to == pfe::Closure::Closed ? "Close track" : "Open track"),
        s_(s),
        from_(from),
        to_(to) {}
  void redo() override {
    UndoAccess::mutate(s_, [&](pfe::TrackDocument& d) { d.setClosure(to_); });
  }
  void undo() override {
    UndoAccess::mutate(s_, [&](pfe::TrackDocument& d) { d.setClosure(from_); });
  }

 private:
  EditorSession* s_;
  pfe::Closure from_, to_;
};

class SetSettingsCommand : public QUndoCommand {
 public:
  SetSettingsCommand(EditorSession* s, pfe::GeometrySettings from, pfe::GeometrySettings to)
      : QUndoCommand("Change geometry settings"), s_(s), from_(from), to_(to) {}
  void redo() override {
    UndoAccess::mutate(s_, [&](pfe::TrackDocument& d) { d.setSettings(to_); });
  }
  void undo() override {
    UndoAccess::mutate(s_, [&](pfe::TrackDocument& d) { d.setSettings(from_); });
  }

 private:
  EditorSession* s_;
  pfe::GeometrySettings from_, to_;
};

class SetNameCommand : public QUndoCommand {
 public:
  SetNameCommand(EditorSession* s, std::string from, std::string to)
      : QUndoCommand("Rename track"), s_(s), from_(std::move(from)), to_(std::move(to)) {}
  int id() const override { return kNameCommand; }
  bool mergeWith(const QUndoCommand* other) override {
    to_ = static_cast<const SetNameCommand*>(other)->to_;
    return true;
  }
  void redo() override {
    UndoAccess::mutate(s_, [&](pfe::TrackDocument& d) { d.setName(to_); });
  }
  void undo() override {
    UndoAccess::mutate(s_, [&](pfe::TrackDocument& d) { d.setName(from_); });
  }

 private:
  EditorSession* s_;
  std::string from_, to_;
};

// Validates an edit on a copy first, so that rejected input never reaches the history.
template <class Fn>
bool dryRun(const pfe::TrackDocument& doc, Fn&& fn, QString* why) {
  pfe::TrackDocument copy = doc;
  try {
    fn(copy);
    return true;
  } catch (const pfe::EditError& e) {
    *why = QString::fromUtf8(e.what());
    return false;
  }
}

}  // namespace

// ----------------------------------------------------------------------------------------------------
// session

EditorSession::EditorSession(QObject* parent) : QObject(parent) {
  connect(&undo_, &QUndoStack::cleanChanged, this, [this](bool clean) { emit modifiedChanged(!clean); });
  debounce_.setSingleShot(true);
  connect(&debounce_, &QTimer::timeout, this, [this] {
    if (!drag_ && !isReportCurrent()) requestAsync();
  });
  connect(&runner_, &ValidationRunner::finished, this, &EditorSession::onRunnerFinished);
  regenerate(Mode::Immediate);
}

void EditorSession::setDefaultWidths(double left, double right) {
  pfe::TrackDocument::checkWidth(left);
  pfe::TrackDocument::checkWidth(right);
  defaultLeft_ = left;
  defaultRight_ = right;
}

template <class Fn>
bool EditorSession::mutate(Fn&& fn) {
  try {
    const auto before = doc_.revision();
    fn(doc_);
    if (doc_.revision() == before) return true;  // no-op edit
  } catch (const pfe::EditError& e) {
    emit editRejected(QString::fromUtf8(e.what()));
    return false;
  }
  regenerate(drag_ ? Mode::Skip : Mode::Auto);
  return true;
}

// Regenerates derived geometry and schedules validation.
//  Skip      during drags: no validation, the report becomes stale (cheap local diagnostics live in the
//  Track). Auto      after an edit: small tracks validate synchronously BEFORE the change is announced, so
//  listeners see
//            geometry and its matching report together; large tracks go to the worker after a debounce.
//  Immediate like Auto but without the debounce (new document, end of a drag, explicit request).
void EditorSession::regenerate(Mode mode) {
  derived_ = pfe::generateGeometry(doc_);
  ++generation_;  // every previously requested background result is now obsolete
  if (selection_ && !doc_.find(*selection_)) {
    selection_.reset();
    emit selectionChanged();
  }
  const bool sync = mode != Mode::Skip && derived_.data.centerline.size() <= asyncThreshold_;
  if (sync) {
    runner_.cancelPending();
    debounce_.stop();
    validateNow();  // before announcing the change, so listeners see geometry and report together
  }
  emit geometryChanged();
  if (sync || !isReportCurrent()) emit validationChanged();
  if (mode != Mode::Skip && !sync) {
    if (mode == Mode::Immediate) {
      debounce_.stop();
      requestAsync();
    } else {
      debounce_.start(debounceMs_);
    }
  }
}

void EditorSession::validateNow() {
  const auto t0 = std::chrono::steady_clock::now();
  report_ = pg::validateTrackData(derived_.data);
  lastValidationMs_ =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  reportGeneration_ = generation_;
  hasReport_ = true;
}

void EditorSession::requestAsync() { runner_.request(derived_.data, generation_); }

void EditorSession::onRunnerFinished(quint64 generation, double milliseconds) {
  // Accept only the result that belongs to the geometry on screen NOW. Anything else (an edit happened
  // meanwhile, a drag is in progress, the document was replaced) is obsolete and must never be shown as
  // current.
  if (generation != generation_ || drag_) {
    ++discarded_;
    return;
  }
  report_ = runner_.lastReport();
  lastValidationMs_ = milliseconds;
  reportGeneration_ = generation;
  hasReport_ = true;
  emit validationChanged();
}

void EditorSession::requestValidation() {
  if (isReportCurrent() || drag_) return;
  debounce_.stop();
  if (derived_.data.centerline.size() <= asyncThreshold_) {
    validateNow();
    emit validationChanged();
  } else {
    requestAsync();
  }
}

const pg::ValidationReport& EditorSession::validatedReport() {
  if (!isReportCurrent()) {
    runner_.cancelPending();
    debounce_.stop();
    validateNow();
    emit validationChanged();
  }
  return report_;
}

void EditorSession::replaceDocument(pfe::TrackDocument document, const QString& filePath) {
  drag_.reset();
  undo_.clear();  // history of the previous document is meaningless for the new one
  doc_ = std::move(document);
  filePath_ = filePath;
  selection_.reset();
  hasReport_ = false;
  runner_.cancelPending();
  emit selectionChanged();
  regenerate(Mode::Immediate);
  undo_.setClean();
  emit documentReplaced();
}

std::optional<pfe::PointId> EditorSession::appendPoint(pg::Vec2 p) {
  QString why;
  if (!dryRun(doc_, [&](pfe::TrackDocument& d) { d.appendPoint(p, defaultLeft_, defaultRight_); }, &why)) {
    emit editRejected(why);
    return std::nullopt;
  }
  auto* cmd = new AddPointCommand(this, doc_.size(), p, defaultLeft_, defaultRight_);
  undo_.push(cmd);  // redo() inserts the point and selects it
  return cmd->pointId() != 0 ? std::optional(cmd->pointId()) : std::nullopt;
}

std::optional<pfe::PointId> EditorSession::insertNear(pg::Vec2 p, double maxDistance) {
  const auto target = pfe::findInsertion(doc_, derived_, p);
  if (!target || target->distance > maxDistance) return std::nullopt;
  // New points take the widths the track has at that position, so the boundary does not jump.
  const double left = target->leftWidth, right = target->rightWidth;
  QString why;
  if (!dryRun(
          doc_, [&](pfe::TrackDocument& d) { d.insertPoint(target->index, target->position, left, right); },
          &why)) {
    emit editRejected(why);
    return std::nullopt;
  }
  auto* cmd = new AddPointCommand(this, target->index, target->position, left, right);
  undo_.push(cmd);
  return cmd->pointId();
}

bool EditorSession::removePoint(pfe::PointId id) {
  if (!doc_.find(id)) {
    emit editRejected(QString("unknown control point id %1").arg(id));
    return false;
  }
  undo_.push(new RemovePointCommand(this, id));
  return true;
}

bool EditorSession::setPointPosition(pfe::PointId id, pg::Vec2 p) {
  const pfe::ControlPoint* cp = doc_.find(id);
  QString why;
  if (!cp || !dryRun(doc_, [&](pfe::TrackDocument& d) { d.movePoint(id, p); }, &why)) {
    emit editRejected(cp ? why : QString("unknown control point id %1").arg(id));
    return false;
  }
  if (cp->position == p) return true;
  const pg::Vec2 from = cp->position;
  mutate([&](pfe::TrackDocument& d) { d.movePoint(id, p); });
  undo_.push(new MovePointCommand(this, id, from, p));
  return true;
}

bool EditorSession::setPointWidths(pfe::PointId id, double left, double right) {
  const pfe::ControlPoint* cp = doc_.find(id);
  QString why;
  if (!cp || !dryRun(doc_, [&](pfe::TrackDocument& d) { d.setPointWidths(id, left, right); }, &why)) {
    emit editRejected(cp ? why : QString("unknown control point id %1").arg(id));
    return false;
  }
  if (cp->leftWidth == left && cp->rightWidth == right) return true;
  undo_.push(new SetWidthsCommand(this, id, cp->leftWidth, cp->rightWidth, left, right));
  return true;
}

bool EditorSession::setAllWidths(double left, double right) {
  QString why;
  if (!dryRun(doc_, [&](pfe::TrackDocument& d) { d.checkWidth(left), d.checkWidth(right); }, &why)) {
    emit editRejected(why);
    return false;
  }
  bool any = false;
  for (const pfe::ControlPoint& p : doc_.points()) any = any || p.leftWidth != left || p.rightWidth != right;
  if (!any) return true;
  std::vector<SetAllWidthsCommand::Old> old;
  old.reserve(doc_.size());
  for (const pfe::ControlPoint& p : doc_.points()) old.push_back({p.id, p.leftWidth, p.rightWidth});
  undo_.push(new SetAllWidthsCommand(this, std::move(old), left, right));
  return true;
}

bool EditorSession::setClosure(pfe::Closure c) {
  if (c == doc_.closure()) return true;
  undo_.push(new SetClosureCommand(this, doc_.closure(), c));
  return true;
}

bool EditorSession::setSettings(const pfe::GeometrySettings& s) {
  QString why;
  if (!dryRun(doc_, [&](pfe::TrackDocument& d) { d.setSettings(s); }, &why)) {
    emit editRejected(why);
    return false;
  }
  if (s == doc_.settings()) return true;
  undo_.push(new SetSettingsCommand(this, doc_.settings(), s));
  return true;
}

bool EditorSession::setName(const QString& name) {
  const std::string to = name.toStdString();
  if (to == doc_.name()) return true;
  undo_.push(new SetNameCommand(this, doc_.name(), to));
  return true;
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
  mutate(
      [&](pfe::TrackDocument& d) { d.movePoint(drag_->id, p); });  // live preview: no history, no validation
}

void EditorSession::endDrag() {
  if (!drag_) return;
  const Drag drag = *drag_;
  drag_.reset();
  const pfe::ControlPoint* cp = doc_.find(drag.id);
  if (cp && cp->position != drag.start) {
    undo_.push(
        new MovePointCommand(this, drag.id, drag.start, cp->position));  // ONE command for the whole drag
  }
  requestValidation();  // the one validation of the whole drag (sync for small tracks, worker for large ones)
}
