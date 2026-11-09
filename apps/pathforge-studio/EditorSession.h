#pragma once

#include <QObject>
#include <QString>
#include <QTimer>
#include <QUndoStack>
#include <cstdint>
#include <optional>
#include <pathforge/editor/derived_geometry.hpp>
#include <pathforge/editor/track_document.hpp>
#include <pathforge/geometry/validation.hpp>

#include "ValidationRunner.h"

// Owns the editable document of one Studio window and everything derived from it.
//
// Ownership and lifecycle: the TrackDocument is the only authoritative state. After every change the session
// regenerates DerivedGeometry (synchronously, it is cheap; see docs/editor.md for timings) and emits
// geometryChanged(). Validation results are tagged with the document revision they were computed for; a
// result is "current" only while that revision is still the document's revision. The UI never edits geometry
// itself.
class EditorSession : public QObject {
  Q_OBJECT
 public:
  explicit EditorSession(QObject* parent = nullptr);

  const pathforge::editor::TrackDocument& document() const { return doc_; }
  const pathforge::editor::DerivedGeometry& derived() const { return derived_; }
  std::optional<pathforge::editor::PointId> selection() const { return selection_; }
  const QString& filePath() const { return filePath_; }

  // Editing history. Every edit (including each completed drag, and each width/closure/settings change) is
  // one command; replacing the document clears the history. isModified() is false right after load/save/new.
  QUndoStack* undoStack() { return &undo_; }
  bool isModified() const { return !undo_.isClean(); }
  void markSaved() { undo_.setClean(); }

  // Validation result and the revision it belongs to. isReportCurrent() is false while a newer revision has
  // not been validated yet (e.g. during a drag); callers must then treat the report as stale.
  const pathforge::geometry::ValidationReport& report() const { return report_; }
  // Returns a report that is guaranteed to belong to the current geometry, validating synchronously if
  // necessary. Use this before anything downstream (simulation, optimisation, export) consumes the track.
  const pathforge::geometry::ValidationReport& validatedReport();

  // ---- Validation scheduling. Tracks with at most asyncThreshold() samples validate synchronously (a few
  // ms); larger ones are validated on a worker thread after a debounce, and the report is "pending" (not
  // current) meanwhile.
  std::size_t asyncThreshold() const { return asyncThreshold_; }
  void setAsyncThreshold(std::size_t samples) { asyncThreshold_ = samples; }
  void setValidationDebounce(int milliseconds) { debounceMs_ = milliseconds; }
  // Starts validation of the current geometry now (no-op if the report is already current).
  void requestValidation();
  bool validationRunning() const { return runner_.busy() || debounce_.isActive(); }
  double lastValidationMilliseconds() const { return lastValidationMs_; }
  std::uint64_t discardedResults() const {
    return discarded_;
  }  // obsolete background results that were dropped
  ValidationRunner* runner() { return &runner_; }
  // Incremented every time derived geometry is regenerated; lets views skip redundant redraws.
  std::uint64_t generation() const { return generation_; }
  bool isReportCurrent() const { return hasReport_ && reportGeneration_ == generation_; }

  double defaultLeftWidth() const { return defaultLeft_; }
  double defaultRightWidth() const { return defaultRight_; }
  void setDefaultWidths(double left, double right);

  // Replaces the whole document (new / open / preset). Clears the selection.
  void replaceDocument(pathforge::editor::TrackDocument document, const QString& filePath);
  void setFilePath(const QString& path) { filePath_ = path; }

  // Edits. Each returns false / nullopt and emits editRejected() if the document refused the input.
  std::optional<pathforge::editor::PointId> appendPoint(pathforge::geometry::Vec2 position);
  // Inserts on the derived curve nearest to `position` if it is within `maxDistance` [m].
  std::optional<pathforge::editor::PointId> insertNear(pathforge::geometry::Vec2 position,
                                                       double maxDistance);
  bool removePoint(pathforge::editor::PointId id);
  bool setPointPosition(pathforge::editor::PointId id, pathforge::geometry::Vec2 position);
  bool setPointWidths(pathforge::editor::PointId id, double left, double right);
  // Sets every control point to the same widths (constant-width track) as ONE undoable step.
  bool setAllWidths(double left, double right);
  bool setClosure(pathforge::editor::Closure closure);
  bool setSettings(const pathforge::editor::GeometrySettings& settings);
  bool setName(const QString& name);

  void select(std::optional<pathforge::editor::PointId> id);

  // Drag session: the document is updated live (cheap regeneration, no validation) and validated once at the
  // end.
  bool beginDrag(pathforge::editor::PointId id);
  void dragTo(pathforge::geometry::Vec2 position);
  void endDrag();
  bool isDragging() const { return drag_.has_value(); }

 signals:
  void geometryChanged();  // derived geometry regenerated (also during drags)
  void selectionChanged();
  void validationChanged();  // report() changed or became stale
  void documentReplaced();
  void modifiedChanged(bool modified);
  void editRejected(const QString& reason);

 private:
  struct Drag {
    pathforge::editor::PointId id;
    pathforge::geometry::Vec2 start;
  };

  enum class Mode { Skip, Auto, Immediate };  // see regenerate()

  friend struct UndoAccess;  // the undo commands mutate the document through these private helpers

  // Raw mutation: applies `fn` to the document, regenerates derived geometry and (outside drags) validates.
  // Does NOT record history; commands call it from redo()/undo(). Returns false and emits editRejected() on
  // EditError.
  template <class Fn>
  bool mutate(Fn&& fn);
  void regenerate(Mode validation);
  void validateNow();
  void requestAsync();
  void onRunnerFinished(quint64 generation, double milliseconds);

  pathforge::editor::TrackDocument doc_;
  pathforge::editor::DerivedGeometry derived_;
  std::optional<pathforge::editor::PointId> selection_;
  std::optional<Drag> drag_;
  QString filePath_;
  QUndoStack undo_;
  pathforge::geometry::ValidationReport report_;
  std::uint64_t reportGeneration_ = 0;
  bool hasReport_ = false;
  std::uint64_t generation_ = 0;
  std::size_t asyncThreshold_ = 1500;
  int debounceMs_ = 150;
  double lastValidationMs_ = 0.0;
  std::uint64_t discarded_ = 0;
  QTimer debounce_;
  ValidationRunner runner_;  // declared last: destroyed first, so no job outlives the state it reports into
  double defaultLeft_ = 5.0;
  double defaultRight_ = 5.0;
};
