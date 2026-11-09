#pragma once

#include <QObject>
#include <QString>
#include <cstdint>
#include <optional>
#include <pathforge/editor/derived_geometry.hpp>
#include <pathforge/editor/track_document.hpp>
#include <pathforge/geometry/validation.hpp>

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

  // Validation result and the revision it belongs to. isReportCurrent() is false while a newer revision has
  // not been validated yet (e.g. during a drag); callers must then treat the report as stale.
  const pathforge::geometry::ValidationReport& report() const { return report_; }
  bool isReportCurrent() const { return reportRevision_ == doc_.revision() && hasReport_; }

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
  void editRejected(const QString& reason);

 private:
  struct Drag {
    pathforge::editor::PointId id;
    pathforge::geometry::Vec2 start;
  };

  template <class Fn>
  bool apply(Fn&& fn);
  void regenerate();
  void validateNow();

  pathforge::editor::TrackDocument doc_;
  pathforge::editor::DerivedGeometry derived_;
  std::optional<pathforge::editor::PointId> selection_;
  std::optional<Drag> drag_;
  QString filePath_;
  pathforge::geometry::ValidationReport report_;
  std::uint64_t reportRevision_ = 0;
  bool hasReport_ = false;
  double defaultLeft_ = 5.0;
  double defaultRight_ = 5.0;
};
