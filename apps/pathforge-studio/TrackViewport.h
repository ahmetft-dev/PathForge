#pragma once

#include <QGraphicsScene>
#include <QGraphicsView>
#include <optional>
#include <pathforge/editor/derived_geometry.hpp>
#include <pathforge/editor/track_document.hpp>
#include <pathforge/geometry/validation.hpp>
#include <vector>

class QGraphicsEllipseItem;

// 2D track viewport and editing surface. World coordinates are meters with y up; the scene stores (x, -y)
// because Qt's y axis points down. The viewport draws what it is given (derived geometry, control points,
// issues) and turns mouse input into requests; it never edits or computes geometry itself.
//
// Mouse:  left-press on a control point (within kHitRadiusPx) selects it and starts a drag after
// kDragThresholdPx;
//         left-drag on empty space pans; a left click on empty space deselects, or appends a point in add
//         mode; Shift+click inserts a point on the nearest track segment (a preview marker follows the
//         pointer while Shift is held); wheel zooms around the pointer.
class TrackViewport : public QGraphicsView {
  Q_OBJECT
 public:
  static constexpr double kHitRadiusPx = 8.0;
  static constexpr double kDragThresholdPx = 3.0;
  static constexpr double kInsertRadiusPx = 12.0;
  static constexpr double kHandleSpacingPx = 6.0;  // dense handles are thinned to this on-screen spacing

  explicit TrackViewport(QWidget* parent = nullptr);

  // Replaces the drawn content. `report` is drawn as overlays and must be nullptr when it does not belong to
  // the geometry being drawn (stale validation must never be shown as current).
  void setContent(const pathforge::editor::DerivedGeometry& derived,
                  const std::vector<pathforge::editor::ControlPoint>& points,
                  std::optional<pathforge::editor::PointId> selected,
                  const pathforge::geometry::ValidationReport* report);

  void setAddMode(bool on);
  bool addMode() const { return addMode_; }
  void showInsertPreview(std::optional<pathforge::geometry::Vec2> world);

  // Centers the view on a world position [m] without changing the zoom.
  void focusOn(double xMeters, double yMeters);
  void fitToView();
  void zoomBy(double factor);
  double pixelsPerMeter() const { return transform().m11(); }

  // Coordinate helpers (viewport pixels <-> world meters) and screen-space hit test; public for tests.
  QPointF worldToViewport(pathforge::geometry::Vec2 world) const;
  pathforge::geometry::Vec2 viewportToWorld(QPointF viewportPos) const;
  std::optional<pathforge::editor::PointId> pointAt(QPointF viewportPos) const;

 signals:
  // The zoom changed: the level of detail of the drawn polylines depends on it, so the owner should call
  // setContent() again with the same data.
  void scaleChanged();
  void cursorMoved(double xMeters, double yMeters);
  void pointPressed(pathforge::editor::PointId id);
  void pointDragged(double xMeters, double yMeters);
  void dragFinished();
  void emptyClicked();
  void addRequested(double xMeters, double yMeters);
  void insertRequested(double xMeters, double yMeters);
  void insertPreviewRequested(double xMeters, double yMeters);

 protected:
  void drawBackground(QPainter* painter, const QRectF& rect) override;
  void wheelEvent(QWheelEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void keyReleaseEvent(QKeyEvent* event) override;
  void resizeEvent(QResizeEvent* event) override;

 private:
  enum class Press { None, Point, Empty };
  void updateCursor(QPointF viewportPos, Qt::KeyboardModifiers modifiers);

  QGraphicsScene scene_;
  QRectF extentRect_;
  bool hasContent_ = false;
  bool userMoved_ = false;  // keep auto-fit on resize until the user pans/zooms
  bool addMode_ = false;
  std::vector<pathforge::editor::ControlPoint> points_;
  std::optional<pathforge::geometry::Vec2> preview_;
  QGraphicsEllipseItem* previewItem_ = nullptr;
  Press press_ = Press::None;
  QPointF pressPos_;
  bool dragging_ = false;
};
