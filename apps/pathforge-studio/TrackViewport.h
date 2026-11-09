#pragma once

#include <QGraphicsPathItem>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <pathforge/geometry/track.hpp>
#include <pathforge/geometry/validation.hpp>

// 2D track viewport. World coordinates are meters with y up; the scene stores
// (x, -y) because Qt's y axis points down. No geometry is computed here: the view
// only draws points and boundaries that come from pathforge::geometry.
class TrackViewport : public QGraphicsView {
  Q_OBJECT
 public:
  explicit TrackViewport(QWidget* parent = nullptr);

  // Draws centerline and both boundaries. Open centerlines are drawn open (no closing segment).
  // Issues from `report` are overlaid: affected segments and issue locations (errors red, warnings amber).
  void setTrack(const pathforge::geometry::Track& track, const pathforge::geometry::ValidationReport& report);
  // Centers the view on a world position [m] without changing the zoom.
  void focusOn(double xMeters, double yMeters);
  void fitToView();
  void zoomBy(double factor);

 signals:
  void cursorMoved(double xMeters, double yMeters);

 protected:
  void drawBackground(QPainter* painter, const QRectF& rect) override;
  void wheelEvent(QWheelEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void resizeEvent(QResizeEvent* event) override;

 private:
  QGraphicsScene scene_;
  QGraphicsPathItem* extent_ = nullptr;  // outermost drawn item, used for fit-to-view
  QRectF extentRect_;
  bool userMoved_ = false;  // keep auto-fit on resize until the user pans/zooms
};
