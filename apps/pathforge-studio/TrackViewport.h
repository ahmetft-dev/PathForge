#pragma once

#include <pathforge/geometry/path.hpp>

#include <QGraphicsPathItem>
#include <QGraphicsScene>
#include <QGraphicsView>

// 2D track viewport. World coordinates are meters with y up; the scene stores
// (x, -y) because Qt's y axis points down. No geometry is computed here: the view
// only draws points that come from pathforge::geometry.
class TrackViewport : public QGraphicsView {
  Q_OBJECT
 public:
  explicit TrackViewport(QWidget* parent = nullptr);

  void setPath(const pathforge::geometry::Path& path);
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
  QGraphicsPathItem* centerline_ = nullptr;
  bool userMoved_ = false;  // keep auto-fit on resize until the user pans/zooms
};
