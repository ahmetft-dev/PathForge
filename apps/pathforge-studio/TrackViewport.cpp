#include "TrackViewport.h"

#include <QMouseEvent>
#include <QPainter>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>

namespace {
constexpr double kMinScale = 1e-4;  // pixels per meter
constexpr double kMaxScale = 1e4;

// Smallest 1-2-5 grid step that is at least `minPixels` wide on screen.
double gridStep(double pixelsPerMeter, double minPixels) {
  const double raw = minPixels / pixelsPerMeter;
  const double mag = std::pow(10.0, std::floor(std::log10(raw)));
  for (double m : {1.0, 2.0, 5.0, 10.0}) {
    if (m * mag >= raw) return m * mag;
  }
  return 10.0 * mag;
}
}  // namespace

TrackViewport::TrackViewport(QWidget* parent) : QGraphicsView(parent) {
  setScene(&scene_);
  setRenderHint(QPainter::Antialiasing);
  setDragMode(QGraphicsView::ScrollHandDrag);
  setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
  setResizeAnchor(QGraphicsView::AnchorViewCenter);
  setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  setFrameShape(QFrame::NoFrame);
  setMouseTracking(true);
  setBackgroundBrush(QColor(0x14, 0x16, 0x1a));
}

void TrackViewport::setPath(const pathforge::geometry::Path& path) {
  scene_.clear();
  centerline_ = nullptr;

  const auto& pts = path.points();
  QPainterPath pp;
  pp.moveTo(pts.front().x, -pts.front().y);
  for (std::size_t i = 1; i < pts.size(); ++i) pp.lineTo(pts[i].x, -pts[i].y);
  if (path.isClosed()) pp.closeSubpath();

  QPen pen(QColor(0x4f, 0xc3, 0xf7), 2.0);
  pen.setCosmetic(true);  // constant on-screen width regardless of zoom
  pen.setJoinStyle(Qt::RoundJoin);
  centerline_ = scene_.addPath(pp, pen);

  // Start marker: first point.
  QPen markPen(QColor(0xff, 0xb7, 0x4d), 1.0);
  markPen.setCosmetic(true);
  const QPointF s(pts.front().x, -pts.front().y);
  auto* marker = scene_.addEllipse(-3, -3, 6, 6, markPen, QBrush(QColor(0xff, 0xb7, 0x4d)));
  marker->setFlag(QGraphicsItem::ItemIgnoresTransformations);  // constant on-screen size
  marker->setPos(s);

  // Generous scene rect so panning is not clamped to the track bounds.
  const QRectF r = centerline_->boundingRect();
  const double m = std::max(r.width(), r.height()) * 2.0;
  scene_.setSceneRect(r.adjusted(-m, -m, m, m));
  userMoved_ = false;
  fitToView();
}

void TrackViewport::fitToView() {
  if (!centerline_) return;
  const QRectF r = centerline_->boundingRect();
  const double margin = std::max(r.width(), r.height()) * 0.08;
  fitInView(r.adjusted(-margin, -margin, margin, margin), Qt::KeepAspectRatio);
  userMoved_ = false;
}

void TrackViewport::zoomBy(double factor) {
  const double s = transform().m11() * factor;
  if (s < kMinScale || s > kMaxScale) return;
  scale(factor, factor);
  userMoved_ = true;
}

void TrackViewport::wheelEvent(QWheelEvent* event) {
  const double dy = event->angleDelta().y();
  if (dy == 0) return;
  zoomBy(std::pow(1.0015, dy));
  event->accept();
}

void TrackViewport::mouseMoveEvent(QMouseEvent* event) {
  QGraphicsView::mouseMoveEvent(event);
  const QPointF p = mapToScene(event->position().toPoint());
  emit cursorMoved(p.x(), -p.y());
  if (event->buttons() & Qt::LeftButton) userMoved_ = true;
}

void TrackViewport::resizeEvent(QResizeEvent* event) {
  QGraphicsView::resizeEvent(event);
  if (!userMoved_) fitToView();
}

void TrackViewport::drawBackground(QPainter* painter, const QRectF& rect) {
  QGraphicsView::drawBackground(painter, rect);
  const double scale = transform().m11();
  const double minor = gridStep(scale, 24.0);
  const double major = minor * 5.0;

  auto drawLines = [&](double step, const QColor& color) {
    QPen pen(color, 1.0);
    pen.setCosmetic(true);
    painter->setPen(pen);
    const double x0 = std::floor(rect.left() / step) * step;
    for (double x = x0; x <= rect.right(); x += step)
      painter->drawLine(QPointF(x, rect.top()), QPointF(x, rect.bottom()));
    const double y0 = std::floor(rect.top() / step) * step;
    for (double y = y0; y <= rect.bottom(); y += step)
      painter->drawLine(QPointF(rect.left(), y), QPointF(rect.right(), y));
  };
  // Guard against pathological line counts at extreme zoom-out.
  if (rect.width() / minor < 500 && rect.height() / minor < 500) drawLines(minor, QColor(0x22, 0x26, 0x2d));
  if (rect.width() / major < 500 && rect.height() / major < 500) drawLines(major, QColor(0x2c, 0x32, 0x3b));

  QPen axis(QColor(0x55, 0x5d, 0x6b), 1.0);
  axis.setCosmetic(true);
  painter->setPen(axis);
  painter->drawLine(QPointF(rect.left(), 0), QPointF(rect.right(), 0));
  painter->drawLine(QPointF(0, rect.top()), QPointF(0, rect.bottom()));
}
