#include "TrackViewport.h"

#include <QGraphicsEllipseItem>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>

namespace pfe = pathforge::editor;
namespace pg = pathforge::geometry;

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

QPainterPath toPainterPath(const std::vector<pg::Vec2>& pts, bool closed) {
  QPainterPath pp;
  if (pts.empty()) return pp;
  pp.moveTo(pts.front().x, -pts.front().y);
  for (std::size_t i = 1; i < pts.size(); ++i) pp.lineTo(pts[i].x, -pts[i].y);
  if (closed && pts.size() > 2) pp.closeSubpath();
  return pp;
}

QPen cosmeticPen(const QColor& c, double width, Qt::PenStyle style = Qt::SolidLine) {
  QPen pen(c, width, style);
  pen.setCosmetic(true);  // constant on-screen width regardless of zoom
  pen.setJoinStyle(Qt::RoundJoin);
  return pen;
}

void growTo(QRectF& r, const std::vector<pg::Vec2>& pts) {
  for (const pg::Vec2& p : pts) {
    const QRectF pr(p.x, -p.y, 0.0, 0.0);
    r = r.isNull() ? QRectF(p.x, -p.y, 1e-9, 1e-9) : r.united(pr.adjusted(0, 0, 1e-9, 1e-9));
  }
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
  setFocusPolicy(Qt::StrongFocus);
  setBackgroundBrush(QColor(0x14, 0x16, 0x1a));
  scene_.setSceneRect(-1000, -1000, 2000, 2000);
}

void TrackViewport::setContent(const pfe::DerivedGeometry& derived,
                               const std::vector<pfe::ControlPoint>& points,
                               std::optional<pfe::PointId> selected, const pg::ValidationReport* report) {
  scene_.clear();
  previewItem_ = nullptr;
  points_ = points;
  const bool closed = derived.data.closure == pfe::Closure::Closed;
  QRectF extent;

  // Layer order: boundaries below the centerline, overlays above, handles on top. Restrained, colour-blind
  // distinguishable hues (left = green, right = orange, centerline = blue).
  if (derived.track) {
    const pg::Track& t = *derived.track;
    scene_.addPath(toPainterPath(t.leftBoundary(), closed), cosmeticPen(QColor(0x66, 0xbb, 0x6a), 1.4));
    scene_.addPath(toPainterPath(t.rightBoundary(), closed), cosmeticPen(QColor(0xff, 0x8a, 0x50), 1.4));
    scene_.addPath(toPainterPath(t.centerline().points(), closed),
                   cosmeticPen(QColor(0x4f, 0xc3, 0xf7), 1.8));
    growTo(extent, t.leftBoundary());
    growTo(extent, t.rightBoundary());
  } else if (points.size() >= 2) {
    // Incomplete or degenerate document: show the raw control polygon so the user sees what exists.
    scene_.addPath(toPainterPath(derived.data.centerline, closed),
                   cosmeticPen(QColor(0x90, 0x96, 0xa0), 1.2, Qt::DashLine));
  }

  if (report) {
    auto polylineOf = [&](pg::PolylineId id) -> const std::vector<pg::Vec2>* {
      if (id == pg::PolylineId::Centerline) return &derived.data.centerline;
      if (!derived.track) return nullptr;
      return id == pg::PolylineId::LeftBoundary ? &derived.track->leftBoundary()
                                                : &derived.track->rightBoundary();
    };
    for (const pg::ValidationIssue& issue : report->issues) {
      if (issue.severity == pg::Severity::Info) continue;
      const QColor color =
          issue.severity == pg::Severity::Error ? QColor(0xff, 0x52, 0x52) : QColor(0xff, 0xca, 0x28);
      for (const pg::SegmentRef& ref : issue.segments) {
        const auto* pl = polylineOf(ref.polyline);
        if (!pl || ref.index >= pl->size() || (!closed && ref.index + 1 >= pl->size())) continue;
        const pg::Vec2 a = (*pl)[ref.index], b = (*pl)[(ref.index + 1) % pl->size()];
        scene_.addLine(a.x, -a.y, b.x, -b.y, cosmeticPen(color, 3.0))->setZValue(5);
      }
      if (issue.location) {
        QPainterPath cross;  // small X, constant on-screen size
        cross.moveTo(-5, -5);
        cross.lineTo(5, 5);
        cross.moveTo(-5, 5);
        cross.lineTo(5, -5);
        auto* m = scene_.addPath(cross, cosmeticPen(color, 2.0));
        m->setFlag(QGraphicsItem::ItemIgnoresTransformations);
        m->setPos(issue.location->x, -issue.location->y);
        m->setZValue(6);
      }
    }
  }

  // Control point handles (constant on-screen size). The first point of the track is drawn as a ring.
  std::vector<pg::Vec2> positions;
  positions.reserve(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    const pfe::ControlPoint& p = points[i];
    positions.push_back(p.position);
    const bool sel = selected && *selected == p.id;
    const double r = sel ? 6.0 : 4.0;
    const QColor fill = sel ? QColor(0xff, 0xca, 0x28) : QColor(0x1b, 0x1e, 0x23);
    const QColor edge = sel ? QColor(0xff, 0xe0, 0x82) : QColor(0xe8, 0xea, 0xee);
    QGraphicsItem* h = i == 0 ? static_cast<QGraphicsItem*>(scene_.addEllipse(
                                    -r, -r, 2 * r, 2 * r, cosmeticPen(edge, 1.5), QBrush(fill)))
                              : static_cast<QGraphicsItem*>(scene_.addRect(
                                    -r, -r, 2 * r, 2 * r, cosmeticPen(edge, 1.5), QBrush(fill)));
    h->setFlag(QGraphicsItem::ItemIgnoresTransformations);
    h->setPos(p.position.x, -p.position.y);
    h->setZValue(sel ? 11 : 10);
  }
  growTo(extent, positions);

  extentRect_ = extent;
  hasContent_ = !extent.isNull();
  // Generous scene rect so panning is never clamped to the content.
  const QRectF base = hasContent_ ? extent : QRectF(-100, -100, 200, 200);
  const double m = std::max({base.width(), base.height(), 200.0}) * 4.0;
  scene_.setSceneRect(base.adjusted(-m, -m, m, m));
  showInsertPreview(preview_);
}

void TrackViewport::setAddMode(bool on) {
  addMode_ = on;
  viewport()->setCursor(on ? Qt::CrossCursor : Qt::ArrowCursor);
}

void TrackViewport::showInsertPreview(std::optional<pg::Vec2> world) {
  preview_ = world;
  if (!world) {
    if (previewItem_) previewItem_->hide();
    return;
  }
  if (!previewItem_) {
    previewItem_ = scene_.addEllipse(-5, -5, 10, 10, cosmeticPen(QColor(0xff, 0xca, 0x28), 1.5), Qt::NoBrush);
    previewItem_->setFlag(QGraphicsItem::ItemIgnoresTransformations);
    previewItem_->setZValue(20);
  }
  previewItem_->setPos(world->x, -world->y);
  previewItem_->show();
}

QPointF TrackViewport::worldToViewport(pg::Vec2 w) const {
  return viewportTransform().map(QPointF(w.x, -w.y));
}

pg::Vec2 TrackViewport::viewportToWorld(QPointF v) const {
  // Exact inverse of the scene->viewport transform; keeps sub-pixel precision (mapToScene(QPoint) would
  // round).
  const QPointF scenePos = viewportTransform().inverted().map(v);
  return {scenePos.x(), -scenePos.y()};
}

std::optional<pfe::PointId> TrackViewport::pointAt(QPointF v) const {
  std::optional<pfe::PointId> best;
  double bestD = kHitRadiusPx;
  for (const pfe::ControlPoint& p : points_) {
    const QPointF d = worldToViewport(p.position) - v;
    const double dist = std::hypot(d.x(), d.y());
    if (dist <= bestD) {
      bestD = dist;
      best = p.id;
    }
  }
  return best;
}

void TrackViewport::focusOn(double x, double y) {
  centerOn(x, -y);
  userMoved_ = true;
}

void TrackViewport::fitToView() {
  const QRectF r = hasContent_ ? extentRect_ : QRectF(-100, -100, 200, 200);
  const double size = std::max({r.width(), r.height(), 1.0});
  const double margin = size * 0.08;
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

void TrackViewport::mousePressEvent(QMouseEvent* event) {
  if (event->button() != Qt::LeftButton) {
    QGraphicsView::mousePressEvent(event);
    return;
  }
  pressPos_ = event->position();
  dragging_ = false;
  if (const auto id = pointAt(pressPos_); id && !(event->modifiers() & Qt::ShiftModifier)) {
    press_ = Press::Point;
    emit pointPressed(*id);
    event->accept();
    return;
  }
  press_ = Press::Empty;
  QGraphicsView::mousePressEvent(event);  // hand-drag panning
}

void TrackViewport::mouseMoveEvent(QMouseEvent* event) {
  const QPointF pos = event->position();
  const pg::Vec2 w = viewportToWorld(pos);
  emit cursorMoved(w.x, w.y);
  if (press_ == Press::Point) {
    const QPointF d = pos - pressPos_;
    if (!dragging_ && std::hypot(d.x(), d.y()) >= kDragThresholdPx) dragging_ = true;
    if (dragging_) emit pointDragged(w.x, w.y);
    event->accept();
    return;
  }
  QGraphicsView::mouseMoveEvent(event);
  if (press_ == Press::Empty && (event->buttons() & Qt::LeftButton)) {
    const QPointF d = pos - pressPos_;
    if (std::hypot(d.x(), d.y()) >= kDragThresholdPx) userMoved_ = true;  // panning
  }
  if (press_ == Press::None && (event->modifiers() & Qt::ShiftModifier)) {
    emit insertPreviewRequested(w.x, w.y);
  } else if (preview_) {
    showInsertPreview(std::nullopt);
  }
  updateCursor(pos, event->modifiers());
}

void TrackViewport::mouseReleaseEvent(QMouseEvent* event) {
  if (event->button() != Qt::LeftButton) {
    QGraphicsView::mouseReleaseEvent(event);
    return;
  }
  const Press press = press_;
  press_ = Press::None;
  if (press == Press::Point) {
    if (dragging_) emit dragFinished();
    dragging_ = false;
    event->accept();
    return;
  }
  QGraphicsView::mouseReleaseEvent(event);
  const QPointF d = event->position() - pressPos_;
  if (press == Press::Empty && std::hypot(d.x(), d.y()) < kDragThresholdPx) {  // a click, not a pan
    const pg::Vec2 w = viewportToWorld(event->position());
    if (event->modifiers() & Qt::ShiftModifier) {
      emit insertRequested(w.x, w.y);
    } else if (addMode_) {
      emit addRequested(w.x, w.y);
    } else {
      emit emptyClicked();
    }
  }
  updateCursor(event->position(), event->modifiers());
}

void TrackViewport::keyReleaseEvent(QKeyEvent* event) {
  if (event->key() == Qt::Key_Shift) showInsertPreview(std::nullopt);
  QGraphicsView::keyReleaseEvent(event);
}

void TrackViewport::updateCursor(QPointF pos, Qt::KeyboardModifiers modifiers) {
  if (press_ != Press::None) return;
  if (pointAt(pos) && !(modifiers & Qt::ShiftModifier)) {
    viewport()->setCursor(Qt::SizeAllCursor);
  } else if (addMode_ || (modifiers & Qt::ShiftModifier)) {
    viewport()->setCursor(Qt::CrossCursor);
  } else {
    viewport()->setCursor(Qt::OpenHandCursor);
  }
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
