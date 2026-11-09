#include "MainWindow.h"

#include <QAction>
#include <QComboBox>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QMenuBar>
#include <QPushButton>
#include <QSpinBox>
#include <QStatusBar>
#include <QToolBar>
#include <QVBoxLayout>
#include <optional>
#include <pathforge/geometry/circle.hpp>
#include <pathforge/geometry/frames.hpp>
#include <pathforge/geometry/shapes.hpp>
#include <pathforge/geometry/track.hpp>

#include "TrackViewport.h"

namespace pg = pathforge::geometry;

MainWindow::MainWindow() {
  setWindowTitle("PathForge Studio");
  resize(1280, 800);

  viewport_ = new TrackViewport(this);
  setCentralWidget(viewport_);

  auto* left = new QDockWidget("Track", this);
  left->setFeatures(QDockWidget::DockWidgetMovable);
  left->setWidget(buildSidebar());
  addDockWidget(Qt::LeftDockWidgetArea, left);

  auto* right = new QDockWidget("Diagnostics", this);
  right->setFeatures(QDockWidget::DockWidgetMovable);
  right->setWidget(buildDiagnostics());
  addDockWidget(Qt::RightDockWidgetArea, right);

  // Actions
  auto* fit = new QAction("Fit to View", this);
  fit->setShortcut(QKeySequence(Qt::Key_F));
  connect(fit, &QAction::triggered, viewport_, &TrackViewport::fitToView);
  auto* zin = new QAction("Zoom In", this);
  zin->setShortcut(QKeySequence::ZoomIn);
  connect(zin, &QAction::triggered, this, [this] { viewport_->zoomBy(1.25); });
  auto* zout = new QAction("Zoom Out", this);
  zout->setShortcut(QKeySequence::ZoomOut);
  connect(zout, &QAction::triggered, this, [this] { viewport_->zoomBy(0.8); });
  auto* quit = new QAction("Quit", this);
  quit->setShortcut(QKeySequence::Quit);
  connect(quit, &QAction::triggered, this, &QWidget::close);

  menuBar()->addMenu("&File")->addAction(quit);
  auto* view = menuBar()->addMenu("&View");
  view->addAction(fit);
  view->addAction(zin);
  view->addAction(zout);
  view->addSeparator();
  view->addAction(left->toggleViewAction());
  view->addAction(right->toggleViewAction());

  auto* tb = addToolBar("View");
  tb->setMovable(false);
  tb->addAction(fit);
  tb->addAction(zin);
  tb->addAction(zout);

  cursorLabel_ = new QLabel(this);
  statusBar()->addPermanentWidget(cursorLabel_);
  statusBar()->showMessage("Scroll to zoom, drag to pan, F to fit");
  connect(viewport_, &TrackViewport::cursorMoved, this, [this](double x, double y) {
    cursorLabel_->setText(QString("x %1 m   y %2 m").arg(x, 0, 'f', 2).arg(y, 0, 'f', 2));
  });

  generateTrack();
}

QWidget* MainWindow::buildSidebar() {
  auto* w = new QWidget;
  auto* lay = new QVBoxLayout(w);

  auto* box = new QGroupBox("Synthetic preset");
  form_ = new QFormLayout(box);
  preset_ = new QComboBox;
  preset_->addItems({"Circle", "Oval", "S-Curve"});
  form_->addRow("Preset", preset_);

  auto spin = [](double lo, double hi, double value, const char* suffix) {
    auto* s = new QDoubleSpinBox;
    s->setRange(lo, hi);
    s->setDecimals(2);
    s->setSuffix(suffix);
    s->setValue(value);
    return s;
  };
  radius_ = spin(1.0, 10000.0, 100.0, " m");
  length_ = spin(2.0, 100000.0, 300.0, " m");
  width_ = spin(1.0, 100000.0, 120.0, " m");
  amplitude_ = spin(0.1, 10000.0, 30.0, " m");
  form_->addRow("Radius", radius_);
  form_->addRow("Length", length_);
  form_->addRow("Width", width_);
  form_->addRow("Amplitude", amplitude_);

  points_ = new QSpinBox;
  points_->setRange(8, 100000);
  points_->setValue(360);
  form_->addRow("Points", points_);
  trackWidth_ = spin(0.0, 1000.0, 10.0, " m");
  form_->addRow("Track width", trackWidth_);
  lay->addWidget(box);

  auto* gen = new QPushButton("Generate");
  connect(gen, &QPushButton::clicked, this, &MainWindow::generateTrack);
  connect(preset_, &QComboBox::currentIndexChanged, this, &MainWindow::presetChanged);
  lay->addWidget(gen);
  lay->addStretch();
  w->setMinimumWidth(240);
  presetChanged();
  return w;
}

// Show only the parameters relevant to the selected preset and load its defaults.
void MainWindow::presetChanged() {
  const int p = preset_->currentIndex();  // 0 circle, 1 oval, 2 s-curve
  form_->setRowVisible(radius_, p == 0);
  form_->setRowVisible(length_, p != 0);
  form_->setRowVisible(width_, p == 1);
  form_->setRowVisible(amplitude_, p == 2);
  if (p == 0) points_->setValue(360);
  if (p == 1) points_->setValue(512);
  if (p == 2) points_->setValue(256);
  // During construction the diagnostics panel does not exist yet; the constructor generates the
  // initial track itself.
  if (viewport_ && !diag_.isEmpty()) generateTrack();
}

QWidget* MainWindow::buildDiagnostics() {
  auto* w = new QWidget;
  auto* form = new QFormLayout(w);
  for (const char* key : {"Preset", "Track length", "Reference length", "Length error", "Points", "Closed",
                          "Track width", "Segment min", "Segment max", "Curvature min", "Curvature max",
                          "Max |curvature|", "Sign changes", "Boundary check"}) {
    auto* v = new QLabel("–");
    v->setTextInteractionFlags(Qt::TextSelectableByMouse);
    v->setWordWrap(true);
    form->addRow(QString(key) + ":", v);
    diag_[key] = v;
  }
  w->setMinimumWidth(260);
  return w;
}

void MainWindow::setDiagnostic(const QString& key, const QString& value) { diag_[key]->setText(value); }

void MainWindow::generateTrack() {
  try {
    const std::size_t n = static_cast<std::size_t>(points_->value());
    const int preset = preset_->currentIndex();
    std::optional<double> reference;  // analytical length of the continuous curve, if known
    std::optional<pg::Path> path;
    QString name;
    if (preset == 0) {
      const double r = radius_->value();
      path = pg::makeCircle(r, n);
      reference = pg::circleCircumference(r);
      name = QString("Circle (R = %1 m)").arg(r);
    } else if (preset == 1) {
      const double len = length_->value(), wid = width_->value();
      path = pg::makeStadium(len, wid, n);
      reference = pg::roundedRectanglePerimeter(len, wid, wid / 2.0);
      name = QString("Oval (%1 x %2 m)").arg(len).arg(wid);
    } else {
      path = pg::makeSCurve(length_->value(), amplitude_->value(), n);
      name = QString("S-Curve (L = %1 m, A = %2 m)").arg(length_->value()).arg(amplitude_->value());
    }
    const double half = trackWidth_->value() / 2.0;
    const pg::Track track = pg::Track::create(std::move(*path), half, half);
    const pg::PathStats st = pg::computeStats(track.centerline());
    const pg::TrackDiagnostics& d = track.diagnostics();
    viewport_->setTrack(track);

    auto m = [](double v) { return QString::number(v, 'f', 4) + " m"; };
    auto k = [](double v) { return QString::number(v, 'g', 6) + " 1/m"; };
    setDiagnostic("Preset", name);
    setDiagnostic("Track length", m(st.length));
    setDiagnostic("Reference length", reference ? m(*reference) : QString("n/a"));
    setDiagnostic(
        "Length error",
        reference ? QString("%1 m (discretisation)").arg(*reference - st.length, 0, 'e', 3) : QString("n/a"));
    setDiagnostic("Points", QString::number(st.pointCount));
    setDiagnostic("Closed", track.centerline().isClosed() ? "yes" : "no (open path)");
    setDiagnostic("Track width", QString("%1 m (L %2 / R %3)")
                                     .arg(track.totalWidth(), 0, 'f', 2)
                                     .arg(track.leftWidth(), 0, 'f', 2)
                                     .arg(track.rightWidth(), 0, 'f', 2));
    setDiagnostic("Segment min", m(st.minSegment));
    setDiagnostic("Segment max", m(st.maxSegment));
    setDiagnostic("Curvature min", k(st.minCurvature));
    setDiagnostic("Curvature max", k(st.maxCurvature));
    setDiagnostic("Max |curvature|", k(st.maxAbsCurvature));
    setDiagnostic("Sign changes",
                  QString::number(pg::countSignChanges(track.frames().curvature, 0.25 * st.maxAbsCurvature)) +
                      " (|k| > 25% of max)");
    setDiagnostic("Boundary check", d.anyLocalIssue()
                                        ? QString("LOCAL FOLD detected (L %1 / R %2 flipped segments)")
                                              .arg(d.leftFlippedSegments)
                                              .arg(d.rightFlippedSegments)
                                        : QString("no local folds (global validity not checked)"));
    statusBar()->showMessage(QString("Generated %1, %2 points").arg(name).arg(n), 4000);
  } catch (const pg::GeometryError& e) {
    statusBar()->showMessage(QString("Invalid geometry: %1").arg(e.what()));
  }
}
