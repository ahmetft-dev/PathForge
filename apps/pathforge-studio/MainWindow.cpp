#include "MainWindow.h"

#include "TrackViewport.h"

#include <pathforge/geometry/circle.hpp>

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
  auto* form = new QFormLayout(box);
  preset_ = new QComboBox;
  preset_->addItem("Circle");
  form->addRow("Preset", preset_);

  radius_ = new QDoubleSpinBox;
  radius_->setRange(1.0, 10000.0);
  radius_->setDecimals(2);
  radius_->setSuffix(" m");
  radius_->setValue(100.0);
  form->addRow("Radius", radius_);

  points_ = new QSpinBox;
  points_->setRange(3, 100000);
  points_->setValue(360);
  form->addRow("Points", points_);
  lay->addWidget(box);

  auto* gen = new QPushButton("Generate");
  connect(gen, &QPushButton::clicked, this, &MainWindow::generateTrack);
  lay->addWidget(gen);
  lay->addStretch();
  w->setMinimumWidth(220);
  return w;
}

QWidget* MainWindow::buildDiagnostics() {
  auto* w = new QWidget;
  auto* form = new QFormLayout(w);
  for (const char* key : {"Track length", "Points", "Closed", "Segment min", "Segment max", "Curvature min",
                          "Curvature max", "Mean |curvature|", "Reference 2πR", "Length error", "Reference 1/R"}) {
    auto* v = new QLabel("–");
    v->setTextInteractionFlags(Qt::TextSelectableByMouse);
    form->addRow(QString(key) + ":", v);
    diag_[key] = v;
  }
  w->setMinimumWidth(240);
  return w;
}

void MainWindow::setDiagnostic(const QString& key, const QString& value) { diag_[key]->setText(value); }

void MainWindow::generateTrack() {
  try {
    const double r = radius_->value();
    const auto n = static_cast<std::size_t>(points_->value());
    const pg::Path track = pg::makeCircle(r, n);
    const pg::PathStats st = pg::computeStats(track);
    viewport_->setPath(track);

    const double exact = pg::circleCircumference(r);
    auto m = [](double v) { return QString::number(v, 'f', 4) + " m"; };
    auto k = [](double v) { return QString::number(v, 'g', 9) + " 1/m"; };
    setDiagnostic("Track length", m(st.length));
    setDiagnostic("Points", QString::number(st.pointCount));
    setDiagnostic("Closed", track.isClosed() ? "yes" : "no");
    setDiagnostic("Segment min", m(st.minSegment));
    setDiagnostic("Segment max", m(st.maxSegment));
    setDiagnostic("Curvature min", k(st.minCurvature));
    setDiagnostic("Curvature max", k(st.maxCurvature));
    setDiagnostic("Mean |curvature|", k(st.meanAbsCurvature));
    setDiagnostic("Reference 2πR", m(exact));
    setDiagnostic("Length error", QString("%1 m (discretisation)").arg(exact - st.length, 0, 'e', 3));
    setDiagnostic("Reference 1/R", k(1.0 / r));
    statusBar()->showMessage(QString("Generated circle: R = %1 m, %2 points").arg(r).arg(n), 4000);
  } catch (const pg::GeometryError& e) {
    statusBar()->showMessage(QString("Invalid geometry: %1").arg(e.what()));
  }
}
