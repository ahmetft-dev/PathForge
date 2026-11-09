#include "MainWindow.h"

#include <QAction>
#include <QComboBox>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QListWidget>
#include <QMenuBar>
#include <QMessageBox>
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
#include <pathforge/geometry/track_data.hpp>
#include <pathforge/geometry/validation.hpp>
#include <pathforge/io/track_io.hpp>

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

  auto* open = new QAction("&Open Track…", this);
  open->setShortcut(QKeySequence::Open);
  connect(open, &QAction::triggered, this, &MainWindow::openTrack);
  auto* save = new QAction("&Save Track", this);
  save->setShortcut(QKeySequence::Save);
  connect(save, &QAction::triggered, this, &MainWindow::saveTrack);
  auto* saveAs = new QAction("Save Track &As…", this);
  saveAs->setShortcut(QKeySequence::SaveAs);
  connect(saveAs, &QAction::triggered, this, &MainWindow::saveTrackAs);
  auto* file = menuBar()->addMenu("&File");
  file->addAction(open);
  file->addAction(save);
  file->addAction(saveAs);
  file->addSeparator();
  file->addAction(quit);
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
  auto* outer = new QVBoxLayout(w);
  auto* form = new QFormLayout;
  form->setRowWrapPolicy(
      QFormLayout::WrapLongRows);  // long values drop below their label instead of clipping
  form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
  outer->addLayout(form);
  for (const char* key : {"Track name", "File", "Reference length", "Length error", "Track length", "Points",
                          "Closed", "Width", "Segment min", "Segment max", "Curvature min", "Curvature max",
                          "Max |curvature|", "Sign changes", "Validation", "Boundary gap"}) {
    auto* v = new QLabel("–");
    v->setTextInteractionFlags(Qt::TextSelectableByMouse);
    v->setWordWrap(true);
    form->addRow(QString(key) + ":", v);
    diag_[key] = v;
  }
  outer->addWidget(new QLabel("Validation issues (click to locate):"));
  issueList_ = new QListWidget;
  issueList_->setMinimumHeight(140);
  issueList_->setWordWrap(true);
  connect(issueList_, &QListWidget::currentRowChanged, this, &MainWindow::issueSelected);
  outer->addWidget(issueList_, 1);
  w->setMinimumWidth(340);
  return w;
}

void MainWindow::setDiagnostic(const QString& key, const QString& value) { diag_[key]->setText(value); }

void MainWindow::updateTitle() {
  QString t = "PathForge Studio — " + QString::fromStdString(current_.name);
  if (!filePath_.isEmpty()) t += " [" + QFileInfo(filePath_).fileName() + "]";
  setWindowTitle(t);
}

void MainWindow::issueSelected(int row) {
  if (row < 0 || row >= static_cast<int>(report_.issues.size())) return;
  const auto& loc = report_.issues[static_cast<std::size_t>(row)].location;
  if (loc) viewport_->focusOn(loc->x, loc->y);
}

bool MainWindow::display(const pg::TrackData& data, const QString& file, std::optional<double> reference,
                         QString* error) {
  const pg::ValidationReport report = pg::validateTrackData(data);
  if (!report.checked.structure) {
    // The data cannot be turned into a track (e.g. duplicate points, mismatched widths): show why, change
    // nothing.
    QStringList lines;
    for (const auto& i : report.issues) lines << QString::fromStdString(i.message);
    if (error)
      *error = "The track data is structurally invalid and cannot be displayed:\n" + lines.join("\n");
    return false;
  }
  const pg::Track track = pg::buildTrack(data);
  const pg::PathStats st = pg::computeStats(track.centerline());
  viewport_->setTrack(track, report);
  current_ = data;
  report_ = report;
  filePath_ = file;

  auto m = [](double v) { return QString::number(v, 'f', 4) + " m"; };
  auto k = [](double v) { return QString::number(v, 'g', 6) + " 1/m"; };
  setDiagnostic("Track name", QString::fromStdString(data.name));
  setDiagnostic("File", file.isEmpty() ? "(unsaved)" : file);
  setDiagnostic("Track length", m(st.length));
  setDiagnostic("Reference length", reference ? m(*reference) : QString("n/a"));
  setDiagnostic("Length error", reference
                                    ? QString("%1 m (discretisation)").arg(*reference - st.length, 0, 'e', 3)
                                    : QString("n/a"));
  setDiagnostic("Points", QString::number(st.pointCount));
  setDiagnostic("Closed", track.centerline().isClosed() ? "yes" : "no (open path)");
  setDiagnostic("Width", track.hasConstantWidths() ? QString("constant, L %1 / R %2 m (total %3 m)")
                                                         .arg(track.leftWidth(), 0, 'f', 2)
                                                         .arg(track.rightWidth(), 0, 'f', 2)
                                                         .arg(track.totalWidth(), 0, 'f', 2)
                                                   : QString("variable, total %1 … %2 m")
                                                         .arg(track.minTotalWidth(), 0, 'f', 2)
                                                         .arg(track.maxTotalWidth(), 0, 'f', 2));
  setDiagnostic("Segment min", m(st.minSegment));
  setDiagnostic("Segment max", m(st.maxSegment));
  setDiagnostic("Curvature min", k(st.minCurvature));
  setDiagnostic("Curvature max", k(st.maxCurvature));
  setDiagnostic("Max |curvature|", k(st.maxAbsCurvature));
  setDiagnostic("Sign changes",
                QString::number(pg::countSignChanges(track.frames().curvature, 0.25 * st.maxAbsCurvature)) +
                    " (|k| > 25% of max)");
  setDiagnostic("Validation", QString("%1 (%2 error(s), %3 warning(s))")
                                  .arg(pg::toString(report.status()))
                                  .arg(report.errorCount)
                                  .arg(report.warningCount));
  setDiagnostic("Boundary gap", report.clearance.computed
                                    ? QString("%1 m (min distance between left and right boundary polylines)")
                                          .arg(report.clearance.minBoundaryDistance, 0, 'f', 3)
                                    : QString("n/a"));

  issueList_->clear();
  for (const auto& i : report.issues) {
    auto* item = new QListWidgetItem(
        QString("[%1] %2: %3")
            .arg(pg::toString(i.severity), pg::toString(i.type), QString::fromStdString(i.message)));
    if (i.severity == pg::Severity::Error) item->setForeground(QColor(0xff, 0x6e, 0x6e));
    if (i.severity == pg::Severity::Warning) item->setForeground(QColor(0xff, 0xca, 0x28));
    issueList_->addItem(item);
  }
  if (report.suppressed > 0)
    issueList_->addItem(QString("… %1 further issue(s) not listed").arg(report.suppressed));
  if (report.issues.empty()) issueList_->addItem("No issues found by the implemented checks.");
  updateTitle();
  return true;
}

QString MainWindow::openTrackFile(const QString& path) {
  const pathforge::io::LoadResult r = pathforge::io::loadTrackFile(path.toStdString());
  if (!r.ok()) return QString::fromStdString(r.error);
  QString error;
  if (!display(*r.data, path, std::nullopt, &error)) return error;
  QString note;
  for (const std::string& w : r.warnings) note += QString::fromStdString(w) + "; ";
  statusBar()->showMessage("Opened " + path + (note.isEmpty() ? "" : " — " + note), 6000);
  return {};
}

QString MainWindow::saveTrackFile(const QString& path, bool overwrite) {
  const auto r = pathforge::io::saveTrackFile(
      path.toStdString(), current_,
      overwrite ? pathforge::io::OverwritePolicy::Overwrite : pathforge::io::OverwritePolicy::Fail);
  if (!r.ok()) return QString::fromStdString(r.error);
  filePath_ = path;
  setDiagnostic("File", path);
  updateTitle();
  statusBar()->showMessage("Saved " + path, 4000);
  return {};
}

void MainWindow::openTrack() {
  const QString path =
      QFileDialog::getOpenFileName(this, "Open Track", filePath_, "PathForge track (*.json);;All files (*)");
  if (path.isEmpty()) return;
  const QString error = openTrackFile(path);
  if (!error.isEmpty()) {
    QMessageBox::warning(this, "Cannot open track", error + "\n\nThe current track was not changed.");
  }
}

void MainWindow::saveTrackAs() {
  const QString path =
      QFileDialog::getSaveFileName(this, "Save Track As", filePath_, "PathForge track (*.json)");
  if (path.isEmpty()) return;  // the dialog already asked for confirmation when the file exists
  const QString error = saveTrackFile(path, true);
  if (!error.isEmpty()) QMessageBox::warning(this, "Cannot save track", error);
}

void MainWindow::saveTrack() {
  if (filePath_.isEmpty()) {
    saveTrackAs();
    return;
  }
  const QString error = saveTrackFile(filePath_, true);
  if (!error.isEmpty()) QMessageBox::warning(this, "Cannot save track", error);
}

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
    QString error;
    if (!display(pg::toTrackData(track, name.toStdString()), QString(), reference, &error)) {
      statusBar()->showMessage(error);
      return;
    }
    statusBar()->showMessage(QString("Generated %1, %2 points").arg(name).arg(n), 4000);
  } catch (const pg::GeometryError& e) {
    statusBar()->showMessage(QString("Invalid geometry: %1").arg(e.what()));
  }
}
