#include "MainWindow.h"

#include <QAction>
#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMenuBar>
#include <QMessageBox>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QStatusBar>
#include <QToolBar>
#include <QVBoxLayout>
#include <optional>
#include <pathforge/editor/derived_geometry.hpp>
#include <pathforge/geometry/circle.hpp>
#include <pathforge/geometry/frames.hpp>
#include <pathforge/geometry/shapes.hpp>
#include <pathforge/geometry/track.hpp>
#include <pathforge/geometry/track_data.hpp>
#include <pathforge/geometry/validation.hpp>
#include <pathforge/io/document_io.hpp>

#include "EditorSession.h"
#include "TrackViewport.h"

namespace pg = pathforge::geometry;
namespace pfe = pathforge::editor;
namespace pio = pathforge::io;

MainWindow::MainWindow() {
  resize(1360, 840);
  session_ = new EditorSession(this);

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

  // ---- Actions
  auto* newAct = new QAction("&New Track", this);
  newAct->setShortcut(QKeySequence::New);
  connect(newAct, &QAction::triggered, this, &MainWindow::newTrack);
  auto* open = new QAction("&Open Track…", this);
  open->setShortcut(QKeySequence::Open);
  connect(open, &QAction::triggered, this, &MainWindow::openTrack);
  auto* save = new QAction("&Save Track", this);
  save->setShortcut(QKeySequence::Save);
  connect(save, &QAction::triggered, this, [this] { saveTrack(); });
  auto* saveAs = new QAction("Save Track &As…", this);
  saveAs->setShortcut(QKeySequence::SaveAs);
  connect(saveAs, &QAction::triggered, this, [this] { saveTrackAs(); });
  auto* quit = new QAction("Quit", this);
  quit->setShortcut(QKeySequence::Quit);
  connect(quit, &QAction::triggered, this, &QWidget::close);

  addPointsAction_ = new QAction("Add Points", this);
  addPointsAction_->setCheckable(true);
  addPointsAction_->setShortcut(QKeySequence(Qt::Key_A));
  addPointsAction_->setToolTip("Click on empty space to append control points (A)");
  connect(addPointsAction_, &QAction::toggled, viewport_, &TrackViewport::setAddMode);
  closedAction_ = new QAction("Closed Loop", this);
  closedAction_->setCheckable(true);
  closedAction_->setShortcut(QKeySequence(Qt::Key_L));
  closedAction_->setToolTip("Close or reopen the track (L)");
  connect(closedAction_, &QAction::triggered, this,
          [this](bool on) { session_->setClosure(on ? pfe::Closure::Closed : pfe::Closure::Open); });
  deleteAction_ = new QAction("Delete Point", this);
  deleteAction_->setShortcuts({QKeySequence::Delete, QKeySequence(Qt::Key_Backspace)});
  connect(deleteAction_, &QAction::triggered, this, &MainWindow::deleteSelected);

  auto* validateAct = new QAction("Validate Now", this);
  validateAct->setShortcut(QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_V));
  connect(validateAct, &QAction::triggered, session_, &EditorSession::requestValidation);

  auto* fit = new QAction("Fit to View", this);
  fit->setShortcut(QKeySequence(Qt::Key_F));
  connect(fit, &QAction::triggered, viewport_, &TrackViewport::fitToView);
  auto* zin = new QAction("Zoom In", this);
  zin->setShortcut(QKeySequence::ZoomIn);
  connect(zin, &QAction::triggered, this, [this] { viewport_->zoomBy(1.25); });
  auto* zout = new QAction("Zoom Out", this);
  zout->setShortcut(QKeySequence::ZoomOut);
  connect(zout, &QAction::triggered, this, [this] { viewport_->zoomBy(0.8); });

  auto* file = menuBar()->addMenu("&File");
  file->addAction(newAct);
  file->addAction(open);
  file->addAction(save);
  file->addAction(saveAs);
  file->addSeparator();
  file->addAction(quit);
  undoAction_ = session_->undoStack()->createUndoAction(this, "&Undo");
  undoAction_->setShortcut(QKeySequence::Undo);
  redoAction_ = session_->undoStack()->createRedoAction(this, "&Redo");
  redoAction_->setShortcut(QKeySequence::Redo);
  auto* edit = menuBar()->addMenu("&Edit");
  edit->addAction(undoAction_);
  edit->addAction(redoAction_);
  edit->addSeparator();
  edit->addAction(addPointsAction_);
  edit->addAction(closedAction_);
  edit->addAction(deleteAction_);
  edit->addSeparator();
  edit->addAction(validateAct);
  auto* view = menuBar()->addMenu("&View");
  view->addAction(fit);
  view->addAction(zin);
  view->addAction(zout);
  view->addSeparator();
  view->addAction(left->toggleViewAction());
  view->addAction(right->toggleViewAction());

  auto* tb = addToolBar("Main");
  tb->setMovable(false);
  tb->addAction(newAct);
  tb->addAction(open);
  tb->addAction(save);
  tb->addSeparator();
  tb->addAction(undoAction_);
  tb->addAction(redoAction_);
  tb->addSeparator();
  tb->addAction(addPointsAction_);
  tb->addAction(closedAction_);
  tb->addAction(deleteAction_);
  tb->addSeparator();
  tb->addAction(fit);
  tb->addAction(zin);
  tb->addAction(zout);

  cursorLabel_ = new QLabel(this);
  statusBar()->addPermanentWidget(cursorLabel_);
  statusBar()->showMessage(
      "Click a point to select, drag to move · Shift+click inserts · A: add points · Delete removes · drag "
      "empty space "
      "to pan");

  // ---- Wiring: viewport requests -> session edits; session changes -> UI refresh.
  connect(viewport_, &TrackViewport::cursorMoved, this, [this](double x, double y) {
    cursorLabel_->setText(QString("x %1 m   y %2 m").arg(x, 0, 'f', 2).arg(y, 0, 'f', 2));
  });
  connect(viewport_, &TrackViewport::pointPressed, session_, &EditorSession::beginDrag);
  connect(viewport_, &TrackViewport::pointDragged, this,
          [this](double x, double y) { session_->dragTo({x, y}); });
  connect(viewport_, &TrackViewport::dragFinished, session_, &EditorSession::endDrag);
  connect(viewport_, &TrackViewport::emptyClicked, this, [this] { session_->select(std::nullopt); });
  connect(viewport_, &TrackViewport::addRequested, this,
          [this](double x, double y) { session_->appendPoint({x, y}); });
  connect(viewport_, &TrackViewport::insertRequested, this, [this](double x, double y) {
    const double radius = TrackViewport::kInsertRadiusPx / viewport_->pixelsPerMeter();
    if (!session_->insertNear({x, y}, radius))
      statusBar()->showMessage("No track segment under the pointer", 3000);
  });
  connect(viewport_, &TrackViewport::insertPreviewRequested, this, [this](double x, double y) {
    const double radius = TrackViewport::kInsertRadiusPx / viewport_->pixelsPerMeter();
    const auto t = pfe::findInsertion(session_->document(), session_->derived(), {x, y});
    viewport_->showInsertPreview(t && t->distance <= radius ? std::optional(t->position) : std::nullopt);
  });

  connect(session_, &EditorSession::geometryChanged, this, &MainWindow::onGeometryChanged);
  connect(session_, &EditorSession::selectionChanged, this, [this] {
    refreshViewport();
    syncDocumentWidgets();  // enables/disables selection-dependent actions
  });
  connect(session_, &EditorSession::validationChanged, this, &MainWindow::onValidationChanged);
  connect(session_, &EditorSession::documentReplaced, this, &MainWindow::onDocumentReplaced);
  connect(session_, &EditorSession::modifiedChanged, this, [this](bool m) { setWindowModified(m); });
  connect(session_, &EditorSession::editRejected, this,
          [this](const QString& why) { statusBar()->showMessage("Edit rejected: " + why, 5000); });

  generatePreset();
}

// ----------------------------------------------------------------------------------------------------------
// panels

QWidget* MainWindow::buildSidebar() {
  auto* w = new QWidget;
  auto* lay = new QVBoxLayout(w);

  auto* box = new QGroupBox("Synthetic preset");
  form_ = new QFormLayout(box);
  preset_ = new QComboBox;
  preset_->addItems({"Circle", "Oval", "S-Curve"});
  form_->addRow("Preset", preset_);

  auto spin = [](double lo, double hi, double value, const char* suffix, int decimals = 2) {
    auto* s = new QDoubleSpinBox;
    s->setRange(lo, hi);
    s->setDecimals(decimals);
    s->setSuffix(suffix);
    s->setValue(value);
    s->setKeyboardTracking(false);
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
  auto* gen = new QPushButton("Generate preset");
  gen->setToolTip("Replaces the document with an exactly sampled preset (polyline control points)");
  connect(gen, &QPushButton::clicked, this, &MainWindow::generatePreset);
  connect(preset_, &QComboBox::currentIndexChanged, this, &MainWindow::presetChanged);
  form_->addRow(gen);
  lay->addWidget(box);

  auto* doc = new QGroupBox("Document");
  auto* df = new QFormLayout(doc);
  nameEdit_ = new QLineEdit;
  connect(nameEdit_, &QLineEdit::editingFinished, this, [this] { session_->setName(nameEdit_->text()); });
  df->addRow("Name", nameEdit_);
  closedCheck_ = new QCheckBox("Closed loop");
  connect(closedCheck_, &QCheckBox::toggled, this,
          [this](bool on) { session_->setClosure(on ? pfe::Closure::Closed : pfe::Closure::Open); });
  df->addRow(closedCheck_);
  interpolation_ = new QComboBox;
  interpolation_->addItems({"Catmull-Rom spline", "Polyline"});
  connect(interpolation_, &QComboBox::currentIndexChanged, this,
          [this] { applyDocumentSettingsFromWidgets(); });
  df->addRow("Interpolation", interpolation_);
  spacing_ = spin(0.0, 1000.0, 2.0, " m", 3);
  spacing_->setSpecialValueText("off");
  spacing_->setToolTip("Equal arc-length resampling spacing; 0 = keep interpolated vertices");
  connect(spacing_, &QDoubleSpinBox::valueChanged, this, [this] { applyDocumentSettingsFromWidgets(); });
  df->addRow("Resample spacing", spacing_);
  defaultLeft_ = spin(0.0, 1000.0, 5.0, " m");
  defaultRight_ = spin(0.0, 1000.0, 5.0, " m");
  auto setDefaults = [this] { session_->setDefaultWidths(defaultLeft_->value(), defaultRight_->value()); };
  connect(defaultLeft_, &QDoubleSpinBox::valueChanged, this, setDefaults);
  connect(defaultRight_, &QDoubleSpinBox::valueChanged, this, setDefaults);
  df->addRow("New point width L", defaultLeft_);
  df->addRow("New point width R", defaultRight_);
  lay->addWidget(doc);

  lay->addStretch();
  w->setMinimumWidth(260);
  presetChanged();
  return w;
}

void MainWindow::presetChanged() {
  const int p = preset_->currentIndex();  // 0 circle, 1 oval, 2 s-curve
  form_->setRowVisible(radius_, p == 0);
  form_->setRowVisible(length_, p != 0);
  form_->setRowVisible(width_, p == 1);
  form_->setRowVisible(amplitude_, p == 2);
  if (p == 0) points_->setValue(360);
  if (p == 1) points_->setValue(512);
  if (p == 2) points_->setValue(256);
}

QWidget* MainWindow::buildDiagnostics() {
  auto* w = new QWidget;
  auto* outer = new QVBoxLayout(w);
  // ---- Selected point: position and widths in meters. Edits are committed when the field loses focus or
  // Enter is pressed (keyboard tracking is off), so typing a value is one undo step, not one per keystroke.
  pointBox_ = new QGroupBox("Selected point");
  auto* pf = new QFormLayout(pointBox_);
  pointTitle_ = new QLabel("No point selected");
  pf->addRow(pointTitle_);
  auto coordSpin = [](const char* name, double lo, double hi, int decimals, const char* suffix) {
    auto* sp = new QDoubleSpinBox;
    sp->setObjectName(name);
    sp->setRange(lo, hi);
    sp->setDecimals(decimals);
    sp->setSuffix(suffix);
    sp->setKeyboardTracking(false);
    return sp;
  };
  pointX_ = coordSpin("pointX", -pfe::kMaxCoordinate, pfe::kMaxCoordinate, 3, " m");
  pointY_ = coordSpin("pointY", -pfe::kMaxCoordinate, pfe::kMaxCoordinate, 3, " m");
  pointLeft_ = coordSpin("pointLeft", 0.0, pfe::kMaxWidth, 2, " m");
  pointRight_ = coordSpin("pointRight", 0.0, pfe::kMaxWidth, 2, " m");
  pointLeft_->setToolTip(
      "Left half-width at this control point; interpolated linearly in arc length between points");
  pointRight_->setToolTip(
      "Right half-width at this control point; interpolated linearly in arc length between points");
  pf->addRow("X", pointX_);
  pf->addRow("Y", pointY_);
  pf->addRow("Left width", pointLeft_);
  pf->addRow("Right width", pointRight_);
  applyAllWidths_ = new QPushButton("Set all points to these widths");
  applyAllWidths_->setObjectName("applyAllWidths");
  pf->addRow(applyAllWidths_);
  auto* hint = new QLabel(
      "Widths are defined at control points and interpolated linearly along the track's arc length.");
  hint->setWordWrap(true);
  pf->addRow(hint);
  auto commitPosition = [this] {
    if (const auto id = session_->selection())
      session_->setPointPosition(*id, {pointX_->value(), pointY_->value()});
  };
  auto commitWidths = [this] {
    if (const auto id = session_->selection())
      session_->setPointWidths(*id, pointLeft_->value(), pointRight_->value());
  };
  connect(pointX_, &QDoubleSpinBox::valueChanged, this, commitPosition);
  connect(pointY_, &QDoubleSpinBox::valueChanged, this, commitPosition);
  connect(pointLeft_, &QDoubleSpinBox::valueChanged, this, commitWidths);
  connect(pointRight_, &QDoubleSpinBox::valueChanged, this, commitWidths);
  connect(applyAllWidths_, &QPushButton::clicked, this,
          [this] { session_->setAllWidths(pointLeft_->value(), pointRight_->value()); });
  outer->addWidget(pointBox_);

  auto* form = new QFormLayout;
  form->setRowWrapPolicy(
      QFormLayout::WrapLongRows);  // long values drop below their label instead of clipping
  form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
  outer->addLayout(form);
  for (const char* key : {"File", "Control points", "Samples", "Track length", "Width", "Curvature min",
                          "Curvature max", "Max |curvature|", "Geometry", "Validation", "Boundary gap"}) {
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

// ---------------------------------------------------------------------------------------------- session ->
// UI

void MainWindow::refreshViewport() {
  // The viewport content is a pure function of (derived geometry, selection, whether a current report exists
  // and which one). Skip the (comparatively expensive) scene rebuild when nothing it shows has changed.
  const ViewKey key{session_->generation(), session_->selection().value_or(0),
                    session_->isReportCurrent() ? 1u : 0u};
  if (key == lastView_) return;
  lastView_ = key;
  viewport_->setContent(session_->derived(), session_->document().points(), session_->selection(),
                        session_->isReportCurrent() ? &session_->report() : nullptr);
}

void MainWindow::syncDocumentWidgets() {
  const pfe::TrackDocument& d = session_->document();
  const QSignalBlocker b1(nameEdit_), b2(closedCheck_), b3(interpolation_), b4(spacing_), b5(closedAction_);
  if (!nameEdit_->hasFocus()) nameEdit_->setText(QString::fromStdString(d.name()));
  closedCheck_->setChecked(d.closure() == pfe::Closure::Closed);
  closedAction_->setChecked(d.closure() == pfe::Closure::Closed);
  interpolation_->setCurrentIndex(d.settings().interpolation == pfe::Interpolation::CatmullRom ? 0 : 1);
  spacing_->setValue(d.settings().resampleSpacing);
  deleteAction_->setEnabled(session_->selection().has_value());
  syncPointPanel();
}

void MainWindow::syncPointPanel() {
  const auto id = session_->selection();
  const pfe::ControlPoint* p = id ? session_->document().find(*id) : nullptr;
  pointBox_->setEnabled(p != nullptr);
  const QSignalBlocker b1(pointX_), b2(pointY_), b3(pointLeft_), b4(pointRight_);
  if (!p) {
    pointTitle_->setText("No point selected");
    return;
  }
  pointTitle_->setText(QString("Point %1 of %2")
                           .arg(*session_->document().indexOf(p->id) + 1)
                           .arg(session_->document().size()));
  // Do not overwrite a field the user is typing in.
  if (!pointX_->hasFocus()) pointX_->setValue(p->position.x);
  if (!pointY_->hasFocus()) pointY_->setValue(p->position.y);
  if (!pointLeft_->hasFocus()) pointLeft_->setValue(p->leftWidth);
  if (!pointRight_->hasFocus()) pointRight_->setValue(p->rightWidth);
}

void MainWindow::applyDocumentSettingsFromWidgets() {
  pfe::GeometrySettings s = session_->document().settings();
  s.interpolation =
      interpolation_->currentIndex() == 0 ? pfe::Interpolation::CatmullRom : pfe::Interpolation::Polyline;
  s.resampleSpacing = spacing_->value() < 1e-3 ? 0.0 : spacing_->value();
  session_->setSettings(s);
}

void MainWindow::onGeometryChanged() {
  refreshViewport();
  syncDocumentWidgets();
  const pfe::DerivedGeometry& g = session_->derived();
  auto m = [](double v) { return QString::number(v, 'f', 3) + " m"; };
  auto k = [](double v) { return QString::number(v, 'g', 6) + " 1/m"; };
  setDiagnostic("Control points", QString::number(session_->document().size()));
  setDiagnostic("Samples", g.track ? QString::number(g.data.centerline.size()) : QString("–"));
  if (g.track) {
    const pg::PathStats st = pg::computeStats(g.track->centerline());
    setDiagnostic("Track length", m(st.length));
    setDiagnostic("Width", QString("total %1 … %2 m")
                               .arg(g.track->minTotalWidth(), 0, 'f', 2)
                               .arg(g.track->maxTotalWidth(), 0, 'f', 2));
    setDiagnostic("Curvature min", k(st.minCurvature));
    setDiagnostic("Curvature max", k(st.maxCurvature));
    setDiagnostic("Max |curvature|", k(st.maxAbsCurvature));
    setDiagnostic("Geometry", "generated");
  } else {
    for (const char* key : {"Track length", "Width", "Curvature min", "Curvature max", "Max |curvature|"}) {
      setDiagnostic(key, "–");
    }
    setDiagnostic("Geometry", QString::fromStdString(g.message));
  }
  updateTitle();
}

void MainWindow::onValidationChanged() {
  refreshViewport();
  issueList_->clear();
  if (!session_->isReportCurrent()) {
    // Fast preview: the Track's own O(N) local diagnostics are available immediately; the full (global)
    // validation runs after the edit completes (or on the worker thread for large tracks).
    QString text = session_->isDragging() ? "pending - preview while dragging" : "pending - validating...";
    if (const auto& t = session_->derived().track) {
      const auto& d = t->diagnostics();
      text += QString(" (local check: %1)")
                  .arg(d.anyLocalIssue() ? QString("%1 reversed/overly offset boundary segment(s)")
                                               .arg(d.leftFlippedSegments + d.rightFlippedSegments +
                                                    d.leftCurvatureViolations + d.rightCurvatureViolations)
                                         : QString("no local folds"));
    }
    setDiagnostic("Validation", text);
    setDiagnostic("Boundary gap", "–");
    return;
  }
  const pg::ValidationReport& r = session_->report();
  setDiagnostic("Validation", QString("%1 (%2 error(s), %3 warning(s)) - %4 ms")
                                  .arg(pg::toString(r.status()))
                                  .arg(r.errorCount)
                                  .arg(r.warningCount)
                                  .arg(session_->lastValidationMilliseconds(), 0, 'f', 1));
  setDiagnostic("Boundary gap", r.clearance.computed
                                    ? QString("%1 m (min distance between left and right boundary "
                                              "polylines)")
                                          .arg(r.clearance.minBoundaryDistance, 0, 'f', 3)
                                    : QString("n/a"));
  for (const auto& i : r.issues) {
    auto* item = new QListWidgetItem(
        QString("[%1] %2: %3")
            .arg(pg::toString(i.severity), pg::toString(i.type), QString::fromStdString(i.message)));
    if (i.severity == pg::Severity::Error) item->setForeground(QColor(0xff, 0x6e, 0x6e));
    if (i.severity == pg::Severity::Warning) item->setForeground(QColor(0xff, 0xca, 0x28));
    issueList_->addItem(item);
  }
  if (r.suppressed > 0) issueList_->addItem(QString("… %1 further issue(s) not listed").arg(r.suppressed));
  if (r.issues.empty()) issueList_->addItem("No issues found by the implemented checks.");
}

void MainWindow::onDocumentReplaced() {
  syncDocumentWidgets();
  setDiagnostic("File", session_->filePath().isEmpty() ? "(unsaved)" : session_->filePath());
  viewport_->fitToView();
  updateTitle();
}

void MainWindow::updateTitle() {
  QString t = "PathForge Studio — " + QString::fromStdString(session_->document().name());
  if (!session_->filePath().isEmpty()) t += " (" + QFileInfo(session_->filePath()).fileName() + ")";
  setWindowTitle(t + "[*]");  // "[*]" is replaced by Qt with the modified marker while windowModified is set
  setWindowModified(session_->isModified());
}

void MainWindow::issueSelected(int row) {
  if (!session_->isReportCurrent()) return;
  const auto& issues = session_->report().issues;
  if (row < 0 || row >= static_cast<int>(issues.size())) return;
  if (const auto& loc = issues[static_cast<std::size_t>(row)].location) viewport_->focusOn(loc->x, loc->y);
}

// ------------------------------------------------------------------------------------------------- commands

void MainWindow::deleteSelected() {
  if (const auto id = session_->selection()) session_->removePoint(*id);
}

void MainWindow::newTrack() {
  if (!maybeSave()) return;
  pfe::TrackDocument d;
  d.setName("Untitled track");
  session_->replaceDocument(std::move(d), QString());
  addPointsAction_->setChecked(true);
  statusBar()->showMessage("New track: click to add control points; press L to close the loop", 6000);
}

void MainWindow::generatePreset() {
  if (!maybeSave()) return;
  try {
    const std::size_t n = static_cast<std::size_t>(points_->value());
    const int preset = preset_->currentIndex();
    std::optional<pg::Path> path;
    QString name;
    if (preset == 0) {
      path = pg::makeCircle(radius_->value(), n);
      name = QString("Circle (R = %1 m)").arg(radius_->value());
    } else if (preset == 1) {
      path = pg::makeStadium(length_->value(), width_->value(), n);
      name = QString("Oval (%1 x %2 m)").arg(length_->value()).arg(width_->value());
    } else {
      path = pg::makeSCurve(length_->value(), amplitude_->value(), n);
      name = QString("S-Curve (L = %1 m, A = %2 m)").arg(length_->value()).arg(amplitude_->value());
    }
    const double half = trackWidth_->value() / 2.0;
    const pg::Track track = pg::Track::create(std::move(*path), half, half);
    // Presets are exact analytic samples; they become polyline control points so the geometry is not altered.
    session_->replaceDocument(pio::documentFromTrackData(pg::toTrackData(track, name.toStdString())),
                              QString());
    addPointsAction_->setChecked(false);
    statusBar()->showMessage(QString("Generated %1, %2 points").arg(name).arg(n), 4000);
  } catch (const std::exception& e) {
    statusBar()->showMessage(QString("Invalid preset: %1").arg(e.what()));
  }
}

QString MainWindow::openTrackFile(const QString& path) {
  pio::DocumentLoadResult r = pio::loadTrackDocumentFile(path.toStdString());
  if (!r.ok()) return QString::fromStdString(r.error);
  if (!r.document) {
    QString why;
    for (const auto& w : r.warnings) why += QString::fromStdString(w) + "\n";
    return "The track cannot be edited:\n" + why;
  }
  // A legacy (v1) file is imported; it keeps no file path so that Save cannot silently convert it in place.
  const bool legacy = r.schemaVersion == 1;
  session_->replaceDocument(std::move(*r.document), legacy ? QString() : path);
  addPointsAction_->setChecked(false);
  QString note;
  for (const std::string& w : r.warnings) note += QString::fromStdString(w) + "; ";
  statusBar()->showMessage(
      (legacy ? "Imported legacy track " + path + " (use Save As to store it as an editable document)"
              : "Opened " + path) +
          (note.isEmpty() ? "" : " — " + note),
      8000);
  return {};
}

QString MainWindow::saveTrackFile(const QString& path, bool overwrite) {
  const auto r =
      pio::saveTrackDocumentFile(path.toStdString(), session_->document(),
                                 overwrite ? pio::OverwritePolicy::Overwrite : pio::OverwritePolicy::Fail);
  if (!r.ok()) return QString::fromStdString(r.error);
  session_->setFilePath(path);
  session_->markSaved();
  setDiagnostic("File", path);
  updateTitle();
  statusBar()->showMessage("Saved " + path, 4000);
  return {};
}

void MainWindow::openTrack() {
  const QString path = QFileDialog::getOpenFileName(this, "Open Track", session_->filePath(),
                                                    "PathForge track (*.json);;All files (*)");
  if (path.isEmpty()) return;
  const QString error = openTrackFile(path);
  if (!error.isEmpty())
    QMessageBox::warning(this, "Cannot open track", error + "\n\nThe current track was not changed.");
}

bool MainWindow::saveTrackAs() {
  const QString path =
      QFileDialog::getSaveFileName(this, "Save Track As", session_->filePath(), "PathForge track (*.json)");
  if (path.isEmpty())
    return false;  // cancelled; the dialog already asked for confirmation when the file exists
  const QString error = saveTrackFile(path, true);
  if (!error.isEmpty()) {
    QMessageBox::warning(this, "Cannot save track", error);
    return false;
  }
  return true;
}

bool MainWindow::saveTrack() {
  if (session_->filePath().isEmpty()) return saveTrackAs();
  const QString error = saveTrackFile(session_->filePath(), true);
  if (!error.isEmpty()) {
    QMessageBox::warning(this, "Cannot save track", error);
    return false;
  }
  return true;
}

bool MainWindow::maybeSave() {
  if (!session_->isModified()) return true;
  QMessageBox::StandardButton choice;
  if (prompt_) {
    choice = prompt_();
  } else {
    choice = QMessageBox::warning(this, "Unsaved changes",
                                  QString("The track \"%1\" has unsaved changes.")
                                      .arg(QString::fromStdString(session_->document().name())),
                                  QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel,
                                  QMessageBox::Save);
  }
  if (choice == QMessageBox::Save) return saveTrack();  // a failed or cancelled save keeps the document open
  return choice == QMessageBox::Discard;
}

void MainWindow::closeEvent(QCloseEvent* event) {
  if (maybeSave()) {
    event->accept();
  } else {
    event->ignore();
  }
}
