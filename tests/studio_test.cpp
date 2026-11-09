// Qt widget tests for PathForge Studio (QtTest, offscreen platform). They drive the real MainWindow,
// EditorSession and TrackViewport through synthetic mouse/keyboard events and check the document, not
// screenshots.
#include <QAction>
#include <QApplication>
#include <QDir>
#include <QDoubleSpinBox>
#include <QGraphicsLineItem>
#include <QGraphicsPathItem>
#include <QGraphicsScene>
#include <QMouseEvent>
#include <QPushButton>
#include <QTemporaryDir>
#include <QUndoStack>
#include <QtTest>
#include <functional>
#include <pathforge/editor/derived_geometry.hpp>
#include <pathforge/geometry/validation.hpp>
#include <pathforge/io/document_io.hpp>

#include "EditorSession.h"
#include "MainWindow.h"
#include "TrackViewport.h"

namespace pfe = pathforge::editor;
namespace pg = pathforge::geometry;

namespace {

void send(QWidget* w, QEvent::Type type, QPointF pos, Qt::MouseButton button, Qt::MouseButtons buttons,
          Qt::KeyboardModifiers mods = Qt::NoModifier) {
  QMouseEvent e(type, pos, w->mapToGlobal(pos), button, buttons, mods);
  QApplication::sendEvent(w, &e);
}

// Press at `from`, move in a few steps to `to`, release.
void drag(TrackViewport* v, QPointF from, QPointF to, Qt::KeyboardModifiers mods = Qt::NoModifier) {
  QWidget* vp = v->viewport();
  send(vp, QEvent::MouseButtonPress, from, Qt::LeftButton, Qt::LeftButton, mods);
  for (int i = 1; i <= 5; ++i) {
    send(vp, QEvent::MouseMove, from + (to - from) * (i / 5.0), Qt::NoButton, Qt::LeftButton, mods);
  }
  send(vp, QEvent::MouseButtonRelease, to, Qt::LeftButton, Qt::NoButton, mods);
}

void click(TrackViewport* v, QPointF at, Qt::KeyboardModifiers mods = Qt::NoModifier) {
  send(v->viewport(), QEvent::MouseButtonPress, at, Qt::LeftButton, Qt::LeftButton, mods);
  send(v->viewport(), QEvent::MouseButtonRelease, at, Qt::LeftButton, Qt::NoButton, mods);
}

QAction* action(MainWindow& w, const QString& text) {
  for (QAction* a : w.findChildren<QAction*>()) {
    if (a->text().remove('&').startsWith(text)) return a;
  }
  return nullptr;
}

QString dataFile(const char* name) { return QString(PATHFORGE_TEST_DATA_DIR) + "/" + name; }

// Name stored in a saved v2 file.
std::string pio_name(const QString& path) {
  const auto r = pathforge::io::loadTrackDocumentFile(path.toStdString());
  return r.document ? r.document->name() : std::string("<unreadable>");
}

}  // namespace

class StudioTest : public QObject {
  Q_OBJECT

 private slots:
  void presetStartsAsValidExactPolylineDocument() {
    MainWindow w;
    w.show();
    const auto& d = w.session()->document();
    QCOMPARE(d.size(), std::size_t{360});
    QCOMPARE(d.settings().interpolation, pfe::Interpolation::Polyline);
    QVERIFY(w.session()->derived().track.has_value());
    QVERIFY(w.session()->isReportCurrent());
    QCOMPARE(w.session()->report().status(), pg::ValidationStatus::Valid);
  }

  void createTrackFromScratchWithClicks() {
    MainWindow w;
    w.resize(1200, 800);
    w.show();
    action(w, "New Track")->trigger();
    EditorSession* s = w.session();
    TrackViewport* v = w.viewport();
    QVERIFY(s->document().empty());
    QVERIFY(v->addMode());
    QCOMPARE(s->derived().status, pfe::DerivedStatus::TooFewPoints);

    const QPointF clicks[] = {{300, 400}, {500, 250}, {700, 400}, {500, 600}};
    for (QPointF c : clicks) {
      const pg::Vec2 expect = v->viewportToWorld(c);
      click(v, c);
      // The point lands exactly where the click maps to in world coordinates.
      QCOMPARE(s->document().points().back().position.x, expect.x);
      QCOMPARE(s->document().points().back().position.y, expect.y);
      QCOMPARE(s->selection(), s->document().points().back().id);
    }
    QCOMPARE(s->document().size(), std::size_t{4});
    QCOMPARE(s->derived().status, pfe::DerivedStatus::Ok);  // open spline through 4 points
    QCOMPARE(s->document().closure(), pfe::Closure::Open);

    action(w, "Closed Loop")->trigger();
    QCOMPARE(s->document().closure(), pfe::Closure::Closed);
    QCOMPARE(s->derived().status, pfe::DerivedStatus::Ok);
    QCOMPARE(s->derived().data.closure, pfe::Closure::Closed);
    action(w, "Closed Loop")->trigger();  // reopen
    QCOMPARE(s->document().closure(), pfe::Closure::Open);
  }

  void selectionUsesScreenSpaceRadiusAtAnyZoom() {
    MainWindow w;
    w.show();
    TrackViewport* v = w.viewport();
    EditorSession* s = w.session();
    const pfe::ControlPoint p = s->document().points()[90];
    for (double zoom : {1.0, 8.0, 0.25}) {
      v->fitToView();
      v->zoomBy(zoom);
      v->focusOn(p.position.x, p.position.y);
      const QPointF at = v->worldToViewport(p.position);
      // An exact hit always selects that point (nearest wins); at low zoom neighbouring samples are < 8 px
      // apart, so offset hits are only unambiguous where points are well separated on screen.
      QCOMPARE(v->pointAt(at), p.id);
      if (zoom >= 8.0) {
        QCOMPARE(v->pointAt(at + QPointF(3, 3)), p.id);  // inside the 8 px hit radius
        QVERIFY(!v->pointAt(at + QPointF(30, 0)).has_value() || v->pointAt(at + QPointF(30, 0)) != p.id);
        QVERIFY(!v->pointAt(at + QPointF(0, -12)).has_value());  // outside the radius of every point
      }
      click(v, at);
      QCOMPARE(s->selection(), p.id);
      QCOMPARE(s->document().find(p.id)->position, p.position);  // a click must not move the point
    }
    // Clicking empty space deselects.
    v->fitToView();
    click(v, QPointF(5, 5));
    QVERIFY(!s->selection().has_value());
  }

  void dragMovesPointAndValidatesOnceAtTheEnd() {
    MainWindow w;
    w.show();
    TrackViewport* v = w.viewport();
    EditorSession* s = w.session();
    const pfe::ControlPoint p = s->document().points()[0];
    const QPointF from = v->worldToViewport(p.position);
    const QPointF to = from + QPointF(40, -25);
    const pg::Vec2 target = v->viewportToWorld(to);

    QWidget* vp = v->viewport();
    send(vp, QEvent::MouseButtonPress, from, Qt::LeftButton, Qt::LeftButton);
    send(vp, QEvent::MouseMove, from + QPointF(1, 1), Qt::NoButton,
         Qt::LeftButton);  // below the drag threshold
    QCOMPARE(s->document().find(p.id)->position, p.position);
    send(vp, QEvent::MouseMove, (from + to) / 2, Qt::NoButton, Qt::LeftButton);
    QVERIFY(s->isDragging());
    QVERIFY(!s->isReportCurrent());  // live preview, validation pending
    send(vp, QEvent::MouseMove, to, Qt::NoButton, Qt::LeftButton);
    send(vp, QEvent::MouseButtonRelease, to, Qt::LeftButton, Qt::NoButton);
    QVERIFY(!s->isDragging());
    QVERIFY(s->isReportCurrent());
    QCOMPARE(s->document().find(p.id)->position.x, target.x);
    QCOMPARE(s->document().find(p.id)->position.y, target.y);
    QCOMPARE(s->document().size(), std::size_t{360});
  }

  void panningDoesNotEditTheDocument() {
    MainWindow w;
    w.show();
    TrackViewport* v = w.viewport();
    const auto revision = w.session()->document().revision();
    const pg::Vec2 before = v->viewportToWorld(QPointF(100, 100));
    drag(v, QPointF(20, 20), QPointF(220, 120));  // starts on empty space (corner)
    const pg::Vec2 after = v->viewportToWorld(QPointF(100, 100));
    QVERIFY(std::abs(after.x - before.x) > 1e-6 || std::abs(after.y - before.y) > 1e-6);
    QCOMPARE(w.session()->document().revision(), revision);
    QVERIFY(!w.session()->selection().has_value());
  }

  void shiftClickInsertsOnTheNearestSegment() {
    MainWindow w;
    w.show();
    action(w, "New Track")->trigger();
    EditorSession* s = w.session();
    pfe::GeometrySettings settings;
    settings.interpolation = pfe::Interpolation::Polyline;
    s->setSettings(settings);
    s->appendPoint({0, 0});
    s->appendPoint({100, 0});
    s->appendPoint({200, 0});
    TrackViewport* v = w.viewport();
    v->fitToView();
    const pfe::PointId second = s->document().points()[1].id;
    click(v, v->worldToViewport({150, 0}) + QPointF(0, 4), Qt::ShiftModifier);
    QCOMPARE(s->document().size(), std::size_t{4});
    QCOMPARE(s->document().points()[2].id, *s->selection());  // inserted between the 2nd and 3rd point
    QCOMPARE(s->document().points()[1].id, second);
    QVERIFY(std::abs(s->document().points()[2].position.y) < 1e-9);  // placed ON the track
    // Far from the track nothing is inserted.
    click(v, v->worldToViewport({150, 80}), Qt::ShiftModifier);
    QCOMPARE(s->document().size(), std::size_t{4});
  }

  void deletingPointsDegradesGracefully() {
    MainWindow w;
    w.show();
    action(w, "New Track")->trigger();
    EditorSession* s = w.session();
    for (pg::Vec2 p : {pg::Vec2{0, 0}, pg::Vec2{50, 10}, pg::Vec2{100, 0}}) s->appendPoint(p);
    QAction* del = action(w, "Delete Point");
    for (int guard = 0; !s->document().empty() && guard < 10; ++guard) {  // bounded: never loop on a bug
      s->select(s->document().points().back().id);
      QVERIFY(del->isEnabled());
      del->trigger();
      QVERIFY(s->isReportCurrent());  // incomplete geometry is validated and explained, not hidden
    }
    QVERIFY(s->document().empty());
    QCOMPARE(s->derived().status, pfe::DerivedStatus::TooFewPoints);
    QVERIFY(!del->isEnabled());  // nothing selected
    del->trigger();              // no-op
    QVERIFY(s->document().empty());
  }

  void rejectsNonFiniteOrOutOfRangeDragTargets() {
    MainWindow w;
    w.show();
    EditorSession* s = w.session();
    const pfe::ControlPoint p = s->document().points()[3];
    QSignalSpy rejected(s, &EditorSession::editRejected);
    QVERIFY(s->beginDrag(p.id));
    s->dragTo({std::numeric_limits<double>::quiet_NaN(), 0.0});
    s->dragTo({5e7, 0.0});
    s->endDrag();
    QCOMPARE(rejected.count(), 2);
    QCOMPARE(s->document().find(p.id)->position, p.position);
  }

  void openSaveAndLegacyImport() {
    QTemporaryDir dir;
    QVERIFY(dir.isValid());
    MainWindow w;
    w.show();
    EditorSession* s = w.session();
    QVERIFY(w.openTrackFile(dataFile("editable_circuit_v2.json")).isEmpty());
    QCOMPARE(s->document().size(), std::size_t{8});
    QCOMPARE(s->filePath(), dataFile("editable_circuit_v2.json"));
    s->setPointWidths(s->document().points()[2].id, 7.0, 3.0);

    const QString out = dir.filePath("edited.json");
    QVERIFY(w.saveTrackFile(out, false).isEmpty());
    QVERIFY(!w.saveTrackFile(out, false).isEmpty());  // overwrite protection
    const auto saved = s->document();
    QVERIFY(w.openTrackFile(dataFile("valid_oval.json")).isEmpty());  // legacy v1
    QVERIFY(s->filePath().isEmpty());                                 // must use Save As
    QCOMPARE(s->document().settings().interpolation, pfe::Interpolation::Polyline);
    QVERIFY(w.openTrackFile(out).isEmpty());
    QCOMPARE(s->document().points()[2].leftWidth, 7.0);
    QCOMPARE(s->document().size(), saved.size());

    // Failed loads leave the current document untouched.
    const auto revision = s->document().revision();
    for (const char* bad : {"malformed.json", "wrong_version.json", "does_not_exist.json"}) {
      QVERIFY(!w.openTrackFile(dataFile(bad)).isEmpty());
    }
    QCOMPARE(s->document().revision(), revision);
    QCOMPARE(s->filePath(), out);
  }

  void invalidGeometryIsShownWithOverlays() {
    MainWindow w;
    w.show();
    QVERIFY(w.openTrackFile(dataFile("editable_bowtie_v2.json")).isEmpty());
    EditorSession* s = w.session();
    QVERIFY(s->isReportCurrent());
    QCOMPARE(s->report().status(), pg::ValidationStatus::Invalid);
    int red = 0;
    for (QGraphicsItem* item : w.viewport()->scene()->items()) {
      if (auto* line = dynamic_cast<QGraphicsLineItem*>(item);
          line && line->pen().color() == QColor(0xff, 0x52, 0x52)) {
        ++red;
      }
    }
    QVERIFY(red >= 2);  // the two crossing centerline segments at least
    // The invalid document can still be edited and saved (work in progress).
    QTemporaryDir dir;
    QVERIFY(w.saveTrackFile(dir.filePath("wip.json"), false).isEmpty());
  }

  // ---- Undo / redo
  // ---------------------------------------------------------------------------------------------

  static bool sameDocument(const pfe::TrackDocument& a, const pfe::TrackDocument& b) {
    return a.name() == b.name() && a.closure() == b.closure() && a.settings() == b.settings() &&
           a.points() == b.points();
  }

  void everyEditKindIsUndoableAndRedoable() {
    MainWindow w;
    w.show();
    action(w, "New Track")->trigger();
    EditorSession* s = w.session();
    QUndoStack* u = s->undoStack();
    pfe::GeometrySettings poly;
    poly.interpolation = pfe::Interpolation::Polyline;
    s->setSettings(poly);
    s->appendPoint({0, 0});
    s->appendPoint({100, 0});
    s->appendPoint({200, 50});
    QCOMPARE(u->count(), 4);

    struct Step {
      const char* name;
      std::function<void()> edit;
    };
    const pfe::PointId second = s->document().points()[1].id;
    const std::vector<Step> steps = {
        {"insert", [&] { QVERIFY(s->insertNear({50, 0}, 5.0).has_value()); }},
        {"move", [&] { QVERIFY(s->setPointPosition(second, {110, 12})); }},
        {"widths", [&] { QVERIFY(s->setPointWidths(second, 8.0, 3.0)); }},
        {"close", [&] { QVERIFY(s->setClosure(pfe::Closure::Closed)); }},
        {"settings",
         [&] {
           pfe::GeometrySettings g = s->document().settings();
           g.resampleSpacing = 5.0;
           QVERIFY(s->setSettings(g));
         }},
        {"rename", [&] { QVERIFY(s->setName("Renamed")); }},
        {"delete", [&] { QVERIFY(s->removePoint(second)); }},
    };
    for (const Step& step : steps) {
      const pfe::TrackDocument before = s->document();
      const int count = u->count();
      step.edit();
      QCOMPARE(u->count(), count + 1);  // exactly one history entry per user action
      const pfe::TrackDocument after = s->document();
      QVERIFY2(!sameDocument(before, after), step.name);
      u->undo();
      QVERIFY2(sameDocument(s->document(), before), step.name);
      QCOMPARE(s->derived().data.centerline, pfe::generateGeometry(before).data.centerline);
      QVERIFY(s->isReportCurrent());
      u->redo();
      QVERIFY2(sameDocument(s->document(), after), step.name);
    }
  }

  void multiStepSequencesRestoreStateAndRedoBranchIsDiscarded() {
    MainWindow w;
    w.show();
    action(w, "New Track")->trigger();
    EditorSession* s = w.session();
    QUndoStack* u = s->undoStack();
    std::vector<pfe::TrackDocument> states{s->document()};
    for (pg::Vec2 p : {pg::Vec2{0, 0}, pg::Vec2{80, 10}, pg::Vec2{160, -10}, pg::Vec2{240, 20}}) {
      s->appendPoint(p);
      states.push_back(s->document());
    }
    s->setPointPosition(s->document().points()[1].id, {85, 30});
    states.push_back(s->document());
    s->removePoint(s->document().points()[2].id);
    states.push_back(s->document());
    for (std::size_t i = states.size() - 1; i-- > 0;) {  // undo all the way back
      u->undo();
      QVERIFY2(sameDocument(s->document(), states[i]), qPrintable(QString::number(i)));
    }
    QVERIFY(!u->canUndo());
    for (std::size_t i = 1; i < states.size(); ++i) {  // and redo all the way forward
      u->redo();
      QVERIFY2(sameDocument(s->document(), states[i]), qPrintable(QString::number(i)));
    }
    u->undo();
    u->undo();
    s->appendPoint({500, 500});  // a new edit after undo discards the redo branch
    QVERIFY(!u->canRedo());
  }

  void deletedPointComesBackWithTheSameIdAndSelection() {
    MainWindow w;
    w.show();
    EditorSession* s = w.session();
    const pfe::ControlPoint p = s->document().points()[5];
    s->select(p.id);
    action(w, "Delete Point")->trigger();
    QVERIFY(!s->document().find(p.id));
    QVERIFY(!s->selection().has_value());
    s->undoStack()->undo();
    QCOMPARE(s->document().points()[5], p);  // same id, position, widths and index
    QCOMPARE(s->selection(), p.id);
  }

  void aDragIsExactlyOneUndoableStep() {
    MainWindow w;
    w.show();
    TrackViewport* v = w.viewport();
    EditorSession* s = w.session();
    const pfe::ControlPoint p = s->document().points()[0];
    const int before = s->undoStack()->count();
    const QPointF from = v->worldToViewport(p.position);
    QWidget* vp = v->viewport();
    send(vp, QEvent::MouseButtonPress, from, Qt::LeftButton, Qt::LeftButton);
    for (int i = 1; i <= 100; ++i) {  // a hundred mouse-move events
      send(vp, QEvent::MouseMove, from + QPointF(i * 0.4, -i * 0.25), Qt::NoButton, Qt::LeftButton);
    }
    QCOMPARE(s->undoStack()->count(), before);  // nothing recorded while dragging
    send(vp, QEvent::MouseButtonRelease, from + QPointF(40, -25), Qt::LeftButton, Qt::NoButton);
    QCOMPARE(s->undoStack()->count(), before + 1);
    const pg::Vec2 moved = s->document().find(p.id)->position;
    QVERIFY(moved != p.position);
    s->undoStack()->undo();
    QCOMPARE(s->document().find(p.id)->position, p.position);  // one undo returns to the start of the drag
    s->undoStack()->redo();
    QCOMPARE(s->document().find(p.id)->position, moved);
    // A press without movement is a click: no history entry, no change.
    const int n = s->undoStack()->count();
    click(v, v->worldToViewport(moved));
    QCOMPARE(s->undoStack()->count(), n);
  }

  void consecutiveWidthEditsOfOnePointMergeIntoOneStep() {
    MainWindow w;
    w.show();
    EditorSession* s = w.session();
    const pfe::ControlPoint p = s->document().points()[2];
    const int before = s->undoStack()->count();
    for (double l : {5.5, 6.0, 6.5, 7.0}) s->setPointWidths(p.id, l, p.rightWidth);
    QCOMPARE(s->undoStack()->count(), before + 1);
    QCOMPARE(s->document().find(p.id)->leftWidth, 7.0);
    s->undoStack()->undo();
    QCOMPARE(s->document().find(p.id)->leftWidth, p.leftWidth);
    QVERIFY(!s->setPointWidths(p.id, -1.0, 5.0));  // rejected input is not recorded
    QCOMPARE(s->undoStack()->count(), before + 1);
  }

  void modifiedStateTracksUnsavedChanges() {
    QTemporaryDir dir;
    MainWindow w;
    w.show();
    EditorSession* s = w.session();
    QVERIFY(!s->isModified());
    QVERIFY(!w.isWindowModified());
    QVERIFY(w.openTrackFile(dataFile("editable_circuit_v2.json")).isEmpty());
    QVERIFY(!s->isModified());  // loading is not a modification
    s->setPointWidths(s->document().points()[0].id, 9.0, 9.0);
    QVERIFY(s->isModified());
    QVERIFY(w.isWindowModified());
    QVERIFY(w.windowTitle().contains("[*]"));
    const QString out = dir.filePath("saved.json");
    QVERIFY(w.saveTrackFile(out, false).isEmpty());
    QVERIFY(!s->isModified());  // saved
    QVERIFY(!w.isWindowModified());
    s->undoStack()->undo();  // moving away from the saved state ...
    QVERIFY(s->isModified());
    s->undoStack()->redo();  // ... and back to it
    QVERIFY(!s->isModified());
  }

  void unsavedChangesPromptOffersSaveDiscardCancel() {
    QTemporaryDir dir;
    MainWindow w;
    w.show();
    EditorSession* s = w.session();
    QVERIFY(w.openTrackFile(dataFile("editable_circuit_v2.json")).isEmpty());
    const QString mine = dir.filePath("mine.json");
    QVERIFY(w.saveTrackFile(mine, false).isEmpty());  // now backed by a writable file
    s->setName("edited");
    int prompts = 0;
    QMessageBox::StandardButton answer = QMessageBox::Cancel;
    w.setUnsavedPrompt([&] {
      ++prompts;
      return answer;
    });

    action(w, "New Track")->trigger();  // Cancel: nothing happens
    QCOMPARE(prompts, 1);
    QCOMPARE(s->document().name(), std::string("edited"));
    QVERIFY(s->isModified());

    answer = QMessageBox::Save;  // Save: file written, then the document is replaced
    action(w, "New Track")->trigger();
    QCOMPARE(prompts, 2);
    QVERIFY(s->document().empty());
    QCOMPARE(pio_name(mine), std::string("edited"));  // the edit reached the file

    s->appendPoint({1, 1});
    answer = QMessageBox::Discard;  // Discard: replaced without saving
    w.findChild<QPushButton*>()->click();
    QCOMPARE(prompts, 3);
    QCOMPARE(s->document().size(), std::size_t{360});

    // An unmodified document never prompts.
    action(w, "New Track")->trigger();
    QCOMPARE(prompts, 3);
  }

  void closingWithUnsavedChangesCanBeCancelled() {
    MainWindow w;
    w.show();
    w.session()->appendPoint({10, 10});
    QMessageBox::StandardButton answer = QMessageBox::Cancel;
    w.setUnsavedPrompt([&] { return answer; });
    QVERIFY(!w.close());
    QVERIFY(w.isVisible());
    answer = QMessageBox::Discard;
    QVERIFY(w.close());
  }

  void standardShortcutsAreBoundToTheEditingActions() {
    MainWindow w;
    w.show();
    QVERIFY(action(w, "Undo")->shortcuts().contains(QKeySequence(QKeySequence::Undo)));
    QVERIFY(action(w, "Redo")->shortcuts().contains(QKeySequence(QKeySequence::Redo)));
    QVERIFY(action(w, "Save Track")->shortcuts().contains(QKeySequence(QKeySequence::Save)));
    QVERIFY(action(w, "Open Track")->shortcuts().contains(QKeySequence(QKeySequence::Open)));
    QVERIFY(action(w, "Delete Point")->shortcuts().contains(QKeySequence(QKeySequence::Delete)));
    // Real key delivery (needs an active window; the offscreen platform provides one).
    w.activateWindow();
    if (!QTest::qWaitForWindowActive(&w, 1000)) QSKIP("window cannot be activated on this platform");
    EditorSession* s = w.session();
    const std::size_t n = s->document().size();
    s->appendPoint({123, 45});
    QCOMPARE(s->document().size(), n + 1);
    QTest::keySequence(&w, QKeySequence::Undo);
    QCOMPARE(s->document().size(), n);
    QTest::keySequence(&w, QKeySequence::Redo);
    QCOMPARE(s->document().size(), n + 1);
  }

  // ---- Width editing
  // -------------------------------------------------------------------------------------------

  static QDoubleSpinBox* spin(MainWindow& w, const char* name) { return w.findChild<QDoubleSpinBox*>(name); }

  void pointPanelShowsAndEditsTheSelectedPoint() {
    MainWindow w;
    w.show();
    EditorSession* s = w.session();
    QVERIFY(!spin(w, "pointLeft")->isEnabled());  // nothing selected
    const pfe::ControlPoint p = s->document().points()[12];
    s->select(p.id);
    QVERIFY(spin(w, "pointLeft")->isEnabled());
    QCOMPARE(spin(w, "pointX")->value(), std::round(p.position.x * 1000) / 1000);
    QCOMPARE(spin(w, "pointLeft")->value(), 5.0);
    QCOMPARE(spin(w, "pointRight")->value(), 5.0);

    // Position typed into the panel moves the point as ONE undoable step.
    const int n = s->undoStack()->count();
    spin(w, "pointX")->setValue(12.5);
    spin(w, "pointY")->setValue(-40.25);
    QCOMPARE(s->document().find(p.id)->position, (pg::Vec2{12.5, -40.25}));
    QCOMPARE(s->undoStack()->count(), n + 2);  // one per committed field
    s->undoStack()->undo();
    s->undoStack()->undo();
    QCOMPARE(s->document().find(p.id)->position, p.position);
  }

  void leftAndRightWidthsAreIndependentAndUpdateBoundaries() {
    MainWindow w;
    w.show();
    EditorSession* s = w.session();
    const std::size_t i = 10;
    s->select(s->document().points()[i].id);
    spin(w, "pointLeft")->setValue(8.0);
    const pfe::ControlPoint p = s->document().points()[i];
    QCOMPARE(p.leftWidth, 8.0);
    QCOMPARE(p.rightWidth, 5.0);  // untouched
    // Preset circle R = 100, CCW: left boundary is inside (R - w), right boundary outside (R + w).
    const auto& track = *s->derived().track;
    QVERIFY(std::abs(pg::norm(track.leftBoundary()[i]) - (100.0 - 8.0)) < 1e-9);
    QVERIFY(std::abs(pg::norm(track.rightBoundary()[i]) - (100.0 + 5.0)) < 1e-9);
    spin(w, "pointRight")->setValue(2.5);
    QVERIFY(std::abs(pg::norm(s->derived().track->rightBoundary()[i]) - (100.0 + 2.5)) < 1e-9);
    QVERIFY(std::abs(pg::norm(s->derived().track->leftBoundary()[i]) - (100.0 - 8.0)) < 1e-9);
    // Neighbouring samples keep their original width: the profile is variable, not constant.
    QVERIFY(!s->derived().track->hasConstantWidths());
    QCOMPARE(s->derived().data.leftWidths[i - 1], 5.0);
    QCOMPARE(s->derived().data.leftWidths[i + 1], 5.0);
    QVERIFY(s->isReportCurrent());
    QCOMPARE(s->report().status(), pg::ValidationStatus::Valid);
  }

  void applyAllMakesAConstantWidthTrackInOneUndoStep() {
    MainWindow w;
    w.show();
    EditorSession* s = w.session();
    s->select(s->document().points()[3].id);
    spin(w, "pointLeft")->setValue(9.0);
    QVERIFY(!s->derived().track->hasConstantWidths());
    const int n = s->undoStack()->count();
    spin(w, "pointRight")->setValue(4.0);
    w.findChild<QPushButton*>("applyAllWidths")->click();
    for (const auto& p : s->document().points()) {
      QCOMPARE(p.leftWidth, 9.0);
      QCOMPARE(p.rightWidth, 4.0);
    }
    QVERIFY(s->derived().track->hasConstantWidths());
    s->undoStack()->undo();  // the whole "apply to all" is one step
    QCOMPARE(s->document().points()[0].leftWidth, 5.0);
    QCOMPARE(s->document().points()[3].leftWidth, 9.0);
    QCOMPARE(s->undoStack()->count(), n + 1);  // the right-width edit merged into the left-width step
  }

  void widthsStayAlignedWhenPointsAreInsertedAndDeleted() {
    MainWindow w;
    w.show();
    action(w, "New Track")->trigger();
    EditorSession* s = w.session();
    pfe::GeometrySettings poly;
    poly.interpolation = pfe::Interpolation::Polyline;
    poly.resampleSpacing = 5.0;
    s->setSettings(poly);
    s->setDefaultWidths(4.0, 2.0);
    const pfe::PointId a = *s->appendPoint({0, 0});
    s->setDefaultWidths(8.0, 6.0);
    const pfe::PointId b = *s->appendPoint({100, 0});
    s->setDefaultWidths(5.0, 5.0);
    const pfe::PointId c = *s->appendPoint({200, 0});
    const auto mid = s->insertNear({50, 1}, 5.0);  // halfway between a and b
    QVERIFY(mid.has_value());
    QCOMPARE(s->document().points()[1].id, *mid);
    QVERIFY(std::abs(s->document().find(*mid)->leftWidth - 6.0) < 1e-9);   // (4 + 8) / 2
    QVERIFY(std::abs(s->document().find(*mid)->rightWidth - 4.0) < 1e-9);  // (2 + 6) / 2
    // Every other point keeps its own widths by identity, not by index.
    QCOMPARE(s->document().find(a)->leftWidth, 4.0);
    QCOMPARE(s->document().find(b)->leftWidth, 8.0);
    QCOMPARE(s->document().find(c)->leftWidth, 5.0);
    s->removePoint(b);
    QCOMPARE(s->document().find(a)->leftWidth, 4.0);
    QCOMPARE(s->document().find(c)->leftWidth, 5.0);
    QCOMPARE(s->document().size(), std::size_t{3});
    // Derived widths interpolate between the surviving control points in arc length.
    const auto& g = s->derived();
    QCOMPARE(g.data.leftWidths.size(), g.data.centerline.size());
    QVERIFY(std::abs(g.data.leftWidths.front() - 4.0) < 1e-9);
    QVERIFY(std::abs(g.data.leftWidths.back() - 5.0) < 1e-9);
  }

  void invalidWidthsCannotBeEnteredAndZeroIsReportedAsAnError() {
    MainWindow w;
    w.show();
    EditorSession* s = w.session();
    s->select(s->document().points()[5].id);
    QDoubleSpinBox* left = spin(w, "pointLeft");
    QCOMPARE(left->minimum(), 0.0);  // the editor never produces negative widths
    left->setValue(-3.0);            // clamped by the control to its minimum
    QVERIFY(s->document().points()[5].leftWidth >= 0.0);
    left->setValue(0.0);  // zero is representable (repairable) but invalid
    QVERIFY(s->isReportCurrent());
    QCOMPARE(s->report().status(), pg::ValidationStatus::Invalid);
    bool zeroWidthIssue = false;
    for (const auto& i : s->report().issues)
      zeroWidthIssue = zeroWidthIssue || i.type == pg::TrackIssue::InvalidWidth;
    QVERIFY(zeroWidthIssue);
    left->setValue(5.0);  // repaired
    QCOMPARE(s->report().status(), pg::ValidationStatus::Valid);
  }

  // ---- Level of detail
  // ----------------------------------------------------------------------------------------

  static QGraphicsPathItem* centerlineItem(TrackViewport* v) {
    for (QGraphicsItem* item : v->scene()->items()) {
      if (auto* p = dynamic_cast<QGraphicsPathItem*>(item); p && p->pen().color() == QColor(0x4f, 0xc3, 0xf7))
        return p;
    }
    return nullptr;
  }

  static pfe::TrackDocument denseEllipse(int n) {
    pfe::TrackDocument d;
    d.setClosure(pfe::Closure::Closed);
    pfe::GeometrySettings g;
    g.interpolation = pfe::Interpolation::Polyline;
    g.resampleSpacing = 0.0;
    d.setSettings(g);
    for (int i = 0; i < n; ++i) {
      const double t = 2.0 * M_PI * i / n;
      d.appendPoint({2000.0 * std::cos(t), 1200.0 * std::sin(t)}, 5.0, 5.0);
    }
    return d;
  }

  void drawnPolylinesAreDecimatedToOnePixelWithoutLosingShape() {
    MainWindow w;
    w.resize(1200, 800);
    w.show();
    EditorSession* s = w.session();
    s->setAsyncThreshold(100000);
    s->replaceDocument(denseEllipse(6000), QString());
    TrackViewport* v = w.viewport();
    v->fitToView();
    QApplication::processEvents();
    const QGraphicsPathItem* item = centerlineItem(v);
    QVERIFY(item);
    const QPainterPath path = item->path();
    const int drawn = path.elementCount();
    QVERIFY2(drawn < 6000 / 3,
             qPrintable(QString("drawn vertices: %1").arg(drawn)));  // far fewer than the samples
    QVERIFY(drawn > 50);
    const auto& samples = s->derived().data.centerline;
    // First and last samples are always drawn.
    QCOMPARE(path.elementAt(0).x, samples.front().x);
    QCOMPARE(-path.elementAt(0).y, samples.front().y);
    // (QPainterPath::closeSubpath() appends a final line back to the start, so the last SAMPLE is the element
    // before.)
    QCOMPARE(path.elementAt(drawn - 2).x, samples.back().x);
    QCOMPARE(-path.elementAt(drawn - 2).y, samples.back().y);
    QCOMPARE(path.elementAt(drawn - 1).x, samples.front().x);
    // Every original vertex lies within one screen pixel of a drawn vertex (so the picture is the same to a
    // pixel).
    const double onePixel = 1.0 / v->pixelsPerMeter();
    double worst = 0.0;
    for (const pg::Vec2& p : samples) {
      double best = 1e300;
      for (int i = 0; i < drawn; ++i) {
        best = std::min(best, std::hypot(p.x - path.elementAt(i).x, p.y + path.elementAt(i).y));
      }
      worst = std::max(worst, best);
    }
    QVERIFY2(worst <= onePixel * 1.0001, qPrintable(QString("worst %1 px").arg(worst / onePixel)));

    // Zooming in restores detail (the drawing is rebuilt for the new scale), and fitting again decimates
    // again.
    v->zoomBy(40.0);
    QApplication::processEvents();
    QVERIFY(centerlineItem(v)->path().elementCount() > drawn * 2);
    v->fitToView();
    QCOMPARE(centerlineItem(v)->path().elementCount(), drawn);
    // The data itself is never decimated: validation and overlays use all 6000 samples.
    QCOMPARE(s->derived().data.centerline.size(), std::size_t{6000});
  }

  void denseControlHandlesAreThinnedButEveryPointStaysSelectable() {
    MainWindow w;
    w.resize(1200, 800);
    w.show();
    EditorSession* s = w.session();
    s->setAsyncThreshold(100000);
    s->replaceDocument(denseEllipse(4000), QString());
    TrackViewport* v = w.viewport();
    v->fitToView();
    QApplication::processEvents();
    int handles = 0;
    for (QGraphicsItem* item : v->scene()->items()) {
      if (item->flags().testFlag(QGraphicsItem::ItemIgnoresTransformations) && item->zValue() >= 10)
        ++handles;
    }
    QVERIFY2(handles < 4000 / 3, qPrintable(QString("handles drawn: %1").arg(handles)));
    QVERIFY(handles >= 10);
    // Selection still works for any point, and a selected point is always drawn.
    const pfe::ControlPoint p = s->document().points()[1234];
    s->select(p.id);
    QApplication::processEvents();
    bool selectedDrawn = false;
    for (QGraphicsItem* item : v->scene()->items()) {
      if (item->zValue() == 11 && item->pos() == QPointF(p.position.x, -p.position.y)) selectedDrawn = true;
    }
    QVERIFY(selectedDrawn);
    QCOMPARE(v->pointAt(v->worldToViewport(p.position)), p.id);
  }

  void zoomFitAndResizeStillWork() {
    MainWindow w;
    w.resize(1200, 800);
    w.show();
    TrackViewport* v = w.viewport();
    v->fitToView();
    const double fit = v->pixelsPerMeter();
    v->zoomBy(2.0);
    QVERIFY(std::abs(v->pixelsPerMeter() - 2 * fit) < 1e-9 * fit);
    v->fitToView();
    QVERIFY(std::abs(v->pixelsPerMeter() - fit) < 1e-9 * fit);
    w.resize(800, 500);
    QApplication::processEvents();
    QVERIFY(v->pixelsPerMeter() < fit);  // auto-fit on resize while the user has not moved the view
  }
};

QTEST_MAIN(StudioTest)
#include "studio_test.moc"
