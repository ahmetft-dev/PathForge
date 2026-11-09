// Qt widget tests for PathForge Studio (QtTest, offscreen platform). They drive the real MainWindow,
// EditorSession and TrackViewport through synthetic mouse/keyboard events and check the document, not
// screenshots.
#include <QAction>
#include <QApplication>
#include <QDir>
#include <QGraphicsLineItem>
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
