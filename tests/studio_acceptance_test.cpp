// Phase 1D acceptance scenarios A-E for PathForge Studio, written as an AUTOMATED harness: they drive the
// real MainWindow, EditorSession and TrackViewport with synthetic mouse events (QApplication::sendEvent) and
// read the document and the widgets back. They run on whatever Qt platform plugin is active: offscreen under
// CTest, and the native Cocoa plugin when launched with QT_QPA_PLATFORM=cocoa (a real NSWindow is created and
// shown). They are NOT manual human testing: no person looks at the screen, and OS-level input (a physical
// mouse, window-server focus, trackpad gestures, GPU compositing) is not exercised.
#include <QAction>
#include <QApplication>
#include <QComboBox>
#include <QElapsedTimer>
#include <QGraphicsPolygonItem>
#include <QGraphicsScene>
#include <QLabel>
#include <QListWidget>
#include <QMouseEvent>
#include <QTemporaryDir>
#include <QTimer>
#include <QUndoStack>
#include <QtTest>
#include <cmath>
#include <numbers>
#include <pathforge/editor/derived_geometry.hpp>
#include <pathforge/geometry/cubic_spline.hpp>
#include <pathforge/geometry/spline.hpp>
#include <pathforge/geometry/validation.hpp>

#include "EditorSession.h"
#include "MainWindow.h"
#include "TrackViewport.h"

namespace pfe = pathforge::editor;
namespace pg = pathforge::geometry;

namespace {

void send(QWidget* w, QEvent::Type type, QPointF pos, Qt::MouseButton button, Qt::MouseButtons buttons) {
  QMouseEvent e(type, pos, w->mapToGlobal(pos), button, buttons, Qt::NoModifier);
  QApplication::sendEvent(w, &e);
}
void click(TrackViewport* v, QPointF at) {
  send(v->viewport(), QEvent::MouseButtonPress, at, Qt::LeftButton, Qt::LeftButton);
  send(v->viewport(), QEvent::MouseButtonRelease, at, Qt::LeftButton, Qt::NoButton);
}
void drag(TrackViewport* v, QPointF from, QPointF to) {
  QWidget* vp = v->viewport();
  send(vp, QEvent::MouseButtonPress, from, Qt::LeftButton, Qt::LeftButton);
  for (int i = 1; i <= 6; ++i)
    send(vp, QEvent::MouseMove, from + (to - from) * (i / 6.0), Qt::NoButton, Qt::LeftButton);
  send(vp, QEvent::MouseButtonRelease, to, Qt::LeftButton, Qt::NoButton);
}
QAction* action(MainWindow& w, const QString& text) {
  for (QAction* a : w.findChildren<QAction*>()) {
    if (a->text().remove('&').startsWith(text)) return a;
  }
  return nullptr;
}
QComboBox* interpolationCombo(MainWindow& w) { return w.findChild<QComboBox*>("interpolation"); }
QString dataFile(const char* name) { return QString(PATHFORGE_TEST_DATA_DIR) + "/" + name; }

int countItems(MainWindow& w, bool polygons) {
  int n = 0;
  for (QGraphicsItem* item : w.viewport()->scene()->items()) {
    if (polygons) {
      n += dynamic_cast<QGraphicsPolygonItem*>(item) != nullptr;
    } else if (auto* l = dynamic_cast<QGraphicsLineItem*>(item);
               l && l->pen().color() == QColor(0xff, 0x52, 0x52)) {
      ++n;
    }
  }
  return n;
}

// Clicks `points` (viewport pixels) into a fresh closed track and returns nothing; add mode is left on by New
// Track.
void drawClosedTrack(MainWindow& w, const std::vector<QPointF>& pixels) {
  action(w, "New Track")->trigger();
  for (QPointF p : pixels) click(w.viewport(), p);
  if (w.session()->document().closure() != pfe::Closure::Closed) action(w, "Closed Loop")->trigger();
}

const std::vector<QPointF> kOval = {{300, 400}, {400, 280}, {600, 250}, {800, 280},
                                    {900, 400}, {800, 520}, {600, 550}, {400, 520}};

// Spread of the analytic curvature of the current centerline's own control polygon under each interpolation.
struct Spread {
  double minK, maxK;
};
Spread analyticSpread(const pfe::TrackDocument& d) {
  std::vector<pg::Vec2> c;
  for (const auto& p : d.points()) c.push_back(p.position);
  const pg::SplineOptions o{16, d.settings().alpha};
  const std::vector<double> k = d.settings().interpolation == pfe::Interpolation::CubicC2
                                    ? pg::cubicSplineCurvature(c, d.closure(), o)
                                    : pg::catmullRomCurvature(c, d.closure(), o);
  Spread s{1e300, -1e300};
  for (double v : k) {
    if (!std::isfinite(v)) continue;
    s.minK = std::min(s.minK, v);
    s.maxK = std::max(s.maxK, v);
  }
  return s;
}

}  // namespace

class StudioAcceptanceTest : public QObject {
  Q_OBJECT

 private slots:
  void initTestCase() { qInfo() << "Qt platform plugin:" << QGuiApplication::platformName(); }

  // A. New C2 track: draw, close, switch to the C2 spline, edit, undo/redo, save and reload.
  void testA_newC2Track() {
    QTemporaryDir dir;
    MainWindow w;
    w.resize(1200, 800);
    w.show();
    QVERIFY(QTest::qWaitForWindowExposed(&w));
    drawClosedTrack(w, kOval);
    EditorSession* s = w.session();
    TrackViewport* v = w.viewport();
    QCOMPARE(s->document().size(), std::size_t{8});
    QComboBox* combo = interpolationCombo(w);
    QVERIFY(combo);
    combo->setCurrentIndex(1);  // Cubic spline (C2)
    QCOMPARE(s->document().settings().interpolation, pfe::Interpolation::CubicC2);
    QVERIFY(s->isReportCurrent());
    QCOMPARE(s->report().status(), pg::ValidationStatus::Valid);
    QVERIFY(s->derived().splineHealth.checked);
    QCOMPARE(s->derived().splineHealth.degenerateSamples, std::size_t{0});
    QVERIFY(s->report().checked.surface);

    // Edit with real mouse drags: move a point, insert on the track with shift-click is covered elsewhere.
    action(w, "Add Points")->setChecked(false);
    const auto id = s->document().points()[2].id;
    const QPointF from = v->worldToViewport(s->document().points()[2].position);
    drag(v, from, from + QPointF(30, -20));
    QVERIFY(s->isReportCurrent());
    QCOMPARE(s->report().status(), pg::ValidationStatus::Valid);
    const pg::Vec2 moved = s->document().find(id)->position;
    s->undoStack()->undo();
    QVERIFY(s->document().find(id)->position != moved);
    s->undoStack()->redo();
    QCOMPARE(s->document().find(id)->position.x, moved.x);

    const QString file = dir.filePath("a.json");
    QVERIFY(w.saveTrackFile(file, false).isEmpty());
    const auto before = s->derived().data.centerline;
    action(w, "New Track")->trigger();
    QVERIFY(w.openTrackFile(file).isEmpty());
    QCOMPARE(s->document().settings().interpolation, pfe::Interpolation::CubicC2);
    QCOMPARE(interpolationCombo(w)->currentIndex(), 1);
    QCOMPARE(s->derived().data.centerline, before);
    QCOMPARE(s->report().status(), pg::ValidationStatus::Valid);
  }

  // B. Catmull-Rom vs C2 on the same control points; the UI must not present either as superior.
  void testB_compareInterpolations() {
    MainWindow w;
    w.resize(1200, 800);
    w.show();
    QVERIFY(QTest::qWaitForWindowExposed(&w));
    drawClosedTrack(w, kOval);
    EditorSession* s = w.session();
    QComboBox* combo = interpolationCombo(w);
    QCOMPARE(combo->currentIndex(), 0);  // Catmull-Rom is the unchanged default
    const auto controls = s->document().points();
    const auto cr = s->derived().data.centerline;
    const Spread crK = analyticSpread(s->document());
    combo->setCurrentIndex(1);
    const auto c2 = s->derived().data.centerline;
    const Spread c2K = analyticSpread(s->document());
    QVERIFY(cr != c2);
    QCOMPARE(s->document().points(), controls);  // the control points are untouched by the switch
    QCOMPARE(s->report().status(), pg::ValidationStatus::Valid);
    qInfo().nospace() << "B: analytic curvature range Catmull-Rom [" << crK.minK << ", " << crK.maxK
                      << "] 1/m, C2 [" << c2K.minK << ", " << c2K.maxK << "] 1/m";
    combo->setCurrentIndex(0);
    QCOMPARE(s->derived().data.centerline, cr);  // switching back restores the exact Catmull-Rom geometry
    s->undoStack()->undo();                      // undo the switch back to Catmull-Rom -> C2 again
    QCOMPARE(s->document().settings().interpolation, pfe::Interpolation::CubicC2);
    QCOMPARE(s->derived().data.centerline, c2);

    // No text anywhere in the window declares a winner.
    for (QLabel* l : w.findChildren<QLabel*>()) {
      const QString t = l->text().toLower();
      for (const char* word : {"better", "best", "superior", "recommended", "optimal"}) {
        QVERIFY2(!t.contains(word), qPrintable("label claims superiority: " + l->text()));
      }
    }
    for (int i = 0; i < combo->count(); ++i) {
      const QString t = combo->itemText(i).toLower();
      QVERIFY(!t.contains("better") && !t.contains("best") && !t.contains("recommended"));
    }
  }

  // C. Self-overlapping geometry: highlighted, still editable and savable, repaired by dragging.
  void testC_invalidThenRepaired() {
    QTemporaryDir dir;
    MainWindow w;
    w.resize(1200, 800);
    w.show();
    QVERIFY(QTest::qWaitForWindowExposed(&w));
    // A bow-tie: A(300,300) B(700,300) C(300,500) D(700,500) closed.
    drawClosedTrack(w, {{300, 300}, {700, 300}, {300, 500}, {700, 500}});
    EditorSession* s = w.session();
    TrackViewport* v = w.viewport();
    interpolationCombo(w)->setCurrentIndex(1);
    QVERIFY(s->isReportCurrent());
    QCOMPARE(s->report().status(), pg::ValidationStatus::Invalid);
    QVERIFY(s->report().errorCount > 0);
    QVERIFY(countItems(w, false) + countItems(w, true) > 0);  // highlighted in the viewport
    auto* list = w.findChild<QListWidget*>();
    QVERIFY(list && list->count() > 0);  // and listed
    QVERIFY(list->item(0)->text().size() > 5);

    // Still editable and savable while invalid.
    QVERIFY(w.saveTrackFile(dir.filePath("wip.json"), false).isEmpty());
    action(w, "Add Points")->setChecked(false);
    const auto pts = s->document().points();  // A B C D
    const QPointF cPix = v->worldToViewport(pts[2].position), dPix = v->worldToViewport(pts[3].position);
    drag(v, cPix, dPix + QPointF(0, 0.0));  // C onto D: coincident points -> generation fails, still editable
    QVERIFY(s->isReportCurrent());
    QVERIFY(s->report().status() != pg::ValidationStatus::Valid);
    const QPointF d2 = v->worldToViewport(s->document().points()[3].position);
    drag(v, d2, cPix);  // D to C's old place: now A B D' C' is a clean quadrilateral
    QVERIFY(s->isReportCurrent());
    QCOMPARE(s->report().status(), pg::ValidationStatus::Valid);
    QCOMPARE(countItems(w, true), 0);
    QCOMPARE(countItems(w, false), 0);
    s->undoStack()->undo();
    s->undoStack()->undo();
    QCOMPARE(s->report().status(),
             pg::ValidationStatus::Invalid);  // undo restores the defect AND its diagnostics
    QVERIFY(w.saveTrackFile(dir.filePath("again.json"), false).isEmpty());
  }

  // D. Persistence: C2 and Catmull-Rom documents, legacy v1 and v2 files.
  void testD_persistence() {
    QTemporaryDir dir;
    MainWindow w;
    w.show();
    QVERIFY(QTest::qWaitForWindowExposed(&w));
    EditorSession* s = w.session();
    // Legacy sampled v1 file opens as a polyline document with its status unchanged.
    QVERIFY(w.openTrackFile(dataFile("valid_oval.json")).isEmpty());
    QCOMPARE(s->document().settings().interpolation, pfe::Interpolation::Polyline);
    QCOMPARE(s->report().status(), pg::ValidationStatus::Valid);
    QVERIFY(w.openTrackFile(dataFile("bowtie.json")).isEmpty());
    QCOMPARE(s->report().status(), pg::ValidationStatus::Invalid);
    // v2 files of every interpolation round-trip with identical geometry.
    for (const char* f : {"editable_circuit_v2.json", "editable_c2_v2.json"}) {
      QVERIFY(w.openTrackFile(dataFile(f)).isEmpty());
      const auto interp = s->document().settings().interpolation;
      const auto geometry = s->derived().data.centerline;
      const QString out = dir.filePath(QString("rt_") + f);
      QVERIFY(w.saveTrackFile(out, false).isEmpty());
      QVERIFY(w.openTrackFile(dataFile("valid_oval.json")).isEmpty());
      QVERIFY(w.openTrackFile(out).isEmpty());
      QCOMPARE(s->document().settings().interpolation, interp);
      QCOMPARE(s->derived().data.centerline, geometry);
      QCOMPARE(s->report().status(), pg::ValidationStatus::Valid);
    }
    QVERIFY(w.openTrackFile(dataFile("editable_bowtie_v2.json")).isEmpty());  // invalid but loadable
    QCOMPARE(s->report().status(), pg::ValidationStatus::Invalid);
    QVERIFY(!w.openTrackFile(dataFile("wrong_version.json")).isEmpty());  // clear refusal, document unchanged
    QCOMPARE(s->report().status(), pg::ValidationStatus::Invalid);
  }

  // E. Responsiveness with a large C2 track: the GUI thread keeps running while validation is in the
  // background, edits stay cheap, rapid undo/redo never leaves a stale report as current.
  void testE_responsivenessAndStaleValidation() {
    MainWindow w;
    w.resize(1280, 800);
    w.show();
    QVERIFY(QTest::qWaitForWindowExposed(&w));
    EditorSession* s = w.session();
    TrackViewport* v = w.viewport();
    pfe::TrackDocument d;
    d.setClosure(pfe::Closure::Closed);
    pfe::GeometrySettings gs;
    gs.interpolation = pfe::Interpolation::CubicC2;
    gs.resampleSpacing = 1.0;
    d.setSettings(gs);
    for (int i = 0; i < 24; ++i) {
      const double a = 2.0 * std::numbers::pi * i / 24.0;
      d.appendPoint({2000.0 * std::cos(a), 1200.0 * std::sin(a)}, 5.0, 5.0);
    }
    // Event-loop gap monitor: a 5 ms timer; the largest gap between ticks is the longest GUI stall.
    QElapsedTimer clock;
    clock.start();
    qint64 last = 0, maxGap = 0;
    bool measuring =
        false;  // only while the harness itself is idle in the event loop (waiting for validation)
    QTimer ticker;
    ticker.setInterval(5);
    connect(&ticker, &QTimer::timeout, &ticker, [&] {
      const qint64 now = clock.elapsed();
      if (measuring) maxGap = std::max(maxGap, now - last);
      last = now;
    });
    ticker.start();
    auto waitCurrent = [&] {
      last = clock.elapsed();
      measuring = true;
      QTRY_VERIFY_WITH_TIMEOUT(s->isReportCurrent(), 20000);
      measuring = false;
    };

    s->replaceDocument(d, QString());
    QVERIFY(s->derived().data.centerline.size() > 9000);
    v->fitToView();
    QVERIFY(!s->isReportCurrent());  // validation of about 10k samples runs in the background
    waitCurrent();
    QCOMPARE(s->report().status(), pg::ValidationStatus::Valid);
    const qint64 stallDuringLoadAndValidation = maxGap;

    // Edit latency: a real drag of one control point.
    action(w, "Add Points")->setChecked(false);
    const auto id = s->document().points()[5].id;
    const QPointF from = v->worldToViewport(s->document().points()[5].position);
    QElapsedTimer t;
    t.start();
    drag(v, from, from + QPointF(25, 15));
    const qint64 dragMs = t.elapsed();
    QVERIFY(s->document().find(id)->position != pg::Vec2{});
    waitCurrent();

    // Rapid undo/redo storm; at the end the report must be the truth for the geometry on screen.
    QElapsedTimer storm;
    storm.start();
    for (int i = 0; i < 20; ++i) {
      s->undoStack()->undo();
      s->undoStack()->redo();
    }
    const qint64 stormMs = storm.elapsed();
    QVERIFY(!s->isReportCurrent() ||
            s->report().status() == pg::validateTrackData(s->derived().data).status());
    waitCurrent();
    QCOMPARE(s->report().status(), pg::validateTrackData(s->derived().data).status());
    // Switching interpolation under load is one undo step and again never shows a stale report.
    interpolationCombo(w)->setCurrentIndex(0);
    QVERIFY(!s->isReportCurrent());
    interpolationCombo(w)->setCurrentIndex(1);
    waitCurrent();
    QCOMPARE(s->report().status(), pg::validateTrackData(s->derived().data).status());
    ticker.stop();
    qInfo().nospace() << "E: samples " << s->derived().data.centerline.size()
                      << ", drag (6 moves + release, incl. "
                      << "scene rebuilds) " << dragMs << " ms, 40 undo/redo steps " << stormMs
                      << " ms, longest GUI-thread gap while validating " << maxGap
                      << " ms (during load+first validation " << stallDuringLoadAndValidation
                      << " ms), discarded stale results " << s->discardedResults();
    // Generous bounds: they catch "validation runs on the GUI thread" (~300 ms+ here), not scheduler noise.
    QVERIFY2(maxGap < 250, "the GUI thread stalled");
    QVERIFY2(dragMs < 500, "dragging a control point is too slow");
  }
};

QTEST_MAIN(StudioAcceptanceTest)
#include "studio_acceptance_test.moc"
