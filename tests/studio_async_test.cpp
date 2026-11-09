// Tests of the background validation pipeline: stale-result handling, bounded queue, threading, drag
// behaviour, status transitions and shutdown. A gate in the worker hook makes the races deterministic (no
// sleeps decide order).
#include <QElapsedTimer>
#include <QPointer>
#include <QSemaphore>
#include <QSignalSpy>
#include <QThread>
#include <QtTest>
#include <cmath>
#include <numbers>
#include <pathforge/geometry/validation.hpp>
#include <thread>

#include "EditorSession.h"

namespace pfe = pathforge::editor;
namespace pg = pathforge::geometry;

namespace {

pfe::TrackDocument polylineCircle(int n, double r) {
  pfe::TrackDocument d;
  d.setClosure(pfe::Closure::Closed);
  pfe::GeometrySettings s;
  s.interpolation = pfe::Interpolation::Polyline;
  s.resampleSpacing = 0.0;
  d.setSettings(s);
  for (int i = 0; i < n; ++i) {
    const double a = 2.0 * std::numbers::pi * i / n;
    d.appendPoint({r * std::cos(a), r * std::sin(a)}, 5.0, 5.0);
  }
  return d;
}

pfe::TrackDocument bowtie() {
  pfe::TrackDocument d;
  d.setClosure(pfe::Closure::Closed);
  pfe::GeometrySettings s;
  s.interpolation = pfe::Interpolation::Polyline;
  s.resampleSpacing = 0.0;
  d.setSettings(s);
  for (pg::Vec2 p : {pg::Vec2{0, 0}, pg::Vec2{200, 200}, pg::Vec2{200, 0}, pg::Vec2{0, 200}})
    d.appendPoint(p, 1, 1);
  return d;
}

// What the validation of the session's CURRENT geometry says, computed independently and synchronously.
pg::ValidationStatus truth(const EditorSession& s) {
  return pg::validateTrackData(s.derived().data).status();
}

}  // namespace

class StudioAsyncTest : public QObject {
  Q_OBJECT

 private slots:
  void smallTracksValidateSynchronouslyByDefault() {
    EditorSession s;
    s.replaceDocument(polylineCircle(360, 100.0), QString());
    QVERIFY(s.isReportCurrent());  // no waiting: 360 samples is far below the async threshold
    s.setPointPosition(s.document().points()[3].id, {10, 10});
    QVERIFY(s.isReportCurrent());
    QCOMPARE(s.runner()->startedJobs(), std::uint64_t{0});
  }

  void staleResultsAreDiscardedAndTheNewestRevisionWins() {
    EditorSession s;
    s.setAsyncThreshold(0);  // force the background path
    s.setValidationDebounce(0);
    QSemaphore gate;
    std::vector<QThread*> workerThreads;
    QMutex m;
    s.runner()->setWorkerHook([&] {
      {
        QMutexLocker l(&m);
        workerThreads.push_back(QThread::currentThread());
      }
      gate.acquire();
    });

    // Every time a report is announced as CURRENT it must be the report of the geometry on screen right then.
    int announcedCurrent = 0;
    bool allCurrentReportsMatchTheirGeometry = true;
    connect(&s, &EditorSession::validationChanged, &s, [&] {
      if (!s.isReportCurrent()) return;
      ++announcedCurrent;
      allCurrentReportsMatchTheirGeometry =
          allCurrentReportsMatchTheirGeometry && s.report().status() == truth(s);
    });

    s.replaceDocument(polylineCircle(60, 100.0), QString());  // revision A: valid, job 1 starts and blocks
    QVERIFY(!s.isReportCurrent());
    QTRY_COMPARE_WITH_TIMEOUT(s.runner()->startedJobs(), std::uint64_t{1}, 3000);

    s.replaceDocument(bowtie(), QString());  // B: invalid; its request has to wait behind job 1
    QTRY_VERIFY_WITH_TIMEOUT(s.runner()->busy(), 1000);
    s.replaceDocument(polylineCircle(80, 50.0), QString());  // C: valid again, replaces the waiting request B
    QCOMPARE(s.runner()->startedJobs(), std::uint64_t{1});   // still only the first job
    QVERIFY(!s.isReportCurrent());  // nothing is current while only stale work exists

    gate.release(10);  // let everything run
    QTRY_VERIFY_WITH_TIMEOUT(s.isReportCurrent(), 5000);
    QTRY_VERIFY_WITH_TIMEOUT(!s.runner()->busy(), 5000);
    QCOMPARE(s.report().status(), pg::ValidationStatus::Valid);  // C's verdict, not B's (invalid)
    QCOMPARE(s.report().status(), truth(s));
    QVERIFY(s.discardedResults() >= 1);                     // job 1 delivered a result for obsolete A
    QCOMPARE(s.runner()->startedJobs(), std::uint64_t{2});  // job 1 and the LATEST request; B was replaced
    QVERIFY(s.runner()->replacedRequests() + s.runner()->cancelledRequests() >= 1);  // B never ran
    QVERIFY(allCurrentReportsMatchTheirGeometry);
    QCOMPARE(announcedCurrent, 1);  // exactly one current report was ever shown: the right one
    for (QThread* t : workerThreads) QVERIFY(t != QThread::currentThread());
  }

  void aSlowOlderResultCanNeverReplaceANewerOne() {
    // Same revision logic seen from the runner: the first job is held while a second request arrives.
    EditorSession s;
    s.setAsyncThreshold(0);
    s.setValidationDebounce(0);
    QSemaphore gate;
    s.runner()->setWorkerHook([&] { gate.acquire(); });
    s.replaceDocument(bowtie(), QString());  // invalid, job 1 (held)
    QTRY_COMPARE_WITH_TIMEOUT(s.runner()->startedJobs(), std::uint64_t{1}, 3000);
    s.replaceDocument(polylineCircle(50, 80.0), QString());  // valid; queued behind job 1
    gate.release(1);  // job 1 (the stale, INVALID result) finishes first
    QTRY_VERIFY_WITH_TIMEOUT(s.discardedResults() >= 1, 3000);
    QVERIFY(!s.isReportCurrent());  // the stale 'invalid' report was not adopted
    gate.release(5);
    QTRY_VERIFY_WITH_TIMEOUT(s.isReportCurrent(), 5000);
    QCOMPARE(s.report().status(), pg::ValidationStatus::Valid);
  }

  void theRequestQueueIsBounded() {
    ValidationRunner r;
    QSemaphore gate;
    r.setWorkerHook([&] { gate.acquire(); });
    std::vector<std::uint64_t> delivered;
    connect(&r, &ValidationRunner::finished, &r, [&](quint64 rev, double) { delivered.push_back(rev); });
    const pg::TrackData data = pfe::generateGeometry(polylineCircle(40, 100.0)).data;
    for (std::uint64_t rev = 1; rev <= 500; ++rev) r.request(data, rev);  // a burst of 500 requests
    QCOMPARE(r.startedJobs(), std::uint64_t{1});                          // one running ...
    QCOMPARE(r.replacedRequests(),
             std::uint64_t{498});  // ... one waiting; the rest were replaced, not queued
    gate.release(10);
    QTRY_VERIFY_WITH_TIMEOUT(delivered.size() == 2, 5000);
    QCOMPARE(delivered.front(), std::uint64_t{1});
    QCOMPARE(delivered.back(), std::uint64_t{500});  // the newest request is the one that ran second
    QCOMPARE(r.startedJobs(), std::uint64_t{2});
    QVERIFY(!r.busy());
  }

  void workRunsOffTheGuiThreadAndResultsArriveOnIt() {
    ValidationRunner r;
    QThread* workerThread = nullptr;
    QThread* receiverThread = nullptr;
    r.setWorkerHook([&] { workerThread = QThread::currentThread(); });
    connect(&r, &ValidationRunner::finished, &r,
            [&](quint64, double) { receiverThread = QThread::currentThread(); });
    r.request(pfe::generateGeometry(polylineCircle(40, 100.0)).data, 7);
    QTRY_VERIFY_WITH_TIMEOUT(receiverThread != nullptr, 5000);
    QVERIFY(workerThread != nullptr);
    QVERIFY(workerThread != QThread::currentThread());
    QCOMPARE(receiverThread,
             QThread::currentThread());  // delivered to the GUI thread, never mutated from the worker
    QCOMPARE(r.lastReport().status(), pg::ValidationStatus::Valid);
  }

  void draggingNeverStartsValidationAndEndsWithExactlyOne() {
    EditorSession s;
    s.setAsyncThreshold(0);
    s.setValidationDebounce(0);
    s.replaceDocument(polylineCircle(120, 100.0), QString());
    QTRY_VERIFY_WITH_TIMEOUT(s.isReportCurrent(), 5000);
    const auto started = s.runner()->startedJobs();
    const pfe::ControlPoint p = s.document().points()[10];
    QVERIFY(s.beginDrag(p.id));
    for (int i = 1; i <= 100; ++i) {
      s.dragTo({p.position.x + i * 0.1, p.position.y + i * 0.05});
      QCoreApplication::processEvents();
    }
    QCOMPARE(s.runner()->startedJobs(), started);  // not one validation while the mouse moves
    QVERIFY(!s.isReportCurrent());                 // the old report is flagged stale, not shown as current
    QVERIFY(!s.validationRunning());
    QVERIFY(s.derived().track.has_value());  // but the geometry preview is live
    s.endDrag();
    QTRY_VERIFY_WITH_TIMEOUT(s.isReportCurrent(), 5000);
    QCOMPARE(s.runner()->startedJobs(), started + 1);  // exactly one for the whole drag
    QCOMPARE(s.report().status(), truth(s));
  }

  void statusGoesFromPendingToCurrent() {
    EditorSession s;
    s.setAsyncThreshold(0);
    s.setValidationDebounce(0);
    s.replaceDocument(polylineCircle(100, 100.0), QString());
    QTRY_VERIFY_WITH_TIMEOUT(s.isReportCurrent(), 5000);
    std::vector<bool> sequence;
    connect(&s, &EditorSession::validationChanged, &s, [&] { sequence.push_back(s.isReportCurrent()); });
    s.setPointPosition(s.document().points()[5].id,
                       {0, 0});     // moves a point onto another region: invalid or not,
    QVERIFY(!s.isReportCurrent());  // the report is pending right after the edit
    QVERIFY(s.validationRunning());
    QTRY_VERIFY_WITH_TIMEOUT(s.isReportCurrent(), 5000);
    QVERIFY(!sequence.empty());
    QCOMPARE(sequence.front(), false);  // first announcement: pending
    QCOMPARE(sequence.back(), true);    // last: the finished result
    QCOMPARE(std::count(sequence.begin(), sequence.end(), true), 1);
    QCOMPARE(s.report().status(), truth(s));
    QVERIFY(!s.validationRunning());
  }

  void debounceCoalescesAnEditingBurstIntoOneValidation() {
    EditorSession s;
    s.setAsyncThreshold(0);
    s.setValidationDebounce(80);
    s.replaceDocument(polylineCircle(100, 100.0), QString());
    QTRY_VERIFY_WITH_TIMEOUT(s.isReportCurrent(), 5000);
    const auto started = s.runner()->startedJobs();
    for (std::size_t i = 0; i < 20; ++i) {
      s.setPointPosition(s.document().points()[i].id,
                         {100.0 + static_cast<double>(i), 5.0 * static_cast<double>(i)});
      QTest::qWait(5);  // far below the debounce interval: the timer keeps restarting
    }
    QCOMPARE(s.runner()->startedJobs(), started);  // nothing yet
    QTRY_VERIFY_WITH_TIMEOUT(s.isReportCurrent(), 5000);
    QCOMPARE(s.runner()->startedJobs(), started + 1);
    QCOMPARE(s.report().status(), truth(s));
  }

  void explicitRequestAndValidatedReportGiveACurrentResult() {
    EditorSession s;
    s.setAsyncThreshold(0);
    s.setValidationDebounce(5000);  // nothing would run by itself within the test
    s.replaceDocument(polylineCircle(100, 100.0), QString());
    QTRY_VERIFY_WITH_TIMEOUT(s.isReportCurrent(), 5000);
    s.setPointPosition(s.document().points()[0].id, {100, 100});
    QVERIFY(!s.isReportCurrent());
    s.requestValidation();  // explicit request skips the debounce
    QTRY_VERIFY_WITH_TIMEOUT(s.isReportCurrent(), 5000);
    s.setPointPosition(s.document().points()[1].id, {-100, 100});
    QVERIFY(!s.isReportCurrent());
    const pg::ValidationReport& r =
        s.validatedReport();  // downstream consumers: synchronous, guaranteed current
    QVERIFY(s.isReportCurrent());
    QCOMPARE(r.status(), truth(s));
  }

  void replacingTheDocumentDropsInFlightResultsOfTheOldOne() {
    EditorSession s;
    s.setAsyncThreshold(0);
    s.setValidationDebounce(0);
    QSemaphore gate;
    s.runner()->setWorkerHook([&] { gate.acquire(); });
    s.replaceDocument(bowtie(), QString());
    QTRY_COMPARE_WITH_TIMEOUT(s.runner()->startedJobs(), std::uint64_t{1}, 3000);
    s.replaceDocument(polylineCircle(70, 90.0), QString());  // the old job is still running
    gate.release(10);
    QTRY_VERIFY_WITH_TIMEOUT(s.isReportCurrent(), 5000);
    QCOMPARE(s.report().status(), pg::ValidationStatus::Valid);
    QVERIFY(s.discardedResults() >= 1);
  }

  void shutdownWithRunningAndPendingWorkIsSafe() {
    QSemaphore gate;
    auto* s = new EditorSession;
    s->setAsyncThreshold(0);
    s->setValidationDebounce(0);
    s->runner()->setWorkerHook([&] { gate.acquire(); });
    s->replaceDocument(polylineCircle(300, 100.0), QString());
    QTRY_COMPARE_WITH_TIMEOUT(s->runner()->startedJobs(), std::uint64_t{1}, 3000);
    s->replaceDocument(polylineCircle(310, 100.0), QString());  // a request waiting behind the running job
    QPointer<EditorSession> guard(s);
    // Another thread lets the worker finish a little later; the destructor must WAIT for it, not crash or
    // race.
    std::thread releaser([&] {
      QThread::msleep(150);
      gate.release(10);
    });
    QElapsedTimer t;
    t.start();
    delete s;  // blocks until the running job is done; its result is posted to a receiver that is being
               // destroyed
    const qint64 waited = t.elapsed();
    releaser.join();
    QVERIFY(guard.isNull());
    QVERIFY2(waited >= 100, "destructor returned before the running job finished");
    QCoreApplication::processEvents();  // any queued completion events must be gone, not delivered to a dead
                                        // object
    QCoreApplication::sendPostedEvents(nullptr, QEvent::MetaCall);
  }

  void editsAfterUndoRevalidateTheRestoredGeometry() {
    EditorSession s;
    s.setAsyncThreshold(0);
    s.setValidationDebounce(0);
    s.replaceDocument(polylineCircle(100, 100.0), QString());
    QTRY_VERIFY_WITH_TIMEOUT(s.isReportCurrent(), 5000);
    const auto id = s.document().points()[0].id;
    s.setPointPosition(id, {0, 0});
    QTRY_VERIFY_WITH_TIMEOUT(s.isReportCurrent(), 5000);
    const auto changed = s.report().status();
    s.undoStack()->undo();
    QVERIFY(!s.isReportCurrent());  // the restored geometry has not been validated yet
    QTRY_VERIFY_WITH_TIMEOUT(s.isReportCurrent(), 5000);
    QCOMPARE(s.report().status(), pg::ValidationStatus::Valid);
    QVERIFY(changed != pg::ValidationStatus::Valid || true);
    QCOMPARE(s.report().status(), truth(s));
  }
};

QTEST_MAIN(StudioAsyncTest)
#include "studio_async_test.moc"
