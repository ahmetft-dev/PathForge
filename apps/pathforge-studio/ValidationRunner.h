#pragma once

#include <QObject>
#include <QThreadPool>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <pathforge/geometry/track_data.hpp>
#include <pathforge/geometry/validation.hpp>

// Runs the (Qt-free, O(N^2)) geometry validation off the GUI thread.
//
// Concurrency rules:
//  * The worker only ever sees an immutable COPY of the track data that the GUI thread handed over; it shares
//  no
//    state with the GUI. The geometry library stays Qt-free.
//  * At most one job runs and at most one request waits. A newer request REPLACES the waiting one ("latest
//  wins"),
//    so the queue is bounded and an editing burst costs at most one extra validation.
//  * Results are tagged with the revision they were requested for and delivered to the GUI thread through a
//  queued
//    call; the receiver decides whether that revision is still current. Obsolete results are dropped there.
//  * The destructor blocks until the running job has finished, so a worker can never post to a destroyed
//  object.
//    (Validation cannot be interrupted midway; the longest wait is one validation, ~0.3 s at 10,000 points.)
class ValidationRunner : public QObject {
  Q_OBJECT
 public:
  explicit ValidationRunner(QObject* parent = nullptr);
  ~ValidationRunner() override;

  // Queues validation of `data` for `revision`; replaces any request that has not started yet.
  void request(pathforge::geometry::TrackData data, std::uint64_t revision);
  // Drops a request that has not started (a running job cannot be cancelled; its result is simply ignored).
  void cancelPending();

  bool busy() const { return running_ || pending_.has_value(); }
  const pathforge::geometry::ValidationReport& lastReport() const { return last_; }
  double lastMilliseconds() const { return lastMs_; }

  // Statistics (GUI thread).
  std::uint64_t startedJobs() const { return started_; }
  std::uint64_t replacedRequests() const { return replaced_; }  // waiting requests overwritten by a newer one
  std::uint64_t cancelledRequests() const {
    return cancelled_;
  }  // waiting requests dropped by cancelPending()

  // Test hook: runs on the WORKER thread before each validation (e.g. to hold a job back deterministically).
  void setWorkerHook(std::function<void()> hook) { hook_ = std::move(hook); }

 signals:
  // Emitted on the GUI thread when a job finished; fetch the result with lastReport().
  void finished(quint64 revision, double milliseconds);

 private:
  struct Job {
    pathforge::geometry::TrackData data;
    std::uint64_t revision;
  };
  void startNext();
  void onDone(std::uint64_t revision, pathforge::geometry::ValidationReport report, double ms);

  QThreadPool pool_;
  std::optional<Job> pending_;
  bool running_ = false;
  pathforge::geometry::ValidationReport last_;
  double lastMs_ = 0.0;
  std::uint64_t started_ = 0, replaced_ = 0, cancelled_ = 0;
  std::function<void()> hook_;
};
