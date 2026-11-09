#include "ValidationRunner.h"

#include <QMetaObject>
#include <QRunnable>
#include <chrono>

namespace pg = pathforge::geometry;

namespace {

class ValidationJob : public QRunnable {
 public:
  ValidationJob(ValidationRunner* owner, pg::TrackData data, std::uint64_t revision,
                std::function<void()> hook,
                std::function<void(std::uint64_t, pg::ValidationReport, double)> done)
      : owner_(owner),
        data_(std::move(data)),
        revision_(revision),
        hook_(std::move(hook)),
        done_(std::move(done)) {}

  void run() override {
    if (hook_) hook_();
    const auto t0 = std::chrono::steady_clock::now();
    pg::ValidationReport report = pg::validateTrackData(data_);
    const double ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    // Back to the GUI thread. `owner_` is alive: its destructor waits for this job (see ValidationRunner).
    auto result = std::make_shared<pg::ValidationReport>(std::move(report));
    auto done = done_;
    const std::uint64_t revision = revision_;
    QMetaObject::invokeMethod(
        owner_, [done, result, revision, ms] { done(revision, std::move(*result), ms); },
        Qt::QueuedConnection);
  }

 private:
  ValidationRunner* owner_;
  pg::TrackData data_;
  std::uint64_t revision_;
  std::function<void()> hook_;
  std::function<void(std::uint64_t, pg::ValidationReport, double)> done_;
};

}  // namespace

ValidationRunner::ValidationRunner(QObject* parent) : QObject(parent) {
  pool_.setMaxThreadCount(1);  // one validation at a time; keeps CPU use and memory bounded
}

ValidationRunner::~ValidationRunner() {
  pending_.reset();
  pool_.waitForDone();  // never destroy the receiver of a posted result while its job is still running
}

void ValidationRunner::request(pg::TrackData data, std::uint64_t revision) {
  if (pending_) ++replaced_;
  pending_ = Job{std::move(data), revision};
  if (!running_) startNext();
}

void ValidationRunner::cancelPending() {
  if (pending_) ++cancelled_;
  pending_.reset();
}

void ValidationRunner::startNext() {
  if (!pending_ || running_) return;
  Job job = std::move(*pending_);
  pending_.reset();
  running_ = true;
  ++started_;
  pool_.start(new ValidationJob(this, std::move(job.data), job.revision, hook_,
                                [this](std::uint64_t revision, pg::ValidationReport report, double ms) {
                                  onDone(revision, std::move(report), ms);
                                }));
}

void ValidationRunner::onDone(std::uint64_t revision, pg::ValidationReport report, double ms) {
  running_ = false;
  last_ = std::move(report);
  lastMs_ = ms;
  emit finished(revision, ms);
  startNext();  // the waiting request, if any, starts only after the result has been delivered
}
