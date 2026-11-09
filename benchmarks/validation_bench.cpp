// Measures validateTrack() wall time for representative synthetic tracks (Phase 1B).
// Usage: validation_bench [repeats]   (default 7; the median is reported)
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <pathforge/geometry/circle.hpp>
#include <pathforge/geometry/shapes.hpp>
#include <pathforge/geometry/validation.hpp>
#include <vector>

using namespace pathforge::geometry;
using Clock = std::chrono::steady_clock;

static double medianMs(const std::function<void()>& fn, int repeats) {
  std::vector<double> t;
  for (int i = 0; i < repeats; ++i) {
    const auto a = Clock::now();
    fn();
    t.push_back(std::chrono::duration<double, std::milli>(Clock::now() - a).count());
  }
  std::sort(t.begin(), t.end());
  return t[t.size() / 2];
}

int main(int argc, char** argv) {
  const int repeats = argc > 1 ? std::atoi(argv[1]) : 7;
  std::printf(
      "validateTrack wall time, median of %d runs [ms]; status must be 'valid' for the timing to be "
      "representative\n",
      repeats);
  std::printf("%-22s %6s  %10s  %-8s %s\n", "track", "N", "time [ms]", "status", "time/N^2 [ns]");
  struct Case {
    const char* name;
    std::function<Path(std::size_t)> make;
  };
  const Case cases[] = {
      {"circle R=100", [](std::size_t n) { return makeCircle(100.0, n); }},
      {"stadium 300x120", [](std::size_t n) { return makeStadium(300.0, 120.0, n); }},
      {"s-curve L=300 A=30", [](std::size_t n) { return makeSCurve(300.0, 30.0, n); }},
  };
  for (const Case& c : cases) {
    for (std::size_t n : {100u, 1000u, 4000u, 10000u}) {
      const Track t = Track::create(c.make(n), 5.0, 5.0);
      ValidationReport r;
      const double ms = medianMs([&] { r = validateTrack(t); }, repeats);
      std::printf("%-22s %6zu  %10.3f  %-8s %.2f\n", c.name, n, ms, toString(r.status()),
                  ms * 1e6 / (static_cast<double>(n) * static_cast<double>(n)));
    }
  }
  return 0;
}
