// Qt-free cost of the editor's per-edit pipeline: derived-geometry regeneration, full validation and the
// insertion lookup, for documents producing about 100 / 1,000 / 4,000 / 10,000 samples.
// Usage: editor_bench [repeats]   (default 9; the median is reported)
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <numbers>
#include <pathforge/editor/derived_geometry.hpp>
#include <pathforge/geometry/validation.hpp>
#include <vector>

using namespace pathforge::editor;
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

// Closed spline through `controlPoints` points on an ellipse whose circumference is ~ samples * spacing.
static TrackDocument splineDocument(std::size_t samples, std::size_t controlPoints, double spacing) {
  const double perimeter = static_cast<double>(samples) * spacing;
  const double a = perimeter / (2.0 * std::numbers::pi) * 1.25,
               b = a * 0.6;  // ellipse; its true length is a bit less
  TrackDocument d;
  d.setClosure(Closure::Closed);
  GeometrySettings s;
  s.resampleSpacing = spacing;
  d.setSettings(s);
  for (std::size_t i = 0; i < controlPoints; ++i) {
    const double t = 2.0 * std::numbers::pi * static_cast<double>(i) / static_cast<double>(controlPoints);
    d.appendPoint({a * std::cos(t), b * std::sin(t)}, 5.0, 5.0);
  }
  return d;
}

static TrackDocument polylineDocument(std::size_t points) {
  TrackDocument d;
  d.setClosure(Closure::Closed);
  GeometrySettings s;
  s.interpolation = Interpolation::Polyline;
  s.resampleSpacing = 0.0;
  d.setSettings(s);
  for (std::size_t i = 0; i < points; ++i) {
    const double t = 2.0 * std::numbers::pi * static_cast<double>(i) / static_cast<double>(points);
    d.appendPoint({1000.0 * std::cos(t), 600.0 * std::sin(t)}, 5.0, 5.0);
  }
  return d;
}

int main(int argc, char** argv) {
  const int repeats = argc > 1 ? std::atoi(argv[1]) : 9;
  std::printf("Editor pipeline cost, median of %d runs [ms]\n", repeats);
  std::printf("%-34s %8s %12s %14s %12s\n", "document", "samples", "regenerate", "validate", "insert-find");
  struct Case {
    const char* name;
    TrackDocument doc;
  };
  std::vector<Case> cases;
  for (std::size_t n : {100u, 1000u, 4000u, 10000u}) {
    cases.push_back({"spline, 24 control points", splineDocument(n, 24, 1.0)});
  }
  for (std::size_t n : {1000u, 4000u, 10000u})
    cases.push_back({"polyline, N control points (preset)", polylineDocument(n)});
  for (const Case& c : cases) {
    DerivedGeometry g;
    const double regen = medianMs([&] { g = generateGeometry(c.doc); }, repeats);
    pathforge::geometry::ValidationReport r;
    const double val = medianMs([&] { r = pathforge::geometry::validateTrackData(g.data); }, repeats);
    const double ins = medianMs([&] { (void)findInsertion(c.doc, g, {10.0, 10.0}); }, repeats);
    std::printf("%-34s %8zu %12.3f %14.3f %12.3f   %s\n", c.name, g.data.centerline.size(), regen, val, ins,
                pathforge::geometry::toString(r.status()));
  }
  return 0;
}
