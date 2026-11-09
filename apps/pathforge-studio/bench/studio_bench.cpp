// End-to-end cost of dragging a control point in Studio: document edit -> derived geometry -> scene rebuild
// -> repaint. Validation is NOT part of a drag (it runs once on release), so it is reported separately by
// editor_bench. Uses the real MainWindow; the platform plugin decides what "paint" means:
//   QT_QPA_PLATFORM=offscreen  software raster into a buffer (default here, reproducible)
//   (unset)                    the native platform, painted through grab() of the viewport
// Usage: studio_bench [steps]   (drag steps per case, default 120)
#include <QApplication>
#include <QElapsedTimer>
#include <QGraphicsView>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <vector>

#include "EditorSession.h"
#include "MainWindow.h"
#include "TrackViewport.h"

namespace pfe = pathforge::editor;

static pfe::TrackDocument splineDoc(std::size_t samples) {
  const double perimeter = static_cast<double>(samples);  // 1 m spacing
  const double a = perimeter / (2.0 * std::numbers::pi) * 1.25, b = a * 0.6;
  pfe::TrackDocument d;
  d.setClosure(pfe::Closure::Closed);
  pfe::GeometrySettings s;
  s.resampleSpacing = 1.0;
  d.setSettings(s);
  for (int i = 0; i < 24; ++i) {
    const double t = 2.0 * std::numbers::pi * i / 24;
    d.appendPoint({a * std::cos(t), b * std::sin(t)}, 5.0, 5.0);
  }
  return d;
}

static pfe::TrackDocument polylineDoc(std::size_t n) {
  pfe::TrackDocument d;
  d.setClosure(pfe::Closure::Closed);
  pfe::GeometrySettings s;
  s.interpolation = pfe::Interpolation::Polyline;
  s.resampleSpacing = 0.0;
  d.setSettings(s);
  for (std::size_t i = 0; i < n; ++i) {
    const double t = 2.0 * std::numbers::pi * static_cast<double>(i) / static_cast<double>(n);
    d.appendPoint({1000.0 * std::cos(t), 600.0 * std::sin(t)}, 5.0, 5.0);
  }
  return d;
}

struct Stats {
  double median, p95, max;
};
static Stats stats(std::vector<double> v) {
  std::sort(v.begin(), v.end());
  return {v[v.size() / 2], v[static_cast<std::size_t>(static_cast<double>(v.size()) * 0.95)], v.back()};
}

int main(int argc, char** argv) {
  QApplication app(argc, argv);
  const int steps = argc > 1 ? std::atoi(argv[1]) : 120;
  MainWindow w;
  w.resize(1280, 800);
  w.show();
  EditorSession* s = w.session();
  TrackViewport* v = w.viewport();
  std::printf("platform: %s, drag steps per case: %d, window 1280x800\n",
              qPrintable(QGuiApplication::platformName()), steps);
  std::printf("%-34s %8s %9s | %-26s | %-26s | %-26s\n", "document", "samples", "controls",
              "edit+scene rebuild [ms]", "repaint [ms]", "end-to-end per move [ms]");
  std::printf("%-34s %8s %9s | %8s %8s %8s | %8s %8s %8s | %8s %8s %8s\n", "", "", "", "median", "p95", "max",
              "median", "p95", "max", "median", "p95", "max");
  struct Case {
    const char* name;
    pfe::TrackDocument doc;
  };
  std::vector<Case> cases;
  for (std::size_t n : {100u, 1000u, 4000u, 10000u})
    cases.push_back({"spline, 24 control points", splineDoc(n)});
  for (std::size_t n : {1000u, 4000u})
    cases.push_back({"polyline, N control points (preset)", polylineDoc(n)});
  for (Case& c : cases) {
    s->setAsyncThreshold(0);  // a drag must never validate; keep any (wrong) validation off the GUI thread
    s->replaceDocument(c.doc, QString());
    v->fitToView();
    QApplication::processEvents();
    const pfe::ControlPoint p = s->document().points()[3];
    s->beginDrag(p.id);
    std::vector<double> edit, paint, total;
    QElapsedTimer t;
    for (int i = 1; i <= steps; ++i) {
      const double a = 0.15 * i;
      t.start();
      s->dragTo({p.position.x + 20.0 * std::cos(a), p.position.y + 20.0 * std::sin(a)});
      const double e = static_cast<double>(t.nsecsElapsed()) / 1e6;
      t.start();
      (void)v->viewport()->grab();  // forces a complete repaint of the viewport
      const double r = static_cast<double>(t.nsecsElapsed()) / 1e6;
      edit.push_back(e);
      paint.push_back(r);
      total.push_back(e + r);
    }
    s->endDrag();
    const Stats e = stats(edit), r = stats(paint), a = stats(total);
    std::printf("%-34s %8zu %9zu | %8.3f %8.3f %8.3f | %8.3f %8.3f %8.3f | %8.3f %8.3f %8.3f\n", c.name,
                s->derived().data.centerline.size(), s->document().size(), e.median, e.p95, e.max, r.median,
                r.p95, r.max, a.median, a.p95, a.max);
  }
  std::printf("(60 FPS = 16.7 ms per frame)\n");
  return 0;
}
