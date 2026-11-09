// pathforge-cli: headless front-end for the PathForge computation libraries (no Qt).
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <optional>
#include <pathforge/geometry/circle.hpp>
#include <pathforge/geometry/frames.hpp>
#include <pathforge/geometry/shapes.hpp>
#include <pathforge/geometry/track.hpp>
#include <string>

using namespace pathforge::geometry;

namespace {

void printHelp(std::FILE* out) {
  std::fputs(
      "PathForge CLI (all lengths in meters)\n"
      "\n"
      "Usage:\n"
      "  pathforge-cli circle [--radius <m>] [--points <n>] [--track-width <m>]\n"
      "  pathforge-cli oval   [--length <m>] [--width <m>] [--points <n>] [--track-width <m>]\n"
      "  pathforge-cli scurve [--length <m>] [--amplitude <m>] [--points <n>] [--track-width <m>]\n"
      "  pathforge-cli --help\n"
      "\n"
      "Tracks:\n"
      "  circle   Closed circular centerline of the given radius.\n"
      "  oval     Closed stadium: straights joined by semicircles of radius width/2.\n"
      "           --length is the overall extent along x, --width the overall extent along y\n"
      "           (centerline dimensions); requires length >= width.\n"
      "  scurve   Open S-shaped path of overall x extent --length, amplitude of the underlying\n"
      "           sine --amplitude (peak lateral offset ~ amplitude).\n"
      "\n"
      "Options:\n"
      "  --points <n>        Sampled points, integer >= 3 (circle) or >= 8 (oval, scurve)\n"
      "  --track-width <m>   TOTAL track width, split equally left/right of the centerline,\n"
      "                      finite and >= 0 (default 10)\n"
      "Defaults: circle --radius 100 --points 360; oval --length 300 --width 120 --points 512;\n"
      "          scurve --length 300 --amplitude 30 --points 256\n",
      out);
}

bool parseDouble(const std::string& s, double& out) {
  char* end = nullptr;
  out = std::strtod(s.c_str(), &end);
  return end != s.c_str() && *end == '\0' && std::isfinite(out);
}

bool parseCount(const std::string& s, std::size_t& out) {
  char* end = nullptr;
  const long long v = std::strtoll(s.c_str(), &end, 10);
  if (end == s.c_str() || *end != '\0' || v < 3) return false;
  out = static_cast<std::size_t>(v);
  return true;
}

struct UsageError {
  std::string message;
};

// Parses "--name value" pairs for the listed option names; anything else is an error.
std::map<std::string, std::string> parseOptions(int argc, char** argv,
                                                std::initializer_list<const char*> allowed) {
  std::map<std::string, std::string> out;
  for (int i = 2; i < argc; ++i) {
    const std::string a = argv[i];
    bool known = false;
    for (const char* name : allowed) known = known || a == name;
    if (!known) throw UsageError{"unknown option '" + a + "'"};
    if (i + 1 >= argc) throw UsageError{a + " requires a value"};
    out[a] = argv[++i];
  }
  return out;
}

double getDouble(const std::map<std::string, std::string>& o, const char* name, double def, bool positive) {
  const auto it = o.find(name);
  if (it == o.end()) return def;
  double v = 0.0;
  if (!parseDouble(it->second, v)) throw UsageError{std::string(name) + " must be a finite number"};
  if (positive && v <= 0.0) throw UsageError{std::string(name) + " must be > 0"};
  if (!positive && v < 0.0) throw UsageError{std::string(name) + " must be >= 0"};
  return v;
}

std::size_t getCount(const std::map<std::string, std::string>& o, const char* name, std::size_t def) {
  const auto it = o.find(name);
  if (it == o.end()) return def;
  std::size_t v = 0;
  if (!parseCount(it->second, v)) throw UsageError{std::string(name) + " must be an integer >= 3"};
  return v;
}

struct Reference {
  std::optional<double> length;     // analytical length of the continuous curve [m]
  std::optional<double> curvature;  // analytical constant curvature magnitude [1/m]
};

void report(const char* type, const std::string& params, const Track& track, const Reference& ref) {
  const Path& c = track.centerline();
  const PathStats st = computeStats(c);
  const TrackDiagnostics& d = track.diagnostics();

  std::printf("PathForge track: %s (SI units: m, 1/m)\n", type);
  std::printf("  parameters                 : %s\n", params.c_str());
  std::printf("  closed                     : %s\n", c.isClosed() ? "yes" : "no (open path)");
  std::printf("  points                     : %zu\n", st.pointCount);
  std::printf("  computed length            : %.9f m\n", st.length);
  if (ref.length) {
    const double e = *ref.length - st.length;
    std::printf("  analytical length          : %.9f m\n", *ref.length);
    std::printf("  length error (analytic-sampled): %.3e m (relative %.3e; discretisation)\n", e,
                e / *ref.length);
  } else {
    std::printf("  analytical length          : n/a\n");
  }
  std::printf("  segment length min / max   : %.9f / %.9f m\n", st.minSegment, st.maxSegment);
  std::printf("  curvature min / max (signed): %.6e / %.6e 1/m  (+ = left/CCW)\n", st.minCurvature,
              st.maxCurvature);
  std::printf("  |curvature| min / mean / max: %.6e / %.6e / %.6e 1/m\n", st.minAbsCurvature,
              st.meanAbsCurvature, st.maxAbsCurvature);
  std::printf("  curvature sign changes     : %zu  (ignoring |k| < 25%% of max)\n",
              countSignChanges(track.frames().curvature, 0.25 * st.maxAbsCurvature));
  if (ref.curvature) {
    const double k = *ref.curvature;
    const double dev =
        std::fmax(std::fabs(std::fabs(st.minCurvature) - k), std::fabs(std::fabs(st.maxCurvature) - k));
    std::printf("  analytical |curvature|     : %.6e 1/m  (max relative deviation %.3e)\n", k, dev / k);
  }
  std::printf("  track width left / right   : %.3f / %.3f m (total %.3f m)\n", track.leftWidth(),
              track.rightWidth(), track.totalWidth());
  std::printf(
      "  boundary local issues      : %s (flipped segments L/R %zu/%zu, curvature violations L/R %zu/%zu)\n",
      d.anyLocalIssue() ? "YES" : "none detected", d.leftFlippedSegments, d.rightFlippedSegments,
      d.leftCurvatureViolations, d.rightCurvatureViolations);
  std::printf("  width*curvature max L / R  : %.4f / %.4f  (>= 1 means the offset folds)\n",
              d.maxLeftWidthCurvature, d.maxRightWidthCurvature);
  std::printf("  note                       : global boundary validity (self-intersection) is not checked\n");
}

std::string fmt(const char* f, double a) {
  char buf[128];
  std::snprintf(buf, sizeof buf, f, a);
  return buf;
}

int runCircle(int argc, char** argv) {
  const auto o = parseOptions(argc, argv, {"--radius", "--points", "--track-width"});
  const double radius = getDouble(o, "--radius", 100.0, true);
  const std::size_t points = getCount(o, "--points", 360);
  const double width = getDouble(o, "--track-width", 10.0, false);
  const Track t = Track::create(makeCircle(radius, points), width / 2, width / 2);
  report("circle", fmt("radius %.6g m", radius) + ", points " + std::to_string(points), t,
         {circleCircumference(radius), 1.0 / radius});
  return 0;
}

int runOval(int argc, char** argv) {
  const auto o = parseOptions(argc, argv, {"--length", "--width", "--points", "--track-width"});
  const double length = getDouble(o, "--length", 300.0, true);
  const double width = getDouble(o, "--width", 120.0, true);
  const std::size_t points = getCount(o, "--points", 512);
  const double trackWidth = getDouble(o, "--track-width", 10.0, false);
  const Track t = Track::create(makeStadium(length, width, points), trackWidth / 2, trackWidth / 2);
  report("oval (stadium)",
         fmt("length %.6g m", length) + fmt(", width %.6g m", width) +
             fmt(", corner radius %.6g m", width / 2) + ", points " + std::to_string(points),
         t, {roundedRectanglePerimeter(length, width, width / 2), std::nullopt});
  return 0;
}

int runSCurve(int argc, char** argv) {
  const auto o = parseOptions(argc, argv, {"--length", "--amplitude", "--points", "--track-width"});
  const double length = getDouble(o, "--length", 300.0, true);
  const double amplitude = getDouble(o, "--amplitude", 30.0, true);
  const std::size_t points = getCount(o, "--points", 256);
  const double trackWidth = getDouble(o, "--track-width", 10.0, false);
  const Track t = Track::create(makeSCurve(length, amplitude, points), trackWidth / 2, trackWidth / 2);
  report("s-curve",
         fmt("length %.6g m", length) + fmt(", amplitude %.6g m", amplitude) + ", points " +
             std::to_string(points),
         t, {});
  return 0;
}

int fail(const std::string& msg) {
  std::fprintf(stderr, "error: %s\n\n", msg.c_str());
  printHelp(stderr);
  return 2;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    printHelp(stderr);
    return 2;
  }
  const std::string cmd = argv[1];
  if (cmd == "--help" || cmd == "-h" || cmd == "help") {
    printHelp(stdout);
    return 0;
  }
  try {
    if (cmd == "circle") return runCircle(argc, argv);
    if (cmd == "oval") return runOval(argc, argv);
    if (cmd == "scurve") return runSCurve(argc, argv);
    return fail("unknown command '" + cmd + "'");
  } catch (const UsageError& e) {
    return fail(e.message);
  } catch (const GeometryError& e) {
    return fail(e.what());
  }
}
