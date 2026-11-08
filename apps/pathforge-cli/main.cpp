// pathforge-cli: headless front-end for the PathForge computation libraries (no Qt).
#include <pathforge/geometry/circle.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

using namespace pathforge::geometry;

namespace {

void printHelp(std::FILE* out) {
  std::fputs(
      "PathForge CLI\n"
      "\n"
      "Usage:\n"
      "  pathforge-cli circle [--radius <m>] [--points <n>]\n"
      "  pathforge-cli --help\n"
      "\n"
      "Commands:\n"
      "  circle   Generate a synthetic circular track and report its geometry.\n"
      "\n"
      "Options (circle):\n"
      "  --radius <m>   Circle radius in meters, finite and > 0   (default 100)\n"
      "  --points <n>   Number of sampled points, integer >= 3     (default 360)\n",
      out);
}

bool parseDouble(const char* s, double& out) {
  char* end = nullptr;
  out = std::strtod(s, &end);
  return end != s && *end == '\0' && std::isfinite(out);
}

bool parseCount(const char* s, std::size_t& out) {
  char* end = nullptr;
  const long long v = std::strtoll(s, &end, 10);
  if (end == s || *end != '\0' || v < 3) return false;
  out = static_cast<std::size_t>(v);
  return true;
}

int fail(const std::string& msg) {
  std::fprintf(stderr, "error: %s\n\n", msg.c_str());
  printHelp(stderr);
  return 2;
}

int runCircle(int argc, char** argv) {
  double radius = 100.0;
  std::size_t points = 360;
  for (int i = 2; i < argc; ++i) {
    const std::string a = argv[i];
    if ((a == "--radius" || a == "--points") && i + 1 >= argc) return fail(a + " requires a value");
    if (a == "--radius") {
      if (!parseDouble(argv[++i], radius) || radius <= 0.0) return fail("--radius must be a finite number > 0");
    } else if (a == "--points") {
      if (!parseCount(argv[++i], points)) return fail("--points must be an integer >= 3");
    } else {
      return fail("unknown option '" + a + "'");
    }
  }

  try {
    const Path track = makeCircle(radius, points);
    const PathStats st = computeStats(track);
    const double exact = circleCircumference(radius);
    const double inscribed = inscribedPolygonPerimeter(radius, points);
    const double kRef = 1.0 / radius;

    std::printf("PathForge circle track (SI units: m, 1/m)\n");
    std::printf("  radius                     : %.6f m\n", radius);
    std::printf("  points                     : %zu\n", st.pointCount);
    std::printf("  computed length            : %.9f m\n", st.length);
    std::printf("  analytical circumference   : %.9f m  (2*pi*R)\n", exact);
    std::printf("  length error vs 2*pi*R     : %.3e m  (relative %.3e; discretisation)\n",
                exact - st.length, (exact - st.length) / exact);
    std::printf("  inscribed N-gon perimeter  : %.9f m  (reference for the sampled path)\n", inscribed);
    std::printf("  length error vs N-gon      : %.3e m  (rounding)\n", st.length - inscribed);
    std::printf("  segment length min / max   : %.9f / %.9f m\n", st.minSegment, st.maxSegment);
    std::printf("  curvature min / max        : %.9e / %.9e 1/m\n", st.minCurvature, st.maxCurvature);
    std::printf("  curvature mean |k|         : %.9e 1/m\n", st.meanAbsCurvature);
    std::printf("  analytical curvature 1/R   : %.9e 1/m\n", kRef);
    std::printf("  max |k - 1/R| / (1/R)      : %.3e\n",
                std::fmax(std::fabs(st.minCurvature - kRef), std::fabs(st.maxCurvature - kRef)) / kRef);
    return 0;
  } catch (const GeometryError& e) {
    return fail(e.what());
  }
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
  if (cmd == "circle") return runCircle(argc, argv);
  return fail("unknown command '" + cmd + "'");
}
