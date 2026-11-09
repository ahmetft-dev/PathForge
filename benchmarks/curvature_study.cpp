// Curvature quality study (Phase 1B): analytic spline curvature vs discrete Menger curvature.
// Prints plain tables; numbers quoted in docs/curvature-study.md were produced by this program.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <pathforge/geometry/circle.hpp>
#include <pathforge/geometry/frames.hpp>
#include <pathforge/geometry/spline.hpp>
#include <vector>

using namespace pathforge::geometry;
constexpr double kPi = std::numbers::pi;

static std::vector<Vec2> circleControl(double r, int m) {
  std::vector<Vec2> c;
  for (int i = 0; i < m; ++i) c.push_back({r * std::cos(2 * kPi * i / m), r * std::sin(2 * kPi * i / m)});
  return c;
}

static std::vector<Vec2> sineControl(double L, double A, int k) {
  std::vector<Vec2> c;
  for (int i = 0; i <= k; ++i)
    c.push_back({L * i / k, (i == 0 || i == k) ? 0.0 : -A * std::sin(2 * kPi * i / k)});
  return c;
}

static double maxAbs(const std::vector<double>& v) {
  double m = 0;
  for (double x : v) m = std::max(m, std::abs(x));
  return m;
}

int main() {
  std::printf("== 1. Straight line (5 collinear, unevenly spaced control points)\n");
  {
    const std::vector<Vec2> c{{0, 0}, {3, 1}, {4, 4.0 / 3}, {10, 10.0 / 3}, {17, 17.0 / 3}};
    const auto a = catmullRomCurvature(c, Closure::Open, {32, 0.5});
    const auto m = catmullRomSpline(c, Closure::Open, {32, 0.5}).curvature();
    std::printf("   max |analytic| = %.3e   max |Menger| = %.3e   (both ~0; rounding only)\n", maxAbs(a),
                maxAbs(m));
  }

  std::printf("\n== 2. Circle R=100, closed Catmull-Rom through M points on it, 64 samples/segment\n");
  std::printf("   %-4s %-16s %-16s %-16s %-18s\n", "M", "max|k*R-1| anal.", "max|k*R-1| Menger",
              "knot-jump k*R", "Menger vs anal. (non-knot)");
  for (int M : {8, 16, 32, 64, 128}) {
    const double R = 100.0;
    const auto ctrl = circleControl(R, M);
    SplineOptions o{64, 0.5};
    const auto a = catmullRomCurvature(ctrl, Closure::Closed, o);
    const Path p = catmullRomSpline(ctrl, Closure::Closed, o);
    const auto m = p.curvature();
    double ea = 0, em = 0, diff = 0, jump = 0;
    for (std::size_t i = 0; i < a.size(); ++i) {
      ea = std::max(ea, std::abs(a[i] * R - 1));
      em = std::max(em, std::abs(m[i] * R - 1));
      const bool nearKnot = (i % 64) <= 1 || (i % 64) >= 63;
      if (!nearKnot) diff = std::max(diff, std::abs(m[i] - a[i]) * R);
      // jump across knot: analytic at sample 0 of the next segment vs analytic just before the knot (u =
      // 63/64)
      if (i % 64 == 0) jump = std::max(jump, std::abs(a[i] - a[(i + a.size() - 1) % a.size()]) * R);
    }
    std::printf("   %-4d %-16.3e %-16.3e %-16.3e %-18.3e\n", M, ea, em, jump, diff);
  }

  std::printf("\n== 3. S-curve (L=300, A=30), 17 control points: analytic vs Menger, 64 samples/segment\n");
  {
    const auto ctrl = sineControl(300, 30, 16);
    SplineOptions o{64, 0.5};
    const auto a = catmullRomCurvature(ctrl, Closure::Open, o);
    const Path p = catmullRomSpline(ctrl, Closure::Open, o);
    const auto m = p.curvature();
    const double kSine = 30 * std::pow(2 * kPi / 300, 2);
    double ea = 0, diffMid = 0;
    int signFlipsAnalytic = 0, signFlipsMenger = 0;
    int la = 0, lm = 0;
    for (std::size_t i = 0; i < a.size(); ++i) {
      const double x = p.points()[i].x;
      // true curvature of y = -A sin(2 pi x / L): k = y''/(1+y'^2)^1.5
      const double w = 2 * kPi / 300, yp = -30 * w * std::cos(w * x), ypp = 30 * w * w * std::sin(w * x);
      const double kt = ypp / std::pow(1 + yp * yp, 1.5);
      ea = std::max(ea, std::abs(a[i] - kt));
      const bool nearKnot = (i % 64) <= 1 || (i % 64) >= 63;
      if (!nearKnot) diffMid = std::max(diffMid, std::abs(m[i] - a[i]));
      if (std::abs(a[i]) > 1e-4) {
        int s = a[i] > 0 ? 1 : -1;
        if (la && s != la) ++signFlipsAnalytic;
        la = s;
      }
      if (std::abs(m[i]) > 1e-4) {
        int s = m[i] > 0 ? 1 : -1;
        if (lm && s != lm) ++signFlipsMenger;
        lm = s;
      }
    }
    std::printf("   true sine peak k = %.4e;   max analytic k = %.4e   max Menger k = %.4e\n", kSine,
                maxAbs(a), maxAbs(m));
    std::printf(
        "   max |analytic spline k - true sine k| = %.3e (spline approximates the sine; %.1f%% of peak)\n",
        ea, 100 * ea / kSine);
    std::printf(
        "   max |Menger - analytic| away from knots = %.3e (discretisation, small); sign flips (|k|>1e-4): "
        "analytic %d, Menger %d\n",
        diffMid, signFlipsAnalytic, signFlipsMenger);
    // Same after resampling to equal arc length (what makeSCurve does) - check Menger noise level
  }

  std::printf(
      "\n== 4. Menger vs analytic as the sampling density grows (circle M=32 and S-curve, away from "
      "knots)\n");
  std::printf("   %-8s %-22s %-22s\n", "samples", "circle max|Menger-anal|*R", "S-curve max|Menger-anal|");
  for (std::size_t s : {8u, 16u, 32u, 64u, 128u, 256u}) {
    SplineOptions o{s, 0.5};
    const auto cc = circleControl(100, 32);
    const auto ac = catmullRomCurvature(cc, Closure::Closed, o);
    const auto mc = catmullRomSpline(cc, Closure::Closed, o).curvature();
    double dc = 0;
    for (std::size_t i = 0; i < ac.size(); ++i) {
      const std::size_t r = i % s;
      if (r == 0 || r + 1 == s) continue;  // skip knot neighbours
      dc = std::max(dc, std::abs(mc[i] - ac[i]) * 100);
    }
    const auto cs = sineControl(300, 30, 16);
    const auto as = catmullRomCurvature(cs, Closure::Open, o);
    const auto ms = catmullRomSpline(cs, Closure::Open, o).curvature();
    double ds = 0;
    for (std::size_t i = 0; i + 1 < as.size(); ++i) {
      const std::size_t r = i % s;
      if (r == 0 || r + 1 == s) continue;
      ds = std::max(ds, std::abs(ms[i] - as[i]));
    }
    std::printf("   %-8zu %-22.3e %-22.3e\n", s, dc, ds);
  }

  std::printf("\n== 5. Closed-loop seam: circle M=32, analytic curvature at the seam samples vs elsewhere\n");
  {
    const auto ctrl = circleControl(100, 32);
    SplineOptions o{64, 0.5};
    const auto a = catmullRomCurvature(ctrl, Closure::Closed, o);
    double maxDev = 0;
    for (std::size_t i = 0; i < a.size(); ++i)
      maxDev = std::max(maxDev, std::abs(a[i] - a[(i + 64) % a.size()]));
    std::printf("   max |k[i] - k[i+64 (wraps over the seam)]| = %.3e  (rotation symmetry; rounding only)\n",
                maxDev);
  }
  return 0;
}
