#pragma once

#include <cstddef>
#include <pathforge/geometry/path.hpp>
#include <pathforge/geometry/spline.hpp>
#include <vector>

namespace pathforge::geometry {

/// Result of a curvature evaluation. `valid` is false (and `value` NaN) where the curvature is not defined or
/// cannot be evaluated reliably; callers must check it instead of using `value` blindly.
struct CurvatureValue {
  double value;
  bool valid;
};

/// Signed curvature  kappa = (x' y'' - y' x'') / (x'^2 + y'^2)^(3/2)  from first and second derivatives with
/// respect to ANY common parameter (the curvature is parameterisation invariant).
///
/// Invalid (value = NaN, valid = false) when a derivative is non-finite or when the speed |r'| is below
/// `minRelativeSpeed * speedScale`. `speedScale` is a reference speed of the curve (e.g. the largest knot
/// speed); the relative test keeps the decision independent of the units and of the parameterisation. Below
/// that threshold the curve is stationary or nearly so (a cusp or a degenerate point) and 1/speed^3 would
/// amplify rounding noise into a plausible-looking but meaningless number, so no number is returned.
inline constexpr double kMinRelativeSpeed = 1e-6;
CurvatureValue curvatureFromDerivatives(Vec2 d1, Vec2 d2, double speedScale);

/// C2-continuous interpolating parametric cubic spline r(t) = (x(t), y(t)) through the control points.
///
/// Parameterisation: knots t_0 = 0, t_{i+1} = t_i + |P_{i+1} - P_i|^alpha with alpha in [0, 1] (1 = chord
/// length, 0.5 = centripetal, 0 = uniform), the same convention as the Catmull-Rom spline. x(t) and y(t) are
/// each the classical cubic spline through (t_i, P_i) and therefore C2 in t; since the curve is regular (r'
/// != 0) the geometric curvature is continuous as well. The parameter only changes HOW the control points are
/// connected, never that they are interpolated.
///
/// Boundary conditions:
///  - Open: natural, r''(t_0) = r''(t_{n-1}) = 0 (zero curvature at both ends; exact for a sine centerline
///  sampled from
///    an inflection to an inflection, but a restriction in general).
///  - Closed: periodic, r, r' and r'' match across the seam t_n = t_{n-1} + |P_0 - P_{n-1}|^alpha, so there
///  is no special
///    treatment of the first point. The last point is NOT repeated.
///
/// Numerics: the second derivatives M_i solve one tridiagonal (open) or cyclic tridiagonal (closed,
/// Sherman-Morrison) system per coordinate by the Thomas algorithm without pivoting. Row i reads
///   h_{i-1} M_{i-1} + 2 (h_{i-1} + h_i) M_i + h_i M_{i+1} = 6 (delta_i - delta_{i-1}).
/// After dividing by 2 (h_{i-1} + h_i) the diagonal is 1 and the off-diagonal entries sum to 1/2, so the
/// matrix is strictly diagonally dominant by a factor of 2 whatever the spacing: elimination without pivoting
/// is stable and the row-scaled condition number is at most 3. Spacing ratios far from 1 do not hurt the
/// conditioning, but they do enlarge the overshoot of the interpolant (tested). Cost: O(n) to build, O(1) per
/// evaluation.
///
/// Limitations: a C2 curve can overshoot, form loops or self-intersect between control points exactly like
/// other interpolants; smoother curvature does not make the curve closer to an intended shape (see
/// docs/curvature-study.md).
///
/// Immutable value type, deterministic, thread-safe for concurrent const use.
class CubicSpline {
 public:
  /// Position, first and second derivative with respect to the knot parameter t.
  struct Sample {
    Vec2 position;
    Vec2 d1;
    Vec2 d2;
  };

  /// @throws GeometryError for invalid control points (non-finite, too few, consecutive duplicates incl. the
  /// closing
  ///   pair for closed curves) or alpha outside [0, 1].
  static CubicSpline create(const std::vector<Vec2>& control, Closure closure, double alpha = 0.5);

  Closure closure() const { return closure_; }
  std::size_t controlCount() const { return points_.size(); }
  /// n - 1 (open) or n (closed).
  std::size_t segmentCount() const {
    return closure_ == Closure::Closed ? points_.size() : points_.size() - 1;
  }
  /// Knot t_i for i in [0, segmentCount()]; the last knot is the total parameter length.
  double knot(std::size_t i) const { return knots_[i]; }
  double parameterLength() const { return knots_.back(); }
  /// Largest |r'| over the knots: the reference speed for degeneracy tests.
  double speedScale() const { return speedScale_; }

  /// Evaluates segment `segment` (0 <= segment < segmentCount()) at local fraction u in [0, 1] (t = t_i + u
  /// h_i). u = 0 and u = 1 return the control points exactly (no rounding).
  Sample evaluate(std::size_t segment, double u) const;
  /// Evaluates at a global parameter t in [0, parameterLength()]; closed curves are periodic. Locating the
  /// segment is O(log n).
  Sample evaluateAt(double t) const;

  CurvatureValue curvature(std::size_t segment, double u) const;

  /// Second derivatives at the knots (exposed for verification of the linear solve).
  const std::vector<Vec2>& knotSecondDerivatives() const { return m_; }

 private:
  Closure closure_ = Closure::Open;
  std::vector<Vec2> points_;
  std::vector<double> knots_;  // size segmentCount() + 1
  std::vector<Vec2> m_;  // second derivatives at the control points (index modulo n for the closing knot)
  double speedScale_ = 0.0;
};

/// Samples the C2 spline like catmullRomSpline(): `samplesPerSegment` samples per control interval at local
/// fractions k / samplesPerSegment (uniform in the knot parameter, control points emitted exactly); closed: n
/// * samplesPerSegment samples, no repeated seam point; open: (n - 1) * samplesPerSegment + 1.
/// @throws GeometryError like CubicSpline::create(), and for an invalid samplesPerSegment.
Path cubicSpline(const std::vector<Vec2>& control, Closure closure, SplineOptions options = {});

/// ANALYTIC curvature of the same spline at exactly the samples cubicSpline() emits (same size and order).
/// Entries are NaN where the curvature is not valid (see curvatureFromDerivatives()). Not the Menger
/// curvature of the polyline.
std::vector<double> cubicSplineCurvature(const std::vector<Vec2>& control, Closure closure,
                                         SplineOptions options = {});

}  // namespace pathforge::geometry
