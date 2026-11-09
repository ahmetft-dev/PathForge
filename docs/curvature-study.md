# Curvature quality study (Phase 1B)

Question: Phase 1A saw sign flips and noise in the discrete (Menger) curvature of Catmull–Rom splines (S-curve
inflection, control-point transitions). Is that an artifact of the discrete estimator or a property of the spline?

Method: `benchmarks/curvature_study.cpp` (build with `-DPATHFORGE_BUILD_BENCHMARKS=ON`) compares

* **Menger curvature** `Path::curvature()` — circumscribed-circle curvature of three consecutive samples, and
* **analytic spline curvature** `catmullRomCurvature()` — `κ = (x′y″ − y′x″)/(x′²+y′²)^{3/2}` with exact
  first/second derivatives (product rule through the Barry–Goldman pyramid), evaluated at the same samples.

They are different quantities and are provided as separately named methods; the existing Menger implementation is
unchanged.

## Results (numbers produced by the study program, centripetal α = 0.5)

| Case | Finding |
| --- | --- |
| Straight line (5 collinear, uneven points) | analytic 8.5e-16, Menger 1.5e-12: both zero to rounding |
| Circle R = 100, closed, 64 samples/segment | spline-vs-true-circle curvature error `max|κR − 1|`: M = 8: 0.51, 16: 0.119, 32: 0.029, 64: 0.0072, 128: 0.0018 → **O(h²), factor ≈ 4 per doubling**, for analytic and Menger alike |
| Menger vs analytic, circle M = 32, away from knots | 4.6e-4 (8 samples/segment) → 1.2e-4 → 2.9e-5 → 7.3e-6 → 1.8e-6 → 4.6e-7 (256): **O(h²) agreement** |
| Same, S-curve (17 control points) | 3.5e-6 → … → 3.5e-9 (same O(h²) trend) |
| Jump of analytic κ at a knot (circle) | 5.6 % of 1/R (M = 8), 1.1 % (16), 0.27 % (32), 0.067 % (64) — the spline's curvature is **discontinuous** at control points |
| S-curve vs the sine's true curvature | spline κ deviates by up to 27 % of the sine's peak κ (17 control points); peak spline κ 1.457e-2 vs sine 1.316e-2 (+11 %) |
| Sign flips on the S-curve (|κ| > 1e-4) | analytic 5, Menger 5 — identical |
| Closed seam (circle M = 32) | analytic κ repeats with the control period across the seam to 4e-16: no seam artifact |

## Conclusions

1. **Menger curvature of a densely sampled spline is a faithful estimate of the spline's own curvature**
   (agreement O(h²) in the sample spacing). The noise is **not** a discretisation artifact.
2. The noise is **intrinsic to Catmull–Rom**: it is C1 only, curvature jumps at every control point and the
   spline's curvature itself oscillates around the true curvature of the shape it interpolates (the sine, the
   circle). The inflection sign flips are present in the analytic curvature too.
3. The error of the spline curvature w.r.t. the underlying smooth shape shrinks only as O(h²) in the *control*
   spacing; for coarse control polygons (≤ 16 points per period) it is large (≥ 12 % on a circle).
4. The seam of closed splines is handled correctly.
5. Resampling finer than the input vertex spacing (see `resample.hpp`) is a separate, real discretisation artifact
   (turning angle ÷ smaller spacing) and was fixed in `makeSCurve` by oversampling the dense spline.

## Limitations of this study

Only the centripetal parameterisation was run; only two shapes (circle, sine S-curve); samples within one
segment of a knot were excluded from the Menger-vs-analytic comparison because the curvature jumps there.
Curvature of the polyline *offset boundaries* was not studied.

## Recommendation for Phase 2 (speed profiles need κ)

* Do **not** feed raw Catmull–Rom or Menger curvature of coarse control polygons directly into cornering-speed
  limits: speed `v = sqrt(a_lat / |κ|)` amplifies the ±10–30 % curvature noise.
* Prefer a **C2** centerline (Phase 1D provides the natural/periodic cubic spline; see below) or a smoothing fit, and evaluate curvature analytically on it; keep the Menger estimate for
  discrete polylines (tracks loaded from files) and apply a short arc-length smoothing window to it.
* Compute curvature on a polyline resampled at a spacing **no finer** than its source vertex spacing.

# Phase 1D: C2 cubic spline and verified curvature accuracy

Phase 1B recommended a C2 centerline before curvature feeds speed limits. Phase 1D adds one (`CubicSpline`,
`cubicSpline()`, `cubicSplineCurvature()`; method in [geometry.md](geometry.md)) and **measures** how the three
curvature quantities compare with analytic references: the discrete Menger curvature of a sampled polyline, the
analytic curvature of the Catmull–Rom spline and the analytic curvature of the C2 spline. The tests are in
`tests/curvature_accuracy_test.cpp`; each tolerance in them is derived from the measured convergence order and the
test comment says which measurement it comes from. C2 is **not** claimed to be better in general.

| Reference | Result (centripetal α = 0.5 unless noted) |
| --- | --- |
| Straight line | all three methods are zero to rounding, frames stable |
| Circle R, M control points per turn | C2 error `max|κR − 1|·M²` = 3.66 (M=8), 3.38 (16), 3.31 (32), 3.29 (64): **O(h²)**, ratios 3.9–4.5. Catmull–Rom has the same order with a ≈ 9× larger constant (≈ 30/M²). Scale-invariant. |
| Menger of a sampled circle | exact (vertices on the circle) |
| S-curve `y = −A sin(2πx/L)`, K intervals | C2 error·K²/(A(2π/L)²) ≈ 6 for every K (O(h²)); exactly one sign change at the middle at every resolution; Catmull–Rom is much less accurate. The sine has zero curvature at its ends, which matches the natural boundary condition, so this case is *favourable* to C2 and says nothing about arbitrary open paths. |
| Menger vs analytic curvature of the *same* sampled C2 curve | ratio per halving of the sample spacing 3.25, 3.65, 3.83, 3.92 away from knots (O(h²)); **exactly ≈ 2 (O(h)) at the samples next to a knot**, because κ′ jumps there. A property of the estimator, not of the spline. |
| Ellipse (closed, analytic κ) | error relative to peak: 13 % (M=12), 5.5 % (20), 1.3 % (40), 0.32 % (80); continuous across the seam; Catmull–Rom jumps at knots, C2 does not. |
| Exact stadium (κ steps 0 → 1/r) | no continuous curve can keep a step. C2 range [−0.18, 1.06]/r, Catmull–Rom [−0.63, 1.35]/r for r = 60 m, 64 points: both ripple, C2 less. |
| 180° hairpin | smooth curvature does **not** mean a closer path: C2 overshoots the control polygon by 2.16 m, Catmull–Rom by 1.66 m (peak |κ| 0.71 vs 1.13). Chordal knots (α = 1) overshoot far more (7.7 m vs 6.0 m), which is why 0.5 is the default. |
| Nearly coincident control points | the curvature is either finite or **flagged invalid** (`CurvatureValue::valid == false`, NaN in the vector API); never a plausible-looking zero or a huge number. |

## What C2 does and does not give

* Gives: continuous position, tangent and curvature; no curvature jump at control points or at the seam; analytic curvature
  independent of the sampling.
* Does not give: a curve closer to the intended shape (hairpin), freedom from overshoot or self-intersection, a free
  end condition (open splines are *natural*: κ = 0 at both ends), or a continuous derivative of curvature (κ′ jumps at knots).
* Which interpolation to use is the user's choice; the editor sets Catmull–Rom by default and neither is presented as better.

## Arc-length consistency

Widths, point insertion and output spacing all use arc length measured along the *dense polyline* of the spline, and the
resampling measures along that same polyline, so they agree with each other exactly. The only error is dense-polyline length vs
true curve length, measured against 10-point Gauss–Legendre quadrature (32 panels per segment) on an ellipse, a hairpin and an
S-curve: the polyline is always *shorter*, and the relative error is ≈ −0.003 … −0.005 / (samples per segment)² (ellipse: −1.8e-4
at 4, −4.5e-5 at 8, −1.1e-5 at 16, −7e-7 at 64 samples per segment). The editor uses ≥ 8 samples per segment and about four per
output spacing, so the length error is below 1e-4 relative. **No arc-length integration was added** — the measurement showed it
unnecessary. Pinned by `CubicSplineArcLength.*`.

## Limitations

Centripetal and chordal parameterisations only (uniform is allowed but was not studied); open splines use natural ends; boundary
polylines (offsets) were not studied; all figures are for the shapes listed.
