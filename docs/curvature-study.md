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
* Prefer a **C2** centerline (natural/clamped cubic spline or cubic B-spline interpolation with chord-length
  parameterisation) or a smoothing fit, and evaluate curvature analytically on it; keep the Menger estimate for
  discrete polylines (tracks loaded from files) and apply a short arc-length smoothing window to it.
* Compute curvature on a polyline resampled at a spacing **no finer** than its source vertex spacing.
