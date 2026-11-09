# Geometry reference (`libs/geometry`)

Namespace `pathforge::geometry`. No Qt, no third-party dependencies. SI units (meters, radians).

## Conventions

- Right-handed frame: x right, y up; angles CCW positive. (Studio flips y only when drawing.)
- A `Path` is an ordered point list, `Closure::Open` or `Closure::Closed`. Closed paths **do not repeat**
  the first point; the closing segment is implicit. `Path::length()` includes it, `arcLengths()` does not.
- Tangent `T` points in the direction of travel (increasing index); **left normal `N = (-Ty, Tx)`**.
  For a CCW path `N` points to the inside of the turn, for a CW path to the outside.
- Curvature is **signed**: positive = left turn (CCW), negative = right turn.
- Invalid input (non-finite values, too few points, duplicate/zero-length segments, bad parameters) throws
  `GeometryError`; `validate()` is the non-throwing variant.

## Methods

| Function | Method | Accuracy / caveats |
| --- | --- | --- |
| `catmullRomSpline` | Interpolating Catmull–Rom, centripetal (`alpha = 0.5`) by default, Barry–Goldman evaluation. Closed paths wrap; open paths use reflected phantom end points. Control points are emitted exactly. | C1 only (curvature is discontinuous at control points). Overshoot and self-intersection are possible. Centripetal parameterisation avoids cusps/loops within a segment (Yuksel et al. 2011) but does not guarantee less overshoot. Output is a polyline: its length is below the true spline length by O(h²). |
| `resampleByArcLength` | Equal arc-length spacing measured **along the input polyline**, linear interpolation inside segments. Interval count `round(L / spacing)`, so the spacing is adjusted to fit exactly (reported in `ResampleResult::spacing`). Closed: seam not duplicated, ≥ 3 samples. | Not exact spline arc length — only as accurate as the input polyline. Chord distance between samples is slightly below the arc spacing (≈ `(κh)²/24` relative). Resampling **finer than the input's vertex spacing** makes the output's discrete curvature meaningless (it grows ∝ 1/spacing). |
| `Path::curvature` | Menger curvature of each vertex and its two neighbours (reciprocal radius of the circumscribed circle, signed). Open-path end points copy the nearest interior value. | Exact for points on a circle; otherwise a discrete estimate with noise ≈ position error / spacing². This is *not* a spline-derivative curvature and the two must not be mixed. |
| `computeFrames` | Tangent = length-weighted bisector of the unit chords at a vertex. | Exact tangent of the circle through three points (any spacing), 2nd-order on smooth curves; open-path ends are one-sided (1st order). Cusps fall back to the outgoing chord. |
| `catmullRomCurvature` | Analytic spline curvature from exact derivatives (separate from Menger). | Discontinuous at control points (C1 spline); see [curvature-study.md](curvature-study.md). |
| `Track::create` | Constant or per-vertex left/right half-widths; `left = c + wL·N`, `right = c − wR·N` per vertex. `sampleWidthProfile` / `evaluateWidthProfile` evaluate piecewise-linear arc-length width profiles at vertices / at arbitrary positions (periodic for closed paths). | Offsets are along vertex normals, not true parallel curves. |
| `makeCircle`, `makeRoundedRectangle`, `makeStadium` | Vertices lie exactly on the analytical curve at equal arc spacing. | Sampled perimeter is slightly below the analytical one (`≤ (h/r)²/24` relative). |
| `makeSCurve` | 17 control points on `y = −A sin(2πx/L)` → centripetal spline → arc-length resampling. | Approximates the sine; the spline's curvature deviates by up to ~27 % of the peak and has small sign flips near the ends/inflection, so count sign changes with a threshold (`countSignChanges`). See [curvature-study.md](curvature-study.md). |

## Track boundaries and validation

`Track::diagnostics()` only counts **local** problems (reversed offset segments, `width × curvature ≥ 1`). Use
`validateTrackData` / `validateTrack` ([validation.md](validation.md)) for global checks: centerline and boundary
self-intersections, left/right crossings, collapse, clearance. A track is only "valid" if all check groups ran
without errors, and even then not proven valid in the continuous sense or drivable. Files: [track-format.md](track-format.md).

## Presets

| Preset | Parameters (CLI/Studio) | Notes |
| --- | --- | --- |
| Circle | `radius`, `points` | closed, CCW, starts at (R, 0) |
| Oval | `length` (x extent), `width` (y extent), `points` | stadium, corner radius `width/2`, closed, CCW, starts at (0, −width/2); requires `length ≥ width`. Not an ellipse. |
| S-Curve | `length`, `amplitude`, `points` | open; left turn then right turn; from (0,0) to (length,0) |

All presets take a total `track width`, split equally left/right.
