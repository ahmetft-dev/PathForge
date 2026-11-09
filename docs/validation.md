# Track validation

`geometry::validateTrackData(TrackData)` and `geometry::validateTrack(Track)` return a `ValidationReport`.
Independent of Qt; used by the CLI (`track validate|inspect`, synthetic-track reports) and Studio.

## What is checked

| Group (`CheckCoverage`) | Issues (`TrackIssue`) |
| --- | --- |
| structure | `TooFewPoints`, `WidthCountMismatch`, `InvalidWidth` (negative / non-finite / **zero**), `NonFiniteCoordinate`, `DegenerateSegment` (zero length, including the closing segment — i.e. a closed path that repeats its first point) |
| centerline | `CenterlineSelfIntersection` |
| boundaries | `LeftBoundarySelfIntersection`, `RightBoundarySelfIntersection` |
| crossings | `BoundaryCrossing` (left vs right boundary) |
| collapse | `BoundaryCollapse` (a boundary touches the centerline; actual clearance below `collapseClearance`) |
| local folds | `LocalBoundaryFold` (error: reversed offset segment; warning: offset ≥ local radius of curvature) |
| clearance | `NarrowSection` (warning: actual clearance below `narrowWidthWarning`) |

Each `ValidationIssue` carries a severity (`Info`/`Warning`/`Error`), affected segments (`SegmentRef`: polyline +
index), an approximate location and a message. At most `maxIssuesPerType` (default 25) issues per type are listed;
`errorCount`/`warningCount`/`suppressed` stay exact.

## Status: valid, invalid, incomplete

* **Invalid** — at least one error.
* **Incomplete** — no error, but some check group did not run (never reported as valid).
* **Valid** — all groups ran and found no error (warnings are allowed).

If the *structure* is bad (e.g. NaN, duplicate points, mismatched widths) no `Track` can be built, the geometric
groups are **skipped and recorded as not run**. **"Valid" means: no problem was found by these checks on the sampled
polylines.** It is not a proof about the continuous track (features smaller than the sampling can be missed) and says
nothing about drivability. In particular, overlap of the *areas* covered by distant parts of the track is only
detected through the boundary crossings it normally causes; there is no area-overlap test.

## Intersection predicates and tolerances

* `orientation(a, b, c)` evaluates the 2D determinant in double precision and trusts its sign only if it exceeds
  Shewchuk's forward error bound `(3 + 16ε)ε (|detleft| + |detright|)`, ε = 2⁻⁵³. The bound is **relative to the
  magnitudes involved**, so there is no absolute epsilon and results are identical under power-of-two scaling and
  translation (tested from 1e-9 to 1e9 and with 1e6 m offsets). Unreliable signs become "collinear (uncertain)".
  This is a filter, not exact arithmetic: signs are never wrong, but configurations within ~1e-16 relative of
  collinear are treated as collinear.
* Segment contacts are classified as `ProperCrossing`, `Touching` (endpoint on the other segment),
  `CollinearOverlap` or `Disjoint`. For **non-adjacent** segments every contact except `Disjoint` is an
  intersection error (a boundary touching itself is invalid for a track).
* **Adjacency:** segments sharing an endpoint — including the closed-loop seam pair (last, first) — may meet only at
  that point. Adjacent segments that fold exactly back (collinear, anti-parallel: a zero-width spike) are reported.
* Degenerate segments use the absolute minimum length of `Path::create` (1 nm). Coordinates above ~1e6 m are fine for
  the predicates but lose sub-millimetre representability in double precision.
* Thresholds `narrowWidthWarning` (2 m) and `collapseClearance` (1 cm) are **generic geometry thresholds, not
  vehicle requirements**, and are configurable (`ValidationOptions`).

## Clearance quantities (three different things)

* **Configured width** — `left_i + right_i`, an input property.
* **Same-vertex distance** — `|left_i − right_i|`. Equals the configured width and is **not** a clearance: it stays
  large when boundaries fold or distant parts of the track approach.
* **Boundary polyline distance** (`minBoundaryDistance`, per vertex in `vertexClearance`) — actual minimum Euclidean
  distance between the left and the right boundary polylines (vertex-to-segment distances in both directions with
  exact pruning). For polylines that do not cross this is exactly the polyline-to-polyline minimum; it is 0 when
  they meet. It describes the sampled polylines, not the continuous curves, and **is never used to certify
  validity**. Distances between non-adjacent parts of the same boundary are not part of it.

## Complexity and measured performance

All pair sweeps are O(N²) in the number of points with bounding-box rejection (4 comparisons per pair); there is
about 7 such sweeps plus a pruned O(N²)-worst-case clearance search. No spatial index was added because it is not
needed at the measured sizes. Measured with `benchmarks/validation_bench` (Release, Apple M4 Pro, AppleClang 21,
median of 7 runs, valid tracks of 10 m total width; single-threaded):

| N points | circle | stadium | S-curve |
| ---: | ---: | ---: | ---: |
| 100 | 0.15 ms | 0.04 ms | 0.04 ms |
| 1,000 | 4.1 ms | 3.1 ms | 3.1 ms |
| 4,000 | 46 ms | 46 ms | 47 ms |
| 10,000 | 294 ms | 292 ms | 306 ms |

Time per N² is ≈ 3 ns for N ≥ 4000 (quadratic scaling confirmed between 4,000 and 10,000 points). These numbers are
for well-behaved tracks where bounding boxes reject most pairs; tracks with very many overlapping segment boxes
(dense spirals, extremely long thin features) were not measured and may be slower. Other machines and compilers were
not measured.

## Live editing (Phase 1C)

The validation API is unchanged; the editor uses it as described in [editor.md](editor.md#live-validation). Measured facts
(Release, Apple M4 Pro): regenerating the derived geometry after an edit costs 0.1 ms (100 samples) to 1.4 ms (10,000 samples)
and validation costs 0.18 ms, 4.1 ms, 45 ms and 291 ms for 100, 1,000, 4,000 and 10,000 samples, so validation dominates and is
therefore (a) skipped during drags, (b) synchronous only up to 1,500 samples (≈ 9 ms, extrapolated from the quadratic fit, not
separately measured) and (c) run on a worker thread for larger tracks, on an immutable copy, with results tagged by geometry
generation and obsolete ones dropped. No spatial index was added: at the measured sizes validation is never on the interactive
path. Tracks much larger than the benchmarked ones, or with many overlapping segment bounding boxes, may need the bounding-box
grid mentioned earlier.

## Corridor surface (Phase 1D)

The track surface is approximated by **strips** `Q_i = (L_i, L_{i+1}, R_{i+1}, R_i)` between corresponding left/right samples
(one per centerline segment; a closed track is an annulus, the last strip joins the first through the seam). In a well-formed
corridor every strip is clockwise (negative signed area). `analyzeSurface()` (`surface.hpp`) reports

| Defect | Meaning | Issue type |
| --- | --- | --- |
| Folded | strip reversed (left/right swapped) or self-intersecting (bow-tie) | `FoldedTrackStrip` |
| Degenerate | area ≈ 0 relative to length × width (collapsed to a line/point) | `DegenerateTrackStrip` |
| Overlap | two strips cover a common area | `SurfaceOverlap` |

What is **not** an overlap: adjacent strips (including the seam pair) share exactly their common cross-section edge `L_{i+1}R_{i+1}`;
touching there, or at the shared vertices, is expected. For adjacent strips only a proper crossing of other edges, a collinear
fold-back or a non-shared vertex strictly inside the other strip counts. Non-adjacent strips share nothing in a well-formed
corridor, so any common point (crossing, touching, collinear overlap, contained vertex) is an overlap. Folded or degenerate strips
are excluded from the overlap tests (their overlaps are a consequence).

**Predicates and cost.** Orientation uses the scale-relative filtered predicate (Shewchuk-style error bound) with a crossing-number
point-in-polygon; strip areas are computed relative to a vertex so coordinates far from the origin do not cancel (tested at 1e6 m
offsets and 1e-3…1e3 scales). The broad phase is a uniform grid with reference-cell de-duplication, O(S) for evenly distributed
strips; `BroadPhase::BruteForce` is the O(S²) reference used as an oracle in the tests.

**No diagnostic floods.** Overlapping strip pairs are clustered into connected *regions* (one `SurfaceOverlap` per region, referencing
the first strip of each side). A surface defect that is already explained by a boundary-level issue (a strip implicated in a boundary
self-intersection, left/right crossing, local fold or collapse) is not reported a second time; it is counted in
`ValidationReport::surfaceDefectsMerged`. In practice almost every genuinely overlapping corridor also produces boundary crossings, so
the surface group mostly *confirms* those and adds its own issue only where the boundary checks stay silent. Strips touching a
zero-width sample are skipped (already `InvalidWidth`) and then `checked.surface` stays false (Incomplete rather than Valid).

**Status.** `CheckCoverage::surface` is part of `all()`; Valid/Invalid/Incomplete semantics are unchanged. Like all checks this
validates the *sampled* corridor and says nothing about the continuous surface between samples or about drivability.

## Spline health (Phase 1D)

For C2 documents the editor evaluates the spline analytically at every control point and three interior fractions per segment.
Zero velocity (cusp/stationary point) gives `InvalidSplineDerivative`; a non-finite derivative or curvature gives
`CurvatureEvaluationFailure`. Both are errors, one issue per kind with the number of affected points, merged into the report by
`mergeSplineHealth()` on the synchronous, asynchronous and CLI paths. A real stationary point is practically unreachable from
distinct control points with the centripetal knots (the unit test therefore uses a synthetic record for the merge); the check is a
guard so that a future curvature consumer never receives a silently meaningless value. Catmull–Rom and polyline interpolation are not checked.

## Measured cost with surface analysis

Release, Apple M4 Pro, AppleClang 21, median of 9 (`benchmarks/editor_bench`, closed ellipse, 1 m spacing):

| Samples | surface (grid) | surface (brute force) | full validation |
| ---: | ---: | ---: | ---: |
| ≈100 | 0.08 ms | 0.08 ms | 0.13 ms |
| ≈1,000 | 0.87 ms | 1.2 ms | 4.0 ms |
| ≈4,000 | 3.5 ms | 9.6 ms | 50 ms |
| ≈10,000 | 8.7 ms | 45 ms | 306 ms |

Surface analysis is linear (≈ 0.9 µs per sample) while the existing boundary sweeps stay quadratic, so it adds ≈ 3 % at 10,000
samples and ≈ 25 % at 1,000. The synchronous threshold of 1,500 samples is therefore ≈ 8 ms (extrapolated, not separately measured).
