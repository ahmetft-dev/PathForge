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
