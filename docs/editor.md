# Interactive track editor (PathForge Studio)

Status: implemented and tested locally (Debug/Release/GCC, native macOS and offscreen Qt tests). The new code has
**not yet run on GitHub Actions**.

## Using it

| Action | How |
| --- | --- |
| New track | File → New Track (empty document, *Add Points* mode on) |
| Add control points | Press **A** (toggle *Add Points*), click on empty space; points are appended in order |
| Select | Click a control point (8 px screen-space hit radius, same at every zoom). Click empty space to deselect |
| Move | Drag a selected/any control point; the geometry updates live. Typing X/Y in the *Selected point* panel also works |
| Insert | **Shift+click** next to the track (within 12 px): the new point goes on the nearest segment, a ring previews the spot |
| Delete | **Delete** / **Backspace** removes the selected point |
| Close / reopen loop | **L**, the *Closed Loop* toolbar button, or the checkbox in *Document* |
| Pan / zoom / fit | Drag on empty space / mouse wheel / **F** — unchanged from earlier phases |
| Widths | *Selected point* panel: left and right half-width in meters (independent); *Set all points to these widths* makes the track constant-width |
| Undo / redo | Platform-standard shortcuts (`QKeySequence::Undo` / `Redo`: Cmd+Z / Shift+Cmd+Z on macOS, Ctrl+Z / Ctrl+Y elsewhere); the *Edit* menu shows what will be undone |
| Open / Save / Save As / New | Standard shortcuts; unsaved changes trigger *Save / Discard / Cancel* before New, Open, preset generation and closing |
| Validate now | Ctrl+Shift+V (see *Live validation*) |

Presets (Circle, Oval, S-Curve) replace the document with exactly sampled **polyline** control points, so their
geometry is not altered by interpolation.

## Data model: editable source vs derived geometry

```
TrackDocument  (authoritative, Qt-free, libs/editor)
   name · open/closed · control points {stable id, x, y, leftWidth, rightWidth}
   settings {interpolation: catmull-rom|polyline, alpha, resampleSpacing}
        │  generateGeometry()                       (pure, deterministic, bit-identical for equal input)
        ▼
   interpolation → optional arc-length resampling → per-sample widths → Track (frames, boundaries)
        │
        ▼  DerivedGeometry {status, TrackData, Track, arc-length tables}
   validateTrackData()  →  ValidationReport   (Qt-free; worker thread for large tracks)
        │
        ▼
   Studio: EditorSession (owns the document, history, selection, derived data, validation state)
           TrackViewport / MainWindow (draw and forward input; no geometry code)
```

* **Control point ids** are never reused within a document and survive insertion, deletion, undo and redo.
  Indices are only positions. Ids are session identities and are not stored in files.
* **Invariants** (all mutators): finite coordinates within ±1e7 m, widths finite in [0, 1e4] m, valid settings. A rejected
  edit throws `EditError` and leaves the document unchanged (the revision counter only moves on real changes).
  *Geometric* validity is deliberately not an invariant: too few points, duplicates, zero widths and self-intersections
  stay editable and savable so that work in progress can be repaired.
* **Widths** are defined at control points and interpolated linearly in the arc length `s` of the dense interpolated
  polyline (periodically through the seam for closed tracks). A point inserted on the track takes the width the track
  has there, so inserting does not move the boundaries.
* **Resampling** measures arc length along the same dense polyline, so derived sample `k` is at `s = k · spacing`
  (the spacing is adjusted so that it fits the length exactly). With Catmull–Rom, every control segment gets
  `clamp(ceil(4·longest chord/spacing), 8, 1024)` dense samples (16 if resampling is off) so the dense polyline is
  several times finer than the output.
* **Incomplete documents** (open < 2 points, closed < 3, coincident neighbours) produce `DerivedStatus::TooFewPoints` /
  `Failed` with the raw control polygon, which the validation engine reports and Studio draws dashed.
* Hard limits (`kMaxDenseSamples` 4e6, `kMaxDerivedSamples` 1e6) turn absurd settings into a clear failure instead of
  exhausting memory.

## Undo / redo

Every user action is one `QUndoCommand` wrapping Qt-free document operations: add, insert, delete, move, width change,
open/close, settings, rename. A **drag** applies live updates directly (no history) and pushes **one** move command on
release, so one undo reverts the whole drag. Consecutive width edits of the same point merge. *Set all widths* is a
single command. Deleted points return with their original id and index. `QUndoStack::isClean()` drives the *modified*
indicator (`[*]` in the title); saving marks the current state clean, and undoing back to it makes the document clean again.

## Live validation

Measured cost (Release, Apple M4 Pro; see *Performance*): regenerating derived geometry is cheap (≤ 1.4 ms for 10,000
samples) but full validation is quadratic (4 ms at 1,000 samples, 45 ms at 4,000, 291 ms at 10,000). So:

* **During a drag** no validation runs. The viewport shows the live geometry; the diagnostics panel shows *pending* plus a
  cheap local check (`Track::diagnostics()`, O(N)): reversed or over-offset boundary segments.
* **After an edit** tracks with ≤ 1,500 samples (default `asyncThreshold`, ≈ 9 ms by quadratic extrapolation of the
  measurements) are validated synchronously *before* the change is announced, so geometry and report arrive together.
  Larger tracks are validated by `ValidationRunner` after a 150 ms debounce.
* **Worker rules:** one worker thread; the job gets an **immutable copy** of the derived data; at most one running job and
  one waiting request (latest wins), so the queue is bounded; results are tagged with the geometry *generation* and delivered
  through a queued call to the GUI thread, which accepts a result only if its generation is still the current one
  (obsolete results are dropped and counted); the runner's destructor waits for the running job, so shutdown with pending
  work cannot touch a destroyed object. A report is "current" only while its generation equals the geometry's; otherwise
  the UI says *pending* and overlays are not drawn.
* `EditorSession::validatedReport()` returns a report that is guaranteed current (it validates synchronously if needed) for
  future simulation/optimisation code; *Validate Now* forces the background run.
* Verified with deterministic gated-worker tests (stale results, newest-wins, bounded queue, thread identity, drag,
  debounce, replaced documents, shutdown with running + pending work) and with ThreadSanitizer (no reports).

## Rendering and level of detail

The viewport draws what it is given. Polylines are drawn with **one-pixel level of detail** (vertices closer than one screen
pixel to the previously drawn one are skipped; first/last always kept) and dense control handles are thinned to 6 px spacing
(selected and first point always drawn). The drawing is rebuilt when the zoom changes. Geometry, validation, issue overlays
and hit-testing always use full-resolution data. Tests check that every sample stays within one pixel of the drawn line.

## Performance

`benchmarks/editor_bench` (Qt-free) and `apps/pathforge-studio/bench/studio_bench` (`-DPATHFORGE_BUILD_BENCHMARKS=ON`),
Release, Apple M4 Pro, AppleClang 21, median values. `studio_bench` used Qt's **offscreen software-raster platform** with a
1280×800 window and `viewport()->grab()` as the repaint; native GPU-composited presentation was not measured.

| Document | Samples | Regenerate | Validate (full) | Edit + scene rebuild | Repaint | **Per mouse move** (p95) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| spline, 24 control points | 133 | 0.10 ms | 0.18 ms | 0.17 ms | 1.2 ms | **1.4 ms** (1.6) |
| spline, 24 control points | 1,015 | 0.32 ms | 4.1 ms | 0.41 ms | 3.8 ms | **4.2 ms** (4.3) |
| spline, 24 control points | 4,061 | 1.36 ms | 45 ms | 1.25 ms | 4.6 ms | **5.9 ms** (6.7) |
| spline, 24 control points | 10,155 | 1.37 ms | 291 ms | 1.81 ms | 4.7 ms | **6.5 ms** (7.5) |
| polyline preset, N control points | 1,000 | 0.03 ms | 3.0 ms | 0.25 ms | 4.2 ms | **4.4 ms** (5.1) |
| polyline preset, N control points | 4,000 | 0.12 ms | 44 ms | 0.43 ms | 4.8 ms | **5.2 ms** (5.3) |

Before the level-of-detail change the per-move cost was 34.3 ms (10,155 samples), 14.8 ms (4,061) and 28.0 ms (4,000-point
preset), dominated by repainting. After it, every measured case is below the 16.7 ms frame budget. Validation is not part of a
drag move and is reported separately; at 10,000 samples it takes ~0.3 s and runs on the worker. Debug builds are much slower.

## Persistence

Schema v2 stores the editable source; see [track-format.md](track-format.md). Legacy v1 (sampled) files open as polyline
documents without resampling.

## Known limitations

* The Catmull–Rom centerline is **C1 only**: curvature is discontinuous at control points and noisy near inflections
  ([curvature-study.md](curvature-study.md)). Nothing in the editor changes this; a C2 centerline is a Phase 2 prerequisite
  before curvature drives speed limits.
* A valid report means *no problem found by the implemented checks on the sampled polylines* ([validation.md](validation.md)):
  no area-overlap test, no drivability statement.
* Width interpolation is linear in arc length between control points; there are no width handles on the track itself.
* Undo history is per document and is cleared when a document is replaced (New / Open / preset); it is not persisted.
* Only one validation runs at a time; an obsolete running job cannot be interrupted and is waited for at shutdown (≤ one validation).
* Rendering performance was measured with the offscreen software rasteriser; the GUI was verified on native macOS (Qt tests
  and screenshots) but not interactively on Windows or Linux.
* A single point selection; no box select, multi-move or snapping/grid.
