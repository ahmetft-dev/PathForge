# Architecture

```
pathforge-studio (Qt 6 Widgets) ─┐
                                 ├─► pathforge::geometry   (no Qt, no third-party deps)
pathforge-cli                   ─┘
         │
         └─► pathforge::io (JSON, files; nlohmann/json) ─► pathforge::geometry
```

- **`libs/geometry`** – `Vec2`, validated immutable `Path` (open or closed), arc length, discrete curvature,
  `computeStats`, Catmull–Rom splines, arc-length resampling, tangent/normal frames, `Track` (centerline,
  widths, boundaries, local diagnostics) and the circle / rounded-rectangle (oval) / S-curve generators. See
  [geometry.md](geometry.md). Namespace `pathforge::geometry`, public headers under
  `libs/geometry/include/pathforge/geometry/`.
- **`libs/io`** – schema-v1 JSON parsing/serialisation and atomic file save/load (`pathforge::io`), Qt-free. It is
  the only module that depends on nlohmann/json (private to its sources). See [track-format.md](track-format.md).
- **`apps/pathforge-cli`** – headless front-end; proves the engine is usable without Qt.
- **`apps/pathforge-studio`** – Qt viewport and panels. It draws points produced by the library and shows
  `PathStats`; it performs no geometry calculations of its own (presets, boundaries and statistics all come from the library).
- **`tests/`** – GoogleTest, registered with CTest per test case.

## Decisions

- **Errors:** invalid geometry throws `GeometryError` from factories (`Path::create`, `makeCircle`), so an
  existing `Path` is always valid. `validate()` is the non-throwing variant.
- **Closed paths** do not repeat the first point; the closing segment is implicit. `arcLengths()` excludes it,
  `length()` includes it.
- **Curvature** is the signed Menger curvature at each vertex (CCW positive). It is exact for points on a circle
  and an approximation otherwise. Open-path endpoints copy the nearest interior value.
- **Reference lengths:** a sampled circle's length equals the inscribed N-gon perimeter, not 2πR; the difference
  (≈ π²R/(3N²)) is discretisation error and is reported as such.
- **Parse, build, validate are separate:** `io` yields an unvalidated `TrackData`; `buildTrack` creates the
  invariant-holding `Track`; `validateTrackData` reports geometric validity. Bad-but-representable data can be
  loaded and inspected.
- Only modules with real content exist; others from the roadmap are added when needed.
