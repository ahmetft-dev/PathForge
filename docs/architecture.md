# Architecture

```
pathforge-studio (Qt 6 Widgets) ─┐
                                 ├─► pathforge::geometry   (no Qt, no third-party deps)
pathforge-cli                   ─┘
```

- **`libs/geometry`** – `Vec2`, validated immutable `Path` (open or closed), arc length, discrete curvature,
  `computeStats`, synthetic circle generator. Namespace `pathforge::geometry`, public headers under
  `libs/geometry/include/pathforge/geometry/`.
- **`apps/pathforge-cli`** – headless front-end; proves the engine is usable without Qt.
- **`apps/pathforge-studio`** – Qt viewport and panels. It draws points produced by the library and shows
  `PathStats`; it performs no geometry calculations of its own.
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
- Only modules with real content exist; others from the roadmap are added when needed.
