# Track file format — schema version 1

JSON, UTF-8, produced and read by `libs/io` (`pathforge::io`, no Qt). Doubles are written in the shortest
form that parses back to the identical value, so a save/load round trip is **bit-exact**. Output is
deterministic (fixed key order, 2-space indent, trailing newline).

```json
{
  "schemaVersion": 1,
  "name": "Example Circuit",
  "units": "SI",
  "coordinateSystem": "xy-right-handed",
  "closed": true,
  "centerline": [
    { "x": 0.0, "y": -60.0 },
    { "x": 11.5, "y": -60.0 },
    { "x": 23.0, "y": -60.0 }
  ],
  "widths": { "left": 5.0, "right": 5.0 }
}
```

| Field | Type | Meaning |
| --- | --- | --- |
| `schemaVersion` | integer, required | Must be `1`. Other values are rejected with an upgrade hint. |
| `name` | string, required | Free-form track name. |
| `units` | string, required | Must be `"SI"`: meters and radians. |
| `coordinateSystem` | string, required | Must be `"xy-right-handed"`: x right, y up, counter-clockwise positive. |
| `closed` | boolean, required | `true`: the last point connects back to the first (do **not** repeat the first point). |
| `centerline` | array of `{x, y}`, required | Centerline vertices in direction of travel, meters. |
| `widths.left`, `widths.right` | number **or** array of numbers, required | Half-widths from the centerline [m] to the left/right boundary. A single number applies to every point (compact form, used when all values are equal); an array gives one value per centerline point. |

**Left/right** are relative to the direction of travel: `left = centerline + w · N`, `N = (-Ty, Tx)`.
**Boundaries are not stored**; they are recomputed on load (no redundant data that could disagree).

## Three separate stages

1. **Parsing** (`parseTrackJson` / `loadTrackFile`): syntax, required fields, types, schema version, finite numbers.
   Failure returns an actionable `error`; nothing else is changed. Numbers that overflow a double (`1e999`) and
   non-JSON literals (`NaN`, `Infinity`) are rejected. Documents nested deeper than 32 levels and files larger than
   128 MiB are rejected.
2. **Semantic content**: a successfully parsed file yields a `TrackData`, which may still describe an unusable track
   (empty centerline, duplicate points, negative or zero widths, width arrays of the wrong length). These **load**
   so that they can be inspected.
3. **Geometric validation** (`geometry::validateTrackData`): reports those problems and all global geometry
   issues; see [validation.md](validation.md).

## Unknown fields

Unknown fields in the document, in a point, or in `widths` are **ignored but reported** in
`LoadResult::warnings` (the CLI prints them to stderr, Studio shows them in the status bar). They are not preserved
on save; saving a loaded file therefore drops them (warned about on load, never silent).

## Saving

`saveTrackFile` writes `<path>.pathforge-tmp` in the same directory and renames it over the target, so a failed or
interrupted write never leaves a truncated target. With `OverwritePolicy::Fail` (default) an existing target is left
untouched and an error is returned. Data that cannot be represented (non-finite numbers, width arrays whose length
differs from the centerline) is refused rather than written lossily; geometrically invalid but representable tracks
**are** saved.
