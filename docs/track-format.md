# Track file format

Two schema versions exist. **Version 2** stores the *editable source* of a track (control points and settings) and is what
Studio saves. **Version 1** stores *sampled geometry* (what the CLI exports) and is still read everywhere. Both use the
same header fields; unsupported versions are rejected with an upgrade hint. The sections below describe version 1 first,
then version 2.

## Schema version 1 (sampled track)

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

## Schema version 2 (editable document)

Stores what the user authored, never derived samples or boundaries; those are regenerated on load, deterministically.

```json
{
  "schemaVersion": 2,
  "name": "Editable circuit",
  "units": "SI",
  "coordinateSystem": "xy-right-handed",
  "closed": true,
  "geometry": { "interpolation": "catmull-rom", "alpha": 0.5, "resampleSpacing": 2.0 },
  "controlPoints": [
    { "x": 100.0, "y": 0.0, "leftWidth": 5.0, "rightWidth": 5.0 },
    { "x": 70.710678, "y": 70.710678, "leftWidth": 6.5, "rightWidth": 4.0 }
  ]
}
```

| Field | Meaning |
| --- | --- |
| header (`schemaVersion`, `name`, `units`, `coordinateSystem`, `closed`) | as in version 1 |
| `geometry.interpolation` | `"catmull-rom"` (centripetal family, C1) or `"polyline"` (straight segments, exact) |
| `geometry.alpha` | Catmull–Rom knot exponent in [0, 1] (0.5 = centripetal); stored for both kinds |
| `geometry.resampleSpacing` | equal arc-length spacing [m] of the derived samples; `0` = no resampling |
| `controlPoints[]` | control points in order: position and the left/right half-widths **at that point** [m]; widths are interpolated linearly in arc length between points (periodically for closed tracks) |

Rules: doubles round-trip bit-exactly; positions must be finite with |x|, |y| ≤ 1e7 m, widths finite in [0, 1e4] m;
coordinates and widths outside those limits are rejected with the offending point named. Geometrically unusable documents
(too few points, duplicates, zero widths, self-intersections) **load fine** so that they can be repaired and saved. Control
point ids are session identities and are not stored. Unknown fields are ignored and reported as warnings; saving drops them.
Save → load → save is byte-identical.

### Reading and migration

`loadTrackDocumentFile()` (and Studio, and `pathforge-cli track …`) accepts both versions:

* **v2** → the document exactly as saved.
* **v1** → imported as a **polyline** document with resampling **off**, one control point per stored sample, widths from the
  stored per-sample widths. The derived geometry then equals the stored samples **bit-for-bit**; no spline is fitted, because
  re-interpreting samples as Catmull–Rom control points would change the geometry. Fitting a smooth spline to a sampled track is
  a deliberate conversion that does not exist yet. A v1 track that cannot be represented as a document (e.g. a negative width)
  still loads as sampled data (CLI validation) but is refused by the editor with an explanation.
* Studio does not remember the path of an imported v1 file, so *Save* cannot silently overwrite it with a v2 document; *Save As*
  writes schema v2. `parseTrackJson()`/`loadTrackFile()` (the sampled-track API) stay v1-only and point v2 files to the document API.

Fixtures: `tests/data/editable_*_v2.json`.
