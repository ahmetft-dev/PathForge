#pragma once

#include <cstddef>
#include <pathforge/geometry/path.hpp>
#include <pathforge/geometry/vec2.hpp>
#include <vector>

namespace pathforge::geometry {

/// Analysis of the track SURFACE represented by the sampled corridor.
///
/// Strip i is the quadrilateral Q_i = (L_i, L_{i+1}, R_{i+1}, R_i) between corresponding left/right boundary
/// samples (closed tracks: strip n-1 closes the loop back to sample 0; open tracks have n-1 strips). For a
/// CCW centerline the strips of a well-formed corridor are clockwise (negative signed area): the left
/// boundary is on the left of travel, so walking L_i -> L_{i+1} -> R_{i+1} -> R_i goes round the strip
/// clockwise. A closed corridor is an ANNULUS, so it is never treated as one simple polygon: only individual
/// strips and pairs of strips are examined.
///
/// This validates the sampled corridor approximation. It says nothing about the continuous surface between
/// samples and nothing about drivability.
enum class StripDefectKind {
  Degenerate,  ///< |signed area| negligible relative to length x width (collapsed strip)
  Folded,      ///< reversed orientation (left/right swapped) or self-intersecting (bow-tie) strip
  Overlap,     ///< two strips cover a common area (see below for what is NOT an overlap)
};

struct StripDefect {
  StripDefectKind kind;
  std::size_t strip;  ///< the strip (for Overlap: the lower-indexed one)
  std::size_t other;  ///< Overlap only: the other strip
  Vec2 location;      ///< a point inside / on the defect [m]
};

enum class BroadPhase { Grid, BruteForce };

struct SurfaceOptions {
  /// A strip is degenerate when |area| <= degenerateAreaRatio * (mean boundary-edge length) * (mean
  /// cross-section width). Well-formed strips have a ratio of about 1; 1e-9 only catches collapse to a
  /// line/point, not narrow-but-real strips.
  double degenerateAreaRatio = 1e-9;
  BroadPhase broadPhase = BroadPhase::Grid;
  /// Strips whose index is flagged here are skipped completely (e.g. samples with zero width, already
  /// reported elsewhere).
  std::vector<char> skipStrip;
};

struct SurfaceAnalysis {
  std::vector<StripDefect> defects;
  std::size_t stripCount = 0;
  std::size_t candidatePairs =
      0;  ///< non-adjacent strip pairs whose bounding boxes overlap (given to the narrow phase)
};

/// Legitimate contacts that are NOT overlaps: adjacent strips (i, i+1, also the seam pair) share exactly
/// their common cross-section edge L_{i+1}R_{i+1}; contact exactly at that edge or at the shared vertices is
/// expected. For adjacent strips an overlap means: a proper crossing of other edges, a collinear fold-back,
/// or a non-shared vertex strictly inside the other strip. For NON-adjacent strips (which share no vertex in
/// a well-formed corridor) any common point - edge crossing, touching, collinear overlap or containment of a
/// vertex - is an overlap. Folded or degenerate strips are reported as such and excluded from overlap tests
/// (their overlaps are a consequence).
///
/// Predicates: orientation() filter (scale-relative error bound, see predicates.hpp) and a crossing-number
/// point-in-polygon built on it. Complexity: O(S) per-strip and adjacent-pair checks plus the narrow phase on
/// candidate pairs; the grid broad phase makes the candidate count roughly linear in S for tracks without
/// many nearby strips; BruteForce is the O(S^2) bounding-box sweep used as the reference.
SurfaceAnalysis analyzeSurface(const std::vector<Vec2>& left, const std::vector<Vec2>& right, Closure closure,
                               const SurfaceOptions& options = {});

/// A connected region of overlapping strip pairs: pairs (i, j) and (i', j') are connected when i, i' and j,
/// j' are each at most one apart (cyclically for closed tracks). One region is ONE defect, whatever number of
/// pairs it contains.
struct OverlapRegion {
  std::size_t firstA, lastA;  ///< strip range of one side (indices as reported, not unwrapped)
  std::size_t firstB, lastB;  ///< strip range of the other side
  std::size_t pairCount;
  Vec2 location;
  std::vector<std::size_t>
      strips;  ///< every strip that takes part (for deduplication against other diagnostics)
};

std::vector<OverlapRegion> clusterOverlaps(const std::vector<StripDefect>& defects, std::size_t stripCount,
                                           bool closed);

/// Crossing-number point-in-polygon with orientation-predicate side tests. Boundary points count as Boundary.
enum class PointLocation { Outside, Boundary, Inside };
PointLocation locatePoint(Vec2 q, const std::vector<Vec2>& polygon);

}  // namespace pathforge::geometry
