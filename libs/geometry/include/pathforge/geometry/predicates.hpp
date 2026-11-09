#pragma once

#include <cstddef>
#include <pathforge/geometry/path.hpp>
#include <pathforge/geometry/vec2.hpp>
#include <vector>

namespace pathforge::geometry {

/// Orientation of c relative to the directed line a -> b:
///   +1  c is strictly left of the line (counter-clockwise turn a, b, c)
///   -1  c is strictly right
///    0  c is collinear OR too close to the line for the floating-point result to be trusted.
///
/// The determinant det = (a-c) x (b-c) is evaluated in double precision and its sign accepted only
/// if |det| exceeds Shewchuk's forward error bound  (3 + 16 eps) eps (|detleft| + |detright|),
/// eps = 2^-53 (https://www.cs.cmu.edu/~quake/robust.html, "orient2d" stage A). The bound is
/// RELATIVE to the magnitudes of the coordinate differences, so it scales with the data: it
/// does not depend on a fixed absolute epsilon, and it is invariant under uniform scaling by a power
/// of two. When the sign is not reliable the result is 0 ("uncertain"), and callers treat that as
/// collinear. This is a filter, not exact arithmetic: sign decisions are never wrong, but some
/// truly non-collinear configurations within ~1e-16 relative are classified as collinear.
int orientation(Vec2 a, Vec2 b, Vec2 c);

enum class SegmentRelation {
  Disjoint,         ///< no common point (within the predicate's reliability)
  ProperCrossing,   ///< interiors cross at a single point, all four orientations reliable
  Touching,         ///< an endpoint of one segment lies on the other segment (T-junction/endpoint contact)
  CollinearOverlap  ///< collinear segments sharing a sub-segment of positive length
};

struct SegmentContact {
  SegmentRelation relation = SegmentRelation::Disjoint;
  Vec2 point{};  ///< crossing point / contact point / midpoint of the overlap (undefined if Disjoint)
};

/// Classifies how segments [p1,p2] and [q1,q2] meet. Both segments must have non-zero length.
/// Rounding: the returned *point* of a proper crossing is computed in double precision (error of
/// order eps * coordinate magnitude); the *classification* uses orientation() only.
SegmentContact intersectSegments(Vec2 p1, Vec2 p2, Vec2 q1, Vec2 q2);

/// Euclidean distance from p to segment [a,b] (a != b).
double pointSegmentDistance(Vec2 p, Vec2 a, Vec2 b);

/// A pair of segments (indices into the respective polylines) that meet.
struct SegmentPairContact {
  std::size_t first;
  std::size_t second;
  SegmentRelation relation;
  Vec2 point;
};

/// Self-intersections of one polyline. Segment i joins point i and i+1 (closed: the last segment
/// joins the last point to point 0). Legitimate neighbours (segments sharing an endpoint, including
/// the closed-loop seam pair last/first) are NOT reported when they meet only at that shared
/// endpoint. Adjacent segments that fold straight back on themselves (collinear and anti-parallel:
/// a zero-width spike) ARE reported as CollinearOverlap. Everything else - crossings, touching,
/// collinear overlap between non-adjacent segments - is reported.
///
/// Complexity: O(S^2) segment pairs for S segments, each pair first rejected by a bounding-box
/// test (4 comparisons), then classified by up to four orientation evaluations. Memory O(S).
/// Stops after `maxResults` contacts (0 = unlimited).
std::vector<SegmentPairContact> findSelfIntersections(const std::vector<Vec2>& points, Closure closure,
                                                      std::size_t maxResults = 0);

/// Contacts between two different polylines (no adjacency exemptions). O(Sa * Sb), same cost model.
std::vector<SegmentPairContact> findContacts(const std::vector<Vec2>& a, Closure closureA,
                                             const std::vector<Vec2>& b, Closure closureB,
                                             std::size_t maxResults = 0);

}  // namespace pathforge::geometry
