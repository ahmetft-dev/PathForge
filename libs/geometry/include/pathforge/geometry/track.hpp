#pragma once

#include <cstddef>
#include <pathforge/geometry/frames.hpp>
#include <pathforge/geometry/path.hpp>
#include <vector>

namespace pathforge::geometry {

/// Local, cheap diagnostics of the offset boundaries. These detect OBVIOUS local degeneracies only.
/// A track for which anyLocalIssue() is false is NOT thereby valid: boundaries can still
/// self-intersect or cross each other globally (e.g. two distant parts of a track close together),
/// which is not checked here (planned for a later validation phase).
struct TrackDiagnostics {
  /// Boundary segments whose direction is reversed relative to the centerline segment
  /// (dot product <= 0): the offset curve has folded back on itself locally.
  std::size_t leftFlippedSegments = 0;
  std::size_t rightFlippedSegments = 0;
  /// Vertices where width * curvature >= 1 on the inner side of the turn, i.e. the offset
  /// distance reaches or exceeds the local radius of curvature (offset curve degenerates).
  std::size_t leftCurvatureViolations = 0;
  std::size_t rightCurvatureViolations = 0;
  /// max over vertices of (leftWidth * kappa) and (rightWidth * -kappa); values >= 1 are violations.
  /// Useful as a margin: 0.5 means the offset is half of the tightest local radius.
  double maxLeftWidthCurvature = 0.0;
  double maxRightWidthCurvature = 0.0;

  bool anyLocalIssue() const {
    return leftFlippedSegments || rightFlippedSegments || leftCurvatureViolations || rightCurvatureViolations;
  }
};

/// Centerline with left/right half-widths and the derived boundary polylines.
///
///   leftBoundary[i]  = centerline[i] + leftWidth  * N[i]
///   rightBoundary[i] = centerline[i] - rightWidth * N[i]
/// with N the left unit normal (-Ty, Tx) of the centerline frame (see PathFrames). "Left" and
/// "right" are relative to the direction of travel (increasing index). Boundaries have one vertex
/// per centerline vertex and the same closure; they are plain point lists, not validated Paths,
/// because a folded offset may legitimately contain coincident points.
///
/// Widths are half-widths measured from the centerline [m]; total width = left + right at a
/// vertex. They are stored PER VERTEX (index-aligned with the centerline), so constant and variable
/// widths share one representation. Width at vertex i applies at the vertex position; between
/// vertices the boundary is the straight segment joining the offset vertices (no interpolation of
/// the width inside a segment is defined beyond that). For arc-length-based profiles see
/// sampleWidthProfile().
///
/// Offset-curve limitations: the offset is computed per vertex along the vertex normal, not as a
/// true parallel curve. Where width >= local radius of curvature on the inside of a turn, the
/// boundary folds; see TrackDiagnostics. Boundary spacing differs from centerline spacing (larger
/// on the outside, smaller on the inside of a turn).
class Track {
 public:
  /// Constant widths.
  /// @throws GeometryError if a width is negative or non-finite. Zero is accepted here so that
  ///   degenerate tracks can still be built and inspected; validateTrack() reports it as an error.
  static Track create(Path centerline, double leftWidth, double rightWidth);

  /// Per-vertex widths; both vectors must have exactly centerline.size() entries.
  /// @throws GeometryError on size mismatch, or a negative / non-finite entry.
  static Track create(Path centerline, std::vector<double> leftWidths, std::vector<double> rightWidths);

  const Path& centerline() const { return centerline_; }
  const PathFrames& frames() const { return frames_; }

  /// True if every left width is bitwise equal and every right width is bitwise equal.
  bool hasConstantWidths() const { return constant_; }
  /// Constant tracks: the width. Variable tracks: the arithmetic mean over vertices (a summary, not
  /// a bound; use leftWidths()/rightWidths() or the min/max accessors for real extents).
  double leftWidth() const { return leftWidth_; }
  double rightWidth() const { return rightWidth_; }
  double totalWidth() const { return leftWidth_ + rightWidth_; }
  double minTotalWidth() const { return minTotal_; }
  double maxTotalWidth() const { return maxTotal_; }
  const std::vector<double>& leftWidths() const { return leftWidths_; }
  const std::vector<double>& rightWidths() const { return rightWidths_; }
  const std::vector<Vec2>& leftBoundary() const { return left_; }
  const std::vector<Vec2>& rightBoundary() const { return right_; }
  const TrackDiagnostics& diagnostics() const { return diagnostics_; }

 private:
  Track(Path centerline, std::vector<double> leftWidths, std::vector<double> rightWidths);

  Path centerline_;
  PathFrames frames_;
  bool constant_ = true;
  double leftWidth_ = 0.0;
  double rightWidth_ = 0.0;
  double minTotal_ = 0.0;
  double maxTotal_ = 0.0;
  std::vector<double> leftWidths_;
  std::vector<double> rightWidths_;
  std::vector<Vec2> left_;
  std::vector<Vec2> right_;
  TrackDiagnostics diagnostics_;
};

}  // namespace pathforge::geometry
