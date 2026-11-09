#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <pathforge/geometry/path.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace pathforge::editor {

using geometry::Closure;
using geometry::Vec2;

/// Stable identity of a control point inside one document. Ids are never reused by a document (deleting and
/// re-inserting through restorePoint() keeps the original id), so UI selection and undo history can refer to
/// a point independently of its current index. 0 is never a valid id.
using PointId = std::uint64_t;

/// One user-authored control point. Widths are half-widths [m] to the left/right of the centerline, defined
/// AT this point and interpolated linearly in arc length between control points (see DerivedGeometry).
struct ControlPoint {
  PointId id = 0;
  Vec2 position;
  double leftWidth = 5.0;
  double rightWidth = 5.0;
  bool operator==(const ControlPoint&) const = default;
};

enum class Interpolation {
  CatmullRom,  ///< centripetal-family Catmull-Rom spline through the control points (C1)
  Polyline,    ///< straight segments between control points (exact, e.g. for imported sampled tracks)
};

/// How derived geometry is generated from the control points.
struct GeometrySettings {
  Interpolation interpolation = Interpolation::CatmullRom;
  double alpha = 0.5;  ///< Catmull-Rom knot exponent in [0, 1] (0.5 = centripetal)
  double resampleSpacing =
      2.0;  ///< [m] equal arc-length resampling; 0 = keep the interpolated vertices as-is
  bool operator==(const GeometrySettings&) const = default;
};

/// Thrown for rejected edits (non-finite or out-of-range input, unknown ids, bad indices). The document is
/// left unchanged when this is thrown.
class EditError : public std::invalid_argument {
 public:
  using std::invalid_argument::invalid_argument;
};

/// Largest accepted |coordinate| [m]. Keeps far-away clicks from producing values where double precision can
/// no longer represent sub-millimetre geometry (ulp(1e7) ~ 2e-9 m) and bounds generated sample counts.
inline constexpr double kMaxCoordinate = 1e7;
/// Largest accepted half-width [m].
inline constexpr double kMaxWidth = 1e4;

/// The authoritative, user-editable description of a track. Derived geometry (centerline samples, boundaries,
/// validation) is NEVER stored here; it is regenerated from this document by generateGeometry().
///
/// Invariants maintained by every mutator: all positions finite with |x|, |y| <= kMaxCoordinate; all widths
/// finite and in [0, kMaxWidth]; ids unique and non-zero; settings valid. Geometric validity (enough points,
/// no self-intersection, ...) is deliberately NOT an invariant: incomplete or invalid tracks must remain
/// editable and savable so they can be repaired. Zero widths are accepted for the same reason and reported by
/// validation.
///
/// Plain value type, no Qt; copying copies the whole document. Not thread-safe; copy it to hand a snapshot to
/// another thread.
class TrackDocument {
 public:
  const std::string& name() const { return name_; }
  void setName(std::string name);

  Closure closure() const { return closure_; }
  void setClosure(Closure closure);

  const GeometrySettings& settings() const { return settings_; }
  /// @throws EditError for alpha outside [0, 1] or a negative / non-finite / too small (< 1 mm) spacing.
  void setSettings(const GeometrySettings& settings);

  const std::vector<ControlPoint>& points() const { return points_; }
  std::size_t size() const { return points_.size(); }
  bool empty() const { return points_.empty(); }
  std::optional<std::size_t> indexOf(PointId id) const;
  const ControlPoint* find(PointId id) const;

  /// Appends a point at the end of the point order; returns its new id.
  PointId appendPoint(Vec2 position, double leftWidth, double rightWidth);
  /// Inserts before `index` (index == size() appends); returns its new id. @throws EditError if index >
  /// size().
  PointId insertPoint(std::size_t index, Vec2 position, double leftWidth, double rightWidth);
  /// Re-inserts a previously removed point with its ORIGINAL id (used by undo). @throws EditError if the id
  /// is 0, already present, or not lower than the next id this document would hand out.
  void restorePoint(std::size_t index, const ControlPoint& point);
  void movePoint(PointId id, Vec2 position);
  void setPointWidths(PointId id, double leftWidth, double rightWidth);
  /// Removes and returns the point. @throws EditError for an unknown id.
  ControlPoint removePoint(PointId id);

  /// Incremented by every call that changes the document (no-op edits leave it unchanged). Used to associate
  /// derived results (e.g. background validation) with the document state they were computed from.
  std::uint64_t revision() const { return revision_; }

  static void checkPosition(Vec2 position);
  static void checkWidth(double width);
  static void checkSettings(const GeometrySettings& settings);

 private:
  std::size_t indexOrThrow(PointId id) const;
  void touch() { ++revision_; }

  std::string name_ = "Untitled";
  Closure closure_ = Closure::Open;
  GeometrySettings settings_;
  std::vector<ControlPoint> points_;
  PointId nextId_ = 1;
  std::uint64_t revision_ = 0;
};

}  // namespace pathforge::editor
