#pragma once

#include <pathforge/geometry/path.hpp>
#include <pathforge/geometry/track.hpp>
#include <string>
#include <vector>

namespace pathforge::geometry {

/// Plain, UNVALIDATED description of a track: what is stored in a file. Unlike Track it can hold
/// data that violates geometric invariants (duplicate points, mismatched width arrays, ...), so
/// that bad data can be loaded, inspected and reported by validateTrackData() instead of being
/// rejected outright. Units: meters; frame: x right, y up, CCW positive.
struct TrackData {
  std::string name;
  Closure closure = Closure::Closed;
  std::vector<Vec2> centerline;
  std::vector<double> leftWidths;   ///< half-widths [m], one per centerline point
  std::vector<double> rightWidths;  ///< half-widths [m], one per centerline point
};

/// Builds the validated Track (boundaries are recomputed, never stored).
/// @throws GeometryError if the data cannot form a Track (see validateTrackData() for a report).
Track buildTrack(const TrackData& data);

/// Copies a Track's defining data (centerline + widths). Boundaries are derived and not copied.
TrackData toTrackData(const Track& track, std::string name);

}  // namespace pathforge::geometry
