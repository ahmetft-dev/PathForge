#include <pathforge/geometry/track_data.hpp>

namespace pathforge::geometry {

Track buildTrack(const TrackData& data) {
  return Track::create(Path::create(data.centerline, data.closure), data.leftWidths, data.rightWidths);
}

TrackData toTrackData(const Track& track, std::string name) {
  TrackData d;
  d.name = std::move(name);
  d.closure = track.centerline().closure();
  d.centerline = track.centerline().points();
  d.leftWidths = track.leftWidths();
  d.rightWidths = track.rightWidths();
  return d;
}

}  // namespace pathforge::geometry
