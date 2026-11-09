#pragma once

#include <filesystem>
#include <optional>
#include <pathforge/editor/track_document.hpp>
#include <pathforge/geometry/track_data.hpp>
#include <pathforge/io/track_io.hpp>
#include <string>
#include <string_view>
#include <vector>

namespace pathforge::io {

/// Schema version of editable track documents. See docs/track-format.md.
inline constexpr int kDocumentSchemaVersion = 2;

/// Result of reading a track file of ANY supported schema version.
///
/// * Version 2 (editable document): `document` holds the control points and settings exactly as saved.
/// * Version 1 (sampled track, legacy): `legacyData` holds the samples exactly as stored, and `document`
/// holds the
///   same samples as a POLYLINE document with resampling switched off, so its derived geometry reproduces the
///   original samples bit-for-bit. No spline is fitted: re-interpreting samples as Catmull-Rom control points
///   would change the geometry. If the samples cannot be represented as a document (e.g. negative widths,
///   coordinates beyond +/-1e7 m) `document` is empty and a warning explains why; `legacyData` is still
///   usable.
///
/// Loading succeeds for geometrically invalid but representable tracks (self-intersections, too few points,
/// zero widths ...) so they can be inspected and repaired; geometric validity is a separate question.
struct DocumentLoadResult {
  int schemaVersion = 0;
  std::optional<editor::TrackDocument> document;
  std::optional<geometry::TrackData> legacyData;
  std::vector<std::string> warnings;
  std::string error;  ///< non-empty iff !ok()
  bool ok() const { return document.has_value() || legacyData.has_value(); }
};

/// Never throws.
DocumentLoadResult parseTrackDocumentJson(std::string_view text);
/// Never throws; read-only.
DocumentLoadResult loadTrackDocumentFile(const std::filesystem::path& path);

/// Schema v2: deterministic key order, shortest round-trip doubles. Control point ids are not stored (they
/// are session identities); points are written in document order and get fresh ids on load.
SerializeResult serializeTrackDocumentJson(const editor::TrackDocument& document);

/// Atomic save (temporary file + rename), see saveTrackFile(). Never throws.
SaveResult saveTrackDocumentFile(const std::filesystem::path& path, const editor::TrackDocument& document,
                                 OverwritePolicy policy = OverwritePolicy::Fail);

/// Converts sampled track data to a polyline document without resampling (exact). @throws editor::EditError
/// if a value cannot be represented in a document.
editor::TrackDocument documentFromTrackData(const geometry::TrackData& data);

}  // namespace pathforge::io
