#pragma once

#include <filesystem>
#include <optional>
#include <pathforge/geometry/track_data.hpp>
#include <string>
#include <string_view>
#include <vector>

namespace pathforge::io {

/// Current track file schema version. See docs/track-format.md.
inline constexpr int kTrackSchemaVersion = 1;

/// Largest accepted file / document size [bytes] and JSON nesting depth. They bound memory and
/// stack use on malformed or hostile input.
inline constexpr std::size_t kMaxTrackFileBytes = 128u * 1024u * 1024u;
inline constexpr std::size_t kMaxJsonDepth = 32;

/// Result of reading a track. Parsing succeeds if the document is well-formed and conforms to the
/// schema; it says NOTHING about geometric validity (call geometry::validateTrackData() on `data`).
struct LoadResult {
  std::optional<geometry::TrackData> data;
  std::vector<std::string> warnings;  ///< non-fatal findings, e.g. unknown fields that were ignored
  std::string error;                  ///< actionable message; non-empty iff !ok()
  bool ok() const { return data.has_value(); }
};

/// Parses a schema-v1 track document. Never throws; malformed input yields `error`.
LoadResult parseTrackJson(std::string_view text);

/// Reads and parses a file. Never throws. A failure leaves nothing behind (read-only operation).
LoadResult loadTrackFile(const std::filesystem::path& path);

struct SerializeResult {
  std::string text;
  std::string error;  ///< non-empty on failure
  bool ok() const { return error.empty(); }
};

/// Serialises to schema v1 (deterministic key order, 2-space indent, shortest round-trip doubles).
/// Fails (instead of writing lossy data) for non-finite numbers or width arrays whose size differs
/// from the centerline. Geometrically invalid tracks (e.g. self-intersecting) ARE serialised.
/// Boundaries are not stored; they are recomputed on load.
SerializeResult serializeTrackJson(const geometry::TrackData& data);

enum class OverwritePolicy { Fail, Overwrite };

struct SaveResult {
  std::string error;  ///< non-empty on failure
  bool ok() const { return error.empty(); }
};

/// Writes `data` to `path` via a temporary file in the same directory, then renames it over the
/// target, so a failed or interrupted write never leaves a truncated target. With
/// OverwritePolicy::Fail an existing path is left untouched and an error is returned (the existence
/// check and the rename are not atomic together: another process creating the file in between is
/// not detected). The parent directory must already exist. Never throws.
SaveResult saveTrackFile(const std::filesystem::path& path, const geometry::TrackData& data,
                         OverwritePolicy policy = OverwritePolicy::Fail);

}  // namespace pathforge::io
