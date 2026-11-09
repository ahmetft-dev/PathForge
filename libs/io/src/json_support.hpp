#pragma once
// Internal helpers shared by the v1 (sampled track) and v2 (editable document) readers/writers. Not
// installed.

#include <filesystem>
#include <nlohmann/json.hpp>
#include <optional>
#include <pathforge/geometry/track_data.hpp>
#include <pathforge/io/track_io.hpp>
#include <set>
#include <string>
#include <string_view>
#include <vector>

namespace pathforge::io::detail {

using ordered_json = nlohmann::ordered_json;

inline constexpr const char* kUnits = "SI";  // meters, radians
inline constexpr const char* kCoordinateSystem = "xy-right-handed";

struct ParseError {
  std::string message;
};

/// Rejects documents nested deeper than kMaxJsonDepth before they are handed to the DOM parser.
bool nestingTooDeep(std::string_view text);
/// Size and nesting guards, then DOM parse. @throws ParseError or nlohmann::json::exception.
ordered_json parseGuarded(std::string_view text);

double requireFiniteNumber(const ordered_json& v, const std::string& what);
void warnUnknown(const ordered_json& obj, const std::set<std::string>& known, const std::string& where,
                 std::vector<std::string>& warnings);
/// Returns a reference into `obj`. `where` is a string_view so that no temporary std::string is bound to a
/// reference parameter of a function that returns a reference (GCC -Wdangling-reference).
const ordered_json& require(const ordered_json& obj, const char* key, std::string_view where);

/// Reads "schemaVersion" (must be an integer). @throws ParseError.
long long readSchemaVersion(const ordered_json& doc);

/// Fields common to all schema versions.
struct Header {
  std::string name;
  geometry::Closure closure = geometry::Closure::Closed;
};
Header readHeader(const ordered_json& doc);
void writeHeader(ordered_json& doc, int schemaVersion, const std::string& name, geometry::Closure closure);

/// Body of a schema-v1 document (version already checked by the caller). @throws ParseError.
geometry::TrackData readTrackV1(const ordered_json& doc, std::vector<std::string>& warnings);

/// Reads a whole file with existence, type and size checks; returns nullopt and sets `error` on failure.
std::optional<std::string> readTextFile(const std::filesystem::path& path, std::string& error);
/// Writes via a temporary file in the same directory and renames it over the target; returns an error message
/// or an empty string. Honors OverwritePolicy::Fail.
std::string writeTextFileAtomically(const std::filesystem::path& path, const std::string& text,
                                    OverwritePolicy policy);

}  // namespace pathforge::io::detail
