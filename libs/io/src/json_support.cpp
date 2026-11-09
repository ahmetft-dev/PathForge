#include "json_support.hpp"

#include <cmath>
#include <fstream>
#include <system_error>

namespace pathforge::io::detail {

using geometry::Closure;

bool nestingTooDeep(std::string_view text) {
  std::size_t depth = 0;
  bool inString = false, escaped = false;
  for (char c : text) {
    if (inString) {
      if (escaped) {
        escaped = false;
      } else if (c == '\\') {
        escaped = true;
      } else if (c == '"') {
        inString = false;
      }
      continue;
    }
    if (c == '"') {
      inString = true;
    } else if (c == '[' || c == '{') {
      if (++depth > kMaxJsonDepth) return true;
    } else if ((c == ']' || c == '}') && depth > 0) {
      --depth;
    }
  }
  return false;
}

ordered_json parseGuarded(std::string_view text) {
  if (text.size() > kMaxTrackFileBytes) throw ParseError{"document too large"};
  if (nestingTooDeep(text)) {
    throw ParseError{"JSON nesting deeper than " + std::to_string(kMaxJsonDepth) +
                     " levels is not supported"};
  }
  return ordered_json::parse(text.begin(), text.end());
}

double requireFiniteNumber(const ordered_json& v, const std::string& what) {
  if (!v.is_number()) throw ParseError{what + " must be a number"};
  const double d = v.get<double>();
  if (!std::isfinite(d)) throw ParseError{what + " must be finite (number out of range)"};
  return d;
}

void warnUnknown(const ordered_json& obj, const std::set<std::string>& known, const std::string& where,
                 std::vector<std::string>& warnings) {
  for (auto it = obj.begin(); it != obj.end(); ++it) {
    if (!known.count(it.key()))
      warnings.push_back("unknown field '" + it.key() + "' in " + where + " ignored");
  }
}

const ordered_json& require(const ordered_json& obj, const char* key, std::string_view where) {
  const auto it = obj.find(key);
  if (it == obj.end()) {
    throw ParseError{"missing required field '" + std::string(key) + "' in " + std::string(where)};
  }
  return *it;
}

long long readSchemaVersion(const ordered_json& doc) {
  if (!doc.is_object()) throw ParseError{"top-level JSON value must be an object"};
  const ordered_json& ver = require(doc, "schemaVersion", "document");
  if (!ver.is_number_integer()) throw ParseError{"schemaVersion must be an integer"};
  return ver.get<long long>();
}

Header readHeader(const ordered_json& doc) {
  Header h;
  const ordered_json& name = require(doc, "name", "document");
  if (!name.is_string()) throw ParseError{"name must be a string"};
  h.name = name.get<std::string>();
  const ordered_json& units = require(doc, "units", "document");
  if (!units.is_string() || units.get<std::string>() != kUnits) {
    throw ParseError{std::string("units must be \"") + kUnits + "\" (meters, radians)"};
  }
  const ordered_json& cs = require(doc, "coordinateSystem", "document");
  if (!cs.is_string() || cs.get<std::string>() != kCoordinateSystem) {
    throw ParseError{std::string("coordinateSystem must be \"") + kCoordinateSystem +
                     "\" (x right, y up, counter-clockwise positive)"};
  }
  const ordered_json& closed = require(doc, "closed", "document");
  if (!closed.is_boolean()) throw ParseError{"closed must be true or false"};
  h.closure = closed.get<bool>() ? Closure::Closed : Closure::Open;
  return h;
}

void writeHeader(ordered_json& doc, int schemaVersion, const std::string& name, Closure closure) {
  doc["schemaVersion"] = schemaVersion;
  doc["name"] = name;
  doc["units"] = kUnits;
  doc["coordinateSystem"] = kCoordinateSystem;
  doc["closed"] = closure == Closure::Closed;
}

namespace {
std::vector<double> readWidths(const ordered_json& w, const char* side, std::size_t n) {
  const std::string what = std::string("widths.") + side;
  if (w.is_number()) return std::vector<double>(n, requireFiniteNumber(w, what));
  if (!w.is_array()) throw ParseError{what + " must be a number (constant) or an array (per point)"};
  std::vector<double> out;
  out.reserve(w.size());
  for (std::size_t i = 0; i < w.size(); ++i)
    out.push_back(requireFiniteNumber(w[i], what + "[" + std::to_string(i) + "]"));
  return out;
}
}  // namespace

geometry::TrackData readTrackV1(const ordered_json& doc, std::vector<std::string>& warnings) {
  geometry::TrackData d;
  const Header h = readHeader(doc);
  d.name = h.name;
  d.closure = h.closure;

  const ordered_json& cl = require(doc, "centerline", "document");
  if (!cl.is_array()) throw ParseError{"centerline must be an array of {\"x\", \"y\"} points"};
  d.centerline.reserve(cl.size());
  for (std::size_t i = 0; i < cl.size(); ++i) {
    const std::string where = "centerline[" + std::to_string(i) + "]";
    if (!cl[i].is_object()) throw ParseError{where + " must be an object with x and y"};
    warnUnknown(cl[i], {"x", "y"}, where, warnings);
    d.centerline.push_back({requireFiniteNumber(require(cl[i], "x", where), where + ".x"),
                            requireFiniteNumber(require(cl[i], "y", where), where + ".y")});
  }

  const ordered_json& w = require(doc, "widths", "document");
  if (!w.is_object()) throw ParseError{"widths must be an object with left and right"};
  warnUnknown(w, {"left", "right"}, "widths", warnings);
  d.leftWidths = readWidths(require(w, "left", "widths"), "left", d.centerline.size());
  d.rightWidths = readWidths(require(w, "right", "widths"), "right", d.centerline.size());

  warnUnknown(doc, {"schemaVersion", "name", "units", "coordinateSystem", "closed", "centerline", "widths"},
              "document", warnings);
  return d;
}

std::optional<std::string> readTextFile(const std::filesystem::path& path, std::string& error) {
  std::error_code ec;
  if (!std::filesystem::exists(path, ec)) {
    error = "file not found: " + path.string();
    return std::nullopt;
  }
  if (!std::filesystem::is_regular_file(path, ec)) {
    error = "not a regular file: " + path.string();
    return std::nullopt;
  }
  const auto size = std::filesystem::file_size(path, ec);
  if (ec) {
    error = "cannot determine size of " + path.string() + ": " + ec.message();
    return std::nullopt;
  }
  if (size > kMaxTrackFileBytes) {
    error = "file too large (" + std::to_string(size) + " bytes; limit " +
            std::to_string(kMaxTrackFileBytes) + ")";
    return std::nullopt;
  }
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    error = "cannot open " + path.string() + " for reading";
    return std::nullopt;
  }
  std::string text(static_cast<std::size_t>(size), '\0');
  in.read(text.data(), static_cast<std::streamsize>(size));
  if (!in && !in.eof()) {
    error = "read error on " + path.string();
    return std::nullopt;
  }
  text.resize(static_cast<std::size_t>(in.gcount()));
  return text;
}

std::string writeTextFileAtomically(const std::filesystem::path& path, const std::string& text,
                                    OverwritePolicy policy) {
  std::error_code ec;
  if (policy == OverwritePolicy::Fail && std::filesystem::exists(path, ec)) {
    return "refusing to overwrite existing file: " + path.string();
  }
  std::filesystem::path tmp = path;
  tmp += ".pathforge-tmp";
  {
    std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
    if (!out)
      return "cannot create temporary file " + tmp.string() +
             " (does the directory exist and is it writable?)";
    out.write(text.data(), static_cast<std::streamsize>(text.size()));
    out.flush();
    if (!out) {
      out.close();
      std::filesystem::remove(tmp, ec);
      return "write failed for " + tmp.string();
    }
  }
  std::filesystem::rename(tmp, path, ec);  // replaces an existing target atomically on POSIX
  if (ec) {
    std::error_code ignore;
    std::filesystem::remove(tmp, ignore);
    return "cannot move " + tmp.string() + " to " + path.string() + ": " + ec.message();
  }
  return {};
}

}  // namespace pathforge::io::detail
