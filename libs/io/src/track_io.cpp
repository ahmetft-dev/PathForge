#include <cmath>
#include <fstream>
#include <nlohmann/json.hpp>
#include <pathforge/io/track_io.hpp>
#include <set>
#include <sstream>
#include <system_error>

namespace pathforge::io {
namespace {

using ordered_json = nlohmann::ordered_json;
using geometry::Closure;
using geometry::Vec2;

constexpr const char* kUnits = "SI";  // meters, radians
constexpr const char* kCoordinateSystem = "xy-right-handed";

struct ParseError {
  std::string message;
};

// Rejects documents nested deeper than kMaxJsonDepth before they are handed to the DOM parser.
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

const ordered_json& require(const ordered_json& obj, const char* key, const std::string& where) {
  const auto it = obj.find(key);
  if (it == obj.end()) throw ParseError{"missing required field '" + std::string(key) + "' in " + where};
  return *it;
}

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

geometry::TrackData readDocument(const ordered_json& doc, std::vector<std::string>& warnings) {
  if (!doc.is_object()) throw ParseError{"top-level JSON value must be an object"};

  const ordered_json& ver = require(doc, "schemaVersion", "document");
  if (!ver.is_number_integer()) throw ParseError{"schemaVersion must be an integer"};
  const auto v = ver.get<long long>();
  if (v != kTrackSchemaVersion) {
    throw ParseError{"unsupported schemaVersion " + std::to_string(v) + " (this build reads version " +
                     std::to_string(kTrackSchemaVersion) + "); upgrade PathForge or re-export the track"};
  }

  geometry::TrackData d;
  const ordered_json& name = require(doc, "name", "document");
  if (!name.is_string()) throw ParseError{"name must be a string"};
  d.name = name.get<std::string>();

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
  d.closure = closed.get<bool>() ? Closure::Closed : Closure::Open;

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

}  // namespace

LoadResult parseTrackJson(std::string_view text) {
  LoadResult r;
  try {
    if (text.size() > kMaxTrackFileBytes) {
      r.error = "document too large";
      return r;
    }
    if (nestingTooDeep(text)) {
      r.error = "JSON nesting deeper than " + std::to_string(kMaxJsonDepth) + " levels is not supported";
      return r;
    }
    const ordered_json doc = ordered_json::parse(text.begin(), text.end());
    r.data = readDocument(doc, r.warnings);
  } catch (const ParseError& e) {
    r.data.reset();
    r.error = e.message;
  } catch (const nlohmann::json::exception& e) {
    r.data.reset();
    r.error = std::string("malformed JSON: ") + e.what();
  } catch (const std::exception& e) {
    r.data.reset();
    r.error = std::string("could not read track: ") + e.what();
  }
  return r;
}

LoadResult loadTrackFile(const std::filesystem::path& path) {
  LoadResult r;
  std::error_code ec;
  if (!std::filesystem::exists(path, ec)) {
    r.error = "file not found: " + path.string();
    return r;
  }
  if (!std::filesystem::is_regular_file(path, ec)) {
    r.error = "not a regular file: " + path.string();
    return r;
  }
  const auto size = std::filesystem::file_size(path, ec);
  if (ec) {
    r.error = "cannot determine size of " + path.string() + ": " + ec.message();
    return r;
  }
  if (size > kMaxTrackFileBytes) {
    r.error = "file too large (" + std::to_string(size) + " bytes; limit " +
              std::to_string(kMaxTrackFileBytes) + ")";
    return r;
  }
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    r.error = "cannot open " + path.string() + " for reading";
    return r;
  }
  std::string text(static_cast<std::size_t>(size), '\0');
  in.read(text.data(), static_cast<std::streamsize>(size));
  if (!in && !in.eof()) {
    r.error = "read error on " + path.string();
    return r;
  }
  text.resize(static_cast<std::size_t>(in.gcount()));
  return parseTrackJson(text);
}

SerializeResult serializeTrackJson(const geometry::TrackData& d) {
  SerializeResult r;
  const std::size_t n = d.centerline.size();
  if (d.leftWidths.size() != n || d.rightWidths.size() != n) {
    r.error = "width arrays must have one entry per centerline point to be saved";
    return r;
  }
  for (const Vec2& p : d.centerline) {
    if (!geometry::isFinite(p)) {
      r.error = "centerline contains a non-finite coordinate; it cannot be represented in JSON";
      return r;
    }
  }
  for (const auto* w : {&d.leftWidths, &d.rightWidths}) {
    for (double x : *w) {
      if (!std::isfinite(x)) {
        r.error = "widths contain a non-finite value; it cannot be represented in JSON";
        return r;
      }
    }
  }

  auto widthJson = [](const std::vector<double>& w) {
    // Compact constant form when every entry is bitwise equal (and there is at least one).
    bool constant = !w.empty();
    for (double x : w) constant = constant && x == w.front();
    if (constant) return ordered_json(w.front());
    ordered_json a = ordered_json::array();
    for (double x : w) a.push_back(x);
    return a;
  };

  ordered_json doc = ordered_json::object();
  doc["schemaVersion"] = kTrackSchemaVersion;
  doc["name"] = d.name;
  doc["units"] = kUnits;
  doc["coordinateSystem"] = kCoordinateSystem;
  doc["closed"] = d.closure == Closure::Closed;
  ordered_json pts = ordered_json::array();
  for (const Vec2& p : d.centerline) pts.push_back(ordered_json{{"x", p.x}, {"y", p.y}});
  doc["centerline"] = std::move(pts);
  doc["widths"] = ordered_json{{"left", widthJson(d.leftWidths)}, {"right", widthJson(d.rightWidths)}};
  try {
    r.text = doc.dump(2) + "\n";
  } catch (const std::exception& e) {
    r.error = std::string("serialisation failed: ") + e.what();  // e.g. invalid UTF-8 in the name
  }
  return r;
}

SaveResult saveTrackFile(const std::filesystem::path& path, const geometry::TrackData& data,
                         OverwritePolicy policy) {
  SaveResult r;
  const SerializeResult s = serializeTrackJson(data);
  if (!s.ok()) {
    r.error = s.error;
    return r;
  }
  std::error_code ec;
  if (policy == OverwritePolicy::Fail && std::filesystem::exists(path, ec)) {
    r.error = "refusing to overwrite existing file: " + path.string();
    return r;
  }
  std::filesystem::path tmp = path;
  tmp += ".pathforge-tmp";
  {
    std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
    if (!out) {
      r.error =
          "cannot create temporary file " + tmp.string() + " (does the directory exist and is it writable?)";
      return r;
    }
    out.write(s.text.data(), static_cast<std::streamsize>(s.text.size()));
    out.flush();
    if (!out) {
      out.close();
      std::filesystem::remove(tmp, ec);
      r.error = "write failed for " + tmp.string();
      return r;
    }
  }
  std::filesystem::rename(tmp, path, ec);  // replaces an existing target atomically on POSIX
  if (ec) {
    std::error_code ignore;
    std::filesystem::remove(tmp, ignore);
    r.error = "cannot move " + tmp.string() + " to " + path.string() + ": " + ec.message();
  }
  return r;
}

}  // namespace pathforge::io
