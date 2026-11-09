#include <pathforge/io/document_io.hpp>

#include "json_support.hpp"

namespace pathforge::io {

using detail::ordered_json;
using detail::ParseError;
using editor::EditError;
using editor::GeometrySettings;
using editor::Interpolation;
using editor::TrackDocument;

namespace {

constexpr const char* kCatmullRom = "catmull-rom";
constexpr const char* kPolyline = "polyline";

TrackDocument readDocumentV2(const ordered_json& doc, std::vector<std::string>& warnings) {
  TrackDocument d;
  const detail::Header h = detail::readHeader(doc);

  const ordered_json& g = detail::require(doc, "geometry", "document");
  if (!g.is_object()) throw ParseError{"geometry must be an object"};
  detail::warnUnknown(g, {"interpolation", "alpha", "resampleSpacing"}, "geometry", warnings);
  GeometrySettings s;
  const ordered_json& interp = detail::require(g, "interpolation", "geometry");
  if (!interp.is_string()) throw ParseError{"geometry.interpolation must be a string"};
  const std::string kind = interp.get<std::string>();
  if (kind == kCatmullRom) {
    s.interpolation = Interpolation::CatmullRom;
  } else if (kind == kPolyline) {
    s.interpolation = Interpolation::Polyline;
  } else {
    throw ParseError{"geometry.interpolation must be \"catmull-rom\" or \"polyline\" (got \"" + kind + "\")"};
  }
  s.alpha = detail::requireFiniteNumber(detail::require(g, "alpha", "geometry"), "geometry.alpha");
  s.resampleSpacing = detail::requireFiniteNumber(detail::require(g, "resampleSpacing", "geometry"),
                                                  "geometry.resampleSpacing");

  try {
    d.setName(h.name);
    d.setClosure(h.closure);
    d.setSettings(s);
  } catch (const EditError& e) {
    throw ParseError{std::string("geometry: ") + e.what()};
  }

  const ordered_json& cps = detail::require(doc, "controlPoints", "document");
  if (!cps.is_array()) throw ParseError{"controlPoints must be an array"};
  for (std::size_t i = 0; i < cps.size(); ++i) {
    const std::string where = "controlPoints[" + std::to_string(i) + "]";
    const ordered_json& p = cps[i];
    if (!p.is_object()) throw ParseError{where + " must be an object"};
    detail::warnUnknown(p, {"x", "y", "leftWidth", "rightWidth"}, where, warnings);
    const double x = detail::requireFiniteNumber(detail::require(p, "x", where), where + ".x");
    const double y = detail::requireFiniteNumber(detail::require(p, "y", where), where + ".y");
    const double l =
        detail::requireFiniteNumber(detail::require(p, "leftWidth", where), where + ".leftWidth");
    const double r =
        detail::requireFiniteNumber(detail::require(p, "rightWidth", where), where + ".rightWidth");
    try {
      d.appendPoint({x, y}, l, r);
    } catch (const EditError& e) {
      throw ParseError{where + ": " + e.what()};
    }
  }
  detail::warnUnknown(
      doc, {"schemaVersion", "name", "units", "coordinateSystem", "closed", "geometry", "controlPoints"},
      "document", warnings);
  return d;
}

}  // namespace

TrackDocument documentFromTrackData(const geometry::TrackData& data) {
  TrackDocument d;
  d.setName(data.name);
  d.setClosure(data.closure);
  GeometrySettings s;
  s.interpolation = Interpolation::Polyline;
  s.resampleSpacing = 0.0;
  d.setSettings(s);
  const std::size_t n = data.centerline.size();
  if (data.leftWidths.size() != n || data.rightWidths.size() != n) {
    throw EditError("width arrays must have one entry per centerline point");
  }
  for (std::size_t i = 0; i < n; ++i)
    d.appendPoint(data.centerline[i], data.leftWidths[i], data.rightWidths[i]);
  return d;
}

DocumentLoadResult parseTrackDocumentJson(std::string_view text) {
  DocumentLoadResult r;
  try {
    const ordered_json doc = detail::parseGuarded(text);
    const long long v = detail::readSchemaVersion(doc);
    if (v == kDocumentSchemaVersion) {
      r.document = readDocumentV2(doc, r.warnings);
    } else if (v == kTrackSchemaVersion) {
      r.legacyData = detail::readTrackV1(doc, r.warnings);
      try {
        r.document = documentFromTrackData(*r.legacyData);
        r.warnings.push_back(
            "schema v1 (sampled) track imported as a polyline document without resampling; "
            "saving writes schema v2");
      } catch (const EditError& e) {
        r.warnings.push_back(std::string("schema v1 track cannot be edited: ") + e.what());
      }
    } else {
      throw ParseError{"unsupported schemaVersion " + std::to_string(v) +
                       " (this build reads versions 1 and 2); upgrade PathForge or re-export the track"};
    }
    r.schemaVersion = static_cast<int>(v);
  } catch (const ParseError& e) {
    r = DocumentLoadResult{};
    r.error = e.message;
  } catch (const nlohmann::json::exception& e) {
    r = DocumentLoadResult{};
    r.error = std::string("malformed JSON: ") + e.what();
  } catch (const std::exception& e) {
    r = DocumentLoadResult{};
    r.error = std::string("could not read track: ") + e.what();
  }
  return r;
}

DocumentLoadResult loadTrackDocumentFile(const std::filesystem::path& path) {
  DocumentLoadResult r;
  const auto text = detail::readTextFile(path, r.error);
  if (!text) return r;
  return parseTrackDocumentJson(*text);
}

SerializeResult serializeTrackDocumentJson(const TrackDocument& d) {
  SerializeResult r;
  ordered_json doc = ordered_json::object();
  detail::writeHeader(doc, kDocumentSchemaVersion, d.name(), d.closure());
  const GeometrySettings& s = d.settings();
  doc["geometry"] =
      ordered_json{{"interpolation", s.interpolation == Interpolation::CatmullRom ? kCatmullRom : kPolyline},
                   {"alpha", s.alpha},
                   {"resampleSpacing", s.resampleSpacing}};
  ordered_json pts = ordered_json::array();
  for (const editor::ControlPoint& p : d.points()) {
    pts.push_back(ordered_json{
        {"x", p.position.x}, {"y", p.position.y}, {"leftWidth", p.leftWidth}, {"rightWidth", p.rightWidth}});
  }
  doc["controlPoints"] = std::move(pts);
  try {
    r.text = doc.dump(2) + "\n";
  } catch (const std::exception& e) {
    r.error = std::string("serialisation failed: ") + e.what();  // e.g. invalid UTF-8 in the name
  }
  return r;
}

SaveResult saveTrackDocumentFile(const std::filesystem::path& path, const TrackDocument& document,
                                 OverwritePolicy policy) {
  SaveResult r;
  const SerializeResult s = serializeTrackDocumentJson(document);
  if (!s.ok()) {
    r.error = s.error;
    return r;
  }
  r.error = detail::writeTextFileAtomically(path, s.text, policy);
  return r;
}

}  // namespace pathforge::io
