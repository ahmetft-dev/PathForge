#include <cmath>
#include <pathforge/io/track_io.hpp>

#include "json_support.hpp"

namespace pathforge::io {

using detail::ordered_json;
using detail::ParseError;
using geometry::Closure;
using geometry::Vec2;

LoadResult parseTrackJson(std::string_view text) {
  LoadResult r;
  try {
    const ordered_json doc = detail::parseGuarded(text);
    const long long v = detail::readSchemaVersion(doc);
    if (v == 2) {
      throw ParseError{
          "schemaVersion 2 is an editable track document; read it with loadTrackDocumentFile() "
          "(pathforge-cli and Studio do this automatically)"};
    }
    if (v != kTrackSchemaVersion) {
      throw ParseError{"unsupported schemaVersion " + std::to_string(v) +
                       " (this build reads versions 1 and 2); upgrade PathForge or re-export the track"};
    }
    r.data = detail::readTrackV1(doc, r.warnings);
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
  const auto text = detail::readTextFile(path, r.error);
  if (!text) return r;
  return parseTrackJson(*text);
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
  detail::writeHeader(doc, kTrackSchemaVersion, d.name, d.closure);
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
  r.error = detail::writeTextFileAtomically(path, s.text, policy);
  return r;
}

}  // namespace pathforge::io
