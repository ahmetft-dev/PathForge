#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <pathforge/editor/derived_geometry.hpp>
#include <pathforge/geometry/validation.hpp>
#include <pathforge/io/document_io.hpp>
#include <sstream>

using namespace pathforge::editor;
using namespace pathforge::io;
namespace fs = std::filesystem;
using pathforge::geometry::ValidationStatus;

namespace {

TrackDocument sampleDocument() {
  TrackDocument d;
  d.setName("Circuit \xC3\xA9t\xC3\xA9");  // UTF-8 name survives
  d.setClosure(Closure::Closed);
  GeometrySettings s;
  s.alpha = 0.3;
  s.resampleSpacing = 1.25;
  d.setSettings(s);
  for (int i = 0; i < 9; ++i) {
    const double a = 2.0 * std::numbers::pi * i / 9;
    d.appendPoint({120.0 * std::cos(a) + 0.1, 80.0 * std::sin(a) - 1.0 / 3.0}, 4.0 + 0.25 * i,
                  5.0 - 0.125 * i);
  }
  return d;
}

void expectSameContent(const TrackDocument& a, const TrackDocument& b) {
  EXPECT_EQ(a.name(), b.name());
  EXPECT_EQ(a.closure(), b.closure());
  EXPECT_EQ(a.settings(), b.settings());
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a.points()[i].position, b.points()[i].position) << i;  // bitwise
    EXPECT_EQ(a.points()[i].leftWidth, b.points()[i].leftWidth) << i;
    EXPECT_EQ(a.points()[i].rightWidth, b.points()[i].rightWidth) << i;
  }
}

std::string replaceOnce(std::string s, const std::string& from, const std::string& to) {
  const auto pos = s.find(from);
  EXPECT_NE(pos, std::string::npos) << "missing: " << from;
  if (pos != std::string::npos) s.replace(pos, from.size(), to);
  return s;
}

std::string dataFile(const char* name) { return std::string(PATHFORGE_TEST_DATA_DIR) + "/" + name; }

class TempDir {
 public:
  TempDir() {
    static int counter = 0;
    path_ = fs::temp_directory_path() / ("pathforge_docio_test_" + std::to_string(++counter) + "_" +
                                         std::to_string(::testing::UnitTest::GetInstance()->random_seed()));
    fs::create_directories(path_);
  }
  ~TempDir() {
    std::error_code ec;
    fs::remove_all(path_, ec);
  }
  const fs::path& path() const { return path_; }

 private:
  fs::path path_;
};

}  // namespace

TEST(DocumentJson, RoundTripPreservesEditableStateExactly) {
  const TrackDocument d = sampleDocument();
  const SerializeResult s = serializeTrackDocumentJson(d);
  ASSERT_TRUE(s.ok()) << s.error;
  const DocumentLoadResult r = parseTrackDocumentJson(s.text);
  ASSERT_TRUE(r.ok()) << r.error;
  EXPECT_EQ(r.schemaVersion, 2);
  EXPECT_FALSE(r.legacyData.has_value());
  EXPECT_TRUE(r.warnings.empty());
  ASSERT_TRUE(r.document.has_value());
  expectSameContent(d, *r.document);
  // Derived geometry is a pure function of the editable state, so it is reproduced bit-for-bit.
  EXPECT_EQ(generateGeometry(*r.document).data.centerline, generateGeometry(d).data.centerline);
  EXPECT_EQ(generateGeometry(*r.document).data.leftWidths, generateGeometry(d).data.leftWidths);
}

TEST(DocumentJson, SaveLoadSaveIsByteIdenticalAndLoadedDocumentsStayEditable) {
  const std::string first = serializeTrackDocumentJson(sampleDocument()).text;
  DocumentLoadResult r = parseTrackDocumentJson(first);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(serializeTrackDocumentJson(*r.document).text, first);  // ids are not stored: no drift
  // Save -> Load -> Edit -> Save -> Load.
  TrackDocument& d = *r.document;
  d.insertPoint(2, {200.0, 5.0}, 7.0, 7.0);
  d.setPointWidths(d.points()[0].id, 9.5, 1.5);
  d.setClosure(Closure::Open);
  const DocumentLoadResult again = parseTrackDocumentJson(serializeTrackDocumentJson(d).text);
  ASSERT_TRUE(again.ok());
  expectSameContent(d, *again.document);
}

TEST(DocumentJson, OpenPolylineAndEmptyDocumentsRoundTrip) {
  TrackDocument open;
  GeometrySettings s;
  s.interpolation = Interpolation::Polyline;
  s.resampleSpacing = 0.0;
  open.setSettings(s);
  open.appendPoint({0.1, 0.2}, 1e-3, 3.0);
  open.appendPoint({1e6, -1e6}, 0.0, 1e4);
  DocumentLoadResult r = parseTrackDocumentJson(serializeTrackDocumentJson(open).text);
  ASSERT_TRUE(r.ok());
  expectSameContent(open, *r.document);
  const TrackDocument empty;
  r = parseTrackDocumentJson(serializeTrackDocumentJson(empty).text);
  ASSERT_TRUE(r.ok());
  expectSameContent(empty, *r.document);
}

// A legacy sampled track becomes a polyline document without resampling, so its derived data is EXACTLY the
// stored samples; this holds even for invalid tracks, whose raw data is passed through for diagnostics.
TEST(DocumentJson, LegacyV1TracksImportWithoutChangingGeometry) {
  for (const char* f : {"valid_oval.json", "bowtie.json", "duplicate_points.json", "unknown_field.json"}) {
    const DocumentLoadResult r = loadTrackDocumentFile(dataFile(f));
    ASSERT_TRUE(r.ok()) << f << ": " << r.error;
    EXPECT_EQ(r.schemaVersion, 1) << f;
    ASSERT_TRUE(r.legacyData.has_value());
    ASSERT_TRUE(r.document.has_value());
    EXPECT_EQ(r.document->settings().interpolation, Interpolation::Polyline);
    EXPECT_EQ(r.document->settings().resampleSpacing, 0.0);
    const DerivedGeometry g = generateGeometry(*r.document);
    EXPECT_EQ(g.data.centerline, r.legacyData->centerline) << f;
    EXPECT_EQ(g.data.leftWidths, r.legacyData->leftWidths) << f;
    EXPECT_EQ(g.data.rightWidths, r.legacyData->rightWidths) << f;
    EXPECT_EQ(g.data.closure, r.legacyData->closure) << f;
    EXPECT_EQ(validateTrackData(g.data).status(), validateTrackData(*r.legacyData).status()) << f;
    bool warned = false;
    for (const auto& w : r.warnings) warned = warned || w.find("polyline document") != std::string::npos;
    EXPECT_TRUE(warned) << f;
  }
}

TEST(DocumentJson, LegacyTracksThatCannotBeEditedStillLoadAsData) {
  std::ifstream in(dataFile("valid_oval.json"), std::ios::binary);
  std::stringstream ss;
  ss << in.rdbuf();
  // Negative widths are representable in a v1 file (validation reports them) but not in an editable document.
  const DocumentLoadResult r =
      parseTrackDocumentJson(replaceOnce(ss.str(), "\"left\": 5.0", "\"left\": -5.0"));
  ASSERT_TRUE(r.ok()) << r.error;
  EXPECT_TRUE(r.legacyData.has_value());
  EXPECT_FALSE(r.document.has_value());
  bool warned = false;
  for (const auto& w : r.warnings) warned = warned || w.find("cannot be edited") != std::string::npos;
  EXPECT_TRUE(warned);
}

TEST(DocumentJson, InvalidButRepairableDocumentsLoad) {
  TrackDocument d;
  d.setClosure(Closure::Closed);
  d.appendPoint({0, 0}, 0.0, 5.0);  // zero width and only two points of a closed track
  d.appendPoint({10, 0}, 5.0, 5.0);
  const DocumentLoadResult r = parseTrackDocumentJson(serializeTrackDocumentJson(d).text);
  ASSERT_TRUE(r.ok()) << r.error;
  EXPECT_EQ(generateGeometry(*r.document).status, DerivedStatus::TooFewPoints);
  EXPECT_EQ(validateTrackData(generateGeometry(*r.document).data).status(), ValidationStatus::Invalid);
}

TEST(DocumentJson, RejectsUnsupportedVersionsAndCorruptDocuments) {
  const std::string base = serializeTrackDocumentJson(sampleDocument()).text;
  const DocumentLoadResult v3 =
      parseTrackDocumentJson(replaceOnce(base, "\"schemaVersion\": 2", "\"schemaVersion\": 3"));
  EXPECT_FALSE(v3.ok());
  EXPECT_NE(v3.error.find("unsupported schemaVersion 3"), std::string::npos);
  EXPECT_FALSE(
      parseTrackDocumentJson(replaceOnce(base, "\"schemaVersion\": 2", "\"schemaVersion\": \"2\"")).ok());

  struct Case {
    std::string from, to, expect;
  };
  const Case cases[] = {
      {"\"geometry\"", "\"geo\"", "'geometry'"},
      {"\"controlPoints\"", "\"points\"", "'controlPoints'"},
      {"\"catmull-rom\"", "\"bezier\"", "interpolation"},
      {"\"alpha\": 0.3", "\"alpha\": 1.5", "alpha"},
      {"\"resampleSpacing\": 1.25", "\"resampleSpacing\": -1.0", "spacing"},
      {"\"leftWidth\": 4.0", "\"leftWidth\": -4.0", "controlPoints[0]"},
      {"\"leftWidth\": 4.0", "\"lw\": 4.0", "'leftWidth'"},
      {"\"closed\": true", "\"closed\": \"yes\"", "closed"},
      {"\"units\": \"SI\"", "\"units\": \"imperial\"", "units"},
  };
  for (const Case& c : cases) {
    const DocumentLoadResult r = parseTrackDocumentJson(replaceOnce(base, c.from, c.to));
    EXPECT_FALSE(r.ok()) << c.to;
    EXPECT_FALSE(r.document.has_value());
    EXPECT_NE(r.error.find(c.expect), std::string::npos) << c.to << " -> " << r.error;
  }
  // Coordinates beyond the document limit are rejected with the offending point named.
  const std::string far = replaceOnce(base, "\"y\": -0.3333333333333333", "\"y\": 1e8");
  EXPECT_NE(parseTrackDocumentJson(far).error.find("controlPoints[0]"), std::string::npos);
  EXPECT_NE(parseTrackDocumentJson("{").error.find("malformed JSON"), std::string::npos);
  EXPECT_FALSE(parseTrackDocumentJson(std::string(100, '[') + std::string(100, ']')).ok());
  EXPECT_FALSE(parseTrackDocumentJson(base.substr(0, base.size() / 2)).ok());  // truncated file
}

TEST(DocumentJson, UnknownFieldsAreReportedNotSilentlyDropped) {
  const std::string base = serializeTrackDocumentJson(sampleDocument()).text;
  const DocumentLoadResult r = parseTrackDocumentJson(
      replaceOnce(base, "\"resampleSpacing\"", "\"smoothing\": 2,\n    \"resampleSpacing\""));
  ASSERT_TRUE(r.ok()) << r.error;
  ASSERT_EQ(r.warnings.size(), 1u);
  EXPECT_NE(r.warnings[0].find("smoothing"), std::string::npos);
}

TEST(DocumentFile, SaveLoadOverwriteAndFailures) {
  TempDir dir;
  const fs::path file = dir.path() / "track.json";
  const TrackDocument d = sampleDocument();
  ASSERT_TRUE(saveTrackDocumentFile(file, d).ok());
  const DocumentLoadResult r = loadTrackDocumentFile(file);
  ASSERT_TRUE(r.ok()) << r.error;
  expectSameContent(d, *r.document);

  TrackDocument other = d;
  other.setName("other");
  const SaveResult refused = saveTrackDocumentFile(file, other);  // default policy: never overwrite
  EXPECT_FALSE(refused.ok());
  EXPECT_EQ(loadTrackDocumentFile(file).document->name(), d.name());
  ASSERT_TRUE(saveTrackDocumentFile(file, other, OverwritePolicy::Overwrite).ok());
  EXPECT_EQ(loadTrackDocumentFile(file).document->name(), "other");

  EXPECT_FALSE(saveTrackDocumentFile(dir.path() / "missing" / "x.json", d).ok());
  EXPECT_FALSE(fs::exists(dir.path() / "missing"));
  EXPECT_NE(loadTrackDocumentFile(dir.path() / "none.json").error.find("not found"), std::string::npos);
  std::size_t files = 0;
  for (const auto& e : fs::directory_iterator(dir.path())) {
    (void)e;
    ++files;
  }
  EXPECT_EQ(files, 1u);  // no temporary files left behind
}

TEST(DocumentFile, SampledDataConversionChecksArraySizes) {
  pathforge::geometry::TrackData bad;
  bad.centerline = {{0, 0}, {1, 0}};
  bad.leftWidths = {1.0};
  bad.rightWidths = {1.0, 1.0};
  EXPECT_THROW(documentFromTrackData(bad), EditError);
}
