#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <numbers>
#include <pathforge/geometry/circle.hpp>
#include <pathforge/geometry/shapes.hpp>
#include <pathforge/geometry/validation.hpp>
#include <pathforge/io/track_io.hpp>
#include <sstream>
#include <string>

using namespace pathforge::geometry;
using namespace pathforge::io;
namespace fs = std::filesystem;

namespace {

TrackData sample(const std::string& name = "Sample") {
  return toTrackData(Track::create(makeStadium(300.0, 120.0, 64), 5.0, 4.0), name);
}

std::string replaceOnce(std::string s, const std::string& from, const std::string& to) {
  const auto pos = s.find(from);
  EXPECT_NE(pos, std::string::npos) << "missing: " << from;
  if (pos != std::string::npos) s.replace(pos, from.size(), to);
  return s;
}

// Unique scratch directory removed at the end of each test.
class TempDir {
 public:
  TempDir() {
    static int counter = 0;
    path_ = fs::temp_directory_path() /
            ("pathforge_io_test_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) + "_" +
             std::to_string(++counter));
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

std::string slurp(const fs::path& p) {
  std::ifstream in(p, std::ios::binary);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

std::string dataFile(const char* name) { return std::string(PATHFORGE_TEST_DATA_DIR) + "/" + name; }

}  // namespace

TEST(TrackJson, RoundTripIsExactForGeneratedTracks) {
  for (const TrackData& d :
       {sample("oval"), toTrackData(Track::create(makeCircle(100.0, 360), 5.0, 5.0), "c"),
        toTrackData(Track::create(makeSCurve(300.0, 30.0, 256), 5.0, 5.0), "s")}) {
    const SerializeResult s = serializeTrackJson(d);
    ASSERT_TRUE(s.ok()) << s.error;
    const LoadResult r = parseTrackJson(s.text);
    ASSERT_TRUE(r.ok()) << r.error;
    EXPECT_TRUE(r.warnings.empty());
    EXPECT_EQ(r.data->name, d.name);
    EXPECT_EQ(r.data->closure, d.closure);
    EXPECT_EQ(r.data->centerline, d.centerline);  // bitwise-equal doubles
    EXPECT_EQ(r.data->leftWidths, d.leftWidths);
    EXPECT_EQ(r.data->rightWidths, d.rightWidths);
  }
}

// nlohmann writes the shortest decimal string that parses back to the same double, so every finite double
// (including subnormals and the extremes) must survive exactly.
TEST(TrackJson, DoublePrecisionSurvivesRoundTrip) {
  TrackData d;
  d.name = "precision";
  d.closure = Closure::Open;
  d.centerline = {{0.1, 1.0 / 3.0},
                  {std::numbers::pi * 1e6, -std::numbers::e},
                  {5e-324, std::numeric_limits<double>::max()},
                  {-std::numeric_limits<double>::min(), 1.0000000000000002}};
  d.leftWidths = {0.1, 1.0 / 3.0, 1e-300, 2.2250738585072014e-308};
  d.rightWidths = {1e300, 4.9e-324, 7.0, 0.30000000000000004};
  const LoadResult r = parseTrackJson(serializeTrackJson(d).text);
  ASSERT_TRUE(r.ok()) << r.error;
  EXPECT_EQ(r.data->centerline, d.centerline);
  EXPECT_EQ(r.data->leftWidths, d.leftWidths);
  EXPECT_EQ(r.data->rightWidths, d.rightWidths);
}

TEST(TrackJson, OutputIsDeterministicWithDocumentedKeyOrder) {
  const TrackData d = sample();
  const std::string a = serializeTrackJson(d).text, b = serializeTrackJson(d).text;
  EXPECT_EQ(a, b);
  const auto pos = [&](const char* key) { return a.find(key); };
  EXPECT_LT(pos("\"schemaVersion\""), pos("\"name\""));
  EXPECT_LT(pos("\"name\""), pos("\"units\""));
  EXPECT_LT(pos("\"units\""), pos("\"coordinateSystem\""));
  EXPECT_LT(pos("\"coordinateSystem\""), pos("\"closed\""));
  EXPECT_LT(pos("\"closed\""), pos("\"centerline\""));
  EXPECT_LT(pos("\"centerline\""), pos("\"widths\""));
  EXPECT_EQ(a.back(), '\n');
}

TEST(TrackJson, ConstantWidthsUseCompactFormAndVariableWidthsUseArrays) {
  const std::string constant = serializeTrackJson(sample()).text;
  EXPECT_NE(constant.find("\"left\": 5.0"), std::string::npos);
  EXPECT_NE(constant.find("\"right\": 4.0"), std::string::npos);
  TrackData v = sample();
  v.leftWidths[3] = 6.5;
  const std::string variable = serializeTrackJson(v).text;
  EXPECT_NE(variable.find("\"left\": ["), std::string::npos);
  const LoadResult r = parseTrackJson(variable);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.data->leftWidths, v.leftWidths);
  // A constant given as one number is expanded to one value per point.
  const LoadResult c = parseTrackJson(constant);
  ASSERT_TRUE(c.ok());
  EXPECT_EQ(c.data->leftWidths.size(), c.data->centerline.size());
}

TEST(TrackJson, RejectsMalformedSyntax) {
  for (const char* text :
       {"", "{", "not json", "[1,2,3]", "{\"schemaVersion\": 1,}", "{\"a\": NaN}", "{\"a\": Infinity}"}) {
    const LoadResult r = parseTrackJson(text);
    EXPECT_FALSE(r.ok()) << text;
    EXPECT_FALSE(r.error.empty()) << text;
  }
  EXPECT_NE(parseTrackJson("{").error.find("malformed JSON"), std::string::npos);
}

TEST(TrackJson, ReportsMissingRequiredFieldsByName) {
  const std::string base = serializeTrackJson(sample()).text;
  struct Case {
    const char* field;
    std::string from;
  };
  for (const Case& c :
       {Case{"schemaVersion", "\"schemaVersion\""}, Case{"name", "\"name\""}, Case{"units", "\"units\""},
        Case{"coordinateSystem", "\"coordinateSystem\""}, Case{"closed", "\"closed\""},
        Case{"centerline", "\"centerline\""}, Case{"widths", "\"widths\""}}) {
    const LoadResult r = parseTrackJson(replaceOnce(base, c.from, "\"renamed\""));
    EXPECT_FALSE(r.ok()) << c.field;
    EXPECT_NE(r.error.find(c.field), std::string::npos) << r.error;
  }
  // Missing members inside nested objects.
  EXPECT_NE(parseTrackJson(replaceOnce(base, "\"x\"", "\"xx\"")).error.find("'x'"), std::string::npos);
  EXPECT_NE(parseTrackJson(replaceOnce(base, "\"left\"", "\"l\"")).error.find("'left'"), std::string::npos);
}

TEST(TrackJson, RejectsUnsupportedSchemaVersionsWithActionableMessage) {
  const std::string base = serializeTrackJson(sample()).text;
  const LoadResult v3 = parseTrackJson(replaceOnce(base, "\"schemaVersion\": 1", "\"schemaVersion\": 3"));
  ASSERT_FALSE(v3.ok());
  EXPECT_NE(v3.error.find("unsupported schemaVersion 3"), std::string::npos);
  EXPECT_NE(v3.error.find("upgrade"), std::string::npos);
  // Version 2 exists but is an editable document: the sampled-track reader points to the right API.
  const LoadResult v2 = parseTrackJson(replaceOnce(base, "\"schemaVersion\": 1", "\"schemaVersion\": 2"));
  ASSERT_FALSE(v2.ok());
  EXPECT_NE(v2.error.find("editable track document"), std::string::npos);
  EXPECT_FALSE(parseTrackJson(replaceOnce(base, "\"schemaVersion\": 1", "\"schemaVersion\": 0")).ok());
  EXPECT_FALSE(parseTrackJson(replaceOnce(base, "\"schemaVersion\": 1", "\"schemaVersion\": \"1\"")).ok());
  EXPECT_FALSE(parseTrackJson(replaceOnce(base, "\"schemaVersion\": 1", "\"schemaVersion\": 1.5")).ok());
}

TEST(TrackJson, RejectsWrongTypesAndUnits) {
  const std::string base = serializeTrackJson(sample()).text;
  EXPECT_FALSE(parseTrackJson(replaceOnce(base, "\"units\": \"SI\"", "\"units\": \"feet\"")).ok());
  EXPECT_FALSE(parseTrackJson(replaceOnce(base, "\"xy-right-handed\"", "\"xy-left-handed\"")).ok());
  EXPECT_FALSE(parseTrackJson(replaceOnce(base, "\"closed\": true", "\"closed\": 1")).ok());
  EXPECT_FALSE(parseTrackJson(replaceOnce(base, "\"name\": \"Sample\"", "\"name\": 5")).ok());
  EXPECT_FALSE(parseTrackJson(replaceOnce(base, "\"y\": -60.0", "\"y\": \"abc\"")).ok());
  EXPECT_FALSE(parseTrackJson(replaceOnce(base, "\"left\": 5.0", "\"left\": true")).ok());
}

TEST(TrackJson, UnknownFieldsAreIgnoredButReported) {
  const std::string base = serializeTrackJson(sample()).text;
  const LoadResult top = parseTrackJson(replaceOnce(base, "\"closed\"", "\"comment\": \"x\",\n  \"closed\""));
  ASSERT_TRUE(top.ok()) << top.error;
  ASSERT_EQ(top.warnings.size(), 1u);
  EXPECT_NE(top.warnings[0].find("comment"), std::string::npos);
  const LoadResult point = parseTrackJson(replaceOnce(base, "\"x\": 0.0,", "\"x\": 0.0, \"z\": 1.0,"));
  ASSERT_TRUE(point.ok());
  EXPECT_NE(point.warnings.at(0).find("'z'"), std::string::npos);
  const LoadResult widths = parseTrackJson(replaceOnce(base, "\"left\": 5.0", "\"extra\": 1, \"left\": 5.0"));
  ASSERT_TRUE(widths.ok());
  EXPECT_NE(widths.warnings.at(0).find("widths"), std::string::npos);
  // Ignored fields are NOT preserved: saving the loaded data drops them (documented, reported as warnings).
  EXPECT_EQ(serializeTrackJson(*top.data).text.find("comment"), std::string::npos);
}

TEST(TrackJson, NonFiniteNumbersAreRejectedOnReadAndWrite) {
  const std::string base = serializeTrackJson(sample()).text;
  const LoadResult big = parseTrackJson(replaceOnce(base, "\"y\": -60.0", "\"y\": 1e999"));
  EXPECT_FALSE(big.ok());
  EXPECT_NE(big.error.find("1e999"), std::string::npos);  // the parser names the out-of-range literal

  const double nan = std::numeric_limits<double>::quiet_NaN();
  TrackData d = sample();
  d.centerline[2].x = nan;
  EXPECT_FALSE(serializeTrackJson(d).ok());
  d = sample();
  d.leftWidths[1] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(serializeTrackJson(d).ok());
  d = sample();
  d.rightWidths.pop_back();  // size mismatch cannot be represented
  EXPECT_FALSE(serializeTrackJson(d).ok());
}

TEST(TrackJson, RejectsExcessiveNesting) {
  const std::string deep = std::string(100, '[') + std::string(100, ']');
  const LoadResult r = parseTrackJson(deep);
  EXPECT_FALSE(r.ok());
  EXPECT_NE(r.error.find("nesting"), std::string::npos);
  // Brackets inside strings must not count.
  const std::string ok = replaceOnce(serializeTrackJson(sample()).text, "\"Sample\"",
                                     "\"[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[\"");
  EXPECT_TRUE(parseTrackJson(ok).ok());
}

// Syntactically valid files can still describe unusable tracks; they load (so they can be inspected) and the
// geometric validation reports the problem. These are separate results by design.
TEST(TrackJson, ParsingIsSeparateFromGeometricValidity) {
  const std::string base = serializeTrackJson(sample()).text;
  // Empty centerline: widths given as constants expand to zero entries.
  TrackData empty = sample();
  empty.centerline.clear();
  empty.leftWidths.clear();
  empty.rightWidths.clear();
  const LoadResult e = parseTrackJson(serializeTrackJson(empty).text);
  ASSERT_TRUE(e.ok()) << e.error;
  EXPECT_EQ(validateTrackData(*e.data).status(), ValidationStatus::Invalid);

  const LoadResult dup = loadTrackFile(dataFile("duplicate_points.json"));
  ASSERT_TRUE(dup.ok()) << dup.error;
  const ValidationReport rd = validateTrackData(*dup.data);
  EXPECT_EQ(rd.status(), ValidationStatus::Invalid);
  EXPECT_FALSE(rd.checked.structure);

  const LoadResult bow = loadTrackFile(dataFile("bowtie.json"));
  ASSERT_TRUE(bow.ok());
  EXPECT_EQ(validateTrackData(*bow.data).status(), ValidationStatus::Invalid);

  // Negative width: loadable, reported by validation.
  const LoadResult neg = parseTrackJson(replaceOnce(base, "\"left\": 5.0", "\"left\": -5.0"));
  ASSERT_TRUE(neg.ok());
  EXPECT_EQ(validateTrackData(*neg.data).status(), ValidationStatus::Invalid);

  // Width array of the wrong length.
  const LoadResult mis = parseTrackJson(replaceOnce(base, "\"left\": 5.0", "\"left\": [1.0, 2.0]"));
  ASSERT_TRUE(mis.ok());
  const ValidationReport rm = validateTrackData(*mis.data);
  EXPECT_EQ(rm.status(), ValidationStatus::Invalid);
  EXPECT_EQ(rm.issues.at(0).type, TrackIssue::WidthCountMismatch);
}

TEST(TrackFile, SaveThenLoadRoundTrips) {
  TempDir dir;
  const fs::path file = dir.path() / "track.json";
  const TrackData d = sample("File Track");
  ASSERT_TRUE(saveTrackFile(file, d).ok());
  const LoadResult r = loadTrackFile(file);
  ASSERT_TRUE(r.ok()) << r.error;
  EXPECT_EQ(r.data->centerline, d.centerline);
  EXPECT_EQ(r.data->name, "File Track");
  // No temporary file is left behind.
  std::size_t files = 0;
  for (const auto& e : fs::directory_iterator(dir.path())) {
    (void)e;
    ++files;
  }
  EXPECT_EQ(files, 1u);
}

TEST(TrackFile, OverwritePolicyIsRespected) {
  TempDir dir;
  const fs::path file = dir.path() / "track.json";
  ASSERT_TRUE(saveTrackFile(file, sample("first")).ok());
  const std::string before = slurp(file);
  const SaveResult refused = saveTrackFile(file, sample("second"), OverwritePolicy::Fail);
  ASSERT_FALSE(refused.ok());
  EXPECT_NE(refused.error.find("overwrite"), std::string::npos);
  EXPECT_EQ(slurp(file), before);  // untouched
  ASSERT_TRUE(saveTrackFile(file, sample("second"), OverwritePolicy::Overwrite).ok());
  EXPECT_NE(slurp(file), before);
  EXPECT_EQ(loadTrackFile(file).data->name, "second");
}

TEST(TrackFile, SaveFailuresLeaveNoPartialFiles) {
  TempDir dir;
  // Missing directory.
  const SaveResult missing = saveTrackFile(dir.path() / "nope" / "track.json", sample());
  EXPECT_FALSE(missing.ok());
  EXPECT_FALSE(fs::exists(dir.path() / "nope"));
  // Target is a directory: the final rename must fail, and the temporary file must be cleaned up.
  fs::create_directory(dir.path() / "asdir");
  const SaveResult isDir = saveTrackFile(dir.path() / "asdir", sample(), OverwritePolicy::Overwrite);
  EXPECT_FALSE(isDir.ok());
  EXPECT_FALSE(fs::exists(dir.path() / "asdir.pathforge-tmp"));
  // Unserialisable data is rejected before anything is written.
  TrackData bad = sample();
  bad.centerline[0].x = std::numeric_limits<double>::infinity();
  const fs::path target = dir.path() / "bad.json";
  EXPECT_FALSE(saveTrackFile(target, bad).ok());
  EXPECT_FALSE(fs::exists(target));
}

TEST(TrackFile, LoadFailuresAreReportedAndHarmless) {
  TempDir dir;
  EXPECT_NE(loadTrackFile(dir.path() / "missing.json").error.find("not found"), std::string::npos);
  EXPECT_NE(loadTrackFile(dir.path()).error.find("not a regular file"), std::string::npos);
  const fs::path corrupt = dir.path() / "corrupt.json";
  {
    std::ofstream(corrupt, std::ios::binary) << "{ \"schemaVersion\": 1, \"name\": ";
  }
  const LoadResult r = loadTrackFile(corrupt);
  EXPECT_FALSE(r.ok());
  EXPECT_FALSE(r.data.has_value());
  EXPECT_NE(r.error.find("malformed JSON"), std::string::npos);
  const fs::path empty = dir.path() / "empty.json";
  {
    std::ofstream(empty, std::ios::binary);
  }
  EXPECT_FALSE(loadTrackFile(empty).ok());
  const fs::path binary = dir.path() / "binary.json";
  {
    std::ofstream out(binary, std::ios::binary);
    for (int i = 0; i < 256; ++i) out.put(static_cast<char>(i));
  }
  EXPECT_FALSE(loadTrackFile(binary).ok());  // must not crash on arbitrary bytes
}

TEST(TrackFile, FixturesBehaveAsDocumented) {
  EXPECT_TRUE(loadTrackFile(dataFile("valid_oval.json")).ok());
  EXPECT_NE(loadTrackFile(dataFile("wrong_version.json")).error.find("unsupported schemaVersion 3"),
            std::string::npos);
  EXPECT_FALSE(loadTrackFile(dataFile("malformed.json")).ok());
  const LoadResult extra = loadTrackFile(dataFile("unknown_field.json"));
  ASSERT_TRUE(extra.ok());
  EXPECT_EQ(extra.warnings.size(), 1u);
}

TEST(TrackIntegration, GeneratedTrackSurvivesJsonAndValidates) {
  TempDir dir;
  const fs::path file = dir.path() / "oval.json";
  const Track original = Track::create(makeStadium(300.0, 120.0, 512), 5.0, 5.0);
  ASSERT_TRUE(saveTrackFile(file, toTrackData(original, "oval")).ok());
  const LoadResult loaded = loadTrackFile(file);
  ASSERT_TRUE(loaded.ok());
  const Track rebuilt = buildTrack(*loaded.data);
  EXPECT_EQ(rebuilt.leftBoundary(), original.leftBoundary());  // boundaries are recomputed identically
  EXPECT_EQ(rebuilt.rightBoundary(), original.rightBoundary());
  EXPECT_EQ(validateTrackData(*loaded.data).status(), ValidationStatus::Valid);
}
