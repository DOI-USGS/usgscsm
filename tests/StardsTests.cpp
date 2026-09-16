#define _USE_MATH_DEFINES

#include "Utilities.h"
#include "UsgsAstroFrameSensorModel.h"
#include "UsgsAstroLsSensorModel.h"
#include "UsgsAstroPlugin.h"

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

// Without STARDS support this translation unit is intentionally empty.
#ifdef USGSCSM_ENABLE_STARDS

using json = nlohmann::json;

namespace {

// Generated at build time from data/ ISDs; see tests/CMakeLists.txt. Also the
// scratch directory the write-path tests use, since it is inside the build tree.
const std::string kLineScanJson =
    std::string(STARDS_FIXTURE_DIR) + "/lineScanState.json";
const std::string kLineScanStards =
    std::string(STARDS_FIXTURE_DIR) + "/lineScanState.stards";
const std::string kFrameJson =
    std::string(STARDS_FIXTURE_DIR) + "/frameState.json";
// Committed golden, so a STARDS format change still fails a test. Relative to
// the ctest working directory, which is the source tests/ dir.
const std::string kFrameStards = "data/frameState.stards";

std::string scratchPath(const std::string &name) {
  return std::string(STARDS_FIXTURE_DIR) + "/" + name;
}

// csm_translate writes states as "<MODELNAME>\n<json>". stateAsJson() skips that
// preamble on the way in, so the whole file can be handed back as-is; the model
// name is what isUsgsCsmState() pulls out of it.
std::string readState(const std::string &path, std::string &modelName) {
  std::string full;
  EXPECT_TRUE(readFileInString(path, full)) << "could not read " << path;
  EXPECT_TRUE(isUsgsCsmState(full, modelName)) << "not a model state: " << path;
  return full;
}

// A model built from the JSON state and one built from the STARDS state must
// project the given pixel to the same ground point.
void expectSameGroundPoint(const std::string &jsonPath,
                           const std::string &stardsPath,
                           const csm::ImageCoord &imagePt) {
  std::string modelName;
  const std::string state = readState(jsonPath, modelName);

  std::unique_ptr<csm::RasterGM> fromJson(
      getUsgsCsmModelFromJsonState(state, modelName, nullptr));
  std::unique_ptr<csm::RasterGM> fromStards(
      getUsgsCsmModelFromStards(stardsPath, nullptr));
  ASSERT_NE(fromJson.get(), nullptr);
  ASSERT_NE(fromStards.get(), nullptr);

  const csm::EcefCoord gJson = fromJson->imageToGround(imagePt, 0.0);
  const csm::EcefCoord gStards = fromStards->imageToGround(imagePt, 0.0);
  EXPECT_NEAR(gJson.x, gStards.x, 1e-6);
  EXPECT_NEAR(gJson.y, gStards.y, 1e-6);
  EXPECT_NEAR(gJson.z, gStards.z, 1e-6);
}

}  // namespace

// A STARDS state and the equivalent JSON state give the same VariantMap.
TEST(Stards, VariantMapMatchesJsonState) {
  std::string modelName;
  VariantMap fromJson =
      variantMapFromJson(stateAsJson(readState(kLineScanJson, modelName)));
  VariantMap fromStards = variantMapFromStards(kLineScanStards);

  // Every key present in the JSON state is present in the STARDS state.
  for (const std::string &key : fromJson.keys()) {
    ASSERT_TRUE(fromStards.contains(key)) << "missing key: " << key;
  }

  // A representative scalar and a large array agree.
  EXPECT_DOUBLE_EQ(fromJson.get<double>("m_focalLength"),
                   fromStards.get<double>("m_focalLength"));
  std::vector<double> qJson = fromJson.get<std::vector<double>>("m_quaternions");
  std::vector<double> qStards =
      fromStards.get<std::vector<double>>("m_quaternions");
  ASSERT_EQ(qJson.size(), qStards.size());
  for (size_t i = 0; i < qJson.size(); ++i) {
    EXPECT_DOUBLE_EQ(qJson[i], qStards[i]) << "quaternion element " << i;
  }
}

// A line scan model from STARDS projects identically to one from JSON.
TEST(Stards, LineScanModelMatchesJsonState) {
  expectSameGroundPoint(kLineScanJson, kLineScanStards,
                        csm::ImageCoord(500.0, 500.0));
}

// A frame model from the committed golden projects identically to one from JSON.
TEST(Stards, FrameModelMatchesJsonState) {
  expectSameGroundPoint(kFrameJson, kFrameStards, csm::ImageCoord(7.5, 7.5));
}

// The plugin routes a STARDS image file through the STARDS state path.
TEST(Stards, PluginConstructsFromStardsFile) {
  csm::Isd isd;
  isd.setFilename(kLineScanStards);
  UsgsAstroPlugin plugin;
  std::unique_ptr<csm::Model> model(plugin.constructModelFromISD(
      isd, UsgsAstroLsSensorModel::_SENSOR_MODEL_NAME, nullptr));
  EXPECT_NE(dynamic_cast<UsgsAstroLsSensorModel *>(model.get()), nullptr);
}

// Writing a model out to STARDS and reading it back preserves the geometry, so
// the write path is exercised independently of the csm_translate-built fixtures.
TEST(Stards, ModelSurvivesWriteThenRead) {
  std::string modelName;
  const std::string state = readState(kLineScanJson, modelName);
  std::unique_ptr<csm::RasterGM> source(
      getUsgsCsmModelFromJsonState(state, modelName, nullptr));
  ASSERT_NE(source.get(), nullptr);

  const std::string out = scratchPath("writeThenRead.stards");
  writeUsgsCsmModelToStards(source.get(), out);
  EXPECT_EQ(modelFormatOfFile(out), ModelFormat::Stards);

  expectSameGroundPoint(kLineScanJson, out, csm::ImageCoord(500.0, 500.0));
}

// Every VariantMap value type survives a write/read cycle, including the string
// vectors that only appear in some states and the size-1 vectors that STARDS
// cannot distinguish from scalars.
TEST(Stards, VariantMapRoundTripsEveryValueType) {
  VariantMap vm;
  vm.set<std::string>("m_modelName", "USGS_ASTRO_FRAME_SENSOR_MODEL");
  vm.set<int>("anInt", -7);
  vm.set<double>("aDouble", 1.5);
  vm.set<bool>("aBool", true);
  vm.set<std::vector<int>>("intVector", {1, 2, 3});
  vm.set<std::vector<double>>("doubleVector", {1.5, -2.5, 3.5});
  vm.set<std::vector<std::string>>("stringVector", {"REAL", "FIXED", "REAL"});

  const std::string out = scratchPath("allTypes.stards");
  variantMapToStards(vm, out);
  VariantMap back = variantMapFromStards(out);

  EXPECT_EQ(back.get<std::string>("m_modelName"),
            "USGS_ASTRO_FRAME_SENSOR_MODEL");
  EXPECT_EQ(back.get<int>("anInt"), -7);
  EXPECT_DOUBLE_EQ(back.get<double>("aDouble"), 1.5);
  // STARDS has no bool dtype, so a bool comes back as the int it was stored as.
  EXPECT_EQ(back.get<int>("aBool"), 1);
  EXPECT_EQ(back.get<std::vector<int>>("intVector"), std::vector<int>({1, 2, 3}));
  EXPECT_EQ(back.get<std::vector<double>>("doubleVector"),
            std::vector<double>({1.5, -2.5, 3.5}));
  EXPECT_EQ(back.get<std::vector<std::string>>("stringVector"),
            std::vector<std::string>({"REAL", "FIXED", "REAL"}));
}

// Values longer than the array threshold go to array storage and still read back
// unchanged, so both storage namespaces are covered.
TEST(Stards, LongArrayGoesToArrayStorageAndReadsBack) {
  std::vector<double> big(STARDS_DEFAULT_ARRAY_THRESHOLD + 10);
  for (size_t i = 0; i < big.size(); ++i) big[i] = static_cast<double>(i) * 0.25;

  VariantMap vm;
  vm.set<std::string>("m_modelName", "USGS_ASTRO_FRAME_SENSOR_MODEL");
  vm.set<std::vector<double>>("big", big);

  const std::string out = scratchPath("longArray.stards");
  variantMapToStards(vm, out);
  EXPECT_EQ(variantMapFromStards(out).get<std::vector<double>>("big"), big);
}

// An unrecognized compression name is rejected before any file is written.
TEST(Stards, RejectsUnknownCompression) {
  VariantMap vm;
  vm.set<std::string>("m_modelName", "USGS_ASTRO_FRAME_SENSOR_MODEL");
  EXPECT_THROW(variantMapToStards(vm, scratchPath("badCompression.stards"),
                                  "brotli"),
               csm::Error);
}

// A STARDS state with no m_modelName cannot name a model to build.
TEST(Stards, MissingModelNameThrows) {
  VariantMap vm;
  vm.set<double>("m_focalLength", 500.0);
  const std::string out = scratchPath("noModelName.stards");
  variantMapToStards(vm, out);
  EXPECT_THROW(getUsgsCsmModelFromStards(out, nullptr), csm::Error);
}

// The format sniffer keys off content, not the extension.
TEST(Stards, FormatDetectedFromContent) {
  EXPECT_EQ(modelFormatOfFile(kFrameStards), ModelFormat::Stards);
  EXPECT_EQ(modelFormatOfFile(kLineScanJson), ModelFormat::Text);
  EXPECT_EQ(modelFormatOfFile("data/does_not_exist.stards"),
            ModelFormat::Unknown);
  EXPECT_EQ(modelFormatFromBytes("STARDS\x01"), ModelFormat::Stards);
  EXPECT_EQ(modelFormatFromBytes("{\"m_modelName\": \"x\"}"), ModelFormat::Text);
  EXPECT_EQ(modelFormatFromBytes(std::string("\x82", 1)), ModelFormat::Msgpack);
  EXPECT_EQ(modelFormatFromBytes(""), ModelFormat::Unknown);
}

// A missing STARDS file raises a clear error rather than crashing.
TEST(Stards, MissingFileThrows) {
  EXPECT_THROW(variantMapFromStards("data/does_not_exist.stards"), csm::Error);
}

#endif  // USGSCSM_ENABLE_STARDS
