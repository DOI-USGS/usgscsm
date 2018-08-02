#include "UsgsAstroFramePlugin.h"

#include <json/json.hpp>

#include <fstream>

#include <gtest/gtest.h>

using json = nlohmann::json;

class FrameIsdTest : public ::testing::Test {
   protected:

      csm::Isd isd;

   virtual void SetUp() {
      std::ifstream isdFile("data/simpleFramerISD.json");
      json jsonIsd = json::parse(isdFile);
      isd.clearAllParams();
      for (json::iterator it = jsonIsd.begin(); it != jsonIsd.end(); ++it) {
         isd.addParam(it.key(), it.value().dump());
      }
   }
};

TEST(FramePluginTests, PluginName) {
   UsgsAstroFramePlugin testPlugin;
   EXPECT_EQ("UsgsAstroFramePluginCSM", testPlugin.getPluginName());;
}

TEST(FramePluginTests, ManufacturerName) {
   UsgsAstroFramePlugin testPlugin;
   EXPECT_EQ("UsgsAstrogeology", testPlugin.getManufacturer());;
}

TEST(FramePluginTests, ReleaseDate) {
   UsgsAstroFramePlugin testPlugin;
   EXPECT_EQ("20170425", testPlugin.getReleaseDate());;
}

TEST(FramePluginTests, NumModels) {
   UsgsAstroFramePlugin testPlugin;
   EXPECT_EQ(1, testPlugin.getNumModels());;
}

TEST(FramePluginTests, NoStateName) {
   UsgsAstroFramePlugin testPlugin;
   std::string badState = "{\"not_a_name\":\"bad_name\"}";
   EXPECT_FALSE(testPlugin.canModelBeConstructedFromState(
         "USGS_ASTRO_FRAME_SENSOR_MODEL",
         badState));;
}

TEST(FramePluginTests, BadStateName) {
   UsgsAstroFramePlugin testPlugin;
   std::string badState = "{\"model_name\":\"bad_name\"}";
   EXPECT_FALSE(testPlugin.canModelBeConstructedFromState(
         "USGS_ASTRO_FRAME_SENSOR_MODEL",
         badState));;
}

TEST(FramePluginTests, BadStateValue) {
   UsgsAstroFramePlugin testPlugin;
   std::string badState = "{"
         "\"model_name\":\"USGS_ASTRO_FRAME_SENSOR_MODEL\","
         "\"bad_param\":\"bad_value\"}";
   EXPECT_FALSE(testPlugin.canModelBeConstructedFromState(
         "USGS_ASTRO_FRAME_SENSOR_MODEL",
         badState));;
}

TEST(FramePluginTests, MissingStateValue) {
   UsgsAstroFramePlugin testPlugin;
   std::string badState = "{"
         "\"model_name\":\"USGS_ASTRO_FRAME_SENSOR_MODEL\"}";
   EXPECT_FALSE(testPlugin.canModelBeConstructedFromState(
         "USGS_ASTRO_FRAME_SENSOR_MODEL",
         badState));;
}

TEST_F(FrameIsdTest, ConstructFromISD) {
   UsgsAstroFramePlugin testPlugin;
   EXPECT_TRUE(testPlugin.canModelBeConstructedFromISD(
               isd,
               "USGS_ASTRO_FRAME_SENSOR_MODEL"));
}

int main(int argc, char **argv) {
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
