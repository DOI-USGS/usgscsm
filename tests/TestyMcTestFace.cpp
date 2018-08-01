#include "UsgsAstroFramePlugin.h"

#include <gtest/gtest.h>

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

TEST(FramePluginTests, BadStateName) {
   UsgsAstroFramePlugin testPlugin;
   std::string badState = "{"
         "\"model_name\":\"bad_name\"}";
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

int main(int argc, char **argv) {
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
