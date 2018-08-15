#include "UsgsAstroFramePlugin.h"
#include "UsgsAstroFrameSensorModel.h"

#include <json/json.hpp>

#include <sstream>
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
         if (it.value().size() >1 ) {
            std::vector<double> v = it.value();
            for (int j=0;j < v.size(); j++) {
               std::ostringstream val;
               val << std::setprecision(15) << v[j];
               isd.addParam(it.key(),val.str());
            }
         }
         else {
            isd.addParam(it.key(), it.value().dump());
         }
      }
      isdFile.close();
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

TEST_F(FrameIsdTest, Constructible) {
   UsgsAstroFramePlugin testPlugin;
   EXPECT_TRUE(testPlugin.canModelBeConstructedFromISD(
               isd,
               "USGS_ASTRO_FRAME_SENSOR_MODEL"));
}

TEST_F(FrameIsdTest, ConstructValidCamera) {
   UsgsAstroFramePlugin testPlugin;
   csm::Model *cameraModel = NULL;
   EXPECT_NO_THROW(
         cameraModel = testPlugin.constructModelFromISD(
               isd,
               "USGS_ASTRO_FRAME_SENSOR_MODEL",
               NULL)
   );
   UsgsAstroFrameSensorModel *frameModel = dynamic_cast<UsgsAstroFrameSensorModel *>(cameraModel);
   EXPECT_FALSE(frameModel == NULL);
   if (cameraModel) {
      delete cameraModel;
   }
}

TEST_F(FrameIsdTest, ConstructInValidCamera) {
   UsgsAstroFramePlugin testPlugin;
   // Remove the model_name keyword from the ISD to make it invalid
   isd.clearParams("model_name");
   csm::Model *cameraModel = NULL;
   EXPECT_THROW(
         testPlugin.constructModelFromISD(
               isd,
               "USGS_ASTRO_FRAME_SENSOR_MODEL",
               NULL),
         csm::Error
   );
   if (cameraModel) {
      delete cameraModel;
   }
}

int main(int argc, char **argv) {
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
