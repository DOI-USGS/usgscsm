#include "UsgsAstroFramePlugin.h"
#include "UsgsAstroFrameSensorModel.h"

#include <sstream>
#include <fstream>

#include <gtest/gtest.h>

#include "Fixtures.h"

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

TEST_F(SimpleFrameIsdTest, Constructible) {
   UsgsAstroFramePlugin testPlugin;
   EXPECT_TRUE(testPlugin.canModelBeConstructedFromISD(
               isd,
               "USGS_ASTRO_FRAME_SENSOR_MODEL"));
}

TEST_F(SimpleFrameIsdTest, NotConstructible) {
   UsgsAstroFramePlugin testPlugin;
   isd.setFilename("Not a file");
   EXPECT_FALSE(testPlugin.canModelBeConstructedFromISD(
                isd,
                "USGS_ASTRO_FRAME_SENSOR_MODEL"));
}

TEST_F(SimpleFrameIsdTest, ConstructValidCamera) {
   UsgsAstroFramePlugin testPlugin;
   csm::Model *cameraModel = NULL;
   EXPECT_NO_THROW(
         cameraModel = testPlugin.constructModelFromISD(
               isd,
               "USGS_ASTRO_FRAME_SENSOR_MODEL",
               NULL)
   );
   UsgsAstroFrameSensorModel *frameModel = dynamic_cast<UsgsAstroFrameSensorModel *>(cameraModel);
   EXPECT_NE(frameModel, nullptr);
   if (cameraModel) {
      delete cameraModel;
   }
}

TEST_F(SimpleFrameIsdTest, ConstructInValidCamera) {
   UsgsAstroFramePlugin testPlugin;
   // Remove the model_name keyword from the ISD to make it invalid
   isd.clearParams("model_name");
   csm::Model *cameraModel = NULL;
   try {
      testPlugin.constructModelFromISD(
            isd,
            "USGS_ASTRO_FRAME_SENSOR_MODEL",
            NULL);

   }
   catch(csm::Error &e) {
      EXPECT_EQ(e.getError(), csm::Error::ISD_NOT_SUPPORTED);
   }
   catch(...) {
      FAIL() << "Expected csm ISD_NOT_SUPPORTED error";
   }
   if (cameraModel) {
      delete cameraModel;
   }
}

int main(int argc, char **argv) {
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
