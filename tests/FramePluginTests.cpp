#include "UsgsAstroPlugin.h"
#include "UsgsAstroFrameSensorModel.h"

#include <sstream>
#include <fstream>

#include <gtest/gtest.h>

#include "Fixtures.h"

TEST(FramePluginTests, PluginName) {
   UsgsAstroPlugin testPlugin;
   EXPECT_EQ("UsgsAstroPluginCSM", testPlugin.getPluginName());;
}

TEST(FramePluginTests, ManufacturerName) {
   UsgsAstroPlugin testPlugin;
   EXPECT_EQ("UsgsAstrogeology", testPlugin.getManufacturer());;
}

TEST(FramePluginTests, ReleaseDate) {
   UsgsAstroPlugin testPlugin;
   EXPECT_EQ("20170425", testPlugin.getReleaseDate());;
}

TEST(FramePluginTests, NumModels) {
   UsgsAstroPlugin testPlugin;
   EXPECT_EQ(2, testPlugin.getNumModels());;
}

TEST(FramePluginTests, NoStateName) {
   UsgsAstroPlugin testPlugin;
   std::string badState = "{\"not_a_name\":\"bad_name\"}";
   EXPECT_FALSE(testPlugin.canModelBeConstructedFromState(
         "USGS_ASTRO_FRAME_SENSOR_MODEL",
         badState));;
}

TEST(FramePluginTests, BadStateName) {
   UsgsAstroPlugin testPlugin;
   std::string badState = "{\"m_model_name\":\"bad_name\"}";
   EXPECT_FALSE(testPlugin.canModelBeConstructedFromState(
         "USGS_ASTRO_FRAME_SENSOR_MODEL",
         badState));;
}

TEST(FramePluginTests, BadStateValue) {
   UsgsAstroPlugin testPlugin;
   std::string badState = "{"
         "\"m_model_name\":\"USGS_ASTRO_FRAME_SENSOR_MODEL\","
         "\"bad_param\":\"bad_value\"}";
   EXPECT_FALSE(testPlugin.canModelBeConstructedFromState(
         "USGS_ASTRO_FRAME_SENSOR_MODEL",
         badState));;
}

TEST(FramePluginTests, MissingStateValue) {
   UsgsAstroPlugin testPlugin;
   std::string badState = "{"
         "\"m_model_name\":\"USGS_ASTRO_FRAME_SENSOR_MODEL\"}";
   EXPECT_FALSE(testPlugin.canModelBeConstructedFromState(
         "USGS_ASTRO_FRAME_SENSOR_MODEL",
         badState));;
}

TEST_F(FrameIsdTest, Constructible) {
   UsgsAstroPlugin testPlugin;
   EXPECT_TRUE(testPlugin.canModelBeConstructedFromISD(
               isd,
               "USGS_ASTRO_FRAME_SENSOR_MODEL"));
}

TEST_F(FrameIsdTest, NotConstructible) {
   UsgsAstroPlugin testPlugin;
   isd.setFilename("Not a file");
   EXPECT_FALSE(testPlugin.canModelBeConstructedFromISD(
                isd,
                "USGS_ASTRO_FRAME_SENSOR_MODEL"));
}


TEST_F(FrameIsdTest, ConstructValidCamera) {
   UsgsAstroPlugin testPlugin;
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


TEST_F(FrameIsdTest, ConstructInValidCamera) {
   UsgsAstroPlugin testPlugin;
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
