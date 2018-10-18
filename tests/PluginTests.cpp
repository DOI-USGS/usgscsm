#include "UsgsAstroPlugin.h"
#include "UsgsAstroFrameSensorModel.h"
#include "UsgsAstroLsSensorModel.h"

#include <sstream>
#include <fstream>

#include <gtest/gtest.h>

#include "Fixtures.h"

TEST(PluginTests, PluginName) {
   UsgsAstroPlugin testPlugin;
   EXPECT_EQ("UsgsAstroPluginCSM", testPlugin.getPluginName());;
}

TEST(PluginTests, ManufacturerName) {
   UsgsAstroPlugin testPlugin;
   EXPECT_EQ("UsgsAstrogeology", testPlugin.getManufacturer());;
}

TEST(PluginTests, ReleaseDate) {
   UsgsAstroPlugin testPlugin;
   EXPECT_EQ("20170425", testPlugin.getReleaseDate());;
}

TEST(PluginTests, NumModels) {
   UsgsAstroPlugin testPlugin;
   EXPECT_EQ(2, testPlugin.getNumModels());;
}

TEST(FrameStateTests, NoStateName) {
   UsgsAstroPlugin testPlugin;
   std::string badState = "{\"not_a_name\":\"bad_name\"}";
   EXPECT_FALSE(testPlugin.canModelBeConstructedFromState(
         "USGS_ASTRO_FRAME_SENSOR_MODEL",
         badState));;
}

TEST(FrameStateTests, BadStateName) {
   UsgsAstroPlugin testPlugin;
   std::string badState = "{\"m_model_name\":\"bad_name\"}";
   EXPECT_FALSE(testPlugin.canModelBeConstructedFromState(
         "USGS_ASTRO_FRAME_SENSOR_MODEL",
         badState));;
}

TEST(FrameStateTests, BadStateValue) {
   UsgsAstroPlugin testPlugin;
   std::string badState = "{"
         "\"m_model_name\":\"USGS_ASTRO_FRAME_SENSOR_MODEL\","
         "\"bad_param\":\"bad_value\"}";
   EXPECT_FALSE(testPlugin.canModelBeConstructedFromState(
         "USGS_ASTRO_FRAME_SENSOR_MODEL",
         badState));;
}

TEST(FrameStateTests, MissingStateValue) {
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
   isd.setFilename("data/constVelocityLineScan.img");
   csm::Model *cameraModel = NULL;
   try {
      testPlugin.constructModelFromISD(
            isd,
            "USGS_ASTRO_FRAME_SENSOR_MODEL",
            NULL);
      FAIL() << "Expected csm SENSOR_MODEL_NOT_CONSTRUCTIBLE error";
   }
   catch(csm::Error &e) {
      EXPECT_EQ(e.getError(), csm::Error::SENSOR_MODEL_NOT_CONSTRUCTIBLE);
   }
   catch(...) {
      FAIL() << "Expected csm SENSOR_MODEL_NOT_CONSTRUCTIBLE error";
   }
   if (cameraModel) {
      delete cameraModel;
   }
}

TEST_F(ConstVelLineScanIsdTest, Constructible) {
   UsgsAstroPlugin testPlugin;
   EXPECT_TRUE(testPlugin.canModelBeConstructedFromISD(
               isd,
               "USGS_ASTRO_LINE_SCANNER_SENSOR_MODEL"));
}

TEST_F(ConstVelLineScanIsdTest, NotConstructible) {
   UsgsAstroPlugin testPlugin;
   isd.setFilename("Not a file");
   EXPECT_FALSE(testPlugin.canModelBeConstructedFromISD(
               isd,
               "USGS_ASTRO_LINE_SCANNER_SENSOR_MODEL"));
}

TEST_F(ConstVelLineScanIsdTest, ConstructValidCamera) {
   UsgsAstroPlugin testPlugin;
   csm::Model *cameraModel = NULL;
   EXPECT_NO_THROW(
         cameraModel = testPlugin.constructModelFromISD(
               isd,
               "USGS_ASTRO_LINE_SCANNER_SENSOR_MODEL",
               NULL)
   );
   UsgsAstroLsSensorModel *frameModel = dynamic_cast<UsgsAstroLsSensorModel *>(cameraModel);
   EXPECT_NE(frameModel, nullptr);
   if (cameraModel) {
      delete cameraModel;
   }
}

TEST_F(ConstVelLineScanIsdTest, ConstructInValidCamera) {
   UsgsAstroPlugin testPlugin;
   // Remove the model_name keyword from the ISD to make it invalid
   isd.clearAllParams();
   csm::Model *cameraModel = NULL;
   try {
      testPlugin.constructModelFromISD(
            isd,
            "USGS_ASTRO_LINE_SCANNER_SENSOR_MODEL",
            NULL);
      FAIL() << "Expected csm ISD_NOT_SUPPORTED error";

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
