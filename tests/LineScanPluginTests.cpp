#include <sstream>
#include <fstream>

#include <gtest/gtest.h>

#include "Fixtures.h"

TEST(LineScanPluginTests, PluginName) {
   UsgsAstroLsPlugin testPlugin;
   EXPECT_EQ("USGS_ASTRO_LINE_SCANNER_PLUGIN", testPlugin.getPluginName());;
}

TEST(LineScanPluginTests, ManufacturerName) {
   UsgsAstroLsPlugin testPlugin;
   EXPECT_EQ("BAE_SYSTEMS_GXP", testPlugin.getManufacturer());;
}

TEST(LineScanPluginTests, ReleaseDate) {
   UsgsAstroLsPlugin testPlugin;
   EXPECT_EQ("20171230", testPlugin.getReleaseDate());;
}

TEST(LineScanPluginTests, NumModels) {
   UsgsAstroLsPlugin testPlugin;
   EXPECT_EQ(1, testPlugin.getNumModels());;
}

TEST_F(ConstVelLineScanIsdTest, Constructible) {
   UsgsAstroLsPlugin testPlugin;
   EXPECT_TRUE(testPlugin.canModelBeConstructedFromISD(
               isd,
               "USGS_ASTRO_LINE_SCANNER_SENSOR_MODEL"));
}

TEST_F(ConstVelLineScanIsdTest, ConstructValidCamera) {
   UsgsAstroLsPlugin testPlugin;
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
   UsgsAstroLsPlugin testPlugin;
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
