#include "UsgsAstroPlugin.h"
#include "UsgsAstroFrameSensorModel.h"
#include "UsgsAstroLsSensorModel.h"

#include <sstream>
#include <fstream>

#include <gtest/gtest.h>

#include "Fixtures.h"

TEST(PluginTests, PluginName) {
   UsgsAstroPlugin testPlugin;
   EXPECT_EQ("UsgsAstroPluginCSM", testPlugin.getPluginName());
}

TEST(PluginTests, ManufacturerName) {
   UsgsAstroPlugin testPlugin;
   EXPECT_EQ("UsgsAstrogeology", testPlugin.getManufacturer());
}

TEST(PluginTests, ReleaseDate) {
   UsgsAstroPlugin testPlugin;
   EXPECT_EQ("20190222", testPlugin.getReleaseDate());
}

TEST(PluginTests, NumModels) {
   UsgsAstroPlugin testPlugin;
   EXPECT_EQ(3, testPlugin.getNumModels());
}

TEST(PluginTests, BadISDFile) {
   UsgsAstroPlugin testPlugin;
   csm::Isd badIsd("Not a file");
   EXPECT_FALSE(testPlugin.canModelBeConstructedFromISD(
                badIsd,
                "USGS_ASTRO_FRAME_SENSOR_MODEL"));
   EXPECT_FALSE(testPlugin.canModelBeConstructedFromISD(
                badIsd,
                "USGS_ASTRO_LINE_SCANNER_SENSOR_MODEL"));
}

TEST(FrameStateTests, NoStateName) {
   UsgsAstroPlugin testPlugin;
   std::string badState = "{\"not_a_name\":\"bad_name\"}";
   EXPECT_FALSE(testPlugin.canModelBeConstructedFromState(
         "USGS_ASTRO_FRAME_SENSOR_MODEL",
         badState));
}

TEST(FrameStateTests, BadStateName) {
   UsgsAstroPlugin testPlugin;
   std::string badState = "{\"m_model_name\":\"bad_name\"}";
   EXPECT_FALSE(testPlugin.canModelBeConstructedFromState(
         "USGS_ASTRO_FRAME_SENSOR_MODEL",
         badState));
}

TEST(FrameStateTests, BadStateValue) {
   UsgsAstroPlugin testPlugin;
   std::string badState = "{"
         "\"m_model_name\":\"USGS_ASTRO_FRAME_SENSOR_MODEL\","
         "\"bad_param\":\"bad_value\"}";
   EXPECT_FALSE(testPlugin.canModelBeConstructedFromState(
         "USGS_ASTRO_FRAME_SENSOR_MODEL",
         badState));
}

TEST(FrameStateTests, MissingStateValue) {
   UsgsAstroPlugin testPlugin;
   std::string badState = "{"
         "\"m_model_name\":\"USGS_ASTRO_FRAME_SENSOR_MODEL\"}";
   EXPECT_FALSE(testPlugin.canModelBeConstructedFromState(
         "USGS_ASTRO_FRAME_SENSOR_MODEL",
         badState));
}

TEST_F(FrameIsdTest, Constructible) {
   UsgsAstroPlugin testPlugin;
   EXPECT_TRUE(testPlugin.canModelBeConstructedFromISD(
               isd,
               "USGS_ASTRO_FRAME_SENSOR_MODEL"));
}

TEST_F(FrameIsdTest, ConstructibleFromState) {
   UsgsAstroPlugin testPlugin;
   std::string modelState = testPlugin.getStateFromISD(isd);
   EXPECT_TRUE(testPlugin.canModelBeConstructedFromState(
        "USGS_ASTRO_FRAME_SENSOR_MODEL",
        modelState));
}

TEST_F(FrameIsdTest, NotConstructible) {
   UsgsAstroPlugin testPlugin;
   isd.setFilename("data/constVelocityLineScan.img");
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
   isd.setFilename("data/empty.img");
   csm::Model *cameraModel = NULL;
   try {
      testPlugin.constructModelFromISD(
            isd,
            "USGS_ASTRO_FRAME_SENSOR_MODEL",
            nullptr);
      FAIL() << "Expected an error";
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

TEST_F(ConstVelLineScanIsdTest, ConstructibleFromState) {
   UsgsAstroPlugin testPlugin;
   std::string modelState = testPlugin.getStateFromISD(isd);
   EXPECT_TRUE(testPlugin.canModelBeConstructedFromState(
         "USGS_ASTRO_LINE_SCANNER_SENSOR_MODEL",
         modelState));
}

TEST_F(ConstVelLineScanIsdTest, NotConstructible) {
   UsgsAstroPlugin testPlugin;
   isd.setFilename("data/simpleFramerISD.img");
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
   isd.setFilename("data/empty.img");
   csm::Model *cameraModel = NULL;
   try {
      testPlugin.constructModelFromISD(
            isd,
            "USGS_ASTRO_LINE_SCANNER_SENSOR_MODEL",
            nullptr);
      FAIL() << "Expected an error";

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

TEST_F(SarIsdTest, Constructible) {
  UsgsAstroPlugin testPlugin;
  csm::WarningList warnings;
  EXPECT_TRUE(testPlugin.canModelBeConstructedFromISD(
              isd,
              "USGS_ASTRO_SAR_SENSOR_MODEL",
              &warnings));
  for (auto &warn: warnings) {
    std::cerr << "Warning in " << warn.getFunction() << std::endl;
    std::cerr << "  " << warn.getMessage() << std::endl;
  }
}

TEST_F(SarIsdTest, ConstructibleFromState) {
   UsgsAstroPlugin testPlugin;
   csm::WarningList warnings;
   std::string modelState = testPlugin.getStateFromISD(isd);
   EXPECT_TRUE(testPlugin.canModelBeConstructedFromState(
         "USGS_ASTRO_SAR_SENSOR_MODEL",
         modelState,
         &warnings));
  for (auto &warn: warnings) {
    std::cerr << "Warning in " << warn.getFunction() << std::endl;
    std::cerr << "  " << warn.getMessage() << std::endl;
  }
}

TEST_F(SarIsdTest, NotConstructible) {
   UsgsAstroPlugin testPlugin;
   isd.setFilename("data/simpleFramerISD.img");
   EXPECT_FALSE(testPlugin.canModelBeConstructedFromISD(
               isd,
               "USGS_ASTRO_SAR_SENSOR_MODEL"));
}

TEST_F(SarIsdTest, ConstructValidCamera) {
   UsgsAstroPlugin testPlugin;
   csm::WarningList warnings;
   csm::Model *cameraModel = NULL;
   EXPECT_NO_THROW(
         cameraModel = testPlugin.constructModelFromISD(
               isd,
               "USGS_ASTRO_SAR_SENSOR_MODEL",
               &warnings)
   );
   for (auto &warn: warnings) {
     std::cerr << "Warning in " << warn.getFunction() << std::endl;
     std::cerr << "  " << warn.getMessage() << std::endl;
   }
   UsgsAstroSarSensorModel *sarModel = dynamic_cast<UsgsAstroSarSensorModel *>(cameraModel);
   EXPECT_NE(sarModel, nullptr);
   if (cameraModel) {
      delete cameraModel;
   }
}

TEST_F(SarIsdTest, ConstructInValidCamera) {
   UsgsAstroPlugin testPlugin;
   isd.setFilename("data/empty.img");
   csm::Model *cameraModel = NULL;
   try {
      testPlugin.constructModelFromISD(
            isd,
            "USGS_ASTRO_SAR_SENSOR_MODEL",
            nullptr);
      FAIL() << "Expected an error";

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

int main(int argc, char **argv) {
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
