#include "UsgsAstroFramePlugin.h"
#include "UsgsAstroFrameSensorModel.h"

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
      for (json::iterator it = jsonIsd.begin(); it != jsonIsd.end(); ++it) {
         json jsonValue = it.value();
         if (jsonValue.size() > 1) {
            for (int i = 0; i < jsonValue.size(); i++) {
               isd.addParam(it.key(), jsonValue[i].dump());
            }
         }
         else {
            isd.addParam(it.key(), jsonValue.dump());
         }
      }
      isdFile.close();
   }
};

class FrameSensorModel : public ::testing::Test {
   protected:

      UsgsAstroFrameSensorModel *sensorModel;
      csm::Isd isd;
      void SetUp() override {
         sensorModel = NULL;
         std::ifstream isdFile("data/simpleFramerISD.json");
         json jsonIsd = json::parse(isdFile);
         //csm::Isd isd;
         for (json::iterator it = jsonIsd.begin(); it != jsonIsd.end(); ++it) {
            json jsonValue = it.value();
            if (jsonValue.size() > 1) {
               for (int i = 0; i < jsonValue.size(); i++) {
                  isd.addParam(it.key(), jsonValue[i].dump());
               }
            }
            else {
               isd.addParam(it.key(), jsonValue.dump());
            }
         }
         isdFile.close();

         UsgsAstroFramePlugin frameCameraPlugin;
         
         csm::Model *model = frameCameraPlugin.constructModelFromISD(
               isd,
               "USGS_ASTRO_FRAME_SENSOR_MODEL");
         
         sensorModel = dynamic_cast<UsgsAstroFrameSensorModel *>(model);
         
         ASSERT_NE(sensorModel, nullptr);
      }

      void TearDown() override {
         if (sensorModel) {
            delete sensorModel;
            sensorModel = NULL;
         }
      }
};

//NOTE: The imagePt layour is (Lines,Samples)
//      Also subtract 0.5 from the lines/samples. Hence Samples=0 and Lines=15 -> 14.5,-0.5

//centered and slightly off-center:
TEST_F(FrameSensorModel, Center) {
   csm::ImageCoord imagePt(7.5, 7.5);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   EXPECT_NEAR(groundPt.x, 10.0, 1e-8);
   EXPECT_NEAR(groundPt.y, 0, 1e-8);
   EXPECT_NEAR(groundPt.z, 0, 1e-8);
}

TEST_F(FrameSensorModel, SlightlyOffCenter) {
   csm::ImageCoord imagePt(7.5, 6.5);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   EXPECT_NEAR(groundPt.x, 9.80194018, 1e-8);
   EXPECT_NEAR(groundPt.y, 0, 1e-8);
   EXPECT_NEAR(groundPt.z, 1.98039612, 1e-8);
}

//Test all four corners:
TEST_F(FrameSensorModel, OffBody1) {
   csm::ImageCoord imagePt(15.0, 0.0);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   EXPECT_NEAR(groundPt.x, 0.44979759, 1e-8);
   EXPECT_NEAR(groundPt.y, -14.99325304, 1e-8);
   EXPECT_NEAR(groundPt.z, 14.99325304, 1e-8);
}
TEST_F(FrameSensorModel, OffBody2) {
   csm::ImageCoord imagePt(0.0, 15.0);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   EXPECT_NEAR(groundPt.x, 0.44979759, 1e-8);
   EXPECT_NEAR(groundPt.y, 14.99325304, 1e-8);
   EXPECT_NEAR(groundPt.z, -14.99325304, 1e-8);
}
TEST_F(FrameSensorModel, OffBody3) {
   csm::ImageCoord imagePt(0.0, 0.0);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   EXPECT_NEAR(groundPt.x, 0.44979759, 1e-8);
   EXPECT_NEAR(groundPt.y, 14.99325304, 1e-8);
   EXPECT_NEAR(groundPt.z, 14.99325304, 1e-8);
}
TEST_F(FrameSensorModel, OffBody4) {
   csm::ImageCoord imagePt(15.0, 15.0);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   EXPECT_NEAR(groundPt.x, 0.44979759, 1e-8);
   EXPECT_NEAR(groundPt.y, -14.99325304, 1e-8);
   EXPECT_NEAR(groundPt.z, -14.99325304, 1e-8);
}


// Focal Length Tests:
TEST_F(FrameIsdTest, FL500_OffBody4) {
   isd.clearParams("focal_length");
   isd.addParam("focal_length","500.0");
   UsgsAstroFramePlugin frameCameraPlugin;
         
   csm::Model *model = frameCameraPlugin.constructModelFromISD(
         isd,
         "USGS_ASTRO_FRAME_SENSOR_MODEL");
   
   UsgsAstroFrameSensorModel* sensorModel = dynamic_cast<UsgsAstroFrameSensorModel *>(model);
   
   ASSERT_NE(sensorModel, nullptr);
   csm::ImageCoord imagePt(15.0, 15.0);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   EXPECT_NEAR(groundPt.x, 9.77688917, 1e-8);
   EXPECT_NEAR(groundPt.y, -1.48533467, 1e-8);
   EXPECT_NEAR(groundPt.z, -1.48533467, 1e-8);
}
TEST_F(FrameIsdTest, FL500_OffBody3) {
   isd.clearParams("focal_length");
   isd.addParam("focal_length","500.0");
   UsgsAstroFramePlugin frameCameraPlugin;
         
   csm::Model *model = frameCameraPlugin.constructModelFromISD(
         isd,
         "USGS_ASTRO_FRAME_SENSOR_MODEL");
   
   UsgsAstroFrameSensorModel* sensorModel = dynamic_cast<UsgsAstroFrameSensorModel *>(model);
   
   ASSERT_NE(sensorModel, nullptr);
   csm::ImageCoord imagePt(0.0, 0.0);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   EXPECT_NEAR(groundPt.x, 9.77688917, 1e-8);
   EXPECT_NEAR(groundPt.y, 1.48533467, 1e-8);
   EXPECT_NEAR(groundPt.z, 1.48533467, 1e-8);
}
TEST_F(FrameIsdTest, FL500_Center) {
   isd.clearParams("focal_length");
   isd.addParam("focal_length","500.0");
   UsgsAstroFramePlugin frameCameraPlugin;
         
   csm::Model *model = frameCameraPlugin.constructModelFromISD(
         isd,
         "USGS_ASTRO_FRAME_SENSOR_MODEL");
   
   UsgsAstroFrameSensorModel* sensorModel = dynamic_cast<UsgsAstroFrameSensorModel *>(model);
   
   ASSERT_NE(sensorModel, nullptr);
   csm::ImageCoord imagePt(7.5, 7.5);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   EXPECT_NEAR(groundPt.x, 10.0, 1e-8);
   EXPECT_NEAR(groundPt.y, 0.0, 1e-8);
   EXPECT_NEAR(groundPt.z, 0.0, 1e-8);
}
TEST_F(FrameIsdTest, FL500_SlightlyOffCenter) {
   isd.clearParams("focal_length");
   isd.addParam("focal_length","500.0");
   UsgsAstroFramePlugin frameCameraPlugin;
         
   csm::Model *model = frameCameraPlugin.constructModelFromISD(
         isd,
         "USGS_ASTRO_FRAME_SENSOR_MODEL");
   
   UsgsAstroFrameSensorModel* sensorModel = dynamic_cast<UsgsAstroFrameSensorModel *>(model);
   
   ASSERT_NE(sensorModel, nullptr);
   csm::ImageCoord imagePt(7.5, 6.5);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   EXPECT_NEAR(groundPt.x, 9.99803960, 1e-8);
   EXPECT_NEAR(groundPt.y, 0.0, 1e-8);
   EXPECT_NEAR(groundPt.z, 1.98000392e-01, 1e-8);
   
}
