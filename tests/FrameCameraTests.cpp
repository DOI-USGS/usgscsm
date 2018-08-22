#include "UsgsAstroFramePlugin.h"
#include "UsgsAstroFrameSensorModel.h"

#include <json.hpp>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <vector>
#include <gtest/gtest.h>

using namespace std;
using json = nlohmann::json;






class FrameSensorModel : public testing::Test {



      protected :
      UsgsAstroFrameSensorModel *sensorModel;


      void SetUp() override {
         sensorModel = NULL;


          vector<double> odtx{0.0,1.0020558791381275,0.0,0.0,-0.0005448742222712919,0.0,0.000006597498811862692,0.0,0.000006683129056014682,0.0};
          vector<double> odty{-0.000013657975359540047,0.0,1.0,0.000885544334965699,0.0,0.0003338939138331483,0.0,0.00000774756721313425,0.0,0.00000779484564042716};

         std::ifstream isdFile("data/simpleFramerISD.json");

         json jsonIsd = json::parse(isdFile);

         csm::Isd isd;
         for (json::iterator it = jsonIsd.begin(); it != jsonIsd.end(); ++it) {
            json jsonValue = it.value();
            if (jsonValue.size() > 1) {
               for (int i = 0; i < jsonValue.size(); i++) {                              
                 if (it.key() == "odt_x") {
                    ostringstream strval;
                    strval << setprecision(20) << odtx[i];
                    isd.addParam(it.key(), strval.str());
                  }
                  else if(it.key() == "odt_y") {
                   ostringstream strval;
                   strval << setprecision(20) << odty[i];
                   isd.addParam(it.key(), strval.str());
                  }
                  else
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
   csm::ImageCoord imagePt(7.0, 7.0);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   EXPECT_NEAR(groundPt.x, 10.0, 1e-8);
   EXPECT_NEAR(groundPt.y, 0, 1e-8);
   EXPECT_NEAR(groundPt.z, 0, 1e-8);
}
TEST_F(FrameSensorModel, SlightlyOffCenter) {
   csm::ImageCoord imagePt(7.0, 6.0);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   EXPECT_NEAR(groundPt.x, 9.80194018, 1e-8);
   EXPECT_NEAR(groundPt.y, 0, 1e-8);
   EXPECT_NEAR(groundPt.z, 1.98039612, 1e-8);
}

//Test all four corners:
TEST_F(FrameSensorModel, OffBody1) {
   csm::ImageCoord imagePt(14.5, -0.5);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   EXPECT_NEAR(groundPt.x, 0.44979759, 1e-8);
   EXPECT_NEAR(groundPt.y, -14.99325304, 1e-8);
   EXPECT_NEAR(groundPt.z, 14.99325304, 1e-8);
}
TEST_F(FrameSensorModel, OffBody2) {
   csm::ImageCoord imagePt(-0.5, 14.5);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   EXPECT_NEAR(groundPt.x, 0.44979759, 1e-8);
   EXPECT_NEAR(groundPt.y, 14.99325304, 1e-8);
   EXPECT_NEAR(groundPt.z, -14.99325304, 1e-8);
}
TEST_F(FrameSensorModel, OffBody3) {
   csm::ImageCoord imagePt(-0.5, -0.5);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   EXPECT_NEAR(groundPt.x, 0.44979759, 1e-8);
   EXPECT_NEAR(groundPt.y, 14.99325304, 1e-8);
   EXPECT_NEAR(groundPt.z, 14.99325304, 1e-8);
}
TEST_F(FrameSensorModel, OffBody4) {
   csm::ImageCoord imagePt(14.5, 14.5);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   EXPECT_NEAR(groundPt.x, 0.44979759, 1e-8);
   EXPECT_NEAR(groundPt.y, -14.99325304, 1e-8);
   EXPECT_NEAR(groundPt.z, -14.99325304, 1e-8);
}

TEST_F(FrameSensorModel, setFocalPlane) {

   csm::ImageCoord imagePt(6.99224757243428, 7.065069863824555);
   double ux,uy;
    sensorModel->setFocalPlane(imagePt.samp, imagePt.line, ux, uy);
   EXPECT_NEAR(ux,7.0,1e-2 );
   EXPECT_NEAR(uy,7.0,1e-2);

}

