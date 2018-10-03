#include "UsgsAstroFramePlugin.h"
#include "UsgsAstroFrameSensorModel.h"

#include <json.hpp>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <map>
#include <vector>
#include <gtest/gtest.h>

using namespace std;
using json = nlohmann::json;


class FramerParameterizedTest : public ::testing::TestWithParam<csm::ImageCoord> {

protected:
  csm::Isd isd;

      void printIsd(csm::Isd &localIsd) {
           multimap<string,string> isdmap= localIsd.parameters();
           for (auto it = isdmap.begin(); it != isdmap.end();++it){

                      cout << it->first << " : " << it->second << endl;
           }
      }
      UsgsAstroFrameSensorModel* createModel(csm::Isd &modifiedIsd) {

        UsgsAstroFramePlugin frameCameraPlugin;
        csm::Model *model = frameCameraPlugin.constructModelFromISD(
              modifiedIsd,"USGS_ASTRO_FRAME_SENSOR_MODEL");

        UsgsAstroFrameSensorModel* sensorModel = dynamic_cast<UsgsAstroFrameSensorModel *>(model);

        if (sensorModel)
          return sensorModel;
        else
          return nullptr;
      }


   virtual void SetUp() {
     isd.setFilename("data/simpleFramerISD.img");
   };
};


class FrameIsdTest : public ::testing::Test {
  protected:
    csm::Isd isd;
    void printIsd(csm::Isd &localIsd) {
      multimap<string,string> isdmap= localIsd.parameters();
      for (auto it = isdmap.begin(); it != isdmap.end();++it){
        cout << it->first << " : " << it->second << endl;
      }
    }
    UsgsAstroFrameSensorModel* createModel(csm::Isd &modifiedIsd) {
      UsgsAstroFramePlugin frameCameraPlugin;
      csm::Model *model = frameCameraPlugin.constructModelFromISD(
              modifiedIsd,"USGS_ASTRO_FRAME_SENSOR_MODEL");
      UsgsAstroFrameSensorModel* sensorModel = dynamic_cast<UsgsAstroFrameSensorModel *>(model);
      if (sensorModel)
        return sensorModel;
      else
        return nullptr;
      }


    virtual void SetUp() {
      isd.setFilename("data/simpleFramerISD.img");
   }
};

class FrameSensorModel : public ::testing::Test {
   protected:
      protected :
      csm::Isd isd;
      UsgsAstroFrameSensorModel *sensorModel;

      void SetUp() override {
         sensorModel = NULL;

         isd.setFilename("data/simpleFramerISD.img");
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

INSTANTIATE_TEST_CASE_P(JacobianTest,FramerParameterizedTest,
                        ::testing::Values(csm::ImageCoord(2.5,2.5),csm::ImageCoord(7.5,7.5)));

TEST_P(FramerParameterizedTest,JacobianTest) {

   UsgsAstroFrameSensorModel* sensorModel = createModel(isd);
   std::string modelState = sensorModel->getModelState(); 
   auto state = json::parse(modelState);

   state["m_odtX"] = {1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0};
   state["m_odtY"] = {0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0};
   sensorModel->replaceModelState(state.dump()); 
   
   double Jxx,Jxy,Jyx,Jyy;
   ASSERT_NE(sensorModel, nullptr);

   csm::ImageCoord imagePt1 = GetParam();
   cout << "[" << imagePt1.samp << "," << imagePt1.line << "]"<< endl;
   sensorModel->distortionJacobian(imagePt1.samp, imagePt1.line, Jxx, Jxy,Jyx,Jyy);

   double determinant = fabs(Jxx*Jyy - Jxy*Jyx);
   EXPECT_GT(determinant,1e-3);

   delete sensorModel;
   sensorModel=NULL;
}

// NOTE: The imagePt format is (Lines,Samples)

// centered and slightly off-center:
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

// Test all four corners:
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



TEST_F(FrameIsdTest, setFocalPlane1) {
  csm::ImageCoord imagePt(7.5, 7.5);
  double ux,uy;

  UsgsAstroFrameSensorModel* sensorModel = createModel(isd);

  std::string modelState = sensorModel->getModelState(); 
  auto state = json::parse(modelState);

  state["m_odtX"] = {0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
  state["m_odtY"] = {0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
  sensorModel->replaceModelState(state.dump()); 

  ASSERT_NE(sensorModel, nullptr);
  sensorModel->setFocalPlane(imagePt.samp, imagePt.line, ux, uy);
  EXPECT_NEAR(imagePt.samp,7.5,1e-8 );
  EXPECT_NEAR(imagePt.line,7.5,1e-8);   
  delete sensorModel;
  sensorModel = NULL;
}


TEST_F(FrameIsdTest, Jacobian1) {
  csm::ImageCoord imagePt(7.5, 7.5);

  UsgsAstroFrameSensorModel* sensorModel = createModel(isd);
  std::string modelState = sensorModel->getModelState(); 
  auto state = json::parse(modelState);
  state["m_odtX"] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0};
  state["m_odtY"] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,1.0};
  sensorModel->replaceModelState(state.dump()); 

  double Jxx,Jxy,Jyx,Jyy;

  ASSERT_NE(sensorModel, nullptr);
  sensorModel->distortionJacobian(imagePt.samp, imagePt.line, Jxx, Jxy,Jyx,Jyy);

  EXPECT_NEAR(Jxx,56.25,1e-8 );
  EXPECT_NEAR(Jxy,112.5,1e-8);
  EXPECT_NEAR(Jyx,56.25,1e-8);
  EXPECT_NEAR(Jyy,281.25,1e-8);
  
  delete sensorModel;
  sensorModel = NULL;
}


TEST_F(FrameIsdTest, distortMe_AllCoefficientsOne) {
  csm::ImageCoord imagePt(7.5, 7.5);

  UsgsAstroFrameSensorModel* sensorModel = createModel(isd);
  std::string modelState = sensorModel->getModelState(); 
  auto state = json::parse(modelState);
  state["m_odtX"] = {1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0};
  state["m_odtY"] = {1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0};
  sensorModel->replaceModelState(state.dump()); 

  double dx,dy;
  ASSERT_NE(sensorModel, nullptr);
  sensorModel->distortionFunction(imagePt.samp, imagePt.line,dx,dy );
  
  EXPECT_NEAR(dx,1872.25,1e-8 );
  EXPECT_NEAR(dy,1872.25,1e-8);
  
  delete sensorModel;
  sensorModel = NULL;
}

TEST_F(FrameIsdTest, setFocalPlane_AllCoefficientsOne) {
  csm::ImageCoord imagePt(1872.25, 1872.25);

  UsgsAstroFrameSensorModel* sensorModel = createModel(isd);
  std::string modelState = sensorModel->getModelState(); 
  auto state = json::parse(modelState);
  state["m_odtX"] = {1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0};
  state["m_odtY"] = {1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0};
  sensorModel->replaceModelState(state.dump()); 

  double ux,uy;
  ASSERT_NE(sensorModel, nullptr);
  sensorModel->setFocalPlane(imagePt.samp, imagePt.line,ux,uy );

  // The Jacobian is singular, so the setFocalPlane should break out of it's iteration and
  // returns the same distorted coordinates that were passed in.
  EXPECT_NEAR(ux,imagePt.samp,1e-8 );
  EXPECT_NEAR(uy,imagePt.line,1e-8);

  delete sensorModel;
  sensorModel = NULL;
}


TEST_F(FrameIsdTest, distortMe_AlternatingOnes) {
  csm::ImageCoord imagePt(7.5, 7.5);

  UsgsAstroFrameSensorModel* sensorModel = createModel(isd);
  std::string modelState = sensorModel->getModelState(); 
  auto state = json::parse(modelState);
  state["m_odtX"] = {1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0};
  state["m_odtY"] = {0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0};
  sensorModel->replaceModelState(state.dump()); 

  double dx,dy;
  ASSERT_NE(sensorModel, nullptr);
  sensorModel->distortionFunction(imagePt.samp, imagePt.line,dx,dy );
  
  EXPECT_NEAR(dx,908.5,1e-8 );
  EXPECT_NEAR(dy,963.75,1e-8);
  
  delete sensorModel;
  sensorModel = NULL;
}


TEST_F(FrameIsdTest, setFocalPlane_AlternatingOnes) {
  csm::ImageCoord imagePt(963.75, 908.5);

  UsgsAstroFrameSensorModel* sensorModel = createModel(isd);
  std::string modelState = sensorModel->getModelState(); 
  auto state = json::parse(modelState);
  state["m_odtX"] = {1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0};
  state["m_odtY"] = {0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0};
  sensorModel->replaceModelState(state.dump()); 

  double ux,uy;
  ASSERT_NE(sensorModel, nullptr);

  sensorModel->setFocalPlane(imagePt.samp, imagePt.line,ux,uy );

  EXPECT_NEAR(ux,7.5,1e-8 );
  EXPECT_NEAR(uy,7.5,1e-8);

  delete sensorModel;
  sensorModel = NULL;
}



// Focal Length Tests:
TEST_F(FrameIsdTest, FL500_OffBody4) {
  std::string key = "m_focalLength";
  double newValue = 500.0;

  UsgsAstroFrameSensorModel* sensorModel = createModel(isd);
  std::string modelState = sensorModel->getModelState(); 
  auto state = json::parse(modelState);
  state[key] = newValue; 
  sensorModel->replaceModelState(state.dump()); 
  
  ASSERT_NE(sensorModel, nullptr);
  csm::ImageCoord imagePt(15.0, 15.0);
  csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
  EXPECT_NEAR(groundPt.x, 9.77688917, 1e-8);
  EXPECT_NEAR(groundPt.y, -1.48533467, 1e-8);
  EXPECT_NEAR(groundPt.z, -1.48533467, 1e-8);
  
  delete sensorModel;
  sensorModel = NULL;
}


TEST_F(FrameIsdTest, FL500_OffBody3) {
  std::string key = "m_focalLength";
  double newValue = 500.0;

  UsgsAstroFrameSensorModel* sensorModel = createModel(isd);
  std::string modelState = sensorModel->getModelState(); 
  auto state = json::parse(modelState);
  state[key] = newValue; 
  sensorModel->replaceModelState(state.dump()); 
  
  ASSERT_NE(sensorModel, nullptr);
  csm::ImageCoord imagePt(0.0, 0.0);
  csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
  EXPECT_NEAR(groundPt.x, 9.77688917, 1e-8);
  EXPECT_NEAR(groundPt.y, 1.48533467, 1e-8);
  EXPECT_NEAR(groundPt.z, 1.48533467, 1e-8);
  
  delete sensorModel;
  sensorModel = NULL;
}


TEST_F(FrameIsdTest, FL500_Center) {
  std::string key = "m_focalLength";
  double newValue = 500.0;

  UsgsAstroFrameSensorModel* sensorModel = createModel(isd);
  std::string modelState = sensorModel->getModelState(); 
  auto state = json::parse(modelState);
  state[key] = newValue; 
  sensorModel->replaceModelState(state.dump()); 

  ASSERT_NE(sensorModel, nullptr);
  csm::ImageCoord imagePt(7.5, 7.5);
  csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
  EXPECT_NEAR(groundPt.x, 10.0, 1e-8);
  EXPECT_NEAR(groundPt.y, 0.0, 1e-8);
  EXPECT_NEAR(groundPt.z, 0.0, 1e-8);
  
  delete sensorModel;
  sensorModel = NULL;
}


TEST_F(FrameIsdTest, FL500_SlightlyOffCenter) {
  std::string key = "m_focalLength";
  double newValue = 500.0;

  UsgsAstroFrameSensorModel* sensorModel = createModel(isd);
  std::string modelState = sensorModel->getModelState(); 
  auto state = json::parse(modelState);
  state[key] = newValue; 
  sensorModel->replaceModelState(state.dump()); 

  ASSERT_NE(sensorModel, nullptr);
  csm::ImageCoord imagePt(7.5, 6.5);
  csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
  EXPECT_NEAR(groundPt.x, 9.99803960, 1e-8);
  EXPECT_NEAR(groundPt.y, 0.0, 1e-8);
  EXPECT_NEAR(groundPt.z, 1.98000392e-01, 1e-8);

  delete sensorModel;
  sensorModel = NULL;
}

// Observer x position:
TEST_F(FrameIsdTest, X10_SlightlyOffCenter) {
   double newValue = 10.0;
   UsgsAstroFrameSensorModel* sensorModel = createModel(isd);
   sensorModel->setParameterValue(0, newValue); // X

   ASSERT_NE(sensorModel, nullptr);
   csm::ImageCoord imagePt(7.5, 6.5);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   EXPECT_NEAR(groundPt.x, 10.0, 1e-8);
   EXPECT_NEAR(groundPt.y, 0.0, 1e-8);
   EXPECT_NEAR(groundPt.z, 0.0, 1e-8);

   delete sensorModel;
   sensorModel = NULL;
}


TEST_F(FrameIsdTest, X1e9_SlightlyOffCenter) {
   double newValue = 1000000000.0;

   UsgsAstroFrameSensorModel* sensorModel = createModel(isd);
   sensorModel->setParameterValue(0, newValue); // X

   ASSERT_NE(sensorModel, nullptr);
   csm::ImageCoord imagePt(7.5, 6.5);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   // Note: In the following, the tolerance was increased due to the very large distance being tested (~6.68 AU).
   EXPECT_NEAR(groundPt.x, 3.99998400e+03, 1e-4);
   EXPECT_NEAR(groundPt.y, 0.0, 1e-4);
   EXPECT_NEAR(groundPt.z, 1.99999200e+06, 1e-4);

   delete sensorModel;
   sensorModel = NULL;
}


// Angle rotations:
TEST_F(FrameIsdTest, Rotation_omegaPi_Center) {
   UsgsAstroFrameSensorModel* sensorModel = createModel(isd);
   sensorModel->setParameterValue(3, M_PI); // omega

   ASSERT_NE(sensorModel, nullptr);
   csm::ImageCoord imagePt(7.5, 7.5);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   EXPECT_NEAR(groundPt.x, -10.0, 1e-8);
   EXPECT_NEAR(groundPt.y, 0.0, 1e-8);
   EXPECT_NEAR(groundPt.z, 0.0, 1e-8);

   delete sensorModel;
   sensorModel = NULL;
}


TEST_F(FrameIsdTest, Rotation_NPole_Center) {
   UsgsAstroFrameSensorModel* sensorModel = createModel(isd);
   sensorModel->setParameterValue(4, -M_PI); // phi
   sensorModel->setParameterValue(0, 0.0); // X
   sensorModel->setParameterValue(1, 0.0); // Y 
   sensorModel->setParameterValue(2, 1000.0); // Z

   ASSERT_NE(sensorModel, nullptr);
   csm::ImageCoord imagePt(7.5, 7.5);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   EXPECT_NEAR(groundPt.x, 0.0, 1e-8);
   EXPECT_NEAR(groundPt.y, 0.0, 1e-8);
   EXPECT_NEAR(groundPt.z, 10.0, 1e-8);

   delete sensorModel;
   sensorModel = NULL;
}


TEST_F(FrameIsdTest, Rotation_SPole_Center) {
   UsgsAstroFrameSensorModel* sensorModel = createModel(isd);
   sensorModel->setParameterValue(4, 0.0); // phi
   sensorModel->setParameterValue(0, 0.0); // X
   sensorModel->setParameterValue(1, 0.0); // Y 
   sensorModel->setParameterValue(2, -1000.0); // Z

   ASSERT_NE(sensorModel, nullptr);
   csm::ImageCoord imagePt(7.5, 7.5);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   EXPECT_NEAR(groundPt.x, 0.0, 1e-8);
   EXPECT_NEAR(groundPt.y, 0.0, 1e-8);
   EXPECT_NEAR(groundPt.z, -10.0, 1e-8);

   delete sensorModel;
   sensorModel = NULL;
}


// Ellipsoid axis tests:
TEST_F(FrameIsdTest, SemiMajorAxis100x_Center) {
  std::string key = "m_majorAxis";
  double newValue = 1000.0;

  UsgsAstroFrameSensorModel* sensorModel = createModel(isd);
  std::string modelState = sensorModel->getModelState(); 
  auto state = json::parse(modelState);
  state[key] = newValue; 
  sensorModel->replaceModelState(state.dump()); 

  ASSERT_NE(sensorModel, nullptr);
  csm::ImageCoord imagePt(7.5, 7.5);
  csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
  EXPECT_NEAR(groundPt.x, 1000.0, 1e-8);
  EXPECT_NEAR(groundPt.y, 0.0, 1e-8);
  EXPECT_NEAR(groundPt.z, 0.0, 1e-8);

  delete sensorModel;
  sensorModel = NULL;
}


TEST_F(FrameIsdTest, SemiMajorAxis10x_SlightlyOffCenter) {
  std::string key = "m_majorAxis";
  double newValue = 100.0;

  UsgsAstroFrameSensorModel* sensorModel = createModel(isd);
  std::string modelState = sensorModel->getModelState(); 
  auto state = json::parse(modelState);
  state[key] = newValue; 
  sensorModel->replaceModelState(state.dump()); 

  ASSERT_NE(sensorModel, nullptr);
  csm::ImageCoord imagePt(7.5, 6.5);
  csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
  // Note: In the following, the tolerance was increased due to the combination of an offset image point and
  //       a very large deviation from sphericity.
  EXPECT_NEAR(groundPt.x, 9.83606557e+01, 1e-7);
  EXPECT_NEAR(groundPt.y, 0.0, 1e-7);
  EXPECT_NEAR(groundPt.z, 1.80327869, 1e-7);
  
  delete sensorModel;
  sensorModel = NULL;
}


// The following test is for the scenario where the semi_minor_axis is actually larger
// than the semi_major_axis:
TEST_F(FrameIsdTest, SemiMinorAxis10x_SlightlyOffCenter) {
   std::string key = "m_minorAxis";
   double newValue = 100.0;

   UsgsAstroFrameSensorModel* sensorModel = createModel(isd);
   std::string modelState = sensorModel->getModelState(); 
   auto state = json::parse(modelState);
   state[key] = newValue;
   sensorModel->replaceModelState(state.dump()); 

   ASSERT_NE(sensorModel, nullptr);
   csm::ImageCoord imagePt(7.5, 6.5);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   EXPECT_NEAR(groundPt.x, 9.99803960, 1e-8);
   EXPECT_NEAR(groundPt.y, 0.0, 1e-8);
   EXPECT_NEAR(groundPt.z, 1.98000392, 1e-8);

   delete sensorModel;
   sensorModel = NULL;
}
