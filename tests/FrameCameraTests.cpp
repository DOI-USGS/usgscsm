#include "UsgsAstroPlugin.h"
#include "UsgsAstroFrameSensorModel.h"

#include <json/json.hpp>
#include <gtest/gtest.h>

#include "Fixtures.h"

using json = nlohmann::json;
// NOTE: The imagePt format is (Lines,Samples)

TEST_F(FrameSensorModel, State) {
   std::string modelState = sensorModel->getModelState();
   EXPECT_NO_THROW(
         sensorModel->replaceModelState(modelState)
   );
   EXPECT_EQ(sensorModel->getModelState(), modelState);
}

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

TEST_F(FrameSensorModel, getReferencePoint) {
  csm::ImageCoord imagePt(7.5, 7.5);
  csm::EcefCoord groundPtCenter = sensorModel->imageToGround(imagePt, 0.0);
  csm::EcefCoord groundPtRef = sensorModel->getReferencePoint();
  EXPECT_EQ(groundPtRef.x, groundPtCenter.x);
  EXPECT_EQ(groundPtRef.y, groundPtCenter.y);
  EXPECT_EQ(groundPtRef.z, groundPtCenter.z);
}

TEST_F(FrameSensorModel, OffBody4) {
   csm::ImageCoord imagePt(15.0, 15.0);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   EXPECT_NEAR(groundPt.x, 0.44979759, 1e-8);
   EXPECT_NEAR(groundPt.y, -14.99325304, 1e-8);
   EXPECT_NEAR(groundPt.z, -14.99325304, 1e-8);
}

TEST_F(FrameSensorModel, getImageIdentifier) {
  EXPECT_EQ("simpleFramerISD", sensorModel->getImageIdentifier());
}

TEST_F(FrameSensorModel, Inversion) {
   csm::ImageCoord imagePt1(9.0, 9.0);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt1, 0.0);
   csm::ImageCoord imagePt2 = sensorModel->groundToImage(groundPt);
   EXPECT_DOUBLE_EQ(imagePt1.line, imagePt2.line);
   EXPECT_DOUBLE_EQ(imagePt1.samp, imagePt2.samp);
}

TEST_F(OrbitalFrameSensorModel, Center) {
   csm::ImageCoord imagePt(8.0, 8.0);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   EXPECT_DOUBLE_EQ(groundPt.x, 1000000.0);
   EXPECT_DOUBLE_EQ(groundPt.y, 0);
   EXPECT_DOUBLE_EQ(groundPt.z, 0);
}

TEST_F(FrameSensorModel, Radii) {
   csm::Ellipsoid ellipsoid = sensorModel->getEllipsoid();
   EXPECT_DOUBLE_EQ(ellipsoid.getSemiMajorRadius(), 10);
   EXPECT_DOUBLE_EQ(ellipsoid.getSemiMinorRadius(), 10);
}

TEST_F(FrameSensorModel, SetRadii) {
   csm::Ellipsoid ellipsoid1(1000, 1500);
   sensorModel->setEllipsoid(ellipsoid1);
   csm::Ellipsoid ellipsoid2 = sensorModel->getEllipsoid();
   EXPECT_DOUBLE_EQ(ellipsoid2.getSemiMajorRadius(), 1000);
   EXPECT_DOUBLE_EQ(ellipsoid2.getSemiMinorRadius(), 1500);
}

TEST_F(OrbitalFrameSensorModel, GroundPartials) {
   csm::EcefCoord groundPt(1000000.0, 0.0, 0.0);
   std::vector<double> partials = sensorModel->computeGroundPartials(groundPt);
   // Pixels are 100m
   // lines are increasing z and samples are increasing y in body fixed
   // lines partials should be 0 except for the z partial which should be 1/100
   // sample partials should be 0 except for the y partial which should be 1/100
   EXPECT_DOUBLE_EQ(partials[0], 0.0);
   EXPECT_DOUBLE_EQ(partials[1], 0.0);
   EXPECT_DOUBLE_EQ(partials[2], 1 / 100.0);
   EXPECT_DOUBLE_EQ(partials[3], 0.0);
   EXPECT_DOUBLE_EQ(partials[4], 1 / 100.0);
   EXPECT_DOUBLE_EQ(partials[5], 0.0);
}


// Focal Length Tests:
TEST_F(FrameStateTest, FL500_OffBody4) {
  std::string key = "m_focalLength";
  double newValue = 500.0;
  UsgsAstroFrameSensorModel* sensorModel = createModifiedStateSensorModel(key, newValue);

  ASSERT_NE(sensorModel, nullptr);
  csm::ImageCoord imagePt(15.0, 15.0);
  csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
  EXPECT_NEAR(groundPt.x, 9.77688917, 1e-8);
  EXPECT_NEAR(groundPt.y, -1.48533467, 1e-8);
  EXPECT_NEAR(groundPt.z, -1.48533467, 1e-8);

  delete sensorModel;
  sensorModel = NULL;
}


TEST_F(FrameStateTest, FL500_OffBody3) {
  std::string key = "m_focalLength";
  double newValue = 500.0;
  UsgsAstroFrameSensorModel* sensorModel = createModifiedStateSensorModel(key, newValue);

  ASSERT_NE(sensorModel, nullptr);
  csm::ImageCoord imagePt(0.0, 0.0);
  csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
  EXPECT_NEAR(groundPt.x, 9.77688917, 1e-8);
  EXPECT_NEAR(groundPt.y, 1.48533467, 1e-8);
  EXPECT_NEAR(groundPt.z, 1.48533467, 1e-8);

  delete sensorModel;
  sensorModel = NULL;
}


TEST_F(FrameStateTest, FL500_Center) {
  std::string key = "m_focalLength";
  double newValue = 500.0;
  UsgsAstroFrameSensorModel* sensorModel = createModifiedStateSensorModel(key, newValue);

  ASSERT_NE(sensorModel, nullptr);
  csm::ImageCoord imagePt(7.5, 7.5);
  csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
  EXPECT_NEAR(groundPt.x, 10.0, 1e-8);
  EXPECT_NEAR(groundPt.y, 0.0, 1e-8);
  EXPECT_NEAR(groundPt.z, 0.0, 1e-8);

  delete sensorModel;
  sensorModel = NULL;
}


TEST_F(FrameStateTest, FL500_SlightlyOffCenter) {
  std::string key = "m_focalLength";
  double newValue = 500.0;
  UsgsAstroFrameSensorModel* sensorModel = createModifiedStateSensorModel(key, newValue);

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
TEST_F(FrameSensorModel, X10_SlightlyOffCenter) {
   double newValue = 10.0;
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

TEST_F(FrameSensorModel, X1e9_SlightlyOffCenter) {
   double newValue = 1000000000.0;
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
TEST_F(FrameSensorModel, Rotation_omegaPi_Center) {
   sensorModel->setParameterValue(3, 1.0);
   sensorModel->setParameterValue(4, 1.0);
   sensorModel->setParameterValue(5, 1.0);

   sensorModel->setParameterValue(6, 1.0);

   sensorModel->setParameterValue(0, 1000.0); // X
   sensorModel->setParameterValue(1, 0.0); // Y
   sensorModel->setParameterValue(2, 0.0); // Z

   ASSERT_NE(sensorModel, nullptr);
   csm::ImageCoord imagePt(7.5, 7.5);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   EXPECT_NEAR(groundPt.x, -10.0, 1e-8);
   EXPECT_NEAR(groundPt.y, 0.0, 1e-8);
   EXPECT_NEAR(groundPt.z, 0.0, 1e-8);

   delete sensorModel;
   sensorModel = NULL;
}


TEST_F(FrameSensorModel, Rotation_NPole_Center) {
  sensorModel->setParameterValue(3, 0.0);
  sensorModel->setParameterValue(4, -1.0);
  sensorModel->setParameterValue(5, 0.0);

  sensorModel->setParameterValue(6, 0.0);

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


TEST_F(FrameSensorModel, Rotation_SPole_Center) {
   sensorModel->setParameterValue(3, 0.0); // phi
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
TEST_F(FrameStateTest, SemiMajorAxis100x_Center) {
  std::string key = "m_majorAxis";
  double newValue = 1000.0;

  UsgsAstroFrameSensorModel* sensorModel = createModifiedStateSensorModel(key, newValue);

  ASSERT_NE(sensorModel, nullptr);
  csm::ImageCoord imagePt(7.5, 7.5);
  csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
  EXPECT_NEAR(groundPt.x, 1000.0, 1e-8);
  EXPECT_NEAR(groundPt.y, 0.0, 1e-8);
  EXPECT_NEAR(groundPt.z, 0.0, 1e-8);

  delete sensorModel;
  sensorModel = NULL;
}


TEST_F(FrameStateTest, SemiMajorAxis10x_SlightlyOffCenter) {
  std::string key = "m_majorAxis";
  double newValue = 100.0;
  UsgsAstroFrameSensorModel* sensorModel = createModifiedStateSensorModel(key, newValue);

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
TEST_F(FrameStateTest, SemiMinorAxis10x_SlightlyOffCenter) {
  std::string key = "m_minorAxis";
  double newValue = 100.0;
  UsgsAstroFrameSensorModel* sensorModel = createModifiedStateSensorModel(key, newValue);

  ASSERT_NE(sensorModel, nullptr);
  csm::ImageCoord imagePt(7.5, 6.5);
  csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
  EXPECT_NEAR(groundPt.x, 9.99803960, 1e-8);
  EXPECT_NEAR(groundPt.y, 0.0, 1e-8);
  EXPECT_NEAR(groundPt.z, 1.98000392, 1e-8);

  delete sensorModel;
  sensorModel = NULL;
}


TEST_F(FrameStateTest, SampleSumming) {
  std::string key = "m_detectorSampleSumming";
  double newValue = 2.0;
  UsgsAstroFrameSensorModel* sensorModel = createModifiedStateSensorModel(key, newValue);

  ASSERT_NE(sensorModel, nullptr);
   csm::ImageCoord imagePt(7.5, 3.75);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   EXPECT_NEAR(groundPt.x, 10.0, 1e-8);
   EXPECT_NEAR(groundPt.y, 0, 1e-8);
   EXPECT_NEAR(groundPt.z, 0, 1e-8);

  delete sensorModel;
  sensorModel = NULL;
}


TEST_F(FrameStateTest, LineSumming) {
  std::string key = "m_detectorLineSumming";
  double newValue = 2.0;
  UsgsAstroFrameSensorModel* sensorModel = createModifiedStateSensorModel(key, newValue);

  ASSERT_NE(sensorModel, nullptr);
   csm::ImageCoord imagePt(3.75, 7.5);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   EXPECT_NEAR(groundPt.x, 10.0, 1e-8);
   EXPECT_NEAR(groundPt.y, 0, 1e-8);
   EXPECT_NEAR(groundPt.z, 0, 1e-8);

  delete sensorModel;
  sensorModel = NULL;
}


TEST_F(FrameStateTest, StartSample) {
  std::string key = "m_startingDetectorSample";
  double newValue = 5.0;
  UsgsAstroFrameSensorModel* sensorModel = createModifiedStateSensorModel(key, newValue);

  ASSERT_NE(sensorModel, nullptr);
   csm::ImageCoord imagePt(7.5, 2.5);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   EXPECT_NEAR(groundPt.x, 10.0, 1e-8);
   EXPECT_NEAR(groundPt.y, 0, 1e-8);
   EXPECT_NEAR(groundPt.z, 0, 1e-8);

  delete sensorModel;
  sensorModel = NULL;
}


TEST_F(FrameStateTest, StartLine) {
  std::string key = "m_startingDetectorLine";
  double newValue = 5.0;
  UsgsAstroFrameSensorModel* sensorModel = createModifiedStateSensorModel(key, newValue);

  ASSERT_NE(sensorModel, nullptr);
   csm::ImageCoord imagePt(2.5, 7.5);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   EXPECT_NEAR(groundPt.x, 10.0, 1e-8);
   EXPECT_NEAR(groundPt.y, 0, 1e-8);
   EXPECT_NEAR(groundPt.z, 0, 1e-8);

  delete sensorModel;
  sensorModel = NULL;
}
