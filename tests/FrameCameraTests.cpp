#include "UsgsAstroPlugin.h"
#include "UsgsAstroFrameSensorModel.h"

#include <json/json.hpp>
#include <gtest/gtest.h>

#include "Fixtures.h"

using json = nlohmann::json;
// NOTE: The imagePt format is (Lines,Samples)

// ****************************************************************************
//   Basic sensor model tests
// ****************************************************************************

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

// ****************************************************************************
//   Orbital sensor model tests
// ****************************************************************************

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

// ****************************************************************************
//   Modified sensor model tests
// ****************************************************************************

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

// ****************************************************************************
//   Adjustable parameter tests
// ****************************************************************************

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

// ****************************************************************************
//   Logging tests
// ****************************************************************************

TEST_F(FrameSensorModelLogging, GroundToImage) {
  csm::EcefCoord groundPt(10.0, 0.0, 0.0);
  sensorModel->groundToImage(groundPt);
}

TEST_F(FrameSensorModelLogging, GroundToImageAdjustments) {
  csm::EcefCoord groundPt(10.0, 0.0, 0.0);
  std::vector<double> adjustments(7, 0.0);
  sensorModel->groundToImage(groundPt, adjustments);
}

TEST_F(FrameSensorModelLogging, GroundToImageCovariance) {
  csm::EcefCoordCovar groundPt(10.0, 0.0, 0.0,
                               1.0, 0.0, 0.0,
                                    1.0, 0.0,
                                         1.0);
  sensorModel->groundToImage(groundPt);
}

TEST_F(FrameSensorModelLogging, ImageToGround) {
  csm::ImageCoord imagePt(7.5, 7.5);
  sensorModel->imageToGround(imagePt, 0.0, 1.0);
}

TEST_F(FrameSensorModelLogging, ImageToGroundCovariance) {
  csm::ImageCoordCovar imagePt(7.5, 7.5,
                               1.0, 0.0,
                                    1.0);
  sensorModel->imageToGround(imagePt, 0.0);
}

TEST_F(FrameSensorModelLogging, ImageToProximateImagingLocus) {
  csm::ImageCoord imagePt(7.5, 7.5);
  csm::EcefCoord groundPt(10.0, 0.0, 0.0);
  sensorModel->imageToProximateImagingLocus(imagePt, groundPt);
}

TEST_F(FrameSensorModelLogging, ImageToRemoteImagingLocus) {
  csm::ImageCoord imagePt(7.5, 7.5);
  sensorModel->imageToRemoteImagingLocus(imagePt);
}

TEST_F(FrameSensorModelLogging, GetImageStart) {
  sensorModel->getImageStart();
}

TEST_F(FrameSensorModelLogging, GetImageSize) {
  sensorModel->getImageSize();
}

TEST_F(FrameSensorModelLogging, GetValidImageRange) {
  sensorModel->getValidImageRange();
}

TEST_F(FrameSensorModelLogging, GetValidHeightRange) {
  sensorModel->getValidHeightRange();
}

TEST_F(FrameSensorModelLogging, GetImageTime) {
  csm::ImageCoord imagePt(7.5, 7.5);
  sensorModel->getImageTime(imagePt);
}

TEST_F(FrameSensorModelLogging, GetSensorPositionPixel) {
  csm::ImageCoord imagePt(7.5, 7.5);
  sensorModel->getSensorPosition(imagePt);
}

TEST_F(FrameSensorModelLogging, GetSensorPositionTime) {
  csm::ImageCoord imagePt(7.5, 7.5);
  double time = sensorModel->getImageTime(imagePt);
  // Dump the constents of the stream because getModelState wrote to it
  oss.str("");
  oss.clear();
  sensorModel->getSensorPosition(time);
}

TEST_F(FrameSensorModelLogging, GetSensorVelocityPixel) {
  csm::ImageCoord imagePt(7.5, 7.5);
  sensorModel->getSensorVelocity(imagePt);
}

TEST_F(FrameSensorModelLogging, GetSensorVelocityTime) {
  csm::ImageCoord imagePt(7.5, 7.5);
  double time = sensorModel->getImageTime(imagePt);
  // Dump the constents of the stream because getModelState wrote to it
  oss.str("");
  oss.clear();
  sensorModel->getSensorVelocity(time);
}

TEST_F(FrameSensorModelLogging, ComputeSensorPartialsGround) {
  csm::EcefCoord groundPt(10.0, 0.0, 0.0);
  sensorModel->computeSensorPartials(0, groundPt);
}

TEST_F(FrameSensorModelLogging, ComputeSensorPartialsBoth) {
  csm::ImageCoord imagePt(7.5, 7.5);
  csm::EcefCoord groundPt(10.0, 0.0, 0.0);
  sensorModel->computeSensorPartials(0, imagePt, groundPt);
}

TEST_F(FrameSensorModelLogging, ComputeAllSensorPartialsGround) {
  csm::EcefCoord groundPt(10.0, 0.0, 0.0);
  sensorModel->computeAllSensorPartials(groundPt);
}

TEST_F(FrameSensorModelLogging, ComputeAllSensorPartialsBoth) {
  csm::ImageCoord imagePt(7.5, 7.5);
  csm::EcefCoord groundPt(10.0, 0.0, 0.0);
  sensorModel->computeAllSensorPartials(imagePt, groundPt);
}

TEST_F(FrameSensorModelLogging, GetCorrelationModel) {
  sensorModel->getCorrelationModel();
}

TEST_F(FrameSensorModelLogging, GetUnmodeledCrossCovariance) {
  csm::ImageCoord imagePt1(7.5, 7.5);
  csm::ImageCoord imagePt2(8.5, 8.5);
  sensorModel->getUnmodeledCrossCovariance(imagePt1, imagePt2);
}

TEST_F(FrameSensorModelLogging, GetVersion) {
  sensorModel->getVersion();
}

TEST_F(FrameSensorModelLogging, GetModelName) {
  sensorModel->getModelName();
}

TEST_F(FrameSensorModelLogging, GetPedigree) {
  sensorModel->getPedigree();
}

TEST_F(FrameSensorModelLogging, GetImageIdentifier) {
  sensorModel->getImageIdentifier();
}

TEST_F(FrameSensorModelLogging, SetImageIdentifier) {
  sensorModel->setImageIdentifier("logging_ID");
}

TEST_F(FrameSensorModelLogging, GetSensorIdentifier) {
  sensorModel->getSensorIdentifier();
}

TEST_F(FrameSensorModelLogging, GetPlatformIdentifier) {
  sensorModel->getPlatformIdentifier();
}

TEST_F(FrameSensorModelLogging, GetCollectionIdentifier) {
  sensorModel->getCollectionIdentifier();
}

TEST_F(FrameSensorModelLogging, GetTrajectoryIdentifier) {
  sensorModel->getTrajectoryIdentifier();
}

TEST_F(FrameSensorModelLogging, GetSensorType) {
  sensorModel->getSensorType();
}

TEST_F(FrameSensorModelLogging, GetSensorMode) {
  sensorModel->getSensorMode();
}

TEST_F(FrameSensorModelLogging, GetReferenceDateAndTime) {
  sensorModel->getReferenceDateAndTime();
}

TEST_F(FrameSensorModelLogging, GetModelState) {
  sensorModel->getModelState();
}

TEST_F(FrameSensorModelLogging, replaceModelState) {
  std::string state = sensorModel->getModelState();
  // Dump the constents of the stream because getModelState wrote to it
  oss.str("");
  oss.clear();
  sensorModel->replaceModelState(state);
}

TEST_F(FrameSensorModelLogging, GetReferencePoint) {
  sensorModel->getReferencePoint();
}

TEST_F(FrameSensorModelLogging, SetReferencePoint) {
  csm::EcefCoord groundPt(10.0, 0.0, 0.0);
  sensorModel->setReferencePoint(groundPt);
}

TEST_F(FrameSensorModelLogging, GetNumParameters) {
  sensorModel->getNumParameters();
}

TEST_F(FrameSensorModelLogging, GetParameterName) {
  sensorModel->getParameterName(0);
}

TEST_F(FrameSensorModelLogging, GetParameterUnits) {
  sensorModel->getParameterUnits(0);
}

TEST_F(FrameSensorModelLogging, HasShareableParameters) {
  sensorModel->hasShareableParameters();
}

TEST_F(FrameSensorModelLogging, IsParameterShareable) {
  sensorModel->isParameterShareable(0);
}

TEST_F(FrameSensorModelLogging, GetParameterSharingCriteria) {
  try {
    sensorModel->getParameterSharingCriteria(0);
  }
  catch (...) {
    // Just testing logging, so do nothing, it should still log
  }
}

TEST_F(FrameSensorModelLogging, GetParameterValue) {
  sensorModel->getParameterValue(0);
}

TEST_F(FrameSensorModelLogging, SetParameterValue) {
  sensorModel->setParameterValue(0, 10);
}

TEST_F(FrameSensorModelLogging, GetParameterType) {
  sensorModel->getParameterType(0);
}

TEST_F(FrameSensorModelLogging, SetParameterType) {
  sensorModel->setParameterType(0, csm::param::REAL);
}

TEST_F(FrameSensorModelLogging, GetParameterCovariance) {
  sensorModel->getParameterCovariance(0, 0);
}

TEST_F(FrameSensorModelLogging, SetParameterCovariance) {
  sensorModel->setParameterCovariance(0, 0, 1);
}

TEST_F(FrameSensorModelLogging, GetNumGeometricCorrectionSwitches) {
  sensorModel->getNumGeometricCorrectionSwitches();
}

TEST_F(FrameSensorModelLogging, GetGeometricCorrectionName) {
  try {
    sensorModel->getGeometricCorrectionName(0);
  }
  catch (...) {
    // Just testing logging, so do nothing, it should still log
  }
}

TEST_F(FrameSensorModelLogging, SetGeometricCorrectionSwitch) {
  try {
    sensorModel->setGeometricCorrectionSwitch(0, true, csm::param::REAL);
  }
  catch (...) {
    // Just testing logging, so do nothing, it should still log
  }
}

TEST_F(FrameSensorModelLogging, GetGeometricCorrectionSwitch) {
  try {
    sensorModel->getGeometricCorrectionSwitch(0);
  }
  catch (...) {
    // Just testing logging, so do nothing, it should still log
  }
}

TEST_F(FrameSensorModelLogging, GetIlluminationDirection) {
  csm::EcefCoord groundPt(10.0, 0.0, 0.0);
  sensorModel->getIlluminationDirection(groundPt);
}

TEST_F(FrameSensorModelLogging, ComputeGroundPartials) {
  csm::EcefCoord groundPt(10.0, 0.0, 0.0);
  sensorModel->computeGroundPartials(groundPt);
}

TEST_F(FrameSensorModelLogging, IsValidModelState) {
  sensorModel->isValidModelState("{\"test_key\" : \"test_string\"}", nullptr);
}

TEST_F(FrameSensorModelLogging, IsValidIsd) {
  sensorModel->isValidIsd("{\"test_key\" : \"test_string\"}", nullptr);
}

TEST_F(FrameSensorModelLogging, ConstructStateFromIsd) {
  try {
    sensorModel->constructStateFromIsd("{\"test_key\" : \"test_string\"}", nullptr);
  }
  catch (...) {
    // Just testing logging, so do nothing, it should still log
  }
}

TEST_F(FrameSensorModelLogging, GetValue) {
  std::vector<double> adjustments(7, 0.0);
  sensorModel->getValue(0, adjustments);
}

TEST_F(FrameSensorModelLogging, CalcRotationMatrix) {
  double m[3][3];
  sensorModel->calcRotationMatrix(m);
}

TEST_F(FrameSensorModelLogging, CalcRotationMatrixAdjusted) {
  std::vector<double> adjustments(7, 0.0);
  double m[3][3];
  sensorModel->calcRotationMatrix(m, adjustments);
}

TEST_F(FrameSensorModelLogging, losEllipsoidIntersect) {
  double height, xc, yc, zc, xl, yl, zl, x, y, z;
  height = 0.0;
  xc = 1000;
  yc = 0.0;
  zc = 0.0;
  xl = -1.0;
  yl = 0.0;
  zl = 0.0;
  sensorModel->losEllipsoidIntersect(
        height,
        xc, yc, zc,
        xl, yl, zl,
        x, y, z);
}

TEST_F(OrbitalFrameSensorModel, ReferenceDateTime) {
  std::string date = sensorModel->getReferenceDateAndTime();
  EXPECT_EQ(date, "20000101T001640");
}
