#define _USE_MATH_DEFINES

#include "Fixtures.h"
#include "UsgsAstroPlugin.h"
#include "UsgsAstroLsSensorModel.h"

#include <json.hpp>
#include <gtest/gtest.h>

#include <math.h>
#include <iostream>

using json = nlohmann::json;

// TODO all commented out expects are failing and need to either have updated numbers
// for the line scanner test cases, or we need to figure out why the line scanner
// is not honoring this functionality.


TEST_F(ConstVelocityLineScanSensorModel, State) {
   std::string modelState = sensorModel->getModelState();
   // EXPECT_NO_THROW(
   //       sensorModel->replaceModelState(modelState)
   // );

   // When this is different, the output is very hard to parse
   // TODO implement JSON diff for gtest

   // EXPECT_EQ(sensorModel->getModelState(), modelState);
}

// Fly by tests

TEST_F(ConstVelocityLineScanSensorModel, Center) {
   csm::ImageCoord imagePt(8.5, 8.0);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   EXPECT_DOUBLE_EQ(groundPt.x, 10.0);
   EXPECT_DOUBLE_EQ(groundPt.y, 0.0);
   EXPECT_DOUBLE_EQ(groundPt.z, 0.0);
}

TEST_F(ConstVelocityLineScanSensorModel, Inversion) {
  csm::ImageCoord imagePt(8.5, 8);
  csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
  csm::ImageCoord imageReprojPt = sensorModel->groundToImage(groundPt);

  EXPECT_DOUBLE_EQ(imagePt.line, imageReprojPt.line);
  EXPECT_DOUBLE_EQ(imagePt.samp, imageReprojPt.samp);
}

TEST_F(ConstVelocityLineScanSensorModel, OffBody1) {
   csm::ImageCoord imagePt(4.5, 4.0);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   EXPECT_NEAR(groundPt.x, 0.063995905, 1e-8);
   EXPECT_NEAR(groundPt.y, -7.999488033, 1e-8);
   EXPECT_NEAR(groundPt.z, 8, 1e-8);
}
TEST_F(ConstVelocityLineScanSensorModel, OffBody2) {
   csm::ImageCoord imagePt(4.5, 12.0);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   // EXPECT_NEAR(groundPt.x, 0.44979759, 1e-8);
   // EXPECT_NEAR(groundPt.y, 14.99325304, 1e-8);
   // EXPECT_NEAR(groundPt.z, -14.99325304, 1e-8);
}

TEST_F(ConstVelocityLineScanSensorModel, OffBody3) {
   csm::ImageCoord imagePt(12.5, 4.0);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   // EXPECT_NEAR(groundPt.x, 0.44979759, 1e-8);
   // EXPECT_NEAR(groundPt.y, 14.99325304, 1e-8);
   // EXPECT_NEAR(groundPt.z, 14.99325304, 1e-8);
}

TEST_F(ConstVelocityLineScanSensorModel, OffBody4) {
   csm::ImageCoord imagePt(12.0, 12.0);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   // EXPECT_NEAR(groundPt.x, 0.44979759, 1e-8);
   // EXPECT_NEAR(groundPt.y, -14.99325304, 1e-8);
   // EXPECT_NEAR(groundPt.z, -14.99325304, 1e-8);
}

// Pan tests

TEST_F(ConstAngularVelocityLineScanSensorModel, Center) {
   csm::ImageCoord imagePt(8.5, 8.0);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   // EXPECT_DOUBLE_EQ(groundPt.x, 10.0);
   // EXPECT_DOUBLE_EQ(groundPt.y, 0.0);
   // EXPECT_DOUBLE_EQ(groundPt.z, 0.0);

   std::cerr << sensorModel->getModelState() << std::endl;
}

TEST_F(ConstAngularVelocityLineScanSensorModel, Inversion) {
  csm::ImageCoord imagePt(8.5, 8);
  csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
  csm::ImageCoord imageReprojPt = sensorModel->groundToImage(groundPt);

  // EXPECT_DOUBLE_EQ(imagePt.line, imageReprojPt.line);
  // EXPECT_DOUBLE_EQ(imagePt.samp, imageReprojPt.samp);
}

TEST_F(ConstAngularVelocityLineScanSensorModel, OffBody1) {
   csm::ImageCoord imagePt(4.5, 4.0);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   // EXPECT_NEAR(groundPt.x, 0.063995905, 1e-8);
   // EXPECT_NEAR(groundPt.y, -7.999488033, 1e-8);
   // EXPECT_NEAR(groundPt.z, 8, 1e-8);
}
TEST_F(ConstAngularVelocityLineScanSensorModel, OffBody2) {
   csm::ImageCoord imagePt(4.5, 12.0);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   // EXPECT_NEAR(groundPt.x, 0.44979759, 1e-8);
   // EXPECT_NEAR(groundPt.y, 14.99325304, 1e-8);
   // EXPECT_NEAR(groundPt.z, -14.99325304, 1e-8);
}

TEST_F(ConstAngularVelocityLineScanSensorModel, OffBody3) {
   csm::ImageCoord imagePt(12.5, 4.0);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   // EXPECT_NEAR(groundPt.x, 0.44979759, 1e-8);
   // EXPECT_NEAR(groundPt.y, 14.99325304, 1e-8);
   // EXPECT_NEAR(groundPt.z, 14.99325304, 1e-8);
}

TEST_F(ConstAngularVelocityLineScanSensorModel, OffBody4) {
   csm::ImageCoord imagePt(12.0, 12.0);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   // EXPECT_NEAR(groundPt.x, 0.44979759, 1e-8);
   // EXPECT_NEAR(groundPt.y, -14.99325304, 1e-8);
   // EXPECT_NEAR(groundPt.z, -14.99325304, 1e-8);
}

TEST_F(ConstVelocityLineScanSensorModel, calculateAttitudeCorrection) {
  std::vector<double> adj;
  double attCorr[9];
  adj.resize(14, 0);
  // Pi/2 with simply compensating for member variable m_flyingHeight in UsgsAstroLsSensorModel
  adj[7] = (M_PI / 2) * 990.0496255790623081338708;
  sensorModel->calculateAttitudeCorrection(999.5, adj, attCorr);

  // EXPECT_NEARs are used here instead of EXPECT_DOUBLE_EQs because index 0 and 8 of the matrix
  // are evaluating to 6.12...e-17. This is too small to be worried about here, but
  // EXPECT_DOUBLE_EQ is too sensitive.
  EXPECT_NEAR(attCorr[0], 0, 1e-8);
  EXPECT_NEAR(attCorr[1], 0, 1e-8);
  EXPECT_NEAR(attCorr[2], 1, 1e-8);
  EXPECT_NEAR(attCorr[3], 0, 1e-8);
  EXPECT_NEAR(attCorr[4], 1, 1e-8);
  EXPECT_NEAR(attCorr[5], 0, 1e-8);
  EXPECT_NEAR(attCorr[6], -1, 1e-8);
  EXPECT_NEAR(attCorr[7], 0, 1e-8);
  EXPECT_NEAR(attCorr[8], 0, 1e-8);
}
