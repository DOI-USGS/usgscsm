#define _USE_MATH_DEFINES

#include "Fixtures.h"
#include "UsgsAstroPlugin.h"
#include "UsgsAstroLsSensorModel.h"

#include <json.hpp>
#include <gtest/gtest.h>

#include <math.h>
#include <iostream>

using json = nlohmann::json;


TEST_F(ConstVelocityLineScanSensorModel, State) {
   std::string modelState = sensorModel->getModelState();
   sensorModel->replaceModelState(modelState);

   // When this is different, the output is very hard to parse
   // TODO implement JSON diff for gtest

   EXPECT_EQ(sensorModel->getModelState(), modelState);
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

TEST_F(ConstVelocityLineScanSensorModel, OffBody) {
   csm::ImageCoord imagePt(4.5, 4.0);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   EXPECT_NEAR(groundPt.x, 0.063995905, 1e-8);
   EXPECT_NEAR(groundPt.y, -7.999488033, 1e-8);
   EXPECT_NEAR(groundPt.z, 8, 1e-8);
}

TEST_F(ConstVelocityLineScanSensorModel, ProximateImageLocus) {
   csm::ImageCoord imagePt(8.5, 8.0);
   csm::EcefCoord groundPt(5, 5, 5);
   csm::EcefLocus locus = sensorModel->imageToProximateImagingLocus(imagePt, groundPt);
   EXPECT_DOUBLE_EQ(locus.direction.x, -1.0);
   EXPECT_DOUBLE_EQ(locus.direction.y,  0.0);
   EXPECT_DOUBLE_EQ(locus.direction.z,  0.0);
   EXPECT_DOUBLE_EQ(locus.point.x,      5.0);
   EXPECT_DOUBLE_EQ(locus.point.y,      0.0);
   EXPECT_DOUBLE_EQ(locus.point.z,      0.0);
}

TEST_F(ConstVelocityLineScanSensorModel, RemoteImageLocus) {
   csm::ImageCoord imagePt(8.5, 8.0);
   csm::EcefLocus locus = sensorModel->imageToRemoteImagingLocus(imagePt);
   EXPECT_DOUBLE_EQ(locus.direction.x, -1.0);
   EXPECT_DOUBLE_EQ(locus.direction.y,  0.0);
   EXPECT_DOUBLE_EQ(locus.direction.z,  0.0);
   EXPECT_DOUBLE_EQ(locus.point.x,      1000.0);
   EXPECT_DOUBLE_EQ(locus.point.y,      0.0);
   EXPECT_DOUBLE_EQ(locus.point.z,      0.0);
}

// Pan tests

TEST_F(ConstAngularVelocityLineScanSensorModel, Center) {
   csm::ImageCoord imagePt(8.5, 8.0);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   EXPECT_DOUBLE_EQ(groundPt.x, 10.0);
   EXPECT_DOUBLE_EQ(groundPt.y, 0.0);
   EXPECT_DOUBLE_EQ(groundPt.z, 0.0);
}

TEST_F(ConstAngularVelocityLineScanSensorModel, Inversion) {
  csm::ImageCoord imagePt(8.5, 8);
  csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
  csm::ImageCoord imageReprojPt = sensorModel->groundToImage(groundPt);

  EXPECT_DOUBLE_EQ(imagePt.line, imageReprojPt.line);
  EXPECT_DOUBLE_EQ(imagePt.samp, imageReprojPt.samp);
}

TEST_F(ConstAngularVelocityLineScanSensorModel, OffBody) {
   csm::ImageCoord imagePt(4.5, 4.0);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   EXPECT_NEAR(groundPt.x, 4.15414478, 1e-8);
   EXPECT_NEAR(groundPt.y, -7.98311067, 1e-8);
   EXPECT_NEAR(groundPt.z, 63.82129588, 1e-8);
}

TEST_F(ConstVelocityLineScanSensorModel, calculateAttitudeCorrection) {
  std::vector<double> adj;
  double attCorr[9];
  adj.resize(15, 0);
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

TEST_F(OrbitalLineScanSensorModel, Center) {
  csm::ImageCoord imagePt(8.5, 8.0);
  csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
  EXPECT_DOUBLE_EQ(groundPt.x, 999999.680000017);
  EXPECT_DOUBLE_EQ(groundPt.y, 0.0);
  EXPECT_DOUBLE_EQ(groundPt.z, -799.99991466668735);
}

TEST_F(OrbitalLineScanSensorModel, Inversion) {
  for (double line = 0.5; line < 16; line++) {
    csm::ImageCoord imagePt(line, 8);
    csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
    csm::ImageCoord imageReprojPt = sensorModel->groundToImage(groundPt);

    // groundToImage has a default precision of 0.001m and each pixel is 100m
    // so we should be within 0.1 pixels
    EXPECT_NEAR(imagePt.line, imageReprojPt.line, 0.1);
    EXPECT_NEAR(imagePt.samp, imageReprojPt.samp, 0.1);
  }
}

TEST_F(OrbitalLineScanSensorModel, ImageToGroundHeight) {
  csm::ImageCoord imagePt(8.5, 8);
  csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 100.0);
  double height = sqrt(groundPt.x*groundPt.x +
                       groundPt.y*groundPt.y +
                       groundPt.z*groundPt.z);

  EXPECT_DOUBLE_EQ(height, 1000100);
}

TEST_F(OrbitalLineScanSensorModel, InversionHeight) {
  for (double line = 0.5; line < 16; line++) {
    csm::ImageCoord imagePt(line, 8);
    csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 100.0);
    csm::ImageCoord imageReprojPt = sensorModel->groundToImage(groundPt);

    // groundToImage has a default precision of 0.001m and each pixel is 100m
    // so we should be within 0.1 pixels
    EXPECT_NEAR(imagePt.line, imageReprojPt.line, 0.1);
    EXPECT_NEAR(imagePt.samp, imageReprojPt.samp, 0.1);
  }
}

TEST_F(OrbitalLineScanSensorModel, InversionReallyHigh) {
  for (double line = 0.5; line < 16; line++) {
    csm::ImageCoord imagePt(line, 8);
    csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 49000.0);
    csm::ImageCoord imageReprojPt = sensorModel->groundToImage(groundPt);

    // groundToImage has a default precision of 0.001m and each pixel is 2m
    // so we should be within 0.002 pixels
    EXPECT_NEAR(imagePt.line, imageReprojPt.line, 0.002);
    EXPECT_NEAR(imagePt.samp, imageReprojPt.samp, 0.002);
  }
}
