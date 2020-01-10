#define _USE_MATH_DEFINES

#include "Fixtures.h"
#include "UsgsAstroPlugin.h"
#include "UsgsAstroLsSensorModel.h"

#include <json/json.hpp>
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
   EXPECT_NEAR(groundPt.x, 10.0, 1e-12);
   EXPECT_NEAR(groundPt.y, 0.0, 1e-12);
   EXPECT_NEAR(groundPt.z, 0.0, 1e-12);
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
   EXPECT_NEAR(locus.direction.x, -1.0, 1e-12);
   EXPECT_NEAR(locus.direction.y,  0.0, 1e-12);
   EXPECT_NEAR(locus.direction.z,  0.0, 1e-12);
   EXPECT_NEAR(locus.point.x,      5.0, 1e-12);
   EXPECT_NEAR(locus.point.y,      0.0, 1e-12);
   EXPECT_NEAR(locus.point.z,      0.0, 1e-12);
}

TEST_F(ConstVelocityLineScanSensorModel, RemoteImageLocus) {
   csm::ImageCoord imagePt(8.5, 8.0);
   csm::EcefLocus locus = sensorModel->imageToRemoteImagingLocus(imagePt);
   EXPECT_NEAR(locus.direction.x, -1.0, 1e-12);
   EXPECT_NEAR(locus.direction.y,  0.0, 1e-12);
   EXPECT_NEAR(locus.direction.z,  0.0, 1e-12);
   EXPECT_NEAR(locus.point.x,      1000.0, 1e-12);
   EXPECT_NEAR(locus.point.y,      0.0, 1e-12);
   EXPECT_NEAR(locus.point.z,      0.0, 1e-12);
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

TEST_F(OrbitalLineScanSensorModel, getIlluminationDirectionLagrange) {
  // Unlike other getIlluminationDirection tests, the default ISD is parameterized
  //  such that it will enter the lagrange branch.
  csm::ImageCoord imagePt(8.5,8);
  csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
  csm::EcefVector direction = sensorModel->getIlluminationDirection(groundPt);

  double imageTime = sensorModel->getImageTime(imagePt);
  double sunPos[3];
  lagrangeInterp(sensorModel->m_sunPosition.size()/3,
                              &(sensorModel->m_sunPosition[0]),
                              sensorModel->m_t0Ephem,
                              sensorModel->m_dtEphem,
                              imageTime,
                              3, 8, sunPos);

  double expected_x = groundPt.x - sunPos[0];
  double expected_y = groundPt.y - sunPos[1];
  double expected_z = groundPt.z - sunPos[2];

  // Normalize
  double scale = sqrt((expected_x * expected_x) + (expected_y * expected_y) + (expected_z * expected_z));
  expected_x /= scale;
  expected_y /= scale;
  expected_z /= scale;

  EXPECT_DOUBLE_EQ(direction.x, expected_x);
  EXPECT_DOUBLE_EQ(direction.y, expected_y);
  EXPECT_DOUBLE_EQ(direction.z, expected_z);
}

TEST_F(OrbitalLineScanSensorModel, getIlluminationDirectionLinear) {
  // Get state information, replace sun position / velocity to hit second case:
  //  One position, one velocity.
  std::string state = sensorModel->getModelState();
  json jState = json::parse(state);
  jState["m_sunPosition"] = std::vector<double>{100.0,100.0,100.0};
  jState["m_sunVelocity"] = std::vector<double>{50.0, 25.0, 10.0};
  sensorModel->replaceModelState(jState.dump());

  csm::ImageCoord imagePt(8.5,8);
  csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
  csm::EcefVector direction = sensorModel->getIlluminationDirection(groundPt);

  // Calculate expected sun direction
  // Image Time = .00000000000000011102230246251565404236316680908203125
  double imageTime = sensorModel->getImageTime(imagePt);
  double expected_x = 999999.680000017 - ((imageTime*50.0) + 100);
  double expected_y = 0.0 -  ((imageTime*25.0) + 100);
  double expected_z = -799.99991466668735 - ((imageTime*10.0) + 100);

  // Normalize
  double scale = sqrt((expected_x * expected_x) + (expected_y * expected_y) + (expected_z * expected_z));
  expected_x /= scale;
  expected_y /= scale;
  expected_z /= scale;

  EXPECT_DOUBLE_EQ(direction.x, expected_x);
  EXPECT_DOUBLE_EQ(direction.y, expected_y);
  EXPECT_DOUBLE_EQ(direction.z, expected_z);

}

TEST_F(OrbitalLineScanSensorModel, getIlluminationDirectionStationary) {
  // Get state information, replace sun position / velocity to hit third case:
  //  One position, no velocity.
  std::string state = sensorModel->getModelState();
  json jState = json::parse(state);
  jState["m_sunPosition"] = std::vector<double>{100.0,100.0,100.0};
  jState["m_sunVelocity"] = std::vector<double>{};
  sensorModel->replaceModelState(jState.dump());

  csm::ImageCoord imagePt(8.5,8);
  csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
  csm::EcefVector direction = sensorModel->getIlluminationDirection(groundPt);

  // Calculate expected sun direction
  double expected_x = 999999.680000017 - 100;
  double expected_y = 0.0 - 100;
  double expected_z = -799.99991466668735 - 100;

  double scale = sqrt((expected_x * expected_x) + (expected_y * expected_y) + (expected_z * expected_z));

  expected_x /= scale;
  expected_y /= scale;
  expected_z /= scale;

  EXPECT_DOUBLE_EQ(direction.x, expected_x);
  EXPECT_DOUBLE_EQ(direction.y, expected_y);
  EXPECT_DOUBLE_EQ(direction.z, expected_z);
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

TEST_F(OrbitalLineScanSensorModel, ReferenceDateTime) {
  std::string date = sensorModel->getReferenceDateAndTime();
  EXPECT_EQ(date, "20000101T001639");
}
