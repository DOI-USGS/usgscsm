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
  double achievedPrecision;
  csm::ImageCoord imagePt(8.5, 8);
  csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
  csm::ImageCoord imageReprojPt = sensorModel->groundToImage(groundPt, 0.001, &achievedPrecision);

  EXPECT_LT(achievedPrecision, 0.001);
  EXPECT_NEAR(imagePt.line, imageReprojPt.line, 1e-3);
  EXPECT_NEAR(imagePt.samp, imageReprojPt.samp, 1e-3);
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

TEST_F(ConstVelocityLineScanSensorModel, calculateAttitudeCorrection) {
  std::vector<double> adj;
  double attCorr[9];
  adj.resize(15, 0);
  // Pi/2 with simply compensating for member variable m_flyingHeight in UsgsAstroLsSensorModel
  adj[7] = (M_PI / 2) * sensorModel->m_flyingHeight;
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
  // These are the ground point coordinates minus constant sun positions.
  double expected_x = groundPt.x - sensorModel->m_sunPosition[0];
  double expected_y = groundPt.y - sensorModel->m_sunPosition[1];
  double expected_z = groundPt.z - sensorModel->m_sunPosition[2];

  //normalize
  double scale = sqrt((expected_x * expected_x) + (expected_y * expected_y) + (expected_z * expected_z));
  expected_x /= scale;
  expected_y /= scale;
  expected_z /= scale;

  EXPECT_DOUBLE_EQ(direction.x, expected_x);
  EXPECT_DOUBLE_EQ(direction.y, expected_y);
  EXPECT_DOUBLE_EQ(direction.z, expected_z);
}

TEST_F(OrbitalLineScanSensorModel, getSunPositionLagrange){
  std::cout<<sensorModel->m_t0Ephem<<std::endl;
  csm::EcefVector sunPosition = sensorModel->getSunPosition(-.6);
  EXPECT_DOUBLE_EQ(sunPosition.x, 125);
  EXPECT_DOUBLE_EQ(sunPosition.y, 125);
  EXPECT_DOUBLE_EQ(sunPosition.z, 125);
}

TEST_F(OrbitalLineScanSensorModel, getSunPositionLinear){
  // Get state information, replace sun position / velocity to hit third case:
  //  One position, no velocity.
  std::string state = sensorModel->getModelState();
  json jState = json::parse(state);
  jState["m_sunPosition"] = std::vector<double>{100.0,100.0,100.0};
  jState["m_sunVelocity"] = std::vector<double>{50.0, 50.0, 50.0};
  sensorModel->replaceModelState(jState.dump());

  csm::EcefVector sunPosition = sensorModel->getSunPosition(.5);
  EXPECT_DOUBLE_EQ(sunPosition.x, 125);
  EXPECT_DOUBLE_EQ(sunPosition.y, 125);
  EXPECT_DOUBLE_EQ(sunPosition.z, 125);
}

TEST_F(OrbitalLineScanSensorModel, getSunPositionStationary){
  // Get state information, replace sun position / velocity to hit third case:
  //  One position, no velocity.
  std::string state = sensorModel->getModelState();
  json jState = json::parse(state);
  jState["m_sunPosition"] = std::vector<double>{100.0,100.0,100.0};
  jState["m_sunVelocity"] = std::vector<double>{};
  sensorModel->replaceModelState(jState.dump());

  csm::EcefVector sunPosition = sensorModel->getSunPosition(1);
  EXPECT_DOUBLE_EQ(sunPosition.x, 100);
  EXPECT_DOUBLE_EQ(sunPosition.y, 100);
  EXPECT_DOUBLE_EQ(sunPosition.z, 100);
}

TEST_F(OrbitalLineScanSensorModel, Center) {
  csm::ImageCoord imagePt(8.5, 8.0);
  csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
  EXPECT_DOUBLE_EQ(groundPt.x, 999999.70975015126);
  EXPECT_DOUBLE_EQ(groundPt.y, 0.0);
  EXPECT_DOUBLE_EQ(groundPt.z, -761.90525203960692);
}

TEST_F(OrbitalLineScanSensorModel, Inversion) {
  double achievedPrecision;
  for (double line = 0.5; line < 16; line++) {
    csm::ImageCoord imagePt(line, 8);
    csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
    csm::ImageCoord imageReprojPt = sensorModel->groundToImage(groundPt, 0.001, &achievedPrecision);

    // groundToImage has a default precision of 0.001m and each pixel is 100m
    // so we should be within 0.1 pixels
    EXPECT_LT(achievedPrecision, 0.001);
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
    csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 4900.0);
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

TEST_F(TwoLineScanSensorModels, CrossCovariance) {
  std::vector<double> crossCovars = sensorModel1->getCrossCovarianceMatrix(*sensorModel2);
  ASSERT_EQ(crossCovars.size(), sensorModel1->getNumParameters() * sensorModel2->getNumParameters());
  for (int i = 0; i < sensorModel1->getNumParameters(); i++) {
    for (int j = 0; j < sensorModel2->getNumParameters(); j++) {
      EXPECT_EQ(crossCovars[i * sensorModel2->getNumParameters() + j], 0.0)
                << "Value at row " << i << " column " << j;
    }
  }

  std::vector<double> covars = sensorModel1->getCrossCovarianceMatrix(*sensorModel1);
  ASSERT_EQ(covars.size(), 16*16);
  for (int i = 0; i < 16; i++) {
    for (int j = 0; j < 16; j++) {
      if (i == j) {
        EXPECT_GT(covars[i * 16 + j], 0.0) << "Value at row " << i << " column " << j;
      }
      else {
        EXPECT_EQ(covars[i * 16 + j], 0.0) << "Value at row " << i << " column " << j;
      }
    }
  }

  std::vector<double> fixedCovars = sensorModel1->getCrossCovarianceMatrix(*sensorModel1, csm::param::NON_ADJUSTABLE);
  EXPECT_EQ(fixedCovars.size(), 0);
}

TEST_F(ConstVelocityLineScanSensorModel, FocalLengthAdjustment) {
  csm::ImageCoord imagePt(8.5, 4.0);
  sensorModel->setParameterValue(15, 0.9 * sensorModel->m_halfSwath);
  csm::EcefLocus locus = sensorModel->imageToRemoteImagingLocus(imagePt);
  EXPECT_DOUBLE_EQ(locus.direction.x, -5.0 / sqrt(5 * 5 + 0.4 * 0.4));
  EXPECT_DOUBLE_EQ(locus.direction.y, -0.4 / sqrt(5 * 5 + 0.4 * 0.4));
  EXPECT_DOUBLE_EQ(locus.direction.z, 0.0);
}
