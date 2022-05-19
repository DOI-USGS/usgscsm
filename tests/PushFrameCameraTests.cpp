#define _USE_MATH_DEFINES

#include "Fixtures.h"
#include "UsgsAstroPushFrameSensorModel.h"
#include "UsgsAstroPlugin.h"

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <math.h>
#include <iostream>

using json = nlohmann::json;

TEST_F(OrbitalPushFrameSensorModel, State) {
  std::string modelState = sensorModel->getModelState();
  sensorModel->replaceModelState(modelState);

  // When this is different, the output is very hard to parse
  // TODO implement JSON diff for gtest

  EXPECT_EQ(sensorModel->getModelState(), modelState);
}

TEST_F(OrbitalPushFrameSensorModel, getIlluminationDirectionStationary) {
  // Get state information, replace sun position / velocity to hit third case:
  //  One position, no velocity.
  std::string state = sensorModel->getModelState();
  json jState = stateAsJson(state);
  jState["m_sunPosition"] = std::vector<double>{100.0, 100.0, 100.0};
  jState["m_sunVelocity"] = std::vector<double>{};
  sensorModel->replaceModelState(jState.dump());

  csm::ImageCoord imagePt(8.5, 8);
  csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
  csm::EcefVector direction = sensorModel->getIlluminationDirection(groundPt);

  // Calculate expected sun direction
  // These are the ground point coordinates minus constant sun positions.
  double expected_x = groundPt.x - sensorModel->m_sunPosition[0];
  double expected_y = groundPt.y - sensorModel->m_sunPosition[1];
  double expected_z = groundPt.z - sensorModel->m_sunPosition[2];

  // normalize
  double scale = sqrt((expected_x * expected_x) + (expected_y * expected_y) +
                      (expected_z * expected_z));
  expected_x /= scale;
  expected_y /= scale;
  expected_z /= scale;

  EXPECT_DOUBLE_EQ(direction.x, expected_x);
  EXPECT_DOUBLE_EQ(direction.y, expected_y);
  EXPECT_DOUBLE_EQ(direction.z, expected_z);
}

TEST_F(OrbitalPushFrameSensorModel, getSunPositionLagrange) {
  csm::EcefVector sunPosition = sensorModel->getSunPosition(-.6);
  EXPECT_NEAR(sunPosition.x, 125000000, 1e-5);
  EXPECT_NEAR(sunPosition.y, 125000000, 1e-5);
  EXPECT_NEAR(sunPosition.z, 125000000, 1e-5);
}

TEST_F(OrbitalPushFrameSensorModel, getSunPositionLinear) {
  // Get state information, replace sun position / velocity to hit third case:
  //  One position, no velocity.
  std::string state = sensorModel->getModelState();
  json jState = stateAsJson(state);
  jState["m_sunPosition"] = std::vector<double>{100.0, 100.0, 100.0};
  jState["m_sunVelocity"] = std::vector<double>{50.0, 50.0, 50.0};
  sensorModel->replaceModelState(jState.dump());

  csm::EcefVector sunPosition = sensorModel->getSunPosition(.5);
  EXPECT_DOUBLE_EQ(sunPosition.x, 125);
  EXPECT_DOUBLE_EQ(sunPosition.y, 125);
  EXPECT_DOUBLE_EQ(sunPosition.z, 125);
}

TEST_F(OrbitalPushFrameSensorModel, getSunPositionStationary) {
  // Get state information, replace sun position / velocity to hit third case:
  //  One position, no velocity.
  std::string state = sensorModel->getModelState();
  json jState = stateAsJson(state);
  jState["m_sunPosition"] = std::vector<double>{100.0, 100.0, 100.0};
  jState["m_sunVelocity"] = std::vector<double>{};
  sensorModel->replaceModelState(jState.dump());

  csm::EcefVector sunPosition = sensorModel->getSunPosition(1);
  EXPECT_DOUBLE_EQ(sunPosition.x, 100);
  EXPECT_DOUBLE_EQ(sunPosition.y, 100);
  EXPECT_DOUBLE_EQ(sunPosition.z, 100);
}

TEST_F(OrbitalPushFrameSensorModel, Center) {
  csm::ImageCoord imagePt(6.0, 8.0);
  csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
  EXPECT_NEAR(groundPt.x, 1000000, 1e-9);
  EXPECT_NEAR(groundPt.y, 0.0, 1e-9);
  EXPECT_NEAR(groundPt.z, 0.0, 1e-9);

  double achievedPrecision;
  groundPt = csm::EcefCoord(1000000, 0, 0);
  imagePt = sensorModel->groundToImage(groundPt, 0.001, &achievedPrecision);
  EXPECT_LE(achievedPrecision, 0.001);
  EXPECT_NEAR(imagePt.line, 6, 0.001);
  EXPECT_NEAR(imagePt.samp, 8.0, 0.001);
}

TEST_F(OrbitalPushFrameSensorModel, Inversion) {
  double achievedPrecision;
  for (double line = 0.5; line < 360; line++) {
    for (double samp = 0.5; samp < 16; samp++) {
      csm::ImageCoord imagePt(line, samp);
      csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
      csm::ImageCoord imageReprojPt =
          sensorModel->groundToImage(groundPt, 0.001, &achievedPrecision);

      // It is a known issue that the image-to-ground and ground-to-image
      // operations do not agree for this sensor, because the framelets
      // overlap. Hence use below a generous agreement threshold.
      EXPECT_NEAR(imagePt.line, imageReprojPt.line, 10.0);
      EXPECT_NEAR(imagePt.samp, imageReprojPt.samp, 10.0);
    }
  }
}

TEST_F(OrbitalPushFrameSensorModel, ImageToGroundHeight) {
  csm::ImageCoord imagePt(6.0, 8.0);
  csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
  double height = 100.0;
  csm::EcefCoord groundPtHt = sensorModel->imageToGround(imagePt, height);
  csm::EcefVector heightVec(
        groundPtHt.x - groundPt.x,
        groundPtHt.y - groundPt.y,
        groundPtHt.z - groundPt.z);

  // These numbers may not be perfectly equal because the ray may not
  // go perfectly straight down.
  EXPECT_NEAR(height, norm(heightVec), 0.01);
}
