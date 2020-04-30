#define _USE_MATH_DEFINES

#include "UsgsAstroSarSensorModel.h"

#include "Warning.h"
#include "Fixtures.h"

#include <json/json.hpp>
#include <gtest/gtest.h>

#include <math.h>
#include <string>
#include <fstream>
#include <iostream>

using json = nlohmann::json;

TEST(SarTests, stateFromIsd) {
  std::ifstream isdFile("data/orbitalSar.json");
  json isdJson;
  isdFile >> isdJson;
  std::string isdString = isdJson.dump();
  csm::WarningList warnings;
  std::string stateString;
  try {
    stateString = UsgsAstroSarSensorModel::constructStateFromIsd(isdString, &warnings);
  }
  catch(...) {
    for (auto &warn: warnings) {
      std::cerr << "Warning in " << warn.getFunction() << std::endl;
      std::cerr << "  " << warn.getMessage() << std::endl;
    }
    FAIL() << "constructStateFromIsd errored";
  }
  EXPECT_TRUE(warnings.empty());
}

TEST_F(SarSensorModel, State) {
  std::string modelState = sensorModel->getModelState();
  EXPECT_NO_THROW(
      sensorModel->replaceModelState(modelState)
      );
  EXPECT_EQ(sensorModel->getModelState(), modelState);
}

TEST_F(SarSensorModel, Center) {
  csm::ImageCoord imagePt(500.0, 500.0);
  csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
  // TODO these tolerances are bad
  EXPECT_NEAR(groundPt.x, 1737391.90602155, 1e-2);
  EXPECT_NEAR(groundPt.y, -3749.98835331, 1e-2);
  EXPECT_NEAR(groundPt.z, -3749.99708833, 1e-2);
}

TEST_F(SarSensorModel, GroundToImage) {
  csm::EcefCoord groundPt(1737391.90602155, -3749.98835331, -3749.99708833);
  csm::ImageCoord imagePt = sensorModel->groundToImage(groundPt, 0.001);
  EXPECT_NEAR(imagePt.line, 500.0, 0.001);
  // Due to position interpolation, the sample is slightly less accurate than the line
  EXPECT_NEAR(imagePt.samp, 500.0, 0.002);
}

TEST_F(SarSensorModel, spacecraftPosition) {
  csm::EcefVector position = sensorModel->getSpacecraftPosition(-0.0025);
  EXPECT_NEAR(position.x, 3.73740000e+06, 1e-8);
  EXPECT_NEAR(position.y, 0.00000000e+00, 1e-8);
  EXPECT_NEAR(position.z, 0.00000000e+00, 1e-8);
}

TEST_F(SarSensorModel, spacecraftVelocity) {
  csm::EcefVector velocity = sensorModel->getSensorVelocity(-0.0025);
  EXPECT_NEAR(velocity.x, 0.00000000e+00, 1e-8);
  EXPECT_NEAR(velocity.y, 0.00000000e+00, 1e-8);
  EXPECT_NEAR(velocity.z, -3.73740000e+06, 1e-8);
}

TEST_F(SarSensorModel, getRangeCoefficients) {
  std::vector<double> coeffs = sensorModel->getRangeCoefficients(-0.0025);
  EXPECT_NEAR(coeffs[0], 2000000.0000039602, 1e-8);
  EXPECT_NEAR(coeffs[1], -1.0504347070801814e-08, 1e-8);
  EXPECT_NEAR(coeffs[2], 5.377926500384916e-07, 1e-8);
  EXPECT_NEAR(coeffs[3], -1.3072206620088014e-15, 1e-8);
}

TEST_F(SarSensorModel, imageToProximateImagingLocus) {
  csm::EcefLocus locus = sensorModel->imageToProximateImagingLocus(
      csm::ImageCoord(500.0, 500.0),
      csm::EcefCoord(1737291.90643026, -3750.17585202, -3749.78124955));
  EXPECT_NEAR(locus.point.x, 1737391.90602155, 1e-2);
  EXPECT_NEAR(locus.point.y, -3749.98835331, 1e-2);
  EXPECT_NEAR(locus.point.z, -3749.99708833, 1e-2);
  EXPECT_NEAR(locus.direction.x, 0.0018750001892442036, 1e-5);
  EXPECT_NEAR(locus.direction.y, -0.9999982421774111, 1e-5);
  EXPECT_NEAR(locus.direction.z, -4.047002203562211e-06, 1e-5);
}

TEST_F(SarSensorModel, imageToRemoteImagingLocus) {
  csm::EcefLocus locus = sensorModel->imageToRemoteImagingLocus(
      csm::ImageCoord(500.0, 500.0));
      EXPECT_NEAR(locus.point.x, 1737388.3904556315, 1e-2);
      EXPECT_NEAR(locus.point.y, 0.0, 1e-2);
      EXPECT_NEAR(locus.point.z, -3749.9807653094517, 1e-2);
      EXPECT_NEAR(locus.direction.x, 0.0, 1e-8);
      EXPECT_NEAR(locus.direction.y, -1.0, 1e-8);
      EXPECT_NEAR(locus.direction.z, 0.0, 1e-8);
}
