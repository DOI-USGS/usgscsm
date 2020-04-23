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

// Placeholder for future "real" i->g test.
TEST_F(SarSensorModel, Center) {
  csm::ImageCoord imagePt(7.5, 7.5);
  csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
  EXPECT_NEAR(groundPt.x, 0, 1e-8);
  EXPECT_NEAR(groundPt.y, 0, 1e-8);
  EXPECT_NEAR(groundPt.z, 0, 1e-8);
}

TEST_F(SarSensorModel, GroundToImage) {
  // ground point from test image campt ISIS run
  csm::EcefCoord groundPt(-1605729.6097547, -466940.66390268,
                                487897.51906835);
  csm::ImageCoord imagePt2 = sensorModel->groundToImage(groundPt);
  EXPECT_NEAR(groundPt.x, 0, 1e-8);
  EXPECT_NEAR(groundPt.y, 0, 1e-8);
  EXPECT_NEAR(groundPt.z, 0, 1e-8);
}

TEST_F(SarSensorModel, spacecraftPosition) {
//   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt1, 0.0);
  csm::EcefVector position = sensorModel->getSpacecraftPosition(0.0);
  EXPECT_NEAR(position.x, 3.73740000e+06, 1e-8);
  EXPECT_NEAR(position.y, 0.00000000e+00, 1e-8);
  EXPECT_NEAR(position.z, -0.00000000e+00, 1e-8);
}

TEST_F(SarSensorModel, spacecraftVelocity) {
  csm::EcefVector velocity = sensorModel->getSpacecraftVelocity(0.0);
  EXPECT_NEAR(velocity.x, -0.00000000e+00, 1e-8);
  EXPECT_NEAR(velocity.y, 0.00000000e+00, 1e-8);
  EXPECT_NEAR(velocity.z, -3.73740000e+06, 1e-8);
}

//TEST_F(SarSensorModel, spacecraftPositionInterp) {
//  csm::EcefVector position = sensorModel->getSpacecraftPosition(4.125);
//  EXPECT_NEAR(position.x, 3.73737771e+06, 1e-8);
//  EXPECT_NEAR(position.y, 0.00000000e+00, 1e-8);
//  EXPECT_NEAR(position.z, -1.29068467e+04, 1e-8);
//}
//
//TEST_F(SarSensorModel, spacecraftVelocityInterp) {
//  csm::EcefVector velocity = sensorModel->getSpacecraftVelocity(4.125);
//  EXPECT_NEAR(velocity.x, -1.21001717e+04, 1e-8);
//  EXPECT_NEAR(velocity.y,  0.00000000e+00, 1e-8);
//  EXPECT_NEAR(velocity.z, -3.73738041e+06, 1e-8);
//}

TEST_F(SarSensorModel, getRangeCoefficientsNoInterp) {
  std::vector<double> coeffs = sensorModel->getRangeCoefficients(3.25441417470548e+08);
  EXPECT_NEAR(coeffs[0], 7.99423808710000e+04, 1e-8);
  EXPECT_NEAR(coeffs[1], 6.92122900000000e-01, 1e-8);
  EXPECT_NEAR(coeffs[2], 3.40193700000000e-06, 1e-8);
  EXPECT_NEAR(coeffs[3], -2.39924200000000e-11, 1e-8);
}

// Halfway between 2 values at indices 10 and 11 in the list
TEST_F(SarSensorModel, getRangeCoefficientsInterp) {
  std::vector<double> coeffs = sensorModel->getRangeCoefficients(325441567.760548);
  EXPECT_NEAR(coeffs[0], 7.99423758150000e+04, 1e-8);
  EXPECT_NEAR(coeffs[1], 6.84453800000000e-01, 1e-8);
  EXPECT_NEAR(coeffs[2], 3.46968000000000e-06, 1e-8);
  EXPECT_NEAR(coeffs[3], -2.42686600000000e-11, 1e-8);
}
