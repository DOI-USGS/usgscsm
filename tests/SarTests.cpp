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
  EXPECT_NEAR(groundPt.x, 1737387.8590770673, 1e-3);
  EXPECT_NEAR(groundPt.y, -5303.280537826621, 1e-3);
  EXPECT_NEAR(groundPt.z, -3749.9796183814506, 1e-3);
}

TEST_F(SarSensorModel, GroundToImage) {
  csm::EcefCoord groundPt(1737387.8590770673, -5303.280537826621, -3749.9796183814506);
  csm::ImageCoord imagePt = sensorModel->groundToImage(groundPt, 0.001);
  EXPECT_NEAR(imagePt.line, 500.0, 1e-3);
  EXPECT_NEAR(imagePt.samp, 500.0, 1e-3);
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
  EXPECT_NEAR(coeffs[0], 2000000.0, 1e-8);
  EXPECT_NEAR(coeffs[1], 0.0040333608396661775, 1e-8);
  EXPECT_NEAR(coeffs[2], 0.0, 1e-8);
  EXPECT_NEAR(coeffs[3], 0.0, 1e-8);
}

TEST_F(SarSensorModel, computeGroundPartials) {
  csm::EcefCoord groundPt(1737400.0, 0.0, 0.0);
  std::vector<double> partials = sensorModel->computeGroundPartials(groundPt);
  ASSERT_EQ(partials.size(), 6);
  EXPECT_NEAR(partials[0], -0.00023385137104424322, 1e-8);
  EXPECT_NEAR(partials[1], -3.3408269124235446e-05, 1e-8);
  EXPECT_NEAR(partials[2], -1.0 / 7.5, 1e-8);
  EXPECT_NEAR(partials[3], -33.057612335589731, 1e-8);
  EXPECT_NEAR(partials[4], 6.3906252180458977e-05, 1e-8);
  EXPECT_NEAR(partials[5], 0.0077565105242805047, 1e-8);
}

TEST_F(SarSensorModel, imageToProximateImagingLocus) {
  csm::EcefLocus locus = sensorModel->imageToProximateImagingLocus(
      csm::ImageCoord(500.0, 500.0),
      csm::EcefCoord(1737287.8590770673, -5403.280537826621, -3849.9796183814506));
  EXPECT_NEAR(locus.point.x, 1737388.1260092105, 1e-2);
  EXPECT_NEAR(locus.point.y, -5403.0102509726485, 1e-2);
  EXPECT_NEAR(locus.point.z, -3749.9801945280433, 1e-2);
  EXPECT_NEAR(locus.direction.x, 0.002701478402694769, 1e-5);
  EXPECT_NEAR(locus.direction.y, -0.9999963509835628, 1e-5);
  EXPECT_NEAR(locus.direction.z, -5.830873570731962e-06, 1e-5);
}

TEST_F(SarSensorModel, imageToRemoteImagingLocus) {
  csm::EcefLocus locus = sensorModel->imageToRemoteImagingLocus(
      csm::ImageCoord(500.0, 500.0));
      EXPECT_NEAR(locus.point.x, 1737380.8279381434, 1e-3);
      EXPECT_NEAR(locus.point.y, 0.0, 1e-3);
      EXPECT_NEAR(locus.point.z, -3749.964442364465, 1e-3);
      EXPECT_NEAR(locus.direction.x, 0.0, 1e-8);
      EXPECT_NEAR(locus.direction.y, -1.0, 1e-8);
      EXPECT_NEAR(locus.direction.z, 0.0, 1e-8);
}
