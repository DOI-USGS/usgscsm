#define _USE_MATH_DEFINES

#include "UsgsAstroSarSensorModel.h"

#include "Fixtures.h"
#include "Warning.h"

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <math.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using json = nlohmann::json;

std::vector<std::string> compareJson(json &leftJson, json &rightJson) {
  std::vector<std::string> differences;
  for (json::iterator it = leftJson.begin(); it != leftJson.end(); ++it) {
    std::string key = it.key();
    if (!rightJson.contains(key)) {
      differences.push_back("Right JSON object is missing key [" + key +
          "] which is present in the left JSON object.");
    }
    else if (leftJson[key].type() != rightJson[key].type()) {
      differences.push_back("Different types for key [" + key + "].");
    }
    else if (leftJson[key].is_object() && rightJson[key].is_object()) {
      differences.push_back("Difference(s) in object for key [" + key + "].");
      std::vector<std::string> deeperDiffs = compareJson(leftJson[key], rightJson[key]);
      differences.insert(differences.end(), deeperDiffs.begin() , deeperDiffs.end());
    }
    else if (leftJson[key] != rightJson[key]) {
      differences.push_back("Different values for key [" + key + "].");
    }
  }

  for (json::iterator it = rightJson.begin(); it != rightJson.end(); ++it) {
    std::string key = it.key();
    if (!leftJson.contains(key)) {
      differences.push_back("Left JSON object is missing key [" + key +
          "] which is present in the right JSON object.");
    }
  }
  return differences;
}

TEST_F(SarSensorModel, stateFromIsd) {
  std::ifstream isdFile("data/orbitalSar.json");
  json isdJson;
  isdFile >> isdJson;
  std::string isdString = isdJson.dump();
  csm::WarningList warnings;
  std::string stateString;
  try {
    stateString = sensorModel->constructStateFromIsd(isdString, &warnings);
  } catch (...) {
    for (auto &warn : warnings) {
      std::cerr << "Warning in " << warn.getFunction() << std::endl;
      std::cerr << "  " << warn.getMessage() << std::endl;
    }
    FAIL() << "constructStateFromIsd errored";
  }
  EXPECT_TRUE(warnings.empty());
}

TEST_F(SarSensorModel, State) {
  std::string modelState = sensorModel->getModelState();
  EXPECT_NO_THROW(sensorModel->replaceModelState(modelState));
  std::string newModelState = sensorModel->getModelState();
  json oldJson = json::parse(modelState);
  json newJson = json::parse(newModelState);
  std::vector<std::string> differences = compareJson(oldJson, newJson);
  EXPECT_TRUE(differences.empty());
  for (std::string &difference : differences) {
    std::cerr << difference << std::endl;
  }
}

TEST_F(SarSensorModel, Center) {
  csm::ImageCoord imagePt(500.0, 500.0);
  csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
  EXPECT_NEAR(groundPt.x, 1737387.8590770673, 1e-3);
  EXPECT_NEAR(groundPt.y, -5303.280537826621, 1e-3);
  EXPECT_NEAR(groundPt.z, -3749.9796183814506, 1e-3);
}

TEST_F(SarSensorModel, GroundToImage) {
  csm::EcefCoord groundPt(1737387.8590770673, -5303.280537826621,
                          -3749.9796183814506);
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
  EXPECT_NEAR(partials[0], -0.00023385137107532946, 1e-8);
  EXPECT_NEAR(partials[1], -0.00023385788390593021, 1e-8);
  EXPECT_NEAR(partials[2], -0.13339937076082886, 1e-8);
  EXPECT_NEAR(partials[3], -33.05761233362206, 1e-8);
  EXPECT_NEAR(partials[4], 7.5445337157968123e-05, 1e-8);
  EXPECT_NEAR(partials[5], 0.0077604615628256903, 1e-8);
}

TEST_F(SarSensorModel, imageToProximateImagingLocus) {
  csm::EcefLocus locus = sensorModel->imageToProximateImagingLocus(
      csm::ImageCoord(500.0, 500.0),
      csm::EcefCoord(1737287.8590770673, -5403.280537826621,
                     -3849.9796183814506));
  EXPECT_NEAR(locus.point.x, 1737388.1260092105, 1e-2);
  EXPECT_NEAR(locus.point.y, -5403.0102509726485, 1e-2);
  EXPECT_NEAR(locus.point.z, -3749.9801945280433, 1e-2);
  EXPECT_NEAR(locus.direction.x, 0.002701478402694769, 1e-5);
  EXPECT_NEAR(locus.direction.y, -0.9999963509835628, 1e-5);
  EXPECT_NEAR(locus.direction.z, -5.830873570731962e-06, 1e-5);
}

TEST_F(SarSensorModel, imageToRemoteImagingLocus) {
  csm::EcefLocus locus =
      sensorModel->imageToRemoteImagingLocus(csm::ImageCoord(500.0, 500.0));
  EXPECT_NEAR(locus.point.x, 1737380.8279381434, 1e-3);
  EXPECT_NEAR(locus.point.y, 0.0, 1e-3);
  EXPECT_NEAR(locus.point.z, -3749.964442364465, 1e-3);
  EXPECT_NEAR(locus.direction.x, 0.0, 1e-8);
  EXPECT_NEAR(locus.direction.y, -1.0, 1e-8);
  EXPECT_NEAR(locus.direction.z, 0.0, 1e-8);
}

TEST_F(SarSensorModel, adjustedPositionVelocity) {
  std::vector<double> adjustments = {1000000.0, 0.2, -10.0,
                                     -20.0,     0.5, 2000000.0};
  csm::EcefCoord sensorPosition = sensorModel->getSensorPosition(0.0);
  csm::EcefVector sensorVelocity = sensorModel->getSensorVelocity(0.0);
  csm::EcefCoord adjPosition =
      sensorModel->getAdjustedSensorPosition(0.0, adjustments);
  csm::EcefVector adjVelocity =
      sensorModel->getAdjustedSensorVelocity(0.0, adjustments);

  EXPECT_NEAR(adjPosition.x, sensorPosition.x + adjustments[0], 1e-2);
  EXPECT_NEAR(adjPosition.y, sensorPosition.y + adjustments[1], 1e-2);
  EXPECT_NEAR(adjPosition.z, sensorPosition.z + adjustments[2], 1e-2);
  EXPECT_NEAR(adjVelocity.x, sensorVelocity.x + adjustments[3], 1e-8);
  EXPECT_NEAR(adjVelocity.y, sensorVelocity.y + adjustments[4], 1e-8);
  EXPECT_NEAR(adjVelocity.z, sensorVelocity.z + adjustments[5], 1e-2);
}
