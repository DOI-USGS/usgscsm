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
  csm::Isd("data/orbitalSar.json");
  UsgsAstroPlugin plugin;
  std::string isdString = plugin.loadImageSupportData(isd);
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
  json oldJson = stateAsJson(modelState);
  json newJson = stateAsJson(newModelState);
  std::vector<std::string> differences = compareJson(oldJson, newJson);
  for (std::string &difference : differences) {
    std::cerr << difference << std::endl;
  }
  EXPECT_TRUE(differences.empty());
}

// Apply the identity transform to a state
TEST_F(SarSensorModel, ApplyTransformToState) {
  std::string modelState = sensorModel->getModelState();

  ale::Rotation r; // identity rotation
  ale::Vec3d t;    // zero translation

  // The input state has some "-0" values which get transformed by
  // applying the identity transform into "0", which results in the
  // state string changing. Hence, for this test, compare the results
  // of applying the identity transform once vs twice, which are the
  // same.

  // First application
  UsgsAstroSarSensorModel::applyTransformToState(r, t, modelState);

  // Second application
  std::string modelState2 = modelState;
  UsgsAstroSarSensorModel::applyTransformToState(r, t, modelState2);

  EXPECT_EQ(modelState, modelState2);
}

TEST_F(SarSensorModel, Center) {
  csm::ImageCoord imagePt(500.0, 500.0);
  csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
  EXPECT_NEAR(groundPt.x, -1520427.2924246688, 1e-3);
  EXPECT_NEAR(groundPt.y, -450240.98096817418, 1e-3);
  EXPECT_NEAR(groundPt.z, 710030.04690435971,  1e-3);
}

TEST_F(SarSensorModel, GroundToImage) {
  csm::EcefCoord groundPt(-1520427.2924246688, -450240.98096817418, 710030.04690435971);
  csm::ImageCoord imagePt = sensorModel->groundToImage(groundPt, 0.001);
  EXPECT_NEAR(imagePt.line, 500.0, 1e-3);
  EXPECT_NEAR(imagePt.samp, 500.0, 1e-3);
}

TEST_F(SarSensorModel, spacecraftPosition) {
  csm::EcefVector position = sensorModel->getSpacecraftPosition(-0.0025);
  EXPECT_NEAR(position.x, 212048065766.25,  1e-8);
  EXPECT_NEAR(position.y, 64616398201.375,  1e-8);
  EXPECT_NEAR(position.z, 488643469101.125, 1e-8);
}

TEST_F(SarSensorModel, spacecraftVelocity) {
  csm::EcefVector velocity = sensorModel->getSensorVelocity(-0.0025);
  EXPECT_NEAR(velocity.x, -427443125.77587891, 1e-8);
  EXPECT_NEAR(velocity.y, -144589932.36868286, 1e-8);
  EXPECT_NEAR(velocity.z, 203454745.77563477,  1e-8);
}

TEST_F(SarSensorModel, getRangeCoefficients) {
  std::vector<double> coeffs = sensorModel->getRangeCoefficients(-0.0025);
  EXPECT_NEAR(coeffs[0], 108115.564453125, 1e-8);
  EXPECT_NEAR(coeffs[1], 28749.89952051267,1e-8);
  EXPECT_NEAR(coeffs[2], -0.25643537028547314, 1e-8);
  EXPECT_NEAR(coeffs[3], 1.0983187306945561e-06, 1e-8);
}

TEST_F(SarSensorModel, computeGroundPartials) {
  csm::EcefCoord groundPt(-1520427.2924246688, -450240.98096817418, 710030.04690435971);

  std::vector<double> partials = sensorModel->computeGroundPartials(groundPt);
  ASSERT_EQ(partials.size(), 6);
  EXPECT_NEAR(partials[0], -0.052475430922178629, 1e-8);
  EXPECT_NEAR(partials[1], -0.015883011992733977, 1e-8);
  EXPECT_NEAR(partials[2], -0.12147919510717504,  1e-8);
  EXPECT_NEAR(partials[3], 0.075598918597946374,  1e-8);
  EXPECT_NEAR(partials[4], 0.16130604972264942,   1e-8);
  EXPECT_NEAR(partials[5], -0.053874104068177074, 1e-8);
}

TEST_F(SarSensorModel, imageToProximateImagingLocus) {
  double precision;
  csm::WarningList warnings;
  csm::EcefLocus locus = sensorModel->imageToProximateImagingLocus(
      csm::ImageCoord(500.0, 500.0),
      csm::EcefCoord(1737287.8590770673, -5403.280537826621,
                     -3849.9796183814506),
      0.001,
      &precision,
      &warnings);
  EXPECT_NEAR(locus.point.x, -1479321.6695206575, 1e-2);
  EXPECT_NEAR(locus.point.y, -511087.07386316231, 1e-2);
  EXPECT_NEAR(locus.point.z, 700287.08593895065, 1e-2);
  EXPECT_NEAR(locus.direction.x, -0.1672556005460577, 1e-5);
  EXPECT_NEAR(locus.direction.y, 0.9842529974996953, 1e-5);
  EXPECT_NEAR(locus.direction.z, -0.057197910790887985, 1e-5);
  EXPECT_LT(precision, 0.001);
  EXPECT_TRUE(warnings.empty());
}

TEST_F(SarSensorModel, imageToRemoteImagingLocus) {
  double precision;
  csm::WarningList warnings;
  csm::ImageCoord imagePt(500.0, 500.0);
  csm::EcefLocus locus = sensorModel->imageToRemoteImagingLocus(
      imagePt,
      0.001,
      &precision,
      &warnings);

  csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
  csm::EcefCoord sensorPt = sensorModel->getSensorPosition(imagePt);

  double lookX = groundPt.x - sensorPt.x;
  double lookY = groundPt.y - sensorPt.y;
  double lookZ = groundPt.z - sensorPt.z;
  double lookMag = sqrt(lookX * lookX + lookY * lookY + lookZ * lookZ);
  lookX /= lookMag;
  lookY /= lookMag;
  lookZ /= lookMag;

  EXPECT_NEAR(locus.direction.x, lookX, 1e-10);
  EXPECT_NEAR(locus.direction.y, lookY, 1e-10);
  EXPECT_NEAR(locus.direction.z, lookZ, 1e-10);

  EXPECT_LT(precision, 0.001);
  EXPECT_TRUE(warnings.empty());
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

TEST_F(SarSensorModel, ReferenceDateTime) {
  std::string date = sensorModel->getReferenceDateAndTime();
  EXPECT_EQ(date, "2020-08-16T08:52:18Z");
}
