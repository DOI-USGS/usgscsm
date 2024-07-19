#define _USE_MATH_DEFINES

#include "Fixtures.h"
#include "UsgsAstroProjectedSensorModel.h"
#include "UsgsAstroPlugin.h"
#include "UsgsAstroPluginSupport.h"

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <math.h>
#include <iostream>

using json = nlohmann::json;

TEST_F(ConstVelocityProjectedSensorModel, ConstructFromState) {
  std::string modelState = sensorModel->getModelState();
  csm::RasterGM * model = getUsgsCsmModelFromState(modelState, UsgsAstroProjectedSensorModel::_SENSOR_MODEL_NAME, NULL);

  // When this is different, the output is very hard to parse
  // TODO implement JSON diff for gtest

  EXPECT_EQ(sensorModel->getModelState(), model->getModelState());
}

TEST_F(ConstVelocityProjectedSensorModel, ReplaceModelState) {
  std::string modelState = sensorModel->getModelState();
  sensorModel->replaceModelState(modelState);

  // When this is different, the output is very hard to parse
  // TODO implement JSON diff for gtest

  EXPECT_EQ(sensorModel->getModelState(), modelState);
}

TEST_F(ConstVelocityProjectedSensorModel, Center) {
  csm::ImageCoord imagePt(8.0, 8.0);
  csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
  EXPECT_NEAR(groundPt.x, 9.8989858043564229, 1e-9);
  EXPECT_NEAR(groundPt.y, -1.4118584049198812, 1e-9);
  EXPECT_NEAR(groundPt.z, 0.12936726636640034, 1e-9);
}

TEST_F(ConstVelocityProjectedSensorModel, Inversion) {
  double achievedPrecision;
  csm::ImageCoord imagePt(8.5, 8);
  csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
  csm::ImageCoord imageReprojPt =
      sensorModel->groundToImage(groundPt, 0.001, &achievedPrecision);

  EXPECT_LT(achievedPrecision, 0.001);
  EXPECT_NEAR(imagePt.line, imageReprojPt.line, 1e-3);
  EXPECT_NEAR(imagePt.samp, imageReprojPt.samp, 1e-3);
}

TEST_F(ConstVelocityProjectedSensorModel, OffBody) {
  csm::ImageCoord imagePt(0.0, 4.0);
  csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
  EXPECT_NEAR(groundPt.x, 9.8988664296556497, 1e-9);
  EXPECT_NEAR(groundPt.y, -1.4125636827694117, 1e-9);
  EXPECT_NEAR(groundPt.z, 0.1307946862733107, 1e-9);
}

TEST_F(ConstVelocityProjectedSensorModel, ProximateImageLocus) {
  csm::ImageCoord imagePt(8.0, 8.0);
  csm::EcefCoord groundPt(10, 2, 1);
  double precision;
  csm::WarningList warnings;
  csm::EcefLocus remoteLocus = sensorModel->imageToRemoteImagingLocus(imagePt);
  csm::EcefLocus locus = sensorModel->imageToProximateImagingLocus(
      imagePt, groundPt, 0.001, &precision, &warnings);
  double locusToGroundX = locus.point.x - groundPt.x;
  double locusToGroundY = locus.point.y - groundPt.y;
  double locusToGroundZ = locus.point.z - groundPt.z;
  EXPECT_NEAR(locus.direction.x, remoteLocus.direction.x, 1e-9);
  EXPECT_NEAR(locus.direction.y, remoteLocus.direction.y, 1e-4);
  EXPECT_NEAR(locus.direction.z, remoteLocus.direction.z, 1e-9);
  EXPECT_NEAR(locusToGroundX * locus.direction.x +
                  locusToGroundY * locus.direction.y +
                  locusToGroundZ * locus.direction.z,
              0.0, 1e-9);
  EXPECT_LT(precision, 0.001);
  EXPECT_TRUE(warnings.empty());
}

TEST_F(ConstVelocityProjectedSensorModel, RemoteImageLocus) {
  csm::ImageCoord imagePt(8.5, 8.0);
  double precision;
  csm::WarningList warnings;
  csm::EcefLocus locus = sensorModel->imageToRemoteImagingLocus(
      imagePt, 0.001, &precision, &warnings);
  csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
  double lookX = groundPt.x - locus.point.x;
  double lookY = groundPt.y - locus.point.y;
  double lookZ = groundPt.z - locus.point.z;
  double lookMag = sqrt(lookX * lookX + lookY * lookY + lookZ * lookZ);
  lookX /= lookMag;
  lookY /= lookMag;
  lookZ /= lookMag;
  EXPECT_NEAR(locus.direction.x, lookX, 1e-9);
  EXPECT_NEAR(locus.direction.y, lookY, 1e-4);
  EXPECT_NEAR(locus.direction.z, lookZ, 1e-6);
  EXPECT_NEAR(locus.point.x, 1000.0, 1e-9);
  EXPECT_NEAR(locus.point.y, 0.0, 1e-9);
  EXPECT_NEAR(locus.point.z, 1.1193790655763731, 1e-9);
  EXPECT_LT(precision, 0.001);
  EXPECT_TRUE(warnings.empty());
}