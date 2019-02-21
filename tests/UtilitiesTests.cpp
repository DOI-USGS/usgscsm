#define _USE_MATH_DEFINES

#include "Fixtures.h"
#include "UsgsAstroPlugin.h"
#include "Utilities.h"

#include <json.hpp>
#include <gtest/gtest.h>

#include <math.h>

using json = nlohmann::json;


TEST(UtilitiesTests, calculateRotationMatrixFromEuler) {
   double euler[3], rotationMatrix[9];
   euler[0] = 0;
   euler[1] = M_PI/2;
   euler[2] = 0;
   calculateRotationMatrixFromEuler(euler, rotationMatrix);

   // EXPECT_NEARs are used here instead of EXPECT_DOUBLE_EQs because index 0 and 8 of the matrix
   // are evaluating to 6.12...e-17. This is too small to be worried about here, but
   // EXPECT_DOUBLE_EQ is too sensitive.
   EXPECT_NEAR(rotationMatrix[0], 0, 1e-8);
   EXPECT_NEAR(rotationMatrix[1], 0, 1e-8);
   EXPECT_NEAR(rotationMatrix[2], 1, 1e-8);
   EXPECT_NEAR(rotationMatrix[3], 0, 1e-8);
   EXPECT_NEAR(rotationMatrix[4], 1, 1e-8);
   EXPECT_NEAR(rotationMatrix[5], 0, 1e-8);
   EXPECT_NEAR(rotationMatrix[6], -1, 1e-8);
   EXPECT_NEAR(rotationMatrix[7], 0, 1e-8);
   EXPECT_NEAR(rotationMatrix[8], 0, 1e-8);
}

TEST(UtilitiesTests, calculateRotationMatrixFromQuaternions) {
   double q[4], rotationMatrix[9];
   q[0] = 0;
   q[1] = -1/sqrt(2);
   q[2] = 0;
   q[3] = 1/sqrt(2);
   calculateRotationMatrixFromQuaternions(q, rotationMatrix);
   EXPECT_DOUBLE_EQ(rotationMatrix[0], 0);
   EXPECT_DOUBLE_EQ(rotationMatrix[1], 0);
   EXPECT_DOUBLE_EQ(rotationMatrix[2], -1);
   EXPECT_DOUBLE_EQ(rotationMatrix[3], 0);
   EXPECT_DOUBLE_EQ(rotationMatrix[4], 1);
   EXPECT_DOUBLE_EQ(rotationMatrix[5], 0);
   EXPECT_DOUBLE_EQ(rotationMatrix[6], 1);
   EXPECT_DOUBLE_EQ(rotationMatrix[7], 0);
   EXPECT_DOUBLE_EQ(rotationMatrix[8], 0);
}

TEST(UtilitiesTests, computeDistortedFocalPlaneCoordinates) {
   double iTransS[] = {0.0, 0.0, 10.0};
   double iTransL[] = {0.0, 10.0, 0.0};
   std::tuple<double, double> natFocalPlane;
   computeDistortedFocalPlaneCoordinates(0.5, 4, 8, 0.5, 1, 0, iTransS, iTransL, natFocalPlane);
   EXPECT_DOUBLE_EQ(std::get<0> (natFocalPlane), 0);
   EXPECT_DOUBLE_EQ(std::get<1> (natFocalPlane), -0.4);
}

TEST(UtilitiesTests, createCameraLookVector) {
  double cameraLook[3];
  createCameraLookVector(0, -0.4, 1, 50, cameraLook);
  EXPECT_NEAR(cameraLook[0], 0, 1e-8);
  EXPECT_NEAR(cameraLook[1], 0.007999744, 1e-8);
  EXPECT_NEAR(cameraLook[2], -0.999968001, 1e-8);
}
