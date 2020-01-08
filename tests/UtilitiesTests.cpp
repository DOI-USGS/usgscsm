#define _USE_MATH_DEFINES

#include "Fixtures.h"
#include "UsgsAstroPlugin.h"
#include "Utilities.h"

#include <json/json.hpp>
#include <gtest/gtest.h>

#include <math.h>

using json = nlohmann::json;

TEST(UtilitiesTests, distanceToLine) {
  // Line passing through (0, 1) and (2,2)
  double a = -1.0 / sqrt(5);
  double b =  2.0 / sqrt(5);
  double c = -2.0 / sqrt(5);
  // Point on line
  EXPECT_NEAR(distanceToLine(4.0, 3.0, a, b, c), 0, 1e-12);
  // Point above line
  EXPECT_DOUBLE_EQ(distanceToLine(1.0, 4.0, a, b, c), sqrt(5));
  // Point below line
  EXPECT_DOUBLE_EQ(distanceToLine(4.5, 2.0, a, b, c), -sqrt(5) / 2.0);
}

TEST(UtilitiesTests, line) {
  double x1 = 0.0;
  double y1 = 1.0;
  double x2 = 2.0;
  double y2 = 2.0;
  double a, b, c;
  line(x1, y1, x2, y2, a, b, c);
  EXPECT_DOUBLE_EQ(a, -1.0 / sqrt(5));
  EXPECT_DOUBLE_EQ(b,  2.0 / sqrt(5));
  EXPECT_DOUBLE_EQ(c, -2.0 / sqrt(5));
}

TEST(UtilitiesTests, distanceToPlane) {
  // Plane passing through (0, 0, 1), (0, 2, 0) and (3, 0, 0)
  double a =   2.0 / 7.0;
  double b =   3.0 / 7.0;
  double c =   6.0 / 7.0;
  double d = -12.0 / 7.0;
  // Points on plane
  EXPECT_NEAR(distanceToPlane(0.0, 0.0, 2.0, a, b, c, d), 0.0, 1e-12);
  EXPECT_NEAR(distanceToPlane(0.0, 4.0, 0.0, a, b, c, d), 0.0, 1e-12);
  EXPECT_NEAR(distanceToPlane(6.0, 0.0, 0.0, a, b, c, d), 0.0, 1e-12);
  // Points off plane
  EXPECT_DOUBLE_EQ(distanceToPlane(0.0, 0.0, 0.0, a, b, c, d), 12.0 / 7.0);
  EXPECT_DOUBLE_EQ(distanceToPlane(0.0, 0.0, 1.0, a, b, c, d),  6.0 / 7.0);
  EXPECT_DOUBLE_EQ(distanceToPlane(0.0, 2.0, 0.0, a, b, c, d),  6.0 / 7.0);
  EXPECT_DOUBLE_EQ(distanceToPlane(3.0, 0.0, 0.0, a, b, c, d),  6.0 / 7.0);
}

TEST(UtilitiesTests, plane) {
  double x0 = 0.0;
  double y0 = 0.0;
  double z0 = 2.0;
  double v1x =  0.0;
  double v1y =  4.0;
  double v1z = -2.0;
  double v2x =  6.0;
  double v2y =  0.0;
  double v2z = -2.0;
  double a, b, c, d;
  plane(x0, y0, z0, v1x, v1y, v1z, v2x, v2y, v2z, a, b, c, d);
  EXPECT_DOUBLE_EQ(a, -2.0 / 7.0);
  EXPECT_DOUBLE_EQ(b, -3.0 / 7.0);
  EXPECT_DOUBLE_EQ(c, -6.0 / 7.0);
  EXPECT_DOUBLE_EQ(d, 12.0 / 7.0);
}

TEST(UtilitiesTests, fitLinearApproximation) {
  std::vector<double> x = {0, 1, 2, 3,  4, 5, 6, 7, 8, 9, 10};
  std::vector<double> y = {1, 0, 5, 7, -2, 1, 2, 2, 1, 0,  1};

  std::vector<int> nodes = fitLinearApproximation(x, y, 1.0);
  ASSERT_EQ(nodes.size(), 7);
  EXPECT_EQ(nodes[0],  0);
  EXPECT_EQ(nodes[1],  1);
  EXPECT_EQ(nodes[2],  3);
  EXPECT_EQ(nodes[3],  4);
  EXPECT_EQ(nodes[4],  6);
  EXPECT_EQ(nodes[5],  9);
  EXPECT_EQ(nodes[6], 10);
}


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
   double undistortedFocalPlaneX, undistortedFocalPlaneY;
   computeDistortedFocalPlaneCoordinates(
         0.5, 4.0,
         8.0, 0.5,
         1.0, 1.0,
         0.0, 0.0,
         iTransS, iTransL,
         undistortedFocalPlaneX, undistortedFocalPlaneY);
   EXPECT_DOUBLE_EQ(undistortedFocalPlaneX, 0);
   EXPECT_DOUBLE_EQ(undistortedFocalPlaneY, -0.4);
}

TEST(UtilitiesTests, computeDistortedFocalPlaneCoordinatesSumming) {
   double iTransS[] = {0.0, 0.0, 10.0};
   double iTransL[] = {0.0, 10.0, 0.0};
   double undistortedFocalPlaneX, undistortedFocalPlaneY;
   computeDistortedFocalPlaneCoordinates(
         2.0, 4.0,
         8.0, 8.0,
         2.0, 2.0,
         0.0, 0.0,
         iTransS, iTransL,
         undistortedFocalPlaneX, undistortedFocalPlaneY);
   EXPECT_DOUBLE_EQ(undistortedFocalPlaneX, -0.4);
   EXPECT_DOUBLE_EQ(undistortedFocalPlaneY, 0);
}

TEST(UtilitiesTests, computeDistortedFocalPlaneCoordinatesAffine) {
   double iTransS[] = {-10.0, 0.0, 0.1};
   double iTransL[] = {10.0, -0.1, 0.0};
   double undistortedFocalPlaneX, undistortedFocalPlaneY;
   computeDistortedFocalPlaneCoordinates(
         11.0, -9.0,
         0.0, 0.0,
         1.0, 1.0,
         0.0, 0.0,
         iTransS, iTransL,
         undistortedFocalPlaneX, undistortedFocalPlaneY);
   EXPECT_NEAR(undistortedFocalPlaneX, -10.0, 1e-12);
   EXPECT_NEAR(undistortedFocalPlaneY, 10.0, 1e-12);
}


TEST(UtilitiesTests, computeDistortedFocalPlaneCoordinatesStart) {
   double iTransS[] = {0.0, 0.0, 10.0};
   double iTransL[] = {0.0, 10.0, 0.0};
   double undistortedFocalPlaneX, undistortedFocalPlaneY;
   computeDistortedFocalPlaneCoordinates(
         2.0, 4.0,
         8.0, 8.0,
         1.0, 1.0,
         2.0, 1.0,
         iTransS, iTransL,
         undistortedFocalPlaneX, undistortedFocalPlaneY);
   EXPECT_DOUBLE_EQ(undistortedFocalPlaneX, -0.5);
   EXPECT_DOUBLE_EQ(undistortedFocalPlaneY, -0.2);
}

TEST(UtilitiesTests, computePixel) {
   double iTransS[] = {0.0, 0.0, 10.0};
   double iTransL[] = {0.0, 10.0, 0.0};
   double line, sample;
   computePixel(
         0.0, -0.4,
         8.0, 0.5,
         1.0, 1.0,
         0.0, 0.0,
         iTransS, iTransL,
         line, sample);
   EXPECT_DOUBLE_EQ(line, 0.5);
   EXPECT_DOUBLE_EQ(sample, 4.0);
}

TEST(UtilitiesTests, computePixelSumming) {
   double iTransS[] = {0.0, 0.0, 10.0};
   double iTransL[] = {0.0, 10.0, 0.0};
   double line, sample;
   computePixel(
         -0.4, 0.0,
         8.0, 8.0,
         2.0, 2.0,
         0.0, 0.0,
         iTransS, iTransL,
         line, sample);
   EXPECT_DOUBLE_EQ(line, 2.0);
   EXPECT_DOUBLE_EQ(sample, 4.0);
}

TEST(UtilitiesTests, computePixelStart) {
   double iTransS[] = {0.0, 0.0, 10.0};
   double iTransL[] = {0.0, 10.0, 0.0};
   double line, sample;
   computePixel(
         -0.5, -0.2,
         8.0, 8.0,
         1.0, 1.0,
         2.0, 1.0,
         iTransS, iTransL,
         line, sample);
   EXPECT_DOUBLE_EQ(line, 2.0);
   EXPECT_DOUBLE_EQ(sample, 4.0);
}

TEST(UtilitiesTests, computePixelStartSumming) {
   double iTransS[] = {0.0, 0.0, 10.0};
   double iTransL[] = {0.0, 10.0, 0.0};
   double line, sample;
   computePixel(
         -0.5, -0.2,
         8.0, 8.0,
         2.0, 4.0,
         2.0, 1.0,
         iTransS, iTransL,
         line, sample);
   EXPECT_DOUBLE_EQ(line, 0.5);
   EXPECT_DOUBLE_EQ(sample, 2.0);
}

TEST(UtilitiesTests, createCameraLookVector) {
  double cameraLook[3];
  createCameraLookVector(0, -0.4, 1, 50, cameraLook);
  EXPECT_NEAR(cameraLook[0], 0, 1e-8);
  EXPECT_NEAR(cameraLook[1], 0.007999744, 1e-8);
  EXPECT_NEAR(cameraLook[2], -0.999968001, 1e-8);
}

TEST(UtilitiesTests, lagrangeInterp1Point) {
  int numTime = 1;
  std::vector<double> singlePoint = {1};
  std::vector<double> interpPoint = {0};
  double startTime = 0;
  double delTime = 1;
  double time = 0;
  int vectorLength = 1;
  int order = 8;

  try {
     lagrangeInterp(numTime, &singlePoint[0], startTime, delTime,
                    time, vectorLength, order, &interpPoint[0]);
     FAIL() << "Expected an error";
  }
  catch(csm::Error &e) {
     EXPECT_EQ(e.getError(), csm::Error::INDEX_OUT_OF_RANGE);
  }
  catch(...) {
     FAIL() << "Expected csm INDEX_OUT_OF_RANGE error";
  }
}

TEST(UtilitiesTests, lagrangeInterp2ndOrder) {
  int numTime = 8;
  std::vector<double> interpValues = {1, 2, 4, 8, 16, 32, 64, 128};
  std::vector<double> outputValue = {0};
  double startTime = 0;
  double delTime = 1;
  double time = 3.5;
  int vectorLength = 1;
  int order = 2;

  lagrangeInterp(numTime, &interpValues[0], startTime, delTime,
                    time, vectorLength, order, &outputValue[0]);
  EXPECT_DOUBLE_EQ(outputValue[0], 24.0 / 2.0);
}

TEST(UtilitiesTests, lagrangeInterp4thOrder) {
  int numTime = 8;
  std::vector<double> interpValues = {1, 2, 4, 8, 16, 32, 64, 128};
  std::vector<double> outputValue = {0};
  double startTime = 0;
  double delTime = 1;
  double time = 3.5;
  int vectorLength = 1;
  int order = 4;

  lagrangeInterp(numTime, &interpValues[0], startTime, delTime,
                    time, vectorLength, order, &outputValue[0]);
  EXPECT_DOUBLE_EQ(outputValue[0], 180.0 / 16.0);
}

TEST(UtilitiesTests, lagrangeInterp6thOrder) {
  int numTime = 8;
  std::vector<double> interpValues = {1, 2, 4, 8, 16, 32, 64, 128};
  std::vector<double> outputValue = {0};
  double startTime = 0;
  double delTime = 1;
  double time = 3.5;
  int vectorLength = 1;
  int order = 6;

  lagrangeInterp(numTime, &interpValues[0], startTime, delTime,
                    time, vectorLength, order, &outputValue[0]);
  EXPECT_DOUBLE_EQ(outputValue[0], 2898.0 / 256.0);
}

TEST(UtilitiesTests, lagrangeInterp8thOrder) {
  int numTime = 8;
  std::vector<double> interpValues = {1, 2, 4, 8, 16, 32, 64, 128};
  std::vector<double> outputValue = {0};
  double startTime = 0;
  double delTime = 1;
  double time = 3.5;
  int vectorLength = 1;
  int order = 8;

  lagrangeInterp(numTime, &interpValues[0], startTime, delTime,
                    time, vectorLength, order, &outputValue[0]);
  EXPECT_DOUBLE_EQ(outputValue[0], 23169.0 / 2048.0);
}

TEST(UtilitiesTests, lagrangeInterpReduced2ndOrder) {
  int numTime = 8;
  std::vector<double> interpValues = {1, 2, 4, 8, 16, 32, 64, 128};
  std::vector<double> outputValue = {0};
  double startTime = 0;
  double delTime = 1;
  double time = 0.5;
  int vectorLength = 1;
  int order = 8;

  lagrangeInterp(numTime, &interpValues[0], startTime, delTime,
                    time, vectorLength, order, &outputValue[0]);
  EXPECT_DOUBLE_EQ(outputValue[0], 3.0 / 2.0);

  time = 6.5;
  lagrangeInterp(numTime, &interpValues[0], startTime, delTime,
                    time, vectorLength, order, &outputValue[0]);
  EXPECT_DOUBLE_EQ(outputValue[0], 192.0 / 2.0);
}

TEST(UtilitiesTests, lagrangeInterpReduced4thOrder) {
  int numTime = 8;
  std::vector<double> interpValues = {1, 2, 4, 8, 16, 32, 64, 128};
  std::vector<double> outputValue = {0};
  double startTime = 0;
  double delTime = 1;
  double time = 1.5;
  int vectorLength = 1;
  int order = 8;

  lagrangeInterp(numTime, &interpValues[0], startTime, delTime,
                    time, vectorLength, order, &outputValue[0]);
  EXPECT_DOUBLE_EQ(outputValue[0], 45.0 / 16.0);

  time = 5.5;
  lagrangeInterp(numTime, &interpValues[0], startTime, delTime,
                    time, vectorLength, order, &outputValue[0]);
  EXPECT_DOUBLE_EQ(outputValue[0], 720.0 / 16.0);
}

TEST(UtilitiesTests, lagrangeInterpReduced6thOrder) {
  int numTime = 8;
  std::vector<double> interpValues = {1, 2, 4, 8, 16, 32, 64, 128};
  std::vector<double> outputValue = {0};
  double startTime = 0;
  double delTime = 1;
  double time = 2.5;
  int vectorLength = 1;
  int order = 8;

  lagrangeInterp(numTime, &interpValues[0], startTime, delTime,
                    time, vectorLength, order, &outputValue[0]);
  EXPECT_DOUBLE_EQ(outputValue[0], 1449.0 / 256.0);

  time = 4.5;
  lagrangeInterp(numTime, &interpValues[0], startTime, delTime,
                    time, vectorLength, order, &outputValue[0]);
  EXPECT_DOUBLE_EQ(outputValue[0], 5796.0 / 256.0);
}

TEST(UtilitiesTests, lagrangeInterp2D) {
  int numTime = 2;
  std::vector<double> interpValues = {0, 1, 1, 2};
  std::vector<double> outputValue = {0, 0};
  double startTime = 0;
  double delTime = 1;
  double time = 0.5;
  int vectorLength = 2;
  int order = 2;

  lagrangeInterp(numTime, &interpValues[0], startTime, delTime,
                    time, vectorLength, order, &outputValue[0]);
  EXPECT_DOUBLE_EQ(outputValue[0], 0.5);
  EXPECT_DOUBLE_EQ(outputValue[1], 1.5);
}
