#define _USE_MATH_DEFINES

#include "Fixtures.h"
#include "UsgsAstroPlugin.h"
#include "Utilities.h"

#include <json/json.hpp>
#include <gtest/gtest.h>

#include <math.h>
#include <stdexcept>
#include <functional>

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

TEST(UtilitiesTests, brentRoot) {
  std::function<double(double)> testPoly = [](double x) {
    return (x - 2) * (x + 1) * (x + 7);
  };

  EXPECT_NEAR(brentRoot(1.0, 3.0, testPoly, 1e-10), 2.0, 1e-10);
  EXPECT_NEAR(brentRoot(0.0, -3.0, testPoly, 1e-10), -1.0, 1e-10);
  EXPECT_THROW(brentRoot(-3.0, 3.0, testPoly), std::invalid_argument);
}

TEST(UtilitiesTests, secantRoot) {
  std::function<double(double)> testPoly = [](double x) {
    return (x - 2) * (x + 1) * (x + 7);
  };
  EXPECT_NEAR(secantRoot(1.0, 3.0, testPoly, 1e-10), 2.0, 1e-10);
  EXPECT_NEAR(secantRoot(0.0, -3.0, testPoly, 1e-10), -1.0, 1e-10);
  EXPECT_THROW(secantRoot(-1000.0, 1000.0, testPoly), csm::Error);
}

TEST(UtilitiesTests, vectorProduct) {
  csm::EcefVector vec(1.0, 2.0, 3.0);
  csm::EcefVector leftVec = 2.0 * vec;
  csm::EcefVector rightVec = vec * 2;
  EXPECT_DOUBLE_EQ(leftVec.x, 2.0);
  EXPECT_DOUBLE_EQ(leftVec.y, 4.0);
  EXPECT_DOUBLE_EQ(leftVec.z, 6.0);
  EXPECT_DOUBLE_EQ(rightVec.x, 2.0);
  EXPECT_DOUBLE_EQ(rightVec.y, 4.0);
  EXPECT_DOUBLE_EQ(rightVec.z, 6.0);
}

TEST(UtilitiesTests, vectorDivision) {
  csm::EcefVector vec(2.0, 4.0, 6.0);
  csm::EcefVector divVec = vec / 2.0;
  EXPECT_DOUBLE_EQ(divVec.x, 1.0);
  EXPECT_DOUBLE_EQ(divVec.y, 2.0);
  EXPECT_DOUBLE_EQ(divVec.z, 3.0);
}

TEST(UtilitiesTests, vectorAddition) {
  csm::EcefVector vec1(2.0, 4.0, 6.0);
  csm::EcefVector vec2(1.0, 2.0, 3.0);
  csm::EcefVector sumVec = vec1 + vec2;
  EXPECT_DOUBLE_EQ(sumVec.x, 3.0);
  EXPECT_DOUBLE_EQ(sumVec.y, 6.0);
  EXPECT_DOUBLE_EQ(sumVec.z, 9.0);
}

TEST(UtilitiesTests, vectorSubtraction) {
  csm::EcefVector vec1(2.0, 4.0, 6.0);
  csm::EcefVector vec2(1.0, 2.0, 3.0);
  csm::EcefVector diffVec = vec1 - vec2;
  EXPECT_DOUBLE_EQ(diffVec.x, 1.0);
  EXPECT_DOUBLE_EQ(diffVec.y, 2.0);
  EXPECT_DOUBLE_EQ(diffVec.z, 3.0);
}

TEST(UtilitiesTests, vectorDot) {
  csm::EcefVector unitX(1.0, 0.0, 0.0);
  csm::EcefVector unitY(0.0, 1.0, 0.0);
  csm::EcefVector unitZ(0.0, 0.0, 1.0);
  csm::EcefVector testVec(1.0, 2.0, 3.0);
  EXPECT_DOUBLE_EQ(dot(testVec, unitX), 1.0);
  EXPECT_DOUBLE_EQ(dot(testVec, unitY), 2.0);
  EXPECT_DOUBLE_EQ(dot(testVec, unitZ), 3.0);
}

TEST(UtilitiesTests, vectorCross) {
  csm::EcefVector unitX(1.0, 0.0, 0.0);
  csm::EcefVector unitY(0.0, 1.0, 0.0);
  csm::EcefVector unitZ(0.0, 0.0, 1.0);

  csm::EcefVector unitXY = cross(unitX, unitY);
  EXPECT_DOUBLE_EQ(unitXY.x, 0.0);
  EXPECT_DOUBLE_EQ(unitXY.y, 0.0);
  EXPECT_DOUBLE_EQ(unitXY.z, 1.0);


  csm::EcefVector unitYX = cross(unitY, unitX);
  EXPECT_DOUBLE_EQ(unitYX.x, 0.0);
  EXPECT_DOUBLE_EQ(unitYX.y, 0.0);
  EXPECT_DOUBLE_EQ(unitYX.z, -1.0);

  csm::EcefVector unitXZ = cross(unitX, unitZ);
  EXPECT_DOUBLE_EQ(unitXZ.x, 0.0);
  EXPECT_DOUBLE_EQ(unitXZ.y, -1.0);
  EXPECT_DOUBLE_EQ(unitXZ.z, 0.0);


  csm::EcefVector unitZX = cross(unitZ, unitX);
  EXPECT_DOUBLE_EQ(unitZX.x, 0.0);
  EXPECT_DOUBLE_EQ(unitZX.y, 1.0);
  EXPECT_DOUBLE_EQ(unitZX.z, 0.0);

  csm::EcefVector unitYZ = cross(unitY, unitZ);
  EXPECT_DOUBLE_EQ(unitYZ.x, 1.0);
  EXPECT_DOUBLE_EQ(unitYZ.y, 0.0);
  EXPECT_DOUBLE_EQ(unitYZ.z, 0.0);


  csm::EcefVector unitZY = cross(unitZ, unitY);
  EXPECT_DOUBLE_EQ(unitZY.x, -1.0);
  EXPECT_DOUBLE_EQ(unitZY.y, 0.0);
  EXPECT_DOUBLE_EQ(unitZY.z, 0.0);
}

TEST(UtilitiesTests, vectorNorm) {
  csm::EcefVector testVec(1.0, 2.0, 3.0);
  EXPECT_DOUBLE_EQ(norm(testVec), sqrt(14.0));
  csm::EcefVector normVec = normalized(testVec);
  EXPECT_DOUBLE_EQ(normVec.x, 1.0 / sqrt(14.0));
  EXPECT_DOUBLE_EQ(normVec.y, 2.0 / sqrt(14.0));
  EXPECT_DOUBLE_EQ(normVec.z, 3.0 / sqrt(14.0));
}

TEST(UtilitiesTests, vectorProjection) {
  csm::EcefVector unitX(1.0, 0.0, 0.0);
  csm::EcefVector unitY(0.0, 1.0, 0.0);
  csm::EcefVector unitZ(0.0, 0.0, 1.0);

  csm::EcefVector testVec(1.0, 2.0, 3.0);

  csm::EcefVector projX = projection(testVec, unitX);
  EXPECT_DOUBLE_EQ(projX.x, 1.0);
  EXPECT_DOUBLE_EQ(projX.y, 0.0);
  EXPECT_DOUBLE_EQ(projX.z, 0.0);

  csm::EcefVector rejectX = rejection(testVec, unitX);
  EXPECT_DOUBLE_EQ(rejectX.x, 0.0);
  EXPECT_DOUBLE_EQ(rejectX.y, 2.0);
  EXPECT_DOUBLE_EQ(rejectX.z, 3.0);

  csm::EcefVector projY = projection(testVec, unitY);
  EXPECT_DOUBLE_EQ(projY.x, 0.0);
  EXPECT_DOUBLE_EQ(projY.y, 2.0);
  EXPECT_DOUBLE_EQ(projY.z, 0.0);

  csm::EcefVector rejectY = rejection(testVec, unitY);
  EXPECT_DOUBLE_EQ(rejectY.x, 1.0);
  EXPECT_DOUBLE_EQ(rejectY.y, 0.0);
  EXPECT_DOUBLE_EQ(rejectY.z, 3.0);

  csm::EcefVector projZ = projection(testVec, unitZ);
  EXPECT_DOUBLE_EQ(projZ.x, 0.0);
  EXPECT_DOUBLE_EQ(projZ.y, 0.0);
  EXPECT_DOUBLE_EQ(projZ.z, 3.0);

  csm::EcefVector rejectZ = rejection(testVec, unitZ);
  EXPECT_DOUBLE_EQ(rejectZ.x, 1.0);
  EXPECT_DOUBLE_EQ(rejectZ.y, 2.0);
  EXPECT_DOUBLE_EQ(rejectZ.z, 0.0);
}
