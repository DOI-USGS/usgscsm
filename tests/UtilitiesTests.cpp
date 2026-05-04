#define _USE_MATH_DEFINES

#include "Fixtures.h"
#include "UsgsAstroPlugin.h"
#include "Utilities.h"

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <math.h>
#include <functional>
#include <stdexcept>

using json = nlohmann::json;

TEST(UtilitiesTests, calculateRotationMatrixFromEuler) {
  double euler[3], rotationMatrix[9];
  euler[0] = 0;
  euler[1] = M_PI / 2;
  euler[2] = 0;
  calculateRotationMatrixFromEuler(euler, rotationMatrix);

  // EXPECT_NEARs are used here instead of EXPECT_DOUBLE_EQs because index 0 and
  // 8 of the matrix are evaluating to 6.12...e-17. This is too small to be
  // worried about here, but EXPECT_DOUBLE_EQ is too sensitive.
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
  q[1] = -1 / sqrt(2);
  q[2] = 0;
  q[3] = 1 / sqrt(2);
  calculateRotationMatrixFromQuaternions(q, rotationMatrix);
  EXPECT_NEAR(rotationMatrix[0], 0, 1e-15);
  EXPECT_NEAR(rotationMatrix[1], 0, 1e-15);
  EXPECT_NEAR(rotationMatrix[2], -1, 1e-15);
  EXPECT_NEAR(rotationMatrix[3], 0, 1e-15);
  EXPECT_NEAR(rotationMatrix[4], 1, 1e-15);
  EXPECT_NEAR(rotationMatrix[5], 0, 1e-15);
  EXPECT_NEAR(rotationMatrix[6], 1, 1e-15);
  EXPECT_NEAR(rotationMatrix[7], 0, 1e-15);
  EXPECT_NEAR(rotationMatrix[8], 0, 1e-15);
}

TEST(UtilitiesTests, computeDistortedFocalPlaneCoordinates) {
  double iTransS[] = {0.0, 0.0, 10.0};
  double iTransL[] = {0.0, 10.0, 0.0};
  double undistortedFocalPlaneX, undistortedFocalPlaneY;
  computeDistortedFocalPlaneCoordinates(
      0.5, 4.0, 8.0, 0.5, 1.0, 1.0, 0.0, 0.0, iTransS, iTransL,
      undistortedFocalPlaneX, undistortedFocalPlaneY);
  EXPECT_DOUBLE_EQ(undistortedFocalPlaneX, 0);
  EXPECT_DOUBLE_EQ(undistortedFocalPlaneY, -0.4);
}

TEST(UtilitiesTests, computeDistortedFocalPlaneCoordinatesSumming) {
  double iTransS[] = {0.0, 0.0, 10.0};
  double iTransL[] = {0.0, 10.0, 0.0};
  double undistortedFocalPlaneX, undistortedFocalPlaneY;
  computeDistortedFocalPlaneCoordinates(
      2.0, 4.0, 8.0, 8.0, 2.0, 2.0, 0.0, 0.0, iTransS, iTransL,
      undistortedFocalPlaneX, undistortedFocalPlaneY);
  EXPECT_DOUBLE_EQ(undistortedFocalPlaneX, -0.4);
  EXPECT_DOUBLE_EQ(undistortedFocalPlaneY, 0);
}

TEST(UtilitiesTests, computeDistortedFocalPlaneCoordinatesAffine) {
  double iTransS[] = {-10.0, 0.0, 0.1};
  double iTransL[] = {10.0, -0.1, 0.0};
  double undistortedFocalPlaneX, undistortedFocalPlaneY;
  computeDistortedFocalPlaneCoordinates(
      11.0, -9.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, iTransS, iTransL,
      undistortedFocalPlaneX, undistortedFocalPlaneY);
  EXPECT_NEAR(undistortedFocalPlaneX, -10.0, 1e-12);
  EXPECT_NEAR(undistortedFocalPlaneY, 10.0, 1e-12);
}

TEST(UtilitiesTests, computeDistortedFocalPlaneCoordinatesStart) {
  double iTransS[] = {0.0, 0.0, 10.0};
  double iTransL[] = {0.0, 10.0, 0.0};
  double undistortedFocalPlaneX, undistortedFocalPlaneY;
  computeDistortedFocalPlaneCoordinates(
      2.0, 4.0, 8.0, 8.0, 1.0, 1.0, 2.0, 1.0, iTransS, iTransL,
      undistortedFocalPlaneX, undistortedFocalPlaneY);
  EXPECT_DOUBLE_EQ(undistortedFocalPlaneX, -0.5);
  EXPECT_DOUBLE_EQ(undistortedFocalPlaneY, -0.2);
}

TEST(UtilitiesTests, removeJitter) {
  std::vector<double> lineJit = {2.0, 3.0, 4.0};
  std::vector<double> sampleJit = {-10.0, -20.0, -30.0};
  std::vector<double> lineTimes = {0.0, 1.0, 2.0, 3.0, 4.0};

  double testline, testSample;
  double dejitteredLine, dejitteredSample;
  double lineJitter, sampleJitter;

  // Test at t=0, this should do nothing
  testline = 1;
  testSample = 0;
  dejitteredLine = 10;
  dejitteredSample = 10;
  removeJitter(testline, testSample, lineJit, sampleJit, lineTimes, dejitteredLine, dejitteredSample);
  EXPECT_DOUBLE_EQ(dejitteredLine, testline);
  EXPECT_DOUBLE_EQ(dejitteredSample, testSample);

  // Test at t=1 to check proper coefficients
  testline = 2;
  testSample = 5.0;
  dejitteredLine = 10;
  dejitteredSample = 10;
  removeJitter(testline, testSample, lineJit, sampleJit, lineTimes, dejitteredLine, dejitteredSample);
  EXPECT_DOUBLE_EQ(dejitteredLine, testline - (2.0 + 3.0 + 4.0));
  EXPECT_DOUBLE_EQ(dejitteredSample, testSample + (10.0 + 20.0 + 30.0));

  // Test at t=2 to check exponents
  testline = 3;
  testSample = 5.0;
  dejitteredLine = 10;
  dejitteredSample = 10;
  removeJitter(testline, testSample, lineJit, sampleJit, lineTimes, dejitteredLine, dejitteredSample);
  EXPECT_DOUBLE_EQ(dejitteredLine, testline - (8 * 2.0 + 4 * 3.0 + 2 * 4.0));
  EXPECT_DOUBLE_EQ(dejitteredSample, testSample + (8 * 10.0 + 4 * 20.0 + 2 * 30.0));

  // Test at extreme negative line to ensure it bounds to the image
  testline = 1;
  testSample = 0;
  dejitteredLine = 10;
  dejitteredSample = 10;
  removeJitter(testline, testSample, lineJit, sampleJit, lineTimes, dejitteredLine, dejitteredSample);
  lineJitter = testline - dejitteredLine;
  sampleJitter = testSample - dejitteredSample;
  testline = -100;
  dejitteredLine = 10;
  dejitteredSample = 10;
  removeJitter(testline, testSample, lineJit, sampleJit, lineTimes, dejitteredLine, dejitteredSample);
  EXPECT_DOUBLE_EQ(testline - dejitteredLine, lineJitter);
  EXPECT_DOUBLE_EQ(testSample - dejitteredSample, sampleJitter);

  // Test at too large of a line to ensure it bounds to the image
  testline = 5.0;
  testSample = 0;
  dejitteredLine = 10;
  dejitteredSample = 10;
  removeJitter(testline, testSample, lineJit, sampleJit, lineTimes, dejitteredLine, dejitteredSample);
  lineJitter = testline - dejitteredLine;
  sampleJitter = testSample - dejitteredSample;
  testline = 100;
  dejitteredLine = 10;
  dejitteredSample = 10;
  removeJitter(testline, testSample, lineJit, sampleJit, lineTimes, dejitteredLine, dejitteredSample);
  EXPECT_DOUBLE_EQ(testline - dejitteredLine, lineJitter);
  EXPECT_DOUBLE_EQ(testSample - dejitteredSample, sampleJitter);

  // Test at fractional lines to check rounding
  testline = 1.0;
  testSample = 5.0;
  dejitteredLine = 10;
  dejitteredSample = 10;
  removeJitter(testline, testSample, lineJit, sampleJit, lineTimes, dejitteredLine, dejitteredSample);
  lineJitter = testline - dejitteredLine;
  sampleJitter = testSample - dejitteredSample;
  testline = 0.75;
  dejitteredLine = 10;
  dejitteredSample = 10;
  removeJitter(testline, testSample, lineJit, sampleJit, lineTimes, dejitteredLine, dejitteredSample);
  EXPECT_DOUBLE_EQ(testline - dejitteredLine, lineJitter);
  EXPECT_DOUBLE_EQ(testSample - dejitteredSample, sampleJitter);
  testline = 1.35;
  dejitteredLine = 10;
  dejitteredSample = 10;
  removeJitter(testline, testSample, lineJit, sampleJit, lineTimes, dejitteredLine, dejitteredSample);
  EXPECT_DOUBLE_EQ(testline - dejitteredLine, lineJitter);
  EXPECT_DOUBLE_EQ(testSample - dejitteredSample, sampleJitter);
}

TEST(UtilitiesTests, removeJitterErrors) {
  std::vector<double> lineJit = {2.0, 3.0, 4.0};
  std::vector<double> shortLineJit = {2.0, 3.0};
  std::vector<double> sampleJit = {-10.0, -20.0, -30.0};
  std::vector<double> longSampleJit = {-10.0, -20.0, -30.0, -40.0};
  std::vector<double> lineTimes = {0.0, 1.0, 2.0, 3.0, 4.0};
  std::vector<double> emptyLineTimes;
  double testLine = 4;
  double testSample = 15;
  double dejitteredLine, dejitteredSample;

  EXPECT_THROW(
      removeJitter(testLine, testSample, shortLineJit, sampleJit, lineTimes, dejitteredLine, dejitteredSample),
      csm::Error);
  EXPECT_THROW(
      removeJitter(testLine, testSample, lineJit, longSampleJit, lineTimes, dejitteredLine, dejitteredSample),
      csm::Error);
  EXPECT_THROW(
      removeJitter(testLine, testSample, lineJit, sampleJit, emptyLineTimes, dejitteredLine, dejitteredSample),
      csm::Error);
}

TEST(UtilitiesTests, addJitter) {
  // The line jitter must be sufficiently small to make a unique solution.
  // Extremely high jitter can result in multiple solutions to the inversion.
  std::vector<double> lineJit = {0.01, 0.02, 0.03};
  std::vector<double> sampleJit = {-0.01, -0.02, -0.03};
  std::vector<double> lineTimes = {0.0, 1.0, 2.0, 3.0, 4.0};

  double testline = 4;
  double testSample = 15.0;
  double dejitteredLine = 10;
  double dejitteredSample = 10;
  removeJitter(
      testline, testSample,
      lineJit, sampleJit, lineTimes,
      dejitteredLine, dejitteredSample);
  double jitteredLine = 10;
  double jitteredSample = 10;
  double tolerance = 1e-7;
  int maxIts = 30;
  addJitter(
    dejitteredLine, dejitteredSample,
    tolerance, maxIts,
    lineJit, sampleJit, lineTimes,
    jitteredLine, jitteredSample);

  EXPECT_LE(fabs(jitteredLine - testline), tolerance);
  EXPECT_LE(fabs(jitteredSample - testSample), tolerance);
}

TEST(UtilitiesTests, computePixel) {
  double iTransS[] = {0.0, 0.0, 10.0};
  double iTransL[] = {0.0, 10.0, 0.0};
  double line, sample;
  computePixel(0.0, -0.4, 8.0, 0.5, 1.0, 1.0, 0.0, 0.0, iTransS, iTransL, line,
               sample);
  EXPECT_DOUBLE_EQ(line, 0.5);
  EXPECT_DOUBLE_EQ(sample, 4.0);
}

TEST(UtilitiesTests, computePixelSumming) {
  double iTransS[] = {0.0, 0.0, 10.0};
  double iTransL[] = {0.0, 10.0, 0.0};
  double line, sample;
  computePixel(-0.4, 0.0, 8.0, 8.0, 2.0, 2.0, 0.0, 0.0, iTransS, iTransL, line,
               sample);
  EXPECT_DOUBLE_EQ(line, 2.0);
  EXPECT_DOUBLE_EQ(sample, 4.0);
}

TEST(UtilitiesTests, computePixelStart) {
  double iTransS[] = {0.0, 0.0, 10.0};
  double iTransL[] = {0.0, 10.0, 0.0};
  double line, sample;
  computePixel(-0.5, -0.2, 8.0, 8.0, 1.0, 1.0, 2.0, 1.0, iTransS, iTransL, line,
               sample);
  EXPECT_DOUBLE_EQ(line, 2.0);
  EXPECT_DOUBLE_EQ(sample, 4.0);
}

TEST(UtilitiesTests, computePixelStartSumming) {
  double iTransS[] = {0.0, 0.0, 10.0};
  double iTransL[] = {0.0, 10.0, 0.0};
  double line, sample;
  computePixel(-0.5, -0.2, 8.0, 8.0, 2.0, 4.0, 2.0, 1.0, iTransS, iTransL, line,
               sample);
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
    lagrangeInterp(numTime, &singlePoint[0], startTime, delTime, time,
                   vectorLength, order, &interpPoint[0]);
    FAIL() << "Expected an error";
  } catch (csm::Error &e) {
    EXPECT_EQ(e.getError(), csm::Error::INDEX_OUT_OF_RANGE);
  } catch (...) {
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

  lagrangeInterp(numTime, &interpValues[0], startTime, delTime, time,
                 vectorLength, order, &outputValue[0]);
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

  lagrangeInterp(numTime, &interpValues[0], startTime, delTime, time,
                 vectorLength, order, &outputValue[0]);
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

  lagrangeInterp(numTime, &interpValues[0], startTime, delTime, time,
                 vectorLength, order, &outputValue[0]);
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

  lagrangeInterp(numTime, &interpValues[0], startTime, delTime, time,
                 vectorLength, order, &outputValue[0]);
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

  lagrangeInterp(numTime, &interpValues[0], startTime, delTime, time,
                 vectorLength, order, &outputValue[0]);
  EXPECT_DOUBLE_EQ(outputValue[0], 3.0 / 2.0);

  time = 6.5;
  lagrangeInterp(numTime, &interpValues[0], startTime, delTime, time,
                 vectorLength, order, &outputValue[0]);
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

  lagrangeInterp(numTime, &interpValues[0], startTime, delTime, time,
                 vectorLength, order, &outputValue[0]);
  EXPECT_DOUBLE_EQ(outputValue[0], 45.0 / 16.0);

  time = 5.5;
  lagrangeInterp(numTime, &interpValues[0], startTime, delTime, time,
                 vectorLength, order, &outputValue[0]);
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

  lagrangeInterp(numTime, &interpValues[0], startTime, delTime, time,
                 vectorLength, order, &outputValue[0]);
  EXPECT_DOUBLE_EQ(outputValue[0], 1449.0 / 256.0);

  time = 4.5;
  lagrangeInterp(numTime, &interpValues[0], startTime, delTime, time,
                 vectorLength, order, &outputValue[0]);
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

  lagrangeInterp(numTime, &interpValues[0], startTime, delTime, time,
                 vectorLength, order, &outputValue[0]);
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

TEST(UtilitiesTests, polynomialEval) {
  std::vector<double> coeffs = {-12.0, 4.0, -3.0, 1.0};
  EXPECT_DOUBLE_EQ(evaluatePolynomial(coeffs, -1.0), -20.0);
  EXPECT_DOUBLE_EQ(evaluatePolynomialDerivative(coeffs, -1.0), 13.0);
  EXPECT_DOUBLE_EQ(evaluatePolynomial(coeffs, 0.0), -12.0);
  EXPECT_DOUBLE_EQ(evaluatePolynomialDerivative(coeffs, 0.0), 4.0);
  EXPECT_DOUBLE_EQ(evaluatePolynomial(coeffs, 2.0), -8.0);
  EXPECT_DOUBLE_EQ(evaluatePolynomialDerivative(coeffs, 2.0), 4.0);
  EXPECT_DOUBLE_EQ(evaluatePolynomial(coeffs, 3.5), 8.125);
  EXPECT_DOUBLE_EQ(evaluatePolynomialDerivative(coeffs, 3.5), 19.75);
  EXPECT_THROW(evaluatePolynomial(std::vector<double>(), 0.0),
               std::invalid_argument);
  EXPECT_THROW(evaluatePolynomialDerivative(std::vector<double>(), 0.0),
               std::invalid_argument);
}

TEST(UtilitiesTests, polynomialRoot) {
  std::vector<double> oneRootCoeffs = {-12.0, 4.0, -3.0,
                                       1.0};           // roots are 3, +-2i
  std::vector<double> noRootCoeffs = {4.0, 0.0, 1.0};  // roots are +-2i
  EXPECT_NEAR(polynomialRoot(oneRootCoeffs, 0.0), 3.0, 1e-10);
  EXPECT_THROW(polynomialRoot(noRootCoeffs, 0.0), std::invalid_argument);
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

TEST(UtilitiesTests, stateAsJsonWithModel) {
  std::string modelState = "{\"test_key\":\"test_string\"}";
  json state = stateAsJson("MODEL_NAME\n"+modelState);
  EXPECT_STREQ(modelState.c_str(), state.dump().c_str());
}

TEST(UtilitiesTests, stateAsJsonWithoutModel) {
  std::string modelState = "{\"test_key\":\"test_string\"}";
  json state = stateAsJson(modelState);
  EXPECT_STREQ(modelState.c_str(), state.dump().c_str());
}

TEST(UtilitiesTests, ephemTimeToCalendarTime) {

  double ephemTime = 0.0;
  std::string timeStr = ephemTimeToCalendarTime(ephemTime);

  EXPECT_STREQ(timeStr.c_str(), "2000-01-01T00:00:00Z");
}

TEST(UtilitiesTests, fileReaderTest) {
  std::string fromFile;
  readFileInString("data/hello.json", fromFile);
  EXPECT_STREQ(fromFile.c_str(), "\"Hello\"\n");
}

TEST(UtilitiesTests, sanitizeTest) {
  std::string input = "\nHello World\007";
  sanitize(input);
  EXPECT_STREQ(input.c_str(), "\nHello World\n");
}

TEST(UtilitiesTests, pixelToMeterTest) {
  std::vector<double> geoTransform = {-104607.78948592, 4.980137561815, 0.0, -570564.40018202, 0.0, -4.980137561815};
  std::vector<double> ret = pixelToMeter(13.6191, 4981.58, geoTransform);
  EXPECT_NEAR(ret[0], -570632.225173488, 1e-4);
  EXPECT_NEAR(ret[1], -79798.83581073358, 1e-4);
}

TEST(UtilitiesTests, meterToPixelTest) {
  std::vector<double> geoTransform = {-104607.78948592, 4.980137561815, 0.0, -570564.40018202, 0.0, -4.980137561815};
  std::vector<double> ret = meterToPixel(-79798.83581073358, -570632.225173488, geoTransform);
  EXPECT_NEAR(ret[0], 13.6191, 1e-4);
  EXPECT_NEAR(ret[1], 4981.5800000000099, 1e-4);
}

// VariantMap Tests
TEST(VariantMapTests, SetAndGetString) {
  VariantMap vm;
  vm.set<std::string>("key", "value");
  EXPECT_EQ(vm.get<std::string>("key"), "value");
}

TEST(VariantMapTests, SetAndGetInt) {
  VariantMap vm;
  vm.set<int>("key", 42);
  EXPECT_EQ(vm.get<int>("key"), 42);
}

TEST(VariantMapTests, SetAndGetDouble) {
  VariantMap vm;
  vm.set<double>("key", 3.14159);
  EXPECT_DOUBLE_EQ(vm.get<double>("key"), 3.14159);
}

TEST(VariantMapTests, SetAndGetBool) {
  VariantMap vm;
  vm.set<bool>("key", true);
  EXPECT_TRUE(vm.get<bool>("key"));
  vm.set<bool>("key", false);
  EXPECT_FALSE(vm.get<bool>("key"));
}

TEST(VariantMapTests, SetAndGetVectorDouble) {
  VariantMap vm;
  std::vector<double> vec = {1.1, 2.2, 3.3};
  vm.set<std::vector<double>>("key", vec);
  std::vector<double> result = vm.get<std::vector<double>>("key");
  ASSERT_EQ(result.size(), 3);
  EXPECT_DOUBLE_EQ(result[0], 1.1);
  EXPECT_DOUBLE_EQ(result[1], 2.2);
  EXPECT_DOUBLE_EQ(result[2], 3.3);
}

TEST(VariantMapTests, SetAndGetVectorInt) {
  VariantMap vm;
  std::vector<int> vec = {1, 2, 3};
  vm.set<std::vector<int>>("key", vec);
  std::vector<int> result = vm.get<std::vector<int>>("key");
  ASSERT_EQ(result.size(), 3);
  EXPECT_EQ(result[0], 1);
  EXPECT_EQ(result[1], 2);
  EXPECT_EQ(result[2], 3);
}

TEST(VariantMapTests, GetWithDefaultString) {
  VariantMap vm;
  EXPECT_EQ(vm.get<std::string>("nonexistent", "default"), "default");
  vm.set<std::string>("key", "value");
  EXPECT_EQ(vm.get<std::string>("key", "default"), "value");
}

TEST(VariantMapTests, GetWithDefaultInt) {
  VariantMap vm;
  EXPECT_EQ(vm.get<int>("nonexistent", 99), 99);
  vm.set<int>("key", 42);
  EXPECT_EQ(vm.get<int>("key", 99), 42);
}

TEST(VariantMapTests, GetWithDefaultDouble) {
  VariantMap vm;
  EXPECT_DOUBLE_EQ(vm.get<double>("nonexistent", 9.9), 9.9);
  vm.set<double>("key", 3.14);
  EXPECT_DOUBLE_EQ(vm.get<double>("key", 9.9), 3.14);
}

TEST(VariantMapTests, GetWithDefaultBool) {
  VariantMap vm;
  EXPECT_TRUE(vm.get<bool>("nonexistent", true));
  vm.set<bool>("key", false);
  EXPECT_FALSE(vm.get<bool>("key", true));
}

TEST(VariantMapTests, GetWithDefaultVectorDouble) {
  VariantMap vm;
  std::vector<double> defaultVec = {9.9};
  EXPECT_EQ(vm.get<std::vector<double>>("nonexistent", defaultVec), defaultVec);
  std::vector<double> vec = {1.1, 2.2};
  vm.set<std::vector<double>>("key", vec);
  EXPECT_EQ(vm.get<std::vector<double>>("key", defaultVec), vec);
}

TEST(VariantMapTests, GetWithDefaultVectorInt) {
  VariantMap vm;
  std::vector<int> defaultVec = {99};
  EXPECT_EQ(vm.get<std::vector<int>>("nonexistent", defaultVec), defaultVec);
  std::vector<int> vec = {1, 2};
  vm.set<std::vector<int>>("key", vec);
  EXPECT_EQ(vm.get<std::vector<int>>("key", defaultVec), vec);
}

TEST(VariantMapTests, Contains) {
  VariantMap vm;
  EXPECT_FALSE(vm.contains("key"));
  vm.set<int>("key", 42);
  EXPECT_TRUE(vm.contains("key"));
}

TEST(VariantMapTests, Erase) {
  VariantMap vm;
  vm.set<int>("key", 42);
  EXPECT_TRUE(vm.contains("key"));
  vm.erase("key");
  EXPECT_FALSE(vm.contains("key"));
}

TEST(VariantMapTests, Clear) {
  VariantMap vm;
  vm.set<int>("key1", 1);
  vm.set<int>("key2", 2);
  EXPECT_EQ(vm.size(), 2);
  vm.clear();
  EXPECT_EQ(vm.size(), 0);
  EXPECT_TRUE(vm.empty());
}

TEST(VariantMapTests, SizeAndEmpty) {
  VariantMap vm;
  EXPECT_TRUE(vm.empty());
  EXPECT_EQ(vm.size(), 0);
  vm.set<int>("key1", 1);
  EXPECT_FALSE(vm.empty());
  EXPECT_EQ(vm.size(), 1);
  vm.set<int>("key2", 2);
  EXPECT_EQ(vm.size(), 2);
}

TEST(VariantMapTests, Keys) {
  VariantMap vm;
  vm.set<int>("key1", 1);
  vm.set<int>("key2", 2);
  vm.set<int>("key3", 3);
  std::vector<std::string> keys = vm.keys();
  ASSERT_EQ(keys.size(), 3);
  // Keys should be in sorted order (std::map property)
  EXPECT_EQ(keys[0], "key1");
  EXPECT_EQ(keys[1], "key2");
  EXPECT_EQ(keys[2], "key3");
}

TEST(VariantMapTests, CopyConstructor) {
  VariantMap vm1;
  vm1.set<int>("key", 42);
  vm1.set<std::string>("name", "test");

  VariantMap vm2(vm1);
  EXPECT_EQ(vm2.get<int>("key"), 42);
  EXPECT_EQ(vm2.get<std::string>("name"), "test");

  // Ensure deep copy - modifying vm2 doesn't affect vm1
  vm2.set<int>("key", 99);
  EXPECT_EQ(vm1.get<int>("key"), 42);
  EXPECT_EQ(vm2.get<int>("key"), 99);
}

TEST(VariantMapTests, AssignmentOperator) {
  VariantMap vm1;
  vm1.set<int>("key", 42);
  vm1.set<std::string>("name", "test");

  VariantMap vm2;
  vm2 = vm1;
  EXPECT_EQ(vm2.get<int>("key"), 42);
  EXPECT_EQ(vm2.get<std::string>("name"), "test");

  // Ensure deep copy - modifying vm2 doesn't affect vm1
  vm2.set<int>("key", 99);
  EXPECT_EQ(vm1.get<int>("key"), 42);
  EXPECT_EQ(vm2.get<int>("key"), 99);
}

TEST(VariantMapTests, VariantMapFromJson) {
  json j = {
    {"string_key", "value"},
    {"int_key", 42},
    {"double_key", 3.14},
    {"bool_key", true},
    {"vector_key", {1.1, 2.2, 3.3}}
  };

  VariantMap vm = variantMapFromJson(j);

  EXPECT_EQ(vm.get<std::string>("string_key"), "value");
  EXPECT_EQ(vm.get<int>("int_key"), 42);
  EXPECT_DOUBLE_EQ(vm.get<double>("double_key"), 3.14);
  EXPECT_TRUE(vm.get<bool>("bool_key"));
  std::vector<double> vec = vm.get<std::vector<double>>("vector_key");
  ASSERT_EQ(vec.size(), 3);
  EXPECT_DOUBLE_EQ(vec[0], 1.1);
  EXPECT_DOUBLE_EQ(vec[1], 2.2);
  EXPECT_DOUBLE_EQ(vec[2], 3.3);
}

TEST(VariantMapTests, JsonFromVariantMap) {
  VariantMap vm;
  vm.set<std::string>("string_key", "value");
  vm.set<int>("int_key", 42);
  vm.set<double>("double_key", 3.14);
  vm.set<bool>("bool_key", true);
  vm.set<std::vector<double>>("vector_key", {1.1, 2.2, 3.3});
  vm.set<std::vector<int>>("int_vector_key", {1, 2, 3});

  json j = jsonFromVariantMap(vm);

  EXPECT_EQ(j["string_key"].get<std::string>(), "value");
  EXPECT_EQ(j["int_key"].get<int>(), 42);
  EXPECT_DOUBLE_EQ(j["double_key"].get<double>(), 3.14);
  EXPECT_TRUE(j["bool_key"].get<bool>());
  EXPECT_EQ(j["vector_key"].get<std::vector<double>>(), std::vector<double>({1.1, 2.2, 3.3}));
  EXPECT_EQ(j["int_vector_key"].get<std::vector<int>>(), std::vector<int>({1, 2, 3}));
}

TEST(VariantMapTests, RoundTripJsonConversion) {
  json original = {
    {"string_key", "value"},
    {"int_key", 42},
    {"double_key", 3.14},
    {"bool_key", true},
    {"vector_key", {1.1, 2.2, 3.3}}
  };

  VariantMap vm = variantMapFromJson(original);
  json roundtrip = jsonFromVariantMap(vm);

  EXPECT_EQ(original, roundtrip);
}

TEST(VariantMapTests, MultipleTypes) {
  VariantMap vm;
  vm.set<std::string>("name", "test");
  vm.set<int>("count", 42);
  vm.set<double>("value", 3.14);
  vm.set<bool>("flag", true);
  vm.set<std::vector<double>>("data", {1.0, 2.0, 3.0});

  EXPECT_EQ(vm.size(), 5);
  EXPECT_EQ(vm.get<std::string>("name"), "test");
  EXPECT_EQ(vm.get<int>("count"), 42);
  EXPECT_DOUBLE_EQ(vm.get<double>("value"), 3.14);
  EXPECT_TRUE(vm.get<bool>("flag"));
  EXPECT_EQ(vm.get<std::vector<double>>("data").size(), 3);
}

TEST(VariantMapTests, DumpsFunction) {
  VariantMap vm;
  vm.set<std::string>("name", "test");
  vm.set<int>("count", 42);
  vm.set<double>("pi", 3.14159);
  vm.set<bool>("enabled", true);
  vm.set<std::vector<double>>("coords", {1.1, 2.2, 3.3});
  vm.set<std::vector<int>>("indices", {1, 2, 3});

  // Test that dumps returns a string
  std::string output = vm.dumps();
  EXPECT_FALSE(output.empty());
  EXPECT_NE(output.find("name"), std::string::npos);
  EXPECT_NE(output.find("test"), std::string::npos);
  EXPECT_NE(output.find("count"), std::string::npos);
  EXPECT_NE(output.find("42"), std::string::npos);

  // Test the std::cout usage pattern
  std::cout << "\n=== Testing VariantMap::dumps() ===" << std::endl;
  std::cout << vm.dumps() << std::endl;
  std::cout << "=== End of dumps test ===" << std::endl;

  // Verify content is still accessible after dumps
  EXPECT_EQ(vm.get<std::string>("name"), "test");
  EXPECT_EQ(vm.get<int>("count"), 42);
}

TEST(VariantMapTests, EmptyArrayHandling) {
  // Test that empty arrays are preserved during JSON conversion
  nlohmann::json j = {
    {"m_sunPosition", std::vector<double>{100.0, 100.0, 100.0}},
    {"m_sunVelocity", std::vector<double>{}}
  };

  VariantMap vm = variantMapFromJson(j);

  // Empty array should be present in the map
  EXPECT_TRUE(vm.contains("m_sunPosition"));
  EXPECT_TRUE(vm.contains("m_sunVelocity"));

  // Should be able to retrieve empty array
  std::vector<double> sunPos = vm.get<std::vector<double>>("m_sunPosition");
  std::vector<double> sunVel = vm.get<std::vector<double>>("m_sunVelocity");

  EXPECT_EQ(sunPos.size(), 3);
  EXPECT_EQ(sunVel.size(), 0);

  // Round-trip conversion should preserve empty arrays
  nlohmann::json j2 = jsonFromVariantMap(vm);
  EXPECT_TRUE(j2.contains("m_sunVelocity"));
  EXPECT_TRUE(j2["m_sunVelocity"].is_array());
  EXPECT_EQ(j2["m_sunVelocity"].size(), 0);
}

TEST(VariantMapTests, GetValueType) {
  VariantMap vm;
  vm.set<int>("int_key", 5);
  vm.set<double>("double_key", 3.14);
  vm.set<std::string>("string_key", "hello");
  vm.set<bool>("bool_key", true);
  vm.set<std::vector<double>>("vec_double", {1.1, 2.2});
  vm.set<std::vector<int>>("vec_int", {1, 2, 3});

  EXPECT_EQ(vm.getValueType("int_key"), VariantMap::ValueType::Int);
  EXPECT_EQ(vm.getValueType("double_key"), VariantMap::ValueType::Double);
  EXPECT_EQ(vm.getValueType("string_key"), VariantMap::ValueType::String);
  EXPECT_EQ(vm.getValueType("bool_key"), VariantMap::ValueType::Bool);
  EXPECT_EQ(vm.getValueType("vec_double"), VariantMap::ValueType::VectorDouble);
  EXPECT_EQ(vm.getValueType("vec_int"), VariantMap::ValueType::VectorInt);
  EXPECT_EQ(vm.getValueType("missing_key"), VariantMap::ValueType::Unknown);
}

TEST(VariantMapTests, TypePreservation) {
  VariantMap vm;
  vm.set<int>("int_val", 42);
  vm.set<double>("double_val", 42.0);
  vm.set<std::vector<int>>("vec_int", {1, 2, 3});
  vm.set<std::vector<double>>("vec_double", {1.0, 2.0, 3.0});

  nlohmann::json j = jsonFromVariantMap(vm);

  // Verify types are preserved correctly
  EXPECT_TRUE(j["int_val"].is_number_integer());
  EXPECT_TRUE(j["double_val"].is_number_float());
  EXPECT_TRUE(j["vec_int"].is_array());
  EXPECT_TRUE(j["vec_double"].is_array());

  // Verify values
  EXPECT_EQ(j["int_val"].get<int>(), 42);
  EXPECT_DOUBLE_EQ(j["double_val"].get<double>(), 42.0);

  // Round-trip should preserve types
  VariantMap vm2 = variantMapFromJson(j);
  EXPECT_EQ(vm2.getValueType("int_val"), VariantMap::ValueType::Int);
  EXPECT_EQ(vm2.getValueType("double_val"), VariantMap::ValueType::Double);
  EXPECT_EQ(vm2.getValueType("vec_int"), VariantMap::ValueType::VectorInt);
  EXPECT_EQ(vm2.getValueType("vec_double"), VariantMap::ValueType::VectorDouble);
}