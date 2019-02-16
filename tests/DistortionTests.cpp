#include "Distortion.h"

#include <gtest/gtest.h>

#include "Fixtures.h"

// NOTE: The imagePt format is (Lines,Samples)

INSTANTIATE_TEST_CASE_P(JacobianTest,ImageCoordParameterizedTest,
                        ::testing::Values(csm::ImageCoord(2.5,2.5),csm::ImageCoord(7.5,7.5)));

TEST_P(ImageCoordParameterizedTest, JacobianTest) {
   std::vector<double> odtX = {1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0};
   std::vector<double> odtY = {0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0};

   double Jxx,Jxy,Jyx,Jyy;

   csm::ImageCoord imagePt = GetParam();
   std::vector<std::vector<double>> jacobian;
   jacobian = distortionJacobian(imagePt.samp, imagePt.line, odtX, odtY);

   // Jxx * Jyy - Jxy * Jyx
   double determinant = fabs(jacobian[0][0] * jacobian[1][1] - jacobian[0][1] * jacobian[1][0]);
   EXPECT_GT(determinant,1e-3);
}

TEST(Transverse, Jacobian1) {
  csm::ImageCoord imagePt(7.5, 7.5);

  std::vector<double> odtX = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0};
  std::vector<double> odtY = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,1.0};

  std::vector<std::vector<double>> jacobian;
  jacobian = distortionJacobian(imagePt.samp, imagePt.line, odtX, odtY);

  EXPECT_NEAR(jacobian[0][0],56.25,1e-8 );
  EXPECT_NEAR(jacobian[0][1],112.5,1e-8);
  EXPECT_NEAR(jacobian[1][0],56.25,1e-8);
  EXPECT_NEAR(jacobian[1][1],281.25,1e-8);
}

TEST(Transverse, distortMe_AlternatingOnes) {
  csm::ImageCoord imagePt(7.5, 7.5);

  std::vector<double> odtX = {1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0};
  std::vector<double> odtY = {0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0};

  double dx,dy;
  std::tuple<double, double> distortedPoint;
  distortedPoint = distortionFunction(imagePt.samp, imagePt.line, odtX, odtY);

  EXPECT_NEAR(std::get<0>(distortedPoint),908.5,1e-8 );
  EXPECT_NEAR(std::get<1>(distortedPoint),963.75,1e-8);
}

TEST(Transverse,  distortMe_AllCoefficientsOne) {
  csm::ImageCoord imagePt(7.5, 7.5);

  std::vector<double> odtX = {1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0};
  std::vector<double> odtY = {1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0};

  double dx,dy;
  std::tuple<double, double> distortedPoint;
  distortedPoint = distortionFunction(imagePt.samp, imagePt.line, odtX, odtY);

  EXPECT_NEAR(std::get<0>(distortedPoint),1872.25,1e-8 );
  EXPECT_NEAR(std::get<1>(distortedPoint),1872.25,1e-8);
}

TEST(transverse, removeDistortion1) {
  csm::ImageCoord imagePt(7.5, 7.5);

  std::vector<double> odtX = {0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
  std::vector<double> odtY = {0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
  std::vector<std::vector<double>> transverseDistortionCoeffs = {odtX, odtY};

  std::tuple<double, double> undistortedPoint;
  undistortedPoint = removeTransverseDistortion(imagePt.samp, imagePt.line, transverseDistortionCoeffs);

  EXPECT_NEAR(imagePt.samp,7.5,1e-8 );
  EXPECT_NEAR(imagePt.line,7.5,1e-8);
  EXPECT_NEAR(imagePt.line,std::get<0>(undistortedPoint),1e-8);
  EXPECT_NEAR(imagePt.samp,std::get<1>(undistortedPoint),1e-8);
}

TEST(transverse, removeDistortion_AllCoefficientsOne) {
  csm::ImageCoord imagePt(1872.25, 1872.25);

  std::vector<double> odtX = {1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0};
  std::vector<double> odtY = {1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0};
  std::vector<std::vector<double>> transverseDistortionCoeffs = {odtX, odtY};

  std::tuple<double, double> undistortedPoint;
  undistortedPoint = removeTransverseDistortion(imagePt.samp, imagePt.line, transverseDistortionCoeffs);

  // The Jacobian is singular, so the setFocalPlane should break out of it's iteration and
  // returns the same distorted coordinates that were passed in.
  EXPECT_NEAR(std::get<0>(undistortedPoint),imagePt.samp,1e-8 );
  EXPECT_NEAR(std::get<1>(undistortedPoint),imagePt.line,1e-8);
}

TEST(transverse, removeDistortion_AlternatingOnes) {
  csm::ImageCoord imagePt(963.75, 908.5);

  std::vector<double> odtX = {1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0};
  std::vector<double> odtY = {0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0};
  std::vector<std::vector<double>> transverseDistortionCoeffs = {odtX, odtY};

  std::tuple<double, double> undistortedPoint;
  undistortedPoint = removeTransverseDistortion(imagePt.samp, imagePt.line, transverseDistortionCoeffs);

  EXPECT_NEAR(std::get<0>(undistortedPoint),7.5,1e-8 );
  EXPECT_NEAR(std::get<1>(undistortedPoint),7.5,1e-8);
}

TEST(Radial, testRemoveDistortion) {
  csm::ImageCoord imagePt(0.0, 4.0);

  double dx, dy;
  // double coeffs[3] = {0, 0, 0};
  std::vector<double> coeffs = {0, 0, 0};
  std::tuple<double, double> undistortedPoint;

  undistortedPoint = removeRadialDistortion(imagePt.samp, imagePt.line, coeffs);

  EXPECT_NEAR(std::get<0>(undistortedPoint),4,1e-8);
  EXPECT_NEAR(std::get<1>(undistortedPoint),0,1e-8);
}

// If coeffs are 0 then this will have the same result as removeDistortion
// with 0 distortion coefficients
TEST(Radial, testInverseDistortion){
  csm::ImageCoord imagePt(0.0, 4.0);

  double dx, dy;
  double desiredPrecision = 0.01;
  std::vector<double> coeffs = {0, 0, 0};
  std::tuple<double, double> undistortedPoint;

  undistortedPoint = invertDistortion(imagePt.samp, imagePt.line, coeffs, desiredPrecision);

  EXPECT_NEAR(std::get<0>(undistortedPoint),4,1e-8);
  EXPECT_NEAR(std::get<1>(undistortedPoint),0,1e-8);
}

TEST(Radial, testInverseOnesCoeffs){
  csm::ImageCoord imagePt(0.0, 4.0);

  double dx, dy;
  double desiredPrecision = 0.01;
  std::vector<double> coeffs = {1, 1, 1};
  std::tuple<double, double> undistortedPoint;

  undistortedPoint = invertDistortion(imagePt.samp, imagePt.line, coeffs, desiredPrecision);

  EXPECT_NEAR(std::get<0>(undistortedPoint),4,1e-8);
  EXPECT_NEAR(std::get<1>(undistortedPoint),0,1e-8);
}

TEST(Radial, testRemoveDistortion) {
  csm::ImageCoord imagePt(0.0, 4.0);

  double dx, dy;
  double coeffs[3] = {0, 0, 0};
  std::tuple<double, double> undistortedPoint;

  undistortedPoint = removeDistortion(imagePt.samp, imagePt.line, coeffs);

  EXPECT_NEAR(std::get<0>(undistortedPoint),4,1e-8);
  EXPECT_NEAR(std::get<1>(undistortedPoint),0,1e-8);
}

// If coeffs are 0 then this will have the same result as removeDistortion
// with 0 distortion coefficients
TEST(Radial, testInverseDistortion){
  csm::ImageCoord imagePt(0.0, 4.0);

  double dx, dy;
  double desiredPrecision = 0.01;
  double coeffs[3] = {0, 0, 0};
  std::tuple<double, double> undistortedPoint;

  undistortedPoint = invertDistortion(imagePt.samp, imagePt.line, coeffs, desiredPrecision);

  EXPECT_NEAR(std::get<0>(undistortedPoint),4,1e-8);
  EXPECT_NEAR(std::get<1>(undistortedPoint),0,1e-8);
}

TEST(Radial, testInverseOnesCoeffs){
  csm::ImageCoord imagePt(0.0, 4.0);

  double dx, dy;
  double desiredPrecision = 0.01;
  double coeffs[3] = {1, 1, 1};
  std::tuple<double, double> undistortedPoint;

  undistortedPoint = invertDistortion(imagePt.samp, imagePt.line, coeffs, desiredPrecision);

  EXPECT_NEAR(std::get<0>(undistortedPoint),4,1e-8);
  EXPECT_NEAR(std::get<1>(undistortedPoint),0,1e-8);
}
