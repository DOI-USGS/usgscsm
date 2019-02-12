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

   csm::ImageCoord imagePt1 = GetParam();
   distortionJacobian(imagePt1.samp, imagePt1.line, Jxx, Jxy, Jyx, Jyy, odtX, odtY);

   double determinant = fabs(Jxx*Jyy - Jxy*Jyx);
   EXPECT_GT(determinant,1e-3);
}

TEST(Transverse, Jacobian1) {
  csm::ImageCoord imagePt(7.5, 7.5);

  std::vector<double> odtX = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0};
  std::vector<double> odtY = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,1.0};

  double Jxx,Jxy,Jyx,Jyy;
  distortionJacobian(imagePt.samp, imagePt.line, Jxx, Jxy, Jyx, Jyy, odtX, odtY);

  EXPECT_NEAR(Jxx,56.25,1e-8 );
  EXPECT_NEAR(Jxy,112.5,1e-8);
  EXPECT_NEAR(Jyx,56.25,1e-8);
  EXPECT_NEAR(Jyy,281.25,1e-8);
}

TEST(Transverse, distortMe_AlternatingOnes) {
  csm::ImageCoord imagePt(7.5, 7.5);

  std::vector<double> odtX = {1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0};
  std::vector<double> odtY = {0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0};

  double dx,dy;
  distortionFunction(imagePt.samp, imagePt.line, dx, dy, odtX, odtY);

  EXPECT_NEAR(dx,908.5,1e-8 );
  EXPECT_NEAR(dy,963.75,1e-8);
}

TEST(Transverse,  distortMe_AllCoefficientsOne) {
  csm::ImageCoord imagePt(7.5, 7.5);

  std::vector<double> odtX = {1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0};
  std::vector<double> odtY = {1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0};

  double dx,dy;
  distortionFunction(imagePt.samp, imagePt.line, dx, dy, odtX, odtY);

  EXPECT_NEAR(dx,1872.25,1e-8 );
  EXPECT_NEAR(dy,1872.25,1e-8);
}

TEST(Radial, testRemoveDistortion) {
  csm::ImageCoord imagePt(0.0, 4.0);

  double dx, dy;
  double coeffs[3] = {0, 0, 0};

  removeDistortion(imagePt.samp, imagePt.line, dx, dy, coeffs);

  EXPECT_NEAR(dx,4,1e-8);
  EXPECT_NEAR(dy,0,1e-8);
}

// If coeffs are 0 then this will have the same result as removeDistortion
// with 0 distortion coefficients
TEST(Radial, testInverseDistortion){
  csm::ImageCoord imagePt(0.0, 4.0);

  double dx, dy;
  double desiredPrecision = 0.01;
  double coeffs[3] = {0, 0, 0};

  invertDistortion(imagePt.samp, imagePt.line, dx, dy, coeffs, desiredPrecision);

  EXPECT_NEAR(dx,4,1e-8);
  EXPECT_NEAR(dy,0,1e-8);
}

TEST(Radial, testInverseOnesCoeffs){
  csm::ImageCoord imagePt(0.0, 4.0);

  double dx, dy;
  double desiredPrecision = 0.01;
  double coeffs[3] = {1, 1, 1};

  invertDistortion(imagePt.samp, imagePt.line, dx, dy, coeffs, desiredPrecision);

  EXPECT_NEAR(dx,4,1e-8);
  EXPECT_NEAR(dy,0,1e-8);
}
