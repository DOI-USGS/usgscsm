#include "Distortion.h"

#include <gtest/gtest.h>

#include "Fixtures.h"

// NOTE: The imagePt format is (Lines,Samples)

INSTANTIATE_TEST_CASE_P(JacobianTest,ImageCoordParameterizedTest,
                        ::testing::Values(csm::ImageCoord(2.5,2.5),csm::ImageCoord(7.5,7.5)));

TEST_P(ImageCoordParameterizedTest, JacobianTest) {
   std::vector<double> transverseDistortionCoeffs = {1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0,
                                                     0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0};

   double Jxx,Jxy,Jyx,Jyy;

   csm::ImageCoord imagePt = GetParam();
   double jacobian[4];
   distortionJacobian(imagePt.samp, imagePt.line, jacobian, transverseDistortionCoeffs);

   // Jxx * Jyy - Jxy * Jyx
   double determinant = fabs(jacobian[0] * jacobian[3] - jacobian[1] * jacobian[2]);
   EXPECT_GT(determinant,1e-3);
}

TEST(Transverse, Jacobian1) {
  csm::ImageCoord imagePt(7.5, 7.5);

  std::vector<double> transverseDistortionCoeffs = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,
                                                    0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,1.0};

  double jacobian[4];
  distortionJacobian(imagePt.samp, imagePt.line, jacobian, transverseDistortionCoeffs);

  EXPECT_NEAR(jacobian[0],56.25,1e-8 );
  EXPECT_NEAR(jacobian[1],112.5,1e-8);
  EXPECT_NEAR(jacobian[2],56.25,1e-8);
  EXPECT_NEAR(jacobian[3],281.25,1e-8);
}

TEST(Transverse, distortMe_AlternatingOnes) {
  csm::ImageCoord imagePt(7.5, 7.5);

  std::vector<double> transverseDistortionCoeffs = {1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,
                                                    0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0};

  double dx, dy;
  distortionFunction(imagePt.samp, imagePt.line, dx, dy, transverseDistortionCoeffs);

  EXPECT_NEAR(dx,908.5,1e-8 );
  EXPECT_NEAR(dy,963.75,1e-8);
}

TEST(Transverse,  distortMe_AllCoefficientsOne) {
  csm::ImageCoord imagePt(7.5, 7.5);

  std::vector<double> transverseDistortionCoeffs = {1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,
                                                    1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0};

  double dx, dy;
  distortionFunction(imagePt.samp, imagePt.line, dx, dy, transverseDistortionCoeffs);

  EXPECT_NEAR(dx,1872.25,1e-8 );
  EXPECT_NEAR(dy,1872.25,1e-8);
}

TEST(transverse, removeDistortion1) {
  csm::ImageCoord imagePt(7.5, 7.5);
  double ux, uy;

  std::vector<double> transverseDistortionCoeffs = {0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
                                                    0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

  removeDistortion(imagePt.samp, imagePt.line, ux, uy, transverseDistortionCoeffs, DistortionType::TRANSVERSE);

  EXPECT_NEAR(imagePt.samp, 7.5, 1e-8 );
  EXPECT_NEAR(imagePt.line, 7.5, 1e-8);
  EXPECT_NEAR(imagePt.line, ux, 1e-8);
  EXPECT_NEAR(imagePt.samp, uy, 1e-8);
}

TEST(transverse, removeDistortion_AllCoefficientsOne) {
  csm::ImageCoord imagePt(1872.25, 1872.25);
  double ux, uy;

  std::vector<double> transverseDistortionCoeffs = {1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,
                                                    1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0};

  removeDistortion(imagePt.samp, imagePt.line, ux, uy, transverseDistortionCoeffs, DistortionType::TRANSVERSE);

  // The Jacobian is singular, so the setFocalPlane should break out of it's iteration and
  // returns the same distorted coordinates that were passed in.
  EXPECT_NEAR(ux, imagePt.samp,1e-8 );
  EXPECT_NEAR(uy, imagePt.line,1e-8);
}

TEST(transverse, removeDistortion_AlternatingOnes) {
  csm::ImageCoord imagePt(963.75, 908.5);
  double ux, uy;

  std::vector<double> transverseDistortionCoeffs = {1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,
                                                    0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0};

  removeDistortion(imagePt.samp, imagePt.line, ux, uy, transverseDistortionCoeffs, DistortionType::TRANSVERSE);

  EXPECT_NEAR(ux, 7.5, 1e-8 );
  EXPECT_NEAR(uy, 7.5, 1e-8);
}

TEST(Radial, testRemoveDistortion) {
  csm::ImageCoord imagePt(0.0, 4.0);

  double ux, uy;
  std::vector<double> coeffs = {0, 0, 0};

  removeDistortion(imagePt.samp, imagePt.line, ux, uy, coeffs, DistortionType::RADIAL);

  EXPECT_NEAR(ux, 4, 1e-8);
  EXPECT_NEAR(uy, 0, 1e-8);
}

// If coeffs are 0 then this will have the same result as removeDistortion
// with 0 distortion coefficients
TEST(Radial, testInverseDistortion){
  csm::ImageCoord imagePt(0.0, 4.0);

  double dx, dy;
  double desiredPrecision = 0.01;
  std::vector<double> coeffs = {0, 0, 0};

  applyDistortion(imagePt.samp, imagePt.line, dx, dy, coeffs,
                   DistortionType::RADIAL, desiredPrecision);

  EXPECT_NEAR(dx,4,1e-8);
  EXPECT_NEAR(dy,0,1e-8);
}

TEST(Radial, testInverseOnesCoeffs){
  csm::ImageCoord imagePt(0.0, 4.0);

  double dx, dy;
  double desiredPrecision = 0.01;
  std::vector<double> coeffs = {1, 1, 1};

  applyDistortion(imagePt.samp, imagePt.line, dx, dy, coeffs,
                   DistortionType::RADIAL, desiredPrecision);

  EXPECT_NEAR(dx,4,1e-8);
  EXPECT_NEAR(dy,0,1e-8);
}
