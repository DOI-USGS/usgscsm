#include "Distortion.h"

#include <gtest/gtest.h>

#include "Fixtures.h"

// NOTE: The imagePt format is (Lines,Samples)

INSTANTIATE_TEST_CASE_P(JacobianTest,FramerParameterizedTest,
                        ::testing::Values(csm::ImageCoord(2.5,2.5),csm::ImageCoord(7.5,7.5)));

TEST_P(FramerParameterizedTest, JacobianTest) {
   std::vector<double> odtX = {1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0};
   std::vector<double> odtY = {0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0};

   double Jxx,Jxy,Jyx,Jyy;

   csm::ImageCoord imagePt1 = GetParam();
   distortionJacobian(imagePt1.samp, imagePt1.line, Jxx, Jxy, Jyx, Jyy, odtX, odtY);

   double determinant = fabs(Jxx*Jyy - Jxy*Jyx);
   EXPECT_GT(determinant,1e-3);
}

TEST_F(FrameSensorModel, Jacobian1) {
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

TEST_F(FrameSensorModel, distortMe_AlternatingOnes) {
  csm::ImageCoord imagePt(7.5, 7.5);

  std::vector<double> odtX = {1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0};
  std::vector<double> odtY = {0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0};

  double dx,dy;
  distortionFunction(imagePt.samp, imagePt.line, dx, dy, odtX, odtY);

  EXPECT_NEAR(dx,908.5,1e-8 );
  EXPECT_NEAR(dy,963.75,1e-8);
}

TEST_F(FrameSensorModel, distortMe_AllCoefficientsOne) {
  csm::ImageCoord imagePt(7.5, 7.5);

  std::vector<double> odtX = {1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0};
  std::vector<double> odtY = {1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0};

  double dx,dy;
  distortionFunction(imagePt.samp, imagePt.line, dx, dy, odtX, odtY);

  EXPECT_NEAR(dx,1872.25,1e-8 );
  EXPECT_NEAR(dy,1872.25,1e-8);
}
