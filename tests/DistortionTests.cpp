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
  computeTransverseDistortion(imagePt.samp, imagePt.line, dx, dy, transverseDistortionCoeffs);

  EXPECT_NEAR(dx,908.5,1e-8 );
  EXPECT_NEAR(dy,963.75,1e-8);
}

TEST(Transverse,  distortMe_AllCoefficientsOne) {
  csm::ImageCoord imagePt(7.5, 7.5);

  std::vector<double> transverseDistortionCoeffs = {1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,
                                                    1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0};

  double dx, dy;
  computeTransverseDistortion(imagePt.samp, imagePt.line, dx, dy, transverseDistortionCoeffs);

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

TEST(DawnFc, testApply) {
  csm::ImageCoord imagePt(10.0, 10.0);

  double dx, dy;
  double desiredPrecision = 0.0000001;
  std::vector<double> coeffs = {8.4e-06};

  applyDistortion(imagePt.samp, imagePt.line, dx, dy, coeffs,
                  DistortionType::DAWNFC, desiredPrecision);

  ASSERT_DOUBLE_EQ(dx, 10.0168);
  ASSERT_DOUBLE_EQ(dy, 10.0168);
}

TEST(DawnFc, testRemove) {
  csm::ImageCoord imagePt(10.0168, 10.0168);

  double ux, uy;
  double desiredPrecision = 0.0000001;
  // Coeffs obtained from file FC21A0039691_15231053805F1E.IMG
  std::vector<double> coeffs = {8.4e-06};

  removeDistortion(imagePt.samp, imagePt.line, ux, uy, coeffs,
                  DistortionType::DAWNFC, desiredPrecision);

  EXPECT_NEAR(ux, 10.0, 1e-8);
  EXPECT_NEAR(uy, 10.0, 1e-8);
}

TEST(DawnFc, testZeroCoeffs) {
  csm::ImageCoord imagePt(10.0, 10.0);

  double ux, uy, dx, dy;
  double desiredPrecision = 0.0000001;
  std::vector<double> coeffs = {0};

  applyDistortion(imagePt.samp, imagePt.line, dx, dy, coeffs,
                  DistortionType::DAWNFC, desiredPrecision);

  removeDistortion(dx, dy, ux, uy, coeffs,
                  DistortionType::DAWNFC, desiredPrecision);


  ASSERT_DOUBLE_EQ(dx, 10.0);
  ASSERT_DOUBLE_EQ(dy, 10.0);
  ASSERT_DOUBLE_EQ(ux, 10.0);
  ASSERT_DOUBLE_EQ(uy, 10.0);
}

TEST(KaguyaLism, testRemoveCoeffs) {
  csm::ImageCoord imagePt(1.0, 1.0);

  double ux, uy;
  double desiredPrecision = 0.0000001;
  std::vector<double> distortionCoeffs = {0.5, 1, 2, 3, 4,
                                          0.5, 1, 2, 3, 4};

  removeDistortion(imagePt.samp, imagePt.line, ux, uy, distortionCoeffs,
                  DistortionType::KAGUYALISM, desiredPrecision);

  EXPECT_NEAR(ux, 1 + 1 + 2.828427125 + 6 + 11.313708499 + 0.5, 1e-8);
  EXPECT_NEAR(uy, 1 + 1 + 2.828427125 + 6 + 11.313708499 + 0.5, 1e-8);
}


TEST(KaguyaLism, testCoeffs) {
  csm::ImageCoord imagePt(1.0, 1.0);

  double ux, uy, dx, dy;
  double desiredPrecision = 0.0000001;
  // Coeffs obtained from file TC1W2B0_01_05211N095E3380.img
  std::vector<double> coeffs = {-0.0725, -0.0009649900000000001, 0.00098441, 8.5773e-06, -3.7438e-06,
                                0.0214, -0.0013796, 1.3502e-05, 2.7251e-06, -6.193800000000001e-06};

  removeDistortion(imagePt.samp, imagePt.line, ux, uy, coeffs,
                  DistortionType::KAGUYALISM, desiredPrecision);
  applyDistortion(ux, uy, dx, dy, coeffs,
                  DistortionType::KAGUYALISM, desiredPrecision);



  EXPECT_NEAR(ux, 0.9279337415074662, 1e-6);
  EXPECT_NEAR(uy, 1.0200274261995939, 1e-5);
  EXPECT_NEAR(dx, 1.0, 1e-8);
  EXPECT_NEAR(dy, 1.0, 1e-8);
}


TEST(KaguyaLism, testZeroCoeffs) {
  csm::ImageCoord imagePt(1.0, 1.0);

  double ux, uy, dx, dy;
  double desiredPrecision = 0.0000001;
  std::vector<double> coeffs = {0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0};

  applyDistortion(imagePt.samp, imagePt.line, dx, dy, coeffs,
                  DistortionType::KAGUYALISM, desiredPrecision);

  removeDistortion(dx, dy, ux, uy, coeffs,
                  DistortionType::KAGUYALISM, desiredPrecision);

  ASSERT_DOUBLE_EQ(dx, 1.0);
  ASSERT_DOUBLE_EQ(dy, 1.0);
  ASSERT_DOUBLE_EQ(ux, 1.0);
  ASSERT_DOUBLE_EQ(uy, 1.0);
}


// Test for LRO LROC NAC
TEST(LroLrocNac, testLastDetectorSample) {
  double ux, uy, dx, dy;
  double desiredPrecision = 0.0000001;
  // Coeffs obtained from file: lro_lroc_v18.ti
  std::vector<double> coeffs = {1.81E-5};

  removeDistortion(0.0, 5064.0 / 2.0 * 0.007, ux, uy, coeffs,
                  DistortionType::LROLROCNAC, desiredPrecision);

  applyDistortion(ux, uy, dx, dy, coeffs,
                  DistortionType::LROLROCNAC, desiredPrecision);

  EXPECT_NEAR(dx, 0.0, 1e-8);
  EXPECT_NEAR(dy, 17.724, 1e-8);
  EXPECT_NEAR(ux, 0.0, 1e-8);
  EXPECT_NEAR(uy, 17.6237922244, 1e-8);
}


TEST(LroLrocNac, testCoeffs) {
  double ux, uy, dx, dy;
  double desiredPrecision = 0.0000001;
  // Coeff obtained from file: lro_lroc_v18.ti
  std::vector<double> coeffs = {1.81E-5};

  applyDistortion(0.0, 0.0, dx, dy, coeffs,
                  DistortionType::LROLROCNAC, desiredPrecision);

  removeDistortion(dx, dy, ux, uy, coeffs,
                  DistortionType::LROLROCNAC, desiredPrecision);

  EXPECT_NEAR(dx, 0.0, 1e-8);
  EXPECT_NEAR(dy, 0.0, 1e-8);
  EXPECT_NEAR(ux, 0.0, 1e-8);
  EXPECT_NEAR(uy, 0.0, 1e-8);
}


TEST(LroLrocNac, testZeroCoeffs) {

  double ux, uy, dx, dy;
  double desiredPrecision = 0.0000001;
  std::vector<double> coeffs = {0};

  applyDistortion(0.0, 0.0, dx, dy, coeffs,
                  DistortionType::LROLROCNAC, desiredPrecision);

  removeDistortion(dx, dy, ux, uy, coeffs,
                  DistortionType::LROLROCNAC, desiredPrecision);

  ASSERT_DOUBLE_EQ(dx, 0.0);
  ASSERT_DOUBLE_EQ(dy, 0.0);
  ASSERT_DOUBLE_EQ(ux, 0.0);
  ASSERT_DOUBLE_EQ(uy, 0.0);
}
