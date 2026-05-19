#include "Distortion.h"

#include <gtest/gtest.h>

#include "Fixtures.h"

// NOTE: The imagePt format is (Lines,Samples)

INSTANTIATE_TEST_SUITE_P(JacobianTest, ImageCoordParameterizedTest,
                         ::testing::Values(csm::ImageCoord(2.5, 2.5),
                                           csm::ImageCoord(7.5, 7.5)));

TEST_P(ImageCoordParameterizedTest, JacobianTest) {
  std::vector<double> transverseDistortionCoeffs = {
      1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0,
      0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0};

  double Jxx, Jxy, Jyx, Jyy;

  csm::ImageCoord imagePt = GetParam();
  double jacobian[4];
  transverseDistortionJacobian(imagePt.samp, imagePt.line, jacobian,
                               transverseDistortionCoeffs);

  // Jxx * Jyy - Jxy * Jyx
  double determinant =
      fabs(jacobian[0] * jacobian[3] - jacobian[1] * jacobian[2]);
  EXPECT_GT(determinant, 1e-3);
}

TEST(Transverse, Jacobian1) {
  csm::ImageCoord imagePt(7.5, 7.5);

  std::vector<double> transverseDistortionCoeffs = {
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0};

  double jacobian[4];
  transverseDistortionJacobian(imagePt.samp, imagePt.line, jacobian,
                               transverseDistortionCoeffs);

  EXPECT_NEAR(jacobian[0], 56.25, 1e-8);
  EXPECT_NEAR(jacobian[1], 112.5, 1e-8);
  EXPECT_NEAR(jacobian[2], 56.25, 1e-8);
  EXPECT_NEAR(jacobian[3], 281.25, 1e-8);
}

TEST(Transverse, distortMe_AlternatingOnes) {
  csm::ImageCoord imagePt(7.5, 7.5);

  std::vector<double> transverseDistortionCoeffs = {
      1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0,
      0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0};

  double dx, dy;
  computeTransverseDistortion(imagePt.samp, imagePt.line, dx, dy,
                              transverseDistortionCoeffs);

  EXPECT_NEAR(dx, 908.5, 1e-8);
  EXPECT_NEAR(dy, 963.75, 1e-8);
}

TEST(Transverse, distortMe_AllCoefficientsOne) {
  csm::ImageCoord imagePt(7.5, 7.5);

  std::vector<double> transverseDistortionCoeffs = {
      1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
      1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

  double dx, dy;
  computeTransverseDistortion(imagePt.samp, imagePt.line, dx, dy,
                              transverseDistortionCoeffs);

  EXPECT_NEAR(dx, 1872.25, 1e-8);
  EXPECT_NEAR(dy, 1872.25, 1e-8);
}

TEST(transverse, removeDistortion1) {
  csm::ImageCoord imagePt(7.5, 7.5);
  double focalLength = 1000.0;
  double ux, uy;

  std::vector<double> transverseDistortionCoeffs = {
      0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  removeDistortion(imagePt.samp, imagePt.line, ux, uy, transverseDistortionCoeffs,
                   focalLength, DistortionType::TRANSVERSE);

  EXPECT_NEAR(imagePt.samp, 7.5, 1e-8);
  EXPECT_NEAR(imagePt.line, 7.5, 1e-8);
  EXPECT_NEAR(imagePt.line, ux, 1e-8);
  EXPECT_NEAR(imagePt.samp, uy, 1e-8);
}

TEST(transverse, removeDistortion_AllCoefficientsOne) {
  // With all coefficients equal for X and Y, the Jacobian rows are identical
  // and the determinant is exactly zero (analytically). Use small input values
  // so floating-point noise in the determinant stays below the 1e-6 threshold,
  // causing Newton-Raphson to bail on the first iteration and return the input.
  csm::ImageCoord imagePt(0.5, 0.5);
  double focalLength = 1000.0;
  double ux, uy;

  std::vector<double> transverseDistortionCoeffs = {
      1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
      1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

  removeDistortion(imagePt.samp, imagePt.line, ux, uy, transverseDistortionCoeffs,
                   focalLength, DistortionType::TRANSVERSE);

  EXPECT_NEAR(ux, imagePt.samp, 1e-8);
  EXPECT_NEAR(uy, imagePt.line, 1e-8);
}

TEST(transverse, removeDistortion_AlternatingOnes) {
  csm::ImageCoord imagePt(963.75, 908.5);
  double focalLength = 1000.0;
  double ux, uy;

  std::vector<double> transverseDistortionCoeffs = {
      1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0,
      0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0};

  removeDistortion(imagePt.samp, imagePt.line, ux, uy, transverseDistortionCoeffs, 
                   focalLength, DistortionType::TRANSVERSE);

  EXPECT_NEAR(ux, 7.5, 1e-8);
  EXPECT_NEAR(uy, 7.5, 1e-8);
}

TEST(Radial, testUndistortDistort) {
  
  // Distorted pixel
  csm::ImageCoord imagePt(0.0, 1e-1);
  double focalLength = 1000.0;

  // Undistort
  double ux, uy;
  std::vector<double> coeffs = {0.03, 0.00001, 0.000004};
  double tolerance = 1e-2;
  removeDistortion(imagePt.samp, imagePt.line, ux, uy, coeffs, focalLength,
                   DistortionType::RADIAL, tolerance);
  
  // Distort back
  double desiredPrecision = 1e-6;
  double dx, dy;
  applyDistortion(ux, uy, dx, dy, coeffs, focalLength,
                  DistortionType::RADIAL, desiredPrecision, tolerance);
  
  EXPECT_NEAR(dx, imagePt.samp, 1e-8);
  EXPECT_NEAR(dy, imagePt.line, 1e-8);
}

// If coeffs are 0 then this will have the same result as removeDistortion
// with 0 distortion coefficients
TEST(Radial, testInverseDistortion) {
  csm::ImageCoord imagePt(0.0, 4.0);

  double dx, dy;
  double focalLength = 1000.0;
  double desiredPrecision = 0.01;
  std::vector<double> coeffs = {0, 0, 0};
  applyDistortion(imagePt.samp, imagePt.line, dx, dy, coeffs, focalLength,
                  DistortionType::RADIAL, desiredPrecision);

  EXPECT_NEAR(dx, 4, 1e-8);
  EXPECT_NEAR(dy, 0, 1e-8);
}

TEST(Radial, testInverseOnesCoeffs) {
  csm::ImageCoord imagePt(0.0, 4.0);

  double dx, dy;
  double focalLength = 1000.0;
  double desiredPrecision = 0.01;
  std::vector<double> coeffs = {1, 1, 1};

  applyDistortion(imagePt.samp, imagePt.line, dx, dy, coeffs, focalLength,
                  DistortionType::RADIAL, desiredPrecision);

  EXPECT_NEAR(dx, 4, 1e-8);
  EXPECT_NEAR(dy, 0, 1e-8);
}

TEST(DawnFc, testApply) {
  csm::ImageCoord imagePt(10.0, 10.0);

  double dx, dy;
  double focalLength = 1000.0;
  double desiredPrecision = 0.0000001;
  std::vector<double> coeffs = {8.4e-06};

  applyDistortion(imagePt.samp, imagePt.line, dx, dy, coeffs, focalLength,
                  DistortionType::DAWNFC, desiredPrecision);

  ASSERT_DOUBLE_EQ(dx, 10.0168);
  ASSERT_DOUBLE_EQ(dy, 10.0168);
}

TEST(DawnFc, testRemove) {
  csm::ImageCoord imagePt(10.0168, 10.0168);

  double ux, uy;
  double focalLength = 1000.0;
  double desiredPrecision = 0.0000001;
  // Coeffs obtained from file FC21A0039691_15231053805F1E.IMG
  std::vector<double> coeffs = {8.4e-06};

  removeDistortion(imagePt.samp, imagePt.line, ux, uy, coeffs, focalLength,
                   DistortionType::DAWNFC, desiredPrecision);

  EXPECT_NEAR(ux, 10.0, 1e-8);
  EXPECT_NEAR(uy, 10.0, 1e-8);
}

TEST(DawnFc, testZeroCoeffs) {
  csm::ImageCoord imagePt(10.0, 10.0);

  double ux, uy, dx, dy;
  double focalLength = 1000.0;
  double desiredPrecision = 0.0000001;
  std::vector<double> coeffs = {0};

  applyDistortion(imagePt.samp, imagePt.line, dx, dy, coeffs, focalLength,
                  DistortionType::DAWNFC, desiredPrecision);

  removeDistortion(dx, dy, ux, uy, coeffs, focalLength, 
                   DistortionType::DAWNFC, desiredPrecision);

  ASSERT_DOUBLE_EQ(dx, 10.0);
  ASSERT_DOUBLE_EQ(dy, 10.0);
  ASSERT_DOUBLE_EQ(ux, 10.0);
  ASSERT_DOUBLE_EQ(uy, 10.0);
}

TEST(KaguyaLism, testRemoveCoeffs) {
  csm::ImageCoord imagePt(1.0, 1.0);

  double ux, uy;
  double focalLength = 1000.0;
  double desiredPrecision = 0.0000001;
  std::vector<double> distortionCoeffs = {0.5, 1, 2, 3, 4, 0.5, 1, 2, 3, 4};

  removeDistortion(imagePt.samp, imagePt.line, ux, uy, distortionCoeffs, focalLength,
                   DistortionType::KAGUYALISM, desiredPrecision);

  EXPECT_NEAR(ux, 1 + 1 + 2.828427125 + 6 + 11.313708499 + 0.5, 1e-8);
  EXPECT_NEAR(uy, 1 + 1 + 2.828427125 + 6 + 11.313708499 + 0.5, 1e-8);
}

TEST(KaguyaLism, testCoeffs) {
  csm::ImageCoord imagePt(1.0, 1.0);

  double ux, uy, dx, dy;
  double focalLength = 1000.0;
  double desiredPrecision = 0.0000001;
  // Coeffs obtained from file TC1W2B0_01_05211N095E3380.img
  std::vector<double> coeffs = {-0.0725,     -0.0009649900000000001,
                                0.00098441,  8.5773e-06,
                                -3.7438e-06, 0.0214,
                                -0.0013796,  1.3502e-05,
                                2.7251e-06,  -6.193800000000001e-06};

  removeDistortion(imagePt.samp, imagePt.line, ux, uy, coeffs, focalLength,
                   DistortionType::KAGUYALISM, desiredPrecision);
  applyDistortion(ux, uy, dx, dy, coeffs, focalLength,
                  DistortionType::KAGUYALISM, desiredPrecision);

  EXPECT_NEAR(ux, 0.9279337415074662, 1e-6);
  EXPECT_NEAR(uy, 1.0200274261995939, 1e-5);
  EXPECT_NEAR(dx, 1.0, 1e-8);
  EXPECT_NEAR(dy, 1.0, 1e-8);
}

TEST(KaguyaLism, testZeroCoeffs) {
  csm::ImageCoord imagePt(1.0, 1.0);

  double ux, uy, dx, dy;
  double focalLength = 1000.0;
  double desiredPrecision = 0.0000001;
  std::vector<double> coeffs = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  applyDistortion(imagePt.samp, imagePt.line, dx, dy, coeffs, focalLength,
                  DistortionType::KAGUYALISM, desiredPrecision);

  removeDistortion(dx, dy, ux, uy, coeffs, focalLength,
                   DistortionType::KAGUYALISM, desiredPrecision);

  ASSERT_DOUBLE_EQ(dx, 1.0);
  ASSERT_DOUBLE_EQ(dy, 1.0);
  ASSERT_DOUBLE_EQ(ux, 1.0);
  ASSERT_DOUBLE_EQ(uy, 1.0);
}

// Test for LRO LROC NAC
TEST(LroLrocNac, testLastDetectorSample) {
  double ux, uy, dx, dy;
  double focalLength = 1000.0;
  double desiredPrecision = 0.0000001;
  // Coeffs obtained from file: lro_lroc_v18.ti
  std::vector<double> coeffs = {1.81E-5};

  removeDistortion(0.0, 5064.0 / 2.0 * 0.007, ux, uy, coeffs, focalLength,
                   DistortionType::LROLROCNAC, desiredPrecision);

  applyDistortion(ux, uy, dx, dy, coeffs, focalLength, 
                  DistortionType::LROLROCNAC, desiredPrecision);

  EXPECT_NEAR(dx, 0.0, 1e-8);
  EXPECT_NEAR(dy, 17.724, 1e-8);
  EXPECT_NEAR(ux, 0.0, 1e-8);
  EXPECT_NEAR(uy, 17.6237922244, 1e-8);
}

TEST(LroLrocNac, testCoeffs) {
  double ux, uy, dx, dy;
  double focalLength = 1000.0;
  double desiredPrecision = 0.0000001;
  // Coeff obtained from file: lro_lroc_v18.ti
  std::vector<double> coeffs = {1.81E-5};

  applyDistortion(0.0, 0.0, dx, dy, coeffs, focalLength,
                  DistortionType::LROLROCNAC, desiredPrecision);

  removeDistortion(dx, dy, ux, uy, coeffs, focalLength,
                   DistortionType::LROLROCNAC, desiredPrecision);

  EXPECT_NEAR(dx, 0.0, 1e-8);
  EXPECT_NEAR(dy, 0.0, 1e-8);
  EXPECT_NEAR(ux, 0.0, 1e-8);
  EXPECT_NEAR(uy, 0.0, 1e-8);
}

TEST(LroLrocNac, testZeroCoeffs) {
  double ux, uy, dx, dy;
  double focalLength = 1000.0;
  double desiredPrecision = 0.0000001;
  std::vector<double> coeffs = {0};

  applyDistortion(0.0, 0.0, dx, dy, coeffs, focalLength,
                  DistortionType::LROLROCNAC, desiredPrecision);

  removeDistortion(dx, dy, ux, uy, coeffs, focalLength, 
                   DistortionType::LROLROCNAC, desiredPrecision);

  ASSERT_DOUBLE_EQ(dx, 0.0);
  ASSERT_DOUBLE_EQ(dy, 0.0);
  ASSERT_DOUBLE_EQ(ux, 0.0);
  ASSERT_DOUBLE_EQ(uy, 0.0);
}

INSTANTIATE_TEST_SUITE_P(CahvorTest, CoeffOffsetParameterizedTest,
                         ::testing::Values(std::vector<double>(2, 0),
                                           std::vector<double>(2, 1)));

TEST_P(CoeffOffsetParameterizedTest, RemoveDistortionCahvorTest)
{
  csm::ImageCoord imagePt(0.0, 4.0);

  double ux, uy;
  double focalLength = 1000.0;
  std::vector<double> offsets = GetParam();
  std::vector<double> coeffs = {0, 0, 0};
  coeffs.insert(coeffs.end(), offsets.begin(), offsets.end());

  removeDistortion(imagePt.samp, imagePt.line, ux, uy, coeffs, focalLength,
                   DistortionType::CAHVOR);

  EXPECT_NEAR(ux, 4, 1e-8);
  EXPECT_NEAR(uy, 0, 1e-8);
}

// If coeffs are 0 then this will have the same result as removeDistortion
// with 0 distortion coefficients
TEST_P(CoeffOffsetParameterizedTest, InverseDistortionCahvorTest)
{
  csm::ImageCoord imagePt(0.0, 4.0);

  double dx, dy;
  double focalLength = 1000.0;
  double desiredPrecision = 0.01;
  std::vector<double> offsets = GetParam();
  std::vector<double> coeffs = {0, 0, 0};
  coeffs.insert(coeffs.end(), offsets.begin(), offsets.end());

  applyDistortion(imagePt.samp, imagePt.line, dx, dy, coeffs, focalLength,
                  DistortionType::CAHVOR, desiredPrecision);

  EXPECT_NEAR(dx, 4, 1e-8);
  EXPECT_NEAR(dy, 0, 1e-8);
}

TEST_P(CoeffOffsetParameterizedTest, InverseOnesCoeffsCahvorTest)
{
  csm::ImageCoord imagePt(0.0, 4.0);

  double dx, dy;
  double focalLength = 1000.0;
  double desiredPrecision = 0.01;
  std::vector<double> offsets = GetParam();
  std::vector<double> coeffs = {1, 1, 1};
  coeffs.insert(coeffs.end(), offsets.begin(), offsets.end());

  applyDistortion(imagePt.samp, imagePt.line, dx, dy, coeffs, focalLength,
                  DistortionType::CAHVOR, desiredPrecision);

  EXPECT_NEAR(dx, 4, 1e-8);
  EXPECT_NEAR(dy, 0, 1e-8);
}

INSTANTIATE_TEST_SUITE_P(RadTanInversionTest, RadTanTest,
                         ::testing::Values(std::vector<double>(0, 0)));

TEST_P(RadTanTest, RadTanInversionTest)
{

  // Initialize radtan distortion coefficients (k1, k2, p1, p2, k3)  
  std::vector<double> distCoeffs= {0.000031, -0.000056, 1.3e-5, -1.7e-6, 2.9e-8};
  
  double ux = 5.0, uy = 6.0; 
  double focalLength = 1000.0;
  // Compute distortion 
  double dx, dy;
  applyDistortion(ux, uy, dx, dy, distCoeffs, focalLength,  
                  DistortionType::RADTAN, 1e-8, 1e-8);
  
  // Remove distortion (undistort)
  double ux2, uy2;
  removeDistortion(dx, dy, ux2, uy2, distCoeffs, focalLength,
                   DistortionType::RADTAN, 1e-8);
  
  EXPECT_NEAR(dx, 5.0000006007539586, 1e-8);
  EXPECT_NEAR(dy, 6.0000016383447505, 1e-8);
  EXPECT_NEAR(ux2, ux, 1e-8);
  EXPECT_NEAR(uy2, uy, 1e-8);
}

// Tests for KPLO ShadowCam single-coefficient y-only cubic distortion.
// Closed-form distorted -> undistorted:  uy = dy * (1 + dk1 * dy^2);  ux = dx
// Inverse via fixed-point iteration on:  yt_{n+1} = uy / (1 + dk1 * yt_n^2)
// Coefficient sign and magnitude from real IK kplo_shadowcam_v00.ti
// (INS-155151_OD_K = -1.741e-05).

TEST(KploShadowCam, removeDistortionClosedForm) {
  double ux = 0.0, uy = 0.0;
  double focalLength = 699.275;  // mm, from IK
  std::vector<double> coeffs = {-1.741e-05};

  // dy = -18.69 mm corresponds to the left edge of the 3144-sample detector
  // at 12 micron pixel pitch (off-axis by 1557 samples).
  removeDistortion(0.0, -18.69, ux, uy, coeffs, focalLength,
                   DistortionType::KPLOSHADOWCAM, 1e-10);
  // uy = dy * (1 + dk1 * dy^2)
  //    = -18.69 * (1 + (-1.741e-5) * 349.3161)
  //    = -18.69 * (1 - 0.0060816...)
  //    = -18.5763...
  EXPECT_NEAR(ux, 0.0, 1e-12);
  EXPECT_NEAR(uy, -18.69 * (1.0 + (-1.741e-05) * 18.69 * 18.69), 1e-10);
}

TEST(KploShadowCam, applyDistortionAtBoresight) {
  double dx = 99.0, dy = 99.0;
  double focalLength = 699.275;
  std::vector<double> coeffs = {-1.741e-05};

  // On-axis (uy = 0): distorted = undistorted exactly.
  applyDistortion(0.0, 0.0, dx, dy, coeffs, focalLength,
                  DistortionType::KPLOSHADOWCAM, 1e-10);
  EXPECT_NEAR(dx, 0.0, 1e-12);
  EXPECT_NEAR(dy, 0.0, 1e-12);
}

TEST(KploShadowCam, roundTripApplyRemove) {
  double focalLength = 699.275;
  std::vector<double> coeffs = {-1.741e-05};

  for (double uyTest : {-18.0, -5.0, -0.5, 0.5, 5.0, 18.0}) {
    double dx, dy, ux2, uy2;
    applyDistortion(0.0, uyTest, dx, dy, coeffs, focalLength,
                    DistortionType::KPLOSHADOWCAM, 1e-10);
    removeDistortion(dx, dy, ux2, uy2, coeffs, focalLength,
                     DistortionType::KPLOSHADOWCAM, 1e-10);
    EXPECT_NEAR(ux2, 0.0, 1e-9);
    EXPECT_NEAR(uy2, uyTest, 1e-8);
  }
}

TEST(KploShadowCam, roundTripRemoveApply) {
  double focalLength = 699.275;
  std::vector<double> coeffs = {-1.741e-05};

  for (double dyTest : {-18.0, -5.0, -0.5, 0.5, 5.0, 18.0}) {
    double ux, uy, dx2, dy2;
    removeDistortion(0.0, dyTest, ux, uy, coeffs, focalLength,
                     DistortionType::KPLOSHADOWCAM, 1e-10);
    applyDistortion(ux, uy, dx2, dy2, coeffs, focalLength,
                    DistortionType::KPLOSHADOWCAM, 1e-10);
    EXPECT_NEAR(dx2, 0.0, 1e-9);
    EXPECT_NEAR(dy2, dyTest, 1e-8);
  }
}

TEST(KploShadowCam, xPassesThroughUnchanged) {
  // ShadowCam distortion is y-only; ux must equal dx for any dx.
  double focalLength = 699.275;
  std::vector<double> coeffs = {-1.741e-05};
  double ux, uy;

  removeDistortion(3.7, -10.0, ux, uy, coeffs, focalLength,
                   DistortionType::KPLOSHADOWCAM, 1e-10);
  EXPECT_DOUBLE_EQ(ux, 3.7);

  double dx, dy;
  applyDistortion(3.7, -10.0, dx, dy, coeffs, focalLength,
                  DistortionType::KPLOSHADOWCAM, 1e-10);
  EXPECT_DOUBLE_EQ(dx, 3.7);
}

TEST(KploShadowCam, zeroCoefficientIsIdentity) {
  double focalLength = 699.275;
  std::vector<double> coeffs = {0.0};
  double ux, uy, dx, dy;

  removeDistortion(2.0, -12.5, ux, uy, coeffs, focalLength,
                   DistortionType::KPLOSHADOWCAM, 1e-10);
  EXPECT_DOUBLE_EQ(ux, 2.0);
  EXPECT_DOUBLE_EQ(uy, -12.5);

  applyDistortion(2.0, -12.5, dx, dy, coeffs, focalLength,
                  DistortionType::KPLOSHADOWCAM, 1e-10);
  EXPECT_DOUBLE_EQ(dx, 2.0);
  EXPECT_NEAR(dy, -12.5, 1e-12);
}

TEST(KploShadowCam, outOfBoundsPassesThroughOnApply) {
  // |uy| > 40 short-circuits and passes (ux, uy) through unchanged.
  double focalLength = 699.275;
  std::vector<double> coeffs = {-1.741e-05};
  double dx, dy;

  applyDistortion(1.0, 100.0, dx, dy, coeffs, focalLength,
                  DistortionType::KPLOSHADOWCAM, 1e-10);
  EXPECT_DOUBLE_EQ(dx, 1.0);
  EXPECT_DOUBLE_EQ(dy, 100.0);
}

// Tests for the string -> DistortionType and int -> DistortionType mappers.
// The int-mapper case (getDistortionModel(int, ...)) is the one
// UsgsAstroLsSensorModel::constructStateFromIsd calls; missing the
// KPLOSHADOWCAM case caused m_distortionType to default to TRANSVERSE,
// which manifested as a 2x cross-track scaling bug at runtime.

TEST(KploShadowCamMapping, stringToEnum) {
  nlohmann::json isd;
  isd["optical_distortion"]["kplo_shadowcam"]["coefficients"] = {-1.741e-5};
  DistortionType dt = getDistortionModel(isd);
  EXPECT_EQ(dt, DistortionType::KPLOSHADOWCAM);
}

TEST(KploShadowCamMapping, intToEnum) {
  // ale::DistortionType::KPLOSHADOWCAM is the last enum entry; its integer
  // value follows RADTAN. The mapper must explicitly map it, NOT fall
  // through to the default TRANSVERSE.
  DistortionType dt = getDistortionModel(
      static_cast<int>(ale::DistortionType::KPLOSHADOWCAM));
  EXPECT_EQ(dt, DistortionType::KPLOSHADOWCAM);
  EXPECT_NE(dt, DistortionType::TRANSVERSE);
}

TEST(KploShadowCamMapping, coefficientsFromIsd) {
  nlohmann::json isd;
  isd["optical_distortion"]["kplo_shadowcam"]["coefficients"] = {-1.741e-5};
  std::vector<double> coeffs = getDistortionCoeffs(isd);
  ASSERT_EQ(coeffs.size(), 1u);
  EXPECT_NEAR(coeffs[0], -1.741e-5, 1e-12);
}
