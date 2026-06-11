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
  EXPECT_NEAR(ux, 0.0, 1e-12);
  EXPECT_NEAR(uy, -18.69 * (1.0 + (-1.741e-05) * 18.69 * 18.69), 1e-10);
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

TEST(KploShadowCamMapping, stringToEnum) {
  nlohmann::json isd;
  isd["optical_distortion"]["kplo_shadowcam"]["coefficients"] = {-1.741e-5};
  DistortionType dt = getDistortionModel(isd);
  EXPECT_EQ(dt, DistortionType::KPLOSHADOWCAM);
}

TEST(KploShadowCamMapping, intToEnum) {
  // Verify that the integer-mapping overload converts
  // ale::DistortionType::KPLOSHADOWCAM to USGSCSM's matching
  // DistortionType::KPLOSHADOWCAM. ALE and USGSCSM maintain independent
  // DistortionType enums; this test pins the cross-library mapping.
  DistortionType dt = getDistortionModel(
      static_cast<int>(ale::DistortionType::KPLOSHADOWCAM));
  EXPECT_EQ(dt, DistortionType::KPLOSHADOWCAM);
}

// TGO CaSSIS rational ratio-of-quadratics distortion (Tulyakov/Ivanov, EPFL),
// matching ISIS TgoCassisDistortionMap. With chi = [x^2, x*y, y^2, x, y, 1],
// each direction is x_out = (A1.chi)/(A3.chi), y_out = (A2.chi)/(A3.chi). The 36
// coefficients are A1_corr, A2_corr, A3_corr (distorted -> undistorted, used by
// removeDistortion) then A1_dist, A2_dist, A3_dist (undistorted -> distorted,
// used by applyDistortion), 6 each. Values are the real coefficients from the
// ISIS addendum tgoCassisAddendum007.ti. Expected outputs below were computed
// independently from the rational formula.
static const std::vector<double> cassisCoeffs = {
    0.00376130530948266,  -0.0134154156065812,   -1.86749521007237e-05,
    1.00021352681836,     -0.000432362371703953, -0.000948065735350123,
    9.9842559363676e-05,   0.00373543707958162,  -0.0133299918873929,
   -0.000215311328389359,  0.995296015537294,    -0.0183542717710778,
   -3.13320167004204e-05, -7.35655125749807e-06, -1.57664245066771e-05,
    0.00373549465439151,  -0.0141671946930935,    1.0,
    0.00213658795560622,  -0.00711785765064197,   1.10355974742147e-05,
    0.573607182625377,     0.000250884350194894,  0.000550623913037132,
   -5.69725741015406e-05,  0.00215155905679149,  -0.00716392991767185,
    0.000124152787728634,  0.576459544392426,     0.010576940564854,
    1.78250771483506e-05,  4.24592743471094e-06,  9.51220699036653e-06,
    0.00215158425420738,  -0.0066835595774833,    0.573741540971609};

TEST(Cassis, removeDistortion) {
  double ux, uy;
  removeDistortion(2.0, -8.0, ux, uy, cassisCoeffs, 874.9, DistortionType::CASSIS);
  EXPECT_NEAR(ux, 1.9927225905155812, 1e-9);
  EXPECT_NEAR(uy, -7.942225998826645, 1e-9);
}

TEST(Cassis, applyDistortion) {
  double dx, dy;
  applyDistortion(2.0, -8.0, dx, dy, cassisCoeffs, 874.9, DistortionType::CASSIS);
  EXPECT_NEAR(dx, 2.007349177383467, 1e-9);
  EXPECT_NEAR(dy, -8.058521300253895, 1e-9);
}

TEST(Cassis, roundTripApplyRemove) {
  // CORR and DIST are independent EPFL fits, not exact inverses, so the round
  // trip recovers the input to the fit residual (~5e-6 mm), as in ISIS.
  for (auto uv : std::vector<std::pair<double, double>>{
           {1.5, -7.0}, {-3.0, -9.0}, {0.5, -8.5}}) {
    double dx, dy, ux, uy;
    applyDistortion(uv.first, uv.second, dx, dy, cassisCoeffs, 874.9,
                    DistortionType::CASSIS);
    removeDistortion(dx, dy, ux, uy, cassisCoeffs, 874.9, DistortionType::CASSIS);
    EXPECT_NEAR(ux, uv.first, 1e-4);
    EXPECT_NEAR(uy, uv.second, 1e-4);
  }
}

TEST(Cassis, offCcdIdentity) {
  // Outside the CCD (|x| or |y| > ~10.44 mm) the model returns the input
  // unchanged, matching ISIS.
  double ux, uy, dx, dy;
  removeDistortion(15.0, 0.0, ux, uy, cassisCoeffs, 874.9, DistortionType::CASSIS);
  EXPECT_EQ(ux, 15.0);
  EXPECT_EQ(uy, 0.0);
  applyDistortion(0.0, -12.0, dx, dy, cassisCoeffs, 874.9, DistortionType::CASSIS);
  EXPECT_EQ(dx, 0.0);
  EXPECT_EQ(dy, -12.0);
}

TEST(Cassis, wrongCoefficientCount) {
  double ux, uy;
  std::vector<double> badCoeffs(10, 0.0);
  EXPECT_ANY_THROW(removeDistortion(1.0, 1.0, ux, uy, badCoeffs, 874.9,
                                    DistortionType::CASSIS));
}

TEST(CassisMapping, stringToEnum) {
  nlohmann::json isd;
  isd["optical_distortion"]["cassis"]["coefficients"] = cassisCoeffs;
  EXPECT_EQ(getDistortionModel(isd), DistortionType::CASSIS);
}

TEST(CassisMapping, intToEnum) {
  // ALE and USGSCSM keep independent DistortionType enums; this pins the
  // cross-library integer mapping for CASSIS.
  DistortionType dt =
      getDistortionModel(static_cast<int>(ale::DistortionType::CASSIS));
  EXPECT_EQ(dt, DistortionType::CASSIS);
}
