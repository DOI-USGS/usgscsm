#include <string>

#include <csm/Isd.h>

#include <gtest/gtest.h>

#include <MdisPlugin.h>
#include <MdisNacSensorModel.h>

#include "MdisWacSensorModelTest.h"

bool MdisWacSensorModelTest::setupFixtureFailed = false;
std::string MdisWacSensorModelTest::setupFixtureError;
csm::Isd *MdisWacSensorModelTest::isd = nullptr;
std::string MdisWacSensorModelTest::dataFile;
MdisPlugin MdisWacSensorModelTest::mdisPlugin;
MdisNacSensorModel *MdisWacSensorModelTest::mdisModel = nullptr;

/*
 * Test imageToGround - truth extracted as follows:
 * setisis isis3
 * qview /work/projects/IAA_camera/data/EN100790102M.cub
 * F (selects "Find Tool")
 * On top toolbar, select "Find Point"
 * Type in 513, 513 for Sample/Line (ISIS3 pixel center = 1,1)
 * Click "Record Point"
 * Check "XYZ" -> { 1132.18, -1597.75, 1455.66 }
 */
TEST_F(MdisWacSensorModelTest, imageToGroundCenter) {

  // gtest #247 work-around
  if (setupFixtureFailed) {
    FAIL() << setupFixtureError;
  }

  csm::ImageCoord point(512, 512);
  double height = 0.0;
  csm::EcefCoord xyz = mdisModel->imageToGround(point, height);
  //std::cout << std::cout.precision(15) << "\n(512,512) XYZ: "
  //          << xyz.x << ", " << xyz.y << ", " << xyz.z << "\n\n";
  double truth[] = {-73589.5516508502, 562548.342040933, 2372508.44060771};
  EXPECT_NEAR(truth[0], xyz.x, 0.01);
  EXPECT_NEAR(truth[1], xyz.y, 0.01);
  EXPECT_NEAR(truth[2], xyz.z, 0.01);
}

TEST_F(MdisWacSensorModelTest, imageToGroundOffCenter){
  if (setupFixtureFailed) {
    FAIL() << setupFixtureError;
  }

  csm::ImageCoord point(100, 100);
  double height = 0.0;
  csm::EcefCoord xyz = mdisModel->imageToGround(point, height);
  //std::cout << std::cout.precision(15) << "\n(100,100) XYZ: "
  //          << xyz.x << ", " << xyz.y << ", " << xyz.z << "\n\n";
  double truth[] = {-48020.2164819883, 539322.805489926, 2378549.41724731};
  EXPECT_NEAR(truth[0], xyz.x, 0.01);
  EXPECT_NEAR(truth[1], xyz.y, 0.01);
  EXPECT_NEAR(truth[2], xyz.z, 0.01);
}
// Test groundToImage
TEST_F(MdisWacSensorModelTest, groundToImageCenter) {
  // gtest #247 work-around
  if (setupFixtureFailed) {
    FAIL() << setupFixtureError;
  }

  csm::EcefCoord xyz {-73589.5516508502, 562548.342040933, 2372508.44060771};
  csm::ImageCoord pt = mdisModel->groundToImage(xyz);
  // Use 1/2 pixel as tolerance
  EXPECT_NEAR(512, pt.line, 0.1);
  EXPECT_NEAR(512, pt.samp, 0.1);
}

TEST_F(MdisWacSensorModelTest, groundToImageOffCenter) {
  if (setupFixtureFailed) {
    FAIL() << setupFixtureError;
  }

  csm::EcefCoord xyz {-48020.2164819883, 539322.805489926, 2378549.41724731};
  xyz = mdisModel->imageToGround(csm::ImageCoord(100,100),0);
  csm::ImageCoord pt = mdisModel->groundToImage(xyz);
  EXPECT_NEAR(100, pt.line, 0.1);
  EXPECT_NEAR(100, pt.samp, 0.1);
}

