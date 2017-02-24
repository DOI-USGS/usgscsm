#include <string>
#include <csm/Isd.h>

#include <gtest/gtest.h>

#include <MdisPlugin.h>
#include <MdisNacSensorModel.h>
#include "MdisNacSensorModelTest.h"

bool MdisNacSensorModelTest::setupFixtureFailed = false;
std::string MdisNacSensorModelTest::setupFixtureError;
csm::Isd *MdisNacSensorModelTest::isd = nullptr;
std::string MdisNacSensorModelTest::dataFile;
MdisPlugin MdisNacSensorModelTest::mdisPlugin;
MdisNacSensorModel *MdisNacSensorModelTest::mdisModel = nullptr;

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
TEST_F(MdisNacSensorModelTest, imageToGroundCenter) {

  // gtest #247 work-around
  if (setupFixtureFailed) {
    FAIL() << setupFixtureError;
  }

  csm::ImageCoord point(512., 512.);
  double height = 0.0;
  csm::EcefCoord xyz = mdisModel->imageToGround(point, height);
  double truth[] = { 1129.25*1000, -1599.26*1000, 1455.28*1000 };
  EXPECT_EQ(truth[0], xyz.x);
  EXPECT_EQ(truth[1], xyz.y);
  EXPECT_EQ(truth[2], xyz.z);
}

TEST_F(MdisNacSensorModelTest, imageToGroundOffCenter){
  if (setupFixtureFailed) {
    FAIL() << setupFixtureError;
  }
  
  csm::ImageCoord point(100, 100);
  double height = 0.0;
  csm::EcefCoord xyz = mdisModel->imageToGround(point, height);
  double truth[] = { 1115.95*1000, -1603.44*1000, 1460.93*1000 };
  EXPECT_EQ(truth[0], xyz.x);
  EXPECT_EQ(truth[1], xyz.y);
  EXPECT_EQ(truth[2], xyz.z);
}

// Test getIlluminationDirection
TEST_F(MdisNacSensorModelTest, getIlluminationDirection1) {
  // gtest #247 work-around
  if (setupFixtureFailed) {
    FAIL() << setupFixtureError;
  }
  
  // sun position based on EN1007907102M.json
  // -31648725087.588726, -60633907522.72863, -38729485.77334732
  csm::EcefCoord northPole { 0., 0., 2439.4 * 1000 };
  csm::EcefVector illuminationDirection = mdisModel->getIlluminationDirection(northPole);
  EXPECT_NEAR(31648725087.588726, illuminationDirection.x, 0.1);
  EXPECT_NEAR(60633907522.72863, illuminationDirection.y, 0.1);
  EXPECT_NEAR(2439.4*1000 - -38729485.77334732, illuminationDirection.z, 0.1);
}


// Test getSensorPosition(ImageCoord)
TEST_F(MdisNacSensorModelTest, getSensorPositionCoord) {
  // gtest #247 work-around
  if (setupFixtureFailed) {
    FAIL() << setupFixtureError;
  }
  
  csm::EcefCoord sensorPos = mdisModel->getSensorPosition(csm::ImageCoord(512.0, 512.0));
  EXPECT_NEAR(1728181.03, sensorPos.x, 0.01);
  EXPECT_NEAR(-2088202.59, sensorPos.y, 0.01);
  EXPECT_NEAR(2082707.61, sensorPos.z, 0.01);
}

TEST_F(MdisNacSensorModelTest, getSensorPositionCoordOutOfBounds) {
  // gtest #247 work-around
  if (setupFixtureFailed) {
    FAIL() << setupFixtureError;
  }
  
  // Test all possibilites of logical condition, image size = 1024x1024
  EXPECT_THROW({
    // Line is negative
    csm::EcefCoord sensorPos = mdisModel->getSensorPosition(csm::ImageCoord(-1.0, 1.0));
  },
  csm::Error);
  EXPECT_THROW({
    // Sample is negative
    csm::EcefCoord sensorPos = mdisModel->getSensorPosition(csm::ImageCoord(1.0, -1.0));
  },
  csm::Error);
  EXPECT_THROW({
    // Line > 1024
    csm::EcefCoord sensorPos = mdisModel->getSensorPosition(csm::ImageCoord(1100.0, 1.0));
  },
  csm::Error);
  EXPECT_THROW({
    // Sample > 1024
    csm::EcefCoord sensorPos = mdisModel->getSensorPosition(csm::ImageCoord(1.0, 1100.0));
  },
  csm::Error);
}

// Test imageToProximateImagingLocus
TEST_F(MdisNacSensorModelTest, imageToProximateImagingLocus1) {
  // gtest #247 work-around
  if (setupFixtureFailed) {
    FAIL() << setupFixtureError;
  }
  
  csm::ImageCoord point(512.0, 512.0);
  csm::EcefCoord ground(0,0,0);
  csm::EcefLocus proximateLocus = mdisModel->imageToProximateImagingLocus(point, ground);
  
  double spacecraftX = atof(isd->param("x_sensor_origin").c_str());
  double spacecraftY = atof(isd->param("y_sensor_origin").c_str());
  double spacecraftZ = atof(isd->param("z_sensor_origin").c_str());
  EXPECT_EQ(spacecraftX, proximateLocus.point.x);
  EXPECT_EQ(spacecraftY, proximateLocus.point.y);
  EXPECT_EQ(spacecraftZ, proximateLocus.point.z);
  
  EXPECT_NEAR(-0.6015027, proximateLocus.direction.x, tolerance);
  EXPECT_NEAR(0.4910591, proximateLocus.direction.y, tolerance);
  EXPECT_NEAR(-0.630123, proximateLocus.direction.z, tolerance);
}

TEST_F(MdisNacSensorModelTest, distortionModel2) {
  double dx = -6.30;
  double dy = 6.40;
  double udx = 0.0;
  double udy = 0.0;
  double isis3_udx = -6.3036234000160273893698104075156152248383;
  double isis3_udy = 6.3445144408882310216313271666876971721649;
  testMath.setFocalPlane(dx,dy,udx,udy);

  EXPECT_NEAR(udx,isis3_udx,tolerance);
  EXPECT_NEAR(udy,isis3_udy,tolerance);

}


TEST_F(MdisNacSensorModelTest, getImageStart) {
  csm::ImageCoord start = mdisModel->getImageStart();
  EXPECT_EQ(start.line, 1.0);
  EXPECT_EQ(start.samp, 9.0);
}


TEST_F(MdisNacSensorModelTest, getImageSize) {
  csm::ImageVector size = mdisModel->getImageSize();
  EXPECT_EQ(size.line, 1024);
  EXPECT_EQ(size.samp, 1024);
}


TEST_F(MdisNacSensorModelTest, getImageTime) {
  csm::ImageCoord point;
  point.samp = 500;
  point.line = 500;
  double time = mdisModel->getImageTime(point);
  EXPECT_NEAR(time, 418855170.49299997, tolerance);
}


TEST_F(MdisNacSensorModelTest, getSensorPosition) {
  csm::ImageCoord point;
  point.samp = 500;
  point.line = 500;
  csm::EcefCoord position = mdisModel->getSensorPosition(point);
  EXPECT_NEAR(position.x, 1728357.7031238307, tolerance);
  EXPECT_NEAR(position.y, -2088409.0061042644, tolerance);
  EXPECT_NEAR(position.z, 2082873.9280557402, tolerance);
}

