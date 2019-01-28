#include "UsgsAstroPlugin.h"
#include "UsgsAstroLsSensorModel.h"

#include <json.hpp>
#include <gtest/gtest.h>

#include "Fixtures.h"

using json = nlohmann::json;


TEST(LineScanSensorModel, Distortion) {
  csm::Isd isd;
  UsgsAstroLsSensorModel *sensorModel = NULL;

  isd.setFilename("data/constVelocityLineScan.img");
  UsgsAstroPlugin plugin;

  csm::Model *model = plugin.constructModelFromISD(
        isd,
        "USGS_ASTRO_LINE_SCANNER_SENSOR_MODEL");
  sensorModel = dynamic_cast<UsgsAstroLsSensorModel *>(model);

  ASSERT_NE(sensorModel, nullptr);

  csm::ImageCoord imagePt(7.5,8);
  csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
  csm::ImageCoord imageReprojPt = sensorModel->groundToImage(groundPt);

  EXPECT_DOUBLE_EQ(imagePt.line, imageReprojPt.line);
  EXPECT_DOUBLE_EQ(imagePt.samp, imageReprojPt.samp);
}
