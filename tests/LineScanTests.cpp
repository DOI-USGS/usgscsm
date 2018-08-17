#include "UsgsAstroLsPlugin.h"
#include "UsgsAstroLsSensorModel.h"

#include <json.hpp>

#include <sstream>
#include <fstream>

#include <gtest/gtest.h>

using json = nlohmann::json;

class CtxSensorModel : public ::testing::Test {
   protected:

      UsgsAstroLsSensorModel *sensorModel;

      void SetUp() override {
         std::ifstream isdFile("data/rectJ03_045994_1986_XN_18N282W_v6_8bit_keywords.json");
         json jsonIsd = json::parse(isdFile);
         csm::Isd isd;
         for (json::iterator it = jsonIsd.begin(); it != jsonIsd.end(); ++it) {
            json jsonValue = it.value();
            if (jsonValue.size() > 1) {
               for (int i = 0; i < jsonValue.size(); i++) {
                  isd.addParam(it.key(), jsonValue[i].dump());
               }
            }
            else {
               isd.addParam(it.key(), jsonValue.dump());
            }
         }
         isdFile.close();

         UsgsAstroLsPlugin lsPlugin;
         csm::Model *model = lsPlugin.constructModelFromISD(
               isd,
               "USGS_ASTRO_LINE_SCANNER_SENSOR_MODEL");
         sensorModel = dynamic_cast<UsgsAstroLsSensorModel *>(model);

         ASSERT_NE(sensorModel, nullptr);
      }

      void TearDown() override {
         if (sensorModel) {
            delete sensorModel;
            sensorModel = NULL;
         }
      }
};

TEST_F(CtxSensorModel, CenterTop) {
   csm::ImageCoord imagePt(100.0, 2500.0);
   csm::EcefCoord groundPt = sensorModel->imageToGround(imagePt, 0.0);
   EXPECT_EQ(groundPt.x, 697590.1570627667);
   EXPECT_EQ(groundPt.y, 3161700.1496232613);
   EXPECT_EQ(groundPt.z, 3316425.783307383);
}
