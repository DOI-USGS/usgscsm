#ifndef Fixtures_h
#define Fixtures_h

#include "UsgsAstroFramePlugin.h"
#include "UsgsAstroFrameSensorModel.h"

#include <json.hpp>

#include <sstream>
#include <fstream>

#include <gtest/gtest.h>

using json = nlohmann::json;

// Should this be positioned somewhere in the public API?
inline void jsonToIsd(json &object, csm::Isd &isd, std::string prefix="") {
  for (json::iterator it = object.begin(); it != object.end(); ++it) {
     json jsonValue = it.value();
     if (jsonValue.is_array()) {
        for (int i = 0; i < jsonValue.size(); i++) {
           isd.addParam(prefix+it.key(), jsonValue[i].dump());
        }
     }
     else {
        isd.addParam(prefix+it.key(), jsonValue.dump());
     }
  }
}

class FrameSensorModel : public ::testing::Test {
   protected:

      UsgsAstroFrameSensorModel *sensorModel;

      void SetUp() override {
         sensorModel = NULL;
         csm::Isd isd;
         std::ifstream isdFile("data/simpleFramerISD.json");
         json jsonIsd = json::parse(isdFile);
         jsonToIsd(jsonIsd, isd);
         isdFile.close();
         UsgsAstroFramePlugin frameCameraPlugin;
         csm::Model *model = frameCameraPlugin.constructModelFromISD(
               isd,
               "USGS_ASTRO_FRAME_SENSOR_MODEL");
         sensorModel = dynamic_cast<UsgsAstroFrameSensorModel *>(model);

         ASSERT_NE(sensorModel, nullptr);
      }

      void TearDown() override {
         if (sensorModel) {
            delete sensorModel;
            sensorModel = NULL;
         }
      }
};

class FrameIsdTest : public ::testing::Test {
   protected:

      csm::Isd isd;

   virtual void SetUp() {
      std::ifstream isdFile("data/simpleFramerISD.json");
      json jsonIsd = json::parse(isdFile);
      jsonToIsd(jsonIsd, isd);
      isdFile.close();
   }
};
#endif
