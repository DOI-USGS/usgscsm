#ifndef Fixtures_h
#define Fixtures_h

#include "UsgsAstroPlugin.h"
#include "UsgsAstroFrameSensorModel.h"
#include "UsgsAstroLsSensorModel.h"
#include "UsgsAstroSarSensorModel.h"

#include <json/json.hpp>

#include <map>
#include <sstream>
#include <string>
#include <fstream>

#include <gtest/gtest.h>

#include <spdlog/sinks/ostream_sink.h>

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
     else if (jsonValue.is_string()) {
        isd.addParam(prefix+it.key(), jsonValue.get<std::string>());
     }
     else {
        isd.addParam(prefix+it.key(), jsonValue.dump());
     }
  }
}

//////////Framing Camera Sensor Model Fixtures//////////

class FrameSensorModel : public ::testing::Test {
   protected:
      csm::Isd isd;
      UsgsAstroFrameSensorModel *sensorModel;

      void SetUp() override {
         sensorModel = NULL;

         isd.setFilename("data/simpleFramerISD.img");
         UsgsAstroPlugin frameCameraPlugin;

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

class FrameSensorModelLogging : public ::testing::Test {
   protected:
      csm::Isd isd;
      UsgsAstroFrameSensorModel *sensorModel;
      std::ostringstream oss;

      void SetUp() override {
         sensorModel = NULL;

         isd.setFilename("data/simpleFramerISD.img");
         UsgsAstroPlugin frameCameraPlugin;

         csm::Model *model = frameCameraPlugin.constructModelFromISD(
               isd,
               "USGS_ASTRO_FRAME_SENSOR_MODEL");
         sensorModel = dynamic_cast<UsgsAstroFrameSensorModel *>(model);

         ASSERT_NE(sensorModel, nullptr);

         auto ostream_sink = std::make_shared<spdlog::sinks::ostream_sink_mt> (oss);
         // We need a unique ID for the sensor model so that we don't have
         // logger name collisions. Use the sensor model's memory addresss.
         std::uintptr_t sensorId = reinterpret_cast<std::uintptr_t>(sensorModel);
         auto logger = std::make_shared<spdlog::logger>(std::to_string(sensorId), ostream_sink);
         sensorModel->setLogger(logger);
      }

      void TearDown() override {
         if (sensorModel) {
            delete sensorModel;
            sensorModel = NULL;
         }

         EXPECT_FALSE(oss.str().empty());
      }
};

class OrbitalFrameSensorModel : public ::testing::Test {
   protected:
      csm::Isd isd;
      UsgsAstroFrameSensorModel *sensorModel;

      void SetUp() override {
         sensorModel = NULL;

         isd.setFilename("data/orbitalFramer.img");
         UsgsAstroPlugin frameCameraPlugin;

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
      isd.setFilename("data/simpleFramerISD.img");
   }
};

class ConstVelLineScanIsdTest : public ::testing::Test {
   protected:
      csm::Isd isd;

   virtual void SetUp() {
      isd.setFilename("data/constVelocityLineScan.img");
   }
};

class ImageCoordParameterizedTest : public ::testing::TestWithParam<csm::ImageCoord> {};

class FramerParameterizedTest : public ::testing::TestWithParam<csm::ImageCoord> {

protected:
  csm::Isd isd;

  std::string printIsd(csm::Isd &localIsd) {
    std::string str;
    std::multimap<std::string,std::string> isdmap= localIsd.parameters();
    for (auto it = isdmap.begin(); it != isdmap.end();++it){
      str.append(it->first);
      str.append(":");
      str.append(it->second);
    }
    return str;
  }
  UsgsAstroFrameSensorModel* createModel(csm::Isd &modifiedIsd) {

    UsgsAstroPlugin frameCameraPlugin;
    csm::Model *model = frameCameraPlugin.constructModelFromISD(
        modifiedIsd,"USGS_ASTRO_FRAME_SENSOR_MODEL");

    UsgsAstroFrameSensorModel* sensorModel = dynamic_cast<UsgsAstroFrameSensorModel *>(model);

    if (sensorModel)
      return sensorModel;
    else
      return nullptr;
  }


  virtual void SetUp() {
    isd.setFilename("data/simpleFramerISD.img");
  };
};

class FrameStateTest : public ::testing::Test {
  protected:
    csm::Isd isd;
    UsgsAstroFrameSensorModel* createModifiedStateSensorModel(std::string key, double newValue) {
      UsgsAstroPlugin cameraPlugin;
      csm::Model *model = cameraPlugin.constructModelFromISD(isd,"USGS_ASTRO_FRAME_SENSOR_MODEL");

      UsgsAstroFrameSensorModel* sensorModel = dynamic_cast<UsgsAstroFrameSensorModel *>(model);
      if (sensorModel) {
        sensorModel->getModelState();
        std::string modelState = sensorModel->getModelState();
        auto state = json::parse(modelState);
        state[key] = newValue;
        sensorModel->replaceModelState(state.dump());

        return sensorModel;
      }
      else {
        return nullptr;
      }
    }

    void SetUp() override {
      isd.setFilename("data/simpleFramerISD.img");
    }
};

//////////Line Scan Camera Sensor Model Fixtures//////////

class ConstVelocityLineScanSensorModel : public ::testing::Test {
   protected:
      csm::Isd isd;
      UsgsAstroLsSensorModel *sensorModel;

      void SetUp() override {
         sensorModel = NULL;

         isd.setFilename("data/constVelocityLineScan.img");
         UsgsAstroPlugin cameraPlugin;

         csm::Model *model = cameraPlugin.constructModelFromISD(
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

class OrbitalLineScanSensorModel : public ::testing::Test {
   protected:
      csm::Isd isd;
      UsgsAstroLsSensorModel *sensorModel;

      void SetUp() override {
         sensorModel = NULL;

         isd.setFilename("data/orbitalLineScan.img");
         UsgsAstroPlugin cameraPlugin;

         csm::Model *model = cameraPlugin.constructModelFromISD(
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

class TwoLineScanSensorModels : public ::testing::Test {
   protected:
      csm::Isd isd;
      UsgsAstroLsSensorModel *sensorModel1;
      UsgsAstroLsSensorModel *sensorModel2;

      void SetUp() override {
         sensorModel1 = nullptr;
         sensorModel2 = nullptr;

         isd.setFilename("data/orbitalLineScan.img");
         UsgsAstroPlugin cameraPlugin;

         csm::Model *model1 = cameraPlugin.constructModelFromISD(
               isd,
               "USGS_ASTRO_LINE_SCANNER_SENSOR_MODEL");
         sensorModel1 = dynamic_cast<UsgsAstroLsSensorModel *>(model1);
         csm::Model *model2 = cameraPlugin.constructModelFromISD(
               isd,
               "USGS_ASTRO_LINE_SCANNER_SENSOR_MODEL");
         sensorModel2 = dynamic_cast<UsgsAstroLsSensorModel *>(model2);

         ASSERT_NE(sensorModel1, nullptr);
         ASSERT_NE(sensorModel2, nullptr);
      }

      void TearDown() override {
         if (sensorModel1) {
            delete sensorModel1;
            sensorModel1 = nullptr;
         }
         if (sensorModel2) {
            delete sensorModel2;
            sensorModel2 = nullptr;
         }
      }
};

//////////////////
// SAR Fixtures //
//////////////////

class SarIsdTest : public ::testing::Test {
   protected:
      csm::Isd isd;

   virtual void SetUp() {
      isd.setFilename("data/orbitalSar.img");
   }
};

class SarSensorModel : public ::testing::Test {
   protected:
      csm::Isd isd;
      UsgsAstroSarSensorModel *sensorModel;

      void SetUp() override {
         sensorModel = NULL;

         isd.setFilename("data/orbitalSar.img");
         UsgsAstroPlugin sarCameraPlugin;

         csm::Model *model = sarCameraPlugin.constructModelFromISD(
               isd,
               "USGS_ASTRO_SAR_SENSOR_MODEL");
         sensorModel = dynamic_cast<UsgsAstroSarSensorModel *>(model);
         ASSERT_NE(sensorModel, nullptr);
      }

      void TearDown() override {
         if (sensorModel) {
            delete sensorModel;
            sensorModel = NULL;
         }
      }
};

#endif
