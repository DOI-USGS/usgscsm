#ifndef Fixtures_h
#define Fixtures_h

#include "UsgsAstroFrameSensorModel.h"
#include "UsgsAstroLsSensorModel.h"
#include "UsgsAstroPlugin.h"
#include "UsgsAstroPushFrameSensorModel.h"
#include "UsgsAstroSarSensorModel.h"

#include <nlohmann/json.hpp>

#include <fstream>
#include <map>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include <spdlog/sinks/ostream_sink.h>

// Should this be positioned somewhere in the public API?
inline void jsonToIsd(nlohmann::json &object, csm::Isd &isd,
                      std::string prefix = "") {
  for (nlohmann::json::iterator it = object.begin(); it != object.end(); ++it) {
    nlohmann::json jsonValue = it.value();
    if (jsonValue.is_array()) {
      for (int i = 0; i < jsonValue.size(); i++) {
        isd.addParam(prefix + it.key(), jsonValue[i].dump());
      }
    } else if (jsonValue.is_string()) {
      isd.addParam(prefix + it.key(), jsonValue.get<std::string>());
    } else {
      isd.addParam(prefix + it.key(), jsonValue.dump());
    }
  }
}

//////////Framing Camera Sensor Model Fixtures//////////

class FrameSensorModel : public ::testing::Test {
 protected:
  csm::Isd isd;
  std::shared_ptr<csm::Model> model;
  UsgsAstroFrameSensorModel *sensorModel;

  void SetUp() override {
    sensorModel = NULL;

    isd.setFilename("data/simpleFramerISD.img");
    UsgsAstroPlugin frameCameraPlugin;

    model = std::shared_ptr<csm::Model>(frameCameraPlugin.constructModelFromISD(
      isd, UsgsAstroFrameSensorModel::_SENSOR_MODEL_NAME));
    sensorModel = dynamic_cast<UsgsAstroFrameSensorModel *>(model.get());

    ASSERT_NE(sensorModel, nullptr);
  }

  void TearDown() override {
    // The object that sensorModel points to is managed by the smart pointer 'model'.
    sensorModel = NULL;
  }
};

class FrameSensorModelLogging : public ::testing::Test {
 protected:
  csm::Isd isd;
  std::shared_ptr<csm::Model> model;
  UsgsAstroFrameSensorModel *sensorModel;
  std::ostringstream oss;

  void SetUp() override {
    sensorModel = NULL;

    isd.setFilename("data/simpleFramerISD.img");
    UsgsAstroPlugin frameCameraPlugin;

    model = std::shared_ptr<csm::Model>(frameCameraPlugin.constructModelFromISD(
       isd, UsgsAstroFrameSensorModel::_SENSOR_MODEL_NAME));
    sensorModel = dynamic_cast<UsgsAstroFrameSensorModel *>(model.get());

    ASSERT_NE(sensorModel, nullptr);

    auto ostream_sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(oss);
    // We need a unique ID for the sensor model so that we don't have
    // logger name collisions. Use the sensor model's memory addresss.
    std::uintptr_t sensorId = reinterpret_cast<std::uintptr_t>(sensorModel);
    auto logger = std::make_shared<spdlog::logger>(std::to_string(sensorId),
                                                   ostream_sink);
    spdlog::register_logger(logger);

    sensorModel->setLogger(std::to_string(sensorId));
  }

  void TearDown() override {
    if (sensorModel) {
      // Remove the logger from the registry for other tests
      std::uintptr_t sensorId = reinterpret_cast<std::uintptr_t>(sensorModel);
      spdlog::drop(std::to_string(sensorId));

      // No deletion is needed because the resource is owned by the smart pointer 'model'.
      sensorModel = NULL;
    }

    EXPECT_FALSE(oss.str().empty());
  }
};

class OrbitalFrameSensorModel : public ::testing::Test {
 protected:
  csm::Isd isd;
  std::shared_ptr<csm::Model> model;
  UsgsAstroFrameSensorModel *sensorModel;

  void SetUp() override {
    sensorModel = NULL;

    isd.setFilename("data/orbitalFramer.img");
    UsgsAstroPlugin frameCameraPlugin;

    model = std::shared_ptr<csm::Model>(frameCameraPlugin.constructModelFromISD(
      isd, UsgsAstroFrameSensorModel::_SENSOR_MODEL_NAME));
    sensorModel = dynamic_cast<UsgsAstroFrameSensorModel *>(model.get());

    ASSERT_NE(sensorModel, nullptr);
  }

  void TearDown() override {
    // No deletion is needed because the resource is owned by the smart pointer 'model'.
    sensorModel = NULL;
  }
};

class FrameIsdTest : public ::testing::Test {
 protected:
  csm::Isd isd;

  virtual void SetUp() { isd.setFilename("data/simpleFramerISD.img"); }
};

class ConstVelLineScanIsdTest : public ::testing::Test {
 protected:
  csm::Isd isd;

  virtual void SetUp() { isd.setFilename("data/constVelocityLineScan.img"); }
};

class ImageCoordParameterizedTest
    : public ::testing::TestWithParam<csm::ImageCoord> {};

class FramerParameterizedTest
    : public ::testing::TestWithParam<csm::ImageCoord> {
 protected:
  csm::Isd isd;
  std::shared_ptr<csm::Model> model;

  std::string printIsd(csm::Isd &localIsd) {
    std::string str;
    std::multimap<std::string, std::string> isdmap = localIsd.parameters();
    for (auto it = isdmap.begin(); it != isdmap.end(); ++it) {
      str.append(it->first);
      str.append(":");
      str.append(it->second);
    }
    return str;
  }
  UsgsAstroFrameSensorModel *createModel(csm::Isd &modifiedIsd) {
    UsgsAstroPlugin frameCameraPlugin;
    model = std::shared_ptr<csm::Model>(frameCameraPlugin.constructModelFromISD(
      modifiedIsd, UsgsAstroFrameSensorModel::_SENSOR_MODEL_NAME));

    // The object to which sensorModel points is managed by the smart pointer in 'model'.
    // That one will be deleted when this test is over.
    UsgsAstroFrameSensorModel *sensorModel =
      dynamic_cast<UsgsAstroFrameSensorModel *>(model.get());

    if (sensorModel)
      return sensorModel;
    else
      return nullptr;
  }

  virtual void SetUp() { isd.setFilename("data/simpleFramerISD.img"); };
};

class FrameStateTest : public ::testing::Test {
 protected:
  csm::Isd isd;

  // This function will return a pointer which is managed by the caller
  UsgsAstroFrameSensorModel *createModifiedStateSensorModel(std::string key,
                                                            double newValue) {
    UsgsAstroPlugin cameraPlugin;
    csm::Model *model = cameraPlugin.constructModelFromISD(
          isd, UsgsAstroFrameSensorModel::_SENSOR_MODEL_NAME);

    UsgsAstroFrameSensorModel *sensorModel =
      dynamic_cast<UsgsAstroFrameSensorModel *>(model);
    if (sensorModel) {
      sensorModel->getModelState();
      std::string modelState = sensorModel->getModelState();
      auto state = stateAsJson(modelState);
      state[key] = newValue;
      sensorModel->replaceModelState(state.dump());

      return sensorModel;
    } else {
      return nullptr;
    }
  }

  void SetUp() override { isd.setFilename("data/simpleFramerISD.img"); }
};

//////////Line Scan Camera Sensor Model Fixtures//////////

class ConstVelocityLineScanSensorModel : public ::testing::Test {
 protected:
  csm::Isd isd;
  std::shared_ptr<csm::Model> model;
  UsgsAstroLsSensorModel *sensorModel;

  void SetUp() override {
    sensorModel = NULL;

    isd.setFilename("data/constVelocityLineScan.img");
    UsgsAstroPlugin cameraPlugin;

    model = std::shared_ptr<csm::Model>(cameraPlugin.constructModelFromISD(
     isd, UsgsAstroLsSensorModel::_SENSOR_MODEL_NAME));
    sensorModel = dynamic_cast<UsgsAstroLsSensorModel *>(model.get());

    ASSERT_NE(sensorModel, nullptr);
  }

  void TearDown() override {
    // No deletion is necessary since the resource is managed by the smart pointer in 'model'
    if (sensorModel)
      sensorModel = NULL;
  }
};

class OrbitalLineScanSensorModel : public ::testing::Test {
 protected:
  csm::Isd isd;
  std::shared_ptr<csm::Model> model;
  UsgsAstroLsSensorModel *sensorModel;

  void SetUp() override {
    sensorModel = NULL;

    isd.setFilename("data/orbitalLineScan.img");
    UsgsAstroPlugin cameraPlugin;

    model =  std::shared_ptr<csm::Model>(cameraPlugin.constructModelFromISD(
      isd, UsgsAstroLsSensorModel::_SENSOR_MODEL_NAME));
    sensorModel = dynamic_cast<UsgsAstroLsSensorModel *>(model.get());
    
    ASSERT_NE(sensorModel, nullptr);
  }

  void TearDown() override {
    // The object that sensorModel points to is managed by the smart pointer 'model'.
    sensorModel = NULL;
  }
};

class FlippedOrbitalLineScanSensorModel : public ::testing::Test {
 protected:
  csm::Isd isd;
  UsgsAstroLsSensorModel *sensorModel;
  std::shared_ptr<csm::Model> model;

  void SetUp() override {
    sensorModel = NULL;

    isd.setFilename("data/flippedOrbitalLineScan.img");
    UsgsAstroPlugin cameraPlugin;

    model = std::shared_ptr<csm::Model>(cameraPlugin.constructModelFromISD(
       isd, UsgsAstroLsSensorModel::_SENSOR_MODEL_NAME));
    sensorModel = dynamic_cast<UsgsAstroLsSensorModel *>(model.get());

    ASSERT_NE(sensorModel, nullptr);
  }

  void TearDown() override {
    // The object that sensorModel points to is managed by the smart pointer 'model'
    sensorModel = NULL;
  }
};

class TwoLineScanSensorModels : public ::testing::Test {
 protected:
  csm::Isd isd;
  UsgsAstroLsSensorModel *sensorModel1;
  UsgsAstroLsSensorModel *sensorModel2;
  std::shared_ptr<csm::Model> model1, model2;

  void SetUp() override {
    sensorModel1 = nullptr;
    sensorModel2 = nullptr;

    isd.setFilename("data/orbitalLineScan.img");
    UsgsAstroPlugin cameraPlugin;

    model1 = std::shared_ptr<csm::Model>(cameraPlugin.constructModelFromISD(
       isd, UsgsAstroLsSensorModel::_SENSOR_MODEL_NAME));
    sensorModel1 = dynamic_cast<UsgsAstroLsSensorModel *>(model1.get());
    model2 = std::shared_ptr<csm::Model>(cameraPlugin.constructModelFromISD(
      isd, UsgsAstroLsSensorModel::_SENSOR_MODEL_NAME));
    sensorModel2 = dynamic_cast<UsgsAstroLsSensorModel *>(model2.get());

    ASSERT_NE(sensorModel1, nullptr);
    ASSERT_NE(sensorModel2, nullptr);
  }

  void TearDown() override {
    // Resource deallocation happens via the smart pointers
    sensorModel1 = nullptr;
    sensorModel2 = nullptr;
  }
};

//////////////////
// SAR Fixtures //
//////////////////

class SarIsdTest : public ::testing::Test {
 protected:
  csm::Isd isd;

  virtual void SetUp() { isd.setFilename("data/orbitalSar.img"); }
};

class SarSensorModel : public ::testing::Test {
 protected:
  csm::Isd isd;
  UsgsAstroSarSensorModel *sensorModel;
  std::shared_ptr<csm::Model> model;

  void SetUp() override {
    sensorModel = NULL;

    isd.setFilename("data/orbitalSar.img");
    UsgsAstroPlugin sarCameraPlugin;

    model = std::shared_ptr<csm::Model>(sarCameraPlugin.constructModelFromISD(
       isd, UsgsAstroSarSensorModel::_SENSOR_MODEL_NAME));
    sensorModel = dynamic_cast<UsgsAstroSarSensorModel *>(model.get());
    ASSERT_NE(sensorModel, nullptr);
  }

  void TearDown() override {
    // The smart pointer will take care of dellocation
    sensorModel = NULL;
  }
};

/////////////////////////
// Push Frame Fixtures //
/////////////////////////

class OrbitalPushFrameSensorModel : public ::testing::Test {
  protected:
  csm::Isd isd;
  std::shared_ptr<UsgsAstroPushFrameSensorModel> sensorModel;

  void SetUp() override {
    isd.setFilename("data/orbitalPushFrame.img");
    UsgsAstroPlugin cameraPlugin;
    
    csm::Model * model = cameraPlugin.constructModelFromISD(
       isd, UsgsAstroPushFrameSensorModel::_SENSOR_MODEL_NAME);

    // sensorModel is a smart pointer so it will deallocate the data of 'model'
    sensorModel = std::shared_ptr<UsgsAstroPushFrameSensorModel>(dynamic_cast<UsgsAstroPushFrameSensorModel *>(model));

    ASSERT_NE(sensorModel, nullptr);
  }
};

#endif
