#ifndef Fixtures_h
#define Fixtures_h

#include "UsgsAstroFramePlugin.h"
#include "UsgsAstroFrameSensorModel.h"

#include <json.hpp>

#include <map>
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
      csm::Isd isd;
      UsgsAstroFrameSensorModel *sensorModel;

      void SetUp() override {
         sensorModel = NULL;
         isd.setFilename("data/simpleFramerISD.img");
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

class SimpleFrameIsdTest : public ::testing::Test {
   protected:
      csm::Isd isd;

   virtual void SetUp() {
      isd.setFilename("data/simpleFramerISD.img");
   }
};

class FramerParameterizedTest : public ::testing::TestWithParam<csm::ImageCoord> {

protected:
  csm::Isd isd;
  
  void printIsd(csm::Isd &localIsd) {
    std::multimap<std::string,std::string> isdmap= localIsd.parameters();
    for (auto it = isdmap.begin(); it != isdmap.end();++it){
      
      std::cout << it->first << " : " << it->second << std::endl;
    }
  }
  UsgsAstroFrameSensorModel* createModel(csm::Isd &modifiedIsd) {
    
    UsgsAstroFramePlugin frameCameraPlugin;
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

class FrameIsdTest : public ::testing::Test {
  protected:
    csm::Isd isd;
    void printIsd(csm::Isd &localIsd) {
      std::multimap<std::string,std::string> isdmap= localIsd.parameters();
      for (auto it = isdmap.begin(); it != isdmap.end();++it){
        std::cout << it->first << " : " << it->second << std::endl;
      }
    }
    UsgsAstroFrameSensorModel* createModel(csm::Isd &modifiedIsd) {
      UsgsAstroFramePlugin frameCameraPlugin;
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
   }
};

#endif
