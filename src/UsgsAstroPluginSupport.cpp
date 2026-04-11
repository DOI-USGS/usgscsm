#include "UsgsAstroPluginSupport.h"

#include "UsgsAstroFrameSensorModel.h"
#include "UsgsAstroLsSensorModel.h"
#include "UsgsAstroPushFrameSensorModel.h"
#include "UsgsAstroProjectedSensorModel.h"
#include "UsgsAstroSarSensorModel.h"

#include <fstream>

#include "spdlog/spdlog.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

csm::RasterGM *getUsgsCsmModelFromIsd(
    const std::string &stringIsd, const std::string &modelName,
    csm::WarningList *warnings) {
  if (modelName == UsgsAstroFrameSensorModel::_SENSOR_MODEL_NAME) {
    UsgsAstroFrameSensorModel *model = new UsgsAstroFrameSensorModel();
    try {
      model->replaceModelState(
          model->constructStateFromIsd(stringIsd, warnings));
    } catch (std::exception &e) {
      delete model;
      csm::Error::ErrorType aErrorType =
          csm::Error::SENSOR_MODEL_NOT_CONSTRUCTIBLE;
      std::string aMessage = "Could not construct model [";
      aMessage += modelName;
      aMessage += "] with error [";
      aMessage += e.what();
      aMessage += "]";
      std::string aFunction = "UsgsAstroPluginSupport::getUsgsCsmModelFromIsd()";
      throw csm::Error(aErrorType, aMessage, aFunction);
    }
    return model;
  } else if (modelName == UsgsAstroLsSensorModel::_SENSOR_MODEL_NAME) {
    UsgsAstroLsSensorModel *model = new UsgsAstroLsSensorModel();
    try {
      model->replaceModelState(
          model->constructStateFromIsd(stringIsd, warnings));
    } catch (std::exception &e) {
      delete model;
      csm::Error::ErrorType aErrorType =
          csm::Error::SENSOR_MODEL_NOT_CONSTRUCTIBLE;
      std::string aMessage = "Could not construct model [";
      aMessage += modelName;
      aMessage += "] with error [";
      aMessage += e.what();
      aMessage += "]";
      std::string aFunction = "UsgsAstroPluginSupport::getUsgsCsmModelFromIsd()";
      throw csm::Error(aErrorType, aMessage, aFunction);
    }
    return model;
  } else if (modelName == UsgsAstroSarSensorModel::_SENSOR_MODEL_NAME) {
    UsgsAstroSarSensorModel *model = new UsgsAstroSarSensorModel();
    try {
      model->replaceModelState(
          model->constructStateFromIsd(stringIsd, warnings));
    } catch (std::exception &e) {
      delete model;
      csm::Error::ErrorType aErrorType =
          csm::Error::SENSOR_MODEL_NOT_CONSTRUCTIBLE;
      std::string aMessage = "Could not construct model [";
      aMessage += modelName;
      aMessage += "] with error [";
      aMessage += e.what();
      aMessage += "]";
      std::string aFunction = "UsgsAstroPluginSupport::getUsgsCsmModelFromIsd()";
      throw csm::Error(aErrorType, aMessage, aFunction);
    }
    return model;
  } else if (modelName == UsgsAstroPushFrameSensorModel::_SENSOR_MODEL_NAME) {
    UsgsAstroPushFrameSensorModel *model = new UsgsAstroPushFrameSensorModel();
    try {
      model->replaceModelState(
          model->constructStateFromIsd(stringIsd, warnings));
    } catch (std::exception &e) {
      delete model;
      csm::Error::ErrorType aErrorType =
          csm::Error::SENSOR_MODEL_NOT_CONSTRUCTIBLE;
      std::string aMessage = "Could not construct model [";
      aMessage += modelName;
      aMessage += "] with error [";
      aMessage += e.what();
      aMessage += "]";
      std::string aFunction = "UsgsAstroPluginSupport::getUsgsCsmModelFromIsd()";
      throw csm::Error(aErrorType, aMessage, aFunction);
    }
    return model;
  } else {
    csm::Error::ErrorType aErrorType = csm::Error::SENSOR_MODEL_NOT_SUPPORTED;
    std::string aMessage = "Model [" + modelName + "] not supported.";
    std::string aFunction = "UsgsAstroPluginSupport::getUsgsCsmModelFromIsd()";
    throw csm::Error(aErrorType, aMessage, aFunction);
  }
  
}

csm::RasterGM *getUsgsCsmModelFromJsonState(const std::string &jstr, const std::string &modelName, csm::WarningList *warnings) {
  if (modelName == UsgsAstroFrameSensorModel::_SENSOR_MODEL_NAME) {
    UsgsAstroFrameSensorModel *model = new UsgsAstroFrameSensorModel();
    model->populateModel(jstr);
    return model;
  } else if (modelName == UsgsAstroLsSensorModel::_SENSOR_MODEL_NAME) {
    UsgsAstroLsSensorModel *model = new UsgsAstroLsSensorModel();
    model->populateModel(jstr);
    return model;
  } else if (modelName == UsgsAstroProjectedSensorModel::_SENSOR_MODEL_NAME) {
    UsgsAstroProjectedSensorModel *model = new UsgsAstroProjectedSensorModel();
    model->populateModel(jstr);
    return model;
  } else if (modelName == UsgsAstroPushFrameSensorModel::_SENSOR_MODEL_NAME) {
    UsgsAstroPushFrameSensorModel *model = new UsgsAstroPushFrameSensorModel();
    model->populateModel(jstr);
    return model;
  } else if (modelName == UsgsAstroSarSensorModel::_SENSOR_MODEL_NAME) {
    UsgsAstroSarSensorModel *model = new UsgsAstroSarSensorModel();
    model->populateModel(jstr);
    return model;
  } else {
    csm::Error::ErrorType aErrorType = csm::Error::ISD_NOT_SUPPORTED;
    std::string aMessage = "Model " + modelName + " not supported";
    std::string aFunction = "UsgsAstroPluginSupport::getUsgsCsmModelFromJsonState()";
    throw csm::Error(aErrorType, aMessage, aFunction);
  }
}


std::string getUsgsCsmModelJson(csm::RasterGM *model) {
  if (auto *ls = dynamic_cast<UsgsAstroLsSensorModel*>(model))
    return ls->getModelJson();
  if (auto *fr = dynamic_cast<UsgsAstroFrameSensorModel*>(model))
    return fr->getModelJson();
  if (auto *pf = dynamic_cast<UsgsAstroPushFrameSensorModel*>(model))
    return pf->getModelJson();
  if (auto *sar = dynamic_cast<UsgsAstroSarSensorModel*>(model))
    return sar->getModelJson();
  if (auto *proj = dynamic_cast<UsgsAstroProjectedSensorModel*>(model))
    return proj->getModelJson();
  // Throw for unsupported model, as in the other dispatch functions
  csm::Error::ErrorType aErrorType = csm::Error::SENSOR_MODEL_NOT_SUPPORTED;
  std::string aMessage = "Unsupported model type in getUsgsCsmModelJson()";
  std::string aFunction = "UsgsAstroPluginSupport::getUsgsCsmModelJson()";
  throw csm::Error(aErrorType, aMessage, aFunction);
}
