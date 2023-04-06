#include "UsgsAstroPluginSupport.h"

#include "UsgsAstroFrameSensorModel.h"
#include "UsgsAstroLsSensorModel.h"
#include "UsgsAstroPushFrameSensorModel.h"
#include "UsgsAstroSarSensorModel.h"

#include <fstream>

#include "spdlog/spdlog.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

csm::RasterGM *getUsgsCsmModel(
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
      std::string aFunction = "UsgsAstroPluginSupport::getUsgsCsmModel()";
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
      std::string aFunction = "UsgsAstroPluginSupport::getUsgsCsmModel()";
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
      std::string aFunction = "UsgsAstroPluginSupport::getUsgsCsmModel()";
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
      std::string aFunction = "UsgsAstroPluginSupport::getUsgsCsmModel()";
      throw csm::Error(aErrorType, aMessage, aFunction);
    }
    return model;
  } else {
    csm::Error::ErrorType aErrorType = csm::Error::SENSOR_MODEL_NOT_SUPPORTED;
    std::string aMessage = "Model [" + modelName + "] not supported: ";
    std::string aFunction = "UsgsAstroPluginSupport::getUsgsCsmModel()";
    throw csm::Error(aErrorType, aMessage, aFunction);
  }
  
}