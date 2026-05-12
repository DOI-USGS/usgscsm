#include "UsgsAstroPluginSupport.h"

#include "UsgsAstroFrameSensorModel.h"
#include "UsgsAstroLsSensorModel.h"
#include "UsgsAstroPushFrameSensorModel.h"
#ifndef __EMSCRIPTEN__
#include "UsgsAstroProjectedSensorModel.h"
#endif
#include "UsgsAstroSarSensorModel.h"

#include <fstream>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

csm::RasterGM *getUsgsCsmModelFromIsd(
    const std::string &stringIsd, const std::string &modelName,
    csm::WarningList *warnings) {
  std::cerr << "[DEBUG] getUsgsCsmModelFromIsd called with modelName: " << modelName << std::endl;

  if (modelName == UsgsAstroFrameSensorModel::_SENSOR_MODEL_NAME) {
    std::cerr << "[DEBUG] Creating FrameSensorModel" << std::endl;
    UsgsAstroFrameSensorModel *model = new UsgsAstroFrameSensorModel();
    try {
      std::cerr << "[DEBUG] Calling constructStateFromIsd" << std::endl;
      VariantMap vm = model->constructStateFromIsd(stringIsd, warnings);
      std::cerr << "[DEBUG] constructStateFromIsd succeeded, calling populateModel" << std::endl;
      model->populateModel(vm);
      std::cerr << "[DEBUG] populateModel succeeded for FrameSensorModel" << std::endl;
    } catch (std::exception &e) {
      std::cerr << "[DEBUG] Exception in FrameSensorModel: " << e.what() << std::endl;
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
    std::cerr << "[DEBUG] Creating LsSensorModel" << std::endl;
    UsgsAstroLsSensorModel *model = new UsgsAstroLsSensorModel();
    try {
      std::cerr << "[DEBUG] Calling constructStateFromIsd" << std::endl;
      VariantMap vm = model->constructStateFromIsd(stringIsd, warnings);
      std::cerr << "[DEBUG] constructStateFromIsd succeeded" << std::endl;
      std::cerr << "[DEBUG] VariantMap has " << vm.size() << " entries" << std::endl;
      std::cerr << "[DEBUG] Calling populateModel" << std::endl;
      model->populateModel(vm);
      std::cerr << "[DEBUG] populateModel succeeded for LsSensorModel" << std::endl;
    } catch (std::exception &e) {
      std::cerr << "[DEBUG] Exception in LsSensorModel: " << e.what() << std::endl;
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
    std::cerr << "[DEBUG] Creating SarSensorModel" << std::endl;
    UsgsAstroSarSensorModel *model = new UsgsAstroSarSensorModel();
    try {
      std::cerr << "[DEBUG] Calling constructStateFromIsd" << std::endl;
      VariantMap vm = model->constructStateFromIsd(stringIsd, warnings);
      std::cerr << "[DEBUG] constructStateFromIsd succeeded, calling populateModel" << std::endl;
      model->populateModel(vm);
      std::cerr << "[DEBUG] populateModel succeeded for SarSensorModel" << std::endl;
    } catch (std::exception &e) {
      std::cerr << "[DEBUG] Exception in SarSensorModel: " << e.what() << std::endl;
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
    std::cerr << "[DEBUG] Creating PushFrameSensorModel" << std::endl;
    UsgsAstroPushFrameSensorModel *model = new UsgsAstroPushFrameSensorModel();
    try {
      std::cerr << "[DEBUG] Calling constructStateFromIsd" << std::endl;
      VariantMap vm = model->constructStateFromIsd(stringIsd, warnings);
      std::cerr << "[DEBUG] constructStateFromIsd succeeded, calling populateModel" << std::endl;
      model->populateModel(vm);
      std::cerr << "[DEBUG] populateModel succeeded for PushFrameSensorModel" << std::endl;
    } catch (std::exception &e) {
      std::cerr << "[DEBUG] Exception in PushFrameSensorModel: " << e.what() << std::endl;
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
  VariantMap vm = variantMapFromJson(stateAsJson(jstr));
  return getUsgsCsmModelFromVariantMap(vm, modelName, warnings);
}

csm::RasterGM *getUsgsCsmModelFromVariantMap(const VariantMap &vm, const std::string &modelName, csm::WarningList *warnings) {
  if (modelName == UsgsAstroFrameSensorModel::_SENSOR_MODEL_NAME) {
    UsgsAstroFrameSensorModel *model = new UsgsAstroFrameSensorModel();
    model->populateModel(vm);
    return model;
  } else if (modelName == UsgsAstroLsSensorModel::_SENSOR_MODEL_NAME) {
    UsgsAstroLsSensorModel *model = new UsgsAstroLsSensorModel();
    model->populateModel(vm);
    return model;
#ifndef __EMSCRIPTEN__
  } else if (modelName == UsgsAstroProjectedSensorModel::_SENSOR_MODEL_NAME) {
    UsgsAstroProjectedSensorModel *model = new UsgsAstroProjectedSensorModel();
    model->populateModel(vm);
    return model;
#endif
  } else if (modelName == UsgsAstroPushFrameSensorModel::_SENSOR_MODEL_NAME) {
    UsgsAstroPushFrameSensorModel *model = new UsgsAstroPushFrameSensorModel();
    model->populateModel(vm);
    return model;
  } else if (modelName == UsgsAstroSarSensorModel::_SENSOR_MODEL_NAME) {
    UsgsAstroSarSensorModel *model = new UsgsAstroSarSensorModel();
    model->populateModel(vm);
    return model;
  } else {
    csm::Error::ErrorType aErrorType = csm::Error::ISD_NOT_SUPPORTED;
    std::string aMessage = "Model " + modelName + " not supported";
    std::string aFunction = "UsgsAstroPluginSupport::getUsgsCsmModelFromVariantMap()";
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
#ifndef __EMSCRIPTEN__
  if (auto *proj = dynamic_cast<UsgsAstroProjectedSensorModel*>(model))
    return proj->getModelJson();
#endif
  csm::Error::ErrorType aErrorType = csm::Error::SENSOR_MODEL_NOT_SUPPORTED;
  std::string aMessage = "Unsupported model type in getUsgsCsmModelJson()";
  std::string aFunction = "UsgsAstroPluginSupport::getUsgsCsmModelJson()";
  throw csm::Error(aErrorType, aMessage, aFunction);
}


VariantMap getUsgsCsmModelMap(csm::RasterGM *model) {
  if (auto *ls = dynamic_cast<UsgsAstroLsSensorModel*>(model))
    return ls->getModelMap();
  if (auto *fr = dynamic_cast<UsgsAstroFrameSensorModel*>(model))
    return fr->getModelMap();
  if (auto *pf = dynamic_cast<UsgsAstroPushFrameSensorModel*>(model))
    return pf->getModelMap();
  if (auto *sar = dynamic_cast<UsgsAstroSarSensorModel*>(model))
    return sar->getModelMap();
#ifndef __EMSCRIPTEN__
  if (auto *proj = dynamic_cast<UsgsAstroProjectedSensorModel*>(model))
    return proj->getModelMap();
#endif
  csm::Error::ErrorType aErrorType = csm::Error::SENSOR_MODEL_NOT_SUPPORTED;
  std::string aMessage = "Unsupported model type in getUsgsCsmModelMap()";
  std::string aFunction = "UsgsAstroPluginSupport::getUsgsCsmModelMap()";
  throw csm::Error(aErrorType, aMessage, aFunction);
}

// Quick check if the given string is a USGS CSM JSON ISD (not a model state,
// .sup, etc.). If yes, extract the model name. No JSON parsing is done, just
// raw string searches.
bool isUsgsCsmIsd(const std::string &str, std::string &modelName) {
  modelName.clear();

  // Model state strings have a text prefix before '{'. ISDs start with '{'.
  auto pos = str.find_first_not_of(" \t\n\r");
  if (pos == std::string::npos || str[pos] != '{')
    return false;

  // Check for ISD-only keys (never present in model state)
  if (str.find("\"body_rotation\"") == std::string::npos)
    return false;
  if (str.find("\"instrument_position\"") == std::string::npos)
    return false;

  // Extract the name_model value. Search for the model name prefix
  // which immediately follows the "name_model" key's value quote.
  std::string prefix = "\"USGS_ASTRO_";
  pos = str.find(prefix);
  if (pos == std::string::npos)
    return false;
  auto end = str.find("\"", pos + 1);
  if (end == std::string::npos)
    return false;

  modelName = str.substr(pos + 1, end - pos - 1);
  return !modelName.empty();
}

// Quick check if the given string is a USGS CSM model state (JSON state or GXP
// .sup format). If yes, extract the model name. No JSON parsing is done, just
// raw string searches. Following the logic in stateAsJson() (Utilities.cpp),
// skip to the first '{' to bypass any .sup preamble, then search within the
// JSON portion only.
bool isUsgsCsmState(const std::string &str, std::string &modelName) {
  modelName.clear();

  // Find the start of the JSON blob (skips .sup preamble if present)
  auto brace = str.find_first_of("{");
  if (brace == std::string::npos)
    return false;

  // Model state uses m_modelName; ISDs use name_model
  if (str.find("\"m_modelName\"", brace) == std::string::npos)
    return false;

  // Extract the model name value
  std::string prefix = "\"USGS_ASTRO_";
  auto pos = str.find(prefix, brace);
  if (pos == std::string::npos)
    return false;
  auto end = str.find("\"", pos + 1);
  if (end == std::string::npos)
    return false;

  modelName = str.substr(pos + 1, end - pos - 1);
  return !modelName.empty();

}
