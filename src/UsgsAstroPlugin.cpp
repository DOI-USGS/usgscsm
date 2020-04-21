#include "UsgsAstroPlugin.h"

#include "UsgsAstroFrameSensorModel.h"
#include "UsgsAstroLsSensorModel.h"
#include "UsgsAstroSarSensorModel.h"

#include <algorithm>
#include <cstdlib>
#include <string>
#include <fstream>

#include <math.h>
#include <csm.h>
#include <Error.h>
#include <Plugin.h>
#include <Warning.h>
#include <Version.h>

#include <json/json.hpp>
using json = nlohmann::json;

#ifdef _WIN32
# define DIR_DELIMITER_STR "\\"
#else
# define DIR_DELIMITER_STR  "/"
#endif


// Declaration of static variables
const std::string UsgsAstroPlugin::_PLUGIN_NAME = "UsgsAstroPluginCSM";
const std::string UsgsAstroPlugin::_MANUFACTURER_NAME = "UsgsAstrogeology";
const std::string UsgsAstroPlugin::_RELEASE_DATE = "20190222";
const int         UsgsAstroPlugin::_N_SENSOR_MODELS = 3;

// Static Instance of itself
const UsgsAstroPlugin UsgsAstroPlugin::m_registeredPlugin;

UsgsAstroPlugin::UsgsAstroPlugin() {
}


UsgsAstroPlugin::~UsgsAstroPlugin() {
}


std::string UsgsAstroPlugin::getPluginName() const {
  return _PLUGIN_NAME;
}


std::string UsgsAstroPlugin::getManufacturer() const {
  return _MANUFACTURER_NAME;
}


std::string UsgsAstroPlugin::getReleaseDate() const {
  return _RELEASE_DATE;
}


csm::Version UsgsAstroPlugin::getCsmVersion() const {
  return CURRENT_CSM_VERSION;
}


size_t UsgsAstroPlugin::getNumModels() const {
  return _N_SENSOR_MODELS;
}


std::string UsgsAstroPlugin::getModelName(size_t modelIndex) const {
  std::vector<std::string> supportedModelNames = {
    UsgsAstroFrameSensorModel::_SENSOR_MODEL_NAME,
    UsgsAstroLsSensorModel::_SENSOR_MODEL_NAME,
    UsgsAstroSarSensorModel::_SENSOR_MODEL_NAME
  };
  return supportedModelNames[modelIndex];
}


std::string UsgsAstroPlugin::getModelFamily(size_t modelIndex) const {
  return CSM_RASTER_FAMILY;
}


csm::Version UsgsAstroPlugin::getModelVersion(const std::string &modelName) const {
  return csm::Version(1, 0, 0);
}


bool UsgsAstroPlugin::canModelBeConstructedFromState(const std::string &modelName,
                                                const std::string &modelState,
                                                csm::WarningList *warnings) const {
  try {
    csm::Model* model = constructModelFromState(modelState, warnings);
    return (bool)model;
  }
  catch(std::exception& e) {
    if(warnings) {
      std::string msg = "Could not create model [";
      msg += modelName;
      msg += "] with error [";
      msg += e.what();
      msg += "]";
      warnings->push_back(
        csm::Warning(
          csm::Warning::UNKNOWN_WARNING,
          msg,
          "UsgsAstroFrameSensorModel::canModelBeConstructedFromState()"));
    }
    return false;
  }
  catch(...) {
    if(warnings) {
      std::string msg = "Could not create model [";
      msg += modelName;
      msg += "] with an unknown error.";
      warnings->push_back(
        csm::Warning(
          csm::Warning::UNKNOWN_WARNING,
          msg,
          "UsgsAstroFrameSensorModel::canModelBeConstructedFromState()"));
    }
  }
  return false;
}


bool UsgsAstroPlugin::canModelBeConstructedFromISD(const csm::Isd &imageSupportData,
                                              const std::string &modelName,
                                              csm::WarningList *warnings) const {
  try {
    csm::Model* model = constructModelFromISD(imageSupportData, modelName, warnings);
    return (bool)model;
  }
  catch(std::exception& e) {
    if(warnings) {
      std::string msg = "Could not create model [";
      msg += modelName;
      msg += "] with error [";
      msg += e.what();
      msg += "]";
      warnings->push_back(
        csm::Warning(
          csm::Warning::UNKNOWN_WARNING,
          msg,
          "UsgsAstroFrameSensorModel::canModelBeConstructedFromISD()"));
    }
  }
  catch(...) {
    if(warnings) {
      std::string msg = "Could not create model [";
      msg += modelName;
      msg += "] with an unknown error.";
      warnings->push_back(
        csm::Warning(
          csm::Warning::UNKNOWN_WARNING,
          msg,
          "UsgsAstroFrameSensorModel::canModelBeConstructedFromISD()"));
    }
  }
  return false;
}


// This function takes a csm::Isd which only has the image filename set. It uses this filename to
// find a metadata json file located alongside the image file and returns a json
// encoded string.
std::string UsgsAstroPlugin::loadImageSupportData(const csm::Isd &imageSupportDataOriginal) const {

  // Get image location from the input csm::Isd:
  std::string imageFilename = imageSupportDataOriginal.filename();
  size_t lastIndex = imageFilename.find_last_of(".");
  std::string baseName = imageFilename.substr(0, lastIndex);
  lastIndex = baseName.find_last_of(DIR_DELIMITER_STR);
  std::string filename = baseName.substr(lastIndex + 1);
  std::string isdFilename = baseName.append(".json");

  try {
    std::ifstream isd_sidecar(isdFilename);
    json jsonisd;
    isd_sidecar >> jsonisd;
    jsonisd["image_identifier"] = filename;
    return jsonisd.dump();

  }
  catch (std::exception& e) {
    std::string errorMessage = "Could not read metadata file associated with image [";
    errorMessage += isdFilename;
    errorMessage += "] with error [";
    errorMessage += e.what();
    errorMessage += "]";
    throw csm::Error(csm::Error::FILE_READ, errorMessage,
                     "UsgsAstroPlugin::loadImageSupportData");
  }
}


std::string UsgsAstroPlugin::getModelNameFromModelState(const std::string &modelState,
                                                   csm::WarningList *warnings) const {
  auto state = json::parse(modelState);

  std::string name = state.value<std::string>("name_model", "");

  if (name == "") {
      csm::Error::ErrorType aErrorType = csm::Error::INVALID_SENSOR_MODEL_STATE;
      std::string aMessage = "No 'name_model' key in the model state object.";
      std::string aFunction = "UsgsAstroPlugin::getModelNameFromModelState";
      csm::Error csmErr(aErrorType, aMessage, aFunction);
      throw(csmErr);
  }

  return name;
}


bool UsgsAstroPlugin::canISDBeConvertedToModelState(const csm::Isd &imageSupportData,
                                               const std::string &modelName,
                                               csm::WarningList *warnings) const {
  try {
       convertISDToModelState(imageSupportData, modelName, warnings);
  }
  catch(std::exception& e) {
    if(warnings) {
      std::string msg = "Could not create model [";
      msg += modelName;
      msg += "] state with error [";
      msg += e.what();
      msg += "]";
      warnings->push_back(
        csm::Warning(
          csm::Warning::UNKNOWN_WARNING,
          msg,
          "UsgsAstroFrameSensorModel::canISDBeConvertedToModelState()"));
    }
    return false;
  }
  return true;
}


std::string UsgsAstroPlugin::getStateFromISD(csm::Isd imageSupportData) const {
    std::string stringIsd = loadImageSupportData(imageSupportData);
    json jsonIsd = json::parse(stringIsd);
    return convertISDToModelState(imageSupportData, jsonIsd.at("name_model"));
}


std::string UsgsAstroPlugin::convertISDToModelState(const csm::Isd &imageSupportData,
                                               const std::string &modelName,
                                               csm::WarningList *warnings) const {

  csm::Model* sensor_model = constructModelFromISD(imageSupportData, modelName, warnings);
  return sensor_model->getModelState();
}


csm::Model *UsgsAstroPlugin::constructModelFromISD(const csm::Isd &imageSupportDataOriginal,
                                              const std::string &modelName,
                                              csm::WarningList *warnings) const {
    std::string stringIsd = loadImageSupportData(imageSupportDataOriginal);

    if (modelName == UsgsAstroFrameSensorModel::_SENSOR_MODEL_NAME) {
      UsgsAstroFrameSensorModel *model =  new UsgsAstroFrameSensorModel();
      try {
        model->replaceModelState(model->constructStateFromIsd(stringIsd, warnings));
        if (model->getLogger()) {
          model->getLogger()->info("Constructed model: {}", modelName);
        }
      }
      catch (std::exception& e) {
        csm::Error::ErrorType aErrorType = csm::Error::SENSOR_MODEL_NOT_CONSTRUCTIBLE;
        std::string aMessage = "Could not construct model [";
        aMessage += modelName;
        aMessage += "] with error [";
        aMessage += e.what();
        aMessage += "]";
        std::string aFunction = "UsgsAstroPlugin::constructModelFromISD()";
        throw csm::Error(aErrorType, aMessage, aFunction);
      }
      return model;
    }
    else if (modelName == UsgsAstroLsSensorModel::_SENSOR_MODEL_NAME) {
      UsgsAstroLsSensorModel *model =  new UsgsAstroLsSensorModel();
      try {
        model->replaceModelState(model->constructStateFromIsd(stringIsd, warnings));
      }
      catch (std::exception& e) {
        csm::Error::ErrorType aErrorType = csm::Error::SENSOR_MODEL_NOT_CONSTRUCTIBLE;
        std::string aMessage = "Could not construct model [";
        aMessage += modelName;
        aMessage += "] with error [";
        aMessage += e.what();
        aMessage += "]";
        std::string aFunction = "UsgsAstroPlugin::constructModelFromISD()";
        throw csm::Error(aErrorType, aMessage, aFunction);
      }
      return model;
    }
    else if (modelName == UsgsAstroSarSensorModel::_SENSOR_MODEL_NAME) {
      UsgsAstroSarSensorModel *model =  new UsgsAstroSarSensorModel();
      try {
        model->replaceModelState(model->constructStateFromIsd(stringIsd, warnings));
      }
      catch (std::exception& e) {
        csm::Error::ErrorType aErrorType = csm::Error::SENSOR_MODEL_NOT_CONSTRUCTIBLE;
        std::string aMessage = "Could not construct model [";
        aMessage += modelName;
        aMessage += "] with error [";
        aMessage += e.what();
        aMessage += "]";
        std::string aFunction = "UsgsAstroPlugin::constructModelFromISD()";
        throw csm::Error(aErrorType, aMessage, aFunction);
      }
      return model;
    }
    else {
      csm::Error::ErrorType aErrorType = csm::Error::SENSOR_MODEL_NOT_SUPPORTED;
      std::string aMessage = "Model [" + modelName + "] not supported: ";
      std::string aFunction = "UsgsAstroPlugin::constructModelFromISD()";
      throw csm::Error(aErrorType, aMessage, aFunction);
    }
}


csm::Model *UsgsAstroPlugin::constructModelFromState(const std::string& modelState,
                                                csm::WarningList *warnings) const {

    json state = json::parse(modelState);
    std::string modelName = state["m_modelName"];

    if (modelName == UsgsAstroFrameSensorModel::_SENSOR_MODEL_NAME) {
         UsgsAstroFrameSensorModel* model = new UsgsAstroFrameSensorModel();
         model->replaceModelState(modelState);
         return model;
    }
    else if (modelName == UsgsAstroLsSensorModel::_SENSOR_MODEL_NAME) {
        UsgsAstroLsSensorModel* model = new UsgsAstroLsSensorModel();
        model->replaceModelState(modelState);
        return model;
    }
    else if (modelName == UsgsAstroSarSensorModel::_SENSOR_MODEL_NAME) {
        UsgsAstroSarSensorModel* model = new UsgsAstroSarSensorModel();
        model->replaceModelState(modelState);
        return model;
    }
    else {
      csm::Error::ErrorType aErrorType = csm::Error::ISD_NOT_SUPPORTED;
      std::string aMessage = "Model" + modelName + " not supported: ";
      std::string aFunction = "UsgsAstroPlugin::constructModelFromState()";
      throw csm::Error(aErrorType, aMessage, aFunction);
    }
}
