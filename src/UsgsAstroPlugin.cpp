#include "UsgsAstroPlugin.h"

#include "UsgsAstroFrameSensorModel.h"
#include "UsgsAstroLsSensorModel.h"
#include "UsgsAstroSarSensorModel.h"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <string>

#include <Error.h>
#include <Plugin.h>
#include <Version.h>
#include <Warning.h>
#include <csm.h>
#include <math.h>

#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#ifdef _WIN32
#define DIR_DELIMITER_STR "\\"
#else
#define DIR_DELIMITER_STR "/"
#endif

#define MESSAGE_LOG(...)         \
  if (m_logger) {                \
    m_logger->info(__VA_ARGS__); \
  }

// Declaration of static variables
const std::string UsgsAstroPlugin::_PLUGIN_NAME = "UsgsAstroPluginCSM";
const std::string UsgsAstroPlugin::_MANUFACTURER_NAME = "UsgsAstrogeology";
const std::string UsgsAstroPlugin::_RELEASE_DATE = "20190222";
const int UsgsAstroPlugin::_N_SENSOR_MODELS = 3;

// Static Instance of itself
const UsgsAstroPlugin UsgsAstroPlugin::m_registeredPlugin;

UsgsAstroPlugin::UsgsAstroPlugin() {
  // Build and register the USGSCSM logger on plugin creation
  char *logFilePtr = getenv("USGSCSM_LOG_FILE");

  if (logFilePtr != NULL) {
    std::string logFile(logFilePtr);

    if (logFile != "") {
      std::shared_ptr<spdlog::logger> m_logger = spdlog::get("usgscsm_logger");

      if (logFile == "stdout") {
        std::shared_ptr<spdlog::logger> m_logger =
            spdlog::stdout_color_mt("usgscsm_logger");
      }
      else if (logFile == "stderr") {
        std::shared_ptr<spdlog::logger> m_logger =
            spdlog::stderr_color_mt("usgscsm_logger");
      }
      else if (!m_logger) {
        std::shared_ptr<spdlog::logger> m_logger =
            spdlog::basic_logger_mt("usgscsm_logger", logFile);
      }
    }
  }
}

UsgsAstroPlugin::~UsgsAstroPlugin() {}

std::string UsgsAstroPlugin::getPluginName() const {
  MESSAGE_LOG("Get Plugin Name: {}", _PLUGIN_NAME);
  return _PLUGIN_NAME;
}

std::string UsgsAstroPlugin::getManufacturer() const {
  MESSAGE_LOG("Get Manufacturer Name: {}", _MANUFACTURER_NAME);
  return _MANUFACTURER_NAME;
}

std::string UsgsAstroPlugin::getReleaseDate() const {
  MESSAGE_LOG("Get Release Date: {}", _RELEASE_DATE);
  return _RELEASE_DATE;
}

csm::Version UsgsAstroPlugin::getCsmVersion() const {
  MESSAGE_LOG("Get Current CSM Version");
  return CURRENT_CSM_VERSION;
}

size_t UsgsAstroPlugin::getNumModels() const {
  MESSAGE_LOG("Get Number of Sensor Models: {}", _N_SENSOR_MODELS);
  return _N_SENSOR_MODELS;
}

std::string UsgsAstroPlugin::getModelName(size_t modelIndex) const {
  std::vector<std::string> supportedModelNames = {
      UsgsAstroFrameSensorModel::_SENSOR_MODEL_NAME,
      UsgsAstroLsSensorModel::_SENSOR_MODEL_NAME,
      UsgsAstroSarSensorModel::_SENSOR_MODEL_NAME};
  MESSAGE_LOG("Get Model Name: {}. Used index: {}",
              supportedModelNames[modelIndex], modelIndex);
  return supportedModelNames[modelIndex];
}

std::string UsgsAstroPlugin::getModelFamily(size_t modelIndex) const {
  MESSAGE_LOG("Get Model Familey: {}", CSM_RASTER_FAMILY);
  return CSM_RASTER_FAMILY;
}

csm::Version UsgsAstroPlugin::getModelVersion(
    const std::string &modelName) const {
  MESSAGE_LOG("Get Model Version");
  return csm::Version(1, 0, 0);
}

bool UsgsAstroPlugin::canModelBeConstructedFromState(
    const std::string &modelName, const std::string &modelState,
    csm::WarningList *warnings) const {
  try {
    csm::Model *model = constructModelFromState(modelState, warnings);
    return static_cast<bool>(model);
  } catch (std::exception &e) {
    std::string msg = "Could not create model [";
    msg += modelName;
    msg += "] with error [";
    msg += e.what();
    msg += "]";
    MESSAGE_LOG(msg);
    if (warnings) {
      warnings->push_back(csm::Warning(
          csm::Warning::UNKNOWN_WARNING, msg,
          "UsgsAstroFrameSensorModel::canModelBeConstructedFromState()"));
    }
    return false;
  } catch (...) {
    std::string msg = "Could not create model [";
    msg += modelName;
    msg += "] with an unknown error.";
    MESSAGE_LOG(msg);
    if (warnings) {
      warnings->push_back(csm::Warning(
          csm::Warning::UNKNOWN_WARNING, msg,
          "UsgsAstroFrameSensorModel::canModelBeConstructedFromState()"));
    }
  }
  return false;
}

bool UsgsAstroPlugin::canModelBeConstructedFromISD(
    const csm::Isd &imageSupportData, const std::string &modelName,
    csm::WarningList *warnings) const {
  try {
    csm::Model *model =
        constructModelFromISD(imageSupportData, modelName, warnings);
    return static_cast<bool>(model);
  } catch (std::exception &e) {
    if (warnings) {
      std::string msg = "Could not create model [";
      msg += modelName;
      msg += "] with error [";
      msg += e.what();
      msg += "]";
      MESSAGE_LOG(msg);
      warnings->push_back(csm::Warning(
          csm::Warning::UNKNOWN_WARNING, msg,
          "UsgsAstroFrameSensorModel::canModelBeConstructedFromISD()"));
    }
  } catch (...) {
    if (warnings) {
      std::string msg = "Could not create model [";
      msg += modelName;
      msg += "] with an unknown error.";
      MESSAGE_LOG(msg);
      warnings->push_back(csm::Warning(
          csm::Warning::UNKNOWN_WARNING, msg,
          "UsgsAstroFrameSensorModel::canModelBeConstructedFromISD()"));
    }
  }
  return false;
}

// This function takes a csm::Isd which only has the image filename set. It uses
// this filename to find a metadata json file located alongside the image file
// and returns a json encoded string.
std::string UsgsAstroPlugin::loadImageSupportData(
    const csm::Isd &imageSupportDataOriginal) const {
  // Get image location from the input csm::Isd:
  std::string imageFilename = imageSupportDataOriginal.filename();
  size_t lastIndex = imageFilename.find_last_of(".");
  std::string baseName = imageFilename.substr(0, lastIndex);
  lastIndex = baseName.find_last_of(DIR_DELIMITER_STR);
  std::string filename = baseName.substr(lastIndex + 1);
  std::string isdFilename = baseName.append(".json");
  MESSAGE_LOG("Load Image Support Data using: {}, {}, {}, {}, {}",
              imageFilename, lastIndex, baseName, filename, isdFilename);
  try {
    std::ifstream isd_sidecar(isdFilename);
    json jsonisd;
    isd_sidecar >> jsonisd;
    jsonisd["image_identifier"] = filename;
    return jsonisd.dump();
  } catch (std::exception &e) {
    std::string errorMessage =
        "Could not read metadata file associated with image [";
    errorMessage += isdFilename;
    errorMessage += "] with error [";
    errorMessage += e.what();
    errorMessage += "]";
    MESSAGE_LOG(errorMessage);
    throw csm::Error(csm::Error::FILE_READ, errorMessage,
                     "UsgsAstroPlugin::loadImageSupportData");
  }
}

std::string UsgsAstroPlugin::getModelNameFromModelState(
    const std::string &modelState, csm::WarningList *warnings) const {
  auto state = json::parse(modelState);

  std::string name = state.value<std::string>("name_model", "");
  MESSAGE_LOG("Get model name from model state. State: {}, Name: {}",
              modelState, name);
  if (name == "") {
    csm::Error::ErrorType aErrorType = csm::Error::INVALID_SENSOR_MODEL_STATE;
    std::string aMessage = "No 'name_model' key in the model state object.";
    std::string aFunction = "UsgsAstroPlugin::getModelNameFromModelState";
    MESSAGE_LOG(aMessage);
    csm::Error csmErr(aErrorType, aMessage, aFunction);
    throw(csmErr);
  }

  return name;
}

bool UsgsAstroPlugin::canISDBeConvertedToModelState(
    const csm::Isd &imageSupportData, const std::string &modelName,
    csm::WarningList *warnings) const {
  MESSAGE_LOG("Running canISDBeConvertedToModelState");
  try {
    convertISDToModelState(imageSupportData, modelName, warnings);
  } catch (std::exception &e) {
    if (warnings) {
      std::string msg = "Could not create model [";
      msg += modelName;
      msg += "] state with error [";
      msg += e.what();
      msg += "]";
      MESSAGE_LOG(msg);
      warnings->push_back(csm::Warning(
          csm::Warning::UNKNOWN_WARNING, msg,
          "UsgsAstroFrameSensorModel::canISDBeConvertedToModelState()"));
    }
    return false;
  }
  return true;
}

std::string UsgsAstroPlugin::getStateFromISD(csm::Isd imageSupportData) const {
  MESSAGE_LOG("Running getStateFromISD");
  std::string stringIsd = loadImageSupportData(imageSupportData);
  MESSAGE_LOG("ISD string: {}", stringIsd);
  json jsonIsd = json::parse(stringIsd);
  return convertISDToModelState(imageSupportData, jsonIsd.at("name_model"));
}

std::string UsgsAstroPlugin::convertISDToModelState(
    const csm::Isd &imageSupportData, const std::string &modelName,
    csm::WarningList *warnings) const {
  MESSAGE_LOG("Running convertISDToModelState");
  csm::Model *sensor_model =
      constructModelFromISD(imageSupportData, modelName, warnings);
  return sensor_model->getModelState();
}

csm::Model *UsgsAstroPlugin::constructModelFromISD(
    const csm::Isd &imageSupportDataOriginal, const std::string &modelName,
    csm::WarningList *warnings) const {
  MESSAGE_LOG("Running constructModelFromISD");
  std::string stringIsd = loadImageSupportData(imageSupportDataOriginal);

  MESSAGE_LOG("ISD String: {}", stringIsd);
  if (modelName == UsgsAstroFrameSensorModel::_SENSOR_MODEL_NAME) {
    UsgsAstroFrameSensorModel *model = new UsgsAstroFrameSensorModel();
    try {
      MESSAGE_LOG("Trying to construct a UsgsAstroFrameSensorModel");
      model->replaceModelState(
          model->constructStateFromIsd(stringIsd, warnings));
      MESSAGE_LOG("Constructed model: {}", modelName);
    } catch (std::exception &e) {
      csm::Error::ErrorType aErrorType =
          csm::Error::SENSOR_MODEL_NOT_CONSTRUCTIBLE;
      std::string aMessage = "Could not construct model [";
      aMessage += modelName;
      aMessage += "] with error [";
      aMessage += e.what();
      aMessage += "]";
      MESSAGE_LOG(aMessage);
      std::string aFunction = "UsgsAstroPlugin::constructModelFromISD()";
      throw csm::Error(aErrorType, aMessage, aFunction);
    }
    return model;
  } else if (modelName == UsgsAstroLsSensorModel::_SENSOR_MODEL_NAME) {
    UsgsAstroLsSensorModel *model = new UsgsAstroLsSensorModel();
    try {
      MESSAGE_LOG("Trying to construct a UsgsAstroLsSensorModel");
      model->replaceModelState(
          model->constructStateFromIsd(stringIsd, warnings));
    } catch (std::exception &e) {
      csm::Error::ErrorType aErrorType =
          csm::Error::SENSOR_MODEL_NOT_CONSTRUCTIBLE;
      std::string aMessage = "Could not construct model [";
      aMessage += modelName;
      aMessage += "] with error [";
      aMessage += e.what();
      aMessage += "]";
      std::string aFunction = "UsgsAstroPlugin::constructModelFromISD()";
      MESSAGE_LOG(aMessage);
      throw csm::Error(aErrorType, aMessage, aFunction);
    }
    return model;
  } else if (modelName == UsgsAstroSarSensorModel::_SENSOR_MODEL_NAME) {
    UsgsAstroSarSensorModel *model = new UsgsAstroSarSensorModel();
    MESSAGE_LOG("Trying to construct a UsgsAstroSarSensorModel");
    try {
      model->replaceModelState(
          model->constructStateFromIsd(stringIsd, warnings));
    } catch (std::exception &e) {
      csm::Error::ErrorType aErrorType =
          csm::Error::SENSOR_MODEL_NOT_CONSTRUCTIBLE;
      std::string aMessage = "Could not construct model [";
      aMessage += modelName;
      aMessage += "] with error [";
      aMessage += e.what();
      aMessage += "]";
      std::string aFunction = "UsgsAstroPlugin::constructModelFromISD()";
      MESSAGE_LOG(aMessage);
      throw csm::Error(aErrorType, aMessage, aFunction);
    }
    return model;
  } else {
    csm::Error::ErrorType aErrorType = csm::Error::SENSOR_MODEL_NOT_SUPPORTED;
    std::string aMessage = "Model [" + modelName + "] not supported: ";
    std::string aFunction = "UsgsAstroPlugin::constructModelFromISD()";
    MESSAGE_LOG(aMessage);
    throw csm::Error(aErrorType, aMessage, aFunction);
  }
}

csm::Model *UsgsAstroPlugin::constructModelFromState(
    const std::string &modelState, csm::WarningList *warnings) const {
  MESSAGE_LOG("Runing constructModelFromState with modelState: {}", modelState);
  json state = json::parse(modelState);
  std::string modelName = state["m_modelName"];
  MESSAGE_LOG("Using model name: {}", modelName);

  if (modelName == UsgsAstroFrameSensorModel::_SENSOR_MODEL_NAME) {
    MESSAGE_LOG("Constructing a UsgsAstroFrameSensorModel");
    UsgsAstroFrameSensorModel *model = new UsgsAstroFrameSensorModel();
    model->replaceModelState(modelState);
    return model;
  } else if (modelName == UsgsAstroLsSensorModel::_SENSOR_MODEL_NAME) {
    MESSAGE_LOG("Constructing a UsgsAstroLsSensorModel");
    UsgsAstroLsSensorModel *model = new UsgsAstroLsSensorModel();
    model->replaceModelState(modelState);
    return model;
  } else if (modelName == UsgsAstroSarSensorModel::_SENSOR_MODEL_NAME) {
    MESSAGE_LOG("Constructing a UsgsAstroSarSensorModel");
    UsgsAstroSarSensorModel *model = new UsgsAstroSarSensorModel();
    model->replaceModelState(modelState);
    return model;
  } else {
    csm::Error::ErrorType aErrorType = csm::Error::ISD_NOT_SUPPORTED;
    std::string aMessage = "Model" + modelName + " not supported: ";
    std::string aFunction = "UsgsAstroPlugin::constructModelFromState()";
    MESSAGE_LOG(aMessage);
    throw csm::Error(aErrorType, aMessage, aFunction);
  }
}
