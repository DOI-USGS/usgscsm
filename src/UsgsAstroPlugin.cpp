/** Copyright  © 2017-2022 BAE Systems Information and Electronic Systems Integration Inc.

Redistribution and use in source and binary forms, with or without modification, are permitted
provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions
and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of
conditions and the following disclaimer in the documentation and/or other materials provided
with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. **/

#include "UsgsAstroPlugin.h"
#include "UsgsAstroPluginSupport.h"

#include "UsgsAstroFrameSensorModel.h"
#include "UsgsAstroLsSensorModel.h"
#include "UsgsAstroProjectedSensorModel.h"
#include "UsgsAstroPushFrameSensorModel.h"
#include "UsgsAstroSarSensorModel.h"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <string>

#include <csm/Error.h>
#include <csm/Plugin.h>
#include <csm/Version.h>
#include <csm/Warning.h>
#include <csm/csm.h>

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
    m_logger->log(__VA_ARGS__); \
  }

// Declaration of static variables
const std::string UsgsAstroPlugin::_PLUGIN_NAME = "UsgsAstroPluginCSM";
const std::string UsgsAstroPlugin::_MANUFACTURER_NAME = "UsgsAstrogeology";
const std::string UsgsAstroPlugin::_RELEASE_DATE = "20190222";
const int UsgsAstroPlugin::_N_SENSOR_MODELS = 5;

// Static Instance of itself
const UsgsAstroPlugin UsgsAstroPlugin::m_registeredPlugin;

UsgsAstroPlugin::UsgsAstroPlugin() {
  // Build and register the USGSCSM logger on plugin creation
  char *logFilePtr = getenv("USGSCSM_LOG_FILE");

  if (logFilePtr != NULL) {
    std::string logFile(logFilePtr);

    if (logFile != "") {
      m_logger = spdlog::get("usgscsm_logger");

      if (!m_logger) {
        if (logFile == "stdout") {
          m_logger = spdlog::stdout_color_mt("usgscsm_logger");
        }
        else if (logFile == "stderr") {
          m_logger = spdlog::stderr_color_mt("usgscsm_logger");
        }
        else {
          m_logger = spdlog::basic_logger_mt("usgscsm_logger", logFile);
        }

        char *logLevlPtr = getenv("USGSCSM_LOG_LEVEL");
        if (logLevlPtr != NULL) {
          std::string logLevelStr(logLevlPtr);
          std::transform(logLevelStr.begin(), logLevelStr.end(), logLevelStr.begin(),
              [](unsigned char c){ return std::tolower(c); });
          m_logger->set_level(spdlog::level::from_str(logLevelStr));
        }

      }
    }
  }
}

UsgsAstroPlugin::~UsgsAstroPlugin() {}

std::string UsgsAstroPlugin::getPluginName() const {
  MESSAGE_LOG(spdlog::level::debug, "Get Plugin Name: {}", _PLUGIN_NAME);
  return _PLUGIN_NAME;
}

std::string UsgsAstroPlugin::getManufacturer() const {
  MESSAGE_LOG(spdlog::level::debug, "Get Manufacturer Name: {}", _MANUFACTURER_NAME);
  return _MANUFACTURER_NAME;
}

std::string UsgsAstroPlugin::getReleaseDate() const {
  MESSAGE_LOG(spdlog::level::debug, "Get Release Date: {}", _RELEASE_DATE);
  return _RELEASE_DATE;
}

csm::Version UsgsAstroPlugin::getCsmVersion() const {
  MESSAGE_LOG(spdlog::level::debug, "Get Current CSM Version");
  return CURRENT_CSM_VERSION;
}

size_t UsgsAstroPlugin::getNumModels() const {
  MESSAGE_LOG(spdlog::level::debug, "Get Number of Sensor Models: {}", _N_SENSOR_MODELS);
  return _N_SENSOR_MODELS;
}

std::string UsgsAstroPlugin::getModelName(size_t modelIndex) const {
  std::vector<std::string> supportedModelNames = {
      UsgsAstroFrameSensorModel::_SENSOR_MODEL_NAME,
      UsgsAstroLsSensorModel::_SENSOR_MODEL_NAME,
      UsgsAstroProjectedSensorModel::_SENSOR_MODEL_NAME,
      UsgsAstroSarSensorModel::_SENSOR_MODEL_NAME,
      UsgsAstroPushFrameSensorModel::_SENSOR_MODEL_NAME};
  MESSAGE_LOG(spdlog::level::debug, "Get Model Name: {}. Used index: {}",
              supportedModelNames[modelIndex], modelIndex);
  return supportedModelNames[modelIndex];
}

std::string UsgsAstroPlugin::getModelFamily(size_t modelIndex) const {
  MESSAGE_LOG(spdlog::level::debug, "Get Model Familey: {}", CSM_RASTER_FAMILY);
  return CSM_RASTER_FAMILY;
}

csm::Version UsgsAstroPlugin::getModelVersion(
    const std::string &modelName) const {
  MESSAGE_LOG(spdlog::level::debug, "Get Model Version");
  return csm::Version(1, 0, 0);
}

bool UsgsAstroPlugin::canModelBeConstructedFromState(
    const std::string &modelName, const std::string &modelState,
    csm::WarningList *warnings) const {
  std::string err_msg;
  try {
    // Use a shared_ptr to not have to manually deallocate the pointer
    std::shared_ptr<csm::Model> model(constructModelFromState(modelState, warnings));
    if (model) {
      // The created model name
      std::string createdModelName = model->getModelName();

      // If the model is of expected type, all is good
      if (createdModelName == modelName)
        return true;

      // Need to stay on to deal with the fact that the model is not of expected type.
      err_msg = "Created a model of type " + createdModelName + " instead of the expected "
        + modelName;
    }
  } catch (std::exception &e) {
    err_msg = e.what();
  } catch (...) {
    err_msg = "Unknown error";
  }

  std::string fullMsg = "Could not create model [";
  fullMsg += modelName;
  fullMsg += "] with error [";
  fullMsg += err_msg;
  fullMsg += "]";

  MESSAGE_LOG(spdlog::level::err, fullMsg);
  if (warnings) {
    warnings->push_back(csm::Warning
                        (csm::Warning::UNKNOWN_WARNING, fullMsg,
                         "UsgsAstroPlugin::canModelBeConstructedFromState()"));
  }

  return false;
}

bool UsgsAstroPlugin::canModelBeConstructedFromISD(
    const csm::Isd &imageSupportData, const std::string &modelName,
    csm::WarningList *warnings) const {
  try {
    std::shared_ptr<csm::Model> model(constructModelFromISD(imageSupportData, modelName, warnings));
    if (model)
      return true;

  } catch (std::exception &e) {
    if (warnings) {
      std::string msg = "Could not create model [";
      msg += modelName;
      msg += "] with error [";
      msg += e.what();
      msg += "]";
      MESSAGE_LOG(spdlog::level::warn, msg);
      warnings->push_back(csm::Warning(
          csm::Warning::UNKNOWN_WARNING, msg,
          "UsgsAstroPlugin::canModelBeConstructedFromISD()"));
    }
  } catch (...) {
    if (warnings) {
      std::string msg = "Could not create model [";
      msg += modelName;
      msg += "] with an unknown error.";
      MESSAGE_LOG(spdlog::level::warn, msg);
      warnings->push_back(csm::Warning(
          csm::Warning::UNKNOWN_WARNING, msg,
          "UsgsAstroPlugin::canModelBeConstructedFromISD()"));
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
  MESSAGE_LOG(spdlog::level::info, "Load Image Support Data using: {}, {}, {}, {}, {}",
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
    MESSAGE_LOG(spdlog::level::err, errorMessage);
    throw csm::Error(csm::Error::FILE_READ, errorMessage,
                     "UsgsAstroPlugin::loadImageSupportData");
  }
}

std::string UsgsAstroPlugin::getModelNameFromModelState(
    const std::string &modelState, csm::WarningList *warnings) const {
  auto state = stateAsJson(modelState);

  std::string name = state.value<std::string>("name_model", "");
  MESSAGE_LOG(spdlog::level::debug, "Get model name from model state. State: {}, Name: {}",
              modelState, name);
  if (name == "") {
    csm::Error::ErrorType aErrorType = csm::Error::INVALID_SENSOR_MODEL_STATE;
    std::string aMessage = "No 'name_model' key in the model state object.";
    std::string aFunction = "UsgsAstroPlugin::getModelNameFromModelState";
    MESSAGE_LOG(spdlog::level::err, aMessage);
    csm::Error csmErr(aErrorType, aMessage, aFunction);
    throw(csmErr);
  }

  return name;
}

bool UsgsAstroPlugin::canISDBeConvertedToModelState(
    const csm::Isd &imageSupportData, const std::string &modelName,
    csm::WarningList *warnings) const {
  MESSAGE_LOG(spdlog::level::debug, "Running canISDBeConvertedToModelState");
  try {
    convertISDToModelState(imageSupportData, modelName, warnings);
  } catch (std::exception &e) {
    if (warnings) {
      std::string msg = "Could not create model [";
      msg += modelName;
      msg += "] state with error [";
      msg += e.what();
      msg += "]";
      MESSAGE_LOG(spdlog::level::warn, msg);
      warnings->push_back(csm::Warning(
          csm::Warning::UNKNOWN_WARNING, msg,
          "UsgsAstroPlugin::canISDBeConvertedToModelState()"));
    }
    return false;
  }
  return true;
}

std::string UsgsAstroPlugin::getStateFromISD(csm::Isd imageSupportData) const {
  MESSAGE_LOG(spdlog::level::debug, "Running getStateFromISD");
  std::string stringIsd = loadImageSupportData(imageSupportData);
  MESSAGE_LOG(spdlog::level::trace, "ISD string: {}", stringIsd);
  json jsonIsd = json::parse(stringIsd);
  return convertISDToModelState(imageSupportData, jsonIsd.at("name_model"));
}

std::string UsgsAstroPlugin::convertISDToModelState(
    const csm::Isd &imageSupportData, const std::string &modelName,
    csm::WarningList *warnings) const {
  MESSAGE_LOG(spdlog::level::info, "Running convertISDToModelState");
  std::shared_ptr<csm::Model> sensor_model
    (constructModelFromISD(imageSupportData, modelName, warnings));
  std::string stateString = sensor_model->getModelState();
  return stateString;
}

csm::Model *UsgsAstroPlugin::constructModelFromISD(
    const csm::Isd &imageSupportDataOriginal, const std::string &modelName,
    csm::WarningList *warnings) const {
  MESSAGE_LOG(spdlog::level::info, "Running constructModelFromISD");
  std::string stringIsd = loadImageSupportData(imageSupportDataOriginal);

  // Try to get the projected model, if not return the the unprojected model
  UsgsAstroProjectedSensorModel *projModel = new UsgsAstroProjectedSensorModel();

  try {
    MESSAGE_LOG(spdlog::level::debug, "Trying to construct a UsgsAstroProjectedSensorModel");
    projModel->replaceModelState(
        projModel->constructStateFromIsd(stringIsd, modelName, warnings));
    MESSAGE_LOG(spdlog::level::debug, "Constructed model: {}", modelName);
    return projModel;
  } catch (std::exception &e) {
    delete projModel;
    csm::Error::ErrorType aErrorType =
        csm::Error::SENSOR_MODEL_NOT_CONSTRUCTIBLE;
    std::string aMessage = "Could not construct model [";
    aMessage += modelName;
    aMessage += "] with error [";
    aMessage += e.what();
    aMessage += "]";
    MESSAGE_LOG(spdlog::level::err, aMessage);
  }

  csm::Model *model = getUsgsCsmModelFromIsd(stringIsd, modelName, warnings);
  return model;
}

csm::Model *UsgsAstroPlugin::constructModelFromState(
    const std::string &modelState, csm::WarningList *warnings) const {
  MESSAGE_LOG(spdlog::level::info, "Runing constructModelFromState with modelState: {}", modelState);
  json state = stateAsJson(modelState);
  std::string modelName = state["m_modelName"];
  MESSAGE_LOG(spdlog::level::debug, "Using model name: {}", modelName);

  try {
    return getUsgsCsmModelFromState(modelState, modelName, warnings);
  }
  catch (std::exception &e) {
    csm::Error::ErrorType aErrorType = csm::Error::ISD_NOT_SUPPORTED;
    std::string aMessage = "Model " + modelName + " not supported";
    aMessage += " with error: [";
    aMessage += e.what();
    aMessage += "]";
    std::string aFunction = "UsgsAstroPlugin::constructModelFromState()";
    MESSAGE_LOG(spdlog::level::err, aMessage);
    throw csm::Error(aErrorType, aMessage, aFunction);
  }
}
