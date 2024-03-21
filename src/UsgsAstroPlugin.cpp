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

/**
 * @brief Retrieves the name of the plugin.
 * @description This function returns the name of the plugin, identifying it
 * within the CSM (Community Sensor Model) framework.
 * 
 * @return A string containing the plugin's name.
 */
std::string UsgsAstroPlugin::getPluginName() const {
  MESSAGE_LOG(spdlog::level::debug, "Get Plugin Name: {}", _PLUGIN_NAME);
  return _PLUGIN_NAME;
}

/**
 * @brief Retrieves the manufacturer's name.
 * @description This function returns the name of the manufacturer of the
 * plugin.
 * 
 * @return A string containing the name of the manufacturer.
 */
std::string UsgsAstroPlugin::getManufacturer() const {
  MESSAGE_LOG(spdlog::level::debug, "Get Manufacturer Name: {}", _MANUFACTURER_NAME);
  return _MANUFACTURER_NAME;
}

/**
 * @brief Retrieves the release date of the plugin.
 * @description This function returns the release date of the plugin, indicating
 * when this version of the plugin was made available.
 * 
 * @return A string containing the release date of the plugin.
 */
std::string UsgsAstroPlugin::getReleaseDate() const {
  MESSAGE_LOG(spdlog::level::debug, "Get Release Date: {}", _RELEASE_DATE);
  return _RELEASE_DATE;
}

/**
 * @brief Retrieves the version of the CSM API the plugin conforms to.
 * @description This function returns the version of the Community Sensor Model
 * (CSM) API that this plugin is designed to work with.
 * 
 * @return A csm::Version object representing the CSM API version.
 */
csm::Version UsgsAstroPlugin::getCsmVersion() const {
  MESSAGE_LOG(spdlog::level::debug, "Get Current CSM Version");
  return CURRENT_CSM_VERSION;
}

/**
 * @brief Retrieves the number of sensor models supported by the plugin.
 * @description This function returns the total number of distinct sensor models
 * provided by the plugin.
 * 
 * @return The number of sensor models supported by the plugin as a size_t.
 */
size_t UsgsAstroPlugin::getNumModels() const {
  MESSAGE_LOG(spdlog::level::debug, "Get Number of Sensor Models: {}", _N_SENSOR_MODELS);
  return _N_SENSOR_MODELS;
}

/**
 * @brief Retrieves the name of a specific sensor model supported by the plugin.
 * @description This function returns the name of a sensor model identified by
 * its index.
 * 
 * @param modelIndex The index of the sensor model, within the range of models
 * supported by the plugin.
 * 
 * @return A string containing the name of the specified sensor model.
 */
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

/**
 * @brief Retrieves the model family for a specific sensor model.
 * @description This function returns the family type of the sensor model
 * identified by the provided model index.
 * 
 * @param modelIndex The index of the sensor model within the plugin's supported models.
 * 
 * @return A string identifying the model's family. For this plugin, all models
 * belong to the CSM_RASTER_FAMILY.
 */
std::string UsgsAstroPlugin::getModelFamily(size_t modelIndex) const {
  MESSAGE_LOG(spdlog::level::debug, "Get Model Familey: {}", CSM_RASTER_FAMILY);
  return CSM_RASTER_FAMILY;
}

/**
 * @brief Retrieves the version of a specific sensor model supported by the plugin.
 * @description This function returns the version of the sensor model identified
 * by its name.
 * 
 * @param modelName The name of the sensor model for which to retrieve the version.
 * 
 * @return A csm::Version object representing the version of the specified sensor model.
 */
csm::Version UsgsAstroPlugin::getModelVersion(
    const std::string &modelName) const {
  MESSAGE_LOG(spdlog::level::debug, "Get Model Version");
  return csm::Version(1, 0, 0);
}

/**
 * @brief Determines if a sensor model can be constructed from a given state string.
 * @description This function checks if a sensor model with the specified name
 * can be accurately constructed from a provided state string.
 * 
 * @param modelName The name of the sensor model to be constructed.
 * @param modelState A string representing the state from which the model should 
 * be constructed.
 * @param warnings A pointer to a csm::WarningList for logging any warnings that
 * occur during the model construction check. This can include warnings about
 * mismatches between the intended model and the constructed model or other 
 * issues.
 * 
 * @return A boolean value indicating whether the model can be constructed from
 * the provided state. Returns true if the model can be constructed, false otherwise.
 */
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

/**
 * @brief Checks if a sensor model can be constructed from ISD.
 * @description Determines whether a sensor model specified by modelName
 * can be accurately constructed from a given ISD (Imaging Support Data).
 * 
 * @param imageSupportData The ISD object containing imaging support data
 * necessary for constructing the model.
 * @param modelName The name of the sensor model to be constructed.
 * @param warnings A pointer to a csm::WarningList for logging any warnings
 * that occur during the model construction check. This includes any issues
 * encountered while attempting to construct the model from the ISD.
 * 
 * @return A boolean value indicating whether the model can be constructed
 * from the provided ISD. Returns true if construction is possible, false
 * otherwise.
 */
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

/**
 * @brief Loads image support data from a metadata file.
 * @description Given a csm::Isd object that includes only the image filename,
 * this function locates a corresponding metadata JSON file in the same
 * directory, reads it, and returns its content as a JSON-encoded string.
 * The function appends the image identifier to the JSON object before returning.
 * 
 * @param imageSupportDataOriginal A csm::Isd object containing the filename of
 * the image for which support data is to be loaded.
 * 
 * @return A string containing the JSON-encoded image support data.
 * 
 * @throws csm::Error If there is an issue reading the metadata file, an error
 * is thrown with details of the failure.
 */
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

/**
 * @brief Retrieves the model name from a given model state.
 * @description Extracts the sensor model name from a JSON-encoded string
 * representing the model's state.
 * 
 * @param modelState A JSON-encoded string representing the state of a sensor model.
 * @param warnings A pointer to a csm::WarningList for logging any warnings that
 * occur during the extraction of the model name from the state.
 * 
 * @return The name of the sensor model extracted from the model state.
 * 
 * @throws csm::Error If the 'name_model' key is missing from the model state,
 * indicating an invalid or incomplete state representation.
 */
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

/**
 * @brief Checks if ISD can be converted to a model state for a specific model.
 * @description Determines whether the provided ISD (Imaging Support Data) can
 * be successfully converted into a model state string for the sensor model
 * specified by modelName.
 * 
 * @param imageSupportData The ISD object containing imaging support data.
 * @param modelName The name of the sensor model for which the ISD is to be
 * converted.
 * @param warnings A pointer to a csm::WarningList for logging any warnings that
 * occur during the ISD conversion check. This can include issues encountered
 * while attempting to convert the ISD to a model state.
 * 
 * @return A boolean value indicating whether the ISD can be converted to a
 * model state for the specified model. Returns true if conversion is possible,
 * false otherwise.
 */
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

/**
 * @brief Retrieves a model state from ISD.
 * @description Extracts and returns a model state string from the provided ISD
 * by first loading any associated image support data and then converting that
 * data into a state string that represents the sensor model identified within
 * the ISD.
 * 
 * @param imageSupportData The ISD object from which to derive the model state.
 * 
 * @return A string representing the model state derived from the provided ISD.
 */
std::string UsgsAstroPlugin::getStateFromISD(csm::Isd imageSupportData) const {
  MESSAGE_LOG(spdlog::level::debug, "Running getStateFromISD");
  std::string stringIsd = loadImageSupportData(imageSupportData);
  MESSAGE_LOG(spdlog::level::trace, "ISD string: {}", stringIsd);
  json jsonIsd = json::parse(stringIsd);
  return convertISDToModelState(imageSupportData, jsonIsd.at("name_model"));
}

/**
 * @brief Converts ISD to a model state for a specific sensor model.
 * @description Converts the provided ISD (Imaging Support Data) into a model
 * state string specifically for the sensor model named modelName.
 * 
 * @param imageSupportData The ISD object containing imaging support data.
 * @param modelName The name of the sensor model for which the ISD is to be
 * converted.
 * @param warnings A pointer to a csm::WarningList for logging any warnings that
 * occur during the ISD to model state conversion. This might include issues
 * with the ISD content or format that could affect the conversion process.
 * 
 * @return A string representing the state of the sensor model as derived from
 * the provided ISD.
 */
std::string UsgsAstroPlugin::convertISDToModelState(
    const csm::Isd &imageSupportData, const std::string &modelName,
    csm::WarningList *warnings) const {
  MESSAGE_LOG(spdlog::level::info, "Running convertISDToModelState");
  std::shared_ptr<csm::Model> sensor_model
    (constructModelFromISD(imageSupportData, modelName, warnings));
  std::string stateString = sensor_model->getModelState();
  return stateString;
}

/**
 * @brief Constructs a sensor model from ISD.
 * @description Creates a sensor model based on the given ISD (Imaging Support Data)
 * and the specified model name.
 * 
 * @param imageSupportDataOriginal The ISD containing imaging support data necessary
 * for model construction.
 * @param modelName The name of the sensor model to be constructed.
 * @param warnings A pointer to a csm::WarningList for logging any warnings that occur
 * during model construction.
 * 
 * @return A pointer to the constructed csm::Model.
 * 
 * @throws csm::Error If the model cannot be constructed from the given ISD, an error
 * is thrown detailing the reason for failure.
 */
csm::Model *UsgsAstroPlugin::constructModelFromISD(
    const csm::Isd &imageSupportDataOriginal, const std::string &modelName,
    csm::WarningList *warnings) const {
  MESSAGE_LOG(spdlog::level::info, "Running constructModelFromISD");
  std::string stringIsd = loadImageSupportData(imageSupportDataOriginal);

  csm::Model *model = getUsgsCsmModel(stringIsd, modelName, warnings);

  // Try to get the projected model, if not return the the unprojected model
  try {
    UsgsAstroProjectedSensorModel *projModel = new UsgsAstroProjectedSensorModel();
    try {
      MESSAGE_LOG(spdlog::level::debug, "Trying to construct a UsgsAstroProjectedSensorModel");
      projModel->replaceModelState(
          projModel->constructStateFromIsd(stringIsd, warnings));
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
      std::string aFunction = "UsgsAstroPlugin::getUsgsCsmModel()";
      throw csm::Error(aErrorType, aMessage, aFunction);
    }
  } catch(std::exception &e) {
    MESSAGE_LOG(spdlog::level::info, "Failed to make projected model with error: \n{}", e.what());
    return model;
  }
}

/**
 * @brief Constructs a sensor model from a model state string.
 * @description Instantiates a sensor model using a given state string that
 * encapsulates the model's configuration.
 * 
 * @param modelState A JSON-encoded string representing the state of the sensor model.
 * @param warnings A pointer to a csm::WarningList for logging any warnings that occur
 * during model construction from state.
 * 
 * @return A pointer to the reconstructed csm::Model.
 * 
 * @throws csm::Error If the model cannot be constructed due to an unsupported model
 * name or any other issues, an error is thrown with details of the failure.
 */
csm::Model *UsgsAstroPlugin::constructModelFromState(
    const std::string &modelState, csm::WarningList *warnings) const {
  MESSAGE_LOG(spdlog::level::info, "Runing constructModelFromState with modelState: {}", modelState);
  json state = stateAsJson(modelState);
  std::string modelName = state["m_modelName"];
  MESSAGE_LOG(spdlog::level::debug, "Using model name: {}", modelName);

  if (modelName == UsgsAstroFrameSensorModel::_SENSOR_MODEL_NAME) {
    MESSAGE_LOG(spdlog::level::debug, "Constructing a UsgsAstroFrameSensorModel");
    UsgsAstroFrameSensorModel *model = new UsgsAstroFrameSensorModel();
    model->replaceModelState(modelState);
    return model;
  } else if (modelName == UsgsAstroLsSensorModel::_SENSOR_MODEL_NAME) {
    MESSAGE_LOG(spdlog::level::debug, "Constructing a UsgsAstroLsSensorModel");
    UsgsAstroLsSensorModel *model = new UsgsAstroLsSensorModel();
    model->replaceModelState(modelState);
    return model;
  } else if (modelName == UsgsAstroProjectedSensorModel::_SENSOR_MODEL_NAME) {
    MESSAGE_LOG(spdlog::level::debug, "Constructing a UsgsAstroProjectedSensorModel");
    UsgsAstroProjectedSensorModel *model = new UsgsAstroProjectedSensorModel();
    model->replaceModelState(modelState);
    return model;
  }else if (modelName == UsgsAstroSarSensorModel::_SENSOR_MODEL_NAME) {
    MESSAGE_LOG(spdlog::level::debug, "Constructing a UsgsAstroSarSensorModel");
    UsgsAstroSarSensorModel *model = new UsgsAstroSarSensorModel();
    model->replaceModelState(modelState);
    return model;
  } else {
    csm::Error::ErrorType aErrorType = csm::Error::ISD_NOT_SUPPORTED;
    std::string aMessage = "Model" + modelName + " not supported: ";
    std::string aFunction = "UsgsAstroPlugin::constructModelFromState()";
    MESSAGE_LOG(spdlog::level::err, aMessage);
    throw csm::Error(aErrorType, aMessage, aFunction);
  }
}
