#include "UsgsAstroPlugin.h"

#include "UsgsAstroFrameSensorModel.h"
#include "UsgsAstroLsSensorModel.h"

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
const std::string UsgsAstroPlugin::_RELEASE_DATE = "20170425";
const int         UsgsAstroPlugin::_N_SENSOR_MODELS = 2;
std::shared_ptr<spdlog::logger> UsgsAstroPlugin::_LOGGER = spdlog::basic_logger_mt("USGSCSM-Plugin", "USGSlog.txt");

const std::string UsgsAstroPlugin::_ISD_KEYWORD[] =
{
   "name_model",
   "center_ephemeris_time",
   "dt_ephemeris",
   "focal2pixel_lines",
   "focal2pixel_samples",
   "focal_length_model",
   "image_lines",
   "image_samples",
   "interpolation_method",
   "number_of_ephemerides",
   "optical_distortion",
   "radii",
   "reference_height",
   "sensor_location_unit",
   "sensor_location",
   "sensor_orientation",
   "sensor_velocity",
   "detector_center",
   "starting_detector_line",
   "starting_detector_sample",
   "starting_ephemeris_time",
   "sun_position"
};

// Static Instance of itself
const UsgsAstroPlugin UsgsAstroPlugin::m_registeredPlugin;

UsgsAstroPlugin::UsgsAstroPlugin() {
}


UsgsAstroPlugin::~UsgsAstroPlugin() {
}


std::string UsgsAstroPlugin::getPluginName() const {
  _LOGGER->info("Called getPluginName");
  return _PLUGIN_NAME;
}


std::string UsgsAstroPlugin::getManufacturer() const {
  _LOGGER->info("Called getManufacturer");
  return _MANUFACTURER_NAME;
}


std::string UsgsAstroPlugin::getReleaseDate() const {
  _LOGGER->info("Called getReleaseDate");
  return _RELEASE_DATE;
}


csm::Version UsgsAstroPlugin::getCsmVersion() const {
  _LOGGER->info("Called getCsmVersion");
  return CURRENT_CSM_VERSION;
}


size_t UsgsAstroPlugin::getNumModels() const {
  _LOGGER->info("Called getNumModels");
  return _N_SENSOR_MODELS;
}


std::string UsgsAstroPlugin::getModelName(size_t modelIndex) const {
  _LOGGER->info("Called getModelName at index {}", modelIndex);
  std::vector<std::string> supportedModelNames = {
    UsgsAstroFrameSensorModel::_SENSOR_MODEL_NAME,
    UsgsAstroLsSensorModel::_SENSOR_MODEL_NAME
  };
  return supportedModelNames[modelIndex];
}


std::string UsgsAstroPlugin::getModelFamily(size_t modelIndex) const {
  _LOGGER->info("Called CSM_RASTER_FAMILY at index {}", modelIndex);
  return CSM_RASTER_FAMILY;
}


csm::Version UsgsAstroPlugin::getModelVersion(const std::string &modelName) const {
  _LOGGER->info("Called getModelVersion for model {}", modelName);
  return csm::Version(1, 0, 0);
}


bool UsgsAstroPlugin::canModelBeConstructedFromState(const std::string &modelName,
                                                const std::string &modelState,
                                                csm::WarningList *warnings) const {

    _LOGGER->info("Called canModelBeConstructedFromState for model {} with state {}", modelName, modelState);
    try {
      csm::Model* model = constructModelFromState(modelState, warnings);
      _LOGGER->info("Model constructable");
      return (bool)model;
    }
    catch(...) {
      _LOGGER->info("Model not constructable");
      return false;
    }
}


bool UsgsAstroPlugin::canModelBeConstructedFromISD(const csm::Isd &imageSupportData,
                                              const std::string &modelName,
                                              csm::WarningList *warnings) const {
  _LOGGER->info("Called canModelBeConstructedFromISD for model {} with ISD filename {}", modelName, imageSupportData.filename());
  try {
    csm::Model* model = constructModelFromISD(imageSupportData, modelName, warnings);
    _LOGGER->info("Model constructable");
    return (bool)model;
  }
  catch(...) {
    _LOGGER->info("Model not constructable");
    return false;
  }
}


// This function takes a csm::Isd which only has the image filename set. It uses this filename to
// find a metadata json file located alongside the image file and returns a json
// encoded string.
std::string UsgsAstroPlugin::loadImageSupportData(const csm::Isd &imageSupportDataOriginal) const {
  _LOGGER->info("Called loadImageSupportData for ISD filename {}", imageSupportDataOriginal.filename());

  // Get image location from the input csm::Isd:
  std::string imageFilename = imageSupportDataOriginal.filename();
  size_t lastIndex = imageFilename.find_last_of(".");
  std::string baseName = imageFilename.substr(0, lastIndex);
  lastIndex = baseName.find_last_of(DIR_DELIMITER_STR);
  std::string filename = baseName.substr(lastIndex + 1);
  std::string isdFilename = baseName.append(".json");
  _LOGGER->info("Metadata filename: {}", isdFilename);

  try {
    std::ifstream isd_sidecar(isdFilename);
    json jsonisd;
    isd_sidecar >> jsonisd;
    jsonisd["image_identifier"] = filename;
    _LOGGER->info("Metadata file parsed into {}", jsonisd.dump());
    return jsonisd.dump();

  } catch (...) {
    _LOGGER->info("Failed while parsing metadata file");
    std::string errorMessage = "Could not read metadata file associated with image: ";
    errorMessage.append(isdFilename);
    throw csm::Error(csm::Error::FILE_READ, errorMessage,
                     "UsgsAstroPlugin::loadImageSupportData");
  }
}


std::string UsgsAstroPlugin::getModelNameFromModelState(const std::string &modelState,
                                                   csm::WarningList *warnings) const {
  _LOGGER->info("Called getModelNameFromModelState for state {}", modelState);
  auto state = json::parse(modelState);

  std::string name = state.value<std::string>("name_model", "");

  if (name == "") {
      _LOGGER->info("Could not find model name");
      csm::Error::ErrorType aErrorType = csm::Error::INVALID_SENSOR_MODEL_STATE;
      std::string aMessage = "No 'name_model' key in the model state object.";
      std::string aFunction = "UsgsAstroPlugin::getModelNameFromModelState";
      csm::Error csmErr(aErrorType, aMessage, aFunction);
      throw(csmErr);
  }
  _LOGGER->info("Model name: {}", name);

  return name;
}


bool UsgsAstroPlugin::canISDBeConvertedToModelState(const csm::Isd &imageSupportData,
                                               const std::string &modelName,
                                               csm::WarningList *warnings) const {
  _LOGGER->info("Called canISDBeConvertedToModelState for model {} and ISD with filename {}", modelName, imageSupportData.filename());
  try {
       std::string modelState = convertISDToModelState(imageSupportData, modelName, warnings);
       _LOGGER->info("ISD converted to model state {}", modelState);
  }
  catch(...) {
      _LOGGER->info("Could not convert ISD to model state");
      return false;
  }
  return true;
}


std::string UsgsAstroPlugin::getStateFromISD(csm::Isd imageSupportData) const {
    _LOGGER->info("Called getStateFromISD for ISD with filename {}", imageSupportData.filename());
    std::string stringIsd = loadImageSupportData(imageSupportData);
    json jsonIsd = json::parse(stringIsd);
    _LOGGER->info("Found model name {}", jsonIsd.value("name_model", ""));
    std::string modelState = convertISDToModelState(imageSupportData, jsonIsd.at("name_model"));
    _LOGGER->info("Created state: {}", modelState);
    return modelState;
}


std::string UsgsAstroPlugin::convertISDToModelState(const csm::Isd &imageSupportData,
                                               const std::string &modelName,
                                               csm::WarningList *warnings) const {
  _LOGGER->info("Called convertISDToModelState for model {} and ISD with filename {}", modelName , imageSupportData.filename());

  csm::Model* sensor_model = constructModelFromISD(imageSupportData, modelName, warnings);
  _LOGGER->info("Created model");
  std::string modelState = sensor_model->getModelState();
  _LOGGER->info("Model state: {}", modelState);
  return modelState;
}


csm::Model *UsgsAstroPlugin::constructModelFromISD(const csm::Isd &imageSupportDataOriginal,
                                              const std::string &modelName,
                                              csm::WarningList *warnings) const {
   _LOGGER->info("Called constructModelFromISD for model {} and ISD with filename {}", modelName , imageSupportDataOriginal.filename());

    std::string stringIsd = loadImageSupportData(imageSupportDataOriginal);
    _LOGGER->info("Meta data loaded as {}", stringIsd);

    if (modelName == UsgsAstroFrameSensorModel::_SENSOR_MODEL_NAME) {
      _LOGGER->info("Creating Frame Sensor Model");
      UsgsAstroFrameSensorModel *model =  new UsgsAstroFrameSensorModel();
      _LOGGER->info("Empty model created");
      try {
        _LOGGER->info("Replacing empty model state with state string from ISD");
        model->replaceModelState(model->constructStateFromIsd(stringIsd, warnings));
      }
      catch (...) {
        _LOGGER->info("Failed while replacing model state");
        csm::Error::ErrorType aErrorType = csm::Error::SENSOR_MODEL_NOT_CONSTRUCTIBLE;
        std::string aMessage = "Invalid ISD for Model " + modelName + ": ";
        std::string aFunction = "UsgsAstroPlugin::constructModelFromISD()";
        throw csm::Error(aErrorType, aMessage, aFunction);
      }
      _LOGGER->info("Model created successfully");
      return model;
    }
    else if (modelName == UsgsAstroLsSensorModel::_SENSOR_MODEL_NAME) {
      _LOGGER->info("Creating Line Scan Sensor Model");
      UsgsAstroLsSensorModel *model =  new UsgsAstroLsSensorModel();
      _LOGGER->info("Empty model created");
      try {
        _LOGGER->info("Replacing empty model state with state string from ISD");
        model->replaceModelState(model->constructStateFromIsd(stringIsd, warnings));
      }
      catch (...) {
        _LOGGER->info("Failed while replacing model state");
        csm::Error::ErrorType aErrorType = csm::Error::SENSOR_MODEL_NOT_CONSTRUCTIBLE;
        std::string aMessage = "Invalid ISD for Model " + modelName + ": ";
        std::string aFunction = "UsgsAstroPlugin::constructModelFromISD()";
        throw csm::Error(aErrorType, aMessage, aFunction);
      }
      _LOGGER->info("Model created successfully");
      return model;
    }
    else {
      _LOGGER->info("Invalid model name");
      csm::Error::ErrorType aErrorType = csm::Error::SENSOR_MODEL_NOT_SUPPORTED;
      std::string aMessage = "Model" + modelName + " not supported: ";
      std::string aFunction = "UsgsAstroPlugin::constructModelFromISD()";
      throw csm::Error(aErrorType, aMessage, aFunction);
    }
}


csm::Model *UsgsAstroPlugin::constructModelFromState(const std::string& modelState,
                                                csm::WarningList *warnings) const {
    _LOGGER->info("Called constructModelFromState for state {}", modelState);

    json state = json::parse(modelState);
    std::string modelName = state.value("m_modelName", "");
    _LOGGER->info("Model name: {}", modelName);

    if (modelName == UsgsAstroFrameSensorModel::_SENSOR_MODEL_NAME) {
         _LOGGER->info("Creating Frame Sensor Model");
         UsgsAstroFrameSensorModel* model = new UsgsAstroFrameSensorModel();
         _LOGGER->info("Empty model created");
         _LOGGER->info("Replacing empty model state with state string");
         model->replaceModelState(modelState);
         _LOGGER->info("Model created successfully");
         return model;
    }
    else if (modelName == UsgsAstroLsSensorModel::_SENSOR_MODEL_NAME) {
         _LOGGER->info("Creating Line Scan Sensor Model");
        UsgsAstroLsSensorModel* model = new UsgsAstroLsSensorModel();
        _LOGGER->info("Empty model created");
        _LOGGER->info("Replacing empty model state with state string");
        model->replaceModelState(modelState);
        _LOGGER->info("Model created successfully");
        return model;
    }
    else {
      _LOGGER->info("Invalid model name");
      csm::Error::ErrorType aErrorType = csm::Error::ISD_NOT_SUPPORTED;
      std::string aMessage = "Model" + modelName + " not supported: ";
      std::string aFunction = "UsgsAstroPlugin::constructModelFromState()";
      throw csm::Error(aErrorType, aMessage, aFunction);
    }
}
