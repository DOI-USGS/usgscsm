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

#include <json.hpp>
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
const int         UsgsAstroPlugin::_NUM_ISD_KEYWORDS = 21;

const std::string UsgsAstroPlugin::_ISD_KEYWORD[] =
{
   "model_name",
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

// const json UsgsAstroPlugin::MODEL_KEYWORDS = {
//   {UsgsAstroFrameSensorModel::_SENSOR_MODEL_NAME, UsgsAstroFrameSensorModel::_STATE_KEYWORD},
//   {UsgsAstroLsSensorModel::_SENSOR_MODEL_NAME, UsgsAstroLsSensorModel::_STATE_KEYWORD}
// };


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
  // return SUPPORTED_MODELS[modelIndex];
  return "";
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
    catch(...) {
      return false;
    }
}


bool UsgsAstroPlugin::canModelBeConstructedFromISD(const csm::Isd &imageSupportData,
                                              const std::string &modelName,
                                              csm::WarningList *warnings) const {
  try {
    csm::Model* model = constructModelFromISD(imageSupportData, modelName, warnings);
    return (bool)model;
  }
  catch(...) {
    return false;
  }
}


// This function takes a csm::Isd which only has the image filename set. It uses this filename to
// find a metadata json file located alongside the image file and returns a json
// encoded string.
std::string UsgsAstroPlugin::loadImageSupportData(const csm::Isd &imageSupportDataOriginal) const {

  // Get image location from the input csm::Isd:
  std::string imageFilename = imageSupportDataOriginal.filename();
  size_t lastIndex = imageFilename.find_last_of(".");
  std::string baseName = imageFilename.substr(0, lastIndex);
  std::string isdFilename = baseName.append(".json");

  try {
    std::ifstream isd_sidecar(isdFilename);
    json jsonisd;
    isd_sidecar >> jsonisd;
    return jsonisd.dump();

  } catch (...) {
    std::string errorMessage = "Could not read metadata file associated with image: ";
    errorMessage.append(isdFilename);
    throw csm::Error(csm::Error::FILE_READ, errorMessage,
                     "UsgsAstroPlugin::loadImageSupportData");
  }
}


std::string UsgsAstroPlugin::getModelNameFromModelState(const std::string &modelState,
                                                   csm::WarningList *warnings) const {
  auto state = json::parse(modelState);

  std::string name = state.value<std::string>("model_name", "");

  if (name == "") {
      csm::Error::ErrorType aErrorType = csm::Error::INVALID_SENSOR_MODEL_STATE;
      std::string aMessage = "No 'model_name' key in the model state object.";
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
  catch(...) {
      return false;
  }
  return true;
}


std::string UsgsAstroPlugin::getStateFromISD(csm::Isd imageSupportData) const {
    std::string stringIsd = loadImageSupportData(imageSupportData);
    json jsonIsd = json::parse(stringIsd);
    return convertISDToModelState(imageSupportData, jsonIsd.at("modelName"));
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
      }
      catch (...) {
        csm::Error::ErrorType aErrorType = csm::Error::SENSOR_MODEL_NOT_CONSTRUCTIBLE;
        std::string aMessage = "Invalid ISD for Model " + modelName + ": ";
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
      catch (...) {
        csm::Error::ErrorType aErrorType = csm::Error::SENSOR_MODEL_NOT_CONSTRUCTIBLE;
        std::string aMessage = "Invalid ISD for Model " + modelName + ": ";
        std::string aFunction = "UsgsAstroPlugin::constructModelFromISD()";
        throw csm::Error(aErrorType, aMessage, aFunction);
      }
      return model;
    }
    else {
      csm::Error::ErrorType aErrorType = csm::Error::SENSOR_MODEL_NOT_SUPPORTED;
      std::string aMessage = "Model" + modelName + " not supported: ";
      std::string aFunction = "UsgsAstroPlugin::constructModelFromISD()";
      throw csm::Error(aErrorType, aMessage, aFunction);
    }
}


csm::Model *UsgsAstroPlugin::constructModelFromState(const std::string& modelState,
                                                csm::WarningList *warnings) const {

    json state = json::parse(modelState);
    std::string modelName = state["modelName"];

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
    else {
      csm::Error::ErrorType aErrorType = csm::Error::ISD_NOT_SUPPORTED;
      std::string aMessage = "Model" + modelName + " not supported: ";
      std::string aFunction = "UsgsAstroPlugin::constructModelFromState()";
      throw csm::Error(aErrorType, aMessage, aFunction);
    }
}
