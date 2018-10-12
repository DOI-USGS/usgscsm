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
const std::string USGSAstroPlugin::_PLUGIN_NAME = "USGSAstroPluginCSM";
const std::string USGSAstroPlugin::_MANUFACTURER_NAME = "UsgsAstrogeology";
const std::string USGSAstroPlugin::_RELEASE_DATE = "20170425";
const int         USGSAstroPlugin::_N_SENSOR_MODELS = 2;
const int         USGSAstroPlugin::_NUM_ISD_KEYWORDS = 21;

const std::string USGSAstroPlugin::_ISD_KEYWORD[] =
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

// const json USGSAstroPlugin::MODEL_KEYWORDS = {
//   {UsgsAstroFrameSensorModel::_SENSOR_MODEL_NAME, UsgsAstroFrameSensorModel::_STATE_KEYWORD},
//   {UsgsAstroLsSensorModel::_SENSOR_MODEL_NAME, UsgsAstroLsSensorModel::_STATE_KEYWORD}
// };


// Static Instance of itself
const USGSAstroPlugin USGSAstroPlugin::m_registeredPlugin;

USGSAstroPlugin::USGSAstroPlugin() {
}


USGSAstroPlugin::~USGSAstroPlugin() {
}


std::string USGSAstroPlugin::getPluginName() const {
  return _PLUGIN_NAME;
}


std::string USGSAstroPlugin::getManufacturer() const {
  return _MANUFACTURER_NAME;
}


std::string USGSAstroPlugin::getReleaseDate() const {
  return _RELEASE_DATE;
}


csm::Version USGSAstroPlugin::getCsmVersion() const {
  return CURRENT_CSM_VERSION;
}


size_t USGSAstroPlugin::getNumModels() const {
  return _N_SENSOR_MODELS;
}


std::string USGSAstroPlugin::getModelName(size_t modelIndex) const {
  // return SUPPORTED_MODELS[modelIndex];
  return "";
}


std::string USGSAstroPlugin::getModelFamily(size_t modelIndex) const {
  return CSM_RASTER_FAMILY;
}


csm::Version USGSAstroPlugin::getModelVersion(const std::string &modelName) const {
  return csm::Version(1, 0, 0);
}


bool USGSAstroPlugin::canModelBeConstructedFromState(const std::string &modelName,
                                                const std::string &modelState,
                                                csm::WarningList *warnings) const {

  bool constructible = true;

  // Get the model name from the model state
  std::string model_name_from_state;
  try {
    model_name_from_state = getModelNameFromModelState(modelState, warnings);
  }
  catch(...) {
    return false;
  }

  // Check that the plugin supports the model
  if (modelName != model_name_from_state ||
      modelName != UsgsAstroFrameSensorModel::_SENSOR_MODEL_NAME){
          constructible = false;
      }
  // Check that the necessary keys are there (this does not chek values at all.)
  // This sort of check should probably be moved into the sensor model
  // auto state = json::parse(modelState);
  // for(auto &key : MODEL_KEYWORDS[model_name_from_state]){
  //     if (state.find(key) == state.end()){
  //         constructible = false;
  //         break;
  //     }
  // }
  return constructible;
}


bool USGSAstroPlugin::canModelBeConstructedFromISD(const csm::Isd &imageSupportData,
                                              const std::string &modelName,
                                              csm::WarningList *warnings) const {
  return canISDBeConvertedToModelState(imageSupportData, modelName, warnings);
}


// This function takes a csm::Isd which only has the image filename set. It uses this filename to
// find a metadata json file loacated alongside the image file. It creates and returns new csm::Isd
// with its parameters populated by the metadata file.
std::string USGSAstroPlugin::loadImageSupportData(const csm::Isd &imageSupportDataOriginal) const {

  // Get image location from the input csm::Isd:
  std::string imageFilename = imageSupportDataOriginal.filename();
  std::cout << "image file name " << imageFilename << std::endl;
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
                     "USGSAstroPlugin::loadImageSupportData");
  }
}


std::string USGSAstroPlugin::getModelNameFromModelState(const std::string &modelState,
                                                   csm::WarningList *warnings) const {
  auto state = json::parse(modelState);

  std::string name = state.value<std::string>("model_name", "");

  if (name == "") {
      csm::Error::ErrorType aErrorType = csm::Error::INVALID_SENSOR_MODEL_STATE;
      std::string aMessage = "No 'model_name' key in the model state object.";
      std::string aFunction = "USGSAstroPlugin::getModelNameFromModelState";
      csm::Error csmErr(aErrorType, aMessage, aFunction);
      throw(csmErr);
  }

  // if (MODEL_KEYWORDS.find(name) == MODEL_KEYWORDS.end()){
  //     csm::Error::ErrorType aErrorType = csm::Error::SENSOR_MODEL_NOT_SUPPORTED;
  //     std::string aMessage = "Sensor model not supported.";
  //     std::string aFunction = "USGSAstroPlugin::getModelNameFromModelState()";
  //     csm::Error csmErr(aErrorType, aMessage, aFunction);
  //     throw(csmErr);
  // }

  return name;
}


bool USGSAstroPlugin::canISDBeConvertedToModelState(const csm::Isd &imageSupportData,
                                               const std::string &modelName,
                                               csm::WarningList *warnings) const {
  try {
       convertISDToModelState(imageSupportData, modelName);
  }
  catch(...) {
      // key didn't exist
      return false;
  }
  return true;
}


std::string USGSAstroPlugin::getStateFromISD(csm::Isd imageSupportData) const {
    std::string stringIsd = loadImageSupportData(imageSupportData);
    json jsonIsd = json::parse(stringIsd);
    return convertISDToModelState(imageSupportData, jsonIsd.at("modelName"));
}


std::string USGSAstroPlugin::convertISDToModelState(const csm::Isd &imageSupportData,
                                               const std::string &modelName,
                                               csm::WarningList *warnings) const {

  csm::Model* sensor_model = constructModelFromISD(
                             imageSupportData, modelName);

  if (sensor_model == 0) {
      csm::Error::ErrorType aErrorType = csm::Error::ISD_NOT_SUPPORTED;
      std::string aMessage = "ISD not supported: ";
      std::string aFunction = "USGSAstroPlugin::convertISDToModelState()";
      throw csm::Error(aErrorType, aMessage, aFunction);
  }
  return sensor_model->getModelState();
}

csm::Model *USGSAstroPlugin::constructModelFromISD(const csm::Isd &imageSupportDataOriginal,
                                              const std::string &modelName,
                                              csm::WarningList *warnings) const {


    std::cout << "entering contruction" << std::endl;
    // if (!canISDBeConvertedToModelState(imageSupportDataOriginal, modelName)) {
    //     csm::Error::ErrorType aErrorType = csm::Error::ISD_NOT_SUPPORTED;
    //     std::string aMessage = "Model" + modelName + " not supported: ";
    //     std::string aFunction = "USGSAstroPlugin::constructModelFromISD()";
    //     throw csm::Error(aErrorType, aMessage, aFunction);
    // }
    std::cout << "Can be contructed" << std::endl;


    std::string stringIsd = loadImageSupportData(imageSupportDataOriginal);
    std::cout << "image data loaded" << std::endl;

    std::cout << "ISD: " << stringIsd << std::endl;

    if (modelName == UsgsAstroFrameSensorModel::_SENSOR_MODEL_NAME) {
        std::cout << "Building Frame Sensor Model" << std::endl;
        return new UsgsAstroFrameSensorModel(stringIsd);
    }
    else if (modelName == UsgsAstroLsSensorModel::_SENSOR_MODEL_NAME) {
        return new UsgsAstroLsSensorModel(stringIsd);
    }
    else {
      csm::Error::ErrorType aErrorType = csm::Error::ISD_NOT_SUPPORTED;
      std::string aMessage = "Model" + modelName + " not supported: ";
      std::string aFunction = "USGSAstroPlugin::constructModelFromISD()";
      throw csm::Error(aErrorType, aMessage, aFunction);
    }
}


csm::Model *USGSAstroPlugin::constructModelFromState(const std::string& modelState,
                                                csm::WarningList *warnings) const {

    json state = json::parse(modelState);
    std::string modelName = state["modelName"];
    if (!canModelBeConstructedFromState(modelName, modelState)){
      csm::Error::ErrorType aErrorType = csm::Error::ISD_NOT_SUPPORTED;
      std::string aMessage = "Model" + modelName + " not supported: ";
      std::string aFunction = "USGSAstroPlugin::constructModelFromState()";
      throw csm::Error(aErrorType, aMessage, aFunction);
    }

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
      std::string aFunction = "USGSAstroPlugin::constructModelFromState()";
      throw csm::Error(aErrorType, aMessage, aFunction);
    }
}
