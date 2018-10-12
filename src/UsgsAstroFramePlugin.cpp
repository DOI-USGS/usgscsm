#define USGS_SENSOR_LIBRARY
#include "UsgsAstroFramePlugin.h"
#include "UsgsAstroFrameSensorModel.h"

#include <algorithm>
#include <cstdlib>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <map>

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
const std::string UsgsAstroFramePlugin::_PLUGIN_NAME = "UsgsAstroFramePluginCSM";
const std::string UsgsAstroFramePlugin::_MANUFACTURER_NAME = "UsgsAstrogeology";
const std::string UsgsAstroFramePlugin::_RELEASE_DATE = "20170425";
const int         UsgsAstroFramePlugin::_N_SENSOR_MODELS = 1;
const int         UsgsAstroFramePlugin::_NUM_ISD_KEYWORDS = 36;
const std::string UsgsAstroFramePlugin::_ISD_KEYWORD[] =
{
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
   "sensor_location",
   "sensor_orientation",
   "sensor_velocity",
   "detector_center",
   "starting_detector_line",
   "starting_detector_sample",
   "starting_ephemeris_time",
   "sun_position"
};

const int         UsgsAstroFramePlugin::_NUM_STATE_KEYWORDS = 32;
const std::string UsgsAstroFramePlugin::_STATE_KEYWORD[] =
{
    "m_focalLength",
    "m_iTransS",
    "m_iTransL",
    "m_boresight",
    "m_transX",
    "m_transY",
    "m_majorAxis",
    "m_minorAxis",
    "m_spacecraftVelocity",
    "m_sunPosition",
    "m_startingDetectorSample",
    "m_startingDetectorLine",
    "m_targetName",
    "m_ifov",
    "m_instrumentID",
    "m_focalLengthEpsilon",
    "m_ccdCenter",
    "m_line_pp",
    "m_sample_pp",
    "m_minElevation",
    "m_maxElevation",
    "m_odtX",
    "m_odtY",
    "m_originalHalfLines",
    "m_originalHalfSamples",
    "m_spacecraftName",
    "m_pixelPitch",
    "m_ephemerisTime",
    "m_nLines",
    "m_nSamples",
    "m_currentParameterValue",
    "m_currentParameterCovariance"
};


// Static Instance of itself
const UsgsAstroFramePlugin UsgsAstroFramePlugin::m_registeredPlugin;

UsgsAstroFramePlugin::UsgsAstroFramePlugin() {
}


UsgsAstroFramePlugin::~UsgsAstroFramePlugin() {
}


std::string UsgsAstroFramePlugin::getPluginName() const {
  return _PLUGIN_NAME;
}


std::string UsgsAstroFramePlugin::getManufacturer() const {
  return _MANUFACTURER_NAME;
}


std::string UsgsAstroFramePlugin::getReleaseDate() const {
  return _RELEASE_DATE;
}


csm::Version UsgsAstroFramePlugin::getCsmVersion() const {
  return CURRENT_CSM_VERSION;
}


size_t UsgsAstroFramePlugin::getNumModels() const {
  return _N_SENSOR_MODELS;
}


std::string UsgsAstroFramePlugin::getModelName(size_t modelIndex) const {

  return UsgsAstroFrameSensorModel::_SENSOR_MODEL_NAME;
}


std::string UsgsAstroFramePlugin::getModelFamily(size_t modelIndex) const {
  return CSM_RASTER_FAMILY;
}


csm::Version UsgsAstroFramePlugin::getModelVersion(const std::string &modelName) const {
  return csm::Version(1, 0, 0);
}


bool UsgsAstroFramePlugin::canModelBeConstructedFromState(const std::string &modelName,
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

  // Check that the necessary keys are there (this does not check values at all.)
  auto state = json::parse(modelState);
  for(auto &key : _STATE_KEYWORD){
      if (state.find(key) == state.end()){
          constructible = false;
          break;
      }
  }
  return constructible;
}


bool UsgsAstroFramePlugin::canModelBeConstructedFromISD(const csm::Isd &imageSupportData,
                                              const std::string &modelName,
                                              csm::WarningList *warnings) const {
  return canISDBeConvertedToModelState(imageSupportData, modelName, warnings);
}


csm::Model *UsgsAstroFramePlugin::constructModelFromState(const std::string& modelState,
                                                csm::WarningList *warnings) const {
    csm::Model *sensor_model = 0;

    // Get the sensor model name from the sensor model state
    std::string model_name_from_state = getModelNameFromModelState(modelState);

    if (model_name_from_state != UsgsAstroFrameSensorModel::_SENSOR_MODEL_NAME){
        csm::Error::ErrorType aErrorType = csm::Error::INVALID_SENSOR_MODEL_STATE;
        std::string aMessage = "Model name from state is not recognized.";
        std::string aFunction = "UsgsAstroFramePlugin::constructModelFromState()";
        throw csm::Error(aErrorType, aMessage, aFunction);
    }

    if (!canModelBeConstructedFromState(model_name_from_state, modelState)){
        csm::Error::ErrorType aErrorType = csm::Error::INVALID_SENSOR_MODEL_STATE;
        std::string aMessage = "Model state is not valid.";
        std::string aFunction = "UsgsAstroFramePlugin::constructModelFromState()";
        throw csm::Error(aErrorType, aMessage, aFunction);
    }

    // Create the model from the state
    UsgsAstroFrameSensorModel* mdsensor_model = new UsgsAstroFrameSensorModel();

    auto state = json::parse(modelState);

    mdsensor_model->m_ccdCenter[0] = state["m_ccdCenter"][0];
    mdsensor_model->m_ccdCenter[1] = state["m_ccdCenter"][1];
    mdsensor_model->m_ephemerisTime = state["m_ephemerisTime"];
    mdsensor_model->m_focalLength = state["m_focalLength"];
    mdsensor_model->m_focalLengthEpsilon = state["m_focalLengthEpsilon"];
    mdsensor_model->m_ifov = state["m_ifov"];
    mdsensor_model->m_instrumentID = state["m_instrumentID"];

    mdsensor_model->m_majorAxis = state["m_majorAxis"];
    mdsensor_model->m_minorAxis = state["m_minorAxis"];
    mdsensor_model->m_startingDetectorLine = state["m_startingDetectorLine"];
    mdsensor_model->m_startingDetectorSample = state["m_startingDetectorSample"];
    mdsensor_model->m_line_pp = state["m_line_pp"];
    mdsensor_model->m_sample_pp = state["m_sample_pp"];
    mdsensor_model->m_originalHalfLines = state["m_originalHalfLines"];
    mdsensor_model->m_originalHalfSamples = state["m_originalHalfSamples"];
    mdsensor_model->m_spacecraftName = state["m_spacecraftName"];
    mdsensor_model->m_pixelPitch = state["m_pixelPitch"];
    mdsensor_model->m_nLines = state["m_nLines"];
    mdsensor_model->m_nSamples = state["m_nSamples"];
    mdsensor_model->m_minElevation = state["m_minElevation"];
    mdsensor_model->m_maxElevation = state["m_maxElevation"];

    for (int i=0;i<3;i++){
        mdsensor_model->m_boresight[i] = state["m_boresight"][i];
        mdsensor_model->m_iTransL[i] = state["m_iTransL"][i];
        mdsensor_model->m_iTransS[i] = state["m_iTransS"][i];

        mdsensor_model->m_transX[i] = state["m_transX"][i];
        mdsensor_model->m_transY[i] = state["m_transY"][i];
        mdsensor_model->m_spacecraftVelocity[i] = state["m_spacecraftVelocity"][i];
        mdsensor_model->m_sunPosition[i] = state["m_sunPosition"][i];
    }

    // Having types as vectors, instead of arrays makes interoperability with
    // the JSON library very easy.
    mdsensor_model->m_currentParameterValue = state["m_currentParameterValue"].get<std::vector<double>>();
    mdsensor_model->m_odtX = state["m_odtX"].get<std::vector<double>>();
    mdsensor_model->m_odtY = state["m_odtY"].get<std::vector<double>>();

    mdsensor_model->m_currentParameterCovariance = state["m_currentParameterCovariance"].get<std::vector<double>>();

sensor_model = mdsensor_model;
return sensor_model;
}


// This function takes a csm::Isd which only has the image filename set. It uses this filename to
// find a metadata json file loacated alongside the image file. It creates and returns new csm::Isd
// with its parameters populated by the metadata file.
csm::Isd UsgsAstroFramePlugin::loadImageSupportData(const csm::Isd &imageSupportDataOriginal) const{
  // Get image location from the input csm::Isd:
  std::string imageFilename = imageSupportDataOriginal.filename();

  // Load 'sidecar' ISD file
  size_t lastIndex = imageFilename.find_last_of(".");
  std::string baseName = imageFilename.substr(0, lastIndex);
  std::string isdFilename = baseName.append(".json");

  csm::Isd imageSupportData(isdFilename);
  imageSupportData.clearAllParams();

  try {
    std::ifstream isdFile(isdFilename);
    json jsonIsd = json::parse(isdFile);

    for (json::iterator it = jsonIsd.begin(); it != jsonIsd.end(); ++it) {
     json jsonValue = it.value();
     if (jsonValue.is_array()) {
        for (int i = 0; i < jsonValue.size(); i++) {
           imageSupportData.addParam(it.key(), jsonValue[i].dump());
        }
     }
     else {
        imageSupportData.addParam(it.key(), jsonValue.dump());
     }
  }
    isdFile.close();
  } catch (...) {
    std::string errorMessage = "Could not read metadata file associated with image: ";
    errorMessage.append(isdFilename);
    throw csm::Error(csm::Error::FILE_READ, errorMessage,
                     "UsgsAstroFramePlugin::loadImageSupportData");
  }

  return imageSupportData;
}

csm::Model *UsgsAstroFramePlugin::constructModelFromISD(const csm::Isd &imageSupportDataOriginal,
                                              const std::string &modelName,
                                              csm::WarningList *warnings) const {

  auto metric_conversion = [](double val, std::string from, std::string to="m") {
     json typemap = {
        {"m", 0},
        {"km", 3}
     };

     // everything to lowercase
     std::transform(from.begin(), from.end(), from.begin(), ::tolower);
     std::transform(to.begin(), to.end(), to.begin(), ::tolower);
     return val*pow(10, typemap[from].get<int>() - typemap[to].get<int>());
  };

  // Check if the sensor model can be constructed from ISD given the model name
  if (!canModelBeConstructedFromISD(imageSupportDataOriginal, modelName)) {
    throw csm::Error(csm::Error::ISD_NOT_SUPPORTED,
                     "Sensor model support data provided is not supported by this plugin",
                     "UsgsAstroFramePlugin::constructModelFromISD");
  }

  csm::Isd imageSupportData = loadImageSupportData(imageSupportDataOriginal);

  // Create the empty sensorModel
  UsgsAstroFrameSensorModel *sensorModel = new UsgsAstroFrameSensorModel();

  // Keep track of necessary keywords that are missing from the ISD.
  std::vector<std::string> missingKeywords;

  sensorModel->m_startingDetectorSample =
      atof(imageSupportData.param("starting_detector_sample").c_str());
  sensorModel->m_startingDetectorLine =
      atof(imageSupportData.param("starting_detector_line").c_str());

  if (imageSupportData.param("focal_length_model") == "") {
    missingKeywords.push_back("focal_length_model");
  }
  else {
    json jayson = json::parse(imageSupportData.param("focal_length_model"));
    json focal_length = jayson.value("focal_length", json(""));
    json epsilon = jayson.value("epsilon", json(""));

    sensorModel->m_focalLength = atof(focal_length.dump().c_str());
    sensorModel->m_focalLengthEpsilon = atof(epsilon.dump().c_str());

    if (focal_length == json("")) {
      missingKeywords.push_back("focal_length_model focal_length");
    }
    if (epsilon == json("")) {
      missingKeywords.push_back("focal_length_model epsilon");
    }
  }

  if (imageSupportData.param("sensor_location") == "") {
    missingKeywords.push_back("sensor_location");
  }
  else {
    json jayson = json::parse(imageSupportData.param("sensor_location"));
    json x = jayson.value("x", json(""));
    json y = jayson.value("y", json(""));
    json z = jayson.value("z", json(""));
    json unit = jayson.value("unit", json(""));

    sensorModel->m_currentParameterValue[0] = atof(x.dump().c_str());
    sensorModel->m_currentParameterValue[1] = atof(y.dump().c_str());
    sensorModel->m_currentParameterValue[2] = atof(z.dump().c_str());

    if (x == json("")) {
      missingKeywords.push_back("sensor_location x");
    }
    if (y == json("")) {
      missingKeywords.push_back("sensor_location y");
    }
    if (z == json("")) {
      missingKeywords.push_back("sensor_location z");
    }
    if (unit == json("")) {
      missingKeywords.push_back("sensor_location unit");
    }
    else {
      unit = unit.get<std::string>();
      sensorModel->m_currentParameterValue[0] = metric_conversion(sensorModel->m_currentParameterValue[0], unit);
      sensorModel->m_currentParameterValue[1] = metric_conversion(sensorModel->m_currentParameterValue[1], unit);
      sensorModel->m_currentParameterValue[2] = metric_conversion(sensorModel->m_currentParameterValue[2], unit);
    }
  }

  if (imageSupportData.param("sensor_velocity") == "") {
    missingKeywords.push_back("sensor_velocity");
  }
  else {
    json jayson = json::parse(imageSupportData.param("sensor_velocity"));
    json x = jayson.value("x", json(""));
    json y = jayson.value("y", json(""));
    json z = jayson.value("z", json(""));

    sensorModel->m_spacecraftVelocity[0] = atof(x.dump().c_str());
    sensorModel->m_spacecraftVelocity[1] = atof(y.dump().c_str());
    sensorModel->m_spacecraftVelocity[2] = atof(z.dump().c_str());

    if (x == json("")) {
      missingKeywords.push_back("sensor_velocity x");
    }
    if (y == json("")) {
      missingKeywords.push_back("sensor_velocity y");
    }
    if (z == json("")) {
      missingKeywords.push_back("sensor_velocity z");
    }
  }

  sensorModel->m_ccdCenter[0] = atof(imageSupportData.param("ccd_center", 0).c_str());
  sensorModel->m_ccdCenter[1] = atof(imageSupportData.param("ccd_center", 1).c_str());

    if (x == json("")) {
      missingKeywords.push_back("sun_position x");
    }
    if (y == json("")) {
      missingKeywords.push_back("sun_position y");
    }
    if (z == json("")) {
      missingKeywords.push_back("sun_position z");
    }
  }

  // sun position is not strictly necessary, but is required for getIlluminationDirection.

  sensorModel->m_currentParameterValue[3] = atof(imageSupportData.param("sensor_orientation", 0).c_str());
  sensorModel->m_currentParameterValue[4] = atof(imageSupportData.param("sensor_orientation", 1).c_str());
  sensorModel->m_currentParameterValue[5] = atof(imageSupportData.param("sensor_orientation", 2).c_str());
  sensorModel->m_currentParameterValue[6] = atof(imageSupportData.param("sensor_orientation", 3).c_str());

  if (imageSupportData.param("sensor_orientation", 0) == "") {
    missingKeywords.push_back("sensor_orientation");
  }

  if (imageSupportData.param("optical_distortion") == "") {
    missingKeywords.push_back("optical_distortion");
  }
  else {
    json jayson = json::parse(imageSupportData.param("optical_distortion"));
    std::vector<double> xDistortion = jayson["x"];
    std::vector<double> yDistortion = jayson["y"];
    xDistortion.resize(10, 0.0);
    yDistortion.resize(10, 0.0);
    sensorModel->m_odtX = xDistortion;
    sensorModel->m_odtY = yDistortion;
  }

  sensorModel->m_ephemerisTime = atof(imageSupportData.param("center_ephemeris_time").c_str());
  if (imageSupportData.param("center_ephemeris_time") == "") {
    missingKeywords.push_back("center_ephemeris_time");
  }

  sensorModel->m_nLines = atoi(imageSupportData.param("image_lines").c_str());
  sensorModel->m_nSamples = atoi(imageSupportData.param("image_samples").c_str());
  if (imageSupportData.param("image_lines") == "") {
    missingKeywords.push_back("image_lines");
  }
  if (imageSupportData.param("image_samples") == "") {
    missingKeywords.push_back("image_samples");
  }

  if (imageSupportData.param("detector_center") == "") {
    missingKeywords.push_back("detector_center");
  }
  else {
    json jayson = json::parse(imageSupportData.param("detector_center"));
    json sample = jayson.value("sample", json(""));
    json line = jayson.value("line", json(""));

    sensorModel->m_ccdCenter[0] = atof(line.dump().c_str());
    sensorModel->m_ccdCenter[1] = atof(sample.dump().c_str());

    if (sample == json("")) {
      missingKeywords.push_back("detector_center x");
    }
    if (line == json("")) {
      missingKeywords.push_back("detector_center y");
    }
  }

  sensorModel->m_iTransL[0] = atof(imageSupportData.param("focal2pixel_lines", 0).c_str());
  sensorModel->m_iTransL[1] = atof(imageSupportData.param("focal2pixel_lines", 1).c_str());
  sensorModel->m_iTransL[2] = atof(imageSupportData.param("focal2pixel_lines", 2).c_str());


  if (imageSupportData.param("focal2pixel_lines", 0) == "") {
    missingKeywords.push_back("focal2pixel_lines 0");
  }
  else if (imageSupportData.param("focal2pixel_lines", 1) == "") {
    missingKeywords.push_back("focal2pixel_lines 1");
  }
  else if (imageSupportData.param("focal2pixel_lines", 2) == "") {
    missingKeywords.push_back("focal2pixel_lines 2");
  }

  sensorModel->m_iTransS[0] = atof(imageSupportData.param("focal2pixel_samples", 0).c_str());
  sensorModel->m_iTransS[1] = atof(imageSupportData.param("focal2pixel_samples", 1).c_str());
  sensorModel->m_iTransS[2] = atof(imageSupportData.param("focal2pixel_samples", 2).c_str());
  if (imageSupportData.param("focal2pixel_samples", 0) == "") {
    missingKeywords.push_back("focal2pixel_samples 0");
  }
  else if (imageSupportData.param("focal2pixel_samples", 1) == "") {
    missingKeywords.push_back("focal2pixel_samples 1");
  }
  else if (imageSupportData.param("focal2pixel_samples", 2) == "") {
    missingKeywords.push_back("focal2pixel_samples 2");
  }

  // We don't pass the pixel to focal plane transformation so invert the
  // focal plane to pixel transformation
  double determinant = sensorModel->m_iTransL[1] * sensorModel->m_iTransS[2] -
                       sensorModel->m_iTransL[2] * sensorModel->m_iTransS[1];

  sensorModel->m_transX[1] = sensorModel->m_iTransL[1] / determinant;
  sensorModel->m_transX[2] = - sensorModel->m_iTransS[1] / determinant;
  sensorModel->m_transX[0] = - (sensorModel->m_transX[1] * sensorModel->m_iTransL[0] +
                             sensorModel->m_transX[2] * sensorModel->m_iTransS[0]);

  sensorModel->m_transY[1] = - sensorModel->m_iTransL[2] / determinant;
  sensorModel->m_transY[2] = sensorModel->m_iTransS[2] / determinant;
  sensorModel->m_transY[0] = - (sensorModel->m_transY[1] * sensorModel->m_iTransL[0] +
                             sensorModel->m_transY[2] * sensorModel->m_iTransS[0]);

  if (imageSupportData.param("radii") == "") {
    missingKeywords.push_back("radii");
  }
  else {
    json jayson = json::parse(imageSupportData.param("radii"));
    json semiminor = jayson.value("semiminor", json(""));
    json semimajor = jayson.value("semimajor", json(""));
    json unit = jayson.value("unit", json(""));

    sensorModel->m_minorAxis = atof(semiminor.dump().c_str());
    sensorModel->m_majorAxis = atof(semimajor.dump().c_str());

    if (semiminor == json("")) {
      missingKeywords.push_back("radii semiminor");
    }
    if (semimajor == json("")) {
      missingKeywords.push_back("radii semimajor");
    }
    if (unit == json("")) {
      missingKeywords.push_back("radii unit");
    }
    else {
      unit = unit.get<std::string>();
      sensorModel->m_minorAxis = metric_conversion(sensorModel->m_minorAxis, unit);
      sensorModel->m_majorAxis = metric_conversion(sensorModel->m_majorAxis, unit);
    }
  }

  if (imageSupportData.param("reference_height") == "") {
    missingKeywords.push_back("reference_height");
  }
  else {
    json reference_height = json::parse(imageSupportData.param("reference_height"));
    json maxheight = reference_height.value("maxheight", json(""));
    json minheight = reference_height.value("minheight", json(""));
    json unit = reference_height.value("unit", json(""));

    sensorModel->m_minElevation = atof(minheight.dump().c_str());
    sensorModel->m_maxElevation = atof(maxheight.dump().c_str());

    if (maxheight == json("")) {
      missingKeywords.push_back("reference_height maxheight");
    }
    if (minheight == json("")) {
      missingKeywords.push_back("reference_height minheight");
    }
    if (unit == json("")) {
      missingKeywords.push_back("reference_height unit");
    }
    else {
      unit = unit.get<std::string>();
      sensorModel->m_minElevation = metric_conversion(sensorModel->m_minElevation, unit);
      sensorModel->m_maxElevation = metric_conversion(sensorModel->m_maxElevation, unit);
    }
  }

  // If we are missing necessary keywords from ISD, we cannot create a valid sensor model.
  if (missingKeywords.size() != 0) {

    std::string errorMessage = "ISD is missing the necessary keywords: [";

    for (int i = 0; i < (int)missingKeywords.size(); i++) {
      if (i == (int)missingKeywords.size() - 1) {
        errorMessage += missingKeywords[i] + "]";
      }
      else {
        errorMessage += missingKeywords[i] + ", ";
      }
    }

    throw csm::Error(csm::Error::SENSOR_MODEL_NOT_CONSTRUCTIBLE,
                     errorMessage,
                     "UsgsAstroFramePlugin::constructModelFromISD");
  }

  return sensorModel;
}


std::string UsgsAstroFramePlugin::getModelNameFromModelState(const std::string &modelState,
                                                   csm::WarningList *warnings) const {
  std::string name;
  auto state = json::parse(modelState);
  if(state.find("model_name") != state.end()){
      name = state["model_name"];
  }
  else{
      csm::Error::ErrorType aErrorType = csm::Error::INVALID_SENSOR_MODEL_STATE;
      std::string aMessage = "No 'model_name' key in the model state object.";
      std::string aFunction = "UsgsAstroFramePlugin::getModelNameFromModelState";
      csm::Error csmErr(aErrorType, aMessage, aFunction);
      throw(csmErr);
  }

  if (name != UsgsAstroFrameSensorModel::_SENSOR_MODEL_NAME){
      csm::Error::ErrorType aErrorType = csm::Error::SENSOR_MODEL_NOT_SUPPORTED;
      std::string aMessage = "Sensor model not supported.";
      std::string aFunction = "UsgsAstroFramePlugin::getModelNameFromModelState()";
      csm::Error csmErr(aErrorType, aMessage, aFunction);
      throw(csmErr);
  }


  return UsgsAstroFrameSensorModel::_SENSOR_MODEL_NAME;
}


bool UsgsAstroFramePlugin::canISDBeConvertedToModelState(const csm::Isd &imageSupportData,
                                               const std::string &modelName,
                                               csm::WarningList *warnings) const {
  bool convertible = true;
  if (modelName !=UsgsAstroFrameSensorModel::_SENSOR_MODEL_NAME){
      convertible = false;
  }

  csm::Isd localImageSupportData;
  try {
    localImageSupportData = loadImageSupportData(imageSupportData);
  }
  catch (...) {
     return false;
  }

  std::string value;
  for(auto &key : _ISD_KEYWORD){
      value = localImageSupportData.param(key);
      if (value.empty()){
          convertible = false;
      }
  }
  return convertible;
}


std::string UsgsAstroFramePlugin::convertISDToModelState(const csm::Isd &imageSupportData,
                                               const std::string &modelName,
                                               csm::WarningList *warnings) const {
  csm::Model* sensor_model = constructModelFromISD(
                             imageSupportData, modelName);

  if (sensor_model == 0){
      csm::Error::ErrorType aErrorType = csm::Error::ISD_NOT_SUPPORTED;
      std::string aMessage = "ISD not supported: ";
      std::string aFunction = "UsgsAstroFramePlugin::convertISDToModelState()";
      throw csm::Error(aErrorType, aMessage, aFunction);
  }
  return sensor_model->getModelState();
}
