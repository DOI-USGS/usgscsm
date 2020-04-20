#include "UsgsAstroSarSensorModel.h"
#include "Utilities.h"

#include <string.h>

#include <json/json.hpp>

using json = nlohmann::json;
using namespace std;

const string UsgsAstroSarSensorModel::_SENSOR_MODEL_NAME = "USGS_ASTRO_SAR_SENSOR_MODEL";
const int UsgsAstroSarSensorModel::NUM_PARAMETERS = 6;
const string UsgsAstroSarSensorModel::PARAMETER_NAME[] =
{
   "IT Pos. Bias   ",   // 0
   "CT Pos. Bias   ",   // 1
   "Rad Pos. Bias  ",   // 2
   "X Vel. Bias    ",   // 3
   "Y Vel. Bias    ",   // 4
   "Z Vel. Bias    "    // 5
};

const int UsgsAstroSarSensorModel::NUM_PARAM_TYPES = 4;
const string UsgsAstroSarSensorModel::PARAM_STRING_ALL[] =
{
   "NONE",
   "FICTITIOUS",
   "REAL",
   "FIXED"
};
const csm::param::Type
      UsgsAstroSarSensorModel::PARAM_CHAR_ALL[] =
{
   csm::param::NONE,
   csm::param::FICTITIOUS,
   csm::param::REAL,
   csm::param::FIXED
};

string UsgsAstroSarSensorModel::constructStateFromIsd(
    const string imageSupportData,
    csm::WarningList *warnings)
{
  json isd = json::parse(imageSupportData);
  json state = {};

  csm::WarningList* parsingWarnings = new csm::WarningList;

  state["m_modelName"] = getSensorModelName(isd, parsingWarnings);
  state["m_imageIdentifier"] = getImageId(isd, parsingWarnings);
  state["m_sensorName"] = getSensorName(isd, parsingWarnings);
  state["m_platformName"] = getPlatformName(isd, parsingWarnings);

  state["m_nLines"] = getTotalLines(isd, parsingWarnings);
  state["m_nSamples"] = getTotalSamples(isd, parsingWarnings);

  // Zero computed state values
  state["m_referencePointXyz"] = vector<double>(3, 0.0);

  // sun_position and velocity are required for getIlluminationDirection
  state["m_sunPosition"] = getSunPositions(isd, parsingWarnings);
  state["m_sunVelocity"] = getSunVelocities(isd, parsingWarnings);

  state["m_centerEphemerisTime"] = getCenterTime(isd, parsingWarnings);
  state["m_startingEphemerisTime"] = getStartingTime(isd, parsingWarnings);

  state["m_exposureDuration"] = getExposureDuration(isd, parsingWarnings);

  try {
    state["m_dtEphem"] = isd.at("dt_ephemeris");
  }
  catch(...) {
    parsingWarnings->push_back(
      csm::Warning(
        csm::Warning::DATA_NOT_AVAILABLE,
        "dt_ephemeris not in ISD",
        "UsgsAstroSarSensorModel::constructStateFromIsd()"));
  }

  try {
    state["m_t0Ephem"] = isd.at("t0_ephemeris");
  }
  catch(...) {
    parsingWarnings->push_back(
      csm::Warning(
        csm::Warning::DATA_NOT_AVAILABLE,
        "t0_ephemeris not in ISD",
        "UsgsAstroSarSensorModel::constructStateFromIsd()"));
  }

  state["m_positions"] = getSensorPositions(isd, parsingWarnings);
  state["m_velocities"] = getSensorVelocities(isd, parsingWarnings);

  state["m_currentParameterValue"] = vector<double>(NUM_PARAMETERS, 0.0);

  // get radii
  state["m_minorAxis"] = getSemiMinorRadius(isd, parsingWarnings);
  state["m_majorAxis"] = getSemiMajorRadius(isd, parsingWarnings);

  // set identifiers
  state["m_platformIdentifier"] = getPlatformName(isd, parsingWarnings);
  state["m_sensorIdentifier"] = getSensorName(isd, parsingWarnings);

  // get reference_height
  state["m_minElevation"] = -1000;
  state["m_maxElevation"] = 1000;

  // SAR specific values
  state["m_scaledPixelWidth"] = getScaledPixelWidth(isd, parsingWarnings);
  state["m_scaleConversionCoefficients"] = getScaleConversionCoefficients(isd, parsingWarnings);

  // Default to identity covariance
  state["m_covariance"] =
       vector<double>(NUM_PARAMETERS * NUM_PARAMETERS, 0.0);
  for (int i = 0; i < NUM_PARAMETERS; i++) {
   state["m_covariance"][i * NUM_PARAMETERS + i] = 1.0;
  }

  if (!parsingWarnings->empty()) {
    if (warnings) {
      warnings->insert(warnings->end(), parsingWarnings->begin(), parsingWarnings->end());
    }
    delete parsingWarnings;
    parsingWarnings = nullptr;
    throw csm::Error(csm::Error::SENSOR_MODEL_NOT_CONSTRUCTIBLE,
                     "ISD is invalid for creating the sensor model.",
                     "UsgsAstroSarSensorModel::constructStateFromIsd");
  }

  delete parsingWarnings;
  parsingWarnings = nullptr;

  // The state data will still be updated when a sensor model is created since
  // some state data is not in the ISD and requires a SM to compute them.
  return state.dump();
}

string UsgsAstroSarSensorModel::getModelNameFromModelState(const string& model_state)
{
   // Parse the string to JSON
   auto j = json::parse(model_state);
   // If model name cannot be determined, return a blank string
   std::string model_name;

   if (j.find("m_modelName") != j.end()) {
       model_name = j["m_modelName"];
   } else {
       csm::Error::ErrorType aErrorType = csm::Error::INVALID_SENSOR_MODEL_STATE;
       std::string aMessage = "No 'm_modelName' key in the model state object.";
       std::string aFunction = "UsgsAstroSarSensorModel::getModelNameFromModelState";
       csm::Error csmErr(aErrorType, aMessage, aFunction);
       throw(csmErr);
   }
   if (model_name != _SENSOR_MODEL_NAME){
       csm::Error::ErrorType aErrorType = csm::Error::SENSOR_MODEL_NOT_SUPPORTED;
       std::string aMessage = "Sensor model not supported.";
       std::string aFunction = "UsgsAstroSarSensorModel::getModelNameFromModelState()";
       csm::Error csmErr(aErrorType, aMessage, aFunction);
       throw(csmErr);
   }
   return model_name;

}

void UsgsAstroSarSensorModel::reset()
{
  m_imageIdentifier = "Unknown";
  m_sensorName = "Unknown";
  m_platformIdentifier = "Unknown";
  m_nLines = 0;
  m_nSamples = 0;
  m_exposureDuration = 0;
  m_scaledPixelWidth = 0;
  m_startingEphemerisTime = 0;
  m_centerEphemerisTime = 0;
  m_majorAxis = 0;
  m_minorAxis = 0;
  m_referenceDateAndTime = "Unknown";
  m_platformIdentifier = "Unknown";
  m_sensorIdentifier = "Unknown";
  m_trajectoryIdentifier = "Unknown";
  m_collectionIdentifier = "Unknown";
  m_refElevation = 0;
  m_minElevation = -1000;
  m_maxElevation = 1000;
  m_dtEphem = 0;
  m_t0Ephem = 0;
  m_scaleConversionCoefficients.clear();
  m_positions.clear();
  m_velocities.clear();
  m_currentParameterValue = vector<double>(NUM_PARAMETERS, 0.0);
  m_parameterType = vector<csm::param::Type>(NUM_PARAMETERS, csm::param::REAL);
  m_referencePointXyz.x = 0.0;
  m_referencePointXyz.y = 0.0;
  m_referencePointXyz.z = 0.0;
  m_covariance = vector<double>(NUM_PARAMETERS * NUM_PARAMETERS,0.0);
  m_sunPosition.clear();
  m_sunVelocity.clear();
}

void UsgsAstroSarSensorModel::replaceModelState(const string& argState)
{
  reset();

  auto stateJson = json::parse(argState);

  m_imageIdentifier = stateJson["m_imageIdentifier"].get<string>();
  m_platformIdentifier = stateJson["m_platformIdentifier"].get<string>();
  m_sensorName = stateJson["m_sensorName"].get<string>();
  m_nLines = stateJson["m_nLines"];
  m_nSamples = stateJson["m_nSamples"];
  m_exposureDuration = stateJson["m_exposureDuration"];
  m_scaledPixelWidth = stateJson["m_scaledPixelWidth"];
  m_startingEphemerisTime = stateJson["m_startingEphemerisTime"];
  m_centerEphemerisTime = stateJson["m_centerEphemerisTime"];
  m_majorAxis = stateJson["m_majorAxis"];
  m_minorAxis = stateJson["m_minorAxis"];
  m_referenceDateAndTime = stateJson["m_referenceDateAndTime"].get<string>();
  m_platformIdentifier = stateJson["m_platformIdentifier"].get<string>();
  m_sensorIdentifier = stateJson["m_sensorIdentifier"].get<string>();
  m_trajectoryIdentifier = stateJson["m_trajectoryIdentifier"].get<string>();
  m_collectionIdentifier = stateJson["m_collectionIdentifier"].get<string>();
  m_refElevation = stateJson["m_refElevation"];
  m_minElevation = stateJson["m_minElevation"];
  m_maxElevation = stateJson["m_maxElevation"];
  m_dtEphem = stateJson["m_dtEphem"];
  m_t0Ephem = stateJson["m_t0Ephem"];
  m_scaleConversionCoefficients = stateJson["m_scaleConversionCoefficients"].get<vector<double>>();
  m_positions = stateJson["m_positions"].get<vector<double>>();
  m_velocities = stateJson["m_velocities"].get<vector<double>>();
  m_currentParameterValue = stateJson["m_currentParameterValue"].get<vector<double>>();
  m_referencePointXyz.x = stateJson["m_referencePointXyz"][0];
  m_referencePointXyz.y = stateJson["m_referencePointXyz"][1];
  m_referencePointXyz.z = stateJson["m_referencePointXyz"][2];
  m_covariance = stateJson["m_covariance"].get<vector<double>>();
  m_sunPosition = stateJson["m_sunPosition"].get<vector<double>>();;
  m_sunVelocity = stateJson["m_sunVelocity"].get<vector<double>>();;
}

string UsgsAstroSarSensorModel::getModelState() const
{
  json state = {};

  state["m_modelName"] = _SENSOR_MODEL_NAME;
  state["m_imageIdentifier"] = m_imageIdentifier;
  state["m_sensorName"] = m_sensorName;
  state["m_platformName"] = m_platformName;
  state["m_nLines"] = m_nLines;
  state["m_nSamples"] = m_nSamples;
  state["m_referencePointXyz"] = {
      m_referencePointXyz.x,
      m_referencePointXyz.y,
      m_referencePointXyz.z
  };
  state["m_sunPosition"] = m_sunPosition;
  state["m_sunVelocity"] = m_sunVelocity;
  state["m_centerEphemerisTime"] = m_centerEphemerisTime;
  state["m_startingEphemerisTime"] = m_startingEphemerisTime;
  state["m_exposureDuration"] = m_exposureDuration;
  state["m_dtEphem"] = m_dtEphem;
  state["m_t0Ephem"] = m_t0Ephem;
  state["m_positions"] = m_positions;
  state["m_velocities"] = m_velocities;
  state["m_currentParameterValue"] = m_currentParameterValue;
  state["m_minorAxis"] = m_minorAxis;
  state["m_majorAxis"] = m_majorAxis;
  state["m_platformIdentifier"] = m_platformIdentifier;
  state["m_sensorIdentifier"] = m_sensorIdentifier;
  state["m_minElevation"] = m_minElevation;
  state["m_maxElevation"] = m_maxElevation;
  state["m_scaledPixelWidth"] = m_scaledPixelWidth;
  state["m_scaleConversionCoefficients"] = m_scaleConversionCoefficients;
  state["m_covariance"] = m_covariance;

  return state.dump();
}



csm::ImageCoord UsgsAstroSarSensorModel::groundToImage(
    const csm::EcefCoord& groundPt,
    double desiredPrecision,
    double* achievedPrecision,
    csm::WarningList* warnings) const
{
  return csm::ImageCoord(0.0, 0.0);
}

csm::ImageCoordCovar UsgsAstroSarSensorModel::groundToImage(
    const csm::EcefCoordCovar& groundPt,
    double desiredPrecision,
    double* achievedPrecision,
    csm::WarningList* warnings) const
{
  return csm::ImageCoordCovar(0.0, 0.0,
                              1.0, 0.0,
                                   1.0);
}

csm::EcefCoord UsgsAstroSarSensorModel::imageToGround(
    const csm::ImageCoord& imagePt,
    double height,
    double desiredPrecision,
    double* achievedPrecision,
    csm::WarningList* warnings) const
{
  return csm::EcefCoord(0.0, 0.0, 0.0);
}

csm::EcefCoordCovar UsgsAstroSarSensorModel::imageToGround(
    const csm::ImageCoordCovar& imagePt,
    double height,
    double heightVariance,
    double desiredPrecision,
    double* achievedPrecision,
    csm::WarningList* warnings) const
{
  return csm::EcefCoordCovar(0.0, 0.0, 0.0,
                             1.0, 0.0, 0.0,
                                  1.0, 0.0,
                                       1.0);
}

csm::EcefLocus UsgsAstroSarSensorModel::imageToProximateImagingLocus(
    const csm::ImageCoord& imagePt,
    const csm::EcefCoord& groundPt,
    double desiredPrecision,
    double* achievedPrecision,
    csm::WarningList* warnings) const
{
  return csm::EcefLocus(0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0);
}

csm::EcefLocus UsgsAstroSarSensorModel::imageToRemoteImagingLocus(
    const csm::ImageCoord& imagePt,
    double desiredPrecision,
    double* achievedPrecision,
    csm::WarningList* warnings) const
{
  return csm::EcefLocus(0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0);
}

csm::ImageCoord UsgsAstroSarSensorModel::getImageStart() const
{
  return csm::ImageCoord(0.0, 0.0);
}

csm::ImageVector UsgsAstroSarSensorModel::getImageSize() const
{
  return csm::ImageVector(0.0, 0.0);
}
