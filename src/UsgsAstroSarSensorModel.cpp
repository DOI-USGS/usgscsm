#include "UsgsAstroSarSensorModel.h"
#include "Utilities.h"

#include <functional>
#include <string.h>
#include <cmath>

#include <json/json.hpp>

using json = nlohmann::json;
using namespace std;

#define MESSAGE_LOG(logger, ...) if (logger) { logger->info(__VA_ARGS__); }

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
  state["m_endingEphemerisTime"] = getEndingTime(isd, parsingWarnings);

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
  state["m_wavelength"] = getWavelength(isd, parsingWarnings);

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
    std::string message = "ISD is invalid for creating the sensor model with error [";
    csm::Warning warn = parsingWarnings->front();
    message += warn.getMessage();
    message += "]";
    parsingWarnings = nullptr;
    delete parsingWarnings;
    throw csm::Error(csm::Error::SENSOR_MODEL_NOT_CONSTRUCTIBLE,
                     message,
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
   string model_name;

   if (j.find("m_modelName") != j.end()) {
       model_name = j["m_modelName"];
   } else {
       csm::Error::ErrorType aErrorType = csm::Error::INVALID_SENSOR_MODEL_STATE;
       string aMessage = "No 'm_modelName' key in the model state object.";
       string aFunction = "UsgsAstroSarSensorModel::getModelNameFromModelState";
       csm::Error csmErr(aErrorType, aMessage, aFunction);
       throw(csmErr);
   }
   if (model_name != _SENSOR_MODEL_NAME){
       csm::Error::ErrorType aErrorType = csm::Error::SENSOR_MODEL_NOT_SUPPORTED;
       string aMessage = "Sensor model not supported.";
       string aFunction = "UsgsAstroSarSensorModel::getModelNameFromModelState()";
       csm::Error csmErr(aErrorType, aMessage, aFunction);
       throw(csmErr);
   }
   return model_name;

}

UsgsAstroSarSensorModel::UsgsAstroSarSensorModel()
{
  reset();
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
  m_endingEphemerisTime = 0;
  m_majorAxis = 0;
  m_minorAxis = 0;
  m_platformIdentifier = "Unknown";
  m_sensorIdentifier = "Unknown";
  m_trajectoryIdentifier = "Unknown";
  m_collectionIdentifier = "Unknown";
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
  m_wavelength = 0;
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
  m_wavelength = stateJson["m_wavelength"];
  m_startingEphemerisTime = stateJson["m_startingEphemerisTime"];
  m_centerEphemerisTime = stateJson["m_centerEphemerisTime"];
  m_endingEphemerisTime = stateJson["m_endingEphemerisTime"];
  m_majorAxis = stateJson["m_majorAxis"];
  m_minorAxis = stateJson["m_minorAxis"];
  m_platformIdentifier = stateJson["m_platformIdentifier"].get<string>();
  m_sensorIdentifier = stateJson["m_sensorIdentifier"].get<string>();
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
  m_sunPosition = stateJson["m_sunPosition"].get<vector<double>>();
  m_sunVelocity = stateJson["m_sunVelocity"].get<vector<double>>();
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
  state["m_endingEphemerisTime"] = m_endingEphemerisTime;
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
  state["m_wavelength"] = m_wavelength;
  state["m_scaleConversionCoefficients"] = m_scaleConversionCoefficients;
  state["m_covariance"] = m_covariance;

  return state.dump();
}


csm::ImageCoord UsgsAstroSarSensorModel::groundToImage(
    const csm::EcefCoord& groundPt,
    double desiredPrecision,
    double* achievedPrecision,
    csm::WarningList* warnings) const {

  //MESSAGE_LOG(this->m_logger, "Computing groundToImage(ImageCoord) for {}, {}, {}, with desired precision {}",
  //          groundPt.x, groundPt.y, groundPt.z, desiredPrecision);

  // Find time of closest approach to groundPt and the corresponding slant range by finding
  // the root of the doppler shift frequency
  try {
    double timeTolerance = m_exposureDuration * desiredPrecision / 2.0;
    double time = dopplerShift(groundPt, timeTolerance);
    double slantRangeValue = slantRange(groundPt, time);

    // Find the ground range, based on the ground-range-to-slant-range polynomial defined by the
    // range coefficient set, with a time closest to the calculated time of closest approach
    double groundTolerance = m_scaledPixelWidth * desiredPrecision / 2.0;
    double groundRange = slantRangeToGroundRange(groundPt, time, slantRangeValue, groundTolerance);

    double line = (time - m_startingEphemerisTime) / m_exposureDuration;
    double sample = groundRange / m_scaledPixelWidth;
    return csm::ImageCoord(line, sample);
  } catch (std::exception& error) {
    std::string message = "Could not calculate groundToImage, with error [";
    message += error.what();
    message += "]";
    throw csm::Error(csm::Error::UNKNOWN_ERROR, message, "groundToImage");
  }
}

// Calculate the root
double UsgsAstroSarSensorModel::dopplerShift(
    csm::EcefCoord groundPt,
    double tolerance) const
{
   std::function<double(double)> dopplerShiftFunction = [this, groundPt](double time) {
     csm::EcefVector spacecraftPosition = getSpacecraftPosition(time);
     csm::EcefVector spacecraftVelocity = getSpacecraftVelocity(time);
     double lookVector[3];

     lookVector[0] = spacecraftPosition.x - groundPt.x;
     lookVector[1] = spacecraftPosition.y - groundPt.y;
     lookVector[2] = spacecraftPosition.z - groundPt.z;

     double slantRange = sqrt(pow(lookVector[0], 2) +  pow(lookVector[1], 2) + pow(lookVector[2], 2));


     double dopplerShift = -2.0 * (lookVector[0]*spacecraftVelocity.x + lookVector[1]*spacecraftVelocity.y
                            + lookVector[2]*spacecraftVelocity.z)/(slantRange * m_wavelength);

     return dopplerShift;
   };

  // Do root-finding for "dopplerShift"
  return brentRoot(m_startingEphemerisTime, m_endingEphemerisTime, dopplerShiftFunction, tolerance);
}


double UsgsAstroSarSensorModel::slantRange(csm::EcefCoord surfPt,
    double time) const
{
  csm::EcefVector spacecraftPosition = getSpacecraftPosition(time);
  double lookVector[3];

  lookVector[0] = spacecraftPosition.x - surfPt.x;
  lookVector[1] = spacecraftPosition.y - surfPt.y;
  lookVector[2] = spacecraftPosition.z - surfPt.z;
  return sqrt(pow(lookVector[0], 2) +  pow(lookVector[1], 2) + pow(lookVector[2], 2));
}

double UsgsAstroSarSensorModel::slantRangeToGroundRange(
    const csm::EcefCoord& groundPt,
    double time,
    double slantRange,
    double tolerance) const
{
  std::vector<double> coeffs = getRangeCoefficients(time);

  // Calculates the ground range from the slant range.
  std::function<double(double)> slantRangeToGroundRangeFunction =
    [coeffs, slantRange](double groundRange){
   return slantRange - (coeffs[0] + groundRange * (coeffs[1] + groundRange * (coeffs[2] + groundRange * coeffs[3])));
  };

  // Need to come up with an initial guess when solving for ground
  // range given slant range. Compute the ground range at the
  // near and far edges of the image by evaluating the sample-to-
  // ground-range equation: groundRange=(sample-1)*scaled_pixel_width
  // at the edges of the image. We also need to add some padding to
  // allow for solving for coordinates that are slightly outside of
  // the actual image area. Use sample=-0.25*image_samples and
  // sample=1.25*image_samples.
  double minGroundRangeGuess = (-0.25 * m_nSamples - 1.0) * m_scaledPixelWidth;
  double maxGroundRangeGuess = (1.25 * m_nSamples - 1.0) * m_scaledPixelWidth;

  // Tolerance to 1/20th of a pixel for now.
  return brentRoot(minGroundRangeGuess, maxGroundRangeGuess, slantRangeToGroundRangeFunction, tolerance);
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

pair<csm::ImageCoord, csm::ImageCoord> UsgsAstroSarSensorModel::getValidImageRange() const
{
  return make_pair(csm::ImageCoord(0.0, 0.0), csm::ImageCoord(0.0, 0.0));
}

pair<double, double> UsgsAstroSarSensorModel::getValidHeightRange() const
{
  return make_pair(0.0, 0.0);
}

csm::EcefVector UsgsAstroSarSensorModel::getIlluminationDirection(const csm::EcefCoord& groundPt) const
{
  return csm::EcefVector(0.0, 0.0, 0.0);
}

double UsgsAstroSarSensorModel::getImageTime(const csm::ImageCoord& imagePt) const
{
  return 0.0;
}

csm::EcefCoord UsgsAstroSarSensorModel::getSensorPosition(const csm::ImageCoord& imagePt) const
{
  return csm::EcefCoord(0.0, 0.0, 0.0);
}

csm::EcefCoord UsgsAstroSarSensorModel::getSensorPosition(double time) const
{
  return csm::EcefCoord(0.0, 0.0, 0.0);
}

csm::EcefVector UsgsAstroSarSensorModel::getSensorVelocity(const csm::ImageCoord& imagePt) const
{
  return csm::EcefVector(0.0, 0.0, 0.0);
}

csm::EcefVector UsgsAstroSarSensorModel::getSensorVelocity(double time) const
{
  return csm::EcefVector(0.0, 0.0, 0.0);
}

csm::RasterGM::SensorPartials UsgsAstroSarSensorModel::computeSensorPartials(
    int index,
    const csm::EcefCoord& groundPt,
    double desiredPrecision,
    double* achievedPrecision,
    csm::WarningList* warnings) const
{
  return csm::RasterGM::SensorPartials(0.0, 0.0);
}

csm::RasterGM::SensorPartials UsgsAstroSarSensorModel::computeSensorPartials(
    int index,
    const csm::ImageCoord& imagePt,
    const csm::EcefCoord& groundPt,
    double desiredPrecision,
    double* achievedPrecision,
    csm::WarningList* warnings) const
{
  return csm::RasterGM::SensorPartials(0.0, 0.0);
}

vector<csm::RasterGM::SensorPartials> UsgsAstroSarSensorModel::computeAllSensorPartials(
    const csm::EcefCoord& groundPt,
    csm::param::Set pSet,
    double desiredPrecision,
    double* achievedPrecision,
    csm::WarningList* warnings) const
{
  return vector<csm::RasterGM::SensorPartials>(NUM_PARAMETERS, csm::RasterGM::SensorPartials(0.0, 0.0));
}

vector<double> UsgsAstroSarSensorModel::computeGroundPartials(const csm::EcefCoord& groundPt) const
{
  return vector<double>(6, 0.0);
}

const csm::CorrelationModel& UsgsAstroSarSensorModel::getCorrelationModel() const
{
  return _NO_CORR_MODEL;
}

vector<double> UsgsAstroSarSensorModel::getUnmodeledCrossCovariance(
    const csm::ImageCoord& pt1,
    const csm::ImageCoord& pt2) const
{
  return vector<double>(4, 0.0);
}

csm::EcefCoord UsgsAstroSarSensorModel::getReferencePoint() const
{
  return m_referencePointXyz;
}

void UsgsAstroSarSensorModel::setReferencePoint(const csm::EcefCoord& groundPt)
{
  m_referencePointXyz = groundPt;
}

int UsgsAstroSarSensorModel::getNumParameters() const
{
  return NUM_PARAMETERS;
}

string UsgsAstroSarSensorModel::getParameterName(int index) const
{
  return PARAMETER_NAME[index];
}

string UsgsAstroSarSensorModel::getParameterUnits(int index) const
{
  return "m";
}

bool UsgsAstroSarSensorModel::hasShareableParameters() const
{
  return false;
}

bool UsgsAstroSarSensorModel::isParameterShareable(int index) const
{
  return false;
}

csm::SharingCriteria UsgsAstroSarSensorModel::getParameterSharingCriteria(int index) const
{
  return csm::SharingCriteria();
}

double UsgsAstroSarSensorModel::getParameterValue(int index) const
{
  return m_currentParameterValue[index];
}

void UsgsAstroSarSensorModel::setParameterValue(int index, double value)
{
  m_currentParameterValue[index] = value;
}

csm::param::Type UsgsAstroSarSensorModel::getParameterType(int index) const
{
  return m_parameterType[index];
}

void UsgsAstroSarSensorModel::setParameterType(int index, csm::param::Type pType)
{
  m_parameterType[index] = pType;
}

double UsgsAstroSarSensorModel::getParameterCovariance(
    int index1,
    int index2) const
{
  return m_covariance[index1 * NUM_PARAMETERS + index2];
}


void UsgsAstroSarSensorModel::setParameterCovariance(
    int index1,
    int index2,
    double covariance)

{
  m_covariance[index1 * NUM_PARAMETERS + index2] = covariance;
}

int UsgsAstroSarSensorModel::getNumGeometricCorrectionSwitches() const
{
  return 0;
}

string UsgsAstroSarSensorModel::getGeometricCorrectionName(int index) const
{
  throw csm::Error(
     csm::Error::INDEX_OUT_OF_RANGE,
     "Index is out of range.",
     "UsgsAstroSarSensorModel::getGeometricCorrectionName");
}

void UsgsAstroSarSensorModel::setGeometricCorrectionSwitch(int index,
    bool value,
    csm::param::Type pType)
{
  throw csm::Error(
     csm::Error::INDEX_OUT_OF_RANGE,
     "Index is out of range.",
     "UsgsAstroSarSensorModel::setGeometricCorrectionSwitch");
}

bool UsgsAstroSarSensorModel::getGeometricCorrectionSwitch(int index) const
{
  throw csm::Error(
     csm::Error::INDEX_OUT_OF_RANGE,
     "Index is out of range.",
     "UsgsAstroSarSensorModel::getGeometricCorrectionSwitch");
}

vector<double> UsgsAstroSarSensorModel::getCrossCovarianceMatrix(
    const csm::GeometricModel& comparisonModel,
    csm::param::Set pSet,
    const csm::GeometricModel::GeometricModelList& otherModels) const
{
  // Return covariance matrix
  if (&comparisonModel == this) {
    vector<int> paramIndices = getParameterSetIndices(pSet);
    int numParams = paramIndices.size();
    vector<double> covariances(numParams * numParams, 0.0);
    for (int i = 0; i < numParams; i++) {
      for (int j = 0; j < numParams; j++) {
        covariances[i * numParams + j] = getParameterCovariance(paramIndices[i], paramIndices[j]);
      }
    }
    return covariances;
  }
  // No correlation between models.
  const vector<int>& indices = getParameterSetIndices(pSet);
  size_t num_rows = indices.size();
  const vector<int>& indices2 = comparisonModel.getParameterSetIndices(pSet);
  size_t num_cols = indices.size();

  return vector<double>(num_rows * num_cols, 0.0);
}

csm::Version UsgsAstroSarSensorModel::getVersion() const
{
  return csm::Version(1, 0, 0);
}

string UsgsAstroSarSensorModel::getModelName() const
{
  return _SENSOR_MODEL_NAME;
}

string UsgsAstroSarSensorModel::getPedigree() const
{
  return "USGS_SAR";
}

string UsgsAstroSarSensorModel::getImageIdentifier() const
{
  return m_imageIdentifier;
}

void UsgsAstroSarSensorModel::setImageIdentifier(
    const string& imageId,
    csm::WarningList* warnings)
{
  m_imageIdentifier = imageId;
}

string UsgsAstroSarSensorModel::getSensorIdentifier() const
{
  return m_sensorIdentifier;
}

string UsgsAstroSarSensorModel::getPlatformIdentifier() const
{
  return m_platformIdentifier;
}

string UsgsAstroSarSensorModel::getCollectionIdentifier() const
{
  return m_collectionIdentifier;
}

string UsgsAstroSarSensorModel::getTrajectoryIdentifier() const
{
  return m_trajectoryIdentifier;
}

string UsgsAstroSarSensorModel::getSensorType() const
{
  return "SAR";
}

string UsgsAstroSarSensorModel::getSensorMode() const
{
  return "STRIP";
}

string UsgsAstroSarSensorModel::getReferenceDateAndTime() const
{
  csm::ImageCoord referencePointImage = groundToImage(m_referencePointXyz);
  double relativeTime = getImageTime(referencePointImage);
  time_t ephemTime = m_centerEphemerisTime + relativeTime;
  struct tm t = {0};  // Initalize to all 0's
  t.tm_year = 100;  // This is year-1900, so 100 = 2000
  t.tm_mday = 1;
  time_t timeSinceEpoch = mktime(&t);
  time_t finalTime = ephemTime + timeSinceEpoch;
  char buffer [16];
  strftime(buffer, 16, "%Y%m%dT%H%M%S", localtime(&finalTime));
  buffer[15] = '\0';

  return buffer;
}

csm::Ellipsoid UsgsAstroSarSensorModel::getEllipsoid() const
{
   return csm::Ellipsoid(m_majorAxis, m_minorAxis);
}

void UsgsAstroSarSensorModel::setEllipsoid(const csm::Ellipsoid &ellipsoid)
{
   m_majorAxis = ellipsoid.getSemiMajorRadius();
   m_minorAxis = ellipsoid.getSemiMinorRadius();
}

csm::EcefVector UsgsAstroSarSensorModel::getSpacecraftPosition(double time) const{
  int numPositions = m_positions.size();
  csm::EcefVector spacecraftPosition = csm::EcefVector();

  // If there are multiple positions, use Lagrange interpolation
  if ((numPositions/3) > 1) {
    double position[3];
    lagrangeInterp(numPositions/3, &m_positions[0], m_t0Ephem, m_dtEphem,
                   time, 3, 8, position);
    spacecraftPosition.x = position[0];
    spacecraftPosition.y = position[1];
    spacecraftPosition.z = position[2];
  }
  else {
    spacecraftPosition.x = m_positions[0];
    spacecraftPosition.y = m_positions[1];
    spacecraftPosition.z = m_positions[2];
  }
  // Can add third case if need be, but seems unlikely to come up for SAR
  return spacecraftPosition;
}

csm::EcefVector UsgsAstroSarSensorModel::getSpacecraftVelocity(double time) const {
  int numVelocities = m_velocities.size();
  csm::EcefVector spacecraftVelocity = csm::EcefVector();

  // If there are multiple positions, use Lagrange interpolation
  if ((numVelocities/3) > 1) {
    double velocity[3];
    lagrangeInterp(numVelocities/3, &m_velocities[0], m_t0Ephem, m_dtEphem,
                   time, 3, 8, velocity);
    spacecraftVelocity.x = velocity[0];
    spacecraftVelocity.y = velocity[1];
    spacecraftVelocity.z = velocity[2];
  }
  else {
    spacecraftVelocity.x = m_velocities[0];
    spacecraftVelocity.y = m_velocities[1];
    spacecraftVelocity.z = m_velocities[2];
  }
  return spacecraftVelocity;
}


std::vector<double> UsgsAstroSarSensorModel::getRangeCoefficients(double time) const {
  int numCoeffs = m_scaleConversionCoefficients.size();
  std::vector<double> coeffs, interpCoeffs;

  double endTime = m_scaleConversionCoefficients[numCoeffs - 5];

  // The structure of the input is [time0 a0 a1 a2 a3 time1 a0 a1 a2 a3....]
  for (int i=0; i < numCoeffs; i++) {
    if (i%5 != 0) {
      coeffs.push_back(m_scaleConversionCoefficients[i]);
    }
  }

  if ((numCoeffs/5) > 1) {
    double coefficients[4];

    double dtEphem = (endTime - m_scaleConversionCoefficients[0]) / (coeffs.size()/4);
    lagrangeInterp(coeffs.size()/4, &coeffs[0], m_scaleConversionCoefficients[0], dtEphem,
                   time, 4, 8, coefficients);
    interpCoeffs.push_back(coefficients[0]);
    interpCoeffs.push_back(coefficients[1]);
    interpCoeffs.push_back(coefficients[2]);
    interpCoeffs.push_back(coefficients[3]);
  }
  else {
    interpCoeffs.push_back(m_scaleConversionCoefficients[1]);
    interpCoeffs.push_back(m_scaleConversionCoefficients[2]);
    interpCoeffs.push_back(m_scaleConversionCoefficients[3]);
    interpCoeffs.push_back(m_scaleConversionCoefficients[4]);
  }
  return coeffs;
}
