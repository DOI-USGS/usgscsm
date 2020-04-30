#include "UsgsAstroSarSensorModel.h"
#include "Utilities.h"

#include <functional>
#include <iomanip>
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
  state["m_scaleConversionTimes"] = getScaleConversionTimes(isd, parsingWarnings);
  state["m_wavelength"] = getWavelength(isd, parsingWarnings);
  state["m_lookDirection"] = getLookDirection(isd, parsingWarnings);

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
  m_lookDirection = LookDirection::LEFT;
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
  m_scaleConversionTimes.clear();
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
  std::string lookStr = stateJson["m_lookDirection"];
  if (lookStr.compare("right") == 0 ) {
    m_lookDirection = UsgsAstroSarSensorModel::RIGHT;
  }
  else if (lookStr.compare("left") == 0) {
    m_lookDirection = UsgsAstroSarSensorModel::LEFT;
  }
  else {
    std::string message = "Could not determine look direction from state";
    throw csm::Error(csm::Error::INVALID_SENSOR_MODEL_STATE,
                     message,
                     "UsgsAstroSarSensorModel::replaceModelState");
  }
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
  m_scaleConversionTimes = stateJson["m_scaleConversionTimes"].get<vector<double>>();
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
  if (m_lookDirection == 0) {
    state["m_lookDirection"] = "left";
  }
  else if (m_lookDirection == 1) {
    state["m_lookDirection"] = "right";
  }
  else {
    std::string message = "Could not parse look direction from json state.";
    throw csm::Error(csm::Error::INVALID_SENSOR_MODEL_STATE,
                     message,
                     "UsgsAstroSarSensorModel::getModelState");
  }
  state["m_wavelength"] = m_wavelength;
  state["m_scaleConversionCoefficients"] = m_scaleConversionCoefficients;
  state["m_scaleConversionTimes"] = m_scaleConversionTimes;
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

    double line = (time - m_startingEphemerisTime) / m_exposureDuration + 0.5;
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
   csm::EcefVector groundVec(groundPt.x ,groundPt.y, groundPt.z);
   std::function<double(double)> dopplerShiftFunction = [this, groundVec](double time) {
     csm::EcefVector spacecraftPosition = getSpacecraftPosition(time);
     csm::EcefVector spacecraftVelocity = getSensorVelocity(time);
     csm::EcefVector lookVector = spacecraftPosition - groundVec;

     double slantRange = norm(lookVector);

     double dopplerShift = -2.0 * dot(lookVector, spacecraftVelocity) / (slantRange * m_wavelength);

     return dopplerShift;
   };

  // Do root-finding for "dopplerShift"
  return brentRoot(m_startingEphemerisTime, m_endingEphemerisTime, dopplerShiftFunction, tolerance);
}


double UsgsAstroSarSensorModel::slantRange(csm::EcefCoord surfPt,
    double time) const
{
  csm::EcefVector surfVec(surfPt.x ,surfPt.y, surfPt.z);
  csm::EcefVector spacecraftPosition = getSpacecraftPosition(time);
  return norm(spacecraftPosition - surfVec);
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
    [this, coeffs, slantRange](double groundRange){
   return slantRange - groundRangeToSlantRange(groundRange, coeffs);
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

double UsgsAstroSarSensorModel::groundRangeToSlantRange(double groundRange, const std::vector<double> &coeffs) const
{
  return coeffs[0] + groundRange * (coeffs[1] + groundRange * (coeffs[2] + groundRange * coeffs[3]));
}


csm::ImageCoordCovar UsgsAstroSarSensorModel::groundToImage(
    const csm::EcefCoordCovar& groundPt,
    double desiredPrecision,
    double* achievedPrecision,
    csm::WarningList* warnings) const
{
  // Ground to image with error propagation
  // Compute corresponding image point
  csm::EcefCoord gp(groundPt);

  csm::ImageCoord ip = groundToImage(gp, desiredPrecision, achievedPrecision, warnings);
  csm::ImageCoordCovar result(ip.line, ip.samp);

  // Compute partials ls wrt XYZ
  std::vector<double> prt(6, 0.0);
  prt = computeGroundPartials(groundPt);

  // Error propagation
  double ltx, lty, ltz;
  double stx, sty, stz;
  ltx =
     prt[0] * groundPt.covariance[0] +
     prt[1] * groundPt.covariance[3] +
     prt[2] * groundPt.covariance[6];
  lty =
     prt[0] * groundPt.covariance[1] +
     prt[1] * groundPt.covariance[4] +
     prt[2] * groundPt.covariance[7];
  ltz =
     prt[0] * groundPt.covariance[2] +
     prt[1] * groundPt.covariance[5] +
     prt[2] * groundPt.covariance[8];
  stx =
     prt[3] * groundPt.covariance[0] +
     prt[4] * groundPt.covariance[3] +
     prt[5] * groundPt.covariance[6];
  sty =
     prt[3] * groundPt.covariance[1] +
     prt[4] * groundPt.covariance[4] +
     prt[5] * groundPt.covariance[7];
  stz =
     prt[3] * groundPt.covariance[2] +
     prt[4] * groundPt.covariance[5] +
     prt[5] * groundPt.covariance[8];

  double gp_cov[4]; // Input gp cov in image space
  gp_cov[0] = ltx * prt[0] + lty * prt[1] + ltz * prt[2];
  gp_cov[1] = ltx * prt[3] + lty * prt[4] + ltz * prt[5];
  gp_cov[2] = stx * prt[0] + sty * prt[1] + stz * prt[2];
  gp_cov[3] = stx * prt[3] + sty * prt[4] + stz * prt[5];

  std::vector<double> unmodeled_cov = getUnmodeledError(ip);
  double sensor_cov[4]; // sensor cov in image space
  determineSensorCovarianceInImageSpace(gp, sensor_cov);

  result.covariance[0] = gp_cov[0] + unmodeled_cov[0] + sensor_cov[0];
  result.covariance[1] = gp_cov[1] + unmodeled_cov[1] + sensor_cov[1];
  result.covariance[2] = gp_cov[2] + unmodeled_cov[2] + sensor_cov[2];
  result.covariance[3] = gp_cov[3] + unmodeled_cov[3] + sensor_cov[3];

  return result;
}

csm::EcefCoord UsgsAstroSarSensorModel::imageToGround(
    const csm::ImageCoord& imagePt,
    double height,
    double desiredPrecision,
    double* achievedPrecision,
    csm::WarningList* warnings) const
{
  double time = m_startingEphemerisTime + (imagePt.line - 0.5) * m_exposureDuration;
  double groundRange = imagePt.samp * m_scaledPixelWidth;
  std::vector<double> coeffs = getRangeCoefficients(time);
  double slantRange = groundRangeToSlantRange(groundRange, coeffs);

  // Compute the in-track, cross-track, nadir, coordinate system to solve in
  csm::EcefVector spacecraftPosition = getSpacecraftPosition(time);
  double positionMag = norm(spacecraftPosition);
  csm::EcefVector spacecraftVelocity = getSensorVelocity(time);
  // In-track unit vector
  csm::EcefVector vHat = normalized(spacecraftVelocity);
  // Nadir unit vector
  csm::EcefVector tHat = normalized(rejection(spacecraftPosition, vHat));
  // Cross-track unit vector
  csm::EcefVector uHat = cross(vHat, tHat);

  // Compute the spacecraft position in the new coordinate system
  //   The cross-track unit vector is orthogonal to the position so we ignore it
  double nadirComp = dot(spacecraftPosition, tHat);
  double inTrackComp = dot(spacecraftPosition, vHat);

  // Compute the substituted values
  // Iterate to find proper radius value
  double pointRadius = m_majorAxis + height;
  double radiusSqr;
  double pointHeight;
  csm::EcefVector groundVec;
  do {
    radiusSqr = pointRadius * pointRadius;
    double alpha = (radiusSqr - slantRange * slantRange - positionMag * positionMag) / (2 * nadirComp);
    double beta = sqrt(slantRange * slantRange - alpha * alpha);
    if (m_lookDirection == LEFT) {
      beta *= -1;
    }
    groundVec = alpha * tHat + beta * uHat + spacecraftPosition;
    pointHeight = computeEllipsoidElevation(
        groundVec.x, groundVec.y, groundVec.z,
        m_majorAxis, m_minorAxis);
    pointRadius -= (pointHeight - height);
  } while(fabs(pointHeight - height) > desiredPrecision);


  csm::EcefCoord groundPt(groundVec.x, groundVec.y, groundVec.z);

  return groundPt;
}

csm::EcefCoordCovar UsgsAstroSarSensorModel::imageToGround(
    const csm::ImageCoordCovar& imagePt,
    double height,
    double heightVariance,
    double desiredPrecision,
    double* achievedPrecision,
    csm::WarningList* warnings) const
{
  // Image to ground with error propagation
  // Use numerical partials
  const double DELTA_IMAGE = 1.0;
  const double DELTA_GROUND = m_scaledPixelWidth;
  csm::ImageCoord ip(imagePt.line, imagePt.samp);

  csm::EcefCoord gp = imageToGround(ip, height, desiredPrecision, achievedPrecision, warnings);

  // Compute numerical partials xyz wrt to lsh
  ip.line = imagePt.line + DELTA_IMAGE;
  ip.samp = imagePt.samp;
  csm::EcefCoord gpl = imageToGround(ip, height, desiredPrecision);
  double xpl = (gpl.x - gp.x) / DELTA_IMAGE;
  double ypl = (gpl.y - gp.y) / DELTA_IMAGE;
  double zpl = (gpl.z - gp.z) / DELTA_IMAGE;

  ip.line = imagePt.line;
  ip.samp = imagePt.samp + DELTA_IMAGE;
  csm::EcefCoord gps = imageToGround(ip, height, desiredPrecision);
  double xps = (gps.x - gp.x) / DELTA_IMAGE;
  double yps = (gps.y - gp.y) / DELTA_IMAGE;
  double zps = (gps.z - gp.z) / DELTA_IMAGE;

  ip.line = imagePt.line;
  ip.samp = imagePt.samp; // +DELTA_IMAGE;
  csm::EcefCoord gph = imageToGround(ip, height + DELTA_GROUND, desiredPrecision);
  double xph = (gph.x - gp.x) / DELTA_GROUND;
  double yph = (gph.y - gp.y) / DELTA_GROUND;
  double zph = (gph.z - gp.z) / DELTA_GROUND;

  // Convert sensor covariance to image space
  double sCov[4];
  determineSensorCovarianceInImageSpace(gp, sCov);

  std::vector<double> unmod = getUnmodeledError(imagePt);

  double iCov[4];
  iCov[0] = imagePt.covariance[0] + sCov[0] + unmod[0];
  iCov[1] = imagePt.covariance[1] + sCov[1] + unmod[1];
  iCov[2] = imagePt.covariance[2] + sCov[2] + unmod[2];
  iCov[3] = imagePt.covariance[3] + sCov[3] + unmod[3];

  // Temporary matrix product
  double t00, t01, t02, t10, t11, t12, t20, t21, t22;
  t00 = xpl * iCov[0] + xps * iCov[2];
  t01 = xpl * iCov[1] + xps * iCov[3];
  t02 = xph * heightVariance;
  t10 = ypl * iCov[0] + yps * iCov[2];
  t11 = ypl * iCov[1] + yps * iCov[3];
  t12 = yph * heightVariance;
  t20 = zpl * iCov[0] + zps * iCov[2];
  t21 = zpl * iCov[1] + zps * iCov[3];
  t22 = zph * heightVariance;

  // Ground covariance
  csm::EcefCoordCovar result;
  result.x = gp.x;
  result.y = gp.y;
  result.z = gp.z;

  result.covariance[0] = t00 * xpl + t01 * xps + t02 * xph;
  result.covariance[1] = t00 * ypl + t01 * yps + t02 * yph;
  result.covariance[2] = t00 * zpl + t01 * zps + t02 * zph;
  result.covariance[3] = t10 * xpl + t11 * xps + t12 * xph;
  result.covariance[4] = t10 * ypl + t11 * yps + t12 * yph;
  result.covariance[5] = t10 * zpl + t11 * zps + t12 * zph;
  result.covariance[6] = t20 * xpl + t21 * xps + t22 * xph;
  result.covariance[7] = t20 * ypl + t21 * yps + t22 * yph;
  result.covariance[8] = t20 * zpl + t21 * zps + t22 * zph;

  return result;
}

csm::EcefLocus UsgsAstroSarSensorModel::imageToProximateImagingLocus(
    const csm::ImageCoord& imagePt,
    const csm::EcefCoord& groundPt,
    double desiredPrecision,
    double* achievedPrecision,
    csm::WarningList* warnings) const
{
  // Compute the slant range
  double time = m_startingEphemerisTime + (imagePt.line - 0.5) * m_exposureDuration;
  double groundRange = imagePt.samp * m_scaledPixelWidth;
  std::vector<double> coeffs = getRangeCoefficients(time);
  double slantRange = groundRangeToSlantRange(groundRange, coeffs);

  // Project the sensor to ground point vector onto the 0 doppler plane
  // then compute the closest point at the slant range to that
  csm::EcefVector spacecraftPosition = getSpacecraftPosition(time);
  csm::EcefVector spacecraftVelocity = getSensorVelocity(time);
  csm::EcefVector groundVec(groundPt.x, groundPt.y, groundPt.z);
  csm::EcefVector lookVec = normalized(rejection(groundVec - spacecraftPosition, spacecraftVelocity));
  csm::EcefVector closestVec = spacecraftPosition + slantRange * lookVec;


  // Compute the tangent at the closest point
  csm::EcefVector tangent;
  if (m_lookDirection == LEFT) {
    tangent = cross(spacecraftVelocity, lookVec);
  }
  else {
    tangent = cross(lookVec, spacecraftVelocity);
  }
  tangent = normalized(tangent);

  return csm::EcefLocus(closestVec.x, closestVec.y, closestVec.z,
                        tangent.x,    tangent.y,    tangent.z);
}

csm::EcefLocus UsgsAstroSarSensorModel::imageToRemoteImagingLocus(
    const csm::ImageCoord& imagePt,
    double desiredPrecision,
    double* achievedPrecision,
    csm::WarningList* warnings) const
{
  // Compute the slant range
  double time = m_startingEphemerisTime + (imagePt.line - 0.5) * m_exposureDuration;
  double groundRange = imagePt.samp * m_scaledPixelWidth;
  std::vector<double> coeffs = getRangeCoefficients(time);
  double slantRange = groundRangeToSlantRange(groundRange, coeffs);

  // Project the negative sensor position vector onto the 0 doppler plane
  // then compute the closest point at the slant range to that
  csm::EcefVector spacecraftPosition = getSpacecraftPosition(time);
  csm::EcefVector spacecraftVelocity = getSensorVelocity(time);
  csm::EcefVector lookVec = normalized(rejection(-1 * spacecraftPosition, spacecraftVelocity));
  csm::EcefVector closestVec = spacecraftPosition + slantRange * lookVec;


  // Compute the tangent at the closest point
  csm::EcefVector tangent;
  if (m_lookDirection == LEFT) {
    tangent = cross(spacecraftVelocity, lookVec);
  }
  else {
    tangent = cross(lookVec, spacecraftVelocity);
  }
  tangent = normalized(tangent);

  return csm::EcefLocus(closestVec.x, closestVec.y, closestVec.z,
                        tangent.x,    tangent.y,    tangent.z);
}

csm::ImageCoord UsgsAstroSarSensorModel::getImageStart() const
{
  return csm::ImageCoord(0.0, 0.0);
}

csm::ImageVector UsgsAstroSarSensorModel::getImageSize() const
{
  return csm::ImageVector(m_nLines, m_nSamples);
}

pair<csm::ImageCoord, csm::ImageCoord> UsgsAstroSarSensorModel::getValidImageRange() const
{
  csm::ImageCoord start = getImageStart();
  csm::ImageVector size = getImageSize();
  return make_pair(start, csm::ImageCoord(start.line + size.line, start.samp + size.samp));
}

pair<double, double> UsgsAstroSarSensorModel::getValidHeightRange() const
{
  return make_pair(m_minElevation, m_maxElevation);
}

csm::EcefVector UsgsAstroSarSensorModel::getIlluminationDirection(const csm::EcefCoord& groundPt) const
{
  csm::EcefVector groundVec(groundPt.x, groundPt.y, groundPt.z);
  csm::EcefVector sunPosition = getSunPosition(getImageTime(groundToImage(groundPt)));
  csm::EcefVector illuminationDirection = normalized(groundVec - sunPosition);
  return illuminationDirection;
}

double UsgsAstroSarSensorModel::getImageTime(const csm::ImageCoord& imagePt) const
{
  return m_startingEphemerisTime + (imagePt.line - 0.5) * m_exposureDuration;
}

csm::EcefCoord UsgsAstroSarSensorModel::getSensorPosition(const csm::ImageCoord& imagePt) const
{
  double time = getImageTime(imagePt);
  return getSensorPosition(time);
}

csm::EcefCoord UsgsAstroSarSensorModel::getSensorPosition(double time) const
{
  csm::EcefVector sensorVector = getSpacecraftPosition(time);
  return csm::EcefCoord(sensorVector.x, sensorVector.y, sensorVector.z);
}

csm::EcefVector UsgsAstroSarSensorModel::getSensorVelocity(const csm::ImageCoord& imagePt) const
{
  double time = getImageTime(imagePt);
  return getSensorVelocity(time);
}

csm::EcefVector UsgsAstroSarSensorModel::getSensorVelocity(double time) const
{
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

void UsgsAstroSarSensorModel::determineSensorCovarianceInImageSpace(
    csm::EcefCoord &gp,
    double         sensor_cov[4] ) const
{
  int i, j, totalAdjParams;
  totalAdjParams = getNumParameters();

  std::vector<csm::RasterGM::SensorPartials> sensor_partials = computeAllSensorPartials(gp);

  sensor_cov[0] = 0.0;
  sensor_cov[1] = 0.0;
  sensor_cov[2] = 0.0;
  sensor_cov[3] = 0.0;

  for (i = 0; i < totalAdjParams; i++)
    {
    for (j = 0; j < totalAdjParams; j++)
    {
      sensor_cov[0] += sensor_partials[i].first  * getParameterCovariance(i, j) * sensor_partials[j].first;
      sensor_cov[1] += sensor_partials[i].second * getParameterCovariance(i, j) * sensor_partials[j].first;
      sensor_cov[2] += sensor_partials[i].first  * getParameterCovariance(i, j) * sensor_partials[j].second;
      sensor_cov[3] += sensor_partials[i].second * getParameterCovariance(i, j) * sensor_partials[j].second;
    }
  }
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


std::vector<double> UsgsAstroSarSensorModel::getRangeCoefficients(double time) const {
  int numCoeffs = m_scaleConversionCoefficients.size();
  std::vector<double> interpCoeffs;

  double endTime = m_scaleConversionTimes.back();
  if ((numCoeffs/4) > 1) {
    double coefficients[4];
    double dtEphem = (endTime - m_scaleConversionTimes[0]) / (m_scaleConversionCoefficients.size()/4);
    lagrangeInterp(m_scaleConversionCoefficients.size()/4, &m_scaleConversionCoefficients[0], m_scaleConversionTimes[0], dtEphem,
                   time, 4, 8, coefficients);
    interpCoeffs.push_back(coefficients[0]);
    interpCoeffs.push_back(coefficients[1]);
    interpCoeffs.push_back(coefficients[2]);
    interpCoeffs.push_back(coefficients[3]);
  }
  else {
    interpCoeffs.push_back(m_scaleConversionCoefficients[0]);
    interpCoeffs.push_back(m_scaleConversionCoefficients[1]);
    interpCoeffs.push_back(m_scaleConversionCoefficients[2]);
    interpCoeffs.push_back(m_scaleConversionCoefficients[3]);
  }
  return interpCoeffs;
}

csm::EcefVector UsgsAstroSarSensorModel::getSunPosition(const double imageTime) const
{

  int numSunPositions = m_sunPosition.size();
  int numSunVelocities = m_sunVelocity.size();
  csm::EcefVector sunPosition = csm::EcefVector();

  // If there are multiple positions, use Lagrange interpolation
  if ((numSunPositions/3) > 1) {
    double sunPos[3];
    double endTime = m_t0Ephem + m_nLines * m_exposureDuration;
    double sun_dtEphem = (endTime - m_t0Ephem) / (numSunPositions/3);
    lagrangeInterp(numSunPositions/3, &m_sunPosition[0], m_t0Ephem, sun_dtEphem,
                   imageTime, 3, 8, sunPos);
    sunPosition.x = sunPos[0];
    sunPosition.y = sunPos[1];
    sunPosition.z = sunPos[2];
  }
  else if ((numSunVelocities/3) >= 1){
    // If there is one position triple with at least one velocity triple
    //  then the illumination direction is calculated via linear extrapolation.
      sunPosition.x = (imageTime * m_sunVelocity[0] + m_sunPosition[0]);
      sunPosition.y = (imageTime * m_sunVelocity[1] + m_sunPosition[1]);
      sunPosition.z = (imageTime * m_sunVelocity[2] + m_sunPosition[2]);
  }
  else {
    // If there is one position triple with no velocity triple, then the
    //  illumination direction is the difference of the original vectors.
      sunPosition.x = m_sunPosition[0];
      sunPosition.y = m_sunPosition[1];
      sunPosition.z = m_sunPosition[2];
  }
  return sunPosition;
}
