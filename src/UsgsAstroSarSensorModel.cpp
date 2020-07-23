#include "UsgsAstroSarSensorModel.h"
#include "Utilities.h"

#include <string.h>
#include <cmath>
#include <functional>
#include <iomanip>

#include <nlohmann/json.hpp>

#include "ale/Util.h"

using json = nlohmann::json;
using namespace std;

#define MESSAGE_LOG(...)         \
  if (m_logger) {                \
    m_logger->info(__VA_ARGS__); \
  }

const string UsgsAstroSarSensorModel::_SENSOR_MODEL_NAME =
    "USGS_ASTRO_SAR_SENSOR_MODEL";
const int UsgsAstroSarSensorModel::NUM_PARAMETERS = 6;
const string UsgsAstroSarSensorModel::PARAMETER_NAME[] = {
    "X Pos. Bias   ",  // 0
    "Y Pos. Bias   ",  // 1
    "Z Pos. Bias   ",  // 2
    "X Vel. Bias   ",  // 3
    "Y Vel. Bias   ",  // 4
    "Z Vel. Bias   "   // 5
};

const int UsgsAstroSarSensorModel::NUM_PARAM_TYPES = 4;
const string UsgsAstroSarSensorModel::PARAM_STRING_ALL[] = {
    "NONE", "FICTITIOUS", "REAL", "FIXED"};
const csm::param::Type UsgsAstroSarSensorModel::PARAM_CHAR_ALL[] = {
    csm::param::NONE, csm::param::FICTITIOUS, csm::param::REAL,
    csm::param::FIXED};

string UsgsAstroSarSensorModel::constructStateFromIsd(
    const string imageSupportData, csm::WarningList* warnings) {
  MESSAGE_LOG("UsgsAstroSarSensorModel constructing state from ISD, with {}",
              imageSupportData);
  json isd = json::parse(imageSupportData);
  json state = {};

  csm::WarningList* parsingWarnings = new csm::WarningList;

  // Zero computed state values
  state["m_referencePointXyz"] = vector<double>(3, 0.0);

  state["m_modelName"] = ale::getSensorModelName(isd);
  state["m_imageIdentifier"] = ale::getImageId(isd);
  state["m_sensorName"] = ale::getSensorName(isd);
  state["m_platformName"] = ale::getPlatformName(isd);

  state["m_nLines"] = ale::getTotalLines(isd);
  state["m_nSamples"] = ale::getTotalSamples(isd);

  ale::Orientations j2000_to_target = ale::getBodyRotation(isd);

  ale::States instState = ale::getInstrumentPosition(isd);
  ale::States sunState = ale::getSunPosition(isd);
  std::vector<double> ephemTime = instState.getTimes();
  if (ephemTime.size() < 2) {
    parsingWarnings->push_back(csm::Warning(
        csm::Warning::DATA_NOT_AVAILABLE, "Instrument position has less than 2 states.",
        "UsgsAstroSarSensorModel::constructStateFromIsd()"));
    MESSAGE_LOG("m_dtEphem not in ISD")
  }
  else {
    // Re-compute times so that they are linearly spaced
    double startTime = ephemTime.front();
    double timeStep = (ephemTime.back() - ephemTime.front()) / (ephemTime.size() - 1);
    state["m_t0Ephem"] = startTime;
    state["m_dtEphem"] = timeStep;
    for (int i = 0; i < ephemTime.size(); i++) {
      ephemTime[i] = startTime + i * timeStep;
    }
  }
  std::vector<double> instPositions = {};
  std::vector<double> instVelocities = {};
  std::vector<double> sunPositions = {};
  std::vector<double> sunVelocities = {};
  for (int i = 0; i < ephemTime.size(); i++) {
    ale::State j2000InstState = instState.getState(ephemTime[i], ale::SPLINE);
    ale::State rotatedInstState =
        j2000_to_target.rotateStateAt(ephemTime[i], j2000InstState, ale::SLERP);
    // ALE operates in Km and we want m
    instPositions.push_back(rotatedInstState.position.x * 1000);
    instPositions.push_back(rotatedInstState.position.y * 1000);
    instPositions.push_back(rotatedInstState.position.z * 1000);
    instVelocities.push_back(rotatedInstState.velocity.x * 1000);
    instVelocities.push_back(rotatedInstState.velocity.y * 1000);
    instVelocities.push_back(rotatedInstState.velocity.z * 1000);

    ale::State j2000SunState = sunState.getState(ephemTime[i], ale::SPLINE);
    ale::State rotatedSunState =
        j2000_to_target.rotateStateAt(ephemTime[i], j2000SunState, ale::SLERP);
    // ALE operates in Km and we want m
    sunPositions.push_back(rotatedSunState.position.x * 1000);
    sunPositions.push_back(rotatedSunState.position.y * 1000);
    sunPositions.push_back(rotatedSunState.position.z * 1000);
    sunVelocities.push_back(rotatedSunState.velocity.x * 1000);
    sunVelocities.push_back(rotatedSunState.velocity.y * 1000);
    sunVelocities.push_back(rotatedSunState.velocity.z * 1000);
  }
  state["m_positions"] = instPositions;
  state["m_velocities"] = instVelocities;

  state["m_sunPosition"] = sunPositions;
  state["m_sunVelocity"] = sunVelocities;

  state["m_centerEphemerisTime"] = ale::getCenterTime(isd);
  state["m_startingEphemerisTime"] = ale::getStartingTime(isd);
  state["m_endingEphemerisTime"] = getEndingTime(isd, parsingWarnings);

  state["m_exposureDuration"] = getExposureDuration(isd, parsingWarnings);

  state["m_currentParameterValue"] = vector<double>(NUM_PARAMETERS, 0.0);

  // get radii
  state["m_minorAxis"] = ale::getSemiMinorRadius(isd);
  state["m_majorAxis"] = ale::getSemiMajorRadius(isd);

  // set identifiers
  state["m_platformIdentifier"] = ale::getPlatformName(isd);
  state["m_sensorIdentifier"] = ale::getSensorName(isd);

  // get reference_height
  state["m_minElevation"] = -1000;
  state["m_maxElevation"] = 1000;

  // SAR specific values
  state["m_scaledPixelWidth"] = getScaledPixelWidth(isd, parsingWarnings);
  state["m_scaleConversionCoefficients"] =
      getScaleConversionCoefficients(isd, parsingWarnings);
  state["m_scaleConversionTimes"] =
      getScaleConversionTimes(isd, parsingWarnings);
  state["m_wavelength"] = getWavelength(isd, parsingWarnings);
  state["m_lookDirection"] = getLookDirection(isd, parsingWarnings);

  // Default to identity covariance
  state["m_covariance"] = vector<double>(NUM_PARAMETERS * NUM_PARAMETERS, 0.0);
  for (int i = 0; i < NUM_PARAMETERS; i++) {
    state["m_covariance"][i * NUM_PARAMETERS + i] = 1.0;
  }

  if (!parsingWarnings->empty()) {
    if (warnings) {
      warnings->insert(warnings->end(), parsingWarnings->begin(),
                       parsingWarnings->end());
    }
    std::string message =
        "ISD is invalid for creating the sensor model with error [";
    csm::Warning warn = parsingWarnings->front();
    message += warn.getMessage();
    message += "]";
    parsingWarnings = nullptr;
    delete parsingWarnings;
    MESSAGE_LOG(message);
    throw csm::Error(csm::Error::SENSOR_MODEL_NOT_CONSTRUCTIBLE, message,
                     "UsgsAstroSarSensorModel::constructStateFromIsd");
  }

  delete parsingWarnings;
  parsingWarnings = nullptr;

  // The state data will still be updated when a sensor model is created since
  // some state data is not in the ISD and requires a SM to compute them.
  return state.dump();
}

string UsgsAstroSarSensorModel::getModelNameFromModelState(
    const string& model_state) {
  MESSAGE_LOG("Getting model name from model state: {}", model_state);
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
    MESSAGE_LOG(aMessage);
    csm::Error csmErr(aErrorType, aMessage, aFunction);
    throw(csmErr);
  }
  if (model_name != _SENSOR_MODEL_NAME) {
    csm::Error::ErrorType aErrorType = csm::Error::SENSOR_MODEL_NOT_SUPPORTED;
    string aMessage = "Sensor model not supported.";
    string aFunction = "UsgsAstroSarSensorModel::getModelNameFromModelState()";
    MESSAGE_LOG(aMessage);
    csm::Error csmErr(aErrorType, aMessage, aFunction);
    throw(csmErr);
  }
  return model_name;
}

UsgsAstroSarSensorModel::UsgsAstroSarSensorModel() {
  MESSAGE_LOG("Constructing UsgsAstroSarSensorModel");
  reset();
}

void UsgsAstroSarSensorModel::reset() {
  MESSAGE_LOG("Resetting UsgsAstroSarSensorModel");
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
  m_covariance = vector<double>(NUM_PARAMETERS * NUM_PARAMETERS, 0.0);
  m_sunPosition.clear();
  m_sunVelocity.clear();
  m_wavelength = 0;
  m_noAdjustments = std::vector<double>(NUM_PARAMETERS, 0.0);
}

void UsgsAstroSarSensorModel::replaceModelState(const string& argState) {
  reset();

  MESSAGE_LOG("Replacing model state with: {}", argState);
  auto stateJson = json::parse(argState);

  m_imageIdentifier = stateJson["m_imageIdentifier"].get<string>();
  m_platformIdentifier = stateJson["m_platformIdentifier"].get<string>();
  m_sensorName = stateJson["m_sensorName"].get<string>();
  m_nLines = stateJson["m_nLines"];
  m_nSamples = stateJson["m_nSamples"];
  m_exposureDuration = stateJson["m_exposureDuration"];
  m_scaledPixelWidth = stateJson["m_scaledPixelWidth"];
  std::string lookStr = stateJson["m_lookDirection"];
  if (lookStr.compare("right") == 0) {
    m_lookDirection = UsgsAstroSarSensorModel::RIGHT;
  } else if (lookStr.compare("left") == 0) {
    m_lookDirection = UsgsAstroSarSensorModel::LEFT;
  } else {
    std::string message = "Could not determine look direction from state";
    MESSAGE_LOG(message);
    throw csm::Error(csm::Error::INVALID_SENSOR_MODEL_STATE, message,
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
  m_scaleConversionCoefficients =
      stateJson["m_scaleConversionCoefficients"].get<vector<double>>();
  m_scaleConversionTimes =
      stateJson["m_scaleConversionTimes"].get<vector<double>>();
  m_positions = stateJson["m_positions"].get<vector<double>>();
  m_velocities = stateJson["m_velocities"].get<vector<double>>();
  m_currentParameterValue =
      stateJson["m_currentParameterValue"].get<vector<double>>();
  m_referencePointXyz.x = stateJson["m_referencePointXyz"][0];
  m_referencePointXyz.y = stateJson["m_referencePointXyz"][1];
  m_referencePointXyz.z = stateJson["m_referencePointXyz"][2];
  m_covariance = stateJson["m_covariance"].get<vector<double>>();
  m_sunPosition = stateJson["m_sunPosition"].get<vector<double>>();
  m_sunVelocity = stateJson["m_sunVelocity"].get<vector<double>>();

  // If sensor model is being created for the first time, this routine will set
  // the reference point
  if (m_referencePointXyz.x == 0 && m_referencePointXyz.y == 0 &&
      m_referencePointXyz.z == 0) {
    MESSAGE_LOG("Updating State")

    double lineCtr = m_nLines / 2.0;
    double sampCtr = m_nSamples / 2.0;
    csm::ImageCoord ip(lineCtr, sampCtr);
    MESSAGE_LOG("updateState: center image coordinate set to {} {}", lineCtr,
                sampCtr)

    double refHeight = 0;
    m_referencePointXyz = imageToGround(ip, refHeight);
    MESSAGE_LOG("updateState: reference point (x, y, z) {} {} {}",
                m_referencePointXyz.x, m_referencePointXyz.y,
                m_referencePointXyz.z)
  }
}

string UsgsAstroSarSensorModel::getModelState() const {
  json state = {};

  state["m_modelName"] = _SENSOR_MODEL_NAME;
  state["m_imageIdentifier"] = m_imageIdentifier;
  state["m_sensorName"] = m_sensorName;
  state["m_platformName"] = m_platformName;
  state["m_nLines"] = m_nLines;
  state["m_nSamples"] = m_nSamples;
  state["m_referencePointXyz"] = {m_referencePointXyz.x, m_referencePointXyz.y,
                                  m_referencePointXyz.z};
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
  } else if (m_lookDirection == 1) {
    state["m_lookDirection"] = "right";
  } else {
    std::string message = "Could not parse look direction from json state.";
    MESSAGE_LOG(message);
    throw csm::Error(csm::Error::INVALID_SENSOR_MODEL_STATE, message,
                     "UsgsAstroSarSensorModel::getModelState");
  }
  state["m_wavelength"] = m_wavelength;
  state["m_scaleConversionCoefficients"] = m_scaleConversionCoefficients;
  state["m_scaleConversionTimes"] = m_scaleConversionTimes;
  state["m_covariance"] = m_covariance;

  return state.dump();
}

csm::ImageCoord UsgsAstroSarSensorModel::groundToImage(
    const csm::EcefCoord& groundPt, double desiredPrecision,
    double* achievedPrecision, csm::WarningList* warnings) const {
  MESSAGE_LOG(
      "Computing groundToImage(ImageCoord) for {}, {}, {}, with desired "
      "precision {}"
      "No adjustments.",
      groundPt.x, groundPt.y, groundPt.z, desiredPrecision);

  csm::ImageCoord imagePt = groundToImage(
      groundPt, m_noAdjustments, desiredPrecision, achievedPrecision, warnings);
  return imagePt;
}

csm::ImageCoord UsgsAstroSarSensorModel::groundToImage(
    const csm::EcefCoord& groundPt, const std::vector<double> adj,
    double desiredPrecision, double* achievedPrecision,
    csm::WarningList* warnings) const {
  MESSAGE_LOG(
      "Computing groundToImage(ImageCoord) for {}, {}, {}, with desired "
      "precision {}, and "
      "adjustments.",
      groundPt.x, groundPt.y, groundPt.z, desiredPrecision);

  // Find time of closest approach to groundPt and the corresponding slant range
  // by finding the root of the doppler shift frequency
  try {
    double timeTolerance = m_exposureDuration * desiredPrecision / 2.0;

    double time = dopplerShift(groundPt, timeTolerance, adj);
    double slantRangeValue = slantRange(groundPt, time, adj);

    // Find the ground range, based on the ground-range-to-slant-range
    // polynomial defined by the range coefficient set, with a time closest to
    // the calculated time of closest approach
    double groundTolerance = m_scaledPixelWidth * desiredPrecision / 2.0;
    double groundRange = slantRangeToGroundRange(
        groundPt, time, slantRangeValue, groundTolerance);

    double line = (time - m_startingEphemerisTime) / m_exposureDuration + 0.5;
    double sample = groundRange / m_scaledPixelWidth;
    return csm::ImageCoord(line, sample);
  } catch (std::exception& error) {
    std::string message = "Could not calculate groundToImage, with error [";
    message += error.what();
    message += "]";
    MESSAGE_LOG(message);
    throw csm::Error(csm::Error::UNKNOWN_ERROR, message, "groundToImage");
  }
}

// Calculate the root
double UsgsAstroSarSensorModel::dopplerShift(
    csm::EcefCoord groundPt, double tolerance,
    const std::vector<double> adj) const {
  MESSAGE_LOG("Calculating doppler shift with: {}, {}, {}, and tolerance {}.",
              groundPt.x, groundPt.y, groundPt.z, tolerance);
  csm::EcefVector groundVec(groundPt.x, groundPt.y, groundPt.z);
  std::function<double(double)> dopplerShiftFunction = [this, groundVec,
                                                        adj](double time) {
    csm::EcefVector spacecraftPosition =
        getAdjustedSpacecraftPosition(time, adj);
    csm::EcefVector spacecraftVelocity = getAdjustedSensorVelocity(time, adj);
    csm::EcefVector lookVector = spacecraftPosition - groundVec;

    double slantRange = norm(lookVector);

    double dopplerShift = -2.0 * dot(lookVector, spacecraftVelocity) /
                          (slantRange * m_wavelength);

    return dopplerShift;
  };

  // Do root-finding for "dopplerShift"
  double timePadding = m_exposureDuration * m_nLines * 0.25;
  return brentRoot(m_startingEphemerisTime - timePadding,
                   m_endingEphemerisTime + timePadding, dopplerShiftFunction,
                   tolerance);
}

double UsgsAstroSarSensorModel::slantRange(csm::EcefCoord surfPt, double time,
                                           std::vector<double> adj) const {
  MESSAGE_LOG("Calculating slant range with: {}, {}, {}, and time {}.",
              surfPt.x, surfPt.y, surfPt.z, time);
  csm::EcefVector surfVec(surfPt.x, surfPt.y, surfPt.z);
  csm::EcefVector spacecraftPosition = getAdjustedSpacecraftPosition(time, adj);
  return norm(spacecraftPosition - surfVec);
}

double UsgsAstroSarSensorModel::slantRangeToGroundRange(
    const csm::EcefCoord& groundPt, double time, double slantRange,
    double tolerance) const {
  MESSAGE_LOG(
      "Calculating slant range to ground range with: {}, {}, {}, {}, {}, {}",
      groundPt.x, groundPt.y, groundPt.z, time, slantRange, tolerance);

  std::vector<double> coeffs = getRangeCoefficients(time);

  // Need to come up with an initial guess when solving for ground
  // range given slant range. Naively use the middle of the image.
  double guess = 0.5 * m_nSamples * m_scaledPixelWidth;

  // Tolerance to 1/20th of a pixel for now.
  coeffs[0] -= slantRange;
  return polynomialRoot(coeffs, guess, tolerance);
}

double UsgsAstroSarSensorModel::groundRangeToSlantRange(
    double groundRange, const std::vector<double>& coeffs) const {
  return evaluatePolynomial(coeffs, groundRange);
}

csm::ImageCoordCovar UsgsAstroSarSensorModel::groundToImage(
    const csm::EcefCoordCovar& groundPt, double desiredPrecision,
    double* achievedPrecision, csm::WarningList* warnings) const {
  MESSAGE_LOG("Calculating groundToImage with: {}, {}, {}, {}", groundPt.x,
              groundPt.y, groundPt.z, desiredPrecision);
  // Ground to image with error propagation
  // Compute corresponding image point
  csm::EcefCoord gp(groundPt);

  csm::ImageCoord ip =
      groundToImage(gp, desiredPrecision, achievedPrecision, warnings);
  csm::ImageCoordCovar result(ip.line, ip.samp);

  // Compute partials ls wrt XYZ
  std::vector<double> prt(6, 0.0);
  prt = computeGroundPartials(groundPt);

  // Error propagation
  double ltx, lty, ltz;
  double stx, sty, stz;
  ltx = prt[0] * groundPt.covariance[0] + prt[1] * groundPt.covariance[3] +
        prt[2] * groundPt.covariance[6];
  lty = prt[0] * groundPt.covariance[1] + prt[1] * groundPt.covariance[4] +
        prt[2] * groundPt.covariance[7];
  ltz = prt[0] * groundPt.covariance[2] + prt[1] * groundPt.covariance[5] +
        prt[2] * groundPt.covariance[8];
  stx = prt[3] * groundPt.covariance[0] + prt[4] * groundPt.covariance[3] +
        prt[5] * groundPt.covariance[6];
  sty = prt[3] * groundPt.covariance[1] + prt[4] * groundPt.covariance[4] +
        prt[5] * groundPt.covariance[7];
  stz = prt[3] * groundPt.covariance[2] + prt[4] * groundPt.covariance[5] +
        prt[5] * groundPt.covariance[8];

  double gp_cov[4];  // Input gp cov in image space
  gp_cov[0] = ltx * prt[0] + lty * prt[1] + ltz * prt[2];
  gp_cov[1] = ltx * prt[3] + lty * prt[4] + ltz * prt[5];
  gp_cov[2] = stx * prt[0] + sty * prt[1] + stz * prt[2];
  gp_cov[3] = stx * prt[3] + sty * prt[4] + stz * prt[5];

  std::vector<double> unmodeled_cov = getUnmodeledError(ip);
  double sensor_cov[4];  // sensor cov in image space
  determineSensorCovarianceInImageSpace(gp, sensor_cov);

  result.covariance[0] = gp_cov[0] + unmodeled_cov[0] + sensor_cov[0];
  result.covariance[1] = gp_cov[1] + unmodeled_cov[1] + sensor_cov[1];
  result.covariance[2] = gp_cov[2] + unmodeled_cov[2] + sensor_cov[2];
  result.covariance[3] = gp_cov[3] + unmodeled_cov[3] + sensor_cov[3];

  return result;
}

csm::EcefCoord UsgsAstroSarSensorModel::imageToGround(
    const csm::ImageCoord& imagePt, double height, double desiredPrecision,
    double* achievedPrecision, csm::WarningList* warnings) const {
  MESSAGE_LOG("Calculating imageToGround with: {}, {}, {}, {}", imagePt.samp,
              imagePt.line, height, desiredPrecision);
  double time =
      m_startingEphemerisTime + (imagePt.line - 0.5) * m_exposureDuration;
  double groundRange = imagePt.samp * m_scaledPixelWidth;
  MESSAGE_LOG("Ground range: {}", groundRange);
  std::vector<double> coeffs = getRangeCoefficients(time);
  double slantRange = groundRangeToSlantRange(groundRange, coeffs);
  MESSAGE_LOG("Slant range: {}", slantRange);

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
  MESSAGE_LOG("In track vector: {}, {}, {}", vHat.x, vHat.y, vHat.z);
  MESSAGE_LOG("Nadir vector: {}, {}, {}", tHat.x, tHat.y, tHat.z);
  MESSAGE_LOG("In track vector: {}, {}, {}", uHat.x, uHat.y, uHat.z);

  // Compute the spacecraft position in the new coordinate system
  //   The cross-track unit vector is orthogonal to the position so we ignore it
  double nadirComp = dot(spacecraftPosition, tHat);

  // Iterate to find proper radius value
  double pointRadius = m_majorAxis + height;
  double radiusSqr;
  double pointHeight;
  csm::EcefVector groundVec;
  do {
    MESSAGE_LOG("Iteration radius: {}", pointRadius);
    radiusSqr = pointRadius * pointRadius;
    double alpha =
        (radiusSqr - slantRange * slantRange - positionMag * positionMag) /
        (2 * nadirComp);
    MESSAGE_LOG("Iteration alpha: {}", alpha);
    double beta = sqrt(slantRange * slantRange - alpha * alpha);
    if (m_lookDirection == LEFT) {
      beta *= -1;
    }
    MESSAGE_LOG("Iteration beta: {}", beta);
    groundVec = alpha * tHat + beta * uHat + spacecraftPosition;
    pointHeight = computeEllipsoidElevation(
        groundVec.x, groundVec.y, groundVec.z, m_majorAxis, m_minorAxis);
    pointRadius -= (pointHeight - height);
    MESSAGE_LOG("Iteration ground point: {}, {}, {}", groundVec.x, groundVec.y, groundVec.z);
  } while (fabs(pointHeight - height) > desiredPrecision);

  csm::EcefCoord groundPt(groundVec.x, groundVec.y, groundVec.z);

  return groundPt;
}

csm::EcefCoordCovar UsgsAstroSarSensorModel::imageToGround(
    const csm::ImageCoordCovar& imagePt, double height, double heightVariance,
    double desiredPrecision, double* achievedPrecision,
    csm::WarningList* warnings) const {
  MESSAGE_LOG("Calculating imageToGroundWith: {}, {}, {}, {}, {}", imagePt.samp,
              imagePt.line, height, heightVariance, desiredPrecision);
  // Image to ground with error propagation
  // Use numerical partials
  const double DELTA_IMAGE = 1.0;
  const double DELTA_GROUND = m_scaledPixelWidth;
  csm::ImageCoord ip(imagePt.line, imagePt.samp);

  csm::EcefCoord gp =
      imageToGround(ip, height, desiredPrecision, achievedPrecision, warnings);

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
  ip.samp = imagePt.samp;  // +DELTA_IMAGE;
  csm::EcefCoord gph =
      imageToGround(ip, height + DELTA_GROUND, desiredPrecision);
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
    const csm::ImageCoord& imagePt, const csm::EcefCoord& groundPt,
    double desiredPrecision, double* achievedPrecision,
    csm::WarningList* warnings) const {
  // Compute the slant range
  double time =
      m_startingEphemerisTime + (imagePt.line - 0.5) * m_exposureDuration;
  double groundRange = imagePt.samp * m_scaledPixelWidth;
  std::vector<double> coeffs = getRangeCoefficients(time);
  double slantRange = groundRangeToSlantRange(groundRange, coeffs);

  // Project the sensor to ground point vector onto the 0 doppler plane
  // then compute the closest point at the slant range to that
  csm::EcefVector spacecraftPosition = getSpacecraftPosition(time);
  csm::EcefVector spacecraftVelocity = getSensorVelocity(time);
  csm::EcefVector groundVec(groundPt.x, groundPt.y, groundPt.z);
  csm::EcefVector lookVec =
      normalized(rejection(groundVec - spacecraftPosition, spacecraftVelocity));
  csm::EcefVector closestVec = spacecraftPosition + slantRange * lookVec;

  // Compute the tangent at the closest point
  csm::EcefVector tangent;
  if (m_lookDirection == LEFT) {
    tangent = cross(spacecraftVelocity, lookVec);
  } else {
    tangent = cross(lookVec, spacecraftVelocity);
  }
  tangent = normalized(tangent);

  return csm::EcefLocus(closestVec.x, closestVec.y, closestVec.z, tangent.x,
                        tangent.y, tangent.z);
}

csm::EcefLocus UsgsAstroSarSensorModel::imageToRemoteImagingLocus(
    const csm::ImageCoord& imagePt, double desiredPrecision,
    double* achievedPrecision, csm::WarningList* warnings) const {
  // Compute the slant range
  double time =
      m_startingEphemerisTime + (imagePt.line - 0.5) * m_exposureDuration;
  double groundRange = imagePt.samp * m_scaledPixelWidth;
  std::vector<double> coeffs = getRangeCoefficients(time);
  double slantRange = groundRangeToSlantRange(groundRange, coeffs);

  // Project the negative sensor position vector onto the 0 doppler plane
  // then compute the closest point at the slant range to that
  csm::EcefVector spacecraftPosition = getSpacecraftPosition(time);
  csm::EcefVector spacecraftVelocity = getSensorVelocity(time);
  csm::EcefVector lookVec =
      normalized(rejection(-1 * spacecraftPosition, spacecraftVelocity));
  csm::EcefVector closestVec = spacecraftPosition + slantRange * lookVec;

  // Compute the tangent at the closest point
  csm::EcefVector tangent;
  if (m_lookDirection == LEFT) {
    tangent = cross(spacecraftVelocity, lookVec);
  } else {
    tangent = cross(lookVec, spacecraftVelocity);
  }
  tangent = normalized(tangent);

  return csm::EcefLocus(closestVec.x, closestVec.y, closestVec.z, tangent.x,
                        tangent.y, tangent.z);
}

csm::ImageCoord UsgsAstroSarSensorModel::getImageStart() const {
  return csm::ImageCoord(0.0, 0.0);
}

csm::ImageVector UsgsAstroSarSensorModel::getImageSize() const {
  return csm::ImageVector(m_nLines, m_nSamples);
}

pair<csm::ImageCoord, csm::ImageCoord>
UsgsAstroSarSensorModel::getValidImageRange() const {
  csm::ImageCoord start = getImageStart();
  csm::ImageVector size = getImageSize();
  return make_pair(
      start, csm::ImageCoord(start.line + size.line, start.samp + size.samp));
}

pair<double, double> UsgsAstroSarSensorModel::getValidHeightRange() const {
  return make_pair(m_minElevation, m_maxElevation);
}

csm::EcefVector UsgsAstroSarSensorModel::getIlluminationDirection(
    const csm::EcefCoord& groundPt) const {
  csm::EcefVector groundVec(groundPt.x, groundPt.y, groundPt.z);
  csm::EcefVector sunPosition =
      getSunPosition(getImageTime(groundToImage(groundPt)));
  csm::EcefVector illuminationDirection = normalized(groundVec - sunPosition);
  return illuminationDirection;
}

double UsgsAstroSarSensorModel::getImageTime(
    const csm::ImageCoord& imagePt) const {
  return m_startingEphemerisTime + (imagePt.line - 0.5) * m_exposureDuration;
}

csm::EcefVector UsgsAstroSarSensorModel::getSpacecraftPosition(
    double time) const {
  MESSAGE_LOG("getSpacecraftPosition at {} without adjustments", time)
  csm::EcefCoord spacecraftPosition = getSensorPosition(time);
  return csm::EcefVector(spacecraftPosition.x, spacecraftPosition.y,
                         spacecraftPosition.z);
}

csm::EcefVector UsgsAstroSarSensorModel::getAdjustedSpacecraftPosition(
    double time, std::vector<double> adj) const {
  MESSAGE_LOG("getSpacecraftPosition at {} with adjustments", time)
  csm::EcefCoord spacecraftPosition = getAdjustedSensorPosition(time, adj);
  return csm::EcefVector(spacecraftPosition.x, spacecraftPosition.y,
                         spacecraftPosition.z);
}

csm::EcefCoord UsgsAstroSarSensorModel::getSensorPosition(double time) const {
  MESSAGE_LOG("getSensorPosition at {}.", time)
  csm::EcefCoord sensorPosition =
      getAdjustedSensorPosition(time, m_noAdjustments);
  return sensorPosition;
}

csm::EcefCoord UsgsAstroSarSensorModel::getSensorPosition(
    const csm::ImageCoord& imagePt) const {
  MESSAGE_LOG("getSensorPosition at {}, {}.", imagePt.samp, imagePt.line);
  double time = getImageTime(imagePt);
  return getSensorPosition(time);
}

csm::EcefCoord UsgsAstroSarSensorModel::getAdjustedSensorPosition(
    double time, std::vector<double> adj) const {
  MESSAGE_LOG("getSensorPosition at {}. With adjustments: {}.", time);
  int numPositions = m_positions.size();
  csm::EcefVector spacecraftPosition = csm::EcefVector();

  // If there are multiple positions, use Lagrange interpolation
  if ((numPositions / 3) > 1) {
    double position[3];
    lagrangeInterp(numPositions / 3, &m_positions[0], m_t0Ephem, m_dtEphem,
                   time, 3, 8, position);
    spacecraftPosition.x = position[0] + getValue(0, adj);
    spacecraftPosition.y = position[1] + getValue(1, adj);
    spacecraftPosition.z = position[2] + getValue(2, adj);
  } else {
    spacecraftPosition.x = m_positions[0] + getValue(0, adj);
    spacecraftPosition.y = m_positions[1] + getValue(1, adj);
    spacecraftPosition.z = m_positions[2] + getValue(2, adj);
  }
  MESSAGE_LOG("Adjusted sensor positions is {}, {}, {}.",
      spacecraftPosition.x, spacecraftPosition.y, spacecraftPosition.z);
  return csm::EcefCoord(spacecraftPosition.x, spacecraftPosition.y,
                        spacecraftPosition.z);
}

csm::EcefVector UsgsAstroSarSensorModel::getSensorVelocity(
    const csm::ImageCoord& imagePt) const {
  MESSAGE_LOG("getSensorVelocity at {}, {}. No adjustments.", imagePt.samp,
              imagePt.line);
  double time = getImageTime(imagePt);
  return getSensorVelocity(time);
}

csm::EcefVector UsgsAstroSarSensorModel::getSensorVelocity(double time) const {
  MESSAGE_LOG("getSensorVelocity at {}. Without adjustments.", time);
  csm::EcefVector spacecraftVelocity =
      getAdjustedSensorVelocity(time, m_noAdjustments);
  return spacecraftVelocity;
}

csm::EcefVector UsgsAstroSarSensorModel::getAdjustedSensorVelocity(
    double time, std::vector<double> adj) const {
  MESSAGE_LOG("getSensorVelocity at {}. With adjustments.", time);
  int numVelocities = m_velocities.size();
  csm::EcefVector spacecraftVelocity = csm::EcefVector();

  // If there are multiple positions, use Lagrange interpolation
  if ((numVelocities / 3) > 1) {
    double velocity[3];
    lagrangeInterp(numVelocities / 3, &m_velocities[0], m_t0Ephem, m_dtEphem,
                   time, 3, 8, velocity);
    spacecraftVelocity.x = velocity[0] + getValue(3, adj);
    spacecraftVelocity.y = velocity[1] + getValue(4, adj);
    spacecraftVelocity.z = velocity[2] + getValue(5, adj);
  } else {
    spacecraftVelocity.x = m_velocities[0] + getValue(3, adj);
    spacecraftVelocity.y = m_velocities[1] + getValue(4, adj);
    spacecraftVelocity.z = m_velocities[2] + getValue(5, adj);
  }
  MESSAGE_LOG("Adjusted sensor velocity is {}, {}, {}",
      spacecraftVelocity.x, spacecraftVelocity.y, spacecraftVelocity.z);
  return spacecraftVelocity;
}

csm::RasterGM::SensorPartials UsgsAstroSarSensorModel::computeSensorPartials(
    int index, const csm::EcefCoord& groundPt, double desiredPrecision,
    double* achievedPrecision, csm::WarningList* warnings) const {
  MESSAGE_LOG(
      "Calculating computeSensorPartials for ground point {}, {}, {} with "
      "desired precision {}",
      groundPt.x, groundPt.y, groundPt.z, desiredPrecision)

  // Compute image coordinate first
  csm::ImageCoord imgPt =
      groundToImage(groundPt, desiredPrecision, achievedPrecision);

  // Call overloaded function
  return computeSensorPartials(index, imgPt, groundPt, desiredPrecision,
                               achievedPrecision, warnings);
}

csm::RasterGM::SensorPartials UsgsAstroSarSensorModel::computeSensorPartials(
    int index, const csm::ImageCoord& imagePt, const csm::EcefCoord& groundPt,
    double desiredPrecision, double* achievedPrecision,
    csm::WarningList* warnings) const {
  MESSAGE_LOG(
      "Calculating computeSensorPartials (with image points {}, {}) for ground "
      "point {}, {}, "
      "{} with desired precision {}",
      imagePt.line, imagePt.samp, groundPt.x, groundPt.y, groundPt.z,
      desiredPrecision)

  // Compute numerical partials wrt specific parameter

  const double DELTA = m_scaledPixelWidth;
  std::vector<double> adj(UsgsAstroSarSensorModel::NUM_PARAMETERS, 0.0);
  adj[index] = DELTA;

  csm::ImageCoord adjImgPt = groundToImage(groundPt, adj, desiredPrecision,
                                           achievedPrecision, warnings);

  double linePartial = (adjImgPt.line - imagePt.line) / DELTA;
  double samplePartial = (adjImgPt.samp - imagePt.samp) / DELTA;

  return csm::RasterGM::SensorPartials(linePartial, samplePartial);
}

vector<double> UsgsAstroSarSensorModel::computeGroundPartials(
    const csm::EcefCoord& groundPt) const {
  double GND_DELTA = m_scaledPixelWidth;
  // Partial of line, sample wrt X, Y, Z
  double x = groundPt.x;
  double y = groundPt.y;
  double z = groundPt.z;

  csm::ImageCoord ipB = groundToImage(groundPt);
  csm::ImageCoord ipX = groundToImage(csm::EcefCoord(x + GND_DELTA, y, z));
  csm::ImageCoord ipY = groundToImage(csm::EcefCoord(x, y + GND_DELTA, z));
  csm::ImageCoord ipZ = groundToImage(csm::EcefCoord(x, y, z + GND_DELTA));

  std::vector<double> partials(6, 0.0);
  partials[0] = (ipX.line - ipB.line) / GND_DELTA;
  partials[3] = (ipX.samp - ipB.samp) / GND_DELTA;
  partials[1] = (ipY.line - ipB.line) / GND_DELTA;
  partials[4] = (ipY.samp - ipB.samp) / GND_DELTA;
  partials[2] = (ipZ.line - ipB.line) / GND_DELTA;
  partials[5] = (ipZ.samp - ipB.samp) / GND_DELTA;

  return partials;
}

const csm::CorrelationModel& UsgsAstroSarSensorModel::getCorrelationModel()
    const {
  return _NO_CORR_MODEL;
}

vector<double> UsgsAstroSarSensorModel::getUnmodeledCrossCovariance(
    const csm::ImageCoord& pt1, const csm::ImageCoord& pt2) const {
  return vector<double>(4, 0.0);
}

csm::EcefCoord UsgsAstroSarSensorModel::getReferencePoint() const {
  return m_referencePointXyz;
}

void UsgsAstroSarSensorModel::setReferencePoint(
    const csm::EcefCoord& groundPt) {
  m_referencePointXyz = groundPt;
}

int UsgsAstroSarSensorModel::getNumParameters() const { return NUM_PARAMETERS; }

string UsgsAstroSarSensorModel::getParameterName(int index) const {
  return PARAMETER_NAME[index];
}

string UsgsAstroSarSensorModel::getParameterUnits(int index) const {
  return "m";
}

bool UsgsAstroSarSensorModel::hasShareableParameters() const { return false; }

bool UsgsAstroSarSensorModel::isParameterShareable(int index) const {
  return false;
}

csm::SharingCriteria UsgsAstroSarSensorModel::getParameterSharingCriteria(
    int index) const {
  return csm::SharingCriteria();
}

double UsgsAstroSarSensorModel::getParameterValue(int index) const {
  return m_currentParameterValue[index];
}

void UsgsAstroSarSensorModel::setParameterValue(int index, double value) {
  m_currentParameterValue[index] = value;
}

csm::param::Type UsgsAstroSarSensorModel::getParameterType(int index) const {
  return m_parameterType[index];
}

void UsgsAstroSarSensorModel::setParameterType(int index,
                                               csm::param::Type pType) {
  m_parameterType[index] = pType;
}

double UsgsAstroSarSensorModel::getParameterCovariance(int index1,
                                                       int index2) const {
  return m_covariance[index1 * NUM_PARAMETERS + index2];
}

void UsgsAstroSarSensorModel::setParameterCovariance(int index1, int index2,
                                                     double covariance)

{
  m_covariance[index1 * NUM_PARAMETERS + index2] = covariance;
}

int UsgsAstroSarSensorModel::getNumGeometricCorrectionSwitches() const {
  return 0;
}

string UsgsAstroSarSensorModel::getGeometricCorrectionName(int index) const {
  throw csm::Error(csm::Error::INDEX_OUT_OF_RANGE, "Index is out of range.",
                   "UsgsAstroSarSensorModel::getGeometricCorrectionName");
}

void UsgsAstroSarSensorModel::setGeometricCorrectionSwitch(
    int index, bool value, csm::param::Type pType) {
  throw csm::Error(csm::Error::INDEX_OUT_OF_RANGE, "Index is out of range.",
                   "UsgsAstroSarSensorModel::setGeometricCorrectionSwitch");
}

bool UsgsAstroSarSensorModel::getGeometricCorrectionSwitch(int index) const {
  throw csm::Error(csm::Error::INDEX_OUT_OF_RANGE, "Index is out of range.",
                   "UsgsAstroSarSensorModel::getGeometricCorrectionSwitch");
}

vector<double> UsgsAstroSarSensorModel::getCrossCovarianceMatrix(
    const csm::GeometricModel& comparisonModel, csm::param::Set pSet,
    const csm::GeometricModel::GeometricModelList& otherModels) const {
  // Return covariance matrix
  if (&comparisonModel == this) {
    vector<int> paramIndices = getParameterSetIndices(pSet);
    int numParams = paramIndices.size();
    vector<double> covariances(numParams * numParams, 0.0);
    for (int i = 0; i < numParams; i++) {
      for (int j = 0; j < numParams; j++) {
        covariances[i * numParams + j] =
            getParameterCovariance(paramIndices[i], paramIndices[j]);
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

csm::Version UsgsAstroSarSensorModel::getVersion() const {
  return csm::Version(1, 0, 0);
}

string UsgsAstroSarSensorModel::getModelName() const {
  return _SENSOR_MODEL_NAME;
}

string UsgsAstroSarSensorModel::getPedigree() const { return "USGS_SAR"; }

string UsgsAstroSarSensorModel::getImageIdentifier() const {
  return m_imageIdentifier;
}

void UsgsAstroSarSensorModel::setImageIdentifier(const string& imageId,
                                                 csm::WarningList* warnings) {
  m_imageIdentifier = imageId;
}

string UsgsAstroSarSensorModel::getSensorIdentifier() const {
  return m_sensorIdentifier;
}

string UsgsAstroSarSensorModel::getPlatformIdentifier() const {
  return m_platformIdentifier;
}

string UsgsAstroSarSensorModel::getCollectionIdentifier() const {
  return m_collectionIdentifier;
}

string UsgsAstroSarSensorModel::getTrajectoryIdentifier() const {
  return m_trajectoryIdentifier;
}

string UsgsAstroSarSensorModel::getSensorType() const { return "SAR"; }

string UsgsAstroSarSensorModel::getSensorMode() const { return "STRIP"; }

string UsgsAstroSarSensorModel::getReferenceDateAndTime() const {
  csm::ImageCoord referencePointImage = groundToImage(m_referencePointXyz);
  double relativeTime = getImageTime(referencePointImage);
  time_t ephemTime = m_centerEphemerisTime + relativeTime;
  struct tm t = {0};  // Initalize to all 0's
  t.tm_year = 100;    // This is year-1900, so 100 = 2000
  t.tm_mday = 1;
  time_t timeSinceEpoch = mktime(&t);
  time_t finalTime = ephemTime + timeSinceEpoch;
  char buffer[16];
  strftime(buffer, 16, "%Y%m%dT%H%M%S", localtime(&finalTime));
  buffer[15] = '\0';

  return buffer;
}

csm::Ellipsoid UsgsAstroSarSensorModel::getEllipsoid() const {
  return csm::Ellipsoid(m_majorAxis, m_minorAxis);
}

void UsgsAstroSarSensorModel::setEllipsoid(const csm::Ellipsoid& ellipsoid) {
  m_majorAxis = ellipsoid.getSemiMajorRadius();
  m_minorAxis = ellipsoid.getSemiMinorRadius();
}

void UsgsAstroSarSensorModel::determineSensorCovarianceInImageSpace(
    csm::EcefCoord& gp, double sensor_cov[4]) const {
  int i, j, totalAdjParams;
  totalAdjParams = getNumParameters();

  std::vector<csm::RasterGM::SensorPartials> sensor_partials =
      computeAllSensorPartials(gp);

  sensor_cov[0] = 0.0;
  sensor_cov[1] = 0.0;
  sensor_cov[2] = 0.0;
  sensor_cov[3] = 0.0;

  for (i = 0; i < totalAdjParams; i++) {
    for (j = 0; j < totalAdjParams; j++) {
      sensor_cov[0] += sensor_partials[i].first * getParameterCovariance(i, j) *
                       sensor_partials[j].first;
      sensor_cov[1] += sensor_partials[i].second *
                       getParameterCovariance(i, j) * sensor_partials[j].first;
      sensor_cov[2] += sensor_partials[i].first * getParameterCovariance(i, j) *
                       sensor_partials[j].second;
      sensor_cov[3] += sensor_partials[i].second *
                       getParameterCovariance(i, j) * sensor_partials[j].second;
    }
  }
}

std::vector<double> UsgsAstroSarSensorModel::getRangeCoefficients(
    double time) const {
  int numCoeffs = m_scaleConversionCoefficients.size();
  std::vector<double> interpCoeffs;

  double endTime = m_scaleConversionTimes.back();
  if ((numCoeffs / 4) > 1) {
    double coefficients[4];
    double dtEphem = (endTime - m_scaleConversionTimes[0]) /
                     (m_scaleConversionCoefficients.size() / 4);
    lagrangeInterp(m_scaleConversionCoefficients.size() / 4,
                   &m_scaleConversionCoefficients[0], m_scaleConversionTimes[0],
                   dtEphem, time, 4, 8, coefficients);
    interpCoeffs.push_back(coefficients[0]);
    interpCoeffs.push_back(coefficients[1]);
    interpCoeffs.push_back(coefficients[2]);
    interpCoeffs.push_back(coefficients[3]);
  } else {
    interpCoeffs.push_back(m_scaleConversionCoefficients[0]);
    interpCoeffs.push_back(m_scaleConversionCoefficients[1]);
    interpCoeffs.push_back(m_scaleConversionCoefficients[2]);
    interpCoeffs.push_back(m_scaleConversionCoefficients[3]);
  }
  return interpCoeffs;
}

csm::EcefVector UsgsAstroSarSensorModel::getSunPosition(
    const double imageTime) const {
  int numSunPositions = m_sunPosition.size();
  int numSunVelocities = m_sunVelocity.size();
  csm::EcefVector sunPosition = csm::EcefVector();

  // If there are multiple positions, use Lagrange interpolation
  if ((numSunPositions / 3) > 1) {
    double sunPos[3];
    double endTime = m_t0Ephem + m_nLines * m_exposureDuration;
    double sun_dtEphem = (endTime - m_t0Ephem) / (numSunPositions / 3);
    lagrangeInterp(numSunPositions / 3, &m_sunPosition[0], m_t0Ephem,
                   sun_dtEphem, imageTime, 3, 8, sunPos);
    sunPosition.x = sunPos[0];
    sunPosition.y = sunPos[1];
    sunPosition.z = sunPos[2];
  } else if ((numSunVelocities / 3) >= 1) {
    // If there is one position triple with at least one velocity triple
    //  then the illumination direction is calculated via linear extrapolation.
    sunPosition.x = (imageTime * m_sunVelocity[0] + m_sunPosition[0]);
    sunPosition.y = (imageTime * m_sunVelocity[1] + m_sunPosition[1]);
    sunPosition.z = (imageTime * m_sunVelocity[2] + m_sunPosition[2]);
  } else {
    // If there is one position triple with no velocity triple, then the
    //  illumination direction is the difference of the original vectors.
    sunPosition.x = m_sunPosition[0];
    sunPosition.y = m_sunPosition[1];
    sunPosition.z = m_sunPosition[2];
  }
  return sunPosition;
}

double UsgsAstroSarSensorModel::getValue(
    int index, const std::vector<double>& adjustments) const {
  return m_currentParameterValue[index] + adjustments[index];
}
