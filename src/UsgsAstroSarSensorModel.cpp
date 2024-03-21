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

/**
 * @brief Constructs the sensor model state from ISD (Image Support Data) for SAR sensors.
 *
 * @description Parses the ISD to extract necessary information for constructing
 * the state of a SAR sensor model. This includes setting up the sensor,
 * platform, and image identifiers, geometric properties, and other SAR-specific
 * parameters. The function also handles linear interpolation of instrument and
 * sun states (positions and velocities) based on provided ephemeris times.
 * Warnings are generated for missing or invalid data.
 *
 * @param imageSupportData The ISD in string format containing sensor
 *                         and image acquisition details.
 * @param warnings Pointer to a list for recording any warnings encountered
 *                 during the state construction.
 * @return A string representing the constructed state in JSON format.
 */
string UsgsAstroSarSensorModel::constructStateFromIsd(
    const string imageSupportData, csm::WarningList* warnings) {
  MESSAGE_LOG("UsgsAstroSarSensorModel constructing state from ISD, with {}",
              imageSupportData);
  json isd = json::parse(imageSupportData);
  json state = {};

  std::shared_ptr<csm::WarningList> parsingWarnings(new csm::WarningList);

  // Zero computed state values
  state["m_referencePointXyz"] = vector<double>(3, 0.0);

  state["m_modelName"] = ale::getSensorModelName(isd);
  state["m_imageIdentifier"] = ale::getImageId(isd);
  state["m_sensorName"] = ale::getSensorName(isd);
  state["m_platformName"] = ale::getPlatformName(isd);

  MESSAGE_LOG(
      "m_modelName: {} "
      "m_imageIdentifier: {} "
      "m_sensorName: {} "
      "m_platformName: {} ",
      state["m_modelName"].dump(), state["m_imageIdentifier"].dump(),
      state["m_sensorName"].dump(), state["m_platformName"].dump());
  
  state["m_nLines"] = ale::getTotalLines(isd);
  state["m_nSamples"] = ale::getTotalSamples(isd);
  MESSAGE_LOG(
      "m_nLines: {} "
      "m_nSamples: {} ",
      state["m_nLines"].dump(), state["m_nSamples"].dump());

  ale::States inst_state = ale::getInstrumentPosition(isd);
  std::vector<double> ephemTime = inst_state.getTimes();
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

  // Make the Sun positions and velocities be in target body coordinates
  ale::States sunState = ale::getSunPosition(isd);
  ale::Orientations j2000_to_target = ale::getBodyRotation(isd);
  
  std::vector<double> sunPositions = {};
  std::vector<double> sunVelocities = {};
  for (int i = 0; i < ephemTime.size(); i++) {

    ale::State j2000SunState = sunState.getState(ephemTime[i], ale::SPLINE);
    ale::State rotatedSunState =
        j2000_to_target.rotateStateAt(ephemTime[i], j2000SunState, ale::SLERP);
    // ALE operates in km and we want m
    sunPositions.push_back(rotatedSunState.position.x * 1000);
    sunPositions.push_back(rotatedSunState.position.y * 1000);
    sunPositions.push_back(rotatedSunState.position.z * 1000);
    sunVelocities.push_back(rotatedSunState.velocity.x * 1000);
    sunVelocities.push_back(rotatedSunState.velocity.y * 1000);
    sunVelocities.push_back(rotatedSunState.velocity.z * 1000);
  }
  state["m_sunPosition"] = sunPositions;
  state["m_sunVelocity"] = sunVelocities;

  state["m_startingEphemerisTime"] = ale::getStartingTime(isd);
  state["m_centerEphemerisTime"] = ale::getCenterTime(isd);
  state["m_endingEphemerisTime"] = getEndingTime(isd, parsingWarnings.get());
  MESSAGE_LOG("m_startingEphemerisTime: {} "
              "m_centerEphemerisTime: {} "
              "m_endingEphemerisTime: {} ",
              state["m_startingEphemerisTime"].dump(),
              state["m_centerEphemerisTime"].dump(),
              state["m_endingEphemerisTime"].dump()
              )

  state["m_exposureDuration"] = getExposureDuration(isd, parsingWarnings.get());

  state["m_currentParameterValue"] = vector<double>(NUM_PARAMETERS, 0.0);

  // SAR specific values
  state["m_scaledPixelWidth"] = getScaledPixelWidth(isd, parsingWarnings.get());
  state["m_scaleConversionCoefficients"] =
      getScaleConversionCoefficients(isd, parsingWarnings.get());
  state["m_scaleConversionTimes"] =
      getScaleConversionTimes(isd, parsingWarnings.get());
  state["m_wavelength"] = getWavelength(isd, parsingWarnings.get());
  state["m_lookDirection"] = getLookDirection(isd, parsingWarnings.get());

  // Process positions and velocities
  ale::State interpInstState, rotatedInstState;
  std::vector<double> instPositions = {};
  std::vector<double> instVelocities = {};
  for (int i = 0; i < ephemTime.size(); i++) {
    
    interpInstState = inst_state.getState(ephemTime[i], ale::SPLINE);
    rotatedInstState =
      j2000_to_target.rotateStateAt(ephemTime[i], interpInstState, ale::SLERP);
    
    // ALE operates in km and we want m
    instPositions.push_back(rotatedInstState.position.x * 1000);
    instPositions.push_back(rotatedInstState.position.y * 1000);
    instPositions.push_back(rotatedInstState.position.z * 1000);
    instVelocities.push_back(rotatedInstState.velocity.x * 1000);
    instVelocities.push_back(rotatedInstState.velocity.y * 1000);
    instVelocities.push_back(rotatedInstState.velocity.z * 1000);

  }
  state["m_positions"] = instPositions;
  state["m_velocities"] = instVelocities;

  // Get radii
  // ALE operates in km and we want m
  state["m_minorAxis"] = ale::getSemiMinorRadius(isd) * 1000;
  state["m_majorAxis"] = ale::getSemiMajorRadius(isd) * 1000;
  MESSAGE_LOG("m_minorAxis: {}"
              "m_majorAxis: {}",
              state["m_minorAxis"].dump(), state["m_majorAxis"].dump());
  
  // set identifiers
  state["m_platformIdentifier"] = ale::getPlatformName(isd);
  state["m_sensorIdentifier"] = ale::getSensorName(isd);
  MESSAGE_LOG(
      "m_platformIdentifier: {}"
      "m_sensorIdentifier: {}",
      state["m_platformIdentifier"].dump(), state["m_sensorIdentifier"].dump());

  // Get reference_height
  state["m_minElevation"] = ale::getMinHeight(isd);
  state["m_maxElevation"] = ale::getMaxHeight(isd);
  MESSAGE_LOG(
      "m_minElevation: {}"
      "m_maxElevation: {}",
      state["m_minElevation"].dump(), state["m_maxElevation"].dump());

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
    MESSAGE_LOG(message);
    throw csm::Error(csm::Error::SENSOR_MODEL_NOT_CONSTRUCTIBLE, message,
                     "UsgsAstroSarSensorModel::constructStateFromIsd");
  }

  // The state data will still be updated when a sensor model is created since
  // some state data is not in the ISD and requires a SM to compute them.
  return state.dump();
}

/**
 * @brief Retrieves the model name from the sensor model state JSON.
 *
 * @description Parses the model state JSON string to extract the sensor model name.
 * Throws an error if the model name is missing from the state or if the model
 * is not supported.
 *
 * @param model_state The sensor model state in JSON string format.
 * @return The name of the sensor model.
 * @throws csm::Error if "m_modelName" is missing or if the sensor model is not supported.
 */
string UsgsAstroSarSensorModel::getModelNameFromModelState(
    const string& model_state) {
  // Parse the string to JSON
  auto j = stateAsJson(model_state);
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
  if (model_name != _SENSOR_MODEL_NAME) {
    csm::Error::ErrorType aErrorType = csm::Error::SENSOR_MODEL_NOT_SUPPORTED;
    string aMessage = "Sensor model not supported.";
    string aFunction = "UsgsAstroSarSensorModel::getModelNameFromModelState()";
    csm::Error csmErr(aErrorType, aMessage, aFunction);
    throw(csmErr);
  }
  return model_name;
}

/**
 * @brief Constructor for UsgsAstroSarSensorModel.
 *
 * @description Initializes a new instance of the UsgsAstroSarSensorModel class,
 * setting up the initial state and logging the creation of the model.
 */
UsgsAstroSarSensorModel::UsgsAstroSarSensorModel() {
  MESSAGE_LOG("Constructing UsgsAstroSarSensorModel");
  reset();
}

/**
 * @brief Resets the sensor model to its default state.
 *
 * @description Resets all member variables of the UsgsAstroSarSensorModel to their
 * default values. This includes setting identifiers to "Unknown", setting
 * numerical values to 0 or appropriate defaults, clearing vectors, and
 * preparing the model for a fresh state configuration.
 */
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

/**
 * @brief Replaces the current model state with a new one.
 * 
 * @description This function takes a JSON string representation of a sensor
 * model's state, parses it, and updates the current model state accordingly.
 * It resets the model before applying the new state to ensure a clean
 * update. Throws an error if the look direction is unrecognized.
 * 
 * @param argState The new model state in JSON string format.
 * @throws csm::Error If the look direction from the state is invalid.
 */
void UsgsAstroSarSensorModel::replaceModelState(const string& argState) {
  reset();

  MESSAGE_LOG("Replacing model state with: {}", argState);
  auto stateJson = stateAsJson(argState);

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

/**
 * @brief Retrieves the current model state as a JSON string.
 * 
 * @description Serializes the current state of the sensor model into a JSON
 * string. This includes all relevant parameters and configurations that
 * define the sensor model's state. If the look direction is not correctly
 * defined in the model, it throws an error.
 * 
 * @return A JSON string representing the current state of the model.
 * @throws csm::Error If the look direction is not properly set in the model.
 */
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

  // Use dump(2) to avoid creating the model string as a single long line
  std::string stateString = getModelName() + "\n" + state.dump(2);
  return stateString;
}

/**
 * @brief Applies a geometric transformation to the sensor model state.
 * 
 * @description This function applies a rotation and translation to the positions
 * and velocities within the sensor model's state. The transformation is also
 * applied to the model's reference point. Note that the Sun's position and velocity are not
 * altered by this transformation due to their vast distance.
 * 
 * @param r The rotation to be applied.
 * @param t The translation to be applied.
 * @param stateString The JSON string representation of the model's state
 * to which the transformation will be applied. This string is modified in place.
 */
void UsgsAstroSarSensorModel::applyTransformToState(ale::Rotation const& r, ale::Vec3d const& t,
                                                   std::string& stateString) {

  nlohmann::json j = stateAsJson(stateString);

  // Apply rotation and translation to positions
  std::vector<double> positions = j["m_positions"].get<std::vector<double>>();
  applyRotationTranslationToXyzVec(r, t, positions);
  j["m_positions"] = positions;
  
  // Note that the SAR sensor does not have quaternions

  // Apply rotation to velocities. The translation does not get applied.
  ale::Vec3d zero_t(0, 0, 0);
  std::vector<double> velocities = j["m_velocities"].get<std::vector<double>>();
  applyRotationTranslationToXyzVec(r, zero_t, velocities);
  j["m_velocities"] = velocities;

  // Apply the transform to the reference point
  std::vector<double> refPt = j["m_referencePointXyz"];
  applyRotationTranslationToXyzVec(r, t, refPt);
  j["m_referencePointXyz"] = refPt;

  // We do not change the Sun position or velocity. The idea is that
  // the Sun is so far, that minor camera adjustments won't affect
  // where the Sun is.

  // Update the state string
  stateString = getModelNameFromModelState(stateString) + "\n" + j.dump(2);
}

/**
 * @brief Computes the image coordinates (line and sample) for a given ground
 * point using the current model state without adjustments.
 * 
 * @description Calculates the image coordinates that correspond to a given
 * ground point.
 * 
 * @param groundPt The ground point in ECEF coordinates.
 * @param desiredPrecision The desired precision for the ground-to-image
 * calculation.
 * @param achievedPrecision Pointer to a double to store the achieved precision.
 * @param warnings Pointer to a warning list to capture any warnings.
 * @return The computed image coordinates.
 */
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

/**
 * @brief Computes the image coordinates (line and sample) for a given ground
 * point using the current model state with adjustments.
 * 
 * @description Calculates the image coordinates that correspond to a given
 * ground point, taking into account any adjustments specified. This function
 * finds the time of the closest approach to the ground point and the
 * corresponding slant range, then calculates the ground range using a
 * polynomial defined by the range coefficient set. It accounts for doppler
 * shift to accurately determine the position in the image.
 * 
 * @param groundPt The ground point in ECEF coordinates.
 * @param adj Adjustments to be applied to the sensor model state.
 * @param desiredPrecision The desired precision for the ground-to-image
 * calculation.
 * @param achievedPrecision Pointer to a double to store the achieved precision.
 * @param warnings Pointer to a warning list to capture any warnings.
 * @return The computed image coordinates.
 * @throws csm::Error If the computation fails for any reason.
 */
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
    double sample = groundRange / m_scaledPixelWidth + 0.5;
    return csm::ImageCoord(line, sample);
  } catch (std::exception& error) {
    std::string message = "Could not calculate groundToImage, with error [";
    message += error.what();
    message += "]";
    MESSAGE_LOG(message);
    throw csm::Error(csm::Error::UNKNOWN_ERROR, message, "groundToImage");
  }
}

/**
 * @brief Calculates the Doppler shift frequency for a given ground point.
 * 
 * @description This function computes the Doppler shift frequency, which is
 * necessary to find the time of the closest approach of the SAR satellite to
 * the given ground point. The Doppler shift is calculated using the relative
 * velocity between the spacecraft and the ground point, the wavelength of the
 * radar signal, and the slant range distance.
 * 
 * @param groundPt The ground point in ECEF coordinates for which the Doppler
 * shift is calculated.
 * @param tolerance The tolerance within which the root-finding algorithm should
 * operate to find the time of closest approach.
 * @param adj Adjustments to be applied to the sensor model state.
 * @return The time at which the Doppler shift frequency is minimized, indicating
 * the closest approach to the ground point.
 */
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

/**
 * @brief Calculates the slant range distance between the sensor and a ground point.
 * 
 * @description The slant range is the straight-line distance from the satellite
 * to the ground point at a given time.
 * 
 * @param surfPt The ground point in ECEF coordinates.
 * @param time The time at which the slant range is calculated.
 * @param adj Adjustments to be applied to the sensor model state.
 * @return The slant range distance between the satellite and the ground point.
 */
double UsgsAstroSarSensorModel::slantRange(csm::EcefCoord surfPt, double time,
                                           std::vector<double> adj) const {
  MESSAGE_LOG("Calculating slant range with: {}, {}, {}, and time {}.",
              surfPt.x, surfPt.y, surfPt.z, time);
  csm::EcefVector surfVec(surfPt.x, surfPt.y, surfPt.z);
  csm::EcefVector spacecraftPosition = getAdjustedSpacecraftPosition(time, adj);
  return norm(spacecraftPosition - surfVec);
}

/**
 * @brief Converts slant range distance to ground range distance.
 * 
 * @description This function finds the ground range distance corresponding to a
 * given slant range distance for a SAR satellite at a specific time. The
 * ground range is the projection of the slant range onto the ground (Earth's
 * surface).
 * 
 * @param groundPt The ground point in ECEF coordinates.
 * @param time The time at which the conversion is performed.
 * @param slantRange The slant range distance to be converted.
 * @param tolerance The tolerance within which the solution for the ground range
 * should be found.
 * @return The ground range distance corresponding to the given slant range.
 */
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

/**
 * @brief Converts ground range distance to slant range distance.
 * 
 * @description This function calculates the slant range distance from a given
 * ground range distance using a polynomial defined by the range coefficients.
 * The slant range is the straight-line distance from the satellite to a point
 * on the ground, whereas the ground range is the distance along the ground.
 * 
 * @param groundRange The ground range distance to be converted to slant range.
 * @param coeffs The coefficients of the polynomial used for the conversion.
 * @return The slant range distance corresponding to the given ground range.
 */
double UsgsAstroSarSensorModel::groundRangeToSlantRange(
    double groundRange, const std::vector<double>& coeffs) const {
  return evaluatePolynomial(coeffs, groundRange);
}

/**
 * @brief Converts a ground point with covariance to an image coordinate with covariance.
 * 
 * @description This function converts a ground point, specified in ECEF
 * coordinates with associated covariance, into an image coordinate with
 * covariance. It utilizes the groundToImage function for the conversion and
 * then propagates the uncertainty from the ground point through the
 * transformation to compute the covariance in the image space.
 * 
 * @param groundPt A ground point with covariance in ECEF coordinates.
 * @param desiredPrecision The desired precision for the conversion.
 * @param achievedPrecision Pointer to store the achieved precision.
 * @param warnings Warning list to capture any issues during the process.
 * @return An image coordinate with covariance resulting from the conversion.
 */
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

/**
 * @brief Converts an image coordinate to a ground point in ECEF coordinates.
 * 
 * @description This function calculates the ground point corresponding to a given
 * image coordinate and height above the ellipsoid. The process involves
 * determining the time of the image acquisition and the ground range, then
 * converting the ground range to slant range, and finally solving for the
 * ground point position using the spacecraft position and velocity.
 * 
 * @param imagePt The image coordinate (line, sample) for which to find the corresponding ground point.
 * @param height The height above the ellipsoid at which to find the ground point.
 * @param desiredPrecision The desired precision for the conversion.
 * @param achievedPrecision Pointer to store the achieved precision of the conversion.
 * @param warnings Warning list to capture any issues during the process.
 * @return The corresponding ground point in ECEF coordinates.
 */
csm::EcefCoord UsgsAstroSarSensorModel::imageToGround(
    const csm::ImageCoord& imagePt, double height, double desiredPrecision,
    double* achievedPrecision, csm::WarningList* warnings) const {
  MESSAGE_LOG("Calculating imageToGround with: {}, {}, {}, {}", imagePt.samp,
              imagePt.line, height, desiredPrecision);
  double time =
    m_startingEphemerisTime + (imagePt.line - 0.5) * m_exposureDuration;
  double groundRange = (imagePt.samp - 0.5) * m_scaledPixelWidth;
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
  MESSAGE_LOG("Nadir position component: {}", nadirComp);
  
  // Iterate to find proper radius value. Limit the number of iterations
  // because the desired precision may not drop beyond 1e-10 no matter
  // how many iterations one does. As it was observed, this in fact
  // converges at the first iteration; this is expected for a spherical
  // datum and not for an ellipsoidal one.
  double pointRadius = m_majorAxis + height;
  double radiusSqr;
  double pointHeight;
  csm::EcefVector groundVec;
  for (int it = 0; it < 10; it++) {
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

    if (fabs(pointHeight - height) <= desiredPrecision)
      break;
  }

  if (achievedPrecision)
    *achievedPrecision = fabs(pointHeight - height);

  MESSAGE_LOG("Iteration ground point: {}, {}, {}", groundVec.x, groundVec.y, groundVec.z);

  csm::EcefCoord groundPt(groundVec.x, groundVec.y, groundVec.z);

  return groundPt;
}

/**
 * @brief Converts an image coordinate with covariance to a ground point with
 *        covariance.
 * 
 * @description Converts an image coordinate with covariance to a ground point
 * in ECEF coordinates, considering the height and its variance,
 * sensor modeling, and unmodeled errors for covariance computation.
 * 
 * @param imagePt Image coordinate with covariance.
 * @param height Height above the ellipsoid for the ground point.
 * @param heightVariance Variance of the height above the ellipsoid.
 * @param desiredPrecision Desired precision for the conversion.
 * @param achievedPrecision Pointer to store the achieved precision.
 * @param warnings Warning list for issues during the process.
 * @return Ground point in ECEF coordinates with covariance.
 */
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

/**
 * @brief Computes proximate imaging locus for an image point and ground point.
 * 
 * @description Calculates the closest point on the imaging locus to a ground
 * point, defining the imaging locus as the intersection of the
 * Doppler cone with the Earth's surface.
 * 
 * @param imagePt Image coordinate (line, sample).
 * @param groundPt Ground point in ECEF coordinates.
 * @param desiredPrecision Desired precision for the conversion.
 * @param achievedPrecision Pointer to store the achieved precision.
 * @param warnings Warning list for issues during the process.
 * @return Locus of points forming the proximate imaging locus, including
 *         the direction vector.
 */
csm::EcefLocus UsgsAstroSarSensorModel::imageToProximateImagingLocus(
    const csm::ImageCoord& imagePt, const csm::EcefCoord& groundPt,
    double desiredPrecision, double* achievedPrecision,
    csm::WarningList* warnings) const {
  // Compute the slant range
  double time =
      m_startingEphemerisTime + (imagePt.line - 0.5) * m_exposureDuration;
  double groundRange = (imagePt.samp - 0.5) * m_scaledPixelWidth;
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

  if (achievedPrecision)
    *achievedPrecision = 0.0;

  return csm::EcefLocus(closestVec.x, closestVec.y, closestVec.z, tangent.x,
                        tangent.y, tangent.z);
}

/**
 * @brief Computes the remote imaging locus for an image point.
 * 
 * @description Calculates the imaging locus for an image coordinate by
 * determining the direction from the sensor position through the
 * image point towards the ground.
 * 
 * @param imagePt Image coordinate (line, sample) for locus calculation.
 * @param desiredPrecision Desired precision for the calculation.
 * @param achievedPrecision Pointer to store achieved precision.
 * @param warnings Warning list for issues during the process.
 * @return Locus of points forming the remote imaging locus, including
 *         the direction vector.
 */
csm::EcefLocus UsgsAstroSarSensorModel::imageToRemoteImagingLocus(
    const csm::ImageCoord& imagePt, double desiredPrecision,
    double* achievedPrecision, csm::WarningList* warnings) const {

  // Find the sensor coordinates
  csm::EcefCoord sensorPt = getSensorPosition(imagePt);

  // Compute the intersection of the ray traced through the
  // current pixel with the datum.  
  double height = 0.0; 
  csm::EcefCoord groundPt = imageToGround(imagePt, height, desiredPrecision,
                                        achievedPrecision, warnings);

  // Normalized direction from camera to ground
  csm::EcefVector dir
    = normalized(csm::EcefVector(groundPt.x - sensorPt.x,
                                 groundPt.y - sensorPt.y,
                                 groundPt.z - sensorPt.z));
                 
  return csm::EcefLocus(groundPt.x, groundPt.y, groundPt.z,
                        dir.x, dir.y, dir.z);
}

/**
 * @brief Returns the starting image coordinate.
 * 
 * @description Always returns the origin point (0.0, 0.0) as the start of the image 
 * coordinates.
 * 
 * @return The starting image coordinate.
 */
csm::ImageCoord UsgsAstroSarSensorModel::getImageStart() const {
  return csm::ImageCoord(0.0, 0.0);
}

/**
 * @brief Returns the size of the image.
 * 
 * @description Provides the dimensions of the image in terms of the number of lines 
 * and samples.
 * 
 * @return Image vector indicating the total number of lines and samples.
 */

csm::ImageVector UsgsAstroSarSensorModel::getImageSize() const {
  return csm::ImageVector(m_nLines, m_nSamples);
}

/**
 * @brief Provides the valid range of image coordinates.
 * 
 * @description Computes the range of valid image coordinates starting from the origin
 * to the end of the image based on its size.
 * 
 * @return Pair of image coordinates indicating the start and end of the valid image 
 *         range.
 */
pair<csm::ImageCoord, csm::ImageCoord>
UsgsAstroSarSensorModel::getValidImageRange() const {
  csm::ImageCoord start = getImageStart();
  csm::ImageVector size = getImageSize();
  return make_pair(
      start, csm::ImageCoord(start.line + size.line, start.samp + size.samp));
}

/**
 * @brief Gets the valid range of heights for the model.
 * 
 * @description Returns the minimum and maximum valid elevation values for the sensor 
 * model.
 * 
 * @return Pair of doubles indicating the minimum and maximum valid heights.
 */
pair<double, double> UsgsAstroSarSensorModel::getValidHeightRange() const {
  return make_pair(m_minElevation, m_maxElevation);
}

/**
 * @brief Computes the illumination direction for a ground point.
 * 
 * @description Determines the unit vector from a ground point towards the sun,
 * indicating the direction of illumination.
 * 
 * @param groundPt Ground point in ECEF coordinates.
 * @return Normalized vector pointing towards the sun from the ground point.
 */
csm::EcefVector UsgsAstroSarSensorModel::getIlluminationDirection(
    const csm::EcefCoord& groundPt) const {
  csm::EcefVector groundVec(groundPt.x, groundPt.y, groundPt.z);
  csm::EcefVector sunPosition =
      getSunPosition(getImageTime(groundToImage(groundPt)));
  csm::EcefVector illuminationDirection = normalized(groundVec - sunPosition);
  return illuminationDirection;
}

/**
 * @brief Computes the image time for a given image coordinate.
 * 
 * @description Calculates the time at which a given line in the image was acquired,
 * based on the starting ephemeris time and exposure duration.
 * 
 * @param imagePt Image coordinate (line, sample).
 * @return The time at which the image line was acquired.
 */
double UsgsAstroSarSensorModel::getImageTime(
    const csm::ImageCoord& imagePt) const {
  return m_startingEphemerisTime + (imagePt.line - 0.5) * m_exposureDuration;
}

/**
 * @brief Returns the spacecraft position at a given time without adjustments.
 * 
 * @description Retrieves the ECEF position of the spacecraft at a specific time,
 * without considering any model adjustments.
 * 
 * @param time The time at which to retrieve the spacecraft position.
 * @return Vector representing the spacecraft's position in ECEF coordinates.
 */
csm::EcefVector UsgsAstroSarSensorModel::getSpacecraftPosition(
    double time) const {
  MESSAGE_LOG("getSpacecraftPosition at {} without adjustments", time)
  csm::EcefCoord spacecraftPosition = getSensorPosition(time);
  return csm::EcefVector(spacecraftPosition.x, spacecraftPosition.y,
                         spacecraftPosition.z);
}

/**
 * @brief Returns the adjusted spacecraft position at a given time.
 * 
 * @description Retrieves the ECEF position of the spacecraft at a specific time,
 * considering the provided model adjustments.
 * 
 * @param time The time at which to retrieve the spacecraft position.
 * @param adj Vector of adjustments to apply to the model.
 * @return Vector representing the adjusted spacecraft's position in ECEF coordinates.
 */
csm::EcefVector UsgsAstroSarSensorModel::getAdjustedSpacecraftPosition(
    double time, std::vector<double> adj) const {
  MESSAGE_LOG("getSpacecraftPosition at {} with adjustments", time)
  csm::EcefCoord spacecraftPosition = getAdjustedSensorPosition(time, adj);
  return csm::EcefVector(spacecraftPosition.x, spacecraftPosition.y,
                         spacecraftPosition.z);
}

/**
 * @brief Retrieves the sensor position at a given time.
 * 
 * @description Retrieves the ECEF position of the sensor at a specific time,
 * without applying any model adjustments.
 * 
 * @param time The time at which to retrieve the sensor position.
 * @return The ECEF coordinates of the sensor position at the given time.
 */
csm::EcefCoord UsgsAstroSarSensorModel::getSensorPosition(double time) const {
  MESSAGE_LOG("getSensorPosition at {}.", time)
  csm::EcefCoord sensorPosition =
      getAdjustedSensorPosition(time, m_noAdjustments);
  return sensorPosition;
}

/**
 * @brief Retrieves the sensor position for a given image coordinate.
 * 
 * @description Calculates the sensor position in ECEF coordinates corresponding 
 * to a specific image coordinate by first determining the time of the image 
 * point and then finding the sensor position at that time.
 * 
 * @param imagePt The image coordinate for which to find the sensor position.
 * @return The ECEF coordinates of the sensor position for the given image coordinate.
 */
csm::EcefCoord UsgsAstroSarSensorModel::getSensorPosition(
    const csm::ImageCoord& imagePt) const {
  MESSAGE_LOG("getSensorPosition at {}, {}.", imagePt.samp, imagePt.line);
  double time = getImageTime(imagePt);
  return getSensorPosition(time);
}

/**
 * @brief Retrieves the adjusted sensor position at a given time.
 * 
 * @description Retrieves the ECEF position of the sensor at a specific time,
 * considering the provided model adjustments.
 * 
 * @param time The time at which to retrieve the sensor position.
 * @param adj Vector of adjustments to apply to the model.
 * @return The adjusted ECEF coordinates of the sensor position at the given time.
 */
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

/**
 * @brief Retrieves the sensor velocity for a given image coordinate.
 * 
 * @description Calculates the sensor velocity in ECEF coordinates corresponding 
 * to a specific image coordinate by first determining the time of the image 
 * point and then finding the sensor velocity at that time. No model adjustments 
 * are applied.
 * 
 * @param imagePt The image coordinate for which to find the sensor velocity.
 * @return The ECEF vector representing the sensor velocity for the given image coordinate.
 */
csm::EcefVector UsgsAstroSarSensorModel::getSensorVelocity(
    const csm::ImageCoord& imagePt) const {
  MESSAGE_LOG("getSensorVelocity at {}, {}. No adjustments.", imagePt.samp,
              imagePt.line);
  double time = getImageTime(imagePt);
  return getSensorVelocity(time);
}

/**
 * @brief Retrieves the sensor velocity at a given time.
 * 
 * @description Retrieves the ECEF velocity of the sensor at a specific time,
 * without applying any model adjustments.
 * 
 * @param time The time at which to retrieve the sensor velocity.
 * @return The ECEF vector of the sensor velocity at the given time.
 */
csm::EcefVector UsgsAstroSarSensorModel::getSensorVelocity(double time) const {
  MESSAGE_LOG("getSensorVelocity at {}. Without adjustments.", time);
  csm::EcefVector spacecraftVelocity =
      getAdjustedSensorVelocity(time, m_noAdjustments);
  return spacecraftVelocity;
}

/**
 * @brief Retrieves the adjusted sensor velocity at a given time.
 * 
 * @description Retrieves the ECEF velocity of the sensor at a specific time,
 * considering the provided model adjustments.
 * 
 * @param time The time at which to retrieve the sensor velocity.
 * @param adj Vector of adjustments to apply to the model.
 * @return The adjusted ECEF vector of the sensor velocity at the given time.
 */
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

/**
 * @brief Computes the sensor model partial derivatives for a ground point.
 * 
 * @description Computes the partial derivatives of the image coordinates with 
 * respect to the ground point coordinates, using a given ground point in ECEF 
 * coordinates, desired precision for the computation, and potential adjustments 
 * applied to the sensor model.
 * 
 * @param index Index of the model parameter for which the partial is being 
 * computed.
 * @param groundPt Ground point in ECEF coordinates.
 * @param desiredPrecision The precision desired in the computation of the 
 * partials.
 * @param achievedPrecision The precision achieved in the computation.
 * @param warnings List to accumulate any warnings.
 * @return SensorPartials object containing the partial derivatives of the image 
 * coordinates with respect to the model parameter.
 */
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

/**
 * @brief Computes the sensor model partial derivatives for an image point.
 * 
 * @description Computes the partial derivatives of the image coordinates with
 * respect to a specific sensor model parameter, given an image point, the
 * corresponding ground point, the desired precision for the computation, and
 * any adjustments applied to the sensor model.
 * 
 * @param index The index of the model parameter for which the partial is being
 * computed.
 * @param imagePt The image coordinates (line and sample).
 * @param groundPt The ground point in ECEF coordinates associated with the image
 * point.
 * @param desiredPrecision The precision desired in the computation of the
 * partials.
 * @param achievedPrecision Pointer to a variable where the achieved precision
 * of the computation will be stored. May be NULL if not needed.
 * @param warnings List to accumulate any warnings that occur during the
 * computation.
 * @return SensorPartials object containing the partial derivatives of the image
 * coordinates (line and sample) with respect to the specified model parameter.
 */
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

/**
 * @brief Computes the ground partial derivatives for a ground point.
 * 
 * @description Computes the partial derivatives of the image coordinates with 
 * respect to the ground point coordinates, using only the ground point 
 * coordinates.
 * 
 * @param groundPt Ground point in ECEF coordinates.
 * @return Vector containing the six partial derivatives of the image 
 * coordinates (line and sample) with respect to the ground coordinates (X, Y, Z).
 */
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

/**
 * @brief Returns the correlation model used by the sensor model.
 * 
 * @description Provides access to the correlation model used for error 
 * propagation. SAR sensor models do not model correlation between image points, 
 * so this returns a no-correlation model.
 * 
 * @return A reference to a CorrelationModel object representing the correlation 
 * model used by the sensor model.
 */
const csm::CorrelationModel& UsgsAstroSarSensorModel::getCorrelationModel()
    const {
  return _NO_CORR_MODEL;
}

/**
 * @brief Computes the unmodeled cross covariance between two image points.
 * 
 * @description Computes the covariance matrix between two points in an image 
 * due to unmodeled errors. Since unmodeled errors are not considered in SAR 
 * sensor models, this function returns a zero matrix.
 * 
 * @param pt1 First image coordinate.
 * @param pt2 Second image coordinate.
 * @return Vector representing a 2x2 covariance matrix (flattened) filled with 
 * zeros.
 */
vector<double> UsgsAstroSarSensorModel::getUnmodeledCrossCovariance(
    const csm::ImageCoord& pt1, const csm::ImageCoord& pt2) const {
  return vector<double>(4, 0.0);
}

/**
 * @brief Retrieves the current reference point of the model.
 * 
 * @description Returns the reference point for the sensor model in ECEF 
 * coordinates. The reference point is typically the ground point closest to 
 * the sensor model's center of projection.
 * 
 * @return The ECEF coordinates of the reference point.
 */
csm::EcefCoord UsgsAstroSarSensorModel::getReferencePoint() const {
  return m_referencePointXyz;
}

/**
 * @brief Sets the reference point for the sensor model.
 * 
 * @description Updates the sensor model's reference point using the provided 
 * ECEF coordinates.
 * 
 * @param groundPt The new reference point in ECEF coordinates.
 */
void UsgsAstroSarSensorModel::setReferencePoint(
    const csm::EcefCoord& groundPt) {
  m_referencePointXyz = groundPt;
}

/**
 * @brief Retrieves the number of parameters used by the sensor model.
 * 
 * @description Returns the total number of parameters in the sensor model.
 * 
 * @return The number of parameters.
 */
int UsgsAstroSarSensorModel::getNumParameters() const { return NUM_PARAMETERS; }

/**
 * @brief Retrieves the name of a parameter by its index.
 * 
 * @description Provides the name of the parameter corresponding to the given 
 * index.
 * 
 * @param index The index of the parameter.
 * @return The name of the parameter at the given index.
 */
string UsgsAstroSarSensorModel::getParameterName(int index) const {
  return PARAMETER_NAME[index];
}

/**
 * @brief Retrieves the units of a parameter by its index.
 * 
 * @description Provides the units of the parameter corresponding to the given 
 * index. For SAR models, all parameters are in meters.
 * 
 * @param index The index of the parameter.
 * @return The units of the parameter at the given index.
 */
string UsgsAstroSarSensorModel::getParameterUnits(int index) const {
  return "m";
}

/**
 * @brief Indicates whether the model parameters are shareable.
 * 
 * @description Determines if the model parameters can be shared between 
 * different sensor model instances. SAR sensor models do not support parameter 
 * sharing.
 * 
 * @return False, indicating parameters are not shareable.
 */
bool UsgsAstroSarSensorModel::hasShareableParameters() const { return false; }

/**
 * @brief Indicates if a specific parameter is shareable.
 * 
 * @description Determines if the specific model parameter can be shared 
 * between different sensor model instances. SAR sensor models do not support 
 * parameter sharing.
 * 
 * @param index The index of the parameter.
 * @return False, indicating the parameter is not shareable.
 */
bool UsgsAstroSarSensorModel::isParameterShareable(int index) const {
  return false;
}

/**
 * @brief Retrieves the sharing criteria for a parameter.
 * 
 * @description Provides the criteria under which a parameter can be shared 
 * between different sensor model instances. SAR sensor models do not support 
 * parameter sharing, so this returns a default SharingCriteria object.
 * 
 * @param index The index of the parameter.
 * @return A SharingCriteria object.
 */
csm::SharingCriteria UsgsAstroSarSensorModel::getParameterSharingCriteria(
    int index) const {
  return csm::SharingCriteria();
}

/**
 * @brief Retrieves the value of a parameter by its index.
 * 
 * @description Provides the value of the parameter corresponding to the given 
 * index.
 * 
 * @param index The index of the parameter.
 * @return The value of the parameter at the given index.
 */
double UsgsAstroSarSensorModel::getParameterValue(int index) const {
  return m_currentParameterValue[index];
}

/**
 * @brief Sets the value of a parameter by its index.
 * 
 * @description Updates the value of the parameter corresponding to the given 
 * index.
 * 
 * @param index The index of the parameter.
 * @param value The new value for the parameter.
 */
void UsgsAstroSarSensorModel::setParameterValue(int index, double value) {
  m_currentParameterValue[index] = value;
}

/**
 * @brief Retrieves the type of a parameter by its index.
 * 
 * @description Provides the type of the parameter corresponding to the given 
 * index.
 * 
 * @param index The index of the parameter.
 * @return The type of the parameter at the given index.
 */
csm::param::Type UsgsAstroSarSensorModel::getParameterType(int index) const {
  return m_parameterType[index];
}

/**
 * @brief Sets the type of a parameter by its index.
 * 
 * @description Updates the type of the parameter corresponding to the given 
 * index.
 * 
 * @param index The index of the parameter.
 * @param pType The new type for the parameter.
 */
void UsgsAstroSarSensorModel::setParameterType(int index,
                                               csm::param::Type pType) {
  m_parameterType[index] = pType;
}

/**
 * @brief Retrieves the covariance between two parameters.
 * 
 * @description Provides the covariance between the parameters corresponding to 
 * the given indices.
 * 
 * @param index1 The index of the first parameter.
 * @param index2 The index of the second parameter.
 * @return The covariance between the two parameters.
 */
double UsgsAstroSarSensorModel::getParameterCovariance(int index1,
                                                       int index2) const {
  return m_covariance[index1 * NUM_PARAMETERS + index2];
}

/**
 * @brief Sets the covariance between two parameters.
 * 
 * @description Updates the covariance between the parameters corresponding to 
 * the given indices.
 * 
 * @param index1 The index of the first parameter.
 * @param index2 The index of the second parameter.
 * @param covariance The new covariance value.
 */
void UsgsAstroSarSensorModel::setParameterCovariance(int index1, int index2,
                                                     double covariance)
{
  m_covariance[index1 * NUM_PARAMETERS + index2] = covariance;
}

/**
 * @brief Retrieves the number of geometric correction switches.
 * 
 * @description Provides the total number of geometric correction switches in 
 * the sensor model. SAR sensor models do not use geometric corrections.
 * 
 * @return 0, indicating there are no geometric correction switches.
 */
int UsgsAstroSarSensorModel::getNumGeometricCorrectionSwitches() const {
  return 0;
}

/**
 * @brief Retrieves the name of a geometric correction by its index.
 * 
 * @description Provides the name of the geometric correction corresponding to 
 * the given index. SAR sensor models do not use geometric corrections, so this 
 * function throws an error if called.
 * 
 * @param index The index of the geometric correction.
 * @return This function will throw an INDEX_OUT_OF_RANGE error.
 */
string UsgsAstroSarSensorModel::getGeometricCorrectionName(int index) const {
  throw csm::Error(csm::Error::INDEX_OUT_OF_RANGE, "Index is out of range.",
                   "UsgsAstroSarSensorModel::getGeometricCorrectionName");
}

/**
 * @brief Sets a geometric correction switch by its index.
 * 
 * @description Updates the state of the geometric correction switch 
 * corresponding to the given index. SAR sensor models do not use geometric 
 * corrections, so this function throws an error if called.
 * 
 * @param index The index of the geometric correction switch.
 * @param value The new state for the switch.
 * @param pType The parameter type for the switch.
 */
void UsgsAstroSarSensorModel::setGeometricCorrectionSwitch(
    int index, bool value, csm::param::Type pType) {
  throw csm::Error(csm::Error::INDEX_OUT_OF_RANGE, "Index is out of range.",
                   "UsgsAstroSarSensorModel::setGeometricCorrectionSwitch");
}

/**
 * @brief Retrieves the state of a geometric correction switch by its index.
 * 
 * @description Provides the state of the geometric correction switch 
 * corresponding to the given index. SAR sensor models do not use geometric 
 * corrections, so this function throws an error if called.
 * 
 * @param index The index of the geometric correction switch.
 * @return This function will throw an INDEX_OUT_OF_RANGE error.
 */
bool UsgsAstroSarSensorModel::getGeometricCorrectionSwitch(int index) const {
  throw csm::Error(csm::Error::INDEX_OUT_OF_RANGE, "Index is out of range.",
                   "UsgsAstroSarSensorModel::getGeometricCorrectionSwitch");
}

/**
 * @brief Retrieves the cross-covariance matrix between this model and another.
 * 
 * @description This function computes the cross-covariance matrix between the current
 * sensor model and another given model. If the given model is the same as the
 * current model, it returns the covariance matrix of the current model's
 * parameters. If the models are different, it returns a zero matrix, assuming
 * no correlation between the models.
 * 
 * @param comparisonModel The geometric model to compare with.
 * @param pSet The parameter set for which the covariance is requested.
 * @param otherModels List of other geometric models considered in the
 * covariance computation.
 * @return Vector of doubles representing the flattened cross-covariance matrix.
 */
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

/**
 * @brief Returns the version of the sensor model.
 * 
 * @description Provides the version information of the sensor model implementation.
 * 
 * @return Version object containing the major, minor, and revision numbers.
 */
csm::Version UsgsAstroSarSensorModel::getVersion() const {
  return csm::Version(1, 0, 0);
}

/**
 * @brief Gets the model name.
 * 
 * @description Returns the name of the sensor model.
 * 
 * @return String representing the sensor model name.
 */
string UsgsAstroSarSensorModel::getModelName() const {
  return _SENSOR_MODEL_NAME;
}

/**
 * @brief Returns the pedigree of the sensor model.
 * 
 * @description Provides a string that identifies the lineage or source of the
 * sensor model implementation.
 * 
 * @return String representing the sensor model's pedigree.
 */
string UsgsAstroSarSensorModel::getPedigree() const { return "USGS_SAR"; }

/**
 * @brief Retrieves the image identifier.
 * 
 * @description Returns the identifier of the image associated with the sensor model.
 * 
 * @return String representing the image identifier.
 */
string UsgsAstroSarSensorModel::getImageIdentifier() const {
  return m_imageIdentifier;
}

/**
 * @brief Sets the image identifier.
 * 
 * @description Assigns an identifier to the image associated with the sensor model.
 * 
 * @param imageId The new image identifier.
 * @param warnings Optional pointer to a warning list to collect any warnings.
 */
void UsgsAstroSarSensorModel::setImageIdentifier(const string& imageId,
                                                 csm::WarningList* warnings) {
  m_imageIdentifier = imageId;
}

/**
 * @brief Retrieves the sensor identifier.
 * 
 * @description Returns the identifier of the sensor associated with the sensor model.
 * 
 * @return String representing the sensor identifier.
 */
string UsgsAstroSarSensorModel::getSensorIdentifier() const {
  return m_sensorIdentifier;
}

/**
 * @brief Retrieves the platform identifier.
 * 
 * @description Returns the identifier of the platform (satellite/aircraft) associated
 * with the sensor model.
 * 
 * @return String representing the platform identifier.
 */
string UsgsAstroSarSensorModel::getPlatformIdentifier() const {
  return m_platformIdentifier;
}

/**
 * @brief Retrieves the collection identifier.
 * 
 * @description Returns a string identifier for the collection to which the image
 * belongs.
 * 
 * @return String representing the collection identifier.
 */
string UsgsAstroSarSensorModel::getCollectionIdentifier() const {
  return m_collectionIdentifier;
}

/**
 * @brief Retrieves the trajectory identifier.
 * 
 * @description Returns a string identifier for the trajectory associated with the
 * sensor model.
 * 
 * @return String representing the trajectory identifier.
 */
string UsgsAstroSarSensorModel::getTrajectoryIdentifier() const {
  return m_trajectoryIdentifier;
}

/**
 * @brief Gets the sensor type.
 * 
 * @description Returns a string indicating the type of sensor (e.g., SAR for Synthetic
 * Aperture Radar).
 * 
 * @return String representing the sensor type.
 */
string UsgsAstroSarSensorModel::getSensorType() const { return "SAR"; }

/**
 * @brief Gets the sensor mode.
 * 
 * @description Returns a string indicating the operating mode of the sensor (e.g.,
 * "STRIP" for strip mapping).
 * 
 * @return String representing the sensor mode.
 */
string UsgsAstroSarSensorModel::getSensorMode() const { return "STRIP"; }

/**
 * @brief Gets the reference date and time for the sensor model.
 * 
 * @description Returns a string representing the reference date and time associated
 * with the sensor model.
 * 
 * @return String representing the reference date and time.
 */
string UsgsAstroSarSensorModel::getReferenceDateAndTime() const {
  csm::ImageCoord referencePointImage = groundToImage(m_referencePointXyz);
  double relativeTime =
    UsgsAstroSarSensorModel::getImageTime(referencePointImage);
  time_t ephemTime = m_centerEphemerisTime + relativeTime;

  return ephemTimeToCalendarTime(ephemTime);
}

/**
 * @brief Retrieves the ellipsoid associated with the sensor model.
 * 
 * @description Returns the ellipsoid used by the sensor model, defined by its semi-major
 * and semi-minor axes.
 * 
 * @return Ellipsoid object representing the ellipsoid used by the sensor model.
 */
csm::Ellipsoid UsgsAstroSarSensorModel::getEllipsoid() const {
  return csm::Ellipsoid(m_majorAxis, m_minorAxis);
}

/**
 * @brief Sets the ellipsoid for the sensor model.
 * 
 * @description Assigns a new ellipsoid to be used by the sensor model, defined by its
 * semi-major and semi-minor axes.
 * 
 * @param ellipsoid The new ellipsoid to use.
 */
void UsgsAstroSarSensorModel::setEllipsoid(const csm::Ellipsoid& ellipsoid) {
  m_majorAxis = ellipsoid.getSemiMajorRadius();
  m_minorAxis = ellipsoid.getSemiMinorRadius();
}

/**
 * @brief Determines the sensor covariance in image space.
 * 
 * @description Computes the sensor model covariance matrix transformed into image space
 * for a given ground point.
 * 
 * @param gp The ground point for which to determine the covariance.
 * @param sensor_cov Array to store the resulting 2x2 covariance matrix in image space.
 */
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

/**
 * @brief Retrieves range coefficients interpolated for a specific time.
 * 
 * @description Computes the range coefficients at a given time by interpolating the 
 * provided scale conversion coefficients. If there are multiple sets of coefficients,
 * Lagrange interpolation is used based on the scale conversion times. Otherwise,
 * the single set of coefficients is returned directly.
 * 
 * @param time The time at which to compute the range coefficients.
 * @return Vector of interpolated range coefficients.
 */
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

/**
 * @brief Calculates the sun position at a given image time.
 * 
 * @description Determines the sun's position in ECEF coordinates at a specified
 * image time. If multiple sun positions are available, Lagrange interpolation
 * is applied. If only one position is available but velocities are provided,
 * linear extrapolation is used. Otherwise, the single provided position is
 * returned.
 * 
 * @param imageTime The time of the image for which to calculate the sun position.
 * @return ECEF vector representing the sun's position.
 */
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

/**
 * @brief Retrieves the value of a parameter with adjustments applied.
 * 
 * @description Calculates the value of the specified parameter index by adding
 * the corresponding adjustment from the adjustments vector to the parameter's
 * current value.
 * 
 * @param index The index of the parameter.
 * @param adjustments Vector of adjustments to be applied to the parameters.
 * @return The adjusted value of the parameter.
 */
double UsgsAstroSarSensorModel::getValue(
    int index, const std::vector<double>& adjustments) const {
  return m_currentParameterValue[index] + adjustments[index];
}
