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

#include "UsgsAstroLsSensorModel.h"
#include "Distortion.h"
#include "Utilities.h"
#include "EigenUtilities.h"

#include <float.h>
#include <math.h>
#include <algorithm>
#include <iostream>
#include <sstream>

#include <Error.h>
#include <nlohmann/json.hpp>

#include "ale/Util.h"

#define MESSAGE_LOG(...)         \
  if (m_logger) {                \
    m_logger->log(__VA_ARGS__); \
  }

using json = nlohmann::json;

const std::string UsgsAstroLsSensorModel::_SENSOR_MODEL_NAME =
    "USGS_ASTRO_LINE_SCANNER_SENSOR_MODEL";
const int UsgsAstroLsSensorModel::NUM_PARAMETERS = 16;
const std::string UsgsAstroLsSensorModel::PARAMETER_NAME[] = {
    "IT Pos. Bias   ",  //  0 - "In Track Position Bias" - a constant shift in the spacecraft's position parallel to the flight path 
    "CT Pos. Bias   ",  //  1 - "Cross Track Position Bias" - a constant shift in the spacecraft's position perpendicular to the flight path
    "Rad Pos. Bias  ",  //  2 - "Radial Position Bias" - a constant shift in the spacecraft's "vertical positioning," i.e. distance from the target
    "IT Vel. Bias   ",  //  3 - "In Track Velocity Bias" - a time-dependent linear shift in the spacecraft's position parallel to the flight path
    "CT Vel. Bias   ",  //  4 - "Cross Track Velocity Bias" - a time-dependent linear shift in the spacecraft's position perpendicular to the flight path
    "Rad Vel. Bias  ",  //  5 - "Radial Velocity Bias" - a time-dependent linear shift in the spacecraft's "vertical positioning," i.e. distance from the target
    "Omega Bias     ",  //  6 - The initial omega angle (analogous to "roll")
    "Phi Bias       ",  //  7 - The initial phi angle (analogous to "pitch")
    "Kappa Bias     ",  //  8 - The initial kappa angle (analogous to "yaw")
    "Omega Rate     ",  //  9 - An angular rate that allows the omega angle to vary linearly with time
    "Phi Rate       ",  // 10 - An angular rate that allows the phi angle to vary linearly with time
    "Kappa Rate     ",  // 11 - An angular rate that allows the kappa angle to vary linearly with time
    "Omega Accl     ",  // 12 - An angular acceleration that allows the omega angle to vary quadratically with respect to time
    "Phi Accl       ",  // 13 - An angular acceleration that allows the phi angle to vary quadratically with respect to time
    "Kappa Accl     ",  // 14 - An angular acceleration that allows the kappa angle to vary quadratically with respect to time
    "Focal Bias     "   // 15 - Estimated error of the camera's focal length
};

const std::string UsgsAstroLsSensorModel::_STATE_KEYWORD[] = {
    "m_modelName",
    "m_imageIdentifier",
    "m_sensorName",
    "m_nLines",
    "m_nSamples",
    "m_platformFlag",
    "m_intTimeLines",
    "m_intTimeStartTimes",
    "m_intTimes",
    "m_startingEphemerisTime",
    "m_centerEphemerisTime",
    "m_detectorSampleSumming",
    "m_detectorSampleSumming",
    "m_startingDetectorSample",
    "m_startingDetectorLine",
    "m_ikCode",
    "m_focalLength",
    "m_zDirection",
    "m_distortionType",
    "m_opticalDistCoeffs",
    "m_iTransS",
    "m_iTransL",
    "m_detectorSampleOrigin",
    "m_detectorLineOrigin",
    "m_majorAxis",
    "m_minorAxis",
    "m_platformIdentifier",
    "m_sensorIdentifier",
    "m_minElevation",
    "m_maxElevation",
    "m_dtEphem",
    "m_t0Ephem",
    "m_dtQuat",
    "m_t0Quat",
    "m_numPositions",
    "m_numQuaternions",
    "m_positions",
    "m_velocities",
    "m_quaternions",
    "m_currentParameterValue",
    "m_parameterType",
    "m_referencePointXyz",
    "m_sunPosition",
    "m_sunVelocity",
    "m_gsd",
    "m_flyingHeight",
    "m_halfSwath",
    "m_halfTime",
    "m_covariance",
};

const int UsgsAstroLsSensorModel::NUM_PARAM_TYPES = 4;
const std::string UsgsAstroLsSensorModel::PARAM_STRING_ALL[] = {
    "NONE", "FICTITIOUS", "REAL", "FIXED"};
const csm::param::Type UsgsAstroLsSensorModel::PARAM_CHAR_ALL[] = {
    csm::param::NONE, csm::param::FICTITIOUS, csm::param::REAL,
    csm::param::FIXED};

/**
 * @brief Replaces the sensor model's state with a new state.
 * @description This function updates the internal state of the sensor model based on a 
 * given state string. The state string is parsed, and its contents are used to update
 * model parameters. This function is primarily used for sensor model instantiation or 
 * updating the model state in iterative optimization processes.
 * 
 * @param stateString A JSON string representing the new state of the sensor model. This 
 * string should contain all necessary model parameters and metadata to fully define the 
 * sensor model state.
 */
void UsgsAstroLsSensorModel::replaceModelState(const std::string& stateString) {
  MESSAGE_LOG(spdlog::level::info, "Replacing model state")

  reset();
  auto j = stateAsJson(stateString);
  int num_params = NUM_PARAMETERS;

  m_imageIdentifier = j["m_imageIdentifier"].get<std::string>();
  m_sensorName = j["m_sensorName"];
  m_nLines = j["m_nLines"];
  m_nSamples = j["m_nSamples"];
  m_platformFlag = j["m_platformFlag"];
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_imageIdentifier: {} "
      "m_sensorName: {} "
      "m_nLines: {} "
      "m_nSamples: {} "
      "m_platformFlag: {} ",
      j["m_imageIdentifier"].dump(), j["m_sensorName"].dump(),
      j["m_nLines"].dump(), j["m_nSamples"].dump(), j["m_platformFlag"].dump())

  m_intTimeLines = j["m_intTimeLines"].get<std::vector<double>>();
  m_intTimeStartTimes = j["m_intTimeStartTimes"].get<std::vector<double>>();
  m_intTimes = j["m_intTimes"].get<std::vector<double>>();

  m_startingEphemerisTime = j["m_startingEphemerisTime"];
  m_centerEphemerisTime = j["m_centerEphemerisTime"];
  m_detectorSampleSumming = j["m_detectorSampleSumming"];
  m_detectorLineSumming = j["m_detectorLineSumming"];
  m_startingDetectorSample = j["m_startingDetectorSample"];
  m_startingDetectorLine = j["m_startingDetectorLine"];
  m_ikCode = j["m_ikCode"];
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_startingEphemerisTime: {} "
      "m_centerEphemerisTime: {} "
      "m_detectorSampleSumming: {} "
      "m_detectorLineSumming: {} "
      "m_startingDetectorSample: {} "
      "m_ikCode: {} ",
      j["m_startingEphemerisTime"].dump(), j["m_centerEphemerisTime"].dump(),
      j["m_detectorSampleSumming"].dump(), j["m_detectorLineSumming"].dump(),
      j["m_startingDetectorSample"].dump(), j["m_ikCode"].dump())

  m_focalLength = j["m_focalLength"];
  m_zDirection = j["m_zDirection"];
  m_distortionType = (DistortionType)j["m_distortionType"].get<int>();
  m_opticalDistCoeffs = j["m_opticalDistCoeffs"].get<std::vector<double>>();
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_focalLength: {} "
      "m_zDirection: {} "
      "m_distortionType: {} ",
      j["m_focalLength"].dump(), j["m_zDirection"].dump(),
      j["m_distortionType"].dump())

  for (int i = 0; i < 3; i++) {
    m_iTransS[i] = j["m_iTransS"][i];
    m_iTransL[i] = j["m_iTransL"][i];
  }

  m_detectorSampleOrigin = j["m_detectorSampleOrigin"];
  m_detectorLineOrigin = j["m_detectorLineOrigin"];
  m_majorAxis = j["m_majorAxis"];
  m_minorAxis = j["m_minorAxis"];
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_detectorSampleOrigin: {} "
      "m_detectorLineOrigin: {} "
      "m_majorAxis: {} "
      "m_minorAxis: {} ",
      j["m_detectorSampleOrigin"].dump(), j["m_detectorLineOrigin"].dump(),
      j["m_majorAxis"].dump(), j["m_minorAxis"].dump())

  m_platformIdentifier = j["m_platformIdentifier"];
  m_sensorIdentifier = j["m_sensorIdentifier"];
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_platformIdentifier: {} "
      "m_sensorIdentifier: {} ",
      j["m_platformIdentifier"].dump(), j["m_sensorIdentifier"].dump())

  m_minElevation = j["m_minElevation"];
  m_maxElevation = j["m_maxElevation"];
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_minElevation: {} "
      "m_maxElevation: {} ",
      j["m_minElevation"].dump(), j["m_maxElevation"].dump())

  m_dtEphem = j["m_dtEphem"];
  m_t0Ephem = j["m_t0Ephem"];
  m_dtQuat = j["m_dtQuat"];
  m_t0Quat = j["m_t0Quat"];
  m_numPositions = j["m_numPositions"];
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_dtEphem: {} "
      "m_t0Ephem: {} "
      "m_dtQuat: {} "
      "m_t0Quat: {} ",
      j["m_dtEphem"].dump(), j["m_t0Ephem"].dump(), j["m_dtQuat"].dump(),
      j["m_t0Quat"].dump())

  m_numQuaternions = j["m_numQuaternions"];
  m_referencePointXyz.x = j["m_referencePointXyz"][0];
  m_referencePointXyz.y = j["m_referencePointXyz"][1];
  m_referencePointXyz.z = j["m_referencePointXyz"][2];
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_numQuaternions: {} "
      "m_referencePointX: {} "
      "m_referencePointY: {} "
      "m_referencePointZ: {} ",
      j["m_numQuaternions"].dump(), j["m_referencePointXyz"][0].dump(),
      j["m_referencePointXyz"][1].dump(), j["m_referencePointXyz"][2].dump())

  m_gsd = j["m_gsd"];
  m_flyingHeight = j["m_flyingHeight"];
  m_halfSwath = j["m_halfSwath"];
  m_halfTime = j["m_halfTime"];
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_gsd: {} "
      "m_flyingHeight: {} "
      "m_halfSwath: {} "
      "m_halfTime: {} ",
      j["m_gsd"].dump(), j["m_flyingHeight"].dump(), j["m_halfSwath"].dump(),
      j["m_halfTime"].dump())
  // Vector = is overloaded so explicit get with type required.

  m_positions = j["m_positions"].get<std::vector<double>>();
  m_velocities = j["m_velocities"].get<std::vector<double>>();
  m_quaternions = j["m_quaternions"].get<std::vector<double>>();
  m_currentParameterValue =
      j["m_currentParameterValue"].get<std::vector<double>>();
  m_covariance = j["m_covariance"].get<std::vector<double>>();
  m_sunPosition = j["m_sunPosition"].get<std::vector<double>>();
  m_sunVelocity = j["m_sunVelocity"].get<std::vector<double>>();

  for (int i = 0; i < num_params; i++) {
    for (int k = 0; k < NUM_PARAM_TYPES; k++) {
      if (j["m_parameterType"][i] == PARAM_STRING_ALL[k]) {
        m_parameterType[i] = PARAM_CHAR_ALL[k];
        break;
      }
    }
  }

  // If computed state values are still default, then compute them
  if (m_gsd == 1.0 && m_flyingHeight == 1000.0) {
    updateState();
  }

  try {
    // Approximate the ground-to-image function with a projective transform
    createProjectiveApproximation();
  } catch (...) {
    m_useApproxInitTrans = false;
  }
}

/**
 * @brief Retrieves the model name from a model state JSON string.
 * @description This function extracts and returns the name of the sensor model from 
 * a given state JSON string. It is primarily used to validate or utilize the model name
 * stored within a serialized state representation.
 * 
 * @param model_state A JSON string representing the serialized state of a sensor model. 
 * This string must contain a "m_modelName" key with the model's name as its value.
 * @return std::string The name of the model extracted from the provided state.
 * 
 * @throws csm::Error If the "m_modelName" key is missing from the model state or if the 
 * model name in the state is not supported, indicating either an invalid or incompatible 
 * state string.
 */
std::string UsgsAstroLsSensorModel::getModelNameFromModelState(
    const std::string& model_state) {
  // Parse the string to JSON
  auto j = stateAsJson(model_state);
  // If model name cannot be determined, return a blank string
  std::string model_name;

  if (j.find("m_modelName") != j.end()) {
    model_name = j["m_modelName"];
  } else {
    csm::Error::ErrorType aErrorType = csm::Error::INVALID_SENSOR_MODEL_STATE;
    std::string aMessage = "No 'm_modelName' key in the model state object.";
    std::string aFunction = "UsgsAstroLsPlugin::getModelNameFromModelState";
    csm::Error csmErr(aErrorType, aMessage, aFunction);
    throw(csmErr);
  }
  if (model_name != _SENSOR_MODEL_NAME) {
    csm::Error::ErrorType aErrorType = csm::Error::SENSOR_MODEL_NOT_SUPPORTED;
    std::string aMessage = "Sensor model not supported.";
    std::string aFunction = "UsgsAstroLsPlugin::getModelNameFromModelState()";
    csm::Error csmErr(aErrorType, aMessage, aFunction);
    throw(csmErr);
  }
  return model_name;
}

/**
 * @brief Serializes the sensor model state to a JSON string.
 * @description This method captures the current state of the sensor model, including all relevant
 * parameters and configurations, and serializes this information into a structured JSON string.
 * 
 * @return std::string A JSON string representing the serialized state of the sensor model.
 * The string includes key-value pairs for various model parameters, ensuring a comprehensive
 * snapshot of the model's current configuration and state.
 */
std::string UsgsAstroLsSensorModel::getModelState() const {
  MESSAGE_LOG(spdlog::level::info, "Running getModelState")

  json state;
  state["m_modelName"] = _SENSOR_MODEL_NAME;
  state["m_startingDetectorSample"] = m_startingDetectorSample;
  state["m_startingDetectorLine"] = m_startingDetectorLine;
  state["m_imageIdentifier"] = m_imageIdentifier;
  state["m_sensorName"] = m_sensorName;
  state["m_nLines"] = m_nLines;
  state["m_nSamples"] = m_nSamples;
  state["m_platformFlag"] = m_platformFlag;
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_imageIdentifier: {} "
      "m_sensorName: {} "
      "m_nLines: {} "
      "m_nSamples: {} "
      "m_platformFlag: {} ",
      m_imageIdentifier, m_sensorName, m_nLines, m_nSamples, m_platformFlag)

  state["m_intTimeLines"] = m_intTimeLines;
  state["m_intTimeStartTimes"] = m_intTimeStartTimes;
  state["m_intTimes"] = m_intTimes;
  state["m_startingEphemerisTime"] = m_startingEphemerisTime;
  state["m_centerEphemerisTime"] = m_centerEphemerisTime;
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_startingEphemerisTime: {} "
      "m_centerEphemerisTime: {} ",
      m_startingEphemerisTime, m_centerEphemerisTime)

  state["m_detectorSampleSumming"] = m_detectorSampleSumming;
  state["m_detectorLineSumming"] = m_detectorLineSumming;
  state["m_startingDetectorSample"] = m_startingDetectorSample;
  state["m_ikCode"] = m_ikCode;
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_detectorSampleSumming: {} "
      "m_detectorLineSumming: {} "
      "m_startingDetectorSample: {} "
      "m_ikCode: {} ",
      m_detectorSampleSumming, m_detectorLineSumming, m_startingDetectorSample,
      m_ikCode)

  state["m_focalLength"] = m_focalLength;
  state["m_zDirection"] = m_zDirection;
  state["m_distortionType"] = m_distortionType;
  state["m_opticalDistCoeffs"] = m_opticalDistCoeffs;
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_focalLength: {} "
      "m_zDirection: {} "
      "m_distortionType (0-Radial, 1-Transverse): {} ",
      m_focalLength, m_zDirection, m_distortionType)

  state["m_iTransS"] = std::vector<double>(m_iTransS, m_iTransS + 3);
  state["m_iTransL"] = std::vector<double>(m_iTransL, m_iTransL + 3);

  state["m_detectorSampleOrigin"] = m_detectorSampleOrigin;
  state["m_detectorLineOrigin"] = m_detectorLineOrigin;
  state["m_majorAxis"] = m_majorAxis;
  state["m_minorAxis"] = m_minorAxis;
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_detectorSampleOrigin: {} "
      "m_detectorLineOrigin: {} "
      "m_majorAxis: {} "
      "m_minorAxis: {} ",
      m_detectorSampleOrigin, m_detectorLineOrigin, m_majorAxis, m_minorAxis)

  state["m_platformIdentifier"] = m_platformIdentifier;
  state["m_sensorIdentifier"] = m_sensorIdentifier;
  state["m_minElevation"] = m_minElevation;
  state["m_maxElevation"] = m_maxElevation;
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_platformIdentifier: {} "
      "m_sensorIdentifier: {} "
      "m_minElevation: {} "
      "m_maxElevation: {} ",
      m_platformIdentifier, m_sensorIdentifier, m_minElevation, m_maxElevation)

  state["m_dtEphem"] = m_dtEphem;
  state["m_t0Ephem"] = m_t0Ephem;
  state["m_dtQuat"] = m_dtQuat;
  state["m_t0Quat"] = m_t0Quat;
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_dtEphem: {} "
      "m_t0Ephem: {} "
      "m_dtQuat: {} "
      "m_t0Quat: {} ",
      m_dtEphem, m_t0Ephem, m_dtQuat, m_t0Quat)

  state["m_numPositions"] = m_numPositions;
  state["m_numQuaternions"] = m_numQuaternions;
  state["m_positions"] = m_positions;
  state["m_velocities"] = m_velocities;
  state["m_quaternions"] = m_quaternions;
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_numPositions: {} "
      "m_numQuaternions: {} ",
      m_numPositions, m_numQuaternions)

  state["m_currentParameterValue"] = m_currentParameterValue;
  state["m_parameterType"] = m_parameterType;

  state["m_gsd"] = m_gsd;
  state["m_flyingHeight"] = m_flyingHeight;
  state["m_halfSwath"] = m_halfSwath;
  state["m_halfTime"] = m_halfTime;
  state["m_covariance"] = m_covariance;
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_gsd: {} "
      "m_flyingHeight: {} "
      "m_halfSwath: {} "
      "m_halfTime: {} ",
      m_gsd, m_flyingHeight, m_halfSwath, m_halfTime)

  state["m_referencePointXyz"] = json();
  state["m_referencePointXyz"][0] = m_referencePointXyz.x;
  state["m_referencePointXyz"][1] = m_referencePointXyz.y;
  state["m_referencePointXyz"][2] = m_referencePointXyz.z;
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_referencePointXyz: {} "
      "m_referencePointXyz: {} "
      "m_referencePointXyz: {} ",
      m_referencePointXyz.x, m_referencePointXyz.y, m_referencePointXyz.z)

  state["m_sunPosition"] = m_sunPosition;
  MESSAGE_LOG(spdlog::level::trace, "num sun positions: {} ", m_sunPosition.size())

  state["m_sunVelocity"] = m_sunVelocity;
  MESSAGE_LOG(spdlog::level::trace, "num sun velocities: {} ", m_sunVelocity.size());

  // Use dump(2) to avoid creating the model string as a single long line
  std::string stateString = getModelName() + "\n" + state.dump(2);
  return stateString;
}

/**
 * @brief Applies a rotation and translation transform to the sensor model state.
 * @description This method modifies the sensor model's state by applying a specified rotation
 * and translation to its elements, including quaternions, positions, and velocities. The operation
 * affects the sensor's orientation, position, and motion, ensuring that changes in the sensor's
 * physical setup or perspective are accurately reflected in the model state. The sun's position
 * and velocity are not modified, assuming their significant distance makes any changes
 * inconsequential to the model.
 * 
 * @param r The rotation to be applied, encapsulated in an `ale::Rotation` object, affecting
 * the sensor's orientation and angular velocity.
 * @param t The translation to be applied, represented as an `ale::Vec3d` vector, shifting
 * the sensor's position.
 * @param stateString A reference to a string holding the serialized JSON representation of
 * the sensor model's state. This string is modified in-place to reflect the applied transformation.
 */
void UsgsAstroLsSensorModel::applyTransformToState(ale::Rotation const& r, ale::Vec3d const& t,
                                                   std::string& stateString) {

  nlohmann::json j = stateAsJson(stateString);

  // Apply rotation to quaternions
  std::vector<double> quaternions = j["m_quaternions"].get<std::vector<double>>();
  applyRotationToQuatVec(r, quaternions);
  j["m_quaternions"] = quaternions;

  // Apply rotation and translation to positions
  std::vector<double> positions = j["m_positions"].get<std::vector<double>>();;
  applyRotationTranslationToXyzVec(r, t, positions);
  j["m_positions"] = positions;

  // Apply rotation to velocities. The translation does not get applied.
  ale::Vec3d zero_t(0, 0, 0);
  std::vector<double> velocities = j["m_velocities"].get<std::vector<double>>();;
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
 * @brief Resets the sensor model to its default state.
 * @description This method reinitializes the sensor model to default values, effectively
 * erasing any adjustments or parameter modifications previously made. It sets all member
 * variables to their default or zero-values, clears any data structures such as vectors, and
 * prepares the model for a fresh start. This includes setting the sensor name to a default
 * value, defining the default focal length, initializing distortion coefficients, and setting
 * various model parameters like the number of lines, samples, and geometric parameters like
 * the major and minor axes of the ellipsoid model used for the planet.
 */
void UsgsAstroLsSensorModel::reset() {
  MESSAGE_LOG(spdlog::level::debug, "Running reset()")
  m_useApproxInitTrans = false;  // default until an initial approximation is found

  _no_adjustment.assign(UsgsAstroLsSensorModel::NUM_PARAMETERS, 0.0);

  m_imageIdentifier = "";                 // 1
  m_sensorName = "USGSAstroLineScanner";  // 2
  m_nLines = 0;                           // 3
  m_nSamples = 0;                         // 4
  m_platformFlag = 1;                     // 9
  m_intTimeLines.clear();
  m_intTimeStartTimes.clear();
  m_intTimes.clear();
  m_startingEphemerisTime = 0.0;  // 13
  m_centerEphemerisTime = 0.0;    // 14
  m_detectorSampleSumming = 1.0;  // 15
  m_detectorLineSumming = 1.0;
  m_startingDetectorSample = 1.0;  // 16
  m_startingDetectorLine   = 1.0;
  m_ikCode = -85600;               // 17
  m_focalLength = 350.0;           // 18
  m_zDirection = 1.0;              // 19
  m_distortionType = DistortionType::RADIAL;
  m_opticalDistCoeffs = std::vector<double>(3, 0.0);
  m_iTransS[0] = 0.0;               // 21
  m_iTransS[1] = 0.0;               // 21
  m_iTransS[2] = 150.0;             // 21
  m_iTransL[0] = 0.0;               // 22
  m_iTransL[1] = 150.0;             // 22
  m_iTransL[2] = 0.0;               // 22
  m_detectorSampleOrigin = 2500.0;  // 23
  m_detectorLineOrigin = 0.0;       // 24
  m_majorAxis = 3400000.0;          // 27
  m_minorAxis = 3350000.0;          // 28
  m_platformIdentifier = "";        // 31
  m_sensorIdentifier = "";          // 32
  m_minElevation = -8000.0;         // 34
  m_maxElevation = 8000.0;          // 35
  m_dtEphem = 2.0;                  // 36
  m_t0Ephem = -70.0;                // 37
  m_dtQuat = 0.1;                   // 38
  m_t0Quat = -40.0;                 // 39
  m_numPositions = 0;               // 40
  m_numQuaternions = 0;             // 41
  m_positions.clear();              // 42
  m_velocities.clear();             // 43
  m_quaternions.clear();            // 44

  m_currentParameterValue.assign(NUM_PARAMETERS, 0.0);
  m_parameterType.assign(NUM_PARAMETERS, csm::param::REAL);

  m_referencePointXyz.x = 0.0;
  m_referencePointXyz.y = 0.0;
  m_referencePointXyz.z = 0.0;

  m_sunPosition = std::vector<double>(3, 0.0);
  m_sunVelocity = std::vector<double>(3, 0.0);

  m_gsd = 1.0;
  m_flyingHeight = 1000.0;
  m_halfSwath = 1000.0;
  m_halfTime = 10.0;

  m_covariance =
      std::vector<double>(NUM_PARAMETERS * NUM_PARAMETERS, 0.0);  // 52
}

/**
 * @brief Constructor for UsgsAstroLsSensorModel.
 * @description Initializes a new instance of the UsgsAstroLsSensorModel class, setting up
 * the necessary structures and preparing the model for use. This includes allocating resources
 * and setting initial parameter values to their defaults by invoking the reset method.
 */
UsgsAstroLsSensorModel::UsgsAstroLsSensorModel() {
  _no_adjustment.assign(UsgsAstroLsSensorModel::NUM_PARAMETERS, 0.0);
}

/**
 * @brief Destructor for UsgsAstroLsSensorModel.
 * @description Cleans up an instance of UsgsAstroLsSensorModel when it is no longer needed.
 * This includes releasing any resources allocated during the lifetime of the model instance
 * and flushing the logger to ensure all messages are written out. This method ensures a clean
 * shutdown of the sensor model instance, avoiding memory leaks and ensuring that all resources
 * are properly released.
 */
UsgsAstroLsSensorModel::~UsgsAstroLsSensorModel() {
  if (m_logger) {
    m_logger->flush();
  }
}

/**
 * @brief Updates the internal state of the sensor model.
 * @description This method computes and updates several key parameters of the sensor model
 * based on the current model state, including the reference point (the ground point corresponding
 * to the center of the image), the ground sample distance (GSD), the flying height of the sensor,
 * the half swath, and the half time duration of the image capture process.
 * 
 * The reference point is calculated by converting the center pixel of the image to a ground point
 * using the imageToGround method. The GSD is then computed as the distance between adjacent pixels
 * in ground space, providing a measure of the spatial resolution of the image. The flying height
 * is determined by calculating the distance from the sensor position to the reference point.
 * Additionally, the method calculates the half swath, which represents half the width of the
 * imaged area on the ground, and the half time duration, which estimates half the time taken to
 * capture the image.
 */
void UsgsAstroLsSensorModel::updateState() {
  // If sensor model is being created for the first time
  // This routine will set some parameters not found in the ISD.
  MESSAGE_LOG(spdlog::level::debug, "Updating State")
  // Reference point (image center)
  double lineCtr = m_nLines / 2.0;
  double sampCtr = m_nSamples / 2.0;
  csm::ImageCoord ip(lineCtr, sampCtr);
  MESSAGE_LOG(
      spdlog::level::trace,
      "updateState: center image coordinate set to {} {}",
      lineCtr, sampCtr)

  double refHeight = 0;
  m_referencePointXyz = imageToGround(ip, refHeight);
  MESSAGE_LOG(
      spdlog::level::trace,
      "updateState: reference point (x, y, z) {} {} {}",
      m_referencePointXyz.x, m_referencePointXyz.y, m_referencePointXyz.z)

  // Compute ground sample distance
  ip.line += 1;
  ip.samp += 1;
  csm::EcefCoord delta = imageToGround(ip, refHeight);
  double dx = delta.x - m_referencePointXyz.x;
  double dy = delta.y - m_referencePointXyz.y;
  double dz = delta.z - m_referencePointXyz.z;
  m_gsd = sqrt((dx * dx + dy * dy + dz * dz) / 2.0);
  MESSAGE_LOG(
      spdlog::level::trace,
      "updateState: ground sample distance set to {} "
      "based on dx {} dy {} dz {}",
      m_gsd, dx, dy, dz)

  // Compute the flying height. Use the center of the image, as for
  // m_referencePointXyz and m_gsd.
  ip = csm::ImageCoord(lineCtr, sampCtr);
  csm::EcefCoord sensorPos = getSensorPosition(ip);
  dx = sensorPos.x - m_referencePointXyz.x;
  dy = sensorPos.y - m_referencePointXyz.y;
  dz = sensorPos.z - m_referencePointXyz.z;
  m_flyingHeight = sqrt(dx * dx + dy * dy + dz * dz);
  MESSAGE_LOG(
      spdlog::level::trace,
      "updateState: flight height set to {}"
      "based on dx {} dy {} dz {}",
      m_flyingHeight, dx, dy, dz)

  // Compute half swath
  m_halfSwath = m_gsd * m_nSamples / 2.0;
  MESSAGE_LOG(spdlog::level::trace, "updateState: half swath set to {}", m_halfSwath)

  // Compute half time duration
  double fullImageTime = m_intTimeStartTimes.back() -
                         m_intTimeStartTimes.front() +
                         m_intTimes.back() * (m_nLines - m_intTimeLines.back());
  m_halfTime = fullImageTime / 2.0;
  MESSAGE_LOG(spdlog::level::trace, "updateState: half time duration set to {}", m_halfTime)

  // Parameter covariance, hardcoded accuracy values
  // hardcoded ~1 pixel accuracy values
  int num_params = NUM_PARAMETERS;
  double positionVariance = m_gsd * m_gsd;
  // parameter time is scaled to [0, 2]
  // so divide by 2 for velocities and 4 for accelerations
  double velocityVariance = positionVariance / 2.0;
  double accelerationVariance = positionVariance / 4.0;
  m_covariance.assign(num_params * num_params, 0.0);

  // Set position variances
  m_covariance[0] = positionVariance;
  m_covariance[num_params + 1] = positionVariance;
  m_covariance[2 * num_params + 2] = positionVariance;
  m_covariance[3 * num_params + 3] = velocityVariance;
  m_covariance[4 * num_params + 4] = velocityVariance;
  m_covariance[5 * num_params + 5] = velocityVariance;

  // Set orientation variances
  m_covariance[6 * num_params + 6] = positionVariance;
  m_covariance[7 * num_params + 7] = positionVariance;
  m_covariance[8 * num_params + 8] = positionVariance;
  m_covariance[9 * num_params + 9] = velocityVariance;
  m_covariance[10 * num_params + 10] = velocityVariance;
  m_covariance[11 * num_params + 11] = velocityVariance;
  m_covariance[12 * num_params + 12] = accelerationVariance;
  m_covariance[13 * num_params + 13] = accelerationVariance;
  m_covariance[14 * num_params + 14] = accelerationVariance;

  // Set focal length variance
  m_covariance[15 * num_params + 15] = positionVariance;
}

/**
 * @brief Converts a ground point to its corresponding image coordinates without adjustments.
 * 
 * @description This function estimates the image coordinates (line, sample) for a given 
 * ground point in Earth-Centered, Earth-Fixed (ECEF) coordinates. It utilizes the 
 * groundToImage function with an internal version that incorporates adjustments. 
 * This wrapper function calls the more detailed groundToImage function without providing 
 * any adjustments, thereby using the default model parameters.
 * 
 * @param ground_pt The ground point in ECEF coordinates to be converted to image coordinates.
 * @param desired_precision The desired precision for the estimated image coordinates. 
 * This influences the iterative process's termination condition.
 * @param achieved_precision A pointer to a double where the function stores the achieved 
 * precision of the estimate. It allows the caller to assess the accuracy of the estimation.
 * @param warnings A pointer to a list for recording any warnings encountered during 
 * the computation. It can include warnings about precision not being met.
 * 
 * @return csm::ImageCoord The estimated image coordinates (line, sample) corresponding 
 * to the given ground point.
 */
csm::ImageCoord UsgsAstroLsSensorModel::groundToImage(
    const csm::EcefCoord& ground_pt, double desired_precision,
    double* achieved_precision, csm::WarningList* warnings) const {
  MESSAGE_LOG(
      spdlog::level::info,
      "Computing groundToImage(No adjustments) for ({}, {}, {}), with desired "
      "precision {}",
      ground_pt.x, ground_pt.y, ground_pt.z, desired_precision);

  // The public interface invokes the private interface with no adjustments.
  csm::ImageCoord imagePt = UsgsAstroLsSensorModel::groundToImage(
      ground_pt, _no_adjustment, desired_precision, achieved_precision, warnings);
  MESSAGE_LOG(
      spdlog::level::info,
      "groundToImage result of ({}, {})",
      imagePt.line, imagePt.samp);
  return imagePt;
}

/**
 * @brief Detailed conversion from a ground point to image coordinates, considering adjustments.
 * 
 * @description Performs a refined computation of the image coordinates that view a specified 
 * ground point, taking into account model adjustments. This function applies a projective 
 * approximation for an initial guess of the image coordinates and then iteratively refines 
 * this guess using the secant method to minimize the error in line estimation. The secant 
 * method is a numerical technique for finding a root of a function, which in this context 
 * is the difference between the estimated and actual lines corresponding to the ground point.
 * The process involves adjusting the initial estimate of the image coordinates based on the 
 * given adjustments (if any), and iteratively improving the estimate to achieve or exceed 
 * the desired precision level.
 * 
 * @param groundPt The ECEF coordinates of the ground point to project into image space.
 * @param adj Adjustments to apply to the sensor model parameters for this estimation.
 * @param desiredPrecision The precision goal for the iterative estimation process.
 * @param achievedPrecision Pointer to a double where the achieved precision of the estimate is stored.
 * @param warnings Pointer to a list that will be populated with warnings, should they occur.
 * 
 * @return csm::ImageCoord The refined image coordinates (line, sample) for the ground point.
 */
csm::ImageCoord UsgsAstroLsSensorModel::groundToImage(
    const csm::EcefCoord& groundPt, const std::vector<double>& adj,
    double desiredPrecision, double* achievedPrecision,
    csm::WarningList* warnings) const {

  csm::ImageCoord approxPt;
  computeProjectiveApproximation(groundPt, approxPt);
  MESSAGE_LOG(
      spdlog::level::trace,
      "Computed Proj Approximation: {}, {}",
      approxPt.line, approxPt.samp);

  // Search for the (line, sample) coordinate that views a given
  // ground point. Set this up as a root-finding problem and use the
  // secant method.
  int count = 0;
  double t0 = 0.0;
  double lineErr0 = calcDetectorLineErr(t0, approxPt, groundPt, adj);
  double t1 = 0.1;
  double lineErr1 = calcDetectorLineErr(t1, approxPt, groundPt, adj);
  MESSAGE_LOG(
      spdlog::level::trace,
      "Initial Line Error: {}, {}",
      lineErr0, lineErr1);
  while (std::abs(lineErr1) > desiredPrecision && ++count < 15) {

    if (lineErr1 == lineErr0)
      break; // avoid division by 0

    // Secant method update
    // https://en.wikipedia.org/wiki/Secant_method
    double t2 = t1 - lineErr1 * (t1 - t0) / (lineErr1 - lineErr0);
    double lineErr2 = calcDetectorLineErr(t2, approxPt, groundPt, adj);

    // Update for the next step
    t0 = t1; lineErr0 = lineErr1;
    t1 = t2; lineErr1 = lineErr2;
    MESSAGE_LOG(
        spdlog::level::trace,
        "{} Line Error and (t0, t1): {}, {}, {}, {}",
        count, lineErr0, lineErr1, t0, t1);
  }

  // Update the line with the found value
  approxPt.line += t1;
  MESSAGE_LOG(
      spdlog::level::trace,
      "After line Approximation: {}, {}",
      approxPt.line, approxPt.samp);

  double timei = getImageTime(approxPt);
  std::vector<double> detectorView = computeDetectorView(timei, groundPt, adj);
  MESSAGE_LOG(
      spdlog::level::trace,
      "Detector View: {}, and undistortedY: {}",
      detectorView[0], detectorView[1]);

  // Invert distortion
  double distortedFocalX, distortedFocalY;
  applyDistortion(detectorView[0], detectorView[1], distortedFocalX,
                  distortedFocalY, m_opticalDistCoeffs, m_focalLength,
                  m_distortionType,
                  desiredPrecision);
  MESSAGE_LOG(
      spdlog::level::trace,
      "Distorted X, Y: {}, and undistortedY: {}",
      distortedFocalX, distortedFocalY);

  // Convert to detector line and sample
  double detectorLine = m_iTransL[0] + m_iTransL[1] * distortedFocalX +
                        m_iTransL[2] * distortedFocalY;
  double detectorSample = m_iTransS[0] + m_iTransS[1] * distortedFocalX +
                          m_iTransS[2] * distortedFocalY;
  MESSAGE_LOG(
      spdlog::level::trace,
      "Detector Line, Sample: {}, and undistortedY: {}",
      detectorLine, detectorSample);
  // Convert to image sample line
  double finalUpdate =
      (detectorLine + m_detectorLineOrigin - m_startingDetectorLine) /
      m_detectorLineSumming;
  approxPt.line += finalUpdate;
  approxPt.samp =
      (detectorSample + m_detectorSampleOrigin - m_startingDetectorSample) /
      m_detectorSampleSumming;

  if (achievedPrecision) {
    *achievedPrecision = finalUpdate;
  }

  MESSAGE_LOG(
      spdlog::level::debug,
      "groundToImage result image line sample {} {}",
      approxPt.line, approxPt.samp)

  if (warnings && (desiredPrecision > 0.0) &&
      (std::abs(finalUpdate) > desiredPrecision)) {
    warnings->push_back(csm::Warning(
        csm::Warning::PRECISION_NOT_MET, "Desired precision not achieved.",
        "UsgsAstroLsSensorModel::groundToImage()"));
  }

  return approxPt;
}

/**
 * @brief Converts a ground point with covariance to its corresponding image coordinates with covariance.
 * 
 * @description This function estimates the image coordinates for a given ground point, including
 * the propagation of the ground point's covariance through the transformation. It begins by
 * converting the ECEF ground point to image coordinates using the groundToImage method. It then
 * computes the partial derivatives of the line and sample coordinates with respect to the ground
 * point's X, Y, and Z coordinates. These partial derivatives are used to propagate the ground
 * point's covariance to the image space, taking into account both modeled and unmodeled errors as
 * well as sensor-specific covariance.
 * 
 * @param groundPt The ground point in ECEF coordinates with covariance.
 * @param desired_precision The desired precision for the image coordinate estimation.
 * @param achieved_precision A pointer to a double where the achieved precision of the estimate is stored.
 * @param warnings A pointer to a list that will be populated with warnings, should they occur.
 * 
 * @return csm::ImageCoordCovar The image coordinates and their covariance for the given ground point.
 */
csm::ImageCoordCovar UsgsAstroLsSensorModel::groundToImage(
    const csm::EcefCoordCovar& groundPt, double desired_precision,
    double* achieved_precision, csm::WarningList* warnings) const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Computing groundToImage(Covar) for {}, {}, {}, with desired precision "
      "{}",
      groundPt.x, groundPt.y, groundPt.z, desired_precision);
  // Ground to image with error propagation
  // Compute corresponding image point
  csm::EcefCoord gp;
  gp.x = groundPt.x;
  gp.y = groundPt.y;
  gp.z = groundPt.z;

  csm::ImageCoord ip =
      UsgsAstroLsSensorModel::groundToImage(gp, desired_precision, achieved_precision, warnings);
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
 * @brief Converts an image coordinate to a ground point in ECEF coordinates at a given height.
 * 
 * @description This function computes the ground point in Earth-Centered Earth-Fixed (ECEF) coordinates
 * corresponding to a given image coordinate and a specified height above the ellipsoid model of the Earth.
 * 
 * @param image_pt The image coordinate (line and sample) for which to compute the ground point.
 * @param height The height above the ellipsoid at which to compute the ground point, in meters.
 * @param desired_precision The desired precision for the ground point calculation, in meters.
 * @param achieved_precision A pointer to a double where the achieved precision of the computation will be stored.
 * @param warnings A pointer to a csm::WarningList for logging any warnings that occur during the computation.
 * 
 * @return csm::EcefCoord The computed ground point in ECEF coordinates.
 */
csm::EcefCoord UsgsAstroLsSensorModel::imageToGround(
    const csm::ImageCoord& image_pt, double height, double desired_precision,
    double* achieved_precision, csm::WarningList* warnings) const {
  MESSAGE_LOG(
      spdlog::level::info,
      "Computing imageToGround for {}, {}, {}, with desired precision {}",
      image_pt.line, image_pt.samp, height, desired_precision);
  double xc, yc, zc;
  double vx, vy, vz;
  double xl, yl, zl;
  double dxl, dyl, dzl;
  losToEcf(image_pt.line, image_pt.samp, _no_adjustment, xc, yc, zc, vx, vy, vz,
           xl, yl, zl);

  double aPrec;
  double x, y, z;
  losEllipsoidIntersect(height, xc, yc, zc, xl, yl, zl, x, y, z, aPrec,
                        desired_precision, warnings);

  if (achieved_precision) *achieved_precision = aPrec;

  if (warnings && (desired_precision > 0.0) && (aPrec > desired_precision)) {
    warnings->push_back(csm::Warning(
        csm::Warning::PRECISION_NOT_MET, "Desired precision not achieved.",
        "UsgsAstroLsSensorModel::imageToGround()"));
  }
  MESSAGE_LOG(
      spdlog::level::info,
      "imageToGround result {} {} {}",
      x, y, z);
  return csm::EcefCoord(x, y, z);
}

/**
 * @brief Calculates sensor covariance in the image coordinate system.
 * 
 * @description This function calculates the covariance matrix of the sensor
 * in the image space for a given ground point. It utilizes the sensor's
 * partial derivatives with respect to its parameters to transform the
 * parameter covariance into image space, providing a measure of uncertainty
 * in image coordinates that originates from sensor model parameters.
 * 
 * @param gp The ground point in ECEF coordinates for which to determine sensor covariance.
 * @param sensor_cov An array to store the 2x2 covariance matrix. The array will contain
 * the covariance values in row-major order: [line variance, line-sample covariance,
 * sample-line covariance, sample variance].
 */
void UsgsAstroLsSensorModel::determineSensorCovarianceInImageSpace(
    csm::EcefCoord& gp, double sensor_cov[4]) const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Calculating determineSensorCovarianceInImageSpace for {} {} {}",
      gp.x, gp.y, gp.z)

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
 * @brief Converts an image coordinate with associated uncertainties to a ground point in ECEF coordinates.
 * 
 * @description This function computes the ground point in ECEF coordinates corresponding to a given image
 * coordinate with associated uncertainties. It extends the basic image-to-ground transformation by also
 * considering the uncertainties in the image coordinate and the specified height above the ellipsoid.
 * The function calculates the covariance matrix of the resulting ground point by propagating the uncertainties
 * through the transformation process. This includes uncertainties from the sensor model itself (sensor covariance),
 * as well as any unmodeled errors and the provided height variance.
 * 
 * @param image_pt The image coordinate with covariance (line, sample, and their covariance matrix).
 * @param height The height above the ellipsoid at which to compute the ground point, in meters.
 * @param heightVariance The variance of the height above the ellipsoid.
 * @param desired_precision The desired precision for the ground point calculation, in meters.
 * @param achieved_precision A pointer to a double where the achieved precision of the computation will be stored.
 * @param warnings A pointer to a csm::WarningList for logging any warnings that occur during the computation.
 * 
 * @return csm::EcefCoordCovar The computed ground point in ECEF coordinates with covariance.
 */
csm::EcefCoordCovar UsgsAstroLsSensorModel::imageToGround(
    const csm::ImageCoordCovar& image_pt, double height, double heightVariance,
    double desired_precision, double* achieved_precision,
    csm::WarningList* warnings) const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Calculating imageToGround (covar) for {}, {}, {} with "
      "height varinace {} and desired precision {}",
      image_pt.line, image_pt.samp, height, heightVariance, desired_precision)
  // Image to ground with error propagation
  // Use numerical partials

  const double DELTA_IMAGE = 1.0;
  const double DELTA_GROUND = m_gsd;
  csm::ImageCoord ip(image_pt.line, image_pt.samp);

  csm::EcefCoord gp = UsgsAstroLsSensorModel::imageToGround(ip, height, desired_precision,
                                                            achieved_precision, warnings);

  // Compute numerical partials xyz wrt to lsh
  ip.line = image_pt.line + DELTA_IMAGE;
  ip.samp = image_pt.samp;
  csm::EcefCoord gpl = UsgsAstroLsSensorModel::imageToGround(ip, height, desired_precision);
  double xpl = (gpl.x - gp.x) / DELTA_IMAGE;
  double ypl = (gpl.y - gp.y) / DELTA_IMAGE;
  double zpl = (gpl.z - gp.z) / DELTA_IMAGE;

  ip.line = image_pt.line;
  ip.samp = image_pt.samp + DELTA_IMAGE;
  csm::EcefCoord gps = UsgsAstroLsSensorModel::imageToGround(ip, height, desired_precision);
  double xps = (gps.x - gp.x) / DELTA_IMAGE;
  double yps = (gps.y - gp.y) / DELTA_IMAGE;
  double zps = (gps.z - gp.z) / DELTA_IMAGE;

  ip.line = image_pt.line;
  ip.samp = image_pt.samp;
  csm::EcefCoord gph =
      UsgsAstroLsSensorModel::imageToGround(ip, height + DELTA_GROUND, desired_precision);
  double xph = (gph.x - gp.x) / DELTA_GROUND;
  double yph = (gph.y - gp.y) / DELTA_GROUND;
  double zph = (gph.z - gp.z) / DELTA_GROUND;

  // Convert sensor covariance to image space
  double sCov[4];
  determineSensorCovarianceInImageSpace(gp, sCov);

  std::vector<double> unmod = getUnmodeledError(image_pt);

  double iCov[4];
  iCov[0] = image_pt.covariance[0] + sCov[0] + unmod[0];
  iCov[1] = image_pt.covariance[1] + sCov[1] + unmod[1];
  iCov[2] = image_pt.covariance[2] + sCov[2] + unmod[2];
  iCov[3] = image_pt.covariance[3] + sCov[3] + unmod[3];

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
 * @brief Computes the imaging locus for a given image point proximate to a specified ground point.
 * 
 * @description This function computes the line-of-sight (imaging locus) for a specific image point that is
 * proximate to a given ground point. The computed imaging locus is the line that is closest to the given ground
 * point and lies along the sensor's line-of-sight.
 * 
 * @param image_pt The image coordinate (line and sample) for which to compute the imaging locus.
 * @param ground_pt The ground point in ECEF coordinates near which the imaging locus is to be computed.
 * @param desired_precision The desired precision for the computation, in meters.
 * @param achieved_precision A pointer to a double where the achieved precision of the computation will be stored.
 * @param warnings A pointer to a csm::WarningList for logging any warnings that occur during the computation.
 * 
 * @return csm::EcefLocus The computed imaging locus, including a point on the locus and the unit direction vector
 * of the locus in ECEF coordinates.
 */
csm::EcefLocus UsgsAstroLsSensorModel::imageToProximateImagingLocus(
    const csm::ImageCoord& image_pt, const csm::EcefCoord& ground_pt,
    double desired_precision, double* achieved_precision,
    csm::WarningList* warnings) const {
  MESSAGE_LOG(
      spdlog::level::info,
      "Computing imageToProximateImagingLocus (ground {}, {}, {}) for image "
      "point ({}, {}) with desired precision {}",
      ground_pt.x, ground_pt.y, ground_pt.z, image_pt.line, image_pt.samp,
      desired_precision);

  // Object ray unit direction near given ground location
  const double DELTA_GROUND = m_gsd;

  double x = ground_pt.x;
  double y = ground_pt.y;
  double z = ground_pt.z;

  // Elevation at input ground point
  double height = computeEllipsoidElevation(x, y, z, m_majorAxis, m_minorAxis,
                                            desired_precision);

  // Ground point on object ray with the same elevation
  csm::EcefCoord gp1 =
      UsgsAstroLsSensorModel::imageToGround(image_pt, height, desired_precision, achieved_precision);

  // Vector between 2 ground points above
  double dx1 = x - gp1.x;
  double dy1 = y - gp1.y;
  double dz1 = z - gp1.z;

  // Unit vector along object ray
  csm::EcefCoord gp2 = UsgsAstroLsSensorModel::imageToGround(image_pt, height - DELTA_GROUND,
                                                             desired_precision, achieved_precision);
  double dx2 = gp2.x - gp1.x;
  double dy2 = gp2.y - gp1.y;
  double dz2 = gp2.z - gp1.z;
  double mag2 = sqrt(dx2 * dx2 + dy2 * dy2 + dz2 * dz2);
  dx2 /= mag2;
  dy2 /= mag2;
  dz2 /= mag2;

  // Point on object ray perpendicular to input point

  // Locus
  csm::EcefLocus locus;
  double scale = dx1 * dx2 + dy1 * dy2 + dz1 * dz2;
  gp2.x = gp1.x + scale * dx2;
  gp2.y = gp1.y + scale * dy2;
  gp2.z = gp1.z + scale * dz2;

  double hLocus = computeEllipsoidElevation(gp2.x, gp2.y, gp2.z, m_majorAxis,
                                            m_minorAxis, desired_precision);
  locus.point = UsgsAstroLsSensorModel::imageToGround(image_pt, hLocus, desired_precision,
                                                      achieved_precision, warnings);

  locus.direction.x = dx2;
  locus.direction.y = dy2;
  locus.direction.z = dz2;

  return locus;
}

/**
 * @brief Computes the remote imaging locus for a given image point.
 * 
 * @description This function calculates the remote imaging locus, which is an infinitely extending line from
 * the sensor through the given image point and into space. It is used to model the direction from which the
 * sensor is viewing the Earth's surface at the specified image coordinate.
 * 
 * @param image_pt The image coordinate (line and sample) for which to compute the remote imaging locus.
 * @param desired_precision The desired precision for the calculation, in meters. This parameter influences
 * the accuracy of the computed direction vector but is not directly used in the current implementation.
 * @param achieved_precision A pointer to a double where the achieved precision of the computation will be stored.
 * This is set to 0.0 as the calculation is deterministic.
 * @param warnings A pointer to a csm::WarningList for logging any warnings that occur during the computation.
 * 
 * @return csm::EcefLocus The computed remote imaging locus, including a point on the locus (sensor position)
 * and a direction vector indicating the line of sight from the sensor through the image point.
 */
csm::EcefLocus UsgsAstroLsSensorModel::imageToRemoteImagingLocus(
    const csm::ImageCoord& image_pt, double desired_precision,
    double* achieved_precision, csm::WarningList* warnings) const {
  MESSAGE_LOG(
      spdlog::level::info,
      "Calculating imageToRemoteImagingLocus for point {}, {} with desired "
      "precision {}",
      image_pt.line, image_pt.samp, desired_precision)

  double vx, vy, vz;
  csm::EcefLocus locus;
  losToEcf(image_pt.line, image_pt.samp, _no_adjustment, locus.point.x,
           locus.point.y, locus.point.z, vx, vy, vz, locus.direction.x,
           locus.direction.y, locus.direction.z);
  // losToEcf computes the negative look vector, so negate it
  locus.direction.x = -locus.direction.x;
  locus.direction.y = -locus.direction.y;
  locus.direction.z = -locus.direction.z;

  if (achieved_precision) {
    *achieved_precision = 0.0;
  }

  MESSAGE_LOG(
      spdlog::level::info,
      "imageToProximateImagingLocus result of point ({}, {}, {}) "
      "direction ({}, {}, {}).",
      locus.point.x, locus.point.y, locus.point.z,
      locus.direction.x, locus.direction.y, locus.direction.z);
  return locus;
}

//---------------------------------------------------------------------------
// Uncertainty Propagation
//---------------------------------------------------------------------------

/**
 * @brief Computes partial derivatives of the image coordinates with respect to the ground point coordinates.
 * 
 * @description This function calculates the partial derivatives of the image line and sample (image coordinates)
 * with respect to the ground point X, Y, and Z coordinates (ECEF).
 * 
 * @param ground_pt The ground point in ECEF coordinates for which to compute partial derivatives.
 * 
 * @return std::vector<double> A vector containing the six partial derivatives:
 * [dLine/dX, dLine/dY, dLine/dZ, dSample/dX, dSample/dY, dSample/dZ].
 */
std::vector<double> UsgsAstroLsSensorModel::computeGroundPartials(
    const csm::EcefCoord& ground_pt) const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Computing computeGroundPartials for point {}, {}, {}",
      ground_pt.x, ground_pt.y, ground_pt.z)

  double GND_DELTA = m_gsd;
  // Partial of line, sample wrt X, Y, Z
  double x = ground_pt.x;
  double y = ground_pt.y;
  double z = ground_pt.z;

  csm::ImageCoord ipB = groundToImage(ground_pt);
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
 * @brief Computes the partial derivatives of the image coordinates with respect to a specific sensor model parameter.
 * 
 * @description This function calculates how changes in a specific sensor model parameter affect the image
 * coordinates of a ground point.
 * 
 * @param index The index of the sensor model parameter.
 * @param ground_pt The ground point in ECEF coordinates.
 * @param desired_precision The desired computational precision.
 * @param achieved_precision A pointer to a double where the achieved precision will be stored.
 * @param warnings A pointer to a csm::WarningList for logging any warnings.
 * 
 * @return csm::RasterGM::SensorPartials The partial derivatives of the image line and sample with respect to the specified parameter.
 */
csm::RasterGM::SensorPartials UsgsAstroLsSensorModel::computeSensorPartials(
    int index, const csm::EcefCoord& ground_pt, double desired_precision,
    double* achieved_precision, csm::WarningList* warnings) const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Calculating computeSensorPartials for ground point {}, {}, {} with "
      "desired precision {}",
      ground_pt.x, ground_pt.y, ground_pt.z, desired_precision)

  // Compute image coordinate first
  csm::ImageCoord img_pt =
      groundToImage(ground_pt, desired_precision, achieved_precision);

  // Call overloaded function
  return computeSensorPartials(index, img_pt, ground_pt, desired_precision,
                               achieved_precision, warnings);
}

/**
 * @brief Overloaded method to compute sensor partials using image coordinates.
 * 
 * @description This function calculates how changes in a specific sensor model parameter affect the image
 * coordinates of a ground point; starts with known image coordinates, avoiding the need to recompute them.
 * 
 * @param index The index of the sensor model parameter.
 * @param image_pt The image coordinates.
 * @param ground_pt The ground point in ECEF coordinates.
 * @param desired_precision The desired computational precision.
 * @param achieved_precision A pointer to a double where the achieved precision will be stored.
 * @param warnings A pointer to a csm::WarningList for logging any warnings.
 * 
 * @return csm::RasterGM::SensorPartials The partial derivatives of the image line and sample with respect to the specified parameter.
 */
csm::RasterGM::SensorPartials UsgsAstroLsSensorModel::computeSensorPartials(
    int index, const csm::ImageCoord& image_pt, const csm::EcefCoord& ground_pt,
    double desired_precision, double* achieved_precision,
    csm::WarningList* warnings) const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Calculating computeSensorPartials (with image points {}, {}) for ground "
      "point {}, {}, {} with desired precision {}",
      image_pt.line, image_pt.samp, ground_pt.x, ground_pt.y, ground_pt.z,
      desired_precision)

  // Compute numerical partials ls wrt specific parameter

  const double DELTA = m_gsd;
  std::vector<double> adj(UsgsAstroLsSensorModel::NUM_PARAMETERS, 0.0);
  adj[index] = DELTA;

  csm::ImageCoord img1 = groundToImage(ground_pt, adj, desired_precision,
                                       achieved_precision, warnings);

  double line_partial = (img1.line - image_pt.line) / DELTA;
  double sample_partial = (img1.samp - image_pt.samp) / DELTA;

  return csm::RasterGM::SensorPartials(line_partial, sample_partial);
}

/**
 * @brief Computes all sensor partial derivatives for a ground point.
 * 
 * @description This function calculates the partial derivatives of the image coordinates with respect to all
 * sensor model parameters for a given ground point.
 * 
 * @param ground_pt The ground point in ECEF coordinates.
 * @param pSet The set of parameters for which partials are to be computed.
 * @param desired_precision The desired computational precision.
 * @param achieved_precision A pointer to a double where the achieved precision will be stored.
 * @param warnings A pointer to a csm::WarningList for logging any warnings.
 * 
 * @return std::vector<csm::RasterGM::SensorPartials> A vector of sensor partials for all parameters.
 */
std::vector<csm::RasterGM::SensorPartials>
UsgsAstroLsSensorModel::computeAllSensorPartials(
    const csm::EcefCoord& ground_pt, csm::param::Set pSet,
    double desired_precision, double* achieved_precision,
    csm::WarningList* warnings) const {
  MESSAGE_LOG(
      spdlog::level::info,
      "Computing computeAllSensorPartials for ground point {}, {}, {} with "
      "desired precision {}",
      ground_pt.x, ground_pt.y, ground_pt.z, desired_precision)
  csm::ImageCoord image_pt =
      groundToImage(ground_pt, desired_precision, achieved_precision, warnings);

  return computeAllSensorPartials(image_pt, ground_pt, pSet, desired_precision,
                                  achieved_precision, warnings);
}

/**
 * @brief Computes sensor partial derivatives for an image point with respect to all parameters.
 * 
 * @description This function calculates the partial derivatives of the image line and sample coordinates with respect to
 * all adjustable parameters in the sensor model for a given image and ground point.
 * 
 * @param image_pt The image point coordinates (line, sample).
 * @param ground_pt The ground point in ECEF coordinates.
 * @param pSet The set of parameters to consider in the computation.
 * @param desired_precision The desired precision for the computation.
 * @param achieved_precision A pointer to store the achieved precision.
 * @param warnings A pointer to a csm::WarningList for logging any warnings.
 * 
 * @return A vector of csm::RasterGM::SensorPartials representing the partial derivatives for all parameters.
 */
std::vector<csm::RasterGM::SensorPartials>
UsgsAstroLsSensorModel::computeAllSensorPartials(
    const csm::ImageCoord& image_pt, const csm::EcefCoord& ground_pt,
    csm::param::Set pSet, double desired_precision, double* achieved_precision,
    csm::WarningList* warnings) const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Computing computeAllSensorPartials for image {} {} and ground {}, {}, "
      "{} with desired precision {}",
      image_pt.line, image_pt.samp, ground_pt.x, ground_pt.y, ground_pt.z,
      desired_precision)

  return RasterGM::computeAllSensorPartials(image_pt, ground_pt, pSet, desired_precision,
                                            achieved_precision, warnings);
}

/**
 * @brief Retrieves the covariance between two sensor model parameters.
 * 
 * @description Given two parameter indices, this function returns the covariance between them.
 * 
 * @param index1 The index of the first parameter.
 * @param index2 The index of the second parameter.
 * 
 * @return The covariance between the two parameters.
 */
double UsgsAstroLsSensorModel::getParameterCovariance(int index1,
                                                      int index2) const {
  int index = UsgsAstroLsSensorModel::NUM_PARAMETERS * index1 + index2;

  MESSAGE_LOG(
      spdlog::level::debug,
      "getParameterCovariance for {} {} is {}",
      index1, index2, m_covariance[index])

  return m_covariance[index];
}

/**
 * @brief Sets the covariance between two sensor model parameters.
 * 
 * @description Allows for setting the covariance between two specific sensor model parameters.
 * 
 * @param index1 The index of the first parameter.
 * @param index2 The index of the second parameter.
 * @param covariance The new covariance value to set.
 */
void UsgsAstroLsSensorModel::setParameterCovariance(int index1, int index2,
                                                    double covariance) {
  int index = UsgsAstroLsSensorModel::NUM_PARAMETERS * index1 + index2;

  MESSAGE_LOG(
      spdlog::level::debug,
      "setParameterCovariance for {} {} is {}",
      index1, index2, m_covariance[index])

  m_covariance[index] = covariance;
}

//---------------------------------------------------------------------------
// Time and Trajectory
//---------------------------------------------------------------------------

/**
 * @brief Retrieves the trajectory identifier.
 * 
 * @description Returns a string that identifies the trajectory of the sensor. This could include information like
 * mission name, trajectory number, etc.
 * 
 * @return A string identifying the trajectory.
 */
std::string UsgsAstroLsSensorModel::getTrajectoryIdentifier() const {
  return "UNKNOWN";
}

/**
 * @brief Retrieves the reference date and time for the sensor model.
 * 
 * @description This function calculates and returns the reference date and time corresponding to the sensor model.
 * 
 * @return A string representing the reference date and time.
 */
std::string UsgsAstroLsSensorModel::getReferenceDateAndTime() const {
  csm::EcefCoord referencePointGround =
    UsgsAstroLsSensorModel::getReferencePoint();
  csm::ImageCoord referencePointImage =
    UsgsAstroLsSensorModel::groundToImage(referencePointGround);
  double relativeTime =
    UsgsAstroLsSensorModel::getImageTime(referencePointImage);
  time_t ephemTime = m_centerEphemerisTime + relativeTime;

  return ephemTimeToCalendarTime(ephemTime);
}

/**
 * @brief Computes the image time for a given image point.
 * 
 * @description Calculates the time at which a specific image line (and implicitly, the entire image frame) was acquired.
 * 
 * @param image_pt The image point (line, sample) for which to calculate the image acquisition time.
 * 
 * @return The image acquisition time for the specified image point.
 */
double UsgsAstroLsSensorModel::getImageTime(
    const csm::ImageCoord& image_pt) const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "getImageTime for image line {}",
      image_pt.line)
  double lineFull = image_pt.line;

  auto referenceLineIt =
      std::upper_bound(m_intTimeLines.begin(), m_intTimeLines.end(), lineFull);
  if (referenceLineIt != m_intTimeLines.begin()) {
    --referenceLineIt;
  }
  size_t referenceIndex =
      std::distance(m_intTimeLines.begin(), referenceLineIt);

  // Adding 0.5 to the line results in center exposure time for a given line
  double time = m_intTimeStartTimes[referenceIndex] +
                m_intTimes[referenceIndex] *
                    (lineFull - m_intTimeLines[referenceIndex] + 0.5);

  MESSAGE_LOG(
      spdlog::level::debug,
      "getImageTime is {}",
      time)

  return time;
}

/**
 * @brief Retrieves the sensor position for a specified image point.
 * 
 * @description This function calculates the sensor's position in Earth-Centered Earth-Fixed (ECEF) coordinates at the
 * time the specified image point was acquired.
 * 
 * @param imagePt The image coordinates (line, sample) for which to calculate the sensor position.
 * 
 * @return The sensor's position in ECEF coordinates.
 */
csm::EcefCoord UsgsAstroLsSensorModel::getSensorPosition(
    const csm::ImageCoord& imagePt) const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "getSensorPosition at image coord ({}, {})",
      imagePt.line, imagePt.samp)

  return getSensorPosition(getImageTime(imagePt));
}

/**
 * @brief Retrieves the sensor position for a specified time.
 * 
 * @description Calculates the sensor's position in Earth-Centered Earth-Fixed (ECEF) coordinates at a specific
 * time relative to the start of the image acquisition.
 * 
 * @param time The time at which to calculate the sensor's position.
 * 
 * @return The sensor's position in ECEF coordinates.
 */
csm::EcefCoord UsgsAstroLsSensorModel::getSensorPosition(double time) const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "getSensorPosition at {}",
      time)
  double x, y, z, vx, vy, vz;
  bool calc_vel = false;
  getAdjSensorPosVel(time, _no_adjustment, x, y, z, vx, vy, vz, calc_vel);

  MESSAGE_LOG(
      spdlog::level::debug,
      "getSensorPosition is ({}, {}, {})",
      x, y, z);
  return csm::EcefCoord(x, y, z);
}

/**
 * @brief Retrieves the sensor velocity for a specified image point.
 * 
 * @description This function calculates the sensor's velocity in Earth-Centered Earth-Fixed (ECEF) coordinates at the
 * time the specified image point was acquired.
 * 
 * @param imagePt The image coordinates (line, sample) for which to calculate the sensor velocity.
 * 
 * @return The sensor's velocity as a vector in ECEF coordinates.
 */
csm::EcefVector UsgsAstroLsSensorModel::getSensorVelocity(
    const csm::ImageCoord& imagePt) const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "getSensorVelocity at image coord ({}, {})",
      imagePt.line, imagePt.samp);
  return getSensorVelocity(getImageTime(imagePt));
}

/**
 * @brief Retrieves the sensor velocity for a specified time.
 * 
 * @description Calculates the sensor's velocity in Earth-Centered Earth-Fixed (ECEF) coordinates at a specific
 * time relative to the start of the image acquisition.
 * 
 * @param time The time at which to calculate the sensor's velocity.
 * 
 * @return The sensor's velocity as a vector in ECEF coordinates.
 */
csm::EcefVector UsgsAstroLsSensorModel::getSensorVelocity(double time) const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "getSensorVelocity at {}",
      time);
  double x, y, z, vx, vy, vz;
  getAdjSensorPosVel(time, _no_adjustment, x, y, z, vx, vy, vz);


  MESSAGE_LOG(
      spdlog::level::debug,
      "getSensorVelocity is ({}, {}, {})",
      vx, vy, vz)
  return csm::EcefVector(vx, vy, vz);
}

//---------------------------------------------------------------------------
// Sensor Model Parameters
//---------------------------------------------------------------------------

/**
 * @brief Sets the value of a specific sensor model parameter.
 * 
 * @description Allows updating the value of a specific sensor parameter identified by its index.
 * 
 * @param index The index of the parameter to update.
 * @param value The new value to set for the parameter.
 */
void UsgsAstroLsSensorModel::setParameterValue(int index, double value) {
  m_currentParameterValue[index] = value;
}

/**
 * @brief Retrieves the value of a specific sensor model parameter.
 * 
 * @description Given the index of a sensor parameter, this function returns its current value.
 * 
 * @param index The index of the parameter whose value is to be retrieved.
 * 
 * @return The current value of the specified parameter.
 */
double UsgsAstroLsSensorModel::getParameterValue(int index) const {
  return m_currentParameterValue[index];
}

/**
 * @brief Retrieves the name of a specific sensor model parameter.
 * 
 * @description Given the index of a sensor parameter, this function returns its name.
 * 
 * @param index The index of the parameter whose name is to be retrieved.
 * 
 * @return The name of the specified parameter.
 */
std::string UsgsAstroLsSensorModel::getParameterName(int index) const {
  return PARAMETER_NAME[index];
}

/**
 * @brief Retrieves the units of a specific sensor model parameter.
 * 
 * @description Given the index of a sensor parameter, this function returns its units.
 * 
 * @param index The index of the parameter whose units are to be retrieved.
 * 
 * @return The units of the specified parameter.
 */
std::string UsgsAstroLsSensorModel::getParameterUnits(int index) const {
  // All parameters are meters or scaled to meters
  return "m";
}

/**
 * @brief Retrieves the total number of parameters in the sensor model.
 * 
 * @return The number of parameters.
 */
int UsgsAstroLsSensorModel::getNumParameters() const {
  return UsgsAstroLsSensorModel::NUM_PARAMETERS;
}

/**
 * @brief Retrieves the type of a specific parameter.
 * 
 * @param index The index of the parameter.
 * 
 * @return The type of the parameter.
 */
csm::param::Type UsgsAstroLsSensorModel::getParameterType(int index) const {
  return m_parameterType[index];
}

/**
 * @brief Sets the type for a specific parameter.
 * 
 * @param index The index of the parameter to set.
 * @param pType The parameter type to set.
 */
void UsgsAstroLsSensorModel::setParameterType(int index,
                                              csm::param::Type pType) {
  m_parameterType[index] = pType;
}

//---------------------------------------------------------------------------
// Sensor Model Information
//---------------------------------------------------------------------------

/**
 * @brief Retrieves the sensor model's pedigree.
 * 
 * @return The pedigree of the sensor model.
 */
std::string UsgsAstroLsSensorModel::getPedigree() const {
  return "USGS_LINE_SCANNER";
}

/**
 * @brief Retrieves the image identifier.
 * 
 * @return The image identifier.
 */
std::string UsgsAstroLsSensorModel::getImageIdentifier() const {
  return m_imageIdentifier;
}

/**
 * @brief Sets the image identifier.
 * 
 * @param imageId The new image identifier.
 * @param warnings List to populate with any warnings.
 */
void UsgsAstroLsSensorModel::setImageIdentifier(const std::string& imageId,
                                                csm::WarningList* warnings) {
  // Image id should include the suffix without the path name
  m_imageIdentifier = imageId;
}

/**
 * @brief Retrieves the sensor identifier.
 * 
 * @return The sensor identifier.
 */
std::string UsgsAstroLsSensorModel::getSensorIdentifier() const {
  return m_sensorIdentifier;
}

/**
 * @brief Retrieves the platform identifier.
 * 
 * @return The platform identifier.
 */
std::string UsgsAstroLsSensorModel::getPlatformIdentifier() const {
  return m_platformIdentifier;
}

/**
 * @brief Sets the reference point in ECEF coordinates.
 * 
 * @param ground_pt The new reference point.
 */
void UsgsAstroLsSensorModel::setReferencePoint(
    const csm::EcefCoord& ground_pt) {
  m_referencePointXyz = ground_pt;
}

/**
 * @brief Retrieves the reference point in ECEF coordinates.
 * 
 * @return The current reference point.
 */
csm::EcefCoord UsgsAstroLsSensorModel::getReferencePoint() const {
  // Return ground point at image center
  return m_referencePointXyz;
}

/**
 * @brief Retrieves the name of the sensor model.
 * 
 * @return The sensor model name.
 */
std::string UsgsAstroLsSensorModel::getModelName() const {
  return UsgsAstroLsSensorModel::_SENSOR_MODEL_NAME;
}

/**
 * @brief Retrieves the start coordinate of the image.
 * 
 * @return The start coordinate of the image.
 */
csm::ImageCoord UsgsAstroLsSensorModel::getImageStart() const {
  return csm::ImageCoord(0.0, 0.0);
}

/**
 * @brief Retrieves the size of the image.
 * 
 * @return The size of the image.
 */
csm::ImageVector UsgsAstroLsSensorModel::getImageSize() const {
  return csm::ImageVector(m_nLines, m_nSamples);
}

//---------------------------------------------------------------------------
//  Monoscopic Mensuration
//---------------------------------------------------------------------------

/**
 * @brief Retrieves the valid height range for the sensor model.
 * 
 * @return A pair representing the minimum and maximum valid heights.
 */
std::pair<double, double> UsgsAstroLsSensorModel::getValidHeightRange() const {
  return std::pair<double, double>(m_minElevation, m_maxElevation);
}


/**
 * @brief Retrieves the valid range of image coordinates for this sensor model.
 * 
 * @return A pair of image coordinates specifying the valid range, where the first
 * element is the start coordinate (top-left corner) and the second element
 * is the end coordinate (bottom-right corner).
 */
std::pair<csm::ImageCoord, csm::ImageCoord>
UsgsAstroLsSensorModel::getValidImageRange() const {
  return std::pair<csm::ImageCoord, csm::ImageCoord>(
      csm::ImageCoord(0.0, 0.0),
      csm::ImageCoord(m_nLines,
                      m_nSamples));  // Technically nl and ns are outside the
                                     // image in a zero based system.
}

/**
 * @brief Retrieves the illumination direction for a given ground point.
 * 
 * @param groundPt The ground point for which the illumination direction is queried.
 * 
 * @return The unit vector representing the direction of illumination.
 */
csm::EcefVector UsgsAstroLsSensorModel::getIlluminationDirection(
    const csm::EcefCoord& groundPt) const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Accessing illumination direction of ground point"
      "{} {} {}.",
      groundPt.x, groundPt.y, groundPt.z);

  csm::EcefVector sunPosition =
      getSunPosition(getImageTime(groundToImage(groundPt)));
  csm::EcefVector illuminationDirection =
      csm::EcefVector(groundPt.x - sunPosition.x, groundPt.y - sunPosition.y,
                      groundPt.z - sunPosition.z);

  double scale = sqrt(illuminationDirection.x * illuminationDirection.x +
                      illuminationDirection.y * illuminationDirection.y +
                      illuminationDirection.z * illuminationDirection.z);

  illuminationDirection.x /= scale;
  illuminationDirection.y /= scale;
  illuminationDirection.z /= scale;
  return illuminationDirection;
}

//---------------------------------------------------------------------------
//  Error Correction
//---------------------------------------------------------------------------

/**
 * @brief Retrieves the number of geometric correction switches available in the model.
 * 
 * @return The number of geometric correction switches.
 */
int UsgsAstroLsSensorModel::getNumGeometricCorrectionSwitches() const {
  return 0;
}

/**
 * @brief Retrieves the name of a specific geometric correction switch.
 * 
 * @param index The index of the correction switch.
 * 
 * @return The name of the geometric correction switch.
 */
std::string UsgsAstroLsSensorModel::getGeometricCorrectionName(
    int index) const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Accessing name of geometric correction switch {}. "
      "Geometric correction switches are not supported, throwing exception",
      index);
  // Since there are no geometric corrections, all indices are out of range
  throw csm::Error(csm::Error::INDEX_OUT_OF_RANGE, "Index is out of range.",
                   "UsgsAstroLsSensorModel::getGeometricCorrectionName");
}

/**
 * @brief Sets the state of a specific geometric correction switch.
 * 
 * @param index The index of the correction switch to set.
 * @param value The new state of the correction switch.
 * @param pType The parameter type associated with the correction switch.
 */
void UsgsAstroLsSensorModel::setGeometricCorrectionSwitch(
    int index, bool value, csm::param::Type pType) {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Setting geometric correction switch {} to {} "
      "with parameter type {}. "
      "Geometric correction switches are not supported, throwing exception",
      index, value, pType);
  // Since there are no geometric corrections, all indices are out of range
  throw csm::Error(csm::Error::INDEX_OUT_OF_RANGE, "Index is out of range.",
                   "UsgsAstroLsSensorModel::setGeometricCorrectionSwitch");
}

/**
 * @brief Retrieves the state of a specific geometric correction switch.
 * 
 * @param index The index of the correction switch.
 * 
 * @return The state of the geometric correction switch.
 */
bool UsgsAstroLsSensorModel::getGeometricCorrectionSwitch(int index) const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Accessing value of geometric correction switch {}. "
      "Geometric correction switches are not supported, throwing exception",
      index);
  // Since there are no geometric corrections, all indices are out of range
  throw csm::Error(csm::Error::INDEX_OUT_OF_RANGE, "Index is out of range.",
                   "UsgsAstroLsSensorModel::getGeometricCorrectionSwitch");
}

/**
 * @brief Retrieves the cross covariance matrix between this model and another model.
 * 
 * @param comparisonModel The other geometric model to compare against.
 * @param pSet The parameter set for which the covariance is requested.
 * @param otherModels A list of other geometric models in the scene.
 * 
 * @return A vector representing the cross covariance matrix.
 */
std::vector<double> UsgsAstroLsSensorModel::getCrossCovarianceMatrix(
    const csm::GeometricModel& comparisonModel, csm::param::Set pSet,
    const csm::GeometricModel::GeometricModelList& otherModels) const {
  // Return covariance matrix
  if (&comparisonModel == this) {
    std::vector<int> paramIndices = getParameterSetIndices(pSet);
    int numParams = paramIndices.size();
    std::vector<double> covariances(numParams * numParams, 0.0);
    for (int i = 0; i < numParams; i++) {
      for (int j = 0; j < numParams; j++) {
        covariances[i * numParams + j] =
            getParameterCovariance(paramIndices[i], paramIndices[j]);
      }
    }
    return covariances;
  }
  // No correlation between models.
  const std::vector<int>& indices = getParameterSetIndices(pSet);
  size_t num_rows = indices.size();
  const std::vector<int>& indices2 =
      comparisonModel.getParameterSetIndices(pSet);
  size_t num_cols = indices.size();

  return std::vector<double>(num_rows * num_cols, 0.0);
}

/**
 * @brief Retrieves the correlation model for the sensor.
 * 
 * @return A reference to the correlation model.
 */
const csm::CorrelationModel& UsgsAstroLsSensorModel::getCorrelationModel()
    const {
  // All Line Scanner images are assumed uncorrelated
  return _no_corr_model;
}

/**
 * @brief Retrieves the unmodeled cross covariance between two image points.
 * 
 * @param pt1 The first image point.
 * @param pt2 The second image point.
 * 
 * @return A vector containing the unmodeled cross covariance values.
 */
std::vector<double> UsgsAstroLsSensorModel::getUnmodeledCrossCovariance(
    const csm::ImageCoord& pt1, const csm::ImageCoord& pt2) const {
  // No unmodeled error
  return std::vector<double>(4, 0.0);
}

/**
 * @brief Retrieves the identifier for the collection to which this sensor model belongs.
 * 
 * @return A string representing the collection identifier.
 */
std::string UsgsAstroLsSensorModel::getCollectionIdentifier() const {
  return "UNKNOWN";
}

/**
 * @brief Checks if the sensor model has parameters that can be shared across multiple instances.
 * 
 * @return True if parameters can be shared, false otherwise.
 */
bool UsgsAstroLsSensorModel::hasShareableParameters() const {
  // Parameter sharing is not supported for this sensor
  return false;
}

/**
 * @brief Determines if a specific parameter can be shared across multiple sensor model instances.
 * 
 * @param index The index of the parameter.
 * 
 * @return True if the parameter can be shared, false otherwise.
 */
bool UsgsAstroLsSensorModel::isParameterShareable(int index) const {
  // Parameter sharing is not supported for this sensor
  return false;
}

/**
 * @brief Retrieves the criteria for sharing a specific parameter.
 * 
 * @param index The index of the parameter.
 * 
 * @return The sharing criteria for the specified parameter.
 */
csm::SharingCriteria UsgsAstroLsSensorModel::getParameterSharingCriteria(
    int index) const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Checking sharing criteria for parameter {}. "
      "Sharing is not supported.",
      index);
  return csm::SharingCriteria();
}

/**
 * @brief Retrieves the sensor type.
 * 
 * @return A string representing the sensor type.
 */
std::string UsgsAstroLsSensorModel::getSensorType() const {
  return CSM_SENSOR_TYPE_EO;
}

/**
 * @brief Retrieves the sensor mode.
 * 
 * @return A string representing the sensor mode.
 */
std::string UsgsAstroLsSensorModel::getSensorMode() const {
  return CSM_SENSOR_MODE_PB;
}

/**
 * @brief Retrieves the version of the sensor model.
 * 
 * @return A csm::Version object representing the version.
 */
csm::Version UsgsAstroLsSensorModel::getVersion() const {
  return csm::Version(1, 0, 0);
}

/**
 * @brief Retrieves the ellipsoid used by the sensor model.
 * 
 * @return A csm::Ellipsoid object representing the ellipsoid.
 */
csm::Ellipsoid UsgsAstroLsSensorModel::getEllipsoid() const {
  return csm::Ellipsoid(m_majorAxis, m_minorAxis);
}

/**
 * @brief Sets the ellipsoid used by the sensor model.
 * 
 * @param ellipsoid The new ellipsoid to use.
 */
void UsgsAstroLsSensorModel::setEllipsoid(const csm::Ellipsoid& ellipsoid) {
  m_majorAxis = ellipsoid.getSemiMajorRadius();
  m_minorAxis = ellipsoid.getSemiMinorRadius();
}

/**
 * @brief Retrieves the value of a parameter, adjusted by the specified adjustments.
 * 
 * @param index The index of the parameter.
 * @param adjustments The adjustments to apply to the parameter value.
 * 
 * @return The adjusted parameter value.
 */
double UsgsAstroLsSensorModel::getValue(
    int index, const std::vector<double>& adjustments) const {
  return m_currentParameterValue[index] + adjustments[index];
}

/**
 * @brief Retrieves the quaternions representing the sensor's orientation at a given time.
 * 
 * @param time The time at which to retrieve the quaternions.
 * @param q The array to store the quaternion values.
 */
void UsgsAstroLsSensorModel::getQuaternions(const double& time,
                                            double q[4]) const {
  int nOrder = 8;
  if (m_platformFlag == 0) nOrder = 4;
  int nOrderQuat = nOrder;
  if (m_numQuaternions < 6 && nOrder == 8) nOrderQuat = 4;

  MESSAGE_LOG(
      spdlog::level::debug,
      "Calculating getQuaternions for time {} with {}"
      "order lagrange",
      time, nOrder)
  lagrangeInterp(m_numQuaternions / 4, &m_quaternions[0], m_t0Quat, m_dtQuat,
                 time, 4, nOrderQuat, q);
}

/**
 * @brief Calculates the attitude correction matrix for a given time and set of adjustments.
 * 
 * @param time The time at which to calculate the attitude correction.
 * @param adj The adjustments to apply.
 * @param attCorr The array to store the attitude correction matrix.
 */
void UsgsAstroLsSensorModel::calculateAttitudeCorrection(
    const double& time, const std::vector<double>& adj,
    double attCorr[9]) const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Computing calculateAttitudeCorrection (with adjustment)"
      "for time {}",
      time)
  double aTime = time - m_t0Quat;
  double euler[3];
  double nTime = aTime / m_halfTime;
  double nTime2 = nTime * nTime;
  euler[0] = (getValue(6, adj) + getValue(9, adj) * nTime +
              getValue(12, adj) * nTime2) /
             m_flyingHeight;
  euler[1] = (getValue(7, adj) + getValue(10, adj) * nTime +
              getValue(13, adj) * nTime2) /
             m_flyingHeight;
  euler[2] = (getValue(8, adj) + getValue(11, adj) * nTime +
              getValue(14, adj) * nTime2) /
             m_halfSwath;
  MESSAGE_LOG(
      spdlog::level::trace,
      "calculateAttitudeCorrection: euler {} {} {}",
      euler[0], euler[1], euler[2])

  calculateRotationMatrixFromEuler(euler, attCorr);
}

/**
 * @brief Transforms a line-of-sight vector in image coordinates to the Earth-Centered Fixed (ECF) coordinate system.
 * 
 * @description Computes the ECF coordinates of the sensor position and velocity, as well as the line-of-sight vector
 * in the body-fixed frame. The function accounts for distortion and corrects the line-of-sight vector using the sensor
 * model parameters and adjustments.
 * 
 * @param line The image line coordinate (zero-based).
 * @param sample The image sample coordinate (zero-based, UL pixel center == (0.5, 0.5)).
 * @param adj Adjustments to the sensor model parameters.
 * @param xc Output sensor X coordinate in ECF.
 * @param yc Output sensor Y coordinate in ECF.
 * @param zc Output sensor Z coordinate in ECF.
 * @param vx Output sensor velocity along the X axis.
 * @param vy Output sensor velocity along the Y axis.
 * @param vz Output sensor velocity along the Z axis.
 * @param bodyLookX Output line-of-sight vector X component in body-fixed frame.
 * @param bodyLookY Output line-of-sight vector Y component in body-fixed frame.
 * @param bodyLookZ Output line-of-sight vector Z component in body-fixed frame.
 */
void UsgsAstroLsSensorModel::losToEcf(
    const double& line,              // CSM image convention
    const double& sample,            // UL pixel center == (0.5, 0.5)
    const std::vector<double>& adj,  // Parameter Adjustments for partials
    double& xc,                      // output sensor x coordinate
    double& yc,                      // output sensor y coordinate
    double& zc,                      // output sensor z coordinate
    double& vx,                      // output sensor x velocity
    double& vy,                      // output sensor y velocity
    double& vz,                      // output sensor z velocity
    double& bodyLookX,               // output line-of-sight x coordinate
    double& bodyLookY,               // output line-of-sight y coordinate
    double& bodyLookZ) const         // output line-of-sight z coordinate
{
  //# private_func_description
  // Computes image ray (look vector) in ecf coordinate system.
  // Compute adjusted sensor position and velocity
  MESSAGE_LOG(
      spdlog::level::debug,
      "Computing losToEcf (with adjustments) for"
      "line {} sample {}",
      line, sample);

  double time = getImageTime(csm::ImageCoord(line, sample));
  bool calc_vel = false;
  getAdjSensorPosVel(time, adj, xc, yc, zc, vx, vy, vz, calc_vel);
  // CSM image image convention: UL pixel center == (0.5, 0.5)
  // USGS image convention: UL pixel center == (1.0, 1.0)
  double sampleCSMFull = sample;
  double sampleUSGSFull = sampleCSMFull;

  // Compute distorted image coordinates in mm (sample, line on image (pixels)
  // -> focal plane
  double distortedFocalPlaneX, distortedFocalPlaneY;
  computeDistortedFocalPlaneCoordinates(
      0.0, sampleUSGSFull, m_detectorSampleOrigin, m_detectorLineOrigin,
      m_detectorSampleSumming, m_detectorLineSumming, m_startingDetectorSample,
      m_startingDetectorLine, m_iTransS, m_iTransL, distortedFocalPlaneX,
      distortedFocalPlaneY);
  MESSAGE_LOG(
      spdlog::level::trace,
      "losToEcf: distorted focal plane coordinate {} {}",
      distortedFocalPlaneX, distortedFocalPlaneY)

  // Remove lens
  double undistortedFocalPlaneX, undistortedFocalPlaneY;
  removeDistortion(distortedFocalPlaneX, distortedFocalPlaneY,
                   undistortedFocalPlaneX, undistortedFocalPlaneY,
                   m_opticalDistCoeffs, m_focalLength, m_distortionType);
  MESSAGE_LOG(
      spdlog::level::trace,
      "losToEcf: undistorted focal plane coordinate {} {}",
      undistortedFocalPlaneX, undistortedFocalPlaneY)

  // Define imaging ray (look vector) in camera space
  double cameraLook[3];
  createCameraLookVector(
      undistortedFocalPlaneX, undistortedFocalPlaneY, m_zDirection,
      m_focalLength * (1 - getValue(15, adj) / m_halfSwath), cameraLook);
  MESSAGE_LOG(
      spdlog::level::trace,
      "losToEcf: uncorrected camera look vector {} {} {}",
      cameraLook[0], cameraLook[1], cameraLook[2])

  // Apply attitude correction
  double attCorr[9];
  calculateAttitudeCorrection(time, adj, attCorr);

  double correctedCameraLook[3];
  correctedCameraLook[0] = attCorr[0] * cameraLook[0] +
                           attCorr[1] * cameraLook[1] +
                           attCorr[2] * cameraLook[2];
  correctedCameraLook[1] = attCorr[3] * cameraLook[0] +
                           attCorr[4] * cameraLook[1] +
                           attCorr[5] * cameraLook[2];
  correctedCameraLook[2] = attCorr[6] * cameraLook[0] +
                           attCorr[7] * cameraLook[1] +
                           attCorr[8] * cameraLook[2];
  MESSAGE_LOG(
      spdlog::level::trace,
      "losToEcf: corrected camera look vector {} {} {}",
      correctedCameraLook[0], correctedCameraLook[1], correctedCameraLook[2])
  // Rotate the look vector into the body fixed frame from the camera reference
  // frame by applying the rotation matrix from the sensor quaternions
  double quaternions[4];
  getQuaternions(time, quaternions);
  double cameraToBody[9];
  calculateRotationMatrixFromQuaternions(quaternions, cameraToBody);

  bodyLookX = cameraToBody[0] * correctedCameraLook[0] +
              cameraToBody[1] * correctedCameraLook[1] +
              cameraToBody[2] * correctedCameraLook[2];
  bodyLookY = cameraToBody[3] * correctedCameraLook[0] +
              cameraToBody[4] * correctedCameraLook[1] +
              cameraToBody[5] * correctedCameraLook[2];
  bodyLookZ = cameraToBody[6] * correctedCameraLook[0] +
              cameraToBody[7] * correctedCameraLook[1] +
              cameraToBody[8] * correctedCameraLook[2];
  MESSAGE_LOG(
      spdlog::level::debug,
      "losToEcf: body look vector {} {} {}",
      bodyLookX, bodyLookY, bodyLookZ)
}

/**
 * @brief Corrects the line-of-sight vector for light aberration.
 * 
 * @description The correction is based on the relative motion between the sensor and the observed surface.
 * This function calculates the correction vector due to light aberration, which is the apparent displacement
 * of the position of a celestial object from its true position (or geometric position), caused by the motion
 * of the observer.
 * 
 * @param vx Sensor velocity along the X axis.
 * @param vy Sensor velocity along the Y axis.
 * @param vz Sensor velocity along the Z axis.
 * @param xl Initial line-of-sight vector X component.
 * @param yl Initial line-of-sight vector Y component.
 * @param zl Initial line-of-sight vector Z component.
 * @param dxl Output corrected line-of-sight vector X component.
 * @param dyl Output corrected line-of-sight vector Y component.
 * @param dzl Output corrected line-of-sight vector Z component.
 */
void UsgsAstroLsSensorModel::lightAberrationCorr(
    const double& vx, const double& vy, const double& vz, const double& xl,
    const double& yl, const double& zl, double& dxl, double& dyl,
    double& dzl) const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Computing lightAberrationCorr for camera velocity"
      "{} {} {} and image ray {} {} {}",
      vx, vy, vz, xl, yl, zl)
  //# func_description
  //  Computes light aberration correction vector

  // Compute angle between the image ray and the velocity vector

  double dotP = xl * vx + yl * vy + zl * vz;
  double losMag = sqrt(xl * xl + yl * yl + zl * zl);
  double velocityMag = sqrt(vx * vx + vy * vy + vz * vz);
  double cosThetap = dotP / (losMag * velocityMag);
  double sinThetap = sqrt(1.0 - cosThetap * cosThetap);

  // Image ray is parallel to the velocity vector

  if (1.0 == fabs(cosThetap)) {
    dxl = 0.0;
    dyl = 0.0;
    dzl = 0.0;
    MESSAGE_LOG(
        spdlog::level::warn,
        "lightAberrationCorr: image ray is parallel"
        "to velocity vector")
  }

  // Compute the angle between the corrected image ray and spacecraft
  // velocity.  This key equation is derived using Lorentz transform.

  double speedOfLight = 299792458.0;  // meters per second
  double beta = velocityMag / speedOfLight;
  double cosTheta = (beta - cosThetap) / (beta * cosThetap - 1.0);
  double sinTheta = sqrt(1.0 - cosTheta * cosTheta);

  // Compute line-of-sight correction

  double cfac = ((cosTheta * sinThetap - sinTheta * cosThetap) * losMag) /
                (sinTheta * velocityMag);
  dxl = cfac * vx;
  dyl = cfac * vy;
  dzl = cfac * vz;
  MESSAGE_LOG(
      spdlog::level::debug,
      "lightAberrationCorr: light of sight correction"
      "{} {} {}",
      dxl, dyl, dzl)
}

/**
 * @brief Computes the intersection of a line-of-sight vector with an ellipsoid representing the Earth.
 * 
 * @description This function calculates the intersection point of a ray originating from a given camera position
 * (xc, yc, zc) and directed along a vector (xl, yl, zl) with an ellipsoid defined by the sensor model's major
 * and minor axes. The function also provides the achieved precision of the calculation compared to the
 * desired precision and warns if the line-of-sight does not intersect the ellipsoid.
 * 
 * @param height The height above the ellipsoid at which to calculate the intersection.
 * @param xc The X coordinate of the camera position in ECF.
 * @param yc The Y coordinate of the camera position in ECF.
 * @param zc The Z coordinate of the camera position in ECF.
 * @param xl The X component of the line-of-sight vector.
 * @param yl The Y component of the line-of-sight vector.
 * @param zl The Z component of the line-of-sight vector.
 * @param x Output X coordinate of the intersection point in ECF.
 * @param y Output Y coordinate of the intersection point in ECF.
 * @param z Output Z coordinate of the intersection point in ECF.
 * @param achieved_precision The achieved precision of the intersection calculation.
 * @param desired_precision The desired precision for the intersection calculation.
 * @param warnings A list to hold any warnings generated during the calculation.
 */
void UsgsAstroLsSensorModel::losEllipsoidIntersect(
    const double& height, const double& xc, const double& yc, const double& zc,
    const double& xl, const double& yl, const double& zl, double& x, double& y,
    double& z, double& achieved_precision,
    const double& desired_precision, csm::WarningList* warnings) const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Computing losEllipsoidIntersect for camera position "
      "{} {} {} looking {} {} {} with desired precision {}",
      xc, yc, zc, xl, yl, zl, desired_precision)

  // Helper function which computes the intersection of the image ray
  // with the ellipsoid.  All vectors are in earth-centered-fixed
  // coordinate system with origin at the center of the earth.

  const int MKTR = 10;

  double ap, bp, k;
  ap = m_majorAxis + height;
  bp = m_minorAxis + height;
  k = ap * ap / (bp * bp);

  // Solve quadratic equation for scale factor
  // applied to image ray to compute ground point

  double at, bt, ct, quadTerm;
  at = xl * xl + yl * yl + k * zl * zl;
  bt = 2.0 * (xl * xc + yl * yc + k * zl * zc);
  ct = xc * xc + yc * yc + k * zc * zc - ap * ap;
  quadTerm = bt * bt - 4.0 * at * ct;

  // If quadTerm is negative, the image ray does not
  // intersect the ellipsoid. Setting the quadTerm to
  // zero means solving for a point on the ray nearest
  // the surface of the ellisoid.

  if (0.0 > quadTerm) {
    quadTerm = 0.0;
    std::string message = "Image ray does not intersect ellipsoid";
    if (warnings) {
      warnings->push_back(csm::Warning(
          csm::Warning::NO_INTERSECTION, message, "UsgsAstroLsSensorModel::losElliposidIntersect"));
    }
    MESSAGE_LOG(spdlog::level::warn, message)
  }
  double scale, scale1, h;
  double sprev, hprev;
  double sTerm;
  int ktr = 0;

  // Compute ground point vector

  sTerm = sqrt(quadTerm);
  scale = (-bt - sTerm);
  scale1 = (-bt + sTerm);
  if (fabs(scale1) < fabs(scale)) scale = scale1;
  scale /= (2.0 * at);
  x = xc + scale * xl;
  y = yc + scale * yl;
  z = zc + scale * zl;
  h = computeEllipsoidElevation(x, y, z, m_majorAxis, m_minorAxis,
                                desired_precision);

  achieved_precision = fabs(height - h);
  MESSAGE_LOG(
      spdlog::level::debug,
      "losEllipsoidIntersect: found intersect at {} {} {}"
      "with achieved precision of {}",
      x, y, z, achieved_precision)
}

/**
 * @brief Adjusts the sensor position and velocity based on the model parameters and adjustments.
 * 
 * @description This function calculates the adjusted sensor position (xc, yc, zc) and velocity (vx, vy, vz)
 * at a specified time. It considers both nominal sensor positions/velocities and model parameter adjustments.
 * The function can optionally calculate the sensor velocity.
 * 
 * @param time The time at which to calculate the sensor position and velocity.
 * @param adj A vector of adjustments to the sensor model parameters.
 * @param xc Output adjusted sensor X position in ECF.
 * @param yc Output adjusted sensor Y position in ECF.
 * @param zc Output adjusted sensor Z position in ECF.
 * @param vx Output adjusted sensor X velocity.
 * @param vy Output adjusted sensor Y velocity.
 * @param vz Output adjusted sensor Z velocity.
 * @param calc_vel Flag indicating whether to calculate the velocity.
 */
void UsgsAstroLsSensorModel::getAdjSensorPosVel(const double& time,
                                                const std::vector<double>& adj,
                                                double& xc, double& yc,
                                                double& zc, double& vx,
                                                double& vy, double& vz,
                                                bool calc_vel) const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Calculating getAdjSensorPosVel at time {}",
      time)

  // Sensor position and velocity (4th or 8th order Lagrange).
  int nOrder = 8;
  if (m_platformFlag == 0) nOrder = 4;
  double sensPosNom[3];
  lagrangeInterp(m_numPositions / 3, &m_positions[0], m_t0Ephem, m_dtEphem,
                 time, 3, nOrder, sensPosNom);

  // Avoid computing the velocity and adjustments, if not needed, as those
  // take up at least half of this function's time. Note that if the
  // adjustments are non-null, need to compute the velocity in order
  // to find the adjusted position.
  if (!calc_vel) {
    bool has_adj = false;
    for (size_t it = 0; it < adj.size(); it++) {
      if (getValue(it, adj) != 0) has_adj = true;
    }
    if (!has_adj) {
      xc = sensPosNom[0];
      yc = sensPosNom[1];
      zc = sensPosNom[2];
      vx = 0.0;
      vy = 0.0;
      vz = 0.0;

      MESSAGE_LOG(
          spdlog::level::debug,
          "getAdjSensorPosVel: no velocity, postition ({}, {}, {})",
          xc, yc, zc)
      return;
    }
  }

  double sensVelNom[3];
  lagrangeInterp(m_numPositions / 3, &m_velocities[0], m_t0Ephem, m_dtEphem,
                 time, 3, nOrder, sensVelNom);

  // Compute rotation matrix from ICR to ECF
  double radialUnitVec[3];
  double radMag =
      sqrt(sensPosNom[0] * sensPosNom[0] + sensPosNom[1] * sensPosNom[1] +
           sensPosNom[2] * sensPosNom[2]);
  for (int i = 0; i < 3; i++) radialUnitVec[i] = sensPosNom[i] / radMag;
  double crossTrackUnitVec[3];
  crossTrackUnitVec[0] =
      sensPosNom[1] * sensVelNom[2] - sensPosNom[2] * sensVelNom[1];
  crossTrackUnitVec[1] =
      sensPosNom[2] * sensVelNom[0] - sensPosNom[0] * sensVelNom[2];
  crossTrackUnitVec[2] =
      sensPosNom[0] * sensVelNom[1] - sensPosNom[1] * sensVelNom[0];
  double crossMag = sqrt(crossTrackUnitVec[0] * crossTrackUnitVec[0] +
                         crossTrackUnitVec[1] * crossTrackUnitVec[1] +
                         crossTrackUnitVec[2] * crossTrackUnitVec[2]);
  for (int i = 0; i < 3; i++) crossTrackUnitVec[i] /= crossMag;
  double inTrackUnitVec[3];
  inTrackUnitVec[0] = crossTrackUnitVec[1] * radialUnitVec[2] -
                      crossTrackUnitVec[2] * radialUnitVec[1];
  inTrackUnitVec[1] = crossTrackUnitVec[2] * radialUnitVec[0] -
                      crossTrackUnitVec[0] * radialUnitVec[2];
  inTrackUnitVec[2] = crossTrackUnitVec[0] * radialUnitVec[1] -
                      crossTrackUnitVec[1] * radialUnitVec[0];
  double ecfFromIcr[9];
  ecfFromIcr[0] = inTrackUnitVec[0];
  ecfFromIcr[1] = crossTrackUnitVec[0];
  ecfFromIcr[2] = radialUnitVec[0];
  ecfFromIcr[3] = inTrackUnitVec[1];
  ecfFromIcr[4] = crossTrackUnitVec[1];
  ecfFromIcr[5] = radialUnitVec[1];
  ecfFromIcr[6] = inTrackUnitVec[2];
  ecfFromIcr[7] = crossTrackUnitVec[2];
  ecfFromIcr[8] = radialUnitVec[2];

  // Apply position and velocity corrections
  double aTime = time - m_t0Ephem;
  double dvi = getValue(3, adj) / m_halfTime;
  double dvc = getValue(4, adj) / m_halfTime;
  double dvr = getValue(5, adj) / m_halfTime;
  vx = sensVelNom[0] + ecfFromIcr[0] * dvi + ecfFromIcr[1] * dvc +
       ecfFromIcr[2] * dvr;
  vy = sensVelNom[1] + ecfFromIcr[3] * dvi + ecfFromIcr[4] * dvc +
       ecfFromIcr[5] * dvr;
  vz = sensVelNom[2] + ecfFromIcr[6] * dvi + ecfFromIcr[7] * dvc +
       ecfFromIcr[8] * dvr;
  double di = getValue(0, adj) + dvi * aTime;
  double dc = getValue(1, adj) + dvc * aTime;
  double dr = getValue(2, adj) + dvr * aTime;
  xc = sensPosNom[0] + ecfFromIcr[0] * di + ecfFromIcr[1] * dc +
       ecfFromIcr[2] * dr;
  yc = sensPosNom[1] + ecfFromIcr[3] * di + ecfFromIcr[4] * dc +
       ecfFromIcr[5] * dr;
  zc = sensPosNom[2] + ecfFromIcr[6] * di + ecfFromIcr[7] * dc +
       ecfFromIcr[8] * dr;

  MESSAGE_LOG(
      spdlog::level::debug,
      "getAdjSensorPosVel: postition ({}, {}, {}) "
      "and velocity ({}, {}, {})",
      xc, yc, zc, vx, vy, vz)
}

/**
 * @brief Computes the detector view for a ground point with adjustments.
 * 
 * @description Calculates focal plane coordinates (x, y) corresponding to a
 * ground point at a specific time, considering model adjustments.
 * 
 * @param time Time at which the observation is made.
 * @param groundPoint The ground point being observed.
 * @param adj Adjustments to the sensor model parameters.
 * 
 * @return Vector containing the focal plane coordinates (x, y).
 */
std::vector<double> UsgsAstroLsSensorModel::computeDetectorView(
    const double& time, const csm::EcefCoord& groundPoint,
    const std::vector<double>& adj) const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Computing computeDetectorView (with adjusments)"
      "for ground point {} {} {} at time {} ",
      groundPoint.x, groundPoint.y, groundPoint.z, time);

  // Helper function to compute the CCD pixel that views a ground point based
  // on the exterior orientation at a given time.

  // Get the exterior orientation
  double xc, yc, zc, vx, vy, vz;
  bool calc_vel = false;
  getAdjSensorPosVel(time, adj, xc, yc, zc, vx, vy, vz, calc_vel);

  // Compute the look vector
  double bodyLookX = groundPoint.x - xc;
  double bodyLookY = groundPoint.y - yc;
  double bodyLookZ = groundPoint.z - zc;
  MESSAGE_LOG(
      spdlog::level::trace,
      "computeDetectorView: look vector {} {} {}",
      bodyLookX, bodyLookY, bodyLookZ)

  // Rotate the look vector into the camera reference frame
  double quaternions[4];
  getQuaternions(time, quaternions);
  double bodyToCamera[9];
  calculateRotationMatrixFromQuaternions(quaternions, bodyToCamera);

  // Apply transpose of matrix to rotate body->camera
  double cameraLookX = bodyToCamera[0] * bodyLookX +
                       bodyToCamera[3] * bodyLookY +
                       bodyToCamera[6] * bodyLookZ;
  double cameraLookY = bodyToCamera[1] * bodyLookX +
                       bodyToCamera[4] * bodyLookY +
                       bodyToCamera[7] * bodyLookZ;
  double cameraLookZ = bodyToCamera[2] * bodyLookX +
                       bodyToCamera[5] * bodyLookY +
                       bodyToCamera[8] * bodyLookZ;
  MESSAGE_LOG(
      spdlog::level::trace,
      "computeDetectorView: look vector (camera ref frame)"
      "{} {} {}",
      cameraLookX, cameraLookY, cameraLookZ)

  // Invert the attitude correction
  double attCorr[9];
  calculateAttitudeCorrection(time, adj, attCorr);

  // Apply transpose of matrix to invert the attidue correction
  double adjustedLookX = attCorr[0] * cameraLookX + attCorr[3] * cameraLookY +
                         attCorr[6] * cameraLookZ;
  double adjustedLookY = attCorr[1] * cameraLookX + attCorr[4] * cameraLookY +
                         attCorr[7] * cameraLookZ;
  double adjustedLookZ = attCorr[2] * cameraLookX + attCorr[5] * cameraLookY +
                         attCorr[8] * cameraLookZ;
  MESSAGE_LOG(
      spdlog::level::trace,
      "computeDetectorView: adjusted look vector"
      "{} {} {}",
      adjustedLookX, adjustedLookY, adjustedLookZ)

  // Convert to focal plane coordinate
  double lookScale = (m_focalLength + getValue(15, adj)) / adjustedLookZ;
  double focalX = adjustedLookX * lookScale;
  double focalY = adjustedLookY * lookScale;

  MESSAGE_LOG(
      spdlog::level::debug,
      "computeDetectorView: focal plane coordinates"
      "x:{} y:{} scale:{}",
      focalX, focalY, lookScale)

  return std::vector<double>{focalX, focalY};
}

/**
 * @brief Computes a projective approximation for ground to image conversion.
 * 
 * @description Provides a rapid, approximate conversion from ground
 * coordinates to image coordinates using a projective transformation. If the
 * projective transform has not been initialized, it defaults to the image
 * center.
 * 
 * @param gp The ground point to be approximated.
 * @param ip Reference to store the approximate image point.
 */
void UsgsAstroLsSensorModel::computeProjectiveApproximation(const csm::EcefCoord& gp,
                                                            csm::ImageCoord& ip) const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Computing projective approximation for ground point ({}, {}, {})",
      gp.x, gp.y, gp.z);
  if (m_useApproxInitTrans) {
    std::vector<double> const& u = m_projTransCoeffs; // alias, to save on typing

    double x = gp.x, y = gp.y, z = gp.z;
    double line_den = 1 + u[4]  * x + u[5]  * y + u[6]  * z;
    double samp_den = 1 + u[11] * x + u[12] * y + u[13] * z;

    // Sanity checks. Ensure we don't divide by 0 and that the numbers are valid.
    if (line_den == 0.0 || std::isnan(line_den) || std::isinf(line_den) ||
        samp_den == 0.0 || std::isnan(samp_den) || std::isinf(samp_den)) {

      ip.line = m_nLines / 2.0;
      ip.samp = m_nSamples / 2.0;
      MESSAGE_LOG(
          spdlog::level::warn,
          "Computing initial guess with constant approx line/2 and sample/2");

      return;
    }

    // Apply the formula
    ip.line = (u[0] + u[1] * x + u[2] * y + u[3]  * z) / line_den;
    ip.samp = (u[7] + u[8] * x + u[9] * y + u[10] * z) / samp_den;

    MESSAGE_LOG(
        spdlog::level::debug,
        "Projective approximation before bounding ({}, {})",
        ip.line, ip.samp);

    // Since this is valid only over the image,
    // don't let the result go beyond the image border.
    double numRows = m_nLines;
    double numCols = m_nSamples;
    if (ip.line < 0.0) ip.line = 0.0;
    if (ip.line > numRows) ip.line = numRows;

    if (ip.samp < 0.0) ip.samp = 0.0;
    if (ip.samp > numCols) ip.samp = numCols;
  } else {
    ip.line = m_nLines / 2.0;
    ip.samp = m_nSamples / 2.0;
  }
  MESSAGE_LOG(
      spdlog::level::debug,
      "Projective approximation ({}, {})",
      ip.line, ip.samp);
}

/**
 * @brief Creates a projective transformation approximation.
 * 
 * @description Calculates coefficients for a projective transformation based
 * on selected ground to image point mappings, enabling rapid approximate
 * conversions from ground points to image points.
 */
void UsgsAstroLsSensorModel::createProjectiveApproximation() {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Calculating createProjectiveApproximation");

  // Use 9 points (9*4 eventual matrix rows) as we need to fit 14 variables.
  const int numPts = 9;
  double u_factors[numPts] = {0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 1.0, 1.0, 1.0};
  double v_factors[numPts] = {0.0, 0.5, 1.0, 0.0, 0.5, 1.0, 0.0, 0.5, 1.0};

  csm::EcefCoord refPt = getReferencePoint();

  double desired_precision = 0.01;
  double height = computeEllipsoidElevation(
      refPt.x, refPt.y, refPt.z, m_majorAxis, m_minorAxis, desired_precision);
  if (std::isnan(height)) {
    MESSAGE_LOG(
        spdlog::level::warn,
        "createProjectiveApproximation: computeElevation of "
        "reference point {} {} {} with desired precision "
        "{} and major/minor radii {}, {} returned nan height; nonprojective",
        refPt.x, refPt.y, refPt.z, desired_precision, m_majorAxis, m_minorAxis);
    m_useApproxInitTrans = false;
    return;
  }
  MESSAGE_LOG(
      spdlog::level::trace,
      "createProjectiveApproximation: computeElevation of "
      "reference point {} {} {} with desired precision "
      "{} and major/minor radii {}, {} returned {} height",
      refPt.x, refPt.y, refPt.z, desired_precision, m_majorAxis, m_minorAxis, height);

  double numImageRows = m_nLines;
  double numImageCols = m_nSamples;

  std::vector<csm::ImageCoord> ip(2 * numPts);
  std::vector<csm::EcefCoord>  gp(2 * numPts);

  // Sample at two heights above the ellipsoid in order to get a
  // reliable estimate of the relationship between image points and
  // ground points.

  for (int i = 0; i < numPts; i++) {
    ip[i].line = u_factors[i] * numImageRows;
    ip[i].samp = v_factors[i] * numImageCols;
    gp[i]      = imageToGround(ip[i], height);
  }

  double delta_z = 100.0;
  height += delta_z;
  for (int i = 0; i < numPts; i++) {
    ip[i + numPts].line = u_factors[i] * numImageRows;
    ip[i + numPts].samp = v_factors[i] * numImageCols;
    gp[i + numPts]      = imageToGround(ip[i + numPts], height);
  }

  usgscsm::computeBestFitProjectiveTransform(ip, gp, m_projTransCoeffs);
  m_useApproxInitTrans = true;

  MESSAGE_LOG(
      spdlog::level::debug,
      "Completed createProjectiveApproximation");
}

/**
 * @brief Constructs the sensor model state from ISD (Image Support Data).
 *
 * @description Parses the ISD, extracts relevant information, and populates the
 * model state necessary for sensor model operations. This includes sensor,
 * platform, and image identifiers, along with geometric and optical properties.
 * Warnings are collected if any issues arise during parsing.
 *
 * @param imageSupportData The ISD in string format.
 * @param warnings A pointer to a list for recording any warnings.
 * 
 * @return A string representation of the sensor model state.
 */
std::string UsgsAstroLsSensorModel::constructStateFromIsd(
    const std::string imageSupportData, csm::WarningList* warnings) {
  json state = {};
  MESSAGE_LOG(
      spdlog::level::debug,
      "Constructing state from Isd")
  // Instantiate UsgsAstroLineScanner sensor model
  json jsonIsd = json::parse(imageSupportData);
  std::shared_ptr<csm::WarningList> parsingWarnings(new csm::WarningList);

  state["m_modelName"] = ale::getSensorModelName(jsonIsd);
  state["m_imageIdentifier"] = ale::getImageId(jsonIsd);
  state["m_sensorName"] = ale::getSensorName(jsonIsd);
  state["m_platformName"] = ale::getPlatformName(jsonIsd);
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_modelName: {} "
      "m_imageIdentifier: {} "
      "m_sensorName: {} "
      "m_platformName: {} ",
      state["m_modelName"].dump(), state["m_imageIdentifier"].dump(),
      state["m_sensorName"].dump(), state["m_platformName"].dump())

  state["m_focalLength"] = ale::getFocalLength(jsonIsd);
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_focalLength: {} ",
      state["m_focalLength"].dump())

  state["m_nLines"] = ale::getTotalLines(jsonIsd);
  state["m_nSamples"] = ale::getTotalSamples(jsonIsd);
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_nLines: {} "
      "m_nSamples: {} ",
      state["m_nLines"].dump(), state["m_nSamples"].dump())

  state["m_iTransS"] = ale::getFocal2PixelSamples(jsonIsd);
  state["m_iTransL"] = ale::getFocal2PixelLines(jsonIsd);
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_iTransS: {} "
      "m_iTransL: {} ",
      state["m_iTransS"].dump(), state["m_iTransL"].dump())

  state["m_platformFlag"] = 1;
  state["m_ikCode"] = 0;
  state["m_zDirection"] = 1;
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_platformFlag: {} "
      "m_ikCode: {} "
      "m_zDirection: {} ",
      state["m_platformFlag"].dump(), state["m_ikCode"].dump(),
      state["m_zDirection"].dump())

  state["m_distortionType"] =
      getDistortionModel(ale::getDistortionModel(jsonIsd));
  state["m_opticalDistCoeffs"] = ale::getDistortionCoeffs(jsonIsd);
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_distortionType: {} "
      "m_opticalDistCoeffs: {} ",
      state["m_distortionType"].dump(), state["m_opticalDistCoeffs"].dump())

  // Zero computed state values
  state["m_referencePointXyz"] = std::vector<double>(3, 0.0);
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_referencePointXyz: {} ",
      state["m_referencePointXyz"].dump())

  // Sun position and sensor position use the same times so compute that now
  ale::States inst_state = ale::getInstrumentPosition(jsonIsd);
  std::vector<double> ephemTime = inst_state.getTimes();
  double startTime = ephemTime.front();
  double stopTime = ephemTime.back();
  // Force at least 25 nodes to help with lagrange interpolation
  // These also have to be equally spaced for lagrange interpolation
  if (ephemTime.size() < 25) {
    ephemTime.resize(25);
  }
  double timeStep = (stopTime - startTime) / (ephemTime.size() - 1);
  for (size_t i = 0; i < ephemTime.size(); i++) {
    ephemTime[i] = startTime + i * timeStep;
  }

  try {
    state["m_dtEphem"] = timeStep;
    MESSAGE_LOG(
        spdlog::level::trace,
        "m_dtEphem: {} ",
        state["m_dtEphem"].dump())
  } catch (...) {
    parsingWarnings->push_back(csm::Warning(
        csm::Warning::DATA_NOT_AVAILABLE, "dt_ephemeris not in ISD",
        "UsgsAstroFrameSensorModel::constructStateFromIsd()"));
    MESSAGE_LOG(spdlog::level::warn, "m_dtEphem not in ISD")
  }

  try {
    state["m_t0Ephem"] = startTime - ale::getCenterTime(jsonIsd);
    MESSAGE_LOG(
        spdlog::level::trace,
        "t0_ephemeris: {}",
        state["m_t0Ephem"].dump())
  } catch (...) {
    parsingWarnings->push_back(csm::Warning(
        csm::Warning::DATA_NOT_AVAILABLE, "t0_ephemeris not in ISD",
        "UsgsAstroFrameSensorModel::constructStateFromIsd()"));
    MESSAGE_LOG(spdlog::level::warn, "t0_ephemeris not in ISD")
  }

  ale::States sunState = ale::getSunPosition(jsonIsd);
  ale::Orientations j2000_to_target = ale::getBodyRotation(jsonIsd);
  ale::State interpSunState, rotatedSunState;
  std::vector<double> sunPositions = {};
  std::vector<double> sunVelocities = {};

  for (int i = 0; i < ephemTime.size(); i++) {
    interpSunState = sunState.getState(ephemTime[i], ale::SPLINE);
    rotatedSunState = j2000_to_target.rotateStateAt(ephemTime[i], interpSunState);
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

  // leave these be for now.
  state["m_gsd"] = 1.0;
  state["m_flyingHeight"] = 1000.0;
  state["m_halfSwath"] = 1000.0;
  state["m_halfTime"] = 10.0;
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_gsd: {} "
      "m_flyingHeight: {} "
      "m_halfSwath: {} "
      "m_halfTime: {} ",
      state["m_gsd"].dump(), state["m_flyingHeight"].dump(),
      state["m_halfSwath"].dump(), state["m_halfTime"].dump())

  state["m_centerEphemerisTime"] = ale::getCenterTime(jsonIsd);
  state["m_startingEphemerisTime"] = ale::getStartingTime(jsonIsd);
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_centerEphemerisTime: {} "
      "m_startingEphemerisTime: {} ",
      state["m_centerEphemerisTime"].dump(),
      state["m_startingEphemerisTime"].dump())

  std::vector<std::vector<double>> lineScanRate = ale::getLineScanRate(jsonIsd);
  state["m_intTimeLines"] =
    getIntegrationStartLines(lineScanRate, parsingWarnings.get());
  state["m_intTimeStartTimes"] =
    getIntegrationStartTimes(lineScanRate, parsingWarnings.get());
  state["m_intTimes"] = getIntegrationTimes(lineScanRate, parsingWarnings.get());
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_intTimeLines: {} "
      "m_intTimeStartTimes: {} "
      "m_intTimes: {} ",
      state["m_intTimeLines"].dump(), state["m_intTimeStartTimes"].dump(),
      state["m_intTimes"].dump())

  state["m_detectorSampleSumming"] = ale::getSampleSumming(jsonIsd);
  state["m_detectorLineSumming"] = ale::getLineSumming(jsonIsd);
  state["m_startingDetectorSample"] = ale::getDetectorStartingSample(jsonIsd);
  state["m_startingDetectorLine"] = ale::getDetectorStartingLine(jsonIsd);
  state["m_detectorSampleOrigin"] = ale::getDetectorCenterSample(jsonIsd);
  state["m_detectorLineOrigin"] = ale::getDetectorCenterLine(jsonIsd);
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_detectorSampleSumming: {} "
      "m_detectorLineSumming: {}"
      "m_startingDetectorSample: {} "
      "m_startingDetectorLine: {} "
      "m_detectorSampleOrigin: {} "
      "m_detectorLineOrigin: {} ",
      state["m_detectorSampleSumming"].dump(),
      state["m_detectorLineSumming"].dump(),
      state["m_startingDetectorSample"].dump(),
      state["m_startingDetectorLine"].dump(),
      state["m_detectorSampleOrigin"].dump(),
      state["m_detectorLineOrigin"].dump())

  ale::Orientations j2000_to_sensor = ale::getInstrumentPointing(jsonIsd);
  ale::State interpInstState, rotatedInstState;
  std::vector<double> positions = {};
  std::vector<double> velocities = {};

  for (int i = 0; i < ephemTime.size(); i++) {
    interpInstState = inst_state.getState(ephemTime[i], ale::SPLINE);
    rotatedInstState =
        j2000_to_target.rotateStateAt(ephemTime[i], interpInstState, ale::SLERP);
    // ALE operates in km and we want m
    positions.push_back(rotatedInstState.position.x * 1000);
    positions.push_back(rotatedInstState.position.y * 1000);
    positions.push_back(rotatedInstState.position.z * 1000);
    velocities.push_back(rotatedInstState.velocity.x * 1000);
    velocities.push_back(rotatedInstState.velocity.y * 1000);
    velocities.push_back(rotatedInstState.velocity.z * 1000);
  }

  state["m_positions"] = positions;
  state["m_numPositions"] = positions.size();
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_positions: {}"
      "m_numPositions: {}",
      state["m_positions"].dump(), state["m_numPositions"].dump())

  state["m_velocities"] = velocities;
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_velocities: {}",
      state["m_velocities"].dump())

  // Work-around for issues in ALE <=0.8.5 where Orientations without angular
  // velocities seg fault when you multiply them. This will make the angular
  // velocities in the final Orientations dubious but they are not used
  // in any calculations so this is okay.
  if (j2000_to_sensor.getAngularVelocities().empty()) {
    j2000_to_sensor = ale::Orientations(
        j2000_to_sensor.getRotations(),
        j2000_to_sensor.getTimes(),
        std::vector<ale::Vec3d>(j2000_to_sensor.getTimes().size()),
        j2000_to_sensor.getConstantRotation(),
        j2000_to_sensor.getConstantFrames(),
        j2000_to_sensor.getTimeDependentFrames());
  }
  if (j2000_to_target.getAngularVelocities().empty()) {
    j2000_to_target = ale::Orientations(
        j2000_to_target.getRotations(),
        j2000_to_target.getTimes(),
        std::vector<ale::Vec3d>(j2000_to_target.getTimes().size()),
        j2000_to_target.getConstantRotation(),
        j2000_to_target.getConstantFrames(),
        j2000_to_target.getTimeDependentFrames());
  }

  ale::Orientations sensor_to_j2000 = j2000_to_sensor.inverse();
  ale::Orientations sensor_to_target = j2000_to_target * sensor_to_j2000;
  ephemTime = sensor_to_target.getTimes();
  double quatStep =
      (ephemTime.back() - ephemTime.front()) / (ephemTime.size() - 1);
  try {
    state["m_dtQuat"] = quatStep;
    MESSAGE_LOG(
        spdlog::level::trace,
        "dt_quaternion: {}",
        state["m_dtQuat"].dump())
  } catch (...) {
    parsingWarnings->push_back(csm::Warning(
        csm::Warning::DATA_NOT_AVAILABLE, "dt_quaternion not in ISD",
        "UsgsAstroFrameSensorModel::constructStateFromIsd()"));
    MESSAGE_LOG(
        spdlog::level::warn,
        "dt_quaternion not in ISD")
  }

  try {
    state["m_t0Quat"] = ephemTime[0] - ale::getCenterTime(jsonIsd);
    MESSAGE_LOG(
        spdlog::level::trace,
        "m_t0Quat: {}",
        state["m_t0Quat"].dump())
  } catch (...) {
    parsingWarnings->push_back(csm::Warning(
        csm::Warning::DATA_NOT_AVAILABLE, "t0_quaternion not in ISD",
        "UsgsAstroFrameSensorModel::constructStateFromIsd()"));
    MESSAGE_LOG(
        spdlog::level::warn,
        "t0_quaternion not in ISD")
  }
  std::vector<double> quaternion;
  std::vector<double> quaternions;

  for (size_t i = 0; i < ephemTime.size(); i++) {
    ale::Rotation rotation = sensor_to_target.interpolate(
        ephemTime.front() + quatStep * i, ale::SLERP);
    quaternion = rotation.toQuaternion();
    quaternions.push_back(quaternion[1]);
    quaternions.push_back(quaternion[2]);
    quaternions.push_back(quaternion[3]);
    quaternions.push_back(quaternion[0]);
  }

  state["m_quaternions"] = quaternions;
  state["m_numQuaternions"] = quaternions.size();
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_quaternions: {}"
      "m_numQuaternions: {}",
      state["m_quaternions"].dump(), state["m_numQuaternions"].dump())

  state["m_currentParameterValue"] = std::vector<double>(NUM_PARAMETERS, 0.0);
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_currentParameterValue: {}",
      state["m_currentParameterValue"].dump())

  // Get radii
  // ALE operates in km and we want m
  state["m_minorAxis"] = ale::getSemiMinorRadius(jsonIsd) * 1000;
  state["m_majorAxis"] = ale::getSemiMajorRadius(jsonIsd) * 1000;
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_minorAxis: {}"
      "m_majorAxis: {}",
      state["m_minorAxis"].dump(), state["m_majorAxis"].dump())

  // set identifiers
  state["m_platformIdentifier"] = ale::getPlatformName(jsonIsd);
  state["m_sensorIdentifier"] = ale::getSensorName(jsonIsd);
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_platformIdentifier: {}"
      "m_sensorIdentifier: {}",
      state["m_platformIdentifier"].dump(), state["m_sensorIdentifier"].dump())

  // get reference_height
  state["m_minElevation"] = ale::getMinHeight(jsonIsd);
  state["m_maxElevation"] = ale::getMaxHeight(jsonIsd);
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_minElevation: {}"
      "m_maxElevation: {}",
      state["m_minElevation"].dump(), state["m_maxElevation"].dump())

  // Default to identity covariance
  state["m_covariance"] =
      std::vector<double>(NUM_PARAMETERS * NUM_PARAMETERS, 0.0);
  for (int i = 0; i < NUM_PARAMETERS; i++) {
    state["m_covariance"][i * NUM_PARAMETERS + i] = 1.0;
  }

  if (!parsingWarnings->empty()) {
    if (warnings) {
      warnings->insert(warnings->end(), parsingWarnings->begin(),
                       parsingWarnings->end());
    }
    throw csm::Error(csm::Error::SENSOR_MODEL_NOT_CONSTRUCTIBLE,
                     "ISD is invalid for creating the sensor model.",
                     "UsgsAstroLsSensorModel::constructStateFromIsd");
  }

  // The state data will still be updated when a sensor model is created since
  // some state data is not in the ISD and requires a SM to compute them.
  return state.dump();
}

/**
 * @brief Retrieves the logger associated with the sensor model.
 *
 * @description Accessor for the sensor model's logging mechanism, allowing for
 * logging of operations and errors.
 *
 * @return Shared pointer to the logger.
 */
std::shared_ptr<spdlog::logger> UsgsAstroLsSensorModel::getLogger() {
  return m_logger;
}

/**
 * @brief Sets the logger for the sensor model.
 *
 * @description Associates a named logger with the sensor model, enabling
 * logging of messages under a specific log name.
 *
 * @param logName The name of the logger.
 */
void UsgsAstroLsSensorModel::setLogger(std::string logName) {
  m_logger = spdlog::get(logName);
}

/**
 * @brief Computes the position of the Sun at a given image time.
 *
 * @description Determines the Sun's position in ECEF coordinates at a specified
 * time during the image capture.
 *
 * @param imageTime The time of interest within the image capture timeline.
 * 
 * @return The ECEF vector representing the Sun's position.
 */
csm::EcefVector UsgsAstroLsSensorModel::getSunPosition(
    const double imageTime) const {
  int numSunPositions = m_sunPosition.size();
  int numSunVelocities = m_sunVelocity.size();
  csm::EcefVector sunPosition = csm::EcefVector();

  // If there are multiple positions, use Lagrange interpolation
  if ((numSunPositions / 3) > 1) {
    double sunPos[3];
    lagrangeInterp(numSunPositions / 3, &m_sunPosition[0], m_t0Ephem,
                   m_dtEphem, imageTime, 3, 8, sunPos);
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
 * @brief Calculates the line error for detector positioning.
 *
 * @description A helper function for ground to image transformations, which
 * computes the error in detector line positioning relative to an approximated
 * image point and a given ground point.
 *
 * @param t The time step for adjustment.
 * @param approxPt The approximated image point.
 * @param groundPt The target ground point.
 * @param adj Adjustments to the sensor model parameters.
 * 
 * @return The calculated line error.
 */
double UsgsAstroLsSensorModel::calcDetectorLineErr(double t, csm::ImageCoord const& approxPt,
                                                   const csm::EcefCoord& groundPt,
                                                   const std::vector<double>& adj) const {

  csm::ImageCoord currPt = approxPt;
  currPt.line += t;

  double timei = getImageTime(currPt);
  std::vector<double> detectorView = computeDetectorView(timei, groundPt, adj);

  // Invert distortion
  double distortedFocalX, distortedFocalY;
  applyDistortion(detectorView[0], detectorView[1], distortedFocalX,
                  distortedFocalY, m_opticalDistCoeffs, m_focalLength, m_distortionType);

  // Convert to detector line
  double detectorLine = m_iTransL[0] + m_iTransL[1] * distortedFocalX +
                        m_iTransL[2] * distortedFocalY + m_detectorLineOrigin -
                        m_startingDetectorLine;
  detectorLine /= m_detectorLineSumming;

  return detectorLine;
}

