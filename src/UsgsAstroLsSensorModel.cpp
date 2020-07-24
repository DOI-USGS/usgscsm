//----------------------------------------------------------------------------
//
//                                UNCLASSIFIED
//
// Copyright © 1989-2017 BAE Systems Information and Electronic Systems
// Integration Inc.
//                            ALL RIGHTS RESERVED
// Use of this software product is governed by the terms of a license
// agreement. The license agreement is found in the installation directory.
//
//             For support, please visit http://www.baesystems.com/gxp
//
//  Revision History:
//  Date        Name         Description
//  ----------- ------------ -----------------------------------------------
//  13-Nov-2015 BAE Systems  Initial Implementation
//  24-Apr-2017 BAE Systems  Update for CSM 3.0.2
//  24-OCT-2017 BAE Systems  Update for CSM 3.0.3
//-----------------------------------------------------------------------------
#include "UsgsAstroLsSensorModel.h"
#include "Distortion.h"
#include "Utilities.h"

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
    m_logger->info(__VA_ARGS__); \
  }

using json = nlohmann::json;

const std::string UsgsAstroLsSensorModel::_SENSOR_MODEL_NAME =
    "USGS_ASTRO_LINE_SCANNER_SENSOR_MODEL";
const int UsgsAstroLsSensorModel::NUM_PARAMETERS = 16;
const std::string UsgsAstroLsSensorModel::PARAMETER_NAME[] = {
    "IT Pos. Bias   ",  // 0
    "CT Pos. Bias   ",  // 1
    "Rad Pos. Bias  ",  // 2
    "IT Vel. Bias   ",  // 3
    "CT Vel. Bias   ",  // 4
    "Rad Vel. Bias  ",  // 5
    "Omega Bias     ",  // 6
    "Phi Bias       ",  // 7
    "Kappa Bias     ",  // 8
    "Omega Rate     ",  // 9
    "Phi Rate       ",  // 10
    "Kappa Rate     ",  // 11
    "Omega Accl     ",  // 12
    "Phi Accl       ",  // 13
    "Kappa Accl     ",  // 14
    "Focal Bias     "   // 15
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

//***************************************************************************
// UsgsAstroLineScannerSensorModel::replaceModelState
//***************************************************************************
void UsgsAstroLsSensorModel::replaceModelState(const std::string& stateString) {
  MESSAGE_LOG("Replacing model state")

  reset();
  auto j = json::parse(stateString);
  int num_params = NUM_PARAMETERS;

  m_imageIdentifier = j["m_imageIdentifier"].get<std::string>();
  m_sensorName = j["m_sensorName"];
  m_nLines = j["m_nLines"];
  m_nSamples = j["m_nSamples"];
  m_platformFlag = j["m_platformFlag"];
  MESSAGE_LOG(
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
      "m_detectorSampleOrigin: {} "
      "m_detectorLineOrigin: {} "
      "m_majorAxis: {} "
      "m_minorAxis: {} ",
      j["m_detectorSampleOrigin"].dump(), j["m_detectorLineOrigin"].dump(),
      j["m_majorAxis"].dump(), j["m_minorAxis"].dump())

  m_platformIdentifier = j["m_platformIdentifier"];
  m_sensorIdentifier = j["m_sensorIdentifier"];
  MESSAGE_LOG(
      "m_platformIdentifier: {} "
      "m_sensorIdentifier: {} ",
      j["m_platformIdentifier"].dump(), j["m_sensorIdentifier"].dump())

  m_minElevation = j["m_minElevation"];
  m_maxElevation = j["m_maxElevation"];
  MESSAGE_LOG(
      "m_minElevation: {} "
      "m_maxElevation: {} ",
      j["m_minElevation"].dump(), j["m_maxElevation"].dump())

  m_dtEphem = j["m_dtEphem"];
  m_t0Ephem = j["m_t0Ephem"];
  m_dtQuat = j["m_dtQuat"];
  m_t0Quat = j["m_t0Quat"];
  m_numPositions = j["m_numPositions"];
  MESSAGE_LOG(
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
    setLinearApproximation();
  } catch (...) {
    _linear = false;
  }
}

//***************************************************************************
// UsgsAstroLineScannerSensorModel::getModelNameFromModelState
//***************************************************************************
std::string UsgsAstroLsSensorModel::getModelNameFromModelState(
    const std::string& model_state) {
  // Parse the string to JSON
  auto j = json::parse(model_state);
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

//***************************************************************************
// UsgsAstroLineScannerSensorModel::getModelState
//***************************************************************************
std::string UsgsAstroLsSensorModel::getModelState() const {
  MESSAGE_LOG("Running getModelState")

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
      "m_startingEphemerisTime: {} "
      "m_centerEphemerisTime: {} ",
      m_startingEphemerisTime, m_centerEphemerisTime)

  state["m_detectorSampleSumming"] = m_detectorSampleSumming;
  state["m_detectorLineSumming"] = m_detectorLineSumming;
  state["m_startingDetectorSample"] = m_startingDetectorSample;
  state["m_ikCode"] = m_ikCode;
  MESSAGE_LOG(
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
      "m_referencePointXyz: {} "
      "m_referencePointXyz: {} "
      "m_referencePointXyz: {} ",
      m_referencePointXyz.x, m_referencePointXyz.y, m_referencePointXyz.z)

  state["m_sunPosition"] = m_sunPosition;
  MESSAGE_LOG("num sun positions: {} ", m_sunPosition.size())

  state["m_sunVelocity"] = m_sunVelocity;
  MESSAGE_LOG("num sun velocities: {} ", m_sunVelocity.size())

  return state.dump();
}

//***************************************************************************
// UsgsAstroLineScannerSensorModel::reset
//***************************************************************************
void UsgsAstroLsSensorModel::reset() {
  MESSAGE_LOG("Running reset()")
  _linear = false;  // default until a linear model is made
  _u0 = 0.0;
  _du_dx = 0.0;
  _du_dy = 0.0;
  _du_dz = 0.0;
  _v0 = 0.0;
  _dv_dx = 0.0;
  _dv_dy = 0.0;
  _dv_dz = 0.0;

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

//*****************************************************************************
// UsgsAstroLsSensorModel Constructor
//*****************************************************************************
UsgsAstroLsSensorModel::UsgsAstroLsSensorModel() {
  _no_adjustment.assign(UsgsAstroLsSensorModel::NUM_PARAMETERS, 0.0);
}

//*****************************************************************************
// UsgsAstroLsSensorModel Destructor
//*****************************************************************************
UsgsAstroLsSensorModel::~UsgsAstroLsSensorModel() {}

//*****************************************************************************
// UsgsAstroLsSensorModel updateState
//*****************************************************************************
void UsgsAstroLsSensorModel::updateState() {
  // If sensor model is being created for the first time
  // This routine will set some parameters not found in the ISD.
  MESSAGE_LOG("Updating State")
  // Reference point (image center)
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

  // Compute ground sample distance
  ip.line += 1;
  ip.samp += 1;
  csm::EcefCoord delta = imageToGround(ip, refHeight);
  double dx = delta.x - m_referencePointXyz.x;
  double dy = delta.y - m_referencePointXyz.y;
  double dz = delta.z - m_referencePointXyz.z;
  m_gsd = sqrt((dx * dx + dy * dy + dz * dz) / 2.0);
  MESSAGE_LOG(
      "updateState: ground sample distance set to {} "
      "based on dx {} dy {} dz {}",
      m_gsd, dx, dy, dz)

  // Compute flying height
  csm::EcefCoord sensorPos = getSensorPosition(0.0);
  dx = sensorPos.x - m_referencePointXyz.x;
  dy = sensorPos.y - m_referencePointXyz.y;
  dz = sensorPos.z - m_referencePointXyz.z;
  m_flyingHeight = sqrt(dx * dx + dy * dy + dz * dz);
  MESSAGE_LOG(
      "updateState: flight height set to {}"
      "based on dx {} dy {} dz {}",
      m_flyingHeight, dx, dy, dz)

  // Compute half swath
  m_halfSwath = m_gsd * m_nSamples / 2.0;
  MESSAGE_LOG("updateState: half swath set to {}", m_halfSwath)

  // Compute half time duration
  double fullImageTime = m_intTimeStartTimes.back() -
                         m_intTimeStartTimes.front() +
                         m_intTimes.back() * (m_nLines - m_intTimeLines.back());
  m_halfTime = fullImageTime / 2.0;
  MESSAGE_LOG("updateState: half time duration set to {}", m_halfTime)

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

//---------------------------------------------------------------------------
// Core Photogrammetry
//---------------------------------------------------------------------------

//***************************************************************************
// UsgsAstroLsSensorModel::groundToImage
//***************************************************************************
csm::ImageCoord UsgsAstroLsSensorModel::groundToImage(
    const csm::EcefCoord& ground_pt, double desired_precision,
    double* achieved_precision, csm::WarningList* warnings) const {
  MESSAGE_LOG(
      "Computing groundToImage(No adjustments) for {}, {}, {}, with desired "
      "precision {}",
      ground_pt.x, ground_pt.y, ground_pt.z, desired_precision);

  // The public interface invokes the private interface with no adjustments.
  return groundToImage(ground_pt, _no_adjustment, desired_precision,
                       achieved_precision, warnings);
}

//***************************************************************************
// UsgsAstroLsSensorModel::groundToImage (internal version)
//***************************************************************************
csm::ImageCoord UsgsAstroLsSensorModel::groundToImage(
    const csm::EcefCoord& groundPt, const std::vector<double>& adj,
    double desiredPrecision, double* achievedPrecision,
    csm::WarningList* warnings) const {
  // Search for the line, sample coordinate that viewed a given ground point.
  // This method first uses a linear approximation to get an initial point.
  // Then the detector offset for the line is continuously computed and
  // applied to the line until we achieve the desired precision.

  csm::ImageCoord approxPt;
  computeLinearApproximation(groundPt, approxPt);

  std::vector<double> detectorView;
  double detectorLine = m_nLines;
  double detectorSample = 0;
  double count = 0;
  double timei;

  while (abs(detectorLine) > desiredPrecision && ++count < 15) {
    timei = getImageTime(approxPt);
    detectorView = computeDetectorView(timei, groundPt, adj);

    // Convert to detector line
    detectorLine = m_iTransL[0] + m_iTransL[1] * detectorView[0] +
                   m_iTransL[2] * detectorView[1] + m_detectorLineOrigin -
                   m_startingDetectorLine;
    detectorLine /= m_detectorLineSumming;

    // Convert to image line
    approxPt.line += detectorLine;
  }

  timei = getImageTime(approxPt);
  detectorView = computeDetectorView(timei, groundPt, adj);

  // Invert distortion
  double distortedFocalX, distortedFocalY;
  applyDistortion(detectorView[0], detectorView[1], distortedFocalX,
                  distortedFocalY, m_opticalDistCoeffs, m_distortionType,
                  desiredPrecision);

  // Convert to detector line and sample
  detectorLine = m_iTransL[0] + m_iTransL[1] * distortedFocalX +
                 m_iTransL[2] * distortedFocalY;
  detectorSample = m_iTransS[0] + m_iTransS[1] * distortedFocalX +
                   m_iTransS[2] * distortedFocalY;
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

  MESSAGE_LOG("groundToImage: image line sample {} {}", approxPt.line,
              approxPt.samp)

  if (warnings && (desiredPrecision > 0.0) &&
      (abs(finalUpdate) > desiredPrecision)) {
    warnings->push_back(csm::Warning(
        csm::Warning::PRECISION_NOT_MET, "Desired precision not achieved.",
        "UsgsAstroLsSensorModel::groundToImage()"));
  }

  return approxPt;
}

//***************************************************************************
// UsgsAstroLsSensorModel::groundToImage
//***************************************************************************
csm::ImageCoordCovar UsgsAstroLsSensorModel::groundToImage(
    const csm::EcefCoordCovar& groundPt, double desired_precision,
    double* achieved_precision, csm::WarningList* warnings) const {
  MESSAGE_LOG(
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
      groundToImage(gp, desired_precision, achieved_precision, warnings);
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

//***************************************************************************
// UsgsAstroLsSensorModel::imageToGround
//***************************************************************************
csm::EcefCoord UsgsAstroLsSensorModel::imageToGround(
    const csm::ImageCoord& image_pt, double height, double desired_precision,
    double* achieved_precision, csm::WarningList* warnings) const {
  MESSAGE_LOG(
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
  MESSAGE_LOG("imageToGround for {} {} {} achieved precision",// {}",
              image_pt.line, image_pt.samp, height)//, achieved_precision)
  
  return csm::EcefCoord(x, y, z);
}

//***************************************************************************
// UsgsAstroLineScannerSensorModel::determineSensorCovarianceInImageSpace
//***************************************************************************
void UsgsAstroLsSensorModel::determineSensorCovarianceInImageSpace(
    csm::EcefCoord& gp, double sensor_cov[4]) const {
  MESSAGE_LOG("Calculating determineSensorCovarianceInImageSpace for {} {} {}",
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

//***************************************************************************
// UsgsAstroLsSensorModel::imageToGround
//***************************************************************************
csm::EcefCoordCovar UsgsAstroLsSensorModel::imageToGround(
    const csm::ImageCoordCovar& image_pt, double height, double heightVariance,
    double desired_precision, double* achieved_precision,
    csm::WarningList* warnings) const {
  MESSAGE_LOG(
      "Calculating imageToGround (with error propagation) for {}, {}, {} with "
      "height varinace {} and desired precision {}",
      image_pt.line, image_pt.samp, height, heightVariance, desired_precision)
  // Image to ground with error propagation
  // Use numerical partials

  const double DELTA_IMAGE = 1.0;
  const double DELTA_GROUND = m_gsd;
  csm::ImageCoord ip(image_pt.line, image_pt.samp);

  csm::EcefCoord gp = imageToGround(ip, height, desired_precision,
                                    achieved_precision, warnings);

  // Compute numerical partials xyz wrt to lsh
  ip.line = image_pt.line + DELTA_IMAGE;
  ip.samp = image_pt.samp;
  csm::EcefCoord gpl = imageToGround(ip, height, desired_precision);
  double xpl = (gpl.x - gp.x) / DELTA_IMAGE;
  double ypl = (gpl.y - gp.y) / DELTA_IMAGE;
  double zpl = (gpl.z - gp.z) / DELTA_IMAGE;

  ip.line = image_pt.line;
  ip.samp = image_pt.samp + DELTA_IMAGE;
  csm::EcefCoord gps = imageToGround(ip, height, desired_precision);
  double xps = (gps.x - gp.x) / DELTA_IMAGE;
  double yps = (gps.y - gp.y) / DELTA_IMAGE;
  double zps = (gps.z - gp.z) / DELTA_IMAGE;

  ip.line = image_pt.line;
  ip.samp = image_pt.samp;  // +DELTA_IMAGE;
  csm::EcefCoord gph =
      imageToGround(ip, height + DELTA_GROUND, desired_precision);
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

//***************************************************************************
// UsgsAstroLsSensorModel::imageToProximateImagingLocus
//***************************************************************************
csm::EcefLocus UsgsAstroLsSensorModel::imageToProximateImagingLocus(
    const csm::ImageCoord& image_pt, const csm::EcefCoord& ground_pt,
    double desired_precision, double* achieved_precision,
    csm::WarningList* warnings) const {
  MESSAGE_LOG(
      "Computing imageToProximateImagingLocus (ground {}, {}, {}) for image "
      "point {}, {} with desired precision {}",
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
      imageToGround(image_pt, height, desired_precision, achieved_precision);

  // Vector between 2 ground points above
  double dx1 = x - gp1.x;
  double dy1 = y - gp1.y;
  double dz1 = z - gp1.z;

  // Unit vector along object ray
  csm::EcefCoord gp2 = imageToGround(image_pt, height - DELTA_GROUND,
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
  locus.point = imageToGround(image_pt, hLocus, desired_precision,
                              achieved_precision, warnings);

  locus.direction.x = dx2;
  locus.direction.y = dy2;
  locus.direction.z = dz2;

  return locus;
}

//***************************************************************************
// UsgsAstroLsSensorModel::imageToRemoteImagingLocus
//***************************************************************************
csm::EcefLocus UsgsAstroLsSensorModel::imageToRemoteImagingLocus(
    const csm::ImageCoord& image_pt, double desired_precision,
    double* achieved_precision, csm::WarningList* warnings) const {
  MESSAGE_LOG(
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
  return locus;
}

//---------------------------------------------------------------------------
// Uncertainty Propagation
//---------------------------------------------------------------------------

//***************************************************************************
// UsgsAstroLsSensorModel::computeGroundPartials
//***************************************************************************
std::vector<double> UsgsAstroLsSensorModel::computeGroundPartials(
    const csm::EcefCoord& ground_pt) const {
  MESSAGE_LOG("Computing computeGroundPartials for point {}, {}, {}",
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

//***************************************************************************
// UsgsAstroLsSensorModel::computeSensorPartials
//***************************************************************************
csm::RasterGM::SensorPartials UsgsAstroLsSensorModel::computeSensorPartials(
    int index, const csm::EcefCoord& ground_pt, double desired_precision,
    double* achieved_precision, csm::WarningList* warnings) const {
  MESSAGE_LOG(
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

//***************************************************************************
// UsgsAstroLsSensorModel::computeSensorPartials
//***************************************************************************
csm::RasterGM::SensorPartials UsgsAstroLsSensorModel::computeSensorPartials(
    int index, const csm::ImageCoord& image_pt, const csm::EcefCoord& ground_pt,
    double desired_precision, double* achieved_precision,
    csm::WarningList* warnings) const {
  MESSAGE_LOG(
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

//***************************************************************************
// UsgsAstroLsSensorModel::computeAllSensorPartials
//***************************************************************************
std::vector<csm::RasterGM::SensorPartials>
UsgsAstroLsSensorModel::computeAllSensorPartials(
    const csm::EcefCoord& ground_pt, csm::param::Set pSet,
    double desired_precision, double* achieved_precision,
    csm::WarningList* warnings) const {
  MESSAGE_LOG(
      "Computing computeAllSensorPartials for ground point {}, {}, {} with "
      "desired precision {}",
      ground_pt.x, ground_pt.y, ground_pt.z, desired_precision)
  csm::ImageCoord image_pt =
      groundToImage(ground_pt, desired_precision, achieved_precision, warnings);

  return computeAllSensorPartials(image_pt, ground_pt, pSet, desired_precision,
                                  achieved_precision, warnings);
}

//***************************************************************************
// UsgsAstroLsSensorModel::computeAllSensorPartials
//***************************************************************************
std::vector<csm::RasterGM::SensorPartials>
UsgsAstroLsSensorModel::computeAllSensorPartials(
    const csm::ImageCoord& image_pt, const csm::EcefCoord& ground_pt,
    csm::param::Set pSet, double desired_precision, double* achieved_precision,
    csm::WarningList* warnings) const {
  MESSAGE_LOG(
      "Computing computeAllSensorPartials for image {} {} and ground {}, {}, "
      "{} with desired precision {}",
      image_pt.line, image_pt.samp, ground_pt.x, ground_pt.y, ground_pt.z,
      desired_precision)

  return RasterGM::computeAllSensorPartials(image_pt, ground_pt, pSet, desired_precision,
                                            achieved_precision, warnings);
}

//***************************************************************************
// UsgsAstroLsSensorModel::getParameterCovariance
//***************************************************************************
double UsgsAstroLsSensorModel::getParameterCovariance(int index1,
                                                      int index2) const {
  int index = UsgsAstroLsSensorModel::NUM_PARAMETERS * index1 + index2;

  MESSAGE_LOG("getParameterCovariance for {} {} is {}", index1, index2,
              m_covariance[index])

  return m_covariance[index];
}

//***************************************************************************
// UsgsAstroLsSensorModel::setParameterCovariance
//***************************************************************************
void UsgsAstroLsSensorModel::setParameterCovariance(int index1, int index2,
                                                    double covariance) {
  int index = UsgsAstroLsSensorModel::NUM_PARAMETERS * index1 + index2;

  MESSAGE_LOG("setParameterCovariance for {} {} is {}", index1, index2,
              m_covariance[index])

  m_covariance[index] = covariance;
}

//---------------------------------------------------------------------------
// Time and Trajectory
//---------------------------------------------------------------------------

//***************************************************************************
// UsgsAstroLsSensorModel::getTrajectoryIdentifier
//***************************************************************************
std::string UsgsAstroLsSensorModel::getTrajectoryIdentifier() const {
  return "UNKNOWN";
}

//***************************************************************************
// UsgsAstroLsSensorModel::getReferenceDateAndTime
//***************************************************************************
std::string UsgsAstroLsSensorModel::getReferenceDateAndTime() const {
  csm::EcefCoord referencePointGround =
      UsgsAstroLsSensorModel::getReferencePoint();
  csm::ImageCoord referencePointImage =
      UsgsAstroLsSensorModel::groundToImage(referencePointGround);
  double relativeTime =
      UsgsAstroLsSensorModel::getImageTime(referencePointImage);
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

//***************************************************************************
// UsgsAstroLsSensorModel::getImageTime
//***************************************************************************
double UsgsAstroLsSensorModel::getImageTime(
    const csm::ImageCoord& image_pt) const {
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

  MESSAGE_LOG("getImageTime for image line {} is {}", image_pt.line, time)

  return time;
}

//***************************************************************************
// UsgsAstroLsSensorModel::getSensorPosition
//***************************************************************************
csm::EcefCoord UsgsAstroLsSensorModel::getSensorPosition(
    const csm::ImageCoord& imagePt) const {
  MESSAGE_LOG("getSensorPosition at line {}", imagePt.line)

  return getSensorPosition(getImageTime(imagePt));
}

//***************************************************************************
// UsgsAstroLsSensorModel::getSensorPosition
//***************************************************************************
csm::EcefCoord UsgsAstroLsSensorModel::getSensorPosition(double time) const {
  double x, y, z, vx, vy, vz;
  getAdjSensorPosVel(time, _no_adjustment, x, y, z, vx, vy, vz);

  MESSAGE_LOG("getSensorPosition at {}", time)

  return csm::EcefCoord(x, y, z);
}

//***************************************************************************
// UsgsAstroLsSensorModel::getSensorVelocity
//***************************************************************************
csm::EcefVector UsgsAstroLsSensorModel::getSensorVelocity(
    const csm::ImageCoord& imagePt) const {
  MESSAGE_LOG("getSensorVelocity at {}", imagePt.line)
  return getSensorVelocity(getImageTime(imagePt));
}

//***************************************************************************
// UsgsAstroLsSensorModel::getSensorVelocity
//***************************************************************************
csm::EcefVector UsgsAstroLsSensorModel::getSensorVelocity(double time) const {
  double x, y, z, vx, vy, vz;
  getAdjSensorPosVel(time, _no_adjustment, x, y, z, vx, vy, vz);

  MESSAGE_LOG("getSensorVelocity at {}", time)

  return csm::EcefVector(vx, vy, vz);
}

//---------------------------------------------------------------------------
// Sensor Model Parameters
//---------------------------------------------------------------------------

//***************************************************************************
// UsgsAstroLsSensorModel::setParameterValue
//***************************************************************************
void UsgsAstroLsSensorModel::setParameterValue(int index, double value) {
  m_currentParameterValue[index] = value;
}

//***************************************************************************
// UsgsAstroLsSensorModel::getParameterValue
//***************************************************************************
double UsgsAstroLsSensorModel::getParameterValue(int index) const {
  return m_currentParameterValue[index];
}

//***************************************************************************
// UsgsAstroLsSensorModel::getParameterName
//***************************************************************************
std::string UsgsAstroLsSensorModel::getParameterName(int index) const {
  return PARAMETER_NAME[index];
}

std::string UsgsAstroLsSensorModel::getParameterUnits(int index) const {
  // All parameters are meters or scaled to meters
  return "m";
}

//***************************************************************************
// UsgsAstroLsSensorModel::getNumParameters
//***************************************************************************
int UsgsAstroLsSensorModel::getNumParameters() const {
  return UsgsAstroLsSensorModel::NUM_PARAMETERS;
}

//***************************************************************************
// UsgsAstroLsSensorModel::getParameterType
//***************************************************************************
csm::param::Type UsgsAstroLsSensorModel::getParameterType(int index) const {
  return m_parameterType[index];
}

//***************************************************************************
// UsgsAstroLsSensorModel::setParameterType
//***************************************************************************
void UsgsAstroLsSensorModel::setParameterType(int index,
                                              csm::param::Type pType) {
  m_parameterType[index] = pType;
}

//---------------------------------------------------------------------------
// Sensor Model Information
//---------------------------------------------------------------------------

//***************************************************************************
// UsgsAstroLsSensorModel::getPedigree
//***************************************************************************
std::string UsgsAstroLsSensorModel::getPedigree() const {
  return "USGS_LINE_SCANNER";
}

//***************************************************************************
// UsgsAstroLsSensorModel::getImageIdentifier
//***************************************************************************
std::string UsgsAstroLsSensorModel::getImageIdentifier() const {
  return m_imageIdentifier;
}

//***************************************************************************
// UsgsAstroLsSensorModel::setImageIdentifier
//***************************************************************************
void UsgsAstroLsSensorModel::setImageIdentifier(const std::string& imageId,
                                                csm::WarningList* warnings) {
  // Image id should include the suffix without the path name
  m_imageIdentifier = imageId;
}

//***************************************************************************
// UsgsAstroLsSensorModel::getSensorIdentifier
//***************************************************************************
std::string UsgsAstroLsSensorModel::getSensorIdentifier() const {
  return m_sensorIdentifier;
}

//***************************************************************************
// UsgsAstroLsSensorModel::getPlatformIdentifier
//***************************************************************************
std::string UsgsAstroLsSensorModel::getPlatformIdentifier() const {
  return m_platformIdentifier;
}

//***************************************************************************
// UsgsAstroLsSensorModel::setReferencePoint
//***************************************************************************
void UsgsAstroLsSensorModel::setReferencePoint(
    const csm::EcefCoord& ground_pt) {
  m_referencePointXyz = ground_pt;
}

//***************************************************************************
// UsgsAstroLsSensorModel::getReferencePoint
//***************************************************************************
csm::EcefCoord UsgsAstroLsSensorModel::getReferencePoint() const {
  // Return ground point at image center
  return m_referencePointXyz;
}

//***************************************************************************
// UsgsAstroLsSensorModel::getSensorModelName
//***************************************************************************
std::string UsgsAstroLsSensorModel::getModelName() const {
  return UsgsAstroLsSensorModel::_SENSOR_MODEL_NAME;
}

//***************************************************************************
// UsgsAstroLsSensorModel::getImageStart
//***************************************************************************
csm::ImageCoord UsgsAstroLsSensorModel::getImageStart() const {
  return csm::ImageCoord(0.0, 0.0);
}

//***************************************************************************
// UsgsAstroLsSensorModel::getImageSize
//***************************************************************************
csm::ImageVector UsgsAstroLsSensorModel::getImageSize() const {
  return csm::ImageVector(m_nLines, m_nSamples);
}

//---------------------------------------------------------------------------
//  Monoscopic Mensuration
//---------------------------------------------------------------------------

//***************************************************************************
// UsgsAstroLsSensorModel::getValidHeightRange
//***************************************************************************
std::pair<double, double> UsgsAstroLsSensorModel::getValidHeightRange() const {
  return std::pair<double, double>(m_minElevation, m_maxElevation);
}

//***************************************************************************
// UsgsAstroLsSensorModel::getValidImageRange
//***************************************************************************
std::pair<csm::ImageCoord, csm::ImageCoord>
UsgsAstroLsSensorModel::getValidImageRange() const {
  return std::pair<csm::ImageCoord, csm::ImageCoord>(
      csm::ImageCoord(0.0, 0.0),
      csm::ImageCoord(m_nLines,
                      m_nSamples));  // Technically nl and ns are outside the
                                     // image in a zero based system.
}

//***************************************************************************
// UsgsAstroLsSensorModel::getIlluminationDirection
//***************************************************************************
csm::EcefVector UsgsAstroLsSensorModel::getIlluminationDirection(
    const csm::EcefCoord& groundPt) const {
  MESSAGE_LOG(
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

//***************************************************************************
// UsgsAstroLsSensorModel::getNumGeometricCorrectionSwitches
//***************************************************************************
int UsgsAstroLsSensorModel::getNumGeometricCorrectionSwitches() const {
  return 0;
}

//***************************************************************************
// UsgsAstroLsSensorModel::getGeometricCorrectionName
//***************************************************************************
std::string UsgsAstroLsSensorModel::getGeometricCorrectionName(
    int index) const {
  MESSAGE_LOG(
      "Accessing name of geometric correction switch {}. "
      "Geometric correction switches are not supported, throwing exception",
      index);
  // Since there are no geometric corrections, all indices are out of range
  throw csm::Error(csm::Error::INDEX_OUT_OF_RANGE, "Index is out of range.",
                   "UsgsAstroLsSensorModel::getGeometricCorrectionName");
}

//***************************************************************************
// UsgsAstroLsSensorModel::setGeometricCorrectionSwitch
//***************************************************************************
void UsgsAstroLsSensorModel::setGeometricCorrectionSwitch(
    int index, bool value, csm::param::Type pType) {
  MESSAGE_LOG(
      "Setting geometric correction switch {} to {} "
      "with parameter type {}. "
      "Geometric correction switches are not supported, throwing exception",
      index, value, pType);
  // Since there are no geometric corrections, all indices are out of range
  throw csm::Error(csm::Error::INDEX_OUT_OF_RANGE, "Index is out of range.",
                   "UsgsAstroLsSensorModel::setGeometricCorrectionSwitch");
}

//***************************************************************************
// UsgsAstroLsSensorModel::getGeometricCorrectionSwitch
//***************************************************************************
bool UsgsAstroLsSensorModel::getGeometricCorrectionSwitch(int index) const {
  MESSAGE_LOG(
      "Accessing value of geometric correction switch {}. "
      "Geometric correction switches are not supported, throwing exception",
      index);
  // Since there are no geometric corrections, all indices are out of range
  throw csm::Error(csm::Error::INDEX_OUT_OF_RANGE, "Index is out of range.",
                   "UsgsAstroLsSensorModel::getGeometricCorrectionSwitch");
}

//***************************************************************************
// UsgsAstroLsSensorModel::getCrossCovarianceMatrix
//***************************************************************************
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

//***************************************************************************
// UsgsAstroLineScannerSensorModel::getCorrelationModel
//***************************************************************************
const csm::CorrelationModel& UsgsAstroLsSensorModel::getCorrelationModel()
    const {
  // All Line Scanner images are assumed uncorrelated
  return _no_corr_model;
}

//***************************************************************************
// UsgsAstroLsSensorModel::getUnmodeledCrossCovariance
//***************************************************************************
std::vector<double> UsgsAstroLsSensorModel::getUnmodeledCrossCovariance(
    const csm::ImageCoord& pt1, const csm::ImageCoord& pt2) const {
  // No unmodeled error
  return std::vector<double>(4, 0.0);
}

//***************************************************************************
// UsgsAstroLsSensorModel::getCollectionIdentifier
//***************************************************************************
std::string UsgsAstroLsSensorModel::getCollectionIdentifier() const {
  return "UNKNOWN";
}

//***************************************************************************
// UsgsAstroLsSensorModel::hasShareableParameters
//***************************************************************************
bool UsgsAstroLsSensorModel::hasShareableParameters() const {
  // Parameter sharing is not supported for this sensor
  return false;
}

//***************************************************************************
// UsgsAstroLsSensorModel::isParameterShareable
//***************************************************************************
bool UsgsAstroLsSensorModel::isParameterShareable(int index) const {
  // Parameter sharing is not supported for this sensor
  return false;
}

//***************************************************************************
// UsgsAstroLsSensorModel::getParameterSharingCriteria
//***************************************************************************
csm::SharingCriteria UsgsAstroLsSensorModel::getParameterSharingCriteria(
    int index) const {
  MESSAGE_LOG(
      "Checking sharing criteria for parameter {}. "
      "Sharing is not supported, throwing exception",
      index);
  // Parameter sharing is not supported for this sensor,
  // all indices are out of range
  throw csm::Error(csm::Error::INDEX_OUT_OF_RANGE, "Index out of range.",
                   "UsgsAstroLsSensorModel::getParameterSharingCriteria");
}

//***************************************************************************
// UsgsAstroLsSensorModel::getSensorType
//***************************************************************************
std::string UsgsAstroLsSensorModel::getSensorType() const {
  return CSM_SENSOR_TYPE_EO;
}

//***************************************************************************
// UsgsAstroLsSensorModel::getSensorMode
//***************************************************************************
std::string UsgsAstroLsSensorModel::getSensorMode() const {
  return CSM_SENSOR_MODE_PB;
}

//***************************************************************************
// UsgsAstroLsSensorModel::getVersion
//***************************************************************************
csm::Version UsgsAstroLsSensorModel::getVersion() const {
  return csm::Version(1, 0, 0);
}

//***************************************************************************
// UsgsAstroLineScannerSensorModel::getEllipsoid
//***************************************************************************
csm::Ellipsoid UsgsAstroLsSensorModel::getEllipsoid() const {
  return csm::Ellipsoid(m_majorAxis, m_minorAxis);
}

void UsgsAstroLsSensorModel::setEllipsoid(const csm::Ellipsoid& ellipsoid) {
  m_majorAxis = ellipsoid.getSemiMajorRadius();
  m_minorAxis = ellipsoid.getSemiMinorRadius();
}

//***************************************************************************
// UsgsAstroLineScannerSensorModel::getValue
//***************************************************************************
double UsgsAstroLsSensorModel::getValue(
    int index, const std::vector<double>& adjustments) const {
  return m_currentParameterValue[index] + adjustments[index];
}

//***************************************************************************
// Functions pulled out of losToEcf and computeViewingPixel
// **************************************************************************
void UsgsAstroLsSensorModel::getQuaternions(const double& time,
                                            double q[4]) const {
  int nOrder = 8;
  if (m_platformFlag == 0) nOrder = 4;
  int nOrderQuat = nOrder;
  if (m_numQuaternions < 6 && nOrder == 8) nOrderQuat = 4;

  MESSAGE_LOG(
      "Calculating getQuaternions for time {} with {}"
      "order lagrange",
      time, nOrder)
  lagrangeInterp(m_numQuaternions / 4, &m_quaternions[0], m_t0Quat, m_dtQuat,
                 time, 4, nOrderQuat, q);
}

//***************************************************************************
// UsgsAstroLineScannerSensorModel::calculateAttitudeCorrection
//***************************************************************************
void UsgsAstroLsSensorModel::calculateAttitudeCorrection(
    const double& time, const std::vector<double>& adj,
    double attCorr[9]) const {
  MESSAGE_LOG(
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
  MESSAGE_LOG("calculateAttitudeCorrection: euler {} {} {}", euler[0], euler[1],
              euler[2])

  calculateRotationMatrixFromEuler(euler, attCorr);
}

//***************************************************************************
// UsgsAstroLsSensorModel::losToEcf
//***************************************************************************
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
      "Computing losToEcf (with adjustments) for"
      "line {} sample {}",
      line, sample)

  double time = getImageTime(csm::ImageCoord(line, sample));
  getAdjSensorPosVel(time, adj, xc, yc, zc, vx, vy, vz);
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
  MESSAGE_LOG("losToEcf: distorted focal plane coordinate {} {}",
              distortedFocalPlaneX, distortedFocalPlaneY)

  // Remove lens
  double undistortedFocalPlaneX, undistortedFocalPlaneY;
  removeDistortion(distortedFocalPlaneX, distortedFocalPlaneY,
                   undistortedFocalPlaneX, undistortedFocalPlaneY,
                   m_opticalDistCoeffs, m_distortionType);
  MESSAGE_LOG("losToEcf: undistorted focal plane coordinate {} {}",
              undistortedFocalPlaneX, undistortedFocalPlaneY)

  // Define imaging ray (look vector) in camera space
  double cameraLook[3];
  createCameraLookVector(
      undistortedFocalPlaneX, undistortedFocalPlaneY, m_zDirection,
      m_focalLength * (1 - getValue(15, adj) / m_halfSwath), cameraLook);
  MESSAGE_LOG("losToEcf: uncorrected camera look vector {} {} {}",
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
  MESSAGE_LOG("losToEcf: corrected camera look vector {} {} {}",
              correctedCameraLook[0], correctedCameraLook[1],
              correctedCameraLook[2])
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
  MESSAGE_LOG("losToEcf: body look vector {} {} {}", bodyLookX, bodyLookY,
              bodyLookZ)
}

//***************************************************************************
// UsgsAstroLsSensorModel::lightAberrationCorr
//**************************************************************************
void UsgsAstroLsSensorModel::lightAberrationCorr(
    const double& vx, const double& vy, const double& vz, const double& xl,
    const double& yl, const double& zl, double& dxl, double& dyl,
    double& dzl) const {
  MESSAGE_LOG(
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
      "lightAberrationCorr: light of sight correction"
      "{} {} {}",
      dxl, dyl, dzl)
}

//***************************************************************************
// UsgsAstroLsSensorModel::losEllipsoidIntersect
//**************************************************************************
void UsgsAstroLsSensorModel::losEllipsoidIntersect(
    const double& height, const double& xc, const double& yc, const double& zc,
    const double& xl, const double& yl, const double& zl, double& x, double& y,
    double& z, double& achieved_precision,
    const double& desired_precision, csm::WarningList* warnings) const {
  MESSAGE_LOG(
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
    MESSAGE_LOG(message)
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
      "losEllipsoidIntersect: found intersect at {} {} {}"
      "with achieved precision of {}",
      x, y, z, achieved_precision)
}

//***************************************************************************
// UsgsAstroLineScannerSensorModel::getAdjSensorPosVel
//***************************************************************************
void UsgsAstroLsSensorModel::getAdjSensorPosVel(const double& time,
                                                const std::vector<double>& adj,
                                                double& xc, double& yc,
                                                double& zc, double& vx,
                                                double& vy, double& vz) const {
  MESSAGE_LOG("Calculating getAdjSensorPosVel at time {}", time)

  // Sensor position and velocity (4th or 8th order Lagrange).
  int nOrder = 8;
  if (m_platformFlag == 0) nOrder = 4;
  double sensPosNom[3];
  lagrangeInterp(m_numPositions / 3, &m_positions[0], m_t0Ephem, m_dtEphem,
                 time, 3, nOrder, sensPosNom);
  double sensVelNom[3];
  lagrangeInterp(m_numPositions / 3, &m_velocities[0], m_t0Ephem, m_dtEphem,
                 time, 3, nOrder, sensVelNom);

  MESSAGE_LOG("getAdjSensorPosVel: using {} order Lagrange", nOrder)

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
      "getAdjSensorPosVel: postition {} {} {}"
      "and velocity {} {} {}",
      xc, yc, zc, vx, vy, vz)
}

//***************************************************************************
// UsgsAstroLineScannerSensorModel::computeDetectorView
//***************************************************************************
std::vector<double> UsgsAstroLsSensorModel::computeDetectorView(
    const double& time, const csm::EcefCoord& groundPoint,
    const std::vector<double>& adj) const {
  MESSAGE_LOG(
      "Computing computeDetectorView (with adjusments)"
      "for ground point {} {} {} at time {} ",
      groundPoint.x, groundPoint.y, groundPoint.z, time)

  // Helper function to compute the CCD pixel that views a ground point based
  // on the exterior orientation at a given time.

  // Get the exterior orientation
  double xc, yc, zc, vx, vy, vz;
  getAdjSensorPosVel(time, adj, xc, yc, zc, vx, vy, vz);

  // Compute the look vector
  double bodyLookX = groundPoint.x - xc;
  double bodyLookY = groundPoint.y - yc;
  double bodyLookZ = groundPoint.z - zc;
  MESSAGE_LOG("computeDetectorView: look vector {} {} {}", bodyLookX, bodyLookY,
              bodyLookZ)

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
      "computeDetectorView: look vector (camrea ref frame)"
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
      "computeDetectorView: adjusted look vector"
      "{} {} {}",
      adjustedLookX, adjustedLookY, adjustedLookZ)

  // Convert to focal plane coordinate
  double lookScale = (m_focalLength + getValue(15, adj)) / adjustedLookZ;
  double focalX = adjustedLookX * lookScale;
  double focalY = adjustedLookY * lookScale;

  MESSAGE_LOG(
      "computeDetectorView: focal plane coordinates"
      "x:{} y:{} scale:{}",
      focalX, focalY, lookScale)

  return std::vector<double>{focalX, focalY};
}

//***************************************************************************
// UsgsAstroLineScannerSensorModel::computeLinearApproximation
//***************************************************************************
void UsgsAstroLsSensorModel::computeLinearApproximation(
    const csm::EcefCoord& gp, csm::ImageCoord& ip) const {
  if (_linear) {
    ip.line = _u0 + _du_dx * gp.x + _du_dy * gp.y + _du_dz * gp.z;
    ip.samp = _v0 + _dv_dx * gp.x + _dv_dy * gp.y + _dv_dz * gp.z;

    // Since this is valid only over image,
    // don't let result go beyond the image border.
    double numRows = m_nLines;
    double numCols = m_nSamples;
    if (ip.line < 0.0) ip.line = 0.0;
    if (ip.line > numRows) ip.line = numRows;

    if (ip.samp < 0.0) ip.samp = 0.0;
    if (ip.samp > numCols) ip.samp = numCols;
    MESSAGE_LOG(
        "Computing computeLinearApproximation"
        "with linear approximation")
  } else {
    ip.line = m_nLines / 2.0;
    ip.samp = m_nSamples / 2.0;
    MESSAGE_LOG(
        "Computing computeLinearApproximation"
        "nonlinear approx line/2 and sample/2")
  }
}

//***************************************************************************
// UsgsAstroLineScannerSensorModel::setLinearApproximation
//***************************************************************************
void UsgsAstroLsSensorModel::setLinearApproximation() {
  MESSAGE_LOG("Calculating setLinearApproximation")
  double u_factors[4] = {0.0, 0.0, 1.0, 1.0};
  double v_factors[4] = {0.0, 1.0, 0.0, 1.0};

  csm::EcefCoord refPt = getReferencePoint();

  double desired_precision = 0.01;
  double height = computeEllipsoidElevation(
      refPt.x, refPt.y, refPt.z, m_majorAxis, m_minorAxis, desired_precision);
  if (std::isnan(height)) {
    MESSAGE_LOG(
        "setLinearApproximation: computeElevation of"
        "reference point {} {} {} with desired precision"
        "{} returned nan height; nonlinear",
        refPt.x, refPt.y, refPt.z, desired_precision)
    _linear = false;
    return;
  }
  MESSAGE_LOG(
      "setLinearApproximation: computeElevation of"
      "reference point {} {} {} with desired precision"
      "{} returned {} height",
      refPt.x, refPt.y, refPt.z, desired_precision, height)

  double numRows = m_nLines;
  double numCols = m_nSamples;

  csm::ImageCoord imagePt;
  csm::EcefCoord gp[8];

  int i;
  for (i = 0; i < 4; i++) {
    imagePt.line = u_factors[i] * numRows;
    imagePt.samp = v_factors[i] * numCols;
    gp[i] = imageToGround(imagePt, height);
  }

  double delz = 100.0;
  height += delz;
  for (i = 0; i < 4; i++) {
    imagePt.line = u_factors[i] * numRows;
    imagePt.samp = v_factors[i] * numCols;
    gp[i + 4] = imageToGround(imagePt, height);
  }

  csm::EcefCoord d_du;
  d_du.x = ((gp[2].x + gp[3].x + gp[6].x + gp[7].x) -
            (gp[0].x + gp[1].x + gp[4].x + gp[5].x)) /
           numRows / 4.0;
  d_du.y = ((gp[2].y + gp[3].y + gp[6].y + gp[7].y) -
            (gp[0].y + gp[1].y + gp[4].y + gp[5].y)) /
           numRows / 4.0;
  d_du.z = ((gp[2].z + gp[3].z + gp[6].z + gp[7].z) -
            (gp[0].z + gp[1].z + gp[4].z + gp[5].z)) /
           numRows / 4.0;

  csm::EcefCoord d_dv;
  d_dv.x = ((gp[1].x + gp[3].x + gp[5].x + gp[7].x) -
            (gp[0].x + gp[2].x + gp[4].x + gp[6].x)) /
           numCols / 4.0;
  d_dv.y = ((gp[1].y + gp[3].y + gp[5].y + gp[7].y) -
            (gp[0].y + gp[2].y + gp[4].y + gp[6].y)) /
           numCols / 4.0;
  d_dv.z = ((gp[1].z + gp[3].z + gp[5].z + gp[7].z) -
            (gp[0].z + gp[2].z + gp[4].z + gp[6].z)) /
           numCols / 4.0;

  double mat3x3[9];

  mat3x3[0] = d_du.x;
  mat3x3[1] = d_dv.x;
  mat3x3[2] = 1.0;
  mat3x3[3] = d_du.y;
  mat3x3[4] = d_dv.y;
  mat3x3[5] = 1.0;
  mat3x3[6] = d_du.z;
  mat3x3[7] = d_dv.z;
  mat3x3[8] = 1.0;

  double denom = determinant3x3(mat3x3);

  // Can not get derivatives this way
  if (fabs(denom) < 1.0e-8) {
    MESSAGE_LOG(
        "setLinearApproximation: determinant3x3 of"
        "matrix of partials is {}; nonlinear",
        denom)
    _linear = false;
    return;
  }

  mat3x3[0] = 1.0;
  mat3x3[3] = 0.0;
  mat3x3[6] = 0.0;
  _du_dx = determinant3x3(mat3x3) / denom;

  mat3x3[0] = 0.0;
  mat3x3[3] = 1.0;
  mat3x3[6] = 0.0;
  _du_dy = determinant3x3(mat3x3) / denom;

  mat3x3[0] = 0.0;
  mat3x3[3] = 0.0;
  mat3x3[6] = 1.0;
  _du_dz = determinant3x3(mat3x3) / denom;

  mat3x3[0] = d_du.x;
  mat3x3[3] = d_du.y;
  mat3x3[6] = d_du.z;

  mat3x3[1] = 1.0;
  mat3x3[4] = 0.0;
  mat3x3[7] = 0.0;
  _dv_dx = determinant3x3(mat3x3) / denom;

  mat3x3[1] = 0.0;
  mat3x3[4] = 1.0;
  mat3x3[7] = 0.0;
  _dv_dy = determinant3x3(mat3x3) / denom;

  mat3x3[1] = 0.0;
  mat3x3[4] = 0.0;
  mat3x3[7] = 1.0;
  _dv_dz = determinant3x3(mat3x3) / denom;

  _u0 = -gp[0].x * _du_dx - gp[0].y * _du_dy - gp[0].z * _du_dz;
  _v0 = -gp[0].x * _dv_dx - gp[0].y * _dv_dy - gp[0].z * _dv_dz;

  _linear = true;
  MESSAGE_LOG("Completed setLinearApproximation")
}

//***************************************************************************
// UsgsAstroLineScannerSensorModel::determinant3x3
//***************************************************************************
double UsgsAstroLsSensorModel::determinant3x3(double mat[9]) const {
  return mat[0] * (mat[4] * mat[8] - mat[7] * mat[5]) -
         mat[1] * (mat[3] * mat[8] - mat[6] * mat[5]) +
         mat[2] * (mat[3] * mat[7] - mat[6] * mat[4]);
}

//***************************************************************************
// UsgsAstroLineScannerSensorModel::constructStateFromIsd
//***************************************************************************
std::string UsgsAstroLsSensorModel::constructStateFromIsd(
    const std::string imageSupportData, csm::WarningList* warnings) {
  json state = {};
  MESSAGE_LOG("Constructing state from Isd")
  // Instantiate UsgsAstroLineScanner sensor model
  json jsonIsd = json::parse(imageSupportData);
  csm::WarningList* parsingWarnings = new csm::WarningList;

  int num_params = NUM_PARAMETERS;

  state["m_modelName"] = ale::getSensorModelName(jsonIsd);
  state["m_imageIdentifier"] = ale::getImageId(jsonIsd);
  state["m_sensorName"] = ale::getSensorName(jsonIsd);
  state["m_platformName"] = ale::getPlatformName(jsonIsd);
  MESSAGE_LOG(
      "m_modelName: {} "
      "m_imageIdentifier: {} "
      "m_sensorName: {} "
      "m_platformName: {} ",
      state["m_modelName"].dump(), state["m_imageIdentifier"].dump(),
      state["m_sensorName"].dump(), state["m_platformName"].dump())

  state["m_focalLength"] = ale::getFocalLength(jsonIsd);
  MESSAGE_LOG("m_focalLength: {} ", state["m_focalLength"].dump())

  state["m_nLines"] = ale::getTotalLines(jsonIsd);
  state["m_nSamples"] = ale::getTotalSamples(jsonIsd);
  MESSAGE_LOG(
      "m_nLines: {} "
      "m_nSamples: {} ",
      state["m_nLines"].dump(), state["m_nSamples"].dump())

  state["m_iTransS"] = ale::getFocal2PixelSamples(jsonIsd);
  state["m_iTransL"] = ale::getFocal2PixelLines(jsonIsd);
  MESSAGE_LOG(
      "m_iTransS: {} "
      "m_iTransL: {} ",
      state["m_iTransS"].dump(), state["m_iTransL"].dump())

  state["m_platformFlag"] = 1;
  state["m_ikCode"] = 0;
  state["m_zDirection"] = 1;
  MESSAGE_LOG(
      "m_platformFlag: {} "
      "m_ikCode: {} "
      "m_zDirection: {} ",
      state["m_platformFlag"].dump(), state["m_ikCode"].dump(),
      state["m_zDirection"].dump())

  state["m_distortionType"] =
      getDistortionModel(ale::getDistortionModel(jsonIsd));
  state["m_opticalDistCoeffs"] = ale::getDistortionCoeffs(jsonIsd);
  MESSAGE_LOG(
      "m_distortionType: {} "
      "m_opticalDistCoeffs: {} ",
      state["m_distortionType"].dump(), state["m_opticalDistCoeffs"].dump())

  // Zero computed state values
  state["m_referencePointXyz"] = std::vector<double>(3, 0.0);
  MESSAGE_LOG("m_referencePointXyz: {} ", state["m_referencePointXyz"].dump())

  // Sun position and velocity are required for getIlluminationDirection
  ale::States sunState = ale::getSunPosition(jsonIsd);
  std::vector<ale::State> sunStates = sunState.getStates();
  std::vector<double> ephemTime = sunState.getTimes();
  ale::Orientations j2000_to_target = ale::getBodyRotation(jsonIsd);
  ale::State rotatedSunState;
  std::vector<double> sunPositions = {};
  std::vector<double> sunVelocities = {};

  for (int i = 0; i < ephemTime.size(); i++) {
    rotatedSunState = j2000_to_target.rotateStateAt(ephemTime[i], sunStates[i]);
    // ALE operates in Km and we want m
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
      "m_gsd: {} "
      "m_flyingHeight: {} "
      "m_halfSwath: {} "
      "m_halfTime: {} ",
      state["m_gsd"].dump(), state["m_flyingHeight"].dump(),
      state["m_halfSwath"].dump(), state["m_halfTime"].dump())

  state["m_centerEphemerisTime"] = ale::getCenterTime(jsonIsd);
  state["m_startingEphemerisTime"] = ale::getStartingTime(jsonIsd);
  MESSAGE_LOG(
      "m_centerEphemerisTime: {} "
      "m_startingEphemerisTime: {} ",
      state["m_centerEphemerisTime"].dump(),
      state["m_startingEphemerisTime"].dump())

  std::vector<std::vector<double>> lineScanRate = ale::getLineScanRate(jsonIsd);
  state["m_intTimeLines"] =
      getIntegrationStartLines(lineScanRate, parsingWarnings);
  state["m_intTimeStartTimes"] =
      getIntegrationStartTimes(lineScanRate, parsingWarnings);
  state["m_intTimes"] = getIntegrationTimes(lineScanRate, parsingWarnings);
  MESSAGE_LOG(
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

  ale::States inst_state = ale::getInstrumentPosition(jsonIsd);
  // These are exlusive to LineScanners, leave them here for now.
  ephemTime = inst_state.getTimes();
  try {
    state["m_dtEphem"] = (ephemTime[ephemTime.size() - 1] - ephemTime[0]) /
                         (ephemTime.size() - 1);
    MESSAGE_LOG("m_dtEphem: {} ", state["m_dtEphem"].dump())
  } catch (...) {
    parsingWarnings->push_back(csm::Warning(
        csm::Warning::DATA_NOT_AVAILABLE, "dt_ephemeris not in ISD",
        "UsgsAstroFrameSensorModel::constructStateFromIsd()"));
    MESSAGE_LOG("m_dtEphem not in ISD")
  }

  try {
    state["m_t0Ephem"] = ephemTime[0] - ale::getCenterTime(jsonIsd);
    MESSAGE_LOG("t0_ephemeris: {}", state["m_t0Ephem"].dump())
  } catch (...) {
    parsingWarnings->push_back(csm::Warning(
        csm::Warning::DATA_NOT_AVAILABLE, "t0_ephemeris not in ISD",
        "UsgsAstroFrameSensorModel::constructStateFromIsd()"));
    MESSAGE_LOG("t0_ephemeris not in ISD")
  }

  ale::Orientations j2000_to_sensor = ale::getInstrumentPointing(jsonIsd);
  ephemTime = inst_state.getTimes();
  std::vector<ale::State> instStates = inst_state.getStates();
  ale::State rotatedInstState;
  ale::State rotatedInstStateInv;
  std::vector<double> positions = {};
  std::vector<double> velocities = {};

  for (int i = 0; i < ephemTime.size(); i++) {
    rotatedInstState =
        j2000_to_target.rotateStateAt(ephemTime[i], instStates[i], ale::SLERP);
    // ALE operates in Km and we want m
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
      "m_positions: {}"
      "m_numPositions: {}",
      state["m_positions"].dump(), state["m_numPositions"].dump())

  state["m_velocities"] = velocities;
  MESSAGE_LOG("m_velocities: {}", state["m_velocities"].dump())

  ale::Orientations sensor_to_j2000 = j2000_to_sensor.inverse();
  ale::Orientations sensor_to_target = j2000_to_target * sensor_to_j2000;
  ephemTime = sensor_to_target.getTimes();
  double quatStep =
      (ephemTime.back() - ephemTime.front()) / (ephemTime.size() - 1);
  try {
    state["m_dtQuat"] = quatStep;
    MESSAGE_LOG("dt_quaternion: {}", state["m_dtQuat"].dump())
  } catch (...) {
    parsingWarnings->push_back(csm::Warning(
        csm::Warning::DATA_NOT_AVAILABLE, "dt_quaternion not in ISD",
        "UsgsAstroFrameSensorModel::constructStateFromIsd()"));
    MESSAGE_LOG("dt_quaternion not in ISD")
  }

  try {
    state["m_t0Quat"] = ephemTime[0] - ale::getCenterTime(jsonIsd);
    MESSAGE_LOG("m_t0Quat: {}", state["m_t0Quat"].dump())
  } catch (...) {
    parsingWarnings->push_back(csm::Warning(
        csm::Warning::DATA_NOT_AVAILABLE, "t0_quaternion not in ISD",
        "UsgsAstroFrameSensorModel::constructStateFromIsd()"));
    MESSAGE_LOG("t0_quaternion not in ISD")
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
      "m_quaternions: {}"
      "m_numQuaternions: {}",
      state["m_quaternions"].dump(), state["m_numQuaternions"].dump())

  state["m_currentParameterValue"] = std::vector<double>(NUM_PARAMETERS, 0.0);
  MESSAGE_LOG("m_currentParameterValue: {}",
              state["m_currentParameterValue"].dump())

  // get radii
  // ALE operates in Km and we want m
  state["m_minorAxis"] = ale::getSemiMinorRadius(jsonIsd) * 1000;
  state["m_majorAxis"] = ale::getSemiMajorRadius(jsonIsd) * 1000;
  MESSAGE_LOG(
      "m_minorAxis: {}"
      "m_majorAxis: {}",
      state["m_minorAxis"].dump(), state["m_majorAxis"].dump())

  // set identifiers
  state["m_platformIdentifier"] = ale::getPlatformName(jsonIsd);
  state["m_sensorIdentifier"] = ale::getSensorName(jsonIsd);
  MESSAGE_LOG(
      "m_platformIdentifier: {}"
      "m_sensorIdentifier: {}",
      state["m_platformIdentifier"].dump(), state["m_sensorIdentifier"].dump())

  // get reference_height
  state["m_minElevation"] = ale::getMinHeight(jsonIsd);
  state["m_maxElevation"] = ale::getMaxHeight(jsonIsd);
  MESSAGE_LOG(
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
    delete parsingWarnings;
    parsingWarnings = nullptr;
    throw csm::Error(csm::Error::SENSOR_MODEL_NOT_CONSTRUCTIBLE,
                     "ISD is invalid for creating the sensor model.",
                     "UsgsAstroFrameSensorModel::constructStateFromIsd");
  }

  delete parsingWarnings;
  parsingWarnings = nullptr;

  // The state data will still be updated when a sensor model is created since
  // some state data is not in the ISD and requires a SM to compute them.
  return state.dump();
}

//***************************************************************************
// UsgsAstroLineScannerSensorModel::getLogger
//***************************************************************************
std::shared_ptr<spdlog::logger> UsgsAstroLsSensorModel::getLogger() {
  // MESSAGE_LOG("Getting log pointer, logger is {}",
  //                             m_logger)
  return m_logger;
}

void UsgsAstroLsSensorModel::setLogger(std::string logName) {
  m_logger = spdlog::get(logName);
}

csm::EcefVector UsgsAstroLsSensorModel::getSunPosition(
    const double imageTime) const {
  int numSunPositions = m_sunPosition.size();
  int numSunVelocities = m_sunVelocity.size();
  csm::EcefVector sunPosition = csm::EcefVector();

  // If there are multiple positions, use Lagrange interpolation
  if ((numSunPositions / 3) > 1) {
    double sunPos[3];
    double endTime = m_t0Ephem + (m_dtEphem * ((m_numPositions / 3)));
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
