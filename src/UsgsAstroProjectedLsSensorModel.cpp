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

#include "UsgsAstroProjectedLsSensorModel.h"
#include "Distortion.h"
#include "Utilities.h"
#include "EigenUtilities.h"

#include <float.h>
#include <math.h>
#include <algorithm>
#include <iostream>
#include <sstream>

#include <proj.h>

#include <Error.h>
#include <nlohmann/json.hpp>

#include "ale/Util.h"

#define MESSAGE_LOG(...)         \
  if (m_logger) {                \
    m_logger->log(__VA_ARGS__); \
  }

using json = nlohmann::json;

const std::string UsgsAstroProjectedLsSensorModel::_SENSOR_MODEL_NAME =
    "USGS_ASTRO_PROJECTED_LINE_SCANNER_SENSOR_MODEL";
const int UsgsAstroProjectedLsSensorModel::NUM_PARAMETERS = 16;
const std::string UsgsAstroProjectedLsSensorModel::PARAMETER_NAME[] = {
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

const std::string UsgsAstroProjectedLsSensorModel::_STATE_KEYWORD[] = {
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
    "m_geoTransform",
    "m_projString",
};

const int UsgsAstroProjectedLsSensorModel::NUM_PARAM_TYPES = 4;
const std::string UsgsAstroProjectedLsSensorModel::PARAM_STRING_ALL[] = {
    "NONE", "FICTITIOUS", "REAL", "FIXED"};
const csm::param::Type UsgsAstroProjectedLsSensorModel::PARAM_CHAR_ALL[] = {
    csm::param::NONE, csm::param::FICTITIOUS, csm::param::REAL,
    csm::param::FIXED};

//***************************************************************************
// UsgsAstroLineScannerSensorModel::replaceModelState
//***************************************************************************
void UsgsAstroProjectedLsSensorModel::replaceModelState(const std::string& stateString) {
  reset();

  auto j = stateAsJson(stateString);
  m_geoTransform = j["m_geoTransform"].get<std::vector<double>>();
  m_projString = j["m_projString"];
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_geoTransform: {} "
      "m_projString: {} ",
      j["m_geoTransform"].dump(), j["m_projString"].dump());
  UsgsAstroLsSensorModel::replaceModelState(stateString);
}

//***************************************************************************
// UsgsAstroLineScannerSensorModel::getModelNameFromModelState
//***************************************************************************
std::string UsgsAstroProjectedLsSensorModel::getModelNameFromModelState(
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
    std::string aFunction = "UsgsAstroProjectedLsPlugin::getModelNameFromModelState";
    csm::Error csmErr(aErrorType, aMessage, aFunction);
    throw(csmErr);
  }
  if (model_name != _SENSOR_MODEL_NAME) {
    csm::Error::ErrorType aErrorType = csm::Error::SENSOR_MODEL_NOT_SUPPORTED;
    std::string aMessage = "Sensor model not supported.";
    std::string aFunction = "UsgsAstroProjectedLsPlugin::getModelNameFromModelState()";
    csm::Error csmErr(aErrorType, aMessage, aFunction);
    throw(csmErr);
  }
  return model_name;
}

//***************************************************************************
// UsgsAstroLineScannerSensorModel::getModelState
//***************************************************************************
std::string UsgsAstroProjectedLsSensorModel::getModelState() const {
  auto state = stateAsJson(UsgsAstroLsSensorModel::getModelState());
  state["m_geoTransform"] = m_geoTransform;
  state["m_projString"] = m_projString;
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_geoTransform: {}, {}, {}, {}, {}, {} "
      "m_projString: {} ",
      m_geoTransform[0], 
      m_geoTransform[1], 
      m_geoTransform[2], 
      m_geoTransform[3], 
      m_geoTransform[4], 
      m_geoTransform[5], 
      m_projString);
  // Use dump(2) to avoid creating the model string as a single long line
  std::string stateString = getModelName() + "\n" + state.dump(2);
  return stateString;
}

//***************************************************************************
// UsgsAstroLineScannerSensorModel::reset
//***************************************************************************
void UsgsAstroProjectedLsSensorModel::reset() {
  MESSAGE_LOG(spdlog::level::debug, "Running reset()");

  m_geoTransform = std::vector<double>(6, 0.0);
  m_projString = "";
}

//*****************************************************************************
// UsgsAstroProjectedLsSensorModel Constructor
//*****************************************************************************
UsgsAstroProjectedLsSensorModel::UsgsAstroProjectedLsSensorModel() {
  _no_adjustment.assign(UsgsAstroProjectedLsSensorModel::NUM_PARAMETERS, 0.0);
}

//*****************************************************************************
// UsgsAstroProjectedLsSensorModel Destructor
//*****************************************************************************
UsgsAstroProjectedLsSensorModel::~UsgsAstroProjectedLsSensorModel() {
  if (m_logger) {
    m_logger->flush();
  }
}

//---------------------------------------------------------------------------
// Core Photogrammetry
//---------------------------------------------------------------------------

//***************************************************************************
// UsgsAstroLsSensorModel::groundToImage
//***************************************************************************
csm::ImageCoord UsgsAstroProjectedLsSensorModel::groundToImage(
    const csm::EcefCoord &ground_pt, double desired_precision,
    double *achieved_precision, csm::WarningList *warnings) const {
  
  PJ_CONTEXT *C = proj_context_create();

  /* Create a projection. */
  PJ *isdProj = proj_create(C, m_projString.c_str());
  if (0 == isdProj) {
    MESSAGE_LOG(
        spdlog::level::debug,
        "Failed to create isd transformation object");
    return csm::ImageCoord(0, 0);
  }

  /* Create the geocentric projection for our target */
  std::string radius_a = "+a=" + std::to_string(m_majorAxis);
  std::string radius_b = "+b=" + std::to_string(m_minorAxis);
  std::string projString = "+proj=geocent " + radius_a + " " + radius_b + " +type=crs";
  PJ *ecefProj = proj_create(C, projString.c_str());
  if (0 == ecefProj) {
    MESSAGE_LOG(
        spdlog::level::debug,
        "Failed to create geocent transformation object");
    return csm::ImageCoord(0, 0);
  }

  // Compute the transformation from our ISIS projection to ecef
  PJ *isdProj2ecefProj = proj_create_crs_to_crs_from_pj(C, isdProj, ecefProj, 0, 0);
  PJ_COORD c_in;
  c_in.xyz.x = ground_pt.x;
  c_in.xyz.y = ground_pt.y;
  c_in.xyz.z = ground_pt.z;
  MESSAGE_LOG(
      spdlog::level::info,
      "Ground point {}, {}, {}",
      c_in.xyz.x, c_in.xyz.y, c_in.xyz.z);
  PJ_COORD c_out = proj_trans(isdProj2ecefProj, PJ_INV, c_in);
  MESSAGE_LOG(
      spdlog::level::info,
      "Meters {}, {}",
      c_out.xyz.x, c_out.xyz.y);
  std::vector<double> lineSampleCoord = meterToPixel(c_out.xyz.x, c_out.xyz.y, m_geoTransform);
  csm::ImageCoord imagePt(lineSampleCoord[0], lineSampleCoord[1]);
  MESSAGE_LOG(
      spdlog::level::info,
      "groundToImage result of ({}, {})",
      imagePt.line, imagePt.samp);
  return imagePt;
}

//***************************************************************************
// UsgsAstroLsSensorModel::groundToImage
//***************************************************************************
csm::ImageCoordCovar UsgsAstroProjectedLsSensorModel::groundToImage(
    const csm::EcefCoordCovar &groundPt, double desired_precision,
    double *achieved_precision, csm::WarningList *warnings) const {
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
      groundToImage(gp, desired_precision, achieved_precision, warnings);
  csm::ImageCoordCovar result(ip.line, ip.samp);

  // // Compute partials ls wrt XYZ
  // std::vector<double> prt(6, 0.0);
  // prt = UsgsAstroLsSensorModel::computeGroundPartials(groundPt);

  // // Error propagation
  // double ltx, lty, ltz;
  // double stx, sty, stz;
  // ltx = prt[0] * groundPt.covariance[0] + prt[1] * groundPt.covariance[3] +
  //       prt[2] * groundPt.covariance[6];
  // lty = prt[0] * groundPt.covariance[1] + prt[1] * groundPt.covariance[4] +
  //       prt[2] * groundPt.covariance[7];
  // ltz = prt[0] * groundPt.covariance[2] + prt[1] * groundPt.covariance[5] +
  //       prt[2] * groundPt.covariance[8];
  // stx = prt[3] * groundPt.covariance[0] + prt[4] * groundPt.covariance[3] +
  //       prt[5] * groundPt.covariance[6];
  // sty = prt[3] * groundPt.covariance[1] + prt[4] * groundPt.covariance[4] +
  //       prt[5] * groundPt.covariance[7];
  // stz = prt[3] * groundPt.covariance[2] + prt[4] * groundPt.covariance[5] +
  //       prt[5] * groundPt.covariance[8];

  // double gp_cov[4]; // Input gp cov in image space
  // gp_cov[0] = ltx * prt[0] + lty * prt[1] + ltz * prt[2];
  // gp_cov[1] = ltx * prt[3] + lty * prt[4] + ltz * prt[5];
  // gp_cov[2] = stx * prt[0] + sty * prt[1] + stz * prt[2];
  // gp_cov[3] = stx * prt[3] + sty * prt[4] + stz * prt[5];

  // std::vector<double> unmodeled_cov = getUnmodeledError(ip);
  // double sensor_cov[4]; // sensor cov in image space
  // determineSensorCovarianceInImageSpace(gp, sensor_cov);

  // result.covariance[0] = gp_cov[0] + unmodeled_cov[0] + sensor_cov[0];
  // result.covariance[1] = gp_cov[1] + unmodeled_cov[1] + sensor_cov[1];
  // result.covariance[2] = gp_cov[2] + unmodeled_cov[2] + sensor_cov[2];
  // result.covariance[3] = gp_cov[3] + unmodeled_cov[3] + sensor_cov[3];

  return result;
}

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::imageToGround
//***************************************************************************
csm::EcefCoord UsgsAstroProjectedLsSensorModel::imageToGround(
    const csm::ImageCoord& image_pt, double height, double desired_precision,
    double* achieved_precision, csm::WarningList* warnings) const {
  MESSAGE_LOG(
      spdlog::level::info,
      "Computing imageToGround for {}, {}, {}, with desired precision {}",
      image_pt.line, image_pt.samp, height, desired_precision);

  double x = 0, y = 0, z = 0;
  double meterLine, meterSamp;
  std::vector<double> meterCoord = pixelToMeter(image_pt.line, image_pt.samp, m_geoTransform);
  meterLine = meterCoord[0];
  meterSamp = meterCoord[1];
  PJ_CONTEXT *C = proj_context_create();

  /* Create a projection. */
  PJ *isdProj = proj_create(C, m_projString.c_str());
  if (0 == isdProj) {
    MESSAGE_LOG(
        spdlog::level::debug,
        "Failed to create isd transformation object");
    return csm::EcefCoord(x, y, z);
  }

  /* Create the geocentric projection for our target */
  std::string radius_a = "+a=" + std::to_string(m_majorAxis);
  std::string radius_b = "+b=" + std::to_string(m_minorAxis);
  std::string projString = "+proj=geocent " + radius_a + " " + radius_b + " +type=crs";
  PJ *ecefProj = proj_create(C, projString.c_str());
  if (0 == ecefProj) {
    MESSAGE_LOG(
        spdlog::level::debug,
        "Failed to create geocent transformation object");
    return csm::EcefCoord(x, y, z);
  }

  // Compute the transformation from our ISIS projection to ecef
  PJ *isdProj2ecefProj = proj_create_crs_to_crs_from_pj(C, isdProj, ecefProj, 0, 0);
  PJ_COORD c_in;
  c_in.xy.x = meterSamp;
  c_in.xy.y = meterLine;
  PJ_COORD c_out = proj_trans(isdProj2ecefProj, PJ_FWD, c_in);
  x = c_out.xyz.x, y = c_out.xyz.y, z = c_out.xyz.z;
  MESSAGE_LOG(
      spdlog::level::info,
      "imageToGround result {} {} {}",
      x, y, z);
  return csm::EcefCoord(x, y, z);
}

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::imageToGround
//***************************************************************************
csm::EcefCoordCovar UsgsAstroProjectedLsSensorModel::imageToGround(
    const csm::ImageCoordCovar& image_pt, double height, double heightVariance,
    double desired_precision, double* achieved_precision,
    csm::WarningList* warnings) const {
  csm::EcefCoord groundCoord = imageToGround(image_pt, height);
  csm::ImageCoord cameraImagePt = UsgsAstroLsSensorModel::groundToImage(groundCoord);
  csm::ImageCoordCovar cameraImagePtCovar(cameraImagePt.line, cameraImagePt.samp);

  return UsgsAstroLsSensorModel::imageToGround(cameraImagePtCovar, height, heightVariance, desired_precision, achieved_precision, warnings);
}

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::imageToProximateImagingLocus
//***************************************************************************
csm::EcefLocus UsgsAstroProjectedLsSensorModel::imageToProximateImagingLocus(
    const csm::ImageCoord& image_pt, const csm::EcefCoord& ground_pt,
    double desired_precision, double* achieved_precision,
    csm::WarningList* warnings) const {
  csm::ImageCoord cameraImagePt = UsgsAstroLsSensorModel::groundToImage(ground_pt);

  return UsgsAstroLsSensorModel::imageToProximateImagingLocus(cameraImagePt, ground_pt, desired_precision, achieved_precision, warnings);
}

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::imageToRemoteImagingLocus
//***************************************************************************
csm::EcefLocus UsgsAstroProjectedLsSensorModel::imageToRemoteImagingLocus(
    const csm::ImageCoord& image_pt, double desired_precision,
    double* achieved_precision, csm::WarningList* warnings) const {
  // Go from proj x, y to latlon then ground to image
  // Convert imagept to camera imagept
  csm::EcefCoord groundCoord = imageToGround(image_pt, 0);
  csm::ImageCoord cameraImagePt = UsgsAstroLsSensorModel::groundToImage(groundCoord);

  return UsgsAstroLsSensorModel::imageToRemoteImagingLocus(cameraImagePt, desired_precision, achieved_precision, warnings);
}

//---------------------------------------------------------------------------
// Uncertainty Propagation
//---------------------------------------------------------------------------

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::getParameterCovariance
//***************************************************************************
double UsgsAstroProjectedLsSensorModel::getParameterCovariance(int index1,
                                                      int index2) const {
  int index = UsgsAstroProjectedLsSensorModel::NUM_PARAMETERS * index1 + index2;

  MESSAGE_LOG(
      spdlog::level::debug,
      "getParameterCovariance for {} {} is {}",
      index1, index2, m_covariance[index])

  return m_covariance[index];
}

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::setParameterCovariance
//***************************************************************************
void UsgsAstroProjectedLsSensorModel::setParameterCovariance(int index1, int index2,
                                                    double covariance) {
  int index = UsgsAstroProjectedLsSensorModel::NUM_PARAMETERS * index1 + index2;

  MESSAGE_LOG(
      spdlog::level::debug,
      "setParameterCovariance for {} {} is {}",
      index1, index2, m_covariance[index])

  m_covariance[index] = covariance;
}

//---------------------------------------------------------------------------
// Time and Trajectory
//---------------------------------------------------------------------------

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::getTrajectoryIdentifier
//***************************************************************************
std::string UsgsAstroProjectedLsSensorModel::getTrajectoryIdentifier() const {
  return "UNKNOWN";
}

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::getReferenceDateAndTime
//***************************************************************************
std::string UsgsAstroProjectedLsSensorModel::getReferenceDateAndTime() const {
  csm::EcefCoord referencePointGround =
    UsgsAstroProjectedLsSensorModel::getReferencePoint();
  csm::ImageCoord referencePointImage =
    UsgsAstroProjectedLsSensorModel::groundToImage(referencePointGround);
  double relativeTime =
    UsgsAstroProjectedLsSensorModel::getImageTime(referencePointImage);
  time_t ephemTime = m_centerEphemerisTime + relativeTime;

  return ephemTimeToCalendarTime(ephemTime);
}

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::getImageTime
//***************************************************************************
double UsgsAstroProjectedLsSensorModel::getImageTime(
    const csm::ImageCoord& image_pt) const {
  // Convert imagept to camera imagept
  // proj -> ecef -> groundToImage
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

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::getSensorPosition
//***************************************************************************
csm::EcefCoord UsgsAstroProjectedLsSensorModel::getSensorPosition(
    const csm::ImageCoord& imagePt) const {
  // Convert imagept to camera imagept
  // proj -> ecef -> groundToImage
  MESSAGE_LOG(
      spdlog::level::debug,
      "getSensorPosition at image coord ({}, {})",
      imagePt.line, imagePt.samp)

  return UsgsAstroLsSensorModel::getSensorPosition(getImageTime(imagePt));
}

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::getSensorVelocity
//***************************************************************************
csm::EcefVector UsgsAstroProjectedLsSensorModel::getSensorVelocity(
    const csm::ImageCoord& imagePt) const {
  // Convert imagept to camera imagept
  // proj -> ecef -> groundToImage
  MESSAGE_LOG(
      spdlog::level::debug,
      "getSensorVelocity at image coord ({}, {})",
      imagePt.line, imagePt.samp);
  return UsgsAstroLsSensorModel::getSensorVelocity(getImageTime(imagePt));
}

//---------------------------------------------------------------------------
// Sensor Model Parameters
//---------------------------------------------------------------------------

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::setParameterValue
//***************************************************************************
void UsgsAstroProjectedLsSensorModel::setParameterValue(int index, double value) {
  m_currentParameterValue[index] = value;
}

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::getParameterValue
//***************************************************************************
double UsgsAstroProjectedLsSensorModel::getParameterValue(int index) const {
  return m_currentParameterValue[index];
}

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::getParameterName
//***************************************************************************
std::string UsgsAstroProjectedLsSensorModel::getParameterName(int index) const {
  return PARAMETER_NAME[index];
}

std::string UsgsAstroProjectedLsSensorModel::getParameterUnits(int index) const {
  // All parameters are meters or scaled to meters
  return "m";
}

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::getNumParameters
//***************************************************************************
int UsgsAstroProjectedLsSensorModel::getNumParameters() const {
  return UsgsAstroProjectedLsSensorModel::NUM_PARAMETERS;
}

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::getParameterType
//***************************************************************************
csm::param::Type UsgsAstroProjectedLsSensorModel::getParameterType(int index) const {
  return m_parameterType[index];
}

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::setParameterType
//***************************************************************************
void UsgsAstroProjectedLsSensorModel::setParameterType(int index,
                                              csm::param::Type pType) {
  m_parameterType[index] = pType;
}

//---------------------------------------------------------------------------
// Sensor Model Information
//---------------------------------------------------------------------------

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::getPedigree
//***************************************************************************
std::string UsgsAstroProjectedLsSensorModel::getPedigree() const {
  return "USGS_LINE_SCANNER";
}

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::getImageIdentifier
//***************************************************************************
std::string UsgsAstroProjectedLsSensorModel::getImageIdentifier() const {
  return m_imageIdentifier;
}

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::setImageIdentifier
//***************************************************************************
void UsgsAstroProjectedLsSensorModel::setImageIdentifier(const std::string& imageId,
                                                csm::WarningList* warnings) {
  // Image id should include the suffix without the path name
  m_imageIdentifier = imageId;
}

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::getSensorIdentifier
//***************************************************************************
std::string UsgsAstroProjectedLsSensorModel::getSensorIdentifier() const {
  return m_sensorIdentifier;
}

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::getPlatformIdentifier
//***************************************************************************
std::string UsgsAstroProjectedLsSensorModel::getPlatformIdentifier() const {
  return m_platformIdentifier;
}

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::setReferencePoint
//***************************************************************************
void UsgsAstroProjectedLsSensorModel::setReferencePoint(
    const csm::EcefCoord& ground_pt) {
  m_referencePointXyz = ground_pt;
}

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::getReferencePoint
//***************************************************************************
csm::EcefCoord UsgsAstroProjectedLsSensorModel::getReferencePoint() const {
  // Return ground point at image center
  return m_referencePointXyz;
}

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::getSensorModelName
//***************************************************************************
std::string UsgsAstroProjectedLsSensorModel::getModelName() const {
  return UsgsAstroProjectedLsSensorModel::_SENSOR_MODEL_NAME;
}

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::getImageStart
//***************************************************************************
csm::ImageCoord UsgsAstroProjectedLsSensorModel::getImageStart() const {
  return csm::ImageCoord(0.0, 0.0);
}

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::getImageSize
//***************************************************************************
csm::ImageVector UsgsAstroProjectedLsSensorModel::getImageSize() const {
  return csm::ImageVector(m_nLines, m_nSamples);
}

//---------------------------------------------------------------------------
//  Monoscopic Mensuration
//---------------------------------------------------------------------------

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::getValidHeightRange
//***************************************************************************
std::pair<double, double> UsgsAstroProjectedLsSensorModel::getValidHeightRange() const {
  return std::pair<double, double>(m_minElevation, m_maxElevation);
}

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::getValidImageRange
//***************************************************************************
std::pair<csm::ImageCoord, csm::ImageCoord>
UsgsAstroProjectedLsSensorModel::getValidImageRange() const {
  return std::pair<csm::ImageCoord, csm::ImageCoord>(
      csm::ImageCoord(0.0, 0.0),
      csm::ImageCoord(m_nLines,
                      m_nSamples));  // Technically nl and ns are outside the
                                     // image in a zero based system.
}

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::getIlluminationDirection
//***************************************************************************
csm::EcefVector UsgsAstroProjectedLsSensorModel::getIlluminationDirection(
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

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::getNumGeometricCorrectionSwitches
//***************************************************************************
int UsgsAstroProjectedLsSensorModel::getNumGeometricCorrectionSwitches() const {
  return 0;
}

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::getGeometricCorrectionName
//***************************************************************************
std::string UsgsAstroProjectedLsSensorModel::getGeometricCorrectionName(
    int index) const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Accessing name of geometric correction switch {}. "
      "Geometric correction switches are not supported, throwing exception",
      index);
  // Since there are no geometric corrections, all indices are out of range
  throw csm::Error(csm::Error::INDEX_OUT_OF_RANGE, "Index is out of range.",
                   "UsgsAstroProjectedLsSensorModel::getGeometricCorrectionName");
}

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::setGeometricCorrectionSwitch
//***************************************************************************
void UsgsAstroProjectedLsSensorModel::setGeometricCorrectionSwitch(
    int index, bool value, csm::param::Type pType) {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Setting geometric correction switch {} to {} "
      "with parameter type {}. "
      "Geometric correction switches are not supported, throwing exception",
      index, value, pType);
  // Since there are no geometric corrections, all indices are out of range
  throw csm::Error(csm::Error::INDEX_OUT_OF_RANGE, "Index is out of range.",
                   "UsgsAstroProjectedLsSensorModel::setGeometricCorrectionSwitch");
}

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::getGeometricCorrectionSwitch
//***************************************************************************
bool UsgsAstroProjectedLsSensorModel::getGeometricCorrectionSwitch(int index) const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Accessing value of geometric correction switch {}. "
      "Geometric correction switches are not supported, throwing exception",
      index);
  // Since there are no geometric corrections, all indices are out of range
  throw csm::Error(csm::Error::INDEX_OUT_OF_RANGE, "Index is out of range.",
                   "UsgsAstroProjectedLsSensorModel::getGeometricCorrectionSwitch");
}

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::getCrossCovarianceMatrix
//***************************************************************************
std::vector<double> UsgsAstroProjectedLsSensorModel::getCrossCovarianceMatrix(
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
const csm::CorrelationModel& UsgsAstroProjectedLsSensorModel::getCorrelationModel()
    const {
  // All Line Scanner images are assumed uncorrelated
  return _no_corr_model;
}

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::getUnmodeledCrossCovariance
//***************************************************************************
std::vector<double> UsgsAstroProjectedLsSensorModel::getUnmodeledCrossCovariance(
    const csm::ImageCoord& pt1, const csm::ImageCoord& pt2) const {
  // No unmodeled error
  return std::vector<double>(4, 0.0);
}

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::getCollectionIdentifier
//***************************************************************************
std::string UsgsAstroProjectedLsSensorModel::getCollectionIdentifier() const {
  return "UNKNOWN";
}

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::hasShareableParameters
//***************************************************************************
bool UsgsAstroProjectedLsSensorModel::hasShareableParameters() const {
  // Parameter sharing is not supported for this sensor
  return false;
}

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::isParameterShareable
//***************************************************************************
bool UsgsAstroProjectedLsSensorModel::isParameterShareable(int index) const {
  // Parameter sharing is not supported for this sensor
  return false;
}

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::getParameterSharingCriteria
//***************************************************************************
csm::SharingCriteria UsgsAstroProjectedLsSensorModel::getParameterSharingCriteria(
    int index) const {
  MESSAGE_LOG(
      spdlog::level::debug,
      "Checking sharing criteria for parameter {}. "
      "Sharing is not supported.",
      index);
  return csm::SharingCriteria();
}

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::getSensorType
//***************************************************************************
std::string UsgsAstroProjectedLsSensorModel::getSensorType() const {
  return CSM_SENSOR_TYPE_EO;
}

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::getSensorMode
//***************************************************************************
std::string UsgsAstroProjectedLsSensorModel::getSensorMode() const {
  return CSM_SENSOR_MODE_PB;
}

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::getVersion
//***************************************************************************
csm::Version UsgsAstroProjectedLsSensorModel::getVersion() const {
  return csm::Version(1, 0, 0);
}

//***************************************************************************
// UsgsAstroLineScannerSensorModel::getEllipsoid
//***************************************************************************
csm::Ellipsoid UsgsAstroProjectedLsSensorModel::getEllipsoid() const {
  return csm::Ellipsoid(m_majorAxis, m_minorAxis);
}

void UsgsAstroProjectedLsSensorModel::setEllipsoid(const csm::Ellipsoid& ellipsoid) {
  m_majorAxis = ellipsoid.getSemiMajorRadius();
  m_minorAxis = ellipsoid.getSemiMinorRadius();
}

//***************************************************************************
// UsgsAstroLineScannerSensorModel::getValue
//***************************************************************************
double UsgsAstroProjectedLsSensorModel::getValue(
    int index, const std::vector<double>& adjustments) const {
  return m_currentParameterValue[index] + adjustments[index];
}

//***************************************************************************
// Functions pulled out of losToEcf and computeViewingPixel
// **************************************************************************
void UsgsAstroProjectedLsSensorModel::getQuaternions(const double& time,
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

//***************************************************************************
// UsgsAstroLineScannerSensorModel::calculateAttitudeCorrection
//***************************************************************************
void UsgsAstroProjectedLsSensorModel::calculateAttitudeCorrection(
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

//***************************************************************************
// UsgsAstroLineScannerSensorModel::computeProjectiveApproximation
//***************************************************************************
void UsgsAstroProjectedLsSensorModel::computeProjectiveApproximation(const csm::EcefCoord& gp,
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

//***************************************************************************
// UsgsAstroLineScannerSensorModel::createProjectiveApproximation
//***************************************************************************
void UsgsAstroProjectedLsSensorModel::createProjectiveApproximation() {
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

//***************************************************************************
// UsgsAstroLineScannerSensorModel::constructStateFromIsd
//***************************************************************************
std::string UsgsAstroProjectedLsSensorModel::constructStateFromIsd(
    const std::string imageSupportData, csm::WarningList* warnings) {
  // return UsgsAstroLsSensorModel::constructStateFromIsd(imageSupportData, warnings);
  json lsState = json::parse(UsgsAstroLsSensorModel::constructStateFromIsd(imageSupportData, warnings));
  json state = json::parse(imageSupportData);

  lsState["m_geoTransform"] = getGeoTransform(state);
  lsState["m_projString"] = getProjectionString(state);
  MESSAGE_LOG(
      spdlog::level::trace,
      "m_geoTransform: {} "
      "m_projString: {} ",
      lsState["m_geoTransform"].dump(), lsState["m_projString"].dump());
  // The state data will still be updated when a sensor model is created since
  // some state data is not in the ISD and requires a SM to compute them.
  return lsState.dump();
}

//***************************************************************************
// UsgsAstroLineScannerSensorModel::getLogger
//***************************************************************************
std::shared_ptr<spdlog::logger> UsgsAstroProjectedLsSensorModel::getLogger() {
  return m_logger;
}

void UsgsAstroProjectedLsSensorModel::setLogger(std::string logName) {
  m_logger = spdlog::get(logName);
}
