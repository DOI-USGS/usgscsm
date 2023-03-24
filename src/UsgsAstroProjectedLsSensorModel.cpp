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
  UsgsAstroLsSensorModel::replaceModelState(stateString);

  // Pull proj string from ISD
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

//***************************************************************************
// UsgsAstroLineScannerSensorModel::reset
//***************************************************************************
void UsgsAstroProjectedLsSensorModel::reset() {
  MESSAGE_LOG(spdlog::level::debug, "Running reset()")
  UsgsAstroLsSensorModel::reset();

  // Reset the proj string
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
  csm::ImageCoord imagePt = UsgsAstroLsSensorModel::groundToImage(ground_pt, desired_precision, achieved_precision, warnings);
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
  // MESSAGE_LOG(
  //     spdlog::level::info,
  //     "Computing imageToGround for {}, {}, {}, with desired precision {}",
  //     image_pt.line, image_pt.samp, height, desired_precision);
  // double xc, yc, zc;
  // double vx, vy, vz;
  // double xl, yl, zl;
  // double dxl, dyl, dzl;
  // losToEcf(image_pt.line, image_pt.samp, _no_adjustment, xc, yc, zc, vx, vy, vz,
  //          xl, yl, zl);

  // double aPrec;
  double x = 0, y = 0, z = 0;
  // losEllipsoidIntersect(height, xc, yc, zc, xl, yl, zl, x, y, z, aPrec,
  //                       desired_precision, warnings);

  // if (achieved_precision) *achieved_precision = aPrec;

  // if (warnings && (desired_precision > 0.0) && (aPrec > desired_precision)) {
  //   warnings->push_back(csm::Warning(
  //       csm::Warning::PRECISION_NOT_MET, "Desired precision not achieved.",
  //       "UsgsAstroProjectedLsSensorModel::imageToGround()"));
  // }
  // MESSAGE_LOG(
  //     spdlog::level::info,
  //     "imageToGround result {} {} {}",
  //     x, y, z);
  return csm::EcefCoord(x, y, z);
}

//***************************************************************************
// UsgsAstroProjectedLsSensorModel::imageToGround
//***************************************************************************
csm::EcefCoordCovar UsgsAstroProjectedLsSensorModel::imageToGround(
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
  // Convert imagept to camera imagept
  // proj -> ecef -> groundToImage
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
  ip.samp = image_pt.samp;
  csm::EcefCoord gph =
      imageToGround(ip, height + DELTA_GROUND, desired_precision);
  double xph = (gph.x - gp.x) / DELTA_GROUND;
  double yph = (gph.y - gp.y) / DELTA_GROUND;
  double zph = (gph.z - gp.z) / DELTA_GROUND;

  // Convert sensor covariance to image space
  // double sCov[4];
  // determineSensorCovarianceInImageSpace(gp, sCov);

  std::vector<double> unmod = getUnmodeledError(image_pt);

  double iCov[4];
  iCov[0] = 0;
  iCov[1] = 0;
  iCov[2] = 0;
  iCov[3] = 0;

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
// UsgsAstroProjectedLsSensorModel::imageToProximateImagingLocus
//***************************************************************************
csm::EcefLocus UsgsAstroProjectedLsSensorModel::imageToProximateImagingLocus(
    const csm::ImageCoord& image_pt, const csm::EcefCoord& ground_pt,
    double desired_precision, double* achieved_precision,
    csm::WarningList* warnings) const {
  // Convert imagept to camera imagept
  // proj -> ecef -> groundToImage
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
// UsgsAstroProjectedLsSensorModel::imageToRemoteImagingLocus
//***************************************************************************
csm::EcefLocus UsgsAstroProjectedLsSensorModel::imageToRemoteImagingLocus(
    const csm::ImageCoord& image_pt, double desired_precision,
    double* achieved_precision, csm::WarningList* warnings) const {
  // Go from proj x, y to latlon then ground to image
  // Convert imagept to camera imagept
  // proj -> ecef -> groundToImage
  return UsgsAstroLsSensorModel::imageToRemoteImagingLocus(image_pt, desired_precision, achieved_precision, warnings);
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
                     "UsgsAstroFrameSensorModel::constructStateFromIsd");
  }

  // The state data will still be updated when a sensor model is created since
  // some state data is not in the ISD and requires a SM to compute them.
  return state.dump();
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

csm::EcefVector UsgsAstroProjectedLsSensorModel::getSunPosition(
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
